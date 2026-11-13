//! Source-defined Winkler field: independent of FE tessellation and node IDs.
use super::{
    geometry::Point,
    loads::{ForceUnits, Magnitude},
    model::{Contour, Kind},
};
use spade::{DelaunayTriangulation, Point2, Triangulation};
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Data {
    pub values: Vec<Magnitude>,
    pub interior: Vec<Sample>,
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub exclusions: Vec<Vec<Point>>,
    pub horizontal_enabled: bool,
    pub horizontal: Magnitude,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Sample {
    pub xy: Point,
    pub c1: Magnitude,
}
impl Default for Data {
    fn default() -> Self {
        Self {
            values: vec![],
            interior: vec![],
            exclusions: vec![],
            horizontal_enabled: false,
            horizontal: Magnitude {
                value: 0.,
                units: ForceUnits::TonForce,
            },
        }
    }
}
pub fn default_c1() -> Magnitude {
    Magnitude {
        value: 1000.,
        units: ForceUnits::TonForce,
    }
}
impl Data {
    pub fn map_xy(&mut self, mut map: impl FnMut(&mut Point)) {
        for point in &mut self.interior {
            map(&mut point.xy);
        }
        for point in self.exclusions.iter_mut().flatten() {
            map(point);
        }
    }

    pub fn covers(&self, boundary: &[Point], point: Point) -> bool {
        super::geometry::inside(point, boundary)
            && !self
                .exclusions
                .iter()
                .any(|ring| super::geometry::inside(point, ring))
    }

    pub fn value(&self, i: usize) -> Magnitude {
        self.values.get(i).copied().unwrap_or_else(default_c1)
    }
    pub fn round_values(&mut self, vertices: usize, step: f64, units: ForceUnits, g: f64) {
        self.values.resize(vertices, default_c1());
        let round = |value: &mut Magnitude| {
            let raw = value.in_units(units, g);
            *value = Magnitude {
                value: (raw / step).round() * step,
                units,
            };
        };
        self.values.iter_mut().for_each(round);
        self.interior
            .iter_mut()
            .for_each(|point| round(&mut point.c1));
    }

    pub fn validate(&self, c: &Contour) -> Result<(), String> {
        if self
            .values
            .iter()
            .chain(self.interior.iter().map(|p| &p.c1))
            .chain(std::iter::once(&self.horizontal))
            .any(|v| !v.value.is_finite() || v.value < 0.)
        {
            return Err(format!(
                "Основание {}: C1 и горизонтальная жёсткость должны быть неотрицательны",
                c.id
            ));
        }
        if !self.exclusions.is_empty() {
            use mini_fem::geo::{algorithm::Validation, LineString, Polygon};
            let line = |ring: &[Point]| {
                LineString::from(ring.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>())
            };
            let polygon = Polygon::new(
                line(&c.points),
                self.exclusions.iter().map(|ring| line(ring)).collect(),
            );
            if !polygon.is_valid() {
                return Err(format!(
                    "Основание {}: неверная граница внутренней области",
                    c.id
                ));
            }
        }
        for p in &self.interior {
            if p.xy.iter().any(|v| !v.is_finite()) || !self.covers(&c.points, p.xy) {
                return Err(format!(
                    "Основание {}: точка интерполяции вне основания",
                    c.id
                ));
            }
        }
        Ok(())
    }
}
pub struct Field {
    pub id: u64,
    pub area: f64,
    pub contour: Contour,
    pub triangles: Vec<([Point; 3], [f64; 3])>,
    pub horizontal: f64,
    exact: std::collections::BTreeMap<[u64; 2], f64>,
    triangle_index:
        rstar::RTree<rstar::primitives::GeomWithData<rstar::primitives::Rectangle<Point>, usize>>,
}
pub fn remap_values(original: &Contour, next: &Contour, g: f64, units: ForceUnits) -> Data {
    let mut data = original.foundation.clone().unwrap_or_default();
    let old = vertices(original);
    let values = vertices(next)
        .into_iter()
        .map(|p| {
            if let Some(i) = old
                .iter()
                .position(|q| (q[0] - p[0]).hypot(q[1] - p[1]) < 1e-6)
            {
                return data.value(i);
            }
            let mut best = None::<(f64, f64)>;
            for i in 0..old.len() {
                let a = old[i];
                let b = old[(i + 1) % old.len()];
                let d = [b[0] - a[0], b[1] - a[1]];
                let l = d[0] * d[0] + d[1] * d[1];
                if l < 1e-12 {
                    continue;
                }
                let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.);
                let distance = (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]);
                let value = data.value(i).in_units(units, g) * (1. - t)
                    + data.value((i + 1) % old.len()).in_units(units, g) * t;
                if best.is_none_or(|v| distance < v.0) {
                    best = Some((distance, value));
                }
            }
            Magnitude {
                value: best.map_or(default_c1().in_units(units, g), |v| v.1),
                units,
            }
        })
        .collect();
    data.values = values;
    data
}
pub fn vertices(c: &Contour) -> Vec<Point> {
    if c.curves.is_empty() {
        c.points.clone()
    } else {
        c.handles()
    }
}
pub fn contains(c: &Contour, p: Point) -> bool {
    c.foundation.as_ref().map_or_else(
        || super::geometry::inside(p, &c.points),
        |data| data.covers(&c.points, p),
    )
}
impl Field {
    pub fn new(c: &Contour, g: f64) -> Result<Self, String> {
        let data = c.foundation.clone().unwrap_or_default();
        data.validate(c)?;
        let points = vertices(c);
        let mut samples = points
            .iter()
            .enumerate()
            .map(|(i, &xy)| Sample {
                xy,
                c1: data.value(i),
            })
            .collect::<Vec<_>>();
        samples.extend(data.interior.clone());
        if !c.curves.is_empty() {
            let known = samples.clone();
            let mut seed = DelaunayTriangulation::<Point2<f64>>::new();
            let mut values = std::collections::BTreeMap::new();
            for p in &known {
                let h = seed
                    .insert(Point2::new(p.xy[0], p.xy[1]))
                    .map_err(|e| format!("Основание {}: {e:?}", c.id))?;
                values.insert(h.index(), p.c1.in_units(ForceUnits::KiloNewton, g));
            }
            let triangles = seed
                .inner_faces()
                .map(|f| {
                    let v = f.vertices();
                    (
                        v.map(|v| [v.position().x, v.position().y]),
                        v.map(|v| values[&v.fix().index()]),
                    )
                })
                .collect::<Vec<_>>();
            for &xy in &c.points {
                if known
                    .iter()
                    .any(|p| (p.xy[0] - xy[0]).hypot(p.xy[1] - xy[1]) < 1e-6)
                {
                    continue;
                }
                let mut best = None::<(f64, f64)>;
                for (q, v) in &triangles {
                    let cross = |a: Point, b: Point| a[0] * b[1] - a[1] * b[0];
                    let sub = |a: Point, b: Point| [a[0] - b[0], a[1] - b[1]];
                    let det = cross(sub(q[1], q[0]), sub(q[2], q[0]));
                    if det.abs() < 1e-12 {
                        continue;
                    }
                    let b = cross(sub(xy, q[0]), sub(q[2], q[0])) / det;
                    let d = cross(sub(q[1], q[0]), sub(xy, q[0])) / det;
                    let a = 1. - b - d;
                    let outside = [a, b, d].iter().map(|w| (-*w).max(0.)).sum::<f64>();
                    if best.is_none_or(|old| outside < old.0) {
                        best = Some((outside, a * v[0] + b * v[1] + d * v[2]));
                    }
                }
                if let Some((_, value)) = best {
                    samples.push(Sample {
                        xy,
                        c1: Magnitude {
                            value: value.max(0.),
                            units: ForceUnits::KiloNewton,
                        },
                    });
                }
            }
        }
        let mut tri = DelaunayTriangulation::<Point2<f64>>::new();
        let mut values = std::collections::BTreeMap::new();
        let mut exact = std::collections::BTreeMap::new();
        for p in samples {
            let h = tri
                .insert(Point2::new(p.xy[0], p.xy[1]))
                .map_err(|e| format!("Основание {}: {e:?}", c.id))?;
            let value = p.c1.in_units(ForceUnits::KiloNewton, g);
            exact.insert(p.xy.map(f64::to_bits), value);
            if let Some(old) = values.insert(h.index(), value) {
                if (old - value).abs() > 1e-8 {
                    return Err(format!("Основание {}: разные C1 в совпадающей точке", c.id));
                }
            }
        }
        let triangles: Vec<_> = tri
            .inner_faces()
            .map(|f| {
                let v = f.vertices();
                (
                    v.map(|v| [v.position().x, v.position().y]),
                    v.map(|v| values[&v.fix().index()]),
                )
            })
            .collect();
        use rstar::primitives::{GeomWithData, Rectangle};
        let triangle_index = rstar::RTree::bulk_load(
            triangles
                .iter()
                .enumerate()
                .map(|(index, (p, _))| {
                    let lo = std::array::from_fn(|i| p[0][i].min(p[1][i]).min(p[2][i]) - 1e-5);
                    let hi = std::array::from_fn(|i| p[0][i].max(p[1][i]).max(p[2][i]) + 1e-5);
                    GeomWithData::new(Rectangle::from_corners(lo, hi), index)
                })
                .collect(),
        );
        let area = super::geometry::area(&c.points).abs()
            - data
                .exclusions
                .iter()
                .map(|ring| super::geometry::area(ring).abs())
                .sum::<f64>();
        Ok(Self {
            id: c.id,
            area,
            contour: c.clone(),
            triangles,
            exact,
            triangle_index,
            horizontal: if data.horizontal_enabled {
                data.horizontal.in_units(ForceUnits::KiloNewton, g)
            } else {
                0.
            },
        })
    }
    pub fn at(&self, p: Point) -> Option<f64> {
        if !contains(&self.contour, p) {
            return None;
        }
        if let Some(&value) = self.exact.get(&p.map(f64::to_bits)) {
            return Some(value);
        }
        for entry in self
            .triangle_index
            .locate_in_envelope_intersecting(&rstar::AABB::from_point(p))
        {
            let (q, v) = &self.triangles[entry.data];
            let cross = |a: Point, b: Point| a[0] * b[1] - a[1] * b[0];
            let sub = |a: Point, b: Point| [a[0] - b[0], a[1] - b[1]];
            let det = cross(sub(q[1], q[0]), sub(q[2], q[0]));
            if det.abs() < 1e-12 {
                continue;
            }
            let b = cross(sub(p, q[0]), sub(q[2], q[0])) / det;
            let c = cross(sub(q[1], q[0]), sub(p, q[0])) / det;
            let a = 1. - b - c;
            if [a, b, c].iter().all(|v| *v >= -1e-8) {
                return Some((a * v[0] + b * v[1] + c * v[2]).max(0.));
            }
        }
        None
    }
}
pub fn fields(all: &[Contour], g: f64) -> Result<Vec<Field>, String> {
    let mut fields = all
        .iter()
        .filter(|c| c.kind == Kind::Foundation)
        .map(|c| Field::new(c, g))
        .collect::<Result<Vec<_>, _>>()?;
    fields.sort_by(|a, b| a.area.total_cmp(&b.area).then_with(|| b.id.cmp(&a.id)));
    Ok(fields)
}
pub fn elevation(layout: Option<&super::building_mesh::Layout>, contour: &Contour) -> Option<f64> {
    let Some(layout) = layout else {
        return Some(0.);
    };
    let floor = layout
        .levels
        .iter()
        .find(|level| level.id == layout.owner(contour.id))?;
    Some(contour.placement.slab.z([floor.z, floor.top]))
}

pub fn same_elevation(a: f64, b: f64) -> bool {
    (a - b).abs() < 1e-5
}
/// Remove only interior samples surrounded entirely by the same C1.
pub fn simplify_interior(c: &mut Contour, g: f64) -> Result<usize, String> {
    let original = Field::new(c, g)?;
    let Some(data) = &mut c.foundation else {
        return Ok(0);
    };
    let mut flat = std::collections::BTreeMap::new();
    for (points, values) in &original.triangles {
        let constant = values.iter().all(|v| (v - values[0]).abs() <= 1e-9);
        for p in points {
            flat.entry(p.map(f64::to_bits))
                .and_modify(|ok| *ok &= constant)
                .or_insert(constant);
        }
    }
    let removed: Vec<_> = data
        .interior
        .iter()
        .filter(|p| flat.get(&p.xy.map(f64::to_bits)) == Some(&true))
        .cloned()
        .collect();
    if removed.is_empty() {
        return Ok(0);
    }
    let before = data.interior.clone();
    data.interior
        .retain(|p| flat.get(&p.xy.map(f64::to_bits)) != Some(&true));
    let updated = Field::new(c, g);
    if let Ok(field) = updated {
        if removed.iter().all(|p| {
            field.at(p.xy).is_some_and(|value| {
                (value - p.c1.in_units(ForceUnits::KiloNewton, g)).abs() <= 1e-7
            })
        }) {
            return Ok(removed.len());
        }
    }
    c.foundation.as_mut().unwrap().interior = before;
    Ok(0)
}
#[cfg(test)]
mod tests {
    use super::*;
    fn contour(id: u64, kind: Kind, points: Vec<Point>) -> Contour {
        Contour {
            id,
            kind,
            points,
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
            parent: None,
            curves: vec![],
        }
    }
    #[test]
    fn rounds_boundary_and_interior_values_in_display_units() {
        let mut data = Data {
            values: vec![Magnitude {
                value: 3470.,
                units: ForceUnits::KiloNewton,
            }],
            interior: vec![Sample {
                xy: [12., 34.],
                c1: Magnitude {
                    value: 376.,
                    units: ForceUnits::TonForce,
                },
            }],
            horizontal: Magnitude {
                value: 123.,
                units: ForceUnits::KiloNewton,
            },
            ..Default::default()
        };
        data.round_values(2, 50., ForceUnits::TonForce, 10.);
        assert_eq!(
            data.values.iter().map(|v| v.value).collect::<Vec<_>>(),
            [350., 1000.]
        );
        assert_eq!(data.interior[0].c1.value, 400.);
        assert_eq!(data.interior[0].xy, [12., 34.]);
        assert_eq!(data.horizontal.value, 123.);
    }

    #[test]
    fn affine_field_interpolates_internal_samples_and_preserves_force_units() {
        let mut c = contour(
            1,
            Kind::Foundation,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        );
        let value = |xy: Point| Magnitude {
            value: 100. + xy[0] * 0.2 + xy[1] * 0.3,
            units: ForceUnits::KiloNewton,
        };
        c.foundation = Some(Data {
            values: c.points.iter().copied().map(value).collect(),
            interior: vec![Sample {
                xy: [800., 1200.],
                c1: value([800., 1200.]),
            }],
            ..Default::default()
        });
        let f = Field::new(&c, 10.).unwrap();
        for p in [[100., 100.], [1000., 1000.], [1800., 200.], [800., 1200.]] {
            assert!((f.at(p).unwrap() - value(p).value).abs() < 1e-8);
        }
        assert!(f.at([3000., 0.]).is_none());
        let mut curved = c.clone();
        curved.curves = vec![crate::plan_trace::model::Curve::Ellipse([
            [0., 0.],
            [1000., 0.],
            [0., 1000.],
        ])];
        curved.rebuild().unwrap();
        let curve_value = |p: Point| Magnitude {
            value: 1000. + p[0] * 0.2 + p[1] * 0.3,
            units: ForceUnits::KiloNewton,
        };
        curved.foundation = Some(Data {
            values: vertices(&curved).iter().copied().map(curve_value).collect(),
            ..Default::default()
        });
        let field = Field::new(&curved, 10.).unwrap();
        for p in [[-500., -100.], [400., 300.]] {
            assert!((field.at(p).unwrap() - curve_value(p).value).abs() < 1e-7);
        }

        let mut d = c.foundation.clone().unwrap();
        d.interior[0].c1.value = -1.;
        assert!(d.validate(&c).is_err());
    }
    #[test]
    fn coincident_plans_use_c1_on_the_actual_plate_elevation() {
        use crate::plan_trace::{
            building_mesh, fem, lira_export, lira_txt_reader,
            loads::{LoadCase, LoadType},
            quad_mesh,
        };
        use std::collections::BTreeMap;

        let points = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
        let mut all = Vec::new();
        for (id, z, c1) in [(1, 0., 500.), (2, 1200., 1000.)] {
            let mut slab = contour(id, Kind::Slab, points.clone());
            slab.use_foundation = true;
            slab.placement.slab.offset_mm = z;
            let mut base = contour(id + 10, Kind::Foundation, points.clone());
            base.placement.slab = slab.placement.slab;
            base.foundation = Some(Data {
                values: vec![
                    Magnitude {
                        value: c1,
                        units: ForceUnits::KiloNewton
                    };
                    4
                ],
                horizontal_enabled: true,
                horizontal: if id == 1 {
                    Magnitude {
                        value: 11.,
                        units: ForceUnits::TonForce,
                    }
                } else {
                    Magnitude {
                        value: 220.,
                        units: ForceUnits::KiloNewton,
                    }
                },
                ..Default::default()
            });
            all.extend([slab, base]);
        }
        let layout = building_mesh::Layout {
            levels: vec![building_mesh::Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: all.iter().map(|c| (c.id, 1)).collect(),
            legacy_wall_extent: None,
        };
        let mesh = building_mesh::generate(
            &all,
            500.,
            layout,
            quad_mesh::Options {
                merge: true,
                ..Default::default()
            },
        )
        .unwrap();
        let coefficients = fem::foundation_coefficients(&mesh, &all, 10.).unwrap();
        for (element, coefficient) in mesh.elements.iter().zip(coefficients) {
            let expected = if mesh.node_z(element.nodes[0]) == 0. {
                500.
            } else {
                1000.
            };
            assert!((coefficient - expected).abs() < 1e-8);
        }

        let material = fem::element::Material::default();
        let mut case = LoadCase::new(1);
        case.kind = LoadType::SelfWeight;
        let result = fem::solve_with_support_height(
            &mesh,
            &all,
            &BTreeMap::from([(1, material), (2, material)]),
            &[case],
            10.,
            500.,
            fem::ElementType::Dkt,
            &fem::shell::Settings::default(),
        )
        .unwrap();
        for (n, w) in result.cases[0].w_mm.iter().enumerate() {
            let c1 = if mesh.node_z(n) == 0. { 500. } else { 1000. };
            let expected = material.self_weight_kn_m2(10.) / c1 * 1000.;
            assert!((w - expected).abs() < 2e-5, "node {n}: {w} vs {expected}");
        }

        let (scene, loads) = fem::lira_model(
            &mesh,
            &all,
            &BTreeMap::from([(1, material), (2, material)]),
            &[],
            10.,
            500.,
            &fem::shell::Settings::default(),
        )
        .unwrap();
        let (c1, horizontal) = fem::lira_foundation(&mesh, &all, 10.).unwrap();
        let mut export = lira_export::encode_with_rigid_and_springs(
            &scene,
            &loads,
            &[],
            10.,
            "Test",
            &[],
            &horizontal,
        )
        .unwrap();
        lira_export::append_foundation(&mut export, &scene, &c1, 10.).unwrap();
        let parsed = lira_txt_reader::parse(&export.text).unwrap();
        assert_eq!(parsed.nodes.len(), scene.xyz.len());
        assert_eq!(export.elements, scene.surfaces.len() + mesh.nodes.len());
        let springs = &parsed.elements[scene.surfaces.len()..];
        assert_eq!(springs.len(), mesh.nodes.len());
        for (n, spring) in springs.iter().enumerate() {
            assert_eq!(spring.fe_type, 56);
            assert_eq!(spring.nodes, [n + 1]);
            let k = if mesh.node_z(n) == 0. { 11 } else { 22 };
            assert!(export
                .text
                .contains(&format!("\n{} {k} {k} 0 0 0 0/", spring.stiffness)));
        }
        assert_eq!(
            springs
                .iter()
                .map(|s| s.stiffness)
                .collect::<std::collections::BTreeSet<_>>()
                .len(),
            2
        );
        let doc19 = export.text.split_once("(19/\n").unwrap().1;
        assert_eq!(doc19.matches('/').count(), scene.surfaces.len());

        all.iter_mut()
            .find(|c| c.id == 12)
            .unwrap()
            .foundation
            .as_mut()
            .unwrap()
            .horizontal_enabled = false;
        let (_, horizontal) = fem::lira_foundation(&mesh, &all, 10.).unwrap();
        assert_eq!(
            horizontal.len(),
            mesh.nodes
                .iter()
                .enumerate()
                .filter(|(n, _)| mesh.node_z(*n) == 0.)
                .count()
        );
        assert!(horizontal.keys().all(|&n| mesh.node_z(n) == 0.));

        all.retain(|c| c.id != 12);
        let coefficients = fem::foundation_coefficients(&mesh, &all, 10.).unwrap();
        assert!(mesh.elements.iter().zip(coefficients).all(|(el, c1)| {
            if mesh.node_z(el.nodes[0]) == 0. {
                (c1 - 500.).abs() < 1e-8
            } else {
                c1.is_nan()
            }
        }));
    }

    #[test]
    fn freely_supported_uniform_winkler_plate_balances_for_all_plate_kernels() {
        use crate::plan_trace::{
            fem,
            loads::{LoadCase, LoadData, LoadType},
            meshing, quad_mesh,
        };
        let mut slab = contour(
            1,
            Kind::Slab,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        );
        slab.use_foundation = true;
        let base = contour(2, Kind::Foundation, slab.points.clone());
        let mut load = contour(3, Kind::LoadArea, slab.points.clone());
        load.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            case_id: 1,
            name: String::new(),
            magnitude: Magnitude {
                value: 2.,
                units: ForceUnits::TonForce,
            },
            closed: true,
            mesh_constraint: false,
        });
        let mut all = vec![slab, base, load];
        let raw = meshing::generate(&all, 500.).unwrap();
        let materials = std::collections::BTreeMap::from([(1, fem::element::Material::default())]);
        let cases = vec![LoadCase {
            id: 1,
            name: "Test".into(),
            kind: LoadType::Permanent,
            gamma_f: 1.,
            gamma_l: 1.,
        }];
        for (mesh, kind) in [
            (raw.clone(), fem::ElementType::Dkt),
            (raw.clone(), fem::ElementType::MindlinCr),
            (
                quad_mesh::process(
                    raw,
                    quad_mesh::Options {
                        backend: Default::default(),
                        merge: true,
                        optimise: false,
                    },
                )
                .unwrap(),
                fem::ElementType::Dkt,
            ),
        ] {
            let r =
                fem::solve_load_cases(&mesh, &all, &materials, &cases, 10., 500., kind).unwrap();
            let c = &r.cases[0];
            assert!(
                c.w_mm.iter().all(|w| (*w - 2.).abs() < 2e-5),
                "{:?}",
                c.w_mm
            );
            assert!(c.balance_kn.abs() < 1e-6);
            assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-6));
            assert!((c.applied_kn - 80.).abs() < 1e-8);
        }
        all[1].foundation = Some(Data {
            horizontal_enabled: true,
            horizontal: Magnitude {
                value: 1000.,
                units: ForceUnits::TonForce,
            },
            ..Default::default()
        });
        let shell_settings = fem::shell::Settings {
            enabled: true,
            ..Default::default()
        };
        let shell = fem::solve_with_support_height(
            &meshing::generate(&all, 500.).unwrap(),
            &all,
            &materials,
            &cases,
            10.,
            500.,
            fem::ElementType::Dkt,
            &shell_settings,
        )
        .unwrap();
        assert!(shell.cases[0].w_mm.iter().all(|w| (*w - 2.).abs() < 2e-5));
        assert!(shell.cases[0].balance_kn.abs() < 1e-6);
        let request = fem::NonlinearRequest {
            weights: vec![(1, 1.)],
            caption: "Foundation test".into(),
        };
        let nonlinear = fem::solve_with_modal(
            &meshing::generate(&all, 500.).unwrap(),
            &all,
            &materials,
            &cases,
            10.,
            500.,
            fem::ElementType::Dkt,
            &shell_settings,
            None,
            false,
            Some(&request),
        )
        .unwrap();
        assert!(nonlinear.cases[0]
            .w_mm
            .iter()
            .all(|w| (*w - 2.).abs() < 2e-5));
        assert!(nonlinear.cases[0].balance_kn.abs() < 1e-6);
    }
}
