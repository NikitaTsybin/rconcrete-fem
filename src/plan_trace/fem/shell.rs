//! Flat shell assembly: CST/Q1 membrane, DKT on pure triangles, MITC3/4 on mixed surfaces.
//! Global XYZ translations, physical rotation vectors, no drilling springs.
#[path = "buckling_geometric.rs"]
mod buckling_geometric;
#[path = "effective_length.rs"]
pub(crate) mod effective_length;
#[path = "effective_length_columns.rs"]
mod effective_length_columns;
#[path = "nonlinear.rs"]
mod nonlinear;
use super::super::loads::{LoadCase, LoadType};
use super::*;
type V3 = [f64; 3];
type Row = Vec<(usize, f64)>;
fn dot(a: V3, b: V3) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: V3, b: V3) -> V3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn sub(a: V3, b: V3) -> V3 {
    std::array::from_fn(|i| a[i] - b[i])
}
fn unit(a: V3) -> V3 {
    let l = dot(a, a).sqrt();
    a.map(|v| v / l)
}
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub enum Direction {
    Up,
    Down,
    Both,
}
impl Direction {
    pub fn label(self) -> &'static str {
        match self {
            Self::Up => "Вверх",
            Self::Down => "Вниз",
            Self::Both => "Вверх+вниз",
        }
    }
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum LengthMethod {
    #[default]
    Plane,
    Spatial,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Settings {
    pub length_method: LengthMethod,
    #[serde(default)]
    pub columns: BTreeMap<u64, super::beam::Section>,
    #[serde(default)]
    pub rigid_columns: bool,
    #[serde(default)]
    pub automatic_building_bottom: bool,
    #[serde(default)]
    pub automatic_building_top: bool,
    pub building_bottom_mask: u8,
    pub building_top_mask: u8,
    pub enabled: bool,
    pub direction: Direction,
    pub height_mm: f64,
    pub materials: BTreeMap<u64, Material>,
    pub beams: BTreeMap<u64, super::beam::Section>,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            length_method: LengthMethod::Plane,
            columns: BTreeMap::new(),
            rigid_columns: false,
            automatic_building_bottom: true,
            automatic_building_top: true,
            building_bottom_mask: 7,
            building_top_mask: 3,
            enabled: false,
            direction: Direction::Down,
            height_mm: 3000.,
            materials: BTreeMap::new(),
            beams: BTreeMap::new(),
        }
    }
}
impl Settings {
    pub fn validate(&self) -> Result<(), String> {
        if !self.height_mm.is_finite() || self.height_mm < 100. || self.height_mm > 1_000_000. {
            return Err("Высота опор: от 100 до 1000000 мм".into());
        }
        for beam in self.beams.values().chain(self.columns.values()) {
            beam.validate()?;
        }
        for m in self.materials.values() {
            m.validate()?;
        }
        Ok(())
    }
    pub fn extent(&self) -> [f64; 2] {
        match self.direction {
            Direction::Up => [0., self.height_mm],
            Direction::Down => [-self.height_mm, 0.],
            Direction::Both => [-self.height_mm, self.height_mm],
        }
    }
}
#[derive(Clone)]
pub struct Surface {
    pub rigid_z_mm: f64,
    pub nodes: Vec<usize>,
    pub material: Material,
    pub display: Vec<usize>,
    pub support: Option<u64>,
}
#[derive(Clone, Default)]
pub struct Scene {
    pub xyz: Vec<V3>,
    pub plan: Vec<usize>,
    pub surfaces: Vec<Surface>,
    pub fixed: BTreeSet<usize>,
    pub support_nodes: BTreeMap<u64, Vec<usize>>,
    pub fixed_rotations: BTreeSet<usize>,
    pub beams: Vec<BeamGeometry>,
}
#[derive(Clone)]
pub struct BeamGeometry {
    pub rigid_offset_mm: [f64; 2],
    pub roll_angle_rad: f64,
    pub id: u64,
    pub nodes: [usize; 2],
    pub section: super::beam::Section,
}
impl BeamGeometry {
    /// Global rigid arm, from the mesh node to the elastic axis, in mm.
    pub fn rigid_arm(&self, xyz: &[[f64; 3]]) -> Result<[f64; 3], String> {
        if self
            .rigid_offset_mm
            .iter()
            .any(|v| !v.is_finite() || v.abs() > 1e6)
            || !self.roll_angle_rad.is_finite()
        {
            return Err(format!(
                "Стержень {}: некорректная жёсткая вставка или угол",
                self.id
            ));
        }
        if self.rigid_offset_mm == [0.; 2] {
            return Ok([0.; 3]);
        }
        let e = super::beam::Element::spatial(
            xyz[self.nodes[0]],
            xyz[self.nodes[1]],
            self.section.clone(),
        )?;
        let (_, y, z) = (e.basis[0], e.basis[1], e.basis[2]);
        let (s, c) = self.roll_angle_rad.sin_cos();
        Ok(std::array::from_fn(|i| {
            self.rigid_offset_mm[0] * (c * y[i] + s * z[i])
                + self.rigid_offset_mm[1] * (-s * y[i] + c * z[i])
        }))
    }
}
impl Scene {
    pub(crate) fn apply_surface_placement(&mut self, mesh: &Mesh, all: &[Contour]) {
        let sources = all.iter().map(|c| (c.id, c)).collect::<BTreeMap<_, _>>();
        let has_slab_offsets = all
            .iter()
            .any(|c| c.kind == Kind::Slab && c.placement.slab_rigid_z_mm != 0.);
        for surface in &mut self.surfaces {
            if let Some(wall) = surface.support.and_then(|id| sources.get(&id)) {
                let points = &surface.nodes;
                let a = self.xyz[points[0]];
                let n = cross(
                    sub(self.xyz[points[1]], a),
                    sub(self.xyz[*points.last().unwrap()], a),
                );
                let center: V3 = std::array::from_fn(|i| {
                    points.iter().map(|&node| self.xyz[node][i]).sum::<f64>() / points.len() as f64
                });
                let edge = wall
                    .straight_edges()
                    .into_iter()
                    .filter_map(|(_, a, b)| {
                        let d = [b[0] - a[0], b[1] - a[1]];
                        let length = d[0].hypot(d[1]);
                        if length <= 1e-9
                            || (n[0] * d[0] + n[1] * d[1]).abs() > 1e-8 * length * n[0].hypot(n[1])
                        {
                            return None;
                        }
                        let t = (((center[0] - a[0]) * d[0] + (center[1] - a[1]) * d[1])
                            / (length * length))
                            .clamp(0., 1.);
                        let distance =
                            (center[0] - a[0] - t * d[0]).hypot(center[1] - a[1] - t * d[1]);
                        Some((distance, [d[1], -d[0], 0.]))
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0));
                if edge.is_some_and(|(_, normal)| dot(n, normal) < 0.) {
                    surface.nodes[1..].reverse();
                }
            } else if let Some(&i) = surface.display.first().filter(|_| has_slab_offsets) {
                let slab = mesh
                    .building
                    .as_ref()
                    .and_then(|b| b.element_slabs.get(i))
                    .and_then(|id| sources.get(id).copied())
                    .or_else(|| {
                        let center = std::array::from_fn(|axis| {
                            surface
                                .nodes
                                .iter()
                                .map(|&n| self.xyz[n][axis])
                                .sum::<f64>()
                                / surface.nodes.len() as f64
                        });
                        all.iter()
                            .filter(|c| {
                                c.kind == Kind::Slab
                                    && crate::plan_trace::geometry::inside(center, &c.points)
                            })
                            .min_by(|a, b| {
                                crate::plan_trace::geometry::area(&a.points)
                                    .abs()
                                    .total_cmp(&crate::plan_trace::geometry::area(&b.points).abs())
                            })
                    });
                surface.rigid_z_mm = slab.map_or(0., |c| c.placement.slab_rigid_z_mm);
            }
        }
    }
    pub(crate) fn apply_beam_offsets(&mut self, all: &[Contour]) -> Result<(), String> {
        let offsets = all
            .iter()
            .filter(|c| matches!(c.kind, Kind::Beam | Kind::Column))
            .map(|c| (c.id, &c.placement))
            .collect::<BTreeMap<_, _>>();
        for beam in &mut self.beams {
            if let Some(p) = offsets.get(&beam.id) {
                p.validate()?;
                beam.rigid_offset_mm = p.rigid_yz_mm;
            }
        }
        Ok(())
    }
    pub(crate) fn apply_object_restraints(
        &mut self,
        settings: &Settings,
        ends: impl Fn(u64) -> Option<(bool, bool, [f64; 2])>,
    ) {
        for (&id, nodes) in &self.support_nodes {
            let Some((bottom, top, extent)) = ends(id) else {
                continue;
            };
            for &node in nodes {
                let z = self.xyz[node][2];
                let mask = if bottom
                    && settings.automatic_building_bottom
                    && (z - extent[0]).abs() < 1e-5
                {
                    settings.building_bottom_mask
                } else if top && settings.automatic_building_top && (z - extent[1]).abs() < 1e-5 {
                    settings.building_top_mask
                } else {
                    0
                };
                for axis in 0..3 {
                    if mask & (1 << axis) != 0 {
                        self.fixed.insert(3 * node + axis);
                    }
                    if mask & (1 << (axis + 3)) != 0 {
                        self.fixed_rotations.insert(3 * node + axis);
                    }
                }
            }
        }
    }
    pub(crate) fn apply_building_restraints(&mut self, settings: &Settings, bottom: f64, top: f64) {
        for &node in self.support_nodes.values().flatten() {
            let z = self.xyz[node][2];
            let mask = if settings.automatic_building_bottom && (z - bottom).abs() < 1e-5 {
                settings.building_bottom_mask
            } else if settings.automatic_building_top && (z - top).abs() < 1e-5 {
                settings.building_top_mask
            } else {
                0
            };
            for axis in 0..3 {
                if mask & (1 << axis) != 0 {
                    self.fixed.insert(3 * node + axis);
                }
                if mask & (1 << (axis + 3)) != 0 {
                    self.fixed_rotations.insert(3 * node + axis);
                }
            }
        }
    }
}
pub fn scene(
    mesh: &Mesh,
    geometry_materials: &[Material],
    settings: &Settings,
    step: f64,
    all: &[Contour],
) -> Result<Scene, String> {
    settings.validate()?;
    if mesh.building.is_some() {
        let mut scene = super::building_scene::scene(mesh, geometry_materials, settings, step)?;
        crate::plan_trace::columns::attach(&mut scene, mesh, all, settings);
        let layout = &mesh.building.as_ref().unwrap().layout;
        let bottom = layout
            .levels
            .iter()
            .map(|l| l.z)
            .fold(f64::INFINITY, f64::min);
        let top = layout
            .levels
            .iter()
            .map(|l| l.top)
            .fold(f64::NEG_INFINITY, f64::max);
        scene.apply_object_restraints(settings, |id| {
            let c = all.iter().find(|c| c.id == id)?;
            let level = layout.levels.iter().find(|l| l.id == layout.owner(id))?;
            let extent = if c.kind == Kind::Column && c.placement.levels_default() {
                layout.legacy_wall_extent.unwrap_or([level.z, level.top])
            } else {
                layout.extent(c)
            };
            Some((
                (level.z - bottom).abs() < 1e-5,
                (level.top - top).abs() < 1e-5 || (extent[1] - top).abs() < 1e-5,
                extent,
            ))
        });
        let mut explicit = BTreeMap::<usize, u8>::new();
        for c in all.iter().filter(|c| c.kind.is_restraint()) {
            let floor = layout
                .levels
                .iter()
                .find(|f| f.id == layout.owner(c.id))
                .ok_or("Не назначен этаж опоры")?;
            let z = c.placement.slab.z([floor.z, floor.top]);
            let mut path = if c.kind == Kind::PointRestraint {
                vec![]
            } else {
                c.mesh_boundary(step)?
            };
            if c.closed() && !path.is_empty() {
                path.push(path[0]);
            }
            let mut nodes = vec![];
            for (n, p) in scene.xyz.iter().enumerate() {
                if (p[2] - z).abs() > 1e-6 {
                    continue;
                }
                let on_support = if c.kind == Kind::PointRestraint {
                    c.points
                        .first()
                        .is_some_and(|q| (p[0] - q[0]).hypot(p[1] - q[1]) < 1e-5)
                } else {
                    path.windows(2).any(|edge| {
                        let d = [edge[1][0] - edge[0][0], edge[1][1] - edge[0][1]];
                        let length = d[0] * d[0] + d[1] * d[1];
                        if length < 1e-12 {
                            return false;
                        }
                        let t = (((p[0] - edge[0][0]) * d[0] + (p[1] - edge[0][1]) * d[1])
                            / length)
                            .clamp(0., 1.);
                        (p[0] - edge[0][0] - t * d[0]).hypot(p[1] - edge[0][1] - t * d[1]) < 1e-5
                    })
                };
                if !on_support {
                    continue;
                }
                nodes.push(n);
                *explicit.entry(n).or_default() |= c.restraint_mask;
            }
            if nodes.is_empty() {
                return Err(format!(
                    "Опора {}: на границе нет узлов КЭ на отметке {:.0} мм",
                    c.id, z
                ));
            }
            scene.support_nodes.insert(c.id, nodes);
        }
        // Explicit contours define these nodes' restraints. At intersections
        // combine all explicit contours, leaving automatic supports elsewhere.
        for (n, mask) in explicit {
            for a in 0..3 {
                scene.fixed.remove(&(3 * n + a));
                scene.fixed_rotations.remove(&(3 * n + a));
                if mask & (1 << a) != 0 {
                    scene.fixed.insert(3 * n + a);
                }
                if mask & (1 << (a + 3)) != 0 {
                    scene.fixed_rotations.insert(3 * n + a);
                }
            }
        }
        crate::plan_trace::limits::check(
            scene.xyz.len(),
            scene.surfaces.len().saturating_add(scene.beams.len()),
        )?;
        scene.apply_surface_placement(mesh, all);
        scene.apply_beam_offsets(all)?;
        return Ok(scene);
    }
    let mut s = Scene {
        xyz: mesh.nodes.iter().map(|n| [n.xy[0], n.xy[1], 0.]).collect(),
        plan: (0..mesh.nodes.len()).collect(),
        surfaces: vec![],
        fixed: BTreeSet::new(),
        support_nodes: BTreeMap::new(),
        fixed_rotations: Default::default(),
        beams: vec![],
    };
    let indices = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let mut owned = BTreeSet::new();
    for q in &mesh.quads {
        let d = q
            .display_triangles
            .iter()
            .map(|id| indices.get(id).copied().ok_or("Нет тесселяции MITC4"))
            .collect::<Result<Vec<_>, _>>()?;
        if d[0] == d[1]
            || d.iter().any(|i| owned.contains(i))
            || q.nodes.iter().any(|&n| n >= mesh.nodes.len())
        {
            return Err("Некорректная принадлежность четырёхугольника".into());
        }
        if geometry_materials[d[0]] != geometry_materials[d[1]] {
            return Err("Четырёхугольник пересекает границу материалов".into());
        }
        owned.extend(d.iter().copied());
        s.surfaces.push(Surface {
            rigid_z_mm: 0.,
            nodes: q.nodes.to_vec(),
            material: geometry_materials[d[0]],
            display: d,
            support: None,
        });
    }
    for (i, e) in mesh
        .elements
        .iter()
        .enumerate()
        .filter(|(i, _)| !owned.contains(i))
    {
        s.surfaces.push(Surface {
            rigid_z_mm: 0.,
            nodes: e.nodes.to_vec(),
            material: geometry_materials[i],
            display: vec![i],
            support: None,
        });
    }
    let mut used_beam_edges = BTreeMap::new();
    let mut beam_lines = Vec::new();
    for beam in all.iter().filter(|c| c.kind == Kind::Beam) {
        let mut points = beam.mesh_boundary(step)?;
        if beam.closed() && !points.is_empty() {
            points.push(points[0]);
        }
        beam_lines.extend(points.windows(2).map(|p| [p[0], p[1]]));
    }
    let beam_fragments = super::super::meshing::split_constraints(beam_lines)?;
    for beam in all.iter().filter(|c| c.kind == Kind::Beam) {
        let section = settings
            .beams
            .get(&beam.id)
            .ok_or_else(|| format!("Балка {}: не назначена жёсткость", beam.id))?
            .clone();
        let mut points = beam.mesh_boundary(step)?;
        if beam.closed() && points.len() > 1 {
            points.push(points[0]);
        }
        for pair in points.windows(2) {
            let a = pair[0];
            let b = pair[1];
            let d = [b[0] - a[0], b[1] - a[1]];
            let length = d[0].hypot(d[1]);
            if length <= 1e-6 {
                continue;
            }
            let parameter =
                |p: Point| ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (length * length);
            let on = |p: Point| {
                let t = parameter(p);
                t >= -1e-8
                    && t <= 1. + 1e-8
                    && ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / length <= 1e-5
            };
            let mut covered = 0.;
            for &edge in &mesh.edges {
                let p = edge.map(|n| mesh.nodes[n].xy);
                if !on(p[0]) || !on(p[1]) {
                    continue;
                }
                let key = [edge[0].min(edge[1]), edge[0].max(edge[1])];
                let segment = (p[0][0] - p[1][0]).hypot(p[0][1] - p[1][1]);
                if let Some(&id) = used_beam_edges.get(&key) {
                    if id != beam.id {
                        return Err(format!(
                            "Балки {id} и {} совпадают на одном участке",
                            beam.id
                        ));
                    }
                    continue;
                }
                covered += segment;
                used_beam_edges.insert(key, beam.id);
                let nodes = if parameter(p[0]) < parameter(p[1]) {
                    edge
                } else {
                    [edge[1], edge[0]]
                };
                s.beams.push(BeamGeometry {
                    rigid_offset_mm: [0.; 2],
                    roll_angle_rad: beam.column_angle,
                    id: beam.id,
                    nodes,
                    section: section.clone(),
                });
            }
            if (covered - length).abs() > length.max(1.) * 1e-5 {
                let mut cuts = vec![0., 1.];
                cuts.extend(
                    mesh.nodes
                        .iter()
                        .map(|n| n.xy)
                        .filter(|&p| on(p))
                        .map(parameter),
                );
                cuts.extend(
                    beam_fragments
                        .iter()
                        .flatten()
                        .copied()
                        .filter(|&p| on(p))
                        .map(parameter),
                );
                cuts.sort_by(f64::total_cmp);
                cuts.dedup_by(|a, b| (*a - *b).abs() * length < 1e-5);
                let at = |t: f64| [a[0] + t * d[0], a[1] + t * d[1]];
                for interval in cuts.windows(2) {
                    let midpoint = at((interval[0] + interval[1]) * 0.5);
                    if s.beams.iter().any(|e| {
                        e.id == beam.id
                            && super::super::meshing::on_segment(
                                midpoint,
                                e.nodes.map(|n| [s.xyz[n][0], s.xyz[n][1]]),
                            )
                    }) {
                        continue;
                    }
                    // Missing plate connectivity is still an error. Only the
                    // genuinely free span becomes standalone frame elements.
                    if mesh.elements.iter().any(|e| {
                        let p = e.nodes.map(|n| mesh.nodes[n].xy);
                        let cross = |a: Point, b: Point, c: Point| {
                            (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
                        };
                        let sign = cross(p[0], p[1], p[2]).signum();
                        (0..3).all(|i| cross(p[i], p[(i + 1) % 3], midpoint) * sign >= -1e-6)
                    }) {
                        return Err(format!(
                            "Балка {}: нет согласованного ребра внутри плиты. Перестройте сетку",
                            beam.id
                        ));
                    }
                    let count =
                        ((interval[1] - interval[0]) * length / step).ceil().max(1.) as usize;
                    if count > crate::plan_trace::limits::MAX_NODES
                        || s.xyz.len() + count + 1 > crate::plan_trace::limits::MAX_NODES
                    {
                        return Err("Превышен лимит узлов балок".into());
                    }
                    let mut previous: Option<usize> = None;
                    for i in 0..=count {
                        let p =
                            at(interval[0] + (interval[1] - interval[0]) * i as f64 / count as f64);
                        let node = s
                            .xyz
                            .iter()
                            .position(|q| {
                                q[2].abs() < 1e-6 && (q[0] - p[0]).hypot(q[1] - p[1]) < 1e-5
                            })
                            .unwrap_or_else(|| {
                                let n = s.xyz.len();
                                s.xyz.push([p[0], p[1], 0.]);
                                s.plan.push(usize::MAX);
                                n
                            });
                        if let Some(from) = previous {
                            let key = [from.min(node), from.max(node)];
                            if let Some(id) = used_beam_edges.insert(key, beam.id) {
                                if id != beam.id {
                                    return Err(format!(
                                        "Балки {id} и {} совпадают на одном участке",
                                        beam.id
                                    ));
                                }
                            } else {
                                s.beams.push(BeamGeometry {
                                    rigid_offset_mm: [0.; 2],
                                    roll_angle_rad: beam.column_angle,
                                    id: beam.id,
                                    nodes: [from, node],
                                    section: section.clone(),
                                });
                            }
                        }
                        previous = Some(node);
                    }
                }
            }
        }
    }
    if !settings.enabled {
        for (&id, nodes) in mesh
            .support_boundary
            .iter()
            .filter(|(id, _)| !settings.columns.contains_key(id))
        {
            for &node in nodes {
                s.fixed.extend([3 * node, 3 * node + 1, 3 * node + 2]);
            }
            s.support_nodes.insert(id, nodes.clone());
        }
        crate::plan_trace::columns::attach(&mut s, mesh, all, settings);
        s.apply_surface_placement(mesh, all);
        s.apply_beam_offsets(all)?;
        return Ok(s);
    }
    let [lo, hi] = settings.extent();
    // Shared (plan node, vertical level) IDs also join intersecting supports.
    let mut ids = mesh
        .nodes
        .iter()
        .enumerate()
        .map(|(i, _)| ((i, 0_i32), i))
        .collect::<BTreeMap<_, _>>();
    let wall_step = all
        .iter()
        .filter(|c| c.kind == Kind::Support)
        .map(|c| c.mesh_step(step))
        .min_by(f64::total_cmp)
        .unwrap_or(step);
    let layers = (settings.height_mm / wall_step).ceil().max(1.) as i32;
    if layers > 10000 {
        return Err("Слишком много слоёв опор".into());
    }
    let first = if lo < 0. { -layers } else { 0 };
    let last = if hi > 0. { layers } else { 0 };
    let mut walls = BTreeMap::<[usize; 4], Material>::new();
    for (&support, nodes) in &mesh.support_boundary {
        let material = settings
            .materials
            .get(&support)
            .copied()
            .unwrap_or_else(Material::wall_default);
        material.validate()?;
        let boundary = nodes.iter().copied().collect::<BTreeSet<_>>();
        let mut used = BTreeSet::new();
        for &[a, b] in &mesh.constrained_edges {
            if !boundary.contains(&a) || !boundary.contains(&b) {
                continue;
            }
            let c = all
                .iter()
                .find(|c| c.id == support && c.kind == Kind::Support)
                .ok_or("Опора отсутствует")?;
            let ps = c.mesh_boundary(step)?;
            let segments = ps.windows(2).map(|p| [p[0], p[1]]).chain(if c.closed() {
                Some([*ps.last().unwrap(), ps[0]])
            } else {
                None
            });
            let Some(segment) = segments.into_iter().find(|s| {
                super::super::meshing::on_segment(mesh.nodes[a].xy, *s)
                    && super::super::meshing::on_segment(mesh.nodes[b].xy, *s)
            }) else {
                continue;
            };
            // Constraint edges are undirected (often sorted by node ID).
            // Orient every wall cell along its source contour: local X follows
            // that contour, Y is global +Z, and local Z = X cross Y. Keep this
            // winding in Scene so recovery and LIRA use the same wall face.
            let tangent = [segment[1][0] - segment[0][0], segment[1][1] - segment[0][1]];
            let delta = [
                mesh.nodes[b].xy[0] - mesh.nodes[a].xy[0],
                mesh.nodes[b].xy[1] - mesh.nodes[a].xy[1],
            ];
            let (a, b) = if tangent[0] * delta[0] + tangent[1] * delta[1] >= 0. {
                (a, b)
            } else {
                (b, a)
            };

            for k in first..last {
                if s.xyz.len() + 4 > crate::plan_trace::limits::MAX_NODES
                    || s.surfaces
                        .len()
                        .saturating_add(s.beams.len())
                        .saturating_add(1)
                        > crate::plan_trace::limits::MAX_ELEMENTS
                {
                    return Err("Превышен лимит пространственной модели".into());
                }
                let mut ns = [0; 4];
                for (j, (n, z)) in [(a, k), (b, k), (b, k + 1), (a, k + 1)]
                    .into_iter()
                    .enumerate()
                {
                    ns[j] = *ids.entry((n, z)).or_insert_with(|| {
                        let i = s.xyz.len();
                        s.plan.push(n);
                        s.xyz.push([
                            mesh.nodes[n].xy[0],
                            mesh.nodes[n].xy[1],
                            z as f64 * settings.height_mm / layers as f64,
                        ]);
                        i
                    });
                }
                used.extend(ns);
                let mut key = ns;
                key.sort();
                if let Some(previous) = walls.get(&key) {
                    if *previous != material {
                        return Err("Совпадающие опоры имеют разные материалы".into());
                    }
                    continue;
                }
                walls.insert(key, material);
                s.surfaces.push(Surface {
                    rigid_z_mm: 0.,
                    nodes: ns.to_vec(),
                    material,
                    display: vec![],
                    support: Some(support),
                });
            }
        }
        if used.is_empty() {
            return Err(format!("Опора {support}: нет рёбер сетки на её границе"));
        }
        for &n in &used {
            let z = s.xyz[n][2];
            if (z - lo).abs() < 1e-6 {
                s.fixed.extend([3 * n, 3 * n + 1, 3 * n + 2]);
            }
            if (z - hi).abs() < 1e-6 {
                s.fixed.extend([3 * n, 3 * n + 1]);
            }
        }
        s.support_nodes.insert(support, used.into_iter().collect());
    }
    if s.xyz.len() > crate::plan_trace::limits::MAX_NODES
        || s.surfaces.len().saturating_add(s.beams.len()) > crate::plan_trace::limits::MAX_ELEMENTS
    {
        return Err("Превышен лимит пространственной модели".into());
    }
    crate::plan_trace::columns::attach(&mut s, mesh, all, settings);
    crate::plan_trace::limits::check(s.xyz.len(), s.surfaces.len().saturating_add(s.beams.len()))?;
    s.apply_surface_placement(mesh, all);
    s.apply_beam_offsets(all)?;
    Ok(s)
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct SurfaceForces {
    pub nodes: Vec<usize>,
    pub support: Option<u64>,
    pub axes: [V3; 3],
    pub material: Material,
    pub area: f64,
    pub moments: [f64; 3],
    pub shears: [f64; 2],
    pub membrane: [f64; 3],
}
impl SurfaceForces {
    pub fn scalar(&self, field: usize) -> Option<f64> {
        match field {
            1..=3 => Some(self.moments[field - 1]),
            4..=5 => Some(self.shears[field - 4]),
            12..=14 => Some(self.membrane[field - 12]),
            _ => None,
        }
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Case {
    pub translations_mm: Vec<V3>,
    pub rotations_rad: Vec<V3>,
    pub surface_forces: Vec<SurfaceForces>,
    pub membrane: Vec<[f64; 3]>,
    pub support_reactions_kn: BTreeMap<u64, V3>,
    /// True restrained-node reactions in physical scene order, distinct from
    /// plate-level load transfer stored in CaseResult.reaction_kn.
    #[serde(default)]
    pub restrained_reactions_kn: Vec<V3>,
    #[serde(default)]
    pub plate_transfer_kn: Vec<f64>,
    pub beam_forces: Vec<super::beam::Forces>,
}
enum Kernel {
    Dkt(Triangle, Option<super::mitc3::Tri3>),
    Quad(super::mitc4::Quad4),
}
struct Cell3 {
    surface: Surface,
    basis: [V3; 3],
    kernel: Kernel,
    bend: Vec<Row>,
    mem: Vec<Row>,
    dofs: Vec<usize>,
    k: Vec<Vec<f64>>,
    offset: Vec<f64>,
    membrane_base: Option<Vec<V3>>,
}
fn membrane_d(m: Material) -> [[f64; 3]; 3] {
    let a = m.e_mpa * 1000. * m.thickness_mm / 1000. / (1. - m.nu * m.nu);
    [
        [a, a * m.nu, 0.],
        [a * m.nu, a, 0.],
        [0., 0., a * (1. - m.nu) * 0.5],
    ]
}
#[path = "geometric_membrane.rs"]
mod geometric_membrane;

fn membrane_b(grad: &[Point]) -> Vec<Vec<f64>> {
    let mut b = vec![vec![0.; 2 * grad.len()]; 3];
    for (i, g) in grad.iter().enumerate() {
        b[0][2 * i] = g[0];
        b[1][2 * i + 1] = g[1];
        b[2][2 * i] = g[1];
        b[2][2 * i + 1] = g[0];
    }
    b
}
fn value(r: &Row, q: &[f64]) -> f64 {
    r.iter().map(|&(i, a)| q[i] * a).sum()
}
fn scatter(r: &Row, v: f64, f: &mut [f64]) {
    for &(i, a) in r {
        f[i] += a * v;
        super::cut_forces::load(i, a * v);
    }
}
fn rotation_rows(normals: &[V3], next: &mut usize) -> [Row; 3] {
    if normals.is_empty() {
        return std::array::from_fn(|_| vec![]);
    }
    let n = normals[0];
    let nonparallel = normals.iter().any(|v| dot(n, *v).abs() < 1. - 1e-8);
    let axes = if nonparallel {
        vec![[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]]
    } else {
        let axis = if n[0].abs() < 0.8 {
            [1., 0., 0.]
        } else {
            [0., 1., 0.]
        };
        let t = unit(cross(n, axis));
        vec![t, cross(n, t)]
    };
    let mut rows: [Row; 3] = std::array::from_fn(|_| vec![]);
    for axis in axes {
        let id = *next;
        *next += 1;
        for a in 0..3 {
            if axis[a].abs() > 1e-14 {
                rows[a].push((id, axis[a]));
            }
        }
    }
    rows
}
fn project(rows: &[Row; 3], axis: V3) -> Row {
    let mut r = BTreeMap::<usize, f64>::new();
    for a in 0..3 {
        for &(i, v) in &rows[a] {
            *r.entry(i).or_default() += axis[a] * v;
        }
    }
    r.into_iter().filter(|(_, v)| v.abs() > 1e-14).collect()
}
fn translation(n: usize, axis: V3) -> Row {
    (0..3)
        .filter(|&a| axis[a].abs() > 1e-14)
        .map(|a| (3 * n + a, axis[a]))
        .collect()
}
fn add_matrix(
    k: &mut [Vec<f64>],
    index: &BTreeMap<usize, usize>,
    rows: &[Row],
    local: &[Vec<f64>],
) {
    for (i, r) in rows.iter().enumerate() {
        for (j, s) in rows.iter().enumerate() {
            let v = local[i][j];
            for &(a, x) in r {
                for &(b, y) in s {
                    k[index[&a]][index[&b]] += x * v * y;
                }
            }
        }
    }
}
struct BeamCell {
    geometry: BeamGeometry,
    kernel: super::beam::Element,
    rows: Vec<Row>,
    dofs: Vec<usize>,
    k: Vec<Vec<f64>>,
    offset: [f64; 12],
    base_axis: Option<([f64; 12], [[f64; 3]; 2])>,
}
#[cfg(test)]
fn build(
    scene: &Scene,
    kind: ElementType,
) -> Result<(Vec<Cell3>, Vec<BeamCell>, usize, Vec<[Row; 3]>), String> {
    build_with_rigid(scene, kind, &[], &BTreeMap::new())
}
fn build_with_rigid(
    scene: &Scene,
    kind: ElementType,
    rigid: &[crate::plan_trace::columns::Rigid],
    moment_axes: &BTreeMap<usize, [bool; 3]>,
) -> Result<(Vec<Cell3>, Vec<BeamCell>, usize, Vec<[Row; 3]>), String> {
    if kind == ElementType::MindlinCr {
        return Err("P2/CR с промежуточными узлами пока не совместим с MITC4. Для высоты опор выберите DKT / MITC4".into());
    }
    let use_mitc = scene.surfaces.iter().any(|s| s.nodes.len() == 4);
    #[cfg(test)]
    let use_mitc = use_mitc && std::env::var_os("MINI_FEM_LEGACY_DKT").is_none();
    let mut data = vec![];
    let mut normals = vec![vec![]; scene.xyz.len()];
    for s in &scene.surfaces {
        let p = s
            .nodes
            .iter()
            .map(|&i| scene.xyz[i].map(|v| v / 1000.))
            .collect::<Vec<_>>();
        let first_edge = unit(sub(p[1], p[0]));
        let n = unit(cross(first_edge, sub(*p.last().unwrap(), p[0])));
        let wall_axis = cross([0., 0., 1.], n);
        let e1 = if s.support.is_some() && dot(wall_axis, wall_axis) > 1e-24 {
            unit(wall_axis)
        } else {
            first_edge
        };
        let e2 = cross(n, e1);
        let basis = [e1, e2, n];
        let xy = p
            .iter()
            .map(|&v| {
                let d = sub(v, p[0]);
                [dot(d, e1), dot(d, e2)]
            })
            .collect::<Vec<_>>();
        let kernel = if s.nodes.len() == 4 {
            let quad = super::mitc4::Quad4::new(xy.try_into().unwrap()).map_err(|error| {
                let owner = s.support.map_or_else(
                    || format!("Дисплейные треугольники плиты {:?}", s.display),
                    |id| format!("Стена {id}"),
                );
                format!("{owner}. Узлы {:?}. Координаты, м {p:?}. {error}", s.nodes)
            })?;
            Kernel::Quad(quad)
        } else {
            let xy: [Point; 3] = xy.try_into().unwrap();
            Kernel::Dkt(
                Triangle::new(xy)?,
                if use_mitc {
                    Some(super::mitc3::Tri3::new(xy)?)
                } else {
                    None
                },
            )
        };
        for &i in &s.nodes {
            normals[i].push(n);
        }
        data.push((basis, kernel));
    }
    let plate_normals = normals.clone();
    let mut has_beam = vec![false; scene.xyz.len()];
    let mut has_nonvertical_beam = vec![false; scene.xyz.len()];
    for beam in &scene.beams {
        let axis = sub(scene.xyz[beam.nodes[1]], scene.xyz[beam.nodes[0]]);
        let tangent = unit(axis);
        let reference = if tangent[0].abs() < 0.8 {
            [1., 0., 0.]
        } else {
            [0., 1., 0.]
        };
        for &node in &beam.nodes {
            has_beam[node] = true;
            has_nonvertical_beam[node] |= axis[0].hypot(axis[1]) >= 1e-6;
            normals[node].push(tangent);
            // A standalone frame node needs all three rotations, including
            // torsion. It is not a plate node with a redundant drilling DOF.
            normals[node].push(unit(cross(tangent, reference)));
        }
    }
    let mut rigid_node = vec![false; scene.xyz.len()];
    for group in rigid {
        rigid_node[group.master] = true;
        for &node in &group.nodes {
            rigid_node[node] = true;
        }
        normals[group.master].extend([[1., 0., 0.], [0., 1., 0.]]);
    }
    let drilling = plate_normals
        .iter()
        .enumerate()
        .map(|(node, plate)| {
            let Some(&normal) = plate.first().filter(|_| !rigid_node[node]) else {
                return false;
            };
            let column_joint = has_beam[node]
                && !has_nonvertical_beam[node]
                && plate.iter().all(|n| n[2].abs() > 1. - 1e-8);
            let explicit_drilling = (0..3)
                .any(|a| scene.fixed_rotations.contains(&(3 * node + a)) && normal[a].abs() > 1e-8)
                && plate.iter().all(|p| dot(normal, *p).abs() > 1. - 1e-8);
            let applied_drilling = moment_axes.get(&node).is_some_and(|axes| {
                (0..3).any(|a| axes[a] && normal[a].abs() > 1e-8)
                    && plate.iter().all(|p| dot(normal, *p).abs() > 1. - 1e-8)
            });
            column_joint || explicit_drilling || applied_drilling
        })
        .collect::<Vec<_>>();
    let mut drilling_neighbours = BTreeMap::<usize, BTreeSet<usize>>::new();
    if drilling.iter().any(|&enabled| enabled) {
        for surface in &scene.surfaces {
            for &node in surface.nodes.iter().filter(|&&n| drilling[n]) {
                drilling_neighbours
                    .entry(node)
                    .or_default()
                    .extend(surface.nodes.iter().copied().filter(|&n| n != node));
            }
        }
    }
    let mut next = scene.xyz.len() * 3;
    let rotations = normals
        .iter()
        .enumerate()
        .map(|(node, ns)| {
            let Some(neighbours) = drilling_neighbours.remove(&node) else {
                return rotation_rows(ns, &mut next);
            };
            let plate = &plate_normals[node];
            let normal = plate[0];
            let axis = if normal[0].abs() < 0.8 {
                [1., 0., 0.]
            } else {
                [0., 1., 0.]
            };
            let u = unit(cross(normal, axis));
            let v = cross(normal, u);
            // Plate drilling rotation is the continuum in-plane spin, not a free
            // zero-energy DOF. Fit affine membrane translations on its one-ring.
            let mut j = [0.; 3];
            for &n in &neighbours {
                let delta = sub(scene.xyz[n], scene.xyz[node]);
                let dx = dot(delta, u) / 1000.;
                let dy = dot(delta, v) / 1000.;
                j[0] += dx * dx;
                j[1] += dx * dy;
                j[2] += dy * dy;
            }
            let det = j[0] * j[2] - j[1] * j[1];
            if det <= 1e-20 {
                return rotation_rows(ns, &mut next);
            }
            let mut rows = rotation_rows(plate, &mut next);
            let mut spin = BTreeMap::<usize, f64>::new();
            for n in neighbours {
                let delta = sub(scene.xyz[n], scene.xyz[node]);
                let dx = dot(delta, u) / 1000.;
                let dy = dot(delta, v) / 1000.;
                let gx = (j[2] * dx - j[1] * dy) / det;
                let gy = (j[0] * dy - j[1] * dx) / det;
                for axis in 0..3 {
                    let w = 0.5 * (gx * v[axis] - gy * u[axis]);
                    *spin.entry(3 * n + axis).or_default() += w;
                    *spin.entry(3 * node + axis).or_default() -= w;
                }
            }
            for a in 0..3 {
                rows[a].extend(
                    spin.iter()
                        .map(|(&d, &w)| (d, w * normal[a]))
                        .filter(|(_, w)| w.abs() > 1e-14),
                );
            }
            rows
        })
        .collect::<Vec<_>>();
    let mut cells = vec![];
    for (s, (basis, kernel)) in scene.surfaces.iter().zip(data) {
        let [e1, e2, n] = basis;
        let shift = [0., 0., s.rigid_z_mm / 1000.];
        let translate = |node, axis| {
            let mut row = translation(node, axis);
            if s.rigid_z_mm != 0. {
                row.extend(project(&rotations[node], cross(shift, axis)));
            }
            row
        };
        let mut bend = vec![];
        for &i in &s.nodes {
            bend.push(translate(i, n));
            bend.push(project(&rotations[i], e2.map(|v| -v)));
            bend.push(project(&rotations[i], e1));
        }
        let mem = s
            .nodes
            .iter()
            .flat_map(|&i| [translate(i, e1), translate(i, e2)])
            .collect::<Vec<_>>();
        let dofs = bend
            .iter()
            .chain(&mem)
            .flatten()
            .map(|&(i, _)| i)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let index = dofs
            .iter()
            .enumerate()
            .map(|(i, &d)| (d, i))
            .collect::<BTreeMap<_, _>>();
        let mut k = vec![vec![0.; dofs.len()]; dofs.len()];
        let kb: Vec<Vec<f64>> = match &kernel {
            Kernel::Dkt(t, assumed) => assumed
                .as_ref()
                .map_or_else(
                    || t.stiffness(s.material),
                    |element| element.stiffness(s.material),
                )
                .iter()
                .map(|r| r.to_vec())
                .collect(),
            Kernel::Quad(q) => q.stiffness(s.material).iter().map(|r| r.to_vec()).collect(),
        };
        add_matrix(&mut k, &index, &bend, &kb);
        let d = membrane_d(s.material);
        let samples: Vec<(Vec<Point>, f64)> = match &kernel {
            Kernel::Dkt(t, _) => vec![(t.gradients().to_vec(), t.area)],
            Kernel::Quad(q) => {
                let a = 1. / 3_f64.sqrt();
                let mut v = vec![];
                for r in [-a, a] {
                    for s in [-a, a] {
                        let p = q.sample([r, s])?;
                        v.push((p.grad.to_vec(), p.det));
                    }
                }
                v
            }
        };
        let mut km = vec![vec![0.; mem.len()]; mem.len()];
        for (grad, weight) in samples {
            let b = membrane_b(&grad);
            for i in 0..mem.len() {
                for j in 0..mem.len() {
                    for a in 0..3 {
                        for c in 0..3 {
                            km[i][j] += weight * b[a][i] * d[a][c] * b[c][j];
                        }
                    }
                }
            }
        }
        add_matrix(&mut k, &index, &mem, &km);
        let offset = vec![0.; dofs.len()];
        cells.push(Cell3 {
            offset,
            membrane_base: None,
            surface: s.clone(),
            basis,
            kernel,
            bend,
            mem,
            dofs,
            k,
        });
    }
    let mut beams = vec![];
    for geometry in &scene.beams {
        let points = geometry.nodes.map(|n| scene.xyz[n]);
        let mut kernel =
            super::beam::Element::spatial(points[0], points[1], geometry.section.clone())?;
        let (sin, cos) = geometry.roll_angle_rad.sin_cos();
        let [x, y, z] = kernel.basis;
        kernel.basis = [
            x,
            std::array::from_fn(|i| cos * y[i] + sin * z[i]),
            std::array::from_fn(|i| -sin * y[i] + cos * z[i]),
        ];
        let mut rows = vec![];
        let arm = geometry.rigid_arm(&scene.xyz)?.map(|v| v / 1000.);
        for &node in &geometry.nodes {
            for axis in kernel.basis {
                let mut row = translation(node, axis);
                row.extend(project(&rotations[node], cross(arm, axis)));
                rows.push(row);
            }
            for axis in kernel.basis {
                rows.push(project(&rotations[node], axis));
            }
        }
        let dofs = rows
            .iter()
            .flatten()
            .map(|&(id, _)| id)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let index = dofs.iter().enumerate().map(|(i, &id)| (id, i)).collect();
        let local = kernel.k.iter().map(|r| r.to_vec()).collect::<Vec<_>>();
        let mut k = vec![vec![0.; dofs.len()]; dofs.len()];
        add_matrix(&mut k, &index, &rows, &local);
        beams.push(BeamCell {
            base_axis: None,
            offset: [0.; 12],
            geometry: geometry.clone(),
            kernel,
            rows,
            dofs,
            k,
        });
    }
    Ok((cells, beams, next, rotations))
}
impl Cell3 {
    #[cfg(test)]
    fn area(&self) -> f64 {
        match &self.kernel {
            Kernel::Dkt(t, _) => t.area,
            Kernel::Quad(q) => q.area,
        }
    }
    fn local(&self, p: V3, scene: &Scene) -> Point {
        let d = sub(p, scene.xyz[self.surface.nodes[0]].map(|v| v / 1000.));
        [dot(d, self.basis[0]), dot(d, self.basis[1])]
    }
    fn shape(&self, p: Point) -> Option<Vec<f64>> {
        match &self.kernel {
            Kernel::Dkt(t, _) => {
                let l = t.barycentric(p);
                l.iter()
                    .all(|v| (-1e-8..=1. + 1e-8).contains(v))
                    .then(|| l.to_vec())
            }
            Kernel::Quad(q) => q.natural(p).map(|rs| super::mitc4::shape(rs).0.to_vec()),
        }
    }
    fn force(&self, p: Point, force: V3, f: &mut [f64]) {
        let Some(shape) = self.shape(p) else {
            return;
        };
        let normal = dot(force, self.basis[2]);
        let mut bending = vec![0.; self.bend.len()];
        for (i, v) in shape.iter().enumerate() {
            bending[3 * i] = v * normal;
        }
        for (r, v) in self.bend.iter().zip(bending) {
            scatter(r, v, f);
        }
        for (i, v) in shape.into_iter().enumerate() {
            for a in 0..2 {
                scatter(&self.mem[2 * i + a], v * dot(force, self.basis[a]), f);
            }
        }
    }
    fn pressure(&self, force: V3, f: &mut [f64]) {
        match &self.kernel {
            Kernel::Dkt(t, _) => {
                for l in [
                    [2. / 3., 1. / 6., 1. / 6.],
                    [1. / 6., 2. / 3., 1. / 6.],
                    [1. / 6., 1. / 6., 2. / 3.],
                ] {
                    let p = std::array::from_fn(|a| (0..3).map(|i| l[i] * t.xy[i][a]).sum());
                    self.force(p, force.map(|v| v * t.area / 3.), f);
                }
            }
            Kernel::Quad(q) => {
                let a = 1. / 3_f64.sqrt();
                for r in [-a, a] {
                    for s in [-a, a] {
                        let p = q.sample([r, s]).expect("validated quad");
                        self.force(q.position([r, s]), force.map(|v| v * p.det), f);
                    }
                }
            }
        }
    }
    fn nodal_actions(
        &self,
        ci: usize,
        q: &[f64],
        rotations: &[[Row; 3]],
    ) -> Vec<(usize, [f64; 3])> {
        let values: BTreeMap<_, _> = self
            .dofs
            .iter()
            .enumerate()
            .map(|(i, &a)| {
                (
                    a,
                    super::cut_forces::net(
                        ci,
                        a,
                        self.dofs
                            .iter()
                            .enumerate()
                            .map(|(j, &b)| self.k[i][j] * q[b])
                            .sum::<f64>()
                            + self.offset[i],
                    ),
                )
            })
            .collect();
        self.surface
            .nodes
            .iter()
            .map(|&n| {
                let moment = |axis: usize| {
                    rotations[n][axis]
                        .iter()
                        .map(|(d, w)| values.get(d).copied().unwrap_or(0.) * w)
                        .sum::<f64>()
                };
                (
                    n,
                    [
                        -values.get(&(3 * n + 2)).copied().unwrap_or(0.),
                        moment(0),
                        moment(1),
                    ],
                )
            })
            .collect()
    }
    fn recover(&self, q: &[f64]) -> ([f64; 3], [f64; 2], [f64; 3]) {
        let qb = self.bend.iter().map(|r| value(r, q)).collect::<Vec<_>>();
        let (m, v) = match &self.kernel {
            Kernel::Dkt(t, assumed) => {
                let a = std::array::from_fn(|i| qb[i]);
                if let Some(element) = assumed {
                    element.recover(self.surface.material, a)
                } else {
                    (
                        t.moments(self.surface.material, a),
                        t.shears(self.surface.material, a),
                    )
                }
            }
            Kernel::Quad(e) => e.recover(
                self.surface.material,
                std::array::from_fn(|i| qb[i]),
                [0., 0.],
            ),
        };
        let grad = match &self.kernel {
            Kernel::Dkt(t, _) => t.gradients().to_vec(),
            Kernel::Quad(e) => e.sample([0., 0.]).expect("validated quad").grad.to_vec(),
        };
        let b = membrane_b(&grad);
        let uv = self.mem.iter().map(|r| value(r, q)).collect::<Vec<_>>();
        let eps = b
            .iter()
            .map(|r| r.iter().zip(&uv).map(|(a, b)| a * b).sum::<f64>())
            .collect::<Vec<_>>();
        let membrane = if let Some(base) = &self.membrane_base {
            let local = self.local_translations(q);
            let delta = local
                .iter()
                .zip(base)
                .map(|(u, v)| sub(*u, *v))
                .collect::<Vec<_>>();
            geometric_membrane::stress_linearized(self.surface.material, &grad, base, &delta)
        } else {
            membrane_d(self.surface.material).map(|r| (0..3).map(|i| r[i] * eps[i]).sum())
        };
        (m, v, membrane)
    }
}
struct PreparedLoad {
    f: Vec<f64>,
    pressure: Vec<f64>,
    beam_loads: Vec<[f64; 12]>,
}
fn prepare_load(
    mesh: &Mesh,
    geometry: &[super::Cell],
    scene: &Scene,
    cells: &[Cell3],
    beams: &[BeamCell],
    rotations: &[[Row; 3]],
    point_nodes: &BTreeMap<u64, (usize, usize)>,
    all: &[Contour],
    case: &LoadCase,
    index: &Index,
    owner: &BTreeMap<usize, usize>,
    g: f64,
    step: f64,
    ndof: usize,
) -> Result<PreparedLoad, String> {
    crate::plan_trace::fem::progress::check_cancel()?;
    super::cut_forces::begin(all, cells.len() + usize::from(!point_nodes.is_empty()));
    super::cut_forces::record_interface_loads();
    let mut f = vec![0.; ndof];
    let mut pressure = vec![0.; mesh.elements.len()];
    let mut area_pressure = vec![[None; 3]; cells.len()];
    let point = |p: Point,
                 force: f64,
                 f: &mut [f64],
                 required: bool,
                 load: &Contour|
     -> Result<(), String> {
        for i in index.query(
            [p[0] - 1e-9, p[1] - 1e-9, p[0] + 1e-9, p[1] + 1e-9],
            geometry.len(),
        ) {
            if !mesh.load_matches_element(load, i) {
                continue;
            }
            super::cut_forces::active(owner[&i]);
            let c = &cells[owner[&i]];
            let local = c.local(
                [p[0], p[1], scene.xyz[c.surface.nodes[0]][2] / 1000.],
                &scene,
            );
            if c.shape(local).is_some() {
                c.force(local, load.load.as_ref().unwrap().direction.force(force), f);
                return Ok(());
            }
        }
        if !required {
            return Ok(());
        }
        Err(format!(
            "Нагрузка вне плиты: {:.1}, {:.1} мм",
            p[0] * 1000.,
            p[1] * 1000.
        ))
    };
    for load in all
        .iter()
        .filter(|c| c.load.as_ref().is_some_and(|l| l.case_id == case.id))
    {
        let q = load
            .load
            .as_ref()
            .unwrap()
            .magnitude
            .in_units(ForceUnits::KiloNewton, g);
        if !q.is_finite() {
            return Err("Нечисловая нагрузка".into());
        }
        if q == 0. && load.kind != Kind::LoadArea {
            continue;
        }
        let points = load
            .points
            .iter()
            .map(|p| p.map(|v| v / 1000.))
            .collect::<Vec<_>>();
        match load.kind {
            Kind::LoadPoint => {
                let data = load.load.as_ref().unwrap();
                let exact = point_nodes.get(&load.id).copied();
                if data.moment {
                    let (n, cell) = exact
                        .ok_or_else(|| format!("Момент {}: на его отметке нет узла КЭ", load.id))?;
                    let row = &rotations[n][data.direction.index()];
                    if row.is_empty() {
                        return Err(format!(
                            "Момент {}: у узла нет поворота по заданной оси",
                            load.id
                        ));
                    }
                    super::cut_forces::active(cell);
                    scatter(row, q, &mut f);
                } else if let Some((n, cell)) =
                    exact.filter(|_| data.direction != super::super::loads::Direction::Z)
                {
                    super::cut_forces::active(cell);
                    scatter(&translation(n, data.direction.force(1.)), q, &mut f);
                } else {
                    point(points[0], q, &mut f, true, load)?;
                }
            }
            Kind::LoadLine => {
                let segments = points
                    .windows(2)
                    .map(|p| [p[0], p[1]])
                    .chain(if load.closed() {
                        Some([*points.last().unwrap(), points[0]])
                    } else {
                        None
                    });
                for [a, b] in segments {
                    let len = (a[0] - b[0]).hypot(a[1] - b[1]);
                    let n = (len / super::line_load_step_m(all, step)).ceil().max(1.) as usize;
                    if n > 1_000_000 {
                        return Err("Слишком много точек нагрузки".into());
                    }
                    for i in 0..n {
                        let t = (i as f64 + 0.5) / n as f64;
                        point(
                            [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                            q * len / n as f64,
                            &mut f,
                            false,
                            load,
                        )?;
                    }
                }
            }
            Kind::LoadArea => {
                let area = effective_load_area(all, load);
                let candidates = index
                    .query(bbox(&points), geometry.len())
                    .into_iter()
                    .filter(|&i| mesh.load_matches_element(load, i))
                    .map(|i| owner[&i])
                    .collect::<BTreeSet<_>>();
                for i in candidates {
                    let c = &cells[i];
                    let p = polygon(
                        &c.surface
                            .nodes
                            .iter()
                            .map(|&n| [scene.xyz[n][0] / 1000., scene.xyz[n][1] / 1000.])
                            .collect::<Vec<_>>(),
                    );
                    if area.0.iter().any(|a| polygon_area_load_hits(&p, a)) {
                        let delta = area_pressure_delta_for(
                            &mut area_pressure[i][load.load.as_ref().unwrap().direction.index()],
                            q,
                            load.load.as_ref().unwrap().additive,
                        );
                        super::cut_forces::active(i);
                        let direction = load.load.as_ref().unwrap().direction;
                        c.pressure(direction.force(delta), &mut f);
                        if direction == super::super::loads::Direction::Z {
                            for &d in &c.surface.display {
                                pressure[d] += delta;
                            }
                        }
                    }
                }
            }
            _ => {}
        }
    }
    if case.kind == LoadType::SelfWeight {
        for (ci, c) in cells.iter().enumerate() {
            super::cut_forces::active(ci);
            let q = c.surface.material.self_weight_kn_m2(g);
            c.pressure([0., 0., -q], &mut f);
            for &d in &c.surface.display {
                pressure[d] += q;
            }
        }
    }
    #[cfg(test)]
    super::cut_forces::audit_rhs(&f);
    super::cut_forces::active(usize::MAX);
    let beam_loads = beams
        .iter()
        .map(|beam| {
            if case.kind == LoadType::SelfWeight {
                let value = beam
                    .kernel
                    .section
                    .material
                    .weight
                    .in_units(ForceUnits::KiloNewton, g)
                    * beam.kernel.section.area();
                beam.kernel.uniform_load([0., 0., -value])
            } else {
                [0.; 12]
            }
        })
        .collect::<Vec<_>>();
    for (beam, load) in beams.iter().zip(&beam_loads) {
        for (row, &value) in beam.rows.iter().zip(load) {
            scatter(row, value, &mut f);
        }
    }
    Ok(PreparedLoad {
        f,
        pressure,
        beam_loads,
    })
}
pub(super) fn solve(
    mesh: &Mesh,
    geometry: &[super::Cell],
    all: &[Contour],
    cases: &[LoadCase],
    g: f64,
    step: f64,
    kind: ElementType,
    settings: &Settings,
    modal: Option<(&BTreeMap<u64, Material>, &[(u64, f64)])>,
    include_effective_lengths: bool,
    nonlinear_request: Option<&super::NonlinearRequest>,
    buckling_request: Option<&super::BucklingRequest>,
) -> Result<Results, String> {
    super::progress::stage("Подготовка узлов и связей пространственной модели");
    let scene = scene(
        mesh,
        &geometry.iter().map(|c| c.material).collect::<Vec<_>>(),
        settings,
        step,
        all,
    )?;
    let rigid = crate::plan_trace::columns::append_contour_rigid_links(
        crate::plan_trace::columns::rigid(all, settings, mesh, &scene),
        mesh,
        &scene,
    )?;
    super::progress::stage("Локальные матрицы оболочек и стержней");
    let point_loads = all
        .iter()
        .filter(|c| {
            c.kind == Kind::LoadPoint
                && c.load
                    .as_ref()
                    .is_some_and(|l| l.moment || l.direction.index() != 2)
        })
        .collect::<Vec<_>>();
    let mut point_nodes = BTreeMap::new();
    let mut moment_axes = BTreeMap::<usize, [bool; 3]>::new();
    if let Some(building) = mesh.building.as_ref().filter(|_| !point_loads.is_empty()) {
        use rstar::RTree;
        let nodes = RTree::bulk_load(
            scene
                .xyz
                .iter()
                .copied()
                .enumerate()
                .map(|(id, point)| rstar::primitives::GeomWithData::new(point, id))
                .collect(),
        );
        for c in point_loads {
            let Some(&[x, y]) = c.points.first() else {
                continue;
            };
            let z = building.layout.source_z(c, all);
            if let Some(node) = nodes.nearest_neighbor(&[x, y, z]).filter(|node| {
                let p = node.geom();
                (p[0] - x).abs() < 1e-5 && (p[1] - y).abs() < 1e-5 && (p[2] - z).abs() < 1e-5
            }) {
                point_nodes.insert(c.id, node.data);
                let load = c.load.as_ref().unwrap();
                if load.moment {
                    moment_axes.entry(node.data).or_default()[load.direction.index()] = true;
                }
            }
        }
    }
    let (mut cells, mut beams, ndof, rotations) =
        build_with_rigid(&scene, kind, &rigid, &moment_axes)?;
    let mut point_cells = BTreeMap::new();
    if !point_nodes.is_empty() {
        let loaded = point_nodes.values().copied().collect::<BTreeSet<_>>();
        for (cell, c) in cells.iter().enumerate() {
            for &node in &c.surface.nodes {
                if loaded.contains(&node) {
                    point_cells.entry(node).or_insert(cell);
                }
            }
        }
    }
    let point_nodes = point_nodes
        .into_iter()
        .map(|(id, n)| (id, (n, point_cells.get(&n).copied().unwrap_or(cells.len()))))
        .collect::<BTreeMap<_, _>>();
    super::progress::check_cancel()?;
    super::progress::stage("Закрепления и степени свободы");
    let springs = super::foundation::build(mesh, all, g, true, false)?;
    let matrices = cells
        .iter()
        .map(|c| (&c.dofs, &c.k))
        .chain(beams.iter().map(|c| (&c.dofs, &c.k)))
        .chain(springs.iter().map(|c| (&c.dofs, &c.k)))
        .collect::<Vec<_>>();
    let fixed = scene.fixed.clone();
    if mesh.building.is_none()
        && !settings.enabled
        && beams.is_empty()
        && settings.columns.is_empty()
        && nonlinear_request.is_none()
        && buckling_request.is_none()
        && scene.surfaces.iter().all(|s| s.rigid_z_mm == 0.)
    {
        return Err("Оболочечная ветвь требует высоты опор, балок или нелинейного расчёта".into());
    }
    let projection = super::rigid::projection(
        ndof,
        &scene.xyz,
        &rotations,
        &rigid,
        &fixed,
        &scene.fixed_rotations,
    )?;
    let mut diagonal = vec![0.; ndof];
    if rigid.is_empty() {
        for (dofs, k) in &matrices {
            for (i, &d) in dofs.iter().enumerate() {
                diagonal[d] += k[i][i];
            }
        }
    } else {
        for (dofs, k) in &matrices {
            for (i, &a) in dofs.iter().enumerate() {
                for (j, &b) in dofs.iter().enumerate() {
                    if k[i][j] == 0. {
                        continue;
                    }
                    for &(p, u) in &projection[a] {
                        for &(q, v) in &projection[b] {
                            if p == q {
                                diagonal[p] += u * k[i][j] * v;
                            }
                        }
                    }
                }
            }
        }
    }
    let mut active = vec![false; ndof];
    for &(d, _) in projection.iter().flatten() {
        active[d] = true;
    }
    let mut map = vec![usize::MAX; ndof];
    let mut free = 0;
    for (i, _) in active.iter().enumerate().filter(|(_, active)| **active) {
        if diagonal[i] <= 0. || !diagonal[i].is_finite() {
            return Err(format!("Оболочка: свободная степень {i} без жёсткости"));
        }
        map[i] = free;
        free += 1;
    }
    let scale = diagonal
        .iter()
        .map(|v| if *v > 0. { 1. / v.sqrt() } else { 1. })
        .collect::<Vec<_>>();
    crate::plan_trace::fem::progress::stage("Сборка матрицы жёсткости элементов");
    crate::plan_trace::fem::progress::check_cancel()?;
    let mut ts = vec![];
    for (element, (dofs, k)) in matrices.iter().enumerate() {
        if element % 1024 == 0 {
            super::progress::check_cancel()?;
        }
        for (i, &a) in dofs.iter().enumerate() {
            if projection[a].is_empty() {
                continue;
            }
            for (j, &b) in dofs.iter().enumerate() {
                let stiffness = k[i][j];
                if stiffness == 0. {
                    continue;
                }
                for &(p, u) in &projection[a] {
                    let row = map[p];
                    for &(q, v) in &projection[b] {
                        let col = map[q];
                        if row <= col {
                            ts.push(Triplet::new(
                                row,
                                col,
                                u * stiffness * v * scale[p] * scale[q],
                            ));
                        }
                    }
                }
            }
        }
    }
    crate::plan_trace::fem::progress::stage("Сборка разреженной матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    let matrix = SparseColMat::<usize, f64>::try_new_from_triplets(free, free, &ts)
        .map_err(|e| format!("Оболочка: сборка {e:?}"))?;
    // Keep a small reference matrix for the direct-condensation diagnostic only.
    let direct_length_stiffness = (include_effective_lengths
        && settings.length_method == LengthMethod::Spatial
        && free <= 384
        && all
            .iter()
            .any(|c| c.kind == Kind::Column && c.calculate_effective_length))
    .then(|| {
        let mut dense = Mat::<f64>::zeros(free, free);
        for entry in &ts {
            dense[(entry.row, entry.col)] += entry.val;
            if entry.row != entry.col {
                dense[(entry.col, entry.row)] += entry.val;
            }
        }
        dense
    });
    drop(ts);
    crate::plan_trace::fem::progress::stage("Разложение матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    let selected_lengths = include_effective_lengths
        && all.iter().any(|c| {
            matches!(c.kind, Kind::Support | Kind::Column) && c.calculate_effective_length
        });
    let initial_factor =
        if nonlinear_request.is_none() || selected_lengths || buckling_request.is_some() {
            Some(
                matrix
                    .sp_cholesky(Side::Upper)
                    .map_err(|e| format!("Оболочка: проверьте закрепления и связность: {e:?}"))?,
            )
        } else {
            None
        };
    let buckling_elastic = buckling_request.is_some().then_some(matrix);
    #[cfg(test)]
    if std::env::var_os("MINI_FEM_PILOT_PROBES").is_some() {
        let mut probes = Vec::new();
        for (&id, nodes) in &scene.support_nodes {
            let Some(wall) = all.iter().find(|c| c.id == id && c.kind == Kind::Support) else {
                continue;
            };
            let along = sub(
                [wall.points[1][0], wall.points[1][1], 0.],
                [wall.points[0][0], wall.points[0][1], 0.],
            );
            let length = dot(along, along).sqrt();
            if length == 0. {
                continue;
            }
            let tangent = along.map(|v| v / length);
            let normal = [-tangent[1], tangent[0], 0.];
            let low = nodes
                .iter()
                .map(|&n| scene.xyz[n][2])
                .fold(f64::INFINITY, f64::min);
            let high = nodes
                .iter()
                .map(|&n| scene.xyz[n][2])
                .fold(f64::NEG_INFINITY, f64::max);
            for z in [low, high] {
                let edge = nodes
                    .iter()
                    .copied()
                    .filter(|&n| (scene.xyz[n][2] - z).abs() < 1e-5)
                    .collect::<Vec<_>>();
                for moment in [false, true] {
                    let mut rhs = BTreeMap::<usize, f64>::new();
                    for &n in &edge {
                        for axis in 0..3 {
                            let terms = if moment {
                                rotations[n][axis].clone()
                            } else {
                                vec![(3 * n + axis, normal[axis])]
                            };
                            for (d, w) in terms {
                                for &(p, u) in &projection[d] {
                                    let direction = if moment { tangent[axis] } else { 1. };
                                    *rhs.entry(map[p]).or_default() +=
                                        direction * w * u * scale[p] / edge.len() as f64;
                                }
                            }
                        }
                    }
                    if rhs.values().any(|w| w.abs() > 1e-14) {
                        probes.push(rhs);
                    }
                }
            }
        }
        assert!(!probes.is_empty());
        for count in [4, 32, 128] {
            let rhs = Mat::from_fn(free, count, |i, j| {
                probes[j % probes.len()].get(&i).copied().unwrap_or(0.)
            });
            let mut times = Vec::new();
            for _ in 0..3 {
                let start = web_time::Instant::now();
                let x = initial_factor.as_ref().unwrap().solve(rhs.as_ref());
                times.push(start.elapsed().as_secs_f64());
                assert!(x[(0, 0)].is_finite());
                std::hint::black_box(x);
            }
            eprintln!(
                "free={free} walls={} available_probes={} batch={count} seconds={times:?}",
                scene.support_nodes.len(),
                probes.len()
            );
        }
    }
    let owner = cells
        .iter()
        .enumerate()
        .flat_map(|(i, c)| c.surface.display.iter().map(move |&d| (d, i)))
        .collect::<BTreeMap<_, _>>();
    let index = Index::new(geometry, step / 1000.);
    let weighted_loads = |weights: &[(u64, f64)], stage: &str| -> Result<_, String> {
        let mut load = vec![0.; ndof];
        let mut beam_loads = vec![[0.; 12]; beams.len()];
        for &(id, weight) in weights {
            super::progress::check_cancel()?;
            if !weight.is_finite() {
                return Err("Нечисловой коэффициент опорного загружения".into());
            }
            if weight == 0. {
                continue;
            }
            let case = cases
                .iter()
                .find(|c| c.id == id)
                .ok_or("Отсутствует загружение, заданное в настройках расчёта")?;
            super::progress::stage(&format!("{stage}: {}", case.name));
            let prepared = prepare_load(
                mesh,
                geometry,
                &scene,
                &cells,
                &beams,
                &rotations,
                &point_nodes,
                all,
                case,
                &index,
                &owner,
                g,
                step,
                ndof,
            )?;
            for (target, source) in beam_loads.iter_mut().zip(&prepared.beam_loads) {
                for i in 0..12 {
                    target[i] += weight * source[i];
                }
            }
            for (f, p) in load.iter_mut().zip(prepared.f) {
                *f += weight * p;
            }
        }
        Ok((load, beam_loads))
    };
    let buckling = buckling_request
        .zip(buckling_elastic)
        .map(|(request, elastic)| {
            let solve = |rhs: &Mat<f64>| {
                initial_factor
                    .as_ref()
                    .expect("initial elastic factor for buckling")
                    .solve(rhs.as_ref())
            };
            validate_buckling(request, scene.xyz.len())?;
            let (load, beam_loads) =
                weighted_loads(&request.weights, "Общая устойчивость · опорные нагрузки")?;
            super::progress::stage("Общая устойчивость · исходные усилия");
            let force = super::rigid::gather(&projection, &load);
            let mut rhs = Mat::<f64>::zeros(free, 1);
            for (dof, &index) in map.iter().enumerate() {
                if index != usize::MAX {
                    rhs[(index, 0)] = force[dof] * scale[dof];
                }
            }
            let displacement = solve(&rhs);
            let reference = projection
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|&(d, w)| w * scale[d] * displacement[(map[d], 0)])
                        .sum::<f64>()
                })
                .collect::<Vec<_>>();
            super::progress::stage("Общая устойчивость: геометрическая жёсткость");
            let geometric = buckling_geometric::geometric(
                &cells,
                &beams,
                &reference,
                &beam_loads,
                &projection,
                &map,
                &scale,
                free,
            )?;
            super::progress::stage("Общая устойчивость: поиск форм");
            let pairs = super::buckling_spectrum::spectrum(
                free,
                request.forms,
                request.max_factor,
                |q| buckling_geometric::apply_symmetric(&elastic, q),
                |q| buckling_geometric::apply_symmetric(&geometric, q),
                solve,
            )?;
            if pairs.is_empty()
                && free > 128
                && geometric.as_ref().triplet_iter().any(|e| *e.val != 0.)
            {
                super::progress::stage("Общая устойчивость: проверка верхнего КЗУ");
                certify_buckling_cutoff(&elastic, &geometric, request.max_factor)?;
            }
            if !pairs.is_empty() {
                super::progress::stage("Общая устойчивость: чувствительность элементов");
            }
            drop(elastic);
            drop(geometric);
            let sensitivity = buckling_geometric::sensitivities(
                &cells,
                &beams,
                &reference,
                &beam_loads,
                &projection,
                &map,
                &scale,
                &pairs,
            )?;
            let modes = pairs
                .into_iter()
                .zip(sensitivity)
                .map(|((factor, q), sensitivity)| {
                    let mut shape = projection[..3 * scene.xyz.len()]
                        .chunks_exact(3)
                        .map(|rows| {
                            std::array::from_fn(|a| {
                                rows[a]
                                    .iter()
                                    .map(|&(d, w)| w * scale[d] * q[map[d]])
                                    .sum::<f64>()
                            })
                        })
                        .collect::<Vec<_>>();
                    let peak = shape
                        .iter()
                        .map(|p| p[0].hypot(p[1]).hypot(p[2]))
                        .fold(0., f64::max);
                    if peak > 0. {
                        for point in &mut shape {
                            *point = point.map(|v| v / peak);
                        }
                    }
                    super::BucklingMode {
                        factor,
                        shape,
                        sensitivity,
                    }
                })
                .collect();
            Ok(super::BucklingResult {
                caption: request.caption.clone(),
                modes,
                max_factor: request.max_factor,
                requested: request.forms,
            })
        });
    super::progress::check_cancel()?;
    let base = if let Some(request) = nonlinear_request {
        super::progress::stage("Геометрическая нелинейность: базовое сочетание");
        let (load, base_beam_loads) = weighted_loads(
            &request.weights,
            "Геометрическая нелинейность · опорные нагрузки",
        )?;
        Some(nonlinear::solve(
            &springs,
            &mut cells,
            &mut beams,
            &projection,
            &map,
            &scale,
            free,
            &load,
            &base_beam_loads,
        )?)
    } else {
        None
    };
    let factor = base
        .as_ref()
        .map(|base| &base.factor)
        .or(initial_factor.as_ref())
        .expect("linear or nonlinear factor");
    let offset_case = LoadCase {
        id: u64::MAX,
        name: "Касательная: свободный член".into(),
        kind: LoadType::Permanent,
        gamma_f: 1.,
        gamma_l: 1.,
    };
    if base.is_some()
        && all
            .iter()
            .any(|c| c.load.as_ref().is_some_and(|l| l.case_id == u64::MAX))
    {
        return Err("Зарезервированный ID загружения".into());
    }
    let solve_cases = cases
        .iter()
        .cloned()
        .chain(base.as_ref().map(|_| offset_case))
        .collect::<Vec<_>>();
    let same_plane = crate::plan_trace::foundation::same_elevation;
    let mut cut_planes = Vec::<(u64, f64, Vec<Contour>)>::new();
    let mut cuts_by_floor = BTreeMap::<u64, Vec<usize>>::new();
    for contour in all.iter().filter(|c| super::cut_forces::is_cut(c)) {
        let floor = mesh.contour_floor(contour.id);
        let z = mesh
            .building
            .as_ref()
            .map_or(0., |data| data.layout.source_z(contour, all));
        let indices = cuts_by_floor.entry(floor).or_default();
        if let Some(&i) = indices.iter().find(|&&i| same_plane(cut_planes[i].1, z)) {
            cut_planes[i].2.push(contour.clone());
        } else {
            indices.push(cut_planes.len());
            cut_planes.push((floor, z, vec![contour.clone()]));
        }
    }
    for (floor, _, sources) in &mut cut_planes {
        if sources.iter().any(|c| c.kind == Kind::Auxiliary) {
            sources.extend(
                all.iter()
                    .filter(|c| c.kind == Kind::Support && mesh.contour_floor(c.id) == *floor)
                    .cloned(),
            );
        }
    }
    let cut_by_cell = cells
        .iter()
        .map(|c| {
            let display = *c.surface.display.first()?;
            let z = scene.xyz[c.surface.nodes[0]][2];
            cuts_by_floor
                .get(&mesh.element_floor(display))?
                .iter()
                .copied()
                .find(|&i| same_plane(cut_planes[i].1, z))
        })
        .collect::<Vec<_>>();
    let mut result = super::parallel_cases::map(
        &solve_cases,
        free.max(mesh.nodes.len() * 3).max(cells.len() * 12),
        |case| {
            super::progress::stage(&format!("Сборка вектора нагрузок: {}", case.name));
            let PreparedLoad {
                f,
                pressure,
                beam_loads,
            } = prepare_load(
                mesh,
                geometry,
                &scene,
                &cells,
                &beams,
                &rotations,
                &point_nodes,
                all,
                case,
                &index,
                &owner,
                g,
                step,
                ndof,
            )?;
            let mut rhs = Mat::<f64>::zeros(free, 1);
            let projected_f = super::rigid::gather(&projection, &f);
            for i in 0..ndof {
                if map[i] != usize::MAX {
                    rhs[(map[i], 0)] = projected_f[i] * scale[i];
                }
            }
            crate::plan_trace::fem::progress::stage("Вычисление неизвестных");
            crate::plan_trace::fem::progress::check_cancel()?;
            let x = factor.solve(rhs.as_ref());
            let parent_q = (0..ndof)
                .map(|i| {
                    if map[i] == usize::MAX {
                        0.
                    } else {
                        x[(map[i], 0)] * scale[i]
                    }
                })
                .collect::<Vec<_>>();
            let mut q = projection
                .iter()
                .map(|r| value(r, &parent_q))
                .collect::<Vec<_>>();
            if let Some(base) = &base {
                for (u, offset) in q.iter_mut().zip(&base.offset_q) {
                    *u += offset;
                }
            }
            if q.iter().any(|v| !v.is_finite()) {
                return Err("Оболочка: нечисловое решение".into());
            }
            crate::plan_trace::fem::progress::stage("Восстановление усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut residual = f.iter().map(|v| -v).collect::<Vec<_>>();
            let mut moments = vec![[0.; 3]; mesh.elements.len()];
            let mut shears = vec![[0.; 2]; mesh.elements.len()];
            let mut membrane = moments.clone();
            let mut slab_transfer = vec![0.; mesh.nodes.len()];
            let mut cuts = cut_planes
                .iter()
                .map(|(_, _, sources)| super::cut_forces::Collector::new(sources))
                .collect::<Vec<_>>();
            let mut surface_forces = Vec::with_capacity(cells.len());
            for (ci, c) in cells.iter().enumerate() {
                let cut = cut_by_cell[ci];
                for (i, &a) in c.dofs.iter().enumerate() {
                    residual[a] += c.offset[i];
                    for (j, &b) in c.dofs.iter().enumerate() {
                        residual[a] += c.k[i][j] * q[b];
                    }
                }
                if !c.surface.display.is_empty() {
                    let actions = c.nodal_actions(ci, &q, &rotations);
                    for &(n, v) in &actions {
                        slab_transfer[n] += v[0];
                    }
                    if let Some(i) = cut {
                        let poly = c
                            .surface
                            .nodes
                            .iter()
                            .map(|&n| [scene.xyz[n][0], scene.xyz[n][1]])
                            .collect::<Vec<_>>();
                        cuts[i].add(
                            &poly,
                            actions
                                .into_iter()
                                .map(|(n, v)| ([scene.xyz[n][0], scene.xyz[n][1]], v)),
                        );
                    }
                }
                let (m, v, n) = c.recover(&q);
                let p = c
                    .surface
                    .nodes
                    .iter()
                    .map(|&n| scene.xyz[n])
                    .collect::<Vec<_>>();
                let area = (1..p.len() - 1)
                    .map(|i| {
                        dot(cross(sub(p[i], p[0]), sub(p[i + 1], p[0])), c.basis[2]).abs() * 0.5
                    })
                    .sum();
                surface_forces.push(SurfaceForces {
                    nodes: c.surface.nodes.clone(),
                    support: c.surface.support,
                    axes: c.basis,
                    material: c.surface.material,
                    area,
                    moments: m.map(|v| -v),
                    shears: v.map(|v| -v),
                    membrane: n,
                });
                let [a, b, _] = c.basis;
                // All result-display cells are horizontal; rotate local tensors to XY.
                let tensor = |t: [f64; 3]| {
                    [
                        a[0] * a[0] * t[0] + b[0] * b[0] * t[1] + 2. * a[0] * b[0] * t[2],
                        a[1] * a[1] * t[0] + b[1] * b[1] * t[1] + 2. * a[1] * b[1] * t[2],
                        a[0] * a[1] * t[0]
                            + b[0] * b[1] * t[1]
                            + (a[0] * b[1] + b[0] * a[1]) * t[2],
                    ]
                };
                if let Some(i) = cut {
                    let cuts = &mut cuts[i];
                    let qb = c.bend.iter().map(|r| value(r, &q)).collect::<Vec<_>>();
                    let z = scene.xyz[c.surface.nodes[0]][2] / 1000.;
                    cuts.direct_fields(
                        |p| {
                            let local = c.local([p[0] / 1000., p[1] / 1000., z], &scene);
                            let material = c.surface.material;
                            let (m, v) = match &c.kernel {
                                Kernel::Dkt(t, Some(e)) => e.recover_at(
                                    material,
                                    std::array::from_fn(|i| qb[i]),
                                    t.barycentric(local),
                                ),
                                Kernel::Dkt(t, None) => {
                                    let d = std::array::from_fn(|i| qb[i]);
                                    (
                                        t.moments_at(material, d, t.barycentric(local)),
                                        t.shears(material, d),
                                    )
                                }
                                Kernel::Quad(e) => e.recover(
                                    material,
                                    std::array::from_fn(|i| qb[i]),
                                    e.natural(local)?,
                                ),
                            };
                            Some((
                                tensor(m).map(|v| -v),
                                [-(a[0] * v[0] + b[0] * v[1]), -(a[1] * v[0] + b[1] * v[1])],
                            ))
                        },
                        matches!(c.kernel, Kernel::Dkt(_, None)),
                    );
                    if matches!(c.kernel, Kernel::Dkt(_, None)) {
                        cuts.kirchhoff();
                    }
                    cuts.field(
                        tensor(m).map(|v| -v),
                        [-(a[0] * v[0] + b[0] * v[1]), -(a[1] * v[0] + b[1] * v[1])],
                    );
                }
                for &d in &c.surface.display {
                    moments[d] = tensor(m).map(|v| -v);
                    shears[d] = [-(a[0] * v[0] + b[0] * v[1]), -(a[1] * v[0] + b[1] * v[1])];
                    membrane[d] = tensor(n);
                }
            }
            // Plate-only nodal actions, before frame forces are added below.
            let plate_transfer_kn = (0..scene.xyz.len())
                .map(|n| slab_transfer.get(n).copied().unwrap_or(0.))
                .collect::<Vec<_>>();
            let slab_nodes = scene
                .surfaces
                .iter()
                .filter(|s| s.support.is_none())
                .flat_map(|s| s.nodes.iter().copied())
                .collect::<BTreeSet<_>>();
            let mut column_reactions = BTreeMap::<u64, (f64, V3)>::new();
            let mut beam_forces = vec![];
            for (beam, load) in beams.iter().zip(&beam_loads) {
                for (row, &value) in beam.rows.iter().zip(load) {
                    for &(d, weight) in row {
                        if d < 3 * mesh.nodes.len() && d % 3 == 2 {
                            slab_transfer[d / 3] += weight * value;
                        }
                    }
                }
                for (row, &f) in beam.rows.iter().zip(&beam.offset) {
                    scatter(row, f, &mut residual);
                    for &(d, w) in row {
                        if d < 3 * mesh.nodes.len() && d % 3 == 2 {
                            slab_transfer[d / 3] -= w * f;
                        }
                    }
                }
                for (i, &a) in beam.dofs.iter().enumerate() {
                    for (j, &b) in beam.dofs.iter().enumerate() {
                        residual[a] += beam.k[i][j] * q[b];
                        if a < 3 * mesh.nodes.len() && a % 3 == 2 {
                            slab_transfer[a / 3] -= beam.k[i][j] * q[b];
                        }
                    }
                }
                let local = std::array::from_fn(|i| value(&beam.rows[i], &q));
                let points = beam
                    .geometry
                    .nodes
                    .map(|n| [scene.xyz[n][0], scene.xyz[n][1]]);
                if settings.columns.contains_key(&beam.geometry.id) {
                    for end in 0..2 {
                        let node = beam.geometry.nodes[end];
                        if !slab_nodes.contains(&node) {
                            continue;
                        }
                        let force: V3 = std::array::from_fn(|axis| {
                            (0..3)
                                .map(|a| {
                                    let i = 6 * end + a;
                                    let action = (0..12)
                                        .map(|j| beam.kernel.k[i][j] * local[j])
                                        .sum::<f64>()
                                        - load[i]
                                        + beam.offset[i];
                                    -beam.kernel.basis[a][axis] * action
                                })
                                .sum()
                        });
                        let z = scene.xyz[node][2];
                        let entry = column_reactions
                            .entry(beam.geometry.id)
                            .or_insert((z, [0.; 3]));
                        if z > entry.0 + 1e-6 {
                            *entry = (z, [0.; 3]);
                        }
                        if (z - entry.0).abs() < 1e-6 {
                            for a in 0..3 {
                                entry.1[a] += force[a];
                            }
                        }
                    }
                }
                let mut recovered = beam.kernel.recover(
                    beam.geometry.id,
                    points,
                    local,
                    std::array::from_fn(|i| load[i] - beam.offset[i]),
                );
                beam.add_second_order(&mut recovered, &local);
                beam_forces.push(recovered);
            }
            let ground = super::foundation::force(&springs, &q);
            for (r, f) in residual.iter_mut().zip(&ground) {
                *r += f;
            }
            let projected_residual = super::rigid::gather(&projection, &residual);
            let error = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (projected_residual[i] * scale[i]).powi(2))
                .sum::<f64>()
                .sqrt();
            let norm = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (projected_f[i] * scale[i]).powi(2))
                .sum::<f64>()
                .sqrt();
            let norm = base.as_ref().map_or(norm, |b| norm.max(b.reference_norm));
            let relative_residual = error / norm.max(1e-20);
            if relative_residual > 1e-6 && error > 1e-10 {
                return Err(format!("Оболочка: невязка {relative_residual:.3e}"));
            }
            let mut support_reactions = BTreeMap::<u64, V3>::new();
            for (&id, ns) in &scene.support_nodes {
                let r = std::array::from_fn(|a| {
                    ns.iter()
                        .filter(|&&n| fixed.contains(&(3 * n + a)))
                        .map(|&n| {
                            let count = scene
                                .support_nodes
                                .values()
                                .filter(|v| v.contains(&n))
                                .count();
                            residual[3 * n + a] / count as f64
                        })
                        .sum()
                });
                support_reactions.insert(id, r);
            }
            let mut external = vec![[0.; 3]; scene.xyz.len()];
            let mut reactions = external.clone();
            for n in 0..scene.xyz.len() {
                for a in 0..3 {
                    external[n][a] = f[3 * n + a];
                    if fixed.contains(&(3 * n + a)) {
                        reactions[n][a] = residual[3 * n + a];
                    } else {
                        reactions[n][a] = -ground[3 * n + a];
                    }
                }
            }
            let applied_kn = -external.iter().map(|f| f[2]).sum::<f64>();
            let mut reaction_kn = vec![0.; mesh.nodes.len()];
            for n in 0..scene.xyz.len() {
                if let Some(value) = reaction_kn.get_mut(scene.plan[n]) {
                    *value -= reactions[n][2];
                }
            }
            if settings.enabled || mesh.building.is_some() {
                let boundary: BTreeSet<_> =
                    mesh.support_boundary.values().flatten().copied().collect();
                reaction_kn.fill(0.);
                for n in 0..mesh.nodes.len() {
                    reaction_kn[n] = ground[3 * n + 2];
                }
                for &n in &boundary {
                    reaction_kn[n] = slab_transfer[n];
                }
                for (&id, nodes) in &mesh.support_boundary {
                    let vertical = nodes
                        .iter()
                        .map(|&n| {
                            let count = mesh
                                .support_boundary
                                .values()
                                .filter(|v| v.contains(&n))
                                .count()
                                .max(1);
                            -(slab_transfer[n] - ground[3 * n + 2]) / count as f64
                        })
                        .sum();
                    support_reactions.entry(id).or_insert([0.; 3])[2] = vertical;
                }
            }
            for (id, (_, force)) in column_reactions {
                support_reactions.insert(id, force);
            }
            let balance_kn = applied_kn - reactions.iter().map(|f| f[2]).sum::<f64>();
            let mut balance_m_knm = [0.; 2];
            for n in 0..scene.xyz.len() {
                let force = std::array::from_fn(|a| external[n][a] + reactions[n][a]);
                let moment = cross(scene.xyz[n].map(|v| v / 1000.), force);
                for a in 0..2 {
                    balance_m_knm[a] += moment[a];
                }
            }
            #[cfg(test)]
            if std::env::var_os("MINI_FEM_CUT_DEBUG").is_some() {
                std::fs::write(
                    format!("target/cut_transfer_{}.json", case.id),
                    serde_json::to_vec(&slab_transfer).unwrap(),
                )
                .unwrap();
            }
            Ok(CaseResult {
                punching: cuts.into_iter().flat_map(|c| c.finish()).collect(),
                rotations_rad: rotations
                    .iter()
                    .take(mesh.nodes.len())
                    .map(|rows| {
                        std::array::from_fn(|a| rows[a].iter().map(|&(i, w)| q[i] * w).sum())
                    })
                    .collect(),
                id: case.id,
                w_mm: (0..mesh.nodes.len())
                    .map(|n| -q[3 * n + 2] * 1000.)
                    .collect(),
                moments,
                shears,
                reaction_kn,
                loads_kn_m2: pressure,
                foundation_pressure_kn_m2: foundation::pressures(&springs, &q, mesh.elements.len()),
                applied_kn,
                balance_kn,
                balance_m_knm,
                relative_residual,
                shell: Some(Case {
                    rotations_rad: rotations
                        .iter()
                        .map(|rows| std::array::from_fn(|a| value(&rows[a], &q)))
                        .collect(),
                    surface_forces,
                    translations_mm: (0..scene.xyz.len())
                        .map(|n| std::array::from_fn(|a| q[3 * n + a] * 1000.))
                        .collect(),
                    membrane,
                    beam_forces,
                    support_reactions_kn: support_reactions,
                    restrained_reactions_kn: reactions,
                    plate_transfer_kn,
                }),
            })
        },
    )?;
    let nonlinear = if let (Some(base), Some(request)) = (&base, nonlinear_request) {
        let offset = result
            .pop()
            .ok_or("Нет свободного члена касательной модели")?;
        Some(super::NonlinearResult {
            caption: request.caption.clone(),
            weights: request.weights.clone(),
            steps: base.steps,
            iterations: base.iterations,
            offset: Box::new(offset),
        })
    } else {
        None
    };
    let modal = modal.map(|(materials, weights)| {
        super::progress::check_cancel()?;
        super::progress::stage("Собственные формы и частоты");
        let result = (|| {
            let gravity_loads = all
                .iter()
                .filter(|c| {
                    c.load
                        .as_ref()
                        .is_none_or(|l| !l.moment && l.direction.index() == 2)
                })
                .cloned()
                .collect::<Vec<_>>();
            let loads = super::lira_loads_for_scene(
                mesh,
                &gravity_loads,
                materials,
                cases,
                g,
                step,
                &scene,
                geometry,
            )?;
            let mass = modal_mass(&scene, loads, cases, weights, g)?;
            modal_spectrum(mass, &projection, &map, &scale, free, |rhs| {
                factor.solve(rhs.as_ref())
            })
        })();
        super::progress::stage("Собственные формы и частоты: конец");
        result
    });
    let lengths = if selected_lengths {
        super::progress::stage("Расчётные длины пилонов и колонн");
        let solve = |rhs: &Mat<f64>| {
            initial_factor
                .as_ref()
                .expect("elastic factor for effective lengths")
                .solve(rhs.as_ref())
        };
        let result = match settings.length_method {
            LengthMethod::Plane => effective_length::compute(
                &scene,
                all,
                mesh.building.is_some(),
                settings,
                &rotations,
                &projection,
                &map,
                &scale,
                free,
                solve,
            ),
            LengthMethod::Spatial => {
                super::progress::stage("Пространственная устойчивость стеновых панелей");
                effective_length::wall_lengths(
                    &scene,
                    all,
                    mesh.building.is_some(),
                    &projection,
                    &map,
                    &scale,
                    free,
                    &solve,
                    true,
                )
                .and_then(|mut rows| {
                    super::progress::stage("Пространственная устойчивость колонн");
                    let columns = effective_length_columns::compute(
                        &scene,
                        &beams,
                        all,
                        &projection,
                        &map,
                        &scale,
                        free,
                        direct_length_stiffness.as_ref(),
                        solve,
                    )?;
                    rows.extend(spatial_column_lengths(columns));
                    Ok(rows)
                })
            }
        };
        super::progress::stage("Расчётные длины пилонов и колонн: конец");
        Some(result)
    } else {
        None
    };
    Ok(Results {
        buckling,
        nonlinear,
        lengths,
        cases: result,
        modal,
        free_dofs: free,
    })
}

fn validate_buckling(request: &super::BucklingRequest, nodes: usize) -> Result<(), String> {
    if !(1..=30).contains(&request.forms)
        || !request.max_factor.is_finite()
        || request.max_factor <= 0.
    {
        return Err("Устойчивость: задайте от 1 до 30 форм и положительный верхний КЗУ".into());
    }
    if !request.weights.iter().any(|&(_, w)| w != 0.) {
        return Err("Устойчивость: в выбранном виде нет ненулевых коэффициентов загружений".into());
    }
    if nodes.saturating_mul(request.forms) > 4_000_000 {
        return Err("Устойчивость: уменьшите число форм для этой сетки".into());
    }
    Ok(())
}

fn certify_buckling_cutoff(
    elastic: &SparseColMat<usize, f64>,
    geometric: &SparseColMat<usize, f64>,
    max_factor: f64,
) -> Result<(), String> {
    // If K − max_factor·G is positive definite, no positive root is in range.
    // Use this check only when the iterative search would return an empty list.
    super::progress::check_cancel()?;
    let entries = elastic
        .as_ref()
        .triplet_iter()
        .map(|e| Triplet::new(e.row, e.col, *e.val))
        .chain(
            geometric
                .as_ref()
                .triplet_iter()
                .map(|e| Triplet::new(e.row, e.col, -max_factor * e.val)),
        )
        .collect::<Vec<_>>();
    if entries.iter().any(|e| !e.val.is_finite()) {
        return Err("Не удалось проверить верхний КЗУ: нечисловая сдвинутая жёсткость".into());
    }
    let shifted = SparseColMat::try_new_from_triplets(elastic.nrows(), elastic.ncols(), &entries)
        .map_err(|e| format!("Проверка верхнего КЗУ: {e:?}"))?;
    drop(entries);
    shifted.sp_cholesky(Side::Upper).map_err(|_| {
        "Не удалось подтвердить отсутствие форм до заданного КЗУ. Спектральный поиск не разрешил весь диапазон.".to_string()
    })?;
    Ok(())
}

fn spatial_column_lengths(
    columns: Vec<effective_length_columns::ColumnResult>,
) -> Vec<effective_length::Length> {
    use effective_length::Length;
    use effective_length_columns::ModeKind;
    let mut rows = Vec::new();
    for column in columns {
        let base = |axis: &str| Length {
            id: column.id,
            axis: axis.into(),
            segment: None,
            length_m: column.length_m,
            bottom_m: column.bottom_m,
            effective_m: None,
            critical_kn: None,
            diagnostic: None,
        };
        if let Some(reason) = column.diagnostic {
            rows.push(Length {
                diagnostic: Some(reason),
                ..base("Колонна")
            });
        }
        for (i, mode) in column.modes.into_iter().enumerate() {
            let axis = match mode.kind {
                ModeKind::Y => "y",
                ModeKind::Z => "z",
                ModeKind::CoupledBending => "Смешанный изгиб",
                ModeKind::BendingTorsion => "Изгиб с кручением",
                ModeKind::Torsion => "Кручение",
            };
            rows.push(Length {
                effective_m: mode.effective_m,
                critical_kn: Some(mode.critical_kn),
                diagnostic: Some(format!(
                    "Форма {} · доли геометрической энергии y {:.1}% / z {:.1}% / кручение {:.1}% · относительное смещение концов {:.1}%",
                    i + 1,
                    mode.bending_y_fraction * 100.,
                    mode.bending_z_fraction * 100.,
                    mode.torsion_fraction * 100.,
                    mode.sway_fraction * 100.,
                )),
                ..base(axis)
            });
        }
    }
    rows
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rigid_transverse_arm_preserves_rigid_motion_and_eccentric_load_work() {
        let section = super::super::beam::Section {
            material: Material::default(),
            width_mm: 200.,
        };
        let scene = Scene {
            xyz: vec![[0., 0., 0.], [3000., 0., 0.]],
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            fixed_rotations: Default::default(),
            support_nodes: Default::default(),
            beams: vec![BeamGeometry {
                id: 1,
                nodes: [0, 1],
                section,
                roll_angle_rad: 0.,
                rigid_offset_mm: [200., -300.],
            }],
        };
        let (_, beams, n, rotations) = build(&scene, ElementType::Dkt).unwrap();
        let beam = &beams[0];
        let mut q = vec![0.; n];
        // Unit rigid-body rotation about global X: nodal translations vanish on X.
        for node in 0..2 {
            for &(d, w) in &rotations[node][0] {
                q[d] += w;
            }
        }
        let local: [f64; 12] = std::array::from_fn(|i| value(&beam.rows[i], &q));
        assert!((local[1] - 0.3).abs() < 1e-12);
        assert!((local[2] - 0.2).abs() < 1e-12);
        let action: [f64; 12] =
            std::array::from_fn(|i| (0..12).map(|j| beam.kernel.k[i][j] * local[j]).sum());
        assert!(action.iter().all(|v| v.abs() < 1e-6));
        let f = beam.kernel.uniform_load([0., 0., -10.]);
        let work: f64 = local.iter().zip(f).map(|(q, f)| q * f).sum();
        assert!((work + 6.).abs() < 1e-10, "10 kN/m * 3m * 0.2m arm");
    }
    #[test]
    fn membrane_patch_and_rigid_translation_have_exact_energy_without_drilling_springs() {
        let material = Material::default();
        // A vertical rectangle exercises local/global transformations as well.
        for xyz in [
            vec![
                [0., 0., 0.],
                [2000., 0., 0.],
                [2000., 2000., 0.],
                [0., 2000., 0.],
            ],
            vec![
                [0., 0., 0.],
                [2000., 0., 0.],
                [2000., 0., 2000.],
                [0., 0., 2000.],
            ],
        ] {
            let scene = Scene {
                xyz,
                plan: vec![0, 1, 2, 3],
                surfaces: vec![Surface {
                    rigid_z_mm: 0.,
                    nodes: vec![0, 1, 2, 3],
                    material,
                    display: vec![],
                    support: None,
                }],
                fixed: BTreeSet::new(),
                beams: vec![],
                support_nodes: BTreeMap::new(),
                fixed_rotations: Default::default(),
            };
            let (cells, _, n, _) = build(&scene, ElementType::Dkt).unwrap();
            let c = &cells[0];
            let mut q = vec![0.; n];
            for node in 0..4 {
                for a in 0..3 {
                    q[3 * node + a] = [0.001, 0.002, -0.003][a];
                }
            }
            let energy = |q: &[f64]| {
                c.dofs
                    .iter()
                    .enumerate()
                    .map(|(i, &a)| {
                        c.dofs
                            .iter()
                            .enumerate()
                            .map(|(j, &b)| 0.5 * q[a] * c.k[i][j] * q[b])
                            .sum::<f64>()
                    })
                    .sum::<f64>()
            };
            assert!(energy(&q).abs() < 1e-10);
            let eps = [0.001, 0.002, 0.003];
            for node in 0..4 {
                let p = scene.xyz[node].map(|v| v / 1000.);
                let x = dot(p, c.basis[0]);
                let y = dot(p, c.basis[1]);
                let u = eps[0] * x + 0.5 * eps[2] * y;
                let v = eps[1] * y + 0.5 * eps[2] * x;
                for a in 0..3 {
                    q[3 * node + a] = u * c.basis[0][a] + v * c.basis[1][a];
                }
            }
            let d = membrane_d(material);
            let expected = 0.5
                * c.area()
                * (0..3)
                    .map(|a| (0..3).map(|b| eps[a] * d[a][b] * eps[b]).sum::<f64>())
                    .sum::<f64>();
            assert!((energy(&q) - expected).abs() < 1e-9 * expected);
            let (_, _, recovered) = c.recover(&q);
            for a in 0..3 {
                assert!(
                    (recovered[a] - (0..3).map(|b| d[a][b] * eps[b]).sum::<f64>()).abs() < 1e-8
                );
            }
        }
    }
}
#[cfg(test)]
mod nodal_action_audit {
    use super::*;
    #[test]
    fn actual_shell_nodal_actions_equal_physical_energy_derivatives() {
        for orientation in 0..3 {
            for topology in 0..3 {
                let base = vec![
                    [0., 0., 0.],
                    [1700., 200., 0.],
                    [2100., 1500., 0.],
                    [100., 1300., 0.],
                ];
                let xyz = base
                    .into_iter()
                    .map(|p| match orientation {
                        0 => p,
                        1 => [p[0], 0.8 * p[1], 0.6 * p[1]],
                        _ => [p[0], 0., p[1]],
                    })
                    .collect::<Vec<_>>();
                let material = Material::default();
                let mut surfaces = vec![];
                if topology != 1 {
                    surfaces.push(Surface {
                        rigid_z_mm: 0.,
                        nodes: vec![0, 1, 2],
                        material,
                        display: vec![0],
                        support: None,
                    });
                }
                if topology != 0 {
                    surfaces.push(Surface {
                        rigid_z_mm: 0.,
                        nodes: vec![0, 1, 2, 3],
                        material,
                        display: vec![1],
                        support: None,
                    });
                }
                let scene = Scene {
                    xyz,
                    plan: vec![0, 1, 2, 3],
                    surfaces,
                    fixed: BTreeSet::new(),
                    beams: vec![],
                    support_nodes: BTreeMap::new(),
                    fixed_rotations: Default::default(),
                };
                let (cells, _, ndof, rotations) = build(&scene, ElementType::Dkt).unwrap();
                let mut marker = Contour {
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
                    refinement: Some(super::super::super::model::Refinement {
                        inactive_edges: vec![],

                        name: "Продавливание: торец audit".into(),
                        size: 150.,
                    }),
                    id: 99,
                    parent: None,
                    kind: Kind::Auxiliary,
                    points: vec![],
                    curves: vec![],
                };
                marker.points = vec![[0., 0.], [1., 0.], [0., 1.]];
                for (ci, c) in cells.iter().enumerate() {
                    super::cut_forces::begin(&[marker.clone()], cells.len());
                    super::cut_forces::active(ci);
                    let mut f = vec![0.; ndof];
                    c.pressure([0.3, -0.8, -2.4], &mut f);
                    let position = std::array::from_fn(|a| {
                        c.surface
                            .nodes
                            .iter()
                            .map(|&n| scene.xyz[n][a] / 1000.)
                            .sum::<f64>()
                            / c.surface.nodes.len() as f64
                    });
                    c.force(c.local(position, &scene), [0.7, 0.9, -1.3], &mut f);
                    super::cut_forces::audit_rhs(&f);
                    let q = (0..ndof)
                        .map(|i| ((i + 1) as f64 * 0.73).sin() * 2e-4)
                        .collect::<Vec<_>>();
                    let energy = |x: &[f64]| {
                        0.5 * c
                            .dofs
                            .iter()
                            .enumerate()
                            .map(|(i, &a)| {
                                x[a] * c
                                    .dofs
                                    .iter()
                                    .enumerate()
                                    .map(|(j, &b)| c.k[i][j] * x[b])
                                    .sum::<f64>()
                            })
                            .sum::<f64>()
                            - x.iter().zip(&f).map(|(x, f)| x * f).sum::<f64>()
                    };
                    for (n, v) in c.nodal_actions(ci, &q, &rotations) {
                        for axis in 0..3 {
                            let mut plus = q.clone();
                            let mut minus = q.clone();
                            let eps = 1e-7;
                            if axis == 0 {
                                plus[3 * n + 2] -= eps;
                                minus[3 * n + 2] += eps;
                            } else {
                                for &(d, a) in &rotations[n][axis - 1] {
                                    plus[d] += eps * a;
                                    minus[d] -= eps * a;
                                }
                            }
                            let derivative = (energy(&plus) - energy(&minus)) / (2. * eps);
                            assert!((derivative-v[axis]).abs()<1e-6*(1.+v[axis].abs()),"orientation={orientation} topology={topology} cell={ci} node={n} axis={axis} derivative={derivative} action={}",v[axis]);
                        }
                    }
                }
            }
        }
    }
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(crate) struct Mode {
    pub hz: f64,
    #[serde(default)]
    pub effective_mass_t: [f64; 3],
    /// Translations normalized to a maximum nodal magnitude of one.
    pub shape: Vec<[f64; 3]>,
}

#[cfg(test)]
pub(crate) fn frequencies(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[LoadCase],
    weights: &[(u64, f64)],
    g: f64,
    step: f64,
    kind: ElementType,
    settings: &Settings,
) -> Result<(f64, Vec<Mode>), String> {
    let (scene, loads) = super::lira_model(mesh, all, materials, cases, g, step, settings)?;
    let mass = modal_mass(&scene, loads, cases, weights, g)?;
    let rigid = crate::plan_trace::columns::append_contour_rigid_links(
        crate::plan_trace::columns::rigid(all, settings, mesh, &scene),
        mesh,
        &scene,
    )?;
    let springs = super::foundation::build(mesh, all, g, true, false)?;
    let (cells, beams, ndof, rotations) = build_with_rigid(&scene, kind, &rigid, &BTreeMap::new())?;
    let projection = super::rigid::projection(
        ndof,
        &scene.xyz,
        &rotations,
        &rigid,
        &scene.fixed,
        &scene.fixed_rotations,
    )?;
    let active = projection
        .iter()
        .flatten()
        .map(|&(d, _)| d)
        .collect::<BTreeSet<_>>();
    let mut map = vec![usize::MAX; ndof];
    for (i, d) in active.iter().enumerate() {
        map[*d] = i;
    }
    let mut triplets = Vec::new();
    for (dofs, k) in cells
        .iter()
        .map(|c| (&c.dofs, &c.k))
        .chain(beams.iter().map(|b| (&b.dofs, &b.k)))
        .chain(springs.iter().map(|b| (&b.dofs, &b.k)))
    {
        for (i, &a) in dofs.iter().enumerate() {
            for (j, &b) in dofs.iter().enumerate() {
                for &(p, u) in &projection[a] {
                    for &(q, v) in &projection[b] {
                        if map[p] <= map[q] && k[i][j] != 0. {
                            triplets.push(Triplet::new(map[p], map[q], u * k[i][j] * v));
                        }
                    }
                }
            }
        }
    }
    let matrix =
        SparseColMat::<usize, f64>::try_new_from_triplets(active.len(), active.len(), &triplets)
            .map_err(|e| format!("Сборка жёсткости: {e:?}"))?;
    drop(triplets);
    let factor = matrix
        .sp_cholesky(Side::Upper)
        .map_err(|e| format!("Проверьте закрепления и связность: {e:?}"))?;
    modal_spectrum(
        mass,
        &projection,
        &map,
        &vec![1.; ndof],
        active.len(),
        |rhs| factor.solve(rhs.as_ref()),
    )
}
fn modal_mass(
    scene: &Scene,
    loads: Vec<super::super::lira_export::CaseLoads>,
    cases: &[LoadCase],
    weights: &[(u64, f64)],
    g: f64,
) -> Result<Vec<f64>, String> {
    let mut mass = vec![0.; scene.xyz.len()];
    for (case, load) in cases.iter().zip(loads) {
        let weight = weights
            .iter()
            .find(|(id, _)| *id == case.id)
            .map_or(0., |(_, w)| *w)
            / g;
        if weight == 0. {
            continue;
        }
        for (m, load) in mass.iter_mut().zip(load.nodal) {
            *m += weight * load;
        }
        for (surface, q) in scene.surfaces.iter().zip(load.surface) {
            let a = scene.xyz[surface.nodes[0]];
            for edge in surface.nodes[1..].windows(2) {
                let normal = cross(sub(scene.xyz[edge[0]], a), sub(scene.xyz[edge[1]], a));
                let share = q * weight * dot(normal, normal).sqrt() / 6e6;
                for n in [surface.nodes[0], edge[0], edge[1]] {
                    mass[n] += share;
                }
            }
        }
        for (beam, q) in scene.beams.iter().zip(load.beam) {
            let d = sub(scene.xyz[beam.nodes[1]], scene.xyz[beam.nodes[0]]);
            let share = q * weight * dot(d, d).sqrt() / 2000.;
            for n in beam.nodes {
                mass[n] += share;
            }
        }
    }
    if mass.iter().any(|m| !m.is_finite() || *m < -1e-10) {
        return Err("Сочетание даёт отрицательную или некорректную узловую массу".into());
    }
    Ok(mass)
}
fn modal_spectrum(
    mass: Vec<f64>,
    projection: &[Vec<(usize, f64)>],
    map: &[usize],
    scale: &[f64],
    free: usize,
    solve: impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<(f64, Vec<Mode>), String> {
    let total = mass.iter().sum::<f64>();
    let nodes = mass.len();
    let mut mass_rows = Vec::new();
    for (n, m) in mass.iter().copied().enumerate() {
        if m > 0. {
            for row in &projection[3 * n..3 * n + 3] {
                if !row.is_empty() {
                    mass_rows.push((m.sqrt(), row));
                }
            }
        }
    }
    let count = mass_rows.len();
    if count == 0 {
        return Err("В сочетании нет массы на свободных степенях".into());
    }
    // The eigenvalues of B K^-1 Bᵀ are the reciprocals of omega².
    let apply = |q: &Mat<f64>| -> Result<(Mat<f64>, Mat<f64>), String> {
        super::progress::check_cancel()?;
        let mut rhs = Mat::<f64>::zeros(free, q.ncols());
        for (i, (m, row)) in mass_rows.iter().enumerate() {
            for &(d, w) in *row {
                let b = m * w * scale[d];
                for j in 0..q.ncols() {
                    rhs[(map[d], j)] += b * q[(i, j)];
                }
            }
        }
        let x = solve(&rhs);
        super::progress::check_cancel()?;
        let image = Mat::from_fn(count, q.ncols(), |i, j| {
            let (m, row) = mass_rows[i];
            row.iter()
                .map(|&(d, w)| m * w * scale[d] * x[(map[d], j)])
                .sum::<f64>()
        });
        Ok((image, x))
    };
    let (values, reduced_modes) = if count <= 1800 && free.saturating_mul(count) <= 8_000_000 {
        let (flexibility, solution) = apply(&Mat::identity(count, count))?;
        let eigen = flexibility
            .self_adjoint_eigen(Side::Lower)
            .map_err(|e| format!("Расчёт спектра: {e:?}"))?;
        let indices = (0..count)
            .rev()
            .filter(|&i| eigen.S()[i].is_finite() && eigen.S()[i] > 0.)
            .take(10)
            .collect::<Vec<_>>();
        let vectors = Mat::from_fn(count, indices.len(), |i, j| eigen.U()[(i, indices[j])]);
        (
            indices.iter().map(|&i| eigen.S()[i]).collect::<Vec<_>>(),
            &solution * &vectors,
        )
    } else {
        modal_subspace(count, &apply)?
    };
    let frequencies = values
        .into_iter()
        .enumerate()
        .map(|(i, value)| {
            let reduced = reduced_modes.col(i);
            let mut shape = projection[..3 * nodes]
                .chunks_exact(3)
                .map(|rows| {
                    std::array::from_fn(|axis| {
                        rows[axis]
                            .iter()
                            .map(|&(d, w)| w * scale[d] * reduced[map[d]])
                            .sum::<f64>()
                    })
                })
                .collect::<Vec<_>>();
            let max = shape.iter().map(|p| dot(*p, *p).sqrt()).fold(0., f64::max);
            if max > 0. {
                for point in &mut shape {
                    *point = point.map(|v| v / max);
                }
            }
            let mut participation = [0.; 3];
            let mut norm = 0.;
            for (&m, &point) in mass.iter().zip(&shape) {
                let m = m.max(0.);
                norm += m * dot(point, point);
                for axis in 0..3 {
                    participation[axis] += m * point[axis];
                }
            }
            Mode {
                effective_mass_t: if norm > 0. {
                    participation.map(|v| v * v / norm)
                } else {
                    [0.; 3]
                },
                hz: 1. / (std::f64::consts::TAU * value.sqrt()),
                shape,
            }
        })
        .collect::<Vec<_>>();
    if frequencies.is_empty() {
        return Err("Не найдены конечные собственные частоты".into());
    }
    Ok((total, frequencies))
}

fn modal_subspace(
    count: usize,
    apply: impl Fn(&Mat<f64>) -> Result<(Mat<f64>, Mat<f64>), String>,
) -> Result<(Vec<f64>, Mat<f64>), String> {
    let width = count.min(24);
    let mut seed = 17_u64;
    let mut q = Mat::from_fn(count, width, |_, _| {
        seed ^= seed << 13;
        seed ^= seed >> 7;
        seed ^= seed << 17;
        (seed >> 11) as f64 / (1_u64 << 53) as f64 - 0.5
    })
    .qr()
    .compute_thin_Q();
    for _ in 0..80 {
        let (image, solution) = apply(&q)?;
        let small = q.transpose() * &image;
        let eigen = small
            .self_adjoint_eigen(Side::Lower)
            .map_err(|e| format!("Модальный подпространственный расчёт: {e:?}"))?;
        let vectors = Mat::from_fn(width, width.min(10), |i, j| eigen.U()[(i, width - 1 - j)]);
        let modes = &q * &vectors;
        let applied = &image * &vectors;
        let mut values = Vec::new();
        for j in 0..vectors.ncols() {
            let v = eigen.S()[width - 1 - j];
            if !v.is_finite() || v <= 0. {
                break;
            }
            let norm = (0..count)
                .map(|i| (applied[(i, j)] - v * modes[(i, j)]).powi(2))
                .sum::<f64>()
                .sqrt();
            if norm > 1e-5 * v {
                break;
            }
            values.push(v);
        }
        if values.len() == vectors.ncols() {
            return Ok((values, &solution * &vectors));
        }
        q = image.qr().compute_thin_Q();
    }
    Err("Не сошлись десять нижних форм за 80 итераций".into())
}

#[test]
fn modal_subspace_resolves_known_diagonal_spectrum() {
    let (values, _) = modal_subspace(30, |q| {
        let applied = Mat::from_fn(30, q.ncols(), |i, j| (30 - i) as f64 * q[(i, j)]);
        Ok((applied.clone(), applied))
    })
    .unwrap();
    assert!(values
        .iter()
        .enumerate()
        .all(|(i, v)| (v - (30 - i) as f64).abs() < 1e-6));
}

#[cfg(test)]
#[test]
fn modal_spectrum_reuses_scaled_factor_and_restores_fixed_components() {
    // Kx=8 and Ky=18, mass=2. Scaling produces an identity factor.
    let projection = vec![vec![(0, 1.)], vec![(1, 1.)], vec![]];
    let (mass, modes) = modal_spectrum(
        vec![2.],
        &projection,
        &[0, 1, usize::MAX],
        &[1. / 8_f64.sqrt(), 1. / 18_f64.sqrt(), 1.],
        2,
        |rhs| rhs.clone(),
    )
    .unwrap();
    assert_eq!(mass, 2.);
    assert_eq!(modes.len(), 2);
    for (mode, omega) in modes.iter().zip([2., 3.]) {
        assert!((mode.hz * std::f64::consts::TAU - omega).abs() < 1e-12);
        assert_eq!(mode.shape[0][2], 0.);
    }
    assert!((modes[0].shape[0][0].abs() - 1.).abs() < 1e-12);
    assert!((modes[1].shape[0][1].abs() - 1.).abs() < 1e-12);
}
#[cfg(test)]
mod surface_placement_tests {
    use super::*;
    #[test]
    fn plate_z_arm_couples_in_plane_motion_with_rotation() {
        let scene = |z| Scene {
            xyz: vec![[0., 0., 0.], [1000., 0., 0.], [0., 1000., 0.]],
            plan: vec![0, 1, 2],
            surfaces: vec![Surface {
                rigid_z_mm: z,
                nodes: vec![0, 1, 2],
                material: Material::default(),
                display: vec![0],
                support: None,
            }],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let (plain, _, _, _) = build(&scene(0.), ElementType::Dkt).unwrap();
        let (eccentric, _, _, _) = build(&scene(300.), ElementType::Dkt).unwrap();
        assert_eq!(plain[0].mem[0], vec![(0, 1.)]);
        assert!(eccentric[0].mem[0]
            .iter()
            .any(|&(dof, value)| dof >= 9 && (value.abs() - 0.3).abs() < 1e-10));
        assert_eq!(eccentric[0].bend[0], plain[0].bend[0]);
    }
}
