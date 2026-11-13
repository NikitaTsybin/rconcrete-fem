mod building_scene;
pub(super) mod cut_forces;
mod edge_actions;
mod face_trace;
mod line_forces;
mod rigid;
// Native plate analysis. Geometry input is mm, kernel units are m and kN.
pub(super) mod beam;
pub mod element;
mod foundation;
mod mindlin_cr;
pub(super) mod mitc3;
pub(super) mod mitc4;
mod mixed;
pub(super) mod shell;
#[derive(Clone, Copy, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum ElementType {
    Dkt,
    MindlinCr,
}
impl ElementType {
    pub const ALL: [Self; 2] = [Self::Dkt, Self::MindlinCr];
    pub fn label(self) -> &'static str {
        match self {
            Self::Dkt => "Стандартные · DKT / MITC3–4",
            Self::MindlinCr => "Без shear locking · Mindlin P2/CR",
        }
    }
}
use super::{
    geometry::Point,
    loads::ForceUnits,
    meshing::Mesh,
    model::{Contour, Kind},
};
mod buckling_spectrum;
use element::{Material, Triangle};
use faer::{
    linalg::solvers::Solve,
    sparse::{SparseColMat, Triplet},
    Mat, Side,
};
use mini_fem::geo::{Area, BooleanOps, Contains, LineString, Polygon, Relate};
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct CaseResult {
    pub punching: BTreeMap<u64, cut_forces::CutResult>,
    pub id: u64,
    pub shell: Option<shell::Case>,
    pub w_mm: Vec<f64>,
    pub rotations_rad: Vec<[f64; 3]>,
    /// Global XY moments: positive Mx/My stretches the lower slab face.
    /// Physical element-centre values mapped to display cells; a Q4 maps to both display triangles.
    pub moments: Vec<[f64; 3]>,
    pub shears: Vec<[f64; 2]>,
    pub reaction_kn: Vec<f64>,
    /// Applied AREA-load pressure per element, positive downward, kN/m².
    pub loads_kn_m2: Vec<f64>,
    #[serde(default, deserialize_with = "read_foundation_pressures")]
    pub foundation_pressure_kn_m2: Vec<f64>,
    pub applied_kn: f64,
    pub balance_kn: f64,
    pub balance_m_knm: [f64; 2],
    pub relative_residual: f64,
}
fn read_foundation_pressures<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Vec<f64>, D::Error> {
    use serde::Deserialize;
    Ok(Vec::<Option<f64>>::deserialize(d)?
        .into_iter()
        .map(|v| v.unwrap_or(f64::NAN))
        .collect())
}
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct NonlinearRequest {
    pub weights: Vec<(u64, f64)>,
    pub caption: String,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct NonlinearResult {
    pub caption: String,
    pub weights: Vec<(u64, f64)>,
    pub steps: usize,
    pub iterations: usize,
    pub offset: Box<CaseResult>,
}
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct BucklingRequest {
    pub weights: Vec<(u64, f64)>,
    pub caption: String,
    pub forms: usize,
    pub max_factor: f64,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct BucklingMode {
    pub factor: f64,
    pub shape: Vec<[f64; 3]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub sensitivity: Option<BucklingSensitivity>,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct BucklingSensitivity {
    /// Signed contribution in percent, in physical surface order.
    pub surfaces: Vec<f32>,
    /// Signed contribution in percent, in physical beam order.
    pub beams: Vec<f32>,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct BucklingResult {
    pub caption: String,
    pub modes: Vec<BucklingMode>,
    pub max_factor: f64,
    pub requested: usize,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Results {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub buckling: Option<Result<BucklingResult, String>>,
    #[serde(default)]
    pub nonlinear: Option<NonlinearResult>,
    #[serde(default)]
    pub lengths: Option<Result<Vec<shell::effective_length::Length>, String>>,
    #[serde(default)]
    pub modal: Option<Result<(f64, Vec<shell::Mode>), String>>,
    pub cases: Vec<CaseResult>,
    pub free_dofs: usize,
}
pub const DISPLACEMENT_NORM_FIELD: usize = 35;
pub const FIELD_COUNT: usize = DISPLACEMENT_NORM_FIELD + 1;
pub const FIELD_NAMES: [&str; FIELD_COUNT] = [
    "W",
    "Mx",
    "My",
    "Mxy",
    "Qx",
    "Qy",
    "Узловые реакции Rz",
    "Мозаика распределенных нагрузок",
    "Мозаика линейных нагрузок",
    "Мозаика сосредоточенных нагрузок",
    "U",
    "V",
    "Nx",
    "Ny",
    "Nxy",
    "Расстояние до опор",
    "Грузовые площади опор",
    "Балка · N",
    "Балка · Mx",
    "Балка · My",
    "Балка · Mz",
    "Балка · Qy",
    "Балка · Qz",
    "Ux",
    "Uy",
    "Uz",
    "As1 · нижняя X",
    "As2 · верхняя X",
    "As3 · нижняя Y",
    "As4 · верхняя Y",
    "Раскрытие трещин · непродолжительное",
    "Раскрытие трещин · длительное",
    "Образование трещин · 1/λcrc",
    "Коэффициент постели C1",
    "Реакция основания",
    "Суммарное перемещение",
];
pub fn displacement_field(field: usize) -> bool {
    matches!(field, 0 | 10 | 11 | 23..=25 | DISPLACEMENT_NORM_FIELD)
}

pub fn displacement_norm(xyz: [f64; 3], axes: u8) -> f64 {
    xyz.into_iter()
        .enumerate()
        .filter(|(axis, _)| axes & (1 << axis) != 0)
        .map(|(_, value)| value * value)
        .sum::<f64>()
        .sqrt()
}
impl CaseResult {
    pub fn node_value(&self, field: usize, index: usize) -> f64 {
        match field {
            0 => self.w_mm[index],
            23..=25 => self.rotations_rad[index][field - 23],
            10 | 11 => self
                .shell
                .as_ref()
                .map_or(0., |s| s.translations_mm[index][field - 10]),
            _ => unreachable!("not nodal"),
        }
    }

    pub fn element_value(&self, field: usize, index: usize) -> f64 {
        match field {
            1..=3 => self.moments[index][field - 1],
            4..=5 => self.shears[index][field - 4],
            7 => self.loads_kn_m2[index],
            12..=14 => self
                .shell
                .as_ref()
                .map_or(0., |s| s.membrane[index][field - 12]),
            _ => unreachable!("w is nodal"),
        }
    }
}
fn polygon(points: &[Point]) -> Polygon<f64> {
    Polygon::new(
        LineString::from(points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
        vec![],
    )
}
pub(super) struct Cell {
    triangle: Triangle,
    nodes: [usize; 3],
    pub(super) material: Material,
    bbox: [f64; 4],
}
struct Index {
    step: f64,
    bins: BTreeMap<(i64, i64), Vec<usize>>,
    large: Vec<usize>,
}
impl Index {
    fn new(cells: &[Cell], step: f64) -> Self {
        let mut s = Self {
            step,
            bins: BTreeMap::new(),
            large: vec![],
        };
        for (i, c) in cells.iter().enumerate() {
            let [a, b, x, y] = s.range(c.bbox);
            if (x - a + 1) as f64 * (y - b + 1) as f64 > 4096. {
                s.large.push(i);
                continue;
            }
            for u in a..=x {
                for v in b..=y {
                    s.bins.entry((u, v)).or_default().push(i);
                }
            }
        }
        s
    }
    fn range(&self, b: [f64; 4]) -> [i64; 4] {
        b.map(|x| (x / self.step).floor() as i64)
    }
    fn query(&self, b: [f64; 4], count: usize) -> Vec<usize> {
        let [a, d, x, y] = self.range(b);
        if (x - a + 1) as f64 * (y - d + 1) as f64 > 100_000. {
            return (0..count).collect();
        }
        let mut ids = self.large.iter().copied().collect::<BTreeSet<_>>();
        for u in a..=x {
            for v in d..=y {
                if let Some(bin) = self.bins.get(&(u, v)) {
                    ids.extend(bin);
                }
            }
        }
        ids.into_iter().collect()
    }
}
fn bbox(p: &[Point]) -> [f64; 4] {
    p.iter().fold(
        [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ],
        |mut b, p| {
            b[0] = b[0].min(p[0]);
            b[1] = b[1].min(p[1]);
            b[2] = b[2].max(p[0]);
            b[3] = b[3].max(p[1]);
            b
        },
    )
}
// The three rigid plate modes are w=a+bx+cy. Every connected component
// needs fixed w at three non-collinear positions; no artificial stabilisation.
fn check_supports(mesh: &Mesh, fixed: &BTreeSet<usize>) -> Result<(), String> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut parent = (0..mesh.nodes.len()).collect::<Vec<_>>();
    for e in &mesh.elements {
        for &n in &e.nodes[1..] {
            let a = root(&mut parent, e.nodes[0]);
            let b = root(&mut parent, n);
            parent[b] = a;
        }
    }
    let mut groups = BTreeMap::<usize, Vec<Point>>::new();
    let mut component_points = BTreeMap::<usize, Vec<Point>>::new();
    for n in 0..mesh.nodes.len() {
        let r = root(&mut parent, n);
        let group = groups.entry(r).or_default();
        component_points
            .entry(r)
            .or_default()
            .push(mesh.nodes[n].xy);
        if fixed.contains(&n) {
            group.push(mesh.nodes[n].xy);
        }
    }
    for (component, points) in &groups {
        let stable = points.first().is_some_and(|a| {
            let b = points
                .iter()
                .max_by(|p, q| {
                    ((p[0] - a[0]).hypot(p[1] - a[1]))
                        .total_cmp(&((q[0] - a[0]).hypot(q[1] - a[1])))
                })
                .unwrap();
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            length > 1e-6
                && points.iter().any(|p| {
                    ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])).abs() / length
                        > 1e-4
                })
        });
        if !stable {
            let bounds = bbox(&component_points[component]);
            let reason = if points.is_empty() {
                "нет узлов с закреплением w"
            } else {
                "закрепления w лежат на одной прямой или совпадают; возможен свободный наклон плиты"
            };
            return Err(format!(
                "Недостаточно опор: {reason}. Область X={:.1}…{:.1}, Y={:.1}…{:.1} мм; закреплённых узлов: {}. Нужна опора вне этой прямой либо расчётная связь с соседней плитой.",
                bounds[0], bounds[2], bounds[1], bounds[3], points.len()
            ));
        }
    }
    Ok(())
}
fn point_force(
    cells: &[Cell],
    index: &Index,
    p: Point,
    force: f64,
    rhs: &mut [f64],
) -> Result<usize, String> {
    point_force_optional(cells, index, p, force, rhs).ok_or_else(|| {
        format!(
            "Нагрузка вне плиты / в отверстии: X={:.1}, Y={:.1} мм",
            p[0] * 1000.,
            p[1] * 1000.
        )
    })
}
fn point_force_optional(
    cells: &[Cell],
    index: &Index,
    p: Point,
    force: f64,
    rhs: &mut [f64],
) -> Option<usize> {
    for i in index.query(
        [p[0] - 1e-9, p[1] - 1e-9, p[0] + 1e-9, p[1] + 1e-9],
        cells.len(),
    ) {
        cut_forces::active(i);
        let c = &cells[i];
        let l = c.triangle.barycentric(p);
        if l.iter().all(|v| *v >= -1e-8 && *v <= 1. + 1e-8) {
            for j in 0..3 {
                rhs[3 * c.nodes[j]] += force * l[j];
                cut_forces::load(3 * c.nodes[j], force * l[j]);
            }
            return Some(i);
        }
    }
    None
}

/// Within one load case, overlapping AREA zones select the greatest signed q.
/// Return only the increment to the already assembled consistent element load.
fn area_pressure_delta(previous: &mut Option<f64>, candidate: f64) -> f64 {
    let old = previous.unwrap_or(0.);
    let selected = previous.map_or(candidate, |value| value.max(candidate));
    *previous = Some(selected);
    selected - old
}
fn area_pressure_delta_for(previous: &mut Option<f64>, candidate: f64, additive: bool) -> f64 {
    if additive {
        candidate
    } else {
        area_pressure_delta(previous, candidate)
    }
}
fn load_vector(
    cells: &[Cell],
    index: &Index,
    all: &[Contour],
    case: u64,
    g: f64,
    step: f64,
    n: usize,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    let mut f = vec![0.; n];
    let mut totals = vec![0.; cells.len()];
    let mut area_pressure = vec![None; cells.len()];
    for c in all
        .iter()
        .filter(|c| c.load.as_ref().is_some_and(|l| l.case_id == case))
    {
        let value = c
            .load
            .as_ref()
            .unwrap()
            .magnitude
            .in_units(ForceUnits::KiloNewton, g);
        if !value.is_finite() {
            return Err(format!("Нагрузка {} не конечна", c.id));
        }
        if value == 0. && c.kind != Kind::LoadArea {
            continue;
        }
        let points = c
            .points
            .iter()
            .map(|p| [p[0] / 1000., p[1] / 1000.])
            .collect::<Vec<_>>();
        match c.kind {
            Kind::LoadPoint => {
                let p = *points.first().ok_or("Пустая сосредоточенная нагрузка")?;
                point_force(cells, index, p, value, &mut f)?;
            }
            Kind::LoadArea => {
                let area = effective_load_area(all, c);
                for i in index.query(bbox(&points), cells.len()) {
                    let cell = &cells[i];
                    if !area
                        .0
                        .iter()
                        .any(|part| area_load_hits(&cell.triangle, part))
                    {
                        continue;
                    }
                    let force = area_pressure_delta_for(
                        &mut area_pressure[i],
                        value,
                        c.load.as_ref().unwrap().additive,
                    ) * cell.triangle.area;
                    totals[i] += force;
                    cut_forces::active(i);
                    for node in cell.nodes {
                        f[3 * node] += force / 3.;
                        cut_forces::load(3 * node, force / 3.);
                    }
                }
            }
            Kind::LoadLine => {
                let closed = c.closed();
                let segments =
                    points
                        .windows(2)
                        .map(|p| [p[0], p[1]])
                        .chain(if closed && points.len() > 1 {
                            Some([*points.last().unwrap(), points[0]])
                        } else {
                            None
                        });
                for [a, b] in segments {
                    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                    let count = (length / step).ceil().max(1.) as usize;
                    if count > 1_000_000 {
                        return Err(format!("Линейная нагрузка {}: длина сегмента {:.3} м, шаг {:.6} м, требуется {count} точек (лимит 1000000). Проверьте координаты и единицы",c.id,length,step));
                    }
                    for k in 0..count {
                        if k % 4096 == 0 {
                            crate::plan_trace::fem::progress::check_cancel()?;
                        }
                        let t = (k as f64 + 0.5) / count as f64;
                        let p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                        let force = value * length / count as f64;
                        let _ = point_force_optional(cells, index, p, force, &mut f);
                    }
                }
            }
            _ => {}
        }
    }
    Ok((
        f,
        totals
            .iter()
            .zip(cells)
            .map(|(p, c)| p / c.triangle.area)
            .collect(),
    ))
}
fn effective_load_area(all: &[Contour], c: &Contour) -> mini_fem::geo::MultiPolygon<f64> {
    use mini_fem::geo::MapCoords;
    super::model::load_area_material(all, c).map_coords(|p| mini_fem::geo::Coord {
        x: p.x / 1000.,
        y: p.y / 1000.,
    })
}
// Kernel coordinates are metres. Ignore boundary slivers within the same
// numerical length scale as the mesh's sub-micrometre node welding.
// This is only activation, NEVER an overlap fraction for the load magnitude.
fn polygon_area_load_hits(cell: &Polygon<f64>, area: &Polygon<f64>) -> bool {
    use mini_fem::geo::{coordinate_position::CoordPos, dimensions::Dimensions, Area};
    const CONTACT_TOL_M: f64 = 1e-9; // 0.000001 mm, numerical coincidence only
    let ring = &cell.exterior().0;
    if ring.len() < 4 {
        return false;
    }
    let n = (ring.len() - 1) as f64;
    let center = mini_fem::geo::Point::new(
        ring[..ring.len() - 1].iter().map(|p| p.x / n).sum(),
        ring[..ring.len() - 1].iter().map(|p| p.y / n).sum(),
    );
    // Most fully loaded cells are far from the boundary: avoid boolean clipping
    // for their common case. Boundary cells use an explicit positive-area test.
    if area.contains(&center)
        && std::iter::once(area.exterior())
            .chain(area.interiors())
            .all(|r| {
                r.lines().all(|line| {
                    let d = [line.end.x - line.start.x, line.end.y - line.start.y];
                    let length2 = d[0] * d[0] + d[1] * d[1];
                    let t = if length2 > 0. {
                        ((center.x() - line.start.x) * d[0] + (center.y() - line.start.y) * d[1])
                            / length2
                    } else {
                        0.
                    }
                    .clamp(0., 1.);
                    (center.x() - line.start.x - t * d[0])
                        .hypot(center.y() - line.start.y - t * d[1])
                        > CONTACT_TOL_M
                })
            })
    {
        return true;
    }
    if cell.relate(area).get(CoordPos::Inside, CoordPos::Inside) != Dimensions::TwoDimensional {
        return false;
    }
    let perimeter = cell
        .exterior()
        .lines()
        .map(|l| (l.end.x - l.start.x).hypot(l.end.y - l.start.y))
        .sum::<f64>();
    let numerical_area = (perimeter * CONTACT_TOL_M).max(1e-14);
    cell.intersection(area).unsigned_area() > numerical_area
}
fn area_load_hits(triangle: &Triangle, area: &Polygon<f64>) -> bool {
    polygon_area_load_hits(&polygon(&triangle.xy), area)
}
/// Sampling follows the requested meshing sizes, not the smallest accidental
/// sliver area anywhere in the model. Units: input mm, output m.
fn line_load_step_m(all: &[Contour], size_mm: f64) -> f64 {
    let mut step = size_mm.max(100.);
    for c in all {
        if matches!(c.kind, Kind::Slab | Kind::Support)
            && c.mesh_size_mm.is_finite()
            && c.mesh_size_mm > 0.
        {
            step = step.min(c.mesh_size_mm.max(100.));
        }
        if c.kind == Kind::MeshRegion {
            if let Some(r) = &c.refinement {
                if r.size.is_finite() && r.size > 0. {
                    step = step.min(r.size.max(100.));
                }
            }
        }
    }
    step / 1000.
}

pub(super) fn prepare_cells(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    mesh_step_mm: f64,
    _gravity: f64,
) -> Result<Vec<Cell>, String> {
    let building = mesh.building.as_ref();
    if let Some(data) = building {
        data.validate(mesh)?;
    }
    struct SlabArea {
        id: u64,
        area: mini_fem::geo::MultiPolygon<f64>,
        material: Material,
        floor: u64,
        elevation: f64,
        priority_area: f64,
    }
    let mut holes = std::collections::HashMap::<u64, Vec<&Contour>>::new();
    for hole in all.iter().filter(|c| c.active_hole()) {
        if let Some(parent) = hole.parent {
            holes.entry(parent).or_default().push(hole);
        }
    }
    let mut slab_by_id = std::collections::HashMap::new();
    let mut slabs = Vec::new();
    for c in all.iter().filter(|c| c.kind == Kind::Slab) {
        let mut area = mini_fem::geo::MultiPolygon(vec![polygon(&c.mesh_boundary(mesh_step_mm)?)]);
        for h in holes.get(&c.id).into_iter().flatten() {
            area = area.difference(&mini_fem::geo::MultiPolygon(vec![polygon(
                &h.mesh_boundary(mesh_step_mm)?,
            )]));
        }
        let material = *materials
            .get(&c.id)
            .ok_or_else(|| format!("Не задан материал плиты {}", c.id))?;
        material.validate()?;
        let (floor, elevation) =
            building.map_or((0, 0.), |b| (b.layout.owner(c.id), b.layout.extent(c)[0]));
        let priority_area = area.unsigned_area();
        if building.is_some() {
            slab_by_id.entry(c.id).or_insert(slabs.len());
        }
        slabs.push(SlabArea {
            id: c.id,
            area,
            material,
            floor,
            elevation,
            priority_area,
        });
    }
    let mut cells = Vec::with_capacity(mesh.elements.len());
    for (element_index, e) in mesh.elements.iter().enumerate() {
        let xy = e.nodes.map(|i| mesh.nodes[i].xy.map(|v| v / 1000.));
        let triangle = Triangle::new(xy).map_err(|error| {
            let points = e.nodes.map(|i| {
                let n = &mesh.nodes[i];
                format!("{} (X={:.9}, Y={:.9})", n.id, n.xy[0], n.xy[1])
            });
            let a = mesh.nodes[e.nodes[0]].xy;
            let b = mesh.nodes[e.nodes[1]].xy;
            let c = mesh.nodes[e.nodes[2]].xy;
            let area = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5;
            format!(
                "{error}. КЭ {}. Площадь {area:.6e} мм². Узлы: {}",
                e.id,
                points.join("; ")
            )
        })?;
        let center = mini_fem::geo::Point::new(
            e.nodes.iter().map(|&n| mesh.nodes[n].xy[0]).sum::<f64>() / 3.,
            e.nodes.iter().map(|&n| mesh.nodes[n].xy[1]).sum::<f64>() / 3.,
        );
        let level = building.map(|b| (b.element_floors[element_index], b.z[e.nodes[0]]));
        let eligible = |slab: &SlabArea| {
            level.is_none_or(|(floor, z)| slab.floor == floor && (slab.elevation - z).abs() < 0.05)
                && slab.area.contains(&center)
        };
        // Local patches replace the base plate, including their full thickness
        // and weight. Prefer the triangulation owner to keep FE IDs, material,
        // rendering and reinforcement assignment consistent.
        let assigned = building.map(|b| b.element_slabs[element_index]);
        let material = assigned
            .and_then(|id| slab_by_id.get(&id))
            .map(|&index| &slabs[index])
            .filter(|slab| eligible(slab))
            .or_else(|| {
                let mut best: Option<&SlabArea> = None;
                for slab in slabs.iter().filter(|slab| eligible(slab)) {
                    if Some(slab.id) == assigned {
                        return Some(slab);
                    }
                    if best.is_none_or(|previous| {
                        slab.priority_area
                            .total_cmp(&previous.priority_area)
                            .then_with(|| previous.id.cmp(&slab.id))
                            .is_lt()
                    }) {
                        best = Some(slab);
                    }
                }
                best
            })
            .ok_or_else(|| {
                format!(
                    "КЭ {} вне плит: центр X={:.9}, Y={:.9} мм. Перестройте сетку",
                    e.id,
                    center.x(),
                    center.y()
                )
            })?
            .material;
        cells.push(Cell {
            bbox: bbox(&xy),
            triangle,
            nodes: e.nodes,
            material,
        });
    }
    Ok(cells)
}
// Existing callers/tests retain the original DKT entry point.
#[allow(dead_code)]
pub fn solve(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[u64],
    gravity: f64,
    mesh_step_mm: f64,
) -> Result<Results, String> {
    solve_with_type(
        mesh,
        all,
        materials,
        cases,
        gravity,
        mesh_step_mm,
        ElementType::Dkt,
    )
}
pub fn solve_with_type(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[u64],
    gravity: f64,
    mesh_step_mm: f64,
    element_type: ElementType,
) -> Result<Results, String> {
    solve_internal(
        mesh,
        all,
        materials,
        cases,
        &BTreeSet::new(),
        gravity,
        mesh_step_mm,
        element_type,
    )
}
/// Application entry point: automatic gravity acts only in SelfWeight cases.
pub fn solve_load_cases(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    gravity: f64,
    mesh_step_mm: f64,
    element_type: ElementType,
) -> Result<Results, String> {
    let ids = cases.iter().map(|c| c.id).collect::<Vec<_>>();
    let own = cases
        .iter()
        .filter(|c| c.kind == super::loads::LoadType::SelfWeight)
        .map(|c| c.id)
        .collect();
    solve_internal(
        mesh,
        all,
        materials,
        &ids,
        &own,
        gravity,
        mesh_step_mm,
        element_type,
    )
}
pub fn solve_with_support_height(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    kind: ElementType,
    settings: &shell::Settings,
) -> Result<Results, String> {
    solve_with_modal(
        mesh, all, materials, cases, g, step, kind, settings, None, true, None,
    )
}
pub(super) fn solve_with_modal(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    kind: ElementType,
    settings: &shell::Settings,
    modal_weights: Option<&[(u64, f64)]>,
    include_effective_lengths: bool,
    nonlinear: Option<&NonlinearRequest>,
) -> Result<Results, String> {
    solve_with_buckling(
        mesh,
        all,
        materials,
        cases,
        g,
        step,
        kind,
        settings,
        modal_weights,
        include_effective_lengths,
        nonlinear,
        None,
    )
}

pub(super) fn solve_with_buckling(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    kind: ElementType,
    settings: &shell::Settings,
    modal_weights: Option<&[(u64, f64)]>,
    include_effective_lengths: bool,
    nonlinear: Option<&NonlinearRequest>,
    buckling: Option<&BucklingRequest>,
) -> Result<Results, String> {
    if buckling.is_some() && kind == ElementType::MindlinCr {
        return Err("Для расчёта устойчивости выберите стандартные КЭ DKT / MITC3–4".into());
    }
    if mesh.building.is_none()
        && !settings.enabled
        && settings.beams.is_empty()
        && settings.columns.is_empty()
        && nonlinear.is_none()
        && buckling.is_none()
        && !all
            .iter()
            .any(|c| c.kind == Kind::Slab && c.placement.slab_rigid_z_mm != 0.)
    {
        return solve_load_cases(mesh, all, materials, cases, g, step, kind);
    }
    if !g.is_finite() || g <= 0. || !step.is_finite() || step < 100. {
        return Err("Проверьте g и размер КЭ".into());
    }
    let cells = prepare_cells(mesh, all, materials, step, g)?;
    shell::solve(
        mesh,
        &cells,
        all,
        cases,
        g,
        step,
        kind,
        settings,
        modal_weights.map(|weights| (materials, weights)),
        include_effective_lengths,
        nonlinear,
        buckling,
    )
}
fn solve_internal(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[u64],
    self_weight: &BTreeSet<u64>,
    gravity: f64,
    mesh_step_mm: f64,
    element_type: ElementType,
) -> Result<Results, String> {
    if !gravity.is_finite() || gravity <= 0. || !mesh_step_mm.is_finite() || mesh_step_mm < 100. {
        return Err("Проверьте g и размер КЭ".into());
    }
    if mesh.elements.is_empty() {
        return Err("МКЭ-сетка пуста".into());
    }
    if all.iter().any(|c| {
        c.load
            .as_ref()
            .is_some_and(|l| l.moment || l.direction != super::loads::Direction::Z)
    }) {
        return Err(
            "Для горизонтальных сил и сосредоточенных моментов нужен пространственный расчёт"
                .into(),
        );
    }
    let cells = prepare_cells(mesh, all, materials, mesh_step_mm, gravity)?;
    if !mesh.quads.is_empty() {
        if element_type == ElementType::MindlinCr {
            return Err("P2/CR с промежуточными узлами пока не совместим с MITC4. Отключите четырёхугольники или выберите DKT / MITC4".into());
        }
        return mixed::solve(mesh, &cells, all, cases, self_weight, gravity, mesh_step_mm);
    }
    if element_type == ElementType::MindlinCr {
        return mindlin_cr::solve(mesh, &cells, all, cases, self_weight, gravity, mesh_step_mm);
    }
    let ndof = mesh.nodes.len() * 3;
    let fixed = mesh
        .support_boundary
        .values()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>();
    let springs = foundation::build(mesh, all, gravity, false, false)?;
    if springs.is_empty() {
        check_supports(mesh, &fixed)?;
    }
    let mut map = vec![usize::MAX; ndof];
    let mut free = 0;
    for i in 0..ndof {
        if i % 3 != 0 || !fixed.contains(&(i / 3)) {
            map[i] = free;
            free += 1;
        }
    }
    let mut diagonal = vec![0.; free];
    for c in &cells {
        let k = c.triangle.stiffness(c.material);
        for i in 0..9 {
            let d = map[3 * c.nodes[i / 3] + i % 3];
            if d != usize::MAX {
                diagonal[d] += k[i][i];
            }
        }
    }
    foundation::flat_diagonal(&springs, &map, &mut diagonal);
    if diagonal.iter().any(|d| !d.is_finite() || *d <= 0.) {
        return Err("Некорректная жёсткость / свободные узлы".into());
    }
    let scale = diagonal.iter().map(|v| 1. / v.sqrt()).collect::<Vec<_>>();
    crate::plan_trace::fem::progress::stage("Сборка матрицы жёсткости элементов");
    crate::plan_trace::fem::progress::check_cancel()?;
    let mut triplets = Vec::with_capacity(cells.len() * 45);
    for c in &cells {
        let k = c.triangle.stiffness(c.material);
        for i in 0..9 {
            let a = map[3 * c.nodes[i / 3] + i % 3];
            if a == usize::MAX {
                continue;
            }
            for j in 0..9 {
                let b = map[3 * c.nodes[j / 3] + j % 3];
                if b != usize::MAX && a <= b {
                    triplets.push(Triplet::new(a, b, k[i][j] * scale[a] * scale[b]));
                }
            }
        }
    }
    crate::plan_trace::fem::progress::stage("Сборка разреженной матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    foundation::flat_triplets(&springs, &map, &scale, &mut triplets);
    let matrix = SparseColMat::<usize, f64>::try_new_from_triplets(free, free, &triplets)
        .map_err(|e| format!("Сборка разреженной матрицы: {e:?}"))?;
    drop(triplets);
    crate::plan_trace::fem::progress::stage("Разложение матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    let factor = matrix.sp_cholesky(Side::Upper).map_err(|e| {
        format!("Матрица не положительно определена. Проверьте закрепления и связность: {e:?}")
    })?;
    let step = line_load_step_m(all, mesh_step_mm);
    let index = Index::new(&cells, mesh_step_mm / 1000.);
    let results = parallel_cases::map(
        cases,
        free.max(mesh.nodes.len() * 3).max(cells.len() * 12),
        |&id| {
            crate::plan_trace::fem::progress::stage(&format!(
                "Сборка вектора нагрузок: загружение {id}"
            ));
            crate::plan_trace::fem::progress::check_cancel()?;
            cut_forces::begin(all, cells.len());
            let (mut f, mut loads_kn_m2) =
                load_vector(&cells, &index, all, id, gravity, step, ndof)?;
            if self_weight.contains(&id) {
                for (i, c) in cells.iter().enumerate() {
                    let q = c.material.self_weight_kn_m2(gravity);
                    loads_kn_m2[i] += q;
                    cut_forces::active(i);
                    for n in c.nodes {
                        f[3 * n] += q * c.triangle.area / 3.;
                        cut_forces::load(3 * n, q * c.triangle.area / 3.);
                    }
                }
            }
            #[cfg(test)]
            cut_forces::audit_rhs(&f);
            let mut rhs = Mat::<f64>::zeros(free, 1);
            for i in 0..ndof {
                if map[i] != usize::MAX {
                    rhs[(map[i], 0)] = f[i] * scale[map[i]];
                }
            }
            crate::plan_trace::fem::progress::stage("Вычисление неизвестных");
            crate::plan_trace::fem::progress::check_cancel()?;
            let x = factor.solve(rhs.as_ref());
            crate::plan_trace::fem::progress::stage("Восстановление перемещений, усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut q = vec![0.; ndof];
            for i in 0..ndof {
                if map[i] != usize::MAX {
                    q[i] = x[(map[i], 0)] * scale[map[i]];
                }
            }
            if q.iter().any(|v| !v.is_finite()) {
                return Err("Решатель вернул нечисловой результат".into());
            }
            crate::plan_trace::fem::progress::stage("Восстановление усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut residual = f.iter().map(|v| -v).collect::<Vec<_>>();
            let mut cuts = cut_forces::Collector::new(all);
            let mut moments = vec![];
            let mut shears = vec![];
            for (ci, c) in cells.iter().enumerate() {
                let dofs = std::array::from_fn::<_, 9, _>(|i| q[3 * c.nodes[i / 3] + i % 3]);
                let k = c.triangle.stiffness(c.material);
                for i in 0..9 {
                    for j in 0..9 {
                        residual[3 * c.nodes[i / 3] + i % 3] += k[i][j] * dofs[j];
                    }
                }
                if cuts.enabled() {
                    let forces: Vec<_> = (0..9)
                        .map(|i| {
                            cut_forces::net(
                                ci,
                                3 * c.nodes[i / 3] + i % 3,
                                (0..9).map(|j| k[i][j] * dofs[j]).sum(),
                            )
                        })
                        .collect();
                    let poly: Vec<_> = c.nodes.iter().map(|&n| mesh.nodes[n].xy).collect();
                    cuts.add(
                        &poly,
                        c.nodes.iter().enumerate().map(|(i, &n)| {
                            (
                                mesh.nodes[n].xy,
                                cut_forces::plate_node_action([
                                    forces[3 * i],
                                    forces[3 * i + 1],
                                    forces[3 * i + 2],
                                ]),
                            )
                        }),
                    );
                }
                if cuts.enabled() {
                    cuts.direct_fields(
                        |p| {
                            let p = p.map(|v| v / 1000.);
                            Some((
                                c.triangle
                                    .moments_at(c.material, dofs, c.triangle.barycentric(p)),
                                c.triangle.shears(c.material, dofs),
                            ))
                        },
                        true,
                    );
                    cuts.kirchhoff();
                    cuts.field(
                        c.triangle.moments(c.material, dofs),
                        c.triangle.shears(c.material, dofs),
                    );
                }
                moments.push(c.triangle.moments(c.material, dofs));
                shears.push(c.triangle.shears(c.material, dofs));
            }
            let ground = foundation::force(&springs, &q);
            for (r, f) in residual.iter_mut().zip(&ground) {
                *r += f;
            }
            let error = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (residual[i] * scale[map[i]]).powi(2))
                .sum::<f64>()
                .sqrt();
            let norm = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (f[i] * scale[map[i]]).powi(2))
                .sum::<f64>()
                .sqrt();
            let relative_residual = error / norm.max(1e-20);
            if relative_residual > 1e-6 && error > 1e-10 {
                return Err(format!(
                    "Невязка решения слишком велика: {relative_residual:.3e}"
                ));
            }
            let applied_kn = f.iter().step_by(3).sum::<f64>();
            let reaction_kn = (0..mesh.nodes.len())
                .map(|i| {
                    if fixed.contains(&i) {
                        residual[3 * i]
                    } else {
                        -ground[3 * i]
                    }
                })
                .collect::<Vec<_>>();
            let balance_m_knm = std::array::from_fn(|axis| {
                mesh.nodes
                    .iter()
                    .enumerate()
                    .map(|(i, n)| (f[3 * i] + reaction_kn[i]) * n.xy[axis] / 1000.)
                    .sum()
            });
            Ok(CaseResult {
                punching: cuts.finish(),
                shell: None,
                id,
                w_mm: q.iter().step_by(3).map(|v| v * 1000.).collect(),
                rotations_rad: q.chunks_exact(3).map(|v| [-v[2], v[1], 0.]).collect(),
                moments,
                shears,
                loads_kn_m2,
                foundation_pressure_kn_m2: foundation::pressures(&springs, &q, mesh.elements.len()),
                balance_kn: applied_kn + reaction_kn.iter().sum::<f64>(),
                balance_m_knm,
                reaction_kn,
                applied_kn,
                relative_residual,
            })
        },
    )?;
    Ok(Results {
        nonlinear: None,
        lengths: None,
        modal: None,
        buckling: None,
        cases: results,
        free_dofs: free,
    })
}

#[cfg(test)]
mod tests {
    use super::super::{
        loads::{LoadData, Magnitude},
        meshing,
    };
    use super::*;
    #[test]
    fn self_weight_matches_area_load_uses_thickness_and_does_not_duplicate_overlaps() {
        use super::super::loads::{LoadCase, LoadType};
        let mut all = square();
        // An overlapping copy must not add gravity twice, even with equivalent units.
        let mut copy = all[0].clone();
        copy.id = 8;
        all.push(copy);
        let mut hole = all[0].clone();
        hole.id = 9;
        hole.kind = Kind::Hole;
        hole.parent = Some(1);
        hole.points = vec![
            [2000., 2000.],
            [3000., 2000.],
            [3000., 3000.],
            [2000., 3000.],
        ];
        // Copy the same opening to the overlapping slab.
        let mut hole2 = hole.clone();
        hole2.id = 10;
        hole2.parent = Some(8);
        all.extend([hole, hole2]);
        all[2].load.as_mut().unwrap().case_id = 2;
        all[2].load.as_mut().unwrap().magnitude = Magnitude {
            value: 5.5,
            units: ForceUnits::KiloNewton,
        };
        let mesh = meshing::generate(&all, 600.).unwrap();
        let mut own = LoadCase::new(1);
        own.kind = LoadType::SelfWeight;
        let cases = [own, LoadCase::new(2), LoadCase::new(3)];
        let mut kn = Material::default();
        kn.weight = Magnitude {
            value: 27.5,
            units: ForceUnits::KiloNewton,
        };
        let materials = BTreeMap::from([(1, Material::default()), (8, kn)]);
        for kind in ElementType::ALL {
            let r = solve_load_cases(&mesh, &all, &materials, &cases, 10., 600., kind).unwrap();
            let [own, area, empty] = r.cases.as_slice() else {
                panic!("three cases")
            };
            assert!((own.applied_kn - 192.5).abs() < 1e-8);
            assert!(own.loads_kn_m2.iter().all(|q| (*q - 5.5).abs() < 1e-12));
            assert!(empty.applied_kn.abs() < 1e-12);
            for (a, b) in own.w_mm.iter().zip(&area.w_mm) {
                assert!((a - b).abs() < 1e-9);
            }
            assert!(own.balance_kn.abs() < 1e-7);
        }
    }
    #[test]
    fn self_weight_respects_piecewise_thickness_and_gravity() {
        use super::super::loads::{LoadCase, LoadType};
        let mut all = square();
        all.retain(|c| !c.kind.is_load());
        // Two non-overlapping plates sharing an edge.
        all[0].points = vec![[0., 0.], [3000., 0.], [3000., 6000.], [0., 6000.]];
        let mut second = all[0].clone();
        second.id = 5;
        second.points = vec![[3000., 0.], [6000., 0.], [6000., 6000.], [3000., 6000.]];
        all.push(second);
        let material = Material {
            thickness_mm: 400.,
            ..Default::default()
        };
        let materials = BTreeMap::from([(1, Material::default()), (5, material)]);
        let mesh = meshing::generate(&all, 600.).unwrap();
        let mut case = LoadCase::new(1);
        case.kind = LoadType::SelfWeight;
        for kind in ElementType::ALL {
            let r = solve_load_cases(&mesh, &all, &materials, &[case.clone()], 9.81, 600., kind)
                .unwrap();
            assert!((r.cases[0].applied_kn - 291.357).abs() < 1e-7);
            for (e, q) in mesh.elements.iter().zip(&r.cases[0].loads_kn_m2) {
                let x = e
                    .nodes
                    .iter()
                    .map(|&n| mesh.nodes[n].xy[0] / 3.)
                    .sum::<f64>();
                let expected = if x < 3000. { 5.3955 } else { 10.791 };
                assert!((q - expected).abs() < 1e-10);
            }
        }
    }
    #[test]
    fn degenerate_element_error_identifies_actual_ids_and_coordinates() {
        let all = square();
        let mesh = Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![
                super::super::meshing::Node {
                    id: 7,
                    xy: [0., 0.],
                },
                super::super::meshing::Node {
                    id: 8,
                    xy: [1., 0.],
                },
                super::super::meshing::Node {
                    id: 9,
                    xy: [2., 0.],
                },
            ],
            elements: vec![super::super::meshing::Element {
                id: 99,
                nodes: [0, 1, 2],
                supports: vec![],
            }],
            edges: vec![],
            support_boundary: BTreeMap::new(),
        };
        let error = solve(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[1],
            10.,
            500.,
        )
        .err()
        .unwrap();
        assert!(
            error.contains("КЭ 99")
                && error.contains("7 (X=")
                && error.contains("9 (X=")
                && error.contains("Площадь")
        );
    }
    fn contour(id: u64, kind: Kind, points: Vec<Point>) -> Contour {
        Contour {
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
            id,
            kind,
            points,
            curves: vec![],
            parent: None,
            refinement: None,
            load: None,
        }
    }
    fn square() -> Vec<Contour> {
        let p = vec![[0., 0.], [6000., 0.], [6000., 6000.], [0., 6000.]];
        let mut load = contour(3, Kind::LoadArea, p.clone());
        load.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: Magnitude {
                value: 1.,
                units: ForceUnits::KiloNewton,
            },
            closed: true,
        });
        vec![
            contour(1, Kind::Slab, p.clone()),
            contour(2, Kind::Support, p),
            load,
        ]
    }
    #[test]
    fn nonlinear_flat_plate_develops_membrane_action_and_keeps_vertical_balance() {
        let all = square();
        let mesh = meshing::generate(&all, 1000.).unwrap();
        let materials = BTreeMap::from([(
            1,
            Material {
                thickness_mm: 50.,
                ..Default::default()
            },
        )]);
        let cases = vec![super::super::loads::LoadCase::new(1)];
        let settings = shell::Settings::default();
        let linear = solve_with_support_height(
            &mesh,
            &all,
            &materials,
            &cases,
            10.,
            1000.,
            ElementType::Dkt,
            &settings,
        )
        .unwrap();
        let request = NonlinearRequest {
            weights: vec![(1, 1.)],
            caption: "plate".into(),
        };
        let nonlinear = solve_with_modal(
            &mesh,
            &all,
            &materials,
            &cases,
            10.,
            1000.,
            ElementType::Dkt,
            &settings,
            None,
            false,
            Some(&request),
        )
        .unwrap();
        let a = linear.cases[0].w_mm.iter().copied().fold(0_f64, f64::max);
        let b = nonlinear.cases[0]
            .w_mm
            .iter()
            .copied()
            .fold(0_f64, f64::max);
        assert!(b > 0. && b < a * 0.99, "linear={a}, nonlinear={b}");
        assert!(nonlinear.cases[0].balance_kn.abs() < 1e-5);
        assert!(nonlinear.cases[0].relative_residual < 1e-6);
        assert!(nonlinear.cases[0]
            .shell
            .as_ref()
            .unwrap()
            .membrane
            .iter()
            .flatten()
            .any(|n| n.abs() > 1e-3));
    }
    #[test]
    fn mindlin_cr_thin_limit_does_not_lock_and_balances_loads() {
        let all = square();
        let mesh = meshing::generate(&all, 500.).unwrap();
        let mut previous = None;
        for thickness_mm in [200., 20., 2.] {
            let material = Material {
                thickness_mm,
                ..Default::default()
            };
            let r = solve_with_type(
                &mesh,
                &all,
                &BTreeMap::from([(1, material)]),
                &[1],
                10.,
                500.,
                ElementType::MindlinCr,
            )
            .unwrap();
            let c = &r.cases[0];
            let max = c.w_mm.iter().copied().fold(0., f64::max) / 1000.;
            let normalized = max * material.d()[0][0] / 6_f64.powi(4);
            assert!(
                (normalized / 0.00406235 - 1.).abs() < 0.2,
                "h={thickness_mm}, normalized={normalized}"
            );
            if let Some(p) = previous {
                assert!(normalized / p > 0.85, "thin plate collapses / locks");
            }
            previous = Some(normalized);
            assert!((c.applied_kn - 36.).abs() < 1e-8);
            assert!(c.balance_kn.abs() < 1e-3);
            assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-2));
        }
    }
    #[test]
    fn closed_support_fixes_only_perimeter_leaving_interior_w_free() {
        let mut all = square();
        all[1].points = vec![
            [2000., 2000.],
            [4000., 2000.],
            [4000., 4000.],
            [2000., 4000.],
        ];
        let mesh = meshing::generate(&all, 500.).unwrap();
        let fixed = &mesh.support_boundary[&2];
        let interior = mesh
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| {
                n.xy[0] > 2000. + 1e-6
                    && n.xy[0] < 4000. - 1e-6
                    && n.xy[1] > 2000. + 1e-6
                    && n.xy[1] < 4000. - 1e-6
            })
            .map(|(i, _)| i)
            .collect::<Vec<_>>();
        assert!(!interior.is_empty());
        assert!(interior.iter().all(|i| !fixed.contains(i)));
        let result = solve(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[1],
            10.,
            500.,
        )
        .unwrap();
        assert!(fixed.iter().all(|&i| result.cases[0].w_mm[i] == 0.));
        assert!(interior
            .iter()
            .any(|&i| result.cases[0].w_mm[i].abs() > 1e-10));
    }
    #[test]
    fn parallel_cases_match_serial_for_all_plate_kernels_and_preserve_case_order() {
        let mut all = square();
        for id in [2_u64, 3, 4] {
            let mut load = all[2].clone();
            load.id = 10 + id;
            load.load.as_mut().unwrap().case_id = id;
            load.load.as_mut().unwrap().magnitude.value = id as f64;
            all.push(load);
        }
        let mut cut = all[0].clone();
        cut.id = 30;
        cut.kind = Kind::Punching;
        cut.parent = Some(1);
        cut.points = vec![
            [2000., 2000.],
            [4000., 2000.],
            [4000., 4000.],
            [2000., 4000.],
        ];
        cut.refinement = Some(super::super::model::Refinement {
            name: "Область суммирования нагрузок 30".into(),
            size: 500.,
            inactive_edges: vec![],
        });
        all.push(cut);
        let raw = meshing::generate(&all, 1000.).unwrap();
        let materials = BTreeMap::from([(1, Material::default())]);
        for (kind, quads, height) in [
            (ElementType::Dkt, false, false),
            (ElementType::Dkt, true, false),
            (ElementType::MindlinCr, false, false),
            (ElementType::Dkt, true, true),
        ] {
            let mesh = super::super::quad_mesh::process(
                raw.clone(),
                super::super::quad_mesh::Options {
                    backend: Default::default(),
                    merge: quads,
                    optimise: false,
                },
            )
            .unwrap();
            let cases = [1, 2, 3, 4].map(crate::plan_trace::loads::LoadCase::new);
            let settings = shell::Settings {
                enabled: height,
                height_mm: 1000.,
                ..Default::default()
            };
            let run = || {
                solve_with_support_height(
                    &mesh, &all, &materials, &cases, 10., 1000., kind, &settings,
                )
                .unwrap()
            };
            let serial = parallel_cases::with_workers(1, run);
            let parallel = parallel_cases::with_workers(3, run);
            for (a, b) in serial.cases.iter().zip(&parallel.cases) {
                assert_eq!(a.id, b.id);
                assert!(b.balance_kn.abs() < 1e-6);
                for (x, y) in a.w_mm.iter().zip(&b.w_mm) {
                    assert!((x - y).abs() < 1e-10);
                }
                for (x, y) in a.reaction_kn.iter().zip(&b.reaction_kn) {
                    assert!((x - y).abs() < 1e-8);
                }
                for (x, y) in a.moments.iter().flatten().zip(b.moments.iter().flatten()) {
                    assert!((x - y).abs() < 1e-8);
                }
                for (x, y) in a.shears.iter().flatten().zip(b.shears.iter().flatten()) {
                    assert!((x - y).abs() < 1e-8);
                }
                assert!((a.punching[&30].force - b.punching[&30].force).abs() < 1e-8);
            }
        }
    }
    #[test]
    fn square_plate_matches_navier_and_reuses_stiffness_for_multiple_cases() {
        let mut all = square();
        let mut second = all[2].clone();
        second.id = 4;
        second.load.as_mut().unwrap().case_id = 2;
        second.load.as_mut().unwrap().magnitude.value = 2.;
        all.push(second);
        let mesh = meshing::generate(&all, 500.).unwrap();
        let result = solve(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[1, 2],
            10.,
            500.,
        )
        .unwrap();
        let d = Material::default().d()[0][0];
        let navier = 0.00406235 * 6_f64.powi(4) / d * 1000.;
        let max = result.cases[0].w_mm.iter().copied().fold(0., f64::max);
        assert!(
            (max / navier - 1.).abs() < 0.1,
            "DKT {max}, Navier {navier}"
        );
        for c in &result.cases {
            assert!(c.balance_kn.abs() < 1e-6);
            assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-5));
            assert!(c.relative_residual < 1e-8);
        }
        assert!((result.cases[0].applied_kn - 36.).abs() < 1e-8);
        for (a, b) in result.cases[0].w_mm.iter().zip(&result.cases[1].w_mm) {
            assert!((2. * a - b).abs() < 1e-9);
        }
    }
    #[test]
    fn dkt_mitc4_mesh_converges_to_navier_square_plate() {
        let all = square();
        let material = Material::default();
        let navier = 0.00406235 * 6_f64.powi(4) / material.d()[0][0] * 1000.;
        let mut errors = vec![];
        for step in [1000., 500.] {
            let mesh = meshing::generate(&all, step).unwrap();
            let mesh = super::super::quad_mesh::process(
                mesh,
                super::super::quad_mesh::Options {
                    backend: Default::default(),
                    merge: true,
                    optimise: true,
                },
            )
            .unwrap();
            assert!(!mesh.quads.is_empty());
            let result = solve(
                &mesh,
                &all,
                &BTreeMap::from([(1, material)]),
                &[1],
                10.,
                step,
            )
            .unwrap();
            let c = &result.cases[0];
            let max = c.w_mm.iter().copied().fold(0., f64::max);
            let error = (max / navier - 1.).abs();
            println!("DKT/MITC4 step {step}: W={max}, Navier={navier}, relative error={error}");
            assert!(error < 0.15);
            assert!(c.balance_kn.abs() < 1e-7);
            assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-6));
            errors.push(error);
        }
        assert!(errors[1] < errors[0] + 0.01);
    }
    #[test]
    fn missing_supports_are_rejected_instead_of_regularised() {
        let mut all = square();
        all.retain(|c| c.kind != Kind::Support);
        let mesh = meshing::generate(&all, 1000.).unwrap();
        let error = solve(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[1],
            10.,
            1000.,
        )
        .err()
        .unwrap();
        assert!(error.contains("Недостаточно опор"));
    }
    #[test]
    fn point_load_preserves_resultant_and_moments_on_an_oblique_triangle() {
        let xy = [[0., 0.], [3., 0.4], [0.7, 2.]];
        let cells = vec![Cell {
            triangle: Triangle::new(xy).unwrap(),
            nodes: [0, 1, 2],
            material: Material::default(),
            bbox: bbox(&xy),
        }];
        let index = Index::new(&cells, 1.);
        let p = [0.83, 0.68];
        let mut rhs = vec![0.; 9];
        point_force(&cells, &index, p, 17., &mut rhs).unwrap();
        assert!((rhs.iter().sum::<f64>() - 17.).abs() < 1e-12);
        for a in 0..2 {
            assert!(
                ((0..3).map(|i| rhs[3 * i] * xy[i][a]).sum::<f64>() - 17. * p[a]).abs() < 1e-12
            );
        }
        assert!(point_force(&cells, &index, [9., 9.], 1., &mut rhs).is_err());
    }
    #[test]
    fn area_load_activation_requires_positive_overlap_above_roundoff() {
        let t = Triangle::new([[0., 0.], [2., 0.], [0., 2.]]).unwrap();
        assert!(area_load_hits(
            &t,
            &polygon(&[[0.5, 0.5], [0.6, 0.5], [0.5, 0.6]])
        ));
        assert!(area_load_hits(
            &t,
            &polygon(&[[-1., 0.9], [3., 0.9], [3., 1.1], [-1., 1.1]])
        ));
        assert!(area_load_hits(
            &t,
            &polygon(&[[-1., -1.], [4., -1.], [4., 4.], [-1., 4.]])
        ));
        assert!(!area_load_hits(
            &t,
            &polygon(&[[0., 0.], [2., 0.], [2., -1.], [0., -1.]])
        ));
        assert!(!area_load_hits(
            &t,
            &polygon(&[[2., 0.], [3., 0.], [3., 1.]])
        ));

        // An oblique shared edge may differ from the load boundary by a few ULPs.
        let area = polygon(&[
            [0., 1.9999999999999998],
            [1.9999999999999998, 1.7999999999999998],
            [1.4, 0.],
        ]);
        let outside = Triangle::new([[0., 2.], [1., 1.9], [0.5, 2.]]).unwrap();
        assert!(!area_load_hits(&outside, &area));
        let other_side = Triangle::new([[0., 2.], [0.7, 1.], [0., 1.]]).unwrap();
        assert!(!area_load_hits(&other_side, &area));
        let real_crossing = Triangle::new([[0., 2.], [1., 1.9], [0.5, 1.8]]).unwrap();
        assert!(area_load_hits(&real_crossing, &area));
        let thin_sliver = polygon(&[[0., -1.], [2., -1.], [2., 1e-14], [0., 1e-14]]);
        assert!(!area_load_hits(&t, &thin_sliver));
        assert!(area_load_hits(
            &t,
            &polygon(&[[0., -1.], [2., -1.], [2., 0.001], [0., 0.001]])
        ));
    }
    #[test]
    fn overlapping_area_pressure_maximum_is_signed_and_order_independent() {
        for candidates in [
            vec![3., 8., 5., 8.],
            vec![8., 5., 8., 3.],
            vec![-5., -2., -4.],
            vec![-4., -2., -5.],
            vec![-2., 0.],
        ] {
            let mut previous = None;
            let mut assembled = 0.;
            for &candidate in &candidates {
                assembled += area_pressure_delta(&mut previous, candidate);
            }
            let expected = candidates.into_iter().fold(f64::NEG_INFINITY, f64::max);
            assert!((assembled - expected).abs() < 1e-12);
        }
    }
    #[test]
    fn partial_area_loads_apply_full_intensity_to_whole_element_and_sum_with_line() {
        let xy = [[0., 0.], [2., 0.], [0., 2.]];
        let cells = vec![Cell {
            triangle: Triangle::new(xy).unwrap(),
            nodes: [0, 1, 2],
            material: Material::default(),
            bbox: bbox(&xy),
        }];
        let index = Index::new(&cells, 0.5);
        let mut area = contour(
            1,
            Kind::LoadArea,
            vec![[0., 0.], [500., 0.], [500., 500.], [0., 500.]],
        );
        area.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: Magnitude {
                value: 8.,
                units: ForceUnits::KiloNewton,
            },
            closed: true,
        });
        let mut line = contour(2, Kind::LoadLine, vec![[0., 500.], [1000., 500.]]);
        line.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: Magnitude {
                value: 3.,
                units: ForceUnits::KiloNewton,
            },
            closed: false,
        });
        let (f, pressure) = load_vector(
            &cells,
            &index,
            &[area.clone(), area.clone(), line.clone()],
            1,
            10.,
            0.2,
            9,
        )
        .unwrap();
        assert!((pressure[0] * cells[0].triangle.area - 16.).abs() < 1e-12);
        assert!((f.iter().sum::<f64>() - 19.).abs() < 1e-12);
        assert!(
            ((0..3).map(|i| f[3 * i] * xy[i][0]).sum::<f64>() - (32. / 3. + 1.5)).abs() < 1e-12
        );
        assert!(
            ((0..3).map(|i| f[3 * i] * xy[i][1]).sum::<f64>() - (32. / 3. + 1.5)).abs() < 1e-12
        );
        let mut larger = area.clone();
        larger.id = 99;
        larger.load.as_mut().unwrap().magnitude.value = 12.;
        let mut other_case = larger.clone();
        other_case.id = 100;
        other_case.load.as_mut().unwrap().case_id = 2;
        other_case.load.as_mut().unwrap().magnitude.value = 100.;
        for zones in [
            vec![
                area.clone(),
                larger.clone(),
                line.clone(),
                other_case.clone(),
            ],
            vec![
                larger.clone(),
                area.clone(),
                other_case.clone(),
                line.clone(),
            ],
        ] {
            let (forces, pressures) = load_vector(&cells, &index, &zones, 1, 10., 0.2, 9).unwrap();
            assert!((pressures[0] - 12.).abs() < 1e-12);
            assert!((forces.iter().sum::<f64>() - 27.).abs() < 1e-12);
        }
        let mut base = area.clone();
        base.load.as_mut().unwrap().magnitude.value = 10.;
        let mut extra = area.clone();
        extra.id = 123;
        extra.load.as_mut().unwrap().magnitude.value = 5.;
        extra.load.as_mut().unwrap().additive = true;
        for zones in [
            vec![base.clone(), extra.clone()],
            vec![extra.clone(), base.clone()],
        ] {
            let (forces, pressures) = load_vector(&cells, &index, &zones, 1, 10., 0.2, 9).unwrap();
            assert!((pressures[0] - 15.).abs() < 1e-12);
            assert!((forces.iter().sum::<f64>() - 30.).abs() < 1e-12);
        }
        extra.load.as_mut().unwrap().additive = false;
        let (_, pressures) = load_vector(&cells, &index, &[extra, base], 1, 10., 0.2, 9).unwrap();
        assert!((pressures[0] - 10.).abs() < 1e-12);
        // Midpoint quadrature: three 0.5 m pieces lie in this triangle,
        // the remaining samples are outside and must not be redistributed.
        let mut crossing = contour(3, Kind::LoadLine, vec![[-1000., 500.], [2000., 500.]]);
        crossing.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: Magnitude {
                value: 3.,
                units: ForceUnits::KiloNewton,
            },
            closed: false,
        });
        let (f, _) = load_vector(&cells, &index, &[crossing], 1, 10., 0.5, 9).unwrap();
        assert!((f.iter().sum::<f64>() - 4.5).abs() < 1e-12);
    }
}

#[cfg(test)]
pub(super) fn source_area_projection(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[(super::loads::LoadCase, f64)],
    g: f64,
    step: f64,
) -> Result<Vec<f64>, String> {
    let source = cases
        .iter()
        .map(|(case, _)| case.clone())
        .collect::<Vec<_>>();
    let mut result = vec![0.; mesh.elements.len()];
    let mut factors = cases.iter().map(|(_, factor)| *factor);
    source_area_projection_cases(mesh, all, materials, &source, g, step, |_, values| {
        let factor = factors.next().expect("one weight per source case");
        for (sum, q) in result.iter_mut().zip(values) {
            *sum += q * factor;
        }
    })?;
    Ok(result)
}

pub(super) fn source_area_projection_cases(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    consume: impl FnMut(&super::loads::LoadCase, Vec<f64>),
) -> Result<(), String> {
    let cells = prepare_cells(mesh, all, materials, step, g)?;
    mixed::source_area_projection_cases(mesh, &cells, all, cases, g, step, consume)
}

/// Physical geometry and separated nodal/area/beam loads, before analysis.
pub(super) fn lira_model(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    settings: &shell::Settings,
) -> Result<(shell::Scene, Vec<super::lira_export::CaseLoads>), String> {
    if !g.is_finite() || g <= 0. || !step.is_finite() || step < 100. {
        return Err("Проверьте g и размер КЭ".into());
    }
    let cells = prepare_cells(mesh, all, materials, step, g)?;
    let scene = shell::scene(
        mesh,
        &cells.iter().map(|c| c.material).collect::<Vec<_>>(),
        settings,
        step,
        all,
    )?;
    if all.iter().any(|c| {
        c.load.as_ref().is_some_and(|l| {
            (l.moment || l.direction.index() != 2) && cases.iter().any(|case| case.id == l.case_id)
        })
    }) {
        return Err(
            "Экспорт ЛИРА пока поддерживает только нагрузки по Z без сосредоточенных моментов"
                .into(),
        );
    }
    let loads = lira_loads_for_scene(mesh, all, materials, cases, g, step, &scene, &cells)?;
    Ok((scene, loads))
}

pub(super) fn lira_loads_for_scene(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    scene: &shell::Scene,
    cells: &[Cell],
) -> Result<Vec<super::lira_export::CaseLoads>, String> {
    let mut loads = if mesh.building.is_some() {
        building_lira_loads(mesh, all, materials, cases, g, step, scene)?
    } else {
        mixed::export_loads(mesh, cells, all, cases, g, step, scene)?
    };
    for (case, load) in cases.iter().zip(&mut loads) {
        load.nodal.resize(scene.xyz.len(), 0.);
        let floor_count = scene
            .surfaces
            .iter()
            .take_while(|s| s.support.is_none())
            .count();
        if load.surface.len() != floor_count {
            return Err("Не совпадает порядок физических КЭ при экспорте".into());
        }
        load.surface.resize(scene.surfaces.len(), 0.);
        load.beam.resize(scene.beams.len(), 0.);
        if case.kind != super::loads::LoadType::SelfWeight {
            continue;
        }
        for (q, surface) in load.surface.iter_mut().zip(&scene.surfaces) {
            if surface.support.is_some() {
                *q = surface.material.self_weight_kn_m2(g);
            }
        }
        for (q, beam) in load.beam.iter_mut().zip(&scene.beams) {
            *q = beam
                .section
                .material
                .weight
                .in_units(ForceUnits::KiloNewton, g)
                * beam.section.area();
        }
    }
    Ok(loads)
}

/// Assemble each storey's loads in its own XY domain, preserving global joint IDs.
fn building_lira_loads(
    mesh: &Mesh,
    all: &[Contour],
    materials: &BTreeMap<u64, Material>,
    cases: &[super::loads::LoadCase],
    g: f64,
    step: f64,
    scene: &shell::Scene,
) -> Result<Vec<super::lira_export::CaseLoads>, String> {
    let data = mesh.building.as_ref().ok_or("Нет этажной топологии")?;
    let slab_count = scene
        .surfaces
        .iter()
        .take_while(|s| s.support.is_none())
        .count();
    let mut output = cases
        .iter()
        .map(|_| super::lira_export::CaseLoads {
            nodal: vec![0.; scene.xyz.len()],
            surface: vec![0.; slab_count],
            beam: vec![0.; scene.beams.len()],
        })
        .collect::<Vec<_>>();
    let mut planes = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (mesh.element_floor(i), mesh.node_z(e.nodes[0])))
        .collect::<Vec<_>>();
    planes.sort_by(|a, b| a.0.cmp(&b.0).then_with(|| a.1.total_cmp(&b.1)));
    planes.dedup_by(|a, b| a.0 == b.0 && (a.1 - b.1).abs() < 0.05);
    for (floor, z) in planes {
        let sources = all
            .iter()
            .filter(|c| data.layout.matches_source_plane(c, all, floor, z, 0.05))
            .cloned()
            .collect::<Vec<_>>();
        let elements = mesh
            .elements
            .iter()
            .enumerate()
            .filter(|(i, _)| {
                mesh.element_floor(*i) == floor
                    && (mesh.node_z(mesh.elements[*i].nodes[0]) - z).abs() < 0.05
            })
            .map(|(_, e)| e.clone())
            .collect::<Vec<_>>();
        if elements.is_empty() {
            continue;
        }
        let element_ids = elements.iter().map(|e| e.id).collect::<BTreeSet<_>>();
        let local = Mesh {
            building: None,
            nodes: mesh.nodes.clone(),
            quads: mesh
                .quads
                .iter()
                .filter(|q| {
                    q.display_triangles
                        .iter()
                        .all(|id| element_ids.contains(id))
                })
                .cloned()
                .collect(),
            elements,
            edges: vec![],
            constrained_nodes: BTreeSet::new(),
            constrained_edges: BTreeSet::new(),
            support_boundary: BTreeMap::new(),
        };
        let geometry = prepare_cells(&local, &sources, materials, step, g)?;
        let beam_indices = scene
            .beams
            .iter()
            .enumerate()
            .filter(|(_, b)| {
                mesh.contour_floor(b.id) == floor
                    && b.nodes.iter().all(|&n| (scene.xyz[n][2] - z).abs() < 0.05)
            })
            .map(|(i, _)| i)
            .collect::<Vec<_>>();
        let local_scene = shell::Scene {
            xyz: scene.xyz.iter().map(|p| [p[0], p[1], p[2] - z]).collect(),
            plan: vec![],
            surfaces: vec![],
            fixed: BTreeSet::new(),
            support_nodes: BTreeMap::new(),
            fixed_rotations: Default::default(),
            beams: beam_indices
                .iter()
                .map(|&i| scene.beams[i].clone())
                .collect(),
        };
        let local_loads =
            mixed::export_loads(&local, &geometry, &sources, cases, g, step, &local_scene)?;
        // Both global and local physical order is Q4 first, then standalone T3.
        let surface_indices = scene
            .surfaces
            .iter()
            .enumerate()
            .take(slab_count)
            .filter(|(_, s)| {
                s.display.first().is_some_and(|&i| {
                    mesh.element_floor(i) == floor
                        && (mesh.node_z(mesh.elements[i].nodes[0]) - z).abs() < 0.05
                })
            })
            .map(|(i, _)| i)
            .collect::<Vec<_>>();
        for (target, source) in output.iter_mut().zip(local_loads) {
            if source.surface.len() != surface_indices.len()
                || source.beam.len() != beam_indices.len()
            {
                return Err("Несогласованный порядок КЭ этажа при экспорте".into());
            }
            for (to, from) in target.nodal.iter_mut().zip(source.nodal) {
                *to += from;
            }
            for (&i, value) in surface_indices.iter().zip(source.surface) {
                target.surface[i] += value;
            }
            for (&i, value) in beam_indices.iter().zip(source.beam) {
                target.beam[i] += value;
            }
        }
    }
    Ok(output)
}

pub(crate) mod progress;

mod parallel_cases;
#[cfg(test)]
pub(crate) fn with_case_workers<R>(count: usize, f: impl FnOnce() -> R) -> R {
    parallel_cases::with_workers(count, f)
}

/// The same physical-element ownership and centre sampling used by Winkler assembly.
pub fn foundation_coefficients(mesh: &Mesh, all: &[Contour], g: f64) -> Result<Vec<f64>, String> {
    foundation::coefficients(mesh, all, g, None)
}

pub(super) fn lira_foundation(
    mesh: &Mesh,
    all: &[Contour],
    g: f64,
) -> Result<(Vec<f64>, BTreeMap<usize, f64>), String> {
    let mut horizontal = BTreeMap::new();
    let c1 = foundation::coefficients(mesh, all, g, Some(&mut horizontal))?;
    Ok((c1, horizontal))
}

pub(crate) fn precompute_wall_mesh(mesh: &mut Mesh, step: f64) -> Result<(), String> {
    building_scene::precompute_walls(mesh, step)
}
