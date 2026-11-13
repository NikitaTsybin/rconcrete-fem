//! Separate plate-plane triangulations with shared physical contact nodes and common XYZ joints.
use super::{
    geometry::Point,
    meshing::{self, Element, Mesh, Node},
    model::{self, Contour, Kind},
    quad_mesh,
};
use std::collections::{BTreeMap, BTreeSet, HashMap};
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Level {
    pub id: u64,
    pub z: f64,
    pub top: f64,
}
#[derive(Clone, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Layout {
    pub levels: Vec<Level>,
    pub owners: BTreeMap<u64, u64>,
    #[serde(default)]
    pub legacy_wall_extent: Option<[f64; 2]>,
}
impl Layout {
    pub(super) fn floor_extent(&self, c: &Contour) -> [f64; 2] {
        let level = self
            .levels
            .iter()
            .find(|l| l.id == self.owner(c.id))
            .unwrap();
        [level.z, level.top]
    }
    pub fn extent(&self, c: &Contour) -> [f64; 2] {
        let extent = self.floor_extent(c);
        if c.kind == Kind::Support {
            if c.placement.levels_default() {
                self.legacy_wall_extent.unwrap_or(extent)
            } else {
                c.placement.wall(extent)
            }
        } else if matches!(c.kind, Kind::Column | Kind::RigidLink) {
            c.placement.wall(extent)
        } else if c.is_free_beam() {
            c.beam_path(extent)
                .iter()
                .map(|p| p[2])
                .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], z| {
                    [lo.min(z), hi.max(z)]
                })
        } else {
            let z = c.placement.slab.z(extent);
            [z, z]
        }
    }
    pub fn source_z(&self, c: &Contour, all: &[Contour]) -> f64 {
        self.extent(Self::level_source(c, all))[0]
    }
    fn level_source<'a>(c: &'a Contour, all: &'a [Contour]) -> &'a Contour {
        if matches!(
            c.kind,
            Kind::Hole | Kind::MeshRegion | Kind::LoadHole | Kind::Punching
        ) {
            c.parent
                .and_then(|id| all.iter().find(|p| p.id == id))
                .unwrap_or(c)
        } else {
            c
        }
    }
    pub fn matches_source_plane(
        &self,
        c: &Contour,
        all: &[Contour],
        floor: u64,
        z: f64,
        tolerance: f64,
    ) -> bool {
        let source = Self::level_source(c, all);
        (self.owner(source.id) == floor
            || source.kind.is_load() && source.placement.slab != Default::default())
            && (self.extent(source)[0] - z).abs() < tolerance
    }
    pub fn owner(&self, id: u64) -> u64 {
        self.owners.get(&id).copied().unwrap_or(0)
    }
    pub fn validate(&self) -> Result<(), String> {
        let mut ids = BTreeSet::new();
        let mut z = BTreeSet::new();
        if self.levels.is_empty() || self.levels.len() > 500 {
            return Err("Некорректные этажи расчётной модели".into());
        }
        for l in &self.levels {
            if l.id == 0
                || !ids.insert(l.id)
                || !l.z.is_finite()
                || !l.top.is_finite()
                || l.top - l.z < 100.
                || !z.insert((l.z * 1e4).round() as i64)
            {
                return Err("Некорректные отметки этажей".into());
            }
        }
        if self.owners.values().any(|id| !ids.contains(id)) {
            return Err("Объект относится к отсутствующему этажу".into());
        }
        Ok(())
    }
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Wall {
    pub id: u64,
    #[serde(default)]
    pub mesh_size_mm: f64,
    pub bottom: [usize; 2],
    pub top: [usize; 2],
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Beam {
    #[serde(default)]
    pub roll_angle_rad: f64,
    pub id: u64,
    pub nodes: [usize; 2],
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Data {
    #[serde(default)]
    pub wall_mesh_step: f64,
    #[serde(default)]
    pub wall_mesh: Option<Vec<(u64, [[f64; 3]; 3])>>,
    #[serde(default)]
    pub wall_quads: Vec<(u64, [[f64; 3]; 4])>,
    #[serde(default)]
    pub wall_quad_options: quad_mesh::Options,
    #[serde(default)]
    pub contacts: Vec<[[f64; 3]; 2]>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub rigid_links: Vec<super::rigid_links::Station>,
    pub layout: Layout,
    pub z: Vec<f64>,
    pub node_floors: Vec<u64>,
    pub element_floors: Vec<u64>,
    pub element_slabs: Vec<u64>,
    pub walls: Vec<Wall>,
    pub beams: Vec<Beam>,
}
impl Data {
    pub fn validate(&self, mesh: &Mesh) -> Result<(), String> {
        self.layout.validate()?;
        if self.z.len() != mesh.nodes.len()
            || self.node_floors.len() != mesh.nodes.len()
            || self.element_floors.len() != mesh.elements.len()
            || self.element_slabs.len() != mesh.elements.len()
            || self.z.iter().any(|z| !z.is_finite())
            || self.walls.iter().any(|w| {
                w.bottom
                    .iter()
                    .chain(&w.top)
                    .any(|&n| n >= mesh.nodes.len())
            })
            || self
                .beams
                .iter()
                .any(|b| b.nodes.iter().any(|&n| n >= mesh.nodes.len()))
        {
            return Err("Некорректная пространственная топология здания".into());
        }
        Ok(())
    }

    pub(super) fn validate_beam_edges(
        &self,
        mesh: &Mesh,
        is_column: impl Fn(u64) -> bool,
    ) -> Result<(), String> {
        let mut edges = std::collections::HashMap::with_capacity(self.beams.len());
        for beam in &self.beams {
            let mut key = beam.nodes;
            key.sort_unstable();
            if let Some(previous) = edges.insert(key, beam.id) {
                let describe = |id| {
                    let kind = if is_column(id) {
                        "колонна"
                    } else {
                        "балка"
                    };
                    format!(
                        "{kind} {id} (объект {id}, этаж ID {})",
                        self.layout.owner(id)
                    )
                };
                let [a, b] = key.map(|n| {
                    let [x, y] = mesh.nodes[n].xy;
                    [x, y, self.z[n]]
                });
                let length = (b[0] - a[0]).hypot(b[1] - a[1]).hypot(b[2] - a[2]);
                return Err(format!(
                    "Совпадающие стержни здания: {} и {}. Общий участок: X={:.6}, Y={:.6}, Z={:.6}; X={:.6}, Y={:.6}, Z={:.6} мм. Длина {:.6} мм; узлы {} и {}",
                    describe(previous), describe(beam.id), a[0], a[1], a[2], b[0], b[1], b[2],
                    length, key[0] + 1, key[1] + 1,
                ));
            }
        }
        Ok(())
    }
}
/// Numerical coincidence only, not an engineering snapping tolerance.
pub(super) const WELD_TOLERANCE_MM: f64 = 1e-5;

fn beam_breakpoints(a: Point, b: Point, pins: &[Point]) -> Vec<f64> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len = d[0].hypot(d[1]);
    let mut ts = vec![0., 1.];
    ts.extend(
        pins.iter()
            .filter(|&&p| meshing::on_segment(p, [a, b]))
            .map(|p| {
                let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (len * len);
                if t * len < WELD_TOLERANCE_MM {
                    0.
                } else if (1. - t) * len < WELD_TOLERANCE_MM {
                    1.
                } else {
                    t
                }
            }),
    );
    ts.sort_by(f64::total_cmp);
    ts.dedup_by(|a, b| (*a - *b).abs() * len < WELD_TOLERANCE_MM);
    ts
}

fn weld_coincident(a: [f64; 3], b: [f64; 3]) -> bool {
    (0..3).all(|i| (a[i] - b[i]).abs() < WELD_TOLERANCE_MM)
}

pub struct Weld {
    pub xyz: Vec<[f64; 3]>,
    bins: HashMap<(i64, i64, i64), Vec<usize>>,
}
impl Weld {
    pub fn new() -> Self {
        Self {
            xyz: vec![],
            bins: HashMap::new(),
        }
    }
    pub(super) fn find(&self, p: [f64; 3]) -> Option<usize> {
        let k = p.map(|v| (v / WELD_TOLERANCE_MM).floor() as i64);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    if let Some(ids) = self.bins.get(&(k[0] + dx, k[1] + dy, k[2] + dz)) {
                        for &n in ids {
                            if weld_coincident(self.xyz[n], p) {
                                return Some(n);
                            }
                        }
                    }
                }
            }
        }
        None
    }
    pub fn add(&mut self, p: [f64; 3]) -> usize {
        if let Some(n) = self.find(p) {
            return n;
        }
        let k = p.map(|v| (v / WELD_TOLERANCE_MM).floor() as i64);
        let n = self.xyz.len();
        self.xyz.push(p);
        self.bins.entry((k[0], k[1], k[2])).or_default().push(n);
        n
    }

    pub(super) fn add_face<const N: usize>(
        &mut self,
        points: [[f64; 3]; N],
        valid: impl FnOnce([[f64; 3]; N]) -> bool,
    ) -> Result<[usize; N], [[f64; 3]; N]> {
        let before = self.xyz.len();
        let indices = points.map(|p| self.add(p));
        let welded = indices.map(|n| self.xyz[n]);
        if valid(welded) {
            return Ok(indices);
        }
        // Roll back only nodes newly created for a rejected face.
        while self.xyz.len() > before {
            let p = self.xyz.pop().unwrap();
            let [x, y, z] = p.map(|v| (v / WELD_TOLERANCE_MM).floor() as i64);
            let bin = self.bins.get_mut(&(x, y, z)).unwrap();
            let removed = bin.pop();
            debug_assert_eq!(removed, Some(self.xyz.len()));
            if bin.is_empty() {
                self.bins.remove(&(x, y, z));
            }
        }
        Err(welded)
    }
}
fn node(mesh: &mut Mesh, data: &mut Data, weld: &mut Weld, p: Point, z: f64) -> usize {
    let n = weld.add([p[0], p[1], z]);
    if n == mesh.nodes.len() {
        mesh.nodes.push(Node {
            id: n as u64 + 1,
            xy: p,
        });
        data.z.push(z);
        data.node_floors.push(
            data.layout
                .levels
                .iter()
                .find(|l| (l.z - z).abs() < 1e-5)
                .map_or(0, |l| l.id),
        );
    }
    n
}
fn edge(a: usize, b: usize) -> [usize; 2] {
    [a.min(b), a.max(b)]
}

fn mesh_free_beams(
    all: &[Contour],
    size: f64,
    physical_contacts: &[[[f64; 3]; 2]],
    point_pins: &[[f64; 3]],
    mesh: &mut Mesh,
    data: &mut Data,
    weld: &mut Weld,
) -> Result<(), String> {
    use rstar::{RTree, AABB};
    let beams = all.iter().filter(|c| c.is_free_beam()).collect::<Vec<_>>();
    if beams.is_empty() {
        return Ok(());
    }
    let pins: RTree<[f64; 3]> = RTree::bulk_load(
        physical_contacts
            .iter()
            .flatten()
            .copied()
            .chain(point_pins.iter().copied())
            .chain(
                mesh.nodes
                    .iter()
                    .zip(&data.z)
                    .map(|(node, &z)| [node.xy[0], node.xy[1], z]),
            )
            .collect(),
    );
    for c in beams {
        super::fem::progress::check_cancel()?;
        let step = c.mesh_step(size);
        if !step.is_finite() || step < 100. {
            return Err(format!("Балка {}: размер КЭ меньше 100 мм", c.id));
        }
        let path = c.beam_path(data.layout.floor_extent(c));
        for segment in path.windows(2) {
            let [a, b] = [segment[0], segment[1]];
            let d = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
            let l2 = d.iter().map(|x| x * x).sum::<f64>();
            let length = l2.sqrt();
            if !length.is_finite() || length <= WELD_TOLERANCE_MM {
                return Err(format!("Балка {}: совпадают вершины в пространстве", c.id));
            }
            let at = |t: f64| {
                if t == 0. {
                    a
                } else if t == 1. {
                    b
                } else {
                    std::array::from_fn(|i| a[i] + t * d[i])
                }
            };
            let bounds = AABB::from_corners(
                std::array::from_fn(|i| a[i].min(b[i]) - WELD_TOLERANCE_MM),
                std::array::from_fn(|i| a[i].max(b[i]) + WELD_TOLERANCE_MM),
            );
            let mut ts = vec![0., 1.];
            for p in pins.locate_in_envelope_intersecting(&bounds) {
                let t = (0..3).map(|i| (p[i] - a[i]) * d[i]).sum::<f64>() / l2;
                if (0. ..=1.).contains(&t) && weld_coincident(at(t), *p) {
                    ts.push(t);
                }
            }
            ts.sort_by(f64::total_cmp);
            ts.dedup_by(|a, b| (*a - *b).abs() * length < WELD_TOLERANCE_MM);
            for pair in ts.windows(2) {
                let count = (((pair[1] - pair[0]) * length / step - 1e-10).ceil() as usize).max(1);
                if count.saturating_add(data.beams.len()) > super::limits::MAX_ELEMENTS {
                    return Err("Превышен лимит стержневых КЭ здания".into());
                }
                for i in 0..count {
                    let xyz = [i, i + 1].map(|i| {
                        at(if i == 0 {
                            pair[0]
                        } else if i == count {
                            pair[1]
                        } else {
                            pair[0] + (pair[1] - pair[0]) * (i as f64 / count as f64)
                        })
                    });
                    let nodes = xyz.map(|[x, y, z]| node(mesh, data, weld, [x, y], z));
                    if mesh.nodes.len() > super::limits::MAX_NODES {
                        return Err("Превышен лимит пространственной сетки здания".into());
                    }
                    if nodes[0] == nodes[1] {
                        continue;
                    }
                    mesh.constrained_edges.insert(edge(nodes[0], nodes[1]));
                    mesh.constrained_nodes.extend(nodes);
                    data.beams.push(Beam {
                        id: c.id,
                        roll_angle_rad: c.column_angle,
                        nodes,
                    });
                }
            }
        }
    }
    Ok(())
}
// Keep only portions not owned by the canonical contact chain. Sampling both
// the contour and an overlapping contact creates two shifted sequences of nodes.
pub(super) fn outside_contacts(edge: [Point; 2], contacts: &[[Point; 2]]) -> Vec<[Point; 2]> {
    let [a, b] = edge;
    let d = [b[0] - a[0], b[1] - a[1]];
    let l = d[0].hypot(d[1]);
    if l < 1e-6 {
        return vec![];
    }
    let mut covered = Vec::new();
    for e in contacts {
        if e.iter()
            .all(|p| ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / l < 1e-5)
        {
            let mut t = e.map(|p| ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (l * l));
            t.sort_by(f64::total_cmp);
            let lo = t[0].max(0.);
            let hi = t[1].min(1.);
            if hi - lo > 1e-5 / l {
                covered.push([lo, hi]);
            }
        }
    }
    covered.sort_by(|a, b| a[0].total_cmp(&b[0]));
    let at = |t: f64| [a[0] + d[0] * t, a[1] + d[1] * t];
    let mut out = Vec::new();
    let mut end = 0.;
    for [lo, hi] in covered {
        if lo - end > 1e-5 / l {
            out.push([at(end), at(lo)]);
        }
        end = end.max(hi);
    }
    if 1. - end > 1e-5 / l {
        out.push([at(end), b]);
    }
    out
}
fn polygon(points: &[Point]) -> mini_fem::geo::Polygon<f64> {
    use mini_fem::geo::{LineString, Polygon};
    Polygon::new(
        LineString::from(
            points
                .iter()
                .copied()
                .chain(points.first().copied())
                .map(|p| (p[0], p[1]))
                .collect::<Vec<_>>(),
        ),
        vec![],
    )
}

type SlabDomain = (u64, u64, f64, mini_fem::geo::MultiPolygon<f64>, f64);

fn check_coplanar_slab_overlap(domains: &[SlabDomain]) -> Result<(), String> {
    use mini_fem::geo::{
        coordinate_position::CoordPos, dimensions::Dimensions, Area, BooleanOps, BoundingRect,
        Relate,
    };
    use rstar::{
        primitives::{GeomWithData, Rectangle},
        RTree, RTreeObject, AABB,
    };

    let index = RTree::bulk_load(
        domains
            .iter()
            .enumerate()
            .filter_map(|(i, (_, _, z, domain, _))| {
                let bounds = domain.bounding_rect()?;
                Some(GeomWithData::new(
                    Rectangle::from_aabb(AABB::from_corners(
                        [bounds.min().x, bounds.min().y, *z],
                        [bounds.max().x, bounds.max().y, *z],
                    )),
                    i,
                ))
            })
            .collect(),
    );
    for entry in index.iter() {
        let (floor, slab, z, domain, _) = &domains[entry.data];
        let bounds = entry.envelope();
        let [x0, y0, _] = bounds.lower();
        let [x1, y1, _] = bounds.upper();
        let search = AABB::from_corners([x0, y0, z - 1e-5], [x1, y1, z + 1e-5]);
        for other in index.locate_in_envelope_intersecting(&search) {
            if other.data <= entry.data {
                continue;
            }
            let (other_floor, other_slab, other_z, other_domain, _) = &domains[other.data];
            if floor == other_floor
                || (z - other_z).abs() >= 1e-5
                || domain
                    .relate(other_domain)
                    .get(CoordPos::Inside, CoordPos::Inside)
                    != Dimensions::TwoDimensional
            {
                continue;
            }
            let area_mm2 = domain.intersection(other_domain).unsigned_area();
            if area_mm2 > 1e-6 {
                return Err(format!(
                    "Наложение плит {slab} (этаж {floor}) и {other_slab} (этаж {other_floor}) на Z={z:.6} мм. Площадь {:.6} м². Проверьте геометрию и привязки уровней.",
                    area_mm2 / 1e6,
                ));
            }
        }
    }
    Ok(())
}

pub fn generate(
    all: &[Contour],
    size: f64,
    layout: Layout,
    options: quad_mesh::Options,
) -> Result<Mesh, String> {
    let graph = super::intersections::build_sampled(all, Some(&layout), [0., 0.], size)?;
    let contacts = graph
        .lines
        .iter()
        .map(|s| s.ends)
        .chain(graph.points.iter().map(|(p, _)| [*p, *p]))
        .collect::<Vec<_>>();
    generate_with_intersections(all, size, layout, options, &contacts)
}
pub fn generate_with_intersections(
    all: &[Contour],
    size: f64,
    layout: Layout,
    options: quad_mesh::Options,
    physical_contacts: &[[[f64; 3]; 2]],
) -> Result<Mesh, String> {
    use super::fem::progress;
    use mini_fem::geo::{BooleanOps, Contains};
    layout.validate()?;
    if all.iter().any(|c| layout.owner(c.id) == 0) {
        return Err("Не назначен этаж исходного объекта".into());
    }
    for beam in all.iter().filter(|c| c.is_free_beam()) {
        model::validate_candidate(all, beam).map_err(|e| format!("Балка {}: {e}", beam.id))?;
    }
    let rigid_links = super::rigid_links::prepare(all, &layout, size, physical_contacts)?;
    let mut rigid_pins = BTreeMap::<u64, Vec<[f64; 3]>>::new();
    for station in &rigid_links {
        for attachment in std::iter::once(&station.master).chain(&station.slaves) {
            rigid_pins
                .entry(attachment.owner)
                .or_default()
                .push(attachment.xyz);
        }
    }
    let wall_link_pins = all
        .iter()
        .filter(|c| c.kind == Kind::Support)
        .filter_map(|c| rigid_pins.get(&c.id))
        .flatten()
        .copied()
        .collect::<Vec<_>>();
    let spatial_pins = all
        .iter()
        .filter(|c| {
            c.kind == Kind::PointRestraint
                || (c.kind == Kind::LoadPoint
                    && c.load
                        .as_ref()
                        .is_some_and(|l| l.mesh_constraint || l.moment))
        })
        .filter_map(|c| c.points.first().map(|p| [p[0], p[1], layout.extent(c)[0]]))
        .collect::<Vec<_>>();
    let mut wall_contacts = physical_contacts.to_vec();
    wall_contacts.extend(spatial_pins.iter().map(|p| [*p, *p]));
    wall_contacts.extend(wall_link_pins.iter().map(|p| [*p, *p]));
    let mut domains = vec![];
    for c in all.iter().filter(|c| c.kind == Kind::Slab) {
        let mut domain =
            mini_fem::geo::MultiPolygon(vec![polygon(&c.mesh_boundary(c.mesh_step(size))?)]);
        for h in all
            .iter()
            .filter(|h| h.active_hole() && h.parent == Some(c.id))
        {
            domain = domain.difference(&polygon(&h.mesh_boundary(size)?));
        }
        use mini_fem::geo::Area;
        let priority_area = domain.unsigned_area();
        domains.push((
            layout.owner(c.id),
            c.id,
            layout.extent(c)[0],
            domain,
            priority_area,
        ));
    }
    check_coplanar_slab_overlap(&domains)?;
    let free_beams = all.iter().any(Contour::is_free_beam);
    let mut contact_pins_by_z = BTreeMap::<i64, Vec<[f64; 3]>>::new();
    if free_beams {
        use rstar::{primitives::Line, RTree};
        let axes: RTree<Line<[f64; 3]>> = RTree::bulk_load(
            all.iter()
                .filter(|c| c.is_free_beam())
                .flat_map(|c| {
                    c.beam_path(layout.floor_extent(c))
                        .windows(2)
                        .map(|e| Line::new(e[0], e[1]))
                        .collect::<Vec<_>>()
                })
                .collect(),
        );
        for [a, b] in physical_contacts {
            if weld_coincident(*a, *b)
                && axes
                    .locate_within_distance(*a, 3. * WELD_TOLERANCE_MM.powi(2))
                    .next()
                    .is_some()
            {
                contact_pins_by_z
                    .entry((a[2] / WELD_TOLERANCE_MM).floor() as i64)
                    .or_default()
                    .push(*a);
            }
        }
    }
    let contact_pins_at = |z: f64| {
        let key = (z / WELD_TOLERANCE_MM).floor() as i64;
        contact_pins_by_z
            .range(key - 1..=key + 1)
            .flat_map(|(_, pins)| pins.iter())
            .filter(move |p| (p[2] - z).abs() < WELD_TOLERANCE_MM)
    };
    let paths = all
        .iter()
        .filter(|c| matches!(c.kind, Kind::Support | Kind::Beam) && !c.is_free_beam())
        .map(|c| {
            let mut p = c.mesh_boundary(c.mesh_step(size))?;
            if c.closed() && !p.is_empty() {
                p.push(p[0]);
            }
            Ok((c, p))
        })
        .collect::<Result<Vec<_>, String>>()?;
    // A contact network contains only wall/beam lines, not plate/hole outlines.
    // Other plane outlines supply intersection anchors ON those physical lines.
    // They never become constraints in another plate's interior.
    // Build finite XY contact networks per physical height range. A contour
    // on an unrelated storey must not cut every wall sharing its XY projection.
    let source_lines = model::mesh_sources(all)
        .filter(|c| !c.is_free_beam())
        .map(|c| {
            let mut p = c.mesh_boundary(c.mesh_step(size))?;
            if c.closed() && !p.is_empty() {
                p.push(p[0]);
            }
            let extent = if matches!(c.kind, Kind::Support | Kind::Beam) {
                layout.extent(&c)
            } else {
                let z = layout.source_z(&c, all);
                [z, z]
            };
            Ok((c, p, extent))
        })
        .collect::<Result<Vec<_>, String>>()?;
    let mut groups = BTreeMap::<[i64; 2], Vec<usize>>::new();
    for (i, (c, _)) in paths.iter().enumerate() {
        groups
            .entry(layout.extent(c).map(|z| (z * 1e5).round() as i64))
            .or_default()
            .push(i);
    }
    let overlaps = |a: [f64; 2], b: [f64; 2]| a[0] <= b[1] + 1e-5 && b[0] <= a[1] + 1e-5;
    let mut contacts = Vec::<([Point; 2], Vec<[f64; 2]>)>::new();
    for members in groups.values() {
        let extent = layout.extent(paths[members[0]].0);
        let mut lines = Vec::new();
        let mut unique = BTreeSet::new();
        let mut point_anchors = Vec::new();
        for e in physical_contacts
            .iter()
            .filter(|e| overlaps([e[0][2].min(e[1][2]), e[0][2].max(e[1][2])], extent))
        {
            point_anchors.extend(e.iter().map(|p| [p[0], p[1]]));
        }
        for (c, p, h) in source_lines.iter().filter(|(_, _, h)| overlaps(*h, extent)) {
            if c.kind == Kind::LoadPoint {
                point_anchors.extend(p.iter().copied());
            }
            for e in p.windows(2) {
                let mut key =
                    [e[0], e[1]].map(|p| p.map(|v| if v == 0. { 0 } else { v.to_bits() }));
                key.sort();
                if unique.insert(key) {
                    lines.push([e[0], e[1]]);
                }
            }
            let _ = h;
        }
        for [a, b] in meshing::split_constraints(lines)? {
            let covering = members
                .iter()
                .copied()
                .filter(|&i| {
                    paths[i].1.windows(2).any(|e| {
                        meshing::on_segment(a, [e[0], e[1]]) && meshing::on_segment(b, [e[0], e[1]])
                    })
                })
                .collect::<Vec<_>>();
            if covering.is_empty() {
                continue;
            }
            let requested = covering
                .iter()
                .map(|&i| paths[i].0.mesh_step(size))
                .fold(size, f64::min);
            if !requested.is_finite() || requested < 100. {
                return Err("Размер КЭ должен быть не менее 100 мм".into());
            }
            let d = [b[0] - a[0], b[1] - a[1]];
            let length = d[0].hypot(d[1]);
            if length < 1e-6 {
                continue;
            }
            let mut ts = vec![0., 1.];
            ts.extend(
                point_anchors
                    .iter()
                    .filter(|&&p| meshing::on_segment(p, [a, b]))
                    .map(|p| ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (length * length)),
            );
            ts.sort_by(f64::total_cmp);
            ts.dedup_by(|a, b| (*a - *b).abs() * length < 1e-6);
            for interval in ts.windows(2) {
                let count =
                    (((interval[1] - interval[0]) * length / requested).ceil() as usize).max(1);
                if count > super::limits::MAX_NODES
                    || contacts.len().saturating_add(count) > super::limits::MAX_NODES
                {
                    return Err("Превышен лимит узлов контакта конструкций".into());
                }
                for k in 0..count {
                    let edge = [k, k + 1].map(|i| {
                        let t = interval[0] + (interval[1] - interval[0]) * i as f64 / count as f64;
                        [a[0] + d[0] * t, a[1] + d[1] * t]
                    });
                    contacts.push((edge, vec![extent]));
                }
            }
        }
    }
    let mut data = Data {
        wall_quad_options: options,
        wall_mesh_step: 0.,
        wall_mesh: None,
        wall_quads: vec![],
        contacts: wall_contacts,
        rigid_links,
        layout,
        z: vec![],
        node_floors: vec![],
        element_floors: vec![],
        element_slabs: vec![],
        walls: vec![],
        beams: vec![],
    };
    let mut mesh = Mesh {
        building: None,
        nodes: vec![],
        quads: vec![],
        elements: vec![],
        edges: vec![],
        constrained_edges: BTreeSet::new(),
        constrained_nodes: BTreeSet::new(),
        support_boundary: BTreeMap::new(),
    };
    let mut weld = Weld::new();
    let mut physical = BTreeMap::new();
    for level in data.layout.levels.clone() {
        let mut planes = domains
            .iter()
            .filter(|d| d.0 == level.id)
            .map(|d| d.2)
            .collect::<Vec<_>>();
        planes.sort_by(f64::total_cmp);
        planes.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        for z in planes {
            progress::check_cancel()?;
            if options.backend == meshing::Backend::Gmsh {
                progress::stage(&format!("Gmsh · плиты на Z = {z:.1} мм"));
            }
            let sources = all
                .iter()
                .filter(|c| {
                    if matches!(
                        c.kind,
                        Kind::Column | Kind::RigidLink | Kind::PointRestraint
                    ) || c.is_free_beam()
                    {
                        return false;
                    }
                    if matches!(c.kind, Kind::Support | Kind::Beam) {
                        let extent = data.layout.extent(c);
                        z >= extent[0] - 1e-5 && z <= extent[1] + 1e-5
                    } else {
                        data.layout.matches_source_plane(c, all, level.id, z, 1e-5)
                    }
                })
                .cloned()
                .collect::<Vec<_>>();
            let canonical = physical_contacts
                .iter()
                .filter(|e| (e[0][2] - z).abs() < 1e-5 && (e[1][2] - z).abs() < 1e-5)
                .map(|e| e.map(|p| [p[0], p[1]]))
                .filter(|e| (e[0][0] - e[1][0]).hypot(e[0][1] - e[1][1]) > 1e-5)
                .collect::<Vec<_>>();
            let mut fixed = contacts
                .iter()
                .filter(|(_, spans)| spans.iter().any(|e| z >= e[0] - 1e-5 && z <= e[1] + 1e-5))
                .flat_map(|(e, _)| outside_contacts(*e, &canonical))
                .collect::<Vec<_>>();
            fixed.extend(canonical);
            let slab_pins = sources
                .iter()
                .filter(|c| c.kind == Kind::Slab)
                .flat_map(|c| rigid_pins.get(&c.id).into_iter().flatten())
                .chain(
                    wall_link_pins
                        .iter()
                        .filter(|p| (p[2] - z).abs() < WELD_TOLERANCE_MM),
                )
                .chain(contact_pins_at(z))
                .chain(
                    spatial_pins
                        .iter()
                        .filter(|p| (p[2] - z).abs() < WELD_TOLERANCE_MM),
                )
                .map(|p| [p[0], p[1]])
                .collect::<Vec<_>>();
            let master = meshing::generate_with_contacts_and_pins(
                &sources,
                size,
                &fixed,
                &slab_pins,
                options.backend,
            )?;
            let mut remap = BTreeMap::new();
            let mut face_remap = BTreeMap::new();
            for triangle in &master.elements {
                let center = mini_fem::geo::Point::new(
                    triangle
                        .nodes
                        .iter()
                        .map(|&n| master.nodes[n].xy[0] / 3.)
                        .sum(),
                    triangle
                        .nodes
                        .iter()
                        .map(|&n| master.nodes[n].xy[1] / 3.)
                        .sum(),
                );
                // A local slab patch (capital/drop) replaces the enclosing slab.
                // Use the same priority as FEM material assignment; never stack FE.
                let Some((_, slab, _, _, _)) = domains
                    .iter()
                    .filter(|(floor, _, plane, area, _)| {
                        *floor == level.id && (*plane - z).abs() < 1e-5 && area.contains(&center)
                    })
                    .min_by(|a, b| a.4.total_cmp(&b.4).then_with(|| b.1.cmp(&a.1)))
                else {
                    continue;
                };
                // Evaluate the welded geometry before adding a face or new nodes.
                // Distinct CDT vertices can resolve to the same existing node.
                let points = triangle.nodes.map(|n| master.nodes[n].xy);
                let mut welded = points;
                for (i, [x, y]) in points.into_iter().enumerate() {
                    if let Some(n) = weld.find([x, y, z]) {
                        welded[i] = mesh.nodes[n].xy;
                    } else if let Some(&prior) = welded[..i]
                        .iter()
                        .find(|p| (p[0] - x).abs() < 1e-5 && (p[1] - y).abs() < 1e-5)
                    {
                        welded[i] = prior;
                    }
                }
                let longest = (0..3)
                    .map(|i| {
                        let a = welded[i];
                        let b = welded[(i + 1) % 3];
                        (a[0] - b[0]).hypot(a[1] - b[1])
                    })
                    .fold(0_f64, f64::max);
                let u = [welded[1][0] - welded[0][0], welded[1][1] - welded[0][1]];
                let v = [welded[2][0] - welded[0][0], welded[2][1] - welded[0][1]];
                if (u[0] * v[1] - u[1] * v[0]).abs() <= (longest * 1e-5).max(1e-8) {
                    continue;
                }
                let ns = triangle.nodes.map(|n| {
                    let index = node(&mut mesh, &mut data, &mut weld, master.nodes[n].xy, z);
                    data.node_floors[index] = level.id;
                    remap.insert(n, index);
                    index
                });
                let mut unique = ns;
                unique.sort();
                if let Some((old_floor, old_slab)) = physical.insert(unique, (level.id, *slab)) {
                    return Err(format!("Совпадают КЭ плит {old_slab} (этаж {old_floor}) и {slab} (этаж {}), Z={z:.6} мм; узлы {:?}", level.id, ns.map(|n| n + 1)));
                }
                mesh.elements.push(Element {
                    id: mesh.elements.len() as u64 + 1,
                    nodes: ns,
                    supports: vec![],
                });
                data.element_floors.push(level.id);
                data.element_slabs.push(*slab);
                face_remap.insert(triangle.id, mesh.elements.last().unwrap().id);
            }
            for q in &master.quads {
                if let (Some(&a), Some(&b)) = (
                    face_remap.get(&q.display_triangles[0]),
                    face_remap.get(&q.display_triangles[1]),
                ) {
                    if data.element_slabs[a as usize - 1] != data.element_slabs[b as usize - 1] {
                        continue;
                    }
                    let ns = q.nodes.map(|n| remap[&n]);
                    let mut unique = ns;
                    unique.sort_unstable();
                    if unique.windows(2).any(|p| p[0] == p[1]) {
                        continue;
                    }
                    mesh.quads.push(meshing::Quad {
                        id: a,
                        nodes: ns,
                        display_triangles: [a, b],
                        diagonal: [ns[0].min(ns[2]), ns[0].max(ns[2])],
                    });
                }
            }
            for (&old, &new) in &remap {
                if master.constrained_nodes.contains(&old) {
                    mesh.constrained_nodes.insert(new);
                }
            }
            for &[a, b] in &master.constrained_edges {
                if let (Some(&a), Some(&b)) = (remap.get(&a), remap.get(&b)) {
                    if a != b {
                        mesh.constrained_edges.insert(edge(a, b));
                        mesh.constrained_nodes.extend([a, b]);
                    }
                }
            }
        }
    }
    let plate_pins = mesh
        .nodes
        .iter()
        .enumerate()
        .map(|(i, n)| (n.xy, data.z[i]))
        .collect::<Vec<_>>();
    for (c, path) in paths {
        progress::check_cancel()?;
        let level = data
            .layout
            .levels
            .iter()
            .find(|l| l.id == data.layout.owner(c.id))
            .unwrap()
            .clone();
        let extent = data.layout.extent(c);
        let pins = contacts
            .iter()
            .filter(|(_, spans)| spans.iter().any(|h| overlaps(*h, extent)))
            .flat_map(|(e, _)| e.iter().copied())
            .chain(
                plate_pins
                    .iter()
                    .filter(|(_, z)| *z >= extent[0] - 1e-5 && *z <= extent[1] + 1e-5)
                    .map(|(p, _)| *p),
            )
            .chain(
                rigid_pins
                    .get(&c.id)
                    .into_iter()
                    .flatten()
                    .map(|p| [p[0], p[1]]),
            )
            .chain(
                contact_pins_at(extent[0])
                    .chain(
                        spatial_pins
                            .iter()
                            .filter(|p| (p[2] - extent[0]).abs() < 1e-5),
                    )
                    .filter(|_| c.kind == Kind::Beam)
                    .map(|p| [p[0], p[1]]),
            )
            .collect::<Vec<_>>();
        if c.kind == Kind::Support && extent[1] - extent[0] < 100. {
            return Err(format!("Стена {}: высота менее 100 мм", c.id));
        }
        let mut used = BTreeSet::new();
        for segment in path.windows(2) {
            let [a, b] = [segment[0], segment[1]];
            let d = [b[0] - a[0], b[1] - a[1]];
            let len = d[0].hypot(d[1]);
            if len < 1e-6 {
                continue;
            }
            let sampled = if c.kind == Kind::Support {
                vec![0., 1.]
            } else {
                beam_breakpoints(a, b, &pins)
            };
            for w in sampled.windows(2) {
                let xy = [w[0], w[1]].map(|t| {
                    if t == 0. {
                        a
                    } else if t == 1. {
                        b
                    } else {
                        [a[0] + d[0] * t, a[1] + d[1] * t]
                    }
                });
                let low = xy.map(|p| node(&mut mesh, &mut data, &mut weld, p, extent[0]));
                if low[0] == low[1] {
                    continue;
                }
                mesh.constrained_edges.insert(edge(low[0], low[1]));
                mesh.constrained_nodes.extend(low);
                used.extend(low);
                if c.kind == Kind::Support {
                    let top = xy.map(|p| node(&mut mesh, &mut data, &mut weld, p, extent[1]));
                    mesh.constrained_edges.insert(edge(top[0], top[1]));
                    mesh.constrained_nodes.extend(top);
                    used.extend(top);
                    data.walls.push(Wall {
                        id: c.id,
                        mesh_size_mm: c.mesh_size_mm,
                        bottom: low,
                        top,
                    });
                } else {
                    data.beams.push(Beam {
                        roll_angle_rad: c.column_angle,
                        id: c.id,
                        nodes: low,
                    });
                }
            }
        }
        if c.kind == Kind::Support {
            // Shared plate nodes remain part of the source wall's node group,
            // but must not be extruded into artificial full-height strip edges.
            used.extend(
                plate_pins
                    .iter()
                    .enumerate()
                    .filter(|(_, (p, z))| {
                        *z >= extent[0] - 1e-5
                            && *z <= extent[1] + 1e-5
                            && path
                                .windows(2)
                                .any(|e| meshing::on_segment(*p, [e[0], e[1]]))
                    })
                    .map(|(n, _)| n),
            );
        }
        if c.kind == Kind::Support {
            mesh.support_boundary
                .insert(c.id, used.into_iter().collect());
        }
    }
    mesh_free_beams(
        all,
        size,
        physical_contacts,
        &spatial_pins,
        &mut mesh,
        &mut data,
        &mut weld,
    )?;
    let mut edges = mesh.constrained_edges.clone();
    for e in &mesh.elements {
        for j in 0..3 {
            edges.insert(edge(e.nodes[j], e.nodes[(j + 1) % 3]));
        }
    }
    mesh.edges = edges.into_iter().collect();
    if mesh.nodes.len() > super::limits::MAX_NODES
        || mesh.elements.len() > super::limits::MAX_DISPLAY_TRIANGLES
    {
        return Err("Превышен лимит пространственной сетки здания".into());
    }
    mesh.building = Some(Box::new(data));
    let mut mesh = quad_mesh::process(mesh, options)?;
    super::mesh_pack::pack(&mut mesh)?;
    if let Some(data) = &mesh.building {
        data.validate_beam_edges(&mesh, |id| {
            all.iter().any(|c| c.id == id && c.kind == Kind::Column)
        })?;
    }
    if options.merge || options.backend == meshing::Backend::Gmsh {
        super::fem::precompute_wall_mesh(&mut mesh, size)?;
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        fem,
        loads::{ForceUnits, LoadCase, LoadData, LoadType, Magnitude},
    };
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
            id,
            kind,
            points,
            parent: None,
            curves: vec![],
            support_open: kind == Kind::Support,
            load: None,
            refinement: None,
            grid: None,
        }
    }

    #[test]
    fn beam_split_does_not_extend_past_shared_endpoint() {
        let joint = [3000., 0.];
        let pin = [3000.000022, 0.];
        assert!(meshing::on_segment(pin, [[0., 0.], joint]));
        assert_eq!(beam_breakpoints([0., 0.], joint, &[pin]), vec![0., 1.]);
        let next = beam_breakpoints(joint, [6000., 0.], &[pin]);
        assert_eq!(next.len(), 3);
        assert!((next[1] * 3000. - 0.000022).abs() < 1e-9);
        assert_eq!(
            beam_breakpoints([0., 0.], joint, &[[2999.999995, 0.]]),
            vec![0., 1.]
        );
    }
    #[test]
    fn free_beam_shares_exact_xyz_with_slab_and_column_without_projecting_axis() {
        let mut slab = contour(
            1,
            Kind::Slab,
            vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
        );
        slab.placement.slab.offset_mm = 1000.;
        let mut beam = contour(2, Kind::Beam, vec![[1000., 1100.], [3000., 1100.]]);
        beam.support_open = true;
        beam.placement.beam_z_mm = Some(vec![0., 2000.]);
        let column = contour(3, Kind::Column, vec![[2500., 1100.]]);
        let wall = contour(4, Kind::Support, vec![[2700., 400.], [2700., 2000.]]);
        let all = vec![slab, beam, column, wall];
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: all.iter().map(|c| (c.id, 1)).collect(),
            legacy_wall_extent: None,
        };
        let mesh = generate(&all, 500., layout, quad_mesh::Options::default()).unwrap();
        let mut settings = fem::shell::Settings::default();
        let section = fem::beam::Section {
            material: fem::element::Material::default(),
            width_mm: 200.,
        };
        settings.beams.insert(2, section.clone());
        settings.columns.insert(3, section);
        let scene = fem::shell::scene(
            &mesh,
            &vec![fem::element::Material::default(); mesh.elements.len()],
            &settings,
            500.,
            &all,
        )
        .unwrap();
        let at = |xyz| {
            scene
                .xyz
                .iter()
                .position(|&p| weld_coincident(p, xyz))
                .unwrap()
        };
        let slab_node = at([2000., 1100., 1000.]);
        let column_node = at([2500., 1100., 1500.]);
        let wall_node = at([2700., 1100., 1700.]);
        let beam_uses = |n| {
            scene
                .beams
                .iter()
                .any(|b| b.id == 2 && b.nodes.contains(&n))
        };
        assert!(beam_uses(slab_node));
        assert!(scene.surfaces.iter().any(|f| f.nodes.contains(&slab_node)));
        assert!(beam_uses(column_node));
        assert!(scene
            .beams
            .iter()
            .any(|b| b.id == 3 && b.nodes.contains(&column_node)));
        assert!(beam_uses(wall_node));
        assert!(scene
            .surfaces
            .iter()
            .any(|f| f.support == Some(4) && f.nodes.contains(&wall_node)));
        assert!(!mesh.constrained_edges.iter().any(|&e| e.iter().all(|&n| {
            (mesh.node_z(n) - 1000.).abs() < 1e-5 && (mesh.nodes[n].xy[1] - 1100.).abs() < 1e-5
        })));
    }

    #[test]
    fn free_beam_with_identical_plan_vertices_uses_its_spatial_length() {
        let mut beam = contour(1, Kind::Beam, vec![[0., 0.], [0., 0.]]);
        beam.support_open = true;
        beam.placement.beam_z_mm = Some(vec![1200., 0.]);
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1)]),
            legacy_wall_extent: None,
        };
        assert_eq!(layout.extent(&beam), [0., 1200.]);
        let mesh = generate(&[beam], 500., layout, quad_mesh::Options::default()).unwrap();
        let data = mesh.building.as_ref().unwrap();
        assert_eq!(data.beams.len(), 3);
        for beam in &data.beams {
            let z = beam.nodes.map(|n| mesh.node_z(n));
            assert!((z[0] - z[1] - 400.).abs() < 1e-6);
        }
    }
    fn fixture(
        count: u64,
    ) -> (
        Vec<Contour>,
        Layout,
        BTreeMap<u64, fem::element::Material>,
        fem::shell::Settings,
    ) {
        let mut all = vec![];
        let mut layout = Layout::default();
        let mut materials = BTreeMap::new();
        let mut settings = fem::shell::Settings::default();
        for i in 1..=count {
            let slab = i * 10;
            let support = slab + 1;
            let points = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
            all.push(contour(slab, Kind::Slab, points.clone()));
            let mut wall = contour(support, Kind::Support, points);
            wall.support_open = false;
            all.push(wall);
            layout.levels.push(Level {
                id: i,
                z: (i - 1) as f64 * 3000.,
                top: i as f64 * 3000.,
            });
            layout.owners.insert(slab, i);
            layout.owners.insert(support, i);
            let mut mat = fem::element::Material::default();
            mat.weight = Magnitude {
                value: 25.,
                units: ForceUnits::KiloNewton,
            };
            materials.insert(slab, mat);
            settings
                .materials
                .insert(support, fem::element::Material::wall_default());
        }
        (all, layout, materials, settings)
    }

    #[test]
    fn contour_rigid_links_are_meshed_and_exported_as_independent_stations() {
        use crate::plan_trace::{columns, lira_export};
        let section = fem::beam::Section {
            material: fem::element::Material::default(),
            width_mm: 300.,
        };
        for (master_kind, expected_slaves) in
            [(Kind::Column, 1), (Kind::Support, 1), (Kind::Beam, 2)]
        {
            let lower = contour(
                1,
                Kind::Slab,
                vec![[0., -1000.], [1600., -1000.], [1600., 0.], [0., 0.]],
            );
            let mut materials = BTreeMap::from([(1, fem::element::Material::default())]);
            let mut all = vec![lower];
            let mut settings = fem::shell::Settings::default();
            let trace = if master_kind != Kind::Beam {
                all.push(contour(2, Kind::Support, vec![[0., 0.], [1600., 0.]]));
                if master_kind == Kind::Column {
                    settings.columns.insert(3, section.clone());
                    all.push(contour(3, Kind::Column, vec![[800., 300.]]));
                } else {
                    all.push(contour(3, Kind::Support, vec![[0., 300.], [1600., 300.]]));
                    let mut intermediate = contour(
                        5,
                        Kind::Slab,
                        vec![[0., -500.], [1600., -500.], [1600., 600.], [0., 600.]],
                    );
                    intermediate.placement.slab.offset_mm = 499.;
                    materials.insert(5, fem::element::Material::default());
                    all.push(intermediate);
                }
                vec![[800., 300.], [800., 0.]]
            } else {
                settings.beams.insert(3, section.clone());
                let mut upper = contour(
                    2,
                    Kind::Slab,
                    vec![[0., 0.], [1600., 0.], [1600., 1000.], [0., 1000.]],
                );
                upper.placement.slab.offset_mm = 1000.;
                materials.insert(2, fem::element::Material::default());
                all.push(upper);
                let mut beam = contour(3, Kind::Beam, vec![[0., 0.], [1600., 0.]]);
                beam.support_open = true;
                beam.placement.slab.offset_mm = 500.;
                all.push(beam);
                vec![[100., 0.], [1500., 0.]]
            };
            all.push(contour(4, Kind::RigidLink, trace));
            let mut layout = Some(Layout {
                levels: vec![Level {
                    id: 1,
                    z: 0.,
                    top: 1500.,
                }],
                owners: all.iter().map(|c| (c.id, 1)).collect(),
                legacy_wall_extent: None,
            });
            let sources = columns::meshing_sources(&all, &settings, &mut layout);
            let options = quad_mesh::Options {
                merge: true,
                optimise: true,
                ..Default::default()
            };
            let mesh = generate(&sources, 500., layout.unwrap(), options).unwrap();
            let (scene, loads) =
                fem::lira_model(&mesh, &all, &materials, &[], 10., 500., &settings).unwrap();
            let bodies = columns::append_contour_rigid_links(vec![], &mesh, &scene).unwrap();
            assert!(bodies.len() >= 4);
            let mut used = BTreeSet::new();
            for body in &bodies {
                assert_eq!(body.nodes.len(), expected_slaves + 1);
                assert!(body.nodes.iter().all(|&n| used.insert(n)));
                for &node in &body.nodes {
                    let a = scene.xyz[body.master];
                    let b = scene.xyz[node];
                    if master_kind != Kind::Beam {
                        assert!((a[2] - b[2]).abs() < WELD_TOLERANCE_MM);
                    } else {
                        assert!((a[0] - b[0]).abs().max((a[1] - b[1]).abs()) < WELD_TOLERANCE_MM);
                    }
                }
            }
            let output =
                lira_export::encode_with_rigid(&scene, &loads, &[], 10., "АЖТ", &bodies).unwrap();
            assert_eq!(output.rigid_bodies, bodies.len());
            if master_kind == Kind::Support {
                let station = bodies
                    .iter()
                    .find(|body| (scene.xyz[body.master][2] - 499.).abs() < WELD_TOLERANCE_MM)
                    .unwrap();
                for &node in &station.nodes {
                    assert!(scene
                        .surfaces
                        .iter()
                        .any(|s| s.support.is_none() && s.nodes.contains(&node)));
                    assert!(scene
                        .surfaces
                        .iter()
                        .any(|s| s.support.is_some() && s.nodes.contains(&node)));
                }
                assert!(bodies
                    .iter()
                    .all(|body| (scene.xyz[body.master][2] - 500.).abs() > WELD_TOLERANCE_MM));
            }
            if master_kind == Kind::Column {
                let mut case = LoadCase::new(1);
                case.kind = LoadType::SelfWeight;
                settings.automatic_building_top = false;
                settings.building_bottom_mask = 0x3f;
                let result = fem::solve_with_support_height(
                    &mesh,
                    &all,
                    &materials,
                    &[case],
                    10.,
                    500.,
                    fem::ElementType::Dkt,
                    &settings,
                )
                .unwrap();
                assert!(result.cases[0].applied_kn > 0.);
                assert!(result.cases[0].balance_kn.abs() < 1e-6);
            }
        }
    }

    #[test]
    fn rigid_links_with_one_wall_node_join_both_columns_at_each_station() {
        use crate::plan_trace::columns;
        let mut all = vec![
            contour(1, Kind::Support, vec![[0., 0.], [1600., 0.]]),
            contour(2, Kind::Column, vec![[600., 300.]]),
            contour(3, Kind::Column, vec![[1000., 300.]]),
            contour(4, Kind::RigidLink, vec![[600., 300.], [800., 0.]]),
            contour(5, Kind::RigidLink, vec![[1000., 300.], [800., 0.]]),
        ];
        all[3].placement.rigid_link_targets = Some([2, 1]);
        all[4].placement.rigid_link_targets = Some([3, 1]);
        let mut settings = fem::shell::Settings::default();
        for id in [2, 3] {
            settings.columns.insert(
                id,
                fem::beam::Section {
                    material: fem::element::Material::default(),
                    width_mm: 300.,
                },
            );
        }
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 1500.,
            }],
            owners: all.iter().map(|c| (c.id, 1)).collect(),
            legacy_wall_extent: None,
        };
        let mesh = generate(&all, 500., layout, quad_mesh::Options::default()).unwrap();
        let (scene, _) =
            fem::lira_model(&mesh, &all, &BTreeMap::new(), &[], 10., 500., &settings).unwrap();
        let bodies = columns::append_contour_rigid_links(vec![], &mesh, &scene).unwrap();
        assert_eq!(bodies.len(), 4);
        for body in &bodies {
            assert_eq!(body.nodes.len(), 3);
            let points = body.nodes.iter().map(|&n| scene.xyz[n]).collect::<Vec<_>>();
            let master = scene.xyz[body.master];
            assert_eq!(master[1], 300.);
            assert!(points.iter().all(|p| p[2] == master[2]));
            assert!(points.iter().any(|p| p[0] == 800. && p[1] == 0.));
        }
    }

    #[test]
    fn intermediate_plate_planes_keep_their_own_holes_and_materials() {
        let square = vec![[0., 0.], [3000., 0.], [3000., 3000.], [0., 3000.]];
        let lower = contour(10, Kind::Slab, square.clone());
        let mut upper = contour(20, Kind::Slab, square);
        upper.placement.slab.offset_mm = 1500.;
        let mut hole = contour(
            90,
            Kind::Hole,
            vec![[500., 500.], [1500., 500.], [1500., 1500.], [500., 1500.]],
        );
        hole.parent = Some(10);
        let all = vec![lower, upper, hole.clone()];
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(10, 1), (20, 1), (90, 1)]),
            legacy_wall_extent: None,
        };
        let mut materials = BTreeMap::from([
            (10, fem::element::Material::default()),
            (20, fem::element::Material::default()),
        ]);
        materials.get_mut(&20).unwrap().thickness_mm = 300.;
        let mesh = generate(&all, 500., layout.clone(), quad_mesh::Options::default()).unwrap();
        let mut areas = [0.; 2];
        for e in &mesh.elements {
            let [a, b, c] = e.nodes.map(|n| mesh.nodes[n].xy);
            let area = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5;
            areas[usize::from(mesh.node_z(e.nodes[0]) > 1000.)] += area;
        }
        assert!((areas[0] - 8e6).abs() < 1e-4);
        assert!((areas[1] - 9e6).abs() < 1e-4);
        for &[a, b] in &mesh.constrained_edges {
            if mesh.node_z(a) < 1000. || mesh.node_z(b) < 1000. {
                continue;
            }
            assert!(
                !hole
                    .points
                    .iter()
                    .zip(hole.points.iter().cycle().skip(1))
                    .take(4)
                    .any(|(&p, &q)| meshing::on_segment(mesh.nodes[a].xy, [p, q])
                        && meshing::on_segment(mesh.nodes[b].xy, [p, q])),
                "hole boundary from lower plate leaked into upper plane"
            );
        }
        let cells = fem::prepare_cells(&mesh, &all, &materials, 500., 10.).unwrap();
        for (cell, e) in cells.iter().zip(&mesh.elements) {
            let wanted = if mesh.node_z(e.nodes[0]) > 1000. {
                materials[&20]
            } else {
                materials[&10]
            };
            assert_eq!(cell.material, wanted);
        }
        // A coincident patch must select one owner/material, not stack stiffness.
        let mut overlap = all;
        overlap[1].placement.slab.offset_mm = 0.;
        let mesh = generate(&overlap, 500., layout, quad_mesh::Options::default()).unwrap();
        let cells = fem::prepare_cells(&mesh, &overlap, &materials, 500., 10.).unwrap();
        for (i, cell) in cells.iter().enumerate() {
            let owner = mesh.building.as_ref().unwrap().element_slabs[i];
            assert_eq!(cell.material, materials[&owner]);
        }
    }
    #[test]
    fn overlapping_slabs_of_different_floors_are_rejected_on_the_same_physical_plane() {
        let mut lower = contour(
            10,
            Kind::Slab,
            vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
        );
        lower.placement.slab.offset_mm = 3000.;
        lower.mesh_size_mm = 700.;
        let mut upper = contour(
            20,
            Kind::Slab,
            vec![
                [2000., 1000.],
                [6000., 1000.],
                [6000., 3000.],
                [2000., 3000.],
            ],
        );
        upper.mesh_size_mm = 300.;
        let layout = Layout {
            levels: vec![
                Level {
                    id: 1,
                    z: 0.,
                    top: 3000.,
                },
                Level {
                    id: 2,
                    z: 3000.,
                    top: 6000.,
                },
            ],
            owners: BTreeMap::from([(10, 1), (20, 2), (30, 1)]),
            legacy_wall_extent: None,
        };
        let error = generate(
            &[lower.clone(), upper.clone()],
            500.,
            layout.clone(),
            Default::default(),
        )
        .unwrap_err();
        assert!(
            error.contains("Наложение плит 10 (этаж 1) и 20 (этаж 2) на Z=3000.000000"),
            "{error}"
        );
        assert!(error.contains("Площадь 4.000000 м²"), "{error}");

        let mut hole = contour(
            30,
            Kind::Hole,
            vec![
                [1000., 1000.],
                [3000., 1000.],
                [3000., 3000.],
                [1000., 3000.],
            ],
        );
        hole.parent = Some(lower.id);
        upper.points = vec![
            [1500., 1500.],
            [2500., 1500.],
            [2500., 2500.],
            [1500., 2500.],
        ];
        let mesh = generate(&[lower, upper, hole], 500., layout, Default::default()).unwrap();
        let owners = &mesh.building.as_ref().unwrap().element_slabs;
        assert!(owners.contains(&10) && owners.contains(&20));
    }

    #[test]
    fn capital_patch_replaces_base_plate_independent_of_input_order() {
        use super::super::fem::{self, element::Material};
        let base = contour(
            1,
            Kind::Slab,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        );
        let cap = contour(
            2,
            Kind::Slab,
            vec![[500., 500.], [1500., 500.], [1500., 1500.], [500., 1500.]],
        );
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1)]),
            ..Default::default()
        };
        let normal = Material::default();
        let thick = Material {
            thickness_mm: 400.,
            ..normal
        };
        let materials = BTreeMap::from([(1, normal), (2, thick)]);
        for all in [
            vec![base.clone(), cap.clone()],
            vec![cap.clone(), base.clone()],
        ] {
            let mesh = generate(&all, 500., layout.clone(), quad_mesh::Options::default()).unwrap();
            let cells = fem::prepare_cells(&mesh, &all, &materials, 500., 10.).unwrap();
            let mut area = [0.; 2];
            for (i, cell) in cells.iter().enumerate() {
                let owner = mesh.building.as_ref().unwrap().element_slabs[i];
                assert_eq!(cell.material, materials[&owner]);
                let p = mesh.elements[i].nodes.map(|n| mesh.nodes[n].xy);
                area[(owner - 1) as usize] += ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                    .abs()
                    * 0.5;
            }
            assert!((area[0] - 3e6).abs() < 1e-4);
            assert!((area[1] - 1e6).abs() < 1e-4);
        }
    }
    #[test]
    fn free_rectangular_wall_uses_a_regular_quad_grid() {
        let slab = contour(
            1,
            Kind::Slab,
            vec![
                [5000., 5000.],
                [6000., 5000.],
                [6000., 6000.],
                [5000., 6000.],
            ],
        );
        let wall = contour(2, Kind::Support, vec![[0., 0.], [2000., 0.]]);
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 2500.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1)]),
            legacy_wall_extent: None,
        };
        let options = quad_mesh::Options {
            merge: true,
            ..Default::default()
        };
        let mesh = generate(&[slab, wall], 500., layout, options).unwrap();
        let data = mesh.building.as_ref().unwrap();
        assert!(data.wall_mesh.as_ref().unwrap().is_empty());
        assert_eq!(data.wall_quads.len(), 20);
        for (_, points) in &data.wall_quads {
            assert!(points.iter().all(|p| p[1].abs() < 1e-8
                && (p[0] / 500. - (p[0] / 500.).round()).abs() < 1e-8
                && (p[2] / 500. - (p[2] / 500.).round()).abs() < 1e-8));
        }
    }
    #[test]
    fn wall_free_boundary_across_a_plate_notch_keeps_its_requested_step() {
        let slab = contour(
            1,
            Kind::Slab,
            vec![
                [0., 0.],
                [1000., 0.],
                [1000., 500.],
                [2000., 500.],
                [2000., 0.],
                [3000., 0.],
                [3000., 1000.],
                [0., 1000.],
            ],
        );
        let mut wall = contour(2, Kind::Support, vec![[0., 0.], [3000., 0.]]);
        wall.mesh_size_mm = 250.;
        let all = vec![slab, wall];
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1)]),
            legacy_wall_extent: None,
        };
        let mesh = generate(&all, 500., layout, quad_mesh::Options::default()).unwrap();
        let scene = crate::plan_trace::fem::shell::scene(
            &mesh,
            &vec![crate::plan_trace::fem::element::Material::default(); mesh.elements.len()],
            &crate::plan_trace::fem::shell::Settings::default(),
            500.,
            &all,
        )
        .unwrap();
        let mut xs = scene.support_nodes[&2]
            .iter()
            .filter_map(|&n| {
                let p = scene.xyz[n];
                (p[1].abs() < 1e-5 && p[2].abs() < 1e-5).then_some(p[0])
            })
            .collect::<Vec<_>>();
        xs.sort_by(f64::total_cmp);
        xs.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        assert!(
            xs.windows(2).all(|w| w[1] - w[0] <= 250. + 1e-5),
            "free boundary: {xs:?}"
        );
    }
    #[test]
    fn wall_spans_outside_slabs_are_meshed_and_keep_plate_contact_nodes() {
        let slab = contour(
            1,
            Kind::Slab,
            vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
        );
        let mut wall = contour(2, Kind::Support, vec![[0., 0.], [3000., 0.]]);
        wall.mesh_size_mm = 250.;
        let crossing = contour(3, Kind::Support, vec![[1750., 0.], [1750., 1000.]]);
        let all = vec![slab, wall, crossing];
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1), (3, 1)]),
            legacy_wall_extent: None,
        };
        let mesh = generate(&all, 500., layout, quad_mesh::Options::default()).unwrap();
        let data = mesh.building.as_ref().unwrap();
        assert!(data.walls.iter().any(|w| w.id == 2));
        let scene = crate::plan_trace::fem::shell::scene(
            &mesh,
            &vec![crate::plan_trace::fem::element::Material::default(); mesh.elements.len()],
            &crate::plan_trace::fem::shell::Settings::default(),
            500.,
            &all,
        )
        .unwrap();
        let mut boundary = scene.support_nodes[&2]
            .iter()
            .filter_map(|&n| {
                let p = scene.xyz[n];
                (p[1].abs() < 1e-5 && p[2].abs() < 1e-5).then_some(p[0])
            })
            .collect::<Vec<_>>();
        boundary.sort_by(f64::total_cmp);
        boundary.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        assert_eq!(boundary.first(), Some(&0.));
        assert_eq!(boundary.last(), Some(&3000.));
        assert!(boundary.windows(2).all(|w| w[1] - w[0] <= 250. + 1e-5));
        let contact = mesh
            .elements
            .iter()
            .flat_map(|e| e.nodes)
            .filter(|&n| mesh.nodes[n].xy[1].abs() < 1e-5)
            .collect::<BTreeSet<_>>();
        for n in contact {
            assert!(scene.support_nodes[&2].contains(&n));
        }
        let junction = data
            .walls
            .iter()
            .filter(|w| w.id == 3)
            .flat_map(|w| w.bottom)
            .find(|&n| mesh.nodes[n].xy == [1750., 0.])
            .unwrap();
        assert!(scene.support_nodes[&2].contains(&junction));
    }
    fn load(
        all: &mut Vec<Contour>,
        layout: &mut Layout,
        id: u64,
        floor: u64,
        kind: Kind,
        points: Vec<Point>,
        case: u64,
        value: f64,
    ) {
        let mut c = contour(id, kind, points);
        c.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: "test".into(),
            mesh_constraint: false,
            case_id: case,
            magnitude: Magnitude {
                value,
                units: ForceUnits::KiloNewton,
            },
            closed: kind == Kind::LoadArea,
        });
        all.push(c);
        layout.owners.insert(id, floor);
    }
    #[test]
    fn monolithic_building_shares_storey_joints_and_applies_loads_only_to_their_floor() {
        let (mut all, mut layout, materials, settings) = fixture(3);
        let square = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
        load(
            &mut all,
            &mut layout,
            100,
            1,
            Kind::LoadArea,
            square.clone(),
            1,
            2.,
        );
        load(&mut all, &mut layout, 101, 2, Kind::LoadArea, square, 1, 1.);
        load(
            &mut all,
            &mut layout,
            102,
            3,
            Kind::LoadPoint,
            vec![[1000., 1000.]],
            2,
            5.,
        );
        load(
            &mut all,
            &mut layout,
            103,
            3,
            Kind::LoadLine,
            vec![[0., 1000.], [2000., 1000.]],
            2,
            1.,
        );
        let mesh = generate(
            &all,
            1000.,
            layout,
            quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
        )
        .unwrap();
        let geom = fem::prepare_cells(&mesh, &all, &materials, 1000., 10.).unwrap();
        let scene = fem::shell::scene(
            &mesh,
            &geom.iter().map(|c| c.material).collect::<Vec<_>>(),
            &settings,
            1000.,
            &all,
        )
        .unwrap();
        let floor2 = mesh
            .building
            .as_ref()
            .unwrap()
            .node_floors
            .iter()
            .enumerate()
            .filter(|(_, f)| **f == 2)
            .map(|(i, _)| i)
            .collect::<BTreeSet<_>>();
        let shared = scene
            .surfaces
            .iter()
            .filter(|s| s.support == Some(11))
            .flat_map(|s| s.nodes.iter().copied())
            .filter(|n| floor2.contains(n))
            .collect::<BTreeSet<_>>();
        assert!(!shared.is_empty());
        assert!(scene
            .surfaces
            .iter()
            .filter(|s| s
                .display
                .first()
                .is_some_and(|&i| mesh.element_floor(i) == 2))
            .any(|s| s.nodes.iter().any(|n| shared.contains(n))));
        for &d in &scene.fixed {
            let z = scene.xyz[d / 3][2];
            assert!(z == 0. || z == 9000.);
            if z == 9000. {
                assert_ne!(d % 3, 2);
            }
        }
        let mut own = LoadCase::new(3);
        own.kind = LoadType::SelfWeight;
        let result = fem::solve_with_support_height(
            &mesh,
            &all,
            &materials,
            &[LoadCase::new(1), LoadCase::new(2), own],
            10.,
            1000.,
            fem::ElementType::Dkt,
            &settings,
        )
        .unwrap();
        for (case, expected) in result.cases.iter().zip([12., 7., 60.]) {
            assert!(
                (case.applied_kn - expected).abs() < 1e-7,
                "{} != {}",
                case.applied_kn,
                expected
            );
            assert!(case.balance_kn.abs() < 1e-7);
            assert!(case.relative_residual < 1e-6);
            assert!(case.balance_m_knm.iter().all(|v| v.abs() < 1e-6));
        }
        assert!(
            shared.iter().any(|&n| result.cases[1].w_mm[n] > 1e-8),
            "upper-floor load must compress lower walls"
        );
    }
    #[test]
    fn differing_holes_and_materials_remain_separate_between_plate_planes() {
        let (mut all, mut layout, mut materials, settings) = fixture(2);
        let mut hole = contour(
            90,
            Kind::Hole,
            vec![[500., 500.], [1500., 500.], [1500., 1500.], [500., 1500.]],
        );
        hole.parent = Some(10);
        all.push(hole);
        layout.owners.insert(90, 1);
        materials.get_mut(&20).unwrap().thickness_mm = 300.;
        let square = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
        load(
            &mut all,
            &mut layout,
            100,
            1,
            Kind::LoadArea,
            square.clone(),
            1,
            1.,
        );
        load(&mut all, &mut layout, 101, 2, Kind::LoadArea, square, 1, 2.);
        let mesh = generate(&all, 500., layout, quad_mesh::Options::default()).unwrap();
        let result = fem::solve_with_support_height(
            &mesh,
            &all,
            &materials,
            &[LoadCase::new(1)],
            10.,
            500.,
            fem::ElementType::Dkt,
            &settings,
        )
        .unwrap();
        assert!((result.cases[0].applied_kn - 11.).abs() < 1e-7);
        assert!(result.cases[0].balance_kn.abs() < 1e-6);
    }
    #[test]
    fn building_lira_export_places_loads_on_their_effective_elevations() {
        let (mut all, mut layout, materials, settings) = fixture(2);
        let square = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
        load(
            &mut all,
            &mut layout,
            100,
            1,
            Kind::LoadArea,
            square.clone(),
            1,
            2.,
        );
        load(&mut all, &mut layout, 101, 1, Kind::LoadArea, square, 1, 5.);
        all.last_mut().unwrap().placement.slab.offset_mm = 3000.;
        load(
            &mut all,
            &mut layout,
            102,
            1,
            Kind::LoadPoint,
            vec![[1000., 1000.]],
            2,
            7.,
        );
        all.last_mut().unwrap().placement.slab.anchor = model::LevelAnchor::Top;
        load(
            &mut all,
            &mut layout,
            103,
            1,
            Kind::LoadLine,
            vec![[500., 1000.], [1500., 1000.]],
            2,
            3.,
        );
        all.last_mut().unwrap().placement.slab.anchor = model::LevelAnchor::Top;
        let mesh = generate(
            &all,
            1000.,
            layout,
            quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: false,
            },
        )
        .unwrap();
        let cases = [LoadCase::new(1), LoadCase::new(2)];
        let (scene, loads) =
            fem::lira_model(&mesh, &all, &materials, &cases, 10., 1000., &settings).unwrap();
        for (case, exported) in cases.iter().zip(&loads) {
            let pressure = fem::source_area_projection(
                &mesh,
                &all,
                &materials,
                &[(case.clone(), 1.)],
                10.,
                1000.,
            )
            .unwrap();
            for (surface, q) in scene.surfaces.iter().zip(&exported.surface) {
                for &i in &surface.display {
                    assert!((pressure[i] - q).abs() < 1e-10);
                }
            }
        }
        assert!((loads[0].resultant(&scene)[0] + 28.).abs() < 1e-7);
        assert!((loads[1].resultant(&scene)[0] + 10.).abs() < 1e-7);
        for (surface, &q) in scene.surfaces.iter().zip(&loads[0].surface) {
            if let Some(&i) = surface.display.first() {
                assert_eq!(q, if mesh.element_floor(i) == 1 { 2. } else { 5. });
            }
        }
        for (i, &p) in loads[1].nodal.iter().enumerate() {
            if p.abs() > 1e-10 {
                assert!((scene.xyz[i][2] - 3000.).abs() < 1e-8);
            }
        }
        crate::plan_trace::lira_export::encode(&scene, &loads, &cases, 10., "two storeys").unwrap();
    }
}

#[cfg(test)]
mod placement_tests {
    use super::*;
    use crate::plan_trace::{
        fem,
        model::{LevelAnchor, LevelRef},
    };
    #[test]
    fn shifted_slab_and_wall_share_actual_nodes_and_restraints_use_actual_ends() {
        let mut slab = Contour {
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
            id: 1,
            parent: None,
            kind: Kind::Slab,
            points: vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
            curves: vec![],
        };
        slab.placement.slab = LevelRef {
            anchor: LevelAnchor::Bottom,
            offset_mm: 175.,
        };
        let mut wall = slab.clone();
        wall.id = 2;
        wall.kind = Kind::Support;
        wall.support_open = true;
        wall.points = vec![[0., 0.], [1000., 0.]];
        wall.placement.bottom.offset_mm = -100.;
        wall.placement.top.offset_mm = 200.;
        let all = vec![slab, wall];
        let layout = Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1)]),
            legacy_wall_extent: None,
        };
        let mesh = generate(
            &all,
            500.,
            layout,
            quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: false,
            },
        )
        .unwrap();
        for (i, _) in mesh.elements.iter().enumerate() {
            for &n in &mesh.elements[i].nodes {
                assert!((mesh.node_z(n) - 175.).abs() < 1e-8);
            }
        }
        let materials = BTreeMap::from([(1, fem::element::Material::default())]);
        let settings = fem::shell::Settings::default();
        let (scene, _) =
            fem::lira_model(&mesh, &all, &materials, &[], 10., 500., &settings).unwrap();
        for (&n, p) in scene.support_nodes[&2].iter().map(|n| (n, &scene.xyz[*n])) {
            if (p[2] + 100.).abs() < 1e-5 {
                assert!(scene.fixed.contains(&(3 * n + 2)));
            }
            if p[2].abs() < 1e-5 {
                assert!(!scene.fixed.contains(&(3 * n + 2)));
            }
        }
        for (n, p) in scene
            .xyz
            .iter()
            .enumerate()
            .filter(|(_, p)| p[1].abs() < 1e-8 && (p[2] - 175.).abs() < 1e-8)
        {
            assert!(scene.support_nodes[&2].contains(&n));
            assert!(!scene.fixed.contains(&(3 * n + 2)));
        }
    }
}
