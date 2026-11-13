//! Conforming planar FE model; IDs do not depend on renderer buffers or Spade handles.
use super::{
    geometry::Point,
    model::{self, Contour, Kind},
};
use mini_fem::geo::{
    Area, BooleanOps, BoundingRect, Contains, Distance, Euclidean, Intersects, LineString,
    MultiPolygon, Polygon,
};
use spade::{
    AngleLimit, ConstrainedDelaunayTriangulation, Point2, RefinementParameters, Triangulation,
};
use std::collections::{BTreeMap, BTreeSet};

pub use super::mesh_backend::Backend;

const MAX_NODES: usize = super::limits::MAX_NODES;
const NODE_TOL: f64 = 1e-6; // mm: numerical coincidence only

/// Use the very same representative at shared fragment endpoints BEFORE
/// sampling straight constraints. Welding only the later CDT insertions leaves
/// distinct, microscopically sloping overlapping chains behind.
fn weld_constraint_endpoints(lines: &mut [[Point; 2]]) {
    let mut buckets = BTreeMap::<(i64, i64), Vec<Point>>::new();
    for p in lines.iter_mut().flatten() {
        let key = (
            (p[0] / NODE_TOL).floor() as i64,
            (p[1] / NODE_TOL).floor() as i64,
        );
        let mut found = None;
        'search: for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(points) = buckets.get(&(key.0 + dx, key.1 + dy)) {
                    for &q in points {
                        if (p[0] - q[0]).hypot(p[1] - q[1]) <= NODE_TOL {
                            found = Some(q);
                            break 'search;
                        }
                    }
                }
            }
        }
        if let Some(q) = found {
            *p = q;
        } else {
            buckets.entry(key).or_default().push(*p);
        }
    }
}

/// A CDT can contain faces between numerically collinear constraint fragments.
/// Reject only faces thinner than the same numerical tolerance used to weld nodes.
/// Check in final world coordinates: adding the region origin can lose low bits.
fn numerical_sliver(p: [Point; 3]) -> bool {
    let u = [p[1][0] - p[0][0], p[1][1] - p[0][1]];
    let v = [p[2][0] - p[0][0], p[2][1] - p[0][1]];
    let twice_area = (u[0] * v[1] - u[1] * v[0]).abs();
    let longest = u[0]
        .hypot(u[1])
        .max(v[0].hypot(v[1]))
        .max((p[2][0] - p[1][0]).hypot(p[2][1] - p[1][1]));
    twice_area <= (longest * NODE_TOL).max(1e-8)
}

/// Node the complete constraint arrangement before spacing any of its fragments.
/// This also handles T junctions and overlapping/collinear boundaries.
pub(crate) fn split_constraints(lines: Vec<[Point; 2]>) -> Result<Vec<[Point; 2]>, String> {
    use mini_fem::geo::{
        line_intersection::{line_intersection, LineIntersection},
        Line,
    };
    let mut lines = lines
        .into_iter()
        .filter(|[a, b]| (a[0] - b[0]).hypot(a[1] - b[1]) > NODE_TOL)
        .collect::<Vec<_>>();
    weld_constraint_endpoints(&mut lines);
    lines.sort_by(|a, b| a[0][0].min(a[1][0]).total_cmp(&b[0][0].min(b[1][0])));
    let mut points = lines.iter().map(|s| s.to_vec()).collect::<Vec<_>>();
    let mut intersections = 0;
    for i in 0..lines.len() {
        let [a, b] = lines[i];
        let max_x = a[0].max(b[0]) + NODE_TOL;
        for j in i + 1..lines.len() {
            let [c, d] = lines[j];
            if c[0].min(d[0]) > max_x {
                break;
            }
            if a[1].min(b[1]) > c[1].max(d[1]) + NODE_TOL
                || c[1].min(d[1]) > a[1].max(b[1]) + NODE_TOL
            {
                continue;
            }
            let first = Line::new((a[0], a[1]), (b[0], b[1]));
            let second = Line::new((c[0], c[1]), (d[0], d[1]));
            let mut shared = Vec::new();
            match line_intersection(first, second) {
                Some(LineIntersection::SinglePoint {
                    intersection: p, ..
                }) => shared.push([p.x, p.y]),
                Some(LineIntersection::Collinear { intersection: l }) => {
                    shared.push([l.start.x, l.start.y]);
                    shared.push([l.end.x, l.end.y]);
                }
                None => {}
            }
            // Treat near-coincident endpoints consistently with the vertex welder.
            for (p, s) in [(a, [c, d]), (b, [c, d]), (c, [a, b]), (d, [a, b])] {
                let v = [s[1][0] - s[0][0], s[1][1] - s[0][1]];
                let length = v[0].hypot(v[1]);
                let t = ((p[0] - s[0][0]) * v[0] + (p[1] - s[0][1]) * v[1]) / (length * length);
                if (-NODE_TOL / length..=1. + NODE_TOL / length).contains(&t)
                    && ((p[0] - s[0][0]) * v[1] - (p[1] - s[0][1]) * v[0]).abs() / length
                        <= NODE_TOL
                {
                    shared.push(p);
                }
            }
            intersections += shared.len();
            if intersections > MAX_NODES * 8 {
                return Err("Слишком много пересечений объектов".into());
            }
            points[i].extend(shared.iter().copied());
            points[j].extend(shared);
        }
    }
    let mut result = Vec::new();
    for ([a, b], mut points) in lines.into_iter().zip(points) {
        let d = [b[0] - a[0], b[1] - a[1]];
        let along = |p: &Point| (p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1];
        points.sort_by(|p, q| along(p).total_cmp(&along(q)));
        points.dedup_by(|p, q| (p[0] - q[0]).hypot(p[1] - q[1]) <= NODE_TOL);
        result.extend(points.windows(2).map(|s| [s[0], s[1]]));
        if result.len() > MAX_NODES * 4 {
            return Err("Слишком много фрагментов объектов".into());
        }
    }
    weld_constraint_endpoints(&mut result);
    result.retain(|[a, b]| (a[0] - b[0]).hypot(a[1] - b[1]) > NODE_TOL);
    // Overlapping source lines have now been noded into the same atomic edges.
    // Keep one unoriented copy before spacing them; repeated/reversed copies
    // must not seed independent chains in either the plate or wall CDT.
    for e in &mut result {
        if e[0][0]
            .total_cmp(&e[1][0])
            .then(e[0][1].total_cmp(&e[1][1]))
            .is_gt()
        {
            e.swap(0, 1);
        }
    }
    result.sort_by(|a, b| {
        a.iter()
            .flatten()
            .zip(b.iter().flatten())
            .map(|(a, b)| a.total_cmp(b))
            .find(|o| !o.is_eq())
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    result.dedup();
    Ok(result)
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Node {
    pub id: u64,
    pub xy: Point,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Element {
    pub id: u64,
    pub nodes: [usize; 3],
    pub supports: Vec<u64>,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Quad {
    pub id: u64,
    pub nodes: [usize; 4],
    /// Rendering/search tessellation IDs, not constituent stiffness elements.
    pub display_triangles: [u64; 2],
    pub diagonal: [usize; 2],
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Mesh {
    #[serde(default)]
    pub building: Option<Box<super::building_mesh::Data>>,
    pub nodes: Vec<Node>,
    pub quads: Vec<Quad>,
    pub constrained_nodes: BTreeSet<usize>,
    pub constrained_edges: BTreeSet<[usize; 2]>,
    /// Triangular display cells. Only cells not owned by a Quad are triangular FEs.
    pub elements: Vec<Element>,
    pub edges: Vec<[usize; 2]>,
    /// Geometric node groups, not yet applied displacement boundary conditions.
    pub support_boundary: BTreeMap<u64, Vec<usize>>,
}
impl Mesh {
    pub fn element_floor(&self, i: usize) -> u64 {
        self.building.as_ref().map_or(0, |b| b.element_floors[i])
    }
    pub fn contour_floor(&self, id: u64) -> u64 {
        self.building.as_ref().map_or(0, |b| b.layout.owner(id))
    }
    /// Loads on intermediate landings must not also act on other plates of their storey.
    pub fn load_matches_element(&self, load: &Contour, i: usize) -> bool {
        self.building.as_ref().is_none_or(|b| {
            self.elements[i].nodes.iter().all(|&n| {
                b.layout
                    .matches_source_plane(load, &[], self.element_floor(i), b.z[n], 0.05)
            })
        })
    }
    pub fn node_z(&self, n: usize) -> f64 {
        self.building.as_ref().map_or(0., |b| b.z[n])
    }

    pub fn quad_for_display(&self, index: usize) -> Option<&Quad> {
        let id = self.elements.get(index)?.id;
        self.quads
            .iter()
            .find(|q| q.display_triangles.contains(&id))
    }
    pub fn visible_edges(&self) -> Vec<[usize; 2]> {
        let hidden = self
            .quads
            .iter()
            .map(|q| q.diagonal)
            .collect::<BTreeSet<_>>();
        self.edges
            .iter()
            .copied()
            .filter(|e| !hidden.contains(e))
            .collect()
    }
}
fn polygon(p: &[Point]) -> Polygon<f64> {
    Polygon::new(
        LineString::from(p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
        vec![],
    )
}
fn rings(p: &Polygon<f64>) -> impl Iterator<Item = &LineString<f64>> {
    std::iter::once(p.exterior()).chain(p.interiors())
}
fn segments(p: &Polygon<f64>) -> Vec<[Point; 2]> {
    rings(p)
        .flat_map(|r| {
            r.lines()
                .map(|l| [[l.start.x, l.start.y], [l.end.x, l.end.y]])
        })
        .collect()
}
pub(super) fn on_segment(p: Point, s: [Point; 2]) -> bool {
    let d = [s[1][0] - s[0][0], s[1][1] - s[0][1]];
    let len = d[0].hypot(d[1]);
    if len < 1e-9 {
        return false;
    }
    let t = ((p[0] - s[0][0]) * d[0] + (p[1] - s[0][1]) * d[1]) / (len * len);
    (-1e-8..=1. + 1e-8).contains(&t)
        && ((p[0] - s[0][0]) * d[1] - (p[1] - s[0][1]) * d[0]).abs() / len < 1e-5
}
pub fn generate(all: &[Contour], size: f64) -> Result<Mesh, String> {
    generate_backend(all, size, Backend::Spade)
}
pub fn generate_backend(all: &[Contour], size: f64, backend: Backend) -> Result<Mesh, String> {
    generate_with_contacts_backend(all, size, &[], backend)
}
pub(crate) fn generate_with_contacts(
    all: &[Contour],
    size: f64,
    contacts: &[[Point; 2]],
) -> Result<Mesh, String> {
    generate_with_contacts_backend(all, size, contacts, Backend::Spade)
}
pub(crate) fn generate_with_contacts_backend(
    all: &[Contour],
    size: f64,
    contacts: &[[Point; 2]],
    backend: Backend,
) -> Result<Mesh, String> {
    generate_with_contacts_and_pins(all, size, contacts, &[], backend)
}
pub(super) fn generate_with_contacts_and_pins(
    all: &[Contour],
    size: f64,
    contacts: &[[Point; 2]],
    pins: &[Point],
    backend: Backend,
) -> Result<Mesh, String> {
    if !size.is_finite() || size < 100. {
        return Err("Размер КЭ должен быть не менее 100 мм".into());
    }
    let mut load_points = pins.to_vec();
    let mut load_pins = BTreeSet::new();
    for point in model::mesh_sources(all)
        .filter(|c| matches!(c.kind, Kind::LoadPoint | Kind::PointRestraint))
    {
        model::validate_candidate(all, point)?;
        load_points.push(point.points[0]);
    }
    let prepared = model::mesh_sources(all)
        .filter(|c| !matches!(c.kind, Kind::LoadPoint | Kind::PointRestraint))
        .map(|c| {
            let mut c = c.clone();
            c.rebuild()?;
            if c.kind.is_load() || super::fem::cut_forces::is_cut(&c) {
                let closed = c.closed();
                c.kind = Kind::Triangulation;
                c.support_open = !closed;
                c.parent = None;
                c.load = None;
            }
            Ok(c)
        })
        .collect::<Result<Vec<_>, String>>()?;
    for c in &prepared {
        model::validate_candidate(&prepared, c)?;
    }
    let prepared = prepared
        .into_iter()
        .map(|mut c| {
            let step = if c.kind == Kind::MeshRegion {
                c.refinement.as_ref().map_or(size, |r| r.size)
            } else {
                c.mesh_step(size)
            };
            c.points = c.mesh_boundary(step)?;
            Ok(c)
        })
        .collect::<Result<Vec<_>, String>>()?;
    let all = prepared.as_slice();
    for c in all {
        model::validate_candidate(all, c).map_err(|e| {
            format!("Разбиение границы МКЭ: {e}. Уменьшите размер КЭ для сохранения геометрии")
        })?;
    }
    let mut domain = MultiPolygon::<f64>(vec![]);
    let mut slab_materials = Vec::new();
    for slab in all.iter().filter(|c| c.kind == Kind::Slab) {
        let mut part = MultiPolygon(vec![polygon(&slab.points)]);
        for h in all
            .iter()
            .filter(|c| c.active_hole() && c.parent == Some(slab.id))
        {
            part = part.difference(&polygon(&h.points));
        }
        domain = domain.union(&part);
        slab_materials.push(part);
    }
    if domain.unsigned_area() < 1e-6 {
        return Err("Постройте плиту".into());
    }
    // Clip every sizing region to its own slab material. In overlaps the smaller
    // requested size wins; inside a region its size replaces the global setting.
    let mut sizing = Vec::<(MultiPolygon<f64>, f64)>::new();
    for slab in all
        .iter()
        .filter(|c| c.kind == Kind::Slab && c.mesh_size_mm > 0.)
    {
        let mut material = MultiPolygon(vec![polygon(&slab.points)]);
        for hole in all
            .iter()
            .filter(|c| c.active_hole() && c.parent == Some(slab.id))
        {
            material = material.difference(&polygon(&hole.points));
        }
        // An explicit local region replaces its slab's base size.
        for zone in all
            .iter()
            .filter(|c| c.kind == Kind::MeshRegion && c.parent == Some(slab.id))
        {
            material = material.difference(&polygon(&zone.points));
        }
        if material.unsigned_area() > 1e-6 {
            sizing.push((material, slab.mesh_size_mm));
        }
    }
    for area in all.iter().filter(|c| c.kind == Kind::MeshRegion) {
        let parent = all
            .iter()
            .find(|c| Some(c.id) == area.parent && c.kind == Kind::Slab)
            .ok_or("У области КЭ отсутствует плита")?;
        let mut material = MultiPolygon(vec![polygon(&parent.points)]);
        for hole in all
            .iter()
            .filter(|c| c.active_hole() && c.parent == Some(parent.id))
        {
            material = material.difference(&polygon(&hole.points));
        }
        let clipped = material.intersection(&polygon(&area.points));
        if clipped.unsigned_area() > 1e-6 {
            sizing.push((clipped, area.refinement.as_ref().unwrap().size));
        }
    }
    let grids = all
        .iter()
        .filter(|c| c.kind == Kind::Triangulation)
        .filter_map(|c| {
            c.grid.clone().or_else(|| {
                if c.curves.is_empty() {
                    model::GridSpec::infer(&c.points)
                } else {
                    None
                }
            })
        })
        .collect::<Vec<_>>();
    let grid_area_at = |point: Point| {
        grids
            .iter()
            .filter(|grid| grid.contains(point))
            .map(|grid| grid.cell_area() * 0.5)
            .min_by(f64::total_cmp)
    };
    sizing.sort_by(|a, b| a.1.total_cmp(&b.1));
    let sizing_bounds = sizing
        .iter()
        .map(|(zone, _)| zone.bounding_rect())
        .collect::<Vec<_>>();
    let min_size = sizing.iter().fold(size, |h, (_, step)| h.min(*step));
    let max_size = sizing.iter().fold(size, |h, (_, step)| h.max(*step));
    let mut remaining = domain.clone();
    let mut estimate = 0.;
    for grid in &grids {
        let area = polygon(&[
            grid.at(0., 0.),
            grid.at(grid.nx as f64, 0.),
            grid.at(grid.nx as f64, grid.ny as f64),
            grid.at(0., grid.ny as f64),
        ]);
        estimate += remaining.intersection(&area).unsigned_area() / grid.cell_area();
        remaining = remaining.difference(&area);
    }
    for (area, step) in &sizing {
        estimate += remaining.intersection(area).unsigned_area() / (step * step);
        remaining = remaining.difference(area);
    }
    estimate += remaining.unsigned_area() / (size * size);
    if estimate > MAX_NODES as f64 / 4. {
        return Err("Слишком мелкая сетка: увеличьте размер КЭ".into());
    }
    let supports = all
        .iter()
        .filter(|c| c.kind == Kind::Support)
        .map(|c| (c.id, c))
        .collect::<Vec<_>>();
    let mut mesh = Mesh {
        building: None,
        quads: vec![],
        constrained_nodes: Default::default(),
        constrained_edges: Default::default(),
        nodes: vec![],
        elements: vec![],
        edges: vec![],
        support_boundary: supports.iter().map(|(id, _)| (*id, vec![])).collect(),
    };
    for region in domain.0 {
        let origin = region.exterior().0.first().map_or([0., 0.], |p| [p.x, p.y]);
        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
        let mut constraints = segments(&region);
        constraints.extend_from_slice(contacts);
        // Retain interior slab boundaries for piecewise plate thickness/material.
        for slab in &slab_materials {
            for clipped in slab.intersection(&region).0 {
                constraints.extend(segments(&clipped));
            }
        }
        let mut support_segments = BTreeMap::new();
        for (id, c) in &supports {
            // Only the original support perimeter is a future Z boundary, not clipping edges at a hole.
            let mut points = c.points.clone();
            if c.closed() {
                points.push(points[0]);
            }
            support_segments.insert(
                *id,
                points.windows(2).map(|p| [p[0], p[1]]).collect::<Vec<_>>(),
            );
            // Keep original coordinates. Boolean line clipping quantizes oblique
            // supports and can move nodes off the actual restraint line.
            constraints.extend(points.windows(2).map(|p| [p[0], p[1]]));
        }
        for beam in all
            .iter()
            .filter(|c| matches!(c.kind, Kind::Beam | Kind::Triangulation | Kind::Restraint))
        {
            let points = beam.mesh_boundary(size)?;
            constraints.extend(points.windows(2).map(|p| [p[0], p[1]]));
            if beam.closed() && points.len() > 1 {
                constraints.push([*points.last().unwrap(), points[0]]);
            }
        }
        for (area, _) in &sizing {
            for clipped in area.intersection(&region).0 {
                constraints.extend(segments(&clipped));
            }
        }
        let boundary = segments(&region);
        let constraints = split_constraints(constraints)?
            .into_iter()
            .filter(|[a, b]| {
                let p = [a[0] + (b[0] - a[0]) * 0.5, a[1] + (b[1] - a[1]) * 0.5];
                region.intersects(&mini_fem::geo::Point::new(p[0], p[1]))
                    || boundary.iter().any(|s| on_segment(p, *s))
            });
        // Slab/support boundaries can overlap and be sampled independently.
        // Weld sub-micrometre numerical duplicates BEFORE adding constraints;
        // otherwise e.g. 2800 and 2800.0000000000005 become a zero-area FE.
        // Coordinates remain full precision: do not round the actual geometry.
        let mut buckets = BTreeMap::<(i64, i64), Vec<spade::handles::FixedVertexHandle>>::new();
        let mut insert_node = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
                               p: Point| {
            const TOL: f64 = NODE_TOL;
            let p = Point2::new(p[0] - origin[0], p[1] - origin[1]);
            let key = ((p.x / TOL).floor() as i64, (p.y / TOL).floor() as i64);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    if let Some(handles) = buckets.get(&(key.0 + dx, key.1 + dy)) {
                        for &handle in handles {
                            let q = cdt.vertex(handle).position();
                            if (q.x - p.x).hypot(q.y - p.y) <= TOL {
                                return Ok(handle);
                            }
                        }
                    }
                }
            }
            let handle = cdt.insert(p).map_err(|e| format!("Узел: {e:?}"))?;
            buckets.entry(key).or_default().push(handle);
            Ok::<_, String>(handle)
        };
        let mut constraint_edges = Vec::new();
        for [a, b] in constraints {
            let midpoint =
                mini_fem::geo::Point::new(a[0] + (b[0] - a[0]) * 0.5, a[1] + (b[1] - a[1]) * 0.5);
            let requested = sizing
                .iter()
                .zip(&sizing_bounds)
                .filter(|((zone, _), bounds)| {
                    // Polygon boolean operations snap their output to a numeric grid.
                    // A boundary midpoint can sit microscopically outside the original
                    // sizing zone even when it belongs to that zone's physical edge.
                    const EPS: f64 = 1e-3; // mm, sizing lookup only; never moves nodes
                    bounds.is_some_and(|r| {
                        midpoint.x() >= r.min().x - EPS
                            && midpoint.x() <= r.max().x + EPS
                            && midpoint.y() >= r.min().y - EPS
                            && midpoint.y() <= r.max().y + EPS
                            && (zone.intersects(&midpoint)
                                || zone.0.iter().any(|p| {
                                    rings(p).any(|r| Euclidean.distance(&midpoint, r) <= EPS)
                                }))
                    })
                })
                .map(|((_, h), _)| *h)
                .chain(
                    supports
                        .iter()
                        .filter(|(_, c)| c.mesh_size_mm > 0.)
                        .filter(|(_, c)| {
                            let mut p = c.points.clone();
                            if c.closed() {
                                p.push(p[0]);
                            }
                            p.windows(2)
                                .any(|e| on_segment([midpoint.x(), midpoint.y()], [e[0], e[1]]))
                        })
                        .map(|(_, c)| c.mesh_size_mm),
                )
                .min_by(f64::total_cmp);
            let step = requested.unwrap_or(size);
            // Grid fragments already terminate at grid intersections. Do not seed
            // extra edge nodes from the unrelated global FE spacing.
            let fixed_contact = contacts
                .iter()
                .any(|&e| on_segment(a, e) && on_segment(b, e));
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let count = if (fixed_contact && length <= step * (1. + 1e-8))
                || grid_area_at([midpoint.x(), midpoint.y()]).is_some()
            {
                1
            } else {
                // A geometric intersection may be a whole multi-metre line,
                // unlike a previously sampled wall-contact fragment. Keeping
                // it as one edge creates thin fans alongside a plate boundary.
                let divisions = length / step;
                (if fixed_contact || requested.is_some() {
                    (divisions - 1e-8).ceil()
                } else {
                    divisions.round()
                } as usize)
                    .max(1)
            };
            if count > MAX_NODES || mesh.nodes.len() + cdt.num_vertices() + count + 1 > MAX_NODES {
                return Err("Превышен лимит исходных узлов".into());
            }
            let mut from = insert_node(&mut cdt, a)?;
            for i in 1..=count {
                let t = i as f64 / count as f64;
                let p = if i == count {
                    b
                } else {
                    [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
                };
                let to = insert_node(&mut cdt, p)?;
                if from != to {
                    constraint_edges.push((from, to));
                }
                from = to;
            }
        }
        let mut load_point_handles = vec![];
        for &point in &load_points {
            if region.intersects(&mini_fem::geo::Point::new(point[0], point[1])) {
                load_point_handles.push(insert_node(&mut cdt, point)?);
            }
        }
        for (from, to) in constraint_edges {
            cdt.add_constraint_and_split(from, to, |p| p);
        }
        // Even an almost straight sampled spline may lie microscopically inside
        // the convex hull. Protect its exterior hull chords as well, otherwise
        // encroachment refinement can repeatedly split those irrelevant slivers.
        let hull = cdt
            .convex_hull()
            .map(|edge| (edge.from().fix(), edge.to().fix()))
            .collect::<Vec<_>>();
        for (from, to) in hull {
            cdt.add_constraint(from, to);
        }
        // Parity-based exclude_outer_faces would incorrectly remove support interiors.
        // Refine the CDT, then classify by the actual slab-minus-holes domain.
        let max_area = grids
            .iter()
            .fold(3_f64.sqrt() * 0.25 * max_size * max_size, |area, grid| {
                area.max(grid.cell_area() * 0.5 * (1. + 1e-8))
            });
        // Elongated prescribed grid cells are intentional; angle-only refinement
        // must not destroy them. Non-grid scenes retain the existing quality rule.
        let angle_limit = if grids
            .iter()
            .any(|g| g.dx().max(g.dy()) > 2. * g.dx().min(g.dy()))
        {
            0.
        } else {
            25.
        };
        let raw = if backend == Backend::Gmsh {
            super::mesh_backend::gmsh_refined(
                &mut cdt,
                |p| {
                    let world = mini_fem::geo::Point::new(p[0] + origin[0], p[1] + origin[1]);
                    sizing
                        .iter()
                        .filter(|(zone, _)| zone.intersects(&world))
                        .map(|(_, h)| *h)
                        .min_by(f64::total_cmp)
                        .unwrap_or(size)
                },
                max_size,
                |p| {
                    let world = mini_fem::geo::Point::new(p[0] + origin[0], p[1] + origin[1]);
                    if !region.contains(&world) {
                        return None;
                    }
                    let step = sizing
                        .iter()
                        .find(|(zone, _)| zone.contains(&world))
                        .map_or(size, |(_, h)| *h);
                    Some(
                        grid_area_at([world.x(), world.y()])
                            .unwrap_or(3_f64.sqrt() * 0.25 * step * step),
                    )
                },
            )?
        } else {
            for pass in 0..16 {
                let result = cdt.refine(
                    RefinementParameters::new()
                        .keep_constraint_edges()
                        .with_angle_limit(AngleLimit::from_deg(angle_limit))
                        // Nearby mandatory contours/supports can create short edges.
                        // Do not chase the angle target down to 1% of h² around them:
                        // circumcentre insertion then creates a cascade of tiny faces.
                        // This only stops optional quality refinement; existing short
                        // geometric fragments remain, and oversized faces are checked below.
                        .with_min_required_area(min_size * min_size * 0.10)
                        .with_max_allowed_area(max_area)
                        .with_max_additional_vertices(
                            MAX_NODES.saturating_sub(mesh.nodes.len() + cdt.num_vertices()),
                        ),
                );
                if !result.refinement_complete {
                    return Err("Генератор достиг лимита уточнения. Увеличьте размер КЭ или проверьте близкие границы".into());
                }
                // Do not rely solely on the library completion flag: verify the actual
                // accepted-domain element areas, including disconnected simple rectangles.
                let oversized = cdt
                    .inner_faces()
                    .filter_map(|f| {
                        let p = f.vertices().map(|v| v.position());
                        let area = ((p[1].x - p[0].x) * (p[2].y - p[0].y)
                            - (p[1].y - p[0].y) * (p[2].x - p[0].x))
                            .abs()
                            * 0.5;
                        let center = Point2::new(
                            (p[0].x + p[1].x + p[2].x) / 3.,
                            (p[0].y + p[1].y + p[2].y) / 3.,
                        );
                        let world =
                            mini_fem::geo::Point::new(center.x + origin[0], center.y + origin[1]);
                        let step = sizing
                            .iter()
                            .find(|(zone, _)| zone.contains(&world))
                            .map_or(size, |(_, h)| *h);
                        let target_area = grid_area_at([world.x(), world.y()])
                            .unwrap_or(3_f64.sqrt() * 0.25 * step * step);
                        (area > target_area * 1.000001 && region.contains(&world)).then_some(center)
                    })
                    .collect::<Vec<_>>();
                if oversized.is_empty() {
                    break;
                }
                if pass == 15 || mesh.nodes.len() + cdt.num_vertices() + oversized.len() > MAX_NODES
                {
                    return Err(
                        "Не достигнут заданный размер КЭ. Построение неполной сетки отменено"
                            .into(),
                    );
                }
                for p in oversized {
                    cdt.insert(p)
                        .map_err(|e| format!("Уточнение элемента: {e:?}"))?;
                }
            }
            super::mesh_backend::spade(&cdt)
        };
        let mut ids = BTreeMap::new();
        let mut faces = BTreeMap::new();
        for vs in &raw.triangles {
            let xy = vs.map(|v| {
                let p = raw.points[&v];
                [p[0] + origin[0], p[1] + origin[1]]
            });
            // Do this before registering nodes/IDs, so no unused DOFs are created.
            if numerical_sliver(xy) {
                continue;
            }
            let center = mini_fem::geo::Point::new(
                xy.iter().map(|p| p[0]).sum::<f64>() / 3.,
                xy.iter().map(|p| p[1]).sum::<f64>() / 3.,
            );
            // A Boolean union may quantize a boundary differently from each
            // individual plate-minus-holes. Every physical FE must also have
            // a material owner, using the same polygons as the solver.
            if !region.contains(&center)
                || !slab_materials.iter().any(|area| area.contains(&center))
            {
                continue;
            }
            let mut nodes = [0; 3];
            for i in 0..3 {
                nodes[i] = *ids.entry(vs[i]).or_insert_with(|| {
                    let n = mesh.nodes.len();
                    mesh.nodes.push(Node {
                        id: n as u64 + 1,
                        xy: xy[i],
                    });
                    n
                });
            }
            let ids = supports
                .iter()
                .filter(|(_, c)| c.closed() && polygon(&c.points).contains(&center))
                .map(|(id, _)| *id)
                .collect();
            mesh.elements.push(Element {
                id: mesh.elements.len() as u64 + 1,
                nodes,
                supports: ids,
            });
            let mut key = *vs;
            key.sort();
            faces.insert(key, mesh.elements.last().unwrap().id);
        }
        for q in &raw.quads {
            let mut a = [q[0], q[1], q[2]];
            a.sort();
            let mut b = [q[0], q[2], q[3]];
            b.sort();
            if let (Some(&a), Some(&b)) = (faces.get(&a), faces.get(&b)) {
                let ns = q.map(|n| ids[&n]);
                mesh.quads.push(Quad {
                    id: a,
                    nodes: ns,
                    display_triangles: [a, b],
                    diagonal: [ns[0].min(ns[2]), ns[0].max(ns[2])],
                });
            }
        }
        for e in &raw.constraints {
            let [a, b] = e.map(|n| ids.get(&n).copied());
            if let (Some(a), Some(b)) = (a, b) {
                mesh.constrained_nodes.extend([a, b]);
                mesh.constrained_edges.insert([a.min(b), a.max(b)]);
            }
        }
        for point in load_point_handles {
            if let Some(&node) = ids.get(
                raw.source_nodes
                    .get(&point.index())
                    .unwrap_or(&point.index()),
            ) {
                load_pins.insert(node);
            }
        }
        for n in ids.values().copied() {
            for (id, ss) in &support_segments {
                if ss.iter().any(|s| on_segment(mesh.nodes[n].xy, *s)) {
                    mesh.support_boundary.get_mut(id).unwrap().push(n);
                }
            }
        }
        if mesh.nodes.len() > MAX_NODES {
            return Err("Превышен лимит узлов".into());
        }
    }
    let mut edges = BTreeSet::new();
    for e in &mesh.elements {
        for i in 0..3 {
            let a = e.nodes[i];
            let b = e.nodes[(i + 1) % 3];
            edges.insert([a.min(b), a.max(b)]);
        }
    }
    // Constraint/hull edges whose only adjacent interior face was a numerical
    // sliver are not edges of the physical mesh. Keeping those ghost chords
    // duplicates support lengths and would extrude overlapping wall strips.
    mesh.constrained_edges.retain(|e| edges.contains(e));
    mesh.constrained_nodes = mesh.constrained_edges.iter().flatten().copied().collect();
    mesh.constrained_nodes.extend(load_pins);
    mesh.edges = edges.into_iter().collect();
    if mesh.elements.is_empty() {
        return Err("Не удалось построить элементы внутри плиты".into());
    }
    Ok(mesh)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn opted_in_load_boundaries_and_point_constrain_mesh_without_cutting_plate() {
        use super::super::loads::{ForceUnits, LoadData, Magnitude};
        let mut area = rect(2, Kind::LoadArea, None, 400., 400., 1000., 1000.);
        area.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: true,
            case_id: 1,
            magnitude: Magnitude {
                value: 2.,
                units: ForceUnits::TonForce,
            },
            closed: true,
        });
        let hole = rect(3, Kind::LoadHole, Some(2), 600., 600., 200., 200.);
        let mut point = rect(4, Kind::LoadPoint, None, 0., 0., 1., 1.);
        point.points = vec![[333.125, 444.25]];
        point.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: true,
            case_id: 2,
            magnitude: Magnitude {
                value: 1.,
                units: ForceUnits::TonForce,
            },
            closed: false,
        });
        let mut all = vec![
            rect(1, Kind::Slab, None, 0., 0., 2000., 2000.),
            area,
            hole,
            point,
        ];
        let before = all.clone();
        let mesh = generate(&all, 500.).unwrap();
        let node = mesh
            .nodes
            .iter()
            .position(|n| (n.xy[0] - 333.125).hypot(n.xy[1] - 444.25) < 1e-6)
            .unwrap();
        assert!(mesh.constrained_nodes.contains(&node));
        let optimized = super::super::quad_mesh::process(
            mesh.clone(),
            super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
        )
        .unwrap();
        assert_eq!(optimized.nodes[node].xy, mesh.nodes[node].xy);
        for [a, b] in [[[400., 400.], [1400., 400.]], [[600., 600.], [800., 600.]]] {
            let length = mesh
                .constrained_edges
                .iter()
                .filter_map(|e| {
                    let [p, q] = e.map(|n| mesh.nodes[n].xy);
                    (on_segment(p, [a, b]) && on_segment(q, [a, b]))
                        .then_some((p[0] - q[0]).hypot(p[1] - q[1]))
                })
                .sum::<f64>();
            assert!((length - (a[0] - b[0]).hypot(a[1] - b[1])).abs() < 1e-5);
        }
        let area = mesh
            .elements
            .iter()
            .map(|e| super::super::geometry::area(&e.nodes.map(|n| mesh.nodes[n].xy)))
            .sum::<f64>();
        assert!((area - 4e6).abs() < 1e-3);
        all[1].load.as_mut().unwrap().magnitude.value = 10.;
        assert!(model::same_mesh_input(&before, &all));
        all[1].load.as_mut().unwrap().mesh_constraint = false;
        assert!(!model::same_mesh_input(&before, &all));
        assert!(!model::mesh_sources(&all).any(|c| c.id == 3));
    }
    #[test]
    fn prescribed_grid_cells_keep_two_triangles_despite_smaller_global_size() {
        let mut grid = rect(2, Kind::Triangulation, None, 0., 0., 2000., 2000.);
        grid.grid = Some(model::GridSpec {
            origin: [0., 0.],
            ex: [1000., 0.],
            ey: [0., 1000.],
            nx: 2,
            ny: 2,
        });
        grid.rebuild().unwrap();
        let all = vec![rect(1, Kind::Slab, None, 0., 0., 2000., 2000.), grid];
        let mesh = generate(&all, 200.).unwrap();
        assert_eq!(mesh.nodes.len(), 9);
        assert_eq!(mesh.elements.len(), 8);
        assert!(mesh.support_boundary.is_empty());
        let quads = super::super::quad_mesh::process(
            mesh,
            super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
        )
        .unwrap();
        assert_eq!(quads.quads.len(), 4);
        for q in quads.quads {
            let p = q.nodes.map(|n| quads.nodes[n].xy);
            assert!((super::super::geometry::area(&p) - 1_000_000.).abs() < 1e-5);
        }
    }
    #[test]
    fn triangulation_paths_share_crossing_and_survive_quad_optimisation_without_restraints() {
        let mut all = vec![rect(1, Kind::Slab, None, 0., 0., 4000., 4000.)];
        for (id, points) in [
            (2, vec![[0., 2000.], [4000., 2000.]]),
            (3, vec![[2000., 0.], [2000., 4000.]]),
        ] {
            let mut line = rect(id, Kind::Triangulation, None, 0., 0., 1., 1.);
            line.support_open = true;
            line.points = points;
            all.push(line);
        }
        let original = generate(&all, 500.).unwrap();
        assert_eq!(
            original
                .nodes
                .iter()
                .filter(|n| (n.xy[0] - 2000.).abs() < 1e-6 && (n.xy[1] - 2000.).abs() < 1e-6)
                .count(),
            1
        );
        assert!(original.support_boundary.is_empty());
        for options in [
            super::super::quad_mesh::Options::default(),
            super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
        ] {
            let mesh = super::super::quad_mesh::process(original.clone(), options).unwrap();
            let length: f64 = mesh
                .constrained_edges
                .iter()
                .filter_map(|&[a, b]| {
                    let p = mesh.nodes[a].xy;
                    let q = mesh.nodes[b].xy;
                    if ((p[0] - 2000.).abs() < 1e-6 && (q[0] - 2000.).abs() < 1e-6)
                        || ((p[1] - 2000.).abs() < 1e-6 && (q[1] - 2000.).abs() < 1e-6)
                    {
                        assert_eq!(p, original.nodes[a].xy);
                        assert_eq!(q, original.nodes[b].xy);
                        Some((p[0] - q[0]).hypot(p[1] - q[1]))
                    } else {
                        None
                    }
                })
                .sum();
            assert!((length - 8000.).abs() < 1e-5);
            assert!(mesh.support_boundary.is_empty());
        }
    }

    #[test]
    fn collinear_boundary_faces_are_not_physical_elements() {
        assert!(numerical_sliver([
            [5500., 6000.],
            [4000., 6000.],
            [5000., 6000. + 4.55e-13]
        ]));
        assert!(!numerical_sliver([
            [5500., 6000.],
            [4000., 6000.],
            [5000., 6000.01]
        ]));
        let mut all = vec![rect(1, Kind::Slab, None, 0., 0., 8000., 6000.)];
        let mut support = rect(2, Kind::Support, None, 0., 0., 1., 1.);
        support.support_open = true;
        support.points = vec![[4000., 6000.], [5000., 6000. + 9.09e-13], [5500., 6000.]];
        all.push(support);
        let mesh = generate(&all, 500.).unwrap();
        let mut area = 0.;
        for e in &mesh.elements {
            let xy = e.nodes.map(|i| mesh.nodes[i].xy);
            assert!(!numerical_sliver(xy));
            super::super::fem::element::Triangle::new(xy.map(|p| p.map(|x| x / 1000.))).unwrap();
            area += super::super::geometry::area(&xy);
        }
        assert!((area - 48_000_000.).abs() < 0.01);
        assert!(mesh
            .nodes
            .iter()
            .enumerate()
            .all(|(i, _)| mesh.elements.iter().any(|e| e.nodes.contains(&i))));
    }
    #[test]
    fn project2_roundoff_on_top_boundary_survives_quad_optimisation() {
        let mut slab = rect(1, Kind::Slab, None, 0., 0., 6000., 6000.);
        slab.points[3][1] = 6000.000000000002;
        let mut all = vec![slab];
        for (id, points) in [
            (2, vec![[0., 0.], [0., 6000.000000000002], [6000., 6000.]]),
            (3, vec![[6000., 0.], [4000., 0.]]),
            (4, vec![[6000., 0.], [6000., 2000.]]),
        ] {
            let mut c = rect(id, Kind::Support, None, 0., 0., 1., 1.);
            c.support_open = true;
            c.points = points;
            all.push(c);
        }
        let mesh = generate(&all, 500.).unwrap();
        for options in [
            super::super::quad_mesh::Options::default(),
            super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
        ] {
            let mesh = super::super::quad_mesh::process(mesh.clone(), options).unwrap();
            let mut area = 0.;
            for e in &mesh.elements {
                let xy = e.nodes.map(|i| mesh.nodes[i].xy);
                assert!(!numerical_sliver(xy), "element {}: {xy:?}", e.id);
                super::super::fem::element::Triangle::new(xy.map(|p| p.map(|x| x / 1000.)))
                    .unwrap();
                area += super::super::geometry::area(&xy);
            }
            assert!((area - 36_000_000.).abs() < 0.01);
            for (id, length) in [(2, 12000.), (3, 2000.), (4, 2000.)] {
                let nodes = &mesh.support_boundary[&id];
                let covered = mesh
                    .constrained_edges
                    .iter()
                    .filter(|e| e.iter().all(|i| nodes.contains(i)))
                    .map(|e| {
                        let [a, b] = e.map(|i| mesh.nodes[i].xy);
                        (a[0] - b[0]).hypot(a[1] - b[1])
                    })
                    .sum::<f64>();
                assert!(
                    (covered - length).abs() < 1e-4,
                    "support {id}: {covered} != {length}; edges {:?}",
                    mesh.constrained_edges
                        .iter()
                        .filter(|e| e.iter().all(|i| nodes.contains(i)))
                        .map(|e| e.map(|i| mesh.nodes[i].xy))
                        .collect::<Vec<_>>()
                );
            }
        }
    }
    #[test]
    fn constraint_arrangement_splits_crossings_t_junctions_and_collinear_overlaps() {
        let lines = vec![
            [[0., 0.], [1800., 0.]],
            [[700., -600.], [700., 600.]],
            [[1200., 0.], [1300., 500.]],
            [[300., 0.], [900., 0.]],
        ];
        let pieces = split_constraints(lines).unwrap();
        for x in [300., 700., 900., 1200.] {
            assert!(pieces.iter().any(|s| s[0] == [x, 0.]));
            assert!(pieces.iter().any(|s| s[1] == [x, 0.]));
            assert!(!pieces
                .iter()
                .any(|s| s[0][1] == 0. && s[1][1] == 0. && s[0][0] < x && s[1][0] > x));
        }
        assert!(pieces.contains(&[[700., -600.], [700., 0.]]));
        assert!(pieces.contains(&[[700., 0.], [700., 600.]]));
    }
    #[test]
    fn oblique_t_junction_keeps_complete_support_chains_and_shared_restraints() {
        let slab = rect(1, Kind::Slab, None, 56000., -12200., 25000., 17000.);
        let a = [65800., 3700.];
        let b = [65600., -2800.];
        let junction = [65769.96553446466, 2723.8798701014334];
        let mut all = vec![slab];
        for (id, points) in [
            (2, vec![a, b]),
            (3, vec![[71200., 2400.], junction]),
            (
                4,
                vec![
                    [65712.7837376283, -3931.8794850565023],
                    [65810.35172871634, -6300.],
                ],
            ),
        ] {
            let mut c = rect(id, Kind::Support, None, 0., 0., 1., 1.);
            c.support_open = true;
            c.points = points;
            all.push(c);
        }
        let mesh = generate(&all, 300.).unwrap();
        let joint = mesh
            .nodes
            .iter()
            .position(|n| (n.xy[0] - junction[0]).hypot(n.xy[1] - junction[1]) < NODE_TOL)
            .expect("shared junction node");
        assert!(mesh.support_boundary[&2].contains(&joint));
        assert!(mesh.support_boundary[&3].contains(&joint));
        for c in &all[1..] {
            let s = [c.points[0], c.points[1]];
            let covered: f64 = mesh
                .edges
                .iter()
                .filter(|e| e.iter().all(|&n| on_segment(mesh.nodes[n].xy, s)))
                .map(|e| {
                    assert!(e.iter().all(|n| mesh.support_boundary[&c.id].contains(n)));
                    let p = mesh.nodes[e[0]].xy;
                    let q = mesh.nodes[e[1]].xy;
                    (p[0] - q[0]).hypot(p[1] - q[1])
                })
                .sum();
            let expected = (s[0][0] - s[1][0]).hypot(s[0][1] - s[1][1]);
            assert!(
                (covered - expected).abs() < 1e-5,
                "support {} chain gap: {covered}/{expected}",
                c.id
            );
        }
        // Every fragment is spaced from its own junction to its own endpoint.
        for s in [[a, junction], [junction, b]] {
            let count =
                (((s[1][0] - s[0][0]).hypot(s[1][1] - s[0][1]) / 300.).round() as usize).max(1);
            for i in 0..=count {
                let t = i as f64 / count as f64;
                let p = [
                    s[0][0] + t * (s[1][0] - s[0][0]),
                    s[0][1] + t * (s[1][1] - s[0][1]),
                ];
                assert!(
                    mesh.nodes
                        .iter()
                        .any(|n| (n.xy[0] - p[0]).hypot(n.xy[1] - p[1]) < NODE_TOL),
                    "missing independently spaced support node {p:?}"
                );
            }
        }
    }
    #[test]
    fn coincident_support_and_slab_boundaries_do_not_create_degenerate_elements() {
        let mut all = vec![rect(1, Kind::Slab, None, 0., 0., 8000., 6000.)];
        for (id, points) in [
            (2, vec![[0., 0.], [0., 5000.]]),
            (3, vec![[8000., 6000.], [8000., 4000.]]),
            (4, vec![[8000., 3000.], [8000., 2000.]]),
            (5, vec![[8000., 0.], [7000., 0.]]),
            (6, vec![[3000., 3000.], [5000., 3000.]]),
        ] {
            let mut support = rect(id, Kind::Support, None, 0., 0., 1., 1.);
            support.support_open = true;
            support.points = points;
            all.push(support);
        }
        let mesh = generate(&all, 200.).unwrap();
        for e in &mesh.elements {
            let xy = e.nodes.map(|i| mesh.nodes[i].xy.map(|x| x / 1000.));
            super::super::fem::element::Triangle::new(xy)
                .unwrap_or_else(|err| panic!("element {}: {err}; {xy:?}", e.id));
        }
        let area: f64 = mesh
            .elements
            .iter()
            .map(|e| super::super::geometry::area(&e.nodes.map(|i| mesh.nodes[i].xy)))
            .sum();
        assert!((area - 48_000_000.).abs() < 0.01);
    }
    #[test]
    fn oblique_support_is_a_complete_subdivided_chain_of_shared_plate_edges() {
        let slab = rect(1, Kind::Slab, None, 0., 0., 6000., 4000.);
        let mut support = rect(2, Kind::Support, None, 0., 0., 1., 1.);
        support.support_open = true;
        let a = [371.3, 719.7];
        let b = [5419.1, 3211.9];
        support.points = vec![a, b];
        let mesh = generate(&[slab, support], 500.).unwrap();
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let edges = mesh
            .edges
            .iter()
            .filter(|e| e.iter().all(|&i| on_segment(mesh.nodes[i].xy, [a, b])))
            .collect::<Vec<_>>();
        let mut covered = 0.;
        for e in edges {
            let p = mesh.nodes[e[0]].xy;
            let q = mesh.nodes[e[1]].xy;
            let len = (p[0] - q[0]).hypot(p[1] - q[1]);
            assert!(len <= 550., "unsplit support edge: {len}");
            covered += len;
            assert_eq!(
                mesh.elements
                    .iter()
                    .filter(|t| e.iter().all(|n| t.nodes.contains(n)))
                    .count(),
                2
            );
            assert!(e.iter().all(|n| mesh.support_boundary[&2].contains(n)));
        }
        assert!((covered - length).abs() < 1e-5, "support chain has a gap");
    }
    #[test]
    fn open_support_constrains_only_its_lines_without_a_closing_diagonal() {
        let slab = rect(1, Kind::Slab, None, 0., 0., 4000., 3000.);
        let mut support = rect(2, Kind::Support, None, 0., 0., 1., 1.);
        support.support_open = true;
        support.points = vec![[500., 500.], [3500., 500.], [3500., 2500.]];
        let original = [
            [support.points[0], support.points[1]],
            [support.points[1], support.points[2]],
        ];
        let mesh = generate(&[slab, support], 500.).unwrap();
        let ids = &mesh.support_boundary[&2];
        assert!(ids.len() > 5);
        assert!(ids
            .iter()
            .all(|&n| original.iter().any(|s| on_segment(mesh.nodes[n].xy, *s))));
        for p in [[500., 500.], [3500., 2500.]] {
            assert!(ids
                .iter()
                .any(|&n| (mesh.nodes[n].xy[0] - p[0]).hypot(mesh.nodes[n].xy[1] - p[1]) < 1e-6));
        }
        assert!(
            (mesh
                .elements
                .iter()
                .map(|e| {
                    let p = e.nodes.map(|i| mesh.nodes[i].xy);
                    super::super::geometry::area(&p)
                })
                .sum::<f64>()
                - 12_000_000.)
                .abs()
                < 0.1
        );
    }
    #[test]
    fn almost_straight_spline_boundary_is_not_recursively_split_into_a_fan() {
        let all = vec![Contour {
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
            id: 1,
            parent: None,
            kind: Kind::Slab,
            refinement: None,
            load: None,
            points: vec![],
            curves: vec![
                model::Curve::Spline {
                    closed: false,
                    controls: vec![
                        [0., 0.],
                        [2000., 0.001],
                        [4000., 0.004],
                        [6000., -0.001],
                        [8000., 0.],
                        [10000., 0.002],
                        [12000., 0.],
                    ],
                },
                model::Curve::Line([12000., 0.], [12000., -5000.]),
                model::Curve::Line([12000., -5000.], [0., -5000.]),
                model::Curve::Line([0., -5000.], [0., 0.]),
            ],
        }];
        let mesh = generate(&all, 500.).unwrap();
        assert!(
            mesh.nodes.len() < 3000,
            "{} nodes instead of a moderate 500 mm mesh",
            mesh.nodes.len()
        );
        let upper = mesh.nodes.iter().filter(|n| n.xy[1].abs() < 0.01).count();
        assert!(
            upper <= 30,
            "boundary subdivision unexpectedly grew to {upper} nodes"
        );
        assert!(mesh.elements.len() > 100);
    }
    #[test]
    fn local_regions_keep_conformity_and_use_smaller_size_in_overlap() {
        let mut a = rect(2, Kind::MeshRegion, Some(1), 1000., 1000., 2000., 2000.);
        a.refinement = Some(model::Refinement {
            inactive_edges: vec![],

            name: "Мелкая".into(),
            size: 200.,
        });
        let mut b = rect(3, Kind::MeshRegion, Some(1), 2000., 1500., 2000., 2000.);
        b.refinement = Some(model::Refinement {
            inactive_edges: vec![],

            name: "Вторая".into(),
            size: 400.,
        });
        let mesh = generate(
            &[
                rect(1, Kind::Slab, None, 0., 0., 8000., 5000.),
                a.clone(),
                b.clone(),
            ],
            800.,
        )
        .unwrap();
        let mut total = 0.;
        let mut fine = 0;
        let mut incidence = BTreeMap::<[usize; 2], usize>::new();
        for e in &mesh.elements {
            let p = e.nodes.map(|i| mesh.nodes[i].xy);
            let area = super::super::geometry::area(&p);
            total += area;
            let center = mini_fem::geo::Point::new(
                p.iter().map(|v| v[0]).sum::<f64>() / 3.,
                p.iter().map(|v| v[1]).sum::<f64>() / 3.,
            );
            let step = if polygon(&a.points).contains(&center) {
                fine += 1;
                200.
            } else if polygon(&b.points).contains(&center) {
                400.
            } else {
                800.
            };
            assert!(area <= 3_f64.sqrt() * 0.25 * step * step * 1.00001);
            for i in 0..3 {
                let mut edge = [e.nodes[i], e.nodes[(i + 1) % 3]];
                edge.sort();
                *incidence.entry(edge).or_default() += 1;
            }
        }
        assert!((total - 40_000_000.).abs() < 0.1);
        assert!(fine > 50);
        let boundary = segments(&polygon(&a.points));
        let mut length = 0.;
        for (edge, count) in incidence {
            let p = mesh.nodes[edge[0]].xy;
            let q = mesh.nodes[edge[1]].xy;
            if boundary
                .iter()
                .any(|s| on_segment(p, *s) && on_segment(q, *s))
            {
                assert_eq!(count, 2);
                length += (q[0] - p[0]).hypot(q[1] - p[1]);
            }
        }
        assert!((length - 8000.).abs() < 0.1);
    }
    #[test]
    fn local_slab_step_applies_to_boolean_snapped_perimeter() {
        let mut slab = rect(16, Kind::Slab, None, 0., 0., 1., 1.);
        slab.mesh_size_mm = 200.;
        slab.points = vec![
            [-1077.9858832039374, -917.8080926633208],
            [28651.353758373396, -917.8080926633208],
            [28651.353758373396, 17022.679418082487],
            [19000., 17022.679418082487],
            [13800., 16549.606118864434],
            [-1077.9858832039374, 16549.606118864434],
        ];
        let mesh = generate(std::slice::from_ref(&slab), 800.).unwrap();
        for (a, b) in slab
            .points
            .iter()
            .copied()
            .zip(slab.points.iter().copied().cycle().skip(1))
        {
            let len = |a: Point, b: Point| (a[0] - b[0]).hypot(a[1] - b[1]);
            let on_edge = |p| {
                len(a, p) + len(p, b) - len(a, b) < 1e-3
                    && Euclidean.distance(
                        &mini_fem::geo::Point::new(p[0], p[1]),
                        &mini_fem::geo::Line::new((a[0], a[1]), (b[0], b[1])),
                    ) < 1e-3
            };
            let mut total = 0.;
            for &[i, j] in &mesh.constrained_edges {
                let [p, q] = [mesh.nodes[i].xy, mesh.nodes[j].xy];
                if on_edge(p) && on_edge(q) {
                    let length = len(p, q);
                    assert!(length <= 200.001, "edge {a:?} → {b:?}: {length} mm");
                    total += length;
                }
            }
            assert!((total - len(a, b)).abs() < 1e-3);
        }
    }
    #[test]
    fn local_size_can_also_be_larger_than_global_size() {
        let slab = rect(1, Kind::Slab, None, 0., 0., 6000., 4000.);
        let base = generate(&[slab.clone()], 500.).unwrap();
        let mut region = rect(2, Kind::MeshRegion, Some(1), 0., 0., 6000., 4000.);
        region.refinement = Some(model::Refinement {
            inactive_edges: vec![],

            name: "Крупная".into(),
            size: 1000.,
        });
        let coarse = generate(&[slab, region], 500.).unwrap();
        assert!(coarse.elements.len() < base.elements.len());
    }
    #[test]
    fn oblique_slab_and_hole_refine_at_100mm() {
        let map = |p: &[[f64; 2]]| p.iter().map(|p| [p[0] * 5., -p[1] * 5.]).collect();
        let all = vec![
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
                load: None,
                refinement: None,
                curves: vec![],
                id: 1,
                parent: None,
                kind: Kind::Slab,
                points: map(&[[48., 477.], [124., 180.], [449., 60.], [585., 452.]]),
            },
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
                load: None,
                refinement: None,
                curves: vec![],
                id: 2,
                parent: Some(1),
                kind: Kind::Hole,
                points: map(&[[216., 215.], [274., 193.], [312., 225.], [281., 264.]]),
            },
        ];
        for size in [500., 100.] {
            let m = generate(&all, size).unwrap();
            eprintln!(
                "size {size}: nodes {}, elements {}",
                m.nodes.len(),
                m.elements.len()
            );
            assert!(m.nodes.len() < 5000);
        }
    }
    #[test]
    fn rotated_translated_boundaries_keep_area_size_and_support_interiors() {
        for angle in [0.17_f64, 0.63, 1.21] {
            let mut all = vec![
                rect(1, Kind::Slab, None, 0., 0., 4000., 3000.),
                rect(2, Kind::Hole, Some(1), 800., 700., 300., 400.),
                rect(3, Kind::Support, None, 2000., 500., 600., 600.),
            ];
            let (s, c) = angle.sin_cos();
            for contour in &mut all {
                for p in &mut contour.points {
                    *p = [
                        1_000_000. + c * p[0] - s * p[1],
                        -2_000_000. + s * p[0] + c * p[1],
                    ];
                }
            }
            let mesh = generate(&all, 100.).unwrap();
            let areas = mesh
                .elements
                .iter()
                .map(|e| super::super::geometry::area(&e.nodes.map(|i| mesh.nodes[i].xy)))
                .collect::<Vec<_>>();
            assert!((areas.iter().sum::<f64>() - 11_880_000.).abs() < 0.1);
            assert!(
                areas
                    .iter()
                    .all(|a| *a <= 3_f64.sqrt() * 0.25 * 100. * 100. * 1.00001),
                "angle {angle}, max area {}",
                areas.iter().copied().fold(0., f64::max)
            );
            assert!(!mesh.support_boundary[&3].is_empty());
            assert!(mesh.nodes.len() < 5000);
        }
    }
    #[test]
    fn disconnected_and_concave_slabs_have_no_bridge_elements() {
        let mut a = rect(1, Kind::Slab, None, 0., 0., 2000., 2000.);
        a.points = vec![
            [0., 0.],
            [2000., 0.],
            [2000., 800.],
            [800., 800.],
            [800., 2000.],
            [0., 2000.],
        ];
        let b = rect(2, Kind::Slab, None, 10000., 0., 1000., 1000.);
        let m = generate(&[a, b], 300.).unwrap();
        let area = m
            .elements
            .iter()
            .map(|e| super::super::geometry::area(&e.nodes.map(|i| m.nodes[i].xy)))
            .sum::<f64>();
        assert!((area - 3_560_000.).abs() < 0.01);
        assert!(m
            .nodes
            .iter()
            .all(|n| n.xy[0] <= 2000. || n.xy[0] >= 10000.));
    }
    fn rect(id: u64, kind: Kind, parent: Option<u64>, x: f64, y: f64, w: f64, h: f64) -> Contour {
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
            load: None,
            refinement: None,
            curves: vec![],
            id,
            kind,
            parent,
            points: vec![[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
        }
    }
    #[test]
    fn conforming_supports_holes_area_and_boundary_groups() {
        let all = vec![
            rect(1, Kind::Slab, None, 0., 0., 4000., 3000.),
            rect(2, Kind::Hole, Some(1), 2000., 1000., 500., 500.),
            rect(3, Kind::Support, None, 500., 500., 1000., 1000.),
        ];
        let m = generate(&all, 300.).unwrap();
        let area = m
            .elements
            .iter()
            .map(|e| super::super::geometry::area(&e.nodes.map(|i| m.nodes[i].xy)))
            .sum::<f64>();
        assert!((area - 11_750_000.).abs() < 0.01);
        assert_eq!(m.nodes[0].id, 1);
        assert_eq!(m.elements[0].id, 1);
        let inside = m
            .elements
            .iter()
            .filter(|e| e.supports.contains(&3))
            .map(|e| super::super::geometry::area(&e.nodes.map(|i| m.nodes[i].xy)))
            .sum::<f64>();
        assert!((inside - 1_000_000.).abs() < 0.01);
        let mut incidence = BTreeMap::<[usize; 2], usize>::new();
        for e in &m.elements {
            for i in 0..3 {
                let mut pair = [e.nodes[i], e.nodes[(i + 1) % 3]];
                pair.sort();
                *incidence.entry(pair).or_default() += 1;
            }
        }
        let group = &m.support_boundary[&3];
        assert!(group.len() >= 12);
        let mut length = 0.;
        for (edge, count) in incidence {
            assert!(count <= 2);
            let a = m.nodes[edge[0]].xy;
            let b = m.nodes[edge[1]].xy;
            if segments(&polygon(&all[2].points))
                .iter()
                .any(|s| on_segment(a, *s) && on_segment(b, *s))
            {
                assert_eq!(count, 2, "support edge must share the same nodes with slab");
                length += (b[0] - a[0]).hypot(b[1] - a[1]);
                assert!(group.contains(&edge[0]) && group.contains(&edge[1]));
            }
        }
        assert!((length - 4000.).abs() < 0.01);
        assert!(generate(&all, 99.).is_err());
        assert!(generate(&[], 300.).is_err());
    }
    #[test]
    fn intersecting_supports_and_support_crossing_hole_remain_conforming() {
        let all = vec![
            rect(1, Kind::Slab, None, 0., 0., 3000., 3000.),
            rect(2, Kind::Hole, Some(1), 1400., 1400., 300., 300.),
            rect(3, Kind::Support, None, 500., 500., 1300., 1300.),
            rect(4, Kind::Support, None, 1000., 1000., 1200., 1200.),
        ];
        let m = generate(&all, 350.).unwrap();
        let area = m
            .elements
            .iter()
            .map(|e| super::super::geometry::area(&e.nodes.map(|i| m.nodes[i].xy)))
            .sum::<f64>();
        assert!((area - 8_910_000.).abs() < 0.01);
        assert!(m.elements.iter().any(|e| e.supports.len() == 2));
    }
}
