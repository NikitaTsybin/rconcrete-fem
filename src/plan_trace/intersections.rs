//! Finite physical intersections, independent of FE tessellation.
use super::{
    building_mesh::Layout,
    geometry::Point,
    model::{Contour, Kind},
};
use mini_fem::geo::{
    line_intersection::{line_intersection, LineIntersection},
    BooleanOps, Intersects, Line, LineString, Polygon,
};
use rstar::{RTree, RTreeObject, AABB};
use std::collections::BTreeMap;
#[derive(Clone, Default)]
pub struct Graph {
    pub lines: Vec<Segment>,
    pub points: Vec<([f64; 3], Vec<u64>)>,
}
#[derive(Clone)]
pub struct Segment {
    pub ends: [[f64; 3]; 2],
    pub owners: Vec<u64>,
}
#[derive(Clone)]
struct Face {
    id: u64,
    kind: Kind,
    path: Vec<Point>,
    domain: Option<Polygon<f64>>,
    height: [f64; 2],
    bounds: rstar::AABB<[f64; 3]>,
}
impl rstar::RTreeObject for Face {
    type Envelope = rstar::AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        self.bounds
    }
}
fn ring(points: &[Point]) -> LineString<f64> {
    LineString::from(
        points
            .iter()
            .chain(points.first())
            .map(|p| (p[0], p[1]))
            .collect::<Vec<_>>(),
    )
}

/// Clip by the original segment parameter. Polygon overlay rounds both
/// coordinates to an adaptive grid and can move a structural contact off its axis.
pub(super) fn clip_segment(domain: &Polygon<f64>, [a, b]: [Point; 2]) -> Vec<[Point; 2]> {
    let d = [b[0] - a[0], b[1] - a[1]];
    let length = d[0].hypot(d[1]);
    if length <= 1e-8 {
        return vec![];
    }
    let axis = usize::from(d[1].abs() > d[0].abs());
    let parameter =
        |p: mini_fem::geo::Coord<f64>| (([p.x, p.y][axis] - a[axis]) / d[axis]).clamp(0., 1.);
    let at = |t: f64| match t {
        0. => a,
        1. => b,
        _ => [d[0].mul_add(t, a[0]), d[1].mul_add(t, a[1])],
    };
    let source = Line::new((a[0], a[1]), (b[0], b[1]));
    let mut cuts = vec![0., 1.];
    let mut boundary = Vec::new();
    for side in std::iter::once(domain.exterior())
        .chain(domain.interiors())
        .flat_map(|ring| ring.lines())
    {
        match line_intersection(source, side) {
            Some(LineIntersection::SinglePoint { intersection, .. }) => {
                cuts.push(parameter(intersection));
            }
            Some(LineIntersection::Collinear { intersection }) => {
                let mut span = [parameter(intersection.start), parameter(intersection.end)];
                span.sort_by(f64::total_cmp);
                cuts.extend(span);
                boundary.push(span);
            }
            None => {}
        }
    }
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|x, y| (*x - *y).abs() * length <= 1e-8);
    cuts.windows(2)
        .filter_map(|cut| {
            let middle = (cut[0] + cut[1]) * 0.5;
            let [x, y] = at(middle);
            (domain.intersects(&mini_fem::geo::Point::new(x, y))
                || boundary
                    .iter()
                    .any(|span| (span[0]..=span[1]).contains(&middle)))
            .then(|| [at(cut[0]), at(cut[1])])
        })
        .collect()
}

pub fn height(c: &Contour, layout: Option<&Layout>, legacy: [f64; 2]) -> [f64; 2] {
    layout.map_or_else(
        || {
            if matches!(c.kind, Kind::Support | Kind::Column) {
                legacy
            } else {
                [0., 0.]
            }
        },
        |l| l.extent(c),
    )
}
// Node the finite 3D contact arrangement before any FE spacing. Exact whole
// segment deduplication misses partial overlaps, reversed chains and T joints.
fn noded_lines(
    lines: Vec<Segment>,
    anchors: &[([f64; 3], Vec<u64>)],
) -> Result<Vec<Segment>, String> {
    use rstar::{RTree, RTreeObject, AABB};
    const TOL: f64 = 1e-5;
    struct Indexed {
        index: usize,
        bounds: AABB<[f64; 3]>,
    }
    impl RTreeObject for Indexed {
        type Envelope = AABB<[f64; 3]>;
        fn envelope(&self) -> Self::Envelope {
            self.bounds
        }
    }
    let bounds = |e: [[f64; 3]; 2]| {
        AABB::from_corners(
            std::array::from_fn(|a| e[0][a].min(e[1][a]) - TOL),
            std::array::from_fn(|a| e[0][a].max(e[1][a]) + TOL),
        )
    };
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| a[i] * b[i]).sum::<f64>();
    let sub = |a: [f64; 3], b: [f64; 3]| std::array::from_fn::<_, 3, _>(|i| a[i] - b[i]);
    let tree = RTree::bulk_load(
        lines
            .iter()
            .enumerate()
            .map(|(index, e)| Indexed {
                index,
                bounds: bounds(e.ends),
            })
            .collect(),
    );
    let mut cuts = lines.iter().map(|e| e.ends.to_vec()).collect::<Vec<_>>();
    let on = |p: [f64; 3], e: [[f64; 3]; 2]| {
        let d = sub(e[1], e[0]);
        let l2 = dot(d, d);
        if l2 <= TOL * TOL {
            return false;
        }
        let t = dot(sub(p, e[0]), d) / l2;
        let delta = std::array::from_fn(|i| p[i] - e[0][i] - t * d[i]);
        t >= -TOL / l2.sqrt() && t <= 1. + TOL / l2.sqrt() && dot(delta, delta) <= TOL * TOL
    };
    for (i, e) in lines.iter().enumerate() {
        for other in tree
            .locate_in_envelope_intersecting(&bounds(e.ends))
            .filter(|x| x.index > i)
        {
            let j = other.index;
            let f = &lines[j];
            let mut common = Vec::new();
            for p in e.ends.into_iter().chain(f.ends) {
                if on(p, e.ends) && on(p, f.ends) {
                    common.push(p);
                }
            }
            let u = sub(e.ends[1], e.ends[0]);
            let v = sub(f.ends[1], f.ends[0]);
            let w = sub(e.ends[0], f.ends[0]);
            let a = dot(u, u);
            let b = dot(u, v);
            let c = dot(v, v);
            let d = dot(u, w);
            let h = dot(v, w);
            let det = a * c - b * b;
            if det > 1e-12 * a * c {
                let t = (b * h - c * d) / det;
                let r = (a * h - b * d) / det;
                if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&r) {
                    let p = std::array::from_fn(|k| e.ends[0][k] + t * u[k]);
                    let q = std::array::from_fn(|k| f.ends[0][k] + r * v[k]);
                    if dot(sub(p, q), sub(p, q)) <= TOL * TOL {
                        common.push(p);
                    }
                }
            }
            cuts[i].extend(common.iter().copied());
            cuts[j].extend(common);
        }
    }
    for (p, _) in anchors {
        for e in tree.locate_in_envelope_intersecting(&AABB::from_point(*p)) {
            if on(*p, lines[e.index].ends) {
                cuts[e.index].push(*p);
            }
        }
    }
    let mut out = Vec::new();
    for (line, mut points) in lines.into_iter().zip(cuts) {
        let d = sub(line.ends[1], line.ends[0]);
        points.sort_by(|a, b| {
            dot(sub(*a, line.ends[0]), d).total_cmp(&dot(sub(*b, line.ends[0]), d))
        });
        points.dedup_by(|a, b| dot(sub(*a, *b), sub(*a, *b)) <= TOL * TOL);
        for w in points.windows(2) {
            out.push(Segment {
                ends: [w[0], w[1]],
                owners: line.owners.clone(),
            });
        }
        if out.len() > super::limits::MAX_NODES {
            return Err("Слишком много отрезков пересечений".into());
        }
    }
    Ok(out)
}

pub fn build(all: &[Contour], layout: Option<&Layout>, legacy: [f64; 2]) -> Result<Graph, String> {
    let mut faces = vec![];
    for c in all
        .iter()
        .filter(|c| matches!(c.kind, Kind::Slab | Kind::Support))
    {
        let h = height(c, layout, legacy);
        let mut path = c.points.clone();
        if path.len() < 2 {
            continue;
        }
        let bounds = |p: &[Point]| {
            rstar::AABB::from_corners(
                [
                    p.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - 1e-5,
                    p.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - 1e-5,
                    h[0] - 1e-5,
                ],
                [
                    p.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max) + 1e-5,
                    p.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max) + 1e-5,
                    h[1] + 1e-5,
                ],
            )
        };
        if c.kind == Kind::Slab {
            let holes = all
                .iter()
                .filter(|v| v.active_hole() && v.parent == Some(c.id))
                .map(|v| ring(&v.points))
                .collect();
            faces.push(Face {
                id: c.id,
                kind: c.kind,
                bounds: bounds(&path),
                path: path.clone(),
                domain: Some(Polygon::new(ring(&path), holes)),
                height: h,
            });
        } else {
            if c.closed() {
                path.push(path[0]);
            }
            for e in path.windows(2) {
                faces.push(Face {
                    id: c.id,
                    kind: c.kind,
                    bounds: bounds(e),
                    path: e.to_vec(),
                    domain: None,
                    height: h,
                });
            }
        }
    }
    let tree = rstar::RTree::bulk_load(faces);
    let mut graph = Graph::default();
    let mut seen = std::collections::BTreeSet::new();
    for a in tree.iter() {
        for b in tree.locate_in_envelope_intersecting(&a.bounds) {
            if std::ptr::eq(a, b) {
                continue;
            }
            let ka = a as *const Face as usize;
            let kb = b as *const Face as usize;
            if !seen.insert((ka.min(kb), ka.max(kb))) {
                continue;
            }
            let owners = if a.id == b.id {
                vec![a.id]
            } else {
                vec![a.id.min(b.id), a.id.max(b.id)]
            };
            if a.kind == Kind::Support && b.kind == Kind::Support {
                let lo = a.height[0].max(b.height[0]);
                let hi = a.height[1].min(b.height[1]);
                if hi < lo - 1e-5 {
                    continue;
                }
                match line_intersection(
                    Line::new((a.path[0][0], a.path[0][1]), (a.path[1][0], a.path[1][1])),
                    Line::new((b.path[0][0], b.path[0][1]), (b.path[1][0], b.path[1][1])),
                ) {
                    Some(LineIntersection::SinglePoint {
                        intersection: p, ..
                    }) => {
                        if hi - lo > 1e-5 {
                            graph.lines.push(Segment {
                                ends: [[p.x, p.y, lo], [p.x, p.y, hi]],
                                owners,
                            });
                        } else {
                            graph.points.push(([p.x, p.y, lo], owners));
                        }
                    }
                    Some(LineIntersection::Collinear { intersection: e }) => {
                        let p = [[e.start.x, e.start.y], [e.end.x, e.end.y]];
                        for z in [lo, hi] {
                            graph.lines.push(Segment {
                                ends: p.map(|p| [p[0], p[1], z]),
                                owners: owners.clone(),
                            });
                        }
                    }
                    None => {}
                }
            } else if a.kind != b.kind {
                let (wall, slab) = if a.kind == Kind::Support {
                    (a, b)
                } else {
                    (b, a)
                };
                let z = slab.height[0];
                if z < wall.height[0] - 1e-5 || z > wall.height[1] + 1e-5 {
                    continue;
                }
                for edge in
                    clip_segment(slab.domain.as_ref().unwrap(), [wall.path[0], wall.path[1]])
                {
                    graph.lines.push(Segment {
                        ends: edge.map(|[x, y]| [x, y, z]),
                        owners: owners.clone(),
                    });
                }
            } else if (a.height[0] - b.height[0]).abs() < 1e-5 && a.id != b.id {
                let z = a.height[0];
                let overlap = a
                    .domain
                    .as_ref()
                    .unwrap()
                    .intersection(b.domain.as_ref().unwrap());
                for p in overlap {
                    for r in std::iter::once(p.exterior()).chain(p.interiors()) {
                        for e in r.0.windows(2) {
                            graph.lines.push(Segment {
                                ends: [[e[0].x, e[0].y, z], [e[1].x, e[1].y, z]],
                                owners: owners.clone(),
                            });
                        }
                    }
                }
                // Include hole rings: a capital may exactly fill a parent slab opening.
                let da = a.domain.as_ref().unwrap();
                let db = b.domain.as_ref().unwrap();
                for ra in std::iter::once(da.exterior()).chain(da.interiors()) {
                    for rb in std::iter::once(db.exterior()).chain(db.interiors()) {
                        for ea in ra.0.windows(2) {
                            for eb in rb.0.windows(2) {
                                if let Some(LineIntersection::Collinear { intersection: l }) =
                                    line_intersection(
                                        Line::new(ea[0], ea[1]),
                                        Line::new(eb[0], eb[1]),
                                    )
                                {
                                    graph.lines.push(Segment {
                                        ends: [[l.start.x, l.start.y, z], [l.end.x, l.end.y, z]],
                                        owners: owners.clone(),
                                    });
                                }
                            }
                        }
                    }
                }
                for e in a
                    .path
                    .iter()
                    .zip(a.path.iter().cycle().skip(1))
                    .take(a.path.len())
                {
                    for d in b
                        .path
                        .iter()
                        .zip(b.path.iter().cycle().skip(1))
                        .take(b.path.len())
                    {
                        if let Some(LineIntersection::Collinear { intersection: l }) =
                            line_intersection(
                                Line::new((e.0[0], e.0[1]), (e.1[0], e.1[1])),
                                Line::new((d.0[0], d.0[1]), (d.1[0], d.1[1])),
                            )
                        {
                            graph.lines.push(Segment {
                                ends: [[l.start.x, l.start.y, z], [l.end.x, l.end.y, z]],
                                owners: owners.clone(),
                            });
                        }
                    }
                }
            }
        }
    }
    let mut all_axes = Vec::new();
    // Frame axes are finite too: include beam/column contacts with shells.
    for c in all
        .iter()
        .filter(|c| matches!(c.kind, Kind::Beam | Kind::Column))
    {
        let h = height(c, layout, legacy);
        let axes = if c.kind == Kind::Column {
            c.points
                .first()
                .map(|p| vec![[[p[0], p[1], h[0]], [p[0], p[1], h[1]]]])
                .unwrap_or_default()
        } else if c.is_free_beam() {
            c.beam_path(layout.map_or([0.; 2], |l| l.floor_extent(c)))
                .windows(2)
                .map(|e| [e[0], e[1]])
                .collect::<Vec<_>>()
        } else {
            c.points
                .windows(2)
                .map(|e| [[e[0][0], e[0][1], h[0]], [e[1][0], e[1][1], h[0]]])
                .collect::<Vec<_>>()
        };
        for ends in axes {
            all_axes.push((c.id, ends));
            let bbox = rstar::AABB::from_corners(
                std::array::from_fn(|a| ends[0][a].min(ends[1][a]) - 1e-5),
                std::array::from_fn(|a| ends[0][a].max(ends[1][a]) + 1e-5),
            );
            for f in tree.locate_in_envelope_intersecting(&bbox) {
                let owners = vec![c.id, f.id];
                let d = std::array::from_fn::<_, 3, _>(|a| ends[1][a] - ends[0][a]);
                if f.kind == Kind::Slab {
                    let z = f.height[0];
                    if d[2].abs() > 1e-8 {
                        let t = (z - ends[0][2]) / d[2];
                        if (0. ..=1.).contains(&t) {
                            let p = std::array::from_fn(|a| ends[0][a] + t * d[a]);
                            if mini_fem::geo::Intersects::intersects(
                                f.domain.as_ref().unwrap(),
                                &mini_fem::geo::Point::new(p[0], p[1]),
                            ) {
                                graph.points.push((p, owners));
                            }
                        }
                    } else if (ends[0][2] - z).abs() < 1e-5 {
                        for edge in
                            clip_segment(f.domain.as_ref().unwrap(), ends.map(|p| [p[0], p[1]]))
                        {
                            graph.lines.push(Segment {
                                ends: edge.map(|[x, y]| [x, y, z]),
                                owners: owners.clone(),
                            });
                        }
                    }
                } else {
                    let a = f.path[0];
                    let b = f.path[1];
                    let v = [b[0] - a[0], b[1] - a[1]];
                    let n = [-v[1], v[0]];
                    let denom = n[0] * d[0] + n[1] * d[1];
                    let num = n[0] * (a[0] - ends[0][0]) + n[1] * (a[1] - ends[0][1]);
                    if denom.abs() > 1e-8 {
                        let t = num / denom;
                        if (0. ..=1.).contains(&t) {
                            let p = std::array::from_fn(|a| ends[0][a] + t * d[a]);
                            if p[2] >= f.height[0] - 1e-5
                                && p[2] <= f.height[1] + 1e-5
                                && super::meshing::on_segment([p[0], p[1]], [a, b])
                            {
                                graph.points.push((p, owners));
                            }
                        }
                    } else if num.abs() / v[0].hypot(v[1]) < 1e-5 {
                        let len = v[0].hypot(v[1]);
                        let axis = [v[0] / len, v[1] / len];
                        let uv =
                            ends.map(|p| [(p[0] - a[0]) * axis[0] + (p[1] - a[1]) * axis[1], p[2]]);
                        let rectangle = Polygon::new(
                            ring(&[
                                [0., f.height[0]],
                                [len, f.height[0]],
                                [len, f.height[1]],
                                [0., f.height[1]],
                            ]),
                            vec![],
                        );
                        for edge in clip_segment(&rectangle, uv) {
                            graph.lines.push(Segment {
                                ends: edge
                                    .map(|[u, z]| [a[0] + axis[0] * u, a[1] + axis[1] * u, z]),
                                owners: owners.clone(),
                            });
                        }
                    }
                }
            }
        }
    }
    #[derive(Clone)]
    struct Axis {
        id: u64,
        ends: [[f64; 3]; 2],
    }
    impl rstar::RTreeObject for Axis {
        type Envelope = rstar::AABB<[f64; 3]>;
        fn envelope(&self) -> Self::Envelope {
            rstar::AABB::from_corners(
                std::array::from_fn(|a| self.ends[0][a].min(self.ends[1][a]) - 1e-5),
                std::array::from_fn(|a| self.ends[0][a].max(self.ends[1][a]) + 1e-5),
            )
        }
    }
    let axes = rstar::RTree::bulk_load(
        all_axes
            .into_iter()
            .map(|(id, ends)| Axis { id, ends })
            .collect(),
    );
    let dot = |a: [f64; 3], b: [f64; 3]| a.iter().zip(b).map(|(a, b)| a * b).sum::<f64>();
    for a in axes.iter() {
        for b in axes.locate_in_envelope_intersecting(&rstar::RTreeObject::envelope(a)) {
            if a.id >= b.id {
                continue;
            }
            let u = std::array::from_fn(|i| a.ends[1][i] - a.ends[0][i]);
            let v = std::array::from_fn(|i| b.ends[1][i] - b.ends[0][i]);
            let w = std::array::from_fn(|i| a.ends[0][i] - b.ends[0][i]);
            let (aa, bb, cc, dd, ee) = (dot(u, u), dot(u, v), dot(v, v), dot(u, w), dot(v, w));
            let det = aa * cc - bb * bb;
            if aa < 1e-10 || cc < 1e-10 {
                continue;
            }
            if det.abs() > aa * cc * 1e-12 {
                let t = (bb * ee - cc * dd) / det;
                let s = (aa * ee - bb * dd) / det;
                if !(-1e-9..=1. + 1e-9).contains(&t) || !(-1e-9..=1. + 1e-9).contains(&s) {
                    continue;
                }
                let p = std::array::from_fn(|i| a.ends[0][i] + t * u[i]);
                let q: [f64; 3] = std::array::from_fn(|i| b.ends[0][i] + s * v[i]);
                let d = std::array::from_fn(|i| p[i] - q[i]);
                if dot(d, d) < 1e-10 {
                    graph.points.push((p, vec![a.id, b.id]));
                }
            } else {
                let off = std::array::from_fn(|i| w[i] - u[i] * dd / aa);
                if dot(off, off) > 1e-10 {
                    continue;
                }
                let t0 = -dd / aa;
                let t1 = t0 + bb / aa;
                let lo = t0.min(t1).max(0.);
                let hi = t0.max(t1).min(1.);
                if hi < lo {
                    continue;
                }
                let ends = [lo, hi].map(|t| std::array::from_fn(|i| a.ends[0][i] + t * u[i]));
                if hi - lo < 1e-9 {
                    graph.points.push((ends[0], vec![a.id, b.id]));
                } else {
                    graph.lines.push(Segment {
                        ends,
                        owners: vec![a.id, b.id],
                    });
                }
            }
        }
    }
    let mut unique = std::collections::BTreeMap::new();
    for mut line in noded_lines(graph.lines, &graph.points)? {
        if line.ends[0]
            .iter()
            .zip(line.ends[1])
            .map(|(a, b)| (a - b) * (a - b))
            .sum::<f64>()
            < 1e-10
        {
            continue;
        }
        let mut key = line.ends.map(|p| p.map(|v| (v * 1e5).round() as i64));
        if key[1] < key[0] {
            key.swap(0, 1);
            line.ends.swap(0, 1);
        }
        unique
            .entry(key)
            .and_modify(|old: &mut Segment| {
                for &id in &line.owners {
                    if !old.owners.contains(&id) {
                        old.owners.push(id);
                    }
                }
            })
            .or_insert(line);
    }
    graph.lines = unique.into_values().collect();
    let mut points = std::collections::BTreeMap::<[i64; 3], ([f64; 3], Vec<u64>)>::new();
    for (p, ids) in graph.points {
        let e = points
            .entry(p.map(|v| (v * 1e5).round() as i64))
            .or_insert((p, vec![]));
        for id in ids {
            if !e.1.contains(&id) {
                e.1.push(id);
            }
        }
    }
    graph.points = points.into_values().collect();
    Ok(graph)
}

const CONTACT_TOL: f64 = 1e-5;

struct LoadBoundary {
    ends: [Point; 2],
    z: f64,
    floor: Option<u64>,
}

impl RTreeObject for LoadBoundary {
    type Envelope = AABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(
            [
                self.ends[0][0].min(self.ends[1][0]),
                self.ends[0][1].min(self.ends[1][1]),
                self.z,
            ],
            [
                self.ends[0][0].max(self.ends[1][0]),
                self.ends[0][1].max(self.ends[1][1]),
                self.z,
            ],
        )
    }
}

fn load_boundaries(
    all: &[Contour],
    layout: Option<&Layout>,
    size: f64,
) -> Result<RTree<LoadBoundary>, String> {
    let contours = all.iter().map(|c| (c.id, c)).collect::<BTreeMap<_, _>>();
    let mut boundaries = Vec::new();
    for c in super::model::mesh_sources(all).filter(|c| c.kind.is_load()) {
        // Same source plane as the planar mesh. A load hole inherits its
        // parent's floor and actual placement, including an explicit offset.
        let source = if c.kind == Kind::LoadHole {
            c.parent
                .and_then(|id| contours.get(&id).copied())
                .unwrap_or(c)
        } else {
            c
        };
        let (z, floor) = layout.map_or((0., None), |l| {
            let floor = (source.placement.slab == Default::default()).then(|| l.owner(source.id));
            (l.extent(source)[0], floor)
        });
        let mut points = c.mesh_boundary(size)?;
        if c.closed() || c.kind == Kind::LoadPoint {
            if let Some(&first) = points.first() {
                points.push(first);
            }
        }
        for edge in points.windows(2) {
            boundaries.push(LoadBoundary {
                ends: [edge[0], edge[1]],
                z,
                floor,
            });
        }
        if boundaries.len() > super::limits::MAX_NODES {
            return Err("Слишком много отрезков границ нагрузок для пересечений".into());
        }
    }
    Ok(RTree::bulk_load(boundaries))
}

fn load_contact_cuts(
    line: &Segment,
    slab_floors: &BTreeMap<u64, u64>,
    loads: &RTree<LoadBoundary>,
) -> Vec<f64> {
    let [a, b] = line.ends;
    let mut cuts = Vec::new();
    if loads.size() == 0
        || (a[2] - b[2]).abs() >= CONTACT_TOL
        || !line.owners.iter().any(|id| slab_floors.contains_key(id))
    {
        return cuts;
    }
    let d = [b[0] - a[0], b[1] - a[1]];
    let length = d[0].hypot(d[1]);
    if length <= CONTACT_TOL {
        return cuts;
    }
    let bounds = AABB::from_corners(
        std::array::from_fn(|i| a[i].min(b[i]) - CONTACT_TOL),
        std::array::from_fn(|i| a[i].max(b[i]) + CONTACT_TOL),
    );
    let project = |p: Point| {
        let v = [p[0] - a[0], p[1] - a[1]];
        let offset = (v[0] * d[1] - v[1] * d[0]).abs() / length;
        let t = (v[0] * d[0] + v[1] * d[1]) / (length * length);
        (offset, t)
    };
    let first = Line::new((a[0], a[1]), (b[0], b[1]));
    for load in loads.locate_in_envelope_intersecting(&bounds) {
        if load.floor.is_some_and(|floor| {
            !line
                .owners
                .iter()
                .any(|id| slab_floors.get(id) == Some(&floor))
        }) {
            continue;
        }
        let projected = load.ends.map(project);
        for (offset, t) in projected {
            if offset <= CONTACT_TOL && t > 0. && t < 1. {
                cuts.push(t);
            }
        }
        // Collinear overlaps only introduce their ends. For a transverse
        // intersection, split at the crossing even when neither end touches.
        if load.ends[0] != load.ends[1] && projected.iter().any(|(offset, _)| *offset > CONTACT_TOL)
        {
            let [p, q] = load.ends;
            if let Some(LineIntersection::SinglePoint { intersection, .. }) =
                line_intersection(first, Line::new((p[0], p[1]), (q[0], q[1])))
            {
                let (_, t) = project([intersection.x, intersection.y]);
                if t > 0. && t < 1. {
                    cuts.push(t);
                }
            }
        }
    }
    cuts.retain(|&t| (t.min(1. - t) * length) > CONTACT_TOL);
    cuts.sort_by(f64::total_cmp);
    cuts.dedup_by(|a, b| (*a - *b).abs() * length <= CONTACT_TOL);
    cuts
}

pub fn build_sampled(
    all: &[Contour],
    layout: Option<&Layout>,
    legacy: [f64; 2],
    size: f64,
) -> Result<Graph, String> {
    let mut sampled = all.to_vec();
    for c in &mut sampled {
        if c.is_free_beam() {
            continue;
        }
        if matches!(c.kind, Kind::Slab | Kind::Hole | Kind::Support | Kind::Beam) {
            let size = if matches!(c.kind, Kind::Slab | Kind::Support | Kind::Beam) {
                c.mesh_step(size)
            } else {
                size
            };
            c.points = c.mesh_boundary(size)?;
        }
    }
    let mut graph = build(&sampled, layout, legacy)?;
    let steps = all
        .iter()
        .map(|c| (c.id, c.mesh_step(size)))
        .collect::<BTreeMap<_, _>>();
    let slab_floors = all
        .iter()
        .filter(|c| c.kind == Kind::Slab)
        .map(|c| (c.id, layout.map_or(0, |l| l.owner(c.id))))
        .collect::<BTreeMap<_, _>>();
    let loads = load_boundaries(all, layout, size)?;
    let mut lines = Vec::new();
    for line in graph.lines {
        let step = line
            .owners
            .iter()
            .filter_map(|id| steps.get(id))
            .copied()
            .fold(size, f64::min);
        if !step.is_finite() || step <= 0. {
            return Err("Некорректный шаг пересечений".into());
        }
        let length = line.ends[0]
            .iter()
            .zip(line.ends[1])
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            .sqrt();
        let at = |t: f64| {
            if t == 0. {
                line.ends[0]
            } else if t == 1. {
                line.ends[1]
            } else {
                std::array::from_fn(|a| line.ends[0][a] + (line.ends[1][a] - line.ends[0][a]) * t)
            }
        };
        // Cut at hard load junctions before adding optional spacing nodes.
        // Otherwise two almost coincident nodes remain on the shared chain.
        let mut lo = 0.;
        for hi in load_contact_cuts(&line, &slab_floors, &loads)
            .into_iter()
            .chain([1.])
        {
            let count = ((hi - lo) * length / step).ceil().max(1.) as usize;
            if lines.len().saturating_add(count) > super::limits::MAX_NODES {
                return Err("Превышен лимит узлов пересечений".into());
            }
            let mut from = at(lo);
            for i in 1..=count {
                let to = if i == count {
                    at(hi)
                } else {
                    at(lo + (hi - lo) * i as f64 / count as f64)
                };
                lines.push(Segment {
                    ends: [from, to],
                    owners: line.owners.clone(),
                });
                from = to;
            }
            lo = hi;
        }
    }
    graph.lines = lines;
    Ok(graph)
}

#[cfg(test)]
mod contact_arrangement_regression {
    use super::*;
    #[test]
    fn clipping_contact_preserves_source_axis_and_respects_holes() {
        let x = 8012.587695442262;
        let domain = Polygon::new(
            ring(&[[0., 0.], [64000., 0.], [64000., 20000.], [0., 20000.]]),
            vec![ring(&[
                [8000., 8000.],
                [9000., 8000.],
                [9000., 9000.],
                [8000., 9000.],
            ])],
        );
        let inner = [[x, 9124.628789156044], [x, 10124.628789156044]];
        assert_eq!(clip_segment(&domain, inner), vec![inner]);
        let parts = clip_segment(&domain, [[x, 7500.], [x, 9500.]]);
        assert_eq!(
            parts,
            vec![[[x, 7500.], [x, 8000.]], [[x, 9000.], [x, 9500.]]]
        );
    }

    use crate::plan_trace::{
        building_mesh::Level,
        loads::{ForceUnits, LoadData, Magnitude},
        model::LevelAnchor,
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
            grid: None,
            support_open: kind == Kind::Support,
            load: None,
            refinement: None,
            id,
            parent: None,
            kind,
            points,
            curves: vec![],
        }
    }

    #[test]
    fn hard_load_cuts_replace_nearby_regular_stations_on_the_same_slab() {
        let mut slab = contour(
            1,
            Kind::Slab,
            vec![[-100., -200.], [1100., -200.], [1100., 800.], [-100., 800.]],
        );
        slab.placement.slab.anchor = LevelAnchor::Top;
        let wall = contour(2, Kind::Support, vec![[0., 0.], [1000., 0.]]);
        let mut load = contour(
            3,
            Kind::LoadArea,
            vec![
                [500.05, -100.],
                [1000., -100.],
                [1000., 200.],
                [500.05, 200.],
            ],
        );
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
        let mut hole = contour(
            4,
            Kind::LoadHole,
            vec![[700., -50.], [800., -50.], [800., 50.], [700., 50.]],
        );
        hole.parent = Some(3);
        let mut contours = vec![slab, wall, load, hole];
        let layout = Layout {
            levels: vec![
                Level {
                    id: 10,
                    z: 0.,
                    top: 3000.,
                },
                Level {
                    id: 20,
                    z: 3000.,
                    top: 6000.,
                },
            ],
            owners: [(1, 10), (2, 10), (3, 20), (4, 20)].into(),
            legacy_wall_extent: None,
        };
        let station_x = |graph: &Graph| {
            let mut xs = Vec::new();
            for line in &graph.lines {
                assert_eq!(line.owners, [1, 2]);
                assert!(line.ends.iter().all(|p| p[1] == 0. && p[2] == 3000.));
                xs.extend(line.ends.map(|p| p[0]));
            }
            xs.sort_by(f64::total_cmp);
            xs.dedup();
            xs
        };
        let unmarked = build_sampled(&contours, Some(&layout), [0.; 2], 500.).unwrap();
        assert_eq!(station_x(&unmarked), [0., 500., 1000.]);
        contours[2].load.as_mut().unwrap().mesh_constraint = true;
        // Both lie at 3000 mm, but the load belongs to another floor.
        let wrong_floor = build_sampled(&contours, Some(&layout), [0.; 2], 500.).unwrap();
        assert_eq!(station_x(&wrong_floor), station_x(&unmarked));
        // Explicitly place the load at 3000 mm, permitting it to act on the
        // slab's plane. Its hole inherits that placement from the parent.
        contours[2].placement.slab.anchor = LevelAnchor::Top;
        contours[2].placement.slab.offset_mm = -3000.;
        let shared = build_sampled(&contours, Some(&layout), [0.; 2], 500.).unwrap();
        let xs = station_x(&shared);
        for x in [500.05, 700., 800.] {
            assert!(
                xs.iter().any(|p| (*p - x).abs() < 1e-8),
                "missing {x}: {xs:?}"
            );
        }
        assert!(xs.windows(2).all(|w| w[1] - w[0] > 1.));
        assert!(!xs.contains(&500.));
        assert_eq!(shared.points, unmarked.points);
    }

    #[test]
    fn partial_reversed_overlap_and_t_junction_keep_one_chain() {
        let input = vec![
            Segment {
                ends: [[0., 0., 0.], [1000., 0., 0.]],
                owners: vec![1],
            },
            Segment {
                ends: [[800., 0., 0.], [200., 0., 0.]],
                owners: vec![2],
            },
            Segment {
                ends: [[500., 0., 0.], [500., 0., 500.]],
                owners: vec![3],
            },
        ];
        let lines = noded_lines(input, &[]).unwrap();
        let mut edges = std::collections::BTreeSet::new();
        for mut line in lines {
            line.ends.sort_by(|a, b| a.partial_cmp(b).unwrap());
            edges.insert(line.ends.map(|p| p.map(|v| v as i64)));
        }
        assert_eq!(edges.len(), 5);
        assert!(edges.contains(&[[200, 0, 0], [500, 0, 0]]));
        assert!(edges.contains(&[[500, 0, 0], [800, 0, 0]]));
    }
    #[test]
    fn different_elevations_do_not_create_xy_projection_junctions() {
        let input = vec![
            Segment {
                ends: [[0., 0., 0.], [1000., 0., 0.]],
                owners: vec![1],
            },
            Segment {
                ends: [[500., -500., 10.], [500., 500., 10.]],
                owners: vec![2],
            },
        ];
        assert_eq!(noded_lines(input, &[]).unwrap().len(), 2);
    }
}
