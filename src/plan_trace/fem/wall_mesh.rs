//! Constrained surface triangulation in each wall's local (length, height) plane.
//! Shared contact chains are fixed before optional interior refinement.
use super::*;
use crate::plan_trace::{
    building_mesh::{Data, Wall},
    meshing,
};
use mini_fem::geo::{BooleanOps, Contains, Intersects, LineString, MultiPolygon, Polygon};
use spade::{
    AngleLimit, ConstrainedDelaunayTriangulation, Point2, RefinementParameters, Triangulation,
};

type P = [f64; 2];
const GRID_EPS: f64 = crate::plan_trace::building_mesh::WELD_TOLERANCE_MM;
#[derive(Default)]
pub(super) struct Faces {
    pub triangles: Vec<(u64, [[f64; 3]; 3])>,
    pub quads: Vec<(u64, [[f64; 3]; 4])>,
}
struct Group {
    id: u64,
    origin: P,
    axis: P,
    patches: Vec<usize>,
}
fn ring(p: &[P]) -> LineString<f64> {
    LineString::from(
        p.iter()
            .chain(p.first())
            .map(|p| (p[0], p[1]))
            .collect::<Vec<_>>(),
    )
}
fn local(g: &Group, p: [f64; 3]) -> P {
    [
        (p[0] - g.origin[0]) * g.axis[0] + (p[1] - g.origin[1]) * g.axis[1],
        p[2],
    ]
}
fn world(g: &Group, p: P) -> [f64; 3] {
    [
        g.origin[0] + g.axis[0] * p[0],
        g.origin[1] + g.axis[1] * p[0],
        p[1],
    ]
}
fn planar(g: &Group, p: [f64; 3]) -> bool {
    ((p[0] - g.origin[0]) * g.axis[1] - (p[1] - g.origin[1]) * g.axis[0]).abs() < 1e-5
}

/// The strip grid needs a rectangular domain and all contacts on its boundary.
fn rectangular_axes(
    domain: &MultiPolygon<f64>,
    fragments: &[[P; 2]],
    pins: impl IntoIterator<Item = P>,
) -> Option<[Vec<f64>; 4]> {
    const EPS: f64 = GRID_EPS;
    let [poly] = domain.0.as_slice() else {
        return None;
    };
    if !poly.interiors().is_empty() {
        return None;
    }
    let ext = &poly.exterior().0;
    let limits = |get: fn(&mini_fem::geo::Coord<f64>) -> f64| {
        ext.iter()
            .map(get)
            .fold([f64::INFINITY, f64::NEG_INFINITY], |[lo, hi], v| {
                [lo.min(v), hi.max(v)]
            })
    };
    let [x, z] = [limits(|p| p.x), limits(|p| p.y)];
    if x[1] - x[0] <= EPS || z[1] - z[0] <= EPS {
        return None;
    }
    let sides = |p: P| {
        [
            (p[1] - z[0]).abs() < EPS,
            (p[1] - z[1]).abs() < EPS,
            (p[0] - x[0]).abs() < EPS,
            (p[0] - x[1]).abs() < EPS,
        ]
    };
    let same_side = |[a, b]: [P; 2]| sides(a).into_iter().zip(sides(b)).any(|(a, b)| a && b);
    if !ext
        .windows(2)
        .all(|w| same_side([[w[0].x, w[0].y], [w[1].x, w[1].y]]))
        || !fragments.iter().copied().all(same_side)
    {
        return None;
    }
    let mut axis = [x.to_vec(), x.to_vec(), z.to_vec(), z.to_vec()];
    for mut p in fragments.iter().flatten().copied().chain(pins) {
        let belongs = sides(p);
        if !belongs.iter().any(|&b| b)
            || p[0] < x[0] - EPS
            || p[0] > x[1] + EPS
            || p[1] < z[0] - EPS
            || p[1] > z[1] + EPS
        {
            return None;
        }
        // A corner of the Boolean outline and its physical contact can
        // differ within the weld tolerance. Keep one corner in both chains.
        for (coordinate, bounds) in p.iter_mut().zip([x, z]) {
            for end in bounds {
                if (*coordinate - end).abs() < EPS {
                    *coordinate = end;
                    break;
                }
            }
        }
        for i in 0..4 {
            if belongs[i] {
                axis[i].push(p[usize::from(i >= 2)]);
            }
        }
    }
    for chain in &mut axis {
        chain.sort_by(f64::total_cmp);
        chain.dedup_by(|a, b| (*a - *b).abs() < EPS);
    }
    Some(axis)
}

fn aligned(a: &[f64], b: &[f64]) -> bool {
    a.len() == b.len() && a.iter().zip(b).all(|(a, b)| (a - b).abs() < GRID_EPS)
}

fn rectangular_grid(
    g: &Group,
    [bottom, top, left, right]: &[Vec<f64>; 4],
    output: &mut Faces,
) -> bool {
    let (first, last, rows, opposite, transpose) = if aligned(left, right) {
        (bottom, top, left, right, false)
    } else if aligned(bottom, top) {
        (left, right, bottom, top, true)
    } else if left.len() == right.len() {
        (bottom, top, left, right, false)
    } else if bottom.len() == top.len() {
        (left, right, bottom, top, true)
    } else {
        return false;
    };
    if rows.len().saturating_mul(first.len().max(last.len())) > crate::plan_trace::limits::MAX_NODES
    {
        return false;
    }
    let interior = if first.len() <= last.len() {
        first
    } else {
        last
    };
    let chain = |i| {
        if i == 0 {
            first
        } else if i == rows.len() - 1 {
            last
        } else {
            interior
        }
    };
    let bands = rows.len() - 1;
    let limit = (0..bands)
        .map(|i| {
            let a = chain(i).len() - 1;
            let b = chain(i + 1).len() - 1;
            if a == b {
                a
            } else {
                a + b
            }
        })
        .sum::<usize>();
    if output.triangles.len() + output.quads.len() + limit > crate::plan_trace::limits::MAX_ELEMENTS
    {
        return false;
    }
    let a0 = first[0];
    let width = first.last().unwrap() - a0;
    let point = |x, row: usize| {
        let t = (x - a0) / width;
        [x, rows[row] + (opposite[row] - rows[row]) * t]
    };
    let orient = |p: P| world(g, if transpose { [p[1], p[0]] } else { p });
    let convex = |p: &[P]| {
        (0..p.len()).all(|i| {
            let [a, b, c] = [p[i], p[(i + 1) % p.len()], p[(i + 2) % p.len()]];
            // Two vertices can both weld to an existing contact vertex.
            // Leave more than twice the 3D weld radius between neighbours.
            (b[0] - a[0]).hypot(b[1] - a[1]) > 4e-5
                && (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) > 1e-7
        })
    };
    let (tri_start, quad_start) = (output.triangles.len(), output.quads.len());
    for band in 0..bands {
        let a = chain(band);
        let b = chain(band + 1);
        let lower = |x| point(x, band);
        let upper = |x| point(x, band + 1);
        let (mut i, mut j) = (0, 0);
        let mut valid = true;
        while i + 1 < a.len() || j + 1 < b.len() {
            let pa = lower(a[i]);
            let pb = upper(b[j]);
            let next_a = a.get(i + 1);
            let next_b = b.get(j + 1);
            match (next_a, next_b) {
                (Some(&na), Some(&nb))
                    if a.len() - i == b.len() - j
                        || (na - nb).abs() <= 0.5 * (na - a[i]).min(nb - b[j]) =>
                {
                    let q = [pa, lower(na), upper(nb), pb];
                    valid &= convex(&q);
                    let mut q = q.map(orient);
                    if transpose {
                        q.reverse();
                    }
                    output.quads.push((g.id, q));
                    i += 1;
                    j += 1;
                }
                (Some(&na), nb) if nb.is_none_or(|&nb| na < nb) => {
                    let t = [pa, lower(na), pb];
                    valid &= convex(&t);
                    let mut t = t.map(orient);
                    if transpose {
                        t.reverse();
                    }
                    output.triangles.push((g.id, t));
                    i += 1;
                }
                (_, Some(&nb)) => {
                    let t = [pa, upper(nb), pb];
                    valid &= convex(&t);
                    let mut t = t.map(orient);
                    if transpose {
                        t.reverse();
                    }
                    output.triangles.push((g.id, t));
                    j += 1;
                }
                (None, None) | (Some(_), None) => unreachable!(),
            }
        }
        if !valid {
            output.triangles.truncate(tri_start);
            output.quads.truncate(quad_start);
            return false;
        }
    }
    true
}

#[test]
fn equal_counts_with_shifted_contact_nodes_form_only_quads() {
    let g = Group {
        id: 1,
        origin: [0., 0.],
        axis: [1., 0.],
        patches: vec![],
    };
    let chains = [
        vec![0., 350., 750., 1200.],
        vec![0., 450., 900., 1200.],
        vec![0., 600., 1500., 2250., 3000.],
        vec![0., 800., 1700., 2400., 3000.],
    ];
    let mut mesh = Faces::default();
    assert!(rectangular_grid(&g, &chains, &mut mesh));
    assert!(mesh.triangles.is_empty());
    assert_eq!(mesh.quads.len(), 12);
    let has_node = |point| mesh.quads.iter().any(|(_, q)| q.contains(&point));
    for &x in &chains[0] {
        assert!(has_node([x, 0., 0.]));
    }
    for &x in &chains[1] {
        assert!(has_node([x, 0., 3000.]));
    }
    for &z in &chains[2] {
        assert!(has_node([0., 0., z]));
    }
    for &z in &chains[3] {
        assert!(has_node([1200., 0., z]));
    }
}

#[test]
fn rectangular_grid_uses_one_corner_for_weld_coincident_contacts() {
    let domain = MultiPolygon(vec![Polygon::new(
        ring(&[[0., 0.], [1000., 0.], [1000., 3000.], [0., 3000.]]),
        vec![],
    )]);
    let offset = 8e-6;
    let fragments = [
        [[0., 0.], [500., 0.]],
        [[500., 3000.], [1000. + offset, 3000.]],
        [[1000. + offset, 1500.], [1000. + offset, 3000.]],
        [[0., 0.], [0., 1500.]],
    ];
    let chains = rectangular_axes(&domain, &fragments, [[offset, 3000.]]).unwrap();
    assert_eq!(chains[0], vec![0., 500., 1000.]);
    assert_eq!(chains[0], chains[1]);
    assert_eq!(chains[2], chains[3]);
    assert!(rectangular_axes(&domain, &fragments, [[0.01, 1500.]]).is_none());
}

#[test]
fn mismatched_plate_chains_keep_exact_boundary_nodes_and_a_quad_core() {
    let g = Group {
        id: 158,
        origin: [0., 0.],
        axis: [1., 0.],
        patches: vec![],
    };
    let x0 = vec![0., 400., 800., 1200.];
    let x1 = vec![0., 600., 1200.];
    let z = vec![0., 750., 1500., 2250., 3000.];
    let mut mesh = Faces::default();
    assert!(rectangular_grid(
        &g,
        &[x0.clone(), x1.clone(), z.clone(), z],
        &mut mesh
    ));
    assert!(
        mesh.quads
            .iter()
            .filter(|(_, q)| q.iter().all(|p| p[2] >= 750.))
            .count()
            >= 6
    );
    let mut edges = BTreeMap::new();
    let faces = mesh
        .triangles
        .iter()
        .map(|(_, t)| t.as_slice())
        .chain(mesh.quads.iter().map(|(_, q)| q.as_slice()));
    for face in faces {
        for (&p, &q) in face.iter().zip(face.iter().cycle().skip(1)) {
            let mut edge = [p, q].map(|p| [p[0] as i32, p[2] as i32]);
            edge.sort_unstable();
            *edges.entry(edge).or_insert(0) += 1;
        }
    }
    assert!(edges.iter().all(|([a, b], &count)| {
        let boundary = (a[0] == b[0] && [0, 1200].contains(&a[0]))
            || (a[1] == b[1] && [0, 3000].contains(&a[1]));
        count == if boundary { 1 } else { 2 }
    }));
    for (z, chain) in [(0, x0), (3000, x1)] {
        let actual = edges
            .keys()
            .flat_map(|edge| edge.iter())
            .filter_map(|&[x, height]| (height == z).then_some(x))
            .collect::<BTreeSet<_>>();
        assert_eq!(actual, chain.into_iter().map(|x| x as i32).collect());
    }
}

pub(super) fn triangulate(
    data: &Data,
    mesh: &Mesh,
    size: f64,
    layers: &[[Vec<f64>; 2]],
) -> Result<Faces, String> {
    let xyz = |n: usize| [mesh.nodes[n].xy[0], mesh.nodes[n].xy[1], data.z[n]];
    let mut groups: Vec<Group> = vec![];
    let mut group_ids = BTreeMap::<u64, Vec<usize>>::new();
    let mut junctions = BTreeMap::<[i64; 2], Vec<(u64, f64, f64)>>::new();
    for (i, w) in data.walls.iter().enumerate() {
        let a = xyz(w.bottom[0]);
        let b = xyz(w.bottom[1]);
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0].hypot(d[1]);
        if l < 1e-5 {
            return Err(format!("Стена {}: нулевой участок контакта", w.id));
        }
        for p in [a, b] {
            junctions
                .entry([(p[0] * 1e5).round() as i64, (p[1] * 1e5).round() as i64])
                .or_default()
                .push((w.id, a[2], data.z[w.top[0]]));
        }
        let old = group_ids
            .get(&w.id)
            .into_iter()
            .flatten()
            .copied()
            .find(|&i| planar(&groups[i], a) && planar(&groups[i], b));
        if let Some(index) = old {
            groups[index].patches.push(i);
        } else {
            group_ids.entry(w.id).or_default().push(groups.len());
            groups.push(Group {
                id: w.id,
                origin: [a[0], a[1]],
                axis: [d[0] / l, d[1] / l],
                patches: vec![i],
            });
        }
    }

    // Source wall outlines also live in mesh.edges. They are not plate FE
    // edges, even if their two endpoints happen to belong to a plate.
    let diagonals = mesh
        .quads
        .iter()
        .map(|q| q.diagonal)
        .collect::<BTreeSet<_>>();
    let plate_edges = mesh
        .elements
        .iter()
        .flat_map(|e| {
            (0..3).map(move |i| {
                let mut edge = [e.nodes[i], e.nodes[(i + 1) % 3]];
                edge.sort();
                edge
            })
        })
        .filter(|e| !diagonals.contains(e))
        .collect::<BTreeSet<_>>()
        .into_iter()
        .collect::<Vec<_>>();
    #[derive(Clone)]
    struct Edge {
        nodes: [usize; 2],
        envelope: rstar::AABB<[f64; 3]>,
    }
    impl rstar::RTreeObject for Edge {
        type Envelope = rstar::AABB<[f64; 3]>;
        fn envelope(&self) -> Self::Envelope {
            self.envelope
        }
    }
    let edge_index = rstar::RTree::bulk_load(
        plate_edges
            .iter()
            .map(|&nodes| Edge {
                nodes,
                envelope: rstar::AABB::from_corners(
                    std::array::from_fn(|a| xyz(nodes[0])[a].min(xyz(nodes[1])[a])),
                    std::array::from_fn(|a| xyz(nodes[0])[a].max(xyz(nodes[1])[a])),
                ),
            })
            .collect(),
    );
    let node_index = rstar::RTree::bulk_load(
        (0..mesh.nodes.len())
            .map(|n| rstar::primitives::GeomWithData::new(xyz(n), n))
            .collect(),
    );
    let mut wall_pins = vec![];
    for (i, w) in data.walls.iter().enumerate() {
        for side in 0..2 {
            let xy = mesh.nodes[w.bottom[side]].xy;
            for &z in &layers[i][side] {
                wall_pins.push(rstar::primitives::GeomWithData::new([xy[0], xy[1], z], ()));
            }
        }
    }
    let wall_pin_index = rstar::RTree::bulk_load(wall_pins);
    let group_area = |g: &Group| {
        g.patches
            .iter()
            .map(|&i| {
                let w = &data.walls[i];
                let a = xyz(w.bottom[0]);
                let b = xyz(w.bottom[1]);
                (a[0] - b[0]).hypot(a[1] - b[1]) * (data.z[w.top[0]] - a[2])
            })
            .sum::<f64>()
    };
    groups.sort_by(|a, b| {
        group_area(a)
            .total_cmp(&group_area(b))
            .then_with(|| b.id.cmp(&a.id))
    });
    let mut covered = BTreeMap::<(i64, i64, i64), Vec<(P, P, MultiPolygon<f64>)>>::new();
    #[derive(Clone)]
    struct Contact {
        ends: [[f64; 3]; 2],
    }
    impl rstar::RTreeObject for Contact {
        type Envelope = rstar::AABB<[f64; 3]>;
        fn envelope(&self) -> Self::Envelope {
            rstar::AABB::from_corners(
                std::array::from_fn(|a| self.ends[0][a].min(self.ends[1][a]) - 1e-5),
                std::array::from_fn(|a| self.ends[0][a].max(self.ends[1][a]) + 1e-5),
            )
        }
    }
    let contact_index =
        rstar::RTree::bulk_load(data.contacts.iter().map(|&ends| Contact { ends }).collect());
    let mut output = Faces::default();
    let count = groups.len();
    for (index, g) in groups.into_iter().enumerate() {
        progress::check_cancel()?;
        if data.wall_quad_options.backend == meshing::Backend::Gmsh {
            progress::stage(&format!(
                "Gmsh · стены {}/{count} · объект {}",
                index + 1,
                g.id
            ));
        }
        let points = g
            .patches
            .iter()
            .flat_map(|&i| data.walls[i].bottom.into_iter().chain(data.walls[i].top))
            .map(xyz)
            .collect::<Vec<_>>();
        let bounds = rstar::AABB::from_corners(
            std::array::from_fn(|a| {
                points.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min) - 1e-5
            }),
            std::array::from_fn(|a| {
                points
                    .iter()
                    .map(|p| p[a])
                    .fold(f64::NEG_INFINITY, f64::max)
                    + 1e-5
            }),
        );
        let mut domain = MultiPolygon(vec![]);
        let mut pins = vec![];
        let mut contacts = vec![];
        let mut step = f64::INFINITY;
        for &i in &g.patches {
            let w: &Wall = &data.walls[i];
            step = step.min(if w.mesh_size_mm > 0. {
                w.mesh_size_mm
            } else {
                size
            });
            let a = local(&g, xyz(w.bottom[0]));
            let b = local(&g, xyz(w.bottom[1]));
            let top = data.z[w.top[0]];
            let lo = a[0].min(b[0]);
            let hi = a[0].max(b[0]);
            let p = Polygon::new(
                ring(&[[lo, a[1]], [hi, a[1]], [hi, top], [lo, top]]),
                vec![],
            );
            domain = domain.union(&MultiPolygon(vec![p]));
            for side in 0..2 {
                let q = xyz(w.bottom[side]);
                let x = local(&g, q)[0];
                let points = layers[i][side].iter().map(|&z| [x, z]).collect::<Vec<_>>();
                pins.extend(points.iter().copied());
                let key = [(q[0] * 1e5).round() as i64, (q[1] * 1e5).round() as i64];
                if let Some(neighbours) = junctions.get(&key) {
                    contacts.extend(
                        points
                            .windows(2)
                            .filter(|p| {
                                neighbours.iter().any(|&(id, lo, hi)| {
                                    id != g.id
                                        && (p[0][1] + p[1][1]) * 0.5 >= lo - 1e-5
                                        && (p[0][1] + p[1][1]) * 0.5 <= hi + 1e-5
                                })
                            })
                            .map(|w| [w[0], w[1]]),
                    );
                }
            }
        }
        if !step.is_finite() || step < 100. {
            return Err(format!("Стена {}: размер КЭ меньше 100 мм", g.id));
        }
        let mut axis = g.axis;
        if axis[0] < -1e-10 || axis[0].abs() < 1e-10 && axis[1] < 0. {
            axis = axis.map(|v| -v);
        }
        let key = (
            (axis[0] * 1e8).round() as i64,
            (axis[1] * 1e8).round() as i64,
            ((g.origin[0] * axis[1] - g.origin[1] * axis[0]) * 1e4).round() as i64,
        );
        let original_domain = domain.clone();
        for (origin, old_axis, old) in covered.get(&key).into_iter().flatten() {
            let map_ring = |ring: &LineString<f64>| {
                LineString::from(
                    ring.0
                        .iter()
                        .map(|p| {
                            let xy = [origin[0] + old_axis[0] * p.x, origin[1] + old_axis[1] * p.x];
                            (
                                (xy[0] - g.origin[0]) * g.axis[0]
                                    + (xy[1] - g.origin[1]) * g.axis[1],
                                p.y,
                            )
                        })
                        .collect::<Vec<_>>(),
                )
            };
            let old = MultiPolygon(
                old.0
                    .iter()
                    .map(|p| {
                        Polygon::new(
                            map_ring(p.exterior()),
                            p.interiors().iter().map(map_ring).collect(),
                        )
                    })
                    .collect(),
            );
            domain = domain.difference(&old);
            if domain.0.is_empty() {
                break;
            }
        }
        covered
            .entry(key)
            .or_default()
            .push((g.origin, g.axis, original_domain));
        if domain.0.is_empty() {
            continue;
        }
        // Boolean overlays use an adaptive integer grid. Restore exact physical
        // coordinates at existing contact/patch anchors before inserting them in
        // the CDT; otherwise a boundary shifted by microns can strand a source node.
        let mut anchor_coordinates = [Vec::<f64>::new(), Vec::<f64>::new()];
        for node in node_index.locate_in_envelope_intersecting(&bounds) {
            let p = xyz(node.data);
            if planar(&g, p) {
                let p = local(&g, p);
                for a in 0..2 {
                    anchor_coordinates[a].push(p[a]);
                }
            }
        }
        for a in &mut anchor_coordinates {
            a.sort_by(f64::total_cmp);
            a.dedup_by(|a, b| (*a - *b).abs() < 1e-8);
        }
        let snap_ring = |ring: &mut LineString<f64>| {
            for p in &mut ring.0 {
                for (a, value) in [&mut p.x, &mut p.y].into_iter().enumerate() {
                    let values = &anchor_coordinates[a];
                    let i = values.partition_point(|v| *v < *value);
                    let nearest = [i.checked_sub(1), (i < values.len()).then_some(i)]
                        .into_iter()
                        .flatten()
                        .map(|i| values[i])
                        .min_by(|x, y| (*x - *value).abs().total_cmp(&(*y - *value).abs()));
                    if let Some(v) = nearest.filter(|v| (*v - *value).abs() < 1e-3) {
                        *value = v;
                    }
                }
            }
        };
        for poly in &mut domain.0 {
            poly.exterior_mut(snap_ring);
            poly.interiors_mut(|rings| {
                for ring in rings {
                    snap_ring(ring);
                }
            });
        }
        for node in wall_pin_index.locate_in_envelope_intersecting(&bounds) {
            let p = *node.geom();
            if planar(&g, p) {
                pins.push(local(&g, p));
            }
        }
        let mut boundary = vec![];
        for poly in &domain.0 {
            for ring in std::iter::once(poly.exterior()).chain(poly.interiors()) {
                boundary.extend(
                    ring.0
                        .windows(2)
                        .map(|w| [[w[0].x, w[0].y], [w[1].x, w[1].y]]),
                );
            }
        }
        for e in edge_index.locate_in_envelope_intersecting(&bounds) {
            let a = xyz(e.nodes[0]);
            let b = xyz(e.nodes[1]);
            if !planar(&g, a) || !planar(&g, b) || (a[2] - b[2]).abs() > 1e-5 {
                continue;
            }
            let [a, b] = [local(&g, a), local(&g, b)];
            // Plate meshes are already constrained by the finite wall trace.
            // Keep only edge portions inside this wall's physical rectangles.
            for &i in &g.patches {
                let w = &data.walls[i];
                let p = local(&g, xyz(w.bottom[0]));
                let q = local(&g, xyz(w.bottom[1]));
                if a[1] < p[1] - 1e-5 || a[1] > data.z[w.top[0]] + 1e-5 {
                    continue;
                }
                let lo = a[0].min(b[0]).max(p[0].min(q[0]));
                let hi = a[0].max(b[0]).min(p[0].max(q[0]));
                if hi - lo > 1e-5 {
                    contacts.push([[lo, a[1]], [hi, a[1]]]);
                }
            }
        }
        // Retain source wall anchors as isolated vertices as well as plate contacts.
        // Union/overlap partition can turn a patch corner into an interior point;
        // dropping it would leave its preallocated physical node without any element.
        let mut isolated_pins = vec![];
        for node in node_index.locate_in_envelope_intersecting(&bounds) {
            let p = xyz(node.data);
            if planar(&g, p) {
                let p = local(&g, p);
                if domain.intersects(&mini_fem::geo::Point::new(p[0], p[1])) {
                    pins.push(p);
                    isolated_pins.push(p);
                }
            }
        }
        for contact in contact_index.locate_in_envelope_intersecting(&bounds) {
            let e = &contact.ends;
            if planar(&g, e[0]) && planar(&g, e[1]) {
                let p = e.map(|p| local(&g, p));
                if (p[0][0] - p[1][0]).hypot(p[0][1] - p[1][1]) < 1e-5 {
                    if domain.intersects(&mini_fem::geo::Point::new(p[0][0], p[0][1])) {
                        pins.push(p[0]);
                        isolated_pins.push(p[0]);
                    }
                } else {
                    contacts.extend(
                        domain.0.iter().flat_map(|poly| {
                            crate::plan_trace::intersections::clip_segment(poly, p)
                        }),
                    );
                }
            }
        }
        // Boolean clipping may quantize the same original contact independently
        // from the domain boundary. Restore its known anchor coordinates too,
        // before noding/deduplicating the combined arrangement.
        for p in contacts.iter_mut().flatten() {
            for a in 0..2 {
                let values = &anchor_coordinates[a];
                let i = values.partition_point(|v| *v < p[a]);
                let nearest = [i.checked_sub(1), (i < values.len()).then_some(i)]
                    .into_iter()
                    .flatten()
                    .map(|i| values[i])
                    .min_by(|x, y| (*x - p[a]).abs().total_cmp(&(*y - p[a]).abs()));
                if let Some(v) = nearest.filter(|v| (*v - p[a]).abs() < 1e-3) {
                    p[a] = v;
                }
            }
        }
        let mut lines = Vec::new();
        for &edge in &boundary {
            for [a, b] in crate::plan_trace::building_mesh::outside_contacts(edge, &contacts) {
                let d = [b[0] - a[0], b[1] - a[1]];
                let along = |p: &P| (p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1];
                let mut chain = vec![a, b];
                chain.extend(
                    pins.iter()
                        .copied()
                        .filter(|&p| meshing::on_segment(p, [a, b])),
                );
                chain.sort_by(|p, q| along(p).total_cmp(&along(q)));
                chain.dedup_by(|p, q| (p[0] - q[0]).hypot(p[1] - q[1]) < 1e-5);
                for w in chain.windows(2) {
                    let count = ((w[1][0] - w[0][0]).hypot(w[1][1] - w[0][1]) / step)
                        .ceil()
                        .max(1.) as usize;
                    if count > crate::plan_trace::limits::MAX_NODES {
                        return Err("Превышен лимит узлов границы стены".into());
                    }
                    for i in 0..count {
                        let at = |i: usize| {
                            if i == 0 {
                                w[0]
                            } else if i == count {
                                w[1]
                            } else {
                                std::array::from_fn(|a| {
                                    w[0][a] + (w[1][a] - w[0][a]) * i as f64 / count as f64
                                })
                            }
                        };
                        lines.push([at(i), at(i + 1)]);
                    }
                }
            }
        }
        lines.extend(contacts.into_iter().filter(|e| {
            domain.intersects(&mini_fem::geo::Point::new(
                (e[0][0] + e[1][0]) / 2.,
                (e[0][1] + e[1][1]) / 2.,
            ))
        }));
        let fragments = meshing::split_constraints(lines)?;
        if data.wall_quad_options.merge {
            let boundary_pins = pins
                .iter()
                .copied()
                .filter(|&p| boundary.iter().any(|&e| meshing::on_segment(p, e)));
            if let Some(chains) = rectangular_axes(
                &domain,
                &fragments,
                boundary_pins.chain(isolated_pins.iter().copied()),
            ) {
                if rectangular_grid(&g, &chains, &mut output) {
                    continue;
                }
            }
        }
        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
        let mut registered = BTreeMap::<(i64, i64), Vec<spade::handles::FixedVertexHandle>>::new();
        let mut insert = |cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
                          p: P|
         -> Result<spade::handles::FixedVertexHandle, String> {
            let key = ((p[0] / 1e-5).floor() as i64, (p[1] / 1e-5).floor() as i64);
            for dx in -1..=1 {
                for dy in -1..=1 {
                    for &h in registered
                        .get(&(key.0 + dx, key.1 + dy))
                        .into_iter()
                        .flatten()
                    {
                        let q = cdt.vertex(h).position();
                        if (q.x - p[0]).hypot(q.y - p[1]) < 1e-5 {
                            return Ok(h);
                        }
                    }
                }
            }
            let h = cdt
                .insert(Point2::new(p[0], p[1]))
                .map_err(|e| format!("Стена {}: узел {e:?}", g.id))?;
            registered.entry(key).or_default().push(h);
            Ok(h)
        };
        for [a, b] in fragments {
            let mut pts = vec![a, b];
            pts.extend(
                pins.iter()
                    .copied()
                    .filter(|&p| meshing::on_segment(p, [a, b])),
            );
            pts.sort_by(|p, q| {
                ((p[0] - a[0]) * (b[0] - a[0]) + (p[1] - a[1]) * (b[1] - a[1]))
                    .total_cmp(&((q[0] - a[0]) * (b[0] - a[0]) + (q[1] - a[1]) * (b[1] - a[1])))
            });
            pts.dedup_by(|p, q| (p[0] - q[0]).hypot(p[1] - q[1]) < 1e-6);
            for pair in pts.windows(2) {
                let a = insert(&mut cdt, pair[0])?;
                let b = insert(&mut cdt, pair[1])?;
                if a != b && !cdt.exists_constraint(a, b) {
                    if !cdt.can_add_constraint(a, b) {
                        return Err(format!(
                            "Стена {}: несогласованное пересечение линий контакта",
                            g.id
                        ));
                    }
                    cdt.add_constraint(a, b);
                }
            }
        }
        // Only keep additional pins that belong to the boundary or a contact:
        // internal patch divisions must not seed complete artificial vertical rows.
        for p in pins
            .into_iter()
            .filter(|&p| boundary.iter().any(|&e| meshing::on_segment(p, e)))
        {
            insert(&mut cdt, p)?;
        }
        for p in isolated_pins {
            insert(&mut cdt, p)?;
        }
        let raw = if data.wall_quad_options.backend == meshing::Backend::Gmsh {
            crate::plan_trace::mesh_backend::gmsh_refined(
                &mut cdt,
                |_| step,
                step,
                |p| {
                    domain
                        .contains(&mini_fem::geo::Point::new(p[0], p[1]))
                        .then_some(step * step * 0.5)
                },
            )?
        } else {
            let result = cdt.refine(
                RefinementParameters::new()
                    .keep_constraint_edges()
                    .with_angle_limit(AngleLimit::from_deg(22.))
                    .with_min_required_area(step * step * 0.10)
                    .with_max_allowed_area(step * step * 0.5)
                    .with_max_additional_vertices(
                        crate::plan_trace::limits::MAX_NODES.saturating_sub(cdt.num_vertices()),
                    ),
            );
            if !result.refinement_complete {
                return Err(format!(
                    "Стена {}: достигнут лимит поверхностной триангуляции",
                    g.id
                ));
            }
            crate::plan_trace::mesh_backend::spade(&cdt)
        };
        let mut quad_triangles = BTreeSet::new();
        for q in &raw.quads {
            let p = q.map(|n| raw.points[&n]);
            let center = mini_fem::geo::Point::new(
                p.iter().map(|p| p[0] / 4.).sum(),
                p.iter().map(|p| p[1] / 4.).sum(),
            );
            if domain.contains(&center) {
                if output.triangles.len() + output.quads.len()
                    >= crate::plan_trace::limits::MAX_ELEMENTS
                {
                    return Err("Превышен лимит КЭ стен".into());
                }
                output.quads.push((g.id, p.map(|p| world(&g, p))));
                for mut t in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
                    t.sort();
                    quad_triangles.insert(t);
                }
            }
        }
        for f in &raw.triangles {
            let mut key = *f;
            key.sort();
            if quad_triangles.contains(&key) {
                continue;
            }
            let p = f.map(|n| raw.points[&n]);
            let center = mini_fem::geo::Point::new(
                (p[0][0] + p[1][0] + p[2][0]) / 3.,
                (p[0][1] + p[1][1] + p[2][1]) / 3.,
            );
            if !domain.contains(&center) {
                continue;
            }
            let area = ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                .abs();
            let longest = (p[1][0] - p[0][0])
                .hypot(p[1][1] - p[0][1])
                .max((p[2][0] - p[0][0]).hypot(p[2][1] - p[0][1]))
                .max((p[2][0] - p[1][0]).hypot(p[2][1] - p[1][1]));
            if area <= (longest * 3e-5).max(1e-7) {
                continue;
            }
            output.triangles.push((g.id, p.map(|p| world(&g, p))));
            if output.triangles.len() + output.quads.len() > crate::plan_trace::limits::MAX_ELEMENTS
            {
                return Err("Превышен лимит КЭ стен".into());
            }
        }
    }
    Ok(output)
}
