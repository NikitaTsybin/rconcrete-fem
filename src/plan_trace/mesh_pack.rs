//! Pack generated FE topology before creating any result, load or display indices.
use super::meshing::Mesh;
use std::collections::{BTreeSet, HashSet};

/// Keep the same numerical scale as the spatial node weld.
const TOL: f64 = super::building_mesh::WELD_TOLERANCE_MM;

fn sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn face_normal([a, b, c]: [[f64; 3]; 3]) -> Option<[f64; 3]> {
    let ab = sub(b, a);
    let ac = sub(c, a);
    let bc = sub(c, b);
    let longest2 = dot(ab, ab).max(dot(ac, ac)).max(dot(bc, bc));
    let n = cross(ab, ac);
    let area2 = dot(n, n);
    (area2.is_finite() && area2 > (longest2 * TOL * TOL).max(1e-16)).then_some(n)
}

pub(super) fn valid_triangle(points: [[f64; 3]; 3]) -> bool {
    face_normal(points).is_some()
}

pub(super) fn valid_quad(points: [[f64; 3]; 4]) -> bool {
    let Some(normal) = face_normal([points[0], points[1], points[2]]) else {
        return false;
    };
    (1..4).all(|i| {
        face_normal([points[i], points[(i + 1) % 4], points[(i + 2) % 4]])
            .is_some_and(|n| dot(normal, n) > 0.)
    })
}

const QUAD_DIAGONALS: [[[usize; 3]; 2]; 2] = [[[0, 1, 2], [0, 2, 3]], [[0, 1, 3], [1, 2, 3]]];

fn split_normals(points: [[f64; 3]; 4], diagonal: usize) -> [Option<[f64; 3]>; 2] {
    QUAD_DIAGONALS[diagonal].map(|triangle| face_normal(triangle.map(|i| points[i])))
}

/// Prefer a diagonal with two non-overlapping faces. A collapsed corner can
/// leave one physical triangle. A crossed contour cannot be reconstructed.
pub(super) fn quad_split(points: [[f64; 3]; 4]) -> Option<[[usize; 3]; 2]> {
    let mut fallback = None;
    for (i, triangles) in QUAD_DIAGONALS.into_iter().enumerate() {
        match split_normals(points, i) {
            [Some(a), Some(b)] if dot(a, b) > 0. => return Some(triangles),
            [Some(_), None] | [None, Some(_)] | [None, None] => {
                fallback.get_or_insert(triangles);
            }
            _ => {}
        }
    }
    fallback
}

fn edge(a: usize, b: usize) -> [usize; 2] {
    [a.min(b), a.max(b)]
}

#[derive(Default, PartialEq, Eq, Debug)]
pub(super) struct Packed {
    pub(super) removed_nodes: usize,
    pub(super) removed_faces: usize,
    pub(super) split_quads: usize,
}

pub(super) fn pack(mesh: &mut Mesh) -> Result<Packed, String> {
    use super::fem::progress::check_cancel;

    let xyz = |n: usize| {
        let [x, y] = mesh.nodes[n].xy;
        [x, y, mesh.node_z(n)]
    };
    let mut kept = Vec::with_capacity(mesh.elements.len());
    let mut removed_ids = HashSet::new();
    let mut used = vec![false; mesh.nodes.len()];
    for (i, e) in mesh.elements.iter().enumerate() {
        if i % 4096 == 0 {
            check_cancel()?;
        }
        if valid_triangle(e.nodes.map(xyz)) {
            kept.push(i);
            for n in e.nodes {
                used[n] = true;
            }
        } else {
            removed_ids.insert(e.id);
        }
    }
    let mut packed = Packed {
        removed_faces: removed_ids.len(),
        ..Default::default()
    };
    if kept.is_empty() && !mesh.elements.is_empty() {
        return Err("После упаковки не осталось невырожденных КЭ плит".into());
    }
    let mut bad_quads = HashSet::new();
    for q in &mesh.quads {
        let points = q.nodes.map(xyz);
        let has_removed_face = q
            .display_triangles
            .iter()
            .any(|id| removed_ids.contains(id));
        if has_removed_face || !valid_quad(points) {
            let diagonal = usize::from(!q.diagonal.contains(&q.nodes[0]));
            if !has_removed_face
                && matches!(split_normals(points, diagonal), [Some(a), Some(b)] if dot(a, b) <= 0.)
            {
                return Err(format!(
                    "КЭ плиты {}: пересекающиеся треугольники четырёхугольника",
                    q.id
                ));
            }
            bad_quads.insert(q.id);
        }
    }
    packed.split_quads = bad_quads.len();
    if !bad_quads.is_empty() {
        mesh.quads.retain(|q| !bad_quads.contains(&q.id));
    }
    if let Some(data) = &mesh.building {
        for n in data
            .walls
            .iter()
            .flat_map(|w| w.bottom.into_iter().chain(w.top))
            .chain(data.beams.iter().flat_map(|b| b.nodes))
        {
            used[n] = true;
        }
    }
    packed.removed_nodes = used.iter().filter(|&&used| !used).count();
    if packed.removed_nodes == 0 && packed.removed_faces == 0 {
        return Ok(packed);
    }

    let mut indices = vec![usize::MAX; used.len()];
    let mut next = 0;
    for (n, is_used) in used.iter().copied().enumerate() {
        if is_used {
            indices[n] = next;
            next += 1;
        }
    }
    let remap = |n: usize| indices[n];
    let mut node = 0;
    mesh.nodes.retain(|_| {
        let keep = used[node];
        node += 1;
        keep
    });
    mesh.elements.retain_mut(|element| {
        if removed_ids.contains(&element.id) {
            return false;
        }
        element.nodes = element.nodes.map(remap);
        true
    });
    for q in &mut mesh.quads {
        q.nodes = q.nodes.map(remap);
        q.diagonal = q.diagonal.map(remap);
    }
    let mut edges = BTreeSet::new();
    for e in &mesh.elements {
        for i in 0..3 {
            edges.insert(edge(e.nodes[i], e.nodes[(i + 1) % 3]));
        }
    }
    if let Some(data) = &mut mesh.building {
        let mut index = 0;
        data.z.retain(|_| {
            let keep = used[index];
            index += 1;
            keep
        });
        data.node_floors = data
            .node_floors
            .iter()
            .zip(&used)
            .filter_map(|(&floor, &keep)| keep.then_some(floor))
            .collect();
        data.element_floors = kept.iter().map(|&i| data.element_floors[i]).collect();
        data.element_slabs = kept.iter().map(|&i| data.element_slabs[i]).collect();
        for w in &mut data.walls {
            w.bottom = w.bottom.map(remap);
            w.top = w.top.map(remap);
            for [a, b] in [w.bottom, w.top] {
                if a != b {
                    edges.insert(edge(a, b));
                }
            }
        }
        for b in &mut data.beams {
            b.nodes = b.nodes.map(remap);
            if b.nodes[0] != b.nodes[1] {
                edges.insert(edge(b.nodes[0], b.nodes[1]));
            }
        }
    }
    mesh.edges = edges.iter().copied().collect();
    mesh.constrained_edges = std::mem::take(&mut mesh.constrained_edges)
        .into_iter()
        .filter_map(|e| e.iter().all(|&n| used[n]).then(|| e.map(remap)))
        .filter(|e| edges.contains(e))
        .collect();
    mesh.constrained_nodes = std::mem::take(&mut mesh.constrained_nodes)
        .into_iter()
        .filter(|&n| used[n])
        .map(remap)
        .collect();
    for nodes in mesh.support_boundary.values_mut() {
        nodes.retain(|&n| used[n]);
        for n in nodes {
            *n = remap(*n);
        }
    }
    Ok(packed)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        building_mesh::{Beam, Data, Layout, Level, Weld},
        meshing::{Element, Node, Quad},
    };

    #[test]
    fn concave_quad_uses_the_interior_diagonal_and_crossed_quad_is_rejected() {
        let a = [0., 0., 0.];
        let b = [1., 1., 0.];
        let c = [3., 0., 0.];
        let d = [0., 3., 0.];
        assert!(!valid_quad([a, b, c, d]));
        assert_eq!(quad_split([a, b, c, d]), Some(QUAD_DIAGONALS[1]));
        assert_eq!(quad_split([a, [3., 3., 0.], c, d]), None);
    }

    #[test]
    fn a_face_collapsing_during_weld_leaves_no_new_nodes() {
        let mut weld = Weld::new();
        weld.add([1.4 * TOL, 0., 0.]);
        let a = [0.2 * TOL, 0., 0.];
        let b = [0.8 * TOL, 0., 0.];
        let c = [a[0], 0., 100.];
        assert!(weld.add_face([a, b, c], valid_triangle).is_err());
        assert_eq!(weld.xyz, [[1.4 * TOL, 0., 0.]]);
        assert_eq!(weld.find(a), None);
        assert_eq!(weld.find(b), Some(0));
        assert_eq!(weld.find(c), None);
    }

    #[test]
    fn packing_keeps_the_valid_half_of_a_collapsed_quad_and_spatial_references() {
        let points = [[0., 0.], [500., 0.], [0., 500.], [300., 0.], [500., 0.]];
        let mut mesh = Mesh {
            nodes: points
                .into_iter()
                .enumerate()
                .map(|(i, xy)| Node {
                    id: i as u64 + 1,
                    xy,
                })
                .collect(),
            elements: vec![
                Element {
                    id: 1,
                    nodes: [0, 3, 1],
                    supports: vec![],
                },
                Element {
                    id: 2,
                    nodes: [0, 1, 2],
                    supports: vec![],
                },
            ],
            quads: vec![Quad {
                id: 1,
                nodes: [0, 3, 1, 2],
                display_triangles: [1, 2],
                diagonal: [0, 1],
            }],
            constrained_nodes: BTreeSet::from([0, 3, 4]),
            constrained_edges: BTreeSet::from([[0, 1], [0, 3], [1, 4]]),
            support_boundary: [(7, vec![0, 3, 4])].into(),
            edges: vec![[0, 1], [0, 3], [1, 4]],
            building: Some(Box::new(Data {
                wall_mesh_step: 0.,
                wall_mesh: None,
                wall_quads: vec![],
                wall_quad_options: Default::default(),
                contacts: vec![],
                rigid_links: vec![],
                layout: Layout {
                    levels: vec![Level {
                        id: 1,
                        z: 0.,
                        top: 3000.,
                    }],
                    owners: Default::default(),
                    legacy_wall_extent: None,
                },
                z: vec![0., 0., 0., 0., 3000.],
                node_floors: vec![1, 1, 1, 1, 0],
                element_floors: vec![1, 1],
                element_slabs: vec![20, 21],
                walls: vec![],
                beams: vec![Beam {
                    roll_angle_rad: 0.,
                    id: 7,
                    nodes: [1, 4],
                }],
            })),
        };
        assert_eq!(
            pack(&mut mesh).unwrap(),
            Packed {
                removed_nodes: 1,
                removed_faces: 1,
                split_quads: 1
            }
        );
        assert_eq!(mesh.elements[0].id, 2);
        assert_eq!(mesh.elements[0].nodes, [0, 1, 2]);
        assert_eq!(mesh.nodes[3].id, 5);
        assert_eq!(mesh.constrained_edges, BTreeSet::from([[0, 1], [1, 3]]));
        assert_eq!(mesh.support_boundary[&7], [0, 3]);
        let data = mesh.building.as_ref().unwrap();
        assert_eq!(data.beams[0].nodes, [1, 3]);
        assert_eq!(data.element_slabs, [21]);
        assert_eq!(data.z, [0., 0., 0., 3000.]);
        data.validate(&mesh).unwrap();
        assert_eq!(pack(&mut mesh).unwrap(), Packed::default());
    }
}
