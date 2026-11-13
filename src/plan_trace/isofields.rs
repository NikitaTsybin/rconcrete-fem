//! Area-weighted corner recovery, only across shared non-barrier element edges.
use super::meshing::Mesh;
use std::collections::{BTreeMap, BTreeSet};
pub fn recover(
    mesh: &Mesh,
    values: &[f64],
    materials: &[[u64; 3]],
    barriers: &BTreeSet<[usize; 2]>,
) -> Vec<[f64; 3]> {
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let n = mesh.elements.len() * 3;
    let mut parent = (0..n).collect::<Vec<_>>();
    let mut edges = BTreeMap::<[usize; 2], (usize, [usize; 2])>::new();
    for (i, e) in mesh.elements.iter().enumerate() {
        for (a, b) in [(0, 1), (1, 2), (2, 0)] {
            let (key, corners) = if e.nodes[a] < e.nodes[b] {
                ([e.nodes[a], e.nodes[b]], [3 * i + a, 3 * i + b])
            } else {
                ([e.nodes[b], e.nodes[a]], [3 * i + b, 3 * i + a])
            };
            if let Some(&(other, previous)) = edges.get(&key) {
                if materials[i] == materials[other] && !barriers.contains(&key) {
                    for j in 0..2 {
                        let x = root(&mut parent, corners[j]);
                        let y = root(&mut parent, previous[j]);
                        parent[y] = x;
                    }
                }
            } else {
                edges.insert(key, (i, corners));
            }
        }
    }
    let display_ids = if mesh.quads.is_empty() {
        BTreeMap::new()
    } else {
        mesh.elements
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect::<BTreeMap<_, _>>()
    };
    let mut quad_weights = BTreeMap::<(usize, usize), f64>::new();
    for quad in &mesh.quads {
        let origin = mesh.nodes[quad.nodes[0]].xy;
        let area = (0..4)
            .map(|i| {
                let a = mesh.nodes[quad.nodes[i]].xy;
                let b = mesh.nodes[quad.nodes[(i + 1) % 4]].xy;
                (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])
            })
            .sum::<f64>()
            .abs()
            * 0.5;
        for &node in &quad.nodes {
            let count = quad
                .display_triangles
                .iter()
                .filter_map(|id| display_ids.get(id))
                .filter(|&&i| mesh.elements[i].nodes.contains(&node))
                .count()
                .max(1);
            for &id in &quad.display_triangles {
                if let Some(&i) = display_ids.get(&id) {
                    if mesh.elements[i].nodes.contains(&node) {
                        quad_weights.insert((i, node), area / count as f64);
                    }
                }
            }
        }
    }
    let mut sum = vec![0.; n];
    let mut weight = vec![0.; n];
    for (i, e) in mesh.elements.iter().enumerate() {
        let [a, b, c] = e.nodes.map(|j| mesh.nodes[j].xy);
        let area = ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5;
        for j in 0..3 {
            let r = root(&mut parent, 3 * i + j);
            let physical_weight = quad_weights.get(&(i, e.nodes[j])).copied().unwrap_or(area);
            sum[r] += physical_weight * values[i];
            weight[r] += physical_weight;
        }
    }
    (0..mesh.elements.len())
        .map(|i| {
            std::array::from_fn(|j| {
                let r = root(&mut parent, 3 * i + j);
                if weight[r] > 0. {
                    sum[r] / weight[r]
                } else {
                    values[i]
                }
            })
        })
        .collect()
}

pub fn quad_corner_values(
    mesh: &Mesh,
    quad: &super::meshing::Quad,
    values: &[[f64; 3]],
    ids: &BTreeMap<u64, usize>,
) -> Option<[f64; 4]> {
    let corners = quad.nodes.map(|node| {
        quad.display_triangles.iter().find_map(|id| {
            let &i = ids.get(id)?;
            let corner = mesh
                .elements
                .get(i)?
                .nodes
                .iter()
                .position(|n| *n == node)?;
            values.get(i).map(|v| v[corner])
        })
    });
    let [Some(a), Some(b), Some(c), Some(d)] = corners else {
        return None;
    };
    Some([a, b, c, d])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::meshing::{Element, Node};
    fn square() -> Mesh {
        Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![[0., 0.], [1., 0.], [1., 1.], [0., 1.]]
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
                    nodes: [0, 1, 2],
                    supports: vec![],
                },
                Element {
                    id: 2,
                    nodes: [0, 2, 3],
                    supports: vec![],
                },
            ],
            edges: vec![[0, 1], [1, 2], [0, 2], [2, 3], [0, 3]],
            support_boundary: BTreeMap::new(),
        }
    }
    #[test]
    fn shared_edge_smooths_but_support_and_material_boundaries_do_not() {
        let mesh = square();
        let tags = [[1, 2, 3]; 2];
        let v = recover(&mesh, &[2., 8.], &tags, &BTreeSet::new());
        assert_eq!(v, vec![[5., 2., 5.], [5., 5., 8.]]);
        let split = recover(&mesh, &[2., 8.], &tags, &BTreeSet::from([[0, 2]]));
        assert_eq!(split, vec![[2.; 3], [8.; 3]]);
        let split = recover(&mesh, &[2., 8.], &[[1, 2, 3], [4, 5, 6]], &BTreeSet::new());
        assert_eq!(split, vec![[2.; 3], [8.; 3]]);
    }
}
