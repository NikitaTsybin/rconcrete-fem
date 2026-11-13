//! Conforming convex four-node element topology from adjacent triangles.
//! Paired triangles remain rendering/search tessellation only. FEM uses MITC4.
use super::{
    geometry::Point,
    meshing::{Mesh, Quad},
};
use std::collections::{BTreeMap, BTreeSet};
#[derive(Clone, Copy, Debug, Default, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Options {
    pub backend: super::meshing::Backend,
    pub merge: bool,
    pub optimise: bool,
}
fn edge(a: usize, b: usize) -> [usize; 2] {
    [a.min(b), a.max(b)]
}
fn cross(a: Point, b: Point, c: Point) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}
fn quality(p: [Point; 4]) -> f64 {
    let mut score = 1_f64;
    let mut lengths = [0.; 4];
    let orientation = cross(p[0], p[1], p[2]).signum();
    for i in 0..4 {
        let a = p[(i + 3) % 4];
        let b = p[i];
        let c = p[(i + 1) % 4];
        let u = [a[0] - b[0], a[1] - b[1]];
        let v = [c[0] - b[0], c[1] - b[1]];
        let l = u[0].hypot(u[1]);
        let r = v[0].hypot(v[1]);
        if l <= 1e-6 || r <= 1e-6 || cross(a, b, c) * orientation <= 1e-8 {
            return 0.;
        }
        lengths[i] = r;
        score = score.min(1. - ((u[0] * v[0] + u[1] * v[1]) / (l * r)).abs());
    }
    let lo = lengths.into_iter().fold(f64::INFINITY, f64::min);
    let hi = lengths.into_iter().fold(0., f64::max);
    if hi > 4. * lo {
        return 0.;
    }
    score * (lo / hi).sqrt()
}
pub fn process(mut mesh: Mesh, options: Options) -> Result<Mesh, String> {
    if options.backend == super::meshing::Backend::Gmsh {
        super::limits::check(
            mesh.nodes.len(),
            mesh.elements.len().saturating_sub(mesh.quads.len()),
        )?;
        return Ok(mesh);
    }
    if !options.merge {
        super::limits::check(
            mesh.nodes.len(),
            mesh.elements.len().saturating_sub(mesh.quads.len()),
        )?;
        return Ok(mesh);
    }
    if options.optimise {
        smooth_triangles(&mut mesh);
    }
    let mut adjacency = BTreeMap::<[usize; 2], Vec<usize>>::new();
    for (i, t) in mesh.elements.iter().enumerate() {
        for k in 0..3 {
            adjacency
                .entry(edge(t.nodes[k], t.nodes[(k + 1) % 3]))
                .or_default()
                .push(i);
        }
    }
    let mut candidates = Vec::new();
    for (&shared, tris) in &adjacency {
        if tris.len() != 2 || mesh.constrained_edges.contains(&shared) {
            continue;
        }
        let a = &mesh.elements[tris[0]];
        let b = &mesh.elements[tris[1]];
        if a.supports != b.supports {
            continue;
        }
        let Some(&pa) = a.nodes.iter().find(|n| !shared.contains(n)) else {
            continue;
        };
        let Some(&pb) = b.nodes.iter().find(|n| !shared.contains(n)) else {
            continue;
        };
        if pa == pb {
            continue;
        }
        let mut nodes = [pa, shared[0], pb, shared[1]];
        if cross(
            mesh.nodes[nodes[0]].xy,
            mesh.nodes[nodes[1]].xy,
            mesh.nodes[nodes[2]].xy,
        ) < 0.
        {
            nodes.reverse();
        }
        let score = quality(nodes.map(|n| mesh.nodes[n].xy));
        // Reject concave, nearly triangular, or excessively stretched candidates.
        if score >= 0.10 {
            candidates.push((score, [tris[0], tris[1]], nodes, shared));
        }
    }
    candidates.sort_by(|a, b| b.0.total_cmp(&a.0).then(a.1.cmp(&b.1)));
    let mut neighbours = vec![Vec::<usize>::new(); mesh.elements.len()];
    let mut matching = vec![None; mesh.elements.len()];
    for (i, (_, pair, _, _)) in candidates.iter().enumerate() {
        for &t in pair {
            neighbours[t].push(i);
        }
        if pair.iter().all(|&t| matching[t].is_none()) {
            for &t in pair {
                matching[t] = Some(i);
            }
        }
    }
    // A locally best pair can strand two otherwise pairable neighbours. Repair
    // short augmenting paths, retaining convexity/constraint checks and a
    // stricter quality floor rather than forcing every triangle into a quad.
    for _ in 0..4 {
        let mut changed = false;
        for u in 0..matching.len() {
            if matching[u].is_some() {
                continue;
            }
            'search: for &a in &neighbours[u] {
                let v = *candidates[a].1.iter().find(|&&t| t != u).unwrap();
                let Some(old) = matching[v] else { continue };
                let w = *candidates[old].1.iter().find(|&&t| t != v).unwrap();
                for &b in &neighbours[w] {
                    let x = *candidates[b].1.iter().find(|&&t| t != w).unwrap();
                    if x == u || matching[x].is_some() {
                        continue;
                    }
                    if candidates[a].0.min(candidates[b].0) < 0.20
                        || candidates[a].0 + candidates[b].0 < candidates[old].0 * 0.45
                    {
                        continue;
                    }
                    matching[u] = Some(a);
                    matching[v] = Some(a);
                    matching[w] = Some(b);
                    matching[x] = Some(b);
                    changed = true;
                    break 'search;
                }
            }
        }
        if !changed {
            break;
        }
    }
    for i in matching.into_iter().flatten().collect::<BTreeSet<_>>() {
        let (_, pair, nodes, diagonal) = candidates[i];
        mesh.quads.push(Quad {
            id: mesh.elements[pair[0]].id,
            nodes,
            display_triangles: pair.map(|i| mesh.elements[i].id),
            diagonal,
        });
    }
    // Keep tessellation edges for legacy triangle-only paths; MITC4 has no diagonal DOFs.
    if options.optimise {
        optimise(&mut mesh);
    }
    super::limits::check(
        mesh.nodes.len(),
        mesh.elements.len().saturating_sub(mesh.quads.len()),
    )?;
    Ok(mesh)
}
/// Improve the free-node stars before pairing. In particular, triangle-only
/// transition patches around short internal constraints otherwise never reach
/// the quad optimiser. Never move a geometric constraint or load attachment.
fn smooth_triangles(mesh: &mut Mesh) {
    let mut incident = vec![Vec::new(); mesh.nodes.len()];
    let mut neighbours = vec![BTreeSet::new(); mesh.nodes.len()];
    for (i, t) in mesh.elements.iter().enumerate() {
        for &n in &t.nodes {
            incident[n].push(i);
            neighbours[n].extend(t.nodes.into_iter().filter(|&j| j != n));
        }
    }
    let score = |mesh: &Mesh, i: usize| {
        let p = mesh.elements[i].nodes.map(|n| mesh.nodes[n].xy);
        let sum = (0..3)
            .map(|j| {
                let a = p[j];
                let b = p[(j + 1) % 3];
                (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
            })
            .sum::<f64>();
        2. * 3_f64.sqrt() * cross(p[0], p[1], p[2]).abs() / sum.max(1e-30)
    };
    for _ in 0..6 {
        let mut changed = false;
        for n in 0..mesh.nodes.len() {
            if mesh.constrained_nodes.contains(&n) || neighbours[n].is_empty() {
                continue;
            }
            let before = mesh.nodes[n].xy;
            let mean = neighbours[n].iter().fold([0., 0.], |s, &j| {
                [s[0] + mesh.nodes[j].xy[0], s[1] + mesh.nodes[j].xy[1]]
            });
            let old = incident[n]
                .iter()
                .map(|&i| score(mesh, i))
                .collect::<Vec<_>>();
            let areas = incident[n]
                .iter()
                .map(|&i| {
                    let p = mesh.elements[i].nodes.map(|j| mesh.nodes[j].xy);
                    cross(p[0], p[1], p[2])
                })
                .collect::<Vec<_>>();
            mesh.nodes[n].xy = [
                (before[0] + mean[0] / neighbours[n].len() as f64) * 0.5,
                (before[1] + mean[1] / neighbours[n].len() as f64) * 0.5,
            ];
            let valid = incident[n].iter().zip(&areas).all(|(&i, &area)| {
                let p = mesh.elements[i].nodes.map(|j| mesh.nodes[j].xy);
                cross(p[0], p[1], p[2]) * area.signum() > area.abs() * 0.05
            });
            let new = incident[n]
                .iter()
                .map(|&i| score(mesh, i))
                .collect::<Vec<_>>();
            let min = |v: &[f64]| v.iter().copied().fold(1_f64, f64::min);
            if valid
                && min(&new) >= min(&old) - 1e-12
                && new.iter().sum::<f64>() > old.iter().sum::<f64>() + 1e-8
            {
                changed = true;
            } else {
                mesh.nodes[n].xy = before;
            }
        }
        if !changed {
            break;
        }
    }
}
fn optimise(mesh: &mut Mesh) {
    let n = mesh.nodes.len();
    let mut neighbours = vec![BTreeSet::new(); n];
    let mut incident_tri = vec![Vec::new(); n];
    let mut incident_quad = vec![Vec::new(); n];
    for [a, b] in mesh.visible_edges() {
        neighbours[a].insert(b);
        neighbours[b].insert(a);
    }
    for (i, t) in mesh.elements.iter().enumerate() {
        for &node in &t.nodes {
            incident_tri[node].push(i);
        }
    }
    for (i, q) in mesh.quads.iter().enumerate() {
        for &node in &q.nodes {
            incident_quad[node].push(i);
        }
    }
    for _ in 0..8 {
        let mut changed = false;
        for node in 0..n {
            if mesh.constrained_nodes.contains(&node)
                || incident_quad[node].is_empty()
                || neighbours[node].is_empty()
            {
                continue;
            }
            let before = mesh.nodes[node].xy;
            let mean = neighbours[node].iter().fold([0., 0.], |s, &j| {
                [s[0] + mesh.nodes[j].xy[0], s[1] + mesh.nodes[j].xy[1]]
            });
            let target = [
                (before[0] + mean[0] / neighbours[node].len() as f64) * 0.5,
                (before[1] + mean[1] / neighbours[node].len() as f64) * 0.5,
            ];
            if target.iter().any(|v| !v.is_finite()) {
                continue;
            }
            let old = incident_quad[node]
                .iter()
                .map(|&i| quality(mesh.quads[i].nodes.map(|j| mesh.nodes[j].xy)))
                .collect::<Vec<_>>();
            let areas = incident_tri[node]
                .iter()
                .map(|&i| {
                    let p = mesh.elements[i].nodes.map(|j| mesh.nodes[j].xy);
                    cross(p[0], p[1], p[2])
                })
                .collect::<Vec<_>>();
            mesh.nodes[node].xy = target;
            let valid = incident_tri[node].iter().zip(&areas).all(|(&i, &area)| {
                let p = mesh.elements[i].nodes.map(|j| mesh.nodes[j].xy);
                cross(p[0], p[1], p[2]) * area.signum() > area.abs() * 0.05
            });
            let new = incident_quad[node]
                .iter()
                .map(|&i| quality(mesh.quads[i].nodes.map(|j| mesh.nodes[j].xy)))
                .collect::<Vec<_>>();
            let min = |v: &[f64]| v.iter().copied().fold(1_f64, f64::min);
            if valid
                && min(&new) >= min(&old) - 1e-12
                && new.iter().sum::<f64>() > old.iter().sum::<f64>() + 1e-8
            {
                changed = true;
            } else {
                mesh.nodes[node].xy = before;
            }
        }
        if !changed {
            break;
        }
    }
}

#[cfg(test)]
mod pairing_regression {
    use super::*;
    #[test]
    fn pairing_reconsiders_an_early_square_for_two_valid_neighbours() {
        let points = [
            [0., 0.],
            [1000., 0.],
            [2000., 0.],
            [-1000., 1000.],
            [0., 1000.],
            [1000., 1000.],
        ];
        let mesh = Mesh {
            building: None,
            nodes: points
                .into_iter()
                .enumerate()
                .map(|(i, xy)| super::super::meshing::Node {
                    id: i as u64 + 1,
                    xy,
                })
                .collect(),
            elements: [[0, 1, 4], [0, 4, 3], [1, 2, 5], [1, 5, 4]]
                .into_iter()
                .enumerate()
                .map(|(i, nodes)| super::super::meshing::Element {
                    id: i as u64 + 1,
                    nodes,
                    supports: vec![],
                })
                .collect(),
            quads: vec![],
            edges: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            support_boundary: Default::default(),
        };
        let result = process(
            mesh,
            Options {
                backend: Default::default(),
                merge: true,
                optimise: false,
            },
        )
        .unwrap();
        assert_eq!(result.quads.len(), 2);
        let ids = result
            .quads
            .iter()
            .flat_map(|q| q.display_triangles)
            .collect::<BTreeSet<_>>();
        assert_eq!(ids.len(), 4);
    }
}
