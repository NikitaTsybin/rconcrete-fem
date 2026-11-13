//! Consistent Winkler matrices, integrated with the physical plate shape functions.
use super::*;
use crate::plan_trace::foundation;
pub(super) struct Spring {
    pub dofs: Vec<usize>,
    pub k: Vec<Vec<f64>>,
    pub pressure_cell: Option<(Vec<usize>, f64)>,
}
impl Spring {
    pub fn forces(&self, q: &[f64], out: &mut [f64]) {
        for (i, &a) in self.dofs.iter().enumerate() {
            for (j, &b) in self.dofs.iter().enumerate() {
                out[a] += self.k[i][j] * q[b];
            }
        }
    }
}
pub(super) fn enabled(all: &[Contour]) -> bool {
    all.iter().any(|c| c.kind == Kind::Slab && c.use_foundation)
        && all.iter().any(|c| c.kind == Kind::Foundation)
}
fn physical(mesh: &Mesh) -> Vec<(Vec<usize>, usize)> {
    let ids = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let mut owned = BTreeSet::new();
    let mut out = vec![];
    for q in &mesh.quads {
        let a = ids[&q.display_triangles[0]];
        owned.extend(q.display_triangles.map(|id| ids[&id]));
        out.push((q.nodes.to_vec(), a));
    }
    out.extend(
        mesh.elements
            .iter()
            .enumerate()
            .filter(|(i, _)| !owned.contains(i))
            .map(|(i, e)| (e.nodes.to_vec(), i)),
    );
    out
}
fn owner<'a>(
    mesh: &Mesh,
    all: &'a [Contour],
    nodes: &[usize],
    display: usize,
) -> Option<&'a Contour> {
    if let Some(data) = &mesh.building {
        let id = data.element_slabs[display];
        return all.iter().find(|c| c.id == id);
    }
    let p = std::array::from_fn(|a| {
        nodes.iter().map(|&n| mesh.nodes[n].xy[a]).sum::<f64>() / nodes.len() as f64
    });
    all.iter()
        .filter(|c| c.kind == Kind::Slab && super::super::geometry::inside(p, &c.points))
        .min_by(|a, b| {
            super::super::geometry::area(&a.points)
                .abs()
                .total_cmp(&super::super::geometry::area(&b.points).abs())
                .then_with(|| b.id.cmp(&a.id))
        })
}
fn fields_on_mesh(
    mesh: &Mesh,
    all: &[Contour],
    g: f64,
) -> Result<Vec<(f64, foundation::Field)>, String> {
    let layout = mesh.building.as_ref().map(|data| &data.layout);
    Ok(foundation::fields(all, g)?
        .into_iter()
        .filter_map(|field| Some((foundation::elevation(layout, &field.contour)?, field)))
        .collect())
}

fn sample<'a>(
    mesh: &Mesh,
    all: &[Contour],
    nodes: &[usize],
    display: usize,
    fields: &'a [(f64, foundation::Field)],
) -> Option<(&'a foundation::Field, f64)> {
    owner(mesh, all, nodes, display).filter(|c| c.use_foundation)?;
    let z = mesh.node_z(nodes[0]);
    let center = std::array::from_fn(|a| {
        nodes.iter().map(|&n| mesh.nodes[n].xy[a]).sum::<f64>() / nodes.len() as f64
    });
    fields
        .iter()
        .filter(|(elevation, _)| foundation::same_elevation(*elevation, z))
        .find_map(|(_, field)| field.at(center).map(|c1| (field, c1)))
}

fn add_horizontal(horizontal: &mut BTreeMap<usize, f64>, nodes: &[usize], stiffness: f64) {
    if stiffness > 0. {
        for &node in nodes {
            let value = horizontal.entry(node).or_default();
            *value = value.max(stiffness);
        }
    }
}
fn mass(xy: &[Point], quadratic: bool) -> Result<Vec<Vec<f64>>, String> {
    if xy.len() == 4 {
        let q = mitc4::Quad4::new(xy.try_into().unwrap())?;
        let mut k = vec![vec![0.; 4]; 4];
        for p in mitc4::GAUSS {
            let s = q.sample(p)?;
            for i in 0..4 {
                for j in 0..4 {
                    k[i][j] += s.n[i] * s.n[j] * s.det;
                }
            }
        }
        return Ok(k);
    }
    let area = Triangle::new(xy.try_into().unwrap())?.area;
    if !quadratic {
        return Ok((0..3)
            .map(|i| {
                (0..3)
                    .map(|j| area * if i == j { 1. / 6. } else { 1. / 12. })
                    .collect()
            })
            .collect());
    }
    let mut k = vec![vec![0.; 6]; 6];
    // Degree-four symmetric Dunavant rule; weights sum to one.
    for (a, b, w) in [
        (0.445948490915965, 0.108103018168070, 0.223381589678011),
        (0.091576213509771, 0.816847572980459, 0.109951743655322),
    ] {
        for l in [[a, a, b], [a, b, a], [b, a, a]] {
            let n = [
                l[0] * (2. * l[0] - 1.),
                l[1] * (2. * l[1] - 1.),
                l[2] * (2. * l[2] - 1.),
                4. * l[0] * l[1],
                4. * l[1] * l[2],
                4. * l[2] * l[0],
            ];
            for i in 0..6 {
                for j in 0..6 {
                    k[i][j] += area * w * n[i] * n[j];
                }
            }
        }
    }
    Ok(k)
}
pub(super) fn build(
    mesh: &Mesh,
    all: &[Contour],
    g: f64,
    shell: bool,
    p2: bool,
) -> Result<Vec<Spring>, String> {
    if !enabled(all) {
        return Ok(vec![]);
    }
    let fields = fields_on_mesh(mesh, all, g)?;
    let mut out = vec![];
    let mut horizontal = BTreeMap::<usize, f64>::new();
    let edges = mesh
        .edges
        .iter()
        .enumerate()
        .map(|(i, &[a, b])| ([a.min(b), a.max(b)], i))
        .collect::<BTreeMap<_, _>>();
    let display_ids = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let display_sets = mesh
        .quads
        .iter()
        .map(|q| {
            let indices = q.display_triangles.map(|id| display_ids[&id]);
            (indices[0], indices.to_vec())
        })
        .collect::<BTreeMap<_, _>>();
    for (nodes, display) in physical(mesh) {
        let Some((field, c1)) = sample(mesh, all, &nodes, display, &fields) else {
            continue;
        };
        let mut dofs = nodes
            .iter()
            .map(|n| {
                if p2 {
                    *n
                } else {
                    3 * n + if shell { 2 } else { 0 }
                }
            })
            .collect::<Vec<_>>();
        if p2 {
            for (a, b) in [(0, 1), (1, 2), (2, 0)] {
                dofs.push(
                    mesh.nodes.len() + edges[&[nodes[a].min(nodes[b]), nodes[a].max(nodes[b])]],
                );
            }
        }
        let xy_m = nodes
            .iter()
            .map(|&n| mesh.nodes[n].xy.map(|v| v / 1000.))
            .collect::<Vec<_>>();
        let mut k = mass(&xy_m, p2)?;
        for row in &mut k {
            for v in row {
                *v *= c1;
            }
        }
        let displays = display_sets
            .get(&display)
            .cloned()
            .unwrap_or_else(|| vec![display]);
        let area = if xy_m.len() == 4 {
            (Triangle::new([xy_m[0], xy_m[1], xy_m[2]])?.area
                + Triangle::new([xy_m[0], xy_m[2], xy_m[3]])?.area)
        } else {
            Triangle::new(xy_m.clone().try_into().unwrap())?.area
        };
        out.push(Spring {
            dofs,
            k,
            pressure_cell: Some((displays, area)),
        });
        if shell {
            add_horizontal(&mut horizontal, &nodes, field.horizontal);
        }
    }
    for (n, k) in horizontal {
        for a in 0..2 {
            out.push(Spring {
                dofs: vec![3 * n + a],
                k: vec![vec![k]],
                pressure_cell: None,
            });
        }
    }
    Ok(out)
}
pub(super) fn flat_diagonal(springs: &[Spring], map: &[usize], diagonal: &mut [f64]) {
    for s in springs {
        for (i, &d) in s.dofs.iter().enumerate() {
            if map[d] != usize::MAX {
                diagonal[map[d]] += s.k[i][i];
            }
        }
    }
}
pub(super) fn flat_triplets(
    springs: &[Spring],
    map: &[usize],
    scale: &[f64],
    ts: &mut Vec<Triplet<usize, usize, f64>>,
) {
    for s in springs {
        for (i, &a) in s.dofs.iter().enumerate() {
            for (j, &b) in s.dofs.iter().enumerate() {
                let (a, b) = (map[a], map[b]);
                if a != usize::MAX && b != usize::MAX && a <= b {
                    ts.push(Triplet::new(a, b, s.k[i][j] * scale[a] * scale[b]));
                }
            }
        }
    }
}
pub(super) fn force(springs: &[Spring], q: &[f64]) -> Vec<f64> {
    let mut f = vec![0.; q.len()];
    for s in springs {
        s.forces(q, &mut f);
    }
    f
}

pub(super) fn coefficients(
    mesh: &Mesh,
    all: &[Contour],
    g: f64,
    mut horizontal: Option<&mut BTreeMap<usize, f64>>,
) -> Result<Vec<f64>, String> {
    let fields = fields_on_mesh(mesh, all, g)?;
    let mut values = vec![f64::NAN; mesh.elements.len()];
    let by_id = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let mut quad_display = BTreeMap::new();
    for q in &mesh.quads {
        quad_display.insert(
            by_id[&q.display_triangles[0]],
            q.display_triangles.map(|id| by_id[&id]),
        );
    }
    for (nodes, display) in physical(mesh) {
        if let Some((field, c1)) = sample(mesh, all, &nodes, display, &fields) {
            if let Some(horizontal) = horizontal.as_deref_mut() {
                add_horizontal(horizontal, &nodes, field.horizontal);
            }
            values[display] = c1;
            if let Some(indices) = quad_display.get(&display) {
                for &i in indices {
                    values[i] = c1;
                }
            }
        }
    }
    Ok(values)
}

/// Integral mean pressure of each physical vertical Winkler cell, kN/m².
/// Sum of consistent nodal forces equals integral(C1*w), including P2 edge DOFs.
pub(super) fn pressures(springs: &[Spring], q: &[f64], count: usize) -> Vec<f64> {
    let mut values = vec![f64::NAN; count];
    for s in springs {
        let Some((displays, area)) = &s.pressure_cell else {
            continue;
        };
        let mut integral = 0.;
        for row in &s.k {
            for (j, &d) in s.dofs.iter().enumerate() {
                integral += row[j] * q[d];
            }
        }
        for &i in displays {
            values[i] = integral / area;
        }
    }
    values
}

#[cfg(test)]
mod pressure_tests {
    use super::*;
    #[test]
    fn quadratic_pressure_integrates_midpoint_dofs_and_ignores_xy_springs() {
        let mut k = mass(&[[0., 0.], [1., 0.], [0., 1.]], true).unwrap();
        for row in &mut k {
            for v in row {
                *v *= 1000.;
            }
        }
        let springs = vec![
            Spring {
                dofs: (0..6).collect(),
                k,
                pressure_cell: Some((vec![0, 1], 0.5)),
            },
            Spring {
                dofs: vec![0],
                k: vec![vec![1e9]],
                pressure_cell: None,
            },
        ];
        let q = [0., 0., 0., 0.003, 0.003, 0.003];
        let p = pressures(&springs, &q, 3);
        assert!((p[0] - 3.).abs() < 1e-10);
        assert_eq!(p[0], p[1]);
        assert!(p[2].is_nan());
    }
}
