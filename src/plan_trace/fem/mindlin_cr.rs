//! Reissner–Mindlin: continuous P2 w / nonconforming CR1 rotations.
//! Independent implementation from the variational form; no reduced integration.
use super::*;
const EDGES: [(usize, usize); 3] = [(0, 1), (1, 2), (2, 0)];
const GAUSS: [[f64; 3]; 3] = [
    [2. / 3., 1. / 6., 1. / 6.],
    [1. / 6., 2. / 3., 1. / 6.],
    [1. / 6., 1. / 6., 2. / 3.],
];
fn p2(l: [f64; 3]) -> [f64; 6] {
    [
        l[0] * (2. * l[0] - 1.),
        l[1] * (2. * l[1] - 1.),
        l[2] * (2. * l[2] - 1.),
        4. * l[0] * l[1],
        4. * l[1] * l[2],
        4. * l[2] * l[0],
    ]
}
fn curvature(t: &Triangle) -> [[f64; 12]; 3] {
    let g = t.gradients();
    let mut b = [[0.; 12]; 3];
    for (e, opposite) in [2, 0, 1].into_iter().enumerate() {
        let d = g[opposite].map(|v| -2. * v);
        b[0][6 + 2 * e] = d[0];
        b[1][7 + 2 * e] = d[1];
        b[2][6 + 2 * e] = d[1];
        b[2][7 + 2 * e] = d[0];
    }
    b
}
fn shear_matrix(t: &Triangle, l: [f64; 3]) -> [[f64; 12]; 2] {
    let g = t.gradients();
    let mut b = [[0.; 12]; 2];
    for i in 0..3 {
        for a in 0..2 {
            b[a][i] = (4. * l[i] - 1.) * g[i][a];
        }
    }
    for (e, (i, j)) in EDGES.into_iter().enumerate() {
        for a in 0..2 {
            b[a][3 + e] = 4. * (l[j] * g[i][a] + l[i] * g[j][a]);
        }
        let cr = 1. - 2. * l[[2, 0, 1][e]];
        b[0][6 + 2 * e] = -cr;
        b[1][7 + 2 * e] = -cr;
    }
    b
}
fn shear_rigidity(m: Material) -> f64 {
    5. / 6. * m.e_mpa * 1000. / (2. * (1. + m.nu)) * m.thickness_mm / 1000.
}
fn stiffness(t: &Triangle, m: Material) -> [[f64; 12]; 12] {
    let b = curvature(t);
    let d = m.d();
    let mut k = [[0.; 12]; 12];
    for i in 0..12 {
        for j in 0..12 {
            for a in 0..3 {
                for c in 0..3 {
                    k[i][j] += t.area * b[a][i] * d[a][c] * b[c][j];
                }
            }
        }
    }
    let s = shear_rigidity(m);
    for l in GAUSS {
        let b = shear_matrix(t, l);
        for i in 0..12 {
            for j in 0..12 {
                for a in 0..2 {
                    k[i][j] += t.area / 3. * s * b[a][i] * b[a][j];
                }
            }
        }
    }
    k
}
fn recover(t: &Triangle, m: Material, q: [f64; 12]) -> ([f64; 3], [f64; 2]) {
    recover_at(t, m, q, [1. / 3.; 3])
}
fn recover_at(t: &Triangle, m: Material, q: [f64; 12], l: [f64; 3]) -> ([f64; 3], [f64; 2]) {
    let b = curvature(t);
    let d = m.d();
    let strain = b.map(|row| (0..12).map(|i| row[i] * q[i]).sum::<f64>());
    let moments = d.map(|row| -(0..3).map(|i| row[i] * strain[i]).sum::<f64>());
    let shear = shear_matrix(t, l)
        .map(|row| shear_rigidity(m) * (0..12).map(|i| row[i] * q[i]).sum::<f64>());
    (moments, shear)
}
fn scatter(c: &Cell, dofs: [usize; 12], p: Point, force: f64, f: &mut [f64]) {
    let n = p2(c.triangle.barycentric(p));
    for j in 0..6 {
        f[dofs[j]] += force * n[j];
        super::cut_forces::load(dofs[j], force * n[j]);
    }
}
fn point(
    cells: &[Cell],
    dofs: &[[usize; 12]],
    index: &Index,
    p: Point,
    force: f64,
    f: &mut [f64],
) -> Result<usize, String> {
    point_optional(cells, dofs, index, p, force, f).ok_or_else(|| {
        format!(
            "Нагрузка вне плиты / в отверстии: X={:.1}, Y={:.1} мм",
            p[0] * 1000.,
            p[1] * 1000.
        )
    })
}
fn point_optional(
    cells: &[Cell],
    dofs: &[[usize; 12]],
    index: &Index,
    p: Point,
    force: f64,
    f: &mut [f64],
) -> Option<usize> {
    for i in index.query(
        [p[0] - 1e-9, p[1] - 1e-9, p[0] + 1e-9, p[1] + 1e-9],
        cells.len(),
    ) {
        if cells[i]
            .triangle
            .barycentric(p)
            .iter()
            .all(|v| *v >= -1e-8 && *v <= 1. + 1e-8)
        {
            super::cut_forces::active(i);
            scatter(&cells[i], dofs[i], p, force, f);
            return Some(i);
        }
    }
    None
}

fn loads(
    cells: &[Cell],
    dofs: &[[usize; 12]],
    index: &Index,
    all: &[Contour],
    case: u64,
    g: f64,
    step: f64,
    ndof: usize,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    let mut f = vec![0.; ndof];
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
            return Err("Некорректная величина нагрузки".into());
        }
        if value == 0. && c.kind != Kind::LoadArea {
            continue;
        }
        let points = c
            .points
            .iter()
            .map(|p| p.map(|x| x / 1000.))
            .collect::<Vec<_>>();
        match c.kind {
            Kind::LoadPoint => {
                point(
                    cells,
                    dofs,
                    index,
                    *points.first().ok_or("Пустая нагрузка")?,
                    value,
                    &mut f,
                )?;
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
                    super::cut_forces::active(i);
                    // Three-point quadrature integrates P2 over the FULL element.
                    for l in GAUSS {
                        let p = std::array::from_fn(|axis| {
                            (0..3).map(|j| cell.triangle.xy[j][axis] * l[j]).sum()
                        });
                        scatter(cell, dofs[i], p, force / 3., &mut f);
                    }
                }
            }
            Kind::LoadLine => {
                let segments = points.windows(2).map(|p| [p[0], p[1]]).chain(
                    if c.closed() && points.len() > 1 {
                        Some([*points.last().unwrap(), points[0]])
                    } else {
                        None
                    },
                );
                for [a, b] in segments {
                    let len = (b[0] - a[0]).hypot(b[1] - a[1]);
                    let n = (len / step).ceil().max(1.) as usize;
                    if n > 1_000_000 {
                        return Err("Слишком много точек нагрузки".into());
                    }
                    for j in 0..n {
                        let t = (j as f64 + 0.5) / n as f64;
                        let _ = point_optional(
                            cells,
                            dofs,
                            index,
                            [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])],
                            value * len / n as f64,
                            &mut f,
                        );
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
fn on_segment(p: Point, a: Point, b: Point) -> bool {
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let l2 = dx * dx + dy * dy;
    if l2 < 1e-16 {
        return false;
    }
    let t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l2;
    (-1e-8..=1. + 1e-8).contains(&t)
        && ((p[0] - a[0]) * dy - (p[1] - a[1]) * dx).abs() / l2.sqrt() < 1e-5
}
pub(super) fn solve(
    mesh: &Mesh,
    cells: &[Cell],
    all: &[Contour],
    cases: &[u64],
    self_weight: &BTreeSet<u64>,
    g: f64,
    size: f64,
) -> Result<Results, String> {
    let nv = mesh.nodes.len();
    let ne = mesh.edges.len();
    let nw = nv + ne;
    let ndof = nw + 2 * ne;
    let key = |a: usize, b: usize| if a < b { [a, b] } else { [b, a] };
    let edge_ids = mesh
        .edges
        .iter()
        .enumerate()
        .map(|(i, &[a, b])| (key(a, b), i))
        .collect::<BTreeMap<_, _>>();
    let dofs = cells
        .iter()
        .map(|c| {
            let mut d = [0; 12];
            d[..3].copy_from_slice(&c.nodes);
            for (e, (a, b)) in EDGES.into_iter().enumerate() {
                let i = *edge_ids
                    .get(&key(c.nodes[a], c.nodes[b]))
                    .ok_or("КЭ отсутствует в списке рёбер")?;
                d[3 + e] = nv + i;
                d[6 + 2 * e] = nw + 2 * i;
                d[7 + 2 * e] = nw + 2 * i + 1;
            }
            Ok(d)
        })
        .collect::<Result<Vec<_>, String>>()?;
    let fixed_vertices = mesh
        .support_boundary
        .values()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>();
    let springs = foundation::build(mesh, all, g, false, true)?;
    if springs.is_empty() {
        check_supports(mesh, &fixed_vertices)?;
    }
    let mut fixed = fixed_vertices.clone();
    let mut support_segments = vec![];
    for c in all.iter().filter(|c| c.kind == Kind::Support) {
        let mut points = c.mesh_boundary(size)?;
        if c.closed() {
            points.push(points[0]);
        }
        support_segments.extend(points.windows(2).map(|p| [p[0], p[1]]));
    }
    for (i, &[a, b]) in mesh.edges.iter().enumerate() {
        if !fixed_vertices.contains(&a) || !fixed_vertices.contains(&b) {
            continue;
        }
        let mid = std::array::from_fn(|j| (mesh.nodes[a].xy[j] + mesh.nodes[b].xy[j]) * 0.5);
        if support_segments.iter().any(|&[p, q]| {
            on_segment(mesh.nodes[a].xy, p, q)
                && on_segment(mesh.nodes[b].xy, p, q)
                && on_segment(mid, p, q)
        }) {
            fixed.insert(nv + i);
        }
    }
    let mut map = vec![usize::MAX; ndof];
    let mut free = 0;
    for i in 0..ndof {
        if !fixed.contains(&i) {
            map[i] = free;
            free += 1;
        }
    }
    let mut diagonal = vec![0.; free];
    for (ci, (c, d)) in cells.iter().zip(&dofs).enumerate() {
        let k = stiffness(&c.triangle, c.material);
        for i in 0..12 {
            let a = map[d[i]];
            if a != usize::MAX {
                diagonal[a] += k[i][i];
            }
        }
    }
    foundation::flat_diagonal(&springs, &map, &mut diagonal);
    if diagonal.iter().any(|v| !v.is_finite() || *v <= 0.) {
        return Err("Некорректная жесткость P2/CR".into());
    }
    let scale = diagonal.iter().map(|v| 1. / v.sqrt()).collect::<Vec<_>>();
    crate::plan_trace::fem::progress::stage("Сборка матрицы жёсткости элементов");
    crate::plan_trace::fem::progress::check_cancel()?;
    let mut triplets = Vec::with_capacity(cells.len() * 78);
    for (ci, (c, d)) in cells.iter().zip(&dofs).enumerate() {
        let k = stiffness(&c.triangle, c.material);
        for i in 0..12 {
            let a = map[d[i]];
            if a == usize::MAX {
                continue;
            }
            for j in 0..12 {
                let b = map[d[j]];
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
        .map_err(|e| format!("Сборка P2/CR: {e:?}"))?;
    drop(triplets);
    crate::plan_trace::fem::progress::stage("Разложение матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    let factor = matrix
        .sp_cholesky(Side::Upper)
        .map_err(|e| format!("P2/CR: проверьте закрепления/связность: {e:?}"))?;
    let index = Index::new(cells, size / 1000.);
    let step = super::line_load_step_m(all, size);
    let results = super::parallel_cases::map(
        cases,
        free.max(mesh.nodes.len() * 3).max(cells.len() * 12),
        |&id| {
            crate::plan_trace::fem::progress::stage(&format!(
                "Сборка вектора нагрузок: загружение {id}"
            ));
            crate::plan_trace::fem::progress::check_cancel()?;
            super::cut_forces::begin(all, cells.len());
            let (mut f, mut loads_kn_m2) = loads(cells, &dofs, &index, all, id, g, step, ndof)?;
            if self_weight.contains(&id) {
                for (i, c) in cells.iter().enumerate() {
                    let q = c.material.self_weight_kn_m2(g);
                    loads_kn_m2[i] += q;
                    super::cut_forces::active(i);
                    for l in GAUSS {
                        let p = std::array::from_fn(|a| {
                            (0..3).map(|j| c.triangle.xy[j][a] * l[j]).sum()
                        });
                        scatter(c, dofs[i], p, q * c.triangle.area / 3., &mut f);
                    }
                }
            }
            #[cfg(test)]
            super::cut_forces::audit_rhs(&f);
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
                return Err("P2/CR: нечисловое решение".into());
            }
            let mut cuts = super::cut_forces::Collector::new(all);
            crate::plan_trace::fem::progress::stage("Восстановление усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut residual = f.iter().map(|v| -v).collect::<Vec<_>>();
            let mut moments = vec![];
            let mut shears = vec![];
            for (ci, (c, d)) in cells.iter().zip(&dofs).enumerate() {
                let local = d.map(|i| q[i]);
                let k = stiffness(&c.triangle, c.material);
                for i in 0..12 {
                    for j in 0..12 {
                        residual[d[i]] += k[i][j] * local[j];
                    }
                }
                if cuts.enabled() {
                    let values: Vec<_> = (0..12)
                        .map(|i| {
                            super::cut_forces::net(
                                ci,
                                d[i],
                                (0..12).map(|j| k[i][j] * local[j]).sum(),
                            )
                        })
                        .collect();
                    let poly: Vec<_> = c.nodes.iter().map(|&n| mesh.nodes[n].xy).collect();
                    let mut actions: Vec<_> = c
                        .nodes
                        .iter()
                        .enumerate()
                        .map(|(i, &n)| (mesh.nodes[n].xy, [values[i], 0., 0.]))
                        .collect();
                    for (e, (a, b)) in EDGES.into_iter().enumerate() {
                        let pa = mesh.nodes[c.nodes[a]].xy;
                        let pb = mesh.nodes[c.nodes[b]].xy;
                        actions.push((
                            [(pa[0] + pb[0]) * 0.5, (pa[1] + pb[1]) * 0.5],
                            [values[3 + e], -values[7 + 2 * e], values[6 + 2 * e]],
                        ));
                    }
                    cuts.add(&poly, actions);
                }
                let (m, s) = recover(&c.triangle, c.material, local);
                if cuts.enabled() {
                    cuts.field(m, s);
                    cuts.quadratic();
                    cuts.direct_fields(
                        |p| {
                            Some(recover_at(
                                &c.triangle,
                                c.material,
                                local,
                                c.triangle.barycentric(p.map(|v| v / 1000.)),
                            ))
                        },
                        false,
                    );
                    cuts.quadratic();
                }
                moments.push(m);
                shears.push(s);
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
                return Err(format!("P2/CR: невязка {relative_residual:.3e}"));
            }
            let applied_kn = f[..nw].iter().sum::<f64>();
            let mut reaction_kn = vec![0.; nv];
            for &i in &fixed {
                if i < nv {
                    reaction_kn[i] += residual[i];
                } else {
                    let [a, b] = mesh.edges[i - nv];
                    reaction_kn[a] += residual[i] * 0.5;
                    reaction_kn[b] += residual[i] * 0.5;
                }
            }
            for i in 0..nw {
                if !fixed.contains(&i) {
                    if i < nv {
                        reaction_kn[i] -= ground[i];
                    } else {
                        let [a, b] = mesh.edges[i - nv];
                        reaction_kn[a] -= ground[i] * 0.5;
                        reaction_kn[b] -= ground[i] * 0.5;
                    }
                }
            }
            let mut balance_m_knm = [0.; 2];
            for i in 0..nw {
                let xy = if i < nv {
                    mesh.nodes[i].xy
                } else {
                    let [a, b] = mesh.edges[i - nv];
                    std::array::from_fn(|j| (mesh.nodes[a].xy[j] + mesh.nodes[b].xy[j]) * 0.5)
                };
                for a in 0..2 {
                    balance_m_knm[a] += (f[i]
                        + if fixed.contains(&i) {
                            residual[i]
                        } else {
                            -ground[i]
                        })
                        * xy[a]
                        / 1000.;
                }
            }
            let mut rotations_rad = vec![[0.; 3]; nv];
            let mut counts = vec![0.; nv];
            for (cell, dofs) in cells.iter().zip(&dofs) {
                for vertex in 0..3 {
                    let mut slope = [0.; 2];
                    for (edge, (a, b)) in EDGES.into_iter().enumerate() {
                        let sign = if a == vertex || b == vertex { 1. } else { -1. };
                        for j in 0..2 {
                            slope[j] += sign * q[dofs[6 + 2 * edge + j]];
                        }
                    }
                    let node = cell.nodes[vertex];
                    rotations_rad[node][0] -= slope[1];
                    rotations_rad[node][1] += slope[0];
                    counts[node] += 1.;
                }
            }
            for (rotation, count) in rotations_rad.iter_mut().zip(counts) {
                if count > 0. {
                    for x in rotation {
                        *x /= count;
                    }
                }
            }
            Ok(CaseResult {
                punching: cuts.finish(),
                rotations_rad,
                shell: None,
                id,
                w_mm: q[..nv].iter().map(|v| v * 1000.).collect(),
                moments,
                shears,
                loads_kn_m2,
                foundation_pressure_kn_m2: foundation::pressures(&springs, &q, mesh.elements.len()),
                balance_kn: applied_kn + reaction_kn.iter().sum::<f64>(),
                reaction_kn,
                applied_kn,
                balance_m_knm,
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
    use super::*;
    fn quadratic(x: f64, y: f64) -> [f64; 3] {
        [
            0.001 * (x * x + 3. * x * y + 2. * y * y),
            0.001 * (2. * x + 3. * y),
            0.001 * (3. * x + 4. * y),
        ]
    }
    fn patch(t: &Triangle, linear: bool) -> [f64; 12] {
        let mut q = [0.; 12];
        for i in 0..3 {
            let [x, y] = t.xy[i];
            q[i] = if linear {
                1. + 2. * x - 3. * y
            } else {
                quadratic(x, y)[0]
            };
        }
        for (e, (i, j)) in EDGES.into_iter().enumerate() {
            let [x, y] = std::array::from_fn(|a| (t.xy[i][a] + t.xy[j][a]) * 0.5);
            let v = if linear {
                [1. + 2. * x - 3. * y, 2., -3.]
            } else {
                quadratic(x, y)
            };
            q[3 + e] = v[0];
            q[6 + 2 * e] = v[1];
            q[7 + 2 * e] = v[2];
        }
        q
    }
    #[test]
    fn rigid_and_quadratic_patch_has_zero_shear_for_both_orientations() {
        for xy in [
            [[0., 0.], [2., 0.], [0.3, 1.4]],
            [[0., 0.], [0.3, 1.4], [2., 0.]],
        ] {
            let t = Triangle::new(xy).unwrap();
            for linear in [true, false] {
                let q = patch(&t, linear);
                for l in GAUSS {
                    for row in shear_matrix(&t, l) {
                        assert!((0..12).map(|i| row[i] * q[i]).sum::<f64>().abs() < 1e-12);
                    }
                }
                let (m, s) = recover(&t, Material::default(), q);
                assert!(s.iter().all(|v| v.abs() < 1e-8));
                if linear {
                    assert!(m.iter().all(|v| v.abs() < 1e-7));
                } else {
                    let d = Material::default().d();
                    for a in 0..3 {
                        let expected = -(0..3)
                            .map(|j| d[a][j] * [0.002, 0.004, 0.006][j])
                            .sum::<f64>();
                        assert!((m[a] - expected).abs() < 1e-7);
                    }
                }
            }
        }
    }
    #[test]
    fn consistent_p2_forces_preserve_resultant_and_application_point() {
        let xy = [[0., 0.], [2., 0.], [0., 2.]];
        let t = Triangle::new(xy).unwrap();
        let p = [0.3, 0.7];
        let n = p2(t.barycentric(p));
        let locations = [xy[0], xy[1], xy[2], [1., 0.], [1., 1.], [0., 1.]];
        assert!((n.iter().sum::<f64>() - 1.).abs() < 1e-12);
        for a in 0..2 {
            assert!(((0..6).map(|i| n[i] * locations[i][a]).sum::<f64>() - p[a]).abs() < 1e-12);
        }
    }
    #[test]
    fn full_area_pressure_integrates_p2_exactly() {
        let xy = [[0., 0.], [2., 0.], [0., 2.]];
        let cells = vec![Cell {
            triangle: Triangle::new(xy).unwrap(),
            nodes: [0, 1, 2],
            material: Material::default(),
            bbox: bbox(&xy),
        }];
        let dofs = [std::array::from_fn(|i| i)];
        let index = Index::new(&cells, 1.);
        let c = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 1,
            kind: Kind::LoadArea,
            grid: None,
            support_open: false,
            parent: None,
            refinement: None,
            curves: vec![],
            points: vec![[0., 0.], [2000., 0.], [0., 2000.]],
            load: Some(super::super::super::loads::LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                name: String::new(),
                mesh_constraint: false,
                case_id: 1,
                magnitude: super::super::super::loads::Magnitude {
                    value: 1.,
                    units: ForceUnits::KiloNewton,
                },
                closed: true,
            }),
        };
        let (f, pressure) = loads(&cells, &dofs, &index, &[c.clone()], 1, 10., 0.5, 12).unwrap();
        let mut partial = c;
        partial.points = vec![[0., 0.], [100., 0.], [0., 100.]];
        let (partial_f, partial_pressure) =
            loads(&cells, &dofs, &index, &[partial], 1, 10., 0.5, 12).unwrap();
        assert_eq!(partial_f, f);
        assert_eq!(partial_pressure, pressure);
        assert!((pressure[0] * cells[0].triangle.area - 2.).abs() < 1e-12);
        assert!(f[..3].iter().all(|v| v.abs() < 1e-12));
        assert!(f[3..6].iter().all(|v| (*v - 2. / 3.).abs() < 1e-12));
        assert!((f.iter().sum::<f64>() - 2.).abs() < 1e-12);
        let mut point = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 2,
            kind: Kind::LoadPoint,
            grid: None,
            support_open: false,
            parent: None,
            refinement: None,
            curves: vec![],
            points: vec![[100., 100.]],
            load: Some(super::super::super::loads::LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                name: String::new(),
                mesh_constraint: false,
                case_id: 1,
                magnitude: super::super::super::loads::Magnitude {
                    value: 3.,
                    units: ForceUnits::KiloNewton,
                },
                closed: false,
            }),
        };
        let (f, p) = loads(&cells, &dofs, &index, &[point.clone()], 1, 10., 0.5, 12).unwrap();
        assert!((f.iter().sum::<f64>() - 3.).abs() < 1e-12);
        assert_eq!(p, vec![0.]);
        point.kind = Kind::LoadLine;
        point.points.push([1000., 100.]);
        let (f, p) = loads(&cells, &dofs, &index, &[point], 1, 10., 0.5, 12).unwrap();
        assert!((f.iter().sum::<f64>() - 2.7).abs() < 1e-12);
        assert_eq!(p, vec![0.]);
    }
}
