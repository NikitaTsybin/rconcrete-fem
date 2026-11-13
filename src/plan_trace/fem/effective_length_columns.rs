//! Spatial stability of a prismatic column under unit axial compression.
use super::*;

#[derive(Clone, Copy, Debug, Eq, PartialEq, serde::Serialize, serde::Deserialize)]
pub(super) enum ModeKind {
    Y,
    Z,
    CoupledBending,
    BendingTorsion,
    Torsion,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub(super) struct ColumnMode {
    pub kind: ModeKind,
    pub critical_kn: f64,
    pub effective_m: Option<f64>,
    pub bending_y_fraction: f64,
    pub bending_z_fraction: f64,
    pub torsion_fraction: f64,
    pub sway_fraction: f64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub(super) struct ColumnResult {
    pub id: u64,
    pub length_m: f64,
    pub bottom_m: f64,
    pub modes: Vec<ColumnMode>,
    pub diagnostic: Option<String>,
}

struct Column {
    section: super::beam::Section,
    stations: Vec<f64>,
    rows: Vec<Row>,
    old: Mat<f64>,
}

pub(super) fn compute(
    scene: &Scene,
    beams: &[BeamCell],
    all: &[Contour],
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    free: usize,
    direct_k: Option<&Mat<f64>>,
    solve: impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<ColumnResult>, String> {
    let selected = all
        .iter()
        .filter(|c| c.kind == Kind::Column && c.calculate_effective_length)
        .map(|c| (c.id, Vec::new()))
        .collect::<BTreeMap<_, _>>();
    let mut groups = selected;
    for beam in beams {
        if let Some(group) = groups.get_mut(&beam.geometry.id) {
            group.push(beam);
        }
    }
    let mut scales_by_dof = vec![1.; free];
    for (&dof, &weight) in map.iter().zip(scale) {
        if dof != usize::MAX {
            scales_by_dof[dof] = weight;
        }
    }
    let mut out = Vec::with_capacity(groups.len());
    for (id, group) in groups {
        super::super::progress::check_cancel()?;
        let mut result = ColumnResult {
            id,
            length_m: 0.,
            bottom_m: 0.,
            modes: Vec::new(),
            diagnostic: None,
        };
        match Column::from_beams(scene, &group, projection, map, scale).and_then(|column| {
            result.length_m = *column.stations.last().unwrap();
            result.bottom_m = group
                .iter()
                .flat_map(|beam| beam.geometry.nodes)
                .map(|node| scene.xyz[node][2] / 1000.)
                .fold(f64::INFINITY, f64::min);
            let compliance = compliance(&column.rows, free, &solve)?;
            let (boundary, note) = boundary_stiffness(
                &column.rows,
                &scales_by_dof,
                compliance,
                &column.old,
                direct_k,
            )?;
            result.diagnostic = note;
            column.solve(&boundary)
        }) {
            Ok(modes) => result.modes = modes,
            Err(error) => result.diagnostic = Some(error),
        }
        super::super::progress::check_cancel()?;
        out.push(result);
    }
    Ok(out)
}

impl Column {
    fn from_beams(
        scene: &Scene,
        beams: &[&BeamCell],
        projection: &[Row],
        map: &[usize],
        scale: &[f64],
    ) -> Result<Self, String> {
        let first = beams
            .first()
            .ok_or("Для колонны отсутствуют стержневые КЭ")?;
        if beams.len() > 24 {
            return Err("Пространственная модель поддерживает до 24 исходных КЭ в колонне".into());
        }
        let section = first.kernel.section.clone();
        let basis = first.kernel.basis;
        let origin = scene.xyz[first.geometry.nodes[0]];
        let distance = |n: usize| dot(sub(scene.xyz[n], origin), basis[0]) / 1000.;
        let mut nodes = beams
            .iter()
            .flat_map(|beam| beam.geometry.nodes)
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        nodes.sort_by(|&a, &b| distance(a).total_cmp(&distance(b)));
        let node_indices = nodes
            .iter()
            .enumerate()
            .map(|(i, &node)| (node, i))
            .collect::<BTreeMap<_, _>>();
        let first_distance = distance(nodes[0]);
        let stations: Vec<_> = nodes
            .iter()
            .map(|&n| distance(n) - first_distance)
            .collect();
        let length = *stations.last().unwrap();
        if nodes.len() != beams.len() + 1
            || stations.windows(2).any(|v| v[1] - v[0] < 1e-8)
            || nodes.iter().any(|&n| {
                let delta = sub(scene.xyz[n], origin);
                dot(delta, basis[1]).hypot(dot(delta, basis[2])) > 1e-4
            })
        {
            return Err("Требуется непрерывная прямолинейная колонна".into());
        }

        let size = 6 * nodes.len();
        let mut endpoint_rows = vec![None::<Row>; size];
        let mut old = Mat::<f64>::zeros(size, size);
        let mut connections = BTreeSet::new();
        for beam in beams {
            let indices = beam.geometry.nodes.map(|node| node_indices[&node]);
            if indices[0].abs_diff(indices[1]) != 1
                || !connections.insert(indices[0].min(indices[1]))
                || beam.kernel.section != section
                || (0..3).any(|i| dot(basis[i], beam.kernel.basis[i]).abs() < 1. - 1e-9)
            {
                return Err(
                    "Требуется колонна с постоянным сечением и общими главными осями".into(),
                );
            }
            let signs = std::array::from_fn::<_, 12, _>(|i| {
                dot(basis[i % 3], beam.kernel.basis[i % 3]).signum()
            });
            let coordinate = |i: usize| 6 * indices[i / 6] + i % 6;
            let units = |i: usize| if i % 6 < 3 { 1. } else { length };
            for i in 0..12 {
                let mut coefficients = BTreeMap::<usize, f64>::new();
                for &(dof, coefficient) in &beam.rows[i] {
                    for &(active, weight) in &projection[dof] {
                        *coefficients.entry(map[active]).or_default() +=
                            coefficient * weight * scale[active] * signs[i] * units(i);
                    }
                }
                let row: Row = coefficients.into_iter().filter(|(_, x)| *x != 0.).collect();
                let slot = &mut endpoint_rows[coordinate(i)];
                if let Some(previous) = slot {
                    if !same_row(previous, &row) {
                        return Err("У соседних КЭ колонны различаются кинематические связи".into());
                    }
                } else {
                    *slot = Some(row);
                }
                for j in 0..12 {
                    old[(coordinate(i), coordinate(j))] +=
                        beam.kernel.k[i][j] * signs[i] * signs[j] / (units(i) * units(j));
                }
            }
        }
        Ok(Self {
            section,
            stations,
            rows: endpoint_rows.into_iter().map(Option::unwrap).collect(),
            old,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn section(b: f64, h: f64) -> super::super::beam::Section {
        super::super::beam::Section {
            width_mm: b,
            material: Material {
                thickness_mm: h,
                ..Default::default()
            },
        }
    }

    fn col(section: super::super::beam::Section, stations: Vec<f64>) -> Column {
        let length = *stations.last().unwrap();
        let size = 6 * stations.len();
        let mut old = Mat::zeros(size, size);
        for (i, edge) in stations.windows(2).enumerate() {
            let segment = super::super::beam::Element::new(
                [0., 0.],
                [(edge[1] - edge[0]) * 1000., 0.],
                section.clone(),
            )
            .unwrap();
            let units = |j: usize| if j % 6 < 3 { 1. } else { length };
            for a in 0..12 {
                for b in 0..12 {
                    old[(6 * i + a, 6 * i + b)] += segment.k[a][b] / (units(a) * units(b));
                }
            }
        }
        Column {
            section,
            stations,
            rows: Vec::new(),
            old,
        }
    }

    #[test]
    fn spatial_column_euler_four_restraint_cases() {
        let column = col(section(40., 60.), vec![0., 3.]);
        let cases: &[(&[usize], f64)] = &[
            (&[4, 5, 10, 11], 1.),
            (&[], 0.5),
            (&[10, 11], 0.69915566),
            (&[6, 7, 8, 9, 10, 11], 2.),
        ];
        for &(free, expected) in cases {
            let boundary = Boundary {
                basis: Mat::from_fn(12, free.len(), |i, j| if i == free[j] { 1. } else { 0. }),
                environment: Mat::zeros(free.len(), free.len()),
            };
            let modes = column.solve(&boundary).unwrap();
            for kind in [ModeKind::Y, ModeKind::Z] {
                let mode = modes
                    .iter()
                    .find(|m| m.kind == kind && m.effective_m.is_some())
                    .unwrap();
                let mu = mode.effective_m.unwrap() / 3.;
                assert!(
                    (mu - expected).abs() < 0.001,
                    "{kind:?}: {mu} vs {expected}"
                );
            }
        }
    }

    #[test]
    fn spatial_column_resolves_repeated_uncoupled_bending_roots() {
        let column = col(section(50., 50.), vec![0., 3.]);
        let boundary = Boundary {
            basis: Mat::zeros(12, 0),
            environment: Mat::zeros(0, 0),
        };
        let modes = column.solve(&boundary).unwrap();
        let y = modes
            .iter()
            .find(|m| m.kind == ModeKind::Y && m.effective_m.is_some())
            .unwrap();
        let z = modes
            .iter()
            .find(|m| m.kind == ModeKind::Z && m.effective_m.is_some())
            .unwrap();
        assert!((y.critical_kn / z.critical_kn - 1.).abs() < 1e-9);
    }

    // Assemble a vertical column and three oblique space-frame members in a
    // common global basis. Nodes 0, 3, 4 are fixed, nodes 1 and 2 are free.
    fn skew_frame(column_section: &super::super::beam::Section) -> (Mat<f64>, Mat<f64>) {
        let xyz = [
            [0., 0., 0.],
            [0., 0., 3000.],
            [2200., 1400., 3100.],
            [4800., -800., 400.],
            [2000., 5000., 2500.],
        ];
        let members = [
            ([0, 1], column_section.clone()),
            ([1, 2], section(34., 46.)),
            ([2, 3], section(39., 43.)),
            ([2, 4], section(37., 36.)),
        ];
        let active = |n: usize| match n {
            1 => Some(0),
            2 => Some(6),
            _ => None,
        };
        let mut environment = Mat::zeros(12, 12);
        let mut full = Mat::zeros(12, 12);
        for (number, ([a, b], section)) in members.into_iter().enumerate() {
            let kernel = super::super::beam::Element::spatial(xyz[a], xyz[b], section).unwrap();
            let transform = Mat::from_fn(12, 12, |i, j| {
                let offset = active([a, b][i / 6]);
                if let Some(offset) = offset.filter(|offset| (j >= *offset) && (j < *offset + 6)) {
                    if (i % 6) / 3 == (j - offset) / 3 {
                        return kernel.basis[i % 3][(j - offset) % 3];
                    }
                }
                0.
            });
            let member = Mat::from_fn(12, 12, |i, j| kernel.k[i][j]);
            let global = transform.transpose() * member.as_ref() * transform.as_ref();
            full += global.as_ref();
            if number > 0 {
                environment += global.as_ref();
            }
        }
        (full, environment)
    }

    #[test]
    fn spatial_column_skew_frame_matches_direct_condensation_and_keeps_mixed_mode() {
        let column = col(section(45., 45.), vec![0., 3.]);
        let (full, environment) = skew_frame(&column.section);
        let mut rows = vec![vec![]; 12];
        for (i, &j) in [2, 0, 1, 5, 3, 4].iter().enumerate() {
            rows[6 + i].push((j, if i < 3 { 1. } else { 3. }));
        }
        let factor = full.llt(Side::Lower).unwrap();
        let c = compliance(&rows, 12, &|rhs| factor.solve(rhs.as_ref())).unwrap();
        let (boundary, note) =
            boundary_stiffness(&rows, &[1.; 12], c, &column.old, Some(&full)).unwrap();
        assert!(note.is_none());

        let direct_end = Mat::from_fn(6, 6, |i, j| environment[(i, j)]);
        let link = Mat::from_fn(6, 6, |i, j| environment[(i + 6, j)]);
        let inner = Mat::from_fn(6, 6, |i, j| environment[(i + 6, j + 6)]);
        let direct =
            direct_end - link.transpose() * inner.llt(Side::Lower).unwrap().solve(link.as_ref());
        let to_global = Mat::from_fn(6, 12, |i, j| {
            rows[j]
                .iter()
                .find(|&&(d, _)| d == i)
                .map_or(0., |&(_, scale)| scale.recip())
        });
        let direct_basis = to_global.as_ref() * boundary.basis.as_ref();
        let reference = direct_basis.transpose() * direct.as_ref() * direct_basis.as_ref();
        assert!(
            largest_entry(&(&reference - &boundary.environment)) < 1e-8 * largest_entry(&reference)
        );
        let fallback = direct_environment(&rows, &boundary.basis, &column.old, &full).unwrap();
        assert!(largest_entry(&(&reference - &fallback)) < 1e-8 * largest_entry(&reference));

        let modes = column.solve(&boundary).unwrap();
        let reference_modes = column
            .solve(&Boundary {
                basis: boundary.basis,
                environment: reference,
            })
            .unwrap();
        let first = &modes[0];
        eprintln!(
            "skew frame: {:?}, Ncr={:.6} kN, y={:.6}, z={:.6}, torsion={:.6}, sway={:.6}",
            first.kind,
            first.critical_kn,
            first.bending_y_fraction,
            first.bending_z_fraction,
            first.torsion_fraction,
            first.sway_fraction
        );
        assert!((first.critical_kn / reference_modes[0].critical_kn - 1.).abs() < 1e-8);
        assert!(matches!(
            first.kind,
            ModeKind::CoupledBending | ModeKind::BendingTorsion
        ));
        assert!(
            first.bending_y_fraction >= PARTICIPATION && first.bending_z_fraction >= PARTICIPATION
        );
        assert!(first.effective_m.is_none());
        assert!(modes.iter().all(|m| m.effective_m.is_none()));
    }

    #[test]
    fn spatial_column_ill_conditioned_compliance_uses_direct_environment() {
        let column = col(section(45., 55.), vec![0., 3.]);
        // An almost rigid axial restraint at the free end leaves the two
        // bending planes essentially cantilevered and makes C ill conditioned.
        let axial = 1e17;
        let full = Mat::from_fn(6, 6, |i, j| {
            column.old[(6 + i, 6 + j)] + if (i, j) == (0, 0) { axial } else { 0. }
        });
        let scales: Vec<_> = (0..6).map(|i| full[(i, i)].sqrt().recip()).collect();
        let scaled = Mat::from_fn(6, 6, |i, j| full[(i, j)] * scales[i] * scales[j]);
        let rows = (0..12)
            .map(|i| {
                if i < 6 {
                    vec![]
                } else {
                    vec![(i - 6, scales[i - 6])]
                }
            })
            .collect::<Vec<_>>();
        let factor = scaled.llt(Side::Lower).unwrap();
        let c = compliance(&rows, 6, &|rhs| factor.solve(rhs.as_ref())).unwrap();
        assert!(boundary_stiffness(&rows, &scales, c.clone(), &column.old, None).is_err());
        let (boundary, diagnostic) =
            boundary_stiffness(&rows, &scales, c, &column.old, Some(&scaled)).unwrap();
        assert!(diagnostic
            .unwrap()
            .contains("Использована прямая конденсация"));
        let modes = column.solve(&boundary).unwrap();
        let governing: Vec<_> = modes.iter().filter_map(|m| m.effective_m).collect();
        assert_eq!(governing.len(), 2);
        assert!(governing.into_iter().all(|l| (l / 3. - 2.).abs() < 0.001));
    }
}

fn same_row(a: &Row, b: &Row) -> bool {
    let mut difference = BTreeMap::<usize, f64>::new();
    for &(i, value) in a {
        *difference.entry(i).or_default() += value;
    }
    for &(i, value) in b {
        *difference.entry(i).or_default() -= value;
    }
    let magnitude = a.iter().chain(b).map(|(_, x)| x.abs()).fold(0., f64::max);
    difference
        .values()
        .all(|value| value.abs() <= 1e-9 * magnitude)
}

fn compliance(
    rows: &[Row],
    free: usize,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Mat<f64>, String> {
    let mut c = Mat::zeros(rows.len(), rows.len());
    let mut probes = Vec::<(usize, &Row)>::new();
    let mut source = vec![None; rows.len()];
    for (i, row) in rows.iter().enumerate().filter(|(_, row)| !row.is_empty()) {
        let duplicate = probes.iter().position(|(_, previous)| *previous == row);
        let index = duplicate.unwrap_or(probes.len());
        if duplicate.is_none() {
            probes.push((i, row));
        }
        source[i] = Some(index);
    }
    // Both RHS and solution exist during the substitution.
    let batch_size = ((64 * 1024 * 1024) / (16 * free.max(1))).clamp(1, 32);
    for batch in probes.chunks(batch_size) {
        super::super::progress::check_cancel()?;
        let mut rhs = Mat::zeros(free, batch.len());
        for (j, (_, row)) in batch.iter().enumerate() {
            for &(dof, value) in *row {
                rhs[(dof, j)] = value;
            }
        }
        let response = solve(&rhs);
        for (j, &(index, _)) in batch.iter().enumerate() {
            for (i, row) in rows.iter().enumerate() {
                c[(i, index)] = row.iter().map(|&(d, v)| v * response[(d, j)]).sum();
            }
        }
    }
    Ok(Mat::from_fn(rows.len(), rows.len(), |i, j| {
        source[j].map_or(0., |k| c[(i, probes[k].0)])
    }))
}

struct Boundary {
    basis: Mat<f64>,
    environment: Mat<f64>,
}

fn boundary_stiffness(
    rows: &[Row],
    solver_scales: &[f64],
    compliance: Mat<f64>,
    old: &Mat<f64>,
    direct_k: Option<&Mat<f64>>,
) -> Result<(Boundary, Option<String>), String> {
    let n = rows.len();
    let gram = Mat::from_fn(n, n, |i, j| {
        let (a, b) = (&rows[i], &rows[j]);
        a.iter()
            .filter_map(|&(d, x)| {
                b.binary_search_by_key(&d, |&(k, _)| k)
                    .ok()
                    .map(|k| (x / solver_scales[d]) * (b[k].1 / solver_scales[d]))
            })
            .sum::<f64>()
    });
    let eigen = gram
        .self_adjoint_eigen(Side::Lower)
        .map_err(|_| "Не определены свободные перемещения концов колонны")?;
    let max = eigen.S()[n - 1];
    let active: Vec<_> = (0..n)
        .filter(|&i| eigen.S()[i] > max * 1e-12 && eigen.S()[i] > 0.)
        .collect();
    let basis = Mat::from_fn(n, active.len(), |i, j| eigen.U()[(i, active[j])]);
    if active.is_empty() {
        return Ok((
            Boundary {
                basis,
                environment: Mat::zeros(0, 0),
            },
            None,
        ));
    }
    let (environment, note) = match compliance_environment(&basis, &compliance, old) {
        Ok(environment) => (environment, None),
        Err(reason) => {
            let Some(k) = direct_k else {
                return Err(reason);
            };
            let environment = direct_environment(rows, &basis, old, k)
                .map_err(|err| format!("{reason}. Прямая конденсация: {err}"))?;
            (
                environment,
                Some(format!(
                    "{reason}. Использована прямая конденсация окружения"
                )),
            )
        }
    };
    Ok((Boundary { basis, environment }, note))
}

fn compliance_environment(
    basis: &Mat<f64>,
    compliance: &Mat<f64>,
    old: &Mat<f64>,
) -> Result<Mat<f64>, String> {
    let n = compliance.nrows();
    let symmetric = Mat::from_fn(n, n, |i, j| (compliance[(i, j)] + compliance[(j, i)]) / 2.);
    let max_entry = largest_entry(&symmetric);
    if !finite(compliance)
        || (0..n).any(|i| {
            (0..i).any(|j| (compliance[(i, j)] - compliance[(j, i)]).abs() > 1e-8 * max_entry)
        })
    {
        return Err("Нарушена симметрия податливости концов колонны".into());
    }
    let c = basis.transpose() * symmetric.as_ref() * basis.as_ref();
    let eig = c
        .self_adjoint_eigen(Side::Lower)
        .map_err(|_| "Не удалось разложить матрицу податливости")?;
    let last = eig.S()[c.nrows() - 1];
    if last <= 0. || eig.S()[0] <= 1e-12 * last {
        return Err(
            "Податливость концов плохо обусловлена, требуется прямая конденсация окружения".into(),
        );
    }
    let factor = c
        .llt(Side::Lower)
        .map_err(|_| "Податливость концов вырождена")?;
    let inverse = factor.solve(Mat::identity(c.nrows(), c.nrows()));
    let member = basis.transpose() * old.as_ref() * basis.as_ref();
    let environment = Mat::from_fn(c.nrows(), c.nrows(), |i, j| {
        (inverse[(i, j)] + inverse[(j, i)] - member[(i, j)] - member[(j, i)]) / 2.
    });
    let max_stiffness = largest_entry(&inverse).max(largest_entry(&member));
    check_environment(&environment, max_stiffness)?;
    let closure =
        c.as_ref() * (&environment + &member) - Mat::<f64>::identity(c.nrows(), c.nrows());
    if largest_entry(&closure) > 1e-5 {
        return Err("Нарушена проверка обратимости эквивалентной жёсткости окружения".into());
    }
    Ok(environment)
}

fn direct_environment(
    rows: &[Row],
    basis: &Mat<f64>,
    old: &Mat<f64>,
    k: &Mat<f64>,
) -> Result<Mat<f64>, String> {
    let free = k.nrows();
    let active = basis.ncols();
    let mut d = Mat::<f64>::zeros(rows.len(), free);
    for (i, row) in rows.iter().enumerate() {
        for &(j, value) in row {
            d[(i, j)] = value;
        }
    }
    let a = basis.transpose() * d.as_ref();
    let svd = a.svd().map_err(|_| "не определены граничные координаты")?;
    if active > free || svd.S()[active - 1] <= 1e-12 * svd.S()[0] {
        return Err("кинематика граничных координат плохо обусловлена".into());
    }
    let right_inverse = Mat::from_fn(free, active, |i, j| {
        (0..active)
            .map(|s| svd.V()[(i, s)] * svd.U()[(j, s)] / svd.S()[s])
            .sum::<f64>()
    });
    let nullspace = Mat::from_fn(free, free - active, |i, j| svd.V()[(i, active + j)]);
    let member = d.transpose() * old.as_ref() * d.as_ref();
    let surroundings = k - &member;
    let kp = surroundings.as_ref() * right_inverse.as_ref();
    let kept = right_inverse.transpose() * kp.as_ref();
    let largest = largest_entry(&kept);
    let condensed = if nullspace.ncols() == 0 {
        kept
    } else {
        let kw = nullspace.transpose() * surroundings.as_ref() * nullspace.as_ref();
        let coupling = nullspace.transpose() * kp.as_ref();
        let factor = kw
            .llt(Side::Lower)
            .map_err(|_| "внутренняя часть окружения имеет механизм")?;
        kept - coupling.transpose() * factor.solve(coupling.as_ref())
    };
    let symmetric = Mat::from_fn(active, active, |i, j| {
        (condensed[(i, j)] + condensed[(j, i)]) / 2.
    });
    check_environment(&symmetric, largest)?;
    Ok(symmetric)
}

fn check_environment(matrix: &Mat<f64>, stiffness_scale: f64) -> Result<(), String> {
    if !finite(matrix) {
        return Err("Матрица жёсткости окружения содержит неконечные значения".into());
    }
    let spectrum = matrix
        .self_adjoint_eigenvalues(Side::Lower)
        .map_err(|_| "Не удалось проверить жёсткость окружения")?;
    if spectrum[0] < -1e-8 * stiffness_scale {
        return Err(
            "При вычитании жёсткости колонны нарушена устойчивость упругого окружения".into(),
        );
    }
    Ok(())
}

fn largest_entry(matrix: &Mat<f64>) -> f64 {
    (0..matrix.nrows())
        .flat_map(|i| (0..matrix.ncols()).map(move |j| matrix[(i, j)].abs()))
        .fold(0., f64::max)
}

fn finite(matrix: &Mat<f64>) -> bool {
    (0..matrix.nrows()).all(|i| (0..matrix.ncols()).all(|j| matrix[(i, j)].is_finite()))
}

fn rigidities(section: &super::beam::Section) -> ([f64; 2], f64, f64) {
    let b = section.width_mm / 1000.;
    let h = section.material.thickness_mm / 1000.;
    let e = section.material.e_mpa * 1000.;
    let inertia = [b * h.powi(3) / 12., h * b.powi(3) / 12.];
    let shear = (5. / 6.) * e / (2. * (1. + section.material.nu)) * b * h;
    (
        inertia.map(|i| e * i),
        shear,
        (inertia[0] + inertia[1]) / (b * h),
    )
}

// Geometric work of a compressive unit force. Integrate the slope of the
// exact static Timoshenko interpolation, including its constant shear strain.
fn geometric_parts(section: &super::beam::Section, length: f64) -> [[[f64; 12]; 12]; 3] {
    let (ei, shear, polar_radius_sq) = rigidities(section);
    let mut parts = [[[0.; 12]; 12]; 3];
    let gauss = (3_f64 / 5.).sqrt();
    for (axis, ids, sign) in [(0, [2, 4, 8, 10], -1.), (1, [1, 5, 7, 11], 1.)] {
        let flexibility = ei[axis] / (shear * length * length);
        for (xi, weight) in [(-gauss, 5. / 9.), (0., 8. / 9.), (gauss, 5. / 9.)] {
            let s = (1. + xi) / 2.;
            let factor = 6. * (s * (1. - s) + 2. * flexibility) / (1. + 12. * flexibility);
            let derivative = [
                -factor / length,
                sign * (1. - s - factor / 2.),
                factor / length,
                sign * (s - factor / 2.),
            ];
            for i in 0..4 {
                for j in 0..4 {
                    parts[axis][ids[i]][ids[j]] +=
                        derivative[i] * derivative[j] * weight * length / 2.;
                }
            }
        }
    }
    let torsion = polar_radius_sq / length;
    parts[2][3][3] = torsion;
    parts[2][9][9] = torsion;
    parts[2][3][9] = -torsion;
    parts[2][9][3] = -torsion;
    parts
}

fn bilinear(a: &Mat<f64>, i: usize, matrix: &Mat<f64>, b: &Mat<f64>, j: usize) -> f64 {
    (0..matrix.nrows())
        .map(|k| {
            a[(k, i)]
                * (0..matrix.ncols())
                    .map(|l| matrix[(k, l)] * b[(l, j)])
                    .sum::<f64>()
        })
        .sum()
}

const PARTICIPATION: f64 = 0.01;
fn mode_kind(fraction: [f64; 3]) -> ModeKind {
    let [y, z, torsion] = fraction.map(|part| part >= PARTICIPATION);
    match (y, z, torsion) {
        (true, false, false) => ModeKind::Y,
        (false, true, false) => ModeKind::Z,
        (false, false, true) => ModeKind::Torsion,
        (_, _, true) => ModeKind::BendingTorsion,
        _ => ModeKind::CoupledBending,
    }
}

impl Column {
    fn subdivisions(&self, target: usize) -> Vec<usize> {
        let length = self.stations.last().unwrap();
        self.stations
            .windows(2)
            .map(|v| ((v[1] - v[0]) / length * target as f64).ceil() as usize)
            .collect()
    }

    fn solve(&self, boundary: &Boundary) -> Result<Vec<ColumnMode>, String> {
        let mut previous: Option<Vec<ColumnMode>> = None;
        let mut last = vec![];
        for target in [8, 16, 32, 48] {
            super::super::progress::check_cancel()?;
            let subdivisions = self.subdivisions(target);
            if subdivisions == last {
                continue;
            }
            let modes = self.solve_refinement(boundary, &subdivisions)?;
            let axes = |modes: &[ColumnMode]| {
                [0, 1, 2].map(|axis| {
                    modes
                        .iter()
                        .find(|m| {
                            [
                                m.bending_y_fraction,
                                m.bending_z_fraction,
                                m.torsion_fraction,
                            ][axis]
                                >= PARTICIPATION
                        })
                        .map(|m| (m.critical_kn, m.kind, m.effective_m.is_some()))
                })
            };
            if previous.as_ref().is_some_and(|previous| {
                axes(previous)
                    .into_iter()
                    .zip(axes(&modes))
                    .all(|(a, b)| match (a, b) {
                        (Some((a, a_kind, a_length)), Some((b, b_kind, b_length))) => {
                            (a / b - 1.).abs() <= 0.002 && a_kind == b_kind && a_length == b_length
                        }
                        (None, None) => true,
                        _ => false,
                    })
            }) {
                return Ok(modes);
            }
            previous = Some(modes);
            last = subdivisions;
        }
        Err("Не достигнута сходимость критических сил при уточнении колонны".into())
    }

    fn solve_refinement(
        &self,
        boundary: &Boundary,
        subdivisions: &[usize],
    ) -> Result<Vec<ColumnMode>, String> {
        let length = *self.stations.last().unwrap();
        let nodes = subdivisions.iter().sum::<usize>() + 1;
        let size = 6 * nodes;
        let mut stiffness = Mat::<f64>::zeros(size, size);
        let mut geometric: [Mat<f64>; 3] = std::array::from_fn(|_| Mat::zeros(size, size));
        let mut interface = vec![0];
        let mut cursor = 0;
        let units = |i: usize| if i % 6 < 3 { 1. } else { length };
        for (&count, stations) in subdivisions.iter().zip(self.stations.windows(2)) {
            let h = (stations[1] - stations[0]) / count as f64;
            let kernel =
                super::beam::Element::new([0., 0.], [h * 1000., 0.], self.section.clone())?;
            let parts = geometric_parts(&self.section, h);
            for _ in 0..count {
                for i in 0..12 {
                    for j in 0..12 {
                        let ij = (6 * cursor + i, 6 * cursor + j);
                        let factor = 1. / (units(i) * units(j));
                        stiffness[ij] += factor * kernel.k[i][j];
                        for axis in 0..3 {
                            geometric[axis][ij] += factor * parts[axis][i][j];
                        }
                    }
                }
                cursor += 1;
            }
            interface.push(cursor);
        }
        let mut new_nodes = vec![0; nodes];
        let mut added = 0;
        for (node, slot) in new_nodes.iter_mut().enumerate() {
            if interface.binary_search(&node).is_err() {
                *slot = added;
                added += 1;
            }
        }
        let active = boundary.basis.ncols();
        let reduced = active + added * 6;
        let transform = Mat::from_fn(size, reduced, |i, j| {
            match interface.binary_search(&(i / 6)) {
                Ok(old) if j < active => boundary.basis[(old * 6 + i % 6, j)],
                Err(_) if j == active + new_nodes[i / 6] * 6 + i % 6 => 1.,
                _ => 0.,
            }
        });
        let mut k = transform.transpose() * stiffness.as_ref() * transform.as_ref();
        for i in 0..active {
            for j in 0..active {
                k[(i, j)] += boundary.environment[(i, j)];
            }
        }
        // Diagonal change of units leaves the spectrum unchanged and avoids mixing raw units.
        let scales: Vec<_> = (0..reduced).map(|i| k[(i, i)].sqrt().recip()).collect();
        if scales.iter().any(|v| !v.is_finite()) {
            return Err("Уточнённая модель колонны имеет механизм".into());
        }
        let k = Mat::from_fn(reduced, reduced, |i, j| k[(i, j)] * scales[i] * scales[j]);
        let scaled_transform = Mat::from_fn(size, reduced, |i, j| transform[(i, j)] * scales[j]);
        let factor = k
            .llt(Side::Lower)
            .map_err(|_| "Уточнённая модель колонны имеет механизм")?;
        let l = factor.L();
        let parts = geometric.map(|g| {
            let mut reduced = scaled_transform.transpose() * g.as_ref() * scaled_transform.as_ref();
            l.solve_lower_triangular_in_place(reduced.as_mut());
            l.solve_lower_triangular_in_place(reduced.as_mut().transpose_mut());
            reduced
        });
        let g = Mat::from_fn(reduced, reduced, |i, j| {
            parts.iter().map(|a| a[(i, j)]).sum::<f64>()
        });
        let eigen = g
            .self_adjoint_eigen(Side::Lower)
            .map_err(|_| "Не решена спектральная задача устойчивости колонны")?;
        let largest = eigen.S()[reduced - 1];
        let mut first = reduced;
        let mut modes = Vec::new();
        let mut axes_covered = [false; 2];
        let (ei, _, _) = rigidities(&self.section);
        while first > 0 && modes.len() < 12 {
            let root = eigen.S()[first - 1];
            if root <= largest * 1e-12 || root <= 0. || !root.is_finite() {
                break;
            }
            let mut low = first - 1;
            while low > 0 && root - eigen.S()[low - 1] <= root * 1e-8 {
                low -= 1;
            }
            let count = first - low;
            let mut vectors = Mat::from_fn(reduced, count, |i, j| eigen.U()[(i, first - 1 - j)]);
            if count > 1 {
                // A repeated root admits arbitrary mixtures in its eigenspace. Resolve
                // equal independent principal-plane roots by their geometric work.
                let direction = &parts[0] - &parts[1];
                let rotation = Mat::from_fn(count, count, |i, j| {
                    bilinear(&vectors, i, &direction, &vectors, j)
                })
                .self_adjoint_eigen(Side::Lower)
                .map_err(|_| "Не разделены кратные формы колонны")?;
                vectors = vectors.as_ref() * rotation.U();
            }
            let mut physical = vectors.clone();
            l.transpose()
                .solve_upper_triangular_in_place(physical.as_mut());
            let physical = scaled_transform.as_ref() * physical.as_ref();
            let mut torsion_in_cluster = false;
            for i in 0..count {
                let mut fractions = parts
                    .each_ref()
                    .map(|p| bilinear(&vectors, i, p, &vectors, i).max(0.));
                let work = fractions.iter().sum::<f64>();
                if work <= 0. {
                    continue;
                }
                fractions = fractions.map(|v| v / work);
                let kind = mode_kind(fractions);
                if kind == ModeKind::Torsion && std::mem::replace(&mut torsion_in_cluster, true) {
                    continue;
                }
                let critical_kn = 1. / work;
                let axis = match kind {
                    ModeKind::Y => Some(0),
                    ModeKind::Z => Some(1),
                    _ => None,
                };
                let effective_m = axis
                    .filter(|&axis| !axes_covered[axis])
                    .map(|axis| std::f64::consts::PI * (ei[axis] / critical_kn).sqrt());
                for axis in 0..2 {
                    axes_covered[axis] |= fractions[axis] >= PARTICIPATION;
                }
                if modes.len() >= 12 {
                    break;
                }
                let top = 6 * (nodes - 1);
                let end_shift = (physical[(top + 1, i)] - physical[(1, i)])
                    .hypot(physical[(top + 2, i)] - physical[(2, i)]);
                let max_shift = (0..nodes)
                    .map(|n| physical[(6 * n + 1, i)].hypot(physical[(6 * n + 2, i)]))
                    .fold(0., f64::max);
                modes.push(ColumnMode {
                    kind,
                    critical_kn,
                    effective_m,
                    bending_y_fraction: fractions[0],
                    bending_z_fraction: fractions[1],
                    torsion_fraction: fractions[2],
                    sway_fraction: if max_shift > 0. {
                        (end_shift / (2. * max_shift)).min(1.)
                    } else {
                        0.
                    },
                });
            }
            first = low;
        }
        if modes.is_empty() {
            return Err("Положительные критические силы колонны отсутствуют".into());
        }
        Ok(modes)
    }
}
