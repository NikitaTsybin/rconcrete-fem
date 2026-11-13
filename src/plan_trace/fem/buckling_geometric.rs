//! Signed initial-stress stiffness for global linear buckling.
//! Compression gives a positive contribution to `K - lambda * G`.
use super::*;

pub(super) fn geometric(
    cells: &[Cell3],
    beams: &[BeamCell],
    reference: &[f64],
    beam_loads: &[[f64; 12]],
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    free: usize,
) -> Result<SparseColMat<usize, f64>, String> {
    if reference.len() != projection.len()
        || beam_loads.len() != beams.len()
        || reference.iter().any(|v| !v.is_finite())
    {
        return Err("Некорректное состояние для расчёта устойчивости".into());
    }
    let mut assembly = Vec::new();
    let reduce = |row: &Row| reduce_row(row, projection, map, scale);
    for (i, cell) in cells.iter().enumerate() {
        if i % 256 == 0 {
            super::super::progress::check_cancel()?;
        }
        let local = shell_local(cell, reference)?;
        if local.iter().flatten().all(|&v| v == 0.) {
            continue;
        }
        // The membrane and normal displacement rows already include rigid offsets.
        for axis in 0..3 {
            let rows: Vec<_> = shell_rows(cell, axis).map(reduce).collect();
            append(&mut assembly, &rows, &local)?;
        }
    }
    for (i, (beam, applied)) in beams.iter().zip(beam_loads).enumerate() {
        if i % 256 == 0 {
            super::super::progress::check_cancel()?;
        }
        let Some(local) = beam_local(beam, reference, applied) else {
            continue;
        };
        let rows: Vec<_> = beam.rows.iter().map(reduce).collect();
        append(&mut assembly, &rows, &local)?;
    }
    SparseColMat::try_new_from_triplets(free, free, &assembly)
        .map_err(|error| format!("Сборка геометрической жёсткости: {error:?}"))
}

fn shell_rows(cell: &Cell3, axis: usize) -> impl Iterator<Item = &Row> {
    (0..cell.surface.nodes.len()).map(move |n| {
        if axis < 2 {
            &cell.mem[2 * n + axis]
        } else {
            &cell.bend[3 * n]
        }
    })
}

fn shell_local(cell: &Cell3, reference: &[f64]) -> Result<[[f64; 4]; 4], String> {
    let uv: Vec<_> = cell.mem.iter().map(|row| value(row, reference)).collect();
    let mut local = [[0.; 4]; 4];
    match &cell.kernel {
        Kernel::Dkt(triangle, _) => shell_sample(
            cell.surface.material,
            &triangle.gradients(),
            &uv,
            triangle.area,
            &mut local,
        ),
        Kernel::Quad(quad) => {
            for natural in super::super::mitc4::GAUSS {
                let sample = quad.sample(natural)?;
                shell_sample(
                    cell.surface.material,
                    &sample.grad,
                    &uv,
                    sample.det,
                    &mut local,
                );
            }
        }
    }
    Ok(local)
}

fn beam_local(beam: &BeamCell, reference: &[f64], applied: &[f64; 12]) -> Option<[[f64; 12]; 12]> {
    // The reference coordinates are physical metres and radians. The
    // equilibrated axial force varies linearly under uniform member load.
    let axial =
        beam.kernel.k[6][6] * (value(&beam.rows[6], reference) - value(&beam.rows[0], reference));
    let compression = [-axial - applied[0], -axial + applied[6]];
    (compression != [0.; 2]).then(|| beam_geometric(&beam.kernel, compression))
}

pub(super) fn sensitivities(
    cells: &[Cell3],
    beams: &[BeamCell],
    reference: &[f64],
    beam_loads: &[[f64; 12]],
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    modes: &[(f64, Vec<f64>)],
) -> Result<Vec<Option<super::super::BucklingSensitivity>>, String> {
    let elements = cells.len().saturating_add(beams.len());
    if modes.is_empty() || elements.saturating_mul(modes.len()) > 4_000_000 {
        return Ok(vec![None; modes.len()]);
    }
    let mut energies = vec![vec![0.; elements]; modes.len()];
    let reduce = |row: &Row| reduce_row(row, projection, map, scale);
    for (i, cell) in cells.iter().enumerate() {
        if i % 256 == 0 {
            super::super::progress::check_cancel()?;
        }
        let local = shell_local(cell, reference)?;
        if local.iter().flatten().all(|&v| v == 0.) {
            continue;
        }
        for axis in 0..3 {
            let rows: Vec<_> = shell_rows(cell, axis).map(reduce).collect();
            for ((_, mode), output) in modes.iter().zip(&mut energies) {
                output[i] += local_energy(&rows, &local, mode);
            }
        }
    }
    for (i, (beam, applied)) in beams.iter().zip(beam_loads).enumerate() {
        if i % 256 == 0 {
            super::super::progress::check_cancel()?;
        }
        let Some(local) = beam_local(beam, reference, applied) else {
            continue;
        };
        let rows: Vec<_> = beam.rows.iter().map(reduce).collect();
        for ((_, mode), output) in modes.iter().zip(&mut energies) {
            output[cells.len() + i] = local_energy(&rows, &local, mode);
        }
    }
    Ok(energies
        .into_iter()
        .map(|energy| normalize_energy(&energy, cells.len()))
        .collect())
}

fn local_energy<const N: usize>(rows: &[Row], local: &[[f64; N]; N], mode: &[f64]) -> f64 {
    let mut dofs = [0.; N];
    for (q, row) in dofs.iter_mut().zip(rows) {
        *q = value(row, mode);
    }
    let mut energy = 0.;
    for i in 0..rows.len() {
        let mut product = local[i][i] * dofs[i];
        for j in i + 1..rows.len() {
            product += 2. * local[i][j] * dofs[j];
        }
        energy += dofs[i] * product;
    }
    energy
}

fn normalize_energy(
    energies: &[f64],
    surfaces: usize,
) -> Option<super::super::BucklingSensitivity> {
    let mut total = 0.;
    let mut compensation = 0.;
    let mut absolute_sum = 0.;
    for &energy in energies {
        let corrected = energy - compensation;
        let sum = total + corrected;
        compensation = (sum - total) - corrected;
        total = sum;
        absolute_sum += energy.abs();
    }
    // Extreme cancellation makes percentages unreliable at f32 storage precision.
    if !total.is_finite() || total <= f64::from(f32::EPSILON) * absolute_sum {
        return None;
    }
    let (surface_energy, beam_energy) = energies.split_at(surfaces);
    let percent = |entries: &[f64]| {
        entries
            .iter()
            .map(|energy| (100. * (energy / total)) as f32)
            .collect()
    };
    Some(super::super::BucklingSensitivity {
        surfaces: percent(surface_energy),
        beams: percent(beam_energy),
    })
}

fn shell_sample(
    material: Material,
    gradients: &[Point],
    uv: &[f64],
    area: f64,
    local: &mut [[f64; 4]; 4],
) {
    let mut strain = [0.; 3];
    for (&[dx, dy], uv) in gradients.iter().zip(uv.chunks_exact(2)) {
        strain[0] += dx * uv[0];
        strain[1] += dy * uv[1];
        strain[2] += dy * uv[0] + dx * uv[1];
    }
    let [nx, ny, nxy] = membrane_d(material).map(|row| dot(row, strain));
    for (i, &[gx, gy]) in gradients.iter().enumerate() {
        for (j, &[hx, hy]) in gradients.iter().enumerate().skip(i) {
            local[i][j] -= area * (nx * gx * hx + ny * gy * hy + nxy * (gx * hy + gy * hx));
        }
    }
}

fn beam_geometric(element: &super::super::beam::Element, compression: [f64; 2]) -> [[f64; 12]; 12] {
    let mut local = [[0.; 12]; 12];
    let length = element.length;
    let material = element.section.material;
    let b = element.section.width_mm / 1000.;
    let h = material.thickness_mm / 1000.;
    let inertia = [b * h.powi(3) / 12., h * b.powi(3) / 12.];
    let flexibility_scale = 2. * (1. + material.nu) / ((5. / 6.) * b * h * length * length);
    let gauss = (3_f64 / 5.).sqrt();
    for (axis, ids, sign) in [(0, [2, 4, 8, 10], -1.), (1, [1, 5, 7, 11], 1.)] {
        let flexibility = inertia[axis] * flexibility_scale;
        for (xi, weight) in [(-gauss, 5. / 9.), (0., 8. / 9.), (gauss, 5. / 9.)] {
            let t = (1. + xi) * 0.5;
            let factor = 6. * (t * (1. - t) + 2. * flexibility) / (1. + 12. * flexibility);
            let gradient = [
                -factor / length,
                sign * (1. - t - factor * 0.5),
                factor / length,
                sign * (t - factor * 0.5),
            ];
            let n = (1. - t) * compression[0] + t * compression[1];
            for i in 0..4 {
                for j in i..4 {
                    local[ids[i]][ids[j]] += n * length * weight * 0.5 * gradient[i] * gradient[j];
                }
            }
        }
    }
    // The polar geometric term uses the same Saint-Venant rod model as the
    // spatial column calculation. The section is assumed not to warp.
    let torsion =
        (compression[0] + compression[1]) * 0.5 * (inertia[0] + inertia[1]) / (b * h * length);
    local[3][3] = torsion;
    local[9][9] = torsion;
    local[3][9] = -torsion;
    local
}

fn reduce_row(row: &Row, projection: &[Row], map: &[usize], scale: &[f64]) -> Row {
    let mut reduced = BTreeMap::new();
    for &(dof, a) in row {
        for &(independent, b) in &projection[dof] {
            let index = map[independent];
            if index != usize::MAX {
                *reduced.entry(index).or_insert(0.) += a * b * scale[independent];
            }
        }
    }
    reduced.into_iter().filter(|&(_, v)| v != 0.).collect()
}

fn append<const N: usize>(
    assembly: &mut Vec<Triplet<usize, usize, f64>>,
    rows: &[Row],
    local: &[[f64; N]; N],
) -> Result<(), String> {
    for (i, a) in rows.iter().enumerate() {
        for (j, b) in rows.iter().enumerate().skip(i) {
            let coefficient = local[i][j];
            if !coefficient.is_finite() {
                return Err("Геометрическая жёсткость содержит неконечные значения".into());
            }
            if coefficient == 0. {
                continue;
            }
            for &(p, u) in a {
                for &(q, v) in b {
                    if i == j && p > q {
                        continue;
                    }
                    let factor = if i != j && p == q { 2. } else { 1. };
                    let entry = factor * coefficient * u * v;
                    if !entry.is_finite() {
                        return Err("Некорректная приведённая геометрическая жёсткость".into());
                    }
                    assembly.push(Triplet::new(p.min(q), p.max(q), entry));
                }
            }
        }
    }
    Ok(())
}

pub(super) fn apply_symmetric(matrix: &SparseColMat<usize, f64>, x: &Mat<f64>) -> Mat<f64> {
    let mut result = Mat::<f64>::zeros(matrix.nrows(), x.ncols());
    let symbolic = matrix.symbolic();
    for k in 0..x.ncols() {
        for j in 0..matrix.ncols() {
            let input = x[(j, k)];
            let mut transposed = 0.;
            for (i, &value) in symbolic.row_idx_of_col(j).zip(matrix.val_of_col(j)) {
                result[(i, k)] += value * input;
                if i != j {
                    transposed += value * x[(i, k)];
                }
            }
            result[(j, k)] += transposed;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn global_buckling_t3_uses_signed_membrane_stress() {
        let material = Material {
            nu: 0.,
            ..Material::default()
        };
        let gradients = [[-1., -1.], [1., 0.], [0., 1.]];
        let stiffness = membrane_d(material)[0][0];
        let mut local = [[0.; 4]; 4];
        shell_sample(
            material,
            &gradients,
            &[0., 0., -0.001, 0., 0., 0.],
            0.5,
            &mut local,
        );
        assert!((local[1][1] - 0.0005 * stiffness).abs() < 1e-9);
        assert_eq!(local[2][2], 0.);
        assert_eq!(local[0][1], -local[1][1]);
        shell_sample(
            material,
            &gradients,
            &[0., 0., 0.001, 0., 0., 0.],
            0.5,
            &mut local,
        );
        assert!(local.iter().flatten().all(|&v| v == 0.));
    }

    #[test]
    fn global_buckling_q4_integrates_affine_field() {
        use super::super::super::mitc4::{Quad4, GAUSS};
        let material = Material {
            nu: 0.,
            ..Material::default()
        };
        let xy = [[0., 0.], [2., 0.], [2., 1.], [0., 1.]];
        let quad = Quad4::new(xy).unwrap();
        let uv = [0., 0., -0.002, 0., -0.002, 0., 0., 0.];
        let mut local = [[0.; 4]; 4];
        for natural in GAUSS {
            let point = quad.sample(natural).unwrap();
            shell_sample(material, &point.grad, &uv, point.det, &mut local);
        }
        let energy = |direction: usize| {
            (0..4)
                .flat_map(|i| (i..4).map(move |j| (i, j)))
                .map(|(i, j)| {
                    let weight = if i == j { 1. } else { 2. };
                    weight * local[i][j] * xy[i][direction] * xy[j][direction]
                })
                .sum::<f64>()
        };
        assert!((energy(0) - 0.002 * membrane_d(material)[0][0]).abs() < 1e-8);
        assert!(energy(1).abs() < 1e-8);
    }

    #[test]
    fn global_buckling_rod_has_signed_and_varying_axial_stress() {
        let section = super::super::super::beam::Section {
            width_mm: 150.,
            material: Material {
                thickness_mm: 100.,
                ..Material::default()
            },
        };
        let element =
            super::super::super::beam::Element::new([0., 0.], [3000., 0.], section).unwrap();
        let uniform = beam_geometric(&element, [12., 12.]);
        let tension = beam_geometric(&element, [-12., -12.]);
        assert!(uniform
            .iter()
            .flatten()
            .zip(tension.iter().flatten())
            .all(|(a, b)| a + b == 0.));
        assert!(uniform[1][1] > 0. && uniform[2][2] > 0. && uniform[3][3] > 0.);
        let varying = beam_geometric(&element, [24., 0.]);
        assert!((varying[1][1] - uniform[1][1]).abs() < 1e-12);
        assert!(varying[5][5] > varying[11][11]);
        assert!((varying[3][3] - uniform[3][3]).abs() < 1e-12);
    }

    #[test]
    fn global_buckling_upper_assembly_matches_projected_symmetric_matrix() {
        let mut entries = Vec::new();
        let local = [[2., 5.], [0., 3.]];
        let rows = [vec![(0, 2.), (1, 1.)], vec![(0, -1.), (1, 2.)]];
        append(&mut entries, &rows, &local).unwrap();
        let sparse = SparseColMat::try_new_from_triplets(2, 2, &entries).unwrap();
        let image = apply_symmetric(&sparse, &Mat::identity(2, 2));
        // R^T [2 5; 5 3] R with rows (2,1), (-1,2).
        assert_eq!(
            (image[(0, 0)], image[(0, 1)], image[(1, 0)], image[(1, 1)]),
            (-9., 13., 13., 34.)
        );
    }

    #[test]
    fn global_buckling_sensitivity_matches_assembled_shell_and_rod_energies() {
        let material = Material::default();
        let scene = Scene {
            xyz: vec![
                [0., 0., 0.],
                [1000., 0., 0.],
                [1000., 1000., 0.],
                [0., 1000., 0.],
            ],
            plan: vec![],
            surfaces: vec![Surface {
                rigid_z_mm: 50.,
                nodes: vec![0, 1, 2, 3],
                material,
                display: vec![],
                support: None,
            }],
            fixed: Default::default(),
            fixed_rotations: Default::default(),
            support_nodes: Default::default(),
            beams: [[0, 1], [3, 2]]
                .into_iter()
                .enumerate()
                .map(|(i, nodes)| BeamGeometry {
                    rigid_offset_mm: [100., 30.],
                    roll_angle_rad: 0.2,
                    id: i as u64,
                    nodes,
                    section: super::super::super::beam::Section {
                        material,
                        width_mm: 200.,
                    },
                })
                .collect(),
        };
        let (cells, beams, ndof, _) = build(&scene, ElementType::Dkt).unwrap();
        let mut reference = vec![0.; ndof];
        reference[3] = -0.004;
        reference[6] = 0.001;
        let beam_loads = [[0.; 12]; 2];
        let mut projection = (0..ndof).map(|i| vec![(i, 1.)]).collect::<Vec<_>>();
        projection[0].clear();
        projection[8] = vec![(2, 0.1), (5, 0.5)];
        let map = (0..ndof).collect::<Vec<_>>();
        let scale = (0..ndof).map(|i| 0.5 + i as f64 / 20.).collect::<Vec<_>>();
        let mut mode = vec![0.; ndof];
        mode[5] = 0.01;
        let calculate_energy = |cells: &[Cell3], beams: &[BeamCell]| {
            let g = geometric(
                cells,
                beams,
                &reference,
                &beam_loads[..beams.len()],
                &projection,
                &map,
                &scale,
                ndof,
            )
            .unwrap();
            let vector = Mat::from_fn(ndof, 1, |i, _| mode[i]);
            let image = apply_symmetric(&g, &vector);
            (0..ndof).map(|i| mode[i] * image[(i, 0)]).sum::<f64>()
        };
        let total = calculate_energy(&cells, &beams);
        let expected = [
            calculate_energy(&cells, &[]),
            calculate_energy(&[], &beams[..1]),
            calculate_energy(&[], &beams[1..]),
        ]
        .map(|energy| 100. * energy / total);
        assert!(expected[1] > 0. && expected[2] < 0.);
        let scaled = mode.iter().map(|v| -3. * v).collect();
        let modes = [(1., mode), (1., scaled), (1., vec![0.; ndof])];
        let sensitivities = sensitivities(
            &cells,
            &beams,
            &reference,
            &beam_loads,
            &projection,
            &map,
            &scale,
            &modes,
        )
        .unwrap();
        for sensitivity in &sensitivities[..2] {
            let sensitivity = sensitivity.as_ref().unwrap();
            let actual = [
                sensitivity.surfaces[0],
                sensitivity.beams[0],
                sensitivity.beams[1],
            ];
            assert!((actual.iter().sum::<f32>() - 100.).abs() < 1e-4);
            for (actual, expected) in actual.into_iter().zip(expected) {
                assert!((f64::from(actual) - expected).abs() < 1e-5);
            }
        }
        assert!(sensitivities[2].is_none());
    }
}
