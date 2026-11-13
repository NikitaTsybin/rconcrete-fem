//! Conservative second-order shell/frame formulation with fixed reference axes.
//! Membrane and axial stretching use Green--Lagrange strains. Bending/torsion
//! use the existing small-rotation kernels. Loads have fixed world directions.
use super::*;
type Factor = faer::sparse::linalg::solvers::Llt<usize, f64>;

struct ElementState {
    force: Vec<f64>,
    tangent: Vec<Vec<f64>>,
}

impl Cell3 {
    pub(super) fn local_translations(&self, q: &[f64]) -> Vec<V3> {
        self.surface
            .nodes
            .iter()
            .map(|&n| {
                self.basis
                    .map(|axis| (0..3).map(|a| axis[a] * q[3 * n + a]).sum())
            })
            .collect()
    }
    fn samples(&self) -> Vec<(Vec<Point>, f64)> {
        match &self.kernel {
            Kernel::Dkt(t, _) => vec![(t.gradients().to_vec(), t.area)],
            Kernel::Quad(e) => super::super::mitc4::GAUSS
                .into_iter()
                .map(|p| {
                    let p = e.sample(p).expect("validated quad");
                    (p.grad.to_vec(), p.det)
                })
                .collect(),
        }
    }
    fn nonlinear_state(&self, q: &[f64]) -> ElementState {
        let local = self.local_translations(q);
        let zero = vec![[0.; 3]; local.len()];
        let rows = self
            .surface
            .nodes
            .iter()
            .flat_map(|&n| self.basis.map(|axis| translation(n, axis)))
            .collect::<Vec<_>>();
        let index = self
            .dofs
            .iter()
            .enumerate()
            .map(|(i, &d)| (d, i))
            .collect::<BTreeMap<_, _>>();
        let mut force = self
            .k
            .iter()
            .map(|r| r.iter().zip(&self.dofs).map(|(k, &d)| k * q[d]).sum())
            .collect::<Vec<f64>>();
        let mut tangent = self.k.clone();
        for (grad, weight) in self.samples() {
            let state = geometric_membrane::integrate(self.surface.material, &grad, &local, weight);
            let linear = geometric_membrane::integrate(self.surface.material, &grad, &zero, weight);
            let delta_k = state
                .tangent
                .iter()
                .zip(&linear.tangent)
                .map(|(a, b)| a.iter().zip(b).map(|(a, b)| a - b).collect::<Vec<_>>())
                .collect::<Vec<_>>();
            add_matrix(&mut tangent, &index, &rows, &delta_k);
            for (i, row) in rows.iter().enumerate() {
                let linear_force = linear.tangent[i]
                    .iter()
                    .enumerate()
                    .map(|(j, k)| k * local[j / 3][j % 3])
                    .sum::<f64>();
                let delta = state.force[i / 3][i % 3] - linear_force;
                for &(d, w) in row {
                    force[index[&d]] += w * delta;
                }
            }
        }
        ElementState { force, tangent }
    }
}

impl BeamCell {
    pub(super) fn add_second_order(
        &self,
        result: &mut super::super::beam::Forces,
        local: &[f64; 12],
    ) {
        let Some((base, forces)) = &self.base_axis else {
            return;
        };
        let current = result.ends.map(|v| [v[0], v[4], v[5]]);
        let a = super::super::beam::geometric_moments(self.kernel.length, local, forces);
        let b = super::super::beam::geometric_moments(self.kernel.length, base, &current);
        let c = super::super::beam::geometric_moments(self.kernel.length, base, forces);
        result.geometric_moments = Some(std::array::from_fn(|i| {
            std::array::from_fn(|j| a[i][j] + b[i][j] - c[i][j])
        }));
    }

    fn nonlinear_local(&self, q: &[f64]) -> ElementState {
        let q: [f64; 12] = std::array::from_fn(|i| value(&self.rows[i], q));
        beam_state(&self.kernel, &q)
    }
    fn nonlinear_state(&self, q: &[f64]) -> ElementState {
        let local = self.nonlinear_local(q);
        let index = self
            .dofs
            .iter()
            .enumerate()
            .map(|(i, &d)| (d, i))
            .collect::<BTreeMap<_, _>>();
        let mut tangent = vec![vec![0.; self.dofs.len()]; self.dofs.len()];
        add_matrix(&mut tangent, &index, &self.rows, &local.tangent);
        let mut force = vec![0.; self.dofs.len()];
        for (row, f) in self.rows.iter().zip(local.force) {
            for &(d, w) in row {
                force[index[&d]] += w * f;
            }
        }
        ElementState { force, tangent }
    }
}

// Consistent beam-column stretching energy integrated with cubic transverse
// displacement and constant axial gradient. This supplies P-delta terms and
// the matching internal force, including rotational nodal components.
fn beam_state(e: &super::beam::Element, q: &[f64; 12]) -> ElementState {
    let l = e.length;
    let ea = e.section.material.e_mpa * 1000. * e.section.area();
    let mut tangent = e.k.iter().map(|r| r.to_vec()).collect::<Vec<_>>();
    let mut force =
        e.k.iter()
            .map(|r| r.iter().zip(q).map(|(k, u)| k * u).sum())
            .collect::<Vec<f64>>();
    let mut axial = [0.; 12];
    axial[0] = -1. / l;
    axial[6] = 1. / l;
    let u = (q[6] - q[0]) / l;
    for i in [0, 6] {
        force[i] -= ea * l * axial[i] * u;
        for j in [0, 6] {
            tangent[i][j] -= ea * l * axial[i] * axial[j];
        }
    }
    for (t, weight) in [
        (0.5 - 0.5 * (3_f64 / 5.).sqrt(), 5. / 18.),
        (0.5, 4. / 9.),
        (0.5 + 0.5 * (3_f64 / 5.).sqrt(), 5. / 18.),
    ] {
        let h = [
            (-6. * t + 6. * t * t) / l,
            1. - 4. * t + 3. * t * t,
            (6. * t - 6. * t * t) / l,
            -2. * t + 3. * t * t,
        ];
        let mut gradients = [axial, [0.; 12], [0.; 12]];
        for (ids, sign, axis) in [([1, 5, 7, 11], 1., 1), ([2, 4, 8, 10], -1., 2)] {
            for i in 0..4 {
                gradients[axis][ids[i]] = h[i] * if i % 2 == 1 { sign } else { 1. };
            }
        }
        let g: V3 = gradients.map(|r| r.iter().zip(q).map(|(a, b)| a * b).sum());
        let strain = g[0] + 0.5 * dot(g, g);
        let b: [f64; 12] =
            std::array::from_fn(|i| axial[i] + (0..3).map(|a| g[a] * gradients[a][i]).sum::<f64>());
        for i in 0..12 {
            force[i] += ea * l * weight * strain * b[i];
            for j in 0..12 {
                let hessian = (0..3)
                    .map(|a| gradients[a][i] * gradients[a][j])
                    .sum::<f64>();
                tangent[i][j] += ea * l * weight * (b[i] * b[j] + strain * hessian);
            }
        }
    }
    ElementState { force, tangent }
}

struct State {
    springs: Vec<ElementState>,
    cells: Vec<ElementState>,
    beams: Vec<ElementState>,
    force: Vec<f64>,
}

struct System<'a> {
    springs: &'a [super::super::foundation::Spring],
    cells: &'a [Cell3],
    beams: &'a [BeamCell],
    projection: &'a [Row],
    map: &'a [usize],
    scale: &'a [f64],
    free: usize,
}
impl System<'_> {
    fn state(&self, q: &[f64]) -> State {
        let cells = self
            .cells
            .iter()
            .map(|c| c.nonlinear_state(q))
            .collect::<Vec<_>>();
        let beams = self
            .beams
            .iter()
            .map(|b| b.nonlinear_state(q))
            .collect::<Vec<_>>();
        let springs = self
            .springs
            .iter()
            .map(|s| ElementState {
                force: s
                    .k
                    .iter()
                    .map(|row| row.iter().zip(&s.dofs).map(|(v, &d)| v * q[d]).sum())
                    .collect(),
                tangent: s.k.clone(),
            })
            .collect::<Vec<_>>();
        let mut force = vec![0.; q.len()];
        for (dofs, s) in self
            .cells
            .iter()
            .map(|c| &c.dofs)
            .zip(&cells)
            .chain(self.beams.iter().map(|b| &b.dofs).zip(&beams))
            .chain(self.springs.iter().map(|s| &s.dofs).zip(&springs))
        {
            for (&d, &f) in dofs.iter().zip(&s.force) {
                force[d] += f;
            }
        }
        State {
            springs,
            cells,
            beams,
            force,
        }
    }
    fn factor(&self, state: &State) -> Result<Factor, String> {
        let mut triplets = Vec::new();
        for (dofs, s) in self
            .cells
            .iter()
            .map(|c| &c.dofs)
            .zip(&state.cells)
            .chain(self.beams.iter().map(|b| &b.dofs).zip(&state.beams))
            .chain(self.springs.iter().map(|s| &s.dofs).zip(&state.springs))
        {
            for (i, &a) in dofs.iter().enumerate() {
                for (j, &b) in dofs.iter().enumerate() {
                    let k = s.tangent[i][j];
                    if k == 0. {
                        continue;
                    }
                    for &(p, u) in &self.projection[a] {
                        for &(q, v) in &self.projection[b] {
                            if self.map[p] <= self.map[q] {
                                triplets.push(Triplet::new(
                                    self.map[p],
                                    self.map[q],
                                    k * u * v * self.scale[p] * self.scale[q],
                                ));
                            }
                        }
                    }
                }
            }
        }
        let matrix =
            SparseColMat::<usize, f64>::try_new_from_triplets(self.free, self.free, &triplets)
                .map_err(|e| format!("Нелинейная сборка: {e:?}"))?;
        matrix.sp_cholesky(Side::Upper).map_err(|e| {
            format!("Касательная жёсткость потеряла положительную определённость: {e:?}")
        })
    }
    fn reduced(&self, f: &[f64]) -> Mat<f64> {
        let gathered = super::super::rigid::gather(self.projection, f);
        let mut rhs = Mat::zeros(self.free, 1);
        for (i, &d) in self
            .map
            .iter()
            .enumerate()
            .filter(|(_, d)| **d != usize::MAX)
        {
            rhs[(d, 0)] = gathered[i] * self.scale[i];
        }
        rhs
    }
    fn expand(&self, x: &Mat<f64>) -> Vec<f64> {
        let parent = self
            .map
            .iter()
            .enumerate()
            .map(|(i, &d)| {
                if d == usize::MAX {
                    0.
                } else {
                    x[(d, 0)] * self.scale[i]
                }
            })
            .collect::<Vec<_>>();
        self.projection.iter().map(|r| value(r, &parent)).collect()
    }
    fn step(
        &self,
        start: &[f64],
        load: &[f64],
        fraction: f64,
    ) -> Result<(Vec<f64>, usize), String> {
        let target = load.iter().map(|v| v * fraction).collect::<Vec<_>>();
        let target_norm = norm(&self.reduced(&target)).max(1e-12);
        let mut q = start.to_vec();
        for iteration in 0..30 {
            super::super::progress::check_cancel()?;
            let state = self.state(&q);
            let residual = target
                .iter()
                .zip(&state.force)
                .map(|(a, b)| a - b)
                .collect::<Vec<_>>();
            let rhs = self.reduced(&residual);
            let error = norm(&rhs);
            super::super::progress::stage(&format!(
                "Ньютон: нагрузка {:.1}% · итерация {} · невязка {:.2e}",
                100. * fraction,
                iteration + 1,
                error / target_norm
            ));
            if error <= 1e-7 * target_norm + 1e-10 {
                return Ok((q, iteration));
            }
            let factor = self.factor(&state)?;
            let delta = self.expand(&factor.solve(rhs.as_ref()));
            let mut accepted = None;
            let mut alpha = 1.;
            for _ in 0..10 {
                let trial = q
                    .iter()
                    .zip(&delta)
                    .map(|(u, d)| u + alpha * d)
                    .collect::<Vec<_>>();
                if trial.iter().all(|v| v.is_finite()) {
                    let trial_state = self.state(&trial);
                    let r = target
                        .iter()
                        .zip(&trial_state.force)
                        .map(|(a, b)| a - b)
                        .collect::<Vec<_>>();
                    if norm(&self.reduced(&r)) < error * (1. - 1e-4 * alpha) {
                        accepted = Some(trial);
                        break;
                    }
                }
                alpha *= 0.5;
            }
            q = accepted.ok_or("Ньютон: не удалось уменьшить невязку")?;
        }
        Err("Ньютон: превышено число итераций шага".into())
    }
}
fn norm(x: &Mat<f64>) -> f64 {
    x.col(0).iter().map(|v| v * v).sum::<f64>().sqrt()
}

pub(super) struct Base {
    pub factor: Factor,
    pub offset_q: Vec<f64>,
    pub steps: usize,
    pub iterations: usize,
    pub reference_norm: f64,
}
pub(super) fn solve(
    springs: &[super::super::foundation::Spring],
    cells: &mut [Cell3],
    beams: &mut [BeamCell],
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    free: usize,
    load: &[f64],
    beam_loads: &[[f64; 12]],
) -> Result<Base, String> {
    let system = System {
        springs,
        cells,
        beams,
        projection,
        map,
        scale,
        free,
    };
    let reference_norm = norm(&system.reduced(load));
    if reference_norm <= 1e-14 {
        return Err("Базовое сочетание не создаёт нагрузки на свободные степени".into());
    }
    let mut q = vec![0.; load.len()];
    let mut fraction = 0_f64;
    let mut step = 0.25_f64;
    let mut steps = 0;
    let mut iterations = 0;

    while fraction < 1. - 1e-12 {
        super::super::progress::check_cancel()?;
        let next = (fraction + step).min(1.);
        match system.step(&q, load, next) {
            Ok((next_q, count)) => {
                q = next_q;
                fraction = next;
                steps += 1;
                iterations += count;
                if count < 5 {
                    step = (step * 1.5).min(0.25);
                }
            }
            Err(error) => {
                super::super::progress::check_cancel()?;
                step *= 0.5;
                if step < 1. / 256. {
                    return Err(format!(
                        "Нелинейный расчёт не сошёлся на {:.1}% нагрузки: {error}",
                        next * 100.
                    ));
                }
            }
        }
    }
    let state = system.state(&q);
    let factor = system.factor(&state)?;
    let linear_q = system.expand(&factor.solve(system.reduced(&state.force).as_ref()));
    let offset_q = q.iter().zip(linear_q).map(|(u, v)| u - v).collect();
    for (cell, state) in cells.iter_mut().zip(state.cells) {
        cell.membrane_base = Some(cell.local_translations(&q));
        cell.offset = state
            .force
            .iter()
            .enumerate()
            .map(|(i, f)| {
                f - state.tangent[i]
                    .iter()
                    .zip(&cell.dofs)
                    .map(|(k, &d)| k * q[d])
                    .sum::<f64>()
            })
            .collect();
        cell.k = state.tangent;
    }
    for (beam, load) in beams.iter_mut().zip(beam_loads) {
        let state = beam.nonlinear_local(&q);
        let local: [f64; 12] = std::array::from_fn(|i| value(&beam.rows[i], &q));
        let forces = [
            std::array::from_fn(|a| -state.force[a] + load[a]),
            std::array::from_fn(|a| state.force[6 + a] - load[6 + a]),
        ];
        beam.base_axis = Some((local, forces));
        beam.offset = std::array::from_fn(|i| {
            state.force[i]
                - state.tangent[i]
                    .iter()
                    .zip(local)
                    .map(|(k, u)| k * u)
                    .sum::<f64>()
        });
        beam.kernel.k = std::array::from_fn(|i| std::array::from_fn(|j| state.tangent[i][j]));
        let index = beam
            .dofs
            .iter()
            .enumerate()
            .map(|(i, &d)| (d, i))
            .collect::<BTreeMap<_, _>>();
        beam.k = vec![vec![0.; beam.dofs.len()]; beam.dofs.len()];
        add_matrix(&mut beam.k, &index, &beam.rows, &state.tangent);
    }
    Ok(Base {
        factor,
        offset_q,
        steps,
        iterations,
        reference_norm,
    })
}
#[cfg(test)]
mod tests {
    use super::*;
    fn kernel(length: f64) -> super::super::super::beam::Element {
        super::super::super::beam::Element::new(
            [0., 0.],
            [length * 1000., 0.],
            super::super::super::beam::Section {
                width_mm: 200.,
                material: Material::default(),
            },
        )
        .unwrap()
    }
    #[test]
    fn beam_tangent_is_derivative_of_internal_force() {
        let e = kernel(3.);
        let q = [
            0., 0., 0., 0., 0., 0., -0.002, 0.03, -0.02, 0.001, 0.004, 0.008,
        ];
        let s = beam_state(&e, &q);
        for j in 0..12 {
            let mut plus = q;
            let mut minus = q;
            let h = 1e-6;
            plus[j] += h;
            minus[j] -= h;
            let a = beam_state(&e, &plus);
            let b = beam_state(&e, &minus);
            for i in 0..12 {
                let fd = (a.force[i] - b.force[i]) / (2. * h);
                assert!(
                    (fd - s.tangent[i][j]).abs() < 1e-4 + 1e-7 * s.tangent[i][j].abs(),
                    "{i} {j}: {fd} != {}",
                    s.tangent[i][j]
                );
            }
        }
    }
    #[test]
    fn compressed_cantilever_amplifies_sway_and_converges_to_equilibrium() {
        let n = 6;
        let length = 3.;
        let dofs = 6 * (n + 1);
        let mut beams = (0..n)
            .map(|i| {
                let e = kernel(length / n as f64);
                let indices = (6 * i..6 * i + 12).collect::<Vec<_>>();
                BeamCell {
                    geometry: BeamGeometry {
                        id: i as u64,
                        nodes: [i, i + 1],
                        section: e.section.clone(),
                        rigid_offset_mm: [0.; 2],
                        roll_angle_rad: 0.,
                    },
                    rows: indices.iter().map(|&d| vec![(d, 1.)]).collect(),
                    dofs: indices,
                    k: e.k.iter().map(|r| r.to_vec()).collect(),
                    offset: [0.; 12],
                    base_axis: None,
                    kernel: e,
                }
            })
            .collect::<Vec<_>>();
        let e = &beams[0].kernel;
        let width = e.section.width_mm / 1000.;
        let h = e.section.material.thickness_mm / 1000.;
        let young = e.section.material.e_mpa * 1000.;
        let ei = young * h * width.powi(3) / 12.;
        let pcr = std::f64::consts::PI.powi(2) * ei / (4. * length * length);
        let p = 0.2 * pcr;
        let lateral = 1e-3;
        let projection = (0..dofs)
            .map(|d| if d < 6 { vec![] } else { vec![(d, 1.)] })
            .collect::<Vec<_>>();
        let map = (0..dofs)
            .map(|d| if d < 6 { usize::MAX } else { d - 6 })
            .collect::<Vec<_>>();
        let mut diagonal = vec![0.; dofs];
        for beam in &beams {
            for (i, &d) in beam.dofs.iter().enumerate() {
                diagonal[d] += beam.k[i][i];
            }
        }
        let scale = diagonal
            .into_iter()
            .map(|v| if v > 0. { 1. / v.sqrt() } else { 1. })
            .collect::<Vec<_>>();
        let mut load = vec![0.; dofs];
        load[6 * n] = -p;
        load[6 * n + 1] = lateral;
        let base = solve(
            &[],
            &mut [],
            &mut beams,
            &projection,
            &map,
            &scale,
            dofs - 6,
            &load,
            &vec![[0.; 12]; n],
        )
        .unwrap();
        let system = System {
            springs: &[],
            cells: &[],
            beams: &beams,
            projection: &projection,
            map: &map,
            scale: &scale,
            free: dofs - 6,
        };
        let linear = system.expand(&base.factor.solve(system.reduced(&load).as_ref()));
        let q = linear
            .iter()
            .zip(&base.offset_q)
            .map(|(a, b)| a + b)
            .collect::<Vec<_>>();
        let k = (p / ei).sqrt();
        let analytical = lateral / p * ((k * length).tan() / k - length);
        let relative = (q[6 * n + 1] / analytical - 1.).abs();
        assert!(
            relative < 0.02,
            "deflection={}, analytical={analytical}, relative={relative}",
            q[6 * n + 1]
        );
        assert!(base.iterations > 0 && base.steps >= 4);
        for beam in &beams {
            let local = std::array::from_fn(|i| value(&beam.rows[i], &q));
            let mut recovered = beam.kernel.recover(
                beam.geometry.id,
                [[0., 0.], [500., 0.]],
                local,
                beam.offset.map(|v| -v),
            );
            beam.add_second_order(&mut recovered, &local);
            let right = recovered.at(1.);
            assert!((right[2] - recovered.ends[1][2]).abs() < 1e-5);
            assert!((right[3] - recovered.ends[1][3]).abs() < 1e-5);
        }
        let mut force = vec![0.; dofs];
        for beam in &beams {
            let local = beam.rows.iter().map(|r| value(r, &q)).collect::<Vec<_>>();
            for i in 0..12 {
                let action = beam.kernel.k[i]
                    .iter()
                    .zip(&local)
                    .map(|(k, u)| k * u)
                    .sum::<f64>()
                    + beam.offset[i];
                scatter(&beam.rows[i], action, &mut force);
            }
        }
        let residual = force
            .iter()
            .zip(&load)
            .map(|(a, b)| a - b)
            .collect::<Vec<_>>();
        assert!(norm(&system.reduced(&residual)) < 1e-7 * norm(&system.reduced(&load)));
    }
    #[test]
    fn shell_tangent_and_actions_are_consistent_on_triangle_and_quad() {
        for nodes in [vec![0, 1, 2], vec![0, 1, 2, 3]] {
            let scene = Scene {
                xyz: vec![
                    [0., 0., 0.],
                    [2000., 0., 0.],
                    [2000., 2000., 0.],
                    [0., 2000., 0.],
                ],
                plan: vec![0, 1, 2, 3],
                surfaces: vec![Surface {
                    rigid_z_mm: 0.,
                    nodes,
                    material: Material::default(),
                    display: vec![],
                    support: None,
                }],
                fixed: Default::default(),
                fixed_rotations: Default::default(),
                support_nodes: Default::default(),
                beams: vec![],
            };
            let (cells, _, dofs, _) = build(&scene, ElementType::Dkt).unwrap();
            let cell = &cells[0];
            let mut q = vec![0.; dofs];
            q[3] = -0.002;
            q[5] = 0.05;
            q[8] = -0.03;
            q[10] = 0.001;
            let state = cell.nonlinear_state(&q);
            let zero = cell.nonlinear_state(&vec![0.; dofs]);
            for i in 0..cell.dofs.len() {
                for j in 0..cell.dofs.len() {
                    assert!((zero.tangent[i][j] - cell.k[i][j]).abs() < 1e-8);
                }
            }
            for (j, &d) in cell.dofs.iter().enumerate() {
                let h = 1e-6;
                let mut plus = q.clone();
                plus[d] += h;
                let mut minus = q.clone();
                minus[d] -= h;
                let plus = cell.nonlinear_state(&plus);
                let minus = cell.nonlinear_state(&minus);
                for i in 0..cell.dofs.len() {
                    let fd = (plus.force[i] - minus.force[i]) / (2. * h);
                    assert!(
                        (fd - state.tangent[i][j]).abs() < 1e-3 + 1e-7 * state.tangent[i][j].abs()
                    );
                }
            }
        }
    }
}
