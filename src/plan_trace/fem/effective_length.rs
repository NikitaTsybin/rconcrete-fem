use super::*;

#[derive(serde::Serialize, serde::Deserialize)]
pub(crate) struct Length {
    pub id: u64,
    pub axis: String,
    #[serde(default)]
    pub segment: Option<[[f64; 2]; 2]>,
    pub length_m: f64,
    pub bottom_m: f64,
    pub effective_m: Option<f64>,
    pub critical_kn: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub diagnostic: Option<String>,
}

struct Probe {
    id: u64,
    axis: &'static str,
    length: f64,
    bottom: f64,
    ei: f64,
    rows: [BTreeMap<usize, f64>; 4],
}

pub(super) fn compute(
    scene: &Scene,
    all: &[Contour],
    building: bool,
    settings: &Settings,
    rotations: &[[Row; 3]],
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    free: usize,
    solve: impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<Length>, String> {
    let mut probes = Vec::new();
    let mut add = |id, axis, edges: [Vec<usize>; 2], direction: [f64; 3], ei| {
        let bottom = scene.xyz[edges[0][0]][2] / 1000.;
        let length = scene.xyz[edges[1][0]][2] / 1000. - bottom;
        if length <= 0. {
            return;
        }
        let turn = [-direction[1], direction[0], 0.];
        let rows = std::array::from_fn(|j| {
            let edge = &edges[j / 2];
            let mut row = BTreeMap::new();
            for &node in edge {
                for a in 0..3 {
                    let terms = if j % 2 == 0 {
                        vec![(3 * node + a, direction[a])]
                    } else {
                        rotations[node][a]
                            .iter()
                            .map(|&(d, w)| (d, w * turn[a] * length))
                            .collect()
                    };
                    for (d, w) in terms {
                        for &(p, u) in &projection[d] {
                            *row.entry(map[p]).or_insert(0.) +=
                                w * u * scale[p] / edge.len() as f64;
                        }
                    }
                }
            }
            row
        });
        probes.push(Probe {
            id,
            axis,
            length,
            bottom,
            ei,
            rows,
        });
    };
    for beam in scene.beams.iter().filter(|b| {
        settings.columns.contains_key(&b.id)
            && all
                .iter()
                .any(|c| c.id == b.id && c.kind == Kind::Column && c.calculate_effective_length)
    }) {
        let mut nodes = beam.nodes;
        nodes.sort_by(|&a, &b| scene.xyz[a][2].total_cmp(&scene.xyz[b][2]));
        let kernel = super::super::beam::Element::spatial(
            scene.xyz[beam.nodes[0]],
            scene.xyz[beam.nodes[1]],
            beam.section.clone(),
        )?;
        let [_, y, z] = kernel.basis;
        let (sin, cos) = beam.roll_angle_rad.sin_cos();
        let y_rot = std::array::from_fn(|i| cos * y[i] + sin * z[i]);
        let z_rot = std::array::from_fn(|i| -sin * y[i] + cos * z[i]);
        let b = beam.section.width_mm / 1000.;
        let h = beam.section.material.thickness_mm / 1000.;
        let e = beam.section.material.e_mpa * 1000.;
        add(
            beam.id,
            "z",
            nodes.map(|n| vec![n]),
            y_rot,
            e * h * b.powi(3) / 12.,
        );
        add(
            beam.id,
            "y",
            nodes.map(|n| vec![n]),
            z_rot,
            e * b * h.powi(3) / 12.,
        );
    }
    let mut out = wall_lengths(
        scene, all, building, projection, map, scale, free, &solve, false,
    )?;
    for batch in probes.chunks(8) {
        super::super::progress::check_cancel()?;
        let mut rhs = Mat::zeros(free, batch.len() * 4);
        for (i, p) in batch.iter().enumerate() {
            for (j, row) in p.rows.iter().enumerate() {
                for (&d, &w) in row {
                    rhs[(d, 4 * i + j)] = w;
                }
            }
        }
        let x = solve(&rhs);
        for (i, p) in batch.iter().enumerate() {
            let c = Mat::from_fn(4, 4, |a, b| {
                p.rows[a]
                    .iter()
                    .map(|(&d, &w)| w * x[(d, 4 * i + b)])
                    .sum::<f64>()
                    * p.ei
                    / p.length.powi(3)
            });
            let value = critical(&c);
            out.push(Length {
                id: p.id,
                segment: None,
                axis: p.axis.into(),
                length_m: p.length,
                bottom_m: p.bottom,
                effective_m: value.map(|v| std::f64::consts::PI * p.length / v.sqrt()),
                critical_kn: value.map(|v| v * p.ei / p.length.powi(2)),
                diagnostic: None,
            });
        }
    }
    Ok(out)
}

// Keep the elastic operator of the whole model. Only the chosen panel receives
// vertical reference compression. Horizontal slabs have no in-plane component
// along global Z, so they do not enter this wall-panel calculation.
pub(super) fn wall_lengths(
    scene: &Scene,
    all: &[Contour],
    building: bool,
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    free: usize,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
    spatial: bool,
) -> Result<Vec<Length>, String> {
    let mut faces = BTreeMap::<u64, Vec<&Surface>>::new();
    let mut owners = BTreeMap::<usize, BTreeSet<u64>>::new();
    for face in &scene.surfaces {
        if let Some(id) = face.support {
            faces.entry(id).or_default().push(face);
            for &n in &face.nodes {
                owners.entry(n).or_default().insert(id);
            }
        }
    }
    let mut output = Vec::new();
    let mut operators = Vec::new();
    let mut targets = Vec::new();
    for wall in all
        .iter()
        .filter(|c| c.kind == Kind::Support && c.calculate_effective_length)
    {
        let Some(faces) = faces.get(&wall.id) else {
            continue;
        };
        let mut path = wall.points.clone();
        if wall.closed() && path.first() != path.last() {
            if let Some(&start) = path.first() {
                path.push(start);
            }
        }
        let mut runs: Vec<[[f64; 2]; 2]> = Vec::new();
        for edge in path.windows(2) {
            let [a, b] = [edge[0], edge[1]];
            if (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-6 {
                continue;
            }
            if let Some(run) = runs.last_mut() {
                let [x, y] = *run;
                let u = [y[0] - x[0], y[1] - x[1]];
                let v = [b[0] - a[0], b[1] - a[1]];
                if (u[0] * v[1] - u[1] * v[0]).abs() <= 1e-6 * u[0].hypot(u[1])
                    && u[0] * v[0] + u[1] * v[1] > 0.
                {
                    run[1] = b;
                    continue;
                }
            }
            runs.push([a, b]);
        }
        for [a, b] in runs {
            let width = (b[0] - a[0]).hypot(b[1] - a[1]);
            let dir = [(b[0] - a[0]) / width, (b[1] - a[1]) / width];
            let normal = [-dir[1], dir[0]];
            let local = |p: V3| {
                let x = p[0] - a[0];
                let y = p[1] - a[1];
                (x * dir[0] + y * dir[1], x * normal[0] + y * normal[1])
            };
            let on_run = |n| {
                let (x, d) = local(scene.xyz[n]);
                d.abs() < 1e-4 && x >= -1e-4 && x <= width + 1e-4
            };
            let run_faces = faces
                .iter()
                .copied()
                .filter(|f| f.nodes.iter().copied().all(on_run))
                .collect::<Vec<_>>();
            if run_faces.is_empty() {
                continue;
            }
            let nodes = run_faces
                .iter()
                .flat_map(|f| f.nodes.iter().copied())
                .collect::<BTreeSet<_>>();
            let low = nodes
                .iter()
                .map(|&n| scene.xyz[n][2])
                .fold(f64::INFINITY, f64::min);
            let high = nodes
                .iter()
                .map(|&n| scene.xyz[n][2])
                .fold(f64::NEG_INFINITY, f64::max);
            let mut cuts = vec![0., width];
            let mut joints = BTreeMap::<(i64, u64), (f64, f64)>::new();
            for &n in &nodes {
                let z = scene.xyz[n][2];
                let x = local(scene.xyz[n]).0;
                for &neighbor in owners[&n].iter().filter(|&&id| id != wall.id) {
                    let extents = joints
                        .entry(((x * 1e4).round() as i64, neighbor))
                        .or_insert((z, z));
                    extents.0 = extents.0.min(z);
                    extents.1 = extents.1.max(z);
                }
            }
            for ((x, _), (lo, hi)) in joints {
                if hi - lo > 1e-4 {
                    cuts.push((x as f64 / 1e4).clamp(0., width));
                }
            }
            cuts.sort_by(f64::total_cmp);
            cuts.dedup_by(|x, y| (*x - *y).abs() < 1e-4);
            let levels = if !building
                && low < -1e-4
                && high > 1e-4
                && nodes.iter().any(|&n| scene.xyz[n][2].abs() < 1e-4)
            {
                vec![low, 0., high]
            } else {
                vec![low, high]
            };
            for edge in cuts.windows(2) {
                let [start, end] = [edge[0], edge[1]];
                for level in levels.windows(2) {
                    let [bottom, top] = [level[0], level[1]];
                    let panel = run_faces
                        .iter()
                        .copied()
                        .filter(|f| {
                            let (x, z) = f.nodes.iter().fold((0., 0.), |(x, z), &n| {
                                (x + local(scene.xyz[n]).0, z + scene.xyz[n][2])
                            });
                            let count = f.nodes.len() as f64;
                            (start..end).contains(&(x / count))
                                && (bottom..top).contains(&(z / count))
                        })
                        .collect::<Vec<_>>();
                    if panel.is_empty() || top - bottom < 1e-4 {
                        continue;
                    }
                    super::super::progress::check_cancel()?;
                    let width_m = (end - start) / 1000.;
                    let material = panel[0].material;
                    let ei =
                        material.e_mpa * 1000. * width_m * (material.thickness_mm / 1000.).powi(3)
                            / 12.;
                    let diagnostic = match panel_operator(
                        scene, &panel, normal, dir, width_m, projection, map, scale, spatial,
                    )? {
                        PanelPreparation::Ready(operator) => {
                            targets.push((output.len(), ei));
                            operators.push(operator);
                            None
                        }
                        PanelPreparation::Unavailable(reason) => spatial.then(|| reason.into()),
                    };
                    output.push(Length {
                        id: wall.id,
                        axis: "Из плоскости".into(),
                        segment: Some([start, end].map(|x| [a[0] + dir[0] * x, a[1] + dir[1] * x])),
                        length_m: (top - bottom) / 1000.,
                        bottom_m: bottom / 1000.,
                        effective_m: None,
                        critical_kn: None,
                        diagnostic,
                    });
                }
            }
        }
    }
    let values = solve_panels(&operators, free, solve)?;
    for ((index, ei), value) in targets.into_iter().zip(values) {
        let result = &mut output[index];
        if let Some(value) = value {
            result.critical_kn = Some(value.force_kn);
            let mut out_of_plane = true;
            if let Some(share) = value.in_plane_share {
                out_of_plane = value.independent_modes || share < MIN_PANEL_PARTICIPATION;
                let class = if out_of_plane {
                    "Из плоскости"
                } else if 1. - share < MIN_PANEL_PARTICIPATION {
                    "В плоскости"
                } else {
                    "Смешанная форма"
                };
                result.axis = class.into();
                result.diagnostic = Some(if value.independent_modes {
                    "Независимые формы в плоскости и из плоскости стены с совпадающей в пределах допуска критической силой".into()
                } else {
                    format!(
                        "{class}. Доля геометрической энергии в плоскости стены {:.2}%",
                        100. * share,
                    )
                });
            }
            if out_of_plane {
                result.effective_m = Some(std::f64::consts::PI * (ei / value.force_kn).sqrt());
            }
        } else if spatial {
            result.diagnostic = Some("Не найдена сходящаяся положительная критическая сила".into());
        }
    }
    Ok(output)
}

enum PanelPreparation {
    Ready(Panel),
    Unavailable(&'static str),
}

fn panel_operator(
    scene: &Scene,
    faces: &[&Surface],
    normal: [f64; 2],
    direction: [f64; 2],
    width: f64,
    projection: &[Row],
    map: &[usize],
    scale: &[f64],
    spatial: bool,
) -> Result<PanelPreparation, String> {
    let nodes = faces
        .iter()
        .flat_map(|f| f.nodes.iter().copied())
        .collect::<BTreeSet<_>>();
    let n = nodes.len();
    if n > 1024 {
        return Ok(PanelPreparation::Unavailable(
            "Панель содержит более 1024 узлов",
        ));
    }
    let axes = [normal, direction];
    let rows = axes[..if spatial { 2 } else { 1 }]
        .iter()
        .flat_map(|&axis| nodes.iter().map(move |&node| (axis, node)))
        .map(|(axis, n)| {
            let mut row = BTreeMap::<usize, f64>::new();
            for (component, weight) in axis.into_iter().enumerate() {
                for &(d, w) in &projection[3 * n + component] {
                    *row.entry(map[d]).or_default() += w * scale[d] * weight;
                }
            }
            row
        })
        .collect::<Vec<_>>();
    if rows.iter().all(|r| r.values().all(|v| v.abs() < 1e-15)) {
        return Ok(PanelPreparation::Unavailable(
            "Нет свободных поперечных перемещений узлов",
        ));
    }
    let index = nodes
        .iter()
        .enumerate()
        .map(|(i, &n)| (n, i))
        .collect::<BTreeMap<_, _>>();
    let mut geometric = Mat::<f64>::zeros(n, n);
    for face in faces {
        if spatial && face.nodes.len() == 4 {
            let mut xy = std::array::from_fn(|i| {
                let d = sub(scene.xyz[face.nodes[i]], scene.xyz[face.nodes[0]]);
                [
                    (d[0] * direction[0] + d[1] * direction[1]) / 1000.,
                    d[2] / 1000.,
                ]
            });
            if xy[1][0] * xy[3][1] - xy[3][0] * xy[1][1] < 0. {
                for point in &mut xy {
                    point[0] = -point[0];
                }
            }
            let quad = super::super::mitc4::Quad4::new(xy)?;
            for natural in super::super::mitc4::GAUSS {
                let sample = quad.sample(natural)?;
                for (i, ni) in face.nodes.iter().enumerate() {
                    for (j, nj) in face.nodes.iter().enumerate() {
                        geometric[(index[ni], index[nj])] +=
                            sample.grad[i][1] * sample.grad[j][1] * sample.det / width;
                    }
                }
            }
            continue;
        }
        for edge in face.nodes[1..].windows(2) {
            let ids = [face.nodes[0], edge[0], edge[1]];
            let points = ids.map(|i| scene.xyz[i]);
            let a = sub(points[1], points[0]).map(|x| x / 1000.);
            let b = sub(points[2], points[0]).map(|x| x / 1000.);
            let dx = [
                a[0] * direction[0] + a[1] * direction[1],
                b[0] * direction[0] + b[1] * direction[1],
            ];
            let area2 = (dx[0] * b[2] - dx[1] * a[2]).abs();
            if area2 <= 1e-12 {
                continue;
            }
            let dz = [dx[1] - dx[0], -dx[1], dx[0]];
            for i in 0..3 {
                for j in 0..3 {
                    geometric[(index[&ids[i]], index[&ids[j]])] +=
                        dz[i] * dz[j] / (2. * width * area2);
                }
            }
        }
    }
    let eigen = geometric
        .self_adjoint_eigen(Side::Lower)
        .map_err(|e| format!("Геометрическая жёсткость стены: {e:?}"))?;
    let max = eigen.S()[n - 1];
    let active = (0..n)
        .filter(|&i| eigen.S()[i] > max * 1e-12 && eigen.S()[i] > 0.)
        .collect::<Vec<_>>();
    if active.is_empty() {
        return Ok(PanelPreparation::Unavailable(
            "Нулевая геометрическая жёсткость панели",
        ));
    }
    let basis = Mat::from_fn(n, active.len(), |i, j| {
        eigen.U()[(i, active[j])] * eigen.S()[active[j]].sqrt()
    });
    Ok(PanelPreparation::Ready(Panel { rows, basis }))
}

struct Panel {
    rows: Vec<BTreeMap<usize, f64>>,
    basis: Mat<f64>,
}
impl Panel {
    fn dimension(&self) -> usize {
        self.basis.ncols() * (self.rows.len() / self.basis.nrows())
    }

    fn mode_split(&self) -> Option<usize> {
        (self.rows.len() > self.basis.nrows()).then(|| self.basis.ncols())
    }

    fn scatter(&self, q: &[f64], rhs: &mut Mat<f64>, column: usize) {
        for (rows, q) in self
            .rows
            .chunks_exact(self.basis.nrows())
            .zip(q.chunks_exact(self.basis.ncols()))
        {
            for (i, row) in rows.iter().enumerate() {
                let value = q
                    .iter()
                    .enumerate()
                    .map(|(j, q)| self.basis[(i, j)] * q)
                    .sum::<f64>();
                for (&d, &w) in row {
                    rhs[(d, column)] += w * value;
                }
            }
        }
    }

    fn gather(&self, solution: &Mat<f64>, column: usize) -> Vec<f64> {
        let mut projected = Vec::with_capacity(self.dimension());
        for rows in self.rows.chunks_exact(self.basis.nrows()) {
            let local = rows
                .iter()
                .map(|row| {
                    row.iter()
                        .map(|(&d, &w)| w * solution[(d, column)])
                        .sum::<f64>()
                })
                .collect::<Vec<_>>();
            projected.extend((0..self.basis.ncols()).map(|j| {
                local
                    .iter()
                    .enumerate()
                    .map(|(i, u)| self.basis[(i, j)] * u)
                    .sum::<f64>()
            }));
        }
        projected
    }
}

struct PanelCritical {
    force_kn: f64,
    in_plane_share: Option<f64>,
    independent_modes: bool,
}

const MIN_PANEL_PARTICIPATION: f64 = 0.01;
const PANEL_RESIDUAL_TOLERANCE: f64 = 1e-6;

struct Krylov {
    vectors: Vec<Vec<f64>>,
    diagonal: Vec<f64>,
    off_diagonal: Vec<f64>,
    value: Option<PanelCritical>,
    normal_probe: Option<Vec<f64>>,
    finished: bool,
}
impl Krylov {
    fn new(size: usize) -> Self {
        let mut seed = 7919_u64;
        let mut q = (0..size)
            .map(|_| {
                seed ^= seed << 13;
                seed ^= seed >> 7;
                seed ^= seed << 17;
                (seed >> 11) as f64 / (1_u64 << 53) as f64 - 0.5
            })
            .collect::<Vec<_>>();
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        for v in &mut q {
            *v /= norm;
        }
        Self {
            vectors: vec![q],
            diagonal: vec![],
            off_diagonal: vec![],
            value: None,
            normal_probe: None,
            finished: false,
        }
    }
    fn advance(&mut self, mut image: Vec<f64>, mode_split: Option<usize>) -> Result<(), String> {
        let k = self.diagonal.len();
        let q = &self.vectors[k];
        let alpha = q.iter().zip(&image).map(|(a, b)| a * b).sum();
        self.diagonal.push(alpha);
        // Full two-pass reorthogonalization prevents repeated Ritz roots.
        for _ in 0..2 {
            for vector in &self.vectors {
                let dot = vector.iter().zip(&image).map(|(a, b)| a * b).sum::<f64>();
                for (v, q) in image.iter_mut().zip(vector) {
                    *v -= dot * q;
                }
            }
        }
        let beta = image.iter().map(|v| v * v).sum::<f64>().sqrt();
        let order = self.diagonal.len();
        let small = Mat::from_fn(order, order, |i, j| {
            if i == j {
                self.diagonal[i]
            } else if i + 1 == j {
                self.off_diagonal[i]
            } else if j + 1 == i {
                self.off_diagonal[j]
            } else {
                0.
            }
        });
        let eig = small
            .self_adjoint_eigen(Side::Lower)
            .map_err(|e| format!("Устойчивость панели: {e:?}"))?;
        let largest = eig.S()[order - 1];
        let residual = beta * eig.U()[(order - 1, order - 1)].abs();
        if largest > 1e-20 && residual <= PANEL_RESIDUAL_TOLERANCE * largest {
            let in_plane_share = mode_split.map(|split| {
                // For an eigenvector q of Lᵀ R K⁻¹ Rᵀ L, its two block norms
                // give the directional shares of uᵀ Rᵀ G R u, since Lᵀ R u = λq.
                let mut mode = vec![0.; q.len()];
                for (j, vector) in self.vectors.iter().enumerate() {
                    let coefficient = eig.U()[(j, order - 1)];
                    for (v, &q) in mode.iter_mut().zip(vector) {
                        *v += coefficient * q;
                    }
                }
                let total = mode.iter().map(|v| v * v).sum::<f64>();
                let share = mode[split..].iter().map(|v| v * v).sum::<f64>() / total;
                if share >= MIN_PANEL_PARTICIPATION && 1. - share >= MIN_PANEL_PARTICIPATION {
                    mode[split..].fill(0.);
                    self.normal_probe = Some(mode);
                }
                share
            });
            self.value = Some(PanelCritical {
                force_kn: 1. / largest,
                in_plane_share,
                independent_modes: false,
            });
            self.finished = true;
        } else if beta <= 1e-14 * largest.abs().max(1e-20) || order >= q.len() || order >= 80 {
            self.finished = true;
        } else {
            for v in &mut image {
                *v /= beta;
            }
            self.off_diagonal.push(beta);
            self.vectors.push(image);
        }
        Ok(())
    }
}

fn solve_panels(
    panels: &[Panel],
    free: usize,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<Option<PanelCritical>>, String> {
    let start = web_time::Instant::now();
    let mut states = panels
        .iter()
        .map(|p| Krylov::new(p.dimension()))
        .collect::<Vec<_>>();
    let mut rhs_count = 0;
    loop {
        let active = states
            .iter()
            .enumerate()
            .filter_map(|(i, s)| (!s.finished).then_some(i))
            .collect::<Vec<_>>();
        if active.is_empty() {
            break;
        }
        for batch in active.chunks(32) {
            super::super::progress::check_cancel()?;
            let mut rhs = Mat::zeros(free, batch.len());
            for (j, &i) in batch.iter().enumerate() {
                panels[i].scatter(states[i].vectors.last().unwrap(), &mut rhs, j);
            }
            let solution = solve(&rhs);
            rhs_count += batch.len();
            for (j, &i) in batch.iter().enumerate() {
                let panel = &panels[i];
                states[i].advance(panel.gather(&solution, j), panel.mode_split())?;
            }
        }
        let done = states.iter().filter(|s| s.finished).count();
        super::super::progress::stage(&format!(
            "Расчётные длины: панели {done}/{} · проб {rhs_count}",
            panels.len()
        ));
    }
    // A single Ritz vector can mix independent modes at a repeated eigenvalue.
    // Check whether its normal-only projection still satisfies the eigenproblem.
    let probes = states
        .iter()
        .enumerate()
        .filter_map(|(i, state)| state.normal_probe.as_ref().map(|_| i))
        .collect::<Vec<_>>();
    for batch in probes.chunks(32) {
        super::super::progress::check_cancel()?;
        let mut rhs = Mat::zeros(free, batch.len());
        for (j, &i) in batch.iter().enumerate() {
            panels[i].scatter(states[i].normal_probe.as_ref().unwrap(), &mut rhs, j);
        }
        let solution = solve(&rhs);
        rhs_count += batch.len();
        for (j, &i) in batch.iter().enumerate() {
            let state = &mut states[i];
            let probe = state.normal_probe.take().unwrap();
            let critical = state.value.as_mut().unwrap();
            let eigenvalue = 1. / critical.force_kn;
            let image = panels[i].gather(&solution, j);
            let residual = image
                .iter()
                .zip(&probe)
                .map(|(v, q)| (v - eigenvalue * q).powi(2))
                .sum::<f64>()
                .sqrt();
            let scale = eigenvalue * probe.iter().map(|q| q * q).sum::<f64>().sqrt();
            critical.independent_modes = residual <= 4. * PANEL_RESIDUAL_TOLERANCE * scale;
        }
    }
    #[cfg(test)]
    {
        eprintln!(
            "wall lengths: panels={}, dense_rhs={}, iterative_rhs={rhs_count}, elapsed={:.3}s",
            panels.len(),
            panels.iter().map(Panel::dimension).sum::<usize>(),
            start.elapsed().as_secs_f64()
        );
        if let Ok(count) = std::env::var("MINI_FEM_LENGTH_REFERENCE_PANELS")
            .unwrap_or_default()
            .parse::<usize>()
        {
            for (i, panel) in panels.iter().take(count).enumerate() {
                let reference = dense_reference(panel, free, solve)?;
                let iterative = states[i].value.as_ref().map(|v| v.force_kn);
                eprintln!("panel {i}: iterative={iterative:?}, dense={reference:?}");
                if let (Some(a), Some(b)) = (iterative, reference) {
                    assert!((a / b - 1.).abs() < 1e-5);
                } else {
                    assert_eq!(iterative, reference);
                }
            }
        }
    }
    let _ = start;
    Ok(states.into_iter().map(|s| s.value).collect())
}

#[cfg(test)]
fn dense_reference(
    panel: &Panel,
    free: usize,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Option<f64>, String> {
    let n = panel.dimension();
    let mut flexibility = Mat::zeros(n, n);
    for start in (0..n).step_by(32) {
        let count = 32.min(n - start);
        let mut rhs = Mat::zeros(free, count);
        for j in 0..count {
            let q = (0..n)
                .map(|i| if i == start + j { 1. } else { 0. })
                .collect::<Vec<_>>();
            panel.scatter(&q, &mut rhs, j);
        }
        let solution = solve(&rhs);
        for j in 0..count {
            for (i, v) in panel.gather(&solution, j).into_iter().enumerate() {
                flexibility[(i, start + j)] = v;
            }
        }
    }
    let values = flexibility
        .self_adjoint_eigenvalues(Side::Lower)
        .map_err(|e| format!("{e:?}"))?;
    Ok(values
        .iter()
        .last()
        .filter(|&&v| v > 1e-20 && v.is_finite())
        .map(|v| 1. / v))
}

#[test]
fn batched_panel_iteration_matches_complete_flexibility() {
    let panels = (0..4)
        .map(|shift| Panel {
            rows: (0..20)
                .map(|i| BTreeMap::from([((i + shift) % 30, 1. + i as f64 / 20.)]))
                .collect(),
            basis: Mat::from_fn(20, 18, |i, j| {
                if i == j {
                    1. + i as f64
                } else {
                    ((i * 17 + j * 23 + shift) as f64).sin() * 0.05
                }
            }),
        })
        .collect::<Vec<_>>();
    let solve = |rhs: &Mat<f64>| {
        Mat::from_fn(rhs.nrows(), rhs.ncols(), |i, j| {
            rhs[(i, j)] / (i + 1) as f64
        })
    };
    let iterative = solve_panels(&panels, 30, &solve).unwrap();
    for (panel, value) in panels.iter().zip(iterative) {
        let reference = dense_reference(panel, 30, &solve).unwrap();
        assert!((value.unwrap().force_kn / reference.unwrap() - 1.).abs() < 1e-6);
    }
}

#[test]
fn spatial_wall_panel_resolves_pure_and_mixed_forms() {
    let along = [0.6, 0.8];
    let normal = [-0.8, 0.6];
    let a = [10_000., -20_000.];
    let b = [a[0] + 3000. * along[0], a[1] + 3000. * along[1]];
    let wall = Contour {
        id: 1,
        kind: Kind::Support,
        points: vec![a, b],
        calculate_effective_length: true,
        support_open: true,
        name: String::new(),
        placement: Default::default(),
        mesh_size_mm: 0.,
        column_angle: 0.,
        restraint_mask: 0,
        use_foundation: false,
        foundation: None,
        grid: None,
        load: None,
        refinement: None,
        parent: None,
        curves: vec![],
    };
    let scene = Scene {
        xyz: vec![
            [a[0], a[1], 0.],
            [b[0], b[1], 0.],
            [b[0], b[1], 2000.],
            [a[0], a[1], 2000.],
        ],
        surfaces: vec![Surface {
            rigid_z_mm: 0.,
            nodes: vec![0, 3, 2, 1],
            material: Material::wall_default(),
            display: vec![],
            support: Some(1),
        }],
        plan: vec![],
        fixed: BTreeSet::new(),
        support_nodes: BTreeMap::new(),
        fixed_rotations: BTreeSet::new(),
        beams: vec![],
    };
    // Two independent interface coordinates translate the top line as a whole.
    // For this 2 m rectangle the exact integrated geometric weight is 1/2.
    let mut projection = vec![vec![]; 12];
    for node in [2, 3] {
        for axis in 0..2 {
            projection[3 * node + axis] = vec![(0, normal[axis]), (1, along[axis])];
        }
    }
    let all = [wall];
    for (c, root, class, has_l0) in [
        ([[5., 0.], [0., 2.]], 5., "Из плоскости", true),
        ([[2., 0.], [0., 5.]], 5., "В плоскости", false),
        ([[5., 2.], [2., 2.]], 6., "Смешанная форма", false),
        ([[5., 0.], [0., 5.]], 5., "Из плоскости", true),
    ] {
        let solve = |rhs: &Mat<f64>| {
            Mat::from_fn(2, rhs.ncols(), |i, j| {
                c[i][0] * rhs[(0, j)] + c[i][1] * rhs[(1, j)]
            })
        };
        let run = |spatial| {
            wall_lengths(
                &scene,
                &all,
                true,
                &projection,
                &[0, 1],
                &[1., 1.],
                2,
                &solve,
                spatial,
            )
            .unwrap()
            .pop()
            .unwrap()
        };
        let spatial = run(true);
        assert!((spatial.critical_kn.unwrap() - 2. / root).abs() < 1e-9);
        assert_eq!(spatial.axis, class);
        assert_eq!(spatial.effective_m.is_some(), has_l0);
        let diagnostic = spatial.diagnostic.unwrap();
        if root == 6. {
            // Largest eigenvector of this compliance is proportional to [2, 1].
            assert!(diagnostic.contains("20.00%"));
        }
        assert_eq!(diagnostic.contains("Независимые формы"), c[0][0] == c[1][1]);
        let legacy = run(false);
        assert!((legacy.critical_kn.unwrap() - 2. / c[0][0]).abs() < 1e-9);
        assert!(legacy.effective_m.is_some());
    }
    let fixed = wall_lengths(
        &scene,
        &all,
        true,
        &vec![vec![]; 12],
        &[],
        &[],
        0,
        &|_| unreachable!("a fixed panel has no probe right-hand sides"),
        true,
    );
    let fixed = fixed.unwrap().pop().unwrap();
    assert!(fixed.critical_kn.is_none());
    assert!(fixed.diagnostic.unwrap().contains("Нет свободных"));
}

// Boundary compliance includes the member. Hermite extension uses that elastic
// energy once, then adds only fixed-end interior bending modes.
fn critical(c: &Mat<f64>) -> Option<f64> {
    let eigen = c.self_adjoint_eigen(Side::Lower).ok()?;
    const N: usize = 12;
    let mut basis = Vec::new();
    for j in 0..4 {
        let v = eigen.S()[j];
        if v <= 1e-12 || !v.is_finite() {
            continue;
        }
        let q = |i| eigen.U()[(i, j)] * v.sqrt();
        basis.push(
            (0..=N)
                .flat_map(|i| {
                    let t = i as f64 / N as f64;
                    let w = (1. - 3. * t * t + 2. * t * t * t) * q(0)
                        + (t - 2. * t * t + t * t * t) * q(1)
                        + (3. * t * t - 2. * t * t * t) * q(2)
                        + (-t * t + t * t * t) * q(3);
                    let slope = (-6. * t + 6. * t * t) * q(0)
                        + (1. - 4. * t + 3. * t * t) * q(1)
                        + (6. * t - 6. * t * t) * q(2)
                        + (-2. * t + 3. * t * t) * q(3);
                    [w, slope]
                })
                .collect::<Vec<_>>(),
        );
    }
    let boundary = basis.len();
    for i in 2..2 * N {
        let mut v = vec![0.; 2 * N + 2];
        v[i] = 1.;
        basis.push(v);
    }
    let h = 1. / N as f64;
    let k = [
        [12., 6. * h, -12., 6. * h],
        [6. * h, 4. * h * h, -6. * h, 2. * h * h],
        [-12., -6. * h, 12., -6. * h],
        [6. * h, 2. * h * h, -6. * h, 4. * h * h],
    ];
    let g = [
        [36., 3. * h, -36., 3. * h],
        [3. * h, 4. * h * h, -3. * h, -h * h],
        [-36., -3. * h, 36., -3. * h],
        [3. * h, -h * h, -3. * h, 4. * h * h],
    ];
    let energy = |a: usize, b: usize, m: [[f64; 4]; 4]| {
        let mut sum = 0.;
        for i in 0..N {
            for j in 0..4 {
                for l in 0..4 {
                    sum += basis[a][2 * i + j] * m[j][l] * basis[b][2 * i + l];
                }
            }
        }
        sum
    };
    let size = basis.len();
    let k = Mat::from_fn(size, size, |a, b| {
        if a < boundary || b < boundary {
            if a == b {
                1.
            } else {
                0.
            }
        } else {
            energy(a, b, k) / h.powi(3)
        }
    });
    let g = Mat::from_fn(size, size, |a, b| energy(a, b, g) / (30. * h));
    let factor = k.llt(Side::Lower).ok()?;
    let l = factor.L();
    let mut reduced = g;
    l.solve_lower_triangular_in_place(reduced.as_mut());
    l.solve_lower_triangular_in_place(reduced.as_mut().transpose_mut());
    let values = reduced.self_adjoint_eigenvalues(Side::Lower).ok()?;
    values
        .iter()
        .last()
        .filter(|&&v| v > 0. && v.is_finite())
        .map(|v| 1. / v)
}

#[test]
fn critical_load_matches_euler_for_fixed_and_hinged_ends() {
    let fixed = Mat::zeros(4, 4);
    let mut hinged = Mat::zeros(4, 4);
    hinged[(1, 1)] = 1. / 3.;
    hinged[(3, 3)] = 1. / 3.;
    hinged[(1, 3)] = -1. / 6.;
    hinged[(3, 1)] = -1. / 6.;
    let euler = std::f64::consts::PI.powi(2);
    for (restraints, compliance, expected) in
        [("fixed", fixed, 4. * euler), ("hinged", hinged, euler)]
    {
        let load = critical(&compliance).unwrap();
        assert!(
            (load / expected - 1.).abs() < 1e-3,
            "{restraints}: {load} vs {expected}"
        );
    }
}
