//! Linearized buckling K φ = λ G φ with symmetric, possibly indefinite G.
use super::{progress, Mat, Side};

const DENSE_LIMIT: usize = 128;
const MAX_MODES: usize = 30;
const MAX_BASIS_ENTRIES: usize = 8_000_000;
const RESIDUAL_TOLERANCE: f64 = 1e-6;
const CUTOFF_TOLERANCE: f64 = 1e-9;

pub(super) fn spectrum(
    free: usize,
    requested: usize,
    max_factor: f64,
    elastic: impl Fn(&Mat<f64>) -> Mat<f64>,
    geometric: impl Fn(&Mat<f64>) -> Mat<f64>,
    solve: impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<(f64, Vec<f64>)>, String> {
    let cutoff = max_factor.recip();
    if !(1..=MAX_MODES).contains(&requested)
        || !max_factor.is_finite()
        || !cutoff.is_finite()
        || cutoff <= 0.
    {
        return Err(
            "Число форм устойчивости должно быть от 1 до 30, предел КЗУ — положительным числом"
                .into(),
        );
    }
    if free == 0 {
        return Ok(Vec::new());
    }
    progress::check_cancel()?;
    if free <= DENSE_LIMIT {
        dense_spectrum(free, requested, cutoff, &elastic, &geometric, &solve)
    } else {
        iterative_spectrum(free, requested, cutoff, &elastic, &geometric, &solve)
    }
}

fn within_limit(value: f64, cutoff: f64) -> bool {
    value.is_finite() && value > 0. && value / cutoff >= 1. - CUTOFF_TOLERANCE
}

fn norm_in_metric(x: &Mat<f64>, kx: &Mat<f64>, column: usize) -> f64 {
    (0..x.nrows())
        .map(|i| x[(i, column)] * kx[(i, column)])
        .sum::<f64>()
        .sqrt()
}

fn modes_to_output(values: &[f64], vectors: &Mat<f64>) -> Vec<(f64, Vec<f64>)> {
    values
        .iter()
        .enumerate()
        .map(|(j, &value)| {
            let mut shape: Vec<_> = (0..vectors.nrows()).map(|i| vectors[(i, j)]).collect();
            // Fix the otherwise arbitrary sign for reproducible display.
            if shape
                .iter()
                .max_by(|a, b| a.abs().total_cmp(&b.abs()))
                .is_some_and(|&v| v < 0.)
            {
                shape.iter_mut().for_each(|v| *v = -*v);
            }
            (value.recip(), shape)
        })
        .collect()
}

fn dense_spectrum(
    free: usize,
    requested: usize,
    cutoff: f64,
    elastic: &impl Fn(&Mat<f64>) -> Mat<f64>,
    geometric: &impl Fn(&Mat<f64>) -> Mat<f64>,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<(f64, Vec<f64>)>, String> {
    let identity = Mat::identity(free, free);
    // Exact small reference problem. The global sparse factor remains the one
    // used for verification and for all large-model iterations.
    let factor = elastic(&identity)
        .llt(Side::Lower)
        .map_err(|_| "Не удалось разложить упругую матрицу малой задачи устойчивости")?;
    let lower = factor.L();
    let mut transformed = geometric(&identity);
    lower.solve_lower_triangular_in_place(transformed.as_mut());
    lower.solve_lower_triangular_in_place(transformed.as_mut().transpose_mut());
    let eigen = transformed
        .self_adjoint_eigen(Side::Lower)
        .map_err(|_| "Не удалось решить малую спектральную задачу устойчивости")?;
    let indices: Vec<_> = (0..free)
        .rev()
        .take_while(|&i| within_limit(eigen.S()[i], cutoff))
        .take(requested)
        .collect();
    if indices.is_empty() {
        return Ok(Vec::new());
    }
    let values: Vec<_> = indices.iter().map(|&i| eigen.S()[i]).collect();
    let mut modes = Mat::from_fn(free, indices.len(), |i, j| eigen.U()[(i, indices[j])]);
    lower
        .transpose()
        .solve_upper_triangular_in_place(modes.as_mut());
    progress::check_cancel()?;
    let image = solve(&geometric(&modes));
    let residual = Mat::from_fn(free, modes.ncols(), |i, j| {
        image[(i, j)] - values[j] * modes[(i, j)]
    });
    let metric_residual = elastic(&residual);
    if values.iter().enumerate().any(|(j, value)| {
        !(norm_in_metric(&residual, &metric_residual, j) <= RESIDUAL_TOLERANCE * value)
    }) {
        return Err("Невязка форм устойчивости превышает допуск".into());
    }
    Ok(modes_to_output(&values, &modes))
}

struct Basis {
    vectors: Mat<f64>,
    metric: Mat<f64>,
    image: Mat<f64>,
    count: usize,
}

impl Basis {
    fn new(free: usize, capacity: usize) -> Self {
        Self {
            vectors: Mat::zeros(free, capacity),
            metric: Mat::zeros(free, capacity),
            image: Mat::zeros(free, capacity),
            count: 0,
        }
    }

    fn append(
        &mut self,
        mut block: Mat<f64>,
        elastic: &impl Fn(&Mat<f64>) -> Mat<f64>,
        apply: &impl Fn(&Mat<f64>) -> Mat<f64>,
    ) -> Result<(), String> {
        progress::check_cancel()?;
        let mut metric = elastic(&block);
        let old = self.count;
        for col in 0..block.ncols() {
            if self.count == self.vectors.ncols() {
                break;
            }
            let norm = norm_in_metric(&block, &metric, col);
            for _ in 0..2 {
                for j in 0..self.count {
                    let projection: f64 = (0..block.nrows())
                        .map(|i| self.vectors[(i, j)] * metric[(i, col)])
                        .sum();
                    for i in 0..block.nrows() {
                        block[(i, col)] -= projection * self.vectors[(i, j)];
                        metric[(i, col)] -= projection * self.metric[(i, j)];
                    }
                }
            }
            let norm_after = norm_in_metric(&block, &metric, col);
            if !norm_after.is_finite() || norm_after <= 1e-10 * norm || norm_after == 0. {
                continue;
            }
            for i in 0..block.nrows() {
                self.vectors[(i, self.count)] = block[(i, col)] / norm_after;
                self.metric[(i, self.count)] = metric[(i, col)] / norm_after;
            }
            self.count += 1;
        }
        if self.count > old {
            let vectors = self
                .vectors
                .as_ref()
                .subcols(old, self.count - old)
                .to_owned();
            let image = apply(&vectors);
            if (0..image.ncols()).any(|j| (0..image.nrows()).any(|i| !image[(i, j)].is_finite())) {
                return Err("Нечисловой отклик оператора устойчивости".into());
            }
            self.image
                .as_mut()
                .subcols_mut(old, image.ncols())
                .copy_from(&image);
        }
        progress::check_cancel()
    }

    fn projected(&self) -> Mat<f64> {
        let metric = self.metric.as_ref().subcols(0, self.count);
        let image = self.image.as_ref().subcols(0, self.count);
        let product = metric.transpose() * image;
        Mat::from_fn(self.count, self.count, |i, j| {
            (product[(i, j)] + product[(j, i)]) * 0.5
        })
    }

    fn restart(&mut self, coefficients: &Mat<f64>, count: usize) {
        let kept = coefficients.as_ref().subcols(0, count);
        let vectors = self.vectors.as_ref().subcols(0, self.count) * kept;
        let metric = self.metric.as_ref().subcols(0, self.count) * kept;
        let image = self.image.as_ref().subcols(0, self.count) * kept;
        self.vectors
            .as_mut()
            .subcols_mut(0, count)
            .copy_from(&vectors);
        self.metric
            .as_mut()
            .subcols_mut(0, count)
            .copy_from(&metric);
        self.image.as_mut().subcols_mut(0, count).copy_from(&image);
        self.count = count;
    }
}

fn random_block(rows: usize, cols: usize, seed: &mut u64) -> Mat<f64> {
    Mat::from_fn(rows, cols, |_, _| {
        *seed ^= *seed << 13;
        *seed ^= *seed >> 7;
        *seed ^= *seed << 17;
        (*seed >> 11) as f64 / (1_u64 << 53) as f64 - 0.5
    })
}

fn iterative_spectrum(
    free: usize,
    requested: usize,
    cutoff: f64,
    elastic: &impl Fn(&Mat<f64>) -> Mat<f64>,
    geometric: &impl Fn(&Mat<f64>) -> Mat<f64>,
    solve: &impl Fn(&Mat<f64>) -> Mat<f64>,
) -> Result<Vec<(f64, Vec<f64>)>, String> {
    let capacity = (MAX_BASIS_ENTRIES / free).min(4 * requested + 48).min(free);
    if capacity < requested + 4 {
        return Err("Для этой модели уменьшите число запрашиваемых форм устойчивости".into());
    }
    // Oversample the requested block when memory allows. Closely spaced
    // positive roots must remain together across restarts even when the
    // negative spectrum is much wider.
    let probes = (requested + 12)
        .min((capacity / 2).max(requested + 3))
        .min(capacity - 1);
    let apply = |q: &Mat<f64>| solve(&geometric(q));
    let mut seed = 0xd1b5_4a32_d192_ed03_u64;
    let mut basis = Basis::new(free, capacity);
    basis.append(random_block(free, probes, &mut seed), elastic, &apply)?;
    for iteration in 0..90 {
        if basis.count == 0 {
            break;
        }
        progress::check_cancel()?;
        if iteration % 10 == 0 {
            progress::stage(&format!(
                "Общая устойчивость · поиск форм, шаг {}",
                iteration + 1
            ));
        }
        let eigen = basis
            .projected()
            .self_adjoint_eigen(Side::Lower)
            .map_err(|_| "Не удалось уточнить спектр устойчивости")?;
        let candidates = probes.min(basis.count);
        let values: Vec<_> = (0..candidates)
            .map(|j| eigen.S()[basis.count - 1 - j])
            .collect();
        let coefficients = Mat::from_fn(basis.count, candidates, |i, j| {
            eigen.U()[(i, basis.count - 1 - j)]
        });
        let modes = basis.vectors.as_ref().subcols(0, basis.count) * &coefficients;
        let acted = basis.image.as_ref().subcols(0, basis.count) * &coefficients;
        let residual = Mat::from_fn(free, candidates, |i, j| {
            acted[(i, j)] - values[j] * modes[(i, j)]
        });
        let metric_residual = elastic(&residual);
        let residuals: Vec<_> = (0..candidates)
            .map(|j| norm_in_metric(&residual, &metric_residual, j))
            .collect();
        let accepted = values
            .iter()
            .take_while(|&&v| within_limit(v, cutoff))
            .count();
        let requested_prefix = requested.min(accepted);
        let converged = |j: usize| {
            let value = values[j];
            let tolerance = if j < accepted {
                RESIDUAL_TOLERANCE * value
            } else {
                (RESIDUAL_TOLERANCE * value.abs().max(cutoff)).min(0.25 * (cutoff - value))
            };
            residuals[j] <= tolerance
        };
        let prefix_converged = (0..requested_prefix).all(converged);
        // Guard Ritz pairs below the cutoff need their residuals inside the
        // gap. This is extremal convergence, not a matrix inertia count.
        let guard_converged =
            accepted < candidates && (accepted..candidates).take(2).all(converged);
        let explored = iteration >= 2 || basis.count == free;
        if prefix_converged && (requested_prefix == requested || (explored && guard_converged)) {
            return Ok(modes_to_output(&values[..requested_prefix], &modes));
        }
        // G=0 yields no geometric work in any of the starting probes.
        if accepted == 0
            && iteration == 0
            && values.iter().all(|&v| v == 0.)
            && (0..acted.ncols()).all(|j| (0..free).all(|i| acted[(i, j)] == 0.))
        {
            return Ok(Vec::new());
        }
        if basis.count + candidates > capacity {
            basis.restart(&coefficients, candidates.min(capacity - 1));
        }
        let old = basis.count;
        // Prioritize residuals for the requested modes and the cutoff guard.
        // Residual directions of already converged modes add no useful rank.
        let directions: Vec<_> = (0..candidates)
            .filter(|&j| !converged(j))
            .take(capacity - basis.count)
            .collect();
        let block = Mat::from_fn(free, directions.len(), |i, j| residual[(i, directions[j])]);
        if block.ncols() > 0 {
            basis.append(block, elastic, &apply)?;
        }
        if basis.count == old {
            let additional = (capacity - basis.count).min(probes);
            basis.append(random_block(free, additional, &mut seed), elastic, &apply)?;
        }
    }
    Err("Не удалось подтвердить сходимость требуемых форм до заданного предела КЗУ".into())
}

#[cfg(test)]
mod tests {
    use super::super::Solve;
    use super::*;

    #[test]
    fn buckling_dense_handles_signed_roots_and_inclusive_cutoff() {
        let k = Mat::from_fn(4, 4, |i, j| {
            if i == j {
                2.
            } else if i + j == 1 {
                1.
            } else {
                0.
            }
        });
        let g = Mat::from_fn(4, 4, |i, j| if i == j { [1., 1., -40., 0.][i] } else { 0. });
        let factor = k.llt(Side::Lower).unwrap();
        let get = |count, limit| {
            spectrum(
                4,
                count,
                limit,
                |q| &k * q,
                |q| &g * q,
                |q| factor.solve(q.as_ref()),
            )
            .unwrap()
        };
        let modes = get(4, 3.);
        assert_eq!(modes.len(), 2);
        assert!((modes[0].0 - 1.).abs() < 1e-10);
        assert!((modes[1].0 - 3.).abs() < 1e-10);
        assert_eq!(get(1, 3.).len(), 1);
        assert!(get(4, 0.9).is_empty());
    }

    #[test]
    fn buckling_iterative_resolves_repeated_roots_with_dominant_negative_values() {
        let n = DENSE_LIMIT + 12;
        let elastic = |q: &Mat<f64>| {
            Mat::from_fn(n, q.ncols(), |i, j| (1. + i as f64 / n as f64) * q[(i, j)])
        };
        let geometric = |q: &Mat<f64>| {
            let image = elastic(q);
            Mat::from_fn(n, q.ncols(), |i, j| {
                let value = [0.5, 0.2, 0.2, 0.05].get(i).copied().unwrap_or(-8.);
                value * image[(i, j)]
            })
        };
        let solve = |q: &Mat<f64>| {
            Mat::from_fn(n, q.ncols(), |i, j| q[(i, j)] / (1. + i as f64 / n as f64))
        };
        let modes = spectrum(n, 5, 5.1, elastic, geometric, solve).unwrap();
        assert_eq!(modes.len(), 3);
        for ((factor, _), expected) in modes.iter().zip([2., 5., 5.]) {
            assert!((factor - expected).abs() < 1e-7);
        }
        let dot: f64 = (0..n)
            .map(|i| modes[1].1[i] * modes[2].1[i] * (1. + i as f64 / n as f64))
            .sum();
        assert!(dot.abs() < 1e-8);
        assert!(spectrum(n, 3, 1.5, elastic, geometric, solve)
            .unwrap()
            .is_empty());
    }
}
