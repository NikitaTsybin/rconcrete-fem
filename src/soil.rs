//! Prototype: Boussinesq stresses + layered strain summation. Units: m, tf.
use serde::{Deserialize, Serialize};
use std::sync::atomic::{AtomicBool, Ordering};
#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct Layer {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub material_id: Option<u32>,
    pub bottom_z: f64,
    pub e: f64,
    pub gamma: f64,
}
#[derive(Clone, PartialEq, Serialize, Deserialize)]
pub struct Borehole {
    pub id: u32,
    pub xy: [f64; 2],
    pub mouth_z: f64,
    pub layers: Vec<Layer>,
}
#[derive(Clone, Serialize, Deserialize)]
pub struct Cell {
    pub bounds: [f64; 4],
    #[serde(default = "full_coverage")]
    pub coverage: f64,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub polygon: Vec<[f64; 2]>,
}
fn full_coverage() -> f64 {
    1.
}
impl Cell {
    pub fn from_polygon(mut points: Vec<[f64; 2]>) -> Option<Self> {
        if !matches!(points.len(), 3 | 4) || !points.iter().flatten().all(|v| v.is_finite()) {
            return None;
        }
        if polygon_area(&points) < 0. {
            points[1..].reverse();
        }
        if !valid_polygon(&points) {
            return None;
        }
        let bounds = points.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |[x0, y0, x1, y1], &[x, y]| [x0.min(x), y0.min(y), x1.max(x), y1.max(y)],
        );
        Some(Self {
            bounds,
            coverage: 1.,
            polygon: points,
        })
    }
    pub fn center(&self) -> [f64; 2] {
        if !self.polygon.is_empty() {
            return std::array::from_fn(|a| {
                self.polygon.iter().map(|p| p[a]).sum::<f64>() / self.polygon.len() as f64
            });
        }
        [
            (self.bounds[0] + self.bounds[2]) / 2.,
            (self.bounds[1] + self.bounds[3]) / 2.,
        ]
    }
    pub fn area(&self) -> f64 {
        let area = if self.polygon.is_empty() {
            (self.bounds[2] - self.bounds[0]) * (self.bounds[3] - self.bounds[1])
        } else {
            polygon_area(&self.polygon)
        };
        area * self.coverage
    }
    fn influence(&self, receiver: [f64; 2], z: f64) -> f64 {
        if self.polygon.is_empty() {
            rectangle_influence(receiver, z, self.bounds)
        } else {
            polygon_influence(receiver, z, &self.polygon)
        }
    }
}

fn delta(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn cross(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn polygon_area(points: &[[f64; 2]]) -> f64 {
    let Some((&origin, rest)) = points.split_first() else {
        return 0.;
    };
    rest.windows(2)
        .map(|p| cross(delta(p[0], origin), delta(p[1], origin)))
        .sum::<f64>()
        * 0.5
}

fn valid_polygon(points: &[[f64; 2]]) -> bool {
    let n = points.len();
    if !matches!(n, 3 | 4) || !points.iter().flatten().all(|v| v.is_finite()) {
        return false;
    }
    let area = polygon_area(points);
    area.is_finite()
        && area > 0.
        && (0..n).all(|i| {
            let e1 = delta(points[(i + 1) % n], points[i]);
            let e2 = delta(points[(i + 2) % n], points[(i + 1) % n]);
            let turn = cross(e1, e2);
            turn.is_finite() && turn > 0.
        })
}
#[derive(Clone, Serialize, Deserialize)]
pub struct Options {
    pub base_z: f64,
    pub max_depth: f64,
    #[serde(default)]
    pub min_depth: f64,
    pub dz: f64,
    pub beta: f64,
    pub stress_ratio: f64,
    pub relaxation: f64,
    pub tolerance: f64,
    pub max_iterations: usize,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            base_z: 144.5,
            max_depth: 30.,
            min_depth: 0.,
            dz: 0.25,
            beta: 0.8,
            stress_ratio: 0.2,
            relaxation: 0.5,
            tolerance: 0.005,
            max_iterations: 60,
        }
    }
}
/// Vertical stress / uniform rectangle pressure; signed corner primitive.
pub fn rectangle_influence(receiver: [f64; 2], z: f64, b: [f64; 4]) -> f64 {
    let corner = |x: f64, y: f64| {
        let r = (x * x + y * y + z * z).sqrt();
        ((x * y).atan2(z * r) + x * y * z / r * (1. / (x * x + z * z) + 1. / (y * y + z * z)))
            / (2. * std::f64::consts::PI)
    };
    let [x0, y0, x1, y1] = b;
    let [x, y] = receiver;
    (corner(x1 - x, y1 - y) - corner(x0 - x, y1 - y) - corner(x1 - x, y0 - y)
        + corner(x0 - x, y0 - y))
    .clamp(0., 1.)
}

/// Integral of the Boussinesq vertical stress kernel over a CCW T3 or Q4.
/// If Ω is the solid angle, this integral is (Ω − z·dΩ/dz) / (2π).
fn polygon_influence(receiver: [f64; 2], z: f64, points: &[[f64; 2]]) -> f64 {
    let z2 = z * z;
    let mut r = [[0.; 2]; 4];
    let mut len = [0.; 4];
    for (i, &p) in points.iter().enumerate() {
        r[i] = delta(p, receiver);
        len[i] = (dot(r[i], r[i]) + z2).sqrt();
    }
    let angle = (1..points.len() - 1)
        .map(|i| {
            let j = i + 1;
            let det = z * cross(delta(points[i], points[0]), delta(points[j], points[0]));
            let denominator = len[0] * len[i] * len[j]
                + (dot(r[0], r[i]) + z2) * len[j]
                + (dot(r[i], r[j]) + z2) * len[0]
                + (dot(r[j], r[0]) + z2) * len[i];
            2. * det.atan2(denominator)
        })
        .sum::<f64>();
    let derivative = (0..points.len())
        .map(|i| {
            let j = (i + 1) % points.len();
            let e = delta(points[j], points[i]);
            let length = e[0].hypot(e[1]);
            let tangent = e.map(|x| x / length);
            let d = cross(r[i], tangent);
            let t0 = dot(r[i], tangent);
            let t1 = t0 + length;
            let a2 = d * d + z2;
            // The antiderivative is t/(a²·R). Rationalise differences on
            // the same side of the perpendicular foot to avoid cancellation.
            let integral = if t0 * t1 > 0. {
                length * (t1 + t0) / (len[i] * len[j] * (t1 * len[i] + t0 * len[j]))
            } else {
                (t1 / len[j] - t0 / len[i]) / a2
            };
            d * integral
        })
        .sum::<f64>();
    ((angle + z * derivative) / (2. * std::f64::consts::PI)).clamp(0., 1.)
}
pub struct Engine {
    pub cells: Vec<Cell>,
    pub holes: Vec<Borehole>,
    pub options: Options,
}
#[derive(Serialize, Deserialize)]
pub struct Settlement {
    pub metres: Vec<f64>,
    pub depth: Vec<f64>,
}
type InfluenceRow = (f64, f64, Vec<f32>);
struct InfluenceCache {
    rows: Vec<Vec<InfluenceRow>>,
    row_limit: usize,
}
impl InfluenceCache {
    fn new(cells: usize) -> Self {
        let bytes_per_row = cells.saturating_mul(std::mem::size_of::<f32>()).max(1);
        Self {
            rows: (0..cells).map(|_| Vec::new()).collect(),
            row_limit: (64_usize * 1024 * 1024).div_ceil(bytes_per_row),
        }
    }
}
fn settlement_workers() -> usize {
    #[cfg(not(target_arch = "wasm32"))]
    {
        std::thread::available_parallelism().map_or(1, |n| n.get().min(8))
    }
    #[cfg(target_arch = "wasm32")]
    {
        1
    }
}

fn check_cancel(cancel: Option<&AtomicBool>) -> Result<(), String> {
    if cancel.is_some_and(|flag| flag.load(Ordering::Relaxed)) {
        Err("Расчёт остановлен пользователем".into())
    } else {
        Ok(())
    }
}

impl Engine {
    pub fn validate(&self) -> Result<(), String> {
        let o = &self.options;
        if self.cells.is_empty() || self.holes.is_empty() {
            return Err("No cells or boreholes".into());
        }
        if !(o.dz > 0.
            && o.max_depth > 0.
            && (0. ..=o.max_depth).contains(&o.min_depth)
            && o.beta > 0.
            && o.relaxation > 0.
            && o.relaxation <= 1.
            && o.tolerance > 0.)
            || [
                o.base_z,
                o.dz,
                o.max_depth,
                o.beta,
                o.stress_ratio,
                o.relaxation,
                o.tolerance,
            ]
            .iter()
            .any(|v| !v.is_finite())
            || o.max_iterations == 0
            || o.max_iterations > 1000
            || o.max_depth / o.dz > 100_000.
        {
            return Err("Invalid options".into());
        }
        for c in &self.cells {
            if c.bounds.iter().any(|v| !v.is_finite())
                || !c.coverage.is_finite()
                || c.coverage <= 0.
                || c.bounds[2] <= c.bounds[0]
                || c.bounds[3] <= c.bounds[1]
                || (!c.polygon.is_empty() && !valid_polygon(&c.polygon))
                || !c.area().is_finite()
            {
                return Err("Invalid cell".into());
            }
        }
        for b in &self.holes {
            if b.xy.iter().any(|v| !v.is_finite()) || !b.mouth_z.is_finite() || b.layers.is_empty()
            {
                return Err(format!("Invalid borehole {}", b.id));
            }
            let mut top = b.mouth_z;
            for l in &b.layers {
                if !l.e.is_finite()
                    || l.e <= 0.
                    || l.bottom_z >= top
                    || !l.bottom_z.is_finite()
                    || !l.gamma.is_finite()
                    || l.gamma <= 0.
                {
                    return Err(format!("Invalid borehole {}", b.id));
                }
                top = l.bottom_z;
            }
        }
        Ok(())
    }
    /// Nearest three profiles, IDW interpolation of compliance at absolute elevation.
    fn profile(&self, p: [f64; 2], z: f64) -> Result<(f64, f64), String> {
        let mut hs = self
            .holes
            .iter()
            .map(|h| (((h.xy[0] - p[0]).powi(2) + (h.xy[1] - p[1]).powi(2)), h))
            .collect::<Vec<_>>();
        hs.sort_by(|a, b| a.0.total_cmp(&b.0));
        let mut compliance = 0.;
        let mut gamma = 0.;
        let mut weights = 0.;
        for (d, h) in hs.into_iter().take(3) {
            let l = h
                .layers
                .iter()
                .find(|l| z >= l.bottom_z && z <= h.mouth_z)
                .ok_or_else(|| format!("Borehole {} does not cover Z={z}", h.id))?;
            let w = 1. / d.max(0.01);
            compliance += w / l.e;
            gamma += w * l.gamma;
            weights += w;
        }
        Ok((compliance / weights, gamma / weights))
    }
    pub fn settlements(&self, q: &[f64]) -> Result<Settlement, String> {
        self.evaluate(
            q,
            &mut InfluenceCache::new(self.cells.len()),
            false,
            settlement_workers(),
            None,
        )
    }
    fn evaluate(
        &self,
        q: &[f64],
        cache: &mut InfluenceCache,
        store: bool,
        workers: usize,
        cancel: Option<&AtomicBool>,
    ) -> Result<Settlement, String> {
        check_cancel(cancel)?;
        self.validate()?;
        if q.len() != self.cells.len() || q.iter().any(|v| !v.is_finite() || *v < 0.) {
            return Err("Pressure must be finite compression, tf/m2".into());
        }
        let mut free = if store {
            cache
                .row_limit
                .saturating_sub(cache.rows.iter().map(Vec::len).sum())
        } else {
            0
        };
        let max_layers = (self.options.max_depth / self.options.dz).ceil() as usize + 1;
        let mut output = vec![(0., 0.); self.cells.len()];
        let mut start = 0;
        while start < self.cells.len() {
            let mut end = self.cells.len();
            if free > 0 {
                let mut required = 0;
                end = start;
                // Cache rows in the original receiver order. Parallel batches fit
                // even if every receiver reaches the maximum depth.
                for rows in &cache.rows[start..] {
                    required += max_layers.saturating_sub(rows.len());
                    if required > free {
                        break;
                    }
                    end += 1;
                }
                if end == start {
                    let rows = &mut cache.rows[start];
                    let before = rows.len();
                    output[start] =
                        self.evaluate_cell(&self.cells[start], q, rows, free, cancel)?;
                    free -= rows.len() - before;
                    start += 1;
                    continue;
                }
            }
            let rows = &mut cache.rows[start..end];
            let store_limit = if free > 0 { max_layers } else { 0 };
            free -= self.evaluate_batch(
                &self.cells[start..end],
                q,
                rows,
                &mut output[start..end],
                store_limit,
                workers,
                cancel,
            )?;
            start = end;
        }
        check_cancel(cancel)?;
        let (metres, depth) = output.into_iter().unzip();
        Ok(Settlement { metres, depth })
    }

    fn evaluate_batch(
        &self,
        cells: &[Cell],
        q: &[f64],
        cache: &mut [Vec<InfluenceRow>],
        output: &mut [(f64, f64)],
        store_limit: usize,
        workers: usize,
        cancel: Option<&AtomicBool>,
    ) -> Result<usize, String> {
        let run = |cells: &[Cell], cache: &mut [Vec<InfluenceRow>], output: &mut [(f64, f64)]| {
            let mut added = 0;
            for ((cell, rows), out) in cells.iter().zip(cache).zip(output) {
                let before = rows.len();
                *out = self.evaluate_cell(cell, q, rows, store_limit, cancel)?;
                added += rows.len() - before;
            }
            Ok(added)
        };
        #[cfg(not(target_arch = "wasm32"))]
        {
            let chunk = cells.len().div_ceil(workers).max(8);
            if chunk < cells.len() {
                return std::thread::scope(|scope| {
                    let jobs: Vec<_> = cells
                        .chunks(chunk)
                        .zip(cache.chunks_mut(chunk))
                        .zip(output.chunks_mut(chunk))
                        .map(|((cells, cache), output)| {
                            scope.spawn(move || run(cells, cache, output))
                        })
                        .collect();
                    jobs.into_iter()
                        .map(|job| job.join().expect("soil worker panicked"))
                        .sum()
                });
            }
        }
        #[cfg(target_arch = "wasm32")]
        let _ = workers;
        run(cells, cache, output)
    }

    fn evaluate_cell(
        &self,
        receiver: &Cell,
        q: &[f64],
        rows: &mut Vec<InfluenceRow>,
        mut free: usize,
        cancel: Option<&AtomicBool>,
    ) -> Result<(f64, f64), String> {
        let o = &self.options;
        let xy = receiver.center();
        let mut s = 0.;
        let mut z = 0.;
        let mut overburden = 0.;
        let mut previous_margin: f64 = f64::INFINITY;
        let mut layer_index = 0;
        // Excavation compensation is not inferred: cutoff uses overburden below base.
        while z < o.max_depth - 1e-9 {
            check_cancel(cancel)?;
            let h = o.dz.min(o.max_depth - z);
            let mid = z + h / 2.;
            if free > 0 && layer_index == rows.len() {
                let (c, g) = self.profile(xy, o.base_z - mid)?;
                let row = self
                    .cells
                    .iter()
                    .map(|cell| (cell.coverage * cell.influence(xy, mid)) as f32)
                    .collect();
                rows.push((c, g, row));
                free -= 1;
            }
            let (c, g, sigma) = if let Some((c, g, row)) = rows.get(layer_index) {
                (
                    *c,
                    *g,
                    row.iter().zip(q).map(|(w, q)| *w as f64 * q).sum::<f64>(),
                )
            } else {
                let (c, g) = self.profile(xy, o.base_z - mid)?;
                (
                    c,
                    g,
                    self.cells
                        .iter()
                        .zip(q)
                        .map(|(cell, q)| q * cell.coverage * cell.influence(xy, mid))
                        .sum::<f64>(),
                )
            };
            layer_index += 1;
            overburden += g * h;
            let margin = sigma - o.stress_ratio * overburden;
            if z >= 2. * o.dz && z + h >= o.min_depth && o.stress_ratio > 0. && margin < 0. {
                let fraction = if previous_margin.is_finite() {
                    (previous_margin / (previous_margin - margin)).clamp(0., 1.)
                } else {
                    0.
                };
                let fraction = fraction.max(((o.min_depth - z) / h).clamp(0., 1.));
                s += o.beta * sigma * c * h * fraction;
                z += h * fraction;
                break;
            }
            previous_margin = margin;
            s += o.beta * sigma * c * h;
            z += h;
        }
        Ok((s, z))
    }
    /// Host callback solves the SAME reference combination using supplied C1 (tf/m3)
    /// and returns cell-average compression pressure (tf/m2). No result freezing on failure.
    pub fn iterate(
        &self,
        initial: Vec<f64>,
        solve: impl FnMut(&[f64]) -> Result<Vec<f64>, String>,
    ) -> Result<Iteration, String> {
        self.iterate_with_progress(initial, solve, |_, _| Ok(()))
    }
    pub fn iterate_with_progress(
        &self,
        initial: Vec<f64>,
        solve: impl FnMut(&[f64]) -> Result<Vec<f64>, String>,
        progress: impl FnMut(usize, f64) -> Result<(), String>,
    ) -> Result<Iteration, String> {
        self.iterate_with_progress_and_cancel(initial, solve, progress, None)
    }
    pub fn iterate_with_progress_and_cancel(
        &self,
        initial: Vec<f64>,
        mut solve: impl FnMut(&[f64]) -> Result<Vec<f64>, String>,
        mut progress: impl FnMut(usize, f64) -> Result<(), String>,
        cancel: Option<&AtomicBool>,
    ) -> Result<Iteration, String> {
        check_cancel(cancel)?;
        self.validate()?;
        if initial.len() != self.cells.len() || initial.iter().any(|c| !c.is_finite() || *c <= 0.) {
            return Err("Invalid initial C1".into());
        }
        let mut cache = InfluenceCache::new(self.cells.len());
        let mut c = initial;
        let mut history = vec![];
        let workers = settlement_workers();
        for iteration in 0..self.options.max_iterations {
            check_cancel(cancel)?;
            let q = solve(&c)?;
            let s = self.evaluate(&q, &mut cache, true, workers, cancel)?;
            let mut next = c.clone();
            let mut error: f64 = 0.;
            for i in 0..c.len() {
                if q[i] <= 1e-10 {
                    continue;
                }
                if s.metres[i] <= 1e-12 {
                    return Err("Positive pressure with zero settlement".into());
                }
                let target = q[i] / s.metres[i];
                next[i] = (1. - self.options.relaxation) * c[i] + self.options.relaxation * target;
                error = error.max((target - c[i]).abs() / target.max(c[i]));
            }
            history.push(error);
            progress(iteration + 1, error)?;
            check_cancel(cancel)?;
            if error <= self.options.tolerance {
                return Ok(Iteration {
                    converged: true,
                    c1: c,
                    pressure: q,
                    settlement: s,
                    history,
                    iterations: iteration + 1,
                });
            }
            c = next;
        }
        check_cancel(cancel)?;
        let q = solve(&c)?;
        let s = self.evaluate(&q, &mut cache, true, workers, cancel)?;
        Ok(Iteration {
            converged: false,
            c1: c,
            pressure: q,
            settlement: s,
            iterations: self.options.max_iterations,
            history,
        })
    }
}
#[derive(Serialize, Deserialize)]
pub struct Iteration {
    pub converged: bool,
    pub c1: Vec<f64>,
    pub pressure: Vec<f64>,
    pub settlement: Settlement,
    pub history: Vec<f64>,
    pub iterations: usize,
}
#[cfg(test)]
mod tests {
    use super::*;
    fn engine() -> Engine {
        Engine {
            cells: vec![Cell {
                bounds: [-1., -1., 1., 1.],
                coverage: 1.,
                polygon: vec![],
            }],
            holes: vec![Borehole {
                id: 1,
                xy: [0., 0.],
                mouth_z: 0.,
                layers: vec![Layer {
                    material_id: None,
                    bottom_z: -100.,
                    e: 2000.,
                    gamma: 2.,
                }],
            }],
            options: Options {
                base_z: 0.,
                max_depth: 50.,
                stress_ratio: 0.,
                ..Default::default()
            },
        }
    }
    #[test]
    fn rectangle_surface_and_far_field() {
        assert!((rectangle_influence([0., 0.], 1e-5, [-1., -1., 1., 1.]) - 1.).abs() < 1e-5);
        let z = 100.;
        let exact = 3. * 4. / (2. * std::f64::consts::PI * z * z);
        assert!((rectangle_influence([0., 0.], z, [-1., -1., 1., 1.]) / exact - 1.).abs() < 0.001);
    }
    #[test]
    fn polygon_influence_matches_rectangle_for_inside_edge_and_outside_receivers() {
        let cell = Cell::from_polygon(vec![[-2., -1.], [3., -1.], [3., 2.], [-2., 2.]]).unwrap();
        for p in [
            [0., 0.],
            [3., 0.],
            [3., 2.],
            [3.001, 2.],
            [4., -3.],
            [30., 70.],
        ] {
            for z in [1e-4, 0.125, 1., 10., 100.] {
                let expected = rectangle_influence(p, z, cell.bounds);
                let actual = cell.influence(p, z);
                assert!(
                    (actual - expected).abs() < 1e-10,
                    "{p:?}, z={z}: {actual} != {expected}"
                );
            }
        }
    }
    #[test]
    fn convex_quad_is_sum_of_its_two_uniform_triangles() {
        let [a, b, c, d] = [[-1.5, -1.], [3., 0.], [1.5, 2.], [-2., 0.5]];
        let quad = Cell::from_polygon(vec![a, b, c, d]).unwrap();
        let first = Cell::from_polygon(vec![a, b, c]).unwrap();
        let second = Cell::from_polygon(vec![a, c, d]).unwrap();
        assert!((quad.area() - first.area() - second.area()).abs() < 1e-12);
        for p in [[0., 0.], [1., 1.], [-2., 0.5], [-10., 20.]] {
            for z in [0.01, 0.5, 3., 30.] {
                let split = first.influence(p, z) + second.influence(p, z);
                assert!((quad.influence(p, z) - split).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn polygon_cells_normalise_winding_and_legacy_rectangles_keep_their_shape() {
        let cell = Cell::from_polygon(vec![[0., 0.], [0., 3.], [6., 0.]]).unwrap();
        assert_eq!(cell.center(), [2., 1.]);
        assert_eq!(cell.area(), 9.);
        assert!(Cell::from_polygon(vec![[0., 0.], [1., 1.], [2., 2.]]).is_none());
        assert!(Cell::from_polygon(vec![[0., 0.], [2., 2.], [0., 2.], [2., 0.]]).is_none());
        assert!(Cell::from_polygon(vec![[0., 0.], [3., 0.], [1., 0.5], [0., 3.]]).is_none());
        let old: Cell = serde_json::from_str(r#"{"bounds":[0,0,2,3]}"#).unwrap();
        assert!(old.polygon.is_empty());
        assert_eq!(old.area(), 6.);
        assert!(serde_json::to_value(old).unwrap().get("polygon").is_none());
        let mut e = engine();
        e.cells[0] = cell;
        assert!(e.settlements(&[10.]).unwrap().metres[0] > 0.);
        e.cells[0].polygon.reverse();
        assert!(e.validate().is_err());
    }
    #[test]
    fn minimum_depth_limits_early_stress_cutoff_and_reads_old_options() {
        let mut e = engine();
        e.options.stress_ratio = 0.5;
        let natural = e.settlements(&[1.]).unwrap();
        e.options.min_depth = 3.13;
        let limited = e.settlements(&[1.]).unwrap();
        assert!(natural.depth[0] < 3.13);
        assert!((limited.depth[0] - 3.13).abs() < 1e-12);
        assert!(limited.metres[0] > natural.metres[0]);
        e.options.min_depth = e.options.max_depth + 0.1;
        assert!(e.validate().is_err());
        let mut old = serde_json::to_value(Options::default()).unwrap();
        old.as_object_mut().unwrap().remove("min_depth");
        assert_eq!(
            serde_json::from_value::<Options>(old).unwrap().min_depth,
            0.
        );
    }

    #[test]
    fn settlement_linear_and_stiffness_inverse() {
        let e = engine();
        let a = e.settlements(&[10.]).unwrap();
        let b = e.settlements(&[20.]).unwrap();
        assert!((b.metres[0] / a.metres[0] - 2.).abs() < 1e-10);
        assert!(a.metres[0] > 0.);
    }
    #[test]
    fn partial_cell_preserves_resultant_and_equivalent_stress() {
        let full = engine();
        let mut half = engine();
        half.cells[0].coverage = 0.5;
        let a = full.settlements(&[10.]).unwrap();
        let b = half.settlements(&[20.]).unwrap();
        assert!((full.cells[0].area() * 10. - half.cells[0].area() * 20.).abs() < 1e-12);
        assert!((a.metres[0] - b.metres[0]).abs() < 1e-12);
    }
    #[test]
    fn cached_influence_matches_streaming_for_different_pressures() {
        let mut e = engine();
        e.cells.push(Cell {
            bounds: [1., -1., 3., 1.],
            coverage: 0.3,
            polygon: vec![],
        });
        let mut cache = InfluenceCache::new(e.cells.len());
        for q in [[5., 9.], [12., 2.]] {
            let direct = e.settlements(&q).unwrap();
            let cached = e.evaluate(&q, &mut cache, true, 1, None).unwrap();
            for (a, b) in direct.metres.iter().zip(cached.metres) {
                assert!((a - b).abs() < 1e-6 * a.abs().max(1e-6));
            }
        }
        assert!(cache.rows.iter().any(|rows| !rows.is_empty()));
    }
    #[test]
    fn parallel_settlement_matches_serial_when_the_influence_cache_fills() {
        let mut e = engine();
        e.options.max_depth = 8.;
        e.options.stress_ratio = 0.5;
        e.cells = (0..64)
            .map(|i| {
                let x = (i % 8) as f64;
                let y = (i / 8) as f64;
                Cell {
                    bounds: [x, y, x + 1., y + 1.],
                    coverage: if i % 2 == 0 { 1. } else { 0.5 },
                    polygon: vec![],
                }
            })
            .collect();
        let mut serial = InfluenceCache::new(e.cells.len());
        serial.row_limit = 500;
        let mut parallel = InfluenceCache::new(e.cells.len());
        parallel.row_limit = serial.row_limit;
        for level in [1., 8., 20.] {
            let pressure: Vec<_> = (0..64).map(|i| level * (1 + i % 5) as f64).collect();
            let a = e.evaluate(&pressure, &mut serial, true, 1, None).unwrap();
            let b = e.evaluate(&pressure, &mut parallel, true, 4, None).unwrap();
            assert_eq!(a.metres, b.metres);
            assert_eq!(a.depth, b.depth);
            assert_eq!(serial.rows, parallel.rows);
        }
        assert_eq!(
            parallel.rows.iter().map(Vec::len).sum::<usize>(),
            parallel.row_limit
        );
    }

    #[test]
    fn iteration_matches_fixed_pressure_and_rejects_tension() {
        let e = engine();
        let r = e.iterate(vec![1000.], |_| Ok(vec![10.])).unwrap();
        assert!(r.converged);
        assert!((r.c1[0] * r.settlement.metres[0] / 10. - 1.).abs() < 0.005);
        assert!(e.settlements(&[-1.]).is_err());
    }

    #[test]
    fn cancelled_pressure_transfer_does_not_start_settlement_or_return_an_iteration() {
        let e = engine();
        let flag = AtomicBool::new(false);
        let result = e.iterate_with_progress_and_cancel(
            vec![1000.],
            |_| {
                flag.store(true, Ordering::Relaxed);
                Ok(vec![10.])
            },
            |_, _| panic!("cancelled settlement must not report progress"),
            Some(&flag),
        );
        assert_eq!(
            result.err().as_deref(),
            Some("Расчёт остановлен пользователем")
        );
        for store_limit in [0, 1] {
            let mut rows = vec![];
            let result = e.evaluate_cell(&e.cells[0], &[10.], &mut rows, store_limit, Some(&flag));
            assert!(result.is_err());
            assert!(rows.is_empty());
        }
    }
}
