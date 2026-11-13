//! Equilibrated element-edge nodal actions (vertex patch recovery).
//! The constitutive edge traction is the reference; the correction enforces the
//! actual discrete element vector. Opposite sides share one signed unknown.
use super::super::geometry::Point;
use std::collections::BTreeMap;
type Key = (i64, i64);
fn key(p: Point) -> Key {
    ((p[0] * 1e5).round() as i64, (p[1] * 1e5).round() as i64)
}
#[derive(Clone)]
pub(super) struct Cell {
    pub poly: Vec<Point>,
    pub actions: Vec<(Point, [f64; 3])>,
    pub field: [f64; 5],
    pub kirchhoff: bool,
    pub quadratic: bool,
}
pub(super) struct Recovery {
    pub edges: BTreeMap<(usize, usize), [[f64; 3]; 2]>,
    pub errors: BTreeMap<Key, String>,
    pub midpoints: BTreeMap<(usize, usize), [f64; 3]>,
}
impl Recovery {
    pub fn new(cells: &[Cell]) -> Self {
        Self::recover(cells, None, false)
    }
    pub fn nodal_sources(cells: &[Cell]) -> Self {
        Self::recover(cells, None, true)
    }
    pub fn faces(cells: &[Cell], reference: &[Vec<[[f64; 3]; 2]>]) -> Self {
        Self::recover(cells, Some(reference), true)
    }
    fn recover(cells: &[Cell], reference: Option<&[Vec<[[f64; 3]; 2]>]>, sources: bool) -> Self {
        let mut result = Self {
            edges: BTreeMap::new(),
            errors: BTreeMap::new(),
            midpoints: BTreeMap::new(),
        };
        let mut stars: BTreeMap<Key, Vec<(usize, usize)>> = BTreeMap::new();
        for (ci, c) in cells.iter().enumerate() {
            for (i, &p) in c.poly.iter().enumerate() {
                stars.entry(key(p)).or_default().push((ci, i));
            }
        }
        for (node, star) in stars {
            let n = star.len();
            let mut spokes: BTreeMap<Key, Vec<(usize, usize, usize, [f64; 3])>> = BTreeMap::new();
            let mut rhs = vec![[0.; 3]; n];
            for (row, &(ci, vi)) in star.iter().enumerate() {
                let c = &cells[ci];
                rhs[row] = c
                    .actions
                    .iter()
                    .find(|(p, _)| key(*p) == node)
                    .map_or([0.; 3], |(_, v)| *v);
                let area = (0..c.poly.len())
                    .map(|i| {
                        let a = c.poly[i];
                        let b = c.poly[(i + 1) % c.poly.len()];
                        a[0] * b[1] - a[1] * b[0]
                    })
                    .sum::<f64>();
                for (ei, end) in [(vi, 0), ((vi + c.poly.len() - 1) % c.poly.len(), 1)] {
                    let a = c.poly[ei];
                    let b = c.poly[(ei + 1) % c.poly.len()];
                    let dx = (b[0] - a[0]) / 1000.;
                    let dy = (b[1] - a[1]) / 1000.;
                    let length = dx.hypot(dy);
                    let nx = dy / length * area.signum();
                    let ny = -dx / length * area.signum();
                    let [mx, my, mxy, qx, qy] = c.field;
                    let mut prior = [
                        (qx * nx + qy * ny) * length * 0.5,
                        (mxy * nx + my * ny) * length * 0.5,
                        -(mx * nx + mxy * ny) * length * 0.5,
                    ];
                    if c.quadratic {
                        prior = [prior[0] / 3., 0., 0.];
                    }
                    if c.kirchhoff {
                        // DKT tangential slopes follow the derivative of the
                        // edge Hermite trace; twisting moment has corner forces.
                        let tx = dx / length;
                        let ty = dy / length;
                        let mn = -(nx * (mx * nx + mxy * ny) + ny * (mxy * nx + my * ny));
                        let mt = -(tx * (mx * nx + mxy * ny) + ty * (mxy * nx + my * ny));
                        let sign = if end == 0 { 1. } else { -1. };
                        let tangent = (qx * nx + qy * ny) * length * length / 12. * sign;
                        let bx = mn * length * 0.5 * nx + tangent * tx;
                        let by = mn * length * 0.5 * ny + tangent * ty;
                        prior = [prior[0] - sign * mt, -by, bx];
                    }
                    if let Some(value) = reference.and_then(|r| r.get(ci)).and_then(|r| r.get(ei)) {
                        prior = value[end];
                    }
                    let other = if end == 0 { b } else { a };
                    spokes
                        .entry(key(other))
                        .or_default()
                        .push((row, ei, end, prior));
                }
            }
            let mut columns = vec![];
            let mut references = vec![];
            let mut entries = vec![];
            let mut invalid = false;
            for (_, sides) in spokes {
                if sides.len() > 2 {
                    invalid = true;
                    break;
                }
                if sides.len() == 1 {
                    let (row, ei, end, _) = sides[0];
                    let ci = star[row].0;
                    result.edges.entry((ci, ei)).or_insert([[0.; 3]; 2])[end] = [0.; 3];
                    continue;
                }
                let (a, _, _, pa) = sides[0];
                let (b, _, _, pb) = sides[1];
                let mut col = vec![0.; n];
                col[a] = 1.;
                col[b] = -1.;
                columns.push(col);
                references.push(std::array::from_fn::<_, 3, _>(|d| (pa[d] - pb[d]) * 0.5));
                entries.push(sides);
            }
            let total: [f64; 3] = std::array::from_fn(|d| rhs.iter().map(|v| v[d]).sum());
            if sources {
                // Coupling/constraint action is a point measure. Assign it to
                // incident cell sectors, preserving its total and first moments.
                let angles: Vec<_> = star
                    .iter()
                    .map(|&(ci, vi)| {
                        let p = &cells[ci].poly;
                        let a = p[(vi + p.len() - 1) % p.len()];
                        let o = p[vi];
                        let b = p[(vi + 1) % p.len()];
                        let u = [a[0] - o[0], a[1] - o[1]];
                        let v = [b[0] - o[0], b[1] - o[1]];
                        (u[0] * v[1] - u[1] * v[0])
                            .abs()
                            .atan2(u[0] * v[0] + u[1] * v[1])
                    })
                    .collect();
                let angle_sum = angles.iter().sum::<f64>();
                if angle_sum > 1e-12 {
                    for (row, angle) in rhs.iter_mut().zip(angles) {
                        for d in 0..3 {
                            row[d] -= total[d] * angle / angle_sum;
                        }
                    }
                }
            }
            let scale = rhs.iter().flatten().fold(1.0_f64, |s, v| s.max(v.abs()));
            if invalid || (!sources && total.iter().any(|v| v.abs() > 1e-6 * scale)) {
                result.errors.insert(node,format!("Не замыкается баланс КЭ плиты в узле X={:.3}, Y={:.3} мм: ΣF={:.6} кН, ΣMx={:.6}, ΣMy={:.6} кН·м. Возможна передача усилий стене/балке или несогласованное сопряжение",node.0 as f64/1e5,node.1 as f64/1e5,total[0],total[1],total[2]));
                continue;
            }
            // Remove the redundant patch equilibrium equation, fixing only the
            // multiplier gauge. This does not constrain a physical edge force.
            let rank = n.saturating_sub(1);
            let mut gram = vec![vec![0.; rank]; rank];
            for col in &columns {
                for a in 0..rank {
                    for b in 0..rank {
                        gram[a][b] += col[a] * col[b];
                    }
                }
            }
            let mut values = references.clone();
            for d in 0..3 {
                let mut residual: Vec<_> = (0..rank)
                    .map(|row| {
                        rhs[row][d]
                            - columns
                                .iter()
                                .zip(&references)
                                .map(|(c, v)| c[row] * v[d])
                                .sum::<f64>()
                    })
                    .collect();
                let mut matrix = gram.clone();
                if !solve(&mut matrix, &mut residual) {
                    invalid = true;
                    break;
                }
                for (i, col) in columns.iter().enumerate() {
                    values[i][d] += col
                        .iter()
                        .take(rank)
                        .zip(&residual)
                        .map(|(a, b)| a * b)
                        .sum::<f64>();
                }
            }
            if invalid {
                result.errors.insert(
                    node,
                    "Не удалось восстановить усилия в связной звезде узла".into(),
                );
                continue;
            }
            for (sides, v) in entries.into_iter().zip(values) {
                for (side, (row, ei, end, _)) in sides.into_iter().enumerate() {
                    let ci = star[row].0;
                    result.edges.entry((ci, ei)).or_insert([[0.; 3]; 2])[end] =
                        v.map(|x| if side == 0 { x } else { -x });
                }
            }
        }
        if sources {
            let mut mids: BTreeMap<Key, Vec<(usize, usize, [f64; 3])>> = BTreeMap::new();
            for (ci, c) in cells.iter().enumerate().filter(|(_, c)| c.quadratic) {
                for &(p, v) in &c.actions {
                    if c.poly.iter().any(|q| key(*q) == key(p)) {
                        continue;
                    }
                    for ei in 0..c.poly.len() {
                        let a = c.poly[ei];
                        let b = c.poly[(ei + 1) % c.poly.len()];
                        if key(p) == key([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]) {
                            mids.entry(key(p)).or_default().push((ci, ei, v));
                            break;
                        }
                    }
                }
            }
            for sides in mids.values() {
                let total: [f64; 3] = std::array::from_fn(|d| sides.iter().map(|s| s.2[d]).sum());
                for &(ci, ei, v) in sides {
                    result.midpoints.insert(
                        (ci, ei),
                        std::array::from_fn(|d| v[d] - total[d] / sides.len() as f64),
                    );
                }
            }
        }
        result
    }
    pub fn error(&self, p: Point) -> Option<&str> {
        self.errors.get(&key(p)).map(String::as_str)
    }
}
fn solve(a: &mut [Vec<f64>], b: &mut [f64]) -> bool {
    for i in 0..b.len() {
        let pivot = (i..b.len())
            .max_by(|&j, &k| a[j][i].abs().total_cmp(&a[k][i].abs()))
            .unwrap();
        if a[pivot][i].abs() < 1e-12 {
            return false;
        }
        a.swap(i, pivot);
        b.swap(i, pivot);
        let diagonal = a[i][i];
        for k in i..b.len() {
            a[i][k] /= diagonal;
        }
        b[i] /= diagonal;
        for j in 0..b.len() {
            if j != i {
                let factor = a[j][i];
                for k in i..b.len() {
                    a[j][k] -= factor * a[i][k];
                }
                b[j] -= factor * b[i];
            }
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn constitutive_patch_preserves_constant_flux_and_opposite_edge_actions() {
        let mut cells = vec![];
        for (x, y) in [(-1000., -1000.), (0., -1000.), (0., 0.), (-1000., 0.)] {
            let poly = vec![
                [x, y],
                [x + 1000., y],
                [x + 1000., y + 1000.],
                [x, y + 1000.],
            ];
            let mut actions = vec![([0.; 2], [0.; 3]); 4];
            for i in 0..4 {
                actions[i].0 = poly[i];
            }
            let field = [3., 4., 0.5, 2., -1.];
            for i in 0..4 {
                let a = poly[i];
                let b = poly[(i + 1) % 4];
                let nx = (b[1] - a[1]) / 1000.;
                let ny = -(b[0] - a[0]) / 1000.;
                let v = [
                    (field[3] * nx + field[4] * ny) * 0.5,
                    (field[2] * nx + field[1] * ny) * 0.5,
                    -(field[0] * nx + field[2] * ny) * 0.5,
                ];
                for j in [i, (i + 1) % 4] {
                    for d in 0..3 {
                        actions[j].1[d] += v[d];
                    }
                }
            }
            cells.push(Cell {
                poly,
                actions,
                field,
                kirchhoff: false,
                quadratic: false,
            });
        }
        // A non-constitutive perturbation with zero patch resultant must be
        // equilibrated by the correction, without breaking edge antisymmetry.
        let mut perturbed = cells.clone();
        for (ci, sign) in [(0, 1.), (1, -1.)] {
            let (_, v) = perturbed[ci]
                .actions
                .iter_mut()
                .find(|(p, _)| *p == [0., 0.])
                .unwrap();
            for d in 0..3 {
                v[d] += sign * [0.6, 0.2, -0.1][d];
            }
        }
        let corrected = Recovery::new(&perturbed);
        assert!(corrected.error([0., 0.]).is_none());
        let mut edge_sums: BTreeMap<Key, [f64; 3]> = BTreeMap::new();
        for (ci, c) in perturbed.iter().enumerate() {
            let vi = c.poly.iter().position(|p| *p == [0., 0.]).unwrap();
            let mut sum = [0.; 3];
            for (ei, end) in [(vi, 0), ((vi + 3) % 4, 1)] {
                let v = corrected.edges[&(ci, ei)][end];
                let other = c.poly[if end == 0 { (ei + 1) % 4 } else { ei }];
                let edge = edge_sums.entry(key(other)).or_default();
                for d in 0..3 {
                    sum[d] += v[d];
                    edge[d] += v[d];
                }
            }
            for d in 0..3 {
                assert!((sum[d] - c.actions[vi].1[d]).abs() < 1e-12);
            }
        }
        assert!(edge_sums.values().flatten().all(|v| v.abs() < 1e-12));
        let mut coupled = cells.clone();
        let source = [8., 2., -3.];
        let (_, v) = coupled[0]
            .actions
            .iter_mut()
            .find(|(p, _)| *p == [0., 0.])
            .unwrap();
        for d in 0..3 {
            v[d] += source[d];
        }
        assert!(Recovery::new(&coupled).error([0., 0.]).is_some());
        let faces = Recovery::faces(&coupled, &vec![vec![]; coupled.len()]);
        assert!(faces.error([0., 0.]).is_none());
        for (ci, c) in coupled.iter().enumerate() {
            let vi = c.poly.iter().position(|p| *p == [0., 0.]).unwrap();
            let a = faces.edges[&(ci, vi)][0];
            let b = faces.edges[&(ci, (vi + 3) % 4)][1];
            for d in 0..3 {
                assert!((a[d] + b[d] - (c.actions[vi].1[d] - source[d] * 0.25)).abs() < 1e-12);
            }
        }
        let recovered = Recovery::new(&cells);
        assert!(recovered.error([0., 0.]).is_none());
        for (ci, c) in cells.iter().enumerate() {
            let vi = c.poly.iter().position(|p| *p == [0., 0.]).unwrap();
            let a = recovered.edges[&(ci, vi)][0];
            let b = recovered.edges[&(ci, (vi + 3) % 4)][1];
            let original = c.actions[vi].1;
            for d in 0..3 {
                assert!((a[d] + b[d] - original[d]).abs() < 1e-12);
            }
            for ei in [vi, (vi + 3) % 4] {
                let e = [c.poly[ei], c.poly[(ei + 1) % 4]];
                let nx = (e[1][1] - e[0][1]) / 1000.;
                let ny = -(e[1][0] - e[0][0]) / 1000.;
                let expected = (2. * nx - ny) * 0.5;
                let end = if ei == vi { 0 } else { 1 };
                assert!((recovered.edges[&(ci, ei)][end][0] - expected).abs() < 1e-12);
            }
        }
    }
}
