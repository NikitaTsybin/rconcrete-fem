//! Four-node isoparametric Reissner-Mindlin plate, MITC4 transverse shear.
//! DOFs per vertex: [w, slope_x, slope_y]. Units: m, kN. No triangle integration.
//! Bathe & Dvorkin (1985), doi:10.1002/nme.1620210213.
use super::{element::Material, Point};
const G: f64 = 0.5773502691896258;
pub const GAUSS: [[f64; 2]; 4] = [[-G, -G], [G, -G], [G, G], [-G, G]];
#[derive(Clone)]
pub struct Quad4 {
    pub xy: [Point; 4],
    pub area: f64,
}
pub struct Sample {
    pub n: [f64; 4],
    pub grad: [[f64; 2]; 4],
    /// Rows are [dx/dxi,dy/dxi] and [dx/deta,dy/deta].
    pub jac: [[f64; 2]; 2],
    pub det: f64,
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shape_inverse_and_consistent_pressure_preserve_resultants() {
        let q = Quad4::new([[0.1, 0.2], [2.2, 0.3], [2.4, 1.8], [-0.2, 1.4]]).unwrap();
        for p in GAUSS.into_iter().chain([[0., 0.], [-1., -1.], [1., 1.]]) {
            let (n, dn) = shape(p);
            assert!((n.iter().sum::<f64>() - 1.).abs() < 1e-13);
            for axis in 0..2 {
                assert!(dn.iter().map(|d| d[axis]).sum::<f64>().abs() < 1e-13);
            }
            let back = q.natural(q.position(p)).unwrap();
            assert!((back[0] - p[0]).hypot(back[1] - p[1]) < 1e-9);
        }
        let f = q.pressure_load(7.);
        assert!((f.iter().step_by(3).sum::<f64>() - 7. * q.area).abs() < 1e-11);
        for axis in 0..2 {
            let exact = GAUSS
                .into_iter()
                .map(|p| 7. * q.sample(p).unwrap().det * q.position(p)[axis])
                .sum::<f64>();
            let assembled = (0..4).map(|i| f[3 * i] * q.xy[i][axis]).sum::<f64>();
            assert!((assembled - exact).abs() < 1e-11);
        }
    }
    #[test]
    fn rigid_modes_and_distorted_constant_bending_patch_do_not_lock() {
        let element = Quad4::new([[0., 0.], [2., 0.2], [2.2, 1.4], [-0.1, 1.]]).unwrap();
        for thickness in [200., 1., 0.1] {
            let material = Material {
                thickness_mm: thickness,
                ..Default::default()
            };
            let k = element.stiffness(material);
            for i in 0..12 {
                for j in 0..12 {
                    assert!((k[i][j] - k[j][i]).abs() < 1e-12);
                }
            }
            let rigid = std::array::from_fn::<_, 12, _>(|i| {
                let [x, y] = element.xy[i / 3];
                match i % 3 {
                    0 => 0.7 + 0.3 * x - 0.2 * y,
                    1 => 0.3,
                    _ => -0.2,
                }
            });
            let magnitude = k.iter().flatten().map(|v| v.abs()).fold(1., f64::max);
            assert!((0..12).all(
                |i| (0..12).map(|j| k[i][j] * rigid[j]).sum::<f64>().abs() < magnitude * 1e-10
            ));
            let bending = std::array::from_fn::<_, 12, _>(|i| {
                let [x, y] = element.xy[i / 3];
                match i % 3 {
                    0 => 0.5 * x * x + 0.25 * y * y + 0.2 * x * y,
                    1 => x + 0.2 * y,
                    _ => 0.5 * y + 0.2 * x,
                }
            });
            let curvature = [1., 0.5, 0.4];
            let d = material.d();
            for p in GAUSS {
                let (m, s) = element.recover(material, bending, p);
                for a in 0..3 {
                    let expected = -(0..3).map(|b| d[a][b] * curvature[b]).sum::<f64>();
                    assert!((m[a] - expected).abs() < expected.abs().max(1.) * 1e-9);
                }
                let raw = element
                    .shear(p)
                    .map(|r| (0..12).map(|i| r[i] * bending[i]).sum::<f64>());
                assert!(raw.into_iter().all(|v| v.abs() < 1e-10));
                assert!(s.iter().all(|v| v.is_finite()));
            }
        }
    }
}
pub fn shape([r, s]: Point) -> ([f64; 4], [[f64; 2]; 4]) {
    let corners = [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]];
    let n = corners.map(|[a, b]| 0.25 * (1. + a * r) * (1. + b * s));
    let dn = corners.map(|[a, b]| [0.25 * a * (1. + b * s), 0.25 * b * (1. + a * r)]);
    (n, dn)
}
impl Quad4 {
    pub fn new(xy: [Point; 4]) -> Result<Self, String> {
        if xy.iter().flatten().any(|v| !v.is_finite()) {
            return Err("MITC4: неконечные координаты".into());
        }
        let mut q = Self { xy, area: 0. };
        for natural in [[-1., -1.], [1., -1.], [1., 1.], [-1., 1.]]
            .into_iter()
            .chain(GAUSS)
        {
            q.sample(natural)?;
        }
        q.area = GAUSS.into_iter().map(|p| q.sample(p).unwrap().det).sum();
        if !q.area.is_finite() || q.area <= 1e-14 {
            return Err("MITC4: вырожденный четырёхугольник".into());
        }
        Ok(q)
    }
    pub fn sample(&self, p: Point) -> Result<Sample, String> {
        let (n, dn) = shape(p);
        let mut jac = [[0.; 2]; 2];
        // Relative coordinates avoid cancellation after a large rigid translation.
        for i in 0..4 {
            for a in 0..2 {
                for b in 0..2 {
                    jac[a][b] += dn[i][a] * (self.xy[i][b] - self.xy[0][b]);
                }
            }
        }
        let det = jac[0][0] * jac[1][1] - jac[0][1] * jac[1][0];
        if !det.is_finite() || det <= 1e-15 {
            return Err(
                "MITC4: неположительный якобиан; проверьте порядок узлов и выпуклость".into(),
            );
        }
        let grad = dn.map(|d| {
            [
                (jac[1][1] * d[0] - jac[0][1] * d[1]) / det,
                (-jac[1][0] * d[0] + jac[0][0] * d[1]) / det,
            ]
        });
        Ok(Sample { n, grad, jac, det })
    }
    pub fn position(&self, p: Point) -> Point {
        let (n, _) = shape(p);
        std::array::from_fn(|a| {
            self.xy[0][a]
                + (0..4)
                    .map(|i| n[i] * (self.xy[i][a] - self.xy[0][a]))
                    .sum::<f64>()
        })
    }
    pub fn natural(&self, xy: Point) -> Option<Point> {
        let mut p = [0., 0.];
        let scale = self.area.sqrt().max(1e-6);
        for _ in 0..24 {
            let s = self.sample(p).ok()?;
            let actual = self.position(p);
            let residual = [xy[0] - actual[0], xy[1] - actual[1]];
            if residual[0].hypot(residual[1]) <= scale * 1e-11 {
                return p
                    .iter()
                    .all(|v| (-1. - 1e-8..=1. + 1e-8).contains(v))
                    .then_some(p);
            }
            p[0] += (s.jac[1][1] * residual[0] - s.jac[1][0] * residual[1]) / s.det;
            p[1] += (-s.jac[0][1] * residual[0] + s.jac[0][0] * residual[1]) / s.det;
            if p.iter().any(|v| !v.is_finite() || v.abs() > 4.) {
                return None;
            }
        }
        None
    }
    pub fn bending(&self, p: Point) -> [[f64; 12]; 3] {
        let s = self.sample(p).expect("validated MITC4 Jacobian");
        let mut b = [[0.; 12]; 3];
        for i in 0..4 {
            b[0][3 * i + 1] = s.grad[i][0];
            b[1][3 * i + 2] = s.grad[i][1];
            b[2][3 * i + 1] = s.grad[i][1];
            b[2][3 * i + 2] = s.grad[i][0];
        }
        b
    }
    fn tying(&self, p: Point, axis: usize) -> [f64; 12] {
        let s = self.sample(p).expect("validated MITC4 tying point");
        let (_, dn) = shape(p);
        let mut b = [0.; 12];
        for i in 0..4 {
            b[3 * i] = dn[i][axis];
            b[3 * i + 1] = -s.n[i] * s.jac[axis][0];
            b[3 * i + 2] = -s.n[i] * s.jac[axis][1];
        }
        b
    }
    pub fn shear(&self, [r, s]: Point) -> [[f64; 12]; 2] {
        let bottom = self.tying([0., -1.], 0);
        let top = self.tying([0., 1.], 0);
        let left = self.tying([-1., 0.], 1);
        let right = self.tying([1., 0.], 1);
        let sample = self.sample([r, s]).expect("validated MITC4 Jacobian");
        let j = sample.jac;
        let mut b = [[0.; 12]; 2];
        for i in 0..12 {
            let cov_xi = 0.5 * ((1. - s) * bottom[i] + (1. + s) * top[i]);
            let cov_eta = 0.5 * ((1. - r) * left[i] + (1. + r) * right[i]);
            b[0][i] = (j[1][1] * cov_xi - j[0][1] * cov_eta) / sample.det;
            b[1][i] = (-j[1][0] * cov_xi + j[0][0] * cov_eta) / sample.det;
        }
        b
    }
    fn shear_rigidity(m: Material) -> f64 {
        (5. / 6.) * (m.e_mpa * 1000.) / (2. * (1. + m.nu)) * (m.thickness_mm / 1000.)
    }
    pub fn stiffness(&self, m: Material) -> [[f64; 12]; 12] {
        let d = m.d();
        let shear = Self::shear_rigidity(m);
        let mut k = [[0.; 12]; 12];
        for p in GAUSS {
            let det = self.sample(p).unwrap().det;
            let b = self.bending(p);
            let s = self.shear(p);
            for i in 0..12 {
                for j in i..12 {
                    let bending = (0..3)
                        .map(|a| (0..3).map(|c| b[a][i] * d[a][c] * b[c][j]).sum::<f64>())
                        .sum::<f64>();
                    k[i][j] += det * (bending + shear * (s[0][i] * s[0][j] + s[1][i] * s[1][j]));
                }
            }
        }
        for i in 0..12 {
            for j in 0..i {
                k[i][j] = k[j][i];
            }
        }
        k
    }
    pub fn pressure_load(&self, pressure: f64) -> [f64; 12] {
        let mut f = [0.; 12];
        for p in GAUSS {
            let s = self.sample(p).unwrap();
            for i in 0..4 {
                f[3 * i] += pressure * s.n[i] * s.det;
            }
        }
        f
    }
    pub fn recover(&self, m: Material, q: [f64; 12], p: Point) -> ([f64; 3], [f64; 2]) {
        let b = self.bending(p);
        let s = self.shear(p);
        let d = m.d();
        let curvature = b.map(|r| (0..12).map(|i| r[i] * q[i]).sum::<f64>());
        (
            d.map(|r| -(0..3).map(|i| r[i] * curvature[i]).sum::<f64>()),
            s.map(|r| Self::shear_rigidity(m) * (0..12).map(|i| r[i] * q[i]).sum::<f64>()),
        )
    }
}
