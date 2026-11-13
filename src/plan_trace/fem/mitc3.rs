//! Three-node Reissner–Mindlin MITC triangle, [w,beta_x,beta_y] per vertex.
//! Covariant shear a+c*s, b-c*r is tied to the three edge midpoints.
//! Lee & Bathe (2004), doi:10.1016/j.compstruc.2004.02.004, Fig. 8.
//! Plain MITC3 can lock in very thin bending-dominated meshes; it is not P2/CR.
use super::element::{Material, Triangle};
use super::Point;
pub struct Tri3 {
    pub triangle: Triangle,
}
impl Tri3 {
    pub fn new(xy: [Point; 3]) -> Result<Self, String> {
        Ok(Self {
            triangle: Triangle::new(xy)?,
        })
    }
    fn edge(&self, a: usize, b: usize) -> [f64; 9] {
        let mut row = [0.; 9];
        row[3 * a] = -1.;
        row[3 * b] = 1.;
        for axis in 0..2 {
            let delta = self.triangle.xy[b][axis] - self.triangle.xy[a][axis];
            row[3 * a + 1 + axis] = -0.5 * delta;
            row[3 * b + 1 + axis] = -0.5 * delta;
        }
        row
    }
    pub fn shear(&self, l: [f64; 3]) -> [[f64; 9]; 2] {
        let a = self.edge(0, 1);
        let b = self.edge(0, 2);
        let edge = self.edge(1, 2);
        let grad = self.triangle.gradients();
        std::array::from_fn(|axis| {
            std::array::from_fn(|i| {
                let c = b[i] - a[i] - edge[i];
                grad[1][axis] * (a[i] + c * l[2]) + grad[2][axis] * (b[i] - c * l[1])
            })
        })
    }
    pub fn bending(&self) -> [[f64; 9]; 3] {
        let g = self.triangle.gradients();
        let mut b = [[0.; 9]; 3];
        for i in 0..3 {
            b[0][3 * i + 1] = g[i][0];
            b[1][3 * i + 2] = g[i][1];
            b[2][3 * i + 1] = g[i][1];
            b[2][3 * i + 2] = g[i][0];
        }
        b
    }
    fn rigidity(m: Material) -> f64 {
        (5. / 6.) * (m.e_mpa * 1000.) / (2. * (1. + m.nu)) * (m.thickness_mm / 1000.)
    }
    pub fn stiffness(&self, m: Material) -> [[f64; 9]; 9] {
        let b = self.bending();
        let d = m.d();
        let ga = Self::rigidity(m);
        let mut k = [[0.; 9]; 9];
        for l in [
            [2. / 3., 1. / 6., 1. / 6.],
            [1. / 6., 2. / 3., 1. / 6.],
            [1. / 6., 1. / 6., 2. / 3.],
        ] {
            let s = self.shear(l);
            for i in 0..9 {
                for j in 0..9 {
                    let bend = (0..3)
                        .map(|a| (0..3).map(|c| b[a][i] * d[a][c] * b[c][j]).sum::<f64>())
                        .sum::<f64>();
                    k[i][j] += self.triangle.area / 3.
                        * (bend + ga * (s[0][i] * s[0][j] + s[1][i] * s[1][j]));
                }
            }
        }
        k
    }
    pub fn recover(&self, m: Material, q: [f64; 9]) -> ([f64; 3], [f64; 2]) {
        self.recover_at(m, q, [1. / 3.; 3])
    }
    pub fn recover_at(&self, m: Material, q: [f64; 9], l: [f64; 3]) -> ([f64; 3], [f64; 2]) {
        let curvature = self
            .bending()
            .map(|r| (0..9).map(|i| r[i] * q[i]).sum::<f64>());
        let moment = m
            .d()
            .map(|r| -(0..3).map(|i| r[i] * curvature[i]).sum::<f64>());
        let shear = self
            .shear(l)
            .map(|r| Self::rigidity(m) * (0..9).map(|i| r[i] * q[i]).sum::<f64>());
        (moment, shear)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rigid_constant_bending_and_shear_patches_survive_rotation_and_orientation() {
        for xy in [
            [[0., 0.], [2., 0.2], [0.3, 1.7]],
            [[0., 0.], [0.3, 1.7], [2., 0.2]],
        ] {
            let t = Tri3::new(xy).unwrap();
            for h in [200., 2.] {
                let m = Material {
                    thickness_mm: h,
                    ..Default::default()
                };
                let q = std::array::from_fn(|i| {
                    let [x, y] = xy[i / 3];
                    match i % 3 {
                        0 => 0.5 * x * x + 0.25 * y * y + 0.2 * x * y,
                        1 => x + 0.2 * y,
                        _ => 0.5 * y + 0.2 * x,
                    }
                });
                let (mom, s) = t.recover(m, q);
                let expected = m.d().map(|r| -(r[0] + 0.5 * r[1] + 0.4 * r[2]));
                for i in 0..3 {
                    assert!((mom[i] - expected[i]).abs() < expected[i].abs().max(1.) * 1e-10);
                }
                assert!(s.iter().all(|v| v.abs() < SelfHelper::tol(m)));
                let rigid: [f64; 9] = std::array::from_fn(|i| {
                    let [x, y] = xy[i / 3];
                    match i % 3 {
                        0 => 1. + 0.3 * x - 0.2 * y,
                        1 => 0.3,
                        _ => -0.2,
                    }
                });
                let k = t.stiffness(m);
                let max = k.iter().flatten().map(|v| v.abs()).fold(1., f64::max);
                for i in 0..9 {
                    assert!((0..9).map(|j| k[i][j] * rigid[j]).sum::<f64>().abs() < max * 1e-10);
                }
                let shear = std::array::from_fn(|i| {
                    if i % 3 == 0 {
                        0.01 * xy[i / 3][0] - 0.02 * xy[i / 3][1]
                    } else {
                        0.
                    }
                });
                let (_, s) = t.recover(m, shear);
                assert!((s[0] - Tri3::rigidity(m) * 0.01).abs() < SelfHelper::tol(m));
                assert!((s[1] + Tri3::rigidity(m) * 0.02).abs() < SelfHelper::tol(m));
            }
        }
    }
    #[test]
    fn triangle_quad_interface_has_same_moments_and_shears_for_a_common_patch() {
        let material = Material::default();
        let quad =
            super::super::mitc4::Quad4::new([[0., 0.], [1., 0.], [1., 1.], [0., 1.]]).unwrap();
        let tri = Tri3::new([[1., 0.], [2., 0.3], [1., 1.]]).unwrap();
        let displacement = |[x, y]: Point, dof: usize| match dof {
            0 => 0.0005 * x * x + 0.00025 * y * y + 0.0002 * x * y + 0.00001 * x - 0.00002 * y,
            1 => 0.001 * x + 0.0002 * y,
            _ => 0.0005 * y + 0.0002 * x,
        };
        let tq = std::array::from_fn(|i| displacement(tri.triangle.xy[i / 3], i % 3));
        let qq = std::array::from_fn(|i| displacement(quad.xy[i / 3], i % 3));
        let (tm, ts) = tri.recover(material, tq);
        let (qm, qs) = quad.recover(material, qq, [0., 0.]);
        for i in 0..3 {
            assert!((tm[i] - qm[i]).abs() < 1e-9);
        }
        for i in 0..2 {
            assert!((ts[i] - qs[i]).abs() < 1e-8);
        }
        let old = tri.triangle.shears(material, tq);
        println!("common patch: MITC3 Q={ts:?}, MITC4 Q={qs:?}, legacy DKT Q={old:?}");
        assert!((old[0] - qs[0]).hypot(old[1] - qs[1]) > 1.);
    }
    struct SelfHelper;
    impl SelfHelper {
        fn tol(m: Material) -> f64 {
            Tri3::rigidity(m) * 1e-12
        }
    }
}
