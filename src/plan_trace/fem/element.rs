//! Isotropic DKT bending triangle. DOFs are [w, dw/dx, dw/dy] per vertex.
//! All geometry is in global XY metres; all constitutive quantities use kN/metres.
use super::super::geometry::Point;
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Material {
    pub e_mpa: f64,
    pub nu: f64,
    pub thickness_mm: f64,
    #[serde(default = "default_weight")]
    pub weight: super::super::loads::Magnitude,
}
fn default_weight() -> super::super::loads::Magnitude {
    super::super::loads::Magnitude {
        value: 2.75,
        units: super::super::loads::ForceUnits::TonForce,
    }
}
impl Default for Material {
    fn default() -> Self {
        Self {
            e_mpa: 30000.,
            nu: 0.2,
            thickness_mm: 200.,
            weight: default_weight(),
        }
    }
}
impl Material {
    pub fn wall_default() -> Self {
        let mut m = Self::default();
        m.weight.value = 0.;
        m
    }

    pub fn validate(self) -> Result<(), String> {
        if !self.e_mpa.is_finite()
            || self.e_mpa <= 0.
            || !self.thickness_mm.is_finite()
            || self.thickness_mm <= 0.
            || !self.nu.is_finite()
            || self.nu <= -1.
            || self.nu >= 0.5
            || !self.weight.value.is_finite()
            || self.weight.value < 0.
        {
            return Err("Проверьте E > 0, h > 0, −1 < ν < 0,5 и Вес ≥ 0".into());
        }
        Ok(())
    }
    pub fn self_weight_kn_m2(self, g: f64) -> f64 {
        self.weight
            .in_units(super::super::loads::ForceUnits::KiloNewton, g)
            * self.thickness_mm
            / 1000.
    }
    pub fn d(self) -> [[f64; 3]; 3] {
        let t = self.thickness_mm / 1000.;
        let c = self.e_mpa * 1000. * t.powi(3) / (12. * (1. - self.nu * self.nu));
        [
            [c, c * self.nu, 0.],
            [c * self.nu, c, 0.],
            [0., 0., c * (1. - self.nu) / 2.],
        ]
    }
}
pub struct Triangle {
    pub xy: [Point; 3],
    pub area: f64,
    grad: [[f64; 2]; 3],
    slope: [[[f64; 9]; 2]; 6],
}
impl Triangle {
    pub fn new(xy: [Point; 3]) -> Result<Self, String> {
        let x = xy[1][0] - xy[0][0];
        let y = xy[1][1] - xy[0][1];
        let u = xy[2][0] - xy[0][0];
        let v = xy[2][1] - xy[0][1];
        let det = x * v - y * u;
        if !det.is_finite() || det.abs() < 1e-14 {
            return Err("Вырожденный КЭ".into());
        }
        let grad = [
            [(y - v) / det, (u - x) / det],
            [v / det, -u / det],
            [-y / det, x / det],
        ];
        let mut slope = [[[0.; 9]; 2]; 6];
        for i in 0..3 {
            slope[i][0][3 * i + 1] = 1.;
            slope[i][1][3 * i + 2] = 1.;
        }
        // Cubic Hermite tangential derivative at the edge midpoint; normal
        // derivative is the average of its endpoint values (discrete Kirchhoff).
        for (edge, (i, j)) in [(0, 1), (1, 2), (2, 0)].into_iter().enumerate() {
            let dx = xy[j][0] - xy[i][0];
            let dy = xy[j][1] - xy[i][1];
            let len = dx.hypot(dy);
            let t = [dx / len, dy / len];
            let n = [-t[1], t[0]];
            let m = edge + 3;
            for component in 0..2 {
                slope[m][component][3 * i] = -1.5 * t[component] / len;
                slope[m][component][3 * j] = 1.5 * t[component] / len;
                for axis in 0..2 {
                    let value = 0.5 * n[component] * n[axis] - 0.25 * t[component] * t[axis];
                    slope[m][component][3 * i + 1 + axis] = value;
                    slope[m][component][3 * j + 1 + axis] = value;
                }
            }
        }
        Ok(Self {
            xy,
            area: det.abs() * 0.5,
            grad,
            slope,
        })
    }
    pub(super) fn gradients(&self) -> [[f64; 2]; 3] {
        self.grad
    }
    pub fn barycentric(&self, p: Point) -> [f64; 3] {
        let dx = p[0] - self.xy[0][0];
        let dy = p[1] - self.xy[0][1];
        let l1 = self.grad[1][0] * dx + self.grad[1][1] * dy;
        let l2 = self.grad[2][0] * dx + self.grad[2][1] * dy;
        [1. - l1 - l2, l1, l2]
    }
    pub fn b(&self, l: [f64; 3]) -> [[f64; 9]; 3] {
        let mut dn = [[0.; 2]; 6];
        for i in 0..3 {
            for axis in 0..2 {
                dn[i][axis] = (4. * l[i] - 1.) * self.grad[i][axis];
            }
        }
        for (edge, (i, j)) in [(0, 1), (1, 2), (2, 0)].into_iter().enumerate() {
            for axis in 0..2 {
                dn[edge + 3][axis] = 4. * (l[j] * self.grad[i][axis] + l[i] * self.grad[j][axis]);
            }
        }
        let mut b = [[0.; 9]; 3];
        for node in 0..6 {
            for dof in 0..9 {
                b[0][dof] += dn[node][0] * self.slope[node][0][dof];
                b[1][dof] += dn[node][1] * self.slope[node][1][dof];
                b[2][dof] +=
                    dn[node][1] * self.slope[node][0][dof] + dn[node][0] * self.slope[node][1][dof];
            }
        }
        b
    }
    pub fn stiffness(&self, material: Material) -> [[f64; 9]; 9] {
        let d = material.d();
        let mut k = [[0.; 9]; 9];
        for l in [
            [2. / 3., 1. / 6., 1. / 6.],
            [1. / 6., 2. / 3., 1. / 6.],
            [1. / 6., 1. / 6., 2. / 3.],
        ] {
            let b = self.b(l);
            for i in 0..9 {
                for j in 0..9 {
                    for a in 0..3 {
                        for c in 0..3 {
                            k[i][j] += self.area / 3. * b[a][i] * d[a][c] * b[c][j];
                        }
                    }
                }
            }
        }
        k
    }
    pub fn moments(&self, material: Material, q: [f64; 9]) -> [f64; 3] {
        self.moments_at(material, q, [1. / 3.; 3])
    }
    pub(super) fn moments_at(&self, material: Material, q: [f64; 9], l: [f64; 3]) -> [f64; 3] {
        let b = self.b(l);
        let mut curvature = [0.; 3];
        let d = material.d();
        let mut m = [0.; 3];
        for a in 0..3 {
            for i in 0..9 {
                curvature[a] += b[a][i] * q[i];
            }
        }
        for a in 0..3 {
            for c in 0..3 {
                m[a] -= d[a][c] * curvature[c];
            }
        }
        m
    }
    /// Kirchhoff equilibrium recovery, kN/m in global XY. The DKT moment
    /// field is linear, hence these exact element derivatives are constant.
    pub fn shears(&self, material: Material, q: [f64; 9]) -> [f64; 2] {
        let moments =
            [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]].map(|l| self.moments_at(material, q, l));
        let mut dm = [[0.; 2]; 3];
        for i in 0..3 {
            for a in 0..3 {
                for axis in 0..2 {
                    dm[a][axis] += moments[i][a] * self.grad[i][axis];
                }
            }
        }
        [dm[0][0] + dm[2][1], dm[1][1] + dm[2][0]]
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn shear_matches_moment_boundary_flux_and_orientation() {
        let xy = [[0.1, 0.2], [2.3, 0.4], [0.4, 1.7]];
        let nodal = [
            [0.002, 0.001, -0.002],
            [0.003, -0.001, 0.004],
            [-0.001, 0.002, 0.001],
        ];
        let material = Material::default();
        let t = Triangle::new(xy).unwrap();
        let q = std::array::from_fn(|i| nodal[i / 3][i % 3]);
        let shear = t.shears(material, q);
        let mut flux = [0.; 2];
        for (i, j) in [(0, 1), (1, 2), (2, 0)] {
            let mut l = [0.; 3];
            l[i] = 0.5;
            l[j] = 0.5;
            let m = t.moments_at(material, q, l);
            let dx = xy[j][0] - xy[i][0];
            let dy = xy[j][1] - xy[i][1];
            flux[0] += m[0] * dy - m[2] * dx;
            flux[1] += m[2] * dy - m[1] * dx;
        }
        for a in 0..2 {
            assert!((shear[a] - flux[a] / t.area).abs() < 1e-9);
        }
        let reversed = Triangle::new([xy[0], xy[2], xy[1]]).unwrap();
        let perm = [0, 2, 1];
        let rq = std::array::from_fn(|i| nodal[perm[i / 3]][i % 3]);
        let rs = reversed.shears(material, rq);
        for a in 0..2 {
            assert!((shear[a] - rs[a]).abs() < 1e-9);
        }
        let patch = std::array::from_fn(|i| {
            let [x, y] = xy[i / 3];
            match i % 3 {
                0 => x * x + 3. * x * y + 2. * y * y,
                1 => 2. * x + 3. * y,
                _ => 3. * x + 4. * y,
            }
        });
        assert!(t.shears(material, patch).iter().all(|q| q.abs() < 1e-8));
    }
    #[test]
    fn rigid_planes_and_quadratic_patch_have_correct_global_curvatures() {
        for xy in [
            [[0., 0.], [2., 0.], [0.3, 1.4]],
            [[0.3, 1.4], [2., 0.], [0., 0.]],
        ] {
            let t = Triangle::new(xy).unwrap();
            for linear in [true, false] {
                let mut q = [0.; 9];
                for i in 0..3 {
                    let [x, y] = xy[i];
                    q[3 * i] = if linear {
                        1. + 2. * x - 3. * y
                    } else {
                        x * x + 3. * x * y + 2. * y * y
                    };
                    q[3 * i + 1] = if linear { 2. } else { 2. * x + 3. * y };
                    q[3 * i + 2] = if linear { -3. } else { 3. * x + 4. * y };
                }
                for l in [[1. / 3.; 3], [0.2, 0.5, 0.3]] {
                    let b = t.b(l);
                    for a in 0..3 {
                        let value = (0..9).map(|i| b[a][i] * q[i]).sum::<f64>();
                        let target = if linear { 0. } else { [2., 4., 6.][a] };
                        assert!((value - target).abs() < 1e-9);
                    }
                }
            }
        }
    }
}
