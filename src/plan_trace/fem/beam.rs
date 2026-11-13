//! Native 12-DOF Timoshenko frame in local X (longitudinal), Y, Z.
//! Units m/kN. Positive section My gives bottom (z<0) tensile stress.
use super::{element::Material, Point};
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Section {
    pub material: Material,
    pub width_mm: f64,
}
impl Section {
    pub fn validate(&self) -> Result<(), String> {
        self.material.validate()?;
        if !self.width_mm.is_finite() || self.width_mm <= 0. {
            return Err("Ширина балки должна быть положительной".into());
        }
        Ok(())
    }
    pub fn area(&self) -> f64 {
        self.width_mm * self.material.thickness_mm / 1e6
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Forces {
    pub id: u64,
    pub points: [Point; 2],
    #[serde(default)]
    pub length_m: f64,
    pub ends: [[f64; 6]; 2],
    #[serde(default)]
    pub geometric_moments: Option<[[f64; 5]; 2]>,
}
impl Forces {
    pub fn sections(&self) -> [(f64, [f64; 6]); 3] {
        [0., 0.5, 1.].map(|t| (t, self.at(t)))
    }
    // Order N,Mx,My,Mz,Qy,Qz. Moment equilibrium is integrated along each segment.
    pub fn at(&self, t: f64) -> [f64; 6] {
        let length = if self.length_m > 0. {
            self.length_m
        } else {
            (self.points[1][0] - self.points[0][0]).hypot(self.points[1][1] - self.points[0][1])
                / 1000.
        };
        let [a, b] = self.ends;
        let mut value = std::array::from_fn(|i| a[i] + t * (b[i] - a[i]));
        value[2] = a[2] - length * (a[5] * t + 0.5 * (b[5] - a[5]) * t * t);
        value[3] = a[3] - length * (a[4] * t + 0.5 * (b[4] - a[4]) * t * t);
        if let Some(correction) = &self.geometric_moments {
            for a in 0..2 {
                value[2 + a] += correction[a].iter().rev().fold(0., |v, c| v * t + c);
            }
        }
        value
    }
}
pub struct Element {
    pub length: f64,
    pub basis: [[f64; 3]; 3],
    pub k: [[f64; 12]; 12],
    pub section: Section,
}
impl Element {
    pub fn new(a: Point, b: Point, section: Section) -> Result<Self, String> {
        section.validate()?;
        let delta = [(b[0] - a[0]) / 1000., (b[1] - a[1]) / 1000.];
        let length = delta[0].hypot(delta[1]);
        if !length.is_finite() || length <= 1e-8 {
            return Err("Балка нулевой длины".into());
        }
        let x = [delta[0] / length, delta[1] / length, 0.];
        let y = [-x[1], x[0], 0.];
        let z = [0., 0., 1.];
        let e = section.material.e_mpa * 1000.;
        let g = e / (2. * (1. + section.material.nu));
        let width = section.width_mm / 1000.;
        let height = section.material.thickness_mm / 1000.;
        let area = width * height;
        let iy = width * height.powi(3) / 12.;
        let iz = height * width.powi(3) / 12.;
        // Saint-Venant torsion of a rectangle (series), not the polar area moment.
        let big = width.max(height);
        let small = width.min(height);
        let series = (0..64)
            .map(|i| {
                let n = (2 * i + 1) as f64;
                (n * std::f64::consts::PI * big / (2. * small)).tanh() / n.powi(5)
            })
            .sum::<f64>();
        let j = big * small.powi(3) / 3.
            * (1. - 192. * small * series / (std::f64::consts::PI.powi(5) * big));
        let mut k = [[0.; 12]; 12];
        for (a, b, value) in [(0, 6, e * area / length), (3, 9, g * j / length)] {
            k[a][a] += value;
            k[b][b] += value;
            k[a][b] -= value;
            k[b][a] -= value;
        }
        for (ids, sign, ei) in [([1, 5, 7, 11], 1., e * iz), ([2, 4, 8, 10], -1., e * iy)] {
            let phi = 12. * ei / ((5. / 6.) * g * area * length * length);
            let c = ei / (1. + phi);
            let a = 12. * c / length.powi(3);
            let b = 6. * c / length.powi(2);
            let d = (4. + phi) * c / length;
            let f = (2. - phi) * c / length;
            let block = [[a, b, -a, b], [b, d, -b, f], [-a, -b, a, -b], [b, f, -b, d]];
            let signs = [1., sign, 1., sign];
            for i in 0..4 {
                for j in 0..4 {
                    k[ids[i]][ids[j]] += signs[i] * block[i][j] * signs[j];
                }
            }
        }
        Ok(Self {
            length,
            basis: [x, y, z],
            k,
            section,
        })
    }
    pub fn spatial(a: [f64; 3], b: [f64; 3], section: Section) -> Result<Self, String> {
        let d = std::array::from_fn::<_, 3, _>(|i| (b[i] - a[i]) / 1000.);
        let length = d.iter().map(|v| v * v).sum::<f64>().sqrt();
        if !length.is_finite() || length < 1e-9 {
            return Err("Нулевая длина пространственного стержня".into());
        }
        let x = d.map(|v| v / length);
        let reference = if x[2].abs() < 0.95 {
            [0., 0., 1.]
        } else {
            [0., 1., 0.]
        };
        let cross = |a: [f64; 3], b: [f64; 3]| {
            [
                a[1] * b[2] - a[2] * b[1],
                a[2] * b[0] - a[0] * b[2],
                a[0] * b[1] - a[1] * b[0],
            ]
        };
        let y = cross(reference, x);
        let norm = y.iter().map(|v| v * v).sum::<f64>().sqrt();
        let y = y.map(|v| v / norm);
        let z = cross(x, y);
        let mut element = Self::new([0., 0.], [length * 1000., 0.], section)?;
        element.basis = [x, y, z];
        Ok(element)
    }
    pub fn uniform_load(&self, world: [f64; 3]) -> [f64; 12] {
        let q = self
            .basis
            .map(|a| (0..3).map(|i| a[i] * world[i]).sum::<f64>());
        let l = self.length;
        let mut f = [0.; 12];
        for i in 0..3 {
            f[i] = q[i] * l / 2.;
            f[6 + i] = q[i] * l / 2.;
        }
        f[5] = q[1] * l * l / 12.;
        f[11] = -f[5];
        f[4] = -q[2] * l * l / 12.;
        f[10] = -f[4];
        f
    }
    pub fn recover(
        &self,
        id: u64,
        points: [Point; 2],
        displacement: [f64; 12],
        load: [f64; 12],
    ) -> Forces {
        let end: [f64; 12] = std::array::from_fn(|i| {
            (0..12).map(|j| self.k[i][j] * displacement[j]).sum::<f64>() - load[i]
        });
        // Convert the right-face vector moment about Y to engineering sagging My.
        // Positive My stretches z<0; stiffness and nodal equilibrium retain physical signs.
        Forces {
            geometric_moments: None,
            id,
            points,
            length_m: self.length,
            ends: [
                [-end[0], -end[3], end[4], -end[5], -end[1], -end[2]],
                [end[6], end[9], -end[10], end[11], end[7], end[8]],
            ],
        }
    }
}

// Moment equilibrium on the displaced axis. These coefficients are
// linearized at the base state by the shell/frame caller.
pub(super) fn geometric_moments(
    length: f64,
    q: &[f64; 12],
    force: &[[f64; 3]; 2],
) -> [[f64; 5]; 2] {
    let l = length;
    let displacement = [
        [q[6] - q[0], 0., 0.],
        [
            l * q[5],
            -6. * q[1] - 4. * l * q[5] + 6. * q[7] - 2. * l * q[11],
            6. * q[1] + 3. * l * q[5] - 6. * q[7] + 3. * l * q[11],
        ],
        [
            -l * q[4],
            -6. * q[2] + 4. * l * q[4] + 6. * q[8] + 2. * l * q[10],
            6. * q[2] - 3. * l * q[4] - 6. * q[8] - 3. * l * q[10],
        ],
    ];
    let mut coefficients = [[0.; 5]; 2];
    for (result_axis, transverse) in [(0, 2), (1, 1)] {
        for k in 0..3 {
            for (degree, w) in [
                (0, force[0]),
                (1, std::array::from_fn(|a| force[1][a] - force[0][a])),
            ] {
                coefficients[result_axis][k + degree + 1] += (displacement[transverse][k] * w[0]
                    - displacement[0][k] * w[transverse])
                    / (k + degree + 1) as f64;
            }
        }
    }
    coefficients
}

#[cfg(test)]
mod tests {
    use super::*;
    fn element() -> Element {
        Element::new(
            [0., 0.],
            [3000., 0.],
            Section {
                material: Material {
                    thickness_mm: 500.,
                    ..Material::default()
                },
                width_mm: 200.,
            },
        )
        .unwrap()
    }
    fn solve_tip(e: &Element, force: [f64; 6]) -> [f64; 12] {
        let mut a = [[0.; 7]; 6];
        for i in 0..6 {
            for j in 0..6 {
                a[i][j] = e.k[6 + i][6 + j];
            }
            a[i][6] = force[i];
        }
        for column in 0..6 {
            let pivot = (column..6)
                .max_by(|&i, &j| a[i][column].abs().total_cmp(&a[j][column].abs()))
                .unwrap();
            a.swap(column, pivot);
            let d = a[column][column];
            assert!(d.abs() > 1e-12);
            for j in column..7 {
                a[column][j] /= d;
            }
            for i in 0..6 {
                if i != column {
                    let d = a[i][column];
                    for j in column..7 {
                        a[i][j] -= d * a[column][j];
                    }
                }
            }
        }
        let mut q = [0.; 12];
        for i in 0..6 {
            q[6 + i] = a[i][6];
        }
        q
    }
    #[test]
    fn geometric_moment_integrates_axial_force_on_displaced_axis() {
        let mut q = [0.; 12];
        q[7] = 0.03;
        q[5] = 0.01;
        q[11] = 0.01;
        let correction = geometric_moments(3., &q, &[[-100., 0., 0.]; 2]);
        let at = |t: f64| -> f64 { correction[1].iter().rev().fold(0_f64, |v, c| v * t + c) };
        assert!((at(0.5) + 1.5).abs() < 1e-12);
        assert!((at(1.) + 3.).abs() < 1e-12);
    }
    #[test]
    fn six_rigid_modes_and_symmetry() {
        let e = element();
        let scale = e.k.iter().flatten().map(|v| v.abs()).fold(1., f64::max);
        for mode in 0..6 {
            let mut q = [0.; 12];
            if mode < 3 {
                q[mode] = 1.;
                q[6 + mode] = 1.;
            } else {
                q[mode] = 1.;
                q[6 + mode] = 1.;
                if mode == 4 {
                    q[8] = -e.length;
                }
                if mode == 5 {
                    q[7] = e.length;
                }
            }
            for i in 0..12 {
                assert!((0..12).map(|j| e.k[i][j] * q[j]).sum::<f64>().abs() < scale * 1e-10);
                for j in 0..12 {
                    assert!((e.k[i][j] - e.k[j][i]).abs() < scale * 1e-12);
                }
            }
        }
    }
    #[test]
    fn axial_torsion_and_bending_sections_have_physical_signs() {
        let e = element();
        for (dof, component) in [(0, 0), (3, 1), (4, 2), (5, 3), (1, 4), (2, 5)] {
            let mut force = [0.; 6];
            force[dof] = 1.;
            let q = solve_tip(&e, force);
            let result = e.recover(1, [[0., 0.], [3000., 0.]], q, [0.; 12]);
            let expected = if component == 2 { -1. } else { 1. };
            assert!((result.ends[1][component] - expected).abs() < 1e-9);
        }
        let q = solve_tip(&e, [0., 0., -1., 0., 0., 0.]);
        let result = e.recover(1, [[0., 0.], [3000., 0.]], q, [0.; 12]);
        assert!((result.ends[0][2] + 3.).abs() < 1e-9);
        assert!((result.at(0.5)[2] + 1.5).abs() < 1e-9);
        let iy = 0.2 * 0.5_f64.powi(3) / 12.;
        let young = 30000. * 1000.;
        let g = young / (2. * 1.2);
        let expected = -3_f64.powi(3) / (3. * young * iy) - 3. / ((5. / 6.) * g * 0.1);
        assert!((q[8] - expected).abs() < expected.abs() * 1e-10);
        // Downward cantilever load causes hogging: bottom face is compressed.
        assert!(-result.ends[0][2] * (-0.25) / iy < 0.);
        // Reversing the load gives positive sagging My and bottom tension.
        let upward = solve_tip(&e, [0., 0., 1., 0., 0., 0.]);
        let positive = e.recover(1, [[0., 0.], [3000., 0.]], upward, [0.; 12]);
        assert!(positive.ends[0][2] > 0.);
        assert!(-positive.ends[0][2] * (-0.25) / iy > 0.);
    }
    #[test]
    fn uniform_vertical_load_preserves_force_and_first_moment() {
        let e = element();
        let f = e.uniform_load([0., 0., -2.]);
        assert!((f[2] + f[8] + 6.).abs() < 1e-12);
        let total_y = f[4] + f[10] - e.length * f[8];
        assert!((total_y - 9.).abs() < 1e-12);
    }
}
