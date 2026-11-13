//! Polynomial tractions on a physical FE side, reconstructed from consistent
//! face loads. RM: linear; P2/CR: quadratic transverse force; DKT: Hermite
//! conjugate effective shear, normal moment and oriented endpoint forces.
use super::Point;
pub(super) struct Trace {
    pub q: [f64; 3],
    pub cx: [f64; 2],
    pub cy: [f64; 2],
    pub corners: [f64; 2],
}
impl Trace {
    pub fn new(
        a: Point,
        b: Point,
        n: Point,
        v: [[f64; 3]; 2],
        mid: [f64; 3],
        kirchhoff: bool,
        quadratic: bool,
    ) -> Self {
        let length = (b[0] - a[0]).hypot(b[1] - a[1]) / 1000.;
        let mut out = Self {
            q: [0.; 3],
            cx: [0.; 2],
            cy: [0.; 2],
            corners: [0.; 2],
        };
        if quadratic {
            let qa = (9. * v[0][0] - 1.5 * mid[0] + 3. * v[1][0]) / length;
            let qm = (-1.5 * v[0][0] + 2.25 * mid[0] - 1.5 * v[1][0]) / length;
            let qb = (3. * v[0][0] - 1.5 * mid[0] + 9. * v[1][0]) / length;
            out.q = [qa, 4. * qm - 3. * qa - qb, 2. * qa - 4. * qm + 2. * qb];
            out.cx = [mid[1] / length, 0.];
            out.cy = [mid[2] / length, 0.];
        } else if kirchhoff {
            let t = [
                (b[0] - a[0]) / 1000. / length,
                (b[1] - a[1]) / 1000. / length,
            ];
            let tangent = v.map(|v| v[2] * t[0] - v[1] * t[1]);
            let normal = v.map(|v| v[2] * n[0] - v[1] * n[1]);
            let sum = 2. * (v[0][0] + v[1][0]) / length;
            let difference = 60. * (tangent[0] + tangent[1]) / (length * length);
            let qa = (sum + difference) * 0.5;
            let qb = (sum - difference) * 0.5;
            let avg = v[0][0] - length * (7. * qa + 3. * qb) / 20.;
            let delta = 12. / length * (tangent[0] - length * length * (3. * qa + 2. * qb) / 60.);
            let ma = avg - delta * 0.5;
            let mb = avg + delta * 0.5;
            let na = -(4. * normal[0] - 2. * normal[1]) / length;
            let nb = -(4. * normal[1] - 2. * normal[0]) / length;
            out.q = [qa + delta / length, qb - qa, 0.];
            out.cx = [na * n[1], (nb - na) * n[1]];
            out.cy = [-na * n[0], -(nb - na) * n[0]];
            out.corners = [ma, -mb];
        } else {
            out.q = [
                (4. * v[0][0] - 2. * v[1][0]) / length,
                6. * (v[1][0] - v[0][0]) / length,
                0.,
            ];
            out.cx = [
                (4. * v[0][1] - 2. * v[1][1]) / length,
                6. * (v[1][1] - v[0][1]) / length,
            ];
            out.cy = [
                (4. * v[0][2] - 2. * v[1][2]) / length,
                6. * (v[1][2] - v[0][2]) / length,
            ];
        }
        out
    }
    pub fn sample(&self, u: f64) -> [f64; 3] {
        [
            self.q[0] + u * self.q[1] + u * u * self.q[2],
            self.cx[0] + u * self.cx[1],
            self.cy[0] + u * self.cy[1],
        ]
    }
    pub fn integral(&self, a: Point, b: Point) -> [f64; 3] {
        let length = (b[0] - a[0]).hypot(b[1] - a[1]) / 1000.;
        let mut total = [0.; 3];
        for (u, w) in [
            (0.1127016653792583, 5. / 18.),
            (0.5, 4. / 9.),
            (0.8872983346207417, 5. / 18.),
        ] {
            let p = [a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])];
            let [q, cx, cy] = self.sample(u);
            let ds = length * w;
            total[0] += q * ds;
            total[1] += (cx - p[1] / 1000. * q) * ds;
            total[2] += (cy + p[0] / 1000. * q) * ds;
        }
        for (p, f) in [a, b].into_iter().zip(self.corners) {
            total[0] += f;
            total[1] -= p[1] / 1000. * f;
            total[2] += p[0] / 1000. * f;
        }
        total
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn reconstructed_trace_matches_all_dkt_and_mindlin_boundary_virtual_dofs() {
        let a: Point = [200., 500.];
        let b: Point = [1700., 1300.];
        let length = (b[0] - a[0]).hypot(b[1] - a[1]) / 1000.;
        let t = [
            (b[0] - a[0]) / 1000. / length,
            (b[1] - a[1]) / 1000. / length,
        ];
        let n = [t[1], -t[0]];
        let loads = [[2., 0.7, -1.3], [-3., 0.2, 0.9]];
        for kirchhoff in [false, true] {
            let trace = Trace::new(a, b, n, loads, [0.; 3], kirchhoff, false);
            for dof in 0..6 {
                let mut delta = [[0.; 3]; 2];
                delta[dof / 3][dof % 3] = 1.;
                let beta = delta.map(|v| [v[2], -v[1]]);
                let bt = beta.map(|v| v[0] * t[0] + v[1] * t[1]);
                let bn = beta.map(|v| v[0] * n[0] + v[1] * n[1]);
                let mut work = 0.;
                for (u, w) in [
                    (0.1127016653792583, 5. / 18.),
                    (0.5, 4. / 9.),
                    (0.8872983346207417, 5. / 18.),
                ] {
                    let [q, cx, cy] = trace.sample(u);
                    let (dw, theta) = if kirchhoff {
                        let dw = (2. * u * u * u - 3. * u * u + 1.) * delta[0][0]
                            + length * (u * u * u - 2. * u * u + u) * bt[0]
                            + (-2. * u * u * u + 3. * u * u) * delta[1][0]
                            + length * (u * u * u - u * u) * bt[1];
                        let beta_n = (1. - u) * bn[0] + u * bn[1];
                        (dw, [-beta_n * n[1], beta_n * n[0]])
                    } else {
                        (
                            (1. - u) * delta[0][0] + u * delta[1][0],
                            [
                                (1. - u) * delta[0][1] + u * delta[1][1],
                                (1. - u) * delta[0][2] + u * delta[1][2],
                            ],
                        )
                    };
                    work += (q * dw + cx * theta[0] + cy * theta[1]) * length * w;
                }
                work += trace.corners[0] * delta[0][0] + trace.corners[1] * delta[1][0];
                assert!(
                    (work - loads[dof / 3][dof % 3]).abs() < 1e-10,
                    "DKT={kirchhoff} DOF{dof}: {work}"
                );
            }
        }
    }
    #[test]
    fn all_trace_types_preserve_face_force_and_first_moments() {
        let a: Point = [200., 500.];
        let b: Point = [1700., 1300.];
        let l = (b[0] - a[0]).hypot(b[1] - a[1]);
        let n = [(b[1] - a[1]) / l, -(b[0] - a[0]) / l];
        for (kirchhoff, quadratic) in [(false, false), (true, false), (false, true)] {
            let v = if quadratic {
                [[2., 0., 0.], [-3., 0., 0.]]
            } else {
                [[2., 0.7, -1.3], [-3., 0.2, 0.9]]
            };
            let mid = if quadratic { [5., 1.2, -0.8] } else { [0.; 3] };
            let trace = Trace::new(a, b, n, v, mid, kirchhoff, quadratic);
            let got = trace.integral(a, b);
            let mut want = [0.; 3];
            for (p, f) in [a, b, [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]]
                .into_iter()
                .zip([v[0], v[1], mid])
            {
                want[0] += f[0];
                want[1] += f[1] - p[1] / 1000. * f[0];
                want[2] += f[2] + p[0] / 1000. * f[0];
            }
            for d in 0..3 {
                assert!(
                    (got[d] - want[d]).abs() < 1e-10,
                    "{kirchhoff}/{quadratic} {got:?} {want:?}"
                );
            }
        }
    }
}
