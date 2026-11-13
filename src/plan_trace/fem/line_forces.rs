//! Direct one-sided stress-resultant integration. mm geometry, kN/m fields.
//! This recovers boundary stresses, not an equilibrated nodal-force fit.
use super::super::geometry::Point;
const GAUSS: [(f64, f64); 5] = [
    (-0.906179845938664, 0.236926885056189),
    (-0.538469310105683, 0.478628670499366),
    (0., 0.568888888888889),
    (0.538469310105683, 0.478628670499366),
    (0.906179845938664, 0.236926885056189),
];
pub(super) fn integrate(
    poly: &[Point],
    mut sample: impl FnMut(Point) -> Option<([f64; 3], [f64; 2])>,
    kirchhoff: bool,
) -> Result<Vec<[f64; 3]>, String> {
    let area = (0..poly.len())
        .map(|i| {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            a[0] * b[1] - a[1] * b[0]
        })
        .sum::<f64>();
    let mut out = vec![];
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let dx = (b[0] - a[0]) / 1000.;
        let dy = (b[1] - a[1]) / 1000.;
        let length = dx.hypot(dy);
        if length < 1e-12 {
            return Err("Вырожденное ребро КЭ".into());
        }
        let t = [dx / length, dy / length];
        let n = [t[1] * area.signum(), -t[0] * area.signum()];
        let mnt =
            |m: [f64; 3]| n[0] * (m[0] * t[0] + m[2] * t[1]) + n[1] * (m[2] * t[0] + m[1] * t[1]);
        let mut result = [0.; 3];
        let ends = if kirchhoff {
            Some([
                mnt(sample(a).ok_or("Нет поля моментов на начале ребра")?.0),
                mnt(sample(b).ok_or("Нет поля моментов на конце ребра")?.0),
            ])
        } else {
            None
        };
        for (x, w) in GAUSS {
            let u = (1. + x) * 0.5;
            let p = [a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u];
            let (m, q) = sample(p).ok_or("Не удалось вычислить поле усилий в точке ребра")?;
            if m.into_iter().chain(q).any(|v| !v.is_finite()) {
                return Err("Нечисловое поле усилий на ребре".into());
            }
            let mut shear = q[0] * n[0] + q[1] * n[1];
            let couple = if let Some([ma, mb]) = ends {
                shear += (mb - ma) / length;
                let mnn = n[0] * (m[0] * n[0] + m[2] * n[1]) + n[1] * (m[2] * n[0] + m[1] * n[1]);
                [mnn * n[1], -mnn * n[0]]
            } else {
                [m[2] * n[0] + m[1] * n[1], -(m[0] * n[0] + m[2] * n[1])]
            };
            let ds = length * w * 0.5;
            result[0] += shear * ds;
            result[1] += (couple[0] - p[1] / 1000. * shear) * ds;
            result[2] += (couple[1] + p[0] / 1000. * shear) * ds;
        }
        // Oriented endpoint terms of -Mnt * delta(w), including open ends.
        // Added once per directed element side, so shared endpoints telescope
        // only where the one-sided twisting fields actually agree.
        if let Some([ma, mb]) = ends {
            for (p, f) in [(a, ma), (b, -mb)] {
                result[0] += f;
                result[1] -= p[1] / 1000. * f;
                result[2] += p[0] / 1000. * f;
            }
        }
        out.push(result);
    }
    Ok(out)
}

pub(super) fn nodal_reference(
    poly: &[Point],
    mut sample: impl FnMut(Point) -> Option<([f64; 3], [f64; 2])>,
    kirchhoff: bool,
    quadratic: bool,
) -> Result<Vec<[[f64; 3]; 2]>, String> {
    let area = (0..poly.len())
        .map(|i| {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            a[0] * b[1] - a[1] * b[0]
        })
        .sum::<f64>();
    let mut out = vec![];
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        let d = [(b[0] - a[0]) / 1000., (b[1] - a[1]) / 1000.];
        let l = d[0].hypot(d[1]);
        if l < 1e-12 {
            return Err("Вырожденная грань".into());
        }
        let t = [d[0] / l, d[1] / l];
        let n = [t[1] * area.signum(), -t[0] * area.signum()];
        let mut v = [[0.; 3]; 2];
        for (g, w) in GAUSS {
            let u = (g + 1.) * 0.5;
            let ds = l * w * 0.5;
            let p = [a[0] + u * (b[0] - a[0]), a[1] + u * (b[1] - a[1])];
            let (m, q) = sample(p).ok_or("Нет поля на грани")?;
            let qn = q[0] * n[0] + q[1] * n[1];
            let bt = -(t[0] * (m[0] * n[0] + m[2] * n[1]) + t[1] * (m[2] * n[0] + m[1] * n[1]));
            let bn = -(n[0] * (m[0] * n[0] + m[2] * n[1]) + n[1] * (m[2] * n[0] + m[1] * n[1]));
            for end in 0..2 {
                let shape = if end == 0 { 1. - u } else { u };
                if kirchhoff {
                    let (h, dh, hrot, dhrot) = if end == 0 {
                        (
                            2. * u * u * u - 3. * u * u + 1.,
                            6. * u * u - 6. * u,
                            u * u * u - 2. * u * u + u,
                            3. * u * u - 4. * u + 1.,
                        )
                    } else {
                        (
                            -2. * u * u * u + 3. * u * u,
                            -6. * u * u + 6. * u,
                            u * u * u - u * u,
                            3. * u * u - 2. * u,
                        )
                    };
                    v[end][0] += (qn * h + bt * dh / l) * ds;
                    let tangent = (qn * l * hrot + bt * dhrot) * ds;
                    let normal = bn * shape * ds;
                    v[end][1] -= tangent * t[1] + normal * n[1];
                    v[end][2] += tangent * t[0] + normal * n[0];
                } else if quadratic {
                    v[end][0] += qn * shape * (2. * shape - 1.) * ds;
                } else {
                    v[end][0] += qn * shape * ds;
                    v[end][1] += (m[2] * n[0] + m[1] * n[1]) * shape * ds;
                    v[end][2] -= (m[0] * n[0] + m[2] * n[1]) * shape * ds;
                }
            }
        }
        out.push(v);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn consistent_edge_loads_preserve_integrated_force_and_global_moments() {
        let poly = vec![[100., 200.], [2300., 600.], [700., 2100.]];
        let field = |p: Point| Some(([2. + p[0] / 1000., 3., 0.7], [4., -1.]));
        let integral = integrate(&poly, field, false).unwrap();
        let loads = nodal_reference(&poly, field, false, false).unwrap();
        for i in 0..poly.len() {
            let mut sum = [0.; 3];
            for (p, v) in [poly[i], poly[(i + 1) % poly.len()]]
                .into_iter()
                .zip(loads[i])
            {
                sum[0] += v[0];
                sum[1] += v[1] - p[1] / 1000. * v[0];
                sum[2] += v[2] + p[0] / 1000. * v[0];
            }
            for d in 0..3 {
                assert!((sum[d] - integral[i][d]).abs() < 1e-11);
            }
        }
    }
    #[test]
    fn constant_shear_integrates_with_outward_normal_and_orientation() {
        for poly in [
            vec![[0., 0.], [2000., 0.], [2000., 1000.], [0., 1000.]],
            vec![[0., 1000.], [2000., 1000.], [2000., 0.], [0., 0.]],
        ] {
            let r = integrate(&poly, |_| Some(([0.; 3], [3., 5.])), false).unwrap();
            for (i, v) in r.iter().enumerate() {
                let a = poly[i];
                let b = poly[(i + 1) % poly.len()];
                let area = (0..poly.len())
                    .map(|j| {
                        let a = poly[j];
                        let b = poly[(j + 1) % poly.len()];
                        a[0] * b[1] - a[1] * b[0]
                    })
                    .sum::<f64>();
                let expected = (3. * (b[1] - a[1]) - 5. * (b[0] - a[0])) / 1000. * area.signum();
                assert!((v[0] - expected).abs() < 1e-12);
                assert!((v[1] + (a[1] + b[1]) / 2000. * expected).abs() < 1e-12);
                assert!((v[2] - (a[0] + b[0]) / 2000. * expected).abs() < 1e-12);
            }
        }
    }
    #[test]
    fn kirchhoff_corner_terms_equal_full_traction_virtual_work_on_each_open_edge() {
        let poly = vec![[700., 1200.], [2900., 1500.], [2100., 3400.]];
        let field = |p: Point| {
            let x = p[0] / 1000.;
            let y = p[1] / 1000.;
            Some(([2. * x + 3. * y, 4. * x - y, x + 5. * y], [7., 0.]))
        };
        let full = integrate(&poly, field, false).unwrap();
        let reduced = integrate(&poly, field, true).unwrap();
        for (a, b) in full.iter().zip(reduced) {
            for d in 0..3 {
                assert!((a[d] - b[d]).abs() < 1e-11, "{a:?} vs {b:?}");
            }
        }
    }
}
