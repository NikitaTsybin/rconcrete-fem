//! Total-Lagrangian membrane integration point for a flat shell.
//! Gradients are per metre and displacements in metres. The integration
//! weight is the area represented by the point. Bending is separate.
use super::{dot, membrane_d, Material, Point, V3};

pub(super) struct Response {
    pub(super) stress: V3,
    pub(super) force: Vec<V3>,
    pub(super) tangent: Vec<Vec<f64>>,
}

pub(super) fn integrate(material: Material, grad: &[Point], q: &[V3], area: f64) -> Response {
    let mut dx = [1., 0., 0.];
    let mut dy = [0., 1., 0.];
    for (&[gx, gy], u) in grad.iter().zip(q) {
        for axis in 0..3 {
            dx[axis] += gx * u[axis];
            dy[axis] += gy * u[axis];
        }
    }
    let strain = [
        0.5 * (dot(dx, dx) - 1.),
        0.5 * (dot(dy, dy) - 1.),
        dot(dx, dy),
    ];
    let d = membrane_d(material);
    let stress = d.map(|row| dot(row, strain));
    let b = grad
        .iter()
        .flat_map(|&[gx, gy]| {
            std::array::from_fn::<_, 3, _>(|axis| {
                [gx * dx[axis], gy * dy[axis], gx * dy[axis] + gy * dx[axis]]
            })
        })
        .collect::<Vec<_>>();
    let force = b
        .chunks_exact(3)
        .map(|rows| std::array::from_fn(|axis| area * dot(rows[axis], stress)))
        .collect();
    let tangent = b
        .iter()
        .enumerate()
        .map(|(i, &bi)| {
            let [gx, gy] = grad[i / 3];
            b.iter()
                .enumerate()
                .map(|(j, &bj)| {
                    let [hx, hy] = grad[j / 3];
                    let material = dot(bi, d.map(|row| dot(row, bj)));
                    let geometric = if i % 3 == j % 3 {
                        stress[0] * gx * hx + stress[1] * gy * hy + stress[2] * (gx * hy + gy * hx)
                    } else {
                        0.
                    };
                    area * (material + geometric)
                })
                .collect()
        })
        .collect();
    Response {
        stress,
        force,
        tangent,
    }
}

#[cfg(test)]
const GRAD: [Point; 3] = [[-1., -1.], [1., 0.], [0., 1.]];

#[test]
fn tangent_matches_internal_force_derivative() {
    let material = Material::default();
    let q = [[0., 0., 0.], [-0.002, 0.001, 0.08], [0.004, -0.001, -0.03]];
    let base = integrate(material, &GRAD, &q, 0.5);
    for j in 0..9 {
        let mut plus = q;
        let mut minus = q;
        let h = 1e-6;
        plus[j / 3][j % 3] += h;
        minus[j / 3][j % 3] -= h;
        let plus = integrate(material, &GRAD, &plus, 0.5);
        let minus = integrate(material, &GRAD, &minus, 0.5);
        for i in 0..9 {
            let derivative = (plus.force[i / 3][i % 3] - minus.force[i / 3][i % 3]) / (2. * h);
            let analytical = base.tangent[i][j];
            assert!((derivative - analytical).abs() < 1e-3 + 1e-7 * analytical.abs());
        }
    }
}

#[test]
fn finite_rigid_rotation_leaves_membrane_stress_zero() {
    let (sin, cos) = 0.7_f64.sin_cos();
    let q = [[0., 0., 0.], [cos - 1., 0., sin], [0., 0., 0.]];
    let response = integrate(Material::default(), &GRAD, &q, 0.5);
    assert!(response.stress.iter().all(|v| v.abs() < 1e-8));
    assert!(response.force.iter().flatten().all(|v| v.abs() < 1e-8));
}

pub(super) fn stress_linearized(
    material: Material,
    grad: &[Point],
    base: &[V3],
    delta: &[V3],
) -> V3 {
    let mut dx = [1., 0., 0.];
    let mut dy = [0., 1., 0.];
    let mut du = [0.; 3];
    let mut dv = [0.; 3];
    for ((&[gx, gy], u), v) in grad.iter().zip(base).zip(delta) {
        for a in 0..3 {
            dx[a] += gx * u[a];
            dy[a] += gy * u[a];
            du[a] += gx * v[a];
            dv[a] += gy * v[a];
        }
    }
    let strain = [dot(dx, du), dot(dy, dv), dot(dx, dv) + dot(dy, du)];
    let initial = integrate(material, grad, base, 0.).stress;
    let extra = membrane_d(material).map(|row| dot(row, strain));
    std::array::from_fn(|a| initial[a] + extra[a])
}
