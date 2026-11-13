//! Independent port of the notebooks' piecewise-linear NDM (N, N mm, mm, MPa).
//! Positive strain/N is tension; positive M stretches z<0 (bottom).
#[derive(Clone, Copy, Debug)]
pub struct Section {
    pub h: f64,
    pub ab: f64,
    pub at: f64,
    pub rb: f64,
    pub eb: f64,
    pub rt: f64,
    pub rs: f64,
    pub rsc: f64,
}
#[derive(Clone, Copy, Debug)]
pub struct State {
    pub e: f64,
    pub k: f64,
    pub residual: f64,
}
#[derive(Clone, Copy, Default, serde::Serialize)]
pub struct Stats {
    pub equilibrium_calls: usize,
    pub seed_attempts: usize,
    pub fallback_calls: usize,
}
thread_local! { static STATS: std::cell::Cell<Stats> = std::cell::Cell::new(Stats{equilibrium_calls:0,seed_attempts:0,fallback_calls:0}); }
pub fn reset_stats() {
    STATS.with(|s| s.set(Stats::default()));
}
pub fn stats() -> Stats {
    STATS.with(|s| s.get())
}
fn count(update: impl FnOnce(&mut Stats)) {
    STATS.with(|s| {
        let mut value = s.get();
        update(&mut value);
        s.set(value);
    });
}
impl Section {
    pub fn steel(self, e: f64) -> f64 {
        (200000. * e).clamp(-self.rsc, self.rs)
    }
    fn segments(self, tension: bool) -> impl Iterator<Item = [f64; 4]> {
        let eb1 = 0.6 * self.rb / self.eb;
        let cb = 0.4 * self.rb / (0.002 - eb1);
        let et1 = 0.6 * self.rt / self.eb;
        let ct = 0.4 * self.rt / (0.0001 - et1);
        [
            [-0.0035, -0.002, 0., -self.rb],
            [-0.002, -eb1, cb, -0.6 * self.rb + cb * eb1],
            [-eb1, 0., self.eb, 0.],
            [0., et1, self.eb, 0.],
            [et1, 0.0001, ct, 0.6 * self.rt - ct * et1],
            [0.0001, 0.00015, 0., self.rt],
        ]
        .into_iter()
        .take(if tension { 6 } else { 3 })
    }
    pub fn concrete(self, e: f64, tension: bool) -> f64 {
        if e >= 0. && !tension {
            return 0.;
        }
        for [lo, hi, a, b] in self.segments(tension) {
            if e >= lo - 1e-12 && e <= hi + 1e-12 {
                return a * e + b;
            }
        }
        if e < 0. {
            -self.rb
        } else {
            self.rt
        }
    }
    pub fn concrete_nm(self, e: f64, k: f64, tension: bool) -> [f64; 2] {
        if (k * self.h).abs() < 1e-13 {
            return [1000. * self.h * self.concrete(e, tension), 0.];
        }
        let mut result = [0.; 2];
        for [left, right, a, b] in self.segments(tension) {
            let za = (left - e) / k;
            let zb = (right - e) / k;
            let z1 = (-self.h * 0.5).max(za.min(zb));
            let z2 = (self.h * 0.5).min(za.max(zb));
            if z2 <= z1 + 1e-12 {
                continue;
            }
            let s1 = a * (e + k * z1) + b;
            let s2 = a * (e + k * z2) + b;
            let dz = z2 - z1;
            result[0] += 1000. * dz * (s1 + s2) * 0.5;
            result[1] -= 1000. * dz * ((2. * z1 + z2) * s1 + (z1 + 2. * z2) * s2) / 6.;
        }
        result
    }
    fn concrete_modulus(self, e: f64, tension: bool) -> f64 {
        if e > 0. && !tension {
            return 0.;
        }
        self.segments(tension)
            .find(|[lo, hi, _, _]| e >= *lo && e <= *hi)
            .map_or(0., |v| v[2])
    }
    /// Exact tangent integrals [dN/de, dN/dk, -dM/dk]. Moving segment
    /// boundary terms cancel because the admissible stress diagram is continuous.
    fn tangent(self, e: f64, k: f64, areas: [f64; 2], tension: bool) -> [f64; 3] {
        let mut out = [0.; 3];
        if (k * self.h).abs() < 1e-13 {
            let ec = self.concrete_modulus(e, tension);
            out = [1000. * self.h * ec, 0., 1000. * self.h.powi(3) * ec / 12.];
        } else {
            for [left, right, ec, _] in self.segments(tension) {
                if ec == 0. {
                    continue;
                }
                let a = (left - e) / k;
                let b = (right - e) / k;
                let z1 = (-self.h * 0.5).max(a.min(b));
                let z2 = (self.h * 0.5).min(a.max(b));
                if z2 <= z1 {
                    continue;
                }
                let weight = 1000. * ec * (z2 - z1);
                out[0] += weight;
                out[1] += weight * (z1 + z2) / 2.;
                out[2] += weight * (z1 * z1 + z1 * z2 + z2 * z2) / 3.;
            }
        }
        for (area, z) in areas
            .into_iter()
            .zip([-self.h * 0.5 + self.ab, self.h * 0.5 - self.at])
        {
            let strain = e + k * z;
            let es = if strain > -self.rsc / 200000. && strain < self.rs / 200000. {
                200000.
            } else {
                0.
            };
            let stiffness = 100. * area * (es - self.concrete_modulus(strain, tension));
            out[0] += stiffness;
            out[1] += stiffness * z;
            out[2] += stiffness * z * z;
        }
        out
    }
    pub fn nm(self, e: f64, k: f64, areas: [f64; 2], tension: bool) -> [f64; 2] {
        let mut nm = self.concrete_nm(e, k, tension);
        for (area, z) in areas
            .into_iter()
            .zip([-self.h * 0.5 + self.ab, self.h * 0.5 - self.at])
        {
            let strain = e + k * z;
            let force = 100. * area * (self.steel(strain) - self.concrete(strain, tension));
            nm[0] += force;
            nm[1] -= force * z;
        }
        nm
    }
    pub fn beta(self, k: f64) -> f64 {
        let t = k.abs() * self.h * 0.5;
        if t <= 0.00175 {
            t - 0.5 * (0.002 + (0.002_f64.powi(2) + 8. * 0.0015 * t).sqrt())
        } else {
            t - 0.0035
        }
    }
    pub fn bounds(self, k: f64, tension: bool) -> [f64; 2] {
        let [zb, zt] = [-self.h * 0.5 + self.ab, self.h * 0.5 - self.at];
        let lo = self.beta(k).max(-0.025 - k * zb).max(-0.025 - k * zt);
        let mut hi = (0.025 - k * zb).min(0.025 - k * zt);
        if tension {
            hi = hi.min(self.beta_crack(k));
        }
        [lo, hi]
    }
    pub fn beta_crack(self, k: f64) -> f64 {
        let t = k.abs() * self.h * 0.5;
        if t <= 0.000075 {
            -t + 0.5 * (0.0001 + (0.0001_f64.powi(2) + 8. * 0.00005 * t).sqrt())
        } else {
            0.00015 - t
        }
    }
    pub fn curvature_limits(self) -> [f64; 2] {
        [-0.0285 / (self.h - self.ab), 0.0285 / (self.h - self.at)]
    }
    pub fn stresses(self, state: State) -> [f64; 2] {
        [-self.h * 0.5 + self.ab, self.h * 0.5 - self.at].map(|z| self.steel(state.e + state.k * z))
    }
    pub fn equilibrium(
        self,
        load: [f64; 2],
        areas: [f64; 2],
        tension: bool,
        seed: Option<State>,
    ) -> Option<State> {
        count(|s| s.equilibrium_calls += 1);
        if load.iter().any(|x| !x.is_finite()) || areas.iter().any(|x| !x.is_finite() || *x < 0.) {
            return None;
        }
        let target = [load[0] * 1000., load[1] * 1e6];
        if target.iter().all(|v| v.abs() < 1e-9) {
            return Some(State {
                e: 0.,
                k: 0.,
                residual: 0.,
            });
        }
        let half = self.h * 0.5;
        let scale = 1000_f64
            .max(target[0].abs())
            .max(target[1].abs() / half)
            .max(self.h * self.rb);
        let residual = |e: f64, k: f64| {
            let q = self.nm(e, k, areas, tension);
            [
                (q[0] - target[0]) / scale,
                (q[1] - target[1]) / (scale * half),
            ]
        };
        let norm = |f: [f64; 2]| f[0].hypot(f[1]);
        let [kl, kr] = if tension {
            [-0.00365 / self.h, 0.00365 / self.h]
        } else {
            self.curvature_limits()
        };
        let elastic_k = -target[1]
            / (self.eb * 1000. * self.h.powi(3) / 12.
                + 200000. * 100. * (areas[0] + areas[1]) * half * half);
        let elastic_e =
            target[0] / (self.eb * 1000. * self.h + 200000. * 100. * (areas[0] + areas[1]));
        let seeds = seed
            .map(|s| (s.e, s.k))
            .into_iter()
            .chain(std::iter::once((elastic_e, elastic_k)))
            .chain([0.5, 0.35, 0.65, 0.12, 0.88].into_iter().flat_map(|v| {
                let k = kl + v * (kr - kl);
                let [lo, hi] = self.bounds(k, tension);
                [0.32, 0.7, 0.06, 0.94]
                    .into_iter()
                    .filter_map(move |u| (hi >= lo).then_some((lo + u * (hi - lo), k)))
            }));
        for (mut e, mut k) in seeds {
            count(|s| s.seed_attempts += 1);
            let [lo, hi] = self.bounds(k, tension);
            if hi < lo {
                continue;
            }
            e = e.clamp(lo, hi);
            for _ in 0..50 {
                let f = residual(e, k);
                let error = norm(f);
                if error < 1e-10 {
                    return Some(State {
                        e,
                        k,
                        residual: error,
                    });
                }
                let [i0, i1, i2] = self.tangent(e, k, areas, tension);
                let a = i0 / scale;
                let b = i1 / (scale * half);
                let c = -b;
                let d = -i2 / (scale * half * half);
                let det = a * d - b * c;
                if !det.is_finite() || det.abs() < 1e-12 {
                    break;
                }
                let step_e = (-d * f[0] + b * f[1]) / det;
                let step_t = (c * f[0] - a * f[1]) / det;
                let mut alpha = 1.;
                let mut accepted = false;
                for _ in 0..23 {
                    let en = e + alpha * step_e;
                    let kn = k + alpha * step_t / half;
                    let [lo, hi] = self.bounds(kn, tension);
                    if kn >= kl
                        && kn <= kr
                        && en >= lo - 1e-12
                        && en <= hi + 1e-12
                        && norm(residual(en, kn)) < error * (1. - 1e-4 * alpha)
                    {
                        e = en;
                        k = kn;
                        accepted = true;
                        break;
                    }
                    alpha *= 0.5;
                }
                if !accepted {
                    break;
                }
            }
        }
        // Independent bracketed fallback: solve axial equilibrium in e0 for
        // each curvature, then solve moment equilibrium in curvature. This
        // avoids a singular two-variable Newton step near a material plateau.
        count(|s| s.fallback_calls += 1);
        let axial = |k: f64| -> Option<(f64, f64)> {
            let [mut left, mut right] = self.bounds(k, tension);
            if left > right {
                return None;
            }
            let mut fl = self.nm(left, k, areas, tension)[0] - target[0];
            let fr = self.nm(right, k, areas, tension)[0] - target[0];
            if fl * fr > 0. {
                return None;
            }
            for _ in 0..60 {
                let middle = (left + right) * 0.5;
                let fm = self.nm(middle, k, areas, tension)[0] - target[0];
                if fl * fm <= 0. {
                    right = middle;
                } else {
                    left = middle;
                    fl = fm;
                }
            }
            let e = (left + right) * 0.5;
            Some((e, self.nm(e, k, areas, tension)[1] - target[1]))
        };
        let mut previous: Option<(f64, f64)> = None;
        for i in 0..=128 {
            let k = kl + (kr - kl) * i as f64 / 128.;
            if let Some((e, f)) = axial(k) {
                let error = norm(residual(e, k));
                if error < 1e-10 {
                    return Some(State {
                        e,
                        k,
                        residual: error,
                    });
                }
                if let Some((kp, fp)) = previous {
                    if f * fp < 0. {
                        let (mut left, mut right, mut fl) = (kp, k, fp);
                        for _ in 0..55 {
                            let middle = (left + right) * 0.5;
                            let Some((e, fm)) = axial(middle) else {
                                break;
                            };
                            let error = norm(residual(e, middle));
                            if error < 1e-10 {
                                return Some(State {
                                    e,
                                    k: middle,
                                    residual: error,
                                });
                            }
                            if fl * fm <= 0. {
                                right = middle;
                            } else {
                                left = middle;
                                fl = fm;
                            }
                        }
                    }
                }
                previous = Some((k, f));
            } else {
                previous = None;
            }
        }
        None // Failure to converge is NOT a proof of strength or safety.
    }
    pub fn crack_ray(self, load: [f64; 2], areas: [f64; 2]) -> Option<(f64, State)> {
        self.crack_ray_impl(load, areas, true)
    }
    fn crack_ray_impl(self, load: [f64; 2], areas: [f64; 2], fast: bool) -> Option<(f64, State)> {
        let n = load[0] * 1000.;
        let m = load[1] * 1e6 / self.h;
        let den = n * n + m * m;
        if den < 1e-14 {
            return None;
        }
        let evaluate = |k: f64| {
            let e = self.beta_crack(k);
            let [lo, hi] = self.bounds(k, true);
            if e < lo - 1e-11 || e > hi + 1e-11 {
                return None;
            }
            let q = self.nm(e, k, areas, true);
            let moment = q[1] / self.h;
            Some((q[0] * m - moment * n, (q[0] * n + moment * m) / den, e))
        };
        let limit = (0.0035 + 0.00015) / self.h;
        // In (numerically) pure bending, N along the crack boundary is monotone
        // once the compressed edge appears. Bracket it directly; retain the
        // original scan for all other rays and any inadmissible bracket.
        if fast && n.abs() <= 1e-12 * m.abs() && m != 0. {
            let sign = -m.signum();
            let k0 = sign * 0.00015 / self.h;
            let k1 = sign * limit;
            let (mut left, mut right) = (k0.min(k1), k0.max(k1));
            if let (Some((mut fl, _, _)), Some((fr, _, _))) = (evaluate(left), evaluate(right)) {
                if fl * fr < 0. {
                    let mut valid = true;
                    for _ in 0..45 {
                        let mid = (left + right) * 0.5;
                        if let Some((fm, _, _)) = evaluate(mid) {
                            if fl * fm <= 0. {
                                right = mid;
                            } else {
                                left = mid;
                                fl = fm;
                            }
                        } else {
                            valid = false;
                            break;
                        }
                    }
                    let k = (left + right) * 0.5;
                    if valid {
                        if let Some((f, lambda, e)) = evaluate(k) {
                            if lambda > 1e-12 && f.abs() < 1e-5 * (1. + den) {
                                return Some((lambda, State { e, k, residual: 0. }));
                            }
                        }
                    }
                }
            }
        }

        let mut previous = None;
        let mut best: Option<(f64, State)> = None;
        // With N=0, positive bending implies negative curvature (and vice
        // versa) for a monotone constitutive law. Retain the same sample grid.
        let (first, last) = if load[0] == 0. && load[1] > 0. {
            (0, 128)
        } else if load[0] == 0. && load[1] < 0. {
            (128, 256)
        } else {
            (0, 256)
        };
        for i in first..=last {
            let k = -limit + 2. * limit * i as f64 / 256.;
            let current = evaluate(k);
            if let Some((f, lambda, e)) = current {
                if f.abs() < 1e-9 * (1. + den)
                    && lambda > 1e-12
                    && best.is_none_or(|b| lambda < b.0)
                {
                    best = Some((lambda, State { e, k, residual: 0. }));
                }
                if let Some((kp, fp)) = previous {
                    if f * fp < 0. {
                        let (mut left, mut right, mut fl) = (kp, k, fp);
                        let mut valid = true;
                        for _ in 0..45 {
                            let mid = (left + right) * 0.5;
                            if let Some((fm, _, _)) = evaluate(mid) {
                                if fl * fm <= 0. {
                                    right = mid;
                                } else {
                                    left = mid;
                                    fl = fm;
                                }
                            } else {
                                valid = false;
                                break;
                            }
                        }
                        let kr = (left + right) * 0.5;
                        if valid {
                            if let Some((fr, lambda, e)) = evaluate(kr) {
                                if lambda > 1e-12
                                    && fr.abs() < 1e-5 * (1. + den)
                                    && best.is_none_or(|b| lambda < b.0)
                                {
                                    best = Some((
                                        lambda,
                                        State {
                                            e,
                                            k: kr,
                                            residual: 0.,
                                        },
                                    ));
                                }
                            }
                        }
                    }
                }
                previous = Some((k, f));
            } else {
                previous = None;
            }
        }
        best
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn section() -> Section {
        Section {
            h: 200.,
            ab: 35.,
            at: 35.,
            rb: 14.5,
            eb: 30000.,
            rt: 1.55,
            rs: 435.,
            rsc: 400.,
        }
    }
    #[test]
    fn pure_bending_fast_boundary_matches_original_scan() {
        let mut seed = 123456789_u64;
        let mut random = || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            (seed >> 32) as f64 / u32::MAX as f64
        };
        for i in 0..400 {
            let h = 150. + random() * 500.;
            let mut s = section();
            s.h = h;
            s.ab = h * (0.05 + 0.35 * random());
            s.at = h * (0.05 + 0.35 * random());
            let areas = [0.5 + random() * 40., 0.5 + random() * 40.];
            let load = [
                if i % 3 == 0 { 1e-16 } else { 0. },
                (5. + random() * 150.) * if i % 2 == 0 { 1. } else { -1. },
            ];
            let a = s.crack_ray_impl(load, areas, true);
            let b = s.crack_ray_impl(load, areas, false);
            assert_eq!(a.is_some(), b.is_some());
            if let (Some(a), Some(b)) = (a, b) {
                assert!(
                    (a.0 - b.0).abs() < 1e-7 * a.0.abs().max(1.),
                    "{} != {}",
                    a.0,
                    b.0
                );
                assert!((a.1.k - b.1.k).abs() < 1e-10);
            }
        }
    }
    #[test]
    fn analytical_tangent_matches_independent_central_differences() {
        let section = section();
        let areas = [12., 8.];
        for (e, k, tension) in [
            (-0.0002, -0.000006, false),
            (-0.00001, -0.000001, true),
            (-0.001, -0.00002, false),
            (-0.0001, 0., false),
        ] {
            let [i0, i1, i2] = section.tangent(e, k, areas, tension);
            let de = 1e-8;
            let dk = 1e-10;
            let ep = section.nm(e + de, k, areas, tension);
            let em = section.nm(e - de, k, areas, tension);
            let kp = section.nm(e, k + dk, areas, tension);
            let km = section.nm(e, k - dk, areas, tension);
            for (actual, expected) in [
                (i0, (ep[0] - em[0]) / (2. * de)),
                (i1, (kp[0] - km[0]) / (2. * dk)),
                (-i1, (ep[1] - em[1]) / (2. * de)),
                (-i2, (kp[1] - km[1]) / (2. * dk)),
            ] {
                assert!(
                    (actual - expected).abs() < 2e-5 * actual.abs().max(1.),
                    "actual={actual} expected={expected} e={e} k={k}"
                );
            }
        }
    }
    #[test]
    fn moderately_reinforced_asymmetric_section_has_crack_boundary() {
        let s = Section {
            ab: 39.855379014116096,
            at: 39.14385719735736,
            rb: 18.5,
            rs: 500.,
            rsc: 500.,
            ..section()
        };
        let areas = [3.2166488747198736, 2.004131759111665];
        let boundary = s.crack_ray([0., 17.798522], areas);
        let (lambda, state) = boundary.unwrap();
        assert!(lambda > 1. && lambda < 1.001);
        assert!(s
            .equilibrium(
                [0., 17.798522],
                areas,
                true,
                Some(State {
                    e: state.e / lambda,
                    k: state.k / lambda,
                    residual: 0.
                })
            )
            .is_some());
    }
    #[test]
    fn forward_and_inverse_equilibrium_preserve_sign_and_material_limits() {
        let s = section();
        let areas = [12., 8.];
        let (e, k) = (-0.0002, -0.000006);
        let q = s.nm(e, k, areas, false);
        assert!(q[1] > 0.);
        let state = s
            .equilibrium([q[0] / 1000., q[1] / 1e6], areas, false, None)
            .unwrap();
        let recovered = s.nm(state.e, state.k, areas, false);
        assert!((recovered[0] - q[0]).abs() < 1e-3);
        assert!((recovered[1] - q[1]).abs() < 1.);
        assert!(state.residual < 1e-8);
    }
    #[test]
    fn pure_bending_crack_boundary_has_equilibrium_and_positive_multiplier() {
        let s = Section {
            rb: 18.5,
            rs: 500.,
            rsc: 500.,
            ..section()
        };
        let load = [0., 50.];
        let (lambda, state) = s.crack_ray(load, [12., 12.]).unwrap();
        let q = s.nm(state.e, state.k, [12., 12.], true);
        assert!(lambda > 0. && lambda < 1.);
        assert!(q[0].abs() < 1e-3);
        assert!((q[1] / 1e6 - lambda * 50.).abs() < 1e-6);
        assert!(s
            .equilibrium(load, [12., 12.], false, Some(state))
            .is_some());
    }
    #[test]
    fn forward_matches_original_python_notebook_exact_integrals() {
        let rows: serde_json::Value =
            serde_json::from_str(include_str!("../../../assets/ndm/ndm_reference.json")).unwrap();
        for row in rows.as_array().unwrap() {
            let v = |i: usize| row["section"][i].as_f64().unwrap();
            let section = Section {
                h: v(0),
                ab: v(1),
                at: v(2),
                rb: v(3),
                eb: v(4),
                rs: v(5),
                rsc: v(6),
                rt: 1.55,
            };
            let nm = section.nm(
                row["e"].as_f64().unwrap(),
                row["k"].as_f64().unwrap(),
                [
                    row["areas"][0].as_f64().unwrap(),
                    row["areas"][1].as_f64().unwrap(),
                ],
                row["tension"].as_bool().unwrap(),
            );
            for (i, key) in ["n", "m"].into_iter().enumerate() {
                let expected = row[key].as_f64().unwrap();
                assert!((nm[i] - expected).abs() < expected.abs().max(1.) * 1e-10);
            }
        }
    }
    #[test]
    fn crack_boundary_and_layer_stress_match_original_notebook_and_mirror() {
        let rows: serde_json::Value =
            serde_json::from_str(include_str!("../../../assets/ndm/crack_reference.json")).unwrap();
        for row in rows.as_array().unwrap() {
            let v = |i: usize| row["row"][i].as_f64().unwrap();
            let section = Section {
                h: v(4),
                ab: v(5),
                at: v(6),
                rb: 18.5,
                eb: v(3),
                rt: 1.55,
                rs: 500.,
                rsc: 500.,
            };
            let (lambda, precrack) = section.crack_ray([v(0), v(1)], [v(9), v(10)]).unwrap();
            assert!((lambda - row["lambda_crc"].as_f64().unwrap()).abs() < 1e-7);
            let state = section
                .equilibrium(
                    [v(0) * lambda, v(1) * lambda],
                    [v(9), v(10)],
                    false,
                    Some(precrack),
                )
                .unwrap();
            for (i, sigma) in section.stresses(state).into_iter().enumerate() {
                assert!((sigma - row["sigma_crc"][i].as_f64().unwrap()).abs() < 1e-3);
            }
        }
        let section = Section {
            rb: 18.5,
            rs: 500.,
            rsc: 500.,
            ..section()
        };
        let positive = section.crack_ray([0., 50.], [12., 12.]).unwrap();
        let negative = section.crack_ray([0., -50.], [12., 12.]).unwrap();
        assert!((positive.0 - negative.0).abs() < 1e-8);
        assert!((positive.1.k + negative.1.k).abs() < 1e-10);
    }
}
