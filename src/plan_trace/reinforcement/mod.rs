//! Pilot plate reinforcement. ML proposes; independent NDM must prove equilibrium.
pub mod ml;
pub mod ndm;
use super::{
    load_combinations::{Combination, Variant},
    loads::LoadCase,
};
use std::collections::BTreeMap;
use std::sync::{
    atomic::{AtomicBool, AtomicUsize, Ordering},
    Arc,
};
#[derive(Clone, Copy, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Method {
    Ml,
    #[default]
    Ndm,
}
#[derive(Default)]
pub struct Progress {
    pub stage: AtomicUsize,
    pub done: [AtomicUsize; 3],
    pub elapsed_ms: [AtomicUsize; 3],
    pub unchanged: AtomicUsize,
    pub crack_phase: Arc<AtomicUsize>,
    pub corrected: AtomicUsize,
    pub stage_stats: [std::sync::Mutex<ndm::Stats>; 3],
}
thread_local! {
    static CRACK_CORRECTING: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    static CRACK_PROGRESS: std::cell::RefCell<Option<Arc<AtomicUsize>>> = const { std::cell::RefCell::new(None) };
}
fn crack_phase(phase: usize) {
    if phase == 3 {
        CRACK_CORRECTING.set(true);
    }
    if phase == 0 {
        CRACK_CORRECTING.set(false);
    }
    if phase != 3 && CRACK_CORRECTING.get() {
        return;
    }
    CRACK_PROGRESS.with(|slot| {
        if let Some(progress) = slot.borrow().as_ref() {
            progress.store(phase, Ordering::Relaxed);
        }
    });
}
pub const STAGES: [&str; 3] = [
    "ML: подбор площадей",
    "НДМ: проверка прочности",
    "НДМ: образование и раскрытие трещин",
];
pub const CONCRETE: [(&str, f64, f64, f64, f64); 10] = [
    ("B15", 8.5, 24000., 11., 1.1),
    ("B20", 11.5, 27500., 15., 1.35),
    ("B25", 14.5, 30000., 18.5, 1.55),
    ("B30", 17., 32500., 22., 1.75),
    ("B35", 19.5, 34500., 25.5, 1.95),
    ("B40", 22., 36000., 29., 2.1),
    ("B45", 25., 37000., 32., 2.25),
    ("B50", 27.5, 38000., 36., 2.45),
    ("B55", 30., 39000., 39.5, 2.6),
    ("B60", 33., 39500., 43., 2.75),
];
pub const STEEL: [(&str, f64, f64, f64); 2] =
    [("A500", 435., 400., 500.), ("A400", 340., 340., 400.)];
#[derive(Clone, Copy, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub struct Settings {
    pub concrete: usize,
    pub steel: usize,
    pub asbx: f64,
    pub asby: f64,
    pub astx: f64,
    pub asty: f64,
    pub diameter_mm: f64,
    pub minimum_percent: f64,
    pub maximum_percent: Option<f64>,
    pub acrcl: f64,
    pub acrcsh: f64,
    pub wall_height: bool,
    pub wall_height_mm: Option<f64>,
    pub wall_length_factor: f64,
    pub wall_length_from_analysis: bool,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            concrete: 2,
            steel: 0,
            asbx: 35.,
            asby: 45.,
            astx: 45.,
            asty: 35.,
            diameter_mm: 12.,
            minimum_percent: 0.1,
            maximum_percent: None,
            acrcl: 0.3,
            acrcsh: 0.4,
            wall_height: false,
            wall_height_mm: None,
            wall_length_factor: 1.,
            wall_length_from_analysis: false,
        }
    }
}
impl Settings {
    pub fn validate_source(self) -> Result<(), String> {
        if self.concrete >= CONCRETE.len()
            || self.steel >= STEEL.len()
            || [self.asbx, self.asby, self.astx, self.asty]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0.)
            || !self.diameter_mm.is_finite()
            || !(6. ..=40.).contains(&self.diameter_mm)
            || !self.acrcl.is_finite()
            || self.acrcl <= 0.
            || !self.acrcsh.is_finite()
            || self.acrcsh <= 0.
            || !self.minimum_percent.is_finite()
            || !(0.01..=10.).contains(&self.minimum_percent)
        {
            return Err("Некорректные параметры армирования".into());
        }
        self.validate_maximum()?;
        if self.wall_height
            && (!self.wall_length_factor.is_finite()
                || self.wall_length_factor <= 0.
                || self
                    .wall_height_mm
                    .is_some_and(|h| !h.is_finite() || h <= 0.))
        {
            return Err("Проверьте высоту стены и коэффициент расчётной длины".into());
        }
        Ok(())
    }
    pub fn validate(self, h: f64) -> Result<(), String> {
        if self.concrete >= CONCRETE.len()
            || self.steel >= STEEL.len()
            || !h.is_finite()
            || h <= 0.
            || [self.asbx, self.asby, self.astx, self.asty]
                .iter()
                .any(|v| !v.is_finite() || *v <= 0. || *v >= h * 0.5)
            || !self.diameter_mm.is_finite()
            || !(6. ..=40.).contains(&self.diameter_mm)
            || !self.acrcl.is_finite()
            || self.acrcl <= 0.
            || !self.acrcsh.is_finite()
            || self.acrcsh <= 0.
            || !self.minimum_percent.is_finite()
            || !(0.01..=10.).contains(&self.minimum_percent)
        {
            return Err(
                "Проверьте классы, привязки (0<a<h/2), диаметр 6–40 мм и минимальный процент"
                    .into(),
            );
        }
        self.validate_maximum()
    }
    fn validate_maximum(self) -> Result<(), String> {
        if self
            .maximum_percent
            .is_some_and(|v| !v.is_finite() || v < self.minimum_percent || v > 100.)
        {
            return Err(
                "Максимальный процент армирования должен быть не ниже минимального и не выше 100 %"
                    .into(),
            );
        }
        Ok(())
    }
    fn maximum_area(self, h: f64) -> f64 {
        self.maximum_percent
            .map_or(200., |percent| ((percent * h * 10. + 1e-9).floor()) / 100.)
    }
    fn section(self, h: f64, ab: f64, at: f64, service: bool) -> ndm::Section {
        let (_, rb, eb, rbser, rt) = CONCRETE[self.concrete];
        let (_, rs, rsc, rsser) = STEEL[self.steel];
        ndm::Section {
            h,
            ab,
            at,
            rb: if service { rbser } else { rb },
            eb,
            rt,
            rs: if service { rsser } else { rs },
            rsc: if service { rsser } else { rsc },
        }
    }
}
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct Forces {
    pub n: [f64; 3],
    pub m: [f64; 3],
}
impl Forces {
    pub fn angle(self, angle: f64) -> [f64; 2] {
        let (c, s) = (angle.cos(), angle.sin());
        [
            self.n[0] * c * c + self.n[1] * s * s + 2. * self.n[2] * c * s,
            self.m[0] * c * c + self.m[1] * s * s + 2. * self.m[2] * c * s,
        ]
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Group {
    pub id: u64,
    pub displays: Vec<usize>,
    pub h: f64,
    pub settings: Settings,
    #[serde(default)]
    pub wall: Option<Wall>,
}
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
pub struct Wall {
    pub surface: usize,
    pub height_m: f64,
    pub effective_m: f64,
}
impl Wall {
    fn moments(
        self,
        group: &Group,
        full: Forces,
        long: Forces,
        tangent: bool,
    ) -> Result<(Forces, Option<Forces>), String> {
        let n = -full.n[1];
        if n <= 1e-8 {
            return Ok((full, None));
        }
        if !self.height_m.is_finite()
            || self.height_m <= 0.
            || !self.effective_m.is_finite()
            || self.effective_m <= 0.
        {
            return Err("Некорректная высота или расчётная длина стены".into());
        }
        let thickness = group.h / 1000.;
        let accidental = (self.height_m / 600.).max(thickness / 30.).max(0.010);
        let moment = full.m[1];
        let eccentricity = (moment.abs() / n).max(accidental);
        let magnitude = n * eccentricity;
        let amplified = |direction: f64| {
            if tangent {
                return Ok(direction * magnitude);
            }
            let cover = if direction > 0. {
                group.settings.asby
            } else {
                group.settings.asty
            };
            let arm = (group.h * 0.5 - cover) / 1000.;
            let long_y = if long.n[1] < 0. {
                let sign = if long.m[1].abs() <= 1e-9 {
                    direction
                } else {
                    long.m[1].signum()
                };
                sign * long.m[1].abs().max(-long.n[1] * accidental)
            } else {
                long.m[1]
            };
            let long_moment = direction * long_y - long.n[1] * arm;
            let phi = 1. + (long_moment / (magnitude + n * arm)).clamp(0., 1.);
            let delta = (eccentricity / thickness).clamp(0.15, 1.5);
            let eb = CONCRETE[group.settings.concrete].2 * 1000.;
            // One metre of wall. Concrete-only D gives a conservative first estimate.
            let d = 0.15 / (phi * (0.3 + delta)) * eb * thickness.powi(3) / 12.;
            let critical = std::f64::consts::PI.powi(2) * d / self.effective_m.powi(2);
            if n >= critical {
                return Err(format!(
                    "Продольный изгиб: сжатие {:.1} кН/м достигло Ncr {:.1} кН/м",
                    n, critical
                ));
            }
            Ok(direction * magnitude / (1. - n / critical))
        };
        let mut adjusted = full;
        adjusted.m[1] = amplified(if moment < 0. { -1. } else { 1. })?;
        let reverse = if moment.abs() <= 1e-9 {
            let mut other = adjusted;
            other.m[1] = amplified(-1.)?;
            Some(other)
        } else {
            None
        };
        Ok((adjusted, reverse))
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Input {
    #[serde(default)]
    pub method: Method,
    pub groups: Vec<Group>,
    pub cases: BTreeMap<u64, Vec<Forces>>,
    pub loads: Vec<LoadCase>,
    pub combinations: Vec<Combination>,
    pub display_count: usize,
    #[serde(default)]
    pub nonlinear_offset: Option<Vec<Forces>>,
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum AreaView {
    #[default]
    Total,
    Strength,
    CrackIncrement,
}
impl AreaView {
    pub fn label(self) -> &'static str {
        match self {
            Self::Total => "Полная",
            Self::Strength => "Прочность",
            Self::CrackIncrement => "Трещин",
        }
    }
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Cell {
    /// As1 lower X, As2 upper X, As3 lower Y, As4 upper Y, cm²/m.
    pub areas: [f64; 4],
    /// Verified ULS reinforcement before any serviceability increase.
    #[serde(default)]
    pub strength_areas: Option<[f64; 4]>,
    pub crack_short: f64,
    pub crack_long: f64,
    pub crack_ratio: f64,
    pub verified: bool,
    pub service_verified: bool,
    pub diagnostic: String,
    pub governing_strength: u64,
    #[serde(default)]
    pub governing_angle: Option<f64>,
    pub governing_crack: u64,
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Output {
    pub cells: Vec<Cell>,
    pub physical_count: usize,
    pub errors: Vec<(u64, String)>,
    #[serde(default)]
    pub wall_displays: BTreeMap<usize, usize>,
}
impl Output {
    pub fn wall_value_for(&self, field: usize, surface: usize, view: AreaView) -> Option<f64> {
        Some(self.value_for(field, *self.wall_displays.get(&surface)?, view))
    }

    pub fn value_for(&self, field: usize, index: usize, view: AreaView) -> f64 {
        if !(26..=29).contains(&field) || view == AreaView::Total {
            return self.value(field, index);
        }
        let Some(cell) = self.cells.get(index).filter(|c| c.verified) else {
            return f64::NAN;
        };
        let Some(strength) = cell.strength_areas else {
            return f64::NAN;
        };
        match view {
            AreaView::Total => cell.areas[field - 26],
            AreaView::Strength => strength[field - 26],
            AreaView::CrackIncrement => (cell.areas[field - 26] - strength[field - 26]).max(0.),
        }
    }
    pub fn value(&self, field: usize, index: usize) -> f64 {
        let Some(cell) = self.cells.get(index) else {
            return f64::NAN;
        };
        match field {
            26..=29 if cell.verified => cell.areas[field - 26],
            30 if cell.service_verified => cell.crack_short,
            31 if cell.service_verified => cell.crack_long,
            32 if cell.service_verified => cell.crack_ratio,
            _ => f64::NAN,
        }
    }
}
/// Two-variable LP: min Ax+Ay, c² Ax+s² Ay >= requirement, including minima.
fn minimum_pair(constraints: &[[f64; 3]], minimum: f64, maximum: f64) -> Option<[f64; 2]> {
    let mut constraints = constraints.to_vec();
    constraints.extend([
        [1., 0., minimum],
        [0., 1., minimum],
        [-1., 0., -maximum],
        [0., -1., -maximum],
    ]);
    let mut best: Option<[f64; 2]> = None;
    for a in 0..constraints.len() {
        for b in a + 1..constraints.len() {
            let [x, y, v] = constraints[a];
            let [u, w, r] = constraints[b];
            let det = x * w - y * u;
            if det.abs() < 1e-12 {
                continue;
            }
            let pair = [(v * w - y * r) / det, (x * r - v * u) / det];
            if pair
                .iter()
                .any(|v| !v.is_finite() || *v < minimum - 1e-8 || *v > maximum + 1e-8)
            {
                continue;
            }
            if constraints
                .iter()
                .any(|c| c[0] * pair[0] + c[1] * pair[1] < c[2] - 1e-7)
            {
                continue;
            }
            if best.is_none_or(|p| pair[0] + pair[1] < p[0] + p[1]) {
                best = Some(pair);
            }
        }
    }
    // On a flat optimum choose the most balanced feasible pair, not an arbitrary
    // endpoint of the optimum segment (important for symmetric twisting fields).
    if let Some(pair) = best {
        let sum = pair[0] + pair[1];
        let (mut lo, mut hi) = (minimum, sum - minimum);
        for &[a, b, r] in &constraints {
            let d = a - b;
            if d > 1e-12 {
                lo = lo.max((r - b * sum) / d);
            } else if d < -1e-12 {
                hi = hi.min((r - b * sum) / d);
            }
        }
        if lo <= hi + 1e-8 {
            let x = (sum * 0.5).max(lo).min(hi);
            return Some([x, sum - x]);
        }
    }
    best
}
fn directional(areas: [f64; 4], settings: Settings, angle: f64) -> ([f64; 2], [f64; 2]) {
    let c = angle.cos().powi(2);
    let s = angle.sin().powi(2);
    let pair = [areas[0] * c + areas[2] * s, areas[1] * c + areas[3] * s];
    let cover = [
        (areas[0] * c * settings.asbx + areas[2] * s * settings.asby) / pair[0],
        (areas[1] * c * settings.astx + areas[3] * s * settings.asty) / pair[1],
    ];
    (pair, cover)
}
struct Models {
    strength: Option<ml::Network>,
    crack: Option<ml::Network>,
    method: Method,
}
impl Models {
    fn load() -> Result<Self, String> {
        Self::for_method(Method::Ml)
    }
    fn for_method(method: Method) -> Result<Self, String> {
        Ok(Self {
            method,
            strength: if method == Method::Ml {
                Some(ml::Network::load(false)?)
            } else {
                None
            },
            crack: if method == Method::Ml {
                Some(ml::Network::load(true)?)
            } else {
                None
            },
        })
    }
}
struct Combined {
    id: u64,
    design: Vec<Forces>,
    design_long: Vec<Forces>,
    norm: Vec<Forces>,
    long: Vec<Forces>,
    reverse: BTreeMap<usize, [Option<Forces>; 2]>,
    wall_errors: BTreeMap<usize, String>,
}
impl Combined {
    fn strength(&self, i: usize) -> impl Iterator<Item = (Forces, f64)> + '_ {
        let reverse = self.reverse.get(&i).copied().unwrap_or_default();
        [
            (Some(self.design[i]), 1.),
            (reverse[0], 1.),
            (Some(self.design_long[i]), 0.9),
            (reverse[1], 0.9),
        ]
        .into_iter()
        .filter_map(|(value, rb)| value.map(|forces| (forces, rb)))
    }
}
fn combinations(input: &Input) -> Result<Vec<Combined>, String> {
    if input.combinations.is_empty() {
        return Err("Добавьте хотя бы одно сочетание".into());
    }
    let mut out = vec![];
    for combination in &input.combinations {
        let build = |variant| -> Result<Vec<Forces>, String> {
            let mut output = vec![Forces::default(); input.display_count];
            let weights = combination.weights(&input.loads, variant)?;
            let total = weights.iter().map(|(_, weight)| weight).sum::<f64>();
            let mut components = Vec::with_capacity(weights.len() + 1);
            for (case, weight) in weights.into_iter().filter(|(_, w)| *w != 0.) {
                components.push((
                    input
                        .cases
                        .get(&case)
                        .ok_or("Не все загружения рассчитаны")?,
                    weight,
                ));
            }
            if let Some(offset) = &input.nonlinear_offset {
                components.push((offset, 1. - total));
            }
            for (values, weight) in components {
                if values.len() != output.len() {
                    return Err("Результаты относятся к разным сеткам".into());
                }
                for (target, source) in output.iter_mut().zip(values) {
                    for i in 0..3 {
                        target.n[i] += weight * source.n[i];
                        target.m[i] += weight * source.m[i];
                    }
                }
            }
            Ok(output)
        };
        let mut combined = Combined {
            id: combination.id,
            design: build(Variant::Design)?,
            design_long: build(Variant::DesignLong)?,
            norm: build(Variant::Normative)?,
            long: build(Variant::NormativeLong)?,
            reverse: Default::default(),
            wall_errors: Default::default(),
        };
        for group in input
            .groups
            .iter()
            .filter(|group| group.settings.wall_height)
        {
            let Some(wall) = group.wall else {
                continue;
            };
            let i = group.displays[0];
            let correct = || {
                group.settings.validate_source()?;
                group.settings.validate(group.h)?;
                let full = combined.design[i];
                let long = combined.design_long[i];
                let tangent = input.nonlinear_offset.is_some();
                Ok::<_, String>((
                    wall.moments(group, full, long, tangent)?,
                    wall.moments(group, long, long, tangent)?,
                ))
            };
            match correct() {
                Ok(((full, full_reverse), (long, long_reverse))) => {
                    combined.design[i] = full;
                    combined.design_long[i] = long;
                    if full_reverse.is_some() || long_reverse.is_some() {
                        combined.reverse.insert(i, [full_reverse, long_reverse]);
                    }
                }
                Err(error) => {
                    combined.wall_errors.insert(i, error);
                }
            }
        }
        out.push(combined);
    }
    Ok(out)
}
fn raw(load: [f64; 2], section: ndm::Section) -> [f64; 9] {
    [
        load[0],
        load[1],
        section.rb,
        section.eb,
        section.h,
        section.ab,
        section.at,
        section.rs,
        section.rsc,
    ]
}
fn in_domain(s: ndm::Section) -> bool {
    (140. ..=1000.).contains(&s.h) && [s.ab, s.at].iter().all(|a| *a >= 15. && *a <= 0.45 * s.h)
}
#[cfg(test)]
thread_local! {static BENCH_ML_ALLOWED:std::cell::Cell<bool>=const{std::cell::Cell::new(true)};static BENCH_NDM_BISECT:std::cell::Cell<bool>=const{std::cell::Cell::new(false)};}
#[cfg(any(test, feature = "inference-bench"))]
thread_local! {static BENCH_BATCH:std::cell::Cell<bool>=const{std::cell::Cell::new(true)};}
fn batch_allowed() -> bool {
    #[cfg(any(test, feature = "inference-bench"))]
    {
        BENCH_BATCH.with(|v| v.get())
    }
    #[cfg(not(any(test, feature = "inference-bench")))]
    {
        true
    }
}
// Adapt angular sampling to the curvature of the transformed force tensors.
// A shared deterministic set keeps ML batches and NDM verification aligned.
fn adaptive_angles(group: &Group, combined: &[Combined]) -> Vec<f64> {
    let i = group.displays[0];
    let mut tensors: Vec<[f64; 3]> = combined
        .iter()
        .flat_map(|c| {
            [&c.design, &c.design_long, &c.norm, &c.long]
                .into_iter()
                .flat_map(move |v| [v[i].n, v[i].m])
        })
        .collect();
    for reversed in combined
        .iter()
        .filter_map(|c| c.reverse.get(&i))
        .flatten()
        .flatten()
    {
        tensors.push(reversed.m);
    }
    fn split(a: f64, b: f64, tensors: &[[f64; 3]], out: &mut Vec<f64>) {
        let m = (a + b) * 0.5;
        let refine = tensors.iter().any(|t| {
            let scale = t.iter().fold(0.0_f64, |x, y| x.max(y.abs()));
            let value = |x: f64| {
                let (s, c) = x.sin_cos();
                t[0] * c * c + t[1] * s * s + 2.0 * t[2] * s * c
            };
            scale > 1e-12 && (value(m) - (value(a) + value(b)) * 0.5).abs() > scale * 0.01
        });
        if refine && b - a > 1.25_f64.to_radians() {
            split(a, m, tensors, out);
            split(m, b, tensors, out);
        } else {
            out.push(a);
        }
    }
    let mut out = vec![];
    for j in 0..8 {
        split(
            (j as f64 * 22.5).to_radians(),
            ((j + 1) as f64 * 22.5).to_radians(),
            &tensors,
            &mut out,
        );
    }
    out
}
fn batch_proposals(
    models: &Models,
    group: &Group,
    combined: &[Combined],
    minimum: f64,
) -> Vec<[f64; 2]> {
    let mut result = vec![];
    let mut indices = vec![];
    let mut rows = vec![];
    let demands: Vec<_> = combined
        .iter()
        .flat_map(|c| c.strength(group.displays[0]))
        .collect();
    for angle in adaptive_angles(group, combined) {
        for &(forces, rb) in &demands {
            let load = forces.angle(angle);
            let mut sec = group.settings.section(
                group.h,
                group.settings.asbx.max(group.settings.asby),
                group.settings.astx.max(group.settings.asty),
                false,
            );
            sec.rb *= rb;
            if in_domain(sec)
                && models.strength.is_some()
                && ml_allowed()
                && load.iter().any(|v| v.abs() >= 1e-10)
            {
                indices.push(result.len());
                rows.push(raw(load, sec));
                result.push([minimum; 2]);
            } else {
                result.push(proposal(models, load, sec, minimum));
            }
        }
    }
    if let Some(network) = &models.strength {
        for (i, value) in indices.into_iter().zip(network.strength_batch(&rows)) {
            result[i] = value.map(|v| v.max(minimum));
        }
    }
    result
}
#[inline]
fn ml_allowed() -> bool {
    #[cfg(test)]
    {
        BENCH_ML_ALLOWED.with(|v| v.get())
    }
    #[cfg(not(test))]
    {
        true
    }
}
fn proposal(models: &Models, load: [f64; 2], section: ndm::Section, minimum: f64) -> [f64; 2] {
    if load.iter().all(|v| v.abs() < 1e-10) {
        return [minimum; 2];
    }
    if in_domain(section) && ml_allowed() && models.strength.is_some() {
        return models
            .strength
            .as_ref()
            .unwrap()
            .strength(raw(load, section))
            .map(|v| v.max(minimum));
    }
    // Outside the ML training domain use mechanics only, never extrapolate the network.
    let lever = (section.h - section.ab.max(section.at)) * 0.75;
    let tension = load[0].max(0.) * 1000. / (section.rs * 100.);
    let bending = load[1].abs() * 1e6 / (lever * section.rs * 100.);
    if load[1] >= 0. {
        [minimum + bending + tension, minimum + tension]
    } else {
        [minimum + tension, minimum + bending + tension]
    }
}
#[derive(Clone, Copy)]
struct CrackPrediction {
    first: f64,
    ultimate: f64,
    event: usize,
    confidence: f64,
    crc: [f64; 2],
    full: [f64; 2],
}
impl CrackPrediction {
    fn decode(network: &ml::Network, p: &[f32], rsser: f64) -> Option<Self> {
        if p.len() != 9 || p.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let limits: [f64; 2] = std::array::from_fn(|i| {
            ((p[i] * network.ystd[i] + network.ymean[i]).clamp(-15., 15.)).exp() as f64
        });
        let event = (0..3).max_by(|&a, &b| p[a + 6].total_cmp(&p[b + 6]))?;
        let confidence = 1.
            / (0..3)
                .map(|i| ((p[i + 6] - p[event + 6]) as f64).exp())
                .sum::<f64>();
        Some(Self {
            first: if event == 1 {
                limits[0].min(limits[1])
            } else {
                limits[1]
            },
            ultimate: limits[1],
            event,
            confidence,
            crc: [(p[2] as f64).tanh() * rsser, (p[3] as f64).tanh() * rsser],
            full: [(p[4] as f64).tanh() * rsser, (p[5] as f64).tanh() * rsser],
        })
    }
    fn strain(self, h: f64, cover: [f64; 2], crc: bool) -> ndm::State {
        let sigma = if crc { self.crc } else { self.full };
        let zb = -h * 0.5 + cover[0];
        let zt = h * 0.5 - cover[1];
        let k = (sigma[1] - sigma[0]) / (200000. * (zt - zb));
        ndm::State {
            e: sigma[0] / 200000. - k * zb,
            k,
            residual: f64::INFINITY,
        }
    }
}
fn crack_features_for(
    settings: Settings,
    h: f64,
    areas: [f64; 2],
    cover: [f64; 2],
    load: [f64; 2],
) -> Option<[f32; 22]> {
    let section = settings.section(h, cover[0], cover[1], false);
    if !in_domain(section) || h < 150. || areas.iter().any(|a| *a < 0.5 || *a > 200.) {
        return None;
    }
    let raw = raw(load, section);
    let row = std::array::from_fn(|i| if i < 9 { raw[i] } else { areas[i - 9] });
    let (_, _, _, rb, rt) = CONCRETE[settings.concrete];
    Some(ml::crack_features(row, rb, rt, STEEL[settings.steel].3))
}
#[derive(Default)]
struct CrackCache {
    rays: BTreeMap<[u64; 6], (Option<(f64, ndm::State)>, Option<ndm::State>)>,
    predictions: BTreeMap<[u64; 6], Option<CrackPrediction>>,
}
fn crack_key(areas: [f64; 2], cover: [f64; 2], load: [f64; 2]) -> [u64; 6] {
    [
        areas[0].to_bits(),
        areas[1].to_bits(),
        cover[0].to_bits(),
        cover[1].to_bits(),
        load[0].to_bits(),
        load[1].to_bits(),
    ]
}
impl CrackCache {
    fn new() -> Self {
        Self::default()
    }
    fn prepare(&mut self, models: &Models, group: &Group, combined: &[Combined], as4: [f64; 4]) {
        let Some(network) = models.crack.as_ref().filter(|_| ml_allowed()) else {
            return;
        };
        let mut keys = vec![];
        let mut rows = vec![];
        for angle in adaptive_angles(group, combined) {
            let (areas, cover) = directional(as4, group.settings, angle);
            for c in combined {
                for forces in [&c.norm, &c.long] {
                    let load = forces[group.displays[0]].angle(angle);
                    let key = crack_key(areas, cover, load);
                    if self.predictions.contains_key(&key) {
                        continue;
                    }
                    self.predictions.insert(key, None);
                    if load.iter().all(|v| v.abs() < 1e-10) {
                        continue;
                    }
                    if let Some(f) = crack_features_for(group.settings, group.h, areas, cover, load)
                    {
                        keys.push(key);
                        rows.push(f);
                    }
                }
            }
        }
        let out = network.forward_batch(&rows);
        for (r, key) in keys.into_iter().enumerate() {
            let p = std::array::from_fn::<_, 9, _>(|c| out[(r, c)]);
            self.predictions.insert(
                key,
                CrackPrediction::decode(network, &p, STEEL[group.settings.steel].3),
            );
        }
    }
    fn prediction(
        &mut self,
        models: &Models,
        settings: Settings,
        h: f64,
        areas: [f64; 2],
        cover: [f64; 2],
        load: [f64; 2],
    ) -> Option<CrackPrediction> {
        let network = models.crack.as_ref().filter(|_| ml_allowed())?;
        let key = crack_key(areas, cover, load);
        *self.predictions.entry(key).or_insert_with(|| {
            let features = crack_features_for(settings, h, areas, cover, load)?;
            CrackPrediction::decode(
                network,
                &network.forward(&features),
                STEEL[settings.steel].3,
            )
        })
    }
}
fn widths_from_stress(
    settings: Settings,
    h: f64,
    areas: [f64; 2],
    cover: [f64; 2],
    load: [f64; 2],
    precrack: ndm::State,
    sigma_crc: [f64; 2],
    sigma: [f64; 2],
) -> [f64; 2] {
    let mut width = [0.; 2];
    for layer in 0..2 {
        if sigma[layer] <= 0. {
            continue;
        }
        let neutral = if precrack.k.abs() > 1e-14 {
            -precrack.e / precrack.k
        } else {
            0.
        };
        let xt = if precrack.k.abs() < 1e-14 {
            h
        } else if layer == 0 {
            neutral + h * 0.5
        } else {
            h * 0.5 - neutral
        };
        let tension_height = xt.max(2. * cover[layer]).min(0.5 * h).max(0.);
        let diameter = settings.diameter_mm;
        let spacing = (0.5 * 1000. * tension_height / (100. * areas[layer]) * diameter)
            .clamp((10. * diameter).max(100.), (40. * diameter).min(400.));
        let psi = (1. - 0.8 * sigma_crc[layer].max(0.) / sigma[layer]).clamp(0.2, 1.);
        let phi3 = if load[0] > 0. { 1.2 } else { 1. };
        width[layer] = 0.5 * phi3 * psi * sigma[layer] / 200000. * spacing;
    }
    width
}
fn ml_crack_width(
    settings: Settings,
    h: f64,
    areas: [f64; 2],
    cover: [f64; 2],
    load: [f64; 2],
    p: CrackPrediction,
) -> Option<([f64; 2], f64)> {
    // Predictions close to formation, failure or a width limit are checked by NDM.
    if p.confidence < 0.9 || p.ultimate < 1.2 {
        return None;
    }
    if p.first > 1.2 {
        return Some(([0.; 2], if p.event == 0 { 0. } else { 1. / p.first }));
    }
    if p.event == 0
        || p.first > 0.85
        || p.full
            .iter()
            .any(|s| s.abs() > 0.95 * STEEL[settings.steel].3)
    {
        return None;
    }
    let section = settings.section(h, cover[0], cover[1], true);
    crack_phase(2);
    let precrack = section.equilibrium(
        load.map(|v| v * p.first),
        areas,
        true,
        Some(p.strain(h, cover, true)),
    )?;
    let w = widths_from_stress(settings, h, areas, cover, load, precrack, p.crc, p.full);
    let margin = 0.8 * settings.acrcsh.min(settings.acrcl) / 1.4;
    w.iter()
        .all(|v| v.is_finite() && *v < margin)
        .then_some((w, 1. / p.first))
}
fn crack_widths(
    models: &Models,
    settings: Settings,
    h: f64,
    areas: [f64; 2],
    cover: [f64; 2],
    load: [f64; 2],
    cache: &mut CrackCache,
) -> Result<([f64; 2], f64), String> {
    crack_phase(1);
    if load.iter().all(|v| v.abs() < 1e-10) {
        return Ok(([0.; 2], 0.));
    }
    let prediction = cache.prediction(models, settings, h, areas, cover, load);
    if let Some(p) = prediction {
        if let Some(result) = ml_crack_width(settings, h, areas, cover, load, p) {
            return Ok(result);
        }
    }
    let section = settings.section(h, cover[0], cover[1], true);
    let (canonical, scale) = if load[0] == 0. {
        let m = load[1].signum() * h / 1000.;
        ([0., m], (load[1] / m).abs())
    } else if load[1] == 0. {
        ([load[0].signum(), 0.], load[0].abs())
    } else {
        let r = load[0].abs().max((1000. * load[1] / h).abs());
        (load.map(|v| v / r), r)
    };
    let key = [
        areas[0].to_bits(),
        areas[1].to_bits(),
        cover[0].to_bits(),
        cover[1].to_bits(),
        canonical[0].to_bits(),
        canonical[1].to_bits(),
    ];
    let entry = cache
        .rays
        .entry(key)
        .or_insert_with(|| (section.crack_ray(canonical, areas), None));
    let boundary = entry.0.map(|(lambda, state)| (lambda / scale, state));
    let Some((lambda, precrack)) = boundary.filter(|b| b.0 <= 1. + 1e-8) else {
        let seed = boundary.map(|(lambda, state)| ndm::State {
            e: state.e / lambda,
            k: state.k / lambda,
            residual: 0.,
        });
        if section.equilibrium(load, areas, true, seed).is_some() {
            return Ok(([0.; 2], boundary.map_or(0., |b| 1. / b.0)));
        }
        return Err(format!("НДМ: нормативное сечение не проверено до образования трещин: N={:.6}, M={:.6}, As={areas:?}, a={cover:?}, h={h}", load[0], load[1]));
    };
    let at_crack = if let Some(state) = entry.1 {
        state
    } else {
        let unit_lambda = entry.0.unwrap().0;
        let state = section
            .equilibrium(
                canonical.map(|v| v * unit_lambda),
                areas,
                false,
                Some(precrack),
            )
            .ok_or("НДМ: не найдены напряжения при образовании трещин")?;
        entry.1 = Some(state);
        state
    };
    let seed = prediction
        .map(|p| p.strain(h, cover, false))
        .or(Some(at_crack));
    crack_phase(2);
    let full = section
        .equilibrium(load, areas, false, seed)
        .ok_or("НДМ: не найдено равновесие при нормативной нагрузке")?;
    let sigma_crc = section.stresses(at_crack);
    let sigma = section.stresses(full);
    Ok((
        widths_from_stress(settings, h, areas, cover, load, precrack, sigma_crc, sigma),
        1. / lambda,
    ))
}
fn design_cell(
    models: &Models,
    group: &Group,
    combined: &[Combined],
    cancel: &AtomicBool,
    stage: usize,
    mut cell: Cell,
) -> Cell {
    if stage == 2 && cell.verified && cell.strength_areas.is_none() {
        cell.strength_areas = Some(cell.areas);
    }
    if stage == 0 {
        #[cfg(test)]
        if BENCH_NDM_BISECT.with(|v| v.get()) {
            return ndm_size(models, group, combined, cancel, cell);
        }
        if models.method == Method::Ndm {
            return ndm_size(models, group, combined, cancel, cell);
        }
    }
    design_cell_core(models, group, combined, cancel, stage, cell)
}
fn design_cell_core(
    models: &Models,
    group: &Group,
    combined: &[Combined],
    cancel: &AtomicBool,
    stage: usize,
    mut cell: Cell,
) -> Cell {
    let run = || -> Result<Cell, String> {
        group.settings.validate(group.h)?;
        let settings = group.settings;
        let minimum = ((settings.minimum_percent * group.h / 10.).max(0.5) * 100.).ceil() / 100.;
        let maximum = settings.maximum_area(group.h);
        if minimum > maximum {
            return Err(format!(
                "Минимальная площадь {minimum:.2} см²/м превышает предел {maximum:.2} см²/м"
            ));
        }
        let mut constraints = [vec![], vec![]];
        let mut demands = Vec::new();
        if stage < 2 {
            let batch = if stage == 0 && batch_allowed() {
                Some(batch_proposals(models, group, combined, minimum))
            } else {
                None
            };
            let actions: Vec<_> = combined
                .iter()
                .flat_map(|c| {
                    c.strength(group.displays[0])
                        .map(move |(f, rb)| (c.id, f, rb))
                })
                .collect();
            let mut prediction_index = 0;
            for angle in adaptive_angles(group, combined) {
                let c = angle.cos().powi(2);
                let s = angle.sin().powi(2);
                let mut required = [minimum; 2];
                for &(id, forces, rb_factor) in &actions {
                    if cancel.load(Ordering::Relaxed) {
                        return Err("Отменено".into());
                    }
                    let load = forces.angle(angle);
                    if stage == 0 {
                        // Conservative covers make the ML proposal independent of unknown Ax/Ay.
                        let mut section = settings.section(
                            group.h,
                            settings.asbx.max(settings.asby),
                            settings.astx.max(settings.asty),
                            false,
                        );
                        section.rb *= rb_factor;
                        let predicted = batch.as_ref().map_or_else(
                            || proposal(models, load, section, minimum),
                            |b| b[prediction_index],
                        );
                        prediction_index += 1;
                        for layer in 0..2 {
                            required[layer] = required[layer].max(predicted[layer]);
                        }
                    }
                    demands.push((angle, load, id, rb_factor));
                }
                for layer in 0..2 {
                    constraints[layer].push([c, s, required[layer]]);
                }
            }
            let mut out = cell.clone();
            if stage == 0 {
                let solve_layer = |layer: usize| {
                    minimum_pair(&constraints[layer], minimum, maximum)
                        // The proposal estimates the demand. Check the configured
                        // limit in the section solver when the estimate exceeds it.
                        .or_else(|| settings.maximum_percent.map(|_| [maximum; 2]))
                };
                let bottom = solve_layer(0).ok_or_else(|| {
                    format!("Нет допустимого нижнего армирования до {maximum} см²/м")
                })?;
                let top = solve_layer(1).ok_or_else(|| {
                    format!("Нет допустимого верхнего армирования до {maximum} см²/м")
                })?;
                out.governing_angle = constraints
                    .iter()
                    .zip([bottom, top])
                    .flat_map(|(rows, provided)| {
                        rows.iter()
                            .zip(adaptive_angles(group, combined))
                            .map(move |(r, angle)| {
                                (r[2] / (r[0] * provided[0] + r[1] * provided[1]), angle)
                            })
                    })
                    .max_by(|a, b| a.0.total_cmp(&b.0))
                    .map(|(_, angle)| angle);
                out.areas = [bottom[0], top[0], bottom[1], top[1]]
                    .map(|a| ((a * 100.).ceil() / 100.).min(maximum));
                return Ok(out);
            }
            let demands_per_angle = actions.len();
            let mut verified_states = vec![None; demands.len()];
            // All combinations and all sampled directions must pass the independent NDM.
            for attempt in 0..18 {
                let mut failed = None;
                for (demand_index, &(angle, load, combination, rb_factor)) in
                    demands.iter().enumerate()
                {
                    if cancel.load(Ordering::Relaxed) {
                        return Err("Отменено".into());
                    }
                    let (areas, cover) = directional(out.areas, settings, angle);
                    let mut section = settings.section(group.h, cover[0], cover[1], false);
                    section.rb *= rb_factor;
                    let neighbor = demand_index
                        .checked_sub(demands_per_angle)
                        .and_then(|i| verified_states[i]);
                    let seed = verified_states[demand_index].or(neighbor);
                    if let Some(state) = section.equilibrium(load, areas, false, seed) {
                        verified_states[demand_index] = Some(state);
                    } else {
                        failed = Some((combination, angle));
                        break;
                    }
                }
                if let Some((combination, angle)) = failed {
                    out.governing_angle = Some(angle);
                    out.governing_strength = combination;
                    let next = out
                        .areas
                        .map(|a| ((a * 1.15 * 100.).ceil() / 100.).min(maximum));
                    if next == out.areas {
                        return Err(format!("НДМ: прочность не подтверждена в сочетании {combination} при пределе {maximum} см²/м на слой и направление"));
                    }
                    if attempt == 17 {
                        return Err(format!("НДМ: прочность не подтверждена в сочетании {combination} за 18 проверок подбора"));
                    }
                    out.areas = next;
                } else {
                    out.verified = true;
                    break;
                }
            }
            // Every trial was rounded upward BEFORE verification. Therefore
            // the accepted state already checks the exact published areas.
            return Ok(out);
        }
        let mut out = cell.clone();
        for _ in 0..24 {
            crack_phase(0);
            if cancel.load(Ordering::Relaxed) {
                return Err("Отменено".into());
            }
            let mut cache = CrackCache::new();
            cache.prepare(models, group, combined, out.areas);
            out.crack_short = 0.;
            out.crack_long = 0.;
            out.crack_ratio = 0.;
            let mut worst = None;
            let mut worst_ratio = 1. + 1e-8;
            for (ci, combination) in combined.iter().enumerate() {
                for angle in adaptive_angles(group, combined) {
                    if cancel.load(Ordering::Relaxed) {
                        return Err("Отменено".into());
                    }
                    let (areas, cover) = directional(out.areas, settings, angle);
                    let full_load = combination.norm[group.displays[0]].angle(angle);
                    let long_load = combination.long[group.displays[0]].angle(angle);
                    let full = crack_widths(
                        models, settings, group.h, areas, cover, full_load, &mut cache,
                    )?;
                    let long = if full_load == long_load {
                        full
                    } else {
                        crack_widths(
                            models, settings, group.h, areas, cover, long_load, &mut cache,
                        )?
                    };
                    for layer in 0..2 {
                        let short = full.0[layer] + 0.4 * long.0[layer];
                        let duration = 1.4 * long.0[layer];
                        let ratio = (short / settings.acrcsh).max(duration / settings.acrcl);
                        if ratio > worst_ratio {
                            worst_ratio = ratio;
                            worst = Some((ci, angle, layer));
                        }
                        if short > out.crack_short || duration > out.crack_long {
                            out.governing_crack = combination.id;
                        }
                        out.crack_short = out.crack_short.max(short);
                        out.crack_long = out.crack_long.max(duration);
                    }
                    out.crack_ratio = out.crack_ratio.max(full.1).max(long.1);
                }
            }
            let Some((ci, angle, layer)) = worst else {
                out.service_verified = true;
                return Ok(out);
            };
            crack_phase(3);
            out.governing_angle = Some(angle);
            let combination = &combined[ci];
            let base = out.areas;
            let c = angle.cos().powi(2);
            let s = angle.sin().powi(2);
            let active = [(layer, c > 1e-9), (layer + 2, s > 1e-9)];
            let cap = active
                .iter()
                .filter(|(_, used)| *used)
                .map(|(i, _)| maximum / base[*i])
                .fold(f64::INFINITY, f64::min);
            let candidate = |factor: f64| {
                let mut a = base;
                for &(i, used) in &active {
                    if used {
                        a[i] = ((base[i] * factor * 100.).ceil() / 100.).min(maximum);
                    }
                }
                a
            };
            let meets = |a: [f64; 4]| -> Result<bool, String> {
                if cancel.load(Ordering::Relaxed) {
                    return Err("Отменено".into());
                }
                let (areas, cover) = directional(a, settings, angle);
                let mut cache = CrackCache::new();
                let f = crack_widths(
                    models,
                    settings,
                    group.h,
                    areas,
                    cover,
                    combination.norm[group.displays[0]].angle(angle),
                    &mut cache,
                )?;
                let l = crack_widths(
                    models,
                    settings,
                    group.h,
                    areas,
                    cover,
                    combination.long[group.displays[0]].angle(angle),
                    &mut cache,
                )?;
                Ok(
                    f.0[layer] + 0.4 * l.0[layer] <= settings.acrcsh * (1. + 1e-9)
                        && 1.4 * l.0[layer] <= settings.acrcl * (1. + 1e-9),
                )
            };
            let mut lo = 1.;
            let mut hi = 2_f64.min(cap);
            while !meets(candidate(hi))? {
                if hi >= cap - 1e-12 {
                    return Err(format!(
                        "Не обеспечено раскрытие трещин при пределе {maximum} см²/м на слой и направление, сочетание {}",
                        combination.id
                    ));
                }
                lo = hi;
                hi = (hi * 2.).min(cap);
            }
            for _ in 0..16 {
                if active
                    .iter()
                    .filter(|(_, b)| *b)
                    .all(|(i, _)| base[*i] * (hi - lo) <= 0.005)
                {
                    break;
                }
                let mid = (lo + hi) * 0.5;
                if meets(candidate(mid))? {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            out.areas = candidate(hi);
            // Increased reinforcement is independently rechecked for all ULS directions.
            let checked = design_cell(models, group, combined, cancel, 1, out.clone());
            if !checked.diagnostic.is_empty() || !checked.verified {
                return Err(format!("После подбора по трещинам: {}", checked.diagnostic));
            }
            out = checked;
        }
        Err("Не удалось обеспечить все ограничения раскрытия трещин за 24 коррекции".into())
    };
    match run() {
        Ok(out) => out,
        Err(error) => {
            cell.diagnostic = error;
            cell
        }
    }
}
/// Resumable at physical-element boundaries; native and browser use the same steps.
struct DesignJob {
    input: Input,
    models: Models,
    combined: Vec<Combined>,
    cells: Vec<Cell>,
    progress: Arc<Progress>,
    cancel: Arc<AtomicBool>,
    stage: usize,
    index: usize,
    started: web_time::Instant,
}
impl DesignJob {
    fn new(input: Input, progress: Arc<Progress>, cancel: Arc<AtomicBool>) -> Result<Self, String> {
        let models = Models::for_method(input.method)?;
        let combined = combinations(&input)?;
        let cells = vec![
            Cell {
                areas: [0.; 4],
                strength_areas: None,
                crack_short: 0.,
                crack_long: 0.,
                crack_ratio: 0.,
                verified: false,
                service_verified: false,
                diagnostic: String::new(),
                governing_strength: 0,
                governing_angle: None,
                governing_crack: 0
            };
            input.display_count
        ];
        ndm::reset_stats();
        Ok(Self {
            input,
            models,
            combined,
            cells,
            progress,
            cancel,
            stage: 0,
            index: 0,
            started: web_time::Instant::now(),
        })
    }
    /// Returns false only once every stage has completed. No FE is recomputed
    /// merely because the browser yielded between steps.
    fn step(&mut self) -> Result<bool, String> {
        if self.cancel.load(Ordering::Relaxed) {
            return Err("Подбор отменён".into());
        }
        if self.stage == 3 {
            return Ok(false);
        }
        // Direct NDM sizing already establishes ULS feasibility. Skip the entire
        // bookkeeping pass: no cell copies, thread creation or repeated checks.
        if self.stage == 1 && self.input.method == Method::Ndm {
            self.progress.done[1].store(self.input.groups.len(), Ordering::Relaxed);
            self.progress.elapsed_ms[1].store(0, Ordering::Relaxed);
            self.stage = 2;
            self.index = 0;
            self.started = web_time::Instant::now();
            self.progress.stage.store(2, Ordering::Relaxed);
            return Ok(true);
        }
        let stage = self.stage;
        #[cfg(not(target_arch = "wasm32"))]
        let workers = std::thread::available_parallelism()
            .map_or(1, |n| n.get())
            .min(8);
        #[cfg(target_arch = "wasm32")]
        let workers = 1;
        let end = (self.index + workers).min(self.input.groups.len());
        let results = std::thread::scope(|scope| {
            let mut handles = vec![];
            let mut immediate = vec![];
            for index in self.index..end {
                let group = &self.input.groups[index];
                let previous = self.cells[group.displays[0]].clone();
                let wall_models = Models {
                    method: Method::Ndm,
                    strength: None,
                    crack: None,
                };
                let method = if group.wall.is_some() {
                    Method::Ndm
                } else {
                    self.input.method
                };
                let models = if group.wall.is_some() {
                    None
                } else {
                    Some(&self.models)
                };
                let combined = &self.combined;
                let cancel = &self.cancel;
                let progress = self.progress.crack_phase.clone();
                let work = move || {
                    ndm::reset_stats();
                    if stage == 2 {
                        CRACK_PROGRESS.with(|slot| *slot.borrow_mut() = Some(progress));
                    }
                    let previous = previous;
                    let proposed = previous.areas;
                    let models = models.unwrap_or(&wall_models);
                    let diagnostic = combined.iter().find_map(|c| {
                        c.wall_errors
                            .get(&group.displays[0])
                            .map(|e| format!("Сочетание {} · {e}", c.id))
                    });
                    let result = if let Some(diagnostic) = diagnostic {
                        Cell {
                            diagnostic,
                            ..previous
                        }
                    } else if stage == 1 && method == Method::Ndm && previous.verified {
                        previous
                    } else if previous.diagnostic.is_empty() {
                        design_cell(models, group, combined, cancel, stage, previous)
                    } else {
                        previous
                    };

                    CRACK_CORRECTING.set(false);
                    CRACK_PROGRESS.with(|slot| slot.borrow_mut().take());
                    (index, proposed, result, ndm::stats())
                };
                if workers == 1 {
                    immediate.push(work());
                } else {
                    handles.push(scope.spawn(work));
                }
            }
            for handle in handles {
                immediate.push(handle.join().expect("reinforcement worker panicked"));
            }
            immediate
        });
        for (index, proposed, result, stats) in results {
            {
                let mut total = self.progress.stage_stats[stage].lock().unwrap();
                total.equilibrium_calls += stats.equilibrium_calls;
                total.seed_attempts += stats.seed_attempts;
                total.fallback_calls += stats.fallback_calls;
            }
            let group = &self.input.groups[index];
            if stage == 1 && result.verified {
                if result.areas == proposed {
                    self.progress.unchanged.fetch_add(1, Ordering::Relaxed);
                } else {
                    self.progress.corrected.fetch_add(1, Ordering::Relaxed);
                }
            }
            for &display in &group.displays {
                self.cells[display] = result.clone();
            }
            self.index += 1;
            self.progress.done[stage].store(self.index, Ordering::Relaxed);
            self.progress.elapsed_ms[stage].store(
                self.started.elapsed().as_millis() as usize,
                Ordering::Relaxed,
            );
        }
        if self.index == self.input.groups.len() {
            self.stage += 1;
            self.index = 0;
            self.started = web_time::Instant::now();
            self.progress.stage.store(self.stage, Ordering::Relaxed);
            ndm::reset_stats();
        }
        Ok(true)
    }
    fn finish(self) -> Output {
        let errors = self
            .input
            .groups
            .iter()
            .filter_map(|g| {
                let c = &self.cells[g.displays[0]];
                (!c.verified || !c.service_verified).then(|| (g.id, c.diagnostic.clone()))
            })
            .collect();
        Output {
            cells: self.cells,
            physical_count: self.input.groups.len(),
            errors,
            wall_displays: self
                .input
                .groups
                .iter()
                .filter_map(|group| Some((group.wall?.surface, group.displays[0])))
                .collect(),
        }
    }
}
pub fn run(
    input: Input,
    progress: Arc<Progress>,
    cancel: Arc<AtomicBool>,
) -> Result<Output, String> {
    let mut job = DesignJob::new(input, progress, cancel)?;
    while job.step()? {}
    Ok(job.finish())
}
#[derive(serde::Serialize, serde::Deserialize)]
pub struct Snapshot {
    stage: usize,
    done: [usize; 3],
    elapsed_ms: [usize; 3],
    unchanged: usize,
    corrected: usize,
    #[serde(default)]
    crack_phase: usize,
}
impl Snapshot {
    pub fn read(p: &Progress) -> Self {
        Self {
            stage: p.stage.load(Ordering::Relaxed),
            done: std::array::from_fn(|i| p.done[i].load(Ordering::Relaxed)),
            elapsed_ms: std::array::from_fn(|i| p.elapsed_ms[i].load(Ordering::Relaxed)),
            unchanged: p.unchanged.load(Ordering::Relaxed),
            corrected: p.corrected.load(Ordering::Relaxed),
            crack_phase: p.crack_phase.load(Ordering::Relaxed),
        }
    }
    pub fn apply(&self, p: &Progress) {
        p.stage.store(self.stage, Ordering::Relaxed);
        for i in 0..3 {
            p.done[i].store(self.done[i], Ordering::Relaxed);
            p.elapsed_ms[i].store(self.elapsed_ms[i], Ordering::Relaxed);
        }
        p.unchanged.store(self.unchanged, Ordering::Relaxed);
        p.corrected.store(self.corrected, Ordering::Relaxed);
        p.crack_phase.store(self.crack_phase, Ordering::Relaxed);
    }
}
#[cfg(target_arch = "wasm32")]
pub fn worker_run(
    text: &str,
    callback: &js_sys::Function,
) -> Result<String, wasm_bindgen::JsValue> {
    let error = |e: String| wasm_bindgen::JsValue::from_str(&e);
    let input: Input = serde_json::from_str(text).map_err(|e| error(e.to_string()))?;
    let progress = Arc::new(Progress::default());
    let report = || -> Result<(), wasm_bindgen::JsValue> {
        let json =
            serde_json::to_string(&Snapshot::read(&progress)).map_err(|e| error(e.to_string()))?;
        callback.call1(
            &wasm_bindgen::JsValue::NULL,
            &wasm_bindgen::JsValue::from_str(&json),
        )?;
        Ok(())
    };
    report()?;
    let mut job =
        DesignJob::new(input, progress.clone(), Arc::new(AtomicBool::new(false))).map_err(error)?;
    let mut last = web_time::Instant::now();
    let mut stage = 0;
    while job.step().map_err(error)? {
        if last.elapsed().as_millis() >= 80 || job.stage != stage {
            report()?;
            last = web_time::Instant::now();
            stage = job.stage;
        }
    }
    report()?;
    serde_json::to_string(&job.finish()).map_err(|e| error(e.to_string()))
}
/// First test minimum steel, then bisect excess steel
/// along a mechanics-derived 4-area proportion, with full NDM at every trial.
fn ndm_size(
    models: &Models,
    group: &Group,
    combined: &[Combined],
    cancel: &AtomicBool,
    mut cell: Cell,
) -> Cell {
    if let Err(error) = group.settings.validate(group.h) {
        cell.diagnostic = error;
        return cell;
    }
    let minimum = ((group.settings.minimum_percent * group.h / 10.).max(0.5) * 100.).ceil() / 100.;
    let maximum = group.settings.maximum_area(group.h);
    if minimum > maximum {
        cell.diagnostic =
            format!("Минимальная площадь {minimum:.2} см²/м превышает предел {maximum:.2} см²/м");
        return cell;
    }
    let actions: Vec<_> = combined
        .iter()
        .flat_map(|c| {
            c.strength(group.displays[0])
                .map(move |(forces, rb)| (c.id, forces, rb))
        })
        .collect();
    let angles = adaptive_angles(group, combined);
    let mut states = vec![None; angles.len() * actions.len()];
    let failed_angle = std::cell::Cell::new(None);
    let mut feasible = |a: [f64; 4]| {
        let mut index = 0;
        for &angle in &angles {
            let (areas, cover) = directional(a, group.settings, angle);
            for &(combination, forces, rb) in &actions {
                if cancel.load(Ordering::Relaxed) {
                    return false;
                }
                let mut section = group.settings.section(group.h, cover[0], cover[1], false);
                section.rb *= rb;
                let load = forces.angle(angle);
                if let Some(state) = section.equilibrium(load, areas, false, states[index]) {
                    states[index] = Some(state);
                } else {
                    failed_angle.set(Some((angle, combination)));
                    return false;
                }
                index += 1;
            }
        }
        true
    };
    if feasible([minimum; 4]) {
        cell.governing_angle = angles
            .iter()
            .copied()
            .map(|angle| {
                let demand = actions
                    .iter()
                    .map(|(_, f, _)| {
                        let [n, m] = f.angle(angle);
                        n.abs() + m.abs() * 6000. / group.h
                    })
                    .fold(0., f64::max);
                (demand, angle)
            })
            .max_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, angle)| angle);
        cell.areas = [minimum; 4];
        cell.verified = true;
        return cell;
    }
    let upper = design_cell_core(models, group, combined, cancel, 0, cell.clone());
    let upper = if upper.diagnostic.is_empty() {
        design_cell_core(models, group, combined, cancel, 1, upper)
    } else {
        upper
    };
    let base = if upper.verified {
        upper.areas
    } else if feasible([maximum; 4]) {
        // The proposal and its geometric growth only find a starting bracket.
        // Exhausting either must not replace verification at the allowed cap.
        [maximum; 4]
    } else {
        if cancel.load(Ordering::Relaxed) {
            cell.diagnostic = "Отменено".into();
        } else if let Some((angle, combination)) = failed_angle.get() {
            cell.governing_angle = Some(angle);
            cell.governing_strength = combination;
            cell.diagnostic = format!("НДМ: прочность не подтверждена в сочетании {combination} при пределе {maximum} см²/м на слой и направление");
        }
        return cell;
    };
    let mut best = base;
    let mut lo = 0.;
    let mut hi = 1.;
    for _ in 0..16 {
        if cancel.load(Ordering::Relaxed) {
            cell.diagnostic = "Отменено".into();
            return cell;
        }
        if base.iter().all(|a| (a - minimum) * (hi - lo) <= 0.01) {
            break;
        }
        let mid = (lo + hi) * 0.5;
        let trial =
            base.map(|a| (((minimum + (a - minimum) * mid) * 100.).ceil() / 100.).min(maximum));
        if feasible(trial) {
            hi = mid;
            best = trial;
        } else {
            lo = mid;
        }
    }
    cell.governing_angle = failed_angle
        .get()
        .map(|(angle, _)| angle)
        .or(upper.governing_angle);
    cell.governing_strength = failed_angle
        .get()
        .map_or(upper.governing_strength, |(_, combination)| combination);
    cell.areas = best;
    cell.verified = true;
    cell
}
/// Shared native/WASM benchmark: same Input and same stage boundaries.
#[cfg(any(test, feature = "inference-bench"))]
pub(crate) fn benchmark_strength_model(
    mut input: Input,
    mode: u8,
    mut notify: impl FnMut(usize, usize),
) -> Result<serde_json::Value, String> {
    if mode > 2 {
        return Err("Unknown benchmark mode".into());
    }
    struct Reset;
    impl Drop for Reset {
        fn drop(&mut self) {
            BENCH_BATCH.with(|v| v.set(true));
        }
    }
    let _reset = Reset;
    BENCH_BATCH.with(|v| v.set(mode == 1));
    input.method = if mode == 2 { Method::Ndm } else { Method::Ml };
    let physical = input.groups.len();
    let progress = Arc::new(Progress::default());
    let mut job = DesignJob::new(input, progress.clone(), Arc::new(AtomicBool::new(false)))?;
    let end_stage = if mode == 2 { 1 } else { 2 };
    let mut completed = 0;
    while job.stage < end_stage {
        job.step()?;
        completed += 1;
        if completed % 2048 == 0 || job.index == 0 {
            notify(job.stage, job.index);
        }
    }
    let verified = job
        .input
        .groups
        .iter()
        .filter(|g| job.cells[g.displays[0]].verified)
        .count();
    let errors = job
        .input
        .groups
        .iter()
        .filter_map(|g| {
            let c = &job.cells[g.displays[0]];
            (!c.verified).then(|| (g.id, c.diagnostic.clone()))
        })
        .collect::<Vec<_>>();
    if !errors.is_empty() {
        return Err(format!("Unverified FE: {errors:?}"));
    }
    let sums = std::array::from_fn::<_, 4, _>(|i| {
        job.input
            .groups
            .iter()
            .map(|g| job.cells[g.displays[0]].areas[i])
            .sum::<f64>()
    });
    Ok(
        serde_json::json!({"mode":mode,"physical":physical,"verified":verified,"display":job.cells.len(),"stage_ms":[progress.elapsed_ms[0].load(Ordering::Relaxed),if mode==2{0}else{progress.elapsed_ms[1].load(Ordering::Relaxed)}],"physical_As_sums":sums,"errors":errors,"ndm_duplicate_check":false}),
    )
}
#[cfg(test)]
pub(crate) fn benchmark_batch(input: Input, weights: &[f64]) {
    let mut runs = vec![];
    for repeat in 0..3 {
        for batch in if repeat % 2 == 0 {
            [false, true]
        } else {
            [true, false]
        } {
            BENCH_BATCH.with(|v| v.set(batch));
            let progress = Arc::new(Progress::default());
            let start = web_time::Instant::now();
            let mut job = DesignJob::new(
                input.clone(),
                progress.clone(),
                Arc::new(AtomicBool::new(false)),
            )
            .unwrap();
            while job.stage < 2 {
                job.step().unwrap();
            }
            let elapsed = start.elapsed().as_secs_f64();
            let errors = job.cells.iter().filter(|c| !c.verified).count();
            assert_eq!(errors, 0);
            let row = serde_json::json!({"repeat":repeat,"batch":batch,"strength_s":elapsed,"stage_ms":[progress.elapsed_ms[0].load(Ordering::Relaxed),progress.elapsed_ms[1].load(Ordering::Relaxed)],"physical":input.groups.len(),"verified_display":job.cells.len(),"steel_proxy_kg":job.cells.iter().zip(weights).map(|(c,w)|c.areas.iter().sum::<f64>()*w*0.785).sum::<f64>()});
            println!("BATCH_BENCH {row}");
            runs.push(row);
            std::fs::write(
                "target/rc_batch_benchmark.json",
                serde_json::to_vec_pretty(&runs).unwrap(),
            )
            .unwrap();
        }
    }
    BENCH_BATCH.with(|v| v.set(true));
}
/// Test-only experiment: no shipping selector or alternative production solver.
#[cfg(test)]
pub(crate) fn benchmark_ml_vs_mechanics(input: Input, weights: &[f64]) -> serde_json::Value {
    struct Restore;
    impl Drop for Restore {
        fn drop(&mut self) {
            BENCH_NDM_BISECT.with(|v| v.set(false));
            BENCH_ML_ALLOWED.with(|v| v.set(true));
        }
    }
    let _restore = Restore;
    let reference = std::env::var_os("MINI_FEM_BENCH_NDM_REFERENCE").is_some();
    BENCH_NDM_BISECT.with(|v| v.set(reference));
    let mut rows = vec![];
    // Alternating order limits systematic warm-cache/CPU bias.
    for repeat in 0..3 {
        for enabled in if reference {
            vec![false]
        } else if repeat % 2 == 0 {
            vec![true, false]
        } else {
            vec![false, true]
        } {
            BENCH_ML_ALLOWED.with(|v| v.set(enabled));
            let input = input.clone();
            let progress = Arc::new(Progress::default());
            let start = web_time::Instant::now();
            let output = run(input, progress.clone(), Arc::new(AtomicBool::new(false))).unwrap();
            let elapsed = start.elapsed().as_secs_f64();
            let mean = std::array::from_fn::<_, 4, _>(|i| {
                output
                    .cells
                    .iter()
                    .zip(weights)
                    .filter(|(c, _)| c.verified && c.service_verified)
                    .map(|(c, w)| c.areas[i] * w)
                    .sum::<f64>()
                    / weights.iter().sum::<f64>()
            });
            let row = serde_json::json!({"repeat":repeat,"ml":enabled,"ndm_bisection":reference,"stage_ms":progress.elapsed_ms.iter().map(|v|v.load(Ordering::Relaxed)).collect::<Vec<_>>(),"total_s":elapsed,"physical":output.physical_count,"display":output.cells.len(),"strength_verified":output.cells.iter().filter(|c|c.verified).count(),"service_verified":output.cells.iter().filter(|c|c.service_verified).count(),"errors":output.errors,"area_weighted_As":mean,"theoretical_steel_kg":output.cells.iter().zip(weights).map(|(c,w)|c.areas.iter().sum::<f64>()*w*0.785).sum::<f64>(),"stats":progress.stage_stats.iter().map(|s|*s.lock().unwrap()).collect::<Vec<_>>()});
            println!("RC_BENCH {row}");
            rows.push(row);
            std::fs::write(
                if reference {
                    "target/rc_ml_vs_ndm_reference.json"
                } else {
                    "target/rc_ml_vs_ndm_benchmark.json"
                },
                serde_json::to_vec_pretty(&rows).unwrap(),
            )
            .unwrap();
        }
    }
    serde_json::json!(rows)
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ndm_checks_allowed_steel_when_the_proposal_growth_stalls_in_compression() {
        let mut load = LoadCase::new(1);
        load.gamma_f = 1.;
        let group = Group {
            id: 1,
            displays: vec![0],
            h: 200.,
            settings: Settings {
                concrete: 3,
                ..Default::default()
            },
            wall: None,
        };
        let input = Input {
            method: Method::Ndm,
            groups: vec![group],
            cases: BTreeMap::from([(
                1,
                vec![Forces {
                    n: [0., -5600., 0.],
                    m: [0.; 3],
                }],
            )]),
            loads: vec![load],
            combinations: vec![Combination::new(1, [1])],
            display_count: 1,
            nonlinear_offset: None,
        };
        let mut job =
            DesignJob::new(input, Arc::default(), Arc::new(AtomicBool::new(false))).unwrap();
        job.step().unwrap();
        let cell = &job.cells[0];
        assert!(cell.verified, "{}", cell.diagnostic);
        assert!(cell.areas[2] > 25. && cell.areas[2] < 200.);
    }

    #[test]
    fn wall_slenderness_envelopes_accidental_eccentricity_and_reports_instability() {
        let wall = Wall {
            surface: 0,
            height_m: 3.,
            effective_m: 3.,
        };
        let group = Group {
            id: 1,
            displays: vec![0],
            h: 200.,
            settings: Settings {
                asby: 35.,
                ..Default::default()
            },
            wall: Some(wall),
        };
        let compression = Forces {
            n: [2., -1000., 4.],
            m: [8., 0., 6.],
        };
        let (a, b) = wall
            .moments(&group, compression, compression, false)
            .unwrap();
        // B25, e = 10 mm, phi_l = 2, delta_e = 0.15 and concrete-only D.
        // For a 200 mm strip and l0 = 3 m, Ncr = 3655.41 kN/m.
        assert!((a.m[1] - 13.7658981569).abs() < 1e-9);
        assert_eq!(b.unwrap().m[1], -a.m[1]);
        assert_eq!(a.n, compression.n);
        assert_eq!([a.m[0], a.m[2]], [8., 6.]);
        let (tangent, _) = wall
            .moments(&group, compression, compression, true)
            .unwrap();
        assert_eq!(tangent.m[1], 10.);
        let signed = Forces {
            m: [8., -30., 6.],
            ..compression
        };
        let (a, b) = wall.moments(&group, signed, signed, false).unwrap();
        assert!(a.m[1] < -30.);
        assert!(b.is_none());
        let tension = Forces {
            n: [2., 1000., 4.],
            ..compression
        };
        assert_eq!(
            wall.moments(&group, tension, tension, false).unwrap().0.m,
            tension.m
        );
        let excessive = Forces {
            n: [0., -1e6, 0.],
            ..compression
        };
        assert!(wall
            .moments(&group, excessive, excessive, false)
            .unwrap_err()
            .contains("Ncr"));
    }

    #[test]
    fn ml_crack_decoder_uses_both_stress_pairs_and_checks_ambiguous_events() {
        let model = ml::Network::load(true).unwrap();
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../../assets/ndm/reference.json")).unwrap();
        for row in reference["crack"].as_array().unwrap() {
            let raw: Vec<f32> = row["network"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect();
            let p = CrackPrediction::decode(&model, &raw, 500.).unwrap();
            assert!((p.crc[0] - (raw[2] as f64).tanh() * 500.).abs() < 1e-10);
            assert!((p.full[0] - (raw[4] as f64).tanh() * 500.).abs() < 1e-10);
            assert!(p.first > 0. && p.ultimate > 0.);
        }
        let settings = Settings::default();
        let p = CrackPrediction {
            first: 2.,
            ultimate: 3.,
            event: 1,
            confidence: 0.99,
            crc: [10.; 2],
            full: [20.; 2],
        };
        ndm::reset_stats();
        assert_eq!(
            ml_crack_width(settings, 200., [5.; 2], [35.; 2], [0., 1.], p)
                .unwrap()
                .0,
            [0.; 2]
        );
        assert_eq!(ndm::stats().equilibrium_calls, 0);
        assert!(ml_crack_width(
            settings,
            200.,
            [5.; 2],
            [35.; 2],
            [0., 1.],
            CrackPrediction { first: 1.05, ..p }
        )
        .is_none());
        assert!(ml_crack_width(
            settings,
            200.,
            [5.; 2],
            [35.; 2],
            [0., 1.],
            CrackPrediction {
                confidence: 0.5,
                ..p
            }
        )
        .is_none());
    }
    #[test]
    fn ndm_default_has_no_networks_and_no_duplicate_strength_check() {
        assert!(Method::default() == Method::Ndm);
        let models = Models::for_method(Method::Ndm).unwrap();
        assert!(models.strength.is_none() && models.crack.is_none());
        let mut load = LoadCase::new(1);
        load.gamma_f = 1.1;
        load.gamma_l = 0.4;
        let input = Input {
            method: Method::Ndm,
            groups: (0..2)
                .map(|i| Group {
                    id: i as u64 + 1,
                    displays: vec![i],
                    h: 200.,
                    settings: Settings::default(),
                    wall: None,
                })
                .collect(),
            cases: BTreeMap::from([(
                1,
                vec![
                    Forces {
                        n: [0.; 3],
                        m: [20., 0., 0.],
                    },
                    Forces {
                        n: [0.; 3],
                        m: [1e6, 0., 0.],
                    },
                ],
            )]),
            loads: vec![load],
            combinations: vec![Combination::new(1, [1])],
            display_count: 2,
            nonlinear_offset: None,
        };
        let mut legacy = serde_json::to_value(&input).unwrap();
        legacy.as_object_mut().unwrap().remove("method");
        legacy["groups"][0]["settings"]
            .as_object_mut()
            .unwrap()
            .remove("maximum_percent");
        assert!(serde_json::from_value::<Input>(legacy).unwrap().method == Method::Ndm);
        let mut limited = input.clone();
        limited.groups[0].settings.maximum_percent = Some(1.);
        limited.groups[1].settings.maximum_percent = Some(0.1);
        limited.cases.get_mut(&1).unwrap()[1] = Forces {
            n: [300., 0., 0.],
            m: [0.; 3],
        };
        let progress = Arc::new(Progress::default());
        let output = run(input, progress.clone(), Arc::new(AtomicBool::new(false))).unwrap();
        assert!(
            output.cells[0].verified && output.cells[0].service_verified,
            "{:?}",
            output.errors
        );
        assert_eq!(progress.stage_stats[1].lock().unwrap().equilibrium_calls, 0);
        assert!(progress.stage_stats[0].lock().unwrap().equilibrium_calls > 0);
        assert!(!output.cells[1].verified);
        assert_eq!(output.errors.len(), 1);
        assert_eq!(output.errors[0].0, 2);
        assert!(output.errors[0].1.contains("при пределе 200 см²/м"));
        let result = run(
            limited,
            Arc::new(Progress::default()),
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        assert!(result.cells[0].service_verified);
        assert!(result.cells[0].areas.iter().all(|&area| area <= 20.));
        assert!(!result.cells[1].verified);
        assert!(result.errors[0].1.contains("при пределе 2 см²/м"));
    }
    #[test]
    fn resumable_design_matches_batch_and_observes_cancel_between_elements() {
        fn input() -> Input {
            Input {
                method: Method::Ml,
                groups: (0..3)
                    .map(|i| Group {
                        id: i as u64 + 1,
                        displays: vec![i],
                        h: 200.,
                        settings: Settings::default(),
                        wall: None,
                    })
                    .collect(),
                cases: BTreeMap::from([(
                    1,
                    vec![
                        Forces {
                            n: [0.; 3],
                            m: [20., 0., 0.]
                        };
                        3
                    ],
                )]),
                loads: vec![LoadCase::new(1)],
                combinations: vec![Combination::new(1, [1])],
                display_count: 3,
                nonlinear_offset: None,
            }
        }
        let batch = run(input(), Arc::default(), Arc::new(AtomicBool::new(false))).unwrap();
        let wire = serde_json::to_string(&input()).unwrap();
        let transported: Input = serde_json::from_str(&wire).unwrap();
        let transport_result = run(
            transported,
            Arc::default(),
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        let transport_json = serde_json::to_string(&transport_result).unwrap();
        let transport_result: Output = serde_json::from_str(&transport_json).unwrap();
        for (a, b) in transport_result.cells.iter().zip(&batch.cells) {
            assert_eq!(a.areas, b.areas);
            assert!((a.crack_short - b.crack_short).abs() < 1e-10);
            assert!((a.crack_long - b.crack_long).abs() < 1e-10);
        }

        let progress = Arc::new(Progress::default());
        let cancel = Arc::new(AtomicBool::new(false));
        let mut job = DesignJob::new(input(), progress.clone(), cancel.clone()).unwrap();
        assert!(job.step().unwrap());
        assert!((1..=3).contains(&progress.done[0].load(Ordering::Relaxed)));
        assert_eq!(progress.done[1].load(Ordering::Relaxed), 0);
        while job.step().unwrap() {}
        let resumed = job.finish();
        assert_eq!(resumed.errors, batch.errors);
        for (a, b) in resumed.cells.iter().zip(&batch.cells) {
            assert_eq!(a.areas, b.areas);
            assert_eq!(a.crack_short, b.crack_short);
            assert_eq!(a.crack_long, b.crack_long);
        }
        assert_eq!(progress.stage.load(Ordering::Relaxed), 3);
        let snapshot = serde_json::to_string(&Snapshot::read(&progress)).unwrap();
        let received: Snapshot = serde_json::from_str(&snapshot).unwrap();
        let remote = Progress::default();
        received.apply(&remote);
        assert_eq!(
            serde_json::to_string(&Snapshot::read(&remote)).unwrap(),
            snapshot
        );

        for stage in 0..3 {
            assert_eq!(progress.done[stage].load(Ordering::Relaxed), 3);
        }
        let mut job = DesignJob::new(input(), Arc::default(), cancel.clone()).unwrap();
        job.step().unwrap();
        cancel.store(true, Ordering::Relaxed);
        assert!(job.step().is_err());
        assert!(job.index <= 3);
    }
    #[test]
    fn capra_maury_lp_is_minimal_and_twist_requires_both_directions() {
        let p = minimum_pair(&[[1., 0., 2.], [0., 1., 3.], [0.5, 0.5, 6.]], 1., 200.).unwrap();
        assert!((p[0] + p[1] - 12.).abs() < 1e-8);
        assert!(p[0] >= 2. && p[1] >= 3.);
        let force = Forces {
            n: [0.; 3],
            m: [0., 0., 10.],
        };
        assert!((force.angle(45_f64.to_radians())[1] - 10.).abs() < 1e-8);
    }

    #[test]
    fn crack_limit_bisection_only_increases_tension_face_and_rechecks_strength() {
        let models = Models::load().unwrap();
        let settings = Settings {
            acrcl: 0.025,
            acrcsh: 0.035,
            ..Settings::default()
        };
        let group = Group {
            id: 1,
            displays: vec![0],
            h: 200.,
            settings,
            wall: None,
        };
        let combined = vec![Combined {
            id: 1,
            design: vec![Forces {
                n: [0.; 3],
                m: [20., 0., 0.],
            }],
            design_long: vec![Forces {
                n: [0.; 3],
                m: [8., 0., 0.],
            }],
            norm: vec![Forces {
                n: [0.; 3],
                m: [18., 0., 0.],
            }],
            long: vec![Forces {
                n: [0.; 3],
                m: [7., 0., 0.],
            }],
            reverse: Default::default(),
            wall_errors: Default::default(),
        }];
        let cancel = AtomicBool::new(false);
        let initial = Cell {
            areas: [0.; 4],
            strength_areas: None,
            crack_short: 0.,
            crack_long: 0.,
            crack_ratio: 0.,
            verified: false,
            service_verified: false,
            diagnostic: String::new(),
            governing_strength: 0,
            governing_angle: None,
            governing_crack: 0,
        };
        let ml = design_cell(&models, &group, &combined, &cancel, 0, initial);
        let uls = design_cell(&models, &group, &combined, &cancel, 1, ml);
        let sls = design_cell(&models, &group, &combined, &cancel, 2, uls.clone());
        assert!(sls.service_verified, "{}", sls.diagnostic);
        assert!(sls.verified);
        assert!(sls.crack_long <= settings.acrcl * (1. + 1e-8));
        assert!(sls.crack_short <= settings.acrcsh * (1. + 1e-8));
        assert_eq!(sls.strength_areas, Some(uls.areas));
        let output = Output {
            cells: vec![sls.clone()],
            physical_count: 1,
            errors: vec![],
            wall_displays: Default::default(),
        };
        for field in 26..=29 {
            let full = output.value_for(field, 0, AreaView::Total);
            let strength = output.value_for(field, 0, AreaView::Strength);
            let crack = output.value_for(field, 0, AreaView::CrackIncrement);
            assert!((full - strength - crack).abs() < 1e-10);
            assert_eq!(strength, uls.areas[field - 26]);
        }
        let restored: Output =
            serde_json::from_slice(&serde_json::to_vec(&output).unwrap()).unwrap();
        assert_eq!(restored.cells[0].strength_areas, Some(uls.areas));
        assert!(sls.areas[0] > uls.areas[0]);
        assert_eq!(sls.areas[1], uls.areas[1]);
        assert_eq!(sls.areas[3], uls.areas[3]);
    }
    #[test]
    fn pilot_design_checks_both_layers_and_service_results_on_physical_element() {
        let mut load = LoadCase::new(1);
        load.gamma_f = 1.1;
        load.gamma_l = 0.4;
        let input = Input {
            method: Method::Ml,
            groups: vec![Group {
                id: 10,
                displays: vec![0, 1],
                h: 200.,
                settings: Settings::default(),
                wall: None,
            }],
            cases: BTreeMap::from([(
                1,
                vec![
                    Forces {
                        n: [0.; 3],
                        m: [20., 0., 0.]
                    };
                    2
                ],
            )]),
            loads: vec![load],
            combinations: vec![Combination::new(1, [1])],
            display_count: 2,
            nonlinear_offset: None,
        };
        let result = run(
            input,
            Arc::new(Progress::default()),
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        let cell = &result.cells[0];
        assert!(cell.verified, "{}", cell.diagnostic);
        assert!(cell.service_verified, "{}", cell.diagnostic);
        assert!(cell.areas[0] > cell.areas[1]);
        assert!(cell.areas.iter().all(|v| v.is_finite() && *v >= 2.));
        assert_eq!(result.cells[0].areas, result.cells[1].areas);
        assert!(cell.crack_short.is_finite() && cell.crack_short >= 0.);
        assert!(cell.crack_long.is_finite() && cell.crack_long >= 0.);
        assert_eq!(result.physical_count, 1);
        // Both strength variants are checked independently, including reduced Rb.
        for (load, rb_factor) in [([0., 20.], 1.), ([0., 8.], 0.9)] {
            let (areas, cover) = directional(cell.areas, Settings::default(), 0.);
            let mut section = Settings::default().section(200., cover[0], cover[1], false);
            section.rb *= rb_factor;
            assert!(section.equilibrium(load, areas, false, None).is_some());
        }
    }
}

#[cfg(test)]
mod large_batch_bench;
