use super::*;
use crate::plan_trace::fem::{displacement_field, DISPLACEMENT_NORM_FIELD, FIELD_NAMES};
use crate::plan_trace::{
    load_combinations::{Combination, Variant},
    loads::LoadCase,
};
fn weights(xy: [Point; 3], p: Point) -> Option<[f64; 3]> {
    let [a, b, c] = xy;
    let det = (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0]);
    if det.abs() < 1e-12 {
        return None;
    }
    let u = ((p[0] - a[0]) * (c[1] - a[1]) - (p[1] - a[1]) * (c[0] - a[0])) / det;
    let v = ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])) / det;
    let l = [1. - u - v, u, v];
    l.iter().all(|v| *v >= -1e-8).then_some(l)
}
pub(super) fn result_label(
    painter: &egui::Painter,
    p: Pos2,
    text: String,
    bounds: Rect,
    style: drawing_flags::TextStyle,
) -> Rect {
    let galley = painter.layout_no_wrap(text, egui::FontId::proportional(style.size), style.ink());
    let size = galley.size() + egui::vec2(8., 4.);
    let min = Pos2::new(
        p.x.clamp(bounds.left(), (bounds.right() - size.x).max(bounds.left())),
        p.y.clamp(bounds.top(), (bounds.bottom() - size.y).max(bounds.top())),
    );
    let area = Rect::from_min_size(min, size);
    painter.rect_filled(area, 2., Color32::from_rgba_unmultiplied(20, 24, 30, 235));
    painter.galley(min + egui::vec2(4., 2.), galley, style.ink());
    area
}
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct NonlinearOptions {
    enabled: bool,
    combination: u64,
    variant: crate::plan_trace::load_combinations::Variant,
}
impl Default for NonlinearOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            combination: 0,
            variant: crate::plan_trace::load_combinations::Variant::Design,
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
enum AnalysisLoadSource {
    LoadCase(u64),
    Combination(u64),
}

impl AnalysisLoadSource {
    fn resolve(
        self,
        cases: &[LoadCase],
        combinations: &[Combination],
        variant: Variant,
    ) -> Result<(Vec<(u64, f64)>, String), String> {
        let (weights, name) = match self {
            Self::LoadCase(id) => {
                let case = cases
                    .iter()
                    .find(|case| case.id == id)
                    .ok_or("выберите загружение")?;
                (vec![(id, variant.weight(case, 1.)?)], case.display_name())
            }
            Self::Combination(id) => {
                let combination = combinations
                    .iter()
                    .find(|c| c.id == id)
                    .ok_or("выберите сочетание")?;
                (
                    combination.weights(cases, variant)?,
                    format!("Сочетание {}", combination.display_name()),
                )
            }
        };
        Ok((weights, format!("{name} · {}", variant.label())))
    }
}

fn analysis_load_controls(
    ui: &mut egui::Ui,
    id: &'static str,
    source: &mut AnalysisLoadSource,
    variant: &mut Variant,
    cases: &[LoadCase],
    combinations: &[Combination],
) {
    let selected = match *source {
        AnalysisLoadSource::LoadCase(id) => cases
            .iter()
            .find(|case| case.id == id)
            .map(LoadCase::display_name),
        AnalysisLoadSource::Combination(id) => combinations
            .iter()
            .find(|c| c.id == id)
            .map(|c| format!("Сочетание {}", c.display_name())),
    };
    ui.horizontal_wrapped(|ui| {
        egui::ComboBox::from_id_salt((id, "source"))
            .width(160.)
            .truncate()
            .selected_text(selected.unwrap_or_else(|| "Выберите нагрузки".into()))
            .show_ui(ui, |ui| {
                ui.strong("Загружения");
                for case in cases {
                    ui.selectable_value(
                        source,
                        AnalysisLoadSource::LoadCase(case.id),
                        case.display_name(),
                    );
                }
                if !combinations.is_empty() {
                    ui.separator();
                    ui.strong("Сочетания");
                    for combination in combinations {
                        ui.selectable_value(
                            source,
                            AnalysisLoadSource::Combination(combination.id),
                            format!("Сочетание {}", combination.display_name()),
                        );
                    }
                }
            });
        egui::ComboBox::from_id_salt((id, "variant"))
            .selected_text(variant.label())
            .show_ui(ui, |ui| {
                for value in Variant::ALL {
                    ui.selectable_value(variant, value, value.label());
                }
            });
    });
}

#[derive(Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct ModalMassOptions {
    source: Option<AnalysisLoadSource>,
    variant: Variant,
}

impl Default for ModalMassOptions {
    fn default() -> Self {
        Self {
            source: None,
            variant: Variant::Normative,
        }
    }
}

#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct BucklingOptions {
    enabled: bool,
    source: Option<AnalysisLoadSource>,
    variant: Variant,
    forms: usize,
    max_factor: f64,
}

impl Default for BucklingOptions {
    fn default() -> Self {
        Self {
            enabled: false,
            source: None,
            variant: crate::plan_trace::load_combinations::Variant::Design,
            forms: 6,
            max_factor: 10.,
        }
    }
}

#[derive(Clone, Copy)]
pub(super) enum ShapeSource {
    Vibration(f64),
    Buckling(f64),
}

impl ShapeSource {
    pub(super) fn is_buckling(self) -> bool {
        matches!(self, Self::Buckling(_))
    }

    pub(super) fn cache_id(self) -> u64 {
        match self {
            Self::Vibration(_) => 1,
            Self::Buckling(_) => 2,
        }
    }

    pub(super) fn caption(self, number: usize) -> String {
        match self {
            Self::Vibration(hz) => format!("Форма {number} · {hz:.4} Гц"),
            Self::Buckling(factor) => {
                format!("Потеря устойчивости · форма {number} · КЗУ {factor:.4}")
            }
        }
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum VibrationQuantity {
    Axis(usize),
    Magnitude(u8),
}

impl VibrationQuantity {
    pub(super) fn value(self, xyz: [f64; 3]) -> f64 {
        match self {
            Self::Axis(axis) => xyz[axis],
            Self::Magnitude(axes) => crate::plan_trace::fem::displacement_norm(xyz, axes),
        }
    }

    pub(super) fn label(self) -> String {
        match self {
            Self::Axis(axis) => format!("Перемещения {}, отн.", ["X", "Y", "Z"][axis]),
            Self::Magnitude(axes) => {
                let components = ["X", "Y", "Z"]
                    .into_iter()
                    .enumerate()
                    .filter_map(|(axis, name)| (axes & (1 << axis) != 0).then_some(name))
                    .collect::<Vec<_>>()
                    .join("/");
                format!("Модуль перемещений {components}, отн.")
            }
        }
    }
}

const DEFAULT_SHAPE_AMPLITUDE_MM: f64 = 1000.;

#[derive(Clone)]
pub(super) struct ModalView {
    pub stiffness_variant: usize,
    pub result_generation: u64,
    pub mesh_generation: u64,
    pub shape: std::sync::Arc<Vec<[f64; 3]>>,
    pub amplitude: f64,
    pub number: usize,
    pub source: ShapeSource,
    pub zero_translation: bool,
    pub playing: bool,
    pub started: f64,
    pub paused_at: Option<f64>,
    pub vibration_values: Option<(VibrationQuantity, [f64; 2])>,
    sensitivity: Option<BucklingColors>,
    show_sensitivity: bool,
}

#[derive(Clone)]
pub(super) struct BucklingColors {
    pub data: std::sync::Arc<crate::plan_trace::fem::BucklingSensitivity>,
    pub range: [f64; 2],
}

impl BucklingColors {
    fn new(data: &crate::plan_trace::fem::BucklingSensitivity) -> Option<Self> {
        let range = data
            .surfaces
            .iter()
            .chain(&data.beams)
            .copied()
            .map(f64::from)
            .filter(|v| v.is_finite())
            .fold([0_f64; 2], |[min, max], value| {
                [min.min(value), max.max(value)]
            });
        (range != [0.; 2]).then(|| Self {
            data: std::sync::Arc::new(data.clone()),
            range,
        })
    }
}

impl ModalView {
    pub(super) fn active_sensitivity(&self) -> Option<&BucklingColors> {
        self.sensitivity.as_ref().filter(|_| self.show_sensitivity)
    }
    fn set_vibration_quantity(&mut self, quantity: Option<VibrationQuantity>) {
        if self.vibration_values.map(|(old, _)| old) == quantity {
            return;
        }
        self.vibration_values = quantity.and_then(|quantity| {
            let range = self
                .shape
                .iter()
                .map(|&xyz| quantity.value(xyz))
                .filter(|v| v.is_finite())
                .fold([f64::INFINITY, f64::NEG_INFINITY], |[min, max], value| {
                    [min.min(value), max.max(value)]
                });
            range[0].is_finite().then(|| {
                (
                    quantity,
                    crate::plan_trace::result_scale::range(range.into_iter()),
                )
            })
        });
    }
    /// Inactive windows must not keep submitting animation frames to the GPU.
    pub(super) fn animation(&mut self, now: f64, active: bool) -> (f64, bool) {
        if !self.playing {
            self.paused_at = None;
            return (self.amplitude, false);
        }
        if !active {
            self.paused_at.get_or_insert(now);
        } else if let Some(paused) = self.paused_at.take() {
            self.started += (now - paused).max(0.);
        }
        let phase_time = self.paused_at.unwrap_or(now);
        (
            self.amplitude * ((phase_time - self.started) * std::f64::consts::TAU * 0.75).cos(),
            active,
        )
    }
}

#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
pub(super) struct Input {
    #[serde(default)]
    pub(super) nonlinear: Option<crate::plan_trace::fem::NonlinearRequest>,
    #[serde(default)]
    pub(super) buckling: Option<crate::plan_trace::fem::BucklingRequest>,
    #[serde(default)]
    pub(super) effective_lengths: bool,
    #[serde(default)]
    pub(super) modal_weights: Option<Vec<(u64, f64)>>,
    #[serde(default)]
    building: Option<crate::plan_trace::building_mesh::Layout>,
    contours: Vec<Contour>,
    materials: BTreeMap<u64, crate::plan_trace::fem::element::Material>,
    cases: Vec<(u64, bool)>,
    g: f64,
    step: f64,
    element_type: crate::plan_trace::fem::ElementType,
    support_height: crate::plan_trace::fem::shell::Settings,
}
fn length_method_label(method: crate::plan_trace::fem::shell::LengthMethod) -> &'static str {
    use crate::plan_trace::fem::shell::LengthMethod;
    match method {
        LengthMethod::Plane => "Плоский",
        LengthMethod::Spatial => "Пространственный",
    }
}
fn mosaic_mode_control(ui: &mut egui::Ui, id: &str, iso: &mut bool) {
    egui::ComboBox::from_id_salt(id)
        .width(85.)
        .selected_text(if *iso {
            "Изополя"
        } else {
            "Мозаика"
        })
        .show_ui(ui, |ui| {
            ui.selectable_value(iso, false, "Мозаика");
            ui.selectable_value(iso, true, "Изополя");
        });
}

pub(super) fn solve_snapshot(
    mesh: &crate::plan_trace::meshing::Mesh,
    key: &Input,
    cases: &[crate::plan_trace::loads::LoadCase],
) -> Result<crate::plan_trace::fem::Results, String> {
    crate::plan_trace::fem::progress::check_cancel()?;
    crate::plan_trace::fem::progress::stage("Подготовка элементов и матрицы жёсткости");
    crate::plan_trace::fem::solve_with_buckling(
        mesh,
        &key.contours,
        &key.materials,
        cases,
        key.g,
        key.step,
        key.element_type,
        &key.support_height,
        key.modal_weights.as_deref(),
        true,
        key.nonlinear.as_ref(),
        key.buckling.as_ref(),
    )
}
#[derive(Clone, Copy, Default)]
pub(super) struct ModelStats {
    nodes: usize,
    slabs: [usize; 2],
    walls: [usize; 2],
    beams: usize,
}
impl App {
    pub(super) fn result_has_element_field(&self) -> bool {
        matches!(self.result_field, 1..=5 | 12..=14)
    }
    pub(super) fn point_displacements(&self) -> bool {
        displacement_field(self.result_field) && !self.iso_displacements
    }
    pub(super) fn result_display_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_result_values, "Значения");
            if self.punching_view.is_some() {
                return;
            }
            if self.result_field == 15 {
                ui.label("Изополя");
            } else if displacement_field(self.result_field) || self.result_has_element_field() {
                let mode = if displacement_field(self.result_field) {
                    &mut self.iso_displacements
                } else {
                    &mut self.iso_efforts
                };
                mosaic_mode_control(ui, "result_display_mode", mode);
            }
        });
    }
    pub(super) fn force_result_factor(&self) -> f64 {
        if self.load_units == crate::plan_trace::loads::ForceUnits::TonForce {
            1. / self.gravity
        } else {
            1.
        }
    }
    pub(super) fn sync_result_units(&mut self) {
        let factor = self.force_result_factor();
        if factor.is_finite() && factor > 0. && factor != self.result_unit_factor {
            for (i, scale) in self.result_scales.iter_mut().enumerate().filter(|(i, _)| {
                (*i != 0 && *i != 10 && *i != 11 && (*i < 15 || (17..=22).contains(i)) && *i < 26)
                    || matches!(*i, 33 | 34)
            }) {
                let _ = i;
                scale.rescale_values(factor / self.result_unit_factor);
            }
            self.rescale_saved_bounds(factor / self.result_unit_factor);
            self.result_unit_factor = factor;
        }
    }
    pub(super) fn result_field_label(&self, i: usize) -> String {
        let unit = match i {
            0 | 10 | 11 | 15 | DISPLACEMENT_NORM_FIELD => "мм".into(),
            23..=25 => "рад".into(),
            1..=3 => format!("{}·м/м", self.load_units.label()),
            12..=14 if self.membrane_area => format!("{}/м²", self.load_units.label()),
            4..=5 | 8 | 12..=14 => format!("{}/м", self.load_units.label()),
            16 => "м²".into(),
            34 => format!("{}/м²", self.load_units.label()),
            33 => format!("{}/м³", self.load_units.label()),
            26..=29 => "см²/м".into(),
            30..=31 => "мм".into(),
            32 => "—".into(),
            18..=20 => format!("{}·м", self.load_units.label()),
            7 => format!("{}/м²", self.load_units.label()),
            _ => self.load_units.label().into(),
        };
        if i == 6 && self.sum_reactions {
            format!("Суммарные реакции Rz по опорам, {unit}")
        } else if i == DISPLACEMENT_NORM_FIELD && self.displacement_axes != 0 {
            let axes = ['X', 'Y', 'Z']
                .into_iter()
                .enumerate()
                .filter_map(|(axis, label)| {
                    (self.displacement_axes & (1 << axis) != 0).then_some(label)
                })
                .collect::<String>();
            format!("{} {axes}, {unit}", FIELD_NAMES[i])
        } else {
            format!("{}, {unit}", FIELD_NAMES[i])
        }
    }
    pub(super) fn support_node_indices(&self) -> std::collections::BTreeSet<usize> {
        self.fe_mesh
            .as_ref()
            .map(|m| {
                m.support_boundary
                    .values()
                    .flatten()
                    .copied()
                    .filter(|&n| self.spatial.enabled || self.mesh_node_on_floor(n))
                    .collect()
            })
            .unwrap_or_default()
    }
    pub(super) fn sync_load_scale(&mut self) {
        if self.scale_is_saved()
            || !matches!(self.result_field, 7..=9)
            || self.selected_result().is_none()
        {
            return;
        }
        let key = self.view_key();
        if self.load_scale_key == Some(key) {
            return;
        }
        let values = if self.result_field == 7 {
            let factor = self.force_result_factor();
            let mut seen = std::collections::HashSet::new();
            self.selected_result()
                .unwrap()
                .loads_kn_m2
                .iter()
                .enumerate()
                .filter(|(i, _)| self.area_load_visible(*i))
                .map(|(_, v)| *v * factor)
                .filter(|value| value.is_finite() && seen.insert(value.to_bits()))
                .collect()
        } else {
            self.load_mosaic_shapes().iter().map(|s| s.value).collect()
        };
        self.result_scales[self.result_field].sync_discrete_values(values);
        self.load_scale_key = Some(key);
    }
    pub(super) fn sync_variant_scale(&mut self) {
        self.sync_saved_scale();
    }
    pub(super) fn result_scale_controls(&mut self, ui: &mut egui::Ui) {
        if let Some(range) = self
            .displayed_shape(ui.ctx(), true)
            .and_then(|view| view.active_sensitivity().map(|field| field.range))
        {
            egui::CollapsingHeader::new("Шкала").show(ui, |ui| {
                ui.label("Чувствительность устойчивости, %");
                self.buckling_scale
                    .ui_with_header(ui, range, self.display_text, |ui| {
                        mosaic_mode_control(ui, "buckling_display_mode", &mut self.buckling_iso);
                    });
            });
            return;
        }
        if let Some((quantity, range)) = self
            .displayed_shape(ui.ctx(), false)
            .and_then(|view| view.vibration_values)
        {
            egui::CollapsingHeader::new("Шкала").show(ui, |ui| {
                ui.label(quantity.label());
                self.shape_scale
                    .ui_with_header(ui, range, self.display_text, |ui| {
                        mosaic_mode_control(ui, "shape_display_mode", &mut self.shape_iso);
                    });
            });
            return;
        }
        if let Some(index) = self.length_scale_index() {
            egui::CollapsingHeader::new("Шкала").show(ui, |ui| {
                let entries = self.length_mosaic_entries();
                ui.label(if index == 0 {
                    "Расчётные длины, м"
                } else {
                    "Коэффициенты μ"
                });
                if entries.is_empty() {
                    ui.label("Нет видимых расчётных длин.");
                    return;
                }
                let range = crate::plan_trace::result_scale::range(entries.iter().map(|e| e.3));
                ui.push_id(("effective_length_scale", index), |ui| {
                    self.length_scales[index].ui_with_header(ui, range, self.display_text, |ui| {
                        ui.checkbox(&mut self.show_result_values, "Значения");
                    });
                });
            });
            return;
        }
        if self.schema_scale_controls(ui) {
            return;
        }
        if self.punching_view.is_some() {
            self.punching_scale_controls(ui);
            return;
        }
        self.sync_load_scale();
        self.sync_variant_scale();
        egui::CollapsingHeader::new("Шкала")
            .default_open(false)
            .show(ui, |ui| {
                let Some(_case) = self.selected_result() else {
                    ui.label("Шкала доступна после расчёта.");
                    return;
                };
                let auto = if self.spatial.enabled {
                    self.spatial_result_range()
                } else {
                    self.result_range_cached()
                };
                let Some(auto) = auto else {
                    return;
                };
                ui.push_id(self.result_field, |ui| {
                    self.result_scales[self.result_field]
                        .set_reinforcement_mode((26..=29).contains(&self.result_field));
                    let mut scale = std::mem::take(&mut self.result_scales[self.result_field]);
                    scale.ui_with_header(ui, auto, self.display_text, |ui| {
                        self.result_display_controls(ui)
                    });
                    self.result_scales[self.result_field] = scale;
                    self.record_scale_action();
                });
            });
    }
    pub(super) fn paint_result_annotations(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pointer: Option<Pos2>,
    ) {
        self.schema_legend(painter, rect);
        if !self.show_results {
            return;
        }
        if self.punching_view.is_some() {
            self.paint_punching_results(painter, rect);
            return;
        }
        if (17..=22).contains(&self.result_field) {
            self.paint_beam_annotations(painter, rect, pointer);
            return;
        }
        if self.result_field == 6 && self.sum_reactions {
            self.paint_support_reactions(painter, rect, pointer);
            return;
        }
        if matches!(self.result_field, 8 | 9) {
            self.paint_load_mosaic_annotations(painter, rect, pointer);
            return;
        }
        let Some(mesh) = &self.fe_mesh else {
            return;
        };
        let Some(case) = self.selected_result() else {
            return;
        };
        let auto = self.result_range_cached().unwrap();
        let scale = &self.result_scales[self.result_field];
        let factor = self.result_display_factor();
        let support_nodes = self.support_node_indices();
        if self.show_results && self.point_displacements() {
            for (node, data) in mesh.nodes.iter().enumerate() {
                if !self.mesh_node_on_floor(node) {
                    continue;
                }
                let point = self.camera.screen(data.xy, rect);
                if !rect.contains(point) {
                    continue;
                }
                if let Some(color) = scale.color(self.result_node_value(case, node), auto) {
                    painter.circle_filled(point, self.point_size * 0.5, color);
                }
            }
        }
        if self.show_results && self.result_field == 6 {
            for &i in &support_nodes {
                let p = self.camera.screen(mesh.nodes[i].xy, rect);
                if rect.contains(p) {
                    if let Some(color) = scale.color(case.reaction_kn[i] * factor, auto) {
                        painter.circle_filled(p, self.point_size * 0.5, color);
                    }
                }
            }
        }

        if self.show_result_values && self.show_results {
            let mut occupied = BTreeMap::<(i32, i32), Vec<Rect>>::new();
            let positions = self.result_label_positions();
            for &(xy, v) in positions.iter() {
                let p = self.camera.screen(xy, rect);
                if !rect.contains(p) || scale.color(v, auto).is_none() {
                    continue;
                }
                let text = self.display_text.number(v);
                let size = painter
                    .layout_no_wrap(
                        text.clone(),
                        egui::FontId::proportional(self.display_text.size),
                        self.display_text.ink(),
                    )
                    .size()
                    + egui::vec2(8., 4.);
                let candidate = Rect::from_min_size(p - size * 0.5, size);
                let bounds = candidate.expand(2.);
                let lo = (
                    (bounds.left() / 64.).floor() as i32,
                    (bounds.top() / 64.).floor() as i32,
                );
                let hi = (
                    (bounds.right() / 64.).floor() as i32,
                    (bounds.bottom() / 64.).floor() as i32,
                );
                let overlaps = (lo.0..=hi.0).any(|x| {
                    (lo.1..=hi.1).any(|y| {
                        occupied
                            .get(&(x, y))
                            .is_some_and(|items| items.iter().any(|r| r.intersects(bounds)))
                    })
                });
                if overlaps {
                    continue;
                }
                let placed = result_label(painter, candidate.min, text, rect, self.display_text);
                let actual = placed.expand(2.);
                for x in
                    (actual.left() / 64.).floor() as i32..=(actual.right() / 64.).floor() as i32
                {
                    for y in
                        (actual.top() / 64.).floor() as i32..=(actual.bottom() / 64.).floor() as i32
                    {
                        occupied.entry((x, y)).or_default().push(placed);
                    }
                }
            }
        }
        if self.show_results {
            scale.legend(painter, rect, auto, self.display_text);
        }
        let Some(p) = pointer else {
            return;
        };
        if let Some(text) = self.hover_result(p, rect) {
            result_label(
                painter,
                p + egui::vec2(16., 18.),
                text,
                rect,
                self.display_text,
            );
        }
    }
    fn hover_result(&self, p: Pos2, rect: Rect) -> Option<String> {
        if self.punching_view.is_some() {
            return None;
        }
        if self.result_field == 6 && self.sum_reactions {
            return self.support_reaction_hover(p, rect);
        }
        if matches!(self.result_field, 8 | 9) {
            return self.load_mosaic_hover(p, rect);
        }
        let mesh = self.fe_mesh.as_ref()?;
        let case = self.selected_result()?;
        let world = self.camera.world(p, rect);
        let format_value = |location: String, value: f64| {
            format!(
                "{}\n{location}\nЗначение: {}",
                self.result_caption(),
                self.display_text.number(value)
            )
        };
        if self.point_displacements() {
            let (index, _) = mesh
                .nodes
                .iter()
                .enumerate()
                .filter(|(i, _)| self.mesh_node_on_floor(*i))
                .map(|(i, n)| (i, self.camera.screen(n.xy, rect).distance(p)))
                .filter(|(_, distance)| *distance <= (self.point_size * 0.5).max(9.))
                .min_by(|a, b| a.1.total_cmp(&b.1))?;
            let node = &mesh.nodes[index];
            return Some(format_value(
                format!(
                    "Узел {} · X {:.1} · Y {:.1} мм",
                    node.id, node.xy[0], node.xy[1]
                ),
                self.result_node_value(case, index),
            ));
        }
        // Reactions are discrete nodal quantities; hit-test only their markers.
        // No highlighting or snapping is applied to any result tooltip.
        if self.result_field == 6 {
            let support = self.support_node_indices();
            let (i, _) = mesh
                .nodes
                .iter()
                .enumerate()
                .filter(|(i, _)| support.contains(i))
                .map(|(i, n)| (i, self.camera.screen(n.xy, rect).distance(p)))
                .filter(|(_, d)| *d <= 9.)
                .min_by(|a, b| a.1.total_cmp(&b.1))?;
            let n = &mesh.nodes[i];
            return Some(format_value(
                format!("Узел {} · X {:.1} · Y {:.1} мм", n.id, n.xy[0], n.xy[1]),
                case.reaction_kn[i] * self.result_display_factor(),
            ));
        }
        let geometry = self.result_geometry()?;
        let (i, l) = geometry.candidates(world).into_iter().find_map(|i| {
            let e = &mesh.elements[i];
            weights(e.nodes.map(|n| mesh.nodes[n].xy), world).map(|l| (i, l))
        })?;
        let e = &mesh.elements[i];
        if (26..=32).contains(&self.result_field) {
            if let Some(cell) = self
                .reinforcement
                .result
                .as_ref()
                .and_then(|r| r.cells.get(i))
            {
                let verified = if self.result_field <= 29 {
                    cell.verified
                } else {
                    cell.service_verified
                };
                if !verified {
                    return Some(format!(
                        "КЭ {} · НДМ не подтверждено: {}",
                        e.id, cell.diagnostic
                    ));
                }
            }
        }
        let value = if self.result_field == 15 || displacement_field(self.result_field) {
            if let Some(quad) = geometry.quad_by_display.get(&e.id).map(|&q| &mesh.quads[q]) {
                let qi = *geometry.quad_by_display.get(&e.id)?;
                let element = geometry.quads[qi].as_ref()?;
                let natural = element.natural(world)?;
                let (shape, _) = crate::plan_trace::fem::mitc4::shape(natural);
                (0..4)
                    .map(|j| shape[j] * self.result_node_value(case, quad.nodes[j]))
                    .sum()
            } else {
                (0..3)
                    .map(|j| l[j] * self.result_node_value(case, e.nodes[j]))
                    .sum()
            }
        } else if let Some(values) = self.smoothed_efforts() {
            if let Some(quad) = geometry.quad_by_display.get(&e.id).map(|&q| &mesh.quads[q]) {
                let corners = super::super::isofields::quad_corner_values(
                    mesh,
                    quad,
                    &values,
                    &geometry.ids,
                )?;
                let qi = *geometry.quad_by_display.get(&e.id)?;
                let element = geometry.quads[qi].as_ref()?;
                let (n, _) = crate::plan_trace::fem::mitc4::shape(element.natural(world)?);
                (0..4).map(|j| n[j] * corners[j]).sum()
            } else {
                (0..3).map(|j| l[j] * values[i][j]).sum()
            }
        } else {
            self.result_element_value(case, i) * self.result_display_factor()
        };
        if matches!(self.result_field, 33 | 34) && !value.is_finite() {
            return None;
        }
        Some(format_value(
            format!(
                "КЭ {} · X {:.1} · Y {:.1} мм",
                geometry
                    .quad_by_display
                    .get(&e.id)
                    .map(|&q| &mesh.quads[q])
                    .map_or(e.id, |q| q.id),
                world[0],
                world[1]
            ),
            value,
        ))
    }
    fn calculation_contours(&self) -> Vec<Contour> {
        self.contours
            .iter()
            .filter(|c| {
                c.kind.affects_calculation() || crate::plan_trace::fem::cut_forces::is_cut(c)
            })
            .cloned()
            .map(|mut c| {
                c.placement.movement_locked = false;
                if let Some(load) = &mut c.load {
                    load.name.clear();
                }
                // Edge switches are postprocessing; retain every side in the
                // solve snapshot so any mask can be selected without a solve.
                if c.kind == Kind::Punching {
                    if let Some(r) = &mut c.refinement {
                        r.inactive_edges.clear();
                    }
                }
                c
            })
            .collect()
    }
    pub(super) fn analysis_key(&self) -> Input {
        Input {
            effective_lengths: false,
            modal_weights: None,
            nonlinear: self.nonlinear_request().ok().flatten(),
            buckling: self.buckling_request().ok().flatten(),
            building: self.building_layout().map(|mut b| {
                b.owners.retain(|id, _| {
                    self.contours.iter().any(|c| {
                        c.id == *id
                            && (c.kind.affects_calculation()
                                || crate::plan_trace::fem::cut_forces::is_cut(c))
                    })
                });
                b
            }),
            contours: self.calculation_contours(),
            materials: self.materials.clone(),
            cases: self
                .load_cases
                .iter()
                .map(|c| {
                    (
                        c.id,
                        c.kind == crate::plan_trace::loads::LoadType::SelfWeight,
                    )
                })
                .collect(),
            g: self.gravity,
            step: self.mesh_built_size,
            element_type: self.element_type,
            support_height: self.support_height.clone(),
        }
    }
    pub(super) fn poll_analysis(&mut self) {
        if self.analysis_input.as_ref().is_some_and(|k| {
            k.contours != self.calculation_contours()
                || k.materials != self.materials
                || k.g != self.gravity
                || k.step != self.mesh_built_size
                || k.element_type != self.element_type
                || k.support_height != self.support_height
                || k.buckling != self.buckling_request().ok().flatten()
                || !k.cases.iter().copied().eq(self.load_cases.iter().map(|c| {
                    (
                        c.id,
                        c.kind == crate::plan_trace::loads::LoadType::SelfWeight,
                    )
                }))
        }) || self.fe_mesh.is_none()
        {
            if self.analysis_job.is_some() || self.alternative_job.is_some() {
                if let Some(cancel) = &self.solver_cancel {
                    cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                self.alternative_job = None;
                #[cfg(target_arch = "wasm32")]
                {
                    self.solver_worker = None;
                }
            }
            self.input_locked = false;
            self.analysis = None;
            self.combination_cache = None;
            self.analysis_job = None;
            self.analysis_input = None;
        }
        let Some(rx) = &self.analysis_job else {
            return;
        };
        match rx.try_recv() {
            Ok(result) => {
                self.analysis_job = None;
                match result {
                    Ok(r) => {
                        self.status =
                            "Расчёт завершён. w: вниз +; моменты в глобальных осях XY".into();
                        self.result_generation = self.result_generation.wrapping_add(1);
                        self.analysis = Some(r);
                        self.cancel();
                        self.remember_manual_locks();
                        self.input_locked = true;
                        self.combination_cache = None;
                    }
                    Err(e) => {
                        self.input_locked = false;
                        self.status = e;
                    }
                }
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.analysis_job = None;
                self.input_locked = false;
                self.status = "Расчётный поток прерван".into();
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }
    pub(super) fn prepare_load_mosaic_smoke(&mut self, field: usize) -> bool {
        if self.mesh_job.is_some() || self.fe_mesh.is_none() {
            return false;
        }
        for c in self.contours.iter().filter(|c| c.kind == Kind::Slab) {
            self.materials.entry(c.id).or_default();
        }
        let key = self.analysis_key();
        let result = crate::plan_trace::fem::solve_with_support_height(
            self.fe_mesh.as_ref().unwrap(),
            &key.contours,
            &key.materials,
            &self.load_cases,
            key.g,
            key.step,
            key.element_type,
            &key.support_height,
        )
        .expect("result smoke solve");
        self.analysis_input = Some(key);
        self.analysis = Some(result);
        self.result_generation = self.result_generation.wrapping_add(1);
        self.result_field = field;
        if (26..=32).contains(&field) {
            self.prepare_reinforcement_smoke();
        }
        self.input_locked = true;
        true
    }
    pub(super) fn result_button(
        &mut self,
        ui: &mut egui::Ui,
        label: &str,
        field: usize,
        summed: Option<bool>,
    ) {
        let selected = self.punching_view.is_none()
            && self.result_field == field
            && summed.is_none_or(|value| self.sum_reactions == value);
        if ui
            .add(egui::Button::new(label).selected(selected && self.show_results))
            .on_hover_text(self.result_field_label(field))
            .clicked()
        {
            self.set_result_view(ui.ctx(), field, !(selected && self.show_results));
            if let Some(value) = summed {
                self.sum_reactions = value;
            }
        }
    }
    pub(super) fn set_result_view(&mut self, ctx: &egui::Context, field: usize, show: bool) {
        if self.schema_analysis.mode.take().is_some() {
            self.slab_cache = None;
            self.invalidate_scene_buffers();
            self.spatial.cache_reset();
        }
        self.show_results = show;
        self.punching_view = None;
        self.length_view = 0;
        ctx.data_mut(|d| d.remove::<ModalView>(egui::Id::new("modal_view")));
        self.result_field = field;
    }
    fn displacement_buttons(&mut self, ui: &mut egui::Ui, has_shell: bool) {
        let mut sum = self.result_field == DISPLACEMENT_NORM_FIELD;
        let available = if has_shell { 0b111 } else { 0b100 };
        if ui
            .checkbox(&mut sum, "Суммарные")
            .on_hover_text("Модуль перемещения по выбранным компонентам X, Y и Z")
            .changed()
        {
            if sum {
                self.displacement_axes &= available;
                if self.displacement_axes == 0 {
                    self.displacement_axes = available;
                }
                self.set_result_view(ui.ctx(), DISPLACEMENT_NORM_FIELD, true);
            } else {
                let field = [(0b100, 0), (0b001, 10), (0b010, 11)]
                    .into_iter()
                    .find_map(|(mask, field)| {
                        (self.displacement_axes & available & mask != 0).then_some(field)
                    })
                    .unwrap_or(0);
                self.set_result_view(ui.ctx(), field, true);
            }
        }
        for (axis, (label, field)) in [("X", 10), ("Y", 11), ("Z", 0)].into_iter().enumerate() {
            let mask = 1 << axis;
            if available & mask == 0 {
                continue;
            }
            if sum {
                if ui
                    .add(egui::Button::new(label).selected(self.displacement_axes & mask != 0))
                    .clicked()
                {
                    self.displacement_axes ^= mask;
                    self.set_result_view(
                        ui.ctx(),
                        DISPLACEMENT_NORM_FIELD,
                        self.displacement_axes != 0,
                    );
                }
            } else {
                self.result_button(ui, label, field, None);
            }
        }
    }
    pub(super) fn result_view_controls(&mut self, ui: &mut egui::Ui) {
        let calculated = self.analysis.is_some();
        let has_shell = self.result_metadata().is_some_and(|r| r.shell.is_some());
        let has_beams = self
            .result_metadata()
            .and_then(|r| r.shell.as_ref())
            .is_some_and(|s| !s.beam_forces.is_empty());
        let has_supports = !self.support_node_indices().is_empty();
        egui::CollapsingHeader::new("Результаты узлы")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    if calculated {
                        self.displacement_buttons(ui, has_shell);
                    }
                    if self
                        .result_metadata()
                        .is_some_and(|r| !r.rotations_rad.is_empty())
                    {
                        self.result_button(ui, "Ux", 23, None);
                        self.result_button(ui, "Uy", 24, None);
                        if has_shell {
                            self.result_button(ui, "Uz", 25, None);
                        }
                    }
                    if has_supports {
                        self.result_button(ui, "L", 15, None);
                    }
                });
                if calculated && self.selected_result().is_some() && self.fe_mesh.is_some() {
                    ui.horizontal(|ui| {
                        let id = egui::Id::new("static_deformation");
                        let (mut enabled, mut scale) = ui
                            .ctx()
                            .data(|data| data.get_temp::<(bool, f64)>(id).unwrap_or((false, 100.)));
                        if ui
                            .selectable_label(enabled, "Δ")
                            .on_hover_text("Деформированная схема в 3D")
                            .clicked()
                        {
                            enabled = !enabled;
                            if enabled {
                                ui.ctx().data_mut(|d| {
                                    d.remove::<ModalView>(egui::Id::new("modal_view"))
                                });
                            }
                        }
                        ui.add(
                            egui::DragValue::new(&mut scale)
                                .range(1. ..=10000.)
                                .prefix("× "),
                        )
                        .on_hover_text("Масштаб перемещений в 3D");
                        Self::deformation_display_mode(ui);
                        ui.ctx()
                            .data_mut(|data| data.insert_temp(id, (enabled, scale)));
                    });
                }
            });
        if has_beams {
            egui::CollapsingHeader::new("Стержни")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal(|ui| {
                        for (label, field) in [("N", 17), ("Qz", 22), ("Qy", 21)] {
                            self.result_button(ui, label, field, None);
                        }
                    });
                    ui.horizontal(|ui| {
                        for (label, field) in [("Mx", 18), ("My", 19), ("Mz", 20)] {
                            self.result_button(ui, label, field, None);
                        }
                    });
                });
        }
        if calculated {
            egui::CollapsingHeader::new("Пластины")
                .default_open(false)
                .show(ui, |ui| {
                    if has_shell {
                        ui.horizontal(|ui| {
                            for (label, field) in [("Nx", 12), ("Ny", 13), ("Nxy", 14)] {
                                self.result_button(ui, label, field, None);
                            }
                            egui::ComboBox::from_id_salt("membrane_result_units")
                                .width(42.)
                                .selected_text(if self.membrane_area { "A" } else { "L" })
                                .show_ui(ui, |ui| {
                                    ui.selectable_value(&mut self.membrane_area, false, "L")
                                        .on_hover_text("Погонные усилия, сила/м");
                                    ui.selectable_value(&mut self.membrane_area, true, "A")
                                        .on_hover_text("Усилия / толщина, сила/м²");
                                });
                        });
                    }
                    ui.horizontal(|ui| {
                        for (label, field) in [("Mx", 1), ("My", 2), ("Mxy", 3)] {
                            self.result_button(ui, label, field, None);
                        }
                    });
                    ui.horizontal(|ui| {
                        for (label, field) in [("Qx", 4), ("Qy", 5)] {
                            self.result_button(ui, label, field, None);
                        }
                    });
                });
        }
        if self.fe_mesh.is_some() {
            egui::CollapsingHeader::new("Реакции")
                .default_open(false)
                .show(ui, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        if calculated && has_supports {
                            self.result_button(ui, "По узлам", 6, Some(false));
                            self.result_button(ui, "Суммарные", 6, Some(true));
                        }
                        if has_supports {
                            self.result_button(ui, "Грузовые площади", 16, None);
                        }
                        self.result_button(ui, "C1", 33, None);
                        if calculated {
                            self.result_button(ui, "Реакция основания", 34, None);
                        }
                    });
                });
        }
        self.soil_pressure_results(ui);
        egui::CollapsingHeader::new("Нагрузки")
            .id_salt("load_result_fields")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    self.result_button(ui, "qa", 7, None);
                    if self.has_linear_load_results() {
                        self.result_button(ui, "ql", 8, None);
                    }
                    if self.contours.iter().any(|c| c.kind == Kind::LoadPoint) {
                        self.result_button(ui, "qp", 9, None);
                    }
                });
            });
        if calculated {
            self.buckling_results(ui);
            self.modal_and_length_results(ui);
            self.reinforcement_results(ui);
        }
        if ((10..=14).contains(&self.result_field) && !has_shell)
            || ((17..=22).contains(&self.result_field) && !has_beams)
        {
            self.result_field = 0;
        }

        if calculated {
            self.punching_results_controls(ui);
        }
        if matches!(self.result_field, 7..=9) {
            ui.small("Только выбранный тип нагрузки. Линейные — интенсивность на линии, сосредоточенные — сила в точке.");
        }
        if self.result_field == 6 {
            ui.small("Rz: положительное направление вниз.");
            if self.element_type == crate::plan_trace::fem::ElementType::MindlinCr {
                ui.small("P2/CR: реакции промежуточных узлов ребер приведены к исходным узлам.");
            }
        }
        if let Some([min, max]) = self.result_extrema_cached() {
            ui.label(format!(
                "{}min {} · max {}",
                if self.iso_efforts && self.result_has_element_field() {
                    "На виде: "
                } else {
                    ""
                },
                self.display_text.number(min),
                self.display_text.number(max)
            ));
        }
    }
    fn deformation_display_mode(ui: &mut egui::Ui) {
        let id = egui::Id::new("deformation_only");
        let mut only = ui.ctx().data(|d| d.get_temp::<bool>(id)).unwrap_or(false);
        egui::ComboBox::from_id_salt("deformation_display")
            .selected_text(if only {
                "Деформированная"
            } else {
                "Исходная + деф."
            })
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut only, true, "Деформированная");
                ui.selectable_value(&mut only, false, "Исходная + деф.");
            });
        ui.ctx().data_mut(|d| d.insert_temp(id, only));
    }

    pub(super) fn displayed_shape(&self, ctx: &egui::Context, buckling: bool) -> Option<ModalView> {
        ctx.data(|d| d.get_temp::<ModalView>(egui::Id::new("modal_view")))
            .filter(|view| {
                view.source.is_buckling() == buckling
                    && view.mesh_generation == self.mesh_generation
                    && view.result_generation == self.result_generation
                    && view.stiffness_variant == self.stiffness_variant
            })
    }

    fn buckling_results(&mut self, ui: &mut egui::Ui) {
        let Some(result) = self.analysis.as_ref().and_then(|r| r.buckling.as_ref()) else {
            return;
        };
        let view_id = egui::Id::new("modal_view");
        let mut view = self.displayed_shape(ui.ctx(), true);
        let mut changed_view = false;
        ui.collapsing("Формы потери устойчивости", |ui| {
            let result = match result {
                Ok(result) => result,
                Err(error) => {
                    ui.label(error);
                    return;
                }
            };
            ui.label(&result.caption);
            if result.modes.is_empty() {
                ui.label(format!("Положительные формы при КЗУ ≤ {} не найдены", self.display_text.number(result.max_factor)));
                return;
            }
            ui.small(format!("Найдено форм: {} из {} · КЗУ ≤ {}", result.modes.len(), result.requested, self.display_text.number(result.max_factor)));
            let mut selected = view
                .as_ref()
                .and_then(|v| v.number.checked_sub(1))
                .filter(|&i| i < result.modes.len());
            let before = selected;
            egui::ComboBox::from_id_salt("buckling_mode")
                .selected_text(selected.map_or_else(|| "Выберите форму".into(), |i| {
                    format!("Форма {} · КЗУ {:.4}", i + 1, result.modes[i].factor)
                }))
                .show_ui(ui, |ui| {
                    for (i, mode) in result.modes.iter().enumerate() {
                        ui.selectable_value(&mut selected, Some(i), format!("{} · КЗУ {:.4}", i + 1, mode.factor));
                    }
                });
            if let Some(i) = selected.filter(|_| selected != before) {
                let mode = &result.modes[i];
                view = Some(ModalView {
                    stiffness_variant: self.stiffness_variant,
                    result_generation: self.result_generation,
                    mesh_generation: self.mesh_generation,
                    shape: std::sync::Arc::new(mode.shape.clone()),
                    amplitude: view.as_ref().map_or(DEFAULT_SHAPE_AMPLITUDE_MM, |v| v.amplitude),
                    number: i + 1,
                    source: ShapeSource::Buckling(mode.factor),
                    zero_translation: mode.shape.iter().flatten().all(|v| v.abs() <= 1e-12),
                    playing: view.as_ref().is_some_and(|v| v.playing),
                    started: ui.input(|i| i.time),
                    paused_at: None,
                    vibration_values: None,
                    sensitivity: mode.sensitivity.as_ref().and_then(BucklingColors::new),
                    show_sensitivity: view.as_ref().is_some_and(|v| v.show_sensitivity),
                });
                changed_view = true;
            }
            if let Some(active) = &mut view {
                if active.sensitivity.is_some() {
                    ui.checkbox(&mut active.show_sensitivity, "Чувствительность")
                        .on_hover_text("Вклад КЭ в выбранную форму. Плюс снижает КЗУ, минус стабилизирует.");
                } else {
                    ui.small("Чувствительность для этой формы недоступна.");
                }
                if active.zero_translation {
                    ui.small("В этой форме оси узлов не смещаются. Вращения сечений на деформированной схеме не показаны.");
                }
                let mut hide = false;
                ui.horizontal_wrapped(|ui| {
                    Self::shape_amplitude_controls(ui, active);
                    ui.push_id("buckling_deformation_display", Self::deformation_display_mode);
                    hide = ui.button("Скрыть").clicked();
                });
                if hide {
                    view = None;
                    changed_view = true;
                }
            }
        });
        if let Some(view) = view {
            ui.ctx().data_mut(|d| {
                if changed_view {
                    let static_id = egui::Id::new("static_deformation");
                    if let Some((_, scale)) = d.get_temp::<(bool, f64)>(static_id) {
                        d.insert_temp(static_id, (false, scale));
                    }
                }
                d.insert_temp(view_id, view);
            });
            if changed_view {
                self.length_view = 0;
                self.show_results = false;
                self.punching_view = None;
                self.spatial.enabled = true;
                if self.schema_analysis.mode.take().is_some() {
                    self.slab_cache = None;
                    self.invalidate_scene_buffers();
                    self.spatial.cache_reset();
                }
            }
        } else if changed_view {
            ui.ctx().data_mut(|d| d.remove::<ModalView>(view_id));
        }
    }

    fn shape_amplitude_controls(ui: &mut egui::Ui, view: &mut ModalView) {
        ui.label("Амплитуда, мм");
        ui.add(egui::DragValue::new(&mut view.amplitude).range(0. ..=100000.));
        Self::shape_play_button(ui, view);
    }

    fn shape_play_button(ui: &mut egui::Ui, view: &mut ModalView) {
        if ui
            .button(if view.playing { "Стоп" } else { "Пуск" })
            .clicked()
        {
            view.playing = !view.playing;
            view.started = ui.input(|i| i.time);
            view.paused_at = None;
        }
    }
    fn modal_and_length_results(&mut self, ui: &mut egui::Ui) {
        if self.analysis.as_ref().is_some_and(|r| r.modal.is_some()) {
            ui.collapsing("Формы колебаний", |ui| {
                if let Some(result) = self.analysis.as_ref().and_then(|r| r.modal.as_ref()) {
                    match result {
                        Err(error) => {
                            ui.label(error);
                        }
                        Ok((mass, modes)) => {
                            ui.label(format!("Масса: {mass:.3} т"));
                            let view_id = egui::Id::new("modal_view");
                            let amplitude_id = egui::Id::new("vibration_amplitude");
                            let quantity_id = egui::Id::new("vibration_components");
                            let (mut total, mut axes) = ui
                                .ctx()
                                .data(|d| d.get_temp::<(bool, u8)>(quantity_id))
                                .unwrap_or_default();
                            ui.horizontal_wrapped(|ui| {
                                if ui.checkbox(&mut total, "Суммарные").clicked()
                                    && !total
                                    && axes != 0
                                {
                                    axes = 1 << axes.trailing_zeros();
                                }
                                for (axis, label) in ["X", "Y", "Z"].into_iter().enumerate() {
                                    let bit = 1 << axis;
                                    if ui.selectable_label(axes & bit != 0, label).clicked() {
                                        axes = if total {
                                            axes ^ bit
                                        } else if axes == bit {
                                            0
                                        } else {
                                            bit
                                        };
                                    }
                                }
                            });
                            let quantity = (axes != 0).then(|| {
                                if total {
                                    VibrationQuantity::Magnitude(axes)
                                } else {
                                    VibrationQuantity::Axis(axes.trailing_zeros() as usize)
                                }
                            });
                            let mut active = self.displayed_shape(ui.ctx(), false);
                            let mut amplitude = ui
                                .ctx()
                                .data(|d| d.get_temp::<f64>(amplitude_id))
                                .unwrap_or(DEFAULT_SHAPE_AMPLITUDE_MM);
                            let mut hide = false;
                            ui.horizontal_wrapped(|ui| {
                                ui.label("Амплитуда, мм");
                                ui.add(egui::DragValue::new(&mut amplitude).range(0. ..=100000.));
                                if let Some(view) = &mut active {
                                    view.amplitude = amplitude;
                                    Self::shape_play_button(ui, view);
                                } else {
                                    ui.add_enabled(false, egui::Button::new("Пуск"));
                                }
                                hide = ui
                                    .add_enabled(active.is_some(), egui::Button::new("Скрыть"))
                                    .clicked();
                            });
                            let mut chosen = None;
                            let mut cumulative = [0.; 3];
                            let percent =
                                |value| if *mass > 0. { value / *mass * 100. } else { 0. };
                            egui::Grid::new("modal_effective_mass")
                                .striped(true)
                                .show(ui, |ui| {
                                    for title in ["Форма", "X, т", "Y, т", "Z, т"] {
                                        ui.label(title);
                                    }
                                    ui.end_row();
                                    for (index, mode) in modes.iter().enumerate() {
                                        let selected = active
                                            .as_ref()
                                            .is_some_and(|view| view.number == index + 1);
                                        if ui
                                            .selectable_label(
                                                selected,
                                                format!("{} · {:.4} Гц", index + 1, mode.hz),
                                            )
                                            .clicked()
                                        {
                                            chosen = Some(index);
                                        }
                                        for (axis, value) in
                                            mode.effective_mass_t.into_iter().enumerate()
                                        {
                                            cumulative[axis] += value;
                                            ui.label(format!("{value:.2}")).on_hover_text(format!(
                                                "Доля: {:.2}% · накоплено: {:.2}%",
                                                percent(value),
                                                percent(cumulative[axis])
                                            ));
                                        }
                                        ui.end_row();
                                    }
                                    ui.label("Σ, %");
                                    for value in cumulative {
                                        ui.label(format!("{:.1}", percent(value)));
                                    }
                                    ui.end_row();
                                });
                            if let Some(i) = chosen {
                                let mode = &modes[i];
                                if active.as_ref().is_none_or(|view| view.number != i + 1) {
                                    let now = ui.input(|input| input.time);
                                    active = Some(ModalView {
                                        mesh_generation: self.mesh_generation,
                                        stiffness_variant: self.stiffness_variant,
                                        result_generation: self.result_generation,
                                        shape: std::sync::Arc::new(mode.shape.clone()),
                                        amplitude,
                                        number: i + 1,
                                        source: ShapeSource::Vibration(mode.hz),
                                        zero_translation: false,
                                        playing: active.as_ref().is_some_and(|view| view.playing),
                                        started: active.as_ref().map_or(now, |view| view.started),
                                        paused_at: active.as_ref().and_then(|view| view.paused_at),
                                        vibration_values: None,
                                        sensitivity: None,
                                        show_sensitivity: false,
                                    });
                                }
                                let static_id = egui::Id::new("static_deformation");
                                ui.ctx().data_mut(|d| {
                                    if let Some((_, scale)) = d.get_temp::<(bool, f64)>(static_id) {
                                        d.insert_temp(static_id, (false, scale));
                                    }
                                });
                                self.length_view = 0;
                                self.show_results = false;
                                self.punching_view = None;
                                self.spatial.enabled = true;
                                if self.schema_analysis.mode.take().is_some() {
                                    self.slab_cache = None;
                                    self.invalidate_scene_buffers();
                                    self.spatial.cache_reset();
                                }
                            }
                            if let Some(view) = &mut active {
                                view.set_vibration_quantity(quantity);
                            }
                            ui.ctx().data_mut(|d| {
                                d.insert_temp(amplitude_id, amplitude);
                                d.insert_temp(quantity_id, (total, axes));
                                if hide {
                                    d.remove::<ModalView>(view_id);
                                } else if let Some(view) = active {
                                    d.insert_temp(view_id, view);
                                }
                            });
                        }
                    }
                }
            });
        }
        if self
            .analysis
            .as_ref()
            .is_some_and(|r| r.lengths.as_ref().is_some_and(Result::is_ok))
        {
            ui.collapsing("Мозаика расчётных длин", |ui| {
                let method = self
                    .analysis_input
                    .as_ref()
                    .map_or(self.support_height.length_method, |key| {
                        key.support_height.length_method
                    });
                ui.small(format!(
                    "Упругая исходная схема · {}",
                    length_method_label(method)
                ));
                ui.horizontal_wrapped(|ui| {
                    for (value, label) in [(1, "L₀, м"), (2, "μ")] {
                        if ui
                            .selectable_label(self.length_view == value, label)
                            .clicked()
                        {
                            self.length_view = if self.length_view == value { 0 } else { value };
                            self.show_results = false;
                            self.punching_view = None;
                            if self.schema_analysis.mode.take().is_some() {
                                self.slab_cache = None;
                                self.invalidate_scene_buffers();
                                self.spatial.cache_reset();
                            }
                            ui.ctx()
                                .data_mut(|d| d.remove::<ModalView>(egui::Id::new("modal_view")));
                        }
                    }
                    egui::ComboBox::from_id_salt("length_axis")
                        .selected_text(if self.length_axis_y {
                            "Колонны · y"
                        } else {
                            "Колонны · z"
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut self.length_axis_y, true, "Колонны · y");
                            ui.selectable_value(&mut self.length_axis_y, false, "Колонны · z");
                        });
                });
                if method == crate::plan_trace::fem::shell::LengthMethod::Spatial {
                    self.spatial_length_diagnostics(ui);
                }
            });
        }
        if let Some(Err(error)) = self.analysis.as_ref().and_then(|r| r.lengths.as_ref()) {
            ui.label(format!("Расчётные длины: {error}"));
        }
    }

    fn spatial_length_diagnostics(&self, ui: &mut egui::Ui) {
        let Some(Ok(rows)) = self.analysis.as_ref().and_then(|r| r.lengths.as_ref()) else {
            return;
        };
        ui.collapsing("Критические силы и формы", |ui| {
            let height = ui.text_style_height(&egui::TextStyle::Body);
            egui::ScrollArea::vertical()
                .id_salt("spatial_length_diagnostics")
                .max_height(220.)
                .show_rows(ui, height, rows.len(), |ui, indices| {
                    for i in indices {
                        let row = &rows[i];
                        let mut label =
                            format!("АЖТ №{} · Z {:.3} м · {}", row.id, row.bottom_m, row.axis);
                        if let Some(force) = row.critical_kn {
                            label.push_str(&format!(" · Ncr {:.2} кН", force));
                        }
                        if let Some(length) = row.effective_m {
                            label.push_str(&format!(" · L₀ {:.3} м", length));
                        }
                        if let Some(reason) = &row.diagnostic {
                            label.push_str(" · ");
                            label.push_str(reason);
                        }
                        ui.add(egui::Label::new(&label).truncate())
                            .on_hover_text(label);
                    }
                });
        });
    }

    pub(super) fn length_scale_index(&self) -> Option<usize> {
        (!self.show_results && (1..=2).contains(&self.length_view))
            .then(|| usize::from(self.length_view - 1))
    }

    fn length_mosaic_entries(
        &self,
    ) -> Vec<(
        &crate::plan_trace::fem::shell::effective_length::Length,
        Point,
        Point,
        f64,
    )> {
        if self.length_scale_index().is_none() {
            return Vec::new();
        }
        let Some(Ok(rows)) = self.analysis.as_ref().and_then(|r| r.lengths.as_ref()) else {
            return Vec::new();
        };
        rows.iter()
            .filter_map(|row| {
                if matches!(row.axis.as_str(), "y" | "z") && (row.axis == "y") != self.length_axis_y
                {
                    return None;
                }
                let c = self.contours.iter().find(|c| c.id == row.id)?;
                if !self.contour_visible(c) {
                    return None;
                }
                let [a, b] = row
                    .segment
                    .unwrap_or([*c.points.first()?, *c.points.last()?]);
                let value = row.effective_m?
                    / if self.length_view == 2 {
                        row.length_m
                    } else {
                        1.
                    };
                value.is_finite().then_some((row, a, b, value))
            })
            .collect()
    }

    pub(super) fn length_world_colors(
        &self,
    ) -> BTreeMap<u64, Vec<(f64, f64, Color32, Option<[Point; 2]>)>> {
        let Some(index) = self.length_scale_index() else {
            return BTreeMap::new();
        };
        let entries = self.length_mosaic_entries();
        let range = crate::plan_trace::result_scale::range(entries.iter().map(|e| e.3));
        let scale = &self.length_scales[index];
        let mut colors = BTreeMap::<u64, Vec<_>>::new();
        for (row, _, _, value) in entries {
            if let Some(color) = scale.color(value, range) {
                colors.entry(row.id).or_default().push((
                    row.bottom_m * 1000.,
                    (row.bottom_m + row.length_m) * 1000.,
                    color,
                    row.segment,
                ));
            }
        }
        colors
    }
    pub(super) fn length_mosaic(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        projected: Option<&[(Pos2, f64)]>,
        occlusion: Option<&spatial_results::Occlusion>,
        project: impl Fn([f64; 3]) -> (Pos2, f64),
    ) -> Option<(usize, [f64; 2])> {
        let index = self.length_scale_index()?;
        let entries = self.length_mosaic_entries();
        if entries.is_empty() {
            return None;
        }
        let range = crate::plan_trace::result_scale::range(entries.iter().map(|e| e.3));
        let scale = &self.length_scales[index];
        let mut builder = projected.map(|points| spatial_depth::Builder::new(rect, points));
        let mut labels = Vec::new();
        for (row, a, b, value) in entries {
            let Some(ink) = scale.color(value, range) else {
                continue;
            };
            let z = row.bottom_m * 1000.;
            let h = row.length_m * 1000.;
            let pa = project([a[0], a[1], z]);
            let pb = project([b[0], b[1], z]);
            let center = if let Some(builder) = &mut builder {
                let top_a = project([a[0], a[1], z + h]);
                if a == b {
                    if !self.drawing_visibility.support_extents {
                        builder.line([pa, top_a], 5., ink);
                    }
                    (pa.0.lerp(top_a.0, 0.5), (pa.1 + top_a.1) * 0.5)
                } else {
                    let top_b = project([b[0], b[1], z + h]);

                    (pa.0.lerp(top_b.0, 0.5), (pa.1 + top_b.1) * 0.5)
                }
            } else if a != b {
                painter.line_segment([pa.0, pb.0], egui::Stroke::new(5_f32, ink));
                (pa.0.lerp(pb.0, 0.5), 0.)
            } else {
                painter.circle_filled(pa.0, self.point_size.max(4.), ink);
                (pa.0, 0.)
            };
            if self.show_result_values
                && rect.contains(center.0)
                && occlusion.is_none_or(|o| o.visible(center.0, center.1))
            {
                labels.push((center.0, value));
            }
        }
        if let Some(builder) = builder {
            spatial_depth::paint_layer(
                painter,
                rect,
                std::sync::Arc::new(builder.vertices),
                spatial_depth::Layer::EffectiveLengths,
            );
        }
        for (center, value) in labels {
            result_label(
                painter,
                center,
                self.display_text.number(value),
                rect,
                self.display_text,
            );
        }
        Some((index, range))
    }

    fn default_modal_source(&self) -> AnalysisLoadSource {
        self.combinations
            .iter()
            .find(|c| c.id == self.active_combination)
            .or_else(|| self.combinations.first())
            .map_or(AnalysisLoadSource::LoadCase(self.active_load), |c| {
                AnalysisLoadSource::Combination(c.id)
            })
    }

    pub(super) fn modal_weights_for_solve(
        &self,
    ) -> Result<Option<(Vec<(u64, f64)>, String)>, String> {
        if !self.building.enabled || !self.calculate_frequencies {
            return Ok(None);
        }
        self.modal_mass_options
            .source
            .unwrap_or_else(|| self.default_modal_source())
            .resolve(
                &self.load_cases,
                &self.combinations,
                self.modal_mass_options.variant,
            )
            .map(Some)
            .map_err(|error| format!("Собственные частоты: {error}"))
    }

    fn modal_settings(&mut self, ui: &mut egui::Ui) {
        let default_source = self.default_modal_source();
        let before = (self.calculate_frequencies, self.modal_mass_options);
        let enabled =
            self.building.enabled && self.analysis_job.is_none() && self.alternative_job.is_none();
        ui.add_enabled_ui(enabled, |ui| {
            ui.checkbox(
                &mut self.calculate_frequencies,
                "Собственные частоты и формы",
            )
            .on_hover_text("Вычисляет частоты и формы свободных колебаний при МКЭ-расчёте здания.");
            if self.building.enabled && self.calculate_frequencies {
                let displayed_source = self.modal_mass_options.source.unwrap_or(default_source);
                let mut source = displayed_source;
                analysis_load_controls(
                    ui,
                    "modal_mass",
                    &mut source,
                    &mut self.modal_mass_options.variant,
                    &self.load_cases,
                    &self.combinations,
                );
                if source != displayed_source
                    || self.calculate_frequencies != before.0
                    || self.modal_mass_options.variant != before.1.variant
                {
                    self.modal_mass_options.source = Some(source);
                }
            }
        });
        if (self.calculate_frequencies, self.modal_mass_options) != before {
            self.status = "Параметры собственных частот изменены. Выполните расчёт МКЭ.".into();
        }
    }
    pub(super) fn analysis_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Расчёт МКЭ").default_open(false).show(ui,|ui|{
            let compatible=self.element_type!=crate::plan_trace::fem::ElementType::MindlinCr || (!self.nonlinear_options.enabled && !self.buckling_options.enabled && !self.building.enabled && !self.support_height.enabled && self.support_height.beams.is_empty() && self.support_height.columns.is_empty() && self.fe_mesh.as_ref().is_none_or(|m|m.quads.is_empty()));
            if !compatible {ui.colored_label(Color32::from_rgb(200,130,240),"P2/CR требует плоскую треугольную сетку. Для пространственной схемы, четырёхугольников или общей устойчивости выберите DKT / MITC3–4.");}
            let ready=!self.mesh_partial&&compatible&&self.fe_mesh.is_some()&&self.mesh_job.is_none()&&self.analysis_job.is_none()&&model::same_mesh_input(&self.mesh_input,&self.contours)&&self.mesh_built_size==self.mesh_size;
            if self.mesh_partial {ui.colored_label(Color32::LIGHT_BLUE,"Построена сетка выделенного фрагмента. Для расчёта всей схемы снимите выделение и повторите триангуляцию.");}
            if self.fe_mesh.is_none() {
                ui.colored_label(Color32::LIGHT_BLUE, "Сначала выполните триангуляцию");
            }
            if self.fe_mesh.is_some() {
                if self.analysis_job.is_some() || self.alternative_job.is_some() {
                    let stopping=self.solver_cancellation_requested();
                    if ui.add_enabled(!stopping,egui::Button::new(if stopping {"Остановка расчёта…"} else {"Остановить расчёт"}).min_size(egui::vec2(ui.available_width(),26.))).clicked(){self.stop_solver();}
                } else if ui.add_enabled(ready,egui::Button::new("МКЭ расчет").min_size(egui::vec2(ui.available_width(),26.))).clicked(){self.start_variant_analyses(ui.ctx());}
            }
            if self.analysis.is_some() && self.analysis_job.is_none() && self.alternative_job.is_none() {
                ui.horizontal(|ui|{self.reinforcement_controls(ui);});
            }
            self.reinforcement_progress(ui);
            if self.analysis_job.is_some()||self.alternative_job.is_some(){
                let stage=if self.solver_cancellation_requested() {"Остановка расчёта…".into()} else {self.solver_progress.lock().unwrap().last().map(|p|p.0.clone()).unwrap_or_else(||"Запуск расчёта".into())};
                ui.add(egui::ProgressBar::new(0.).animate(false).text(stage));
            } else if let Some(stage) = self.soil_progress_status() {
                ui.add(egui::ProgressBar::new(0.).animate(false).text(stage));
            }

        });
    }
    pub(super) fn nonlinear_request(
        &self,
    ) -> Result<Option<crate::plan_trace::fem::NonlinearRequest>, String> {
        if !self.nonlinear_options.enabled {
            return Ok(None);
        }
        if self.element_type == crate::plan_trace::fem::ElementType::MindlinCr {
            return Err("Для геометрической нелинейности выберите DKT / MITC3–4".into());
        }
        let id = if self.nonlinear_options.combination == 0 {
            self.active_combination
        } else {
            self.nonlinear_options.combination
        };
        let combination = self
            .combinations
            .iter()
            .find(|c| c.id == id)
            .ok_or("Выберите базовое сочетание геометрической нелинейности")?;
        Ok(Some(crate::plan_trace::fem::NonlinearRequest {
            weights: combination.weights(&self.load_cases, self.nonlinear_options.variant)?,
            caption: format!(
                "Сочетание {} · {}",
                combination.display_name(),
                self.nonlinear_options.variant.label()
            ),
        }))
    }

    fn active_buckling_source(&self) -> AnalysisLoadSource {
        if self.result_by_combinations {
            AnalysisLoadSource::Combination(self.active_combination)
        } else {
            AnalysisLoadSource::LoadCase(self.active_load)
        }
    }

    pub(super) fn buckling_request(
        &self,
    ) -> Result<Option<crate::plan_trace::fem::BucklingRequest>, String> {
        let options = &self.buckling_options;
        if !options.enabled {
            return Ok(None);
        }
        if self.element_type == crate::plan_trace::fem::ElementType::MindlinCr {
            return Err("Для расчёта общей устойчивости выберите DKT / MITC3–4".into());
        }
        if !(1..=30).contains(&options.forms)
            || !options.max_factor.is_finite()
            || options.max_factor <= 0.
        {
            return Err("Устойчивость: задайте от 1 до 30 форм и положительный предел КЗУ".into());
        }
        let (weights, caption) = options
            .source
            .unwrap_or_else(|| self.active_buckling_source())
            .resolve(&self.load_cases, &self.combinations, options.variant)
            .map_err(|error| format!("Устойчивость: {error}"))?;
        Ok(Some(crate::plan_trace::fem::BucklingRequest {
            weights,
            caption,
            forms: options.forms,
            max_factor: options.max_factor,
        }))
    }

    fn buckling_settings(&mut self, ui: &mut egui::Ui) {
        let active = self.active_buckling_source();
        let before = self.buckling_options.clone();
        let options = &mut self.buckling_options;
        ui.add_enabled_ui(self.analysis_job.is_none() && self.alternative_job.is_none(), |ui| {
            if ui.checkbox(&mut options.enabled, "Общая устойчивость").changed() && options.enabled {
                options.source.get_or_insert(active);
            }
            if !options.enabled {
                return;
            }
            let displayed_source = options.source.unwrap_or(active);
            let mut source = displayed_source;
            analysis_load_controls(ui, "buckling", &mut source, &mut options.variant,
                &self.load_cases, &self.combinations);
            if source != displayed_source {
                options.source = Some(source);
            }
            ui.horizontal_wrapped(|ui| {
                ui.label("Число форм");
                ui.add(egui::DragValue::new(&mut options.forms).range(1..=30));
                ui.label("Предел КЗУ");
                ui.add(egui::DragValue::new(&mut options.max_factor).range(0.0001..=1e9).speed(0.1))
                    .on_hover_text("Искать формы с положительным коэффициентом запаса устойчивости до этого значения включительно");
            });
            ui.small("Линейная потеря устойчивости при пропорциональном росте выбранных нагрузок.");
        });
        if *options != before {
            self.status = "Параметры общей устойчивости изменены. Выполните расчёт МКЭ.".into();
        }
    }

    pub(super) fn calculation_settings(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Настройки расчёта")
            .id_salt("calculation_settings")
            .default_open(false)
            .show(ui, |ui| {
            ui.add_enabled_ui(!self.input_locked, |ui| {
            let old=self.element_type;
            let allow_p2cr = !self.nonlinear_options.enabled && !self.buckling_options.enabled && !self.building.enabled && !self.support_height.enabled && self.support_height.beams.is_empty() && self.support_height.columns.is_empty()
                && self.fe_mesh.as_ref().is_none_or(|m| m.quads.is_empty());
            if !allow_p2cr && self.element_type == crate::plan_trace::fem::ElementType::MindlinCr {
                self.element_type = crate::plan_trace::fem::ElementType::Dkt;
            }
            ui.horizontal(|ui| {
                ui.label("Тип КЭ").on_hover_text("Выбор конечного элемента для расчёта плит.");
                egui::ComboBox::from_id_salt("plate_element_type")
                    .selected_text(self.element_type.label())
                    .show_ui(ui, |ui| {
                        for t in crate::plan_trace::fem::ElementType::ALL {
                            if t == crate::plan_trace::fem::ElementType::MindlinCr && !allow_p2cr { continue; }
                            ui.selectable_value(&mut self.element_type, t, t.label());
                        }
                    }).response.on_hover_text("Выбирает формулировку конечного элемента для плит. После смены нужно заново выполнить расчёт.");
            });
            if old!=self.element_type{self.analysis=None;self.analysis_input=None;self.analysis_job=None;self.combination_cache=None;self.status="Тип КЭ изменён. Выполните расчёт.".into();}
            });
            if ui.add_enabled(!self.input_locked,egui::Checkbox::new(&mut self.support_height.rigid_columns,"Генерировать АЖТ колонн")).on_hover_text("При построении сетки связывает узлы плиты в габаритах колонны с её ведущим узлом.").changed(){self.clear_mesh();}
            self.reinforcement_method_controls(ui);
            if !self.building.enabled {
                self.support_height_controls(ui);
            }
            ui.add_enabled_ui(!self.input_locked && self.analysis_job.is_none() && self.alternative_job.is_none(), |ui| {
                ui.checkbox(&mut self.nonlinear_options.enabled,"Геометрическая нелинейность").on_hover_text("Учитывает влияние деформаций на равновесие. Выбранное базовое сочетание решается итерационно.");
                if self.nonlinear_options.enabled {
                    if self.nonlinear_options.combination == 0 { self.nonlinear_options.combination = self.active_combination; }
                    ui.horizontal(|ui| {
                    egui::ComboBox::from_id_salt("nonlinear_base_combo").width(100.).selected_text(
                        self.combinations.iter().find(|c|c.id==self.nonlinear_options.combination).map_or_else(||"Выберите сочетание".into(), |c|c.display_name())
                    ).show_ui(ui, |ui| for c in &self.combinations {
                        ui.selectable_value(&mut self.nonlinear_options.combination,c.id,c.display_name());
                    }).response.on_hover_text("Базовое сочетание для геометрически нелинейного расчёта.");
                    egui::ComboBox::from_id_salt("nonlinear_base_variant").width(160.).selected_text(self.nonlinear_options.variant.label())
                        .show_ui(ui, |ui| for v in crate::plan_trace::load_combinations::Variant::ALL {
                            ui.selectable_value(&mut self.nonlinear_options.variant,v,v.label());
                        }).response.on_hover_text("Вариант коэффициентов базового сочетания.");
                    });
                    ui.small("Второй порядок. База решается итерационно; остальные виды — по касательной к базе.");
                }
            });
            self.soil_settings_controls(ui);
            ui.collapsing("Устойчивость", |ui| self.buckling_settings(ui));
            if self.building.enabled {
                ui.add_enabled_ui(!self.input_locked && self.analysis_job.is_none() && self.alternative_job.is_none(), |ui| {
                    if ui.checkbox(&mut self.support_height.automatic_building_bottom, "Автоматические закрепления внизу").on_hover_text("Включает автоматические закрепления нижнего уровня вертикальных конструкций.").changed() && self.support_height.automatic_building_bottom && self.support_height.building_bottom_mask == 0 {
                        self.support_height.building_bottom_mask = 7;
                    }
                    if ui.checkbox(&mut self.support_height.automatic_building_top, "Автоматические закрепления вверху").on_hover_text("Включает автоматические закрепления верхнего уровня вертикальных конструкций.").changed() && self.support_height.automatic_building_top && self.support_height.building_top_mask == 0 {
                        self.support_height.building_top_mask = 3;
                    }
                });
            }
            self.modal_settings(ui);
            self.length_method_controls(ui);
            if self.support_height.enabled && !self.building.enabled {ui.small("Оболочечная модель: U,V,W и повороты. Низ опор: X,Y,Z; верх: X,Y. Повороты свободны.");}
            });
    }
    fn length_method_controls(&mut self, ui: &mut egui::Ui) {
        use crate::plan_trace::fem::shell::LengthMethod;
        ui.add_enabled_ui(
            self.analysis_job.is_none() && self.alternative_job.is_none(),
            |ui| {
                let before = self.support_height.length_method;
                let enabled_before = self.calculate_effective_lengths;
                ui.horizontal_wrapped(|ui| {
                    ui.checkbox(&mut self.calculate_effective_lengths, "Расчётные длины")
                        .on_hover_text(
                            "Вычислять при МКЭ-расчёте для стен и колонн, отмеченных в свойствах.",
                        );
                    ui.add_enabled_ui(self.calculate_effective_lengths, |ui| {
                        egui::ComboBox::from_id_salt("effective_length_method")
                            .selected_text(length_method_label(before))
                            .show_ui(ui, |ui| {
                                for method in [LengthMethod::Plane, LengthMethod::Spatial] {
                                    ui.selectable_value(
                                        &mut self.support_height.length_method,
                                        method,
                                        length_method_label(method),
                                    );
                                }
                            });
                    });
                });
                if enabled_before != self.calculate_effective_lengths
                    || before != self.support_height.length_method
                {
                    self.length_view = 0;
                    self.status = "Настройки расчётных длин изменены. Выполните расчёт МКЭ.".into();
                }
            },
        );
    }
    fn physical_model_stats(&self) -> Option<ModelStats> {
        let key = (self.mesh_generation, self.result_generation);
        if let Some((old, stats)) = self.perf.model_stats.borrow().as_ref() {
            if *old == key {
                return Some(*stats);
            }
        }
        let mut stats = ModelStats::default();
        if let Some(shell) = self
            .analysis
            .as_ref()
            .and_then(|r| r.cases.iter().find_map(|c| c.shell.as_ref()))
        {
            stats.nodes = shell.translations_mm.len();
            stats.beams = shell.beam_forces.len();
            for face in &shell.surface_forces {
                let counts = if face.support.is_some() {
                    &mut stats.walls
                } else {
                    &mut stats.slabs
                };
                match face.nodes.len() {
                    3 => counts[0] += 1,
                    4 => counts[1] += 1,
                    _ => {}
                }
            }
        } else if let Some((generation, _, _, true, scene)) = self
            .spatial
            .cache
            .as_ref()
            .filter(|(generation, _, _, meshed, _)| *meshed && *generation == self.mesh_generation)
        {
            let _ = generation;
            stats.nodes = scene.xyz.len();
            stats.beams = scene.beams.len();
            for face in &scene.surfaces {
                let counts = if face.support.is_some() {
                    &mut stats.walls
                } else {
                    &mut stats.slabs
                };
                match face.nodes.len() {
                    3 => counts[0] += 1,
                    4 => counts[1] += 1,
                    _ => {}
                }
            }
        } else {
            return None;
        }
        *self.perf.model_stats.borrow_mut() = Some((key, stats));
        Some(stats)
    }
    pub(super) fn technical_information(&self, ui: &mut egui::Ui) {
        if self.analysis.is_none() && self.fe_mesh.is_none() {
            return;
        }
        egui::CollapsingHeader::new("Техническая информация")
            .default_open(false)
            .show(ui, |ui| {
                if self.analysis_job.is_none() && self.alternative_job.is_none() {
                    let mut stages=self.solver_progress.lock().unwrap().clone();
                    if !stages.is_empty() {
                        ui.collapsing("Протокол решения", |ui| {
                            stages.sort_by(|a,b|a.1.total_cmp(&b.1));
                            let end=stages.last().map_or(0.,|s|s.1);
                            let group=|label:&str|label.split_once(" | ").map(|p|p.0.to_owned()).unwrap_or_default();
                            for (i,(label,start)) in stages.iter().enumerate() {
                                if label.ends_with(": конец") || label=="Расчёт завершён" || label.ends_with("Загружение завершено") || label.ends_with("Загружение: ошибка") {continue;}
                                let stage_name = label.rsplit(" | ").next().unwrap_or(label);
                                let special = match stage_name {
                                    "Собственные формы и частоты" => Some("Время расчёта собственных форм"),
                                    "Расчётные длины пилонов и колонн" => Some("Время расчёта длин"),
                                    _ => None,
                                };
                                let next = if special.is_some() {
                                    let marker = format!("{label}: конец");
                                    stages[i+1..].iter().find(|s| s.0 == marker)
                                } else {
                                    stages[i+1..].iter().find(|s| group(&s.0)==group(label))
                                }.map_or(end, |s| s.1);
                                ui.small(format!("{}: {:.2} с", special.unwrap_or(label), (next-start).max(0.)));

                            }
                            if let Some((_,seconds))=stages.last(){ui.small(format!("Время расчёта: {seconds:.2} с"));}
                        });
                    }
                }

        if let Some(n) = self.analysis.as_ref().and_then(|r|r.nonlinear.as_ref()) {
            ui.label(format!("Нелинейная база: {} · шагов {} · итераций {}",n.caption,n.steps,n.iterations));
        }
        if let Some(mesh) = &self.fe_mesh {
            if let Some(stats)=self.physical_model_stats() {
                ui.label(format!("Общая модель: узлов {} · КЭ {}",stats.nodes,stats.slabs.iter().sum::<usize>() + stats.walls.iter().sum::<usize>() + stats.beams));
                ui.small(format!("КЭ плит: треугольников {} · четырёхугольников {}",stats.slabs[0],stats.slabs[1]));
                ui.small(format!("КЭ стен: треугольников {} · четырёхугольников {}",stats.walls[0],stats.walls[1]));
                if stats.beams>0 {ui.small(format!("Стержневых КЭ: {}",stats.beams));}
            } else {
                ui.label(format!("Сетка плит: узлов {} · КЭ {}",mesh.nodes.len(),mesh.elements.len().saturating_sub(mesh.quads.len())));
                if self.building.enabled || self.support_height.enabled {ui.small("Общая статистика со стенами появится после формирования пространственной модели или расчёта.");}
            }
            let count = mesh
                .support_boundary
                .values()
                .flatten()
                .copied()
                .collect::<std::collections::BTreeSet<_>>()
                .len();
            ui.small(format!("Узлов плиты на линиях опор: {count}"));
            if self.building.enabled {ui.small("Единая оболочечная модель здания. Общие узлы на стыках; закрепления задаются настройками и опорами.");}
            else if self.support_height.enabled {
                ui.small("Опоры моделируются вертикальными стенами: низ — X/Y/Z, верх — X/Y. Повороты свободны.");
            } else {
                ui.small("На линиях опор закреплён прогиб w. Повороты свободны.");
            }
        }

                if let Some(free_dofs) = self.analysis.as_ref().map(|r| r.free_dofs) {
                    ui.small(format!("Свободных степеней: {} · Cholesky LL^T", free_dofs));
                    if let Some(c) = self.selected_full_result() {
                        ui.small(format!(
                            "Нагрузка {:.3} {} · реакции {:.3} {}",
                            c.applied_kn * self.force_result_factor(),
                            self.load_units.label(),
                            c.reaction_kn.iter().sum::<f64>() * self.force_result_factor(),
                            self.load_units.label()
                        ));
                        ui.small(format!(
                            "Баланс {:.2e} {} · макс. невязка исходных {:.2e}",
                            c.balance_kn * self.force_result_factor(),
                            self.load_units.label(),
                            c.relative_residual
                        ));
                        ui.small(format!(
                            "Баланс моментов XY: {:.2e}; {:.2e} {}·м",
                            c.balance_m_knm[0] * self.force_result_factor(),
                            c.balance_m_knm[1] * self.force_result_factor(),
                            self.load_units.label()
                        ));
                    }
                }
            });
    }
    pub(super) fn paint_analysis(&self, painter: &egui::Painter, rect: Rect) {
        if self.schema_analysis.mode.is_some() {
            self.paint_schema_mesh(painter, rect);
            return;
        }

        if self.punching_view.is_some() {
            return;
        }
        if (17..=22).contains(&self.result_field) {
            self.paint_beam_results(painter, rect);
            return;
        }
        self.paint_cached_results(painter, rect);
        self.paint_support_area_borders(painter, rect);
        self.paint_reinforcement_errors(painter, rect);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn modal_mass_source_uses_only_the_selected_case_or_the_combination_weights() {
        let mut app = App::new_project();
        let first = &mut app.load_cases[0];
        first.name = "Полы".into();
        first.gamma_f = 2.;
        first.gamma_l = 0.5;
        let first_id = first.id;
        let mut second = first.clone();
        second.id = first_id + 1;
        second.name = "Перегородки".into();
        let second_id = second.id;
        app.load_cases.push(second);
        let combination = &mut app.combinations[0];
        combination.name = "Для масс".into();
        combination.coefficients = [(first_id, 2.), (second_id, 1.)].into();
        let combination_id = combination.id;
        assert!(app.modal_weights_for_solve().unwrap().is_none());

        app.calculate_frequencies = true;
        let (weights, title) = app.modal_weights_for_solve().unwrap().unwrap();
        assert_eq!(weights, [(first_id, 1.), (second_id, 0.5)]);
        assert_eq!(title, "Сочетание Для масс · Нормативные");

        app.modal_mass_options = ModalMassOptions {
            source: Some(AnalysisLoadSource::LoadCase(first_id)),
            variant: Variant::NormativeLong,
        };
        let (weights, title) = app.modal_weights_for_solve().unwrap().unwrap();
        assert_eq!(weights, [(first_id, 0.25)]);
        assert_eq!(title, "Полы · Нормативные длит.");

        let mut saved = serde_json::to_value(app.project_snapshot()).unwrap();
        let mut restored = App::new_project();
        let project = serde_json::from_value(saved.clone()).unwrap();
        restored
            .apply_project(project, "modal.slab.zip".into())
            .unwrap();
        assert_eq!(
            restored.modal_weights_for_solve().unwrap().unwrap(),
            (weights, title)
        );

        saved["settings"]
            .as_object_mut()
            .unwrap()
            .remove("modal_mass_options");
        let legacy = serde_json::from_value(saved).unwrap();
        restored
            .apply_project(legacy, "legacy.slab.zip".into())
            .unwrap();
        assert_eq!(
            restored
                .default_modal_source()
                .resolve(
                    &restored.load_cases,
                    &restored.combinations,
                    Variant::Normative
                )
                .unwrap(),
            restored.modal_weights_for_solve().unwrap().unwrap()
        );
        assert_eq!(restored.active_combination, combination_id);
    }

    #[test]
    fn nonlinear_snapshot_keeps_base_equilibrium_and_combines_affinely() {
        let mut app = App::default();
        app.load_project_path(
            std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
                .join("tests/fixtures/columns_two_levels.slab.zip"),
        )
        .unwrap();
        app.support_height.automatic_building_bottom = true;
        app.support_height.automatic_building_top = true;
        app.mesh_size = 1500.;
        app.mesh_built_size = app.mesh_size;
        app.sync_stiffness_sources();
        let mut layout = app.building_layout();
        let sources = crate::plan_trace::columns::meshing_sources(
            &app.contours,
            &app.support_height,
            &mut layout,
        );
        let mut mesh = crate::plan_trace::building_mesh::generate(
            &sources,
            app.mesh_size,
            layout.unwrap(),
            app.mesh_options,
        )
        .unwrap();
        crate::plan_trace::columns::tag_mesh(&mut mesh, &app.contours, &app.support_height);
        app.nonlinear_options.enabled = true;
        app.nonlinear_options.combination = app.combinations[0].id;
        let key = app.analysis_key();
        assert!(key.nonlinear.is_some());
        let result = solve_snapshot(&mesh, &key, &app.load_cases).unwrap();
        assert_eq!(result.cases.len(), app.load_cases.len());
        let base = result.nonlinear.as_ref().unwrap();
        assert!(base.steps >= 4);
        let combined =
            crate::plan_trace::load_combinations::combine(&result, 123, &base.weights).unwrap();
        assert!(combined.relative_residual < 1e-6);
        assert!(combined.balance_kn.abs() < 1e-5 * combined.applied_kn.abs().max(1.));
        assert!(combined
            .shell
            .unwrap()
            .translations_mm
            .iter()
            .flatten()
            .all(|x| x.is_finite()));
    }

    #[test]
    fn spatial_plate_accepts_forces_and_moments_on_global_axes() {
        use crate::plan_trace::loads::{Direction, ForceUnits, Magnitude};
        let mut app = App::default();
        app.building.enabled = true;
        app.begin(Kind::Slab);
        for p in [[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]] {
            app.sketch_click(p, false);
        }
        app.finish();
        app.begin(Kind::Restraint);
        for p in [[0., 0.], [0., 2000.]] {
            app.sketch_click(p, false);
        }
        app.finish();
        app.contours.last_mut().unwrap().restraint_mask = 31;
        let initial_case = app.load_cases[0].clone();
        for (i, (direction, moment)) in [
            (Direction::X, false),
            (Direction::Y, false),
            (Direction::Z, false),
            (Direction::X, true),
            (Direction::Y, true),
            (Direction::Z, true),
        ]
        .into_iter()
        .enumerate()
        {
            let mut case = initial_case.clone();
            case.id += i as u64 + 1;
            app.load_cases.push(case.clone());
            app.active_load = case.id;
            app.begin(Kind::LoadPoint);
            app.new_load_direction = direction;
            app.new_load_moment = moment;
            app.new_load = Magnitude {
                value: 10.,
                units: ForceUnits::KiloNewton,
            };
            app.sketch_click([1500., 1000.], false);
        }
        for (kind, direction, points) in [
            (
                Kind::LoadLine,
                Direction::X,
                vec![[500., 1000.], [1500., 1000.]],
            ),
            (
                Kind::LoadArea,
                Direction::Y,
                vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
            ),
        ] {
            let mut case = initial_case.clone();
            case.id = app.load_cases.last().unwrap().id + 1;
            app.active_load = case.id;
            app.load_cases.push(case);
            app.begin(kind);
            app.new_load_direction = direction;
            for point in points {
                app.sketch_click(point, false);
            }
            app.finish();
        }
        app.sync_stiffness_sources();
        app.mesh_built_size = 500.;
        let mut layout = app.building_layout();
        let sources = crate::plan_trace::columns::meshing_sources(
            &app.contours,
            &app.support_height,
            &mut layout,
        );
        let mut mesh = crate::plan_trace::building_mesh::generate(
            &sources,
            500.,
            layout.unwrap(),
            app.mesh_options,
        )
        .unwrap();
        crate::plan_trace::columns::tag_mesh(&mut mesh, &app.contours, &app.support_height);
        let result = solve_snapshot(&mesh, &app.analysis_key(), &app.load_cases).unwrap();
        assert_eq!(result.cases.len(), 9);
        for (case, axis, expected) in [
            (1, 0, -10.),
            (2, 1, -10.),
            (3, 2, 10.),
            (7, 0, -10.),
            (8, 1, -40.),
        ] {
            let sum = result.cases[case]
                .shell
                .as_ref()
                .unwrap()
                .restrained_reactions_kn
                .iter()
                .map(|r| r[axis])
                .sum::<f64>();
            assert!(
                (sum - expected).abs() < 1e-6,
                "case {case}: {sum} instead of {expected}"
            );
        }
        let loaded_node = mesh
            .nodes
            .iter()
            .position(|n| n.xy == [1500., 1000.])
            .unwrap();
        for axis in 0..3 {
            let rotation =
                result.cases[4 + axis].shell.as_ref().unwrap().rotations_rad[loaded_node][axis];
            assert!(rotation > 0., "moment on axis {axis}: {rotation}");
        }
        for case in &result.cases[1..] {
            assert!(case.relative_residual < 1e-6);
            assert!(case
                .shell
                .as_ref()
                .unwrap()
                .translations_mm
                .iter()
                .flatten()
                .any(|x| x.abs() > 1e-6));
        }
    }

    #[test]
    fn load_names_are_persistent_metadata_not_calculation_input() {
        let mut app = App::default();
        app.begin(Kind::LoadPoint);
        app.sketch_click([100., 100.], false);
        let id = app
            .contours
            .iter()
            .position(|c| c.kind == Kind::LoadPoint)
            .unwrap();
        let before = app.analysis_key();
        app.contours[id].load.as_mut().unwrap().name = "Оборудование".into();
        assert!(before == app.analysis_key());
        let mut value = serde_json::to_value(&app.contours[id]).unwrap();
        let restored: Contour = serde_json::from_value(value.clone()).unwrap();
        assert_eq!(restored.load.unwrap().name, "Оборудование");
        let data = value["load"].as_object_mut().unwrap();
        for key in ["name", "direction", "moment"] {
            data.remove(key);
        }
        let legacy: Contour = serde_json::from_value(value).unwrap();
        let legacy = legacy.load.unwrap();
        assert!(legacy.name.is_empty());
        assert_eq!(legacy.direction, crate::plan_trace::loads::Direction::Z);
        assert!(!legacy.moment);
        app.contours[id].load.as_mut().unwrap().magnitude.value += 1.;
        assert!(before != app.analysis_key());
    }
    #[test]
    fn self_weight_changes_invalidate_analysis_but_view_units_and_factors_do_not() {
        use crate::plan_trace::loads::{ForceUnits, LoadType};
        let mut a = App::default();
        let key = a.analysis_key();
        a.load_units = ForceUnits::KiloNewton;
        a.load_cases[0].gamma_f = 1.1;
        a.load_cases[0].gamma_l = 0.5;
        assert!(a.analysis_key() == key);
        a.load_cases[0].kind = LoadType::SelfWeight;
        assert!(a.analysis_key() != key);
        let key = a.analysis_key();
        a.materials.entry(1).or_default().weight.value = 3.;
        assert!(a.analysis_key() != key);
    }
    #[test]
    fn hover_matches_active_field_interpolation_units_and_load_variant() {
        use crate::plan_trace::{
            fem::{CaseResult, Results},
            load_combinations::Variant,
            loads::ForceUnits,
            meshing::{Element, Mesh, Node},
        };
        let mut a = App::default();
        a.display_text.decimals = 4;
        a.load_units = ForceUnits::KiloNewton;
        a.fe_mesh = Some(Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: [[0., 0.], [1000., 0.], [0., 1000.], [1000., 1000.]]
                .into_iter()
                .enumerate()
                .map(|(i, xy)| Node {
                    id: i as u64 + 1,
                    xy,
                })
                .collect(),
            elements: vec![
                Element {
                    id: 10,
                    nodes: [0, 1, 2],
                    supports: vec![],
                },
                Element {
                    id: 11,
                    nodes: [1, 3, 2],
                    supports: vec![],
                },
            ],
            edges: vec![[0, 1], [0, 2], [1, 2], [1, 3], [2, 3]],
            support_boundary: BTreeMap::from([(9, vec![0])]),
        });
        a.analysis = Some(Results {
            nonlinear: None,
            buckling: None,
            lengths: None,
            modal: None,
            free_dofs: 0,
            cases: vec![CaseResult {
                punching: Default::default(),
                shell: None,
                id: 1,
                w_mm: vec![0., 10., 20., 30.],
                rotations_rad: vec![[0.; 3]; 0],
                moments: vec![[1., 2., 3.], [7., 8., 9.]],
                shears: vec![[4., 5.], [10., 11.]],
                loads_kn_m2: vec![10., 30.],
                foundation_pressure_kn_m2: vec![0., f64::NAN],
                reaction_kn: vec![-6., 0., 0., 0.],
                applied_kn: 6.,
                balance_kn: 0.,
                balance_m_knm: [0.; 2],
                relative_residual: 0.,
            }],
        });
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 1000.));
        let cursor = a.camera.screen([1000. / 3., 1000. / 3.], rect);
        for (field, flat, smooth) in [
            (1, "1.0000", "3.0000"),
            (2, "2.0000", "4.0000"),
            (3, "3.0000", "5.0000"),
            (4, "4.0000", "6.0000"),
            (5, "5.0000", "7.0000"),
            (7, "10.0000", "10.0000"),
        ] {
            a.result_field = field;
            a.iso_efforts = false;
            let text = a.hover_result(cursor, rect).unwrap();
            assert!(text.ends_with(flat), "{text}");
            assert!(text.contains(&a.result_field_label(field)));
            if field == 2 {
                assert!(!text.contains("Mx"));
                assert!(!text.contains("Qx"));
            }
            a.iso_efforts = true;
            let text = a.hover_result(cursor, rect).unwrap();
            assert!(text.ends_with(smooth), "{text}");
        }
        a.result_field = 0;
        assert!(a.hover_result(cursor, rect).unwrap().ends_with("10.0000"));
        let near = a.camera.screen([3., 4.], rect);
        assert!(a.hover_result(near, rect).unwrap().ends_with("0.1100"));
        a.result_field = DISPLACEMENT_NORM_FIELD;
        a.displacement_axes = 0b100;
        assert!(a.hover_result(near, rect).unwrap().ends_with("0.1100"));
        assert!(a
            .hover_result(a.camera.screen([-1., 1.], rect), rect)
            .is_none());
        a.result_field = 6;
        assert!(a.hover_result(cursor, rect).is_none());
        let node = a.camera.screen([0., 0.], rect);
        assert!(a.hover_result(node, rect).unwrap().ends_with("-6.0000"));
        a.result_field = 34;
        assert!(a.hover_result(cursor, rect).unwrap().ends_with("0.0000"));
        assert!(a
            .hover_result(a.camera.screen([750., 750.], rect), rect)
            .is_none());
        for i in 0..2 {
            let surface = crate::plan_trace::fem::shell::Surface {
                nodes: a.fe_mesh.as_ref().unwrap().elements[i].nodes.to_vec(),
                display: vec![i],
                support: None,
                material: Default::default(),
                rigid_z_mm: 0.,
            };
            assert_eq!(
                a.spatial_surface_values(i, &surface),
                (i == 0).then(|| vec![0.; 3])
            );
        }
        a.result_field = 2;
        a.iso_efforts = false;
        a.load_units = ForceUnits::TonForce;
        assert!(a.hover_result(cursor, rect).unwrap().ends_with("0.2000"));
        a.load_cases[0].gamma_f = 2.;
        a.combination_variant = Variant::Normative;
        a.sync_combination_result();
        assert!(a.hover_result(cursor, rect).unwrap().ends_with("0.1000"));
        assert_eq!(
            a.analysis.as_ref().unwrap().cases[0].moments[0],
            [1., 2., 3.]
        );
    }
    #[test]
    fn combined_displacements_preserve_component_mask_and_independent_view_keys() {
        use crate::plan_trace::fem::displacement_norm;
        assert_eq!(displacement_norm([-3., 4., -12.], 0b011), 5.);
        assert_eq!(displacement_norm([-3., 4., -12.], 0b111), 13.);
        assert_eq!(displacement_norm([-3., 4., -12.], 0b100), 12.);

        let mut app = App::default();
        app.result_field = DISPLACEMENT_NORM_FIELD;
        app.displacement_axes = 0b011;
        let scale = app.scale_context_key();
        let view = app.view_key();
        assert_eq!(
            app.result_field_label(DISPLACEMENT_NORM_FIELD),
            "Суммарное перемещение XY, мм"
        );
        app.displacement_axes = 0b111;
        assert!(scale != app.scale_context_key());
        assert_ne!(view, app.view_key());
        assert_eq!(app.result_display_factor(), 1.);
    }
    #[test]
    fn successful_solve_locks_inputs_and_unlock_restores_editing() {
        let mut a = App::default();
        a.fe_mesh = Some(crate::plan_trace::meshing::Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![],
            elements: vec![],
            edges: vec![],
            support_boundary: BTreeMap::new(),
        });
        a.analysis_input = Some(a.analysis_key());
        let (tx, rx) = mpsc::channel();
        a.analysis_job = Some(rx);
        tx.send(Ok(crate::plan_trace::fem::Results {
            nonlinear: None,
            buckling: None,
            lengths: None,
            modal: None,
            cases: vec![],
            free_dofs: 0,
        }))
        .ok()
        .unwrap();
        a.poll_analysis();
        assert!(a.input_locked);
        a.begin(Kind::Support);
        assert!(a.mode.is_none());
        a.clear_mesh();
        assert!(a.fe_mesh.is_some());
        a.input_locked = false;
        a.begin(Kind::Support);
        assert_eq!(a.mode, Some(Kind::Support));
        a.input_locked = true;
        a.fe_mesh = None;
        a.poll_analysis();
        assert!(!a.input_locked);
    }
    #[test]
    fn uncomputed_model_caption_does_not_show_saved_result_mode() {
        let mut a = App::default();
        for mode in [false, true] {
            for field in [0, 1, 2, 3, 4, 5, 6] {
                a.result_field = field;
                a.iso_efforts = mode;
                assert!(a.result_caption().ends_with("Расчётная схема"));
                assert_eq!(a.result_has_element_field(), matches!(field, 1..=5));
            }
        }
    }
    #[test]
    fn force_result_labels_and_values_follow_units_without_mutating_base_results() {
        let mut a = App::default();
        assert_eq!(a.result_field_label(0), "W, мм");
        assert!(a.result_field_label(1).contains("тс·м/м"));
        a.result_field = 6;
        assert!(a.result_field_label(6).contains("Узловые реакции Rz, тс"));
        assert_eq!(a.force_result_factor(), 0.1);
        a.load_units = crate::plan_trace::loads::ForceUnits::KiloNewton;
        a.sync_result_units();
        assert_eq!(a.force_result_factor(), 1.);
        assert!(a.result_field_label(6).ends_with("кН"));
        assert!(a.result_field_label(4).ends_with("кН/м"));
        assert_eq!(a.result_field_label(0), "W, мм");
    }
    #[test]
    fn linear_displacement_interpolation_has_exact_vertex_and_interior_values() {
        let xy = [[0., 0.], [3000., 0.], [0., 2000.]];
        let w = [2., 8., -4.];
        for p in [xy[0], xy[1], xy[2], [600., 800.]] {
            let l = weights(xy, p).unwrap();
            let value = (0..3).map(|i| l[i] * w[i]).sum::<f64>();
            assert!((value - (2. + 0.002 * p[0] - 0.003 * p[1])).abs() < 1e-12);
        }
        assert!(weights(xy, [5000., 5000.]).is_none());
        assert!(weights([[0., 0.]; 3], [0., 0.]).is_none());
    }
}

#[cfg(test)]
mod punching_input_tests {
    use super::*;
    #[test]
    fn calculation_input_retains_punching_and_excludes_plain_auxiliary() {
        let mut app = App::default();
        let plain = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
            id: 1,
            parent: None,
            kind: Kind::Auxiliary,
            points: vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
            curves: vec![],
        };
        let mut punching = plain.clone();
        punching.id = 2;
        punching.refinement = Some(model::Refinement {
            inactive_edges: vec![],

            name: "Продавливание: торец стены 3".into(),
            size: 150.,
        });
        app.contours = vec![plain, punching.clone()];
        let key = app.analysis_key();
        assert_eq!(key.contours, vec![punching]);
    }
}

#[cfg(test)]
mod membrane_and_stats_regression {
    use super::*;
    #[test]
    fn area_view_uses_calculated_thickness_per_floor_and_stats_include_walls() {
        let request: serde_json::Value =
            serde_json::from_str(include_str!("../../fem_web/building_solver_fixture.json"))
                .unwrap();
        let mesh = serde_json::from_value(request["mesh"].clone()).unwrap();
        let mut key: Input = serde_json::from_value(request["first"].clone()).unwrap();
        // This legacy fixture predates explicit restraint settings. This regression
        // assumes the original automatic end restraints, so request them explicitly.
        key.support_height.automatic_building_bottom = true;
        key.support_height.automatic_building_top = true;
        let cases: Vec<crate::plan_trace::loads::LoadCase> =
            serde_json::from_value(request["cases"].clone()).unwrap();
        let mut result = solve_snapshot(&mesh, &key, &cases).unwrap();
        for case in &mut result.cases {
            let shell = case.shell.as_mut().unwrap();
            shell.membrane.fill([20., 30., 5.]);
            for face in shell
                .surface_forces
                .iter_mut()
                .filter(|f| f.support.is_none())
            {
                face.material.thickness_mm = if face.nodes.iter().all(|n| *n < 4) {
                    200.
                } else {
                    400.
                };
            }
        }
        let mut app = App::default();
        app.fe_mesh = Some(mesh);
        app.analysis = Some(result);
        app.result_field = 12;
        app.load_units = crate::plan_trace::loads::ForceUnits::KiloNewton;
        let l = app.scale_context_key();
        assert_eq!(
            app.result_element_value(app.selected_result().unwrap(), 0),
            20.
        );
        app.membrane_area = true;
        assert!(l != app.scale_context_key());
        for (element, expected) in [(0, 100.), (1, 100.), (2, 50.), (3, 50.)] {
            assert!(
                (app.result_element_value(app.selected_result().unwrap(), element) - expected)
                    .abs()
                    < 1e-10
            );
        }
        assert!(app.result_field_label(12).contains("кН/м²"));
        let stats = app.physical_model_stats().unwrap();
        assert!(stats.nodes > app.fe_mesh.as_ref().unwrap().nodes.len());
        assert_eq!(stats.slabs, [4, 0]);
        assert!(stats.walls[0] >= 48);
        assert_eq!(stats.walls[1], 0);
        app.membrane_area = false;
        assert_eq!(
            app.result_element_value(app.selected_result().unwrap(), 2),
            20.
        );
    }
}

#[cfg(test)]
mod modal_focus_tests {
    use super::*;
    #[test]
    fn background_animation_freezes_and_resumes_without_phase_jump() {
        let mut view = ModalView {
            stiffness_variant: 0,
            result_generation: 0,
            mesh_generation: 0,
            shape: Default::default(),
            amplitude: 100.,
            number: 1,
            source: ShapeSource::Vibration(1.),
            zero_translation: false,
            playing: true,
            started: 0.,
            paused_at: None,
            vibration_values: None,
            sensitivity: None,
            show_sensitivity: false,
        };
        view.shape = std::sync::Arc::new(vec![[3., -4., 12.], [0., 2., 0.]]);
        view.set_vibration_quantity(Some(VibrationQuantity::Magnitude(0b011)));
        assert_eq!(
            view.vibration_values.map(|(_, range)| range),
            Some([2., 5.])
        );
        view.set_vibration_quantity(Some(VibrationQuantity::Axis(1)));
        assert_eq!(
            view.vibration_values.map(|(_, range)| range),
            Some([-4., 2.])
        );
        view.shape = std::sync::Arc::new(vec![[0., 0., 1.], [0., 0., -1.]]);
        view.set_vibration_quantity(Some(VibrationQuantity::Axis(0)));
        let (_, [lo, hi]) = view.vibration_values.unwrap();
        assert!(lo < 0. && 0. < hi);
        let before = view.animation(0.2, true).0;
        let (frozen, repaint) = view.animation(0.2, false);
        assert_eq!(frozen, before);
        assert!(!repaint);
        let (later, repaint) = view.animation(60., false);
        assert_eq!(later, frozen);
        assert!(!repaint);
        let (resumed, repaint) = view.animation(60., true);
        assert!((resumed - frozen).abs() < 1e-10);
        assert!(repaint);
        assert!((view.animation(60.1, true).0 - resumed).abs() > 1.);
        view.playing = false;
        assert_eq!(view.animation(100., false), (100., false));
    }
}
