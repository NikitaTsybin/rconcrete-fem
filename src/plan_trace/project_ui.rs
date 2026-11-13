//! Self-contained, versioned ZIP project; derived FE mesh/results are rebuilt.
use super::*;
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
#[cfg(target_arch = "wasm32")]
#[path = "opfs.rs"]
pub(super) mod opfs;
use crate::plan_trace::{
    fem::{element::Material, ElementType},
    grid::Grid,
    load_combinations::{Combination, Variant},
    loads::{ForceUnits, LoadCase, Magnitude},
    result_scale::{Scale, State},
};
use serde::{Deserialize, Serialize};
use std::{
    collections::BTreeSet,
    io::{Read, Write},
    path::Path,
};
const MAX_JSON: u64 = 256 * 1024 * 1024;
const LOAD_CONTOUR_KINDS: [Kind; 4] = [
    Kind::LoadLine,
    Kind::LoadArea,
    Kind::LoadPoint,
    Kind::LoadHole,
];
const CONTOUR_TREE_GROUPS: [(Kind, &str); 12] = [
    (Kind::Slab, "Плиты"),
    (Kind::Hole, "Отверстия"),
    (Kind::Support, "Стены"),
    (Kind::Auxiliary, "Вспомогательные линии"),
    (Kind::Punching, "Области суммирования нагрузок"),
    (Kind::Foundation, "Основания"),
    (Kind::Column, "Колонны"),
    (Kind::Beam, "Балки"),
    (Kind::Triangulation, "Ограничения триангуляции"),
    (Kind::Restraint, "Линейные опоры"),
    (Kind::PointRestraint, "Точечные опоры"),
    (Kind::RigidLink, "АЖТ"),
];

pub(super) fn contour_tree_menu(
    response: &egui::Response,
    count: usize,
    ids: impl Iterator<Item = u64>,
    selection: &mut Option<Vec<u64>>,
) {
    response.context_menu(|ui| {
        if ui
            .add_enabled(count != 0, egui::Button::new("Выделить"))
            .clicked()
        {
            *selection = Some(ids.collect());
            ui.close_menu();
        }
    });
}

fn default_underlay_filter() -> [bool; 2] {
    [true; 2]
}
fn default_true() -> bool {
    true
}
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Project {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    lir_source: Option<lir_import::Source>,
    #[serde(default)]
    building: building::Building,
    #[serde(default)]
    stiffnesses: Option<crate::plan_trace::stiffness::Library>,
    format: String,
    version: u32,
    #[serde(default)]
    underlays: Vec<crate::plan_trace::underlay::Underlay>,
    #[serde(default)]
    measurements: Vec<measurements::Measurement>,
    drawing: Option<Drawing>,
    layers: BTreeMap<String, LayerStyle>,
    contours: Vec<Contour>,
    materials: BTreeMap<u64, Material>,
    cases: Vec<LoadCase>,
    combinations: Vec<Combination>,
    scales: Vec<State>,
    #[serde(default)]
    saved_scales: Vec<scale_context::Saved>,
    settings: Settings,
}
#[derive(Serialize, Deserialize)]
struct Settings {
    #[serde(default)]
    stiffness_variant: usize,
    #[serde(default)]
    reinforcement_method: crate::plan_trace::reinforcement::Method,
    #[serde(default)]
    reinforcement_area_view: crate::plan_trace::reinforcement::AreaView,
    #[serde(default)]
    support_height: crate::plan_trace::fem::shell::Settings,
    #[serde(default)]
    nonlinear_options: fem_ui::NonlinearOptions,
    #[serde(default)]
    buckling_options: fem_ui::BucklingOptions,
    #[serde(default)]
    soil_options: soil_settings::Options,
    #[serde(default)]
    calculate_frequencies: bool,
    #[serde(default)]
    modal_mass_options: fem_ui::ModalMassOptions,
    #[serde(default)]
    calculate_effective_lengths: bool,
    element_type: ElementType,
    mesh_size: f64,
    #[serde(default)]
    mesh_options: crate::plan_trace::quad_mesh::Options,
    #[serde(default = "rounding::default_step")]
    rounding_step: f64,
    #[serde(default)]
    rounding_options: rounding::Options,
    #[serde(default)]
    sections: sections::State,
    #[serde(default = "rounding::default_step")]
    wall_connection_tolerance: f64,
    #[serde(default = "rounding::default_tolerance")]
    simplify_tolerance: f64,
    grid: Grid,
    #[serde(default)]
    snap_settings: construction_snap::Settings,
    camera: Camera,
    fit: bool,
    load_units: ForceUnits,
    gravity: f64,
    new_load: Magnitude,
    active_load: u64,
    active_combination: u64,
    by_combinations: bool,
    variant: Variant,
    #[serde(default)]
    iso_efforts: bool,
    #[serde(default)]
    membrane_area: bool,
    #[serde(default = "default_true")]
    iso_displacements: bool,
    #[serde(default = "default_point_size")]
    point_size: f32,
    result_field: usize,
    #[serde(default = "default_displacement_axes")]
    displacement_axes: u8,
    #[serde(default)]
    repeat_sketch: bool,
    #[serde(default = "default_true")]
    left_open: bool,
    #[serde(default = "default_true")]
    right_open: bool,
    contour_locks: [bool; 3],
    loads_locked: bool,
    #[serde(default)]
    columns_locked: bool,
    #[serde(default = "default_true")]
    columns_selectable: bool,
    #[serde(default = "default_true")]
    foundation_selectable: bool,
    #[serde(default)]
    foundation_locked: bool,
    beams_locked: bool,
    #[serde(default)]
    triangulation_locked: bool,
    #[serde(default)]
    restraints_locked: bool,
    #[serde(default)]
    rigid_links_locked: bool,
    #[serde(default)]
    underlays_locked: bool,
    #[serde(default = "draw_order::default_order")]
    draw_order: Vec<u8>,
    #[serde(
        default = "selection::all_enabled",
        deserialize_with = "selection::read_filter"
    )]
    selection_filter: [bool; 14],
    #[serde(default)]
    selection_all_floors: bool,
    #[serde(default = "default_underlay_filter")]
    underlay_filter: [bool; 2],
    show_loads: bool,
    show_load_values: bool,
    #[serde(default = "default_true")]
    show_measurements: bool,
    #[serde(default = "default_true")]
    select_measurements: bool,
    show_mesh: bool,
    show_support_nodes: bool,
    show_results: bool,
    show_result_values: bool,
    #[serde(default)]
    display_text: drawing_flags::TextStyle,
    #[serde(default)]
    sum_reactions: bool,
    slab_color: [u8; 3],
    #[serde(default = "drawing_flags::default_contour_colors")]
    contour_colors: [[u8; 3]; 3],
    #[serde(default)]
    drawing_visibility: drawing_flags::Visibility,
    #[serde(default)]
    punching_scales: Vec<State>,
    #[serde(default)]
    length_scales: Vec<State>,
    #[serde(default)]
    summation_method: crate::plan_trace::fem::cut_forces::Method,
    slab_opacity: f32,
    #[serde(default = "default_true")]
    show_dxf: bool,
    show_text: bool,
    contrast_text: bool,
    counters: [u64; 4],
}
impl Project {
    fn validate(&mut self) -> Result<(), String> {
        self.validate_impl(false)
    }
    // Saving source data must not lose a user's work because a polygon needs
    // repair. Keep format/IDs/materials/references strict; only this geometric
    // issue is deferred to triangulation/export and reported on save/open.
    fn validate_source(&mut self) -> Result<(), String> {
        self.validate_impl(true)
    }
    fn validate_impl(&mut self, preserve_self_intersections: bool) -> Result<(), String> {
        self.building.validate()?;
        if !self.building.enabled && self.contours.iter().any(|c| c.kind == Kind::RigidLink) {
            return Err("АЖТ требуют режима здания".into());
        }
        if self.contours.iter().any(|c| {
            c.parent
                .is_some_and(|p| self.building.owner(p) != self.building.owner(c.id))
        }) {
            return Err("Родительский и вложенный объекты находятся на разных этажах".into());
        }
        if self.format != "mini-fem-slab-project" || self.version != 1 {
            return Err("Неподдерживаемый формат или версия проекта".into());
        }
        if self.saved_scales.len() > 100000 {
            return Err("Слишком много сохранённых шкал".into());
        }
        for scale in &self.saved_scales {
            scale.validate()?;
        }
        let s = &self.settings;
        s.support_height.validate()?;
        if self.measurements.len() > 10000
            || self.contours.len() > 100_000
            || self.cases.is_empty()
            || self.cases.len() > 4096
            || self.combinations.len() > 4096
        {
            return Err("Некорректный размер проекта".into());
        }
        if !s.display_text.valid()
            || s.sections.planes.iter().any(|p| {
                !p.offset_mm.is_finite()
                    || p.offset_mm.abs() > 1e9
                    || !p.alpha_deg.is_finite()
                    || p.alpha_deg.abs() > 360.
            })
            || !s.rounding_step.is_finite()
            || s.rounding_step <= 0.
            || !s.simplify_tolerance.is_finite()
            || s.simplify_tolerance < 0.
            || !s.snap_settings.valid()
            || !s.mesh_size.is_finite()
            || s.mesh_size < 100.
            || !s.gravity.is_finite()
            || s.gravity <= 0.
            || !s.new_load.value.is_finite()
            || !s.grid.parameters_valid()
            || !s.rounding_options.radius_mm.is_finite()
            || s.rounding_options.radius_mm <= 0.
            || !s.grid.dx.is_finite()
            || !s.grid.dy.is_finite()
            || s.grid.dx <= 0.
            || s.grid.dy <= 0.
            || s.counters.contains(&u64::MAX)
            || !s.point_size.is_finite()
            || !(1. ..=30.).contains(&s.point_size)
            || s.result_field >= crate::plan_trace::fem::FIELD_COUNT
            || s.displacement_axes > 0b111
            || !s.camera.zoom.is_finite()
            || s.camera.zoom < 1e-8
            || s.camera.zoom > 1000.
            || !s.camera.angle.is_finite()
            || s.camera.center.iter().any(|x| !x.is_finite())
            || !s.drawing_visibility.load_opacity.is_finite()
            || !(0. ..=1.).contains(&s.drawing_visibility.load_opacity)
            || !s.slab_opacity.is_finite()
            || !(0.0..=1.).contains(&s.slab_opacity)
        {
            return Err("Некорректные параметры проекта".into());
        }
        let unique = |ids: Vec<u64>| -> Result<BTreeSet<u64>, String> {
            let n = ids.len();
            let ids = ids.into_iter().collect::<BTreeSet<_>>();
            if ids.len() != n || ids.contains(&0) || ids.contains(&u64::MAX) {
                Err("Повторяющиеся или некорректные ID".into())
            } else {
                Ok(ids)
            }
        };
        let contour_ids = unique(self.contours.iter().map(|c| c.id).collect())?;
        if self
            .building
            .contour_floors
            .keys()
            .any(|id| !contour_ids.contains(id))
        {
            return Err("Этаж назначен отсутствующему объекту".into());
        }
        for c in &self.contours {
            if let Some([column, wall]) = c.placement.rigid_link_targets {
                let target_exists =
                    |id, kind| self.contours.iter().any(|t| t.id == id && t.kind == kind);
                if c.kind != Kind::RigidLink
                    || !target_exists(column, Kind::Column)
                    || !target_exists(wall, Kind::Support)
                {
                    return Err(format!(
                        "АЖТ {}: отсутствует связанная колонна или стена",
                        c.id
                    ));
                }
            }
        }
        unique(self.measurements.iter().map(|m| m.id).collect())?;
        for m in &self.measurements {
            m.validate()?;
        }
        let cases = unique(self.cases.iter().map(|c| c.id).collect())?;
        unique(self.combinations.iter().map(|c| c.id).collect())?;
        for c in &self.cases {
            if !c.gamma_f.is_finite() || c.gamma_f < 0. || !c.gamma_l.is_finite() || c.gamma_l < 0.
            {
                return Err("Некорректные коэффициенты загружений".into());
            }
        }
        for combo in &self.combinations {
            if combo
                .coefficients
                .iter()
                .any(|(id, c)| !cases.contains(id) || !c.is_finite())
            {
                return Err("Некорректное сочетание".into());
            }
        }
        let slab_ids: BTreeSet<_> = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .map(|c| c.id)
            .collect();
        for c in &mut self.contours {
            let punching = crate::plan_trace::fem::cut_forces::is_cut(c);
            if c.kind.is_load() != c.load.is_some()
                || (c.kind == Kind::MeshRegion || punching) != c.refinement.is_some()
                || (!matches!(c.kind, Kind::Hole | Kind::MeshRegion | Kind::LoadHole)
                    && !punching
                    && c.parent.is_some())
                || (punching
                    && (c.parent.is_some_and(|id| !slab_ids.contains(&id))
                        || !c
                            .refinement
                            .as_ref()
                            .is_some_and(|r| r.size.is_finite() && r.size > 0.)))
            {
                return Err(format!(
                    "Некорректные связи объекта {} ({})",
                    c.id,
                    c.kind.label()
                ));
            }
            if punching
                && c.refinement
                    .as_ref()
                    .is_some_and(|r| r.inactive_edges.iter().any(|&i| i >= c.points.len()))
            {
                return Err(format!("Некорректные рабочие рёбра объекта {}", c.id));
            }
            if c.points.len() > 4096 || c.curves.len() > 4096 {
                return Err("Слишком сложная геометрия объекта".into());
            }
            if c.curves
                .iter()
                .any(|curve| matches!(curve,Curve::Spline{controls,..} if controls.len()>512))
            {
                return Err("Слишком много точек сплайна".into());
            }
            if c.kind == Kind::Triangulation && c.grid.is_none() && c.curves.is_empty() {
                c.grid = model::GridSpec::infer(&c.points);
            }
            if c.points.iter().flatten().any(|v| !v.is_finite())
                || c.load
                    .as_ref()
                    .is_some_and(|l| !l.magnitude.value.is_finite())
            {
                return Err(format!("Объект {}: некорректные числовые данные", c.id));
            }
            c.rebuild()
                .map_err(|e| format!("{} №{}: {e}", c.kind.label(), c.id))?;
            if c.kind.is_load() && !c.load.as_ref().is_some_and(|l| cases.contains(&l.case_id)) {
                return Err("Нагрузка ссылается на отсутствующее загружение".into());
            }
        }
        for c in &self.contours {
            if let Err(e) = model::validate_candidate(&self.contours, c) {
                if !(preserve_self_intersections
                    && e == "Граница пересекает или касается сама себя")
                {
                    return Err(format!("{} №{}: {e}", c.kind.label(), c.id));
                }
            }
        }
        if self.materials.keys().any(|id| !slab_ids.contains(id)) {
            return Err("Материал ссылается на отсутствующую плиту".into());
        }
        for id in slab_ids {
            self.materials.entry(id).or_default().validate()?;
        }
        if let Some(library) = &self.stiffnesses {
            library.validate(&self.contours)?;
        }
        if !(6..=crate::plan_trace::fem::FIELD_COUNT).contains(&self.scales.len()) {
            return Err("Некорректные шкалы проекта".into());
        }
        for state in &self.scales {
            Scale::from_state(state.clone())?;
        }
        if self
            .layers
            .values()
            .any(|l| !l.width.is_finite() || !(0.1..=12.).contains(&l.width))
        {
            return Err("Некорректная толщина слоя".into());
        }
        let mut underlay_ids = BTreeSet::new();
        for u in &self.underlays {
            u.validate()?;
            if !underlay_ids.insert(u.id) {
                return Err("Повтор ID подложки".into());
            }
        }
        if self
            .building
            .underlay_floors
            .keys()
            .any(|id| !underlay_ids.contains(id))
        {
            return Err("Этаж назначен отсутствующему объекту".into());
        }
        for d in self
            .drawing
            .iter()
            .chain(self.underlays.iter().filter_map(|u| {
                if let crate::plan_trace::underlay::Data::Dxf(d) = &u.data {
                    Some(d)
                } else {
                    None
                }
            }))
        {
            let valid_color = |c: crate::plan_trace::source_dxf::Color| match c {
                crate::plan_trace::source_dxf::Color::Aci(n) => n <= 255,
                crate::plan_trace::source_dxf::Color::Rgb(n) => n <= 0xffffff,
            };
            if d.paths
                .iter()
                .map(|p| p.color)
                .chain(d.labels.iter().map(|l| l.color))
                .chain(d.layers.iter().map(|l| l.color))
                .chain(d.fills.iter().map(|(_, c, _)| *c))
                .any(|c| !valid_color(c))
            {
                return Err("Некорректный цвет подложки".into());
            }
            if !d.unit_mm.is_finite()
                || d.unit_mm <= 0.
                || d.paths.len() > 500_000
                || d.nodes.len() > 4_000_000
            {
                return Err("Некорректная подложка проекта".into());
            }
            let finite = |p: &Point| p.iter().all(|v| v.is_finite());
            if d.nodes.iter().any(|p| !finite(p))
                || d.paths.iter().flat_map(|p| &p.points).any(|p| !finite(p))
                || d.fills
                    .iter()
                    .flat_map(|(t, _, _)| t)
                    .flatten()
                    .any(|p| !finite(p))
                || d.labels.iter().any(|l| {
                    !finite(&l.point)
                        || !l.height.is_finite()
                        || l.height < 0.
                        || !l.angle.is_finite()
                        || !l.width_factor.is_finite()
                        || l.width_factor <= 0.
                        || l.anchor.iter().any(|x| !x.is_finite())
                })
            {
                return Err("Некорректная геометрия DXF-подложки".into());
            }
        }
        Ok(())
    }
    fn read(path: &Path) -> Result<Self, String> {
        Self::read_archive(crate::plan_trace::platform::read(path).map_err(|e| e.to_string())?)
    }
    fn read_archive(bytes: Vec<u8>) -> Result<Self, String> {
        let file = std::io::Cursor::new(bytes);
        let mut zip = zip::ZipArchive::new(file).map_err(|e| format!("Архив проекта: {e}"))?;
        let entry = zip
            .by_name("project.json")
            .map_err(|_| "В архиве нет project.json")?;
        if entry.size() > MAX_JSON {
            return Err("Проект превышает допустимый размер".into());
        }
        let mut bytes = vec![];
        entry
            .take(MAX_JSON + 1)
            .read_to_end(&mut bytes)
            .map_err(|e| e.to_string())?;
        if bytes.len() as u64 > MAX_JSON {
            return Err("Слишком большой проект".into());
        }
        let mut project: Self =
            serde_json::from_slice(&bytes).map_err(|e| format!("Данные проекта: {e}"))?;
        project.validate_source()?;
        Ok(project)
    }
    #[cfg(not(target_arch = "wasm32"))]
    fn write(&mut self, path: &Path) -> Result<(), String> {
        self.validate_source()?;
        let bytes = serde_json::to_vec(self).map_err(|e| e.to_string())?;
        if bytes.len() as u64 > MAX_JSON {
            return Err("Слишком большой проект".into());
        }
        let name = path
            .file_name()
            .ok_or("Не задано имя файла")?
            .to_string_lossy();
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_err(|e| e.to_string())?
            .as_nanos();
        let temp = path.with_file_name(format!(".{name}.{}.{stamp}.tmp", std::process::id()));
        let mut created = false;
        let result = (|| {
            let file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(&temp)
                .map_err(|e| e.to_string())?;
            created = true;
            let mut zip = zip::ZipWriter::new(file);
            zip.start_file(
                "project.json",
                zip::write::SimpleFileOptions::default()
                    .compression_method(zip::CompressionMethod::Deflated),
            )
            .map_err(|e| e.to_string())?;
            zip.write_all(&bytes).map_err(|e| e.to_string())?;
            let file = zip.finish().map_err(|e| e.to_string())?;
            file.sync_all().map_err(|e| e.to_string())?;
            std::fs::rename(&temp, path).map_err(|e| e.to_string())?;
            Ok(())
        })();
        if result.is_err() && created {
            let _ = std::fs::remove_file(&temp);
        }
        result
    }
    #[cfg(target_arch = "wasm32")]
    fn write(&mut self, path: &Path) -> Result<(), String> {
        crate::plan_trace::platform::write(path, self.write_archive()?).map_err(|e| e.to_string())
    }
    #[cfg(target_arch = "wasm32")]
    fn write_archive(&mut self) -> Result<Vec<u8>, String> {
        self.validate_source()?;
        let bytes = serde_json::to_vec(self).map_err(|e| e.to_string())?;
        if bytes.len() as u64 > MAX_JSON {
            return Err("Слишком большой проект".into());
        }
        let mut zip = zip::ZipWriter::new(std::io::Cursor::new(Vec::new()));
        zip.start_file(
            "project.json",
            zip::write::SimpleFileOptions::default()
                .compression_method(zip::CompressionMethod::Deflated),
        )
        .map_err(|e| e.to_string())?;
        zip.write_all(&bytes).map_err(|e| e.to_string())?;
        Ok(zip.finish().map_err(|e| e.to_string())?.into_inner())
    }

    fn state_bytes(&self) -> Option<Vec<u8>> {
        let mut value = serde_json::to_value(self).ok()?;
        if let Some(building) = value.get_mut("building").and_then(|v| v.as_object_mut()) {
            building.remove("active");
            building.remove("hidden_floors");
        }
        // Camera navigation and automatic contour locking are not source edits.
        if let Some(settings) = value.get_mut("settings").and_then(|v| v.as_object_mut()) {
            for key in [
                "camera",
                "fit",
                "contour_locks",
                "loads_locked",
                "columns_locked",
                "restraints_locked",
                "rigid_links_locked",
                "foundation_locked",
                "beams_locked",
                "triangulation_locked",
                "underlays_locked",
                "counters",
            ] {
                settings.remove(key);
            }
        }
        serde_json::to_vec(&value).ok()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn ui_settings_roundtrip_and_migrate_in_one_project() {
        use crate::plan_trace::reinforcement::Method;
        let mut source = sample();
        source.reinforcement_method = Method::Ml;
        source.sum_reactions = true;
        source.drawing_visibility.fill = false;
        source.drawing_visibility.contours = [false, true, false];
        source.drawing_visibility.loads = [true, false, false];
        source.drawing_visibility.load_colors = [[1, 2, 3], [40, 50, 60], [70, 80, 90]];
        source.selection_filter = [
            true, false, true, false, true, false, true, true, true, true, true, true, true, false,
        ];
        source.contour_colors = [[1, 2, 3], [40, 50, 60], [200, 210, 220]];
        source.display_text = drawing_flags::TextStyle {
            color: [30, 40, 50],
            size: 18.,
            decimals: 4,
        };
        let mut data = serde_json::to_value(source.project_snapshot()).unwrap();
        let mut app = App::default();
        app.apply_project(
            serde_json::from_value(data.clone()).unwrap(),
            PathBuf::from("settings.slab.zip"),
        )
        .unwrap();
        assert!(app.reinforcement_method == Method::Ml);
        assert!(app.sum_reactions);
        assert!(!app.drawing_visibility.fill);
        assert_eq!(app.drawing_visibility.contours, [false, true, false]);
        assert_eq!(app.drawing_visibility.loads, [true, false, false]);
        assert_eq!(
            app.drawing_visibility.load_colors,
            source.drawing_visibility.load_colors
        );
        assert_eq!(app.display_text.color, [30, 40, 50]);
        assert_eq!(app.display_text.size, 18.);
        assert_eq!(app.display_text.decimals, 4);
        assert_eq!(app.selection_filter, source.selection_filter);
        assert_eq!(app.contour_colors, source.contour_colors);
        assert_eq!(
            app.contour_color(Kind::Support),
            Color32::from_rgb(200, 210, 220)
        );
        assert!(app.selected.is_none() && app.selection_ids.is_empty());
        data["settings"]["display_text"]["decimals"] = 9.into();
        assert!(app
            .apply_project(
                serde_json::from_value(data.clone()).unwrap(),
                PathBuf::from("bad.slab.zip")
            )
            .is_err());
        let settings = data["settings"].as_object_mut().unwrap();
        for key in [
            "reinforcement_method",
            "sum_reactions",
            "drawing_visibility",
            "display_text",
            "selection_filter",
            "contour_colors",
        ] {
            settings.remove(key);
        }
        app.apply_project(
            serde_json::from_value(data).unwrap(),
            PathBuf::from("legacy.slab.zip"),
        )
        .unwrap();
        assert!(app.reinforcement_method == Method::Ndm);
        assert!(!app.sum_reactions);
        assert!(app.drawing_visibility.fill);
        assert_eq!(app.drawing_visibility.contours, [true; 3]);
        assert_eq!(app.drawing_visibility.loads, [true; 3]);
        assert_eq!(app.display_text.decimals, 2);
        assert_eq!(app.display_text.size, 12.);
        assert_eq!(app.display_text.color, [255; 3]);
        assert_eq!(app.selection_filter, [true; 14]);
        assert_eq!(app.contour_colors, drawing_flags::default_contour_colors());
    }
    #[test]
    fn object_names_and_level_placement_roundtrip_and_legacy_defaults() {
        use crate::plan_trace::model::{LevelAnchor, LevelRef};
        let mut a = sample();
        a.building.enabled = true;
        a.contours[0].name = "Плита со смещением".into();
        a.contours[0].placement.slab = LevelRef {
            anchor: LevelAnchor::Top,
            offset_mm: -150.,
        };
        let mut p = a.project_snapshot();
        p.validate().unwrap();
        let mut value = serde_json::to_value(p).unwrap();
        let mut restored = App::default();
        restored
            .apply_project(
                serde_json::from_value(value.clone()).unwrap(),
                PathBuf::from("levels.zip"),
            )
            .unwrap();
        assert_eq!(restored.contours[0].name, "Плита со смещением");
        assert_eq!(restored.contours[0].placement, a.contours[0].placement);
        value["contours"][0]
            .as_object_mut()
            .unwrap()
            .remove("placement");
        value["contours"][0].as_object_mut().unwrap().remove("name");
        restored
            .apply_project(
                serde_json::from_value(value).unwrap(),
                PathBuf::from("legacy.zip"),
            )
            .unwrap();
        assert!(restored.contours[0].placement.is_default());
        assert!(restored.contours[0].name.is_empty());
    }
    #[test]
    fn columns_properties_and_rigid_settings_roundtrip() {
        let mut a = App::default();
        a.begin(Kind::Column);
        a.sketch_click([1000., 2000.], false);
        a.sync_stiffness_sources();
        a.support_height.rigid_columns = true;
        a.columns_locked = true;
        let mut p = a.project_snapshot();
        p.validate().unwrap();
        let mut preview: Project =
            serde_json::from_slice(&serde_json::to_vec(&p).unwrap()).unwrap();
        preview.settings.support_height.enabled = true;
        preview.settings.drawing_visibility.support_extents = true;
        preview
            .write(Path::new("target/columns_smoke.slab.zip"))
            .unwrap();
        let encoded = serde_json::to_vec(&p).unwrap();
        let mut b = App::default();
        b.apply_project(
            serde_json::from_slice(&encoded).unwrap(),
            PathBuf::from("columns.slab.zip"),
        )
        .unwrap();
        assert_eq!(b.contours[0].kind, Kind::Column);
        assert!(b.support_height.rigid_columns && b.columns_locked);
        assert_eq!(b.support_height.columns[&b.contours[0].id].width_mm, 250.);
    }
    #[test]
    fn named_stiffnesses_migrate_legacy_and_roundtrip_shared_properties() {
        let mut a = sample();
        let mut legacy = a.project_snapshot();
        legacy.stiffnesses = None;
        let mut restored = App::default();
        restored
            .apply_project(legacy, PathBuf::from("legacy.slab.zip"))
            .unwrap();
        assert_eq!(restored.materials[&1].thickness_mm, 220.);
        let slab_property = restored.stiffnesses.assignments[&1];
        restored
            .stiffnesses
            .entries
            .get_mut(&slab_property)
            .unwrap()
            .name = "Перекрытие".into();
        restored
            .stiffnesses
            .entries
            .get_mut(&slab_property)
            .unwrap()
            .material
            .thickness_mm = 310.;
        restored.sync_stiffness_sources();
        assert_eq!(restored.materials[&1].thickness_mm, 310.);
        let bytes = serde_json::to_vec(&restored.project_snapshot()).unwrap();
        let mut reopened = App::default();
        reopened
            .apply_project(
                serde_json::from_slice(&bytes).unwrap(),
                PathBuf::from("shared.slab.zip"),
            )
            .unwrap();
        assert_eq!(reopened.stiffnesses.assignments[&1], slab_property);
        assert_eq!(
            reopened.stiffnesses.entries[&slab_property].name,
            "Перекрытие"
        );
        assert_eq!(reopened.materials[&1].thickness_mm, 310.);
        assert!(reopened
            .support_height
            .materials
            .values()
            .all(|m| m.weight.value == 0.));
        // A direct legacy App cache remains migratable when no reference exists.
        a.stiffnesses.assignments.clear();
        a.sync_stiffness_sources();
        assert_eq!(a.materials[&1].thickness_mm, 220.);
    }
    #[test]
    fn close_request_is_cancelled_only_for_unsaved_changes() {
        fn request(app: &mut App) -> egui::FullOutput {
            let ctx = egui::Context::default();
            let mut input = egui::RawInput::default();
            input
                .viewports
                .get_mut(&egui::ViewportId::ROOT)
                .unwrap()
                .events
                .push(egui::ViewportEvent::Close);
            ctx.run(input, |ctx| app.project_close_guard(ctx))
        }
        let mut app = App::default();
        app.saved_project_state = app.project_state_bytes();
        let out = request(&mut app);
        assert!(!app.close_confirm);
        assert!(!out.viewport_output[&egui::ViewportId::ROOT]
            .commands
            .contains(&egui::ViewportCommand::CancelClose));
        app.gravity = 9.81;
        let out = request(&mut app);
        assert!(app.close_confirm);
        assert!(out.viewport_output[&egui::ViewportId::ROOT]
            .commands
            .contains(&egui::ViewportCommand::CancelClose));
        app.close_allowed = true;
        app.close_confirm = false;
        let out = request(&mut app);
        assert!(!out.viewport_output[&egui::ViewportId::ROOT]
            .commands
            .contains(&egui::ViewportCommand::CancelClose));
    }
    #[test]
    fn unsaved_state_tracks_sources_and_undo_but_not_camera_or_mesh_locks() {
        let mut app = App::default();
        let baseline = app.project_state_bytes().unwrap();
        app.saved_project_state = Some(baseline.clone());
        app.camera.center = [123., 456.];
        app.fit = !app.fit;
        app.set_contour_lock(true);
        assert_eq!(app.project_state_bytes().unwrap(), baseline);
        app.set_contour_lock(false);
        app.load_cases[0].gamma_f = 1.9;
        assert_ne!(app.project_state_bytes().unwrap(), baseline);
        app.load_cases[0].gamma_f = App::default().load_cases[0].gamma_f;
        assert_eq!(app.project_state_bytes().unwrap(), baseline);
        app.mesh_size += 100.;
        assert_ne!(app.project_state_bytes().unwrap(), baseline);
    }
    #[test]
    #[ignore = "requires MINI_FEM_PROJECT_FIXTURE pointing to a local .slab.zip"]
    fn local_project_real_mesh_options_and_spatial_supports() {
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").expect("fixture path"));
        let mut app = App::default();
        app.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        let before = serde_json::to_value(app.project_snapshot()).unwrap();
        let mesh = crate::plan_trace::meshing::generate(&app.contours, app.mesh_size).unwrap();
        let mesh = crate::plan_trace::quad_mesh::process(mesh, app.mesh_options).unwrap();
        for e in &mesh.elements {
            let xy = e.nodes.map(|n| mesh.nodes[n].xy.map(|v| v / 1000.));
            crate::plan_trace::fem::element::Triangle::new(xy).unwrap();
        }
        let flat = crate::plan_trace::fem::solve_load_cases(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_size,
            ElementType::Dkt,
        )
        .unwrap();
        for c in &flat.cases {
            assert!(c.balance_kn.abs() < 1e-5 * c.applied_kn.abs().max(1.));
        }
        let mut settings = app.support_height.clone();
        settings.enabled = true;
        settings.height_mm = 2000.;
        for direction in [
            crate::plan_trace::fem::shell::Direction::Up,
            crate::plan_trace::fem::shell::Direction::Down,
            crate::plan_trace::fem::shell::Direction::Both,
        ] {
            settings.direction = direction;
            let r = crate::plan_trace::fem::solve_with_support_height(
                &mesh,
                &app.contours,
                &app.materials,
                &app.load_cases,
                app.gravity,
                app.mesh_size,
                ElementType::Dkt,
                &settings,
            )
            .unwrap();
            for c in &r.cases {
                assert!(c.balance_kn.abs() < 1e-5 * c.applied_kn.abs().max(1.));
                assert!(c
                    .balance_m_knm
                    .iter()
                    .all(|m| m.abs() < 1e-4 * c.applied_kn.abs().max(1.)));
                assert!(c.relative_residual < 1e-6);
                println!(
                    "{direction:?}: case {} nodes {} Q4 {} free {} load {} balance {}",
                    c.id,
                    mesh.nodes.len(),
                    mesh.quads.len(),
                    r.free_dofs,
                    c.applied_kn,
                    c.balance_kn
                );
            }
        }
        assert!(crate::plan_trace::fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_size,
            ElementType::MindlinCr,
            &settings
        )
        .is_err());
        assert_eq!(
            serde_json::to_value(app.project_snapshot()).unwrap(),
            before
        );
    }
    #[test]
    #[ignore = "requires MINI_FEM_PROJECT_FIXTURE pointing to a local .slab.zip"]
    fn local_project_support_chains_have_no_gaps() {
        use mini_fem::geo::{Area, BooleanOps, LineString, MultiLineString, MultiPolygon, Polygon};
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").expect("fixture path"));
        let p = Project::read(&path).unwrap();
        let mesh = crate::plan_trace::meshing::generate(&p.contours, p.settings.mesh_size).unwrap();
        let poly = |c: &Contour| {
            Polygon::new(
                LineString::from(c.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
                vec![],
            )
        };
        let mut domain = MultiPolygon(vec![]);
        for slab in p.contours.iter().filter(|c| c.kind == Kind::Slab) {
            let mut area = MultiPolygon(vec![poly(slab)]);
            for hole in p
                .contours
                .iter()
                .filter(|c| c.parent == Some(slab.id) && c.kind == Kind::Hole)
            {
                area = area.difference(&poly(hole));
            }
            domain = domain.union(&area);
        }
        let mut missing = Vec::new();
        for c in p
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Support && c.curves.is_empty())
        {
            let mut points = c.points.clone();
            if c.closed() {
                points.push(points[0]);
            }
            let line = MultiLineString(vec![LineString::from(
                points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>(),
            )]);
            let mut expected = 0.;
            let mut covered = 0.;
            for region in &domain.0 {
                for part in region.clip(&line, false).0 {
                    for s in part.lines() {
                        expected += (s.end.x - s.start.x).hypot(s.end.y - s.start.y);
                    }
                }
            }
            // Check against authoritative support coordinates, not quantized
            // Boolean clip output (that was one cause of missing restraints).
            for pair in points.windows(2) {
                let a = pair[0];
                let b = pair[1];
                let d = [b[0] - a[0], b[1] - a[1]];
                let length = d[0].hypot(d[1]);
                for edge in &mesh.edges {
                    let q = edge.map(|i| mesh.nodes[i].xy);
                    let on = |p: Point| {
                        let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / (length * length);
                        (-1e-8..=1. + 1e-8).contains(&t)
                            && ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / length < 1e-5
                    };
                    if q.iter().all(|&p| on(p)) {
                        covered += (q[1][0] - q[0][0]).hypot(q[1][1] - q[0][1]);
                        assert!(
                            edge.iter()
                                .all(|i| mesh.support_boundary[&c.id].contains(i)),
                            "support {} unrestrained node",
                            c.id
                        );
                    }
                }
            }
            println!(
                "support {}: expected={expected:.6} covered={covered:.6}",
                c.id
            );
            if (expected - covered).abs() > 0.001 {
                missing.push(c.id);
            }
        }
        assert!(
            missing.is_empty(),
            "support chains with gaps: {missing:?}; domain area {}",
            domain.unsigned_area()
        );
        println!(
            "nodes={} elements={}",
            mesh.nodes.len(),
            mesh.elements.len()
        );
    }
    #[test]
    #[ignore = "requires MINI_FEM_PROJECT_FIXTURE pointing to a local .slab.zip"]
    fn local_project_triangulates_and_solves_all_cases() {
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").expect("fixture path"));
        let p = Project::read(&path).unwrap();
        let mesh = crate::plan_trace::meshing::generate(&p.contours, p.settings.mesh_size).unwrap();
        let cases = p.cases.iter().map(|c| c.id).collect::<Vec<_>>();
        for kind in ElementType::ALL {
            let results = crate::plan_trace::fem::solve_load_cases(
                &mesh,
                &p.contours,
                &p.materials,
                &p.cases,
                p.settings.gravity,
                p.settings.mesh_size,
                kind,
            )
            .unwrap();
            assert_eq!(results.cases.len(), cases.len());
            for c in results.cases {
                assert!(c.relative_residual < 1e-6);
                assert!(c.balance_kn.abs() < 1e-5 * c.applied_kn.abs().max(1.));
                println!(
                    "{}: nodes={} elements={} case={} force={} kN balance={} residual={}",
                    kind.label(),
                    mesh.nodes.len(),
                    mesh.elements.len(),
                    c.id,
                    c.applied_kn,
                    c.balance_kn,
                    c.relative_residual
                );
            }
        }
    }
    #[test]
    fn project_preserves_independent_pdf_and_dxf_underlays_without_derived_proxy() {
        let mut a = sample();
        a.append_dxf(a.drawing.clone().unwrap());
        a.underlays[0].scale = 3.;
        a.underlays[0].scale_y = Some(1.5);
        a.underlays[0].angle_rad = 0.4;
        a.underlays[0].transparency = 0.35;
        a.underlays[0].origin = [42., 84.];
        a.underlays.push(crate::plan_trace::underlay::Underlay {
            transparency: 0.6,
            id: 3,
            name: "page.pdf".into(),
            visible: true,
            scale: 100.,
            scale_y: Some(80.),
            angle_rad: -0.5,
            origin: [100., 200.],
            data: crate::plan_trace::underlay::Data::Pdf {
                width: 1,
                height: 1,
                rgba: vec![255; 4],
                size_mm: [210., 297.],
                page: 1,
            },
        });
        a.sync_underlays();
        let mut p = a.project_snapshot();
        assert!(p.drawing.is_none());
        let bytes = serde_json::to_vec(&p).unwrap();
        // Exercise the actual archive rather than only JSON serialization.
        // These source names do not exist: reopening must use embedded data.
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!(
            "mini-fem-underlays-{}-{stamp}.slab.zip",
            std::process::id()
        ));
        p.write(&path).unwrap();
        let restored = Project::read(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        let mut b = App::default();
        b.apply_project(restored, path).unwrap();
        assert_eq!(b.underlays.len(), 3);
        assert_eq!(b.underlays[0].scale, 3.);
        assert_eq!(b.underlays[0].transform().scale, [3., 1.5]);
        assert_eq!(b.underlays[0].transform().angle, 0.4);
        assert_eq!(b.underlays[2].transform().scale, [100., 80.]);
        assert_eq!(b.underlays[2].transform().angle, -0.5);
        assert_eq!(b.underlays[0].transparency, 0.35);
        assert_eq!(b.underlays[1].transparency, 0.);
        assert_eq!(b.underlays[2].transparency, 0.6);
        assert_eq!(b.underlays[0].origin, [42., 84.]);
        assert!(matches!(
            &b.underlays[2].data,
            crate::plan_trace::underlay::Data::Pdf { rgba, page: 1, .. }
                if rgba == &[255; 4]
        ));
        assert!(b.drawing.as_ref().unwrap().paths.len() > 1);
        assert_eq!(serde_json::to_vec(&b.project_snapshot()).unwrap(), bytes);
    }
    #[test]
    fn legacy_eight_scales_expand_to_independent_line_and_point_scales() {
        let mut p = sample().project_snapshot();
        p.scales.truncate(8);
        p.settings.result_field = 7;
        let original = serde_json::to_value(&p.scales[7]).unwrap();
        let mut a = App::default();
        a.apply_project(p, PathBuf::from("old.slab.zip")).unwrap();
        assert_eq!(a.result_field, 7);
        assert_eq!(a.result_scales.len(), crate::plan_trace::fem::FIELD_COUNT);
        assert_eq!(
            serde_json::to_value(a.result_scales[7].state()).unwrap(),
            original
        );
        a.result_field = 9;
        let mut updated = a.project_snapshot();
        updated.validate().unwrap();
        assert_eq!(updated.scales.len(), crate::plan_trace::fem::FIELD_COUNT);
    }
    #[test]
    fn effective_length_scales_roundtrip_separately_from_reinforcement() {
        let mut app = sample();
        app.result_field = 28;
        app.length_view = 1;
        app.show_results = false;
        assert_eq!(app.length_scale_index(), Some(0));
        app.length_scales[0].reset_values([2., 6.]).unwrap();
        app.length_view = 2;
        assert_eq!(app.length_scale_index(), Some(1));
        app.length_scales[1].reset_values([0.7, 1.3]).unwrap();
        app.show_results = true;
        assert_eq!(app.length_scale_index(), None);
        let before = app.length_scales.each_ref().map(Scale::signature);
        let mut loaded = App::default();
        loaded
            .apply_project(
                app.project_snapshot(),
                PathBuf::from("length_scales.slab.zip"),
            )
            .unwrap();
        assert_eq!(
            loaded.length_scales.each_ref().map(Scale::signature),
            before
        );
        assert_ne!(before[0], before[1]);
        assert_eq!(
            loaded.result_scales[28].signature(),
            app.result_scales[28].signature()
        );
    }

    #[test]
    fn self_weight_case_and_volumetric_weight_persist_and_old_materials_get_defaults() {
        use crate::plan_trace::loads::LoadType;
        let mut a = sample();
        a.load_cases[0].kind = LoadType::SelfWeight;
        a.materials.get_mut(&1).unwrap().weight = Magnitude {
            value: 24.,
            units: ForceUnits::KiloNewton,
        };
        let mut data = serde_json::to_value(a.project_snapshot()).unwrap();
        let mut b = App::default();
        b.apply_project(
            serde_json::from_value(data.clone()).unwrap(),
            PathBuf::from("weight.slab.zip"),
        )
        .unwrap();
        assert!(b.load_cases[0].kind == LoadType::SelfWeight);
        assert_eq!(b.materials[&1].weight, a.materials[&1].weight);
        data["materials"]["1"]
            .as_object_mut()
            .unwrap()
            .remove("weight");
        // A genuinely old project has no shared property library to override
        // this legacy material default.
        data.as_object_mut().unwrap().remove("stiffnesses");
        b.apply_project(
            serde_json::from_value(data).unwrap(),
            PathBuf::from("old.slab.zip"),
        )
        .unwrap();
        assert_eq!(b.materials[&1].weight, Material::default().weight);
        assert_eq!(b.materials[&1].weight.value, 2.75);
        assert_eq!(
            b.materials[&1].weight.in_units(ForceUnits::KiloNewton, 10.),
            27.5
        );
        data_weight_validation();
    }
    #[test]
    fn snap_preferences_roundtrip_old_defaults_and_invalid_step_rejected() {
        let mut a = sample();
        a.snap_settings = construction_snap::Settings {
            midpoint: false,
            grid_nodes: false,
            geometry_nodes: false,
            nearest: false,
            polar: true,
            angle_step: 22.5,
        };
        let mut data = serde_json::to_value(a.project_snapshot()).unwrap();
        let mut b = App::default();
        b.apply_project(
            serde_json::from_value(data.clone()).unwrap(),
            PathBuf::from("snaps.slab.zip"),
        )
        .unwrap();
        assert_eq!(b.snap_settings.angle_step, 22.5);
        assert!(
            b.snap_settings.polar
                && !b.snap_settings.grid_nodes
                && !b.snap_settings.geometry_nodes
                && !b.snap_settings.nearest
        );
        data["settings"]["snap_settings"]["angle_step"] = serde_json::json!(0.);
        assert!(b
            .apply_project(
                serde_json::from_value(data.clone()).unwrap(),
                PathBuf::from("bad.slab.zip")
            )
            .is_err());
        data["settings"]
            .as_object_mut()
            .unwrap()
            .remove("snap_settings");
        b.apply_project(
            serde_json::from_value(data).unwrap(),
            PathBuf::from("old.slab.zip"),
        )
        .unwrap();
        assert!(
            b.snap_settings.grid_nodes
                && b.snap_settings.geometry_nodes
                && b.snap_settings.nearest
                && !b.snap_settings.polar
        );
    }
    #[test]
    fn measurement_annotations_and_rounding_step_roundtrip_with_legacy_defaults() {
        let mut a = sample();
        a.rounding_step = 25.;
        a.rounding_options = rounding::Options {
            cluster: true,
            radius_mm: 50.,
            by_floor: false,
        };
        a.sections.planes[0] = sections::Plane {
            enabled: true,
            offset_mm: 1234.,
            positive: false,
            ..Default::default()
        };
        a.construction_grid.color = [10, 20, 30];
        a.construction_grid.width_mm = 40000.;
        a.measurements.show = false;
        a.measurements.selectable = false;
        a.measurements.items = vec![measurements::Measurement {
            spatial: None,
            offset: Some(-300.),
            id: 7,
            a: [-250., 100.],
            b: [750., 100.],
        }];
        let mut data = serde_json::to_value(a.project_snapshot()).unwrap();
        let mut b = App::default();
        b.apply_project(
            serde_json::from_value(data.clone()).unwrap(),
            PathBuf::from("dimensions.slab.zip"),
        )
        .unwrap();
        assert_eq!(b.rounding_step, 25.);
        assert!(b.rounding_options.cluster && !b.rounding_options.by_floor);
        assert_eq!(b.rounding_options.radius_mm, 50.);
        assert_eq!(b.sections.planes[0].offset_mm, 1234.);
        assert!(b.sections.planes[0].enabled && !b.sections.planes[0].positive);
        assert_eq!(b.construction_grid.color, [10, 20, 30]);
        assert_eq!(b.construction_grid.width_mm, 40000.);
        assert!(!b.measurements.show);
        assert!(!b.measurements.active && !b.measurements.open);
        assert_eq!(b.measurements.items.len(), 1);
        assert_eq!(b.measurements.items[0].length(), 1000.);
        assert_eq!(b.measurements.items[0].offset, Some(-300.));
        assert!(!b.measurements.selectable);
        let mut bad = data.clone();
        bad["measurements"][0]["b"] = serde_json::json!([-250., 100.]);
        assert!(b
            .apply_project(
                serde_json::from_value(bad).unwrap(),
                PathBuf::from("bad.slab.zip")
            )
            .is_err());
        data.as_object_mut().unwrap().remove("measurements");
        for name in ["show_measurements", "select_measurements", "rounding_step"] {
            data["settings"].as_object_mut().unwrap().remove(name);
        }
        b.apply_project(
            serde_json::from_value(data).unwrap(),
            PathBuf::from("old.slab.zip"),
        )
        .unwrap();
        assert!(
            b.measurements.show && b.measurements.selectable && b.measurements.items.is_empty()
        );
        assert_eq!(b.rounding_step, 10.);
    }
    fn data_weight_validation() {
        for weight in [-1., f64::NAN, f64::INFINITY] {
            let mut m = Material::default();
            m.weight.value = weight;
            assert!(m.validate().is_err());
        }
    }
    #[test]
    #[ignore = "requires MINI_FEM_SELF_CONTACT_PROJECT"]
    fn real_self_contact_project_allows_incremental_vertex_repair_and_undo() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_SELF_CONTACT_PROJECT").unwrap());
        let p = Project::read(&path).unwrap();
        let mut a = App::default();
        a.apply_project(p, path).unwrap();
        a.input_locked = false;
        a.set_kind_lock(Kind::Slab, false);
        let old = a.contours.iter().find(|c| c.id == 522).unwrap().clone();
        assert_eq!(model::self_contacts(&old.points), 2);
        a.change_vertex(522, 64, false);
        assert_eq!(
            a.contours
                .iter()
                .find(|c| c.id == 522)
                .unwrap()
                .points
                .len(),
            old.points.len() - 1,
            "{}",
            a.status
        );
        a.change_vertex(522, 83, false);
        let fixed = a.contours.iter().find(|c| c.id == 522).unwrap();
        assert!(model::valid(&fixed.points).is_ok());
        assert_eq!(
            model::without_collinear_spurs(old.points.clone()),
            fixed.points
        );
        a.undo();
        a.undo();
        assert_eq!(
            a.contours.iter().find(|c| c.id == 522).unwrap().points,
            old.points
        );
    }
    #[test]
    fn source_save_preserves_self_crossing_outline_but_analysis_validation_stays_strict() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([4000., 4000.], false);
        let mut p = a.project_snapshot();
        p.contours[0].points = vec![[0., 0.], [4000., 4000.], [0., 4000.], [4000., 0.]];
        assert!(p.validate().unwrap_err().contains("Граница"));
        p.validate_source().unwrap();
        let raw = serde_json::to_vec(&p).unwrap();
        let mut restored: Project = serde_json::from_slice(&raw).unwrap();
        restored.validate_source().unwrap();
        assert_eq!(restored.contours[0].points, p.contours[0].points);
        assert!(restored.validate().is_err());
    }
    fn sample() -> App {
        let mut a = App::default();
        let d=import::from_bytes(b"0\nSECTION\n2\nENTITIES\n0\nLINE\n8\nPlan\n10\n0\n20\n0\n11\n6000\n21\n0\n0\nTEXT\n8\nPlan\n10\n50\n20\n100\n40\n100\n1\nTest\n0\nENDSEC\n0\nEOF\n","Подложка.dxf").unwrap();
        a.append_dxf(d);
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([6000., 4000.], false);
        a.begin(Kind::Hole);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([4000., 2500.], false);
        a.sketch_click([5000., 3500.], false);
        a.begin(Kind::Support);
        a.choose_tool(Tool::Arc);
        a.sketch_click([1000., 1500.], false);
        a.sketch_click([3000., 1700.], false);
        a.sketch_click([5000., 1500.], false);
        a.finish();
        a.selected = Some(1);
        a.begin(Kind::MeshRegion);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([500., 500.], false);
        a.sketch_click([1500., 1500.], false);
        a.begin(Kind::LoadPoint);
        a.sketch_click([3000., 2000.], false);
        a.load_cases.push(LoadCase::new(2));
        a.load_cases[1].name = "Длительное".into();
        a.load_cases[1].gamma_l = 0.4;
        a.active_load = 2;
        a.next_load = 3;
        a.begin(Kind::LoadLine);
        a.sketch_click([500., 500.], false);
        a.sketch_click([5000., 500.], false);
        a.finish();
        a.combinations[0].coefficients.insert(2, -0.5);
        a.materials.insert(
            1,
            Material {
                thickness_mm: 220.,
                ..Default::default()
            },
        );
        a.element_type = ElementType::MindlinCr;
        a.construction_grid = Grid {
            enabled: true,
            dx: 200.,
            dy: 250.,
            ..Default::default()
        };
        a.show_load_values = false;
        a.layers
            .entry("#1 · Plan".into())
            .or_insert(LayerStyle {
                visible: true,
                color: None,
                original: [255; 3],
                width: 1.,
            })
            .color = Some([30, 60, 90]);
        a
    }

    #[test]
    fn contour_tree_selection_switches_floor_and_respects_visibility_and_filter() {
        let mut app = sample();
        let slab = app.contours[0].clone();
        app.contours = (1..=5)
            .map(|id| Contour {
                id,
                kind: if id == 4 { Kind::Hole } else { Kind::Slab },
                ..slab.clone()
            })
            .collect();
        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors = BTreeMap::from([(2, 2), (3, 2), (4, 2), (5, 2)]);
        app.spatial.enabled = true;
        app.selection_all_floors = false;
        app.view_visibility.contours.insert(3);
        app.selection_filter[1] = false;

        app.select_contour_tree(vec![2, 3, 4, 5]);

        assert_eq!(app.building.active, 2);
        assert_eq!(app.selection_ids, BTreeSet::from([2, 5]));
        assert!(app.tree_load_selection.is_empty());
        app.building.hidden_floors.insert(2);
        app.select_contour_tree(vec![2, 5]);
        assert!(app.selection_ids.is_empty());
    }

    #[test]
    fn project_roundtrip_with_no_combinations_falls_back_to_load_case() {
        let mut p = sample().project_snapshot();
        p.combinations.clear();
        p.settings.by_combinations = true;
        p.validate().unwrap();
        let decoded: Project = serde_json::from_slice(&serde_json::to_vec(&p).unwrap()).unwrap();
        let mut restored = App::default();
        restored
            .apply_project(decoded, PathBuf::from("empty_combinations.slab.zip"))
            .unwrap();
        assert!(restored.combinations.is_empty());
        assert!(!restored.result_by_combinations);
        restored.project_snapshot().validate().unwrap();
    }
    #[test]
    fn project_roundtrip_is_self_contained_and_keeps_geometry_cases_and_settings() {
        let mut a = sample();
        a.result_field = crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD;
        a.displacement_axes = 0b101;
        a.repeat_sketch = true;
        a.combinations[0].name = "Основное сочетание".into();
        if let Some(load) = a.contours.iter_mut().find_map(|c| c.load.as_mut()) {
            load.name = "Оборудование".into();
            load.mesh_constraint = true;
        }
        let mut p = a.project_snapshot();
        p.validate().unwrap();
        let json = serde_json::to_vec(&p).unwrap();
        let value: serde_json::Value = serde_json::from_slice(&json).unwrap();
        assert!(value.get("fe_mesh").is_none() && value.get("analysis").is_none());
        let decoded: Project = serde_json::from_slice(&json).unwrap();
        let mut restored = App::default();
        restored
            .apply_project(decoded, PathBuf::from("test.slab.zip"))
            .unwrap();
        assert_eq!(a.contours, restored.contours);
        assert_eq!(restored.element_type, ElementType::MindlinCr);
        assert_eq!(restored.displacement_axes, 0b101);
        assert_eq!(restored.result_field, a.result_field);
        assert!(restored.repeat_sketch);
        assert!(restored
            .contours
            .iter()
            .any(|c| c.kind == Kind::Support && !c.closed() && !c.curves.is_empty()));
        assert_eq!(restored.load_cases[1].name, "Длительное");
        assert_eq!(restored.combinations[0].coefficients[&2], -0.5);
        assert_eq!(restored.combinations[0].name, "Основное сочетание");
        assert_eq!(restored.drawing.as_ref().unwrap().labels[0].text, "Test");
        assert!(!restored.show_load_values);
        assert!(
            restored.fe_mesh.is_none()
                && restored.analysis.is_none()
                && restored.history.is_empty()
        );
        assert_eq!(
            serde_json::to_value(a.project_snapshot()).unwrap(),
            serde_json::to_value(restored.project_snapshot()).unwrap()
        );
    }
    #[test]
    fn hidden_dxf_is_stored_in_project_but_not_used_for_snapping_or_bounds() {
        let mut a = sample();
        let count = a.drawing.as_ref().unwrap().paths.len();
        assert!(count > 0);
        a.show_dxf = false;
        a.rebuild_index();
        assert!(a.index.query([0., 0.], 1., &[], &[]).is_none());
        let mut b = App::default();
        b.apply_project(a.project_snapshot(), PathBuf::from("hidden.slab.zip"))
            .unwrap();
        assert!(!b.show_dxf);
        assert_eq!(b.drawing.as_ref().unwrap().paths.len(), count);
        assert!(b.index.query([0., 0.], 1., &[], &[]).is_none());
        b.show_dxf = true;
        b.rebuild_index();
        assert!(b.index.query([0., 0.], 1., &[], &[]).is_some());
    }
    #[test]
    fn invalid_project_does_not_replace_current_data() {
        let mut a = sample();
        let before = serde_json::to_value(a.project_snapshot()).unwrap();
        let mut p = a.project_snapshot();
        p.cases.push(LoadCase::new(1));
        assert!(a.apply_project(p, PathBuf::from("bad.zip")).is_err());
        assert_eq!(before, serde_json::to_value(a.project_snapshot()).unwrap());
    }
    #[test]
    fn compressed_project_overwrites_the_same_file() {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path =
            std::env::temp_dir().join(format!("mini-fem-{}-{stamp}.slab.zip", std::process::id()));
        let mut a = sample();
        a.project_snapshot().write(&path).unwrap();
        a.load_cases[0].name = "Изменено".into();
        a.project_snapshot().write(&path).unwrap();
        let p = Project::read(&path).unwrap();
        assert_eq!(p.cases[0].name, "Изменено");
        assert_eq!(p.contours, a.contours);
        std::fs::remove_file(path).unwrap();
    }
}
impl App {
    pub(super) fn write_soil_project(&self, path: &Path) -> Result<(), String> {
        self.project_snapshot().write(path)
    }
    pub(super) fn load_project_path(&mut self, path: PathBuf) -> Result<(), String> {
        self.apply_project(Project::read(&path)?, path)
    }
    pub(super) fn project_snapshot(&self) -> Project {
        let mut library = self.stiffnesses.clone();
        library.reconcile(
            &self.contours,
            &self.materials,
            &self.support_height.materials,
        );
        library.split_legacy_entries();
        let library = library.snapshot(&self.contours);
        let materials = library.resolve(&self.contours, Kind::Slab);
        let mut support_height = self.support_height.clone();
        support_height.materials = library.resolve(&self.contours, Kind::Support);
        Project {
            lir_source: self.lir_source.clone(),
            building: self.building.snapshot(&self.contours, &self.underlays),
            stiffnesses: Some(library),
            format: "mini-fem-slab-project".into(),
            version: 1,
            underlays: self.underlays.clone(),
            drawing: if self.underlays.is_empty() {
                self.drawing.clone()
            } else {
                None
            },
            layers: self.layers.clone(),
            measurements: self.measurements.items.clone(),
            contours: self.contours.clone(),
            materials,
            cases: self.load_cases.clone(),
            combinations: self.combinations.clone(),
            saved_scales: self.scale_store.saved.clone(),
            scales: self.result_scales.iter().map(Scale::state).collect(),
            settings: Settings {
                stiffness_variant: self.stiffness_variant,
                reinforcement_method: self.reinforcement_method,
                reinforcement_area_view: self.reinforcement_area_view,
                support_height,
                nonlinear_options: self.nonlinear_options.clone(),
                buckling_options: self.buckling_options.clone(),
                soil_options: self.soil_options.clone(),
                calculate_frequencies: self.calculate_frequencies,
                modal_mass_options: self.modal_mass_options,
                calculate_effective_lengths: self.calculate_effective_lengths,
                element_type: self.element_type,
                mesh_size: self.mesh_size,
                mesh_options: self.mesh_options,
                rounding_step: self.rounding_step,
                rounding_options: self.rounding_options.clone(),
                sections: self.sections.clone(),
                wall_connection_tolerance: self.intersections.tolerance,
                simplify_tolerance: self.simplify_tolerance,
                grid: self.construction_grid,
                snap_settings: self.snap_settings,
                camera: self.camera,
                fit: self.fit,
                load_units: self.load_units,
                gravity: self.gravity,
                new_load: self.new_load,
                active_load: self.active_load,
                active_combination: self.active_combination,
                by_combinations: self.result_by_combinations,
                variant: self.combination_variant,
                iso_efforts: self.iso_efforts,
                membrane_area: self.membrane_area,
                iso_displacements: self.iso_displacements,
                point_size: self.point_size,
                result_field: self.result_field,
                displacement_axes: self.displacement_axes,
                repeat_sketch: self.repeat_sketch,
                left_open: self.left_open,
                right_open: self.right_open,
                contour_locks: self.contour_locks,
                loads_locked: self.loads_locked,
                columns_locked: self.columns_locked,
                columns_selectable: self.columns_selectable,
                foundation_selectable: self.foundation_selectable,
                foundation_locked: self.foundation_locked,
                beams_locked: self.beams_locked,
                triangulation_locked: self.triangulation_locked,
                restraints_locked: self.restraints_locked,
                rigid_links_locked: self.rigid_links_locked,
                underlays_locked: self.underlays_locked,
                draw_order: self.draw_order.items.clone(),
                selection_filter: self.selection_filter,
                selection_all_floors: self.selection_all_floors,
                underlay_filter: self.underlay_filter,
                show_loads: self.show_loads,
                show_load_values: self.show_load_values,
                show_measurements: self.measurements.show,
                select_measurements: self.measurements.selectable,
                show_mesh: self.show_mesh,
                show_support_nodes: self.show_support_nodes,
                show_results: self.show_results,
                show_result_values: self.show_result_values,
                display_text: self.display_text,
                sum_reactions: self.sum_reactions,
                slab_color: self.slab_color,
                contour_colors: self.contour_colors,
                drawing_visibility: self.drawing_visibility,
                punching_scales: self.punching_scales.iter().map(|s| s.state()).collect(),
                length_scales: self.length_scales.iter().map(|s| s.state()).collect(),
                summation_method: self.summation_method,
                slab_opacity: self.slab_opacity,
                show_dxf: self.show_dxf,
                show_text: self.show_text,
                contrast_text: self.contrast_text,
                counters: [
                    self.next_id,
                    self.next_load,
                    self.next_combination,
                    self.next_region,
                ],
            },
        }
    }
    pub(super) fn apply_project(&mut self, mut p: Project, path: PathBuf) -> Result<(), String> {
        if self.soil_editor_modified() {
            return Err("Сначала завершите редактирование модели грунта.".into());
        }
        p.validate_source()?;
        let mut scales = p
            .scales
            .into_iter()
            .map(Scale::from_state)
            .collect::<Result<Vec<_>, _>>()?;
        while scales.len() < crate::plan_trace::fem::FIELD_COUNT {
            scales.push(Scale::default());
        }
        let scales: [Scale; crate::plan_trace::fem::FIELD_COUNT] =
            scales.try_into().map_err(|_| "Некорректные шкалы")?;
        let s = p.settings;
        let mut next = Self {
            building: p.building,
            lir_source: p.lir_source,
            project_path: Some(path),
            stiffness_variant: if p
                .stiffnesses
                .as_ref()
                .is_some_and(|l| l.alternative_enabled)
            {
                s.stiffness_variant.min(1)
            } else {
                0
            },
            stiffnesses: p.stiffnesses.unwrap_or_default(),
            support_height: s.support_height,
            underlays: p.underlays,
            drawing: p.drawing,
            layers: p.layers,
            measurements: measurements::Tool::restore(p.measurements, s.show_measurements),
            contours: p
                .contours
                .clone()
                .into_iter()
                .map(|mut c| {
                    if super::punching::is_punching(&c) && c.kind == Kind::Auxiliary {
                        let active = super::super::fem::cut_forces::control_edges(&c, &p.contours);
                        if let Some(r) = &mut c.refinement {
                            r.inactive_edges = (0..c.points.len())
                                .filter(|&i| {
                                    !active.contains(&[
                                        c.points[i],
                                        c.points[(i + 1) % c.points.len()],
                                    ])
                                })
                                .collect();
                        }
                        c.kind = Kind::Punching;
                    }
                    if c.kind == Kind::Punching {
                        if let Some(r) = &mut c.refinement {
                            if r.name.trim().is_empty()
                                || r.name == format!("Контур суммирования нагрузок {}", c.id)
                            {
                                r.name = format!("Область суммирования нагрузок {}", c.id);
                            }
                        }
                    }
                    c
                })
                .collect(),
            materials: p.materials,
            load_cases: p.cases,
            combinations: p.combinations,
            scale_store: scale_context::Store {
                saved: p.saved_scales,
                ..Default::default()
            },
            result_scales: scales,
            nonlinear_options: s.nonlinear_options,
            buckling_options: s.buckling_options,
            soil_options: s.soil_options,
            calculate_frequencies: s.calculate_frequencies,
            modal_mass_options: s.modal_mass_options,
            calculate_effective_lengths: s.calculate_effective_lengths,
            element_type: s.element_type,
            reinforcement_method: s.reinforcement_method,
            reinforcement_area_view: s.reinforcement_area_view,
            mesh_size: s.mesh_size,
            mesh_options: s.mesh_options,
            rounding_step: s.rounding_step,
            rounding_options: s.rounding_options,
            sections: s.sections,
            intersections: intersections_ui::State::with_tolerance(s.wall_connection_tolerance),
            simplify_tolerance: s.simplify_tolerance,
            mesh_built_size: s.mesh_size,
            construction_grid: s.grid,
            snap_settings: s.snap_settings,
            camera: s.camera,
            fit: s.fit,
            load_units: s.load_units,
            gravity: s.gravity,
            new_load: s.new_load,
            active_load: s.active_load,
            active_combination: s.active_combination,
            result_by_combinations: s.by_combinations,
            combination_variant: s.variant,
            iso_efforts: s.iso_efforts,
            membrane_area: s.membrane_area,
            iso_displacements: s.iso_displacements,
            point_size: s.point_size,
            result_field: s.result_field,
            displacement_axes: s.displacement_axes,
            repeat_sketch: s.repeat_sketch,
            left_open: s.left_open,
            right_open: s.right_open,
            contour_locks: s.contour_locks,
            contours_locked: s.contour_locks.iter().all(|v| *v)
                && s.loads_locked
                && s.underlays_locked
                && s.beams_locked
                && s.columns_locked
                && s.triangulation_locked
                && s.restraints_locked
                && s.rigid_links_locked,
            loads_locked: s.loads_locked,
            columns_locked: s.columns_locked,
            columns_selectable: s.columns_selectable,
            foundation_selectable: s.foundation_selectable,
            foundation_locked: s.foundation_locked,
            beams_locked: s.beams_locked,
            triangulation_locked: s.triangulation_locked,
            restraints_locked: s.restraints_locked,
            rigid_links_locked: s.rigid_links_locked,
            underlays_locked: s.underlays_locked,
            draw_order: draw_order::Order::restore(s.draw_order),
            selection_filter: s.selection_filter,
            selection_all_floors: s.selection_all_floors,
            underlay_filter: s.underlay_filter,
            show_loads: s.show_loads,
            show_load_values: s.show_load_values,
            show_mesh: s.show_mesh,
            show_support_nodes: s.show_support_nodes,
            show_results: s.show_results,
            show_result_values: s.show_result_values,
            display_text: s.display_text,
            sum_reactions: s.sum_reactions,
            slab_color: s.slab_color,
            contour_colors: s.contour_colors,
            drawing_visibility: s.drawing_visibility.exclusive_colors(),
            summation_method: s.summation_method,
            punching_scales: std::array::from_fn(|i| {
                s.punching_scales
                    .get(i)
                    .cloned()
                    .and_then(|s| Scale::from_state(s).ok())
                    .unwrap_or_default()
            }),
            length_scales: std::array::from_fn(|i| {
                s.length_scales
                    .get(i)
                    .cloned()
                    .and_then(|s| Scale::from_state(s).ok())
                    .unwrap_or_default()
            }),
            slab_opacity: s.slab_opacity,
            show_dxf: s.show_dxf,
            show_text: s.show_text,
            contrast_text: s.contrast_text,
            ..Self::default()
        };
        next.measurements.selectable = s.select_measurements;
        next.next_id = s.counters[0].max(next.contours.iter().map(|c| c.id).max().unwrap_or(0) + 1);
        next.next_load =
            s.counters[1].max(next.load_cases.iter().map(|c| c.id).max().unwrap_or(0) + 1);
        next.next_combination =
            s.counters[2].max(next.combinations.iter().map(|c| c.id).max().unwrap_or(0) + 1);
        next.next_region = s.counters[3].max(1);
        if !next.load_cases.iter().any(|c| c.id == next.active_load) {
            next.active_load = next.load_cases[0].id;
        }
        if !next
            .combinations
            .iter()
            .any(|c| c.id == next.active_combination)
        {
            next.active_combination = next.combinations.first().map_or(0, |c| c.id);
        }
        if next.combinations.is_empty() {
            next.result_by_combinations = false;
        }
        next.result_unit_factor = next.force_result_factor();
        next.sync_slab_cuts();
        next.migrate_underlay();
        if !next.underlays.is_empty() {
            next.sync_underlays();
        } else {
            next.rebuild_index();
        }
        let ids = next
            .contours
            .iter()
            .filter(|c| {
                model::validate_candidate(&next.contours, c)
                    .is_err_and(|e| e == "Граница пересекает или касается сама себя")
            })
            .map(|c| c.id.to_string())
            .collect::<Vec<_>>();
        next.status = if ids.is_empty() {
            "Проект открыт. Постройте КЭ-сетку и выполните расчёт.".into()
        } else {
            format!(
                "Проект открыт. Требуют исправления самокасания/пересечения объектов: {}",
                ids.join(", ")
            )
        };
        self.invalidate_scene_buffers();
        next.perf = std::mem::take(&mut self.perf);
        next.started = self.started;
        next.mesh_generation = self.mesh_generation.wrapping_add(1);
        next.result_generation = self.result_generation.wrapping_add(1);
        next.sync_stiffness_sources();
        next.saved_project_state = next.project_state_bytes();
        *self = next;
        Ok(())
    }
    pub(super) fn project_controls(&mut self, ui: &mut egui::Ui) {
        let ready = self.load.is_none()
            && !self.pdf_busy()
            && self.mode.is_none()
            && self.drag.is_none()
            && self.arc_placement.is_none();
        ui.horizontal(|ui| {
            if ui
                .add_enabled(
                    self.load.is_none() && !self.pdf_busy(),
                    egui::Button::new("Открыть"),
                )
                .clicked()
            {
                if let Some(path) = rfd::FileDialog::new()
                    .add_filter("Проект Mini FEM", &["zip"])
                    .pick_file()
                {
                    match Project::read(&path).and_then(|p| self.apply_project(p, path)) {
                        Ok(()) => {}
                        Err(e) => self.status = format!("Проект: {e}"),
                    }
                }
            }
            let save = ui.add_enabled(ready, egui::Button::new("Сохранить"));
            if save.clicked() {
                self.save_project(false);
            }
            if ui
                .add_enabled(ready, egui::Button::new("Сохранить как…"))
                .clicked()
            {
                self.save_project(true);
            }
        });
        if let Some(path) = &self.project_path {
            ui.add(
                egui::Label::new(format!(
                    "{}{}",
                    path.file_name().unwrap_or_default().to_string_lossy(),
                    saved_time_label(path)
                ))
                .truncate(),
            )
            .on_hover_text(path.display().to_string());
        }
        #[cfg(target_arch = "wasm32")]
        self.opfs_controls(ui, ready);
        ui.separator();
    }
    pub(super) fn save_project(&mut self, save_as: bool) -> bool {
        self.sync_slab_cuts();
        let path = if !save_as {
            self.project_path.clone()
        } else {
            None
        };
        let path = path.or_else(|| {
            rfd::FileDialog::new()
                .add_filter("Проект Mini FEM", &["zip"])
                .set_file_name("Проект.slab.zip")
                .save_file()
        });
        let Some(path) = path else {
            return false;
        };
        match self.project_snapshot().write(&path) {
            Ok(()) => {
                self.project_path = Some(path);
                let ids = self
                    .contours
                    .iter()
                    .filter(|c| {
                        model::validate_candidate(&self.contours, c)
                            .is_err_and(|e| e == "Граница пересекает или касается сама себя")
                    })
                    .map(|c| c.id.to_string())
                    .collect::<Vec<_>>();
                self.status = if ids.is_empty() {
                    "Проект сохранён".into()
                } else {
                    format!(
                        "Проект сохранён. Требуют исправления самокасания/пересечения объектов: {}",
                        ids.join(", ")
                    )
                };
                self.saved_project_state = self.project_state_bytes();
                true
            }
            Err(e) => {
                self.status = format!("Сохранение проекта: {e}");
                false
            }
        }
    }
    pub(super) fn project_state_bytes(&self) -> Option<Vec<u8>> {
        self.project_snapshot().state_bytes()
    }
    pub(super) fn project_close_guard(&mut self, ctx: &egui::Context) {
        if self.smoke {
            return;
        }
        if self.saved_project_state.is_none() {
            self.saved_project_state = self.project_state_bytes();
        }
        if self.defer_close_for_soil_editor(ctx) {
            return;
        }
        if ctx.input(|i| i.viewport().close_requested()) && !self.close_allowed {
            if self.project_state_bytes() != self.saved_project_state || !self.draft.is_empty() {
                ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
                self.close_confirm = true;
            }
        }
        if !self.close_confirm {
            return;
        }
        egui::Modal::new(egui::Id::new("save_before_close")).show(ctx, |ui| {
            ui.heading("Сохранить изменения проекта?");
            ui.label("В проекте есть несохранённые изменения.");
            if self.mode.is_some() { ui.label("Незавершённый объект не входит в файл. Отмените закрытие и завершите построение."); }
            ui.horizontal(|ui| {
                if ui.add_enabled(self.mode.is_none() && self.drag.is_none() && self.load.is_none() && !self.pdf_busy(), egui::Button::new("Сохранить")).clicked() && self.save_project(false) {
                    self.close_allowed = true; self.close_confirm = false;
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
                if ui.button("Не сохранять").clicked() {
                    self.close_allowed = true; self.close_confirm = false;
                    ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                }
                if ui.button("Отмена").clicked() { self.close_confirm = false; }
            });
            ui.label(&self.status);
        });
    }
    pub(super) fn contour_list_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Добавленные объекты")
            .default_open(false)
            .show(ui, |ui| {
                self.soil_models_controls(ui);
                if self.building.enabled {
                    let floors = self.building.floors.clone();
                    for floor in floors {
                        let header = egui::CollapsingHeader::new(format!(
                            "{} · {:.0} мм",
                            floor.name, floor.elevation_mm
                        ))
                        .id_salt(("floor_contours", floor.id))
                        .default_open(false)
                        .show(ui, |ui| {
                            self.floor_list = Some(floor.id);
                            ui.push_id(floor.id, |ui| self.floor_contour_list(ui));
                        });
                        header.header_response.context_menu(|ui| {
                            if ui.button("Показать").clicked() {
                                self.set_floor_visibility_from_menu(floor.id, true);
                                ui.close_menu();
                            }
                            if ui.button("Скрыть").clicked() {
                                self.set_floor_visibility_from_menu(floor.id, false);
                                ui.close_menu();
                            }
                        });
                    }
                    self.floor_list = None;
                } else {
                    self.floor_contour_list(ui);
                }
            });
        egui::CollapsingHeader::new("Модификация объектов")
            .default_open(false)
            .show(ui, |ui| self.rounding_controls(ui));
    }
    fn floor_contour_list(&mut self, ui: &mut egui::Ui) {
        let mut selection = None;
        let regions: Vec<_> = self
            .contours
            .iter()
            .filter(|c| self.on_list_floor(c) && c.kind == Kind::MeshRegion)
            .map(|c| c.id)
            .collect();
        if !regions.is_empty() {
            let header = egui::CollapsingHeader::new("Области триангуляции")
                .default_open(false)
                .show(ui, |ui| self.region_list_controls(ui));
            contour_tree_menu(
                &header.header_response,
                regions.len(),
                regions.into_iter(),
                &mut selection,
            );
        }

        self.load_contour_tree(ui);
        let mut groups: [Vec<&Contour>; CONTOUR_TREE_GROUPS.len()] = Default::default();
        for c in self.contours.iter().filter(|c| self.on_list_floor(c)) {
            let kind = if punching::is_punching(c) {
                Kind::Punching
            } else {
                c.kind
            };
            if let Some(i) = CONTOUR_TREE_GROUPS
                .iter()
                .position(|(group, _)| *group == kind)
            {
                groups[i].push(c);
            }
        }
        for ((_, label), items) in CONTOUR_TREE_GROUPS.into_iter().zip(groups) {
            let count = items.len();
            if count == 0 {
                continue;
            }
            let header = egui::CollapsingHeader::new(format!("{label} ({count})"))
                .default_open(false)
                .show(ui, |ui| {
                    egui::ScrollArea::vertical()
                        .id_salt(("contours", label))
                        .max_height(180.)
                        .show(ui, |ui| {
                            for c in &items {
                                let parent = c
                                    .parent
                                    .filter(|_| !punching::is_punching(c))
                                    .map_or(String::new(), |p| format!(" · плита {p}"));
                                ui.push_id(c.id, |ui| {
                                    let leaf = ui.selectable_label(
                                        self.is_selected(c.id),
                                        format!("{}{parent}", c.display_name()),
                                    );
                                    if leaf.clicked() {
                                        selection = Some(vec![c.id]);
                                    }
                                    contour_tree_menu(
                                        &leaf,
                                        1,
                                        std::iter::once(c.id),
                                        &mut selection,
                                    );
                                });
                            }
                        });
                });
            contour_tree_menu(
                &header.header_response,
                count,
                items.iter().map(|c| c.id),
                &mut selection,
            );
        }
        self.underlay_list_controls(ui);
        if let Some(ids) = selection {
            self.select_contour_tree(ids);
        }
        if self.floor_list.is_none()
            || self
                .selected
                .is_some_and(|id| self.floor_list == Some(self.building.owner(id)))
        {
            self.contour_stiffness_controls(ui);
            if ui
                .add_enabled(
                    self.selected.is_some_and(|id| self.contour_removable(id)),
                    egui::Button::new("Удалить объект"),
                )
                .clicked()
            {
                self.remove();
            }
        }
    }

    fn load_contour_tree(&mut self, ui: &mut egui::Ui) {
        let mut cases = BTreeMap::<u64, [Vec<&Contour>; LOAD_CONTOUR_KINDS.len()]>::new();
        let mut total = 0;
        for c in &self.contours {
            let Some(load) = c.load.as_ref().filter(|_| self.on_list_floor(c)) else {
                continue;
            };
            let Some(kind) = LOAD_CONTOUR_KINDS.iter().position(|&kind| kind == c.kind) else {
                continue;
            };
            cases.entry(load.case_id).or_default()[kind].push(c);
            total += 1;
        }
        if total == 0 {
            return;
        }

        let mut selection = None;
        let root = egui::CollapsingHeader::new(format!("Нагрузки ({total})"))
            .id_salt("load_contours_root")
            .show(ui, |ui| {
                for case in &self.load_cases {
                    let groups = cases.get(&case.id);
                    let case_total = groups.map_or(0, |groups| groups.iter().map(Vec::len).sum());
                    let header =
                        egui::CollapsingHeader::new(format!("{} ({case_total})", case.name))
                            .id_salt(("load_contours", case.id))
                            .show(ui, |ui| {
                                for (kind, items) in
                                    LOAD_CONTOUR_KINDS.iter().zip(groups.into_iter().flatten())
                                {
                                    if items.is_empty() {
                                        continue;
                                    }
                                    let label = kind.label();
                                    let header = egui::CollapsingHeader::new(format!(
                                        "{label} ({})",
                                        items.len()
                                    ))
                                    .id_salt((case.id, label))
                                    .show(ui, |ui| {
                                        for &c in items {
                                            ui.push_id(c.id, |ui| {
                                                let name = c
                                                    .load
                                                    .as_ref()
                                                    .map_or("", |load| load.name.as_str());
                                                let title = if name.is_empty() {
                                                    c.id.to_string()
                                                } else {
                                                    format!("{name} {}", c.id)
                                                };
                                                let leaf = ui.selectable_label(
                                                    self.is_selected(c.id),
                                                    title,
                                                );
                                                if leaf.clicked() {
                                                    selection = Some(vec![c.id]);
                                                }
                                                contour_tree_menu(
                                                    &leaf,
                                                    1,
                                                    std::iter::once(c.id),
                                                    &mut selection,
                                                );
                                            });
                                        }
                                    });
                                    contour_tree_menu(
                                        &header.header_response,
                                        items.len(),
                                        items.iter().map(|c| c.id),
                                        &mut selection,
                                    );
                                }
                            });
                    let ids = groups.into_iter().flatten().flatten().map(|c| c.id);
                    contour_tree_menu(&header.header_response, case_total, ids, &mut selection);
                }
            });
        let ids = cases.values().flatten().flatten().map(|c| c.id);
        contour_tree_menu(&root.header_response, total, ids, &mut selection);
        if let Some(ids) = selection {
            self.select_load_tree(ids);
        }
    }

    pub(super) fn select_contour_tree(&mut self, ids: Vec<u64>) {
        let Some(&first) = ids.first() else {
            return;
        };
        if self.building.enabled {
            self.change_floor(self.building.owner(first));
        }
        self.cancel();
        let ids: BTreeSet<_> = ids.into_iter().collect();
        let selected: Vec<_> = self
            .contours
            .iter()
            .filter(|c| ids.contains(&c.id) && self.selectable(c))
            .map(|c| c.id)
            .collect();
        self.status = format!("Выделено объектов: {} из {}", selected.len(), ids.len());
        self.select_ids(selected, egui::Modifiers::NONE);
    }
}

fn default_point_size() -> f32 {
    10.
}
fn default_displacement_axes() -> u8 {
    0b111
}

#[cfg(windows)]
fn saved_time_label(path: &Path) -> String {
    #[repr(C)]
    struct Ft {
        low: u32,
        high: u32,
    }
    #[repr(C)]
    #[derive(Default)]
    struct St {
        year: u16,
        month: u16,
        weekday: u16,
        day: u16,
        hour: u16,
        minute: u16,
        second: u16,
        millis: u16,
    }
    #[link(name = "kernel32")]
    extern "system" {
        fn FileTimeToLocalFileTime(input: *const Ft, output: *mut Ft) -> i32;
        fn FileTimeToSystemTime(input: *const Ft, output: *mut St) -> i32;
    }
    let Some(time) = std::fs::metadata(path)
        .ok()
        .and_then(|m| m.modified().ok())
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
    else {
        return String::new();
    };
    let ticks = (time.as_secs() + 11644473600) * 10000000 + time.subsec_nanos() as u64 / 100;
    let input = Ft {
        low: ticks as u32,
        high: (ticks >> 32) as u32,
    };
    let mut local = Ft { low: 0, high: 0 };
    let mut out = St::default();
    if unsafe {
        FileTimeToLocalFileTime(&input, &mut local) != 0
            && FileTimeToSystemTime(&local, &mut out) != 0
    } {
        format!(
            " · {:02}.{:02}.{:04} {:02}:{:02}:{:02}",
            out.day, out.month, out.year, out.hour, out.minute, out.second
        )
    } else {
        String::new()
    }
}
#[cfg(not(windows))]
fn saved_time_label(_: &Path) -> String {
    String::new()
}

#[cfg(target_arch = "wasm32")]
impl App {
    pub(super) fn poll_browser_files(&mut self, ctx: &egui::Context) {
        for path in crate::plan_trace::platform::ready() {
            let name = path.to_string_lossy().into_owned();
            let bytes = crate::plan_trace::platform::read(&path).unwrap_or_default();
            let result = match path
                .extension()
                .and_then(|s| s.to_str())
                .unwrap_or("")
                .to_lowercase()
                .as_str()
            {
                "zip" => Project::read(&path).and_then(|p| self.apply_project(p, path.clone())),
                "lir" => {
                    self.load_lir_path(path.clone(), ctx);
                    Ok(())
                }
                "dxf" => import::from_bytes(&bytes, &name).map(|d| self.append_dxf(d)),
                "pdf" => underlay_ui::prepare_pdf(bytes, name).map(|p| self.accept_pdf_import(p)),
                "json" => {
                    if let Some(index) = self.length_scale_index() {
                        self.length_scales[index].open_browser_preset(&bytes)
                    } else {
                        self.result_scales[self.result_field].open_browser_preset(&bytes)
                    }
                }
                _ => Err("Неподдерживаемый файл".into()),
            };
            if let Err(error) = result {
                self.status = error;
            }
        }
    }
}

#[cfg(test)]
mod punching_save_regression {
    use super::*;
    #[test]
    fn punching_contour_archive_roundtrip_preserves_parent_and_geometry() {
        let mut app = App::default();
        let slab = Contour {
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
            kind: Kind::Slab,
            points: vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
            curves: vec![],
        };
        let mut cut = slab.clone();
        cut.id = 2;
        cut.kind = Kind::Punching;
        cut.parent = Some(1);
        cut.points = vec![
            [1000., 1000.],
            [2000., 1000.],
            [2000., 2000.],
            [1000., 2000.],
        ];
        cut.refinement = Some(model::Refinement {
            inactive_edges: vec![1, 3],

            name: "Продавливание: торец стены 3".into(),
            size: 150.,
        });
        app.contours = vec![slab, cut.clone()];
        let mut project = app.project_snapshot();
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos();
        let path = std::env::temp_dir().join(format!("mini-fem-punching-{stamp}.slab.zip"));
        project.write(&path).unwrap();
        let restored = Project::read(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(restored.contours.iter().find(|c| c.id == 2).unwrap(), &cut);
        // A manually drawn summation contour does not need a parent slab.
        cut.parent = None;
        project
            .contours
            .iter_mut()
            .find(|c| c.id == 2)
            .unwrap()
            .parent = None;
        project.write(&path).unwrap();
        let restored = Project::read(&path).unwrap();
        std::fs::remove_file(&path).unwrap();
        assert_eq!(restored.contours.iter().find(|c| c.id == 2).unwrap(), &cut);
        // Optional must not mean a dangling link is accepted.
        project
            .contours
            .iter_mut()
            .find(|c| c.id == 2)
            .unwrap()
            .parent = Some(999);
        assert!(project.write(&path).is_err());
    }
}

#[cfg(test)]
mod cut_project_diagnostic {
    use super::*;
    #[test]
    #[ignore = "local project diagnostic"]
    fn project_cut_wall_balance() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").unwrap());
        let mut app = App::default();
        app.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        let mesh = crate::plan_trace::meshing::generate(&app.contours, app.mesh_size).unwrap();
        let mesh = crate::plan_trace::quad_mesh::process(mesh, app.mesh_options).unwrap();

        let r = crate::plan_trace::fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_size,
            app.element_type,
            &app.support_height,
        )
        .unwrap();
        app.analysis = Some(r);
        if std::env::var_os("MINI_FEM_SUMMATION_DIRECT").is_some() {
            app.summation_method = crate::plan_trace::fem::cut_forces::Method::ElementEdges;
        }
        app.refresh_summation_results();
        app.sync_combination_result();
        let combined = app
            .selected_result()
            .expect("active diagnostic view")
            .clone();
        let cuts:Vec<_>=combined.punching.iter().map(|(id,r)|serde_json::json!({"id":id,"force":r.force,"closing":r.closing_force,"inner":r.inner_force,"net":r.full_net_force,"mx":r.mx,"my":r.my,"center":r.center,"error":r.diagnostic,"method":r.method,"fi":r.edge_forces.iter().filter(|e|e.active).map(|e|serde_json::json!({"edge":e.index+1,"force":e.force,"error":e.diagnostic})).collect::<Vec<_>>(),"source":app.contours.iter().find(|c|c.id==*id).map(|c|&c.refinement)})).collect();
        assert!(combined.balance_kn.abs() < 1e-6 * combined.applied_kn.abs().max(1.));
        for beam in app.contours.iter().filter(|c| c.kind == Kind::Beam) {
            let mut p = beam.mesh_boundary(app.mesh_size).unwrap();
            if beam.closed() {
                p.push(p[0]);
            }
            let length = p
                .windows(2)
                .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
                .sum::<f64>();
            let actual = combined
                .shell
                .as_ref()
                .unwrap()
                .beam_forces
                .iter()
                .filter(|b| b.id == beam.id)
                .map(|b| (b.points[1][0] - b.points[0][0]).hypot(b.points[1][1] - b.points[0][1]))
                .sum::<f64>();
            assert!(
                (actual - length).abs() < length.max(1.) * 1e-6,
                "beam {}: {actual} != {length}",
                beam.id
            );
        }
        for result in combined
            .punching
            .values()
            .filter(|r| r.diagnostic.is_empty())
        {
            assert!(
                (result
                    .edge_forces
                    .iter()
                    .filter(|e| e.active)
                    .map(|e| e.force)
                    .sum::<f64>()
                    - result.force)
                    .abs()
                    < 1e-8
            );
            assert!([result.force, result.mx, result.my]
                .iter()
                .all(|v| v.is_finite()));
        }
        if std::env::var_os("MINI_FEM_BALANCE_WALL51").is_some() {
            let f: [f64; 3] = [105, 106, 119].map(|id| combined.punching[&id].force);
            let load = -[105, 106, 119]
                .iter()
                .map(|id| combined.punching[id].full_net_force)
                .sum::<f64>();
            let reaction = combined.shell.as_ref().unwrap().support_reactions_kn[&51][2];
            assert!(
                (f.iter().sum::<f64>() + load - reaction).abs() < 1e-7,
                "F={f:?} P={load} R={reaction}"
            );
        }
        if std::env::var_os("MINI_FEM_REQUIRE_ALL_CUTS").is_some() {
            assert!(
                combined.punching.values().all(|r| r.diagnostic.is_empty()),
                "{:?}",
                combined.punching
            );
        }
        let report = serde_json::json!({"g":app.gravity,"cuts":cuts,"support_reactions":combined.shell.as_ref().map(|s|&s.support_reactions_kn),"contours":app.contours,"nodes":mesh.nodes.iter().map(|n|n.xy).collect::<Vec<_>>(),"reactions":combined.reaction_kn,"support_boundary":mesh.support_boundary});
        std::fs::write(
            "target/cut_wall_balance.json",
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
    }
}
#[cfg(test)]
mod parallel_benchmark {
    use super::*;
    #[test]
    #[ignore = "requires MINI_FEM_PROJECT_FIXTURE; controlled serial/parallel performance comparison"]
    fn local_parallel_cases_benchmark() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").unwrap());
        let mut app = App::default();
        app.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        app.sync_stiffness_sources();
        let mesh = crate::plan_trace::meshing::generate(&app.contours, app.mesh_size).unwrap();
        let mesh = crate::plan_trace::quad_mesh::process(mesh, app.mesh_options).unwrap();
        let mut serial = None;
        let mut timings = vec![];
        for workers in [1, 4] {
            let events = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
            let sink = events.clone();
            crate::plan_trace::fem::progress::install(move |s, t| {
                sink.lock().unwrap().push((s, t))
            });
            let start = web_time::Instant::now();
            let result = crate::plan_trace::fem::with_case_workers(workers, || {
                crate::plan_trace::fem::solve_with_support_height(
                    &mesh,
                    &app.contours,
                    &app.materials,
                    &app.load_cases,
                    app.gravity,
                    app.mesh_size,
                    app.element_type,
                    &app.support_height,
                )
            })
            .unwrap();
            let total = start.elapsed().as_secs_f64();
            crate::plan_trace::fem::progress::clear();
            let events = events.lock().unwrap();
            assert_eq!(
                events
                    .iter()
                    .filter(|e| e.0.contains("Разложение матрицы"))
                    .count(),
                1
            );
            let (mode, begin) = events
                .iter()
                .find(|e| e.0.starts_with("Решение загружений:"))
                .unwrap();
            let mut max_diff = 0_f64;
            if let Some(reference) = &serial {
                let reference: &crate::plan_trace::fem::Results = reference;
                for (a, b) in reference.cases.iter().zip(&result.cases) {
                    assert_eq!(a.id, b.id);
                    for (x, y) in a.w_mm.iter().zip(&b.w_mm) {
                        max_diff = max_diff.max((x - y).abs());
                    }
                    for (x, y) in a.reaction_kn.iter().zip(&b.reaction_kn) {
                        assert!((x - y).abs() < 1e-6);
                    }
                    assert!(b.balance_kn.abs() < 1e-6 * b.applied_kn.abs().max(1.));
                }
                assert!(max_diff < 1e-8);
            }
            timings.push(serde_json::json!({"requested_workers":workers,"mode":mode,"total_seconds":total,"cases_seconds":total-begin,"cases":result.cases.len(),"free_dofs":result.free_dofs,"max_w_difference":max_diff}));
            println!("{}", timings.last().unwrap());
            if workers == 1 {
                serial = Some(result);
            }
        }
        std::fs::write(
            "target/parallel_mig_benchmark.json",
            serde_json::to_vec_pretty(&timings).unwrap(),
        )
        .unwrap();
    }
}

#[cfg(test)]
mod building_project_tests {
    use super::*;
    #[test]
    fn floors_roundtrip_without_derived_data_and_navigation_is_not_a_source_edit() {
        let mut app = App::default();
        app.building.enabled = true;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1500.], false);
        app.sync_building_objects();
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Второй".into(),
            elevation_mm: 3000.,
            height_mm: 2500.,
        });
        let before = app.project_state_bytes();
        app.change_floor(2);
        assert_eq!(before, app.project_state_bytes());
        let encoded = serde_json::to_vec(&app.project_snapshot()).unwrap();
        let decoded: Project = serde_json::from_slice(&encoded).unwrap();
        let mut restored = App::default();
        restored
            .apply_project(decoded, PathBuf::from("building.slab.zip"))
            .unwrap();
        assert_eq!(restored.building, app.building);
        assert!(restored.fe_mesh.is_none());
        assert!(restored.analysis.is_none());
        let mut legacy: serde_json::Value = serde_json::from_slice(&encoded).unwrap();
        legacy.as_object_mut().unwrap().remove("building");
        let mut legacy: Project = serde_json::from_value(legacy).unwrap();
        assert!(!legacy.building.enabled);
        assert!(legacy.validate().is_ok());
    }
}

#[cfg(test)]
mod building_solve_fixture {
    use super::*;
    #[test]
    #[ignore = "requires MINI_FEM_LOAD_BOUNDARY_PROJECT; constrained triangular stamp fixture"]
    fn area_stamp_boundary_cells_do_not_receive_outside_pressure() {
        let path = PathBuf::from(
            std::env::var_os("MINI_FEM_LOAD_BOUNDARY_PROJECT").expect("load boundary project"),
        );
        let mut app = App::default();
        app.load_project_path(path).unwrap();
        app.sync_stiffness_sources();
        let mesh = crate::plan_trace::meshing::generate(&app.contours, app.mesh_size).unwrap();
        let q = crate::plan_trace::fem::source_area_projection(
            &mesh,
            &app.contours,
            &app.materials,
            &[(app.load_cases[0].clone(), 1.)],
            app.gravity,
            app.mesh_size,
        )
        .unwrap();
        let mut force = 0.;
        for (e, q) in mesh.elements.iter().zip(q) {
            let [a, b, c] = e.nodes.map(|n| mesh.nodes[n].xy);
            let area =
                ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5 / 1e6;
            force += area * q;
        }
        // 1.86m² at 1.5tf/m², g=10; the independent line load is not part of qa.
        assert!(
            (force - 27.9).abs() < 1e-7,
            "area resultant {force} kN instead of 27.9"
        );
    }
    #[test]
    #[ignore = "requires MINI_FEM_PLANE_PROJECT; mesh/material regression only"]
    fn stepped_plate_project_meshing_and_material_ownership() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_PLANE_PROJECT").expect("project path"));
        let mut app = App::default();
        app.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let mut layout = app.building_layout();
        let sources = crate::plan_trace::columns::meshing_sources(
            &app.contours,
            &app.support_height,
            &mut layout,
        );
        let graph = crate::plan_trace::intersections::build_sampled(
            &app.contours,
            layout.as_ref(),
            app.support_height.extent(),
            app.mesh_size,
        )
        .unwrap();
        let contacts = graph
            .lines
            .iter()
            .map(|l| l.ends)
            .chain(graph.points.iter().map(|(p, _)| [*p, *p]))
            .collect::<Vec<_>>();
        let mesh = crate::plan_trace::building_mesh::generate_with_intersections(
            &sources,
            app.mesh_size,
            layout.unwrap(),
            app.mesh_options,
            &contacts,
        )
        .unwrap();
        let geometry = crate::plan_trace::fem::prepare_cells(
            &mesh,
            &app.contours,
            &app.materials,
            app.mesh_size,
            app.gravity,
        )
        .unwrap();
        assert_eq!(geometry.len(), mesh.elements.len());
        let scene = crate::plan_trace::fem::shell::scene(
            &mesh,
            &geometry.iter().map(|c| c.material).collect::<Vec<_>>(),
            &app.support_height,
            app.mesh_size,
            &app.contours,
        )
        .unwrap();
        let used = scene
            .surfaces
            .iter()
            .flat_map(|s| s.nodes.iter().copied())
            .chain(scene.beams.iter().flat_map(|b| b.nodes))
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(
            used.len(),
            scene.xyz.len(),
            "unused physical nodes after wall triangulation: {:?}",
            scene
                .xyz
                .iter()
                .enumerate()
                .filter(|(n, _)| !used.contains(n))
                .take(20)
                .collect::<Vec<_>>()
        );
        println!(
            "PLANE_PROJECT nodes={} triangles={} quads={} shell_faces={}",
            mesh.nodes.len(),
            mesh.elements.len(),
            mesh.quads.len(),
            scene.surfaces.len()
        );
    }
    #[test]
    #[ignore = "local eight-storey project; explicit release run"]
    fn building_project_mesh_and_solve() {
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_BUILDING_PROJECT").expect("project path"));
        let mut app = App::default();
        app.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let step = std::env::var("MINI_FEM_BUILDING_STEP")
            .ok()
            .map(|s| s.parse::<f64>().unwrap())
            .unwrap_or(app.mesh_size);
        let start = web_time::Instant::now();
        let mesh = crate::plan_trace::building_mesh::generate(
            &app.contours,
            step,
            app.building_layout().unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let meshing_seconds = start.elapsed().as_secs_f64();
        println!(
            "BUILDING_MESH nodes={} triangles={} quads={} seconds={meshing_seconds}",
            mesh.nodes.len(),
            mesh.elements.len(),
            mesh.quads.len()
        );
        let mut cases = app.load_cases.clone();
        if let Ok(n) = std::env::var("MINI_FEM_BUILDING_CASES") {
            cases.truncate(n.parse().unwrap());
        }
        let events = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = events.clone();
        crate::plan_trace::fem::progress::install(move |s, t| {
            println!("BUILDING_STAGE {t:.3} {s}");
            sink.lock().unwrap().push((s, t));
        });
        let start = web_time::Instant::now();
        let result = crate::plan_trace::fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &cases,
            app.gravity,
            step,
            app.element_type,
            &app.support_height,
        )
        .unwrap();
        let solve_seconds = start.elapsed().as_secs_f64();
        crate::plan_trace::fem::progress::clear();
        for c in &result.cases {
            assert!(c.w_mm.iter().all(|v| v.is_finite()));
            assert!(c.balance_kn.abs() < 1e-6 * c.applied_kn.abs().max(1.));
            assert!(c.relative_residual < 1e-6);
        }
        assert_eq!(
            events
                .lock()
                .unwrap()
                .iter()
                .filter(|(s, _)| s == "Разложение матрицы жёсткости")
                .count(),
            1
        );
        let report = serde_json::json!({"levels":app.building.floors.len(),"step_mm":step,"mesh_nodes":mesh.nodes.len(),"display_triangles":mesh.elements.len(),"physical_quads":mesh.quads.len(),"free_dofs":result.free_dofs,"meshing_seconds":meshing_seconds,"solve_seconds":solve_seconds,"cases":result.cases.iter().map(|c|serde_json::json!({"id":c.id,"applied_kn":c.applied_kn,"balance_kn":c.balance_kn,"relative_residual":c.relative_residual,"max_w_mm":c.w_mm.iter().copied().fold(f64::NEG_INFINITY,f64::max)})).collect::<Vec<_>>()});
        std::fs::write(
            "reports/building_mig8_solver.json",
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
        println!("BUILDING_RESULT {report}");
    }
}

#[cfg(test)]
impl App {
    pub(super) fn write_lir_preview(&self, path: &Path) -> Result<(), String> {
        let mut project = self.project_snapshot();
        project.write(path)?;
        let check = Project::read(path)?;
        assert!(check.contours.iter().all(|c| !c.kind.is_load()));
        assert!(check.lir_source.is_some());
        Ok(())
    }
}

#[cfg(test)]
mod foundation_archive_regression {
    use super::*;
    #[test]
    fn source_archive_preserves_foundation_samples_and_plate_assignment() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([2000., 2000.], false);
        a.cancel();
        a.contours[0].use_foundation = true;
        a.begin(Kind::Foundation);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([-100., -100.], false);
        a.sketch_click([2100., 2100.], false);
        a.cancel();
        let id = a.contours.last().unwrap().id;
        a.foundation_point_target = Some(id);
        a.place_foundation_point([1000., 1000.]);
        let mut p = a.project_snapshot();
        p.validate().unwrap();
        if let Some(path) = std::env::var_os("MINI_FEM_FOUNDATION_DEMO") {
            let mut demo = a.contours[0].clone();
            demo.id = a.next_id;
            demo.kind = Kind::LoadArea;
            demo.use_foundation = false;
            demo.name = "Равномерная нагрузка 2 тс/м²".into();
            demo.load = Some(crate::plan_trace::loads::LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                case_id: 1,
                name: demo.name.clone(),
                magnitude: crate::plan_trace::loads::Magnitude {
                    value: 2.,
                    units: crate::plan_trace::loads::ForceUnits::TonForce,
                },
                closed: true,
                mesh_constraint: false,
            });
            let mut demo_project: Project =
                serde_json::from_value(serde_json::to_value(&p).unwrap()).unwrap();
            demo_project.contours.push(demo);
            demo_project.write(&PathBuf::from(path)).unwrap();
        }
        let bytes = serde_json::to_vec(&p).unwrap();
        let p: Project = serde_json::from_slice(&bytes).unwrap();
        let mut b = App::default();
        b.apply_project(p, PathBuf::from("foundation.slab.zip"))
            .unwrap();
        assert_eq!(a.contours, b.contours);
        assert!(b.contours[0].use_foundation);
        assert_eq!(
            b.contours
                .last()
                .unwrap()
                .foundation
                .as_ref()
                .unwrap()
                .interior
                .len(),
            1
        );
    }
}
#[cfg(test)]
mod visibility_real_project_bench {
    use super::*;
    #[test]
    #[ignore = "requires MINI_FEM_VISIBILITY_PROJECT; real source visibility timings"]
    fn hide_storeys_retains_source_scene_and_measures_visible_rebuild() {
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_VISIBILITY_PROJECT").expect("project path"));
        let mut a = App::default();
        a.apply_project(Project::read(&path).unwrap(), path)
            .unwrap();
        a.spatial.enabled = true;
        a.sync_stiffness_sources();
        let t = Instant::now();
        let scene =
            spatial_ui::building_source_scene(&a.contours, &a.support_height, &a.building).unwrap();
        a.spatial.source_index = a
            .contours
            .iter()
            .enumerate()
            .map(|(i, c)| (c.id, i))
            .collect();
        a.spatial.source_surface_owners = a.source_surface_owner_index(&scene);
        a.spatial.cache = Some((
            a.mesh_generation,
            a.support_height.clone(),
            a.contours.clone(),
            false,
            scene.clone(),
        ));
        println!(
            "VISIBILITY source_faces={} build_ms={:.2}",
            scene.surfaces.len(),
            t.elapsed().as_secs_f64() * 1000.
        );
        let _ = a.column_spiders();
        let mut previous = scene.surfaces.len();
        for floor in a
            .building
            .floors
            .iter()
            .take(8)
            .map(|f| f.id)
            .collect::<Vec<_>>()
        {
            let t = Instant::now();
            a.set_floor_visibility_from_menu(floor, false);
            let draw = a.spatial_draw_list(&scene);
            let _ = a.column_spiders();
            let _ = spatial_world::geometry(&a, &scene, [0.; 3], 100000., None, vec![floor]);
            assert!(draw.surfaces.len() <= previous);
            previous = draw.surfaces.len();
            assert!(a.spatial.cache.is_some());
            println!(
                "VISIBILITY hide_floor={floor} faces={} elapsed_ms={:.2}",
                previous,
                t.elapsed().as_secs_f64() * 1000.
            );
        }
    }
}

/// Read-only diagnostic: rebuild a single physical plate level with every
/// vertical source intersecting that level, without opening a graphics window.
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn audit_floor_mesh(
    path: PathBuf,
    floor: u64,
    output: PathBuf,
    partial: bool,
) -> Result<(), String> {
    use crate::plan_trace::{fem::progress, mesh_backend};
    progress::install(|stage, elapsed| eprintln!("{elapsed:.3}\t{stage}"));
    progress::stage("Открытие проекта");
    let mut app = App::default();
    app.apply_project(Project::read(&path)?, path)?;
    app.sync_building_objects();
    app.sync_stiffness_sources();
    let mut expanded = app.building_layout();
    let all = crate::plan_trace::columns::meshing_sources(
        &app.contours,
        &app.support_height,
        &mut expanded,
    );
    let layout = expanded.ok_or("Building layout required")?;
    let z = layout
        .levels
        .iter()
        .find(|l| l.id == floor)
        .ok_or("Этаж не найден")?
        .z;
    let sources = all
        .iter()
        .filter(|c| {
            if partial {
                return layout.owner(c.id) == floor;
            }
            if matches!(c.kind, Kind::Support | Kind::Column | Kind::Beam) {
                let e = layout.extent(c);
                z >= e[0] - 1e-5 && z <= e[1] + 1e-5
            } else {
                layout.owner(c.id) == floor
            }
        })
        .cloned()
        .collect::<Vec<_>>();
    progress::stage("Построение пересечений");
    let graph = crate::plan_trace::intersections::build_sampled(
        if partial { &sources } else { &all },
        Some(&layout),
        app.support_height.extent(),
        app.mesh_size,
    )?;
    let contacts = graph
        .lines
        .iter()
        .map(|l| l.ends)
        .chain(graph.points.iter().map(|(p, _)| [*p, *p]))
        .collect::<Vec<_>>();
    let options = app.mesh_options;
    progress::stage("Построение сетки");
    let mesh = mesh_backend::configured(
        mesh_backend::saved_path(),
        options.merge,
        options.optimise,
        || {
            crate::plan_trace::building_mesh::generate_with_intersections(
                &sources,
                app.mesh_size,
                layout,
                options,
                &contacts,
            )
        },
    )?;
    progress::stage("Подготовка расчётных элементов");
    let cells = crate::plan_trace::fem::prepare_cells(
        &mesh,
        &sources,
        &app.materials,
        app.mesh_size,
        app.gravity,
    )?;
    progress::stage("Сборка расчётной схемы");
    let scene = crate::plan_trace::fem::shell::scene(
        &mesh,
        &cells.iter().map(|c| c.material).collect::<Vec<_>>(),
        &app.support_height,
        app.mesh_size,
        &sources,
    )?;
    progress::stage("Запись результата аудита");
    let walls = serde_json::json!({"xyz":scene.xyz,"surfaces":scene.surfaces.iter().filter(|s|s.support.is_some()).map(|s|serde_json::json!({"id":s.support,"nodes":s.nodes})).collect::<Vec<_>>()});
    std::fs::write(
        output.with_extension("walls.json"),
        serde_json::to_vec(&walls).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    std::fs::write(
        output,
        serde_json::to_vec(&mesh).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}
