#[path = "alternative_stiffness.rs"]
mod alternative_stiffness;
#[path = "building.rs"]
mod building;
#[path = "column_render.rs"]
mod column_render;
#[path = "context_menu.rs"]
mod context_menu;
#[path = "diagnostics.rs"]
mod diagnostics;
#[path = "foundation_ui.rs"]
mod foundation_ui;
#[path = "intersections_ui.rs"]
mod intersections_ui;
#[path = "lir_import.rs"]
mod lir_import;
#[path = "object_properties.rs"]
mod object_properties;
#[path = "object_visibility.rs"]
mod object_visibility;
#[path = "plaxis_export.rs"]
mod plaxis_export;
#[path = "punching.rs"]
mod punching;
#[path = "rigid_link_ui.rs"]
mod rigid_link_ui;
#[path = "schema_analysis.rs"]
mod schema_analysis;
#[path = "soil_decode.rs"]
mod soil_decode;
#[cfg(not(target_arch = "wasm32"))]
#[path = "soil_prototype_host.rs"]
mod soil_prototype_host;
#[path = "soil_settings.rs"]
mod soil_settings;
#[path = "source_views.rs"]
mod source_views;
#[path = "spatial_loads.rs"]
mod spatial_loads;
#[path = "support_extents.rs"]
mod support_extents;
#[path = "vertical_conversion.rs"]
mod vertical_conversion;
#[path = "vertical_merge.rs"]
mod vertical_merge;
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
use web_time::Instant;
#[path = "beam_results.rs"]
mod beam_results;
#[path = "clipboard.rs"]
mod clipboard;
#[path = "construction_snap.rs"]
mod construction_snap;
#[cfg(test)]
#[path = "contour_drag_tests.rs"]
mod contour_drag_tests;
#[path = "contour_filter.rs"]
mod contour_filter;
#[path = "contour_merge.rs"]
mod contour_merge;
#[path = "contour_reuse.rs"]
mod contour_reuse;
#[path = "draw_order.rs"]
mod draw_order;
#[path = "drawing_flags.rs"]
pub(super) mod drawing_flags;
#[path = "dxf_contours.rs"]
mod dxf_contours;
#[path = "dynamic_input.rs"]
mod dynamic_input;
#[path = "hover_pick.rs"]
mod hover_pick;
#[path = "lira_export_ui.rs"]
mod lira_export_ui;
#[path = "load_menu.rs"]
mod load_menu;
#[path = "load_mosaics.rs"]
mod load_mosaics;
#[path = "load_value_edit.rs"]
mod load_value_edit;
#[path = "measurements.rs"]
mod measurements;
#[path = "reinforcement_ui.rs"]
mod reinforcement_ui;
#[path = "result_gpu.rs"]
mod result_gpu;
#[path = "rounding.rs"]
mod rounding;
#[path = "scale_context.rs"]
mod scale_context;
#[path = "scaling.rs"]
mod scaling;
#[path = "sections.rs"]
mod sections;
#[path = "selection.rs"]
mod selection;
#[path = "slab_cuts.rs"]
mod slab_cuts;
#[path = "solver_job.rs"]
mod solver_job;
#[path = "spatial_construction.rs"]
mod spatial_construction;
#[path = "spatial_depth.rs"]
mod spatial_depth;
#[path = "spatial_edit.rs"]
mod spatial_edit;
#[path = "spatial_points.rs"]
mod spatial_points;
#[path = "spatial_results.rs"]
mod spatial_results;
#[path = "spatial_summation.rs"]
mod spatial_summation;
#[path = "spatial_ui.rs"]
mod spatial_ui;
#[path = "spatial_underlays.rs"]
mod spatial_underlays;
#[cfg(target_arch = "wasm32")]
pub(crate) use solver_job::worker_run as solver_worker_run;
#[cfg(target_arch = "wasm32")]
pub(crate) use solver_job::worker_run_bytes as solver_worker_run_bytes;
#[path = "spatial_world.rs"]
mod spatial_world;
#[path = "stiffness_ui.rs"]
mod stiffness_ui;
#[path = "support_distance.rs"]
mod support_distance;
#[path = "support_reactions.rs"]
mod support_reactions;
#[path = "triangulation_grid.rs"]
mod triangulation_grid;
#[path = "underlay_drag.rs"]
mod underlay_drag;
#[path = "underlay_ui.rs"]
mod underlay_ui;
use super::{
    geometry::{self, Point},
    import::{self, Drawing},
    model::{self, Contour, Curve, Kind},
    sketch::{self, Tool},
    snap::{self, Index},
};
use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2};
use std::{collections::BTreeMap, path::PathBuf, sync::mpsc, time::Duration};
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
struct Camera {
    center: Point,
    zoom: f64,
    angle: f64,
    #[serde(skip)]
    plane: Option<[Point; 3]>,
}
impl Default for Camera {
    fn default() -> Self {
        Self {
            center: [0., 0.],
            zoom: 1.,
            angle: 0.,
            plane: None,
        }
    }
}
impl Camera {
    fn screen64(self, p: Point, r: Rect) -> Point {
        if let Some([o, x, y]) = self.plane {
            return [
                o[0] + x[0] * p[0] + y[0] * p[1],
                o[1] + x[1] * p[0] + y[1] * p[1],
            ];
        }
        let (s, c) = self.angle.sin_cos();
        let x = p[0] - self.center[0];
        let y = p[1] - self.center[1];
        [
            r.center().x as f64 + (c * x - s * y) * self.zoom,
            r.center().y as f64 - (s * x + c * y) * self.zoom,
        ]
    }
    fn screen(self, p: Point, r: Rect) -> Pos2 {
        let q = self.screen64(p, r);
        egui::pos2(q[0] as f32, q[1] as f32)
    }
    fn world(self, p: Pos2, r: Rect) -> Point {
        if let Some([o, x, y]) = self.plane {
            let d = x[0] * y[1] - x[1] * y[0];
            let q = [p.x as f64 - o[0], p.y as f64 - o[1]];
            return [
                (q[0] * y[1] - q[1] * y[0]) / d,
                (x[0] * q[1] - x[1] * q[0]) / d,
            ];
        }
        let (s, c) = self.angle.sin_cos();
        let x = (p.x - r.center().x) as f64 / self.zoom;
        let y = (r.center().y - p.y) as f64 / self.zoom;
        [
            self.center[0] + c * x + s * y,
            self.center[1] - s * x + c * y,
        ]
    }
}
fn clip_line(a: Point, b: Point, r: Rect) -> Option<[Pos2; 2]> {
    if a.iter().chain(&b).any(|v| !v.is_finite()) {
        return None;
    }
    let dx = b[0] - a[0];
    let dy = b[1] - a[1];
    let mut lo = 0_f64;
    let mut hi = 1_f64;
    for (p, q) in [
        (-dx, a[0] - r.min.x as f64),
        (dx, r.max.x as f64 - a[0]),
        (-dy, a[1] - r.min.y as f64),
        (dy, r.max.y as f64 - a[1]),
    ] {
        if p.abs() < 1e-20 {
            if q < 0. {
                return None;
            }
        } else {
            let t = q / p;
            if p < 0. {
                lo = lo.max(t);
            } else {
                hi = hi.min(t);
            }
            if lo > hi {
                return None;
            }
        }
    }
    Some([
        egui::pos2((a[0] + lo * dx) as f32, (a[1] + lo * dy) as f32),
        egui::pos2((a[0] + hi * dx) as f32, (a[1] + hi * dy) as f32),
    ])
}
fn draw_path(
    painter: &egui::Painter,
    points: &[Point],
    closed: bool,
    camera: Camera,
    rect: Rect,
    stroke: Stroke,
) {
    let n = if closed {
        points.len()
    } else {
        points.len().saturating_sub(1)
    };
    for i in 0..n {
        if let Some(p) = clip_line(
            camera.screen64(points[i], rect),
            camera.screen64(points[(i + 1) % points.len()], rect),
            rect.expand(stroke.width + 1.),
        ) {
            painter.line_segment(p, stroke);
        }
    }
}
fn append_triangle(
    mesh: &mut egui::Mesh,
    t: [Point; 3],
    camera: Camera,
    rect: Rect,
    color: Color32,
) {
    let mut poly = t.map(|p| camera.screen64(p, rect)).to_vec();
    for (axis, limit, low) in [
        (0, rect.min.x as f64, true),
        (0, rect.max.x as f64, false),
        (1, rect.min.y as f64, true),
        (1, rect.max.y as f64, false),
    ] {
        let mut next = Vec::new();
        for i in 0..poly.len() {
            let a = poly[i];
            let b = poly[(i + 1) % poly.len()];
            let ia = if low {
                a[axis] >= limit
            } else {
                a[axis] <= limit
            };
            let ib = if low {
                b[axis] >= limit
            } else {
                b[axis] <= limit
            };
            if ia {
                next.push(a);
            }
            if ia != ib {
                let k = (limit - a[axis]) / (b[axis] - a[axis]);
                let mut p = [a[0] + k * (b[0] - a[0]), a[1] + k * (b[1] - a[1])];
                p[axis] = limit;
                next.push(p);
            }
        }
        poly = next;
    }
    if poly.len() < 3 {
        return;
    }
    let base = mesh.vertices.len() as u32;
    for p in &poly {
        mesh.vertices.push(egui::epaint::Vertex {
            pos: egui::pos2(p[0] as f32, p[1] as f32),
            uv: egui::epaint::WHITE_UV,
            color,
        });
    }
    for i in 1..poly.len() - 1 {
        mesh.indices
            .extend([base, base + i as u32, base + i as u32 + 1]);
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct LayerStyle {
    visible: bool,
    color: Option<[u8; 3]>,
    original: [u8; 3],
    width: f32,
}
impl LayerStyle {
    fn ink(&self, rgb: [u8; 3]) -> Color32 {
        let c = self.color.unwrap_or(rgb);
        Color32::from_rgb(c[0], c[1], c[2])
    }
}
struct VertexDrag {
    id: u64,
    vertex: Option<usize>,
    edge: Option<usize>,
    anchor: Point,
    original: Contour,
    before: Vec<Contour>,
}
struct ArcPlacement {
    original: Contour,
    edge: usize,
    a: Point,
    b: Point,
    preview: Option<Contour>,
    radius: Option<f64>,
}
type MeshResult = (
    Vec<Contour>,
    f64,
    Result<(super::meshing::Mesh, Option<source_views::FoundationView>), String>,
);
#[derive(Default)]
struct MeshFeedback {
    cancel: crate::plan_trace::fem::progress::CancelToken,
    stage: std::sync::Arc<std::sync::Mutex<String>>,
}
impl Drop for MeshFeedback {
    fn drop(&mut self) {
        self.cancel
            .store(true, std::sync::atomic::Ordering::Relaxed);
    }
}
#[derive(Clone)]
struct SketchState {
    line_closed: bool,
    segments: Vec<Curve>,
    tool: Tool,
    draft: Vec<Point>,
    controls: Vec<Point>,
}
#[path = "combination_ui.rs"]
mod combination_ui;
#[path = "fem_ui.rs"]
mod fem_ui;
#[path = "performance.rs"]
mod performance;
#[path = "project_ui.rs"]
mod project_ui;
struct App {
    view_visibility: object_visibility::State,
    building_sync: building::ObjectSync,
    stiffness_sources: stiffness_ui::SourceSync,
    building: building::Building,
    floor_list: Option<u64>,
    solver_cancel: Option<crate::plan_trace::fem::progress::CancelToken>,
    solver_progress: std::sync::Arc<std::sync::Mutex<Vec<(String, f64)>>>,
    #[cfg(target_arch = "wasm32")]
    solver_worker: Option<solver_job::Handle>,
    hover_pick: hover_pick::Hover,
    dxf_menu_target: Option<u64>,
    dxf_contours: Option<dxf_contours::Dialog>,
    stiffnesses: super::stiffness::Library,
    support_height: super::fem::shell::Settings,
    spatial: spatial_ui::View,
    project_path: Option<PathBuf>,
    #[cfg(target_arch = "wasm32")]
    opfs: project_ui::opfs::State,
    saved_project_state: Option<Vec<u8>>,
    close_confirm: bool,
    close_allowed: bool,
    lira_export_job: Option<lira_export_ui::Job>,
    perf: performance::Perf,
    schema_analysis: schema_analysis::State,
    mesh_generation: u64,
    result_generation: u64,
    materials: BTreeMap<u64, super::fem::element::Material>,
    reinforcement: reinforcement_ui::State,
    reinforcement_method: super::reinforcement::Method,
    reinforcement_directions: bool,
    reinforcement_area_view: super::reinforcement::AreaView,
    analysis: Option<super::fem::Results>,
    stiffness_variant: usize,
    alternative_cache: alternative_stiffness::Cache,
    alternative_job: Option<mpsc::Receiver<Result<super::fem::Results, String>>>,
    analysis_job: Option<mpsc::Receiver<Result<super::fem::Results, String>>>,
    analysis_input: Option<fem_ui::Input>,
    nonlinear_options: fem_ui::NonlinearOptions,
    buckling_options: fem_ui::BucklingOptions,
    soil_options: soil_settings::Options,
    soil_view: soil_settings::View,
    foundation_round_step: f64,
    foundation_value_round_step: f64,
    calculate_frequencies: bool,
    modal_mass_options: fem_ui::ModalMassOptions,
    modal_mass_source: Option<String>,
    calculate_effective_lengths: bool,
    length_view: u8,
    length_scales: [super::result_scale::Scale; 2],
    length_axis_y: bool,
    element_type: super::fem::ElementType,
    result_unit_factor: f64,
    iso_efforts: bool,
    membrane_area: bool,
    iso_displacements: bool,
    point_size: f32,
    result_field: usize,
    displacement_axes: u8,
    show_results: bool,
    show_result_values: bool,
    display_text: drawing_flags::TextStyle,
    punching_view: Option<usize>,
    summation_method: super::fem::cut_forces::Method,
    punching_scales: [super::result_scale::Scale; 4],
    load_scale_key: Option<[u64; 8]>,
    sum_reactions: bool,
    result_scales: [super::result_scale::Scale; super::fem::FIELD_COUNT],
    shape_scale: super::result_scale::Scale,
    buckling_scale: super::result_scale::Scale,
    shape_iso: bool,
    buckling_iso: bool,
    variant_scale_pending: [bool; super::fem::FIELD_COUNT],
    scale_store: scale_context::Store,
    combinations: Vec<super::load_combinations::Combination>,
    next_combination: u64,
    input_locked: bool,
    locks_before_auto: Option<([bool; 3], [bool; 9])>,
    dynamic_length: String,
    dynamic_aim: Option<Point>,
    dynamic_rectangle: [String; 2],
    dynamic_rectangle_field: usize,
    dynamic_rectangle_replace: bool,
    dynamic_rect: Option<Rect>,
    result_by_combinations: bool,
    active_combination: u64,
    combination_variant: super::load_combinations::Variant,
    combination_cache: Option<combination_ui::Cache>,
    combination_error: String,
    load_units: super::loads::ForceUnits,
    gravity: f64,
    new_load: super::loads::Magnitude,
    new_load_direction: crate::plan_trace::loads::Direction,
    new_load_moment: bool,
    draft_load_case: Option<u64>,
    draft_line_closed: bool,
    left_open: bool,
    right_open: bool,
    contour_locks: [bool; 3],
    loads_locked: bool,
    column_spider_cache: std::cell::RefCell<Option<(Vec<u8>, Vec<(Point, Vec<Point>, f64)>)>>,
    column_joint_cache: std::cell::RefCell<Option<(Vec<u8>, Vec<super::columns::Joint>)>>,
    mesh_column_signature: column_render::MeshSections,
    columns_locked: bool,
    columns_selectable: bool,
    foundation_selectable: bool,
    foundation_locked: bool,
    beams_locked: bool,
    triangulation_locked: bool,
    restraints_locked: bool,
    rigid_links_locked: bool,
    triangulation_grid: triangulation_grid::Grid,
    grid_editor: Option<triangulation_grid::Editor>,
    underlays_locked: bool,
    draw_order: draw_order::Order,
    support_distance_cache: std::cell::RefCell<Option<std::sync::Arc<support_distance::Data>>>,
    show_loads: bool,
    show_load_values: bool,
    load_cases: Vec<super::loads::LoadCase>,
    active_load: u64,
    next_load: u64,
    arc_placement: Option<ArcPlacement>,
    intersections: intersections_ui::State,
    foundation_point_target: Option<u64>,
    foundation_interaction: foundation_ui::Interaction,
    arc_menu_target: Option<(u64, usize)>,
    construction_grid: super::grid::Grid,
    grid_pick_origin: bool,
    section_pick_origin: Option<usize>,
    contours_locked: bool,
    draft_segments: Vec<Curve>,
    tool: Tool,
    curve_controls: Vec<Point>,
    arc_end_radius: bool,
    arc_radius: f64,
    sketch_history: Vec<SketchState>,
    toolbar_rect: Option<Rect>,
    fe_mesh: Option<super::meshing::Mesh>,
    mesh_input: Vec<Contour>,
    mesh_partial: bool,
    mesh_size: f64,
    mesh_options: super::quad_mesh::Options,
    mesh_requested_options: super::quad_mesh::Options,
    mesh_built_size: f64,
    mesh_job: Option<mpsc::Receiver<MeshResult>>,
    mesh_feedback: Option<MeshFeedback>,
    show_mesh: bool,
    show_support_nodes: bool,
    underlays: Vec<crate::plan_trace::underlay::Underlay>,
    selected_underlay: Option<u64>,
    underlay_selection: std::collections::BTreeSet<u64>,
    underlay_filter: [bool; 2],
    source_view: Option<source_views::Cached>,
    source_pressure: Option<source_views::PressureCache>,
    foundation_view: Option<(u64, source_views::FoundationView)>,
    lir_source: Option<lir_import::Source>,
    lir_dialog: Option<lir_import::Dialog>,
    transform_snap: Option<snap::Snap>,
    scaling_op: Option<scaling::Operation>,
    scaling_history: Vec<scaling::Undo>,
    underlay_drag: Option<underlay_drag::Drag>,
    underlay_history: Vec<underlay_drag::Change>,
    pdf_load: Option<mpsc::Receiver<Result<Option<underlay_ui::PdfImport>, String>>>,
    pdf_choice: Option<underlay_ui::PdfChoice>,
    pdf_textures: BTreeMap<u64, egui::TextureHandle>,
    drawing: Option<Drawing>,
    index: Index,
    fe_snap_index: Index,
    snap_settings: construction_snap::Settings,
    temporary_polar_anchor: Option<Point>,
    dwell_polar_anchor: Option<Point>,
    dwell_polar_candidate: Option<(Point, f64)>,
    layers: BTreeMap<String, LayerStyle>,
    contours: Vec<Contour>,
    next_id: u64,
    next_region: u64,
    selected: Option<u64>,
    clipboard_op: Option<clipboard::Operation>,
    measurements: measurements::Tool,
    rounding_step: f64,
    rounding_options: rounding::Options,
    gmsh_exe: String,
    sections: sections::State,
    slab_cuts: slab_cuts::State,
    simplify_tolerance: f64,
    rectify_angle_deg: f64,
    selection_ids: std::collections::BTreeSet<u64>,
    tree_load_selection: std::collections::BTreeSet<u64>,
    selection_filter: [bool; 14],
    contour_filter: contour_filter::State,
    fe_selection: selection::FeSelection,
    selection_all_floors: bool,
    selection_start: Option<Pos2>,
    selection_modifiers: egui::Modifiers,
    mode: Option<Kind>,
    draft: Vec<Point>,
    reuse_mode: Option<contour_reuse::Mode>,
    repeat_sketch: bool,
    draft_parent: Option<u64>,
    region_parent_choice: bool,
    camera: Camera,
    fit: bool,
    status: String,
    slab_color: [u8; 3],
    contour_colors: [[u8; 3]; 3],
    drawing_flags_open: bool,
    properties: object_properties::State,
    drawing_visibility: drawing_flags::Visibility,
    slab_opacity: f32,
    slab_cache: Option<Vec<(u64, [Point; 3])>>,
    show_dxf: bool,
    show_text: bool,
    contrast_text: bool,
    drag: Option<VertexDrag>,
    load_menu_target: Option<u64>,
    support_menu_target: Option<u64>,
    wall_restraint_dialog: Option<context_menu::WallRestraints>,
    load_value_edit: Option<load_value_edit::Edit>,
    menu_target: Option<(u64, usize, bool)>,
    history: Vec<Vec<Contour>>,
    diagnostic_points: Vec<[f64; 3]>,
    diagnostics: diagnostics::State,
    load: Option<mpsc::Receiver<Result<Option<Drawing>, String>>>,
    smoke: bool,
    smoke_source_preview: bool,
    smoke_section_toggle: bool,
    smoke_show_tools: bool,
    smoke_load_view: Option<usize>,
    frames: usize,
    started: Instant,
    capture_requested: bool,
}
impl Default for App {
    fn default() -> Self {
        Self {
            building_sync: Default::default(),
            stiffness_sources: Default::default(),
            solver_cancel: None,
            solver_progress: Default::default(),
            #[cfg(target_arch = "wasm32")]
            solver_worker: None,
            hover_pick: Default::default(),
            dxf_menu_target: None,
            dxf_contours: None,
            stiffnesses: Default::default(),
            view_visibility: Default::default(),
            building: Default::default(),
            floor_list: None,
            support_height: super::fem::shell::Settings {
                automatic_building_bottom: false,
                automatic_building_top: false,
                ..Default::default()
            },
            spatial: Default::default(),
            project_path: None,
            #[cfg(target_arch = "wasm32")]
            opfs: Default::default(),
            saved_project_state: None,
            close_confirm: false,
            close_allowed: false,
            lira_export_job: None,
            perf: performance::Perf::default(),
            schema_analysis: Default::default(),
            mesh_generation: 0,
            result_generation: 0,
            materials: BTreeMap::new(),
            reinforcement: Default::default(),
            reinforcement_method: Default::default(),
            reinforcement_directions: false,
            reinforcement_area_view: Default::default(),
            analysis: None,
            stiffness_variant: 0,
            alternative_cache: Default::default(),
            alternative_job: None,
            analysis_job: None,
            analysis_input: None,
            combinations: vec![super::load_combinations::Combination::new(1, [1])],
            next_combination: 2,
            input_locked: false,
            locks_before_auto: None,
            dynamic_length: String::new(),
            dynamic_aim: None,
            dynamic_rectangle: std::array::from_fn(|_| String::new()),
            dynamic_rectangle_field: 0,
            dynamic_rectangle_replace: false,
            dynamic_rect: None,
            result_by_combinations: false,
            active_combination: 1,
            combination_variant: super::load_combinations::Variant::Design,
            combination_cache: None,
            combination_error: String::new(),
            nonlinear_options: Default::default(),
            buckling_options: Default::default(),
            soil_options: Default::default(),
            soil_view: Default::default(),
            foundation_round_step: 10.,
            foundation_value_round_step: 10.,
            calculate_frequencies: false,
            modal_mass_options: Default::default(),
            modal_mass_source: None,
            calculate_effective_lengths: false,
            length_view: 0,
            length_scales: std::array::from_fn(|_| super::result_scale::Scale::default()),
            length_axis_y: true,
            element_type: super::fem::ElementType::Dkt,
            result_unit_factor: 0.1,
            iso_efforts: false,
            membrane_area: false,
            iso_displacements: true,
            point_size: 10.,
            result_field: 0,
            displacement_axes: 0b111,
            show_results: true,
            show_result_values: false,
            display_text: drawing_flags::TextStyle::default(),
            punching_view: None,
            summation_method: Default::default(),
            punching_scales: std::array::from_fn(|_| super::result_scale::Scale::default()),
            load_scale_key: None,
            sum_reactions: false,
            result_scales: std::array::from_fn(|_| super::result_scale::Scale::default()),
            shape_scale: Default::default(),
            buckling_scale: Default::default(),
            shape_iso: true,
            buckling_iso: false,
            variant_scale_pending: [false; super::fem::FIELD_COUNT],
            scale_store: Default::default(),
            load_units: super::loads::ForceUnits::TonForce,
            gravity: 10.,
            new_load_direction: Default::default(),
            new_load_moment: false,
            new_load: super::loads::Magnitude {
                value: 1.,
                units: super::loads::ForceUnits::TonForce,
            },
            draft_load_case: None,
            draft_line_closed: false,
            left_open: true,
            right_open: true,
            contour_locks: [false; 3],
            loads_locked: false,
            column_spider_cache: Default::default(),
            column_joint_cache: Default::default(),
            mesh_column_signature: Default::default(),
            columns_locked: false,
            columns_selectable: true,
            foundation_selectable: true,
            foundation_locked: false,
            beams_locked: false,
            triangulation_locked: false,
            restraints_locked: false,
            rigid_links_locked: false,
            triangulation_grid: triangulation_grid::Grid::default(),
            grid_editor: None,
            underlays_locked: false,
            draw_order: draw_order::Order::default(),
            support_distance_cache: std::cell::RefCell::new(None),
            show_loads: true,
            show_load_values: true,
            load_cases: vec![super::loads::LoadCase::new(1)],
            active_load: 1,
            next_load: 2,
            arc_placement: None,
            intersections: Default::default(),
            foundation_point_target: None,
            foundation_interaction: Default::default(),
            arc_menu_target: None,
            construction_grid: super::grid::Grid::default(),
            grid_pick_origin: false,
            section_pick_origin: None,
            contours_locked: false,
            draft_segments: vec![],
            tool: Tool::Line,
            curve_controls: vec![],
            arc_end_radius: false,
            arc_radius: 1000.,
            sketch_history: vec![],
            toolbar_rect: None,
            fe_mesh: None,
            mesh_input: vec![],
            mesh_partial: false,
            mesh_size: 500.,
            mesh_options: Default::default(),
            mesh_requested_options: Default::default(),
            mesh_built_size: 500.,
            mesh_job: None,
            mesh_feedback: None,
            show_mesh: true,
            show_support_nodes: true,
            underlays: vec![],
            selected_underlay: None,
            underlay_selection: Default::default(),
            underlay_filter: [true; 2],
            source_view: None,
            source_pressure: None,
            foundation_view: None,
            lir_source: None,
            lir_dialog: None,
            transform_snap: None,
            scaling_op: None,
            scaling_history: vec![],
            underlay_drag: None,
            underlay_history: vec![],
            pdf_load: None,
            pdf_choice: None,
            pdf_textures: BTreeMap::new(),
            drawing: None,
            index: Index::default(),
            fe_snap_index: Index::default(),
            snap_settings: construction_snap::Settings::default(),
            temporary_polar_anchor: None,
            dwell_polar_anchor: None,
            dwell_polar_candidate: None,
            layers: BTreeMap::new(),
            contours: vec![],
            next_id: 1,
            next_region: 1,
            selected: None,
            clipboard_op: None,
            measurements: measurements::Tool::default(),
            rounding_step: rounding::default_step(),
            rounding_options: Default::default(),
            gmsh_exe: super::mesh_backend::saved_path(),
            sections: Default::default(),
            slab_cuts: Default::default(),
            simplify_tolerance: rounding::default_tolerance(),
            rectify_angle_deg: 1.,
            selection_ids: Default::default(),
            tree_load_selection: Default::default(),
            selection_filter: selection::all_enabled(),
            contour_filter: Default::default(),
            fe_selection: Default::default(),
            selection_all_floors: false,
            selection_start: None,
            selection_modifiers: egui::Modifiers::NONE,
            mode: None,
            draft: vec![],
            reuse_mode: None,
            repeat_sketch: false,
            draft_parent: None,
            region_parent_choice: false,
            camera: Default::default(),
            fit: true,
            status: "Импортируйте DXF для обводки. Все высоты проецируются в Z=0.".into(),
            slab_color: [155; 3],
            contour_colors: drawing_flags::default_contour_colors(),
            drawing_flags_open: false,
            properties: Default::default(),
            drawing_visibility: drawing_flags::Visibility::default(),
            slab_opacity: 0.6,
            slab_cache: None,
            show_dxf: true,
            show_text: true,
            contrast_text: true,
            drag: None,
            load_menu_target: None,
            support_menu_target: None,
            wall_restraint_dialog: None,
            load_value_edit: None,
            menu_target: None,
            history: vec![],
            diagnostic_points: vec![],
            diagnostics: Default::default(),
            load: None,
            smoke: false,
            smoke_source_preview: false,
            smoke_section_toggle: false,
            smoke_show_tools: false,
            smoke_load_view: None,
            frames: 0,
            started: Instant::now(),
            capture_requested: false,
        }
    }
}
impl App {
    fn new_project() -> Self {
        let mut app = Self::default();
        app.building.enabled = true;
        app
    }
    fn request_delete_load_case(&mut self, id: u64) {
        let Some(case) = self.load_cases.iter().find(|c| c.id == id) else {
            return;
        };
        let count = self
            .contours
            .iter()
            .filter(|c| c.load.as_ref().is_some_and(|l| l.case_id == id))
            .count();
        let message=format!("Удалить «{}»?\nНагрузок: {}.\nЗагружение, его нагрузки и коэффициенты в сочетаниях будут удалены. Результаты расчета будут сброшены.",case.display_name(),count);
        if rfd::MessageDialog::new()
            .set_title("Удаление загружения")
            .set_description(message)
            .set_level(rfd::MessageLevel::Warning)
            .set_buttons(rfd::MessageButtons::YesNo)
            .show()
            == rfd::MessageDialogResult::Yes
        {
            self.delete_load_case(id);
        }
    }
    fn delete_load_case(&mut self, id: u64) {
        if self.input_locked {
            return;
        }
        if self.load_cases.len() <= 1 || !self.load_cases.iter().any(|c| c.id == id) {
            return;
        }
        let belongs = |c: &Contour| c.load.as_ref().is_some_and(|l| l.case_id == id);
        if self.loads_locked && self.contours.iter().any(belongs) {
            return;
        }
        if self.drag.is_some() || self.arc_placement.is_some() || self.draft_load_case == Some(id) {
            self.cancel();
        }
        self.contours.retain(|c| !belongs(c));
        // Geometry undo must not resurrect loads belonging to a deleted case.
        for snapshot in &mut self.history {
            snapshot.retain(|c| !belongs(c));
        }
        self.load_cases.retain(|c| c.id != id);
        for combo in &mut self.combinations {
            combo.coefficients.remove(&id);
        }
        if self.active_load == id {
            self.active_load = self.load_cases[0].id;
        }
        if self
            .selected
            .is_some_and(|id| !self.contours.iter().any(|c| c.id == id))
        {
            self.select_only(None);
        }
        self.analysis = None;
        self.analysis_input = None;
        self.analysis_job = None;
        self.combination_cache = None;
        self.combination_error.clear();
        self.menu_target = None;
        self.foundation_point_target = None;
        self.arc_menu_target = None;
        self.status = "Загружение удалено. Геометрическая сетка сохранена.".into();
    }
    fn draw_origin_axes(&self, painter: &egui::Painter, rect: Rect) {
        let origin = self.camera.screen([0., 0.], rect);
        if !origin.x.is_finite() || !origin.y.is_finite() || !rect.expand(80.).contains(origin) {
            return;
        }
        let (s, c) = self.camera.angle.sin_cos();
        for (direction, label, color) in [
            (
                egui::vec2(c as f32, -s as f32),
                "X",
                Color32::from_rgb(245, 95, 85),
            ),
            (
                egui::vec2(-s as f32, -c as f32),
                "Y",
                Color32::from_rgb(100, 220, 130),
            ),
        ] {
            let tip = origin + direction * 58.;
            painter.arrow(origin, direction * 58., Stroke::new(2.0_f32, color));
            let p = tip + direction * 10.;
            let galley =
                painter.layout_no_wrap(label.into(), egui::FontId::proportional(13.), color);
            let area = Rect::from_center_size(p, galley.size() + egui::vec2(4., 2.));
            painter.rect_filled(area, 2., Color32::from_rgba_unmultiplied(20, 24, 30, 225));
            painter.galley(area.min + egui::vec2(2., 1.), galley, color);
        }
        painter.circle_filled(origin, 3., Color32::WHITE);
    }
    fn set_load_visibility(&mut self, visible: bool) {
        self.show_loads = visible;
        if !visible {
            if !self.tree_load_selection.is_empty() {
                self.select_only(None);
            }
            if self.mode.is_some_and(|k| k.is_load())
                || self
                    .drag
                    .as_ref()
                    .is_some_and(|d| d.original.kind.is_load())
                || self
                    .arc_placement
                    .as_ref()
                    .is_some_and(|a| a.original.kind.is_load())
            {
                self.cancel();
            }
            if self
                .selected
                .is_some_and(|id| self.contours.iter().any(|c| c.id == id && c.kind.is_load()))
            {
                self.select_only(None);
            }
            self.menu_target = None;
            self.arc_menu_target = None;
        }
    }
    fn loads_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Нагрузки")
            .default_open(false)
            .show(ui, |ui| {
                if !self.show_loads {
                    ui.colored_label(
                        Color32::from_rgb(70, 160, 255),
                        "Сначала включите нагрузки во флагах рисования",
                    );
                }
                ui.horizontal(|ui| {
                    ui.label("Интенсивность (расчетная)");
                    let mut value = self.new_load.in_units(self.load_units, self.gravity);
                    if ui
                        .add_enabled(
                            !self.input_locked,
                            egui::DragValue::new(&mut value).speed(0.1),
                        )
                        .changed()
                    {
                        self.new_load = super::loads::Magnitude {
                            value,
                            units: self.load_units,
                        };
                    }
                    let dimension = self
                        .mode
                        .filter(|k| k.is_load())
                        .map_or(0, |k| k.load_dimension());
                    ui.label(
                        if self.new_load_moment && self.mode == Some(Kind::LoadPoint) {
                            format!("{}·м", self.load_units.intensity_label(0))
                        } else {
                            self.load_units.intensity_label(dimension)
                        },
                    );
                    let moment = self.new_load_moment && self.mode == Some(Kind::LoadPoint);
                    egui::ComboBox::from_id_salt("new_load_axis")
                        .selected_text(self.new_load_direction.label(moment))
                        .show_ui(ui, |ui| {
                            for axis in [
                                crate::plan_trace::loads::Direction::X,
                                crate::plan_trace::loads::Direction::Y,
                                crate::plan_trace::loads::Direction::Z,
                            ] {
                                ui.selectable_value(
                                    &mut self.new_load_direction,
                                    axis,
                                    axis.label(moment),
                                );
                            }
                        });
                    if !moment {
                        ui.small("+Z: вниз");
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    for (kind, moment) in [
                        (Kind::LoadPoint, false),
                        (Kind::LoadLine, false),
                        (Kind::LoadArea, false),
                        (Kind::LoadPoint, true),
                    ] {
                        if ui
                            .add_enabled(
                                self.load.is_none()
                                    && !self.loads_locked
                                    && !self.input_locked
                                    && self.show_loads,
                                egui::Button::new(if moment {
                                    "Сосредоточенный момент"
                                } else {
                                    kind.label()
                                })
                                .selected(
                                    self.mode == Some(kind) && self.new_load_moment == moment,
                                ),
                            )
                            .clicked()
                        {
                            self.begin(kind);
                            self.new_load_moment = moment && self.mode == Some(Kind::LoadPoint);
                        }
                    }
                });
                if ui
                    .add_enabled(
                        self.selected_load_area().is_some()
                            && !self.loads_locked
                            && !self.input_locked,
                        egui::Button::new("Отверстие нагрузки"),
                    )
                    .clicked()
                {
                    self.begin(Kind::LoadHole);
                }
                let mut select = None;
                let mut remove = None;
                let mut load_mesh_edit = None;
                let units = self.load_units;
                let gravity = self.gravity;
                let active = self.active_load;
                let locked = self.loads_locked || self.input_locked;
                let mesh_editable = !locked
                    && self.mesh_job.is_none()
                    && self.analysis_job.is_none()
                    && self.alternative_job.is_none();
                let display_factor = if self.show_loads {
                    self.displayed_load_factor(active)
                } else {
                    None
                };
                let visible = self.show_loads && display_factor.is_some();
                let mut holes = std::collections::BTreeMap::<u64, Vec<u64>>::new();
                for hole in self.contours.iter().filter(|c| c.kind == Kind::LoadHole) {
                    if let Some(parent) = hole.parent {
                        holes.entry(parent).or_default().push(hole.id);
                    }
                }
                let loads = self
                    .contours
                    .iter()
                    .filter(|c| {
                        c.kind.is_load()
                            && c.kind != Kind::LoadHole
                            && c.load.as_ref().is_some_and(|l| l.case_id == active)
                    })
                    .map(|c| c.id)
                    .collect::<std::collections::BTreeSet<_>>();
                let groups = if self.building.enabled {
                    let mut floors = self.building.floors.clone();
                    floors.sort_by(|a, b| a.elevation_mm.total_cmp(&b.elevation_mm));
                    floors
                        .into_iter()
                        .filter_map(|floor| {
                            let ids = loads
                                .iter()
                                .copied()
                                .filter(|id| self.building.owner(*id) == floor.id)
                                .collect::<std::collections::BTreeSet<_>>();
                            (!ids.is_empty()).then(|| {
                                (
                                    Some(floor.id),
                                    format!("{} ({})", floor.name, ids.len()),
                                    ids,
                                )
                            })
                        })
                        .collect::<Vec<_>>()
                } else {
                    vec![(None, String::new(), loads)]
                };
                for (floor_id, floor_name, floor_ids) in groups {
                    let mut table = |ui: &mut egui::Ui| {
                        egui::Grid::new(("active_case_loads", floor_id))
                            .striped(true)
                            .show(ui, |ui| {
                                for label in ["Нагрузка", "Расчетная", "На виде", "Сетка", ""]
                                {
                                    ui.label(label);
                                }
                                ui.end_row();
                                for c in self.contours.iter_mut().filter(|c| {
                                    floor_ids.contains(&c.id)
                                        && c.kind.is_load()
                                        && c.kind != Kind::LoadHole
                                        && c.load.as_ref().is_some_and(|l| l.case_id == active)
                                }) {
                                    if let Some(data) = &mut c.load {
                                        let label = if data.moment {
                                            "Сосредоточенный момент"
                                        } else {
                                            c.kind.label()
                                        };
                                        let name_id = ui.id().with(("load_name", c.id));
                                        let mut name = if data.name.is_empty()
                                            && !ui.memory(|m| m.has_focus(name_id))
                                        {
                                            format!("{} {}", label, c.id)
                                        } else {
                                            data.name.clone()
                                        };
                                        let response = ui.add(
                                            egui::TextEdit::singleline(&mut name)
                                                .id(name_id)
                                                .desired_width(150.),
                                        );
                                        if response.changed() {
                                            data.name = name;
                                        }
                                        if visible
                                            && (response.clicked() || response.gained_focus())
                                        {
                                            select = Some(c.id);
                                        }
                                        response.on_hover_text(format!(
                                            "{} · {} · ID {}",
                                            label,
                                            data.direction.label(data.moment),
                                            c.id
                                        ));
                                        let mut value = data.magnitude.in_units(units, gravity);
                                        if ui
                                            .add_enabled(
                                                !locked,
                                                egui::DragValue::new(&mut value).speed(0.1).suffix(
                                                    format!(
                                                        " {}",
                                                        units.intensity_label(if data.moment {
                                                            3
                                                        } else {
                                                            c.kind.load_dimension()
                                                        })
                                                    ),
                                                ),
                                            )
                                            .changed()
                                        {
                                            data.magnitude =
                                                super::loads::Magnitude { value, units };
                                        }
                                        ui.label(
                                            display_factor
                                                .map(|f| {
                                                    format!(
                                                        "{:.2}",
                                                        data.magnitude.in_units(units, gravity) * f
                                                    )
                                                })
                                                .unwrap_or_else(|| "—".into()),
                                        );
                                        let mut mesh_constraint = data.mesh_constraint;
                                        let response = ui
                                            .add_enabled(
                                                mesh_editable,
                                                egui::Checkbox::without_text(&mut mesh_constraint),
                                            )
                                            .on_hover_text(super::loads::MESH_CONSTRAINT_HELP);
                                        if response.changed() {
                                            load_mesh_edit =
                                                Some((c.id, mesh_constraint, response.id));
                                        }
                                        if ui
                                            .add_enabled(!locked, egui::Button::new("−").small())
                                            .on_hover_text("Удалить нагрузку")
                                            .clicked()
                                        {
                                            remove = Some(c.id);
                                        }
                                        ui.end_row();
                                        if let Some(children) = holes.get(&c.id) {
                                            egui::CollapsingHeader::new(format!(
                                                "Отверстия ({})",
                                                children.len()
                                            ))
                                            .id_salt(("load_holes", c.id))
                                            .show(
                                                ui,
                                                |ui| {
                                                    for &id in children {
                                                        ui.horizontal(|ui| {
                                                            if ui
                                                                .add_enabled(
                                                                    visible,
                                                                    egui::Button::new(format!(
                                                                        "Отверстие {id}"
                                                                    ))
                                                                    .selected(
                                                                        self.selected == Some(id)
                                                                            || self
                                                                                .selection_ids
                                                                                .contains(&id),
                                                                    ),
                                                                )
                                                                .clicked()
                                                            {
                                                                select = Some(id);
                                                            }
                                                            if ui
                                                                .add_enabled(
                                                                    !locked,
                                                                    egui::Button::new("−").small(),
                                                                )
                                                                .on_hover_text(
                                                                    "Удалить отверстие нагрузки",
                                                                )
                                                                .clicked()
                                                            {
                                                                remove = Some(id);
                                                            }
                                                        });
                                                    }
                                                },
                                            );
                                            ui.label("");
                                            ui.label("");
                                            ui.label("");
                                            ui.end_row();
                                        }
                                    }
                                }
                            });
                    };
                    if let Some(id) = floor_id {
                        egui::CollapsingHeader::new(floor_name)
                            .id_salt(("load_floor", id))
                            .default_open(false)
                            .show(ui, |ui| table(ui));
                    } else {
                        table(ui);
                    }
                }
                if let Some(id) = select {
                    if self.building.enabled {
                        self.change_floor(self.building.owner(id));
                    }
                    self.cancel();
                    self.select_only(Some(id));
                }
                if let Some(id) = remove {
                    if self.building.enabled {
                        self.change_floor(self.building.owner(id));
                    }
                    self.select_only(Some(id));
                    self.remove();
                }
                if let Some((id, enabled, widget)) = load_mesh_edit {
                    self.property_apply(
                        &[object_properties::Item::Contour(id)],
                        object_properties::Change::LoadMeshConstraint(enabled),
                        widget,
                    );
                }
            });
        self.sync_result_units();
    }
    fn load_caption(&self, painter: &egui::Painter, center: Pos2, text: String) -> Rect {
        let color = self.display_text.ink();
        let galley = painter.layout_no_wrap(
            text,
            egui::FontId::proportional(self.display_text.size),
            color,
        );
        let rect = Rect::from_center_size(center, galley.size() + egui::vec2(6., 3.));
        painter.rect_filled(rect, 2., Color32::from_rgba_unmultiplied(25, 29, 36, 225));
        painter.galley(rect.min + egui::vec2(3., 1.5), galley, color);
        rect
    }
    fn draw_self_weight_loads(&self, painter: &egui::Painter, camera: Camera, rect: Rect) {
        if !self.show_loads {
            return;
        }
        let factor: f64 = self
            .load_cases
            .iter()
            .filter(|case| case.kind == super::loads::LoadType::SelfWeight)
            .filter_map(|case| self.displayed_load_factor(case.id))
            .sum();
        if !factor.is_finite() || factor == 0. {
            return;
        }
        if self.drawing_visibility.loads[0] {
            let ink = self.contour_color(Kind::LoadArea);
            let fill = Color32::from_rgba_unmultiplied(
                ink.r(),
                ink.g(),
                ink.b(),
                (self.drawing_visibility.load_opacity * 255.).round() as u8,
            );
            for slab in self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Slab && self.on_active_floor(c))
            {
                let material = self.materials.get(&slab.id).copied().unwrap_or_default();
                let value =
                    material.self_weight_kn_m2(self.gravity) * self.force_result_factor() * factor;
                if !value.is_finite() || value == 0. {
                    continue;
                }
                // Display-only contour: it is never inserted into the editable model.
                let mut area = slab.clone();
                area.kind = Kind::LoadArea;
                area.load = None;
                let triangles = self.cached_load_fill(&area);
                let mut mesh = egui::Mesh::default();
                for &triangle in triangles.iter() {
                    append_triangle(&mut mesh, triangle, camera, rect, fill);
                }
                if !mesh.is_empty() {
                    painter.add(egui::Shape::mesh(mesh));
                }
                draw_path(
                    painter,
                    &slab.points,
                    true,
                    camera,
                    rect,
                    Stroke::new(2_f32, ink),
                );
                for hole in self
                    .contours
                    .iter()
                    .filter(|c| c.active_hole() && c.parent == Some(slab.id))
                {
                    draw_path(
                        painter,
                        &hole.points,
                        true,
                        camera,
                        rect,
                        Stroke::new(2_f32, ink),
                    );
                }
                if self.load_values_visible() && !triangles.is_empty() {
                    let at = area
                        .load_label_positions()
                        .into_iter()
                        .next()
                        .unwrap_or(slab.points[0]);
                    let inside = triangles.iter().any(|t| geometry::inside(at, t));
                    let at = if inside {
                        at
                    } else {
                        let t = triangles
                            .iter()
                            .max_by(|a, b| geometry::area(*a).total_cmp(&geometry::area(*b)))
                            .unwrap();
                        [
                            (t[0][0] + t[1][0] + t[2][0]) / 3.,
                            (t[0][1] + t[1][1] + t[2][1]) / 3.,
                        ]
                    };
                    self.load_caption(
                        painter,
                        camera.screen(at, rect),
                        format!(
                            "{} {}",
                            self.display_text.number(value),
                            self.load_units.intensity_label(2)
                        ),
                    );
                }
            }
        }
        if self.support_height.enabled && self.drawing_visibility.loads[1] {
            let sides = if self.support_height.direction == super::fem::shell::Direction::Both {
                2.
            } else {
                1.
            };
            for support in self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Support && self.on_active_floor(c))
            {
                let material = self
                    .support_height
                    .materials
                    .get(&support.id)
                    .copied()
                    .unwrap_or_else(super::fem::element::Material::wall_default);
                let value = material.self_weight_kn_m2(self.gravity)
                    * self.support_height.height_mm
                    / 1000.
                    * sides
                    * self.force_result_factor()
                    * factor;
                if !value.is_finite() || value == 0. {
                    continue;
                }
                draw_path(
                    painter,
                    &support.points,
                    support.closed(),
                    camera,
                    rect,
                    Stroke::new(2_f32, self.contour_color(Kind::LoadLine)),
                );
                if self.load_values_visible() {
                    let mut line = support.clone();
                    line.kind = Kind::LoadLine;
                    for at in line.load_label_positions() {
                        self.load_caption(
                            painter,
                            camera.screen(at, rect),
                            format!(
                                "{} {}",
                                self.display_text.number(value),
                                self.load_units.intensity_label(1)
                            ),
                        );
                    }
                }
            }
        }
        if self.drawing_visibility.loads[1] {
            for beam in self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Beam && self.on_active_floor(c))
            {
                let Some(section) = self.support_height.beams.get(&beam.id) else {
                    continue;
                };
                let value = section
                    .material
                    .weight
                    .in_units(self.load_units, self.gravity)
                    * section.area()
                    * factor;
                if !value.is_finite() || value == 0. {
                    continue;
                }
                draw_path(
                    painter,
                    &beam.points,
                    beam.closed(),
                    camera,
                    rect,
                    Stroke::new(2_f32, self.contour_color(Kind::LoadLine)),
                );
                if self.load_values_visible() {
                    let mut line = beam.clone();
                    line.kind = Kind::LoadLine;
                    for at in line.load_label_positions() {
                        self.load_caption(
                            painter,
                            camera.screen(at, rect),
                            format!(
                                "{} {}",
                                self.display_text.number(value),
                                self.load_units.intensity_label(1)
                            ),
                        );
                    }
                }
            }
        }
    }
    fn draw_loads(&self, painter: &egui::Painter, camera: Camera, rect: Rect) {
        if self.show_results
            && matches!(self.result_field, 7..=9)
            && self.selected_result().is_some()
        {
            return;
        }
        let start = self.draw_order.start(painter);
        self.draw_self_weight_loads(painter, camera, rect);
        self.draw_order.finish(painter, 7, start);
        for area_first in [true, false] {
            for original in self.contours.iter().filter(|c| {
                c.kind.is_load()
                    && self.contour_visible(c)
                    && (c.kind == Kind::LoadArea) == area_first
            }) {
                let load_start = self.draw_order.start(painter);
                let c = self
                    .arc_placement
                    .as_ref()
                    .filter(|e| e.original.id == original.id)
                    .and_then(|e| e.preview.as_ref())
                    .unwrap_or(original);
                let Some(_) = &c.load else {
                    continue;
                };
                let color = self.object_ink(c.id, c.kind);
                if matches!(
                    c.kind,
                    Kind::LoadPoint | Kind::Column | Kind::PointRestraint
                ) {
                    let p = camera.screen(c.points[0], rect);
                    painter.circle_stroke(p, 7., Stroke::new(1.7_f32, color));
                    for sign in [-1., 1.] {
                        painter.line_segment(
                            [
                                p + egui::vec2(-4.5, sign * 4.5),
                                p + egui::vec2(4.5, -sign * 4.5),
                            ],
                            Stroke::new(1.5_f32, color),
                        );
                    }
                    if c.kind != Kind::LoadHole && self.load_values_visible() {
                        if let Some(value) = self.displayed_load_value(c) {
                            self.load_caption(
                                painter,
                                p + egui::vec2(45., -15.),
                                format!(
                                    "{} {}",
                                    self.display_text.number(value),
                                    self.load_units.intensity_label(0)
                                ),
                            );
                        }
                    }
                } else {
                    if c.kind == Kind::LoadArea {
                        let triangles = self.cached_load_fill(c);
                        let mut mesh = egui::Mesh::default();
                        let ink = self.contour_color(Kind::LoadArea);
                        let color = Color32::from_rgba_unmultiplied(
                            ink.r(),
                            ink.g(),
                            ink.b(),
                            (self.drawing_visibility.load_opacity * 255.).round() as u8,
                        );
                        for &t in triangles.iter() {
                            append_triangle(&mut mesh, t, camera, rect, color);
                        }
                        if !mesh.is_empty() {
                            painter.add(egui::Shape::mesh(mesh));
                        }
                    }
                    draw_path(
                        painter,
                        &c.points,
                        c.closed(),
                        camera,
                        rect,
                        Stroke::new(if self.is_selected(c.id) { 3_f32 } else { 2_f32 }, color),
                    );
                    if c.kind != Kind::LoadHole && self.load_values_visible() {
                        if let Some(value) = self.displayed_load_value(c) {
                            let text = format!(
                                "{} {}",
                                self.display_text.number(value),
                                self.load_units.intensity_label(c.load_dimension())
                            );
                            for p in c.load_label_positions() {
                                let q = camera.screen(p, rect);
                                if rect.contains(q) {
                                    self.load_caption(painter, q, text.clone());
                                }
                            }
                        }
                    }
                }
                if self.is_selected(c.id) && self.mode.is_none() && self.arc_placement.is_none() {
                    for (_, a, b) in c.straight_edges() {
                        let q = camera.screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], rect);
                        if rect.contains(q) {
                            painter.circle_filled(q, 4., Color32::WHITE);
                        }
                    }
                    for p in c.handles() {
                        let q = camera.screen(p, rect);
                        if rect.contains(q) {
                            painter.rect_filled(
                                Rect::from_center_size(q, Vec2::splat(8.)),
                                0.,
                                Color32::WHITE,
                            );
                        }
                    }
                }
                self.draw_order
                    .finish(painter, draw_order::category(c.kind), load_start);
            }
        }
    }
    fn hit_contour(&self, c: &Contour, p: Point, zoom: f64) -> bool {
        if !self.selectable(c) {
            return false;
        }
        if matches!(
            c.kind,
            Kind::LoadPoint | Kind::Column | Kind::PointRestraint
        ) {
            return c
                .points
                .first()
                .is_some_and(|q| (q[0] - p[0]).hypot(q[1] - p[1]) * zoom <= 10.);
        }
        let near_edge = {
            let count = if c.closed() {
                c.points.len()
            } else {
                c.points.len().saturating_sub(1)
            };
            (0..count).any(|i| {
                let a = c.points[i];
                let b = c.points[(i + 1) % c.points.len()];
                let d = [b[0] - a[0], b[1] - a[1]];
                let len = d[0] * d[0] + d[1] * d[1];
                if len < 1e-12 {
                    return false;
                }
                let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len).clamp(0., 1.);
                (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]) * zoom <= 8.
            })
        };
        near_edge || (c.closed() && c.kind != Kind::LoadLine && geometry::inside(p, &c.points))
    }
    fn contour_visible(&self, c: &Contour) -> bool {
        if c.kind == Kind::RigidLink && !self.building.enabled {
            return false;
        }
        if self.fe_isolated()
            || (self.fe_fragment_active() && matches!(c.kind, Kind::Beam | Kind::Column))
        {
            return false;
        }
        let visible = if punching::is_punching(c) {
            self.drawing_visibility.punching
        } else {
            self.drawing_visibility.border(c.kind)
                || (c.kind == Kind::Slab && self.drawing_visibility.fill)
        };
        visible
            && !self.object_hidden(c)
            && self.on_view_floor(c)
            && (!c.kind.is_load()
                || self.tree_load_selection.contains(&c.id)
                || (self.show_loads
                    && c.load
                        .as_ref()
                        .is_some_and(|l| self.displayed_load_factor(l.case_id).is_some())))
    }
    fn active_load_name(&self) -> String {
        self.load_cases
            .iter()
            .find(|c| c.id == self.active_load)
            .map_or_else(|| "Загружение".into(), |c| c.display_name())
    }
    fn kind_index(kind: Kind) -> usize {
        match kind {
            Kind::Slab | Kind::MeshRegion | Kind::Auxiliary | Kind::Punching | Kind::Foundation => {
                0
            }
            Kind::Hole => 1,
            Kind::Support => 2,
            Kind::LoadPoint | Kind::LoadLine | Kind::LoadArea | Kind::LoadHole => 3,
            Kind::Beam | Kind::Column => 4,
            Kind::Triangulation => 5,
            Kind::Restraint | Kind::PointRestraint => 6,
            Kind::RigidLink => 7,
        }
    }
    fn is_kind_locked(&self, kind: Kind) -> bool {
        if kind == Kind::Auxiliary {
            return false;
        }
        if self.input_locked {
            return true;
        }
        if kind == Kind::Foundation {
            return self.foundation_locked;
        }
        if kind.is_restraint() {
            return self.restraints_locked;
        }
        if kind == Kind::RigidLink {
            return self.rigid_links_locked;
        }
        if kind == Kind::Triangulation {
            return self.triangulation_locked;
        }
        if kind == Kind::Column {
            return self.columns_locked;
        }
        if kind == Kind::Beam {
            return self.beams_locked;
        }
        if kind.is_load() {
            self.loads_locked
        } else {
            self.contour_locks[Self::kind_index(kind)]
        }
    }
    fn contour_editable(&self, id: u64) -> bool {
        self.contours.iter().find(|c| c.id == id).is_some_and(|c| {
            c.placement.cut_source.is_none()
                && c.placement.rigid_link_targets.is_none()
                && self.contour_visible(c)
                && !self.is_kind_locked(c.kind)
        })
    }
    pub(super) fn movement_locked_contours(&self) -> std::collections::BTreeSet<u64> {
        let mut pending = self
            .contours
            .iter()
            .filter(|c| c.placement.movement_locked)
            .map(|c| c.id)
            .collect::<Vec<_>>();
        let mut locked = std::collections::BTreeSet::new();
        if pending.is_empty() {
            return locked;
        }
        let by_id = self
            .contours
            .iter()
            .map(|c| (c.id, c))
            .collect::<BTreeMap<_, _>>();
        while let Some(id) = pending.pop() {
            if !locked.insert(id) {
                continue;
            }
            if let Some(c) = by_id.get(&id) {
                pending.extend(c.parent);
                pending.extend(c.placement.cut_source);
                pending.extend(c.placement.rigid_link_targets.into_iter().flatten());
            }
        }
        locked
    }
    fn contour_geometry_editable(&self, id: u64) -> bool {
        self.contour_editable(id) && !self.movement_locked_contours().contains(&id)
    }
    fn movement_blocked(&mut self, id: u64) -> bool {
        if self.movement_locked_contours().contains(&id) {
            self.status = "У объекта заблокировано перемещение.".into();
            true
        } else {
            false
        }
    }
    fn contour_removable(&self, id: u64) -> bool {
        self.contours.iter().find(|c| c.id == id).is_some_and(|c| {
            c.placement.cut_source.is_none()
                && self.contour_visible(c)
                && !self.is_kind_locked(c.kind)
        }) && !self.contours.iter().any(|c| {
            (c.parent == Some(id)
                || c.placement
                    .rigid_link_targets
                    .is_some_and(|targets| targets.contains(&id)))
                && self.is_kind_locked(c.kind)
        })
    }
    fn kind_editing_blocked(&mut self, kind: Kind) -> bool {
        let locked = self.is_kind_locked(kind);
        if locked {
            self.status =
                "Этот тип объектов заблокирован. Снимите соответствующую галочку справа.".into();
        }
        locked
    }
    fn undo_changes_locked(&self, before: &[Contour]) -> bool {
        self.contours
            .iter()
            .chain(before)
            .filter(|c| self.is_kind_locked(c.kind))
            .any(|c| {
                let current = self.contours.iter().find(|v| v.id == c.id);
                let previous = before.iter().find(|v| v.id == c.id);
                if let (Some(a), Some(b)) = (current, previous) {
                    if a.kind == Kind::Punching && b.kind == Kind::Punching {
                        let mut a = a.clone();
                        let mut b = b.clone();
                        if let Some(r) = &mut a.refinement {
                            r.inactive_edges.clear();
                        }
                        if let Some(r) = &mut b.refinement {
                            r.inactive_edges.clear();
                        }
                        return a != b;
                    }
                }
                current != previous
            })
    }
    fn set_kind_lock(&mut self, kind: Kind, locked: bool) {
        if locked
            && (self
                .mode
                .is_some_and(|active| Self::kind_index(active) == Self::kind_index(kind))
                || self
                    .drag
                    .as_ref()
                    .is_some_and(|d| Self::kind_index(d.original.kind) == Self::kind_index(kind))
                || self
                    .arc_placement
                    .as_ref()
                    .is_some_and(|a| Self::kind_index(a.original.kind) == Self::kind_index(kind)))
        {
            let selected = self.selected;
            self.cancel();
            self.selected = selected;
        }
        if kind == Kind::Foundation {
            self.foundation_locked = locked;
        } else if kind == Kind::Column {
            self.columns_locked = locked;
        } else if kind.is_restraint() {
            self.restraints_locked = locked;
        } else if kind == Kind::RigidLink {
            self.rigid_links_locked = locked;
        } else if kind == Kind::Triangulation {
            self.triangulation_locked = locked;
        } else if kind == Kind::Beam {
            self.beams_locked = locked;
        } else if kind.is_load() {
            self.loads_locked = locked;
        } else {
            self.contour_locks[Self::kind_index(kind)] = locked;
        }
        self.contours_locked = self.contour_locks.iter().all(|v| *v)
            && self.loads_locked
            && self.underlays_locked
            && self.beams_locked
            && self.columns_locked
            && self.triangulation_locked
            && self.restraints_locked
            && self.rigid_links_locked
            && self.foundation_locked;
        self.menu_target = None;
        self.arc_menu_target = None;
    }
    fn begin_edge_arc(&mut self, id: u64, edge: usize) {
        let Some(original) = self.contours.iter().find(|c| c.id == id).cloned() else {
            return;
        };
        if self.kind_editing_blocked(original.kind) || self.movement_blocked(id) {
            return;
        }
        let Some((_, a, b)) = original
            .straight_edges()
            .into_iter()
            .find(|(i, _, _)| *i == edge)
        else {
            return;
        };
        self.cancel();
        self.select_only(Some(id));
        self.arc_placement = Some(ArcPlacement {
            original,
            edge,
            a,
            b,
            preview: None,
            radius: None,
        });
        self.status = "Двигайте курсор для задания дуги. ЛКМ — подтвердить, Esc — отменить.".into();
    }
    fn preview_edge_arc(&mut self, pointer: Point) {
        let Some(mut edit) = self.arc_placement.take() else {
            return;
        };
        edit.preview = None;
        edit.radius = None;
        let d = [edit.b[0] - edit.a[0], edit.b[1] - edit.a[1]];
        let length = d[0].hypot(d[1]);
        if length > 1e-6 {
            let center = [edit.a[0] + d[0] * 0.5, edit.a[1] + d[1] * 0.5];
            let normal = [-d[1] / length, d[0] / length];
            let s = (pointer[0] - center[0]) * normal[0] + (pointer[1] - center[1]) * normal[1];
            if s.abs() >= 0.1 {
                let through = [center[0] + s * normal[0], center[1] + s * normal[1]];
                let mut candidate = edit.original.clone();
                match candidate.bend_edge(edit.edge, through).and_then(|_| {
                    if candidate.kind == Kind::Foundation {
                        candidate.foundation = Some(crate::plan_trace::foundation::remap_values(
                            &edit.original,
                            &candidate,
                            self.gravity,
                            self.load_units,
                        ));
                    }
                    model::validate_candidate(&self.contours, &candidate)
                }) {
                    Ok(()) => {
                        edit.radius = Some((length * length * 0.25 + s * s) / (2. * s.abs()));
                        edit.preview = Some(candidate);
                        self.status = "ЛКМ — подтвердить дугу, Esc — отменить.".into();
                    }
                    Err(e) => self.status = e,
                }
            } else {
                self.status =
                    "Сместите курсор в сторону от отрезка для задания изгиба дуги.".into();
            }
        }
        self.arc_placement = Some(edit);
    }
    fn accept_edge_arc(&mut self) {
        let Some((id, kind)) = self
            .arc_placement
            .as_ref()
            .map(|e| (e.original.id, e.original.kind))
        else {
            return;
        };
        if self.kind_editing_blocked(kind) || self.movement_blocked(id) {
            return;
        }
        let Some(edit) = self.arc_placement.as_ref() else {
            return;
        };
        let Some(candidate) = edit.preview.clone() else {
            return;
        };
        let Some(index) = self.contours.iter().position(|c| c.id == candidate.id) else {
            self.arc_placement = None;
            return;
        };
        if self.contours[index] != edit.original {
            self.arc_placement = None;
            self.status = "Объект изменён. Постройте дугу заново.".into();
            return;
        }
        let geometry_change = !candidate.kind.is_load();
        self.history.push(self.contours.clone());
        self.contours[index] = candidate;
        self.arc_placement = None;
        self.slab_cache = None;
        if geometry_change {
            self.fe_mesh = None;
        }
        self.status = "Ребро заменено дугой. Ctrl+Z — отменить.".into();
    }
    fn bottom_bar(&mut self, ctx: &egui::Context) {
        let previous = self.active_load;
        let panel_id = egui::Id::new("construction_grid_bar");
        let previous_height =
            egui::panel::PanelState::load(ctx, panel_id).map_or(0., |state| state.rect.height());
        let panel = egui::TopBottomPanel::bottom(panel_id).show(ctx, |ui| {
            ui.horizontal_wrapped(|ui| {
                if drawing_flags::window_button(ui, self.drawing_flags_open).clicked() {
                    self.drawing_flags_open = !self.drawing_flags_open;
                }
                if drawing_flags::properties_button(ui, self.properties.open).clicked() {
                    self.properties.open = !self.properties.open;
                }
                if contour_filter::window_button(ui, self.contour_filter.open).clicked() {
                    self.contour_filter.open = !self.contour_filter.open;
                }
                ui.separator();
                let mut loads = self.show_loads;
                if load_visibility_icon(ui, loads, false)
                    .on_hover_text("Показывать нагрузки")
                    .clicked()
                {
                    loads = !loads;
                    self.set_load_visibility(loads);
                }
                if load_visibility_icon(ui, self.show_load_values, true)
                    .on_hover_text("Показывать значения нагрузок")
                    .clicked()
                {
                    self.show_load_values = !self.show_load_values;
                }
                if ui
                    .add_enabled_ui(self.analysis.is_some() || self.fe_mesh.is_some(), |ui| {
                        result_values_icon(ui, self.show_result_values)
                    })
                    .inner
                    .on_hover_text("Показывать значения результатов")
                    .clicked()
                {
                    self.show_result_values = !self.show_result_values;
                }
                if self.analysis.is_some() || self.input_locked {
                    let available = self.analysis_job.is_none() && self.alternative_job.is_none();
                    if ui
                        .add_enabled_ui(available, |ui| input_lock_icon(ui, self.input_locked))
                        .inner
                        .on_hover_text(if self.input_locked {
                            "Исходные данные заблокированы. Нажмите, чтобы разблокировать"
                        } else {
                            "Нажмите, чтобы заблокировать исходные данные"
                        })
                        .clicked()
                    {
                        self.input_locked = !self.input_locked;
                        if self.input_locked {
                            self.remember_manual_locks();
                            self.cancel();
                        } else {
                            self.restore_manual_locks();
                        }
                    }
                    ui.separator();
                }
                ui.toggle_value(&mut self.construction_grid.enabled, "Сетка")
                    .on_hover_text("Вспомогательная сетка. ПКМ — шаг, цвет и область.")
                    .context_menu(|ui|{
                        for (label,v) in [("dx, мм",&mut self.construction_grid.dx),("dy, мм",&mut self.construction_grid.dy),("Ширина, мм",&mut self.construction_grid.width_mm),("Высота, мм",&mut self.construction_grid.height_mm)] {
                            ui.horizontal(|ui|{ui.label(label);ui.add(egui::DragValue::new(v).range(0.1..=1e7).speed(10.));});
                        }
                        for (i,label) in ["Центр X, мм","Центр Y, мм"].into_iter().enumerate(){ui.horizontal(|ui|{ui.label(label);ui.add(egui::DragValue::new(&mut self.construction_grid.center[i]).speed(100.));});}
                        let rgb = self.construction_grid.color;
                        let mut color = Color32::from_rgb(rgb[0],rgb[1],rgb[2]);
                        if egui::color_picker::color_picker_color32(ui, &mut color, egui::color_picker::Alpha::Opaque) { self.construction_grid.color = [color.r(),color.g(),color.b()]; }
                        ui.label("Цвет RGB");
                        ui.horizontal(|ui| { for c in &mut self.construction_grid.color { ui.add(egui::DragValue::new(c).range(0..=255)); } });
                        if ui.button("Указать базовую точку").clicked() { self.grid_pick_origin = true; ui.close_menu(); }
                    });
                ui.separator();
                self.floor_selector(ui);
                if self.spatial.enabled && self.building.enabled {
                    if active_floor_icon(ui, self.active_floor_only())
                        .on_hover_text("Активный этаж: показывать только его объекты и результаты. Отключение восстанавливает прежнюю видимость.")
                        .clicked() {
                        self.toggle_active_floor_only();
                    }
                }
                self.support_storey_selector(ui);
                self.result_context_controls(ui);
                ui.separator();
                self.snap_controls(ui);
                ui.separator();
                self.measurement_button(ui);
            });
        });
        // egui anchors this pass to the previous height before it knows how many rows fit.
        if (panel.response.rect.height() - previous_height).abs() > 0.5 {
            ctx.request_repaint();
        }
        if previous != self.active_load {
            if self.mode.is_some_and(|k| k.is_load())
                || self
                    .drag
                    .as_ref()
                    .is_some_and(|d| d.original.kind.is_load())
                || self
                    .arc_placement
                    .as_ref()
                    .is_some_and(|a| a.original.kind.is_load())
            {
                self.cancel();
            }
            if self.selected.is_some_and(|id| {
                self.contours
                    .iter()
                    .find(|c| c.id == id)
                    .is_some_and(|c| !self.contour_visible(c))
            }) {
                self.select_only(None);
            }
        }
    }
    fn draw_construction_grid(&self, painter: &egui::Painter, camera: Camera, rect: Rect) {
        if !self.construction_grid.enabled {
            return;
        }
        let corners = [
            rect.left_top(),
            rect.right_top(),
            rect.right_bottom(),
            rect.left_bottom(),
        ]
        .map(|p| camera.world(p, rect));
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for p in corners {
            bounds[0] = bounds[0].min(p[0]);
            bounds[1] = bounds[1].min(p[1]);
            bounds[2] = bounds[2].max(p[0]);
            bounds[3] = bounds[3].max(p[1]);
        }
        for line in self.construction_grid.lines(bounds, camera.zoom) {
            draw_path(
                painter,
                &line,
                false,
                camera,
                rect,
                Stroke::new(
                    0.6_f32,
                    Color32::from_rgb(
                        self.construction_grid.color[0],
                        self.construction_grid.color[1],
                        self.construction_grid.color[2],
                    ),
                ),
            );
        }
    }
    fn remember_manual_locks(&mut self) {
        if self.locks_before_auto.is_none() {
            self.locks_before_auto = Some((
                self.contour_locks,
                [
                    self.loads_locked,
                    self.columns_locked,
                    self.beams_locked,
                    self.triangulation_locked,
                    self.restraints_locked,
                    self.rigid_links_locked,
                    self.foundation_locked,
                    self.underlays_locked,
                    self.contours_locked,
                ],
            ));
        }
    }
    fn restore_manual_locks(&mut self) {
        if let Some((
            contours,
            [loads, columns, beams, tri, restraints, rigid_links, foundation, underlays, all],
        )) = self.locks_before_auto.take()
        {
            self.contour_locks = contours;
            self.columns_locked = columns;
            self.loads_locked = loads;
            self.beams_locked = beams;
            self.triangulation_locked = tri;
            self.restraints_locked = restraints;
            self.rigid_links_locked = rigid_links;
            self.foundation_locked = foundation;
            self.underlays_locked = underlays;
            self.contours_locked = all;
        }
    }
    fn set_contour_lock(&mut self, locked: bool) {
        self.set_underlay_lock(locked);
        for kind in [
            Kind::Slab,
            Kind::Hole,
            Kind::Support,
            Kind::LoadLine,
            Kind::Beam,
            Kind::Column,
            Kind::Triangulation,
            Kind::Restraint,
            Kind::RigidLink,
            Kind::Foundation,
        ] {
            self.set_kind_lock(kind, locked);
        }
    }
    fn set_underlay_lock(&mut self, locked: bool) {
        if locked {
            self.cancel_underlay_drag();
            if self.scaling_op.is_some()
                && (self.selected_underlay.is_some() || !self.underlay_selection.is_empty())
            {
                self.scaling_op = None;
            }
        }
        self.underlays_locked = locked;
        self.contours_locked = self.contour_locks.iter().all(|v| *v)
            && self.loads_locked
            && locked
            && self.beams_locked
            && self.columns_locked
            && self.triangulation_locked
            && self.restraints_locked
            && self.rigid_links_locked
            && self.foundation_locked;
    }
    fn editing_blocked(&mut self) -> bool {
        let locked = self.input_locked
            || self.loads_locked
            || self.triangulation_locked
            || self.restraints_locked
            || self.rigid_links_locked
            || self.beams_locked
            || self.columns_locked
            || self.underlays_locked
            || self.contour_locks.iter().any(|v| *v);
        if locked {
            self.status = "Для изменения единиц снимите блокировки объектов справа.".into();
        }
        locked
    }
    fn right_sidebar(&mut self, ctx: &egui::Context) {
        let compact = compact_sidebars(ctx);
        let panel = egui::SidePanel::right(if compact {
            "contour_lock_panel_compact"
        } else {
            "contour_lock_panel"
        });
        let panel = if self.right_open {
            panel
                .default_width(390.)
                .min_width(340.)
                .max_width((ctx.available_rect().width() - reserved_canvas_width(ctx)).max(0.))
        } else {
            panel.exact_width(36.).resizable(false)
        };
        panel.show(ctx, |ui| {
            if ui
                .button(if self.right_open { ">" } else { "<" })
                .on_hover_text(if self.right_open {
                    "Свернуть правую панель"
                } else {
                    "Раскрыть правую панель"
                })
                .clicked()
            {
                self.right_open = !self.right_open;
            }
            if !self.right_open {
                return;
            }
            if !compact {
                self.appearance_controls(ui);
                ui.separator();
            }
            egui::ScrollArea::new([compact, true])
                .id_salt("right_controls")
                .max_height(ui.available_height().max(0.))
                .auto_shrink([false, false])
                .scroll_bar_visibility(if compact {
                    egui::scroll_area::ScrollBarVisibility::VisibleWhenNeeded
                } else {
                    egui::scroll_area::ScrollBarVisibility::AlwaysVisible
                })
                .show(ui, |ui| {
                    if compact {
                        self.appearance_controls(ui);
                        ui.separator();
                    }
                    if self.analysis.is_some()
                        || self.fe_mesh.is_some()
                        || self.schema_analysis.mode.is_some()
                    {
                        self.result_scale_controls(ui);
                    }
                    self.section_controls(ui);
                    egui::CollapsingHeader::new("Блокировка объектов")
                        .default_open(false)
                        .show(ui, |ui| {
                            let mut all = self.contours_locked;
                            let partial = !all
                                && (self.loads_locked
                                    || self.triangulation_locked
                                    || self.restraints_locked
                                    || self.rigid_links_locked
                                    || self.beams_locked
                                    || self.columns_locked
                                    || self.underlays_locked
                                    || self.contour_locks.iter().any(|v| *v));
                            if ui
                                .add(
                                    egui::Checkbox::new(&mut all, "Блокировать объекты")
                                        .indeterminate(partial),
                                )
                                .changed()
                            {
                                self.set_contour_lock(all);
                            }
                            for (kind, label) in [
                                (Kind::Slab, "Блокировать плиты"),
                                (Kind::Hole, "Блокировать отверстия"),
                                (Kind::Support, "Блокировать стены"),
                                (Kind::LoadLine, "Блокировать нагрузки"),
                                (Kind::Column, "Блокировать колонны"),
                                (Kind::Beam, "Блокировать балки"),
                                (Kind::Triangulation, "Блокировать ограничения триангуляции"),
                                (Kind::Restraint, "Блокировать опоры"),
                                (Kind::RigidLink, "Блокировать АЖТ"),
                                (Kind::Foundation, "Блокировать основания"),
                            ] {
                                if kind == Kind::RigidLink && !self.building.enabled {
                                    continue;
                                }
                                let mut locked = self.is_kind_locked(kind);
                                if ui.checkbox(&mut locked, label).changed() {
                                    self.set_kind_lock(kind, locked);
                                }
                            }
                            let mut locked = self.underlays_locked;
                            if ui.checkbox(&mut locked, "Блокировать PDF / DXF").changed()
                            {
                                self.set_underlay_lock(locked);
                            }
                        });
                    self.selection_controls(ui);
                    egui::CollapsingHeader::new("Загружения")
                        .default_open(false)
                        .show(ui, |ui| {
                            if self.input_locked {
                                ui.disable();
                            }
                            if ui.button("Добавить").clicked() {
                                self.load_cases
                                    .push(super::loads::LoadCase::new(self.next_load));
                                self.next_load += 1;
                            }
                            let mut remove_case = None;
                            let can_remove = self.load_cases.len() > 1;
                            let locked = self.loads_locked || self.input_locked;
                            let occupied = self
                                .contours
                                .iter()
                                .filter_map(|c| c.load.as_ref().map(|l| l.case_id))
                                .collect::<std::collections::BTreeSet<_>>();
                            egui::Grid::new("load_case_table")
                                .striped(true)
                                .spacing([5., 5.])
                                .show(ui, |ui| {
                                    ui.label("Имя");
                                    ui.label("Тип");
                                    ui.label("γf");
                                    ui.label("γl");
                                    ui.label("");
                                    ui.end_row();
                                    for case in &mut self.load_cases {
                                        ui.add_sized(
                                            [110., 22.],
                                            egui::TextEdit::singleline(&mut case.name),
                                        );
                                        egui::ComboBox::from_id_salt(("load_type", case.id))
                                            .width(100.)
                                            .selected_text(case.kind.label())
                                            .show_ui(ui, |ui| {
                                                for kind in super::loads::LoadType::ALL {
                                                    if ui
                                                        .selectable_value(
                                                            &mut case.kind,
                                                            kind,
                                                            kind.label(),
                                                        )
                                                        .changed()
                                                        && kind
                                                            == super::loads::LoadType::SelfWeight
                                                    {
                                                        case.gamma_l = 1.;
                                                    }
                                                }
                                            });
                                        ui.add(
                                            egui::DragValue::new(&mut case.gamma_f)
                                                .range(0.0..=100.)
                                                .speed(0.05)
                                                .max_decimals(3),
                                        );
                                        ui.add(
                                            egui::DragValue::new(&mut case.gamma_l)
                                                .range(0.0..=100.)
                                                .speed(0.05)
                                                .max_decimals(3),
                                        );
                                        if ui
                                            .add_enabled(
                                                can_remove
                                                    && (!locked || !occupied.contains(&case.id)),
                                                egui::Button::new("−"),
                                            )
                                            .on_hover_text(if !can_remove {
                                                "Должно остаться хотя бы одно загружение"
                                            } else if locked && occupied.contains(&case.id) {
                                                "Снимите блокировку нагрузок"
                                            } else {
                                                "Удалить загружение"
                                            })
                                            .clicked()
                                        {
                                            remove_case = Some(case.id);
                                        }
                                        ui.end_row();
                                    }
                                });
                            if let Some(id) = remove_case {
                                self.request_delete_load_case(id);
                            }
                        });
                    self.loads_controls(ui);
                    self.combination_controls(ui);
                    self.draw_order_controls(ui);
                });
        });
    }
    fn rectangle_seed(&self) -> Option<Point> {
        let p = self
            .draft
            .first()
            .or(self.curve_controls.first())
            .copied()?;
        (self.draft_segments.is_empty()
            && self
                .draft
                .iter()
                .chain(&self.curve_controls)
                .all(|q| *q == p))
        .then_some(p)
    }
    fn can_choose_tool(&self, tool: Tool) -> bool {
        if self.mode == Some(Kind::RigidLink) {
            return tool == Tool::Line;
        }
        self.tool == tool
            || !matches!(tool, Tool::Ellipse | Tool::Rectangle)
            || (self.draft.is_empty() && self.curve_controls.is_empty())
            || (tool == Tool::Rectangle && self.rectangle_seed().is_some())
    }
    fn can_finish_sketch(&self) -> bool {
        if self.mode == Some(Kind::PointRestraint) {
            return self.draft.len() == 1;
        }
        if self.mode == Some(Kind::RigidLink) {
            return self.draft.len() == 2;
        }
        let minimum = if matches!(
            self.mode,
            Some(
                Kind::LoadLine
                    | Kind::Support
                    | Kind::Auxiliary
                    | Kind::Beam
                    | Kind::Triangulation
                    | Kind::Restraint
            )
        ) && !self.draft_line_closed
        {
            2
        } else {
            3
        };
        self.mode.is_some()
            && match self.tool {
                Tool::Line => self.draft.len() >= minimum,
                Tool::Spline => {
                    if self.draft.is_empty() {
                        self.curve_controls.len() >= minimum
                    } else {
                        self.draft.len() + self.curve_controls.len().saturating_sub(1) >= minimum
                    }
                }
                Tool::Arc => self.draft.len() >= minimum && self.curve_controls.len() <= 1,
                Tool::Ellipse | Tool::Rectangle => {
                    self.draft.len() >= minimum && self.curve_controls.is_empty()
                }
            }
    }
    fn sketch_toolbar(&mut self, ui: &mut egui::Ui) {
        self.toolbar_rect = None;
        if self.mode.is_none()
            || matches!(
                self.mode,
                Some(Kind::LoadPoint | Kind::Column | Kind::PointRestraint | Kind::RigidLink)
            )
        {
            return;
        }
        let area = egui::Area::new(egui::Id::new("contour_tools"))
            .order(egui::Order::Foreground)
            .default_pos(ui.available_rect_before_wrap().left_top() + egui::vec2(10., 10.))
            .movable(true)
            .show(ui.ctx(), |ui| {
                ui.set_max_width(460.);
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.horizontal(|ui| {
                        for tool in Tool::ALL {
                            let enabled = self.can_choose_tool(tool);
                            let selected = self.tool == tool;
                            let clicked = ui
                                .add_enabled_ui(enabled, |ui| tool_icon(ui, tool, selected))
                                .inner;
                            if clicked {
                                self.triangulation_grid.active = false;
                                self.choose_tool(tool);
                            }
                        }
                        ui.separator();
                        if ui
                            .add_enabled(self.can_finish_sketch(), egui::Button::new(if matches!(self.mode,Some(Kind::LoadLine|Kind::Support|Kind::Auxiliary|Kind::Beam|Kind::Triangulation|Kind::Restraint)){"Завершить"}else{"Замкнуть"}))
                            .clicked()
                        {
                            self.finish();
                        }
                        if ui.button("Отмена").clicked() {
                            self.cancel();
                        }
                    });
                    if self.mode == Some(Kind::Triangulation) {
                        self.triangulation_grid_controls(ui);
                    }
                    if self.tool == Tool::Arc && !self.triangulation_grid.active {
                        let before = self.arc_end_radius;
                        ui.horizontal(|ui| {
                            ui.selectable_value(&mut self.arc_end_radius, false, "Начало–середина–конец");
                            ui.selectable_value(&mut self.arc_end_radius, true, "Начало–конец–радиус");
                        });
                        if before != self.arc_end_radius {
                            self.curve_controls.truncate(1);
                        }
                        if self.arc_end_radius {
                            ui.horizontal(|ui| {
                                ui.label("Радиус, мм");
                                ui.add(egui::DragValue::new(&mut self.arc_radius).range(0.001..=1e12));
                            });
                            ui.small("Укажите начало и конец, задайте радиус. Курсор выбирает сторону дуги; ЛКМ — подтвердить.");
                        }
                    }
                    self.reuse_controls(ui);
                    ui.checkbox(&mut self.repeat_sketch, "Не прерывать операцию");
                    if self.mode.is_some() {
                        ui.add(egui::Label::new(self.tool.hint()).wrap());
                        if self.tool==Tool::Line {ui.small("Длина: введите число в мм и Enter. Направление — курсором.");}
                        if self.tool==Tool::Rectangle {ui.small("b / h в мм: Tab — переключить, Enter — подтвердить; сторона — курсором.");}
                        if matches!(self.mode,Some(Kind::LoadLine|Kind::Support|Kind::Auxiliary|Kind::Beam|Kind::Triangulation|Kind::Restraint)){ui.small("Enter или двойной щелчок — закончить открытую линию; первая точка — замкнуть.");}
                    }
                });
            });
        self.toolbar_rect = Some(area.response.rect);
    }
    fn sketch_snapshot(&mut self) {
        if self.sketch_history.len() >= 128 {
            self.sketch_history.remove(0);
        }
        self.sketch_history.push(SketchState {
            line_closed: self.draft_line_closed,
            segments: self.draft_segments.clone(),
            tool: self.tool,
            draft: self.draft.clone(),
            controls: self.curve_controls.clone(),
        });
    }
    fn sketch_back(&mut self) {
        if let Some(s) = self.sketch_history.pop() {
            self.draft_line_closed = s.line_closed;
            self.tool = s.tool;
            self.draft_segments = s.segments;
            self.draft = s.draft;
            self.curve_controls = s.controls;
        } else if !self.curve_controls.is_empty() {
            self.curve_controls.pop();
        } else {
            self.draft.pop();
        }
    }
    fn choose_tool(&mut self, tool: Tool) {
        self.reuse_mode = None;
        self.reset_dynamic();
        if tool == self.tool {
            return;
        }
        if !self.can_choose_tool(tool) {
            self.status = "Эллипс и прямоугольник строятся отдельным объектом".into();
            return;
        }
        if tool == Tool::Rectangle {
            if let Some(first) = self.rectangle_seed() {
                self.sketch_snapshot();
                self.reset_dynamic();
                self.draft.clear();
                self.draft_segments.clear();
                self.curve_controls = vec![first];
                self.tool = tool;
                self.status =
                    "Первый угол задан. Укажите противоположный угол прямоугольника.".into();
                return;
            }
        }
        if self.tool == Tool::Spline && self.curve_controls.len() >= 2 {
            match sketch::spline(&self.curve_controls, false) {
                Ok(p) => {
                    self.sketch_snapshot();
                    self.draft_segments.push(Curve::Spline {
                        controls: self.curve_controls.clone(),
                        closed: false,
                    });
                    let skip = usize::from(!self.draft.is_empty());
                    self.draft.extend(p.into_iter().skip(skip));
                }
                Err(e) => {
                    self.status = e;
                    return;
                }
            }
        } else {
            self.sketch_snapshot();
        }
        if self.tool == Tool::Spline && self.curve_controls.len() == 1 && self.draft.is_empty() {
            self.draft.push(self.curve_controls[0]);
        }
        self.curve_controls.clear();
        self.tool = tool;
        self.status = tool.hint().into();
    }
    fn closing_spline(&self) -> (Vec<Point>, bool) {
        let mut controls = self.curve_controls.clone();
        let close_path = !matches!(
            self.mode,
            Some(
                Kind::LoadLine
                    | Kind::Support
                    | Kind::Auxiliary
                    | Kind::Beam
                    | Kind::Triangulation
                    | Kind::Restraint
            )
        ) || self.draft_line_closed;
        let closed = self.draft.is_empty() && close_path;
        if let Some(&first) = self.draft.first().filter(|_| close_path) {
            if controls.last().is_some_and(|p| *p != first) {
                controls.push(first);
            }
        }
        (controls, closed)
    }
    fn completed_sketch(&self) -> Result<Vec<Point>, String> {
        let mut points = self.draft.clone();
        match self.tool {
            Tool::Spline if !self.curve_controls.is_empty() => {
                let (controls, closed) = self.closing_spline();
                let p = sketch::spline(&controls, closed)?;
                let skip = usize::from(!points.is_empty());
                points.extend(p.into_iter().skip(skip));
            }
            Tool::Arc if self.curve_controls.len() > 1 => return Err("Укажите конец дуги".into()),
            Tool::Ellipse | Tool::Rectangle if !self.curve_controls.is_empty() => {
                return Err("Завершите построение фигуры".into())
            }
            _ => {}
        }
        if points.len() > 4096 {
            return Err("Объект превышает лимит 4096 вершин".into());
        }
        if points.len() > 1 && points[0] == *points.last().unwrap() {
            points.pop();
        }
        Ok(points)
    }
    fn completed_curves(&self) -> Vec<Curve> {
        let mut curves = self.draft_segments.clone();
        if self.tool == Tool::Spline && !self.curve_controls.is_empty() {
            let (controls, closed) = self.closing_spline();
            curves.push(Curve::Spline { controls, closed });
        }
        if curves.iter().all(|c| matches!(c, Curve::Line(..))) {
            curves.clear();
        }
        curves
    }
    fn sketch_click(&mut self, p: Point, closes: bool) {
        if self.foundation_point_target.is_some() {
            self.place_foundation_point(p);
            return;
        }

        if self.mode == Some(Kind::Triangulation) && self.triangulation_grid.active {
            match self.triangulation_grid.points(p) {
                Ok(points) => {
                    self.draft = points;
                    self.draft_segments.clear();
                    self.curve_controls.clear();
                    self.draft_line_closed = false;
                    self.tool = Tool::Line;
                    self.finish();
                }
                Err(error) => self.status = error,
            }
            return;
        }
        self.sketch_snapshot();
        if matches!(
            self.mode,
            Some(Kind::LoadPoint | Kind::Column | Kind::PointRestraint)
        ) {
            self.draft = vec![p];
            self.finish();
            return;
        }
        if self.mode == Some(Kind::RigidLink) {
            if self.draft.first() != Some(&p) {
                self.draft.push(p);
            }
            if self.draft.len() == 2 {
                self.finish();
            }
            return;
        }
        if closes
            && matches!(
                self.mode,
                Some(
                    Kind::LoadLine
                        | Kind::Support
                        | Kind::Auxiliary
                        | Kind::Beam
                        | Kind::Triangulation
                        | Kind::Restraint
                )
            )
        {
            self.draft_line_closed = true;
        }
        if closes
            && !(self.tool == Tool::Arc && (self.curve_controls.len() == 2 || self.arc_end_radius))
        {
            self.finish();
            return;
        }
        match self.tool {
            Tool::Line => {
                if self.draft.len() < 4096 && self.draft.last().is_none_or(|q| *q != p) {
                    if let Some(&a) = self.draft.last() {
                        self.draft_segments.push(Curve::Line(a, p));
                    }
                    self.draft.push(p);
                }
            }
            Tool::Spline => {
                if self.curve_controls.is_empty() {
                    if let Some(&last) = self.draft.last() {
                        self.curve_controls.push(last);
                    }
                }
                if self.curve_controls.last().is_none_or(|q| *q != p)
                    && self.curve_controls.len() < 512
                {
                    self.curve_controls.push(p);
                }
            }
            Tool::Arc => {
                if self.curve_controls.is_empty() {
                    if let Some(&last) = self.draft.last() {
                        self.curve_controls.push(last);
                    }
                }
                if self.curve_controls.is_empty() {
                    self.draft.push(p);
                    self.curve_controls.push(p);
                } else if self.curve_controls.len() == 1 {
                    if self.curve_controls[0] != p {
                        self.curve_controls.push(p);
                    }
                } else {
                    let points = if self.arc_end_radius {
                        sketch::arc_by_radius(
                            self.curve_controls[0],
                            self.curve_controls[1],
                            self.arc_radius,
                            p,
                        )
                    } else {
                        Ok([self.curve_controls[0], self.curve_controls[1], p])
                    };
                    let points = match points {
                        Ok(points) => points,
                        Err(e) => {
                            self.status = e;
                            return;
                        }
                    };
                    match sketch::arc(points[0], points[1], points[2]) {
                        Ok(arc) if self.draft.len() + arc.len() - 1 <= 4096 => {
                            let closes =
                                closes || self.draft.first().is_some_and(|q| *q == points[2]);
                            self.draft_segments.push(Curve::Arc(points));
                            self.draft.extend(arc.into_iter().skip(1));
                            self.curve_controls.clear();
                            if closes {
                                self.finish();
                            }
                        }
                        Ok(_) => self.status = "Объект превышает лимит 4096 вершин".into(),
                        Err(e) => self.status = e,
                    }
                }
            }
            Tool::Ellipse | Tool::Rectangle => {
                let required = if self.tool == Tool::Ellipse { 3 } else { 2 };
                if self.curve_controls.len() + 1 < required {
                    if self.tool == Tool::Rectangle {
                        self.reset_dynamic();
                    }
                    self.curve_controls.push(p);
                } else {
                    let result = if self.tool == Tool::Ellipse {
                        sketch::ellipse(self.curve_controls[0], self.curve_controls[1], p)
                    } else {
                        sketch::rectangle(self.curve_controls[0], p)
                    };
                    match result {
                        Ok(points) => {
                            if matches!(
                                self.mode,
                                Some(
                                    Kind::LoadLine
                                        | Kind::Support
                                        | Kind::Auxiliary
                                        | Kind::Beam
                                        | Kind::Triangulation
                                        | Kind::Restraint
                                )
                            ) {
                                self.draft_line_closed = true;
                            }
                            self.draft_segments = if self.tool == Tool::Ellipse {
                                vec![Curve::Ellipse([
                                    self.curve_controls[0],
                                    self.curve_controls[1],
                                    p,
                                ])]
                            } else {
                                vec![]
                            };
                            self.draft = points;
                            self.curve_controls.clear();
                            self.finish();
                        }
                        Err(e) => self.status = e,
                    }
                }
            }
        }
    }
    fn sketch_preview(&self, target: Option<Point>) -> Vec<Point> {
        let mut p = self.draft.clone();
        let mut controls = self.curve_controls.clone();
        if controls.is_empty() && matches!(self.tool, Tool::Arc | Tool::Spline) {
            if let Some(&last) = p.last() {
                controls.push(last);
            }
        }
        if let Some(q) = target {
            controls.push(q);
        }
        let preview = match self.tool {
            Tool::Line => {
                if let Some(q) = target {
                    p.push(q);
                }
                return p;
            }
            Tool::Spline => sketch::spline(&controls, false),
            Tool::Arc if controls.len() == 3 => {
                let points = if self.arc_end_radius {
                    sketch::arc_by_radius(controls[0], controls[1], self.arc_radius, controls[2])
                } else {
                    Ok([controls[0], controls[1], controls[2]])
                };
                points.and_then(|q| sketch::arc(q[0], q[1], q[2]))
            }
            Tool::Ellipse if controls.len() == 3 => {
                sketch::ellipse(controls[0], controls[1], controls[2])
            }
            Tool::Rectangle if controls.len() == 2 => sketch::rectangle(controls[0], controls[1]),
            _ => Ok(controls.clone()),
        };
        let mut q = preview.unwrap_or(controls);
        if matches!(self.tool, Tool::Ellipse | Tool::Rectangle) && q.len() >= 3 {
            q.push(q[0]);
        }
        let skip = usize::from(!p.is_empty());
        p.extend(q.into_iter().skip(skip));
        p
    }
    fn clear_mesh(&mut self) {
        if self.input_locked {
            return;
        }
        if let Some(cancel) = self.solver_cancel.take() {
            cancel.store(true, std::sync::atomic::Ordering::Relaxed);
        }
        #[cfg(target_arch = "wasm32")]
        {
            self.solver_worker = None;
        }
        self.invalidate_scene_buffers();
        self.analysis = None;
        self.alternative_cache = Default::default();
        self.alternative_job = None;
        self.combination_cache = None;
        self.analysis_job = None;
        self.analysis_input = None;
        self.fe_mesh = None;
        self.mesh_input.clear();
        // Dropping the receiver prevents a running worker from publishing its
        // result after Clear or overwriting a later triangulation.
        self.discard_mesh_job();
        self.source_view = None;
        self.foundation_view = None;
        self.restore_manual_locks();
        self.status = "КЭ-сетка удалена.".into();
    }
    fn discard_mesh_job(&mut self) {
        self.mesh_feedback = None;
        self.mesh_job = None;
    }
    fn operation_has_selection(&self) -> bool {
        self.selected.is_some() || !self.selection_ids.is_empty()
    }
    fn operation_target(&self, id: u64) -> bool {
        !self.operation_has_selection() || self.is_selected(id)
    }
    fn operation_sources(&self) -> Vec<Contour> {
        if !self.operation_has_selection() {
            return self.contours.clone();
        }
        let link_group = self
            .contours
            .iter()
            .filter_map(|c| {
                let targets = c.placement.rigid_link_targets?;
                (self.is_selected(c.id) || targets.iter().all(|&id| self.is_selected(id)))
                    .then_some([c.id, targets[0], targets[1]])
            })
            .flatten()
            .collect::<std::collections::BTreeSet<_>>();
        self.contours
            .iter()
            .filter(|c| {
                self.operation_target(c.id)
                    || link_group.contains(&c.id)
                    || (c.kind == Kind::Hole && c.parent.is_some_and(|id| self.is_selected(id)))
            })
            .cloned()
            .collect()
    }
    fn start_mesh(&mut self, ctx: &egui::Context) {
        self.sync_slab_cuts();
        if self.input_locked {
            return;
        }
        self.alternative_cache = Default::default();
        self.alternative_job = None;
        self.analysis = None;
        self.combination_cache = None;
        self.analysis_job = None;
        self.analysis_input = None;
        if self.mesh_job.is_some() {
            return;
        }
        self.sync_building_objects();
        let all = self.contours.clone();
        let scoped = self.operation_sources();
        self.mesh_partial = false;
        if all.len() != scoped.len() {
            let ids = scoped
                .iter()
                .map(|c| c.id)
                .collect::<std::collections::BTreeSet<_>>();
            self.mesh_partial = all
                .iter()
                .any(|c| c.is_mesh_input() && !ids.contains(&c.id));
        }
        let mut layout = self.building_layout();
        self.sync_stiffness_sources();
        let prepared =
            crate::plan_trace::columns::meshing_sources(&scoped, &self.support_height, &mut layout);
        let physical_contacts = if self.intersections.current {
            self.intersections
                .graph
                .lines
                .iter()
                .map(|l| l.ends)
                .chain(
                    self.intersections
                        .graph
                        .points
                        .iter()
                        .map(|(p, _)| [*p, *p]),
                )
                .collect::<Vec<_>>()
        } else {
            vec![]
        };
        let column_settings = self.support_height.clone();
        self.mesh_column_signature = self.column_signature();
        let size = self.mesh_size;
        let gravity = self.gravity;
        let options = self.mesh_options;
        let gmsh_exe = self.gmsh_exe.clone();
        if options.backend == super::meshing::Backend::Gmsh && !super::mesh_backend::available() {
            self.status = "Gmsh доступен только в EXE; в web выберите Spade".into();
            return;
        }
        if options.backend == super::meshing::Backend::Gmsh
            && !std::path::Path::new(&gmsh_exe).is_file()
        {
            self.status = "Укажите путь к gmsh.exe в настройках МКЭ-сетки".into();
            return;
        }
        self.mesh_requested_options = options;
        let (tx, rx) = mpsc::channel();
        self.mesh_job = Some(rx);
        self.mesh_feedback =
            (options.backend == super::meshing::Backend::Gmsh).then(MeshFeedback::default);
        let feedback = self
            .mesh_feedback
            .as_ref()
            .map(|f| (f.cancel.clone(), f.stage.clone()));
        self.status = format!("Построение МКЭ-сетки ({})…", options.backend.label());
        let ctx = ctx.clone();
        crate::plan_trace::platform::spawn(move || {
            use crate::plan_trace::fem::progress;
            let tracked = feedback.is_some();
            if let Some((cancel, stage)) = feedback {
                progress::install_cancel(cancel);
                let ctx = ctx.clone();
                progress::install(move |label, _| {
                    *stage.lock().unwrap() = label;
                    ctx.request_repaint();
                });
                progress::stage("Подготовка геометрии");
            }
            let result = std::panic::catch_unwind(|| {
                super::mesh_backend::configured(gmsh_exe, options.merge, options.optimise, || {
                    if let Some(layout) = layout {
                        if physical_contacts.is_empty() {
                            super::building_mesh::generate(&prepared, size, layout, options)
                        } else {
                            super::building_mesh::generate_with_intersections(
                                &prepared,
                                size,
                                layout,
                                options,
                                &physical_contacts,
                            )
                        }
                    } else {
                        super::meshing::generate_backend(&prepared, size, options.backend)
                            .and_then(|mesh| super::quad_mesh::process(mesh, options))
                            .and_then(|mut mesh| {
                                super::mesh_pack::pack(&mut mesh)?;
                                Ok(mesh)
                            })
                    }
                })
            })
            .unwrap_or_else(|_| Err("Ошибка генератора: проверьте геометрию объектов".into()));
            let result = result.and_then(|mut mesh| {
                progress::check_cancel()?;
                if tracked {
                    progress::stage("Подготовка сетки для просмотра");
                }
                crate::plan_trace::columns::tag_mesh(&mut mesh, &scoped, &column_settings);
                let foundation_view = source_views::FoundationView::prepare(&mesh, &all, gravity);
                Ok((mesh, foundation_view))
            });
            if tracked {
                progress::clear();
            }
            let _ = tx.send((all, size, result));
            ctx.request_repaint();
        });
    }
    fn poll_mesh(&mut self) {
        self.sync_slab_cuts();
        if self.fe_mesh.is_some()
            && (!model::same_mesh_input(&self.mesh_input, &self.contours)
                || self.mesh_built_size != self.mesh_size
                || !self.column_signature_current())
        {
            self.fe_mesh = None;
            if !self.input_locked {
                self.restore_manual_locks();
            }
            self.status = "Геометрия или размер изменены. Перестройте МКЭ-сетку".into();
        }
        if self.fe_mesh.is_none() {
            self.foundation_view = None;
        }
        let Some(rx) = &self.mesh_job else {
            return;
        };
        match rx.try_recv() {
            Ok((all, size, result)) => {
                let cancelled = self.mesh_feedback.as_ref().is_some_and(|feedback| {
                    feedback.cancel.load(std::sync::atomic::Ordering::Relaxed)
                });
                self.discard_mesh_job();
                if cancelled {
                    self.status = "Триангуляция остановлена пользователем".into();
                    return;
                }
                if !model::same_mesh_input(&all, &self.contours)
                    || size != self.mesh_size
                    || self.mesh_requested_options != self.mesh_options
                    || !self.column_signature_current()
                {
                    self.status =
                        "Параметры изменены во время построения. Перестройте МКЭ-сетку".into();
                    return;
                }
                match result {
                    Ok((mesh, foundation_view)) => {
                        self.status = format!(
                            "МКЭ-сетка: {} узлов, {} КЭ",
                            mesh.nodes.len(),
                            mesh.elements.len() - mesh.quads.len()
                        );
                        self.mesh_generation = self.mesh_generation.wrapping_add(1);
                        self.fe_snap_index = Index::mesh_floor(
                            &mesh,
                            self.building.enabled.then_some(self.building.active),
                        );
                        self.fe_mesh = Some(mesh);
                        self.foundation_view =
                            foundation_view.map(|view| (self.mesh_generation, view));
                        self.sync_mesh_object_owners();
                        self.mesh_input = all;
                        self.mesh_built_size = size;
                        self.remember_manual_locks();
                        self.set_contour_lock(true);
                    }
                    Err(e) => {
                        self.fe_mesh = None;
                        self.foundation_view = None;
                        self.status = e;
                    }
                }
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.discard_mesh_job();
                self.status = "Построение сетки прервано".into();
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }
    fn sync_mesh_object_owners(&mut self) {
        let Some(data) = self
            .fe_mesh
            .as_mut()
            .and_then(|mesh| mesh.building.as_mut())
        else {
            return;
        };
        for contour in &self.contours {
            let floor = self.building.owner(contour.id);
            if contour.kind == Kind::Foundation && data.layout.owner(contour.id) != floor {
                self.foundation_view = None;
            }
            data.layout.owners.insert(contour.id, floor);
        }
    }
    fn mesh_controls(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        egui::CollapsingHeader::new("Триангуляция")
            .default_open(true)
            .show(ui, |ui| self.triangulation_controls(ui));
        ui.horizontal_wrapped(|ui| {
        if self.fe_mesh.is_some() && ui.add_enabled(self.mesh_job.is_none()&&self.analysis_job.is_none()&&self.alternative_job.is_none()&&self.lira_export_job.is_none(),egui::Button::new("Экспорт ЛИРА *.TXT"))
            .on_hover_text("Узлы, КЭ42/44, стержни10, жёсткости, закрепления, АЖТ и загружения. Распределённые нагрузки — по площади КЭ, собственный вес балок — по длине, линейные — узловыми вертикальными силами. Единицы: м, тс; сечения см. РСН и результаты не экспортируются.").clicked() {
            self.export_lira_txt(ui.ctx());
        }
            ui.add_enabled_ui(self.contours.iter().any(|c|matches!(c.kind,Kind::Slab|Kind::Support|Kind::Beam|Kind::Column)), |ui| {
                ui.menu_button("Экспорт в Plaxis 3D", |ui| {
                    if ui.button("Команды *.TXT").on_hover_text("Вставить в Commands Plaxis. Плиты с отверстиями разбиваются на односвязные поверхности.").clicked() {
                        ui.close_menu(); self.export_plaxis_txt();
                    }
                    if ui.button("Python-скрипт *.PY").on_hover_text("Запустить через PLAXIS Remote Scripting. Отверстия создаются пересечением поверхностей; результат проверяется перед удалением внутренних частей.").clicked() {
                        ui.close_menu(); self.export_plaxis_python();
                    }
                }).response.on_hover_text("Геометрия, жёсткости и нагрузки выбранного загружения или сочетания. Единицы Plaxis: м, кН.");
            });
        });
        self.lira_export_progress(ui);
    }
    fn triangulation_controls(&mut self, ui: &mut egui::Ui) {
        let enabled = self.arc_placement.is_none()
            && self.scaling_op.is_none()
            && self.mesh_job.is_none()
            && self.drag.is_none()
            && self.mode.is_none()
            && self.contours.iter().any(|c| c.kind == Kind::Slab);
        ui.horizontal_wrapped(|ui| self.intersection_controls(ui));
        ui.add_enabled_ui(!self.input_locked, |ui| {
            let intersection_size = self.mesh_size;
            let previous = self.mesh_options;
            ui.horizontal_wrapped(|ui| {
                ui.label("Размер КЭ, мм");
                ui.add(
                    egui::DragValue::new(&mut self.mesh_size)
                        .range(100.0..=1_000_000.)
                        .speed(10.),
                );
            ui.add_enabled_ui(self.mesh_job.is_none() && !self.input_locked, |ui| {
                egui::ComboBox::from_id_salt("mesh_backend").selected_text(self.mesh_options.backend.label()).show_ui(ui, |ui| {
                    ui.selectable_value(&mut self.mesh_options.backend, super::meshing::Backend::Spade, "Spade");
                    ui.add_enabled_ui(super::mesh_backend::available(), |ui| {
                        ui.selectable_value(&mut self.mesh_options.backend, super::meshing::Backend::Gmsh, "Gmsh")
                            .on_hover_text("Нативный Gmsh. Использует общие узлы и линии пересечений. В web недоступен.");
                    });
                }).response.on_hover_text("Генератор сетки. Для Gmsh укажите установленный gmsh.exe.");
            });
            });
            ui.horizontal_wrapped(|ui| {
                if ui
                    .add_enabled(enabled && self.intersections.current && intersection_size == self.mesh_size, egui::Button::new("Триангуляция"))
                    .clicked()
                {
                    self.start_mesh(ui.ctx());
                }
                if ui
                    .add_enabled(
                    self.fe_mesh.is_some() || (self.mesh_job.is_some() && self.mesh_feedback.is_none()),
                        egui::Button::new("Очистить"),
                    )
                    .clicked()
                {
                    self.clear_mesh();
                }
            });
            if self.mesh_options.backend==super::meshing::Backend::Gmsh {
                ui.horizontal_wrapped(|ui|{
                    let r=ui.add(egui::TextEdit::singleline(&mut self.gmsh_exe).desired_width(200.).hint_text("Путь к gmsh.exe"));
                    if r.lost_focus(){super::mesh_backend::save_path(&self.gmsh_exe);}
                    #[cfg(not(target_arch="wasm32"))]
                    if ui.button("Обзор…").clicked(){if let Some(path)=rfd::FileDialog::new().add_filter("Gmsh",&["exe"]).pick_file(){self.gmsh_exe=path.to_string_lossy().into();super::mesh_backend::save_path(&self.gmsh_exe);}}
                });
            }
            ui.add_enabled(self.mesh_job.is_none(), egui::Checkbox::new(&mut self.mesh_options.merge, "Объединять в четырёхугольники"))
                .on_hover_text("Настоящие четырёхузловые MITC4: собственные функции формы и матрица 12×12. Не пересекают объекты. Требуется режим DKT / MITC4.");
            ui.add_enabled(self.mesh_options.merge && self.mesh_job.is_none(), egui::Checkbox::new(&mut self.mesh_options.optimise, "Оптимизировать четырёхугольники"))
                .on_hover_text("Улучшать форму перемещением внутренних узлов; узлы объектов остаются неподвижными.");
            if !self.mesh_options.merge { self.mesh_options.optimise = false; }
            if previous != self.mesh_options { self.clear_mesh(); }
        });
        if let Some(feedback) = &self.mesh_feedback {
            use std::sync::atomic::Ordering;
            ui.horizontal_wrapped(|ui| {
                ui.spinner();
                let stopping = feedback.cancel.load(Ordering::Relaxed);
                let label = feedback.stage.lock().unwrap().clone();
                ui.label(if stopping {
                    "Остановка…"
                } else {
                    &label
                });
                if ui
                    .add_enabled(!stopping, egui::Button::new("Остановить триангуляцию"))
                    .clicked()
                {
                    feedback.cancel.store(true, Ordering::Relaxed);
                }
            });
        } else if self.mesh_job.is_some() {
            ui.spinner();
        }
    }
    fn contour_icon(ui: &egui::Ui, kind: Kind) -> egui::Button<'static> {
        let index = match kind {
            Kind::Slab => 0,
            Kind::Hole => 1,
            Kind::Support => 2,
            Kind::Beam => 3,
            Kind::Column => 4,
            Kind::Auxiliary => 5,
            Kind::Triangulation => 6,
            Kind::Punching => 7,
            Kind::MeshRegion => 8,
            Kind::Restraint => 9,
            _ => return egui::Button::new(kind.label()),
        };
        const TILE_BYTES: usize = 64 * 64 * 4;
        const ICONS: &[u8; TILE_BYTES * 10] = include_bytes!("contour_icons.rgba");
        let id = egui::Id::new(("contour_icon", index));
        let ctx = ui.ctx();
        let texture = ctx
            .data(|data| data.get_temp::<egui::TextureHandle>(id))
            .unwrap_or_else(|| {
                let pixels = &ICONS[index * TILE_BYTES..(index + 1) * TILE_BYTES];
                let texture = ctx.load_texture(
                    kind.label(),
                    egui::ColorImage::from_rgba_unmultiplied([64, 64], pixels),
                    egui::TextureOptions::LINEAR,
                );
                ctx.data_mut(|data| data.insert_temp(id, texture.clone()));
                texture
            });
        egui::Button::image((texture.id(), egui::vec2(26., 26.)))
    }

    fn region_controls(&mut self, ui: &mut egui::Ui) {
        let slabs = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .map(|c| c.id)
            .collect::<Vec<_>>();
        let enabled = !slabs.is_empty()
            && !self.is_kind_locked(Kind::MeshRegion)
            && self.mesh_job.is_none()
            && self.arc_placement.is_none()
            && self.drag.is_none();
        let disabled_reason = if self.input_locked {
            "Выключите «Блокировать исходные данные» на нижней панели"
        } else if self.is_kind_locked(Kind::MeshRegion) {
            "Снимите «Блокировать плиты» в правой панели"
        } else if slabs.is_empty() {
            "Сначала добавьте плиту"
        } else if self.mesh_job.is_some() {
            "Дождитесь завершения триангуляции"
        } else {
            "Завершите текущее редактирование"
        };
        let button =
            Self::contour_icon(ui, Kind::MeshRegion).selected(self.mode == Some(Kind::MeshRegion));
        if ui
            .add_enabled(enabled, button)
            .on_hover_text(
                "Область триангуляции\nЗадаёт отдельный размер КЭ внутри выбранной плиты.",
            )
            .on_disabled_hover_text(format!("Область триангуляции\n{disabled_reason}"))
            .clicked()
        {
            if self.selected_slab().is_some() {
                self.begin(Kind::MeshRegion);
            } else if slabs.len() == 1 {
                self.cancel();
                self.select_only(Some(slabs[0]));
                self.begin(Kind::MeshRegion);
            } else {
                self.region_parent_choice = true;
            }
        }
        if !enabled {
            self.region_parent_choice = false;
        }
        if self.region_parent_choice {
            let mut open = true;
            let mut parent = None;
            egui::Window::new("Плита для области КЭ")
                .id(egui::Id::new("mesh_region_parent"))
                .collapsible(false)
                .resizable(false)
                .open(&mut open)
                .show(ui.ctx(), |ui| {
                    ui.label("Выберите плиту, внутри которой будет построена область:");
                    for &id in &slabs {
                        if ui.button(format!("Плита {id}")).clicked() {
                            parent = Some(id);
                        }
                    }
                });
            if !open {
                self.region_parent_choice = false;
            }
            if let Some(id) = parent {
                self.cancel();
                self.select_only(Some(id));
                self.begin(Kind::MeshRegion);
            }
        }
    }
    fn region_list_controls(&mut self, ui: &mut egui::Ui) {
        if !self.contours.iter().any(|c| c.kind == Kind::MeshRegion) {
            return;
        }
        let locked = self.is_kind_locked(Kind::MeshRegion);
        let floor_list = self.floor_list;
        let building = self.building.clone();
        let mut select = None;
        let mut selection = None;
        let mut remove = None;
        egui::Grid::new("mesh_region_table")
            .striped(true)
            .spacing([5., 4.])
            .show(ui, |ui| {
                ui.label("Имя");
                ui.label("Размер КЭ, мм");
                ui.label("");
                ui.end_row();
                for c in self.contours.iter_mut().filter(|c| {
                    c.kind == Kind::MeshRegion
                        && floor_list.is_none_or(|id| building.owner(c.id) == id)
                }) {
                    let id = c.id;
                    if let Some(region) = &mut c.refinement {
                        let name =
                            ui.add_sized([110., 22.], egui::TextEdit::singleline(&mut region.name));
                        if name.clicked() {
                            select = Some(id);
                        }
                        project_ui::contour_tree_menu(
                            &name,
                            1,
                            std::iter::once(id),
                            &mut selection,
                        );
                        ui.add(
                            egui::DragValue::new(&mut region.size)
                                .range(100.0..=1_000_000.)
                                .speed(10.),
                        );
                        if ui
                            .add_enabled(!locked, egui::Button::new("−"))
                            .on_hover_text("Удалить область")
                            .clicked()
                        {
                            remove = Some(id);
                        }
                        ui.end_row();
                    }
                }
            });
        if let Some(ids) = selection {
            self.select_contour_tree(ids);
        } else if let Some(id) = select {
            if self.building.enabled {
                self.change_floor(self.building.owner(id));
            }
            self.cancel();
            self.select_only(Some(id));
        }
        if let Some(id) = remove {
            self.remove_region(id);
        }
    }
    fn remove_region(&mut self, id: u64) {
        let Some(region) = self
            .contours
            .iter()
            .find(|c| c.id == id && c.kind == Kind::MeshRegion)
        else {
            return;
        };
        if self.is_kind_locked(region.kind) {
            self.status =
                "Область заблокирована: снимите блокировку плит и исходных данных.".into();
            return;
        }
        // An explicit table action must not depend on the viewport selection filter.
        self.push_contour_removal_undo(true);
        self.contours.retain(|c| c.id != id);
        self.prune_selection();
        self.slab_cache = None;
        self.status = "Отдельная область триангуляции удалена.".into();
    }
    fn rebuild_index(&mut self) {
        self.index = if self.show_dxf {
            self.drawing
                .as_ref()
                .map(|d| {
                    Index::visible(d, |name| {
                        self.floor_layer_visible(name)
                            && self.layers.get(name).is_none_or(|l| l.visible)
                    })
                })
                .unwrap_or_default()
        } else {
            Index::default()
        };
    }
    fn loaded(&mut self, d: Drawing) {
        self.layers = d
            .layers
            .iter()
            .map(|l| {
                (
                    l.name.clone(),
                    LayerStyle {
                        visible: l.visible,
                        color: None,
                        original: l.color.rgb(),
                        width: 1.,
                    },
                )
            })
            .collect();
        self.status = format!(
            "{} · объектов {}, текстов {} · Z=0",
            d.name,
            d.paths.len(),
            d.labels.len()
        );
        self.drawing = Some(d);
        self.rebuild_index();
        self.fit = true;
    }
    fn poll_load(&mut self) {
        match self.load.as_ref().map(|rx| rx.try_recv()) {
            Some(Ok(r)) => {
                self.load = None;
                match r {
                    Ok(Some(d)) => self.append_dxf(d),
                    Ok(None) => self.status = "Выбор файла отменён".into(),
                    Err(e) => self.status = format!("DXF: {e}"),
                }
            }
            Some(Err(mpsc::TryRecvError::Disconnected)) => {
                self.load = None;
                self.status = "Импорт прерван".into();
            }
            _ => {}
        }
    }
    fn open(&mut self, ctx: &egui::Context) {
        if self.load.is_some() || self.pdf_busy() {
            return;
        }
        let (tx, rx) = mpsc::channel();
        self.load = Some(rx);
        self.status = "Выбор DXF…".into();
        let ctx = ctx.clone();
        crate::plan_trace::platform::spawn(move || {
            let result = (|| {
                let Some(path) = rfd::FileDialog::new()
                    .add_filter("DXF", &["dxf"])
                    .pick_file()
                else {
                    return Ok(None);
                };
                import::read(&path).map(Some)
            })();
            let _ = tx.send(result);
            ctx.request_repaint();
        });
    }
    fn set_units(&mut self, unit: f64) {
        if !self.underlays.is_empty() {
            return;
        }
        if self.editing_blocked() {
            return;
        }
        self.arc_placement = None;
        let Some(d) = &mut self.drawing else {
            return;
        };
        let factor = unit / d.unit_mm;
        let scale = |p: &mut Point| {
            p[0] *= factor;
            p[1] *= factor;
        };
        for p in d.paths.iter_mut().flat_map(|p| &mut p.points) {
            scale(p);
        }
        for (t, _, _) in &mut d.fills {
            for p in t.iter_mut().flatten() {
                scale(p);
            }
        }
        for p in &mut d.nodes {
            scale(p);
        }
        for l in &mut d.labels {
            scale(&mut l.point);
            l.height *= factor;
        }
        for c in &mut self.contours {
            for curve in &mut c.curves {
                curve.map(|p| scale(p));
            }
            for p in &mut c.points {
                scale(p);
            }
        }
        for snapshot in &mut self.history {
            for c in snapshot {
                for curve in &mut c.curves {
                    curve.map(|p| scale(p));
                }
                for p in &mut c.points {
                    scale(p);
                }
            }
        }
        for p in &mut self.draft {
            scale(p);
        }
        for p in &mut self.curve_controls {
            scale(p);
        }
        for curve in &mut self.draft_segments {
            curve.map(|p| scale(p));
        }
        for s in &mut self.sketch_history {
            for curve in &mut s.segments {
                curve.map(|p| scale(p));
            }
            for p in s.draft.iter_mut().chain(&mut s.controls) {
                scale(p);
            }
        }
        d.unit_mm = unit;
        self.rebuild_index();
        self.slab_cache = None;
        self.fit = true;
    }
    fn selected_load_area(&self) -> Option<u64> {
        self.contours
            .iter()
            .find(|c| Some(c.id) == self.selected)
            .and_then(|c| {
                if c.kind == Kind::LoadArea {
                    Some(c.id)
                } else if c.kind == Kind::LoadHole {
                    c.parent
                } else {
                    None
                }
            })
    }
    fn selected_slab(&self) -> Option<u64> {
        self.contours
            .iter()
            .find(|c| Some(c.id) == self.selected)
            .and_then(|c| {
                if c.kind == Kind::Slab {
                    Some(c.id)
                } else if matches!(c.kind, Kind::Hole | Kind::MeshRegion) {
                    c.parent
                } else {
                    None
                }
            })
    }
    fn begin(&mut self, kind: Kind) {
        self.new_load_moment = false;
        if kind == Kind::RigidLink && !self.building.enabled {
            self.status = "Для АЖТ включите режим здания".into();
            return;
        }
        self.dwell_polar_anchor = None;
        self.dwell_polar_candidate = None;
        self.reuse_mode = None;
        self.measurements.stop();
        self.clipboard_op = None;
        self.reset_dynamic();
        if self.kind_editing_blocked(kind) {
            return;
        }
        self.arc_placement = None;
        let parent = if kind == Kind::LoadHole {
            self.selected_load_area()
        } else if matches!(kind, Kind::Hole | Kind::MeshRegion) {
            self.selected_slab()
        } else {
            None
        };
        if matches!(kind, Kind::Hole | Kind::MeshRegion | Kind::LoadHole) && parent.is_none() {
            self.status = "Сначала выберите плиту для вложенного объекта".into();
            return;
        }
        self.mode = Some(kind);
        self.draft_load_case = if kind == Kind::LoadHole {
            self.contours
                .iter()
                .find(|c| Some(c.id) == parent)
                .and_then(|c| c.load.as_ref())
                .map(|l| l.case_id)
        } else {
            kind.is_load().then_some(self.active_load)
        };
        self.draft_line_closed = false;
        self.triangulation_grid.active = false;
        self.tool = Tool::Line;
        self.curve_controls.clear();
        self.sketch_history.clear();
        self.draft.clear();
        self.draft_segments.clear();
        self.draft_parent = parent;
        self.status = match kind {
            Kind::PointRestraint => "Укажите точку опоры.",
            Kind::Column => "Укажите центр сечения колонны.",
            Kind::RigidLink => "Укажите две точки в плане. Низ и верх АЖТ задаются в свойствах.",
            Kind::LoadPoint => "Укажите точку сосредоточенной нагрузки.",
            Kind::LoadLine
            | Kind::Support
            | Kind::Auxiliary
            | Kind::Beam
            | Kind::Triangulation
            | Kind::Restraint => "Укажите точки линии. Enter — закончить, первая точка — замкнуть.",
            _ => "Укажите вершины. Первая точка или Enter — замкнуть.",
        }
        .into();
    }
    fn finish(&mut self) {
        let Some(kind) = self.mode else {
            return;
        };
        if self.kind_editing_blocked(kind) {
            return;
        }
        let points = match self.completed_sketch() {
            Ok(p) => p,
            Err(e) => {
                self.status = e;
                return;
            }
        };
        let mut c = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: if kind == Kind::Triangulation && self.triangulation_grid.active {
                self.draft
                    .first()
                    .map(|&origin| self.triangulation_grid.spec(origin))
            } else {
                None
            },
            support_open: kind == Kind::RigidLink
                || matches!(
                    kind,
                    Kind::Support
                        | Kind::Auxiliary
                        | Kind::Beam
                        | Kind::Triangulation
                        | Kind::Restraint
                ) && !self.draft_line_closed,
            load: kind.is_load().then(|| super::loads::LoadData {
                direction: if kind == Kind::LoadHole {
                    Default::default()
                } else {
                    self.new_load_direction
                },
                moment: kind == Kind::LoadPoint && self.new_load_moment,
                additive: false,
                name: String::new(),
                mesh_constraint: false,
                case_id: self.draft_load_case.unwrap_or(self.active_load),
                magnitude: if kind == Kind::LoadHole {
                    super::loads::Magnitude {
                        value: 0.,
                        units: self.load_units,
                    }
                } else {
                    self.new_load
                },
                closed: matches!(kind, Kind::LoadArea | Kind::LoadHole) || self.draft_line_closed,
            }),
            refinement: if matches!(kind, Kind::MeshRegion | Kind::Punching) {
                Some(model::Refinement {
                    inactive_edges: vec![],

                    name: if kind == Kind::Punching {
                        format!("Область суммирования нагрузок {}", self.next_id)
                    } else {
                        format!("Область {}", self.next_region)
                    },
                    size: self.mesh_size,
                })
            } else {
                None
            },
            curves: self.completed_curves(),
            id: self.next_id,
            kind,
            parent: if kind == Kind::LoadHole {
                self.draft_parent.or_else(|| self.selected_load_area())
            } else if matches!(kind, Kind::Hole | Kind::MeshRegion) {
                self.draft_parent.or_else(|| self.selected_slab())
            } else {
                None
            },
            points,
        };
        if let Err(e) = c.rebuild() {
            self.status = e;
            return;
        }
        match model::validate_candidate(&self.contours, &c) {
            Ok(()) => {
                self.reuse_mode = None;
                self.history.push(self.contours.clone());
                self.next_id += 1;
                if kind == Kind::MeshRegion {
                    self.next_region += 1;
                }
                let id = c.id;
                self.contours.push(c);
                self.select_only(Some(id));
                self.reset_dynamic();
                self.draft.clear();
                self.draft_segments.clear();
                self.curve_controls.clear();
                self.sketch_history.clear();
                self.draft_line_closed = false;
                if !self.repeat_sketch {
                    self.mode = None;
                    self.draft_load_case = None;
                    self.draft_parent = None;
                }
                self.slab_cache = None;
                self.status = if self.repeat_sketch {
                    "Объект сохранён. Укажите первую точку следующего; Esc — закончить."
                } else {
                    "Объект сохранён. Вершины можно перетаскивать."
                }
                .into();
            }
            Err(e) => self.status = e,
        }
    }
    fn cancel(&mut self) {
        self.grid_pick_origin = false;
        self.section_pick_origin = None;
        self.cancel_foundation_interaction();
        self.foundation_point_target = None;
        self.diagnostic_points.clear();
        self.dwell_polar_anchor = None;
        self.dwell_polar_candidate = None;
        self.triangulation_grid.active = false;
        self.load_value_edit = None;
        self.region_parent_choice = false;
        self.reuse_mode = None;
        self.scaling_op = None;
        self.cancel_underlay_drag();
        self.cancel_measurement_drag();
        self.selected_underlay = None;
        self.measurements.stop();
        self.clipboard_op = None;
        self.selection_start = None;
        self.load_menu_target = None;
        self.support_menu_target = None;
        self.reset_dynamic();
        self.draft_load_case = None;
        self.draft_line_closed = false;
        self.arc_placement = None;
        self.arc_menu_target = None;
        if let Some(d) = self.drag.take() {
            self.contours = d.before;
            self.slab_cache = None;
            self.invalidate_scene_buffers();
        }
        self.draft.clear();
        self.draft_segments.clear();
        self.curve_controls.clear();
        self.sketch_history.clear();
        self.mode = None;
        self.draft_parent = None;
        self.select_only(None);
    }
    fn remove(&mut self) {
        self.prune_selection();
        let mut ids = self.selection_ids.clone();
        ids.extend(self.selected);
        let dimensions = self.measurements.selected.clone();
        let mut underlays = self.underlay_selection.clone();
        underlays.extend(self.selected_underlay);
        if ids.is_empty() && dimensions.is_empty() && underlays.is_empty() {
            return;
        }
        if ids.iter().any(|id| !self.contour_removable(*id)) {
            self.status = "Объект или связанное с ним отверстие заблокированы.".into();
            return;
        }
        if !underlays.is_empty() && (self.underlays_locked || self.input_locked) {
            self.status = "Подложки заблокированы".into();
            return;
        }
        self.measurements
            .items
            .retain(|m| !dimensions.contains(&m.id));
        self.measurements.selected.clear();
        if ids.is_empty() {
            if !underlays.is_empty() {
                self.remove_underlays(&underlays, false);
            } else {
                self.status = "Выделенные размеры удалены".into();
            }
            return;
        }
        self.arc_placement = None;
        self.select_only(None);
        let linked = self
            .contours
            .iter()
            .filter_map(|c| {
                c.placement
                    .rigid_link_targets
                    .filter(|targets| {
                        ids.contains(&c.id) || targets.iter().any(|id| ids.contains(id))
                    })
                    .map(|_| c.id)
            })
            .collect::<Vec<_>>();
        ids.extend(linked);
        let affects_mesh = self.contours.iter().any(|c| {
            (ids.contains(&c.id) || c.parent.is_some_and(|id| ids.contains(&id)))
                && c.kind.affects_calculation()
        });
        self.push_contour_removal_undo(affects_mesh);
        self.contours
            .retain(|c| !ids.contains(&c.id) && !c.parent.is_some_and(|id| ids.contains(&id)));
        self.slab_cache = None;
        self.invalidate_scene_buffers();
        if affects_mesh {
            self.clear_mesh();
        }
        self.status = "Объекты удалены. Ctrl+Z — отменить".into();
        self.remove_underlays(&underlays, true);
    }

    fn undo(&mut self) {
        if self.measurements.active {
            self.measurements.spatial_draft.pop();
            if self.measurements.second.take().is_none() {
                self.measurements.first = None;
            }
            return;
        }
        if self.scaling_op.is_some()
            || self.measurements.drag.is_some()
            || self.underlay_drag.is_some()
            || self.clipboard_op.is_some()
            || self.drag.is_some()
            || self.arc_placement.is_some()
        {
            self.cancel();
        } else if self.mode.is_some()
            && (!self.draft.is_empty()
                || !self.curve_controls.is_empty()
                || !self.sketch_history.is_empty())
        {
            self.sketch_back();
        } else if self.undo_scaling() {
        } else if self.undo_underlay() {
        } else {
            self.undo_contour_edit();
        }
    }
    fn handle_undo_shortcut(&mut self, ctx: &egui::Context) {
        let undo = ctx.input_mut(|input| {
            let mut found = false;
            input.events.retain(|event| {
                let shortcut = matches!(event, egui::Event::Key {
                    key: egui::Key::Z,
                    pressed: true,
                    modifiers,
                    ..
                } if (modifiers.ctrl || modifiers.command)
                    && !modifiers.alt && !modifiers.shift);
                found |= shortcut;
                !shortcut
            });
            found
        });
        if undo {
            self.undo();
        }
    }
    #[cfg(test)]
    fn press_undo_after_control_release(&mut self) {
        let ctx = egui::Context::default();
        let _ = ctx.run(
            egui::RawInput {
                modifiers: egui::Modifiers::NONE,
                events: vec![egui::Event::Key {
                    key: egui::Key::Z,
                    physical_key: Some(egui::Key::Z),
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::CTRL,
                }],
                ..Default::default()
            },
            |ctx| {
                assert!(!ctx.input(|input| input.modifiers.ctrl));
                self.handle_undo_shortcut(ctx);
            },
        );
    }
    fn undo_contour_edit(&mut self) -> bool {
        if let Some(before) = self.history.last() {
            if self.undo_changes_locked(before) || self.property_undo_locked(self.history.len() - 1)
            {
                self.status="Отмена затрагивает заблокированные объекты. Снимите соответствующую блокировку.".into();
                return false;
            }
            let property_change = self.property_undo(self.history.len() - 1);
            let previous = self.history.pop().unwrap();
            let without_edge_flags = |contours: &[Contour]| {
                contours
                    .iter()
                    .filter(|c| c.kind.affects_calculation())
                    .cloned()
                    .map(|mut c| {
                        c.placement.movement_locked = false;
                        if c.kind == Kind::Punching {
                            if let Some(r) = &mut c.refinement {
                                r.inactive_edges.clear();
                            }
                        }
                        c
                    })
                    .collect::<Vec<_>>()
            };
            let changed_inputs = property_change
                || without_edge_flags(&self.contours) != without_edge_flags(&previous);
            self.contours = previous;
            self.sync_building_objects();
            if property_change {
                self.sync_stiffness_sources();
            }
            self.select_only(None);
            self.slab_cache = None;
            self.invalidate_scene_buffers();
            if changed_inputs {
                self.clear_mesh();
            } else {
                self.refresh_summation_results();
            }
            self.status = "Редактирование объекта отменено".into();
            true
        } else {
            false
        }
    }
    fn change_vertex(&mut self, id: u64, index: usize, add: bool) {
        let Some(old) = self.contours.iter().find(|c| c.id == id).cloned() else {
            return;
        };
        if self.kind_editing_blocked(old.kind) || self.movement_blocked(id) {
            return;
        }
        self.arc_placement = None;
        let mut c = old.clone();
        if !(if add {
            c.can_insert(index)
        } else {
            c.can_remove(index)
        }) {
            self.status = "Эта управляющая точка не может быть добавлена/удалена".into();
            return;
        }
        if let Err(e) = c.change_handle(index, add) {
            self.status = e;
            return;
        }
        if c.kind == Kind::Foundation {
            c.foundation = Some(crate::plan_trace::foundation::remap_values(
                &old,
                &c,
                self.gravity,
                self.load_units,
            ));
        }
        let validation = model::validate_candidate(&self.contours, &c);
        let repairing = !add
            && old.curves.is_empty()
            && old.closed()
            && model::valid(&old.points)
                .is_err_and(|e| e == "Граница пересекает или касается сама себя")
            && validation
                .as_ref()
                .is_err_and(|e| e == "Граница пересекает или касается сама себя")
            && model::self_contacts(&c.points) <= model::self_contacts(&old.points);
        if validation.is_ok() || repairing {
            self.history.push(self.contours.clone());
            *self.contours.iter_mut().find(|v| v.id == id).unwrap() = c;
            self.clear_mesh();
            self.slab_cache = None;
            self.spatial.cache_reset();
            self.rebuild_index();
            self.status = if repairing {
                format!("Объект {id}: вершина удалена; ещё остались самокасания. Ctrl+Z — отменить")
            } else {
                format!("Объект {id}: вершина изменена. Ctrl+Z — отменить")
            };
        } else if let Err(e) = validation {
            self.status = format!("Объект {id}: {e}");
        }
    }

    fn bounds(&self) -> Option<[f64; 4]> {
        if self.fe_isolated() {
            return self.fe_fragment_bounds_2d();
        }
        let mut b = if self.show_dxf {
            self.drawing.as_ref().and_then(Drawing::bounds)
        } else {
            None
        };
        if self.show_dxf {
            for u in self
                .underlays
                .iter()
                .filter(|u| u.visible && u.transparency < 1.)
            {
                if matches!(u.data, crate::plan_trace::underlay::Data::Pdf { .. }) {
                    for p in u.corners().into_iter().flatten() {
                        let q = b.get_or_insert([p[0], p[1], p[0], p[1]]);
                        q[0] = q[0].min(p[0]);
                        q[1] = q[1].min(p[1]);
                        q[2] = q[2].max(p[0]);
                        q[3] = q[3].max(p[1]);
                    }
                }
            }
        }
        for p in self
            .contours
            .iter()
            .filter(|c| self.contour_visible(c))
            .flat_map(|c| &c.points)
            .chain(&self.draft)
            .chain(&self.curve_controls)
            .chain(
                self.measurements
                    .items
                    .iter()
                    .filter(|_| self.measurements.show)
                    .flat_map(|m| [&m.a, &m.b]),
            )
        {
            let q = b.get_or_insert([p[0], p[1], p[0], p[1]]);
            q[0] = q[0].min(p[0]);
            q[1] = q[1].min(p[1]);
            q[2] = q[2].max(p[0]);
            q[3] = q[3].max(p[1]);
        }
        for m in self
            .measurements
            .items
            .iter()
            .filter(|_| self.measurements.show)
        {
            if let Some(offset) = m.offset {
                let length = m.length();
                if length > 1e-6 && length.is_finite() {
                    let n = [-(m.b[1] - m.a[1]) / length, (m.b[0] - m.a[0]) / length];
                    for p in [m.a, m.b].map(|p| [p[0] + n[0] * offset, p[1] + n[1] * offset]) {
                        if p.iter().any(|v| !v.is_finite()) {
                            continue;
                        }
                        let q = b.get_or_insert([p[0], p[1], p[0], p[1]]);
                        q[0] = q[0].min(p[0]);
                        q[1] = q[1].min(p[1]);
                        q[2] = q[2].max(p[0]);
                        q[3] = q[3].max(p[1]);
                    }
                }
            }
        }
        b
    }
    fn contour_snap_edges(&self, exclude: Option<u64>) -> impl Iterator<Item = [Point; 2]> + '_ {
        self.contours
            .iter()
            .filter(move |c| {
                Some(c.id) != exclude && self.contour_visible(c) && self.on_view_floor(c)
            })
            .flat_map(|c| {
                let count = if c.closed() {
                    c.points.len()
                } else {
                    c.points.len().saturating_sub(1)
                };
                c.points
                    .iter()
                    .copied()
                    .zip(c.points.iter().copied().cycle().skip(1))
                    .take(count)
                    .map(|(a, b)| [a, b])
            })
    }
    fn snap(&self, p: Point, exclude: Option<u64>) -> Option<snap::Snap> {
        for anchor in self.polar_anchors() {
            if (anchor[0] - p[0]).hypot(anchor[1] - p[1]) * self.camera.zoom <= 10. {
                return Some(snap::Snap {
                    point: anchor,
                    kind: snap::Kind::Node,
                });
            }
        }
        let mut vertices = self
            .contours
            .iter()
            .filter(|c| Some(c.id) != exclude && self.contour_visible(c) && self.on_view_floor(c))
            .flat_map(|c| c.handles())
            .collect::<Vec<_>>();
        if self.snap_settings.geometry_nodes {
            for grid in self
                .contours
                .iter()
                .filter(|c| {
                    Some(c.id) != exclude && self.contour_visible(c) && self.on_view_floor(c)
                })
                .filter_map(|c| c.grid.as_ref())
            {
                vertices.push(grid.nearest_node(p));
            }
        }
        if exclude.is_none() {
            vertices.extend(&self.draft);
            vertices.extend(&self.curve_controls);
        }
        let mut edges = self.contour_snap_edges(exclude).collect::<Vec<_>>();
        for path in self.support_extent_paths(exclude) {
            vertices.extend(path.iter().copied());
            edges.extend(path.windows(2).map(|p| [p[0], p[1]]));
        }
        if self.snap_settings.geometry_nodes {
            let radius = 10. / self.camera.zoom;
            let mut nearby = edges
                .iter()
                .copied()
                .filter(|[a, b]| {
                    let d = [b[0] - a[0], b[1] - a[1]];
                    let l = d[0] * d[0] + d[1] * d[1];
                    if l < 1e-20 {
                        return false;
                    }
                    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.);
                    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]) <= radius
                })
                .collect::<Vec<_>>();
            for a in self.polar_anchors() {
                let length = (p[0] - a[0]).hypot(p[1] - a[1]) + radius * 2.;
                for i in 0..(360. / self.snap_settings.angle_step).round() as usize {
                    let angle = (i as f64 * self.snap_settings.angle_step).to_radians();
                    nearby.push([
                        a,
                        [a[0] + length * angle.cos(), a[1] + length * angle.sin()],
                    ]);
                }
            }
            for i in 0..nearby.len() {
                for j in i + 1..nearby.len() {
                    let [a, b] = nearby[i];
                    let [c, d] = nearby[j];
                    let u = [b[0] - a[0], b[1] - a[1]];
                    let v = [d[0] - c[0], d[1] - c[1]];
                    let det = u[0] * v[1] - u[1] * v[0];
                    if det.abs() < 1e-12 {
                        continue;
                    }
                    let w = [c[0] - a[0], c[1] - a[1]];
                    let t = (w[0] * v[1] - w[1] * v[0]) / det;
                    let q = (w[0] * u[1] - w[1] * u[0]) / det;
                    if (0. ..=1.).contains(&t) && (0. ..=1.).contains(&q) {
                        let hit = [a[0] + t * u[0], a[1] + t * u[1]];
                        if (hit[0] - p[0]).hypot(hit[1] - p[1]) <= radius {
                            vertices.push(hit);
                        }
                    }
                }
            }
        }
        if self.snap_settings.midpoint {
            vertices.extend(
                edges
                    .iter()
                    .map(|[a, b]| [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]),
            );
        }
        let modes = self.snap_settings;
        let geometry = self.index.query_modes(
            p,
            self.camera.zoom,
            &vertices,
            &edges,
            modes.geometry_nodes,
            modes.nearest,
        );
        let fe = if self.show_mesh && self.fe_mesh.is_some() {
            self.fe_snap_index.query_modes(
                p,
                self.camera.zoom,
                &[],
                &[],
                modes.geometry_nodes,
                modes.nearest,
            )
        } else {
            None
        };
        let geometry = [geometry, fe].into_iter().flatten().min_by(|a, b| {
            let priority = |s: &snap::Snap| if s.kind == snap::Kind::Node { 0 } else { 1 };
            priority(a).cmp(&priority(b)).then_with(|| {
                ((a.point[0] - p[0]).hypot(a.point[1] - p[1]))
                    .total_cmp(&((b.point[0] - p[0]).hypot(b.point[1] - p[1])))
            })
        });
        if geometry.is_some_and(|s| s.kind == snap::Kind::Node) {
            return geometry;
        }
        modes
            .grid_nodes
            .then(|| self.construction_grid.snap(p, self.camera.zoom))
            .flatten()
            .map(|point| snap::Snap {
                point,
                kind: snap::Kind::Node,
            })
            .or(geometry)
    }
    fn creation_ribbon(&mut self, ctx: &egui::Context) {
        egui::TopBottomPanel::top("fem_ribbon").show(ctx, |ui| {
            let _ = ui.selectable_label(true, "Создание");
            ui.separator();
            ui.horizontal_wrapped(|ui| {
                for kind in Kind::ALL {
                    let needs_parent = kind == Kind::Hole && self.selected_slab().is_none();
                    let enabled =
                        !self.is_kind_locked(kind) && self.load.is_none() && !needs_parent;
                    let button = if kind == Kind::Foundation {
                        self.foundation_button(ui, enabled)
                    } else {
                        let icon = Self::contour_icon(ui, kind).selected(self.mode == Some(kind));
                        ui.add_enabled(enabled, icon)
                    };
                    if button
                        .on_hover_text(kind.label())
                        .on_disabled_hover_text(if needs_parent {
                            "Отверстие: сначала выберите плиту"
                        } else {
                            kind.label()
                        })
                        .clicked()
                    {
                        self.begin(kind);
                    }
                }
                if self.building.enabled {
                    let support = Self::contour_icon(ui, Kind::Restraint)
                        .selected(self.mode == Some(Kind::Restraint));
                    if ui
                        .add_enabled(!self.is_kind_locked(Kind::Restraint), support)
                        .on_hover_text("Линейная опора")
                        .clicked()
                    {
                        self.begin(Kind::Restraint);
                    }
                    if self
                        .point_restraint_button(ui)
                        .on_hover_text("Точечная опора")
                        .clicked()
                    {
                        self.begin(Kind::PointRestraint);
                    }
                    if self
                        .rigid_link_button(ui)
                        .on_hover_text("АЖТ. Две точки в плане, низ и верх задаются в свойствах.")
                        .clicked()
                    {
                        self.begin(Kind::RigidLink);
                    }
                }
                self.region_controls(ui);
            });
            if let Some(kind) = self.mode {
                ui.horizontal_wrapped(|ui| {
                    ui.label(format!("Вершин: {}", self.draft.len()));
                    if let Some(parent) = self.draft_parent {
                        ui.small(format!("Отверстие в плите {parent}"));
                    }
                    let finish = if matches!(
                        kind,
                        Kind::LoadLine
                            | Kind::Support
                            | Kind::Auxiliary
                            | Kind::Beam
                            | Kind::Triangulation
                            | Kind::Restraint
                    ) {
                        "Завершить"
                    } else {
                        "Замкнуть"
                    };
                    if ui
                        .add_enabled(self.can_finish_sketch(), egui::Button::new(finish))
                        .clicked()
                    {
                        self.finish();
                    }
                    if ui.button("Отмена").clicked() {
                        self.cancel();
                    }
                });
            }
        });
    }

    fn point_restraint_button(&self, ui: &mut egui::Ui) -> egui::Response {
        let enabled = !self.is_kind_locked(Kind::PointRestraint);
        let response = ui.add_enabled(
            enabled,
            egui::Button::new("")
                .min_size(egui::vec2(42., 34.))
                .selected(self.mode == Some(Kind::PointRestraint)),
        );
        let at = |x, y| response.rect.center() + egui::vec2(x, y);
        let color = if enabled {
            Kind::PointRestraint.color()
        } else {
            ui.visuals().weak_text_color()
        };
        let stroke = Stroke::new(1.6, color);
        ui.painter().add(egui::Shape::closed_line(
            vec![at(0., -4.), at(-9., 8.), at(9., 8.)],
            stroke,
        ));
        ui.painter()
            .circle_filled(at(0., -6.), 2.6, Color32::LIGHT_BLUE);
        ui.painter()
            .line_segment([at(-13., 11.), at(13., 11.)], stroke);
        response
    }

    fn rigid_link_button(&self, ui: &mut egui::Ui) -> egui::Response {
        let enabled = !self.is_kind_locked(Kind::RigidLink);
        let response = ui.add_enabled(
            enabled,
            egui::Button::new("")
                .min_size(egui::vec2(42., 34.))
                .selected(self.mode == Some(Kind::RigidLink)),
        );
        let painter = ui.painter().with_clip_rect(response.rect);
        let at = |x, y| response.rect.center() + egui::vec2(x, y);
        let corners = [at(-16., 9.), at(-4., 2.), at(16., 2.), at(5., 12.)];
        let joint = at(0., 7.);
        let weak = ui.visuals().weak_text_color();
        let orange = if enabled {
            Color32::from_rgb(255, 133, 38)
        } else {
            weak
        };
        let blue = if enabled {
            Color32::from_rgb(71, 158, 255)
        } else {
            weak
        };
        let outline = egui::Stroke::new(1.6, orange);
        painter.add(egui::Shape::closed_line(corners.to_vec(), outline));
        for corner in corners {
            painter.line_segment([corner, joint], outline);
        }
        painter.line_segment([joint, at(0., -12.)], egui::Stroke::new(2., blue));
        painter.circle_filled(joint, 1.8, orange);
        response
    }

    fn sidebar(&mut self, ctx: &egui::Context) {
        let compact = compact_sidebars(ctx);
        let panel = egui::SidePanel::left(if compact {
            "trace_controls_compact"
        } else {
            "trace_controls"
        });
        let panel = if self.left_open {
            let sidebar_space = ctx.available_rect().width() - reserved_canvas_width(ctx);
            let max_width = match (compact, self.right_open) {
                (true, true) => sidebar_space / 2.,
                (false, true) => sidebar_space - 340.,
                (_, false) => sidebar_space - 36.,
            };
            panel
                .default_width(325.)
                .min_width(280.)
                .max_width(max_width.max(0.))
        } else {
            panel.exact_width(36.).resizable(false)
        };
        panel.show(ctx,|ui|{
  if ui.button(if self.left_open { "<" } else { ">" }).on_hover_text(if self.left_open { "Свернуть левую панель" } else { "Раскрыть левую панель" }).clicked() { self.left_open = !self.left_open; }
  if !self.left_open { return; }
  ui.separator();
  egui::ScrollArea::new([compact, true]).show(ui,|ui|{
   self.project_controls(ui);
   egui::CollapsingHeader::new("Импорт").default_open(false).show(ui, |ui| {
       ui.horizontal_wrapped(|ui| {
           if ui.add_enabled(self.load.is_none() && !self.pdf_busy(), egui::Button::new("Импорт *.DXF")).clicked() { self.open(ctx); }
           self.underlay_controls(ui);
           self.lir_import_button(ui);
       });
       if self.load.is_some() { ui.spinner(); }
   });
   if let Some(d)=&self.drawing{ui.add(egui::Label::new(&d.name).wrap());if !d.skipped.is_empty(){egui::CollapsingHeader::new("Диагностика импорта").show(ui,|ui|{for(k,v)in &d.skipped{ui.add(egui::Label::new(format!("{k}: {v}")).wrap());}});}}
   if self.underlays.is_empty(){ if let Some(d)=&self.drawing{let old=d.unit_mm;let mut unit=old;egui::ComboBox::from_label("Единицы DXF").selected_text(match old{1.=>"мм".into(),10.=>"см".into(),1000.=>"м".into(),25.4=>"дюймы".into(),304.8=>"футы".into(),_=>format!("{old} мм/ед.")}).show_ui(ui,|ui|{for(label,value)in[("мм",1.),("см",10.),("м",1000.),("дюймы",25.4),("футы",304.8)]{ui.selectable_value(&mut unit,value,label);}});if unit!=old{self.set_units(unit);}} }
   ui.separator();
   self.floor_controls(ui);self.contour_list_controls(ui);self.schema_analysis_controls(ui);
   self.stiffness_controls(ui);self.mesh_controls(ui);self.analysis_controls(ui);self.results_controls(ui);ui.separator();self.status_message(ui);ui.separator();ui.small("ЛКМ — выбор / вершина. Маркеры выделенного объекта — перетаскивание. ПКМ у вершины/ребра — добавить или удалить. Enter — замкнуть, Esc — отменить, Ctrl+Z — отмена. Колесо — масштаб, средняя кнопка — панорама. Вращение ПКМ доступно только в 3D.");self.calculation_settings(ui);self.technical_information(ui);
  });
 });
    }
    fn canvas(&mut self, ui: &mut egui::Ui) {
        if self.spatial.enabled {
            self.spatial_canvas(ui);
            return;
        }
        self.camera.angle = 0.;
        self.prune_selection();
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        if self.snap_settings.polar
            && (self.mode.is_some() || self.selected.is_some() || self.drag.is_some())
        {
            let candidate = ui
                .input(|i| i.pointer.hover_pos())
                .filter(|p| rect.contains(*p))
                .and_then(|p| {
                    let edited = self.drag.as_ref().map(|d| &d.original).or_else(|| {
                        self.selected
                            .and_then(|id| self.contours.iter().find(|c| c.id == id))
                    });
                    let own = edited
                        .into_iter()
                        .flat_map(|c| c.handles())
                        .filter(|&q| self.camera.screen(q, rect).distance(p) <= 9.)
                        .min_by(|a, b| {
                            self.camera
                                .screen(*a, rect)
                                .distance(p)
                                .total_cmp(&self.camera.screen(*b, rect).distance(p))
                        });
                    own.map(|point| snap::Snap {
                        point,
                        kind: snap::Kind::Node,
                    })
                    .or_else(|| {
                        self.snap(self.camera.world(p, rect), self.drag.as_ref().map(|d| d.id))
                    })
                })
                .filter(|s| s.kind == snap::Kind::Node)
                .map(|s| s.point);
            let now = ui.input(|i| i.time);
            if let Some(point) = candidate {
                match self.dwell_polar_candidate {
                    Some((old, since))
                        if (old[0] - point[0]).hypot(old[1] - point[1]) * self.camera.zoom < 1. =>
                    {
                        if now - since >= 1. {
                            self.dwell_polar_anchor = Some(point);
                        }
                    }
                    _ => self.dwell_polar_candidate = Some((point, now)),
                }
                ui.ctx()
                    .request_repaint_after(std::time::Duration::from_millis(30));
            } else {
                self.dwell_polar_candidate = None;
            }
        } else {
            self.dwell_polar_candidate = None;
            self.dwell_polar_anchor = None;
        }
        if !ui.input(|i| i.modifiers.shift) {
            self.temporary_polar_anchor = None;
        } else if self.mode.is_some() && self.temporary_polar_anchor.is_none() {
            if let Some(pos) = ui
                .input(|i| i.pointer.hover_pos())
                .filter(|p| rect.contains(*p))
            {
                let raw = self.camera.world(pos, rect);
                self.temporary_polar_anchor = Some(self.snap(raw, None).map_or(raw, |s| s.point));
            }
        }

        let painter = ui.painter().with_clip_rect(rect);
        painter.rect_filled(rect, 0., self.canvas_background());

        if self.fit {
            if let Some(b) = self.bounds() {
                self.camera.center = [(b[0] + b[2]) * 0.5, (b[1] + b[3]) * 0.5];
                self.camera.zoom = ((rect.width() as f64 - 70.) / (b[2] - b[0]).max(1.))
                    .min((rect.height() as f64 - 70.) / (b[3] - b[1]).max(1.))
                    .max(1e-8);
                self.camera.angle = 0.;
            }
            self.fit = false;
        }
        if response.double_clicked_by(egui::PointerButton::Middle) {
            self.fit = true;
        }
        if response.dragged_by(egui::PointerButton::Middle) {
            let delta = ui.input(|i| i.pointer.delta());
            let a = self.camera.world(rect.center(), rect);
            let b = self.camera.world(rect.center() + delta, rect);
            self.camera.center[0] += a[0] - b[0];
            self.camera.center[1] += a[1] - b[1];
        }
        if response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            if scroll != 0. {
                let p = ui.input(|i| i.pointer.hover_pos()).unwrap_or(rect.center());
                let before = self.camera.world(p, rect);
                self.camera.zoom =
                    (self.camera.zoom * (scroll as f64 * 0.003).exp()).clamp(1e-8, 1000.);
                let after = self.camera.world(p, rect);
                self.camera.center[0] += before[0] - after[0];
                self.camera.center[1] += before[1] - after[1];
            }
        }
        let camera = self.camera;
        self.dynamic_rect = Some(rect);
        self.hover_pick.surface = Some((rect, ui.layer_id()));
        let screen = |p| camera.screen(p, rect);
        let pointer = ui.input(|i| i.pointer.interact_pos());
        let hovered = pointer.filter(|p| {
            rect.contains(*p)
                && ui.rect_contains_pointer(rect)
                && !self.toolbar_rect.is_some_and(|r| r.contains(*p))
        });
        let picking_grid_origin = self.grid_pick_origin;
        let grid_pointer = hovered.filter(|_| picking_grid_origin);
        if let Some(p) = grid_pointer {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
            if response.clicked_by(egui::PointerButton::Primary) {
                self.construction_grid.center = camera.world(p, rect);
                self.grid_pick_origin = false;
            }
        }
        let hovered = hovered.filter(|_| !picking_grid_origin);
        let locked_geometry = if self.selected.is_some() || !self.selection_ids.is_empty() {
            self.movement_locked_contours()
        } else {
            Default::default()
        };
        let over_selected_vertex =
            hovered.is_some_and(|p| self.selected_vertex_hit(p, rect, &locked_geometry));
        let foundation_busy =
            !picking_grid_origin && self.foundation_canvas_input(ui, &painter, rect, hovered);
        let load_value_busy = picking_grid_origin
            || foundation_busy
            || (!over_selected_vertex && self.load_value_canvas(ui, &painter, rect, hovered));
        let dimension_busy = load_value_busy || self.measurement_drag_canvas(ui, rect, hovered);
        let over_dimension = !over_selected_vertex
            && self.mode.is_none()
            && !self.measurements.active
            && self.scaling_op.is_none()
            && hovered.is_some_and(|p| self.measurement_hit(&painter, p, rect).is_some());
        let underlay_busy = dimension_busy
            || if (!over_dimension && !over_selected_vertex) || self.underlay_drag.is_some() {
                self.handle_underlay_drag(ui, &response, rect, hovered)
            } else {
                false
            };
        self.update_contour_hover(
            if over_dimension || underlay_busy {
                None
            } else {
                hovered
            },
            rect,
        );
        let copying = self.clipboard_op.is_some() || self.scaling_op.is_some();
        let measuring = self.measurements.active;
        let reusing = self.reuse_mode.is_some();
        let placing_arc = self.arc_placement.is_some();
        let mut marker = None;
        if placing_arc {
            if let Some(p) = hovered {
                let raw = camera.world(p, rect);
                let exclude = self.arc_placement.as_ref().map(|edit| edit.original.id);
                let found = self.snap(raw, exclude);
                marker = found;
                self.preview_edge_arc(found.map_or(raw, |s| s.point));
                ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
            }
        }
        let was_dragging = self.drag.is_some()
            && (response.dragged_by(egui::PointerButton::Primary)
                || response.drag_stopped_by(egui::PointerButton::Primary));
        if !underlay_busy
            && !over_dimension
            && !measuring
            && !copying
            && !placing_arc
            && self.mode.is_none()
            && self.drag.is_none()
            && !ui.input(|i| i.modifiers.shift || i.modifiers.ctrl)
            && ui.input(|i| i.pointer.primary_pressed())
        {
            if let (Some(id), Some(p)) = (
                self.selected.filter(|id| {
                    (over_selected_vertex || self.hover_pick.current() == Some(*id))
                        && !locked_geometry.contains(id)
                        && self.contour_editable(*id)
                        && self
                            .contours
                            .iter()
                            .any(|c| c.id == *id && self.selectable(c))
                }),
                hovered,
            ) {
                if let Some(c) = self.contours.iter().find(|c| c.id == id) {
                    if let Some((vertex, _)) = c
                        .handles()
                        .iter()
                        .enumerate()
                        .map(|(i, &q)| (i, screen(q).distance(p)))
                        .filter(|(_, d)| *d <= 9.)
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                    {
                        self.drag = Some(VertexDrag {
                            id,
                            vertex: Some(vertex),
                            edge: None,
                            anchor: c.handles()[vertex],
                            original: c.clone(),
                            before: self.contours.clone(),
                        });
                    } else if let Some((edge, a, b)) =
                        c.straight_edges().into_iter().find(|(_, a, b)| {
                            screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]).distance(p) <= 9.
                        })
                    {
                        self.drag = Some(VertexDrag {
                            id,
                            vertex: None,
                            edge: Some(edge),
                            anchor: [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5],
                            original: c.clone(),
                            before: self.contours.clone(),
                        });
                    } else if self.hit_contour(c, camera.world(p, rect), camera.zoom) {
                        self.drag = Some(VertexDrag {
                            id,
                            vertex: None,
                            edge: None,
                            anchor: camera.world(p, rect),
                            original: c.clone(),
                            before: self.contours.clone(),
                        });
                    }
                }
            }
        }
        if self.drag.is_some() && ui.input(|i| i.pointer.primary_pressed()) {
            self.dwell_polar_anchor = None;
            self.dwell_polar_candidate = None;
            self.temporary_polar_anchor = None;
            // Canvas editing owns undo now, not the last sidebar text field.
            if let Some(id) = ui.memory(|m| m.focused()) {
                ui.memory_mut(|m| m.surrender_focus(id));
            }
        }
        if let (Some(drag), Some(p)) = (
            &self.drag,
            pointer.filter(|_| {
                !picking_grid_origin
                    && (response.dragged_by(egui::PointerButton::Primary)
                        || response.drag_stopped_by(egui::PointerButton::Primary))
            }),
        ) {
            let raw = camera.world(p, rect);
            let found = self
                .locked_vertex_direction(ui, raw)
                .or_else(|| self.snap(raw, Some(drag.id)))
                .or_else(|| self.polar_snap(raw));
            self.draw_polar_guides(
                &painter,
                rect,
                raw,
                found.is_some_and(|s| s.kind == snap::Kind::Angle),
            );
            let mut candidate = drag.original.clone();
            let moved = if let Some(edge) = drag.edge {
                let target = found.map_or(raw, |s| s.point);
                candidate.move_edge(
                    edge,
                    [target[0] - drag.anchor[0], target[1] - drag.anchor[1]],
                )
            } else if let Some(vertex) = drag.vertex {
                candidate.move_handle(vertex, found.map_or(raw, |s| s.point))
            } else if response.dragged_by(egui::PointerButton::Primary)
                || response.drag_stopped_by(egui::PointerButton::Primary)
            {
                let target = found.map_or(raw, |s| s.point);
                candidate.translate([target[0] - drag.anchor[0], target[1] - drag.anchor[1]])
            } else {
                Ok(())
            };
            marker = found;
            match moved.and_then(|_| model::validate_candidate(&self.contours, &candidate)) {
                Ok(()) => {
                    let id = candidate.id;
                    if self.contours.iter().find(|c| c.id == id) != Some(&candidate) {
                        if candidate.is_mesh_input() {
                            self.fe_mesh = None;
                        }
                        if candidate.kind.affects_calculation() {
                            self.analysis = None;
                            self.analysis_input = None;
                            self.analysis_job = None;
                            self.combination_cache = None;
                        }
                    }
                    *self.contours.iter_mut().find(|c| c.id == id).unwrap() = candidate;
                    self.slab_cache = None;
                    self.status.clear();
                }
                Err(e) => self.status = e,
            }
        }
        if !picking_grid_origin && self.drag.is_some() && !ui.input(|i| i.pointer.primary_down()) {
            ui.ctx().data_mut(|data| {
                data.remove::<(u64, Point, f64)>(egui::Id::new("locked_vertex_direction"))
            });
            let drag = self.drag.take().unwrap();
            if self.contours != drag.before {
                self.history.push(drag.before);
            }
        }
        self.draw_order.begin_frame();
        let hidden = self.smoke && (3..8).contains(&self.frames);
        if !hidden {
            let start = self.draw_order.start(&painter);
            self.draw_pdf(&painter, rect);
            self.draw_order.finish(&painter, 0, start);
        }
        let dxf_start = self.draw_order.start(&painter);
        if !hidden && self.show_dxf {
            if let Some(d) = &self.drawing {
                let mut mesh = egui::Mesh::default();
                for (triangles, color, layer) in &d.fills {
                    if !self.floor_layer_visible(layer)
                        || self.layers.get(layer).is_some_and(|l| !l.visible)
                    {
                        continue;
                    }
                    let ink = self.layers.get(layer).map_or_else(
                        || {
                            let c = color.rgb();
                            Color32::from_rgb(c[0], c[1], c[2])
                        },
                        |l| l.ink(color.rgb()),
                    );
                    let ink = self.underlay_ink(layer, ink);
                    for &t in triangles {
                        append_triangle(&mut mesh, t, camera, rect, ink);
                    }
                }
                if !mesh.is_empty() {
                    painter.add(egui::Shape::mesh(mesh));
                }
            }
        }
        self.draw_order.finish(&painter, 1, dxf_start);
        let fill_start = self.draw_order.start(&painter);
        if self.slab_cache.is_none() {
            self.slab_cache = Some(model::slab_triangles_with_owner(&self.floor_contours()));
        }
        let mut mesh = egui::Mesh::default();
        let fill = Color32::from_rgba_unmultiplied(
            self.slab_color[0],
            self.slab_color[1],
            self.slab_color[2],
            (self.slab_opacity * 255.) as u8,
        );
        if self.drawing_visibility.fill && self.fe_fragment_active() {
            if let Some(fe) = &self.fe_mesh {
                for (i, e) in fe.elements.iter().enumerate() {
                    if self.mesh_element_on_floor(i) {
                        let owner = fe
                            .building
                            .as_ref()
                            .map(|data| data.element_slabs[i])
                            .or_else(|| {
                                if !self.drawing_visibility.material_colors
                                    && !self.drawing_visibility.stiffness_colors
                                    && self.schema_analysis.mode.is_none()
                                {
                                    return None;
                                }
                                let p = e.nodes.map(|n| fe.nodes[n].xy);
                                let center = [0, 1].map(|a| (p[0][a] + p[1][a] + p[2][a]) / 3.);
                                self.slab_property_owner(center)
                            });
                        let ink = owner
                            .and_then(|id| self.property_ink(id, Kind::Slab))
                            .map_or(fill, |ink| ink.gamma_multiply(self.slab_opacity));
                        append_triangle(
                            &mut mesh,
                            e.nodes.map(|n| fe.nodes[n].xy),
                            camera,
                            rect,
                            ink,
                        );
                    }
                }
            }
        } else if self.drawing_visibility.fill {
            for &(id, triangle) in self.slab_cache.as_ref().unwrap() {
                let ink = self.property_ink(id, Kind::Slab).map_or(fill, |ink| {
                    Color32::from_rgba_unmultiplied(ink.r(), ink.g(), ink.b(), fill.a())
                });
                append_triangle(&mut mesh, triangle, camera, rect, ink);
            }
        }
        if !mesh.is_empty() {
            painter.add(egui::Shape::mesh(mesh));
        }
        self.draw_order.finish(&painter, 15, fill_start);
        let grid_start = self.draw_order.start(&painter);
        self.draw_construction_grid(&painter, camera, rect);
        self.draw_order.finish(&painter, 14, grid_start);
        let dxf_start = self.draw_order.start(&painter);
        if !hidden && self.show_dxf {
            if let Some(d) = &self.drawing {
                for path in &d.paths {
                    let style = self.layers.get(&path.layer);
                    if !self.floor_layer_visible(&path.layer) || style.is_some_and(|s| !s.visible) {
                        continue;
                    }
                    let c = path.color.rgb();
                    let ink = style.map_or(Color32::from_rgb(c[0], c[1], c[2]), |s| s.ink(c));
                    let ink = self.underlay_ink(&path.layer, ink);
                    draw_path(
                        &painter,
                        &path.points,
                        path.closed,
                        camera,
                        rect,
                        Stroke::new(style.map_or(1., |s| s.width), ink),
                    );
                }
                if self.show_text {
                    for label in &d.labels {
                        let style = self.layers.get(&label.layer);
                        if !self.floor_layer_visible(&label.layer)
                            || style.is_some_and(|s| !s.visible)
                        {
                            continue;
                        }
                        let size = (label.height * camera.zoom) as f32;
                        let size = size.clamp(7., 120.);
                        let rgb = label.color.rgb();
                        let mut ink =
                            style.map_or(Color32::from_rgb(rgb[0], rgb[1], rgb[2]), |s| s.ink(rgb));
                        if self.contrast_text
                            && style.is_none_or(|s| s.color.is_none())
                            && (ink.r() as u16 + ink.g() as u16 + ink.b() as u16) < 140
                        {
                            ink = Color32::from_gray(225);
                        }
                        ink = self.underlay_ink(&label.layer, ink);
                        let galley = painter.layout_no_wrap(
                            label.text.clone(),
                            egui::FontId::proportional(size),
                            ink,
                        );
                        let offset = egui::vec2(
                            galley.size().x * label.anchor[0],
                            galley.size().y * label.anchor[1],
                        );
                        let angle = -(label.angle + camera.angle) as f32;
                        let rotated = egui::vec2(
                            angle.cos() * offset.x - angle.sin() * offset.y,
                            angle.sin() * offset.x + angle.cos() * offset.y,
                        );
                        let position = screen(label.point) - rotated;
                        if Rect::from_min_size(position, galley.size() * label.width_factor as f32)
                            .expand(galley.size().length())
                            .intersects(rect)
                        {
                            painter.add(
                                egui::epaint::TextShape::new(position, galley, ink)
                                    .with_angle(angle),
                            );
                        }
                    }
                }
            }
        }
        self.draw_order.finish(&painter, 1, dxf_start);
        if model::same_mesh_input(&self.mesh_input, &self.contours)
            && self.mesh_built_size == self.mesh_size
        {
            if let Some(mesh) = &self.fe_mesh {
                let start = self.draw_order.start(&painter);
                self.paint_analysis(&painter, rect);
                self.draw_order.finish(&painter, 12, start);
                if self.show_mesh {
                    let start = self.draw_order.start(&painter);
                    self.paint_cached_edges(&painter, rect);
                    self.draw_order.finish(&painter, 11, start);
                }
                if self.fe_fragment_active()
                    && !(self.show_results && (17..=22).contains(&self.result_field))
                {
                    self.paint_fe_fragment_bars_2d(&painter, rect);
                }
                if self.show_support_nodes && self.drawing_visibility.contours[2] {
                    for indices in mesh.support_boundary.values() {
                        for &n in indices {
                            if !self.mesh_node_on_floor(n) {
                                continue;
                            }
                            let p = camera.screen(mesh.nodes[n].xy, rect);
                            if rect.contains(p) {
                                painter.circle_filled(p, 2.5, Color32::from_rgb(90, 210, 255));
                            }
                        }
                    }
                }
            }
        }
        for c in self
            .contours
            .iter()
            .filter(|c| !c.kind.is_load() && self.contour_visible(c))
        {
            let contour_start = self.draw_order.start(&painter);
            let ink = self.object_ink(c.id, c.kind);
            if self.drawing_visibility.border(c.kind)
                && !(c.kind == Kind::Beam && self.beam_result_active(c.id))
            {
                let stroke = Stroke::new(if self.is_selected(c.id) { 3_f32 } else { 2_f32 }, ink);
                if c.kind == Kind::PointRestraint {
                    let tip = camera.screen(c.points[0], rect);
                    painter.add(egui::Shape::closed_line(
                        vec![tip, tip + egui::vec2(-5., 8.), tip + egui::vec2(5., 8.)],
                        stroke,
                    ));
                } else if punching::is_punching(c) {
                    self.paint_source_contour_outline(&painter, c, rect, stroke);
                } else {
                    draw_path(
                        &painter,
                        self.arc_placement
                            .as_ref()
                            .filter(|e| e.original.id == c.id)
                            .and_then(|e| e.preview.as_ref())
                            .map_or(c.points.as_slice(), |p| p.points.as_slice()),
                        c.closed(),
                        camera,
                        rect,
                        stroke,
                    );
                    foundation_ui::draw_foundation_exclusions(&painter, c, camera, rect, stroke);
                }
                if let Some(grid) = &c.grid {
                    if grid.dx().min(grid.dy()) * camera.zoom >= 6. {
                        let corners = [
                            rect.left_top(),
                            rect.right_top(),
                            rect.right_bottom(),
                            rect.left_bottom(),
                        ]
                        .map(|p| grid.coordinates(camera.world(p, rect)));
                        let x0 = corners
                            .iter()
                            .map(|p| p[0])
                            .fold(f64::INFINITY, f64::min)
                            .floor()
                            .clamp(0., grid.nx as f64) as usize;
                        let x1 = corners
                            .iter()
                            .map(|p| p[0])
                            .fold(f64::NEG_INFINITY, f64::max)
                            .ceil()
                            .clamp(0., grid.nx as f64) as usize;
                        let y0 = corners
                            .iter()
                            .map(|p| p[1])
                            .fold(f64::INFINITY, f64::min)
                            .floor()
                            .clamp(0., grid.ny as f64) as usize;
                        let y1 = corners
                            .iter()
                            .map(|p| p[1])
                            .fold(f64::NEG_INFINITY, f64::max)
                            .ceil()
                            .clamp(0., grid.ny as f64) as usize;
                        for x in x0..=x1 {
                            for y in y0..=y1 {
                                let at = camera.screen(grid.at(x as f64, y as f64), rect);
                                if rect.contains(at) {
                                    painter.circle_filled(at, 2.5, ink);
                                }
                            }
                        }
                    }
                }
            }
            if !underlay_busy
                && !measuring
                && !copying
                && !placing_arc
                && !self.is_kind_locked(c.kind)
                && !locked_geometry.contains(&c.id)
                && self.is_selected(c.id)
                && self.mode.is_none()
            {
                for (_, a, b) in c.straight_edges() {
                    let q = screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]);
                    if rect.contains(q) {
                        painter.circle_filled(q, 4., ink);
                        painter.circle_stroke(q, 4., Stroke::new(1_f32, Color32::WHITE));
                    }
                }
                for p in c.handles() {
                    let q = screen(p);
                    if rect.contains(q) {
                        let box_ = Rect::from_center_size(q, Vec2::splat(8.));
                        painter.rect_filled(box_, 0., ink);
                        painter.rect_stroke(
                            box_,
                            0.,
                            Stroke::new(1_f32, Color32::WHITE),
                            egui::StrokeKind::Inside,
                        );
                    }
                }
            }
            self.draw_order
                .finish(&painter, draw_order::category(c.kind), contour_start);
        }
        self.draw_loads(&painter, camera, rect);
        let start = self.draw_order.start(&painter);
        self.paint_load_mosaics(&painter, rect);
        self.draw_order.finish(&painter, 12, start);
        if !underlay_busy && !measuring && !copying && !placing_arc && self.mode.is_none() {
            if let (Some(id), Some(pointer)) = (self.selected, hovered) {
                if let Some(contour) = self
                    .contours
                    .iter()
                    .find(|c| c.id == id && self.contour_visible(c))
                {
                    let arc = contour
                        .curves
                        .iter()
                        .filter_map(|c| c.arc_dimensions())
                        .filter_map(|arc| {
                            let mid = screen(arc.midpoint).distance(pointer);
                            let control = screen(arc.through).distance(pointer);
                            let distance = mid.min(control);
                            (distance <= 10.).then_some((
                                distance,
                                if control < mid {
                                    arc.through
                                } else {
                                    arc.midpoint
                                },
                                arc.ends,
                                arc.radius,
                            ))
                        })
                        .min_by(|a, b| a.0.total_cmp(&b.0));
                    let measurement = if let Some((_, point, ends, radius)) = arc {
                        Some((point, ends.to_vec(), Some(radius)))
                    } else {
                        let points = contour.handles();
                        if points.len() < 2 {
                            None
                        } else {
                            points
                                .iter()
                                .enumerate()
                                .map(|(i, &p)| (i, screen(p).distance(pointer)))
                                .filter(|(_, d)| *d <= 10.)
                                .min_by(|a, b| a.1.total_cmp(&b.1))
                                .map(|(i, _)| {
                                    (
                                        points[i],
                                        (0..points.len())
                                            .filter(|&j| {
                                                j != i
                                                    && (j + 1 == i
                                                        || i + 1 == j
                                                        || (contour.closed()
                                                            && ((i == 0 && j == points.len() - 1)
                                                                || (j == 0
                                                                    && i == points.len() - 1))))
                                            })
                                            .map(|j| points[j])
                                            .collect::<Vec<_>>(),
                                        None,
                                    )
                                })
                        }
                    };
                    if let Some((a, neighbors, radius)) = measurement {
                        let color = self.display_text.ink();
                        for b in neighbors {
                            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                            if !length.is_finite() || length < 1e-6 {
                                continue;
                            }
                            draw_path(
                                &painter,
                                &[a, b],
                                false,
                                camera,
                                rect,
                                Stroke::new(1_f32, color),
                            );
                            let center =
                                screen([a[0] + (b[0] - a[0]) * 0.5, a[1] + (b[1] - a[1]) * 0.5]);
                            let text = painter.layout_no_wrap(
                                format!("{} мм", self.display_text.number(length)),
                                egui::FontId::proportional(self.display_text.size),
                                color,
                            );
                            let label =
                                Rect::from_center_size(center, text.size() + egui::vec2(8., 4.));
                            if label.intersects(rect) {
                                painter.rect_filled(label, 2., Color32::from_rgb(24, 28, 35));
                                painter.galley(label.min + egui::vec2(4., 2.), text, color);
                            }
                        }
                        if let Some(radius) = radius {
                            let text = painter.layout_no_wrap(
                                format!("R {} мм", self.display_text.number(radius)),
                                egui::FontId::proportional(self.display_text.size),
                                color,
                            );
                            let label = Rect::from_min_size(
                                screen(a) + egui::vec2(12., -24.),
                                text.size() + egui::vec2(8., 4.),
                            );
                            painter.rect_filled(label, 2., Color32::from_rgb(24, 28, 35));
                            painter.galley(label.min + egui::vec2(4., 2.), text, color);
                        }
                    }
                }
            }
        }
        self.paint_support_extents(&painter, rect);
        self.paint_columns(&painter, rect);
        self.paint_underlay_selection(&painter, rect);
        let start = self.draw_order.start(&painter);
        self.draw_measurements(&painter, rect);
        self.draw_order.finish(&painter, 13, start);
        self.draw_order.apply(&painter);
        self.draw_origin_axes(&painter, rect);
        self.paint_warning(&painter, |p| self.camera.screen([p[0], p[1]], rect));
        if !picking_grid_origin {
            self.restraint_buttons(painter.ctx(), rect, |p, _| camera.screen(p, rect));
        }
        self.paint_contour_hover(&painter, rect);
        self.paint_selected_contours(&painter, rect, &locked_geometry);
        if !picking_grid_origin {
            self.punching_edge_buttons(ui, rect);
        }
        self.paint_result_annotations(&painter, rect, if measuring { None } else { hovered });
        if let Some((index, range)) = self.length_mosaic(&painter, rect, None, None, |p| {
            (camera.screen([p[0], p[1]], rect), 0.)
        }) {
            self.length_scales[index].legend(&painter, rect, range, self.display_text);
        }
        let mut target = if picking_grid_origin {
            None
        } else {
            self.measurement_canvas(ui, &response, rect, hovered)
        };
        let mut closes = false;
        if let Some(p) = hovered.or(grid_pointer) {
            let raw = camera.world(p, rect);
            if !picking_grid_origin && self.mode.is_some() && !reusing {
                let first = self.draft.first().or(self.curve_controls.first()).copied();
                let can_close = matches!(self.tool, Tool::Line | Tool::Spline | Tool::Arc)
                    && (self.can_finish_sketch()
                        || (self.tool == Tool::Arc
                            && self.curve_controls.len() == 2
                            && self.draft.len() > 1));
                let found = if self.snap_settings.geometry_nodes
                    && can_close
                    && first.is_some_and(|q| screen(q).distance(p) <= 12.)
                {
                    closes = true;
                    Some(snap::Snap {
                        point: first.unwrap(),
                        kind: snap::Kind::Node,
                    })
                } else {
                    self.snap(raw, None).or_else(|| self.polar_snap(raw))
                };
                self.draw_polar_guides(
                    &painter,
                    rect,
                    raw,
                    found.is_some_and(|s| s.kind == snap::Kind::Angle),
                );
                target = Some(found.map_or(raw, |s| s.point));
                marker = found;
                ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
                if let Some(end) = self.dynamic_endpoint(raw) {
                    target = Some(end);
                    marker = None;
                    closes = false;
                }
            }
            let p = target.or_else(|| marker.map(|s| s.point)).unwrap_or(raw);
            let z = if self.building.enabled {
                self.building.floor(self.building.active).elevation_mm
            } else {
                0.
            };
            painter.text(
                rect.right_bottom() - egui::vec2(10., 10.),
                egui::Align2::RIGHT_BOTTOM,
                format!("X {:.1} · Y {:.1} · Z {z:.1} мм", p[0], p[1]),
                egui::FontId::monospace(12.),
                Color32::LIGHT_GRAY,
            );
        }
        if let Some(s) = marker {
            let p = screen(s.point);
            let color = Color32::from_rgb(70, 235, 225);
            if s.kind == snap::Kind::Node {
                painter.circle_stroke(p, 5., Stroke::new(1.7_f32, color));
            } else {
                painter.rect_stroke(
                    Rect::from_center_size(p, Vec2::splat(9.)),
                    0.,
                    Stroke::new(1.7_f32, color),
                    egui::StrokeKind::Middle,
                );
            }
        }
        if let Some(kind) = self.mode {
            self.draw_dynamic_length(&painter, rect, target);
            let p = if kind == Kind::Triangulation && self.triangulation_grid.active {
                target
                    .and_then(|origin| self.triangulation_grid.points(origin).ok())
                    .unwrap_or_default()
            } else {
                self.sketch_preview(target)
            };
            draw_path(
                &painter,
                &p,
                false,
                camera,
                rect,
                Stroke::new(2_f32, self.contour_color(kind)),
            );
            for &p in self.draft.iter().chain(&self.curve_controls) {
                painter.circle_filled(screen(p), 3.5, self.contour_color(kind));
            }
        }
        self.paint_foundation_values(&painter, rect, camera);
        if self.reinforcement_directions {
            self.paint_reinforcement_directions(&painter, rect, |p| {
                camera.screen([p[0], p[1]], rect)
            });
        }
        self.paint_intersections(&painter, rect, camera);
        if picking_grid_origin {
            self.paint_fe_selection_2d(&painter, camera, rect);
            return;
        }
        self.triangulation_grid_editor(ui, rect);
        self.reuse_canvas(ui, &response, rect, hovered);
        self.clipboard_canvas(ui, &response, rect, hovered);
        let _ = self.scaling_canvas(ui, &response, rect, hovered);
        let framed = if dimension_busy {
            self.selection_start = None;
            false
        } else {
            self.selection_frame(ui, &response, rect, hovered)
        };
        let legend_clicked = !underlay_busy
            && !measuring
            && !copying
            && !placing_arc
            && !reusing
            && !was_dragging
            && !framed
            && self.pick_legend_band(rect, &response, ui.input(|i| i.modifiers));
        let fe_clicked = if !legend_clicked
            && !underlay_busy
            && !measuring
            && !copying
            && !placing_arc
            && !reusing
            && !was_dragging
            && !framed
            && self.drag.is_none()
            && self.mode.is_none()
            && response.clicked_by(egui::PointerButton::Primary)
        {
            self.fe_pick_2d(hovered, camera, rect, ui.input(|i| i.modifiers))
        } else {
            false
        };
        self.paint_fe_selection_2d(&painter, camera, rect);
        if placing_arc && response.clicked_by(egui::PointerButton::Primary) {
            self.accept_edge_arc();
        }
        if !fe_clicked
            && !legend_clicked
            && !underlay_busy
            && !measuring
            && !copying
            && !placing_arc
            && !reusing
            && response.clicked_by(egui::PointerButton::Primary)
            && !was_dragging
            && !framed
            && self.drag.is_none()
        {
            if let Some(p) = target {
                self.sketch_click(p, closes);
            } else if let Some(p) = hovered {
                let p = camera.world(p, rect);
                if let Some(id) = self.measurement_hit(&painter, camera.screen(p, rect), rect) {
                    self.select_all_scene([], [], [id], ui.input(|i| i.modifiers));
                } else if let Some(id) = self.hover_pick.current().filter(|id| {
                    self.contours
                        .iter()
                        .any(|c| c.id == *id && self.selectable(c))
                }) {
                    self.select_ids([id], ui.input(|i| i.modifiers));
                } else if let Some(id) = self.hit_underlay(p) {
                    self.select_scene([], [id], ui.input(|i| i.modifiers));
                }
            }
        }
        if !underlay_busy
            && !measuring
            && !copying
            && !placing_arc
            && response.double_clicked_by(egui::PointerButton::Primary)
            && self.mode.is_some()
            && matches!(self.tool, Tool::Line | Tool::Spline)
            && self.can_finish_sketch()
        {
            self.finish();
        }
        if let Some(edit) = &self.arc_placement {
            for p in [edit.a, edit.b] {
                painter.circle_filled(screen(p), 4., Color32::WHITE);
            }
            if let (Some(radius), Some(p)) = (edit.radius, hovered) {
                let color = self.display_text.ink();
                let text = painter.layout_no_wrap(
                    format!("R {} мм", self.display_text.number(radius)),
                    egui::FontId::proportional(13.),
                    color,
                );
                let label =
                    Rect::from_min_size(p + egui::vec2(14., 14.), text.size() + egui::vec2(8., 4.));
                painter.rect_filled(label, 2., Color32::from_rgb(24, 28, 35));
                painter.galley(label.min + egui::vec2(4., 2.), text, color);
            }
        }
        if response.clicked_by(egui::PointerButton::Secondary) {
            self.load_menu_target = None;
            self.support_menu_target = None;
            self.menu_target = None;
            self.arc_menu_target = None;
            if !underlay_busy && !measuring && !copying && !placing_arc && self.mode.is_none() {
                if let Some(pointer) = hovered {
                    let p = camera.world(pointer, rect);
                    self.dxf_menu_target = self.hit_underlay(p);
                    self.load_menu_target = self.load_context_hit(p, camera.zoom);
                    self.support_menu_target = self
                        .contours
                        .iter()
                        .rev()
                        .find(|c| {
                            matches!(c.kind, Kind::Support | Kind::Column)
                                && self.hit_contour(c, p, camera.zoom)
                        })
                        .map(|c| c.id);
                    if self.selected.is_none()
                        && self.selection_ids.is_empty()
                        && self.selected_underlay.is_none()
                        && self.underlay_selection.is_empty()
                        && self.measurements.selected.is_empty()
                    {
                        if let Some(id) = self.measurement_hit(&painter, pointer, rect) {
                            self.select_all_scene([], [], [id], egui::Modifiers::default());
                        } else if let Some(id) = self.load_menu_target.or_else(|| {
                            self.contours
                                .iter()
                                .rev()
                                .find(|c| self.hit_contour(c, p, camera.zoom))
                                .map(|c| c.id)
                        }) {
                            self.select_only(Some(id));
                        } else if let Some(id) = self.hit_underlay(p) {
                            self.select_scene([], [id], egui::Modifiers::default());
                        }
                    }
                    let mut best: Option<(f64, u64, usize)> = None;
                    for contour in self.contours.iter().filter(|c| {
                        self.selectable(c) && self.load_menu_target.is_none_or(|id| c.id == id)
                    }) {
                        if self.is_kind_locked(contour.kind) {
                            continue;
                        }
                        for (edge, a, b) in contour.straight_edges() {
                            let d = [b[0] - a[0], b[1] - a[1]];
                            let len2 = d[0] * d[0] + d[1] * d[1];
                            if len2 < 1e-12 {
                                continue;
                            }
                            let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len2;
                            if !(0.05..=0.95).contains(&t) {
                                continue;
                            }
                            let distance = (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
                                * camera.zoom;
                            if distance <= 12. && best.is_none_or(|old| distance < old.0) {
                                best = Some((distance, contour.id, edge));
                            }
                        }
                    }
                    self.arc_menu_target = best.map(|(_, id, edge)| (id, edge));
                }
            }
            if let (Some(id), Some(p)) = (
                self.selected
                    .filter(|id| !placing_arc && self.contour_geometry_editable(*id)),
                hovered,
            ) {
                if let Some(c) = self.contours.iter().find(|c| c.id == id) {
                    if let Some((i, d)) = c
                        .handles()
                        .iter()
                        .enumerate()
                        .map(|(i, &q)| (i, screen(q).distance(p)))
                        .min_by(|a, b| a.1.total_cmp(&b.1))
                    {
                        if d < 13. {
                            self.menu_target = Some((id, i, true));
                        } else {
                            let q = camera.world(p, rect);
                            let edge = c
                                .points
                                .iter()
                                .copied()
                                .zip(c.points.iter().copied().cycle().skip(1))
                                .enumerate()
                                .take(if c.closed() {
                                    c.points.len()
                                } else {
                                    c.points.len().saturating_sub(1)
                                })
                                .map(|(i, (a, b))| {
                                    let dx = b[0] - a[0];
                                    let dy = b[1] - a[1];
                                    let t = (((q[0] - a[0]) * dx + (q[1] - a[1]) * dy)
                                        / (dx * dx + dy * dy))
                                        .clamp(0., 1.);
                                    (i, (q[0] - a[0] - t * dx).hypot(q[1] - a[1] - t * dy))
                                })
                                .min_by(|a, b| a.1.total_cmp(&b.1));
                            if let Some((i, d)) = edge {
                                if d * camera.zoom < 12. && c.curves.is_empty() {
                                    self.menu_target = Some((id, i, false));
                                }
                            }
                        }
                    }
                }
            }
        }
        if self.mode.is_none() && self.drag.is_none() && self.selected.is_some() {
            if let Some(p) = response.hover_pos() {
                self.draw_polar_guides(&painter, rect, self.camera.world(p, rect), false);
            }
        }
        if !foundation_busy {
            response.context_menu(|ui| self.scene_context_menu(ui));
        }
    }
}
fn result_values_icon(ui: &mut egui::Ui, selected: bool) -> egui::Response {
    let (r, response) = ui.allocate_exact_size(egui::vec2(42., 24.), egui::Sense::click());
    let v = ui.style().interact_selectable(&response, selected);
    let p = ui.painter();
    p.rect_filled(r, 3., v.bg_fill);
    p.circle_filled(r.min + egui::vec2(9., 15.), 2.5, v.fg_stroke.color);
    p.line_segment(
        [r.min + egui::vec2(11., 13.), r.min + egui::vec2(15., 8.)],
        Stroke::new(1., v.fg_stroke.color),
    );
    p.text(
        r.min + egui::vec2(27., 11.),
        egui::Align2::CENTER_CENTER,
        "12",
        egui::FontId::proportional(13.),
        v.fg_stroke.color,
    );
    response
}
// Vector icon: three storeys, with the active middle slab highlighted.
fn active_floor_icon(ui: &mut egui::Ui, selected: bool) -> egui::Response {
    let (r, response) = ui.allocate_exact_size(egui::vec2(30., 24.), egui::Sense::click());
    let v = ui.style().interact_selectable(&response, selected);
    let p = ui.painter();
    p.rect_filled(r, 3., v.bg_fill);
    for (dy, active) in [(4., false), (10., true), (16., false)] {
        let points = vec![
            r.min + egui::vec2(5., dy + 2.),
            r.min + egui::vec2(16., dy),
            r.min + egui::vec2(25., dy + 3.),
            r.min + egui::vec2(14., dy + 5.),
        ];
        let color = if active {
            Color32::from_rgb(50, 190, 255)
        } else {
            v.fg_stroke.color.gamma_multiply(0.5)
        };
        p.add(egui::Shape::convex_polygon(
            points,
            if active {
                color.gamma_multiply(0.55)
            } else {
                Color32::TRANSPARENT
            },
            Stroke::new(1., color),
        ));
    }
    response
}
fn input_lock_icon(ui: &mut egui::Ui, locked: bool) -> egui::Response {
    let (r, response) = ui.allocate_exact_size(egui::vec2(30., 24.), egui::Sense::click());
    let v = ui.style().interact_selectable(&response, locked);
    let p = ui.painter();
    p.rect_filled(r, 3., v.bg_fill);
    let stroke = Stroke::new(1.6, v.fg_stroke.color);
    p.rect_stroke(
        Rect::from_min_max(r.min + egui::vec2(8., 10.), r.min + egui::vec2(22., 21.)),
        2.,
        stroke,
        egui::StrokeKind::Inside,
    );
    let dx = if locked { 0. } else { 4. };
    p.line_segment(
        [
            r.min + egui::vec2(11. + dx, 10.),
            r.min + egui::vec2(11. + dx, 5.),
        ],
        stroke,
    );
    p.line_segment(
        [
            r.min + egui::vec2(11. + dx, 5.),
            r.min + egui::vec2(19. + dx, 5.),
        ],
        stroke,
    );
    p.line_segment(
        [
            r.min + egui::vec2(19. + dx, 5.),
            r.min + egui::vec2(19. + dx, 10.),
        ],
        stroke,
    );
    p.circle_filled(r.min + egui::vec2(15., 15.), 1.6, v.fg_stroke.color);
    response
}
fn load_visibility_icon(ui: &mut egui::Ui, selected: bool, values: bool) -> egui::Response {
    let (r, response) = ui.allocate_exact_size(
        egui::vec2(if values { 44. } else { 30. }, 24.),
        egui::Sense::click(),
    );
    let visuals = ui.style().interact_selectable(&response, selected);
    let p = ui.painter();
    p.rect_filled(r, 3., visuals.bg_fill);
    let ink = visuals.fg_stroke.color;
    let stroke = Stroke::new(1.4_f32, ink);
    let left = r.left() + 6.;
    for x in [left, left + 8., left + 16.] {
        let top = r.top() + 5.;
        let bottom = r.top() + 17.;
        p.line_segment([Pos2::new(x, top), Pos2::new(x, bottom)], stroke);
        p.line_segment(
            [Pos2::new(x - 2.5, bottom - 3.), Pos2::new(x, bottom)],
            stroke,
        );
        p.line_segment(
            [Pos2::new(x + 2.5, bottom - 3.), Pos2::new(x, bottom)],
            stroke,
        );
    }
    p.line_segment(
        [
            Pos2::new(left - 3., r.top() + 19.),
            Pos2::new(left + 19., r.top() + 19.),
        ],
        stroke,
    );
    if values {
        p.text(
            Pos2::new(r.right() - 3., r.center().y),
            egui::Align2::RIGHT_CENTER,
            "q",
            egui::FontId::proportional(13.),
            ink,
        );
    }
    response
}
fn tool_icon(ui: &mut egui::Ui, tool: Tool, selected: bool) -> bool {
    let (r, response) = ui.allocate_exact_size(Vec2::splat(30.), egui::Sense::click());
    let fill = if selected {
        ui.visuals().selection.bg_fill
    } else if response.hovered() {
        ui.visuals().widgets.hovered.bg_fill
    } else {
        ui.visuals().widgets.inactive.bg_fill
    };
    let painter = ui.painter();
    painter.rect_filled(r, 3., fill);
    painter.rect_stroke(
        r,
        3.,
        Stroke::new(
            1_f32,
            if selected {
                ui.visuals().selection.stroke.color
            } else {
                ui.visuals().widgets.inactive.fg_stroke.color
            },
        ),
        egui::StrokeKind::Inside,
    );
    let pos = |x: f32, y: f32| r.min + egui::vec2(x, y);
    let curve = match tool {
        Tool::Line => vec![pos(6., 24.), pos(24., 6.)],
        Tool::Spline => (0..=24)
            .map(|i| {
                let t = i as f32 / 24.;
                pos(
                    5. + 20. * t,
                    24. - 18. * t - 4. * (t * std::f32::consts::TAU).sin(),
                )
            })
            .collect(),
        Tool::Arc => (0..=24)
            .map(|i| {
                let a = std::f32::consts::PI * i as f32 / 24.;
                pos(15. - 10. * a.cos(), 9. + 14. * a.sin())
            })
            .collect(),
        Tool::Ellipse => (0..=32)
            .map(|i| {
                let a = std::f32::consts::TAU * i as f32 / 32.;
                pos(15. + 11. * a.cos(), 15. + 7. * a.sin())
            })
            .collect(),
        Tool::Rectangle => vec![
            pos(5., 7.),
            pos(25., 7.),
            pos(25., 26.),
            pos(5., 26.),
            pos(5., 7.),
        ],
    };
    for pair in curve.windows(2) {
        painter.line_segment(
            [pair[0], pair[1]],
            Stroke::new(
                1.6_f32,
                if selected {
                    ui.visuals().selection.stroke.color
                } else {
                    ui.visuals().text_color()
                },
            ),
        );
    }
    for p in [curve[0], curve[curve.len() / 2], *curve.last().unwrap()] {
        painter.rect_filled(
            Rect::from_center_size(p, Vec2::splat(3.)),
            0.,
            ui.visuals().selection.stroke.color,
        );
    }
    response
        .on_hover_text(format!("{}\n{}", tool.name(), tool.hint()))
        .clicked()
}
impl eframe::App for App {
    fn raw_input_hook(&mut self, ctx: &egui::Context, input: &mut egui::RawInput) {
        if !self.soil_view.open {
            self.rectangle_raw_input(ctx, input);
            self.contour_hover_raw_input(ctx, input);
        }
    }

    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.start_load_fill_frame();
        if self.smoke_section_toggle && self.frames % 8 == 0 {
            let enabled = self.frames % 16 == 0;
            self.sections.planes = [
                (sections::Kind::Xoy, -2000., 0.),
                (sections::Kind::Xoz, -5000., 0.),
                (sections::Kind::Yoz, -5000., 0.),
                (sections::Kind::VerticalAngle, 0., 35.),
            ]
            .into_iter()
            .map(|(kind, offset_mm, alpha_deg)| sections::Plane {
                enabled,
                kind: Some(kind),
                offset_mm,
                alpha_deg,
                ..Default::default()
            })
            .collect();
            self.section_geometry_changed();
        }
        self.poll_soil_job(ctx);
        #[cfg(target_arch = "wasm32")]
        {
            crate::plan_trace::platform::set_repaint_context(ctx);
            self.poll_browser_files(ctx);
            self.poll_opfs();
        }
        self.sync_building_objects();
        self.apply_appearance(ctx);
        self.sync_stiffness_sources();
        self.project_close_guard(ctx);
        #[cfg(target_arch = "wasm32")]
        self.opfs_dialog(ctx);
        self.pace_frame();
        self.tick_fps(ctx);
        self.poll_lira_export();
        self.poll_load();
        self.poll_pdf();
        self.poll_mesh();
        self.poll_intersections(ctx);
        if !self.poll_solver_cancellation() {
            self.poll_analysis();
            self.poll_alternative_analysis();
        }
        if self.analysis_job.is_none() && self.alternative_job.is_none() {
            self.solver_cancel = None;
        }
        #[cfg(target_arch = "wasm32")]
        if self.analysis_job.is_none() && self.alternative_job.is_none() {
            self.solver_worker = None;
        }
        self.refresh_summation_results();
        self.sync_result_units();
        self.sync_combination_result();
        self.poll_reinforcement(ctx);
        if let Some(field) = self.smoke_load_view {
            if self.prepare_load_mosaic_smoke(field) {
                self.smoke_load_view = None;
                self.capture_requested = false;
                self.frames = 0;
            }
        }
        if self.smoke_show_tools && self.fe_mesh.is_some() {
            self.smoke_show_tools = false;
            self.set_contour_lock(false);
            self.begin(Kind::Slab);
        }
        if !self.soil_view.open {
            let choosing_pdf = self.pdf_choice.is_some();
            self.pdf_page_dialog(ctx);
            self.lir_import_window(ctx);
            self.dxf_contours_dialog(ctx);
            self.measurement_window(ctx);
            self.scaling_dialog(ctx);
            self.load_value_dialog(ctx);
            self.foundation_value_dialog(ctx);
            self.wall_restraint_dialog(ctx);
            self.automatic_restraint_dialog(ctx);
            if !choosing_pdf && self.scaling_op.is_none() {
                self.clipboard_keys(ctx);
                if !self.spatial.enabled {
                    self.dynamic_input(ctx);
                }
            }
            if !choosing_pdf
                && !ctx.wants_keyboard_input()
                && ctx.input(|i| i.key_pressed(egui::Key::Escape))
            {
                if self.schema_analysis.mode.take().is_some() {
                    self.slab_cache = None;
                    self.invalidate_scene_buffers();
                    self.spatial.cache_reset();
                }
                self.fe_selection.items.clear();
                self.show_results = false;
                self.punching_view = None;
                self.length_view = 0;
                ctx.data_mut(|d| d.remove::<fem_ui::ModalView>(egui::Id::new("modal_view")));
                if self.spatial.enabled {
                    self.cancel();
                    self.select_only(None);
                    self.spatial.frame_start = None;
                }
            }
            // A sidebar button keeps keyboard focus after Delete. That must not
            // disable global undo; only an actual text editor owns text undo.
            let editing_text = ctx
                .memory(|m| m.focused())
                .is_some_and(|id| egui::TextEdit::load_state(ctx, id).is_some());
            if !choosing_pdf && !editing_text {
                self.handle_undo_shortcut(ctx);
            }
            if !choosing_pdf
                && !editing_text
                && self.clipboard_op.is_none()
                && self.scaling_op.is_none()
                && !self.measurements.active
                && ctx.input(|i| i.key_pressed(egui::Key::Delete))
            {
                self.remove();
            }
            if !choosing_pdf && !ctx.wants_keyboard_input() {
                if ctx.input(|i| i.key_pressed(egui::Key::Escape)) {
                    self.cancel();
                }
                let spatial_dynamic = self.spatial.enabled
                    && (!self.dynamic_length.is_empty()
                        || self.dynamic_rectangle.iter().any(|s| !s.is_empty()));
                if !spatial_dynamic
                    && self.scaling_op.is_none()
                    && ctx.input(|i| i.key_pressed(egui::Key::Enter))
                {
                    self.finish();
                }
                if !spatial_dynamic && ctx.input(|i| i.key_pressed(egui::Key::Backspace)) {
                    self.sketch_back();
                }
            }
            self.creation_ribbon(ctx);
            self.bottom_bar(ctx);
            self.sync_source_view();
            self.sidebar(ctx);
            self.sync_source_view();
            self.schema_refresh();
            self.right_sidebar(ctx);
            self.drawing_flags_window(ctx);
            self.contour_filter_window(ctx);
            self.object_properties_window(ctx);
            self.diagnostics_window(ctx);
            self.soil_pressure_window(ctx);
        } else {
            self.soil_editor_sidebars(ctx);
        }
        let can_view_3d = self.support_height.enabled
            || self.building.enabled
            || self
                .analysis
                .as_ref()
                .and_then(|r| r.buckling.as_ref())
                .is_some_and(|r| r.as_ref().is_ok_and(|v| !v.modes.is_empty()));
        if !can_view_3d {
            self.spatial.enabled = false;
        }
        self.sync_load_scale();
        self.sync_variant_scale();
        egui::CentralPanel::default().show(ctx, |ui| {
            let name = if self.soil_view.open {
                String::new()
            } else if let Some(view) = self
                .spatial
                .enabled
                .then(|| self.displayed_shape(ctx, false))
                .flatten()
            {
                let caption = view.source.caption(view.number);
                self.modal_mass_source.as_ref().map_or_else(
                    || caption.clone(),
                    |source| format!("{caption} · Массы · {source}"),
                )
            } else if self.building.enabled && !self.spatial.enabled {
                format!(
                    "{} · {}",
                    self.building.floor(self.building.active).name,
                    self.result_caption()
                )
            } else {
                self.result_caption()
            };
            ui.horizontal(|ui| {
                let before = self.spatial.enabled;
                if ui
                    .selectable_label(!self.soil_view.open && !self.spatial.enabled, "2D")
                    .clicked()
                {
                    self.spatial.enabled = false;
                    self.soil_view.open = false;
                }
                ui.add_enabled_ui(can_view_3d, |ui| {
                    if ui
                        .selectable_label(!self.soil_view.open && self.spatial.enabled, "3D")
                        .clicked()
                    {
                        self.spatial.enabled = true;
                        self.soil_view.open = false;
                    }
                })
                .response
                .on_disabled_hover_text("Включите учёт высоты опор");
                self.soil_editor_tab(ui);
                if before != self.spatial.enabled {
                    self.cancel();
                }
                let fps_width = 125.;
                let title_width =
                    (ui.available_width() - fps_width - ui.spacing().item_spacing.x).max(0.);
                ui.add_sized(
                    [title_width, 22.],
                    egui::Label::new(egui::RichText::new(&name).strong()).truncate(),
                )
                .on_hover_text(name);
                let (fps_rect, _) =
                    ui.allocate_exact_size(egui::vec2(fps_width, 22.), egui::Sense::hover());
                self.draw_fps(ui.painter(), fps_rect);
            });
            ui.separator();
            if self.soil_view.open {
                self.soil_editor_canvas(ui);
            } else {
                self.sketch_toolbar(ui);
                self.canvas(ui);
            }
        });
        self.soil_editor_prompt(ctx);
        if self.smoke {
            self.frames += 1;
            for event in ctx.input(|i| i.events.clone()) {
                if let egui::Event::Screenshot { image, .. } = event {
                    let mut bytes =
                        format!("P6\n{} {}\n255\n", image.size[0], image.size[1]).into_bytes();
                    for p in &image.pixels {
                        bytes.extend_from_slice(&p.to_array()[..3]);
                    }
                    let _ =
                        crate::plan_trace::platform::write("target/slab_contours_smoke.ppm", bytes);
                }
            }
            if !self.capture_requested && self.started.elapsed() > Duration::from_millis(1200) {
                ctx.send_viewport_cmd(egui::ViewportCommand::Screenshot(egui::UserData::default()));
                self.capture_requested = true;
            }
            let smoke_ready = self.mesh_job.is_none()
                && (self.smoke_load_view.is_none() || self.fe_mesh.is_none());
            if smoke_ready && self.frames >= 60 && self.started.elapsed() > Duration::from_secs(3) {
                if !self.contours.is_empty() && !self.smoke_source_preview {
                    assert!(
                        self.mesh_job.is_none() && self.fe_mesh.is_some(),
                        "Smoke mesh failed: {}",
                        self.status
                    );
                }
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            ctx.request_repaint_after(Duration::from_millis(16));
        }
    }
}
fn curved_fixture() -> Vec<Contour> {
    let mut all = vec![
        Contour {
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
            points: vec![],
            curves: vec![Curve::Spline {
                closed: true,
                controls: vec![
                    [-8500., 500.],
                    [-9000., 2500.],
                    [-8500., 3500.],
                    [-5000., 3700.],
                    [-3500., 3100.],
                    [-4000., 1000.],
                    [-5000., -500.],
                    [-5800., 1200.],
                    [-7500., 700.],
                ],
            }],
        },
        Contour {
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
            id: 2,
            parent: None,
            kind: Kind::Slab,
            points: vec![[0., 0.], [6000., 0.], [6000., 4000.], [0., 4000.]],
            curves: vec![],
        },
        Contour {
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
            id: 3,
            parent: None,
            kind: Kind::Slab,
            points: vec![],
            curves: vec![Curve::Arc([
                [10000., 4000.],
                [10000., 2000.],
                [12000., 2500.],
            ])],
        },
    ];
    for c in &mut all {
        c.rebuild().expect("valid analytic smoke contour");
    }
    all
}
#[cfg(not(target_arch = "wasm32"))]
pub fn run() -> eframe::Result {
    let args = std::env::args_os().collect::<Vec<_>>();
    if args.get(1).is_some_and(|s| s == "--soil-prototype") {
        if let Err(e) = soil_prototype_host::run(&args) {
            eprintln!("{e}");
            std::process::exit(1);
        }
        return Ok(());
    }
    if let Some(i) = args
        .iter()
        .position(|v| v == "--audit-floor-mesh" || v == "--audit-partial-floor-mesh")
    {
        let result = (|| -> Result<(), String> {
            let path = PathBuf::from(args.get(i + 1).ok_or("Укажите проект")?);
            let floor = args
                .get(i + 2)
                .ok_or("Укажите ID этажа")?
                .to_string_lossy()
                .parse()
                .map_err(|_| "Некорректный этаж")?;
            let output = PathBuf::from(args.get(i + 3).ok_or("Укажите JSON результата")?);
            project_ui::audit_floor_mesh(
                path,
                floor,
                output,
                args[i] == "--audit-partial-floor-mesh",
            )
        })();
        if let Err(e) = result {
            eprintln!("{e}");
            std::process::exit(1);
        }
        return Ok(());
    }

    let smoke_tools = args.iter().any(|v| v == "--smoke-tools");
    let smoke_measurements = args.iter().any(|v| v == "--smoke-measurements");
    let smoke_properties = args.iter().any(|v| v == "--smoke-properties");
    let smoke_building = smoke_properties || args.iter().any(|v| v == "--smoke-building");
    let smoke_section_toggle = args.iter().any(|v| v == "--smoke-sections");
    let smoke_soil_editor = args.iter().any(|v| v == "--smoke-soil-editor");
    let smoke_source = smoke_soil_editor
        || smoke_section_toggle
        || smoke_building
        || args.iter().any(|v| v == "--smoke-source-3d");
    let smoke_spatial = smoke_source || args.iter().any(|v| v == "--smoke-3d");
    let smoke_load_view = args
        .iter()
        .position(|v| v == "--smoke-load-view" || v == "--smoke-result-view")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.to_string_lossy().parse::<usize>().ok())
        .filter(|n| (0..33).contains(n));
    let smoke_loads = args.iter().any(|v| v == "--smoke-loads") || smoke_load_view.is_some();
    let smoke_grid = args.iter().any(|v| v == "--smoke-grid");
    let smoke_curves = args.iter().any(|v| v == "--smoke-curves") || smoke_grid;
    let smoke = args.iter().any(|v| v == "--smoke-test")
        || smoke_tools
        || smoke_curves
        || smoke_loads
        || smoke_measurements
        || smoke_spatial;
    let project_path = args
        .iter()
        .position(|v| v == "--project")
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from);
    let path = args
        .iter()
        .position(|v| v == "--dxf")
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from);
    let pdf_path = args
        .iter()
        .position(|v| v == "--pdf")
        .and_then(|i| args.get(i + 1))
        .map(PathBuf::from);
    if let Some(i) = args.iter().position(|v| v == "--check") {
        let result = args
            .get(i + 1)
            .ok_or("Укажите DXF после --check".to_owned())
            .and_then(|p| import::read(&PathBuf::from(p)));
        match result {
            Ok(d) => println!(
                "{}: paths={}, labels={}, nodes={}, bounds={:?}, Z=0, skipped={:?}",
                d.name,
                d.paths.len(),
                d.labels.len(),
                d.nodes.len(),
                d.bounds(),
                d.skipped
            ),
            Err(e) => {
                eprintln!("{e}");
                std::process::exit(1)
            }
        }
        return Ok(());
    }
    let fixture = path.is_none() && pdf_path.is_none() && project_path.is_none();
    let options = eframe::NativeOptions {
        vsync: true,
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 24,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1400., 900.])
            .with_icon(
                eframe::icon_data::from_png_bytes(include_bytes!(
                    "../../assets/rconcrete/rconcrete-icon.png"
                ))
                .expect("embedded rconcrete application icon"),
            )
            .with_title("Rconcrete FEM"),
        ..Default::default()
    };
    eframe::run_native(
        "Rconcrete FEM",
        options,
        Box::new(move |cc| {
            super::render_cache::install_with_depth(
                cc,
                Some(eframe::egui_wgpu::wgpu::TextureFormat::Depth24Plus),
            );
            spatial_depth::install(cc);
            result_gpu::install(cc);
            cc.egui_ctx.set_visuals(egui::Visuals::dark());
            let mut app = App {
                smoke,
                smoke_show_tools: smoke_tools,
                smoke_load_view,
                smoke_source_preview: smoke_source,
                smoke_section_toggle,
                ..if smoke {
                    App::default()
                } else {
                    App::new_project()
                }
            };
            if let Some(path) = project_path {
                if let Err(e) = app.load_project_path(path) {
                    app.status = format!("Проект: {e}");
                    if smoke {
                        panic!("{}", app.status);
                    }
                }
                app.smoke = smoke;
                app.smoke_source_preview = smoke_source;
                app.smoke_section_toggle = smoke_section_toggle;
                if smoke_section_toggle {
                    app.drawing_visibility.structure_edges = true;
                }
                app.smoke_load_view = smoke_load_view;
                if smoke_spatial {
                    app.support_height.enabled = true;
                    app.support_height.direction = super::fem::shell::Direction::Both;
                    app.spatial.enabled = true;
                }
                if smoke && smoke_soil_editor {
                    app.smoke_open_ground_editor(&cc.egui_ctx);
                } else if smoke && !smoke_source {
                    app.start_mesh(&cc.egui_ctx);
                }
            } else if let Some(path) = path {
                match import::read(&path) {
                    Ok(d) => app.append_dxf(d),
                    Err(e) => app.status = e,
                }
            } else if let Some(path) = pdf_path {
                match underlay_ui::read_pdf_input(&path) {
                    Ok(result) => app.accept_pdf_import(result),
                    Err(e) => app.status = e,
                }
            } else if smoke {
                let d=import::from_bytes(b"0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n4\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n62\n7\n70\n1\n10\n0\n20\n0\n10\n10000\n20\n0\n10\n10000\n20\n6000\n10\n0\n20\n6000\n0\nCIRCLE\n62\n3\n10\n3000\n20\n3000\n30\n500\n40\n800\n0\nENDSEC\n0\nEOF\n","Проверка DXF").unwrap();
                app.loaded(d);
            }
            if smoke && fixture {
                for (kind, p) in [
                    (
                        Kind::Slab,
                        vec![[500., 500.], [9500., 500.], [9500., 5500.], [500., 5500.]],
                    ),
                    (
                        Kind::Hole,
                        vec![
                            [6000., 2000.],
                            [7500., 2000.],
                            [7500., 3500.],
                            [6000., 3500.],
                        ],
                    ),
                    (
                        Kind::Support,
                        vec![
                            [4200., 2500.],
                            [4700., 2500.],
                            [4700., 3000.],
                            [4200., 3000.],
                        ],
                    ),
                ] {
                    let id = app.next_id;
                    app.next_id += 1;
                    app.contours.push(Contour {
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
                        curves: vec![],
                        parent: if matches!(kind, Kind::Hole | Kind::MeshRegion) {
                            Some(1)
                        } else {
                            None
                        },
                        id,
                        kind,
                        points: p,
                    });
                }
                if smoke_curves {
                    app.contours = curved_fixture();
                    app.next_id = 4;
                    app.drawing = None;
                    app.index = Index::default();
                    app.layers.clear();
                    app.mesh_size = 500.;
                    app.selected = Some(1);
                    app.fit = true;
                }
                if smoke_grid {
                    app.construction_grid = super::grid::Grid {
                        enabled: true,
                        dx: 500.,
                        dy: 750.,
                        ..Default::default()
                    };
                    let b = app.bounds().expect("smoke geometry bounds");
                    app.camera = Camera {
                        center: [(b[0] + b[2]) * 0.5, (b[1] + b[3]) * 0.5],
                        zoom: 0.03,
                        angle: 0.2,
                        plane: None,
                    };
                    app.fit = false;
                }
                if smoke_loads {
                    app.contours.push(Contour {
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
                        id: 4,
                        parent: Some(1),
                        kind: Kind::MeshRegion,
                        curves: vec![],
                        load: None,
                        refinement: Some(model::Refinement {
                            inactive_edges: vec![],

                            name: "Локально 250 мм".into(),
                            size: 250.,
                        }),
                        points: vec![
                            [1000., 1000.],
                            [4000., 1000.],
                            [4000., 4000.],
                            [1000., 4000.],
                        ],
                    });
                    for (id, kind, points, value, case_id) in [
                        (5, Kind::LoadPoint, vec![[3000., 3500.]], 5., 1),
                        (
                            6,
                            Kind::LoadLine,
                            vec![[1000., 4500.], [4000., 4500.], [4500., 3500.]],
                            2.,
                            1,
                        ),
                        (
                            7,
                            Kind::LoadArea,
                            vec![[5000., 600.], [8500., 600.], [8500., 1800.], [5000., 1800.]],
                            0.8,
                            1,
                        ),
                        (8, Kind::LoadPoint, vec![[8000., 4500.]], 99., 2),
                    ] {
                        app.contours.push(Contour {
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
                            id,
                            parent: None,
                            kind,
                            points,
                            curves: vec![],
                            refinement: None,
                            load: Some(super::loads::LoadData {
                                direction: Default::default(),
                                moment: false,
                                additive: false,
                                name: String::new(),
                                mesh_constraint: false,
                                case_id,
                                magnitude: super::loads::Magnitude {
                                    value,
                                    units: super::loads::ForceUnits::TonForce,
                                },
                                closed: kind == Kind::LoadArea,
                            }),
                        });
                    }
                    app.load_cases.push(super::loads::LoadCase::new(2));
                    app.next_load = 3;
                    app.next_id = 9;
                    app.fit = true;
                }
                if smoke_measurements {
                    app.measurements = measurements::Tool::restore(
                        vec![
                            measurements::Measurement {
                                spatial: None,
                                offset: None,
                                id: 1,
                                a: [0., 0.],
                                b: [10000., 0.],
                            },
                            measurements::Measurement {
                                spatial: None,
                                offset: None,
                                id: 2,
                                a: [1000., 1000.],
                                b: [5000., 4000.],
                            },
                        ],
                        true,
                    );
                    app.measurements.open = true;
                }
                if smoke_spatial {
                    app.support_height.enabled = true;
                    app.support_height.direction = super::fem::shell::Direction::Both;
                    app.spatial.enabled = true;
                }
                if !smoke_source {
                    app.start_mesh(&cc.egui_ctx);
                }
                if smoke_tools {
                    app.begin(Kind::Slab);
                }
            }
            if smoke_building {
                app.building.enabled = true;
                app.contours = app
                    .contours
                    .into_iter()
                    .filter(|c| {
                        matches!(c.kind, Kind::Slab | Kind::Hole | Kind::Support | Kind::Beam)
                    })
                    .collect();
                app.sync_building_objects();
                let original = app.contours.clone();
                let max_id = original.iter().map(|c| c.id).max().unwrap_or(0);
                for floor in 2..=3_u64 {
                    app.building.floors.push(building::Floor {
                        id: floor,
                        name: format!("Этаж {floor}"),
                        elevation_mm: (floor - 1) as f64 * 3000.,
                        height_mm: 2500.,
                    });
                    let mut copy = original.clone();
                    for c in &mut copy {
                        c.id += max_id * (floor - 1);
                        c.parent = c.parent.map(|id| id + max_id * (floor - 1));
                        app.building.contour_floors.insert(c.id, floor);
                    }
                    app.contours.extend(copy);
                }
                app.building.active = 2;
                app.selected = None;
                app.selection_ids.clear();
                app.spatial.enabled = true;
                app.spatial.cache_reset();
                app.spatial.pitch = -0.65;
                let _ = std::fs::write(
                    "target/building_smoke_source.json",
                    serde_json::to_vec_pretty(
                        &serde_json::json!({"contours":app.contours,"building":app.building}),
                    )
                    .unwrap(),
                );
                app.next_id = app.contours.iter().map(|c| c.id).max().unwrap_or(0) + 1;
            }
            if smoke_properties {
                app.properties.open = true;
                app.selection_ids = app
                    .contours
                    .iter()
                    .filter(|c| {
                        app.building.owner(c.id) == app.building.active
                            && matches!(c.kind, Kind::Slab | Kind::Support)
                    })
                    .map(|c| c.id)
                    .collect();
            }
            Ok(Box::new(app))
        }),
    )
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn translating_a_load_preserves_analytic_shape_intensity_and_case() {
        let mut app = App::default();
        app.begin(Kind::LoadLine);
        app.choose_tool(Tool::Arc);
        app.sketch_click([0., 0.], false);
        app.sketch_click([500., 500.], false);
        app.sketch_click([1000., 0.], false);
        app.finish();
        let original = app.contours[0].clone();
        let mut shifted = original.clone();
        shifted.translate([100., -200.]).unwrap();
        assert_eq!(shifted.load, original.load);
        assert_eq!(shifted.id, original.id);
        assert_eq!(shifted.points.len(), original.points.len());
        for (a, b) in shifted.points.iter().zip(&original.points) {
            assert!((a[0] - b[0] - 100.).abs() < 1e-8);
            assert!((a[1] - b[1] + 200.).abs() < 1e-8);
        }
        assert_eq!(shifted.curves.len(), 1);
        assert_eq!(shifted.curves[0].controls()[0], [100., -200.]);
    }
    #[test]
    fn load_lock_and_visibility_protect_editing_without_removing_calculation_data() {
        let mut app = App::default();
        app.begin(Kind::LoadPoint);
        app.sketch_click([1000., 1000.], false);
        let id = app.contours[0].id;
        let original = app.contours.clone();
        app.set_kind_lock(Kind::LoadLine, true);
        for kind in [Kind::LoadPoint, Kind::LoadLine, Kind::LoadArea] {
            assert!(app.is_kind_locked(kind));
        }
        app.selected = Some(id);
        app.remove();
        assert_eq!(app.contours, original);
        app.undo();
        assert_eq!(app.contours, original);
        app.set_kind_lock(Kind::LoadPoint, false);
        assert!(!app.is_kind_locked(Kind::LoadArea));
        app.set_load_visibility(false);
        assert!(!app.contour_visible(&app.contours[0]));
        assert!(!app.hit_contour(&app.contours[0], [1000., 1000.], 1.));
        assert!(app.selected.is_none());
        assert_eq!(app.contours, original);
        app.set_load_visibility(true);
        assert!(app.contour_visible(&app.contours[0]));
    }
    #[test]
    fn tree_selection_can_preview_loads_from_multiple_cases_without_changing_active_case() {
        let mut app = App::default();
        app.load_cases
            .push(crate::plan_trace::loads::LoadCase::new(2));
        for (case, position) in [(1, [100., 0.]), (2, [200., 0.]), (2, [300., 0.])] {
            app.active_load = case;
            app.begin(Kind::LoadPoint);
            app.sketch_click(position, false);
        }
        app.active_load = 1;
        app.set_load_visibility(false);
        let ids = app.contours[..2].iter().map(|c| c.id).collect::<Vec<_>>();
        app.select_load_tree(ids.clone());
        app.prune_selection();
        assert_eq!(app.selection_ids.iter().copied().collect::<Vec<_>>(), ids);
        assert_eq!(app.active_load, 1);
        assert_eq!(app.spatial_load_sources().len(), 2);
        assert!(!app.contour_visible(&app.contours[2]));
        app.cancel();
        assert!(app.tree_load_selection.is_empty());
        assert!(app.spatial_load_sources().is_empty());
    }

    #[test]
    fn support_can_finish_after_two_points_or_close_explicitly() {
        let mut app = App::default();
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 0.], false);
        assert!(app.can_finish_sketch());
        app.finish();
        assert_eq!(app.contours.len(), 1);
        assert!(!app.contours[0].closed());
        assert!(app.hit_contour(&app.contours[0], [1000., 0.], 1.));
        assert!(!app.hit_contour(&app.contours[0], [1000., 1000.], 1.));
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 0.], false);
        app.sketch_click([1000., 1000.], false);
        app.sketch_click([0., 0.], true);
        assert!(app.contours[1].closed());
    }
    #[test]
    fn load_shapes_belong_to_case_and_do_not_change_fe_geometry_or_contour_locks() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([6000., 4000.], false);
        let geometry = app.contours.clone();
        app.set_contour_lock(true);
        app.set_kind_lock(Kind::LoadPoint, false);
        app.begin(Kind::LoadPoint);
        app.sketch_click([1000., 1000.], false);
        assert_eq!(app.contours[1].points, vec![[1000., 1000.]]);
        assert_eq!(app.contours[1].load.as_ref().unwrap().case_id, 1);
        app.begin(Kind::LoadLine);
        app.sketch_click([0., 0.], false);
        app.sketch_click([3000., 0.], false);
        app.sketch_click([3000., 2000.], false);
        app.finish();
        assert!(!app.contours[2].closed());
        assert_eq!(app.contours[2].load_label_positions().len(), 2);
        app.begin(Kind::LoadArea);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([4000., 2000.], false);
        assert_eq!(app.contours[3].load_label_positions(), vec![[2000., 1000.]]);
        assert!(model::same_mesh_input(&geometry, &app.contours));
        let data = app.contours[1].load.as_ref().unwrap();
        assert_eq!(
            data.magnitude
                .in_units(super::super::loads::ForceUnits::KiloNewton, 10.),
            10.
        );
        app.load_cases.push(super::super::loads::LoadCase::new(2));
        app.active_load = 2;
        assert!(!app.contour_visible(&app.contours[1]));
        app.begin(Kind::LoadPoint);
        app.sketch_click([2000., 2000.], false);
        assert_eq!(
            app.contours.last().unwrap().load.as_ref().unwrap().case_id,
            2
        );
        assert!(!app.contours_locked);
        assert!(app.contour_visible(app.contours.last().unwrap()));
    }
    #[test]
    fn sizing_region_requires_slab_and_preserves_parent_metadata_and_undo() {
        let mut app = App::default();
        app.begin(Kind::MeshRegion);
        assert!(app.mode.is_none());
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([6000., 4000.], false);
        app.begin(Kind::MeshRegion);
        assert_eq!(app.draft_parent, Some(1));
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([1000., 1000.], false);
        app.sketch_click([3000., 3000.], false);
        assert_eq!(app.contours[1].kind, Kind::MeshRegion);
        assert_eq!(app.contours[1].parent, Some(1));
        assert_eq!(
            app.contours[1].refinement.as_ref().unwrap().size,
            app.mesh_size
        );
        let previous = app.contours.clone();
        app.contours[1].refinement.as_mut().unwrap().name = "Переименована".into();
        assert!(model::same_mesh_input(&previous, &app.contours));
        app.contours[1].refinement.as_mut().unwrap().size = 200.;
        assert!(!model::same_mesh_input(&previous, &app.contours));
        app.selected = Some(1);
        app.remove();
        assert!(app.contours.is_empty());
        app.undo();
        assert_eq!(app.contours.len(), 2);
        app.selection_filter = [false; 14];
        app.select_only(None);
        app.set_kind_lock(Kind::Slab, true);
        app.remove_region(2);
        assert_eq!(app.contours.len(), 2);
        app.set_kind_lock(Kind::Slab, false);
        app.remove_region(2);
        assert_eq!(app.contours.len(), 1);
        assert_eq!(app.contours[0].kind, Kind::Slab);
        app.undo();
        assert_eq!(app.contours.len(), 2);
    }
    #[test]
    fn per_kind_locks_work_before_mesh_and_protect_cascades_and_undo() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([4000., 3000.], false);
        app.begin(Kind::Hole);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([500., 500.], false);
        app.sketch_click([1000., 1000.], false);
        app.set_kind_lock(Kind::Hole, true);
        assert!(app.fe_mesh.is_none());
        assert!(!app.contours_locked);
        app.selected = Some(1);
        app.remove();
        assert_eq!(app.contours.len(), 2);
        app.change_vertex(1, 0, true);
        assert_eq!(app.contours[0].points.len(), 5);
        app.undo();
        assert_eq!(app.contours[0].points.len(), 4);
        app.set_kind_lock(Kind::Slab, true);
        app.begin(Kind::Support);
        assert_eq!(app.mode, Some(Kind::Support));
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([2000., 1000.], false);
        app.sketch_click([2500., 1500.], false);
        app.set_kind_lock(Kind::Support, true);
        assert!(!app.contours_locked);
        app.set_contour_lock(true);
        assert!(app.contours_locked);
        let before = app.contours.clone();
        let history = app.history.len();
        app.undo();
        assert_eq!(app.contours, before);
        assert_eq!(app.history.len(), history);
        app.set_contour_lock(false);
        assert!(app.contour_locks.iter().all(|v| !*v));
        app.undo();
        assert_eq!(app.contours.len(), 2);
    }
    #[test]
    fn edge_arc_keeps_endpoints_has_radius_preview_and_one_undo() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([4000., 3000.], false);
        let before = app.contours.clone();
        let history = app.history.len();
        app.begin_edge_arc(1, 0);
        app.preview_edge_arc([2000., -500.]);
        assert_eq!(app.contours, before);
        assert_eq!(app.history.len(), history);
        assert!((app.arc_placement.as_ref().unwrap().radius.unwrap() - 4250.).abs() < 1e-6);
        app.accept_edge_arc();
        assert!(app.arc_placement.is_none());
        assert_eq!(app.history.len(), history + 1);
        assert!(matches!(
            app.contours[0].curves[0],
            Curve::Arc([[0., 0.], [2000., -500.], [4000., 0.]])
        ));
        app.undo();
        assert_eq!(app.contours, before);
        app.begin_edge_arc(1, 0);
        app.preview_edge_arc([2000., -800.]);
        app.cancel();
        assert_eq!(app.contours, before);
        app.set_contour_lock(true);
        app.begin_edge_arc(1, 0);
        assert!(app.arc_placement.is_none());
    }
    #[test]
    fn construction_grid_nodes_override_line_projections_but_not_geometry_vertices() {
        let mut app = App::default();
        app.construction_grid = super::super::grid::Grid {
            enabled: true,
            dx: 100.,
            dy: 200.,
            ..Default::default()
        };
        assert_eq!(app.snap([98., 198.], None).unwrap().point, [100., 200.]);
        app.contours.push(Contour {
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
            points: vec![[99., 199.], [200., 199.], [200., 300.]],
            curves: vec![],
        });
        assert_eq!(app.snap([100., 200.], None).unwrap().point, [99., 199.]);
        app.contours[0].points = vec![[0., 195.], [300., 195.], [300., 350.]];
        assert_eq!(app.snap([100., 196.], None).unwrap().point, [100., 200.]);
        app.construction_grid.enabled = false;
        assert_eq!(app.snap([100., 196.], None).unwrap().point, [100., 195.]);
        assert!(app.fe_mesh.is_none());
    }
    #[test]
    fn accepted_mesh_locks_geometry_and_manual_unlock_restores_editing() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        let before = app.contours.clone();
        let mesh = super::super::meshing::generate(&before, app.mesh_size).unwrap();
        let (tx, rx) = mpsc::channel();
        app.mesh_job = Some(rx);
        tx.send((before.clone(), app.mesh_size, Ok((mesh, None))))
            .unwrap();
        app.poll_mesh();
        assert!(app.contours_locked && app.fe_mesh.is_some());
        app.begin(Kind::Support);
        assert!(app.mode.is_none());
        app.change_vertex(1, 0, true);
        app.undo();
        app.remove();
        app.finish();
        assert_eq!(app.contours, before);
        app.set_contour_lock(false);
        app.change_vertex(1, 0, true);
        assert_eq!(app.contours[0].points.len(), 5);
        app.poll_mesh();
        assert!(app.fe_mesh.is_none() && !app.contours_locked);
    }
    #[test]
    fn mixed_line_spline_closes_with_spline_for_click_and_enter() {
        for by_click in [false, true] {
            let mut app = App::default();
            app.begin(Kind::Slab);
            app.sketch_click([0., 0.], false);
            app.sketch_click([3000., 0.], false);
            app.choose_tool(Tool::Spline);
            app.sketch_click([3500., 2000.], false);
            app.sketch_click([1500., 3000.], false);
            if by_click {
                app.sketch_click([0., 0.], true);
            } else {
                app.finish();
            }
            assert_eq!(app.contours.len(), 1, "{}", app.status);
            let c = &app.contours[0];
            assert_eq!(c.curves.len(), 2);
            assert!(matches!(c.curves[0], Curve::Line(..)));
            assert!(
                matches!(&c.curves[1], Curve::Spline { controls, closed: false }
                if controls == &vec![[3000.,0.],[3500.,2000.],[1500.,3000.],[0.,0.]])
            );
            let last = c.points.last().unwrap();
            // The final sampled span must bend, rather than following the chord
            // from the last entered control [1500,3000] to the first vertex.
            assert!((last[0] * 3000. - last[1] * 1500.).abs() > 1.);
            app.undo();
            assert!(app.contours.is_empty());
        }
    }
    #[test]
    fn rectangle_can_start_from_first_entered_point_and_switch_is_undoable() {
        for tool in [Tool::Line, Tool::Spline, Tool::Arc, Tool::Ellipse] {
            let mut app = App::default();
            app.begin(Kind::Slab);
            app.choose_tool(tool);
            app.sketch_click([100., 200.], false);
            assert!(app.can_choose_tool(Tool::Rectangle));
            app.choose_tool(Tool::Rectangle);
            assert_eq!(app.curve_controls, vec![[100., 200.]]);
            app.sketch_back();
            assert_eq!(app.tool, tool);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([6100., 4200.], false);
            assert!(app.mode.is_none());
            assert_eq!(
                app.contours[0].points,
                vec![[100., 200.], [6100., 200.], [6100., 4200.], [100., 4200.]]
            );
        }
    }
    #[test]
    fn analytic_geometry_remains_authoritative_after_edits_undo_and_meshing() {
        let mut all = curved_fixture();
        assert_eq!(all[0].handles().len(), 9);
        assert!(all[0].points.len() > 30);
        let mut app = App::default();
        app.contours = all.clone();
        app.change_vertex(1, 2, true);
        assert_eq!(app.contours[0].handles().len(), 10);
        assert!(
            matches!(&app.contours[0].curves[0],Curve::Spline{controls,..} if controls.len()==10)
        );
        app.undo();
        assert_eq!(app.contours, all);
        let before = all[2].points.clone();
        let mid = all[2].handles()[1];
        all[2].move_handle(1, [mid[0] - 100., mid[1]]).unwrap();
        assert!(matches!(all[2].curves[0], Curve::Arc(_)));
        assert_ne!(all[2].points, before);
        all[0].points.clear(); // Cache is deliberately stale: mesh must use the analytic source.
        let mesh = super::super::meshing::generate(&all, 500.).unwrap();
        let rectangle_elements = mesh
            .elements
            .iter()
            .filter(|e| {
                e.nodes.iter().all(|i| {
                    let p = mesh.nodes[*i].xy;
                    p[0] >= -1e-6 && p[0] <= 6000.00001 && p[1] >= -1e-6 && p[1] <= 4000.00001
                })
            })
            .count();
        assert!(
            rectangle_elements > 100,
            "rectangle only has {rectangle_elements} elements"
        );
    }
    #[test]
    fn toolbar_mouse_click_changes_tool_without_adding_geometry() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.begin(Kind::Slab);
        let mut time = 0.;
        let mut frame = |app: &mut App, events: Vec<egui::Event>| {
            time += 0.04;
            let _ = ctx.run(
                egui::RawInput {
                    time: Some(time),
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(900., 700.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        app.sketch_toolbar(ui);
                        app.canvas(ui);
                    });
                },
            );
        };
        frame(&mut app, vec![]);
        frame(&mut app, vec![]);
        let rect = app.toolbar_rect.unwrap();
        let p = rect.min + egui::vec2(59., 21.);
        for pressed in [true, false] {
            frame(
                &mut app,
                vec![
                    egui::Event::PointerMoved(p),
                    egui::Event::PointerButton {
                        pos: p,
                        button: egui::PointerButton::Primary,
                        pressed,
                        modifiers: Default::default(),
                    },
                ],
            );
        }
        assert_eq!(app.tool, Tool::Spline);
        assert!(app.draft.is_empty() && app.curve_controls.is_empty());
        app.cancel();
        frame(&mut app, vec![]);
        assert!(app.toolbar_rect.is_none());
    }
    #[test]
    fn toolbar_shapes_finish_holes_and_mesh_without_extra_endpoint() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([4000., 3000.], false);
        assert!(app.mode.is_none());
        assert_eq!(app.contours[0].points.len(), 4);
        app.begin(Kind::Hole);
        app.choose_tool(Tool::Ellipse);
        app.sketch_click([2000., 1500.], false);
        app.sketch_click([2600., 1500.], false);
        let preview = app.sketch_preview(Some([2000., 1900.]));
        assert!(preview.len() > 30);
        app.sketch_click([2000., 1900.], false);
        assert!(app.mode.is_none());
        assert_eq!(app.contours.len(), 2);
        assert_eq!(app.contours[1].parent, Some(1));
        assert!(super::super::meshing::generate(&app.contours, 400.).is_ok());
        app.undo();
        assert_eq!(app.contours.len(), 1);
    }
    #[test]
    fn arc_spline_tool_switches_and_click_undo_preserve_contour() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.sketch_click([0., 0.], false);
        app.sketch_click([1000., 0.], false);
        app.choose_tool(Tool::Arc);
        app.sketch_click([1500., 500.], false);
        assert!(!app.can_finish_sketch());
        app.sketch_click([1000., 1000.], false);
        assert!(app.draft.len() > 5);
        app.sketch_back();
        assert_eq!(app.draft.len(), 2);
        assert_eq!(app.curve_controls.len(), 2);
        app.sketch_click([1000., 1000.], false);
        app.choose_tool(Tool::Line);
        app.sketch_click([0., 1000.], false);
        app.sketch_click([0., 0.], true);
        assert_eq!(app.contours.len(), 1);
        assert!(app.mode.is_none());
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Spline);
        for p in [[5000., 0.], [6000., 0.], [6000., 1000.], [5000., 1000.]] {
            app.sketch_click(p, false);
        }
        assert!(app.can_finish_sketch());
        app.finish();
        assert_eq!(app.contours.len(), 2);
        app.begin(Kind::Slab);
        app.sketch_click([8000., 0.], false);
        app.choose_tool(Tool::Spline);
        app.sketch_click([9000., 500.], false);
        app.sketch_click([9000., 1000.], false);
        assert!(app.can_finish_sketch());
        app.choose_tool(Tool::Line);
        app.sketch_click([8000., 1000.], false);
        app.finish();
        assert_eq!(app.contours.len(), 3);
    }
    #[test]
    fn stale_meshing_jobs_and_disconnected_workers_do_not_publish_results() {
        let mut app = App::default();
        let (tx, rx) = mpsc::channel();
        app.mesh_job = Some(rx);
        tx.send((vec![], 300., Err("test".into()))).unwrap();
        app.poll_mesh();
        assert!(app.mesh_job.is_none() && app.fe_mesh.is_none());
        assert!(app.status.contains("Параметры изменены"));
        let (tx, rx) = mpsc::channel();
        app.mesh_job = Some(rx);
        drop(tx);
        app.poll_mesh();
        assert!(app.mesh_job.is_none());
        assert!(app.status.contains("прервано"));
    }
    #[test]
    fn cancelled_gmsh_job_does_not_publish_a_completed_mesh() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        let contours = app.contours.clone();
        let mesh = super::super::meshing::generate(&contours, app.mesh_size).unwrap();
        let (tx, rx) = mpsc::channel();
        app.mesh_job = Some(rx);
        let feedback = MeshFeedback::default();
        feedback
            .cancel
            .store(true, std::sync::atomic::Ordering::Relaxed);
        app.mesh_feedback = Some(feedback);
        tx.send((contours, app.mesh_size, Ok((mesh, None))))
            .unwrap();

        app.poll_mesh();

        assert!(app.fe_mesh.is_none() && app.mesh_job.is_none() && app.mesh_feedback.is_none());
        assert_eq!(app.status, "Триангуляция остановлена пользователем");
    }
    #[test]
    fn undo_deleted_wall_and_slab_restores_owners_after_sync() {
        use crate::plan_trace::underlay::{Data, Underlay};
        for with_pdf in [false, true] {
            let mut app = App::new_project();
            app.building.floors.push(building::Floor {
                id: 2,
                name: "Этаж 2".into(),
                elevation_mm: 3000.,
                height_mm: 3000.,
            });
            app.contours = serde_json::from_value(serde_json::json!([
                {"id": 1, "kind": "Slab", "support_open": false, "curves": [], "points": [[0., 0.], [3000., 0.], [3000., 3000.], [0., 3000.]]},
                {"id": 2, "kind": "Hole", "support_open": false, "curves": [], "parent": 1, "points": [[500., 500.], [1000., 500.], [1000., 1000.], [500., 1000.]]},
                {"id": 3, "kind": "Support", "support_open": true, "curves": [], "points": [[0., 0.], [3000., 0.]]}
            ])).unwrap();
            app.building.contour_floors.extend([(1, 2), (2, 2), (3, 2)]);
            if with_pdf {
                app.underlays.push(Underlay {
                    id: 10,
                    name: "test.pdf".into(),
                    visible: true,
                    transparency: 0.,
                    origin: [0., 0.],
                    scale: 1.,
                    scale_y: None,
                    angle_rad: 0.,
                    data: Data::Pdf {
                        width: 1,
                        height: 1,
                        rgba: vec![0, 0, 0, 255],
                        size_mm: [300., 300.],
                        page: 1,
                    },
                });
                app.building.underlay_floors.insert(10, 2);
            }
            app.spatial.enabled = true;
            app.selection_all_floors = true;
            app.sync_building_objects();
            app.sync_stiffness_sources();
            let sections = app.stiffnesses.assignments.clone();
            let materials = app.stiffnesses.contour_materials.clone();
            app.select_scene([1, 3], with_pdf.then_some(10), egui::Modifiers::NONE);
            app.remove();
            app.sync_building_objects();
            app.sync_stiffness_sources();
            assert!(app.contours.is_empty());
            assert!(app.building.contour_floors.is_empty());
            assert!(app.underlays.is_empty());
            app.press_undo_after_control_release();
            assert_eq!(app.building.active, 1);
            assert_eq!(app.contours.len(), 3);
            assert!(app.contours.iter().all(|c| app.building.owner(c.id) == 2));
            assert_eq!(app.stiffnesses.assignments, sections);
            assert_eq!(app.stiffnesses.contour_materials, materials);
            assert_eq!(app.underlays.len(), usize::from(with_pdf));
            if with_pdf {
                assert_eq!(app.building.underlay_floors[&10], 2);
            }
        }
    }
    #[test]
    fn view_motion_keeps_source_revisions_and_edits_refresh_them() {
        let mut app = App::default();
        app.building.enabled = true;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        let id = app.contours[0].id;
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let building_revision = app.building_sync.revision;
        let stiffness_revision = app.stiffness_sources.revision;
        app.camera.center = [150., 200.];
        app.sync_building_objects();
        app.sync_stiffness_sources();
        assert_eq!(app.building_sync.revision, building_revision);
        assert_eq!(app.stiffness_sources.revision, stiffness_revision);

        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors.insert(id, 2);
        let material_id = app.stiffnesses.material_id(id).unwrap();
        app.stiffnesses
            .elastic
            .get_mut(&material_id)
            .unwrap()
            .primary
            .e_mpa = 23000.;
        app.sync_building_objects();
        app.sync_stiffness_sources();
        assert_eq!(app.source_heights(&app.contours[0]), vec![3000.]);
        assert_eq!(app.materials[&id].e_mpa, 23000.);
        assert_ne!(app.building_sync.revision, building_revision);
        assert_ne!(app.stiffness_sources.revision, stiffness_revision);
    }
    #[test]
    fn clipped_lines_and_mesh_stay_bounded_without_polyline_miter_spikes() {
        let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(800., 600.));
        let line = clip_line([-1e12, 300.], [1e12, 300.], rect).unwrap();
        assert!((line[0].x - 0.).abs() < 0.01 && (line[1].x - 800.).abs() < 0.01);
        assert!(clip_line([-100., -20.], [100., -20.], rect).is_none());
        let camera = Camera::default();
        let mut mesh = egui::Mesh::default();
        append_triangle(
            &mut mesh,
            [[-1e9, -1e9], [1e9, -1e9], [0., 1e9]],
            camera,
            rect,
            Color32::GRAY,
        );
        assert!(!mesh.is_empty());
        assert!(mesh
            .vertices
            .iter()
            .all(|v| rect.expand(0.01).contains(v.pos)));
    }
    #[test]
    fn holes_require_slab_selection_and_vertex_edits_are_undoable_and_validated() {
        let mut a = App::default();
        a.begin(Kind::Hole);
        assert!(a.mode.is_none());
        a.begin(Kind::Slab);
        a.draft = vec![[0., 0.], [100., 0.], [100., 100.], [0., 100.]];
        a.finish();
        let slab = a.selected.unwrap();
        a.begin(Kind::Hole);
        assert_eq!(a.draft_parent, Some(slab));
        a.draft = vec![[20., 20.], [40., 20.], [40., 40.], [20., 40.]];
        a.finish();
        let hole = a.selected.unwrap();
        assert_eq!(a.contours[1].parent, Some(slab));
        let before = a.contours.clone();
        a.change_vertex(hole, 0, true);
        assert_eq!(a.contours[1].points.len(), 5);
        a.undo();
        assert_eq!(a.contours, before);
        a.change_vertex(hole, 0, false);
        assert_eq!(a.contours[1].points.len(), 3);
        a.undo();
        assert_eq!(a.contours, before);
        a.selected = Some(slab);
        a.remove();
        assert!(a.contours.is_empty());
        a.undo();
        assert_eq!(a.contours, before);
    }
    #[test]
    fn hidden_layers_do_not_supply_snaps_and_layer_appearance_preserves_source_colors() {
        let mut a = App::default();
        let d = Drawing {
            unit_mm: 1.,
            nodes: vec![[100., 100.]],
            node_layers: vec!["hidden".into()],
            layers: vec![import::Layer {
                name: "hidden".into(),
                color: super::super::source_dxf::Color::Aci(1),
                visible: false,
            }],
            ..Default::default()
        };
        a.loaded(d);
        assert!(a.index.query([100., 100.], 1., &[], &[]).is_none());
        let style = a.layers.get_mut("hidden").unwrap();
        style.visible = true;
        style.color = Some([20, 30, 40]);
        style.width = 3.;
        assert_eq!(style.ink([200, 0, 0]), Color32::from_rgb(20, 30, 40));
        a.rebuild_index();
        assert!(a.index.query([100., 100.], 1., &[], &[]).is_some());
        a.layers.get_mut("hidden").unwrap().color = None;
        assert_eq!(
            a.layers["hidden"].ink([200, 0, 0]),
            Color32::from_rgb(200, 0, 0)
        );
    }
    #[test]
    fn changing_units_scales_background_and_overlays_together() {
        let mut a = App::default();
        a.loaded(Drawing {
            unit_mm: 1.,
            nodes: vec![[1., 2.]],
            ..Default::default()
        });
        a.contours.push(Contour {
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
            curves: vec![],
            parent: None,
            id: 1,
            kind: Kind::Slab,
            points: vec![[1., 2.], [3., 2.], [3., 4.]],
        });
        a.set_units(1000.);
        assert_eq!(a.drawing.as_ref().unwrap().nodes[0], [1000., 2000.]);
        assert_eq!(a.contours[0].points[0], [1000., 2000.]);
        assert_eq!(
            a.index.query([1001., 2000.], 1., &[], &[]).unwrap().point,
            [1000., 2000.]
        );
    }
    #[test]
    fn cancelled_and_disconnected_import_do_not_leave_pending_state_or_destroy_drawing() {
        let mut app = App::default();
        app.drawing = Some(Drawing {
            name: "old".into(),
            ..Default::default()
        });
        let (tx, rx) = mpsc::channel();
        app.load = Some(rx);
        tx.send(Ok(None)).unwrap();
        app.poll_load();
        assert!(app.load.is_none());
        assert_eq!(app.drawing.as_ref().unwrap().name, "old");
        let (tx, rx) = mpsc::channel();
        app.load = Some(rx);
        drop(tx);
        app.poll_load();
        assert!(app.load.is_none());
    }
    #[test]
    fn grid_origin_pick_keeps_the_slab_visible_and_consumes_the_click() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([1000., 1000.], false);
        app.cancel();
        app.select_only(None);
        app.fit = true;
        app.construction_grid.enabled = true;
        let frame = |app: &mut App, events| {
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(600., 400.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| app.canvas(ui));
                },
            )
        };
        frame(&mut app, vec![]);
        let rect = app.dynamic_rect.unwrap();
        let corner = app.camera.screen([1000., 1000.], rect);
        let expected = app.camera.world(corner, rect);
        let slab_color = app.slab_color;
        let fill = Color32::from_rgba_unmultiplied(
            slab_color[0],
            slab_color[1],
            slab_color[2],
            (app.slab_opacity * 255.) as u8,
        );
        let has_slab = |output: egui::FullOutput| {
            output.shapes.iter().any(|entry| match &entry.shape {
                egui::Shape::Mesh(mesh) => mesh.vertices.iter().any(|v| v.color == fill),
                _ => false,
            })
        };
        assert!(has_slab(frame(&mut app, vec![])));

        app.grid_pick_origin = true;
        let press = |pressed| egui::Event::PointerButton {
            pos: corner,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        assert!(has_slab(frame(
            &mut app,
            vec![egui::Event::PointerMoved(corner), press(true)],
        )));
        assert!(app.grid_pick_origin);
        assert!(has_slab(frame(&mut app, vec![press(false)])));
        assert!(!app.grid_pick_origin);
        assert_eq!(app.construction_grid.center, expected);
        assert!(app.selected.is_none() && app.drag.is_none());
    }

    #[test]
    fn mouse_drawing_snaps_to_imported_nodes_and_closes_without_duplicate_endpoint() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.loaded(Drawing {
            nodes: vec![[0., 0.], [100., 0.]],
            ..Default::default()
        });
        app.fit = false;
        app.mode = Some(Kind::Slab);
        let mut time = 0.;
        let mut viewport = Rect::NOTHING;
        let mut frame = |app: &mut App, events: Vec<egui::Event>| {
            time += 0.03;
            let _ = ctx.run(
                egui::RawInput {
                    time: Some(time),
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, Vec2::new(600., 400.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        viewport = ui.available_rect_before_wrap();
                        app.canvas(ui);
                    });
                },
            );
            viewport
        };
        let rect = frame(&mut app, vec![]);
        frame(&mut app, vec![]);
        let click =
            |app: &mut App, frame: &mut dyn FnMut(&mut App, Vec<egui::Event>) -> Rect, p: Pos2| {
                frame(
                    app,
                    vec![
                        egui::Event::PointerMoved(p),
                        egui::Event::PointerButton {
                            pos: p,
                            button: egui::PointerButton::Primary,
                            pressed: true,
                            modifiers: Default::default(),
                        },
                    ],
                );
                frame(
                    app,
                    vec![egui::Event::PointerButton {
                        pos: p,
                        button: egui::PointerButton::Primary,
                        pressed: false,
                        modifiers: Default::default(),
                    }],
                );
            };
        let first = app.camera.screen([0., 0.], rect);
        let second = app.camera.screen([100., 0.], rect);
        let third = app.camera.screen([100., 100.], rect);
        click(&mut app, &mut frame, first + Vec2::new(7., 2.));
        assert_eq!(app.draft, vec![[0., 0.]]);
        click(&mut app, &mut frame, second + Vec2::new(-3., 2.));
        click(&mut app, &mut frame, third);
        assert_eq!(app.draft.len(), 3);
        click(&mut app, &mut frame, first + Vec2::new(3., 2.));
        assert!(app.draft.is_empty());
        assert_eq!(app.contours.len(), 1);
        assert_eq!(app.contours[0].points.len(), 3);
    }
    #[test]
    fn repeated_support_creation_keeps_tool_but_escape_exits_and_failed_finish_keeps_draft() {
        let mut app = App::default();
        app.repeat_sketch = true;
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        app.finish();
        assert!(app.contours.is_empty());
        assert_eq!(app.draft.len(), 1);
        app.sketch_click([1000., 0.], false);
        app.finish();
        assert_eq!(app.contours.len(), 1);
        assert_eq!(app.mode, Some(Kind::Support));
        assert!(app.draft.is_empty());
        app.sketch_click([0., 500.], false);
        app.sketch_click([1000., 500.], false);
        app.finish();
        assert_eq!(app.contours.len(), 2);
        assert_eq!(app.history.len(), 2);
        app.cancel();
        assert!(app.mode.is_none());
    }
    #[test]
    fn camera_inverse_holds_under_rotation_and_zoom() {
        let r = Rect::from_min_size(Pos2::ZERO, Vec2::new(900., 700.));
        let c = Camera {
            center: [10000., -8000.],
            zoom: 0.3,
            angle: 0.7,
            plane: None,
        };
        let p = [12000., -9000.];
        let q = c.world(c.screen(p, r), r);
        assert!((p[0] - q[0]).hypot(p[1] - q[1]) < 0.001);
    }
}

#[cfg(target_arch = "wasm32")]
thread_local! {
    static WEB_RUNNER: std::cell::RefCell<Option<eframe::WebRunner>> = const {
        std::cell::RefCell::new(None)
    };
}

#[cfg(target_arch = "wasm32")]
pub fn stop_web() {
    if let Some(runner) = WEB_RUNNER.take() {
        runner.destroy();
    }
}

#[cfg(target_arch = "wasm32")]
pub async fn start_web(webgl2: bool) -> Result<(), wasm_bindgen::JsValue> {
    use eframe::egui_wgpu::{wgpu, WgpuSetupCreateNew};
    use wasm_bindgen::JsCast;

    stop_web();
    let canvas = web_sys::window()
        .unwrap()
        .document()
        .unwrap()
        .get_element_by_id("canvas")
        .unwrap()
        .dyn_into::<web_sys::HtmlCanvasElement>()?;
    let mut setup = WgpuSetupCreateNew::default();
    let expected = if webgl2 {
        setup.instance_descriptor.backends = wgpu::Backends::GL;
        setup.device_descriptor = std::sync::Arc::new(|adapter| wgpu::DeviceDescriptor {
            label: Some("FEM WebGL2 device"),
            required_limits: wgpu::Limits {
                max_texture_dimension_2d: adapter.limits().max_texture_dimension_2d.min(8192),
                ..wgpu::Limits::downlevel_webgl2_defaults()
            },
            ..Default::default()
        });
        wgpu::Backend::Gl
    } else {
        setup.instance_descriptor.backends = wgpu::Backends::BROWSER_WEBGPU;
        wgpu::Backend::BrowserWebGpu
    };
    let mut options = eframe::WebOptions {
        depth_buffer: 24,
        ..Default::default()
    };
    options.wgpu_options.wgpu_setup = setup.into();

    let runner = eframe::WebRunner::new();
    let result = runner
        .start(
            canvas,
            options,
            Box::new(move |cc| {
                if !cc
                    .wgpu_render_state
                    .as_ref()
                    .is_some_and(|state| state.adapter.get_info().backend == expected)
                {
                    return Err("Не удалось выбрать запрошенный графический бэкенд".into());
                }
                super::render_cache::install_with_depth(cc, Some(wgpu::TextureFormat::Depth24Plus));
                spatial_depth::install(cc);
                result_gpu::install(cc);
                cc.egui_ctx.set_visuals(egui::Visuals::dark());
                Ok(Box::new(App::new_project()))
            }),
        )
        .await;
    if result.is_ok() {
        WEB_RUNNER.replace(Some(runner));
    } else {
        runner.destroy();
    }
    result
}

#[cfg(test)]
mod column_joint_regression {
    use super::*;
    #[test]
    fn supplied_two_level_columns_solve_with_and_without_rigid_bodies() {
        for rigid in [false, true] {
            let mut app = App::default();
            app.load_project_path(std::path::PathBuf::from(
                "tests/fixtures/columns_two_levels.slab.zip",
            ))
            .unwrap();
            app.sync_stiffness_sources();
            app.support_height.rigid_columns = rigid;
            app.support_height.automatic_building_bottom = true;
            app.support_height.automatic_building_top = true;
            let all = app.contours.clone();
            let mut layout = app.building_layout();
            let sources =
                crate::plan_trace::columns::meshing_sources(&all, &app.support_height, &mut layout);
            let mut mesh = crate::plan_trace::building_mesh::generate(
                &sources,
                app.mesh_size,
                layout.unwrap(),
                app.mesh_options,
            )
            .unwrap();
            crate::plan_trace::columns::tag_mesh(&mut mesh, &all, &app.support_height);
            let result = crate::plan_trace::fem::solve_with_support_height(
                &mesh,
                &all,
                &app.materials,
                &app.load_cases,
                app.gravity,
                app.mesh_size,
                crate::plan_trace::fem::ElementType::Dkt,
                &app.support_height,
            )
            .unwrap_or_else(|e| panic!("rigid={rigid}: {e}"));
            let case = &result.cases[0];
            assert!(
                case.balance_kn.abs() < 1e-6,
                "rigid={rigid}, balance={}",
                case.balance_kn
            );
            assert!(case.relative_residual < 1e-6);
            let shell = case.shell.as_ref().unwrap();
            let applied = case.applied_kn;
            let (scene, _) = crate::plan_trace::fem::lira_model(
                &mesh,
                &all,
                &app.materials,
                &app.load_cases,
                app.gravity,
                app.mesh_size,
                &app.support_height,
            )
            .unwrap();
            assert_eq!(shell.restrained_reactions_kn.len(), scene.xyz.len());
            assert!(
                (shell
                    .restrained_reactions_kn
                    .iter()
                    .map(|r| r[2])
                    .sum::<f64>()
                    - applied)
                    .abs()
                    < 1e-6
            );
            for (&id, nodes) in &scene.support_nodes {
                if app.support_height.columns.contains_key(&id)
                    && nodes.iter().any(|n| scene.fixed.contains(&(3 * n + 2)))
                {
                    assert!(nodes
                        .iter()
                        .any(|&n| shell.restrained_reactions_kn[n][2].abs() > 1e-4));
                }
            }
            assert!(
                app.support_height.columns.keys().all(|id| shell
                    .support_reactions_kn
                    .get(id)
                    .is_some_and(|r| r[2].abs() > 1e-4)),
                "rigid={rigid}, reactions={:?}",
                shell.support_reactions_kn
            );
            assert!(shell.beam_forces.iter().all(|b| b.length_m > 0.
                && b.sections()
                    .iter()
                    .all(|(_, s)| s.iter().all(|v| v.is_finite()))));
            app.analysis = Some(result);
            app.fe_mesh = Some(mesh);
            app.spatial.enabled = true;
            app.result_field = 6;
            app.sum_reactions = false;
            let nodal = app.spatial_point_results(&scene);
            assert!(!nodal.is_empty());
            assert!(nodal
                .iter()
                .any(|(n, _)| scene.fixed.contains(&(3 * n + 2))));
            assert!(nodal
                .iter()
                .any(|(n, _)| !scene.fixed.contains(&(3 * n + 2))));
            let foundation_sum = nodal
                .iter()
                .filter(|(n, _)| scene.fixed.contains(&(3 * n + 2)))
                .map(|(_, v)| *v)
                .sum::<f64>();
            assert!((foundation_sum + applied * app.result_display_factor()).abs() < 1e-6);
            app.sum_reactions = true;
            let totals = app.spatial_point_results(&scene);
            assert!(totals
                .iter()
                .any(|(n, _)| scene.fixed.contains(&(3 * n + 2))));
            assert!(totals
                .iter()
                .any(|(n, _)| !scene.fixed.contains(&(3 * n + 2))));
        }
    }
}

fn compact_sidebars(ctx: &egui::Context) -> bool {
    // Keep compact panel state separate so a resize does not overwrite desktop widths.
    ctx.screen_rect().width() < 325. + 390. + reserved_canvas_width(ctx)
}

fn reserved_canvas_width(ctx: &egui::Context) -> f32 {
    (ctx.screen_rect().width() * 0.4).min(240.)
}

// ComboBox begins its own layout before its final size is known to the wrapped row.
fn bottom_selector_wrap(ui: &mut egui::Ui, width: f32) {
    if ui.available_size_before_wrap().x < width && ui.max_rect().width() >= width {
        ui.end_row();
    }
}

#[cfg(test)]
mod responsive_layout_tests {
    use super::*;

    #[test]
    fn expanded_bar_sections_keep_the_side_panel_width() {
        use crate::plan_trace::stiffness::{beam_material, Family};
        let ctx = egui::Context::default();
        ctx.style_mut(|style| style.animation_time = 0.);
        let mut app = App::default();
        app.left_open = true;
        let bar_name = "Сечение балки с длинным импортированным названием ".repeat(6);
        let id = app.stiffnesses.add(Family::Beam, beam_material());
        app.stiffnesses.entries.get_mut(&id).unwrap().name = bar_name.clone();
        let mut frame = |events| {
            let mut width = 0.;
            let output = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1440., 900.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    app.sidebar(ctx);
                    width = ctx.available_rect().left();
                },
            );
            (width, output)
        };
        let _ = frame(vec![]);
        let (before, _) = frame(vec![]);
        for label in [
            "Материалы и жёсткости",
            "Жёсткости стержней",
            bar_name.as_str(),
        ] {
            let (_, output) = frame(vec![]);
            let pos = output
                .shapes
                .iter()
                .find_map(|shape| match &shape.shape {
                    egui::Shape::Text(text) if text.galley.job.text == label => {
                        Some(text.pos + egui::vec2(6., text.galley.size().y * 0.5))
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("missing header {label}"));
            for pressed in [true, false] {
                frame(vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed,
                        modifiers: egui::Modifiers::NONE,
                    },
                ]);
            }
        }
        for _ in 0..8 {
            let (width, output) = frame(vec![]);
            assert!(
                (width - before).abs() <= 1.,
                "sidebar width {before} → {width}"
            );
            assert!(
                output.shapes.iter().any(|shape| matches!(&shape.shape,
                    egui::Shape::Text(text) if text.galley.job.text == "Балка"
                )),
                "the section editor must be visible"
            );
        }
    }

    #[test]
    fn compact_sidebars_leave_room_for_canvas_without_overwriting_desktop_widths() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.left_open = true;
        app.right_open = true;
        let desktop_widths = [("trace_controls", 475.), ("contour_lock_panel", 460.)];
        ctx.data_mut(|data| {
            for (id, width) in desktop_widths {
                let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(width, 720.));
                data.insert_persisted(egui::Id::new(id), egui::panel::PanelState { rect });
            }
        });
        for (width, canvas_width) in [(1440., 505.), (700., 240.), (1440., 505.)] {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(width, 720.))),
                    ..Default::default()
                },
                |ctx| {
                    app.bottom_bar(ctx);
                    app.sidebar(ctx);
                    app.right_sidebar(ctx);
                    assert!((ctx.available_rect().width() - canvas_width).abs() <= 1.);
                },
            );
        }
    }

    #[test]
    fn bottom_bar_wraps_full_selectors_in_a_narrow_building_view() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.load_project_path(std::path::PathBuf::from(
            "tests/fixtures/columns_two_levels.slab.zip",
        ))
        .unwrap();
        app.input_locked = true;
        let screen = Rect::from_min_size(Pos2::ZERO, egui::vec2(700., 720.));
        let mut frame = || {
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(screen),
                    ..Default::default()
                },
                |ctx| app.bottom_bar(ctx),
            )
        };
        let _ = frame();
        let out = frame();
        let selector = |label: &str| {
            let (text, clip_rect) = out
                .shapes
                .iter()
                .find_map(|shape| match &shape.shape {
                    egui::Shape::Text(text) if text.galley.job.text == label => {
                        Some((text, shape.clip_rect))
                    }
                    _ => None,
                })
                .unwrap_or_else(|| panic!("missing selector {label}"));
            assert!(!text.galley.elided, "truncated selector {label}");
            let text_rect = Rect::from_min_size(text.pos, text.galley.size());
            assert!(clip_rect.intersect(screen).contains_rect(text_rect));
            text.pos.y
        };
        let floor_row = selector("Этаж 1");
        let load_row = selector(&app.active_load_name());
        assert!(load_row > floor_row);
        assert!((selector(app.combination_variant.label()) - load_row).abs() <= 1.);
    }
}

#[cfg(test)]
mod storey_support_regression {
    use super::*;
    #[test]
    fn coincident_storey_walls_have_free_ends_and_generate_above_support() {
        let mut app = App::default();
        app.load_project_path(std::path::PathBuf::from(
            "tests/fixtures/columns_two_levels.slab.zip",
        ))
        .unwrap();
        app.sync_stiffness_sources();
        app.input_locked = false;
        app.set_contour_lock(false);
        // Move both coincident walls' ends inside the plate, retaining their floors.
        for c in app.contours.iter_mut().filter(|c| c.kind == Kind::Support) {
            c.points = vec![[500., 3000.], [5500., 3000.]];
        }
        app.generate_punching_contours(Some(11), punching::SlabSide::Above);
        let generated = app
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Punching)
            .collect::<Vec<_>>();
        assert_eq!(generated.len(), 2, "{}", app.status);
        assert!(generated
            .iter()
            .all(|c| c.parent == Some(1) && app.building.owner(c.id) == 2));
        let before = app.contours.len();
        app.generate_punching_contours(Some(7), punching::SlabSide::Above);
        assert_eq!(app.contours.len(), before + 1, "{}", app.status);
        assert!(app.contours.last().unwrap().parent == Some(1));
        app.generate_punching_contours(Some(2), punching::SlabSide::Above);
        assert_eq!(app.contours.len(), before + 1); // no slab above the highest column
    }
    #[test]
    fn punching_uses_actual_top_and_bottom_slab_elevations() {
        use punching::SlabSide;
        let mut app = App::default();
        app.load_project_path(std::path::PathBuf::from(
            "tests/fixtures/columns_two_levels.slab.zip",
        ))
        .unwrap();
        app.set_contour_lock(false);
        let upper = app.contours.iter_mut().find(|c| c.id == 1).unwrap();
        let mut lower = upper.clone();
        lower.id = app.next_id;
        app.next_id += 1;
        lower.placement.slab.offset_mm = -100.;
        upper.placement.slab.offset_mm = 150.;
        let lower_id = lower.id;
        app.building.contour_floors.insert(lower_id, 1);
        app.contours.push(lower);
        for c in app.contours.iter_mut().filter(|c| matches!(c.id, 7 | 11)) {
            c.placement.bottom.offset_mm = -100.;
            c.placement.top.offset_mm = 150.;
            if c.kind == Kind::Support {
                c.points = vec![[500., 3000.], [5500., 3000.]];
            }
        }
        app.sync_stiffness_sources();
        for (member, count) in [(7, 1), (11, 2)] {
            for (side, slab, z) in [
                (SlabSide::Above, 1, 3150.),
                (SlabSide::Below, lower_id, -100.),
            ] {
                let before = app.contours.len();
                app.generate_punching_contours(Some(member), side);
                let cuts = &app.contours[before..];
                assert_eq!(cuts.len(), count, "{}", app.status);
                let layout = app.building_layout().unwrap();
                for cut in cuts {
                    assert_eq!(cut.parent, Some(slab));
                    assert_eq!(app.source_heights(cut), vec![z]);
                    assert_eq!(layout.source_z(cut, &app.contours), z);
                }
            }
        }
    }
    #[test]
    fn support_storey_filter_changes_view_only_and_ignores_3d() {
        let mut app = App::default();
        app.load_project_path(std::path::PathBuf::from(
            "tests/fixtures/columns_two_levels.slab.zip",
        ))
        .unwrap();
        app.building.active = 2;
        app.spatial.enabled = false;
        for (mode, expected) in [(0, (true, false)), (1, (false, true)), (2, (true, true))] {
            app.drawing_visibility.support_storeys = mode;
            let current = app.contours.iter().find(|c| c.id == 2).unwrap();
            let previous = app.contours.iter().find(|c| c.id == 7).unwrap();
            assert_eq!(
                (app.on_view_floor(current), app.on_view_floor(previous)),
                expected
            );
            assert!(app.on_active_floor(current) && !app.on_active_floor(previous));
        }
        app.spatial.enabled = true;
        assert!(app
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column)
            .all(|c| app.on_view_floor(c)));
    }
}

impl App {
    fn restraint_buttons(
        &mut self,
        ctx: &egui::Context,
        rect: Rect,
        project: impl Fn(Point, f64) -> Pos2,
    ) {
        if !self.building.enabled
            || self.mode.is_some()
            || self.drag.is_some()
            || self.scaling_op.is_some()
            || self.clipboard_op.is_some()
        {
            return;
        }
        let Some(id) = self.selected else {
            return;
        };
        let Some(c) = self
            .contours
            .iter()
            .find(|c| c.id == id && c.kind.is_restraint() && self.contour_visible(c))
        else {
            return;
        };
        let Some(&point) = c.points.first() else {
            return;
        };
        let mut mask = c.restraint_mask;
        let z = self.source_heights(c)[0];
        let anchor = project(point, z);
        if !rect.contains(anchor) {
            return;
        }
        let pos = Pos2::new(
            (anchor.x + 10.).min((rect.right() - 120.).max(rect.left())),
            (anchor.y + 8.).min((rect.bottom() - 66.).max(rect.top())),
        );
        egui::Area::new(egui::Id::new(("restraint_dofs", id)))
            .order(egui::Order::Foreground)
            .fixed_pos(pos)
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style()).show(ui, |ui| {
                    ui.add_enabled_ui(
                        !self.input_locked
                            && self.analysis_job.is_none()
                            && self.alternative_job.is_none(),
                        |ui| {
                            for row in 0..2 {
                                ui.horizontal(|ui| {
                                    for col in 0..3 {
                                        let i = 3 * row + col;
                                        let bit = 1 << i;
                                        let name = ["X", "Y", "Z", "Ux", "Uy", "Uz"][i];
                                        if ui
                                            .selectable_label(mask & bit != 0, name)
                                            .on_hover_text(if row == 0 {
                                                "Закрепить перемещение вдоль глобальной оси"
                                            } else {
                                                "Закрепить поворот вокруг глобальной оси"
                                            })
                                            .clicked()
                                        {
                                            mask ^= bit;
                                        }
                                    }
                                });
                            }
                        },
                    );
                });
            });
        if mask != c.restraint_mask {
            self.history.push(self.contours.clone());
            self.contours
                .iter_mut()
                .find(|c| c.id == id)
                .unwrap()
                .restraint_mask = mask;
            self.poll_analysis();
            self.alternative_cache = Default::default();
            self.spatial.cache_reset();
            self.invalidate_scene_buffers();
            self.status = "Закрепления опоры изменены".into();
        }
    }
}

#[cfg(test)]
mod explicit_restraint_regression {
    use super::*;
    #[test]
    fn rigid_link_is_a_two_point_building_contour_without_stiffness() {
        let mut app = App::default();
        app.begin(Kind::RigidLink);
        assert!(app.mode.is_none());
        app.building.enabled = true;
        app.begin(Kind::RigidLink);
        assert!(!app.can_choose_tool(Tool::Arc));
        app.sketch_click([0., 0.], false);
        app.sketch_click([0., 0.], true);
        assert_eq!(app.draft.len(), 1);
        app.sketch_click([300., 0.], false);
        app.sync_building_objects();
        app.sync_stiffness_sources();

        let c = &app.contours[0];
        assert_eq!(c.kind, Kind::RigidLink);
        assert!(!c.closed());
        assert!(c.is_mesh_input());
        assert!(!c.can_insert(0));
        assert!(app.selectable(c));
        assert!(!app.stiffnesses.assignments.contains_key(&c.id));
        assert_eq!(
            app.source_heights(c),
            app.building.extent(app.building.active)
        );
    }

    #[test]
    fn point_restraint_creates_an_exact_mesh_node_and_uses_its_mask() {
        use crate::plan_trace::{building_mesh, columns, fem};
        use object_properties::{Change, Item};
        let mut app = App::new_project();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1500.], false);
        let point = [743., 581.];
        app.begin(Kind::PointRestraint);
        app.sketch_click(point, false);
        let id = app.contours.last().unwrap().id;
        assert_eq!(app.contours.last().unwrap().kind, Kind::PointRestraint);
        app.property_apply(
            &[Item::Contour(id)],
            Change::RestraintDof(1, true),
            egui::Id::new("x"),
        );
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let mut layout = app.building_layout();
        let sources = columns::meshing_sources(&app.contours, &app.support_height, &mut layout);
        let mesh =
            building_mesh::generate(&sources, 500., layout.unwrap(), app.mesh_options).unwrap();
        let (scene, _) = fem::lira_model(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            500.,
            &app.support_height,
        )
        .unwrap();
        let nodes = &scene.support_nodes[&id];
        assert_eq!(nodes.len(), 1);
        let n = nodes[0];
        assert_eq!(scene.xyz[n], [point[0], point[1], 0.]);
        assert!(scene.fixed.contains(&(3 * n)));
        assert!(!scene.fixed.contains(&(3 * n + 1)));
        assert!(scene.fixed.contains(&(3 * n + 2)));
    }

    #[test]
    fn storey_restraint_enforces_all_six_components_without_springs() {
        let mut app = App::default();
        app.building.enabled = true;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1500.], false);
        app.begin(Kind::Restraint);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 0.], false);
        app.finish();
        let c = app
            .contours
            .iter_mut()
            .find(|c| c.kind == Kind::Restraint)
            .expect("support created");
        c.restraint_mask = 63;
        let id = c.id;
        app.load_cases[0].kind = crate::plan_trace::loads::LoadType::SelfWeight;
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let mut layout = app.building_layout();
        let sources = crate::plan_trace::columns::meshing_sources(
            &app.contours,
            &app.support_height,
            &mut layout,
        );
        let mesh = crate::plan_trace::building_mesh::generate(
            &sources,
            500.,
            layout.unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let (scene, _) = crate::plan_trace::fem::lira_model(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            500.,
            &app.support_height,
        )
        .unwrap();
        let nodes = &scene.support_nodes[&id];
        assert!(nodes.len() >= 3);
        assert!(nodes.iter().all(|&n| scene.xyz[n][1].abs() < 1e-6));
        assert_eq!(scene.fixed.len(), 3 * nodes.len());
        assert_eq!(scene.fixed_rotations.len(), 3 * nodes.len());
        let r = crate::plan_trace::fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            500.,
            crate::plan_trace::fem::ElementType::Dkt,
            &app.support_height,
        )
        .unwrap();
        let r = &r.cases[0];
        assert!(r.applied_kn > 0. && r.balance_kn.abs() < 1e-6 && r.relative_residual < 1e-6);
        let shell = r.shell.as_ref().unwrap();
        for &n in nodes {
            assert!(shell.translations_mm[n].iter().all(|v| v.abs() < 1e-8));
            assert!(shell.rotations_rad[n].iter().all(|v| v.abs() < 1e-10));
        }
    }
}

#[cfg(test)]
mod modal_regression {
    use super::*;

    #[test]
    fn global_buckling_on_saved_fixture_scales_with_reference_load() {
        use crate::plan_trace::fem::BucklingRequest;
        let mut app = App::default();
        app.load_project_path("tests/fixtures/columns_two_levels.slab.zip".into())
            .unwrap();
        app.support_height.automatic_building_bottom = true;
        app.support_height.building_bottom_mask = 63;
        app.mesh_size = 1000.;
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
        let mut input = app.analysis_key();
        input.buckling = Some(BucklingRequest {
            weights: vec![(app.load_cases[0].id, 1.)],
            caption: "Проверка собственного веса".into(),
            forms: 1,
            max_factor: 1e8,
        });
        let input_json = serde_json::to_vec(&input).unwrap();
        let mut input: fem_ui::Input = serde_json::from_slice(&input_json).unwrap();
        let first = fem_ui::solve_snapshot(&mesh, &input, &app.load_cases).unwrap();
        let mode = &first.buckling.as_ref().unwrap().as_ref().unwrap().modes[0];
        assert!(mode.factor.is_finite() && mode.factor > 0.);
        let shell = first.cases[0].shell.as_ref().unwrap();
        assert_eq!(mode.shape.len(), shell.translations_mm.len());
        assert!(
            (mode
                .shape
                .iter()
                .map(|v| v[0].hypot(v[1]).hypot(v[2]))
                .fold(0., f64::max)
                - 1.)
                .abs()
                < 1e-12
        );
        let first_factor = mode.factor;
        let request = input.buckling.as_mut().unwrap();
        request.weights[0].1 = 2.;
        request.max_factor = 0.75 * first_factor;
        let second = fem_ui::solve_snapshot(&mesh, &input, &app.load_cases).unwrap();
        let second_factor = second.buckling.as_ref().unwrap().as_ref().unwrap().modes[0].factor;
        assert!((first_factor / second_factor - 2.).abs() < 1e-5);
        assert_eq!(first.cases[0].w_mm, second.cases[0].w_mm);
        input.buckling.as_mut().unwrap().max_factor = 0.25 * first_factor;
        let limited = fem_ui::solve_snapshot(&mesh, &input, &app.load_cases).unwrap();
        assert!(limited.buckling.unwrap().unwrap().modes.is_empty());
    }

    #[test]
    fn spatial_lengths_run_through_saved_building_and_keep_static_solution() {
        use crate::plan_trace::fem::shell::LengthMethod;
        let mut app = App::default();
        app.load_project_path("tests/fixtures/columns_two_levels.slab.zip".into())
            .unwrap();
        for contour in &mut app.contours {
            contour.calculate_effective_length = matches!(contour.id, 7 | 11);
        }
        app.support_height.automatic_building_bottom = true;
        app.support_height.building_bottom_mask = 63;
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
        app.mesh_built_size = app.mesh_size;
        let plane = fem_ui::solve_snapshot(&mesh, &app.analysis_key(), &app.load_cases).unwrap();
        let old_rows = plane.lengths.as_ref().unwrap().as_ref().unwrap();
        assert!(old_rows.iter().any(|r| r.id == 7 && r.axis == "y"));
        assert!(old_rows
            .iter()
            .any(|r| r.id == 11 && r.effective_m.is_some()));

        app.support_height.length_method = LengthMethod::Spatial;
        let snapshot = serde_json::to_vec(&app.analysis_key()).unwrap();
        let key = serde_json::from_slice(&snapshot).unwrap();
        let spatial = fem_ui::solve_snapshot(&mesh, &key, &app.load_cases).unwrap();
        let rows = spatial.lengths.as_ref().unwrap().as_ref().unwrap();
        for id in [7, 11] {
            assert!(
                rows.iter().any(|r| {
                    r.id == id && r.critical_kn.is_some_and(|n| n > 0.) && r.diagnostic.is_some()
                }),
                "no critical force for contour {id}: {:?}",
                rows.iter()
                    .filter(|r| r.id == id)
                    .map(|r| &r.diagnostic)
                    .collect::<Vec<_>>()
            );
        }
        assert_eq!(plane.cases[0].w_mm, spatial.cases[0].w_mm);
        let mut restored = App::default();
        restored
            .apply_project(app.project_snapshot(), "spatial.slab.zip".into())
            .unwrap();
        assert_eq!(restored.support_height.length_method, LengthMethod::Spatial);
    }

    #[test]
    #[ignore = "Измерение подстановок на проекте здания"]
    fn wall_probe_benchmark() {
        let mut app = App::default();
        app.load_project_path(std::env::var_os("MINI_FEM_MODAL_PROJECT").unwrap().into())
            .unwrap();
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
        let start = web_time::Instant::now();
        crate::plan_trace::fem::progress::install(|stage, seconds| {
            eprintln!("{seconds:.3}s {stage}");
        });
        let result = crate::plan_trace::fem::solve_with_modal(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_size,
            app.element_type,
            &app.support_height,
            None,
            true,
            None,
        )
        .unwrap();
        let rows = result.lengths.unwrap().unwrap();
        eprintln!(
            "length benchmark: results={}, valid={}, total={:.3}s",
            rows.len(),
            rows.iter().filter(|r| r.effective_m.is_some()).count(),
            start.elapsed().as_secs_f64()
        );
        assert!(!rows.is_empty());
    }

    #[test]
    #[ignore = "Полный спектральный расчёт схемы занимает несколько минут в debug"]
    fn modal_mass_scaling() {
        let path = std::env::var_os("MINI_FEM_MODAL_PROJECT")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|| "tests/fixtures/columns_two_levels.slab.zip".into());
        let mut app = App::default();
        app.load_project_path(path).unwrap();
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
        let weights = app.combinations[0]
            .weights(
                &app.load_cases,
                crate::plan_trace::load_combinations::Variant::Normative,
            )
            .unwrap();
        let run = |weights: &[(u64, f64)]| {
            crate::plan_trace::fem::shell::frequencies(
                &mesh,
                &app.contours,
                &app.materials,
                &app.load_cases,
                weights,
                app.gravity,
                app.mesh_size,
                app.element_type,
                &app.support_height,
            )
            .unwrap()
        };
        let (mass, frequencies) = run(&weights);
        let (double_mass, slower) = run(&weights
            .iter()
            .map(|&(id, w)| (id, 2. * w))
            .collect::<Vec<_>>());
        println!("mass={mass}, frequencies={frequencies:?}");
        assert!(mass > 0. && (double_mass / mass - 2.).abs() < 1e-10);
        assert_eq!(frequencies.len(), 10);
        for (f, g) in frequencies.iter().zip(slower) {
            assert!((f.hz / g.hz - 2_f64.sqrt()).abs() < 1e-7);
            assert!(
                (f.shape
                    .iter()
                    .map(|p| p.iter().map(|x| x * x).sum::<f64>().sqrt())
                    .fold(0., f64::max)
                    - 1.)
                    .abs()
                    < 1e-10
            );
        }
    }
}
