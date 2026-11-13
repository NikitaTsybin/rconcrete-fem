use super::*;
use crate::plan_trace::fem;
use mini_fem::soil::{Borehole, Options as SolverOptions};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

#[path = "soil_editor.rs"]
mod editor;
use editor::Editor;

#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
struct SoilMaterial {
    e: f64,
    gamma: f64,
}

#[derive(Clone, Default, Serialize, Deserialize)]
pub(super) struct Ground {
    name: String,
    visible: bool,
    holes: Vec<Borehole>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    materials: BTreeMap<u32, SoilMaterial>,
    ground: [f64; 3],
    model: [f64; 3],
    angle: f64,
    #[serde(default)]
    pressures: Vec<Pressure>,
    #[serde(default)]
    pressure_reference: String,
    #[serde(default)]
    pressure_z: f64,
    #[serde(default)]
    pressure_foundation: u64,
}
impl Ground {
    fn placed(&self) -> Vec<Borehole> {
        let (s, c) = self.angle.to_radians().sin_cos();
        let mut holes = self.holes.clone();
        for h in &mut holes {
            let x = h.xy[0] - self.ground[0];
            let y = h.xy[1] - self.ground[1];
            h.xy = [self.model[0] + c * x - s * y, self.model[1] + s * x + c * y];
            let dz = self.model[2] - self.ground[2];
            h.mouth_z += dz;
            for layer in &mut h.layers {
                layer.bottom_z += dz;
                if let Some(m) = layer.material_id.and_then(|id| self.materials.get(&id)) {
                    layer.e = m.e;
                    layer.gamma = m.gamma;
                }
            }
        }
        holes
    }
}

#[derive(Clone, Serialize, Deserialize)]
#[serde(default)]
pub(super) struct Options {
    pub combination: u64,
    pub variant: crate::plan_trace::load_combinations::Variant,
    grounds: Vec<Ground>,
    selected: usize,
    #[serde(rename = "slab", skip_serializing_if = "zero_id")]
    legacy_slab: u64,
    foundation: u64,
    horizontal_supports: bool,
    pub discretization: Discretization,
    step: f64,
    solver: SolverOptions,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            combination: 1,
            variant: crate::plan_trace::load_combinations::Variant::NormativeLong,
            grounds: Vec::new(),
            selected: 0,
            legacy_slab: 0,
            foundation: 0,
            horizontal_supports: true,
            discretization: Discretization::Regular,
            step: 1000.,
            solver: SolverOptions {
                max_depth: 16.,
                relaxation: 0.8,
                tolerance: 0.02,
                max_iterations: 80,
                ..Default::default()
            },
        }
    }
}
#[derive(Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub(super) enum Discretization {
    Regular,
    FiniteElements,
}
impl Discretization {
    fn label(self) -> &'static str {
        match self {
            Self::Regular => "Регулярная сетка",
            Self::FiniteElements => "Сетка КЭ плит",
        }
    }
}
fn zero_id(id: &u64) -> bool {
    *id == 0
}

#[derive(Clone, Serialize, Deserialize)]
struct Pressure {
    xy: [f64; 2],
    area: f64,
    pz: f64,
    rz: f64,
}
struct Outcome {
    contours: Vec<Contour>,
    foundation: u64,
    floor: u64,
    next_id: u64,
    status: String,
    reference: String,
    z: f64,
    pressures: Vec<Pressure>,
}
#[derive(Clone, Copy, Default, PartialEq)]
enum PressureView {
    #[default]
    None,
    Pz,
    Rz,
    Difference,
}
impl PressureView {
    fn value(self, p: &Pressure) -> f64 {
        match self {
            Self::None => 0.,
            Self::Pz => p.pz,
            Self::Rz => p.rz,
            Self::Difference => p.pz - p.rz,
        }
    }
}
enum Message {
    Progress(String),
    Done(Result<Outcome, String>),
}
type Excavation = (f64, mini_fem::geo::Polygon<f64>);
type GroundFace = ([[f64; 3]; 3], Color32);
type GroundFaces = std::sync::Arc<Vec<GroundFace>>;
struct GroundGeometry {
    faces: GroundFaces,
    bounds: Option<[[f64; 3]; 2]>,
    prisms: Vec<editor::GroundPrism>,
    excavations: Vec<Excavation>,
}
type SharedGround = std::sync::Arc<GroundGeometry>;
pub(super) struct View {
    ground_geometry: std::cell::RefCell<Option<(u64, SharedGround)>>,
    cut_geometry: std::cell::RefCell<Option<(SharedGround, sections::State, GroundFaces)>>,
    pub open: bool,
    pressure_view: PressureView,
    editor: Option<Editor>,
    editor_prompt: Option<EditorExit>,
    status: String,
    job: Option<std::sync::mpsc::Receiver<Message>>,
    signature: Option<(Vec<u8>, u64)>,
    cancel: Option<fem::progress::CancelToken>,
}

#[derive(Clone, Copy)]
enum EditorExit {
    Close,
    Application,
    Create,
    Switch(usize),
}

impl Default for View {
    fn default() -> Self {
        Self {
            ground_geometry: Default::default(),
            cut_geometry: Default::default(),
            open: false,
            pressure_view: PressureView::None,
            editor: None,
            editor_prompt: None,
            status: String::new(),
            job: None,
            signature: None,
            cancel: None,
        }
    }
}

impl App {
    pub(super) fn soil_models_controls(&mut self, ui: &mut egui::Ui) {
        let mut action = None;
        egui::CollapsingHeader::new("Модель грунта").show(ui, |ui| {
            if ui.button("Добавить модель грунта").clicked() {
                action = Some(EditorExit::Create);
            }
            for (i, ground) in self.soil_options.grounds.iter_mut().enumerate() {
                ui.horizontal(|ui| {
                    ui.checkbox(&mut ground.visible, "");
                    ui.selectable_label(
                        self.soil_view.editor.as_ref().is_some_and(|e| e.index == i),
                        &ground.name,
                    )
                    .context_menu(|ui| {
                        if ui.button("Редактировать").clicked() {
                            action = Some(EditorExit::Switch(i));
                            ui.close_menu();
                        }
                    });
                });
            }
        });
        if let Some(action) = action {
            self.request_ground_editor_exit(action, ui.ctx());
        }
    }
    pub(super) fn has_soil_pressure_results(&self) -> bool {
        self.soil_options
            .grounds
            .iter()
            .any(|g| !g.pressures.is_empty())
    }
    pub(super) fn soil_pressure_results(&mut self, ui: &mut egui::Ui) {
        if !self.has_soil_pressure_results() {
            return;
        }
        egui::CollapsingHeader::new("Давления основания").show(ui, |ui| {
            ui.horizontal_wrapped(|ui| {
                for (mode, label) in [
                    (PressureView::Pz, "Pz"),
                    (PressureView::Rz, "Rz"),
                    (PressureView::Difference, "Pz − Rz"),
                ] {
                    if ui
                        .selectable_label(self.soil_view.pressure_view == mode, label)
                        .clicked()
                    {
                        self.soil_view.pressure_view = if self.soil_view.pressure_view == mode {
                            PressureView::None
                        } else {
                            mode
                        };
                    }
                }
            });
        });
    }
    fn cached_ground_geometry(&self) -> SharedGround {
        use std::hash::{Hash, Hasher};
        let mut hash = std::collections::hash_map::DefaultHasher::new();
        for g in &self.soil_options.grounds {
            g.visible.hash(&mut hash);
            g.holes.len().hash(&mut hash);
            g.ground.map(f64::to_bits).hash(&mut hash);
            g.model.map(f64::to_bits).hash(&mut hash);
            g.angle.to_bits().hash(&mut hash);
        }
        let foundations: std::collections::BTreeSet<_> = self
            .contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab && c.use_foundation && !self.object_hidden_without_sections(c)
            })
            .map(|c| c.id)
            .collect();
        for c in self.contours.iter().filter(|c| {
            foundations.contains(&c.id)
                || c.active_hole() && c.parent.is_some_and(|p| foundations.contains(&p))
        }) {
            c.id.hash(&mut hash);
            self.source_heights(c)[0].to_bits().hash(&mut hash);
            for p in &c.points {
                p.map(f64::to_bits).hash(&mut hash);
            }
        }
        let key = hash.finish();
        if let Some((old, faces)) = &*self.soil_view.ground_geometry.borrow() {
            if *old == key {
                return faces.clone();
            }
        }
        let faces = std::sync::Arc::new(self.ground_geometry());
        *self.soil_view.ground_geometry.borrow_mut() = Some((key, faces.clone()));
        faces
    }
    fn ground_excavations(&self, visible_only: bool) -> Vec<Excavation> {
        self.contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab
                    && c.use_foundation
                    && (!visible_only || !self.object_hidden_without_sections(c))
            })
            .map(|c| {
                let ring = |p: &[Point]| {
                    mini_fem::geo::LineString::from(
                        p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>(),
                    )
                };
                let holes = self
                    .contours
                    .iter()
                    .filter(|h| h.active_hole() && h.parent == Some(c.id))
                    .map(|h| ring(&h.points))
                    .collect();
                (
                    self.source_heights(c)[0],
                    mini_fem::geo::Polygon::new(ring(&c.points), holes),
                )
            })
            .collect()
    }
    fn ground_geometry(&self) -> GroundGeometry {
        let excavations = self.ground_excavations(true);
        let mut prisms = Vec::new();
        let mut faces: Vec<_> = self
            .soil_options
            .grounds
            .iter()
            .filter(|g| g.visible)
            .flat_map(|g| editor::volume_geometry(&g.placed(), Some(&mut prisms)))
            .flat_map(|(face, color)| {
                Self::excavate_ground_face(face, &excavations)
                    .into_iter()
                    .map(move |face| (face, color))
            })
            .collect();
        faces.extend(editor::excavation_faces(
            &prisms,
            &excavations,
            &sections::State::default(),
        ));
        let mut bounds = [[f64::INFINITY; 3], [f64::NEG_INFINITY; 3]];
        for point in faces.iter().flat_map(|(face, _)| face) {
            for (axis, &value) in point.iter().enumerate() {
                bounds[0][axis] = bounds[0][axis].min(value);
                bounds[1][axis] = bounds[1][axis].max(value);
            }
        }
        GroundGeometry {
            bounds: (!faces.is_empty()).then_some(bounds),
            faces: std::sync::Arc::new(faces),
            prisms,
            excavations,
        }
    }

    pub(super) fn ground_depth_corners(&self) -> Option<[[f64; 3]; 8]> {
        if !self.soil_options.grounds.iter().any(|g| g.visible) {
            return None;
        }
        let bounds = self.cached_ground_geometry().bounds?;
        Some(std::array::from_fn(|mask| {
            std::array::from_fn(|axis| bounds[(mask >> axis) & 1][axis])
        }))
    }

    fn visible_ground_faces(&self) -> GroundFaces {
        let base = self.cached_ground_geometry();
        if !self.sections.active() {
            return base.faces.clone();
        }
        if let Some((old, state, cut)) = &*self.soil_view.cut_geometry.borrow() {
            let geometry = |s: &sections::State| {
                s.active_planes()
                    .map(|(i, p)| (p.normal(i), p.offset(i), p.positive))
                    .collect::<Vec<_>>()
            };
            if std::sync::Arc::ptr_eq(old, &base) && geometry(state) == geometry(&self.sections) {
                return cut.clone();
            }
        }
        let caps = base
            .prisms
            .iter()
            .flat_map(|p| p.section_faces(&self.sections))
            .flat_map(|(face, color)| {
                Self::excavate_ground_face(face, &base.excavations)
                    .into_iter()
                    .map(move |face| (face, color))
            });
        let cut: GroundFaces = std::sync::Arc::new(
            base.faces
                .iter()
                .copied()
                .chain(caps)
                .flat_map(|(face, color)| {
                    clip_ground_face(face, &self.sections)
                        .into_iter()
                        .map(move |face| (face, color))
                })
                .collect(),
        );
        *self.soil_view.cut_geometry.borrow_mut() =
            Some((base, self.sections.clone(), cut.clone()));
        cut
    }

    fn excavate_ground_face(face: [[f64; 3]; 3], excavations: &[Excavation]) -> Vec<[[f64; 3]; 3]> {
        use mini_fem::geo::BooleanOps;
        let clipped = face.to_vec();
        if excavations.is_empty() {
            return (1..clipped.len() - 1)
                .map(|i| [clipped[0], clipped[i], clipped[i + 1]])
                .collect();
        }
        let mut chunks = vec![clipped];
        let u = std::array::from_fn::<_, 3, _>(|i| face[1][i] - face[0][i]);
        let v = std::array::from_fn::<_, 3, _>(|i| face[2][i] - face[0][i]);
        let det = u[0] * v[1] - u[1] * v[0];
        if det.abs() < 1e-9 {
            return Self::excavate_ground_side(face, excavations);
        }
        let elevation = |x: f64, y: f64| {
            let dx = x - face[0][0];
            let dy = y - face[0][1];
            face[0][2] + ((dx * v[1] - dy * v[0]) * u[2] + (u[0] * dy - u[1] * dx) * v[2]) / det
        };
        for (level, slab) in excavations {
            let mut next = vec![];
            for poly in chunks {
                if !poly.iter().any(|p| p[2] > *level) {
                    next.push(poly);
                    continue;
                }
                let above = clip_ground_polygon(&poly, |p| p[2] - level);
                let below = clip_ground_polygon(&poly, |p| level - p[2]);
                if below.len() >= 3 && poly.iter().any(|p| p[2] < *level) {
                    next.push(below);
                }
                if above.len() < 3 {
                    continue;
                }
                let source = mini_fem::geo::Polygon::new(
                    above
                        .iter()
                        .map(|p| (p[0], p[1]))
                        .collect::<Vec<_>>()
                        .into(),
                    vec![],
                );
                for fragment in source.difference(slab).0 {
                    let rings = std::iter::once(fragment.exterior())
                        .chain(fragment.interiors())
                        .map(|r| r.0.iter().map(|p| [p.x, p.y]).collect())
                        .collect::<Vec<_>>();
                    if let Ok(triangles) = geometry::triangulate(&rings) {
                        next.extend(
                            triangles
                                .into_iter()
                                .map(|tri| tri.map(|[x, y]| [x, y, elevation(x, y)]).to_vec()),
                        );
                    }
                }
            }
            chunks = next;
        }
        chunks
            .into_iter()
            .flat_map(|p| (1..p.len() - 1).map(move |i| [p[0], p[i], p[i + 1]]))
            .collect()
    }
    fn excavate_ground_side(face: [[f64; 3]; 3], excavations: &[Excavation]) -> Vec<[[f64; 3]; 3]> {
        use mini_fem::geo::{BooleanOps, LineString, MultiLineString};
        let [a, b, c] = face;
        let xy_distance = |p: [f64; 3], q: [f64; 3]| (p[0] - q[0]).hypot(p[1] - q[1]);
        let Some((start, end)) = [(a, b), (a, c), (b, c)]
            .into_iter()
            .max_by(|(p, q), (r, s)| xy_distance(*p, *q).total_cmp(&xy_distance(*r, *s)))
            .filter(|(p, q)| xy_distance(*p, *q) > 1e-9)
        else {
            return vec![];
        };
        let d = [end[0] - start[0], end[1] - start[1]];
        let norm = d[0] * d[0] + d[1] * d[1];
        let t = |x: f64, y: f64| ((x - start[0]) * d[0] + (y - start[1]) * d[1]) / norm;
        let axis = MultiLineString(vec![LineString::from(vec![
            (start[0], start[1]),
            (end[0], end[1]),
        ])]);
        let mut parts = vec![face.to_vec()];
        for (level, slab) in excavations {
            if !parts.iter().flatten().any(|p| p[2] > *level) {
                continue;
            }
            let intervals: Vec<_> = slab
                .clip(&axis, true)
                .0
                .into_iter()
                .filter_map(|l| {
                    let from = l.0.first()?;
                    let to = l.0.last()?;
                    let (from, to) = (t(from.x, from.y), t(to.x, to.y));
                    Some((from.min(to), from.max(to)))
                })
                .collect();
            let mut next = Vec::new();
            for poly in parts {
                if !poly.iter().any(|p| p[2] > *level) {
                    next.push(poly);
                    continue;
                }
                if poly.iter().any(|p| p[2] < *level) {
                    next.push(clip_ground_polygon(&poly, |p| level - p[2]));
                }
                let above = clip_ground_polygon(&poly, |p| p[2] - level);
                for &(lo, hi) in &intervals {
                    let side = clip_ground_polygon(&above, |p| t(p[0], p[1]) - lo);
                    next.push(clip_ground_polygon(&side, |p| hi - t(p[0], p[1])));
                }
            }
            parts = next.into_iter().filter(|p| p.len() >= 3).collect();
        }
        parts
            .into_iter()
            .flat_map(|p| (1..p.len() - 1).map(move |i| [p[0], p[i], p[i + 1]]))
            .collect()
    }
    pub(super) fn spatial_ground(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
    ) {
        if !self.soil_options.grounds.iter().any(|g| g.visible) {
            return;
        }
        let mut builder = spatial_depth::Builder::new(rect, projected);
        let faces = self.visible_ground_faces();
        let opacity = self.drawing_visibility.ground_opacity;
        let layer = if opacity >= 0.999 {
            for &(face, color) in faces.iter() {
                builder.triangle(face.map(project), color, 0.);
            }
            spatial_depth::Layer::GroundOpaque
        } else {
            let mut triangles: Vec<_> = faces
                .iter()
                .map(|&(face, color)| {
                    let points = face.map(project);
                    (points.iter().map(|p| p.1).sum::<f64>(), points, color)
                })
                .collect();
            triangles.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
            for (_, triangle, color) in triangles {
                builder.triangle(triangle, editor::opacity_ink(color, opacity), 0.);
            }
            spatial_depth::Layer::Ground
        };
        // The ground faces already include the section cuts in model coordinates.
        let _clips = spatial_depth::clip_guard();
        spatial_depth::paint_layer(painter, rect, std::sync::Arc::new(builder.vertices), layer);
    }
    pub(super) fn soil_pressure_window(&mut self, ctx: &egui::Context) {
        if self.soil_view.pressure_view == PressureView::None {
            return;
        }
        let mut open = true;
        egui::Window::new("Давления основания")
            .open(&mut open)
            .default_size([650., 460.])
            .show(ctx, |ui| self.soil_pressure_canvas(ui));
        if !open {
            self.soil_view.pressure_view = PressureView::None;
        }
    }

    fn soil_pressure_canvas(&self, ui: &mut egui::Ui) {
        let pressure = self
            .soil_options
            .grounds
            .get(self.soil_options.selected)
            .map_or(&[][..], |g| g.pressures.as_slice());
        let mode = self.soil_view.pressure_view;
        if mode != PressureView::None {
            if pressure.is_empty() {
                ui.label("Нет сохранённых данных двух последовательных итераций C1.");
            } else {
                let pz = pressure.iter().map(|p| p.pz * p.area).sum::<f64>();
                let rz = pressure.iter().map(|p| p.rz * p.area).sum::<f64>();
                let peak = pressure
                    .iter()
                    .map(|p| (p.pz - p.rz).abs())
                    .fold(0., f64::max);
                ui.label(format!(
                    "ΣPz {pz:.2} тс · ΣRz {rz:.2} тс · Δ {:.2} тс · max |Pz − Rz| {peak:.3} тс/м²",
                    pz - rz
                ));
                if let Some(ground) = self.soil_options.grounds.get(self.soil_options.selected) {
                    ui.label(&ground.pressure_reference);
                }
                ui.label("Pz — давление предыдущей итерации, Rz — реакция при зафиксированном C1.");
            }
        }
        let (response, painter) = ui.allocate_painter(
            ui.available_size().max(egui::vec2(250., 180.)),
            egui::Sense::hover(),
        );
        if pressure.is_empty() {
            return;
        }
        let pressure_foundation = self
            .soil_options
            .grounds
            .get(self.soil_options.selected)
            .map_or(0, |g| g.pressure_foundation);
        let slab = if pressure_foundation == 0 {
            self.soil_outline()
        } else {
            self.contours
                .iter()
                .find(|c| c.id == pressure_foundation && c.kind == Kind::Foundation)
        };
        let mut low = egui::vec2(f32::INFINITY, f32::INFINITY);
        let mut high = -low;
        let project = |[x, y]: [f64; 2]| egui::vec2(x as f32, -y as f32);
        for xy in pressure.iter().map(|p| p.xy).chain(
            slab.into_iter()
                .flat_map(|s| s.points.iter().map(|p| p.map(|v| v / 1000.))),
        ) {
            low = low.min(project(xy));
            high = high.max(project(xy));
        }
        let span = high - low;
        let scale = ((response.rect.width() - 30.).max(1.) / span.x.max(1.))
            .min((response.rect.height() - 30.).max(1.) / span.y.max(1.));
        let screen = |xy| response.rect.center() + (project(xy) - (low + high) * 0.5) * scale;
        if let Some(slab) = slab {
            let mut outline: Vec<_> = slab
                .points
                .iter()
                .map(|p| screen(p.map(|v| v / 1000.)))
                .collect();
            if let Some(first) = outline.first().copied() {
                outline.push(first);
                painter.add(egui::Shape::line(
                    outline,
                    egui::Stroke::new(2.0_f32, egui::Color32::YELLOW),
                ));
            }
        }
        if mode != PressureView::None && !pressure.is_empty() {
            let max = pressure
                .iter()
                .map(|p| mode.value(p).abs())
                .fold(0., f64::max);
            let mut hover = None;
            for p in pressure {
                let v = mode.value(p);
                let t = (v.abs() / max.max(1e-9)).clamp(0., 1.) as f32;
                let color: egui::Color32 =
                    egui::ecolor::Hsva::new(if v < 0. { 0.60 } else { 0.02 }, t, 0.95, 1.).into();
                let at = screen(p.xy);
                painter.circle_filled(at, 4., color);
                if response
                    .hover_pos()
                    .is_some_and(|mouse| mouse.distance(at) < 6.)
                {
                    hover = Some(p);
                }
            }
            if let Some(p) = hover {
                response.on_hover_text(format!(
                    "Pz {:.3} · Rz {:.3} · Pz − Rz {:.3} тс/м²",
                    p.pz,
                    p.rz,
                    p.pz - p.rz
                ));
            }
            return;
        }
    }
    pub(super) fn soil_progress_status(&self) -> Option<&str> {
        self.soil_view
            .job
            .as_ref()
            .map(|_| self.soil_view.status.as_str())
    }

    pub(super) fn poll_soil_job(&mut self, ctx: &egui::Context) {
        while let Some(message) = self
            .soil_view
            .job
            .as_ref()
            .and_then(|rx| match rx.try_recv() {
                Ok(m) => Some(m),
                Err(std::sync::mpsc::TryRecvError::Disconnected) => {
                    Some(Message::Done(Err("Поток расчёта завершился".into())))
                }
                Err(_) => None,
            })
        {
            match message {
                Message::Progress(status) => {
                    if !self
                        .soil_view
                        .cancel
                        .as_ref()
                        .is_some_and(|cancel| cancel.load(std::sync::atomic::Ordering::Relaxed))
                    {
                        self.soil_view.status = status;
                    }
                }
                Message::Done(result) => {
                    self.soil_view.job = None;
                    self.soil_view.cancel = None;
                    let unchanged =
                        self.soil_view
                            .signature
                            .take()
                            .is_some_and(|(before, mesh_generation)| {
                                mesh_generation == self.mesh_generation
                                    && self.soil_mesh().is_some()
                                    && self.project_state_bytes().as_ref() == Some(&before)
                            });
                    match result {
                        Ok(_)
                            if !unchanged
                                || self.input_locked
                                || self.mesh_job.is_some()
                                || self.analysis_job.is_some()
                                || self.alternative_job.is_some() =>
                        {
                            self.soil_view.status =
                                "Схема изменилась во время расчёта. C1 не применён.".into();
                        }
                        Ok(outcome) => {
                            self.invalidate_scene_buffers();
                            if self.soil_options.horizontal_supports {
                                self.support_height.automatic_building_bottom = true;
                                self.support_height.building_bottom_mask = 3;
                            }
                            self.history.push(self.contours.clone());
                            self.contours = outcome.contours;
                            self.next_id = outcome.next_id;
                            self.building
                                .contour_floors
                                .insert(outcome.foundation, outcome.floor);
                            if let Some(ground) = self
                                .soil_options
                                .grounds
                                .get_mut(self.soil_options.selected)
                            {
                                ground.pressures = outcome.pressures;
                                ground.pressure_reference = outcome.reference;
                                ground.pressure_z = outcome.z;
                                ground.pressure_foundation = outcome.foundation;
                            }
                            self.soil_view.status = outcome.status;
                        }
                        Err(e) => self.soil_view.status = e,
                    }
                }
            }
        }
        if self.soil_view.job.is_some() {
            ctx.request_repaint_after(Duration::from_millis(150));
        }
    }
    fn soil_foundation_id(&self) -> u64 {
        let o = &self.soil_options;
        if self
            .contours
            .iter()
            .any(|c| c.id == o.foundation && c.kind == Kind::Foundation)
        {
            return o.foundation;
        }
        if let Some(slab) = self
            .contours
            .iter()
            .find(|c| c.id == o.legacy_slab && c.kind == Kind::Slab)
        {
            let mut matches = self.contours.iter().filter(|c| {
                c.kind == Kind::Foundation
                    && super::super::foundation::same_elevation(
                        self.source_heights(c)[0],
                        self.source_heights(slab)[0],
                    )
                    && (c.points == slab.points
                        || slab.points.iter().all(|&p| geometry::inside(p, &c.points)))
            });
            if let (Some(c), None) = (matches.next(), matches.next()) {
                return c.id;
            }
        }
        let mut foundations = self.contours.iter().filter(|c| c.kind == Kind::Foundation);
        match (foundations.next(), foundations.next()) {
            (Some(c), None) => c.id,
            _ => 0,
        }
    }

    fn soil_outline(&self) -> Option<&Contour> {
        let id = self.soil_foundation_id();
        self.contours
            .iter()
            .find(|c| c.id == id && c.kind == Kind::Foundation)
            .or_else(|| {
                self.contours
                    .iter()
                    .find(|c| c.id == self.soil_options.legacy_slab && c.kind == Kind::Slab)
            })
    }

    fn soil_mesh(&self) -> Option<&super::super::meshing::Mesh> {
        self.fe_mesh.as_ref().filter(|mesh| {
            self.building.enabled
                && !self.mesh_partial
                && self.mesh_job.is_none()
                && self.mesh_built_size == self.mesh_size
                && model::same_mesh_input(&self.mesh_input, &self.contours)
                && mesh.building.is_some()
        })
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn start_soil_job(&mut self, ctx: &egui::Context) {
        let Some(mesh) = self.soil_mesh() else {
            self.soil_view.status = "Сначала постройте полную МКЭ-сетку.".into();
            return;
        };
        let foundation_id = self.soil_foundation_id();
        if foundation_id == 0 {
            self.soil_view.status = "Выберите основание.".into();
            return;
        }
        let floor = self.building.owner(foundation_id);
        let options = self.soil_options.clone();
        let Some(ground) = options
            .grounds
            .get(options.selected)
            .filter(|ground| !ground.holes.is_empty())
        else {
            self.soil_view.status = "Выберите модель грунта со скважинами.".into();
            return;
        };
        let holes = ground.placed();
        let Some(signature) = self.project_state_bytes() else {
            self.soil_view.status = "Не удалось сохранить состояние схемы".into();
            return;
        };
        let mesh = mesh.clone();
        let mesh_step = self.mesh_built_size;
        self.soil_view.signature = Some((signature, self.mesh_generation));
        let snapshot = self.project_snapshot();
        let cancel = fem::progress::CancelToken::default();
        self.soil_view.cancel = Some(cancel.clone());
        let (tx, rx) = std::sync::mpsc::channel();
        self.soil_view.job = Some(rx);
        self.soil_view.status = "Подготовка модели".into();
        let ctx = ctx.clone();
        std::thread::spawn(move || {
            let stage_tx = tx.clone();
            let repaint = ctx.clone();
            fem::progress::install(move |label, seconds| {
                let _ = stage_tx.send(Message::Progress(format!(
                    "C1 · МКЭ · {label} · {seconds:.1} с"
                )));
                repaint.request_repaint();
            });
            fem::progress::install_cancel(cancel);
            let run = || -> Result<Outcome, String> {
                let mut app = App::default();
                app.apply_project(snapshot, PathBuf::new())?;
                app.sync_stiffness_sources();
                if options.horizontal_supports {
                    app.support_height.automatic_building_bottom = true;
                    app.support_height.building_bottom_mask = 3;
                }
                let report = soil_prototype_host::calculate(
                    &mut app,
                    holes,
                    options.solver,
                    options.step,
                    soil_prototype_host::Start::ExistingMesh {
                        mesh: &mesh,
                        step: mesh_step,
                        foundation: foundation_id,
                    },
                    |s| {
                        fem::progress::check_cancel()?;
                        tx.send(Message::Progress(s))
                            .map_err(|_| "Расчёт остановлен".to_owned())?;
                        ctx.request_repaint();
                        Ok(())
                    },
                )?;
                let result = &report["result"];
                let residual = report["residual"]
                    .as_f64()
                    .ok_or("Не вычислена невязка C1")?
                    * 100.;
                let z = report["ground_base_z"]
                    .as_f64()
                    .ok_or("Не определена отметка основания")?;
                let converged = result["converged"]
                    .as_bool()
                    .ok_or("Нет результата итераций C1")?;
                let iterations = result["iterations"]
                    .as_u64()
                    .ok_or("Нет счётчика итераций C1")?;
                let status = if converged {
                    format!("C1 зафиксирован · итераций {iterations} · невязка {residual:.2}%")
                } else {
                    format!("Достигнут лимит {iterations} итераций · невязка {residual:.2}% · последнее приближение сохранено. Можно продолжить расчёт C1.")
                };
                fem::progress::check_cancel()?;
                let cells: Vec<mini_fem::soil::Cell> =
                    serde_json::from_value(report["cells"].clone()).map_err(|e| e.to_string())?;
                let rz: Vec<f64> = serde_json::from_value(result["pressure"].clone())
                    .map_err(|e| e.to_string())?;
                let pressures = if report["transferred_pressure_tf_m2"].is_null() {
                    Vec::new()
                } else {
                    let pz: Vec<f64> =
                        serde_json::from_value(report["transferred_pressure_tf_m2"].clone())
                            .map_err(|e| e.to_string())?;
                    cells
                        .into_iter()
                        .zip(pz)
                        .zip(rz)
                        .map(|((cell, pz), rz)| Pressure {
                            xy: cell.center(),
                            area: cell.area(),
                            pz,
                            rz,
                        })
                        .collect()
                };
                Ok(Outcome {
                    contours: app.contours,
                    foundation: foundation_id,
                    floor,
                    next_id: app.next_id,
                    status,
                    pressures,
                    z,
                    reference: format!(
                        "{} · {}",
                        app.combinations
                            .iter()
                            .find(|c| c.id == options.combination)
                            .map(|c| c.display_name())
                            .unwrap_or_default(),
                        options.variant.label()
                    ),
                })
            };
            let _ = tx.send(Message::Done(run()));
            ctx.request_repaint();
        });
    }
    pub(super) fn soil_settings_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Расчёт основания").show(ui, |ui| {
            let idle = self.soil_view.job.is_none();
            let current_foundation = self.soil_foundation_id();
            ui.add_enabled_ui(idle, |ui| {
                let o = &mut self.soil_options;
                ui.horizontal_wrapped(|ui| {
                ui.label("Грунт").on_hover_text("Модель скважин и ИГЭ, из которой берутся свойства для расчёта осадки.");
                egui::ComboBox::from_id_salt("soil_ground")
                    .selected_text(
                        o.grounds
                            .get(o.selected)
                            .map_or("Выберите грунт", |g| &g.name),
                    )
                    .show_ui(ui, |ui| {
                        for (i, g) in o.grounds.iter().enumerate() {
                            ui.selectable_value(&mut o.selected, i, &g.name);
                        }
                    }).response.on_hover_text("Модель скважин и ИГЭ для расчёта осадки.");
                });
                ui.horizontal_wrapped(|ui| {
                    let help = "Основание, для которого уточняются значения C1. В расчёт входят КЭ его этажа, чьи центры лежат внутри.";
                    ui.label("Основание").on_hover_text(help);
                    let foundation_label = |c: &Contour| {
                        if c.name.trim().is_empty() { format!("Основание №{}", c.id) }
                        else { format!("{} · №{}", c.name, c.id) }
                    };
                    let mut id = current_foundation;
                    egui::ComboBox::from_id_salt("soil_foundation")
                        .selected_text(self.contours.iter()
                            .find(|c| c.id == id && c.kind == Kind::Foundation)
                            .map_or_else(|| "Выберите основание".into(), foundation_label))
                        .show_ui(ui, |ui| {
                            for c in self.contours.iter().filter(|c| c.kind == Kind::Foundation) {
                                ui.selectable_value(&mut id, c.id, foundation_label(c));
                            }
                        }).response.on_hover_text(help);
                    if id != current_foundation {
                        o.foundation = id;
                        o.legacy_slab = 0;
                    }
                });
                ui.horizontal_wrapped(|ui| {
                    ui.label("Сочетание").on_hover_text("Нагрузки этого сочетания используются для определения давления в основании.");
                    egui::ComboBox::from_id_salt("soil_combination")
                        .selected_text(
                            self.combinations
                                .iter()
                                .find(|c| c.id == o.combination)
                                .map_or_else(|| "Выберите сочетание".into(), |c| c.display_name()),
                        )
                        .show_ui(ui, |ui| {
                            for c in &self.combinations {
                                ui.selectable_value(&mut o.combination, c.id, c.display_name());
                            }
                        }).response.on_hover_text("Выбирает сочетание, по которому выполняются итерации C1.");
                    egui::ComboBox::from_id_salt("soil_variant")
                        .selected_text(o.variant.label())
                        .show_ui(ui, |ui| {
                            for v in crate::plan_trace::load_combinations::Variant::ALL {
                                ui.selectable_value(&mut o.variant, v, v.label());
                            }
                        }).response.on_hover_text("Выбирает вариант коэффициентов сочетания для расчёта основания.");
                });
                ui.checkbox(
                    &mut o.horizontal_supports,
                    "Автоматически закрепить низ только по X/Y",
                ).on_hover_text("Включает автоматические закрепления по X и Y на нижнем уровне. Вертикальная податливость задаётся основанием.");
                ui.horizontal_wrapped(|ui| {
                    ui.label("Сетка грунта");
                    egui::ComboBox::from_id_salt("soil_discretization")
                        .selected_text(o.discretization.label())
                        .show_ui(ui, |ui| {
                            for mode in [Discretization::Regular, Discretization::FiniteElements] {
                                ui.selectable_value(&mut o.discretization, mode, mode.label());
                            }
                        }).response.on_hover_text("Сетка КЭ использует площадь и форму каждого КЭ плиты на основании. На мелкой сетке расчёт осадок может занять больше времени.");
                });
                egui::Grid::new("soil_solver_parameters").show(ui, |ui| {
                    if o.discretization == Discretization::Regular {
                        let help = "Размер ячейки плана для усреднения давления и расчёта осадки и C1.";
                        ui.label("Шаг основания, мм").on_hover_text(help);
                        ui.add(egui::DragValue::new(&mut o.step).range(100.0..=10000.).speed(1.)).on_hover_text(help);
                        ui.end_row();
                    }
                    let min_depth = o.solver.min_depth;
                    let max_depth = o.solver.max_depth;
                    for (label, value, range, help) in [
                        ("Мин. глубина, м", &mut o.solver.min_depth, 0. ..=max_depth, "Минимальная глубина сжимаемой толщи от подошвы. До неё порог напряжений не останавливает суммирование."),
                        ("Макс. глубина, м", &mut o.solver.max_depth, min_depth.max(0.1)..=100., "Максимальная глубина суммирования осадки от подошвы."),
                        ("Шаг по глубине, м", &mut o.solver.dz, 0.01..=5., "Толщина расчётного подслоя при суммировании деформаций по глубине."),
                        ("Коэффициент β", &mut o.solver.beta, 0.01..=2., "Множитель деформаций при послойном суммировании осадки."),
                        ("Порог напряжений", &mut o.solver.stress_ratio, 0.01..=1., "Ограничивает сжимаемую толщу по отношению добавочного вертикального напряжения к напряжению от веса грунта."),
                        ("Релаксация", &mut o.solver.relaxation, 0.01..=1., "Доля нового значения давление / осадка при обновлении C1. При 1 новое значение применяется целиком."),
                        ("Допуск, доля", &mut o.solver.tolerance, 0.001..=0.1, "Предел максимального относительного расхождения между текущим и пересчитанным C1. 0,02 соответствует 2 %."),
                    ] {
                        ui.label(label).on_hover_text(help);
                        ui.add(egui::DragValue::new(value).range(range).speed(0.01)).on_hover_text(help);
                        ui.end_row();
                    }
                    let help = "Число итераций за один запуск. При достижении лимита последнее приближение C1 сохраняется.";
                    ui.label("Лимит итераций").on_hover_text(help);
                    ui.add(egui::DragValue::new(&mut o.solver.max_iterations).range(1..=1000)).on_hover_text(help);
                    ui.end_row();
                });
                #[cfg(not(target_arch = "wasm32"))]
                let ground_ready = o.grounds.get(o.selected).is_some_and(|g| !g.holes.is_empty());
                #[cfg(not(target_arch = "wasm32"))]
                let combination_ready = self.combinations.iter().any(|c| c.id == o.combination);
                #[cfg(not(target_arch = "wasm32"))]
                let disabled_reason = if !self.building.enabled {
                    Some("Включите сборку здания по этажам.")
                } else if self.soil_mesh().is_none() {
                    Some("Постройте полную МКЭ-сетку. Если выделены объекты, снимите выделение перед триангуляцией.")
                } else if self.soil_foundation_id() == 0 {
                    Some("Выберите основание.")
                } else if !ground_ready {
                    Some("Выберите модель грунта со скважинами.")
                } else if !combination_ready {
                    Some("Выберите сочетание.")
                } else if self.analysis_job.is_some() || self.alternative_job.is_some() {
                    Some("Дождитесь завершения расчёта МКЭ.")
                } else if self.input_locked {
                    Some("Снимите блокировку исходных данных на нижней панели.")
                } else {
                    None
                };
                #[cfg(not(target_arch = "wasm32"))]
                ui.small(disabled_reason.unwrap_or("Используется текущая МКЭ-сетка."));
                #[cfg(not(target_arch = "wasm32"))]
                if ui
                    .add_enabled(
                        disabled_reason.is_none(),
                        egui::Button::new("Рассчитать C1"),
                    )
                    .on_hover_text("Уточняет заданный C1 по давлению и осадке на текущей МКЭ-сетке. Сохраняет итоговые C1 в выбранном основании.")
                    .on_disabled_hover_text(disabled_reason.unwrap_or_default())
                    .clicked()
                {
                    self.start_soil_job(ui.ctx());
                }
            });
            if !idle && ui.button("Остановить расчёт C1")
                .on_hover_text("Прерывает итерации. Частичный результат в проект не переносится.").clicked() {
                if let Some(cancel) = &self.soil_view.cancel {
                    cancel.store(true, std::sync::atomic::Ordering::Relaxed);
                }
                self.soil_view.status = "Остановка расчёта C1…".into();
            }
            if !idle {
                ui.spinner();
            }
            #[cfg(target_arch = "wasm32")]
            ui.label("Итерационный расчёт C1 доступен в настольной версии.");
            ui.label(&self.soil_view.status);
        }).header_response.on_hover_text("Расчёт коэффициентов постели в выбранном основании по модели грунта и одному сочетанию.");
    }
}

fn clip_ground_face(face: [[f64; 3]; 3], sections: &sections::State) -> Vec<[[f64; 3]; 3]> {
    let mut clipped = face.to_vec();
    for (i, plane) in sections.active_planes() {
        clipped = clip_ground_polygon(&clipped, |p| plane.distance(i, p));
        if clipped.len() < 3 {
            return vec![];
        }
    }
    (1..clipped.len() - 1)
        .map(|i| [clipped[0], clipped[i], clipped[i + 1]])
        .collect()
}

fn clip_ground_polygon(points: &[[f64; 3]], distance: impl Fn([f64; 3]) -> f64) -> Vec<[f64; 3]> {
    let mut result = Vec::new();
    if let Some(&last) = points.last() {
        let mut a = last;
        let mut da = distance(a);
        for &b in points {
            let db = distance(b);
            if (da >= 0.) != (db >= 0.) {
                let t = da / (da - db);
                result.push(std::array::from_fn(|i| a[i] + t * (b[i] - a[i])));
            }
            if db >= 0. {
                result.push(b);
            }
            a = b;
            da = db;
        }
    }
    result
}
#[cfg(test)]
mod ground_cut_tests {
    use super::*;
    #[test]
    fn clips_plane_and_excavates_above_slab_but_retains_hole() {
        let mut app = App::default();
        app.sections.planes[2].enabled = true;
        let slab = mini_fem::geo::Polygon::new(
            vec![(0., 0.), (2., 0.), (2., 2.), (0., 2.), (0., 0.)].into(),
            vec![vec![
                (0.25, 0.25),
                (0.5, 0.25),
                (0.5, 0.5),
                (0.25, 0.5),
                (0.25, 0.25),
            ]
            .into()],
        );
        let below = [[-1., -1., -2.], [5., -1., -2.], [-1., 5., -2.]];
        assert_eq!(
            App::excavate_ground_face(below, &[(0., slab.clone())]),
            vec![below]
        );
        let result = App::excavate_ground_face(
            [[-1., -1., 2.], [5., -1., 2.], [-1., 5., 2.]],
            &[(0., slab)],
        );
        let result = result
            .into_iter()
            .flat_map(|f| clip_ground_face(f, &app.sections))
            .collect::<Vec<_>>();
        let area = result
            .iter()
            .map(|t| geometry::area(&t.map(|p| [p[0], p[1]])).abs())
            .sum::<f64>();
        assert!((area - (12.5 - 4. + 0.0625)).abs() < 1e-6, "{area}");
        assert!(result
            .iter()
            .flatten()
            .all(|p| p[0] >= -1e-8 && (p[2] - 2.).abs() < 1e-8));
    }
    #[test]
    fn ground_excavation_clips_vertical_faces_only_above_slab() {
        let slab = mini_fem::geo::Polygon::new(
            vec![(0., -1.), (2., -1.), (2., 1.), (0., 1.), (0., -1.)].into(),
            vec![],
        );
        let result =
            App::excavate_ground_face([[-2., 0., -2.], [2., 0., -2.], [2., 0., 2.]], &[(0., slab)]);
        let area = result
            .iter()
            .map(|t| geometry::area(&t.map(|p| [p[0], p[2]])).abs())
            .sum::<f64>();
        assert!((area - 6.).abs() < 1e-6, "{area}");
        assert!(result.iter().flatten().all(|p| p[2] <= 1e-8));
    }
}
