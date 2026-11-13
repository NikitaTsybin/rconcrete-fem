use super::*;
use crate::plan_trace::foundation::{self, Sample};
impl App {
    pub(super) fn foundation_changed(&mut self) {
        self.analysis = None;
        self.analysis_input = None;
        self.combination_cache = None;
        self.alternative_cache = Default::default();
        self.alternative_job = None;
        self.invalidate_scene_buffers();
        self.spatial.visibility_reset();
    }
    pub(super) fn foundation_context_menu(&mut self, ui: &mut egui::Ui) {
        if let Some(id) = self.selected.filter(|id| {
            self.contours
                .iter()
                .any(|c| c.id == *id && c.kind == Kind::Foundation)
        }) {
            if ui
                .add_enabled(
                    self.contour_editable(id),
                    egui::Button::new("Добавить вершину интерполяции"),
                )
                .clicked()
            {
                self.begin(Kind::Foundation);
                self.foundation_point_target = Some(id);
                self.selected = Some(id);
                self.status = "Укажите точку внутри основания. Esc — отменить".into();
                ui.close_menu();
            }
        }
    }
    pub(super) fn place_foundation_point(&mut self, p: Point) {
        let Some(id) = self.foundation_point_target else {
            return;
        };
        let Some(c) = self.contours.iter().find(|c| c.id == id) else {
            return;
        };
        let covered = c.foundation.as_ref().map_or_else(
            || geometry::inside(p, &c.points),
            |data| data.covers(&c.points, p),
        );
        if !covered {
            self.status = "Укажите точку внутри области действия C1".into();
            return;
        }
        let c1 = foundation::Field::new(c, self.gravity)
            .ok()
            .and_then(|f| f.at(p))
            .map(|value| crate::plan_trace::loads::Magnitude {
                value,
                units: crate::plan_trace::loads::ForceUnits::KiloNewton,
            })
            .unwrap_or_else(foundation::default_c1);
        self.history.push(self.contours.clone());
        self.contours
            .iter_mut()
            .find(|c| c.id == id)
            .unwrap()
            .foundation
            .get_or_insert_with(Default::default)
            .interior
            .push(Sample { xy: p, c1 });
        self.foundation_point_target = None;
        self.mode = None;
        self.foundation_changed();
        self.properties.open = true;
        self.status = "Вершина интерполяции добавлена. Значение C1 — в свойствах".into();
    }
    pub(super) fn foundation_property_rows(&mut self, ui: &mut egui::Ui, ids: &[u64]) {
        let slab_ids = ids
            .iter()
            .copied()
            .filter(|id| {
                self.contours
                    .iter()
                    .any(|c| c.id == *id && c.kind == Kind::Slab)
            })
            .collect::<Vec<_>>();
        if !slab_ids.is_empty() {
            let mut value = slab_ids.iter().all(|id| {
                self.contours
                    .iter()
                    .find(|c| c.id == *id)
                    .unwrap()
                    .use_foundation
            });
            if ui
                .add_enabled(
                    !self.input_locked,
                    egui::Checkbox::new(&mut value, "Учитывать основание"),
                )
                .changed()
            {
                self.history.push(self.contours.clone());
                for c in self
                    .contours
                    .iter_mut()
                    .filter(|c| slab_ids.contains(&c.id))
                {
                    c.use_foundation = value;
                }
                self.foundation_changed();
            }
        }
        if ids.len() != 1 {
            return;
        }
        let Some(original) = self
            .contours
            .iter()
            .find(|c| c.id == ids[0] && c.kind == Kind::Foundation)
        else {
            return;
        };
        let empty = foundation::Data::default();
        let mut data = std::borrow::Cow::Borrowed(original.foundation.as_ref().unwrap_or(&empty));
        let vertices = foundation::vertices(original).len();
        let units = self.load_units;
        let gravity = self.gravity;
        let mut removed_points = false;
        let mut simplify = false;
        let mut round_coordinates = false;
        let mut round_values = false;
        let editable = self.contour_editable(original.id);
        let geometry_editable = self.contour_geometry_editable(original.id);
        ui.add_enabled_ui(editable, |ui| {
            egui::CollapsingHeader::new("Вершины и точки интерполяции")
                .id_salt(("foundation_points", original.id))
                .show(ui, |ui| {
                    ui.label(format!("C1, {}/м³", units.label()));
                    ui.horizontal_wrapped(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.foundation_round_step)
                                .range(0.01..=10000.)
                                .suffix(" мм")
                                .update_while_editing(false),
                        );
                        round_coordinates = ui
                            .add_enabled(
                                geometry_editable,
                                egui::Button::new("Округлить координаты"),
                            )
                            .on_hover_text("Округлить X и Y вершин основания и внутренних точек")
                            .clicked();
                    });
                    ui.horizontal_wrapped(|ui| {
                        ui.add(
                            egui::DragValue::new(&mut self.foundation_value_round_step)
                                .range(0.01..=1e12)
                                .suffix(format!(" {}/м³", units.label()))
                                .update_while_editing(false),
                        );
                        round_values = ui
                            .button("Округлить C1")
                            .on_hover_text("Округлить C1 всех вершин и внутренних точек")
                            .clicked();
                    });
                    let count = data.interior.len();
                    ui.horizontal_wrapped(|ui| {
                        if ui
                            .add_enabled(
                                count > 0,
                                egui::Button::new("Удалить все внутренние точки"),
                            )
                            .on_hover_text("Удаляет все точки интерполяции внутри основания")
                            .clicked()
                        {
                            data.to_mut().interior.clear();
                            removed_points = true;
                        }
                        simplify = ui
                            .add_enabled(
                                !data.interior.is_empty(),
                                egui::Button::new("Удалить избыточные"),
                            )
                            .on_hover_text("Удалить избыточные точки в однородных зонах C1")
                            .clicked();
                    });
                    let mut remove = None;
                    let row_height = ui.spacing().interact_size.y;
                    egui::ScrollArea::vertical().max_height(280.).show_rows(
                        ui,
                        row_height,
                        vertices + data.interior.len(),
                        |ui, rows| {
                            for row in rows {
                                ui.push_id(row, |ui| {
                                    ui.horizontal(|ui| {
                                        if row < vertices {
                                            ui.label(format!("Вершина {}", row + 1));
                                            let mut v = data.value(row).in_units(units, gravity);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut v)
                                                        .update_while_editing(false)
                                                        .range(0. ..=1e12),
                                                )
                                                .changed()
                                            {
                                                let values = &mut data.to_mut().values;
                                                if values.len() <= row {
                                                    values
                                                        .resize(row + 1, foundation::default_c1());
                                                }
                                                values[row] = crate::plan_trace::loads::Magnitude {
                                                    value: v,
                                                    units,
                                                };
                                            }
                                        } else {
                                            let i = row - vertices;
                                            let mut p = data.interior[i].clone();
                                            ui.label(format!("Точка {}", i + 1));
                                            let mut changed = false;
                                            for xy in &mut p.xy {
                                                changed |= ui
                                                    .add(
                                                        egui::DragValue::new(xy)
                                                            .update_while_editing(false)
                                                            .speed(10.),
                                                    )
                                                    .changed();
                                            }
                                            let mut v = p.c1.in_units(units, gravity);
                                            if ui
                                                .add(
                                                    egui::DragValue::new(&mut v)
                                                        .update_while_editing(false)
                                                        .range(0. ..=1e12),
                                                )
                                                .changed()
                                            {
                                                p.c1 = crate::plan_trace::loads::Magnitude {
                                                    value: v,
                                                    units,
                                                };
                                                changed = true;
                                            }
                                            if changed {
                                                data.to_mut().interior[i] = p;
                                            }
                                            if ui
                                                .button("−")
                                                .on_hover_text("Удалить точку")
                                                .clicked()
                                            {
                                                remove = Some(i);
                                            }
                                        }
                                    });
                                });
                            }
                        },
                    );
                    if let Some(i) = remove {
                        data.to_mut().interior.remove(i);
                        removed_points = true;
                    }
                });
            let mut horizontal_enabled = data.horizontal_enabled;
            if ui
                .checkbox(&mut horizontal_enabled, "Горизонтальная связь XY")
                .changed()
            {
                data.to_mut().horizontal_enabled = horizontal_enabled;
            }
            if data.horizontal_enabled {
                ui.horizontal(|ui| {
                    ui.label(format!("Kxy, {}/м", units.label()));
                    let mut v = data.horizontal.in_units(units, gravity);
                    if ui
                        .add(
                            egui::DragValue::new(&mut v)
                                .update_while_editing(false)
                                .range(0. ..=1e12),
                        )
                        .changed()
                    {
                        data.to_mut().horizontal =
                            crate::plan_trace::loads::Magnitude { value: v, units };
                    }
                });
            }
        });
        if round_values {
            data.to_mut()
                .round_values(vertices, self.foundation_value_round_step, units, gravity);
        }
        if !round_coordinates && !simplify && matches!(data, std::borrow::Cow::Borrowed(_)) {
            return;
        }
        let mut candidate = original.clone();
        if let std::borrow::Cow::Owned(data) = data {
            candidate.foundation = Some(data);
        }
        let mut edit = || -> Result<Option<String>, String> {
            if round_coordinates {
                let step = self.foundation_round_step;
                let round = |p: &mut Point| *p = p.map(|v| (v / step).round() * step);
                if candidate.curves.is_empty() {
                    candidate.points.iter_mut().for_each(round);
                } else {
                    candidate.curves.iter_mut().for_each(|c| c.map(round));
                }
                if let Some(data) = &mut candidate.foundation {
                    data.map_xy(round);
                }
                candidate.rebuild()?;
                model::validate_candidate(&self.contours, &candidate)?;
                foundation::Field::new(&candidate, gravity)?;
                return Ok(Some("Координаты основания округлены".into()));
            }
            if simplify {
                let count = foundation::simplify_interior(&mut candidate, gravity)?;
                removed_points |= count > 0;
                return Ok(Some(format!("Удалено избыточных точек C1: {count}")));
            }
            if let Some(data) = &candidate.foundation {
                data.validate(&candidate)?;
            }
            Ok(None)
        };
        let status = match edit() {
            Ok(status) => status,
            Err(e) => {
                self.status = e;
                return;
            }
        };
        if candidate != *original {
            let id = original.id;
            self.history.push(self.contours.clone());
            *self.contours.iter_mut().find(|c| c.id == id).unwrap() = candidate;
            if removed_points {
                self.foundation_interaction.edit = None;
            }
            self.foundation_changed();
        }
        if let Some(status) = status {
            self.status = status;
        }
    }
    pub(super) fn paint_foundation_values(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        camera: Camera,
    ) {
        let mut label_cells = std::collections::BTreeSet::new();
        for c in self.contours.iter().filter(|c| {
            c.kind == Kind::Foundation && self.contour_visible(c) && self.is_selected(c.id)
        }) {
            let default_data = foundation::Data::default();
            let data = c.foundation.as_ref().unwrap_or(&default_data);
            let vertices = foundation::vertices(c);
            for (xy, value) in vertices
                .iter()
                .enumerate()
                .map(|(i, &p)| (p, data.value(i)))
                .chain(data.interior.iter().map(|p| (p.xy, p.c1)))
            {
                let at = camera.screen(xy, rect);
                if !rect.expand(8.).contains(at) {
                    continue;
                }
                painter.circle_filled(at, 4., Color32::LIGHT_GREEN);
                if !self.show_load_values
                    || !label_cells.insert(((at.x / 90.) as i32, (at.y / 21.) as i32))
                {
                    continue;
                }
                painter.text(
                    at + Vec2::new(7., -7.),
                    egui::Align2::LEFT_BOTTOM,
                    format!(
                        "C1={} {}/м³",
                        self.display_text
                            .number(value.in_units(self.load_units, self.gravity)),
                        self.load_units.label()
                    ),
                    egui::FontId::proportional(self.display_text.size),
                    self.display_text.ink(),
                );
            }
        }
    }
}

pub(super) fn draw_foundation_exclusions(
    painter: &egui::Painter,
    c: &Contour,
    camera: Camera,
    rect: Rect,
    stroke: Stroke,
) {
    if let Some(data) = &c.foundation {
        for ring in &data.exclusions {
            draw_path(painter, ring, true, camera, rect, stroke);
        }
    }
}

#[cfg(test)]
mod integration_tests {
    use super::*;
    #[test]
    fn selected_foundation_labels_follow_load_values_and_keep_point_markers() {
        let mut app = App::default();
        app.begin(Kind::Foundation);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([-100., -100.], false);
        app.sketch_click([100., 100.], false);
        let id = app.contours[0].id;
        app.select_only(Some(id));
        app.show_loads = false;
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(400., 400.));
        let ctx = egui::Context::default();
        for show in [false, true] {
            app.show_load_values = show;
            let out = ctx.run(
                egui::RawInput {
                    screen_rect: Some(rect),
                    ..Default::default()
                },
                |ctx| {
                    let painter = ctx.layer_painter(egui::LayerId::background());
                    app.paint_foundation_values(&painter, rect, Camera::default());
                },
            );
            assert_eq!(
                out.shapes.iter().any(|s| {
                    matches!(&s.shape, egui::Shape::Text(t) if t.galley.job.text.starts_with("C1="))
                }),
                show
            );
            assert!(out.shapes.iter().any(|s| {
                matches!(&s.shape, egui::Shape::Circle(c) if c.fill == Color32::LIGHT_GREEN)
            }));
        }
    }

    #[test]
    fn foundation_editing_units_interpolation_points_and_archive_roundtrip() {
        let mut a = App::default();
        a.begin(Kind::Foundation);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([2000., 2000.], false);
        a.cancel();
        let id = a.contours[0].id;
        assert_eq!(a.contours[0].kind, Kind::Foundation);
        a.contours[0]
            .foundation
            .get_or_insert_with(Default::default)
            .exclusions = vec![vec![[400., 400.], [600., 400.], [600., 600.], [400., 600.]]];
        a.foundation_point_target = Some(id);
        a.place_foundation_point([500., 500.]);
        assert!(a.contours[0]
            .foundation
            .as_ref()
            .unwrap()
            .interior
            .is_empty());
        a.place_foundation_point([1000., 1000.]);
        assert_eq!(a.contours[0].foundation.as_ref().unwrap().interior.len(), 1);
        let mut moved = a.contours[0].clone();
        moved.translate([100., -50.]).unwrap();
        assert_eq!(moved.foundation.unwrap().interior[0].xy, [1100., 950.]);
        a.set_kind_lock(Kind::Foundation, true);
        assert!(a.is_kind_locked(Kind::Foundation));
        a.set_kind_lock(Kind::Foundation, false);
        a.foundation_selectable = false;
        assert!(!a.selectable(&a.contours[0]));
        a.foundation_selectable = true;
        let restored: Vec<Contour> =
            serde_json::from_slice(&serde_json::to_vec(&a.contours).unwrap()).unwrap();
        assert_eq!(a.contours, restored);
    }
}
impl App {
    pub(super) fn spatial_foundations(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        scene: &crate::plan_trace::fem::shell::Scene,
    ) {
        let mut b = spatial_depth::Builder::new(rect, projected);
        let mut occlusion = None;
        let mut label_cells = std::collections::BTreeSet::new();
        for c in self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Foundation && self.contour_visible(c))
        {
            let z = self.source_heights(c)[0];
            let ink = if self.is_selected(c.id) {
                Color32::YELLOW
            } else {
                self.object_ink(c.id, c.kind)
            };
            for ring in std::iter::once(&c.points)
                .chain(c.foundation.iter().flat_map(|data| &data.exclusions))
            {
                for i in 0..ring.len() {
                    let a = ring[i];
                    let d = ring[(i + 1) % ring.len()];
                    b.line(
                        [project([a[0], a[1], z]), project([d[0], d[1], z])],
                        1.5,
                        ink,
                    );
                }
            }
            if self.is_selected(c.id) {
                let default_data = foundation::Data::default();
                let data = c.foundation.as_ref().unwrap_or(&default_data);
                let vertices = foundation::vertices(c);
                for (xy, c1) in vertices
                    .iter()
                    .enumerate()
                    .map(|(i, &p)| (p, data.value(i)))
                    .chain(data.interior.iter().map(|p| (p.xy, p.c1)))
                {
                    let at = project([xy[0], xy[1], z]);
                    if !rect.expand(8.).contains(at.0) {
                        continue;
                    }
                    b.marker(at, true);
                    if !self.show_load_values {
                        continue;
                    }
                    let label_cell = ((at.0.x / 90.) as i32, (at.0.y / 21.) as i32);
                    if label_cells.contains(&label_cell) {
                        continue;
                    }
                    if occlusion
                        .get_or_insert_with(|| {
                            spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                        })
                        .visible(at.0, at.1)
                    {
                        label_cells.insert(label_cell);
                        painter.text(
                            at.0 + Vec2::new(7., -7.),
                            egui::Align2::LEFT_BOTTOM,
                            format!(
                                "C1={} {}/м³",
                                self.display_text
                                    .number(c1.in_units(self.load_units, self.gravity)),
                                self.load_units.label()
                            ),
                            egui::FontId::proportional(self.display_text.size),
                            self.display_text.ink(),
                        );
                    }
                }
            }
        }
        spatial_depth::paint_layer(
            painter,
            rect,
            std::sync::Arc::new(b.vertices),
            spatial_depth::Layer::Foundation,
        );
    }
}

#[derive(Default)]
pub(super) struct Interaction {
    drag: Option<PointDrag>,
    edit: Option<ValueEdit>,
}
struct PointDrag {
    id: u64,
    index: usize,
    before: Vec<Contour>,
}
struct ValueEdit {
    id: u64,
    index: usize,
    internal: bool,
    text: String,
    at: Pos2,
    focus: bool,
    units: crate::plan_trace::loads::ForceUnits,
}
struct ValueHit {
    id: u64,
    index: usize,
    internal: bool,
    at: Pos2,
    value: crate::plan_trace::loads::Magnitude,
}
impl App {
    pub(super) fn foundation_button(&self, ui: &mut egui::Ui, enabled: bool) -> egui::Response {
        let r = ui.add_enabled(
            enabled,
            egui::Button::new("")
                .min_size(egui::vec2(42., 34.))
                .selected(self.mode == Some(Kind::Foundation)),
        );
        let p = ui.painter().with_clip_rect(r.rect);
        let origin = r.rect.center() - egui::vec2(17., 14.);
        let at = |x: f32, y: f32| origin + egui::vec2(x, y);
        let color = if enabled {
            Color32::LIGHT_GREEN
        } else {
            ui.visuals().weak_text_color()
        };
        p.add(egui::Shape::convex_polygon(
            vec![at(2., 10.), at(10., 3.), at(33., 3.), at(26., 10.)],
            Color32::from_gray(185),
            egui::Stroke::new(1.5, color),
        ));
        for x in [7., 17., 27.] {
            let pts = [
                (x, 11.),
                (x, 13.),
                (x - 2., 15.),
                (x + 2., 17.),
                (x - 2., 19.),
                (x + 2., 21.),
                (x, 23.),
                (x, 25.),
            ]
            .map(|(x, y)| at(x, y))
            .to_vec();
            p.add(egui::Shape::line(pts, egui::Stroke::new(1.3, color)));
            p.line_segment(
                [at(x - 3., 26.), at(x + 3., 26.)],
                egui::Stroke::new(1., color),
            );
        }
        r
    }
    fn foundation_value_hits(
        &self,
        project: impl Fn(&Contour, Point) -> Option<Pos2>,
    ) -> Vec<ValueHit> {
        let mut hits = vec![];
        for c in self.contours.iter().filter(|c| {
            c.kind == Kind::Foundation && self.is_selected(c.id) && self.contour_visible(c)
        }) {
            let default_data = foundation::Data::default();
            let d = c.foundation.as_ref().unwrap_or(&default_data);
            for (index, xy) in foundation::vertices(c).into_iter().enumerate() {
                if let Some(at) = project(c, xy) {
                    hits.push(ValueHit {
                        id: c.id,
                        index,
                        internal: false,
                        at,
                        value: d.value(index),
                    });
                }
            }
            for (index, s) in d.interior.iter().enumerate() {
                if let Some(at) = project(c, s.xy) {
                    hits.push(ValueHit {
                        id: c.id,
                        index,
                        internal: true,
                        at,
                        value: s.c1,
                    });
                }
            }
        }
        hits
    }
    pub(super) fn foundation_canvas_input(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: Rect,
        pointer: Option<Pos2>,
    ) -> bool {
        let camera = self.camera;
        let hits = self.foundation_value_hits(|_, p| {
            let at = camera.screen(p, rect);
            rect.expand(8.).contains(at).then_some(at)
        });
        self.foundation_pointer_input(ui, painter, pointer, hits, |_, p| {
            Some(camera.world(p, rect))
        })
    }
    pub(super) fn foundation_spatial_input(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        occlusion: Option<&spatial_results::Occlusion>,
    ) -> bool {
        if self.foundation_interaction.edit.is_some() {
            return true;
        }
        if self.foundation_interaction.drag.is_none()
            && (self.mode.is_some()
                || !self.contours.iter().any(|c| {
                    c.kind == Kind::Foundation && self.is_selected(c.id) && self.contour_visible(c)
                }))
        {
            return false;
        }
        let hits = self.foundation_value_hits(|c, p| {
            let at = project([p[0], p[1], self.source_heights(c)[0]]);
            (rect.expand(8.).contains(at.0) && occlusion.is_none_or(|o| o.visible(at.0, at.1)))
                .then_some(at.0)
        });
        let planes = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Foundation)
            .map(|c| (c.id, self.source_heights(c)[0]))
            .collect::<std::collections::BTreeMap<_, _>>();
        let pointer = ui
            .input(|i| i.pointer.interact_pos())
            .filter(|p| rect.contains(*p) && ui.rect_contains_pointer(rect));
        self.foundation_pointer_input(ui, painter, pointer, hits, |id, p| {
            spatial_edit::plane_camera(project, *planes.get(&id)?).map(|c| c.world(p, rect))
        })
    }
    fn foundation_pointer_input(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        pointer: Option<Pos2>,
        hits: Vec<ValueHit>,
        world: impl Fn(u64, Pos2) -> Option<Point>,
    ) -> bool {
        if self.foundation_interaction.edit.is_some() {
            return true;
        }
        if let Some(drag) = self.foundation_interaction.drag.take() {
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                self.contours = drag.before;
                self.foundation_changed();
            } else if ui.input(|i| i.pointer.primary_down()) {
                if let Some(raw) = pointer.and_then(|p| world(drag.id, p)) {
                    let target = self.snap(raw, Some(drag.id)).map_or(raw, |s| s.point);
                    if let Some(c) = self.contours.iter_mut().find(|c| c.id == drag.id) {
                        if let Some(sample) = c
                            .foundation
                            .as_mut()
                            .filter(|data| data.covers(&c.points, target))
                            .and_then(|data| data.interior.get_mut(drag.index))
                        {
                            if sample.xy != target {
                                sample.xy = target;
                                self.foundation_changed();
                            }
                        }
                    }
                }
                self.foundation_interaction.drag = Some(drag);
                ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
            } else if self.contours != drag.before {
                self.history.push(drag.before);
            }
            self.selection_start = None;
            self.spatial.frame_start = None;
            return true;
        }
        if self.mode.is_some()
            || self.drag.is_some()
            || self.clipboard_op.is_some()
            || self.scaling_op.is_some()
            || self.measurements.active
            || self.arc_placement.is_some()
            || self.foundation_point_target.is_some()
        {
            return false;
        }
        let mut remove = None;
        let mut menu_busy = false;
        for hit in hits.iter().filter(|h| h.internal) {
            let response = ui.interact(
                Rect::from_center_size(hit.at, egui::vec2(16., 16.)),
                egui::Id::new(("foundation_internal_point", hit.id, hit.index)),
                egui::Sense::click(),
            );
            menu_busy |= response
                .context_menu(|ui| {
                    if ui
                        .add_enabled(
                            self.contour_editable(hit.id),
                            egui::Button::new("Удалить вершину"),
                        )
                        .on_disabled_hover_text("Снимите блокировку основания")
                        .clicked()
                    {
                        remove = Some((hit.id, hit.index));
                        ui.close_menu();
                    }
                })
                .is_some();
        }
        if let Some((id, index)) = remove {
            self.remove_foundation_sample(id, index);
            return true;
        }
        if menu_busy {
            return true;
        }
        let Some(pointer) = pointer else {
            return false;
        };
        // The point handle takes precedence over its text and all underlying geometry.
        if let Some(hit) = hits
            .iter()
            .rev()
            .find(|h| h.internal && h.at.distance(pointer) <= 8.)
        {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Grab);
            if ui.input(|i| i.pointer.primary_pressed()) && self.contour_geometry_editable(hit.id) {
                self.foundation_interaction.drag = Some(PointDrag {
                    id: hit.id,
                    index: hit.index,
                    before: self.contours.clone(),
                });
                self.selection_start = None;
                self.spatial.frame_start = None;
            }
            return true;
        }
        if !self.show_load_values {
            return false;
        }
        for hit in hits.iter().rev() {
            let text = format!(
                "C1={} {}/м³",
                self.display_text
                    .number(hit.value.in_units(self.load_units, self.gravity)),
                self.load_units.label()
            );
            let size = painter
                .layout_no_wrap(
                    text,
                    egui::FontId::proportional(self.display_text.size),
                    self.display_text.ink(),
                )
                .size();
            let area = Rect::from_min_size(hit.at + egui::vec2(7., -7. - size.y), size).expand(2.);
            if area.contains(pointer) {
                ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
                if ui.input(|i| i.pointer.primary_pressed()) {
                    if self.contour_editable(hit.id) {
                        self.foundation_interaction.edit = Some(ValueEdit {
                            id: hit.id,
                            index: hit.index,
                            internal: hit.internal,
                            text: hit
                                .value
                                .in_units(self.load_units, self.gravity)
                                .to_string(),
                            at: pointer,
                            focus: true,
                            units: self.load_units,
                        });
                        self.selection_start = None;
                        self.spatial.frame_start = None;
                    } else {
                        self.status = "Снимите блокировку основания для изменения C1".into();
                    }
                }
                return true;
            }
        }
        false
    }
    fn remove_foundation_sample(&mut self, id: u64, index: usize) {
        if !self.contour_editable(id) {
            return;
        }
        let Some(i) = self.contours.iter().position(|c| {
            c.id == id
                && c.kind == Kind::Foundation
                && c.foundation
                    .as_ref()
                    .is_some_and(|d| index < d.interior.len())
        }) else {
            return;
        };
        self.history.push(self.contours.clone());
        self.contours[i]
            .foundation
            .as_mut()
            .unwrap()
            .interior
            .remove(index);
        self.foundation_interaction.edit = None;
        self.foundation_changed();
        self.status = "Вершина интерполяции основания удалена".into();
    }
    pub(super) fn cancel_foundation_interaction(&mut self) {
        if let Some(drag) = self.foundation_interaction.drag.take() {
            self.contours = drag.before;
            self.foundation_changed();
        }
        self.foundation_interaction.edit = None;
    }
    pub(super) fn foundation_value_dialog(&mut self, ctx: &egui::Context) {
        let Some(edit) = self.foundation_interaction.edit.as_mut() else {
            return;
        };
        let mut open = true;
        let mut accept = false;
        let mut cancel = false;
        egui::Window::new("Коэффициент постели C1")
            .id(egui::Id::new("foundation_value_editor"))
            .default_pos(edit.at)
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label(format!("C1, {}/м³", edit.units.label()));
                let r = ui.text_edit_singleline(&mut edit.text);
                if edit.focus {
                    r.request_focus();
                    edit.focus = false;
                }
                ui.horizontal(|ui| {
                    accept = ui.button("Применить").clicked();
                    cancel = ui.button("Отмена").clicked();
                });
                accept |= ui.input(|i| i.key_pressed(egui::Key::Enter));
                cancel |= ui.input(|i| i.key_pressed(egui::Key::Escape));
            });
        if !open || cancel {
            self.foundation_interaction.edit = None;
            return;
        }
        if !accept {
            return;
        }
        let Ok(value) = edit.text.trim().replace(',', ".").parse::<f64>() else {
            self.status = "Введите неотрицательное значение C1".into();
            return;
        };
        if !value.is_finite() || value < 0. {
            self.status = "Введите конечное неотрицательное значение C1".into();
            return;
        }
        let (id, index, internal, units) = (edit.id, edit.index, edit.internal, edit.units);
        if !self.contour_editable(id) {
            self.foundation_interaction.edit = None;
            return;
        }
        let before = self.contours.clone();
        if let Some(c) = self.contours.iter_mut().find(|c| c.id == id) {
            let count = foundation::vertices(c).len();
            let data = c.foundation.get_or_insert_with(Default::default);
            data.values.resize(count, foundation::default_c1());
            let target = if internal {
                data.interior.get_mut(index).map(|s| &mut s.c1)
            } else {
                data.values.get_mut(index)
            };
            if let Some(target) = target {
                *target = crate::plan_trace::loads::Magnitude { value, units };
            }
        }
        if self.contours != before {
            self.history.push(before);
            self.foundation_changed();
        }
        self.foundation_interaction.edit = None;
    }
}
