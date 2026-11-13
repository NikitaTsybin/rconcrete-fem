use super::*;
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub(super) struct Measurement {
    pub id: u64,
    pub a: Point,
    pub b: Point,
    #[serde(default)]
    pub offset: Option<f64>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub spatial: Option<Spatial>,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub(super) struct Spatial {
    pub z: [f64; 2],
    pub shift: [f64; 3],
}
impl Measurement {
    pub fn xyz(&self) -> [[f64; 3]; 2] {
        let z = self.spatial.as_ref().map_or([0., 0.], |s| s.z);
        [[self.a[0], self.a[1], z[0]], [self.b[0], self.b[1], z[1]]]
    }

    pub fn length(&self) -> f64 {
        (self.b[0] - self.a[0])
            .hypot(self.b[1] - self.a[1])
            .hypot(self.spatial.as_ref().map_or(0., |s| s.z[1] - s.z[0]))
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.id == 0
            || self.id == u64::MAX
            || self.a.iter().chain(&self.b).any(|v| !v.is_finite())
            || !self.length().is_finite()
            || self.length() <= 1e-6
            || self.offset.is_some_and(|v| !v.is_finite())
            || self
                .spatial
                .as_ref()
                .is_some_and(|s| s.z.iter().chain(&s.shift).any(|v| !v.is_finite()))
        {
            return Err("Некорректный измерительный размер".into());
        }
        Ok(())
    }
}
pub(super) struct EndpointDrag {
    before: Measurement,
    end: usize,
    anchor: Point,
}
pub(super) struct Tool {
    pub drag: Option<EndpointDrag>,
    pub items: Vec<Measurement>,
    pub show: bool,
    pub open: bool,
    pub active: bool,
    pub first: Option<Point>,
    pub second: Option<Point>,
    pub spatial_draft: Vec<[f64; 3]>,
    pub selected: std::collections::BTreeSet<u64>,
    pub selectable: bool,
    next: u64,
}
impl Default for Tool {
    fn default() -> Self {
        Self {
            drag: None,
            items: vec![],
            show: true,
            open: false,
            active: false,
            first: None,
            second: None,
            spatial_draft: vec![],
            selected: Default::default(),
            selectable: true,
            next: 1,
        }
    }
}
impl Tool {
    pub fn restore(items: Vec<Measurement>, show: bool) -> Self {
        let next = items
            .iter()
            .map(|m| m.id)
            .max()
            .unwrap_or(0)
            .saturating_add(1);
        Self {
            items,
            show,
            next,
            ..Default::default()
        }
    }
    pub fn stop(&mut self) {
        self.active = false;
        self.spatial_draft.clear();
        self.first = None;
        self.second = None;
    }
    fn remove(&mut self, id: u64) {
        self.items.retain(|m| m.id != id);
        self.selected.remove(&id);
    }
    fn clear(&mut self) {
        self.spatial_draft.clear();
        self.items.clear();
        self.selected.clear();
        self.first = None;
        self.second = None;
    }
    fn click(&mut self, p: Point) -> Result<Option<u64>, String> {
        self.spatial_draft.clear();
        if !self.active {
            return Ok(None);
        }
        if p.iter().any(|v| !v.is_finite()) {
            return Err("Некорректная точка измерения".into());
        }
        if let (Some(a), Some(b)) = (self.first, self.second) {
            if self.items.len() >= 10000 {
                return Err("Достигнут лимит размеров (10000)".into());
            }
            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
            let offset = (-(b[1] - a[1]) * (p[0] - a[0]) + (b[0] - a[0]) * (p[1] - a[1])) / length;
            let m = Measurement {
                spatial: None,
                id: self.next,
                a,
                b,
                offset: Some(offset),
            };
            m.validate()?;
            self.next = self
                .next
                .checked_add(1)
                .ok_or("Исчерпаны номера размеров")?;
            let id = m.id;
            self.items.push(m);
            self.first = None;
            self.second = None;
            Ok(Some(id))
        } else if let Some(a) = self.first {
            let length = (p[0] - a[0]).hypot(p[1] - a[1]);
            if !length.is_finite() || length <= 1e-6 {
                return Err("Точки размера должны различаться и задавать конечную длину".into());
            }
            self.second = Some(p);
            Ok(None)
        } else {
            self.first = Some(p);
            Ok(None)
        }
    }
}
pub(super) struct Layout {
    pub ends: [Point; 2],
    pub extensions: [[Point; 2]; 2],
    pub center: Pos2,
    pub angle: f32,
}
fn rotate(v: Vec2, angle: f32) -> Vec2 {
    egui::vec2(
        angle.cos() * v.x - angle.sin() * v.y,
        angle.sin() * v.x + angle.cos() * v.y,
    )
}
fn point_line_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let d = b - a;
    let t = if d.length_sq() > 1e-12 {
        ((p - a).dot(d) / d.length_sq()).clamp(0., 1.)
    } else {
        0.
    };
    p.distance(a + d * t)
}
impl App {
    fn start_measuring(&mut self) {
        self.cancel();
        self.measurements.open = true;
        self.measurements.active = true;
        self.measurements.show = true;
        self.status = "Измерение: укажите первую точку".into();
    }
    pub(super) fn measurement_button(&mut self, ui: &mut egui::Ui) {
        let r = ui
            .add(
                egui::Button::new("")
                    .min_size(egui::vec2(28., 25.))
                    .selected(self.measurements.open),
            )
            .on_hover_text("Измерения: две точки и положение размерной линии");
        let p = ui.painter();
        let a = r.rect.center() + egui::vec2(-9., 7.);
        let b = r.rect.center() + egui::vec2(9., -7.);
        let n = egui::vec2(0.614, 0.79);
        let stroke = Stroke::new(
            1.4_f32,
            if self.measurements.open {
                Color32::LIGHT_BLUE
            } else {
                ui.visuals().text_color()
            },
        );
        p.line_segment([a, b], stroke);
        p.line_segment([a + n * 6., b + n * 6.], stroke);
        for i in 0..=5 {
            let q = a + (b - a) * (i as f32 / 5.);
            p.line_segment([q, q + n * if i % 2 == 0 { 5. } else { 3. }], stroke);
        }
        if r.clicked() {
            if self.measurements.open {
                self.measurements.open = false;
                self.measurements.stop();
            } else {
                self.start_measuring();
            }
        }
    }
    pub(super) fn measurement_window(&mut self, ctx: &egui::Context) {
        if !self.measurements.open {
            return;
        }
        let mut open = true;
        let mut start = false;
        let mut remove = None;
        let mut select = None;
        egui::Window::new("Измерения")
            .id(egui::Id::new("measurement_window"))
            .open(&mut open)
            .collapsible(false)
            .default_width(340.)
            .default_pos(ctx.screen_rect().center() + egui::vec2(100., -150.))
            .show(ctx, |ui| {
                ui.horizontal(|ui| {
                    if self.measurements.active {
                        if ui.button("Завершить измерение").clicked() {
                            self.measurements.stop();
                        }
                    } else {
                        start = ui.button("Добавить размер").clicked();
                    }
                    if ui
                        .add_enabled(
                            !self.measurements.items.is_empty(),
                            egui::Button::new("Очистить всё"),
                        )
                        .clicked()
                    {
                        self.measurements.clear();
                    }
                });
                if self.measurements.active {
                    ui.add(
                        egui::Label::new(if self.measurements.second.is_some() {
                            "Укажите положение размерной линии. Esc — закончить."
                        } else if self.measurements.first.is_some() {
                            "Укажите вторую точку. Esc — закончить."
                        } else {
                            "Укажите первую точку. Esc — закончить."
                        })
                        .wrap(),
                    );
                }
                ui.separator();
                if self.measurements.items.is_empty() {
                    ui.label("Размеров пока нет.");
                } else {
                    ui.horizontal(|ui| {
                        ui.add_sized([90., 20.], egui::Label::new("Размер"));
                        ui.label("Длина, мм");
                    });
                    egui::ScrollArea::vertical()
                        .id_salt("measurement_rows")
                        .max_height(280.)
                        .show_rows(ui, 24., self.measurements.items.len(), |ui, range| {
                            for index in range {
                                let m = &self.measurements.items[index];
                                ui.push_id(m.id, |ui| {
                                    ui.horizontal(|ui| {
                                        if ui
                                            .add_enabled(
                                                self.measurements.show
                                                    && self.measurements.selectable,
                                                egui::Button::new(format!("Размер {}", m.id))
                                                    .selected(
                                                        self.measurements.selected.contains(&m.id),
                                                    ),
                                            )
                                            .clicked()
                                        {
                                            select = Some(m.id);
                                        }
                                        ui.add_sized(
                                            [120., 20.],
                                            egui::Label::new(self.display_text.number(m.length()))
                                                .truncate(),
                                        )
                                        .on_hover_text(
                                            format!(
                                                "От X {:.3}, Y {:.3} до X {:.3}, Y {:.3} мм",
                                                m.a[0], m.a[1], m.b[0], m.b[1]
                                            ),
                                        );
                                        if ui
                                            .small_button("−")
                                            .on_hover_text("Удалить размер")
                                            .clicked()
                                        {
                                            remove = Some(m.id);
                                        }
                                    });
                                });
                            }
                        });
                }
            });
        if let Some(id) = remove {
            self.measurements.remove(id);
        }
        if let Some(id) = select {
            self.measurements.stop();
            self.select_all_scene([], [], [id], egui::Modifiers::default());
        }
        if !open {
            self.measurements.open = false;
            self.measurements.stop();
        }
        if start {
            self.start_measuring();
        }
    }
    pub(super) fn spatial_measurements(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        occlusion: Option<&spatial_results::Occlusion>,
        input: bool,
    ) -> bool {
        let active = self.measurements.active;
        if !active && (!self.measurements.show || self.measurements.items.is_empty()) {
            return false;
        }
        let pointer = response.hover_pos().filter(|p| rect.contains(*p));
        let floor_z = if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        };
        let target = if active && input {
            pointer.and_then(|p| {
                self.spatial_vertex_snap(p, project, projected, rect, None)
                    .map(|s| s.1)
                    .or_else(|| {
                        spatial_edit::plane_camera(project, floor_z).map(|camera| {
                            let xy = camera.world(p, rect);
                            [xy[0], xy[1], floor_z]
                        })
                    })
            })
        } else {
            None
        };
        // Map a screen displacement back into the camera-facing world plane.
        // Orthographic projection has rank two even when XY is viewed edge-on.
        let screen_shift = |ends: [[f64; 3]; 2], pointer: Pos2| {
            let a = project(ends[0]).0;
            let b = project(ends[1]).0;
            let d = b - a;
            let normal = if d.length() > 1e-4 {
                egui::vec2(-d.y, d.x) / d.length()
            } else {
                egui::vec2(1., 0.)
            };
            let delta = normal * (pointer - (a + (b - a) * 0.5)).dot(normal);
            let origin = project([0., 0., 0.]).0;
            let axes: [Vec2; 3] = std::array::from_fn(|i| {
                let mut p = [0.; 3];
                p[i] = 1000.;
                (project(p).0 - origin) / 1000.
            });
            let xx = axes.iter().map(|v| (v.x as f64).powi(2)).sum::<f64>();
            let yy = axes.iter().map(|v| (v.y as f64).powi(2)).sum::<f64>();
            let xy = axes.iter().map(|v| v.x as f64 * v.y as f64).sum::<f64>();
            let det = xx * yy - xy * xy;
            if det <= 1e-30 {
                return [0.; 3];
            }
            let u = (yy * delta.x as f64 - xy * delta.y as f64) / det;
            let v = (xx * delta.y as f64 - xy * delta.x as f64) / det;
            axes.map(|a| a.x as f64 * u + a.y as f64 * v)
        };
        let mut dimensions = self
            .measurements
            .items
            .iter()
            .filter(|m| self.measurements.show && !self.dimension_hidden(m.id))
            .cloned()
            .collect::<Vec<_>>();
        if active {
            if let (Some(&a), Some(b)) = (
                self.measurements.spatial_draft.first(),
                self.measurements.spatial_draft.get(1).copied().or(target),
            ) {
                let ends = [a, b];
                let shift = if self.measurements.spatial_draft.len() == 2 {
                    pointer.map_or([0.; 3], |p| screen_shift(ends, p))
                } else {
                    [0.; 3]
                };
                dimensions.push(Measurement {
                    id: 0,
                    a: [a[0], a[1]],
                    b: [b[0], b[1]],
                    offset: None,
                    spatial: Some(Spatial {
                        z: [a[2], b[2]],
                        shift,
                    }),
                });
            }
        }
        let painter = ui.painter().with_clip_rect(rect);
        let mut builder = spatial_depth::Builder::new(rect, projected);
        let mut hit = None;
        let mut labels = Vec::new();
        for m in &dimensions {
            let mut ends = m.xyz();
            if m.spatial.is_none() {
                for p in &mut ends {
                    p[2] = floor_z;
                }
            }
            let shift = m.spatial.as_ref().map_or([0.; 3], |s| s.shift);
            let moved = ends.map(|p| std::array::from_fn(|a| p[a] + shift[a]));
            let projected_ends = moved.map(project);
            let selected = self.measurements.selected.contains(&m.id);
            let ink = if selected {
                Color32::LIGHT_BLUE
            } else {
                Color32::from_rgb(255, 215, 110)
            };
            builder.line(projected_ends, if selected { 2.2 } else { 1.4 }, ink);
            for i in 0..2 {
                builder.line([project(ends[i]), projected_ends[i]], 1., ink);
            }
            let a = projected_ends[0].0;
            let b = projected_ends[1].0;
            let d = b - a;
            if d.length() > 1e-4 {
                let tick = egui::vec2(-d.y, d.x).normalized() * 3.;
                for end in projected_ends {
                    builder.line([(end.0 - tick, end.1), (end.0 + tick, end.1)], 1.4, ink);
                }
            }
            let center = a + d * 0.5;
            let depth = (projected_ends[0].1 + projected_ends[1].1) * 0.5;
            if rect.contains(center)
                && occlusion.is_none_or(|o| o.visible(center, depth))
                && m.length() > 1e-6
            {
                let mut angle = d.y.atan2(d.x);
                if angle > std::f32::consts::FRAC_PI_2 {
                    angle -= std::f32::consts::PI;
                }
                if angle < -std::f32::consts::FRAC_PI_2 {
                    angle += std::f32::consts::PI;
                }
                let layout = Layout {
                    ends: [[a.x as f64, a.y as f64], [b.x as f64, b.y as f64]],
                    extensions: [[[0.; 2]; 2]; 2],
                    center,
                    angle,
                };
                labels.push((m.clone(), layout));
                if m.id != 0
                    && pointer.is_some_and(|p| {
                        point_line_distance(p, a, b) < 8. || p.distance(center) < 20.
                    })
                {
                    hit = Some(m.id);
                }
            }
        }
        spatial_depth::paint_layer(
            &painter,
            rect,
            std::sync::Arc::new(builder.vertices),
            spatial_depth::Layer::Measurements,
        );
        for (m, layout) in labels {
            let center = layout.center;
            let angle = layout.angle;
            let box_ = self.measurement_label_box(&painter, &m, &layout);
            painter.add(egui::Shape::convex_polygon(
                box_.to_vec(),
                Color32::from_rgba_unmultiplied(20, 24, 30, 235),
                Stroke::NONE,
            ));
            let galley = painter.layout_no_wrap(
                self.measurement_text(&m),
                egui::FontId::proportional(self.display_text.size),
                self.display_text.ink(),
            );
            let at = center - rotate(galley.size() * 0.5, angle);
            painter.add(
                egui::epaint::TextShape::new(at, galley, self.display_text.ink()).with_angle(angle),
            );
        }
        if active && input {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
            if let Some(target) = target {
                let at = project(target).0;
                painter.circle_stroke(at, 5., Stroke::new(1.5, Color32::LIGHT_BLUE));
            }
            if response.clicked_by(egui::PointerButton::Primary) {
                if self.measurements.spatial_draft.len() == 2 {
                    if let Some(p) = pointer {
                        let a = self.measurements.spatial_draft[0];
                        let b = self.measurements.spatial_draft[1];
                        let m = Measurement {
                            id: self.measurements.next,
                            a: [a[0], a[1]],
                            b: [b[0], b[1]],
                            offset: None,
                            spatial: Some(Spatial {
                                z: [a[2], b[2]],
                                shift: screen_shift([a, b], p),
                            }),
                        };
                        if self.measurements.items.len() >= 10000 {
                            self.status = "Достигнут лимит размеров (10000)".into();
                        } else if let Err(e) = m.validate() {
                            self.status = e;
                        } else {
                            self.measurements.next = self.measurements.next.saturating_add(1);
                            self.measurements.items.push(m);
                            self.measurements.spatial_draft.clear();
                            self.measurements.first = None;
                            self.measurements.second = None;
                            self.status = "Размер добавлен. Укажите первую точку следующего".into();
                        }
                    }
                } else if let Some(p) = target {
                    if self.measurements.spatial_draft.first().is_none_or(|a| {
                        a.iter().zip(p).map(|(a, b)| (a - b).powi(2)).sum::<f64>() > 1e-12
                    }) {
                        self.measurements.spatial_draft.push(p);
                        if self.measurements.spatial_draft.len() == 1 {
                            self.measurements.first = Some([p[0], p[1]]);
                            self.status = "Измерение: укажите вторую точку в 3D".into();
                        } else {
                            self.measurements.second = Some([p[0], p[1]]);
                            self.status = "Укажите положение размерной линии".into();
                        }
                    }
                }
            }
        } else if input
            && self.measurements.selectable
            && response.clicked_by(egui::PointerButton::Primary)
        {
            if let Some(id) = hit {
                self.select_all_scene([], [], [id], ui.input(|i| i.modifiers));
                return true;
            }
        }
        active
    }
    pub(super) fn draw_measurements(&self, painter: &egui::Painter, rect: Rect) {
        if self.measurements.show {
            for m in self
                .measurements
                .items
                .iter()
                .filter(|m| !self.dimension_hidden(m.id))
            {
                self.draw_dimension(painter, rect, m);
            }
        }
    }
    pub(super) fn dimension_layout(&self, m: &Measurement, rect: Rect) -> Option<Layout> {
        let a = self.camera.screen64(m.a, rect);
        let b = self.camera.screen64(m.b, rect);
        let d = [b[0] - a[0], b[1] - a[1]];
        let length = d[0].hypot(d[1]);
        let h = m.offset.map_or(14., |v| -v * self.camera.zoom);
        if !length.is_finite()
            || length < 1e-6
            || !h.is_finite()
            || a.iter().chain(&b).any(|v| !v.is_finite())
        {
            return None;
        }
        let normal = [-d[1] / length, d[0] / length];
        let shift = |p: Point, h: f64| [p[0] + normal[0] * h, p[1] + normal[1] * h];
        let ends = [shift(a, h), shift(b, h)];
        let sign = if h < 0. { -1. } else { 1. };
        let extensions = [
            [shift(a, -3. * sign), shift(a, h + 5. * sign)],
            [shift(b, -3. * sign), shift(b, h + 5. * sign)],
        ];
        let clipped = clip_line(ends[0], ends[1], rect)?;
        let middle = egui::pos2(
            (ends[0][0] + d[0] * 0.5) as f32,
            (ends[0][1] + d[1] * 0.5) as f32,
        );
        let center = if rect.contains(middle) {
            middle
        } else {
            clipped[0] + (clipped[1] - clipped[0]) * 0.5
        };
        let mut angle = (d[1].atan2(d[0])) as f32;
        if angle > std::f32::consts::FRAC_PI_2 {
            angle -= std::f32::consts::PI;
        }
        if angle < -std::f32::consts::FRAC_PI_2 {
            angle += std::f32::consts::PI;
        }
        Some(Layout {
            ends,
            extensions,
            center,
            angle,
        })
    }
    fn measurement_text(&self, m: &Measurement) -> String {
        format!("{} мм", self.display_text.number(m.length()))
    }
    pub(super) fn measurement_label_box(
        &self,
        painter: &egui::Painter,
        m: &Measurement,
        l: &Layout,
    ) -> [Pos2; 4] {
        let size = painter
            .layout_no_wrap(
                self.measurement_text(m),
                egui::FontId::proportional(self.display_text.size),
                self.display_text.ink(),
            )
            .size()
            + egui::vec2(8., 4.);
        [
            egui::vec2(-0.5, -0.5),
            egui::vec2(0.5, -0.5),
            egui::vec2(0.5, 0.5),
            egui::vec2(-0.5, 0.5),
        ]
        .map(|v| l.center + rotate(egui::vec2(v.x * size.x, v.y * size.y), l.angle))
    }
    pub(super) fn measurement_hit(
        &self,
        painter: &egui::Painter,
        p: Pos2,
        rect: Rect,
    ) -> Option<u64> {
        if !self.measurements.show || !self.measurements.selectable {
            return None;
        }
        self.measurements
            .items
            .iter()
            .rev()
            .filter(|m| !self.dimension_hidden(m.id))
            .find_map(|m| {
                let l = self.dimension_layout(m, rect)?;
                let lines = std::iter::once(l.ends).chain(l.extensions);
                let near = lines
                    .filter_map(|s| clip_line(s[0], s[1], rect))
                    .any(|[a, b]| point_line_distance(p, a, b) <= 8.);
                let label = self
                    .measurement_label_box(painter, m, &l)
                    .map(|q| [q.x as f64, q.y as f64]);
                (near || geometry::inside([p.x as f64, p.y as f64], &label)).then_some(m.id)
            })
    }
    fn draw_dimension(&self, painter: &egui::Painter, rect: Rect, m: &Measurement) {
        self.draw_dimension_alpha(painter, rect, m, 1.);
    }
    pub(super) fn draw_dimension_alpha(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        m: &Measurement,
        alpha: f32,
    ) {
        let Some(l) = self.dimension_layout(m, rect) else {
            return;
        };
        let selected = self.measurements.selected.contains(&m.id);
        let ink = (if selected {
            Color32::LIGHT_BLUE
        } else {
            Color32::from_rgb(255, 215, 110)
        })
        .gamma_multiply(alpha);
        let stroke = Stroke::new(if selected { 2.2_f32 } else { 1.4_f32 }, ink);
        for line in std::iter::once(l.ends).chain(l.extensions) {
            if let Some(line) = clip_line(line[0], line[1], rect) {
                painter.line_segment(line, stroke);
            }
        }
        let delta = [l.ends[1][0] - l.ends[0][0], l.ends[1][1] - l.ends[0][1]];
        let length = delta[0].hypot(delta[1]);
        let unit = egui::vec2((delta[0] / length) as f32, (delta[1] / length) as f32);
        let normal = egui::vec2(-unit.y, unit.x);
        for i in 0..2 {
            let p = egui::pos2(l.ends[i][0] as f32, l.ends[i][1] as f32);
            if rect.contains(p) {
                let inward = unit * if i == 0 { 1. } else { -1. };
                painter.add(egui::Shape::convex_polygon(
                    vec![
                        p,
                        p + inward * 7. + normal * 3.,
                        p + inward * 7. - normal * 3.,
                    ],
                    ink,
                    Stroke::NONE,
                ));
            }
        }
        if selected {
            for point in [m.a, m.b] {
                let at = self.camera.screen(point, rect);
                painter.rect_filled(Rect::from_center_size(at, Vec2::splat(8.)), 0., ink);
            }
        }
        let galley = painter.layout_no_wrap(
            self.measurement_text(m),
            egui::FontId::proportional(self.display_text.size),
            self.display_text.ink().gamma_multiply(alpha),
        );
        let box_ = self.measurement_label_box(painter, m, &l);
        painter.add(egui::Shape::convex_polygon(
            box_.to_vec(),
            Color32::from_rgba_unmultiplied(20, 24, 30, 235).gamma_multiply(alpha),
            Stroke::NONE,
        ));
        let origin = l.center - rotate(galley.size() * 0.5, l.angle);
        painter.add(
            egui::epaint::TextShape::new(
                origin,
                galley,
                self.display_text.ink().gamma_multiply(alpha),
            )
            .with_angle(l.angle),
        );
        if selected {
            if let Some(point) = self.dimension_position_handle(m, rect) {
                let at = self.camera.screen(point, rect);
                painter.rect_filled(
                    Rect::from_center_size(at, Vec2::splat(10.)),
                    0.,
                    Color32::from_rgb(20, 24, 30),
                );
                painter.rect_filled(Rect::from_center_size(at, Vec2::splat(6.)), 0., ink);
            }
        }
    }
    fn dimension_position_handle(&self, m: &Measurement, rect: Rect) -> Option<Point> {
        let layout = self.dimension_layout(m, rect)?;
        // Place on the dimension line away from the central value label.
        let at = Pos2::new(
            (layout.ends[0][0] * 0.75 + layout.ends[1][0] * 0.25) as f32,
            (layout.ends[0][1] * 0.75 + layout.ends[1][1] * 0.25) as f32,
        );
        Some(self.camera.world(at, rect))
    }
    pub(super) fn cancel_measurement_drag(&mut self) {
        if let Some(drag) = self.measurements.drag.take() {
            if let Some(m) = self
                .measurements
                .items
                .iter_mut()
                .find(|m| m.id == drag.before.id)
            {
                *m = drag.before;
            }
        }
    }
    pub(super) fn measurement_drag_canvas(
        &mut self,
        ui: &egui::Ui,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> bool {
        if !self.measurements.show || !self.measurements.selectable {
            self.cancel_measurement_drag();
            return false;
        }
        if self.measurements.drag.is_none() {
            if self.mode.is_some()
                || self.measurements.active
                || self.scaling_op.is_some()
                || self.clipboard_op.is_some()
                || self.drag.is_some()
                || self.arc_placement.is_some()
                || self.underlay_drag.is_some()
                || ui.input(|i| i.modifiers.shift || i.modifiers.ctrl)
            {
                return false;
            }
            let Some(pointer) = hovered else {
                return false;
            };
            if !ui.input(|i| i.pointer.primary_pressed()) {
                return false;
            }
            let mut hit: Option<(&Measurement, usize, f32)> = None;
            for m in self
                .measurements
                .items
                .iter()
                .filter(|m| self.measurements.selected.contains(&m.id))
            {
                let handles = [
                    Some(m.a),
                    Some(m.b),
                    self.dimension_position_handle(m, rect),
                ];
                for (end, point) in handles.into_iter().enumerate() {
                    let Some(point) = point else {
                        continue;
                    };
                    let distance = self.camera.screen(point, rect).distance(pointer);
                    if distance <= 9. && hit.as_ref().is_none_or(|(_, _, old)| distance < *old) {
                        hit = Some((m, end, distance));
                    }
                }
            }
            if let Some((m, end, _)) = hit {
                self.measurements.drag = Some(EndpointDrag {
                    before: m.clone(),
                    end,
                    anchor: self.camera.world(pointer, rect),
                });
                self.selection_start = None;
            }
        }
        let Some(drag) = &self.measurements.drag else {
            return false;
        };
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        if let Some(pointer) = hovered {
            let raw = self.camera.world(pointer, rect);
            let original = match drag.end {
                0 => drag.before.a,
                1 => drag.before.b,
                _ => self
                    .dimension_position_handle(&drag.before, rect)
                    .unwrap_or(drag.before.a),
            };
            let raw = [
                original[0] + raw[0] - drag.anchor[0],
                original[1] + raw[1] - drag.anchor[1],
            ];
            let found = self.snap(raw, None);
            let target = found.map_or(raw, |s| s.point);
            let mut next = drag.before.clone();
            if drag.end == 0 {
                next.a = target;
            } else if drag.end == 1 {
                next.b = target;
            } else if (target[0] - original[0]).hypot(target[1] - original[1]) > 1e-6 {
                let delta = [next.b[0] - next.a[0], next.b[1] - next.a[1]];
                next.offset = Some(
                    (-delta[1] * (target[0] - next.a[0]) + delta[0] * (target[1] - next.a[1]))
                        / next.length(),
                );
            }
            if next.validate().is_ok() {
                if let Some(m) = self.measurements.items.iter_mut().find(|m| m.id == next.id) {
                    *m = next;
                }
            }
            if found.is_some() {
                ui.painter().with_clip_rect(rect).circle_stroke(
                    self.camera.screen(target, rect),
                    5.,
                    Stroke::new(1.7_f32, Color32::from_rgb(70, 235, 225)),
                );
            }
        }
        if ui.input(|i| i.pointer.primary_released()) {
            let drag = self.measurements.drag.take().unwrap();
            if self
                .measurements
                .items
                .iter()
                .find(|m| m.id == drag.before.id)
                .is_some_and(|m| {
                    m.a != drag.before.a || m.b != drag.before.b || m.offset != drag.before.offset
                })
            {
                self.record_measurement_edit(drag.before);
            }
        }
        true
    }
    pub(super) fn measurement_canvas(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> Option<Point> {
        if !self.measurements.active {
            return None;
        }
        let p = hovered?;
        let raw = self.camera.world(p, rect);
        let found = self.snap(raw, None).or_else(|| self.polar_snap(raw));
        let target = found.map_or(raw, |s| s.point);
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        let painter = ui.painter().with_clip_rect(rect);
        self.draw_polar_guides(
            &painter,
            rect,
            raw,
            found.is_some_and(|s| s.kind == snap::Kind::Angle),
        );
        if let Some(a) = self.measurements.first {
            let b = self.measurements.second.unwrap_or(target);
            let offset = self.measurements.second.map(|b| {
                let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                (-(b[1] - a[1]) * (target[0] - a[0]) + (b[0] - a[0]) * (target[1] - a[1])) / length
            });
            self.draw_dimension(
                &painter,
                rect,
                &Measurement {
                    spatial: None,
                    id: 0,
                    a,
                    b,
                    offset,
                },
            );
        }
        painter.circle_stroke(
            self.camera.screen(target, rect),
            5.,
            Stroke::new(1.5_f32, Color32::LIGHT_BLUE),
        );
        if response.clicked_by(egui::PointerButton::Primary) {
            match self.measurements.click(target) {
                Ok(Some(id)) => {
                    self.status = format!("Добавлен размер {id}. Укажите первую точку следующего")
                }
                Ok(None) => {
                    self.status = if self.measurements.second.is_some() {
                        "Укажите положение размерной линии"
                    } else {
                        "Измерение: укажите вторую точку"
                    }
                    .into()
                }
                Err(e) => self.status = e,
            }
        }
        Some(target)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn placed_rotated_dimension_can_be_selected_by_line_or_frame_and_deleted() {
        let mut a = App::default();
        a.camera.zoom = 0.5;
        a.camera.angle = 0.3;
        a.measurements.items = vec![Measurement {
            spatial: None,
            id: 1,
            a: [0., 0.],
            b: [0., 400.],
            offset: Some(100.),
        }];
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        let layout = a.dimension_layout(&a.measurements.items[0], rect).unwrap();
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Middle,
                egui::Id::new("dimension"),
            ));
            a.draw_measurements(&painter, rect);
            assert_eq!(a.measurement_hit(&painter, layout.center, rect), Some(1));
            let ids = a.frame_measurement_hits(
                &painter,
                Rect::from_center_size(layout.center, Vec2::splat(12.)),
                rect,
            );
            assert_eq!(ids, vec![1]);
        });
        assert!(out
            .shapes
            .iter()
            .any(|s| matches!(&s.shape,egui::Shape::Text(t) if t.angle.abs()>0.5)));
        a.select_all_scene([], [], [1], egui::Modifiers::default());
        assert!(a.measurements.selected.contains(&1));
        a.remove();
        assert!(a.measurements.items.is_empty());
        assert!(a.contours.is_empty() && a.history.is_empty());
    }
    #[test]
    fn repeated_measurements_delete_clear_and_cancel_keep_only_completed_distances() {
        let mut tool = Tool::default();
        tool.active = true;
        assert!(tool.click([0., 0.]).unwrap().is_none());
        assert!(tool.click([0., 0.]).is_err());
        assert_eq!(tool.first, Some([0., 0.]));
        assert_eq!(tool.click([300., 400.]).unwrap(), None);
        assert!(tool.items.is_empty());
        assert_eq!(tool.click([0., 400.]).unwrap(), Some(1));
        assert!(tool.items[0].offset.unwrap() > 0.);
        assert_eq!(tool.items[0].length(), 500.);
        tool.click([-100., 20.]).unwrap();
        tool.click([900., 20.]).unwrap();
        tool.click([0., -80.]).unwrap();
        assert_eq!(tool.items[1].offset, Some(-100.));
        tool.remove(1);
        assert_eq!(tool.items.len(), 1);
        assert_eq!(tool.items[0].id, 2);
        tool.click([1., 2.]).unwrap();
        tool.stop();
        assert!(tool.first.is_none());
        assert_eq!(tool.items.len(), 1);
        tool.clear();
        assert!(tool.items.is_empty());
        assert!(Measurement {
            spatial: None,
            offset: None,
            id: 1,
            a: [0., 0.],
            b: [f64::INFINITY, 1.]
        }
        .validate()
        .is_err());
    }
    #[test]
    fn measurement_clicks_snap_without_editing_sources_selection_history_or_results() {
        fn frame(ctx: &egui::Context, a: &mut App, events: Vec<egui::Event>) -> Rect {
            let mut rect = Rect::NOTHING;
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        rect = ui.available_rect_before_wrap();
                        a.canvas(ui);
                    });
                },
            );
            rect
        }
        let ctx = egui::Context::default();
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(crate::plan_trace::sketch::Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([6000., 4000.], false);
        let before = a.contours.clone();
        let history = a.history.len();
        a.fit = false;
        a.camera = Camera {
            center: [3000., 2000.],
            zoom: 0.1,
            angle: 0.,
            plane: None,
        };
        a.input_locked = true;
        a.analysis = Some(crate::plan_trace::fem::Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![],
            free_dofs: 0,
        });
        a.start_measuring();
        let rect = frame(&ctx, &mut a, vec![]);
        frame(&ctx, &mut a, vec![]);
        for world in [[5., 6.], [5997., 3995.], [1000., 3000.]] {
            let pos = a.camera.screen(world, rect);
            frame(
                &ctx,
                &mut a,
                vec![
                    egui::Event::PointerMoved(pos),
                    egui::Event::PointerButton {
                        pos,
                        button: egui::PointerButton::Primary,
                        pressed: true,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
            );
            frame(
                &ctx,
                &mut a,
                vec![egui::Event::PointerButton {
                    pos,
                    button: egui::PointerButton::Primary,
                    pressed: false,
                    modifiers: egui::Modifiers::NONE,
                }],
            );
        }
        assert_eq!(a.measurements.items.len(), 1);
        let m = &a.measurements.items[0];
        assert_eq!(m.a, [0., 0.]);
        assert_eq!(m.b, [6000., 4000.]);
        assert!(m.offset.is_some());
        assert_eq!(a.contours, before);
        assert_eq!(a.history.len(), history);
        assert!(a.analysis.is_some());
        assert!(a.drag.is_none() && a.selection_start.is_none());
        assert!(a.selected.is_none());
        a.cancel();
        assert!(!a.measurements.active);
        assert_eq!(a.measurements.items.len(), 1);
    }
    #[test]
    fn dimension_overlay_is_visible_and_hideable_without_changing_data() {
        let ctx = egui::Context::default();
        let mut a = App::default();
        a.measurements.items = vec![Measurement {
            spatial: None,
            offset: None,
            id: 1,
            a: [-100., 0.],
            b: [100., 0.],
        }];
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        let out = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default()
                    .show(ctx, |ui| a.draw_measurements(ui.painter(), rect));
            },
        );
        assert!(out
            .shapes
            .iter()
            .any(|s| matches!(&s.shape,egui::Shape::Text(t) if t.galley.job.text == "200.00 мм")));
        assert_eq!(a.bounds(), Some([-100., 0., 100., 0.]));
        a.measurements.show = false;
        let out = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default()
                    .show(ctx, |ui| a.draw_measurements(ui.painter(), rect));
            },
        );
        assert!(!out.shapes.iter().any(
            |s| matches!(&s.shape,egui::Shape::Text(t) if t.galley.job.text.contains("200.0 мм"))
        ));
        assert_eq!(a.measurements.items.len(), 1);
        assert!(a.bounds().is_none());
    }
}

#[cfg(test)]
mod spatial_dimension_regression {
    use super::*;
    #[test]
    fn true_xyz_distance_and_spatial_annotation_roundtrip() {
        let m = Measurement {
            id: 1,
            a: [0., 0.],
            b: [3000., 0.],
            offset: None,
            spatial: Some(Spatial {
                z: [1000., 5000.],
                shift: [0., 200., 0.],
            }),
        };
        m.validate().unwrap();
        assert_eq!(m.length(), 5000.);
        let saved = serde_json::to_vec(&m).unwrap();
        let restored: Measurement = serde_json::from_slice(&saved).unwrap();
        assert_eq!(restored.xyz(), [[0., 0., 1000.], [3000., 0., 5000.]]);
        assert_eq!(restored.length(), 5000.);
        let legacy: Measurement =
            serde_json::from_str(r#"{"id":2,"a":[0,0],"b":[3000,4000],"offset":null}"#).unwrap();
        assert_eq!(legacy.length(), 5000.);
        assert!(legacy.spatial.is_none());
    }
}
