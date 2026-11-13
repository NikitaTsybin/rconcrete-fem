use super::*;
pub(super) const ANGLES: [f64; 7] = [5., 10., 15., 22.5, 30., 45., 90.];
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct Settings {
    pub grid_nodes: bool,
    pub geometry_nodes: bool,
    pub nearest: bool,
    pub midpoint: bool,
    pub polar: bool,
    pub angle_step: f64,
}
impl Default for Settings {
    fn default() -> Self {
        Self {
            grid_nodes: true,
            geometry_nodes: true,
            nearest: true,
            midpoint: false,
            polar: false,
            angle_step: 45.,
        }
    }
}
impl Settings {
    pub fn valid(self) -> bool {
        ANGLES.contains(&self.angle_step)
    }
}
impl App {
    pub(super) fn snap_controls(&mut self, ui: &mut egui::Ui) {
        if icon_button(ui, self.snap_settings.midpoint, 4)
            .on_hover_text("Середина отрезка")
            .clicked()
        {
            self.snap_settings.midpoint = !self.snap_settings.midpoint;
        }
        let options = &mut self.snap_settings;
        for (icon, value, hint) in [
            (
                0,
                &mut options.grid_nodes,
                "Узлы вспомогательной сетки (если сетка включена)",
            ),
            (
                1,
                &mut options.geometry_nodes,
                "Узлы объектов, DXF и видимой КЭ-сетки",
            ),
            (
                2,
                &mut options.nearest,
                "Ближайшие точки на линиях объектов, DXF и КЭ",
            ),
            (3, &mut options.polar, "Угловая привязка. ПКМ — шаг угла"),
        ] {
            let response = icon_button(ui, *value, icon).on_hover_text(if icon == 3 {
                format!("{hint}: {}°", options.angle_step)
            } else {
                hint.into()
            });
            if response.clicked() {
                *value = !*value;
            }
            if icon == 3 {
                response.context_menu(|ui| {
                    for angle in ANGLES {
                        if ui
                            .selectable_value(&mut options.angle_step, angle, format!("{angle}°"))
                            .clicked()
                        {
                            ui.close_menu();
                        }
                    }
                });
            }
        }
    }
    pub(super) fn polar_anchor(&self) -> Option<Point> {
        if let Some(anchor) = self.temporary_polar_anchor {
            return Some(anchor);
        }
        if !self.snap_settings.polar {
            return None;
        }
        if let Some(anchor) = self.transform_snap_anchor() {
            return Some(anchor);
        }
        if self.measurements.active {
            return if self.measurements.second.is_none() {
                self.measurements.first
            } else {
                None
            };
        }
        if let Some(drag) = &self.drag {
            return Some(drag.anchor);
        }
        if self.mode.is_none() {
            return None;
        }
        self.curve_controls.last().or(self.draft.last()).copied()
    }
    pub(super) fn locked_vertex_direction(&self, ui: &egui::Ui, raw: Point) -> Option<snap::Snap> {
        let key = egui::Id::new("locked_vertex_direction");
        let drag = self.drag.as_ref().filter(|d| d.vertex.is_some());
        let step = self.snap_settings.angle_step.to_radians();
        if !self.snap_settings.polar
            || !ui.input(|i| i.modifiers.shift)
            || step <= 0.
            || drag.is_none()
        {
            ui.ctx()
                .data_mut(|data| data.remove::<(u64, Point, f64)>(key));
            return None;
        }
        let drag = drag?;
        let delta = [raw[0] - drag.anchor[0], raw[1] - drag.anchor[1]];
        let angle = ui.ctx().data_mut(|data| {
            if let Some((id, anchor, angle)) = data.get_temp::<(u64, Point, f64)>(key) {
                if id == drag.id && anchor == drag.anchor {
                    return angle;
                }
            }
            let angle = (delta[1].atan2(delta[0]) / step).round() * step;
            data.insert_temp(key, (drag.id, drag.anchor, angle));
            angle
        });
        let (sin, cos) = angle.sin_cos();
        let distance = delta[0] * cos + delta[1] * sin;
        let projected = [
            drag.anchor[0] + distance * cos,
            drag.anchor[1] + distance * sin,
        ];
        Some(
            self.locked_vertex_intersection(drag.id, drag.anchor, [cos, sin], projected)
                .map_or(
                    snap::Snap {
                        point: projected,
                        kind: snap::Kind::Angle,
                    },
                    |point| snap::Snap {
                        point,
                        kind: snap::Kind::Node,
                    },
                ),
        )
    }

    fn locked_vertex_intersection(
        &self,
        contour: u64,
        anchor: Point,
        direction: Point,
        projected: Point,
    ) -> Option<Point> {
        if !self.snap_settings.geometry_nodes {
            return None;
        }
        let radius = 10. / self.camera.zoom;
        let extents = self.support_extent_paths(Some(contour));
        let alternate = self.dwell_polar_anchor.filter(|a| *a != anchor);
        let alternate_rays = alternate.into_iter().flat_map(|a| {
            let length = (projected[0] - a[0]).hypot(projected[1] - a[1]) + radius;
            (0..(360. / self.snap_settings.angle_step).round() as usize).map(move |i| {
                let angle = (i as f64 * self.snap_settings.angle_step).to_radians();
                [
                    a,
                    [a[0] + length * angle.cos(), a[1] + length * angle.sin()],
                ]
            })
        });
        self.contour_snap_edges(Some(contour))
            .chain(
                extents
                    .iter()
                    .flat_map(|path| path.windows(2).map(|p| [p[0], p[1]])),
            )
            .chain(self.index.nearby_edges(projected, radius))
            .chain(
                (self.show_mesh && self.fe_mesh.is_some())
                    .then(|| self.fe_snap_index.nearby_edges(projected, radius))
                    .into_iter()
                    .flatten(),
            )
            .chain(alternate_rays)
            .filter_map(|edge| {
                let point = snap::axis_intersection(anchor, direction, edge)?;
                let distance = (point[0] - projected[0]).hypot(point[1] - projected[1]);
                (distance <= radius).then_some((distance, point))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))
            .map(|(_, point)| point)
    }

    pub(super) fn polar_snap(&self, p: Point) -> Option<snap::Snap> {
        let point = self
            .polar_anchors()
            .into_iter()
            .filter_map(|anchor| {
                project(anchor, p, self.snap_settings.angle_step, self.camera.zoom)
            })
            .min_by(|a, b| {
                (a[0] - p[0])
                    .hypot(a[1] - p[1])
                    .total_cmp(&(b[0] - p[0]).hypot(b[1] - p[1]))
            })?;
        Some(snap::Snap {
            point,
            kind: snap::Kind::Angle,
        })
    }
    pub(super) fn draw_polar_guides(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        raw: Point,
        active: bool,
    ) {
        for a in self.polar_anchors() {
            self.draw_polar_guides_at(painter, rect, raw, active, a);
        }
    }
    pub(super) fn polar_anchors(&self) -> Vec<Point> {
        let mut anchors = self.polar_anchor().into_iter().collect::<Vec<_>>();
        if self.snap_settings.polar {
            if let Some(a) = self.dwell_polar_anchor {
                if !anchors.contains(&a) {
                    anchors.push(a);
                }
            }
        }
        anchors
    }
    fn draw_polar_guides_at(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        raw: Point,
        active: bool,
        a: Point,
    ) {
        let center_mark = self.camera.screen(a, rect);
        painter.rect_stroke(
            Rect::from_center_size(center_mark, egui::vec2(7., 7.)),
            0.,
            Stroke::new(1.5, Color32::from_rgb(255, 204, 90)),
            egui::StrokeKind::Outside,
        );
        let step = self.snap_settings.angle_step.to_radians();
        let direction = ((raw[1] - a[1]).atan2(raw[0] - a[0]) / step).round() * step;
        let center = self.camera.world(rect.center(), rect);
        let length = (center[0] - a[0]).hypot(center[1] - a[1])
            + rect.size().length() as f64 / self.camera.zoom;
        for i in 0..(360. / self.snap_settings.angle_step).round() as usize {
            let angle = i as f64 * step;
            let highlighted = active
                && (angle.cos() - direction.cos()).hypot(angle.sin() - direction.sin()) < 1e-8;
            let b = [a[0] + length * angle.cos(), a[1] + length * angle.sin()];
            if let Some(line) = clip_line(
                self.camera.screen64(a, rect),
                self.camera.screen64(b, rect),
                rect,
            ) {
                let color = if highlighted {
                    Color32::from_rgb(255, 204, 90)
                } else {
                    Color32::from_rgba_unmultiplied(170, 185, 210, 60)
                };
                painter.extend(egui::Shape::dashed_line(
                    &line,
                    Stroke::new(if highlighted { 1.5_f32 } else { 1_f32 }, color),
                    6.,
                    5.,
                ));
            }
        }
        if active {
            painter.text(
                self.camera.screen(raw, rect) + egui::vec2(14., -18.),
                egui::Align2::LEFT_BOTTOM,
                format!("{:.1}°", direction.to_degrees().rem_euclid(360.)),
                egui::FontId::proportional(12.),
                Color32::from_rgb(255, 204, 90),
            );
        }
    }
}
fn project(anchor: Point, p: Point, step: f64, zoom: f64) -> Option<Point> {
    if !ANGLES.contains(&step)
        || !zoom.is_finite()
        || zoom <= 0.
        || anchor.iter().chain(&p).any(|v| !v.is_finite())
    {
        return None;
    }
    let d = [p[0] - anchor[0], p[1] - anchor[1]];
    if d[0].hypot(d[1]) < 1e-9 {
        return None;
    }
    let step = step.to_radians();
    let angle = (d[1].atan2(d[0]) / step).round() * step;
    let unit = [angle.cos(), angle.sin()];
    let length = d[0] * unit[0] + d[1] * unit[1];
    let q = [anchor[0] + length * unit[0], anchor[1] + length * unit[1]];
    (((q[0] - p[0]).hypot(q[1] - p[1]) * zoom) <= 8.).then_some(q)
}
fn icon_button(ui: &mut egui::Ui, selected: bool, icon: usize) -> egui::Response {
    let r = ui.add(
        egui::Button::new("")
            .min_size(egui::vec2(28., 25.))
            .selected(selected),
    );
    let p = ui.painter();
    let center = r.rect.center();
    let point = |x: f32, y: f32| center + egui::vec2(x, y);
    let ink = if selected {
        ui.visuals().selection.stroke.color
    } else {
        ui.visuals().text_color()
    };
    let stroke = Stroke::new(1.4_f32, ink);
    match icon {
        0 => {
            for x in [-7., 0., 7.] {
                for y in [-7., 0., 7.] {
                    p.circle_filled(point(x, y), 1.4, ink);
                }
            }
        }
        1 => {
            let vertices = [point(-8., 7.), point(-5., -7.), point(8., 3.)];
            for i in 0..3 {
                p.line_segment([vertices[i], vertices[(i + 1) % 3]], stroke);
                p.rect_filled(
                    Rect::from_center_size(vertices[i], Vec2::splat(4.)),
                    0.,
                    ink,
                );
            }
        }
        2 => {
            p.line_segment([point(-9., 6.), point(9., -5.)], stroke);
            p.circle_stroke(point(0., 0.5), 3., stroke);
            p.line_segment([point(0., -8.), point(0., -3.)], stroke);
        }
        4 => {
            p.line_segment([point(-9., 5.), point(9., -5.)], stroke);
            let tip = point(0., 0.);
            p.add(egui::Shape::closed_line(
                vec![
                    tip + egui::vec2(0., -4.),
                    tip + egui::vec2(4., 3.),
                    tip + egui::vec2(-4., 3.),
                ],
                Stroke::new(1.8_f32, ink),
            ));
        }
        _ => {
            p.line_segment([point(-8., 7.), point(9., 7.)], stroke);
            p.line_segment([point(-8., 7.), point(5., -7.)], stroke);
            let arc = (0..=16)
                .map(|i| {
                    let t = -(i as f32) * std::f32::consts::FRAC_PI_4 / 16.;
                    point(-8. + 8. * t.cos(), 7. + 8. * t.sin())
                })
                .collect();
            p.add(egui::Shape::line(arc, stroke));
        }
    }
    r
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn polar_steps_project_only_near_rays_in_all_quadrants_with_pixel_tolerance() {
        for step in ANGLES {
            for i in 0..(360. / step).round() as usize {
                let a = (step * i as f64).to_radians();
                let p = [
                    100. + 1000. * a.cos() - 3. * a.sin(),
                    200. + 1000. * a.sin() + 3. * a.cos(),
                ];
                let q = project([100., 200.], p, step, 1.).unwrap();
                assert!(
                    (q[0] - 100. - 1000. * a.cos()).hypot(q[1] - 200. - 1000. * a.sin()) < 1e-8
                );
            }
        }
        assert!(project([0., 0.], [1000., 20.], 45., 1.).is_none());
        assert!(project([0., 0.], [1000., 20.], 45., 0.2).is_some());
        assert!(project([0., 0.], [0., 0.], 45., 1.).is_none());
        assert!(project([0., 0.], [1., 1.], 0., 1.).is_none());
        assert!(project([0., 0.], [f64::NAN, 1.], 45., 1.).is_none());
    }
    #[test]
    fn flags_separate_nodes_nearest_points_and_visible_finite_element_geometry() {
        let mut a = App::default();
        a.camera.zoom = 1.;
        let mesh = super::super::super::meshing::Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![
                super::super::super::meshing::Node {
                    id: 1,
                    xy: [1000., 0.],
                },
                super::super::super::meshing::Node {
                    id: 2,
                    xy: [1200., 0.],
                },
            ],
            elements: vec![],
            edges: vec![[0, 1]],
            support_boundary: Default::default(),
        };
        a.fe_snap_index = Index::mesh(&mesh);
        a.fe_mesh = Some(mesh);
        a.show_mesh = true;
        assert_eq!(a.snap([1004., 2.], None).unwrap().point, [1000., 0.]);
        a.snap_settings.geometry_nodes = false;
        assert_eq!(a.snap([1004., 2.], None).unwrap().point, [1004., 0.]);
        a.snap_settings.nearest = false;
        assert!(a.snap([1004., 2.], None).is_none());
        a.snap_settings.nearest = true;
        a.show_mesh = false;
        assert!(a.snap([1004., 2.], None).is_none());
        a.construction_grid = super::super::super::grid::Grid {
            enabled: true,
            dx: 100.,
            dy: 100.,
            ..Default::default()
        };
        assert_eq!(a.snap([98., 198.], None).unwrap().point, [100., 200.]);
        a.snap_settings.grid_nodes = false;
        assert!(a.snap([98., 198.], None).is_none());
    }
    #[test]
    fn angular_guides_use_original_vertex_during_drag_without_active_tool() {
        let mut a = App::default();
        a.begin(Kind::Support);
        a.sketch_click([0., 0.], false);
        a.sketch_click([1000., 0.], false);
        a.finish();
        a.cancel();
        let original = a.contours[0].clone();
        a.drag = Some(VertexDrag {
            edge: None,
            id: original.id,
            vertex: Some(0),
            anchor: [0., 0.],
            original,
            before: a.contours.clone(),
        });
        a.snap_settings.polar = true;
        a.snap_settings.angle_step = 45.;
        a.camera.zoom = 1.;
        let snap = a.polar_snap([1000., 1003.]).expect("drag polar projection");
        assert!((snap.point[0] - snap.point[1]).abs() < 1e-8);
        a.snap_settings.polar = false;
        assert!(a.polar_snap([1000., 1003.]).is_none());
    }
    #[test]
    fn locked_vertex_snaps_at_crossings_without_leaving_the_guide() {
        let mut app = App::default();
        app.camera.zoom = 1.;
        app.snap_settings.angle_step = 90.;
        app.begin(Kind::Support);
        app.sketch_click([200., -100.], false);
        app.sketch_click([200., 100.], false);
        app.finish();
        let direction = [1., 0.];
        assert_eq!(
            app.locked_vertex_intersection(99, [0., 0.], direction, [196., 0.]),
            Some([200., 0.])
        );
        assert!(app
            .locked_vertex_intersection(app.contours[0].id, [0., 0.], direction, [196., 0.])
            .is_none());
        app.dwell_polar_anchor = Some([100., 25.]);
        let p = app
            .locked_vertex_intersection(99, [0., 0.], direction, [102., 0.])
            .unwrap();
        assert!((p[0] - 100.).abs() < 1e-9 && p[1].abs() < 1e-9);
    }

    #[test]
    fn polar_keeps_numeric_length_and_enter_on_the_same_direction() {
        let mut a = App::default();
        a.begin(Kind::Support);
        a.snap_settings.polar = true;
        a.snap_settings.angle_step = 45.;
        assert!(a.polar_snap([1000., 4.]).is_none());
        a.sketch_click([0., 0.], false);
        assert!(a.polar_snap([1000., 4.]).is_some());
        a.dynamic_length = "1500".into();
        let p = a.dynamic_endpoint([1000., 4.]).unwrap();
        assert!((p[0] - 1500.).abs() < 1e-9 && p[1].abs() < 1e-9);
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(2000., 800.));
        a.dynamic_rect = Some(rect);
        a.camera.zoom = 0.1;
        let pointer = a.camera.screen([1000., 4.], rect);
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                events: vec![
                    egui::Event::PointerMoved(pointer),
                    egui::Event::Key {
                        key: egui::Key::Enter,
                        physical_key: None,
                        pressed: true,
                        repeat: false,
                        modifiers: egui::Modifiers::NONE,
                    },
                ],
                ..Default::default()
            },
            |ctx| a.dynamic_input(ctx),
        );
        assert!((a.draft.last().unwrap()[0] - 1500.).abs() < 1e-8);
        assert!(a.draft.last().unwrap()[1].abs() < 1e-8);
        a.snap_settings.polar = false;
        assert!(a.polar_snap([2500., 1.]).is_none());
    }
}
