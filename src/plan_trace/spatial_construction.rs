//! Construction uses the existing XY tools with an affine camera for the active storey.
use super::*;
impl App {
    pub(super) fn spatial_construction_z(&self) -> f64 {
        if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        }
    }

    pub(super) fn paint_spatial_construction_grid(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
    ) {
        if !self.construction_grid.enabled {
            return;
        }
        let z = self.spatial_construction_z();
        let Some(camera) = spatial_edit::plane_camera(project, z) else {
            return;
        };
        let bounds = [
            rect.left_top(),
            rect.right_top(),
            rect.right_bottom(),
            rect.left_bottom(),
        ]
        .into_iter()
        .map(|p| camera.world(p, rect))
        .fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |b, [x, y]| [b[0].min(x), b[1].min(y), b[2].max(x), b[3].max(y)],
        );
        let [r, g, b] = self.construction_grid.color;
        let ink = Color32::from_rgb(r, g, b);
        let mut builder = spatial_depth::Builder::new(rect, projected);
        for [a, b] in self.construction_grid.lines(bounds, camera.zoom) {
            builder.line(
                [project([a[0], a[1], z]), project([b[0], b[1], z])],
                1.25,
                ink,
            );
        }
        spatial_depth::paint_layer(
            painter,
            rect,
            std::sync::Arc::new(builder.vertices),
            spatial_depth::Layer::Editing,
        );
    }

    /// Pick a visible vertex and return both its plan coordinates and its world position.
    pub(super) fn spatial_vertex_snap(
        &self,
        pointer: Pos2,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        rect: Rect,
        exclude: Option<u64>,
    ) -> Option<(snap::Snap, [f64; 3])> {
        if !self.snap_settings.geometry_nodes {
            return None;
        }
        let mut candidates = Vec::new();
        let mut consider = |world| {
            if self.sections.contains(world) {
                let screen = project(world);
                let distance = screen.0.distance(pointer);
                if distance <= 12. {
                    candidates.push((distance, screen, world));
                }
            }
        };
        for c in self
            .contours
            .iter()
            .filter(|c| Some(c.id) != exclude && self.contour_visible(c))
        {
            if c.is_free_beam() {
                self.source_beam_path(c).into_iter().for_each(&mut consider);
                continue;
            }
            for z in self.source_heights(c) {
                let mut handles = c.handles();
                if c.kind == Kind::Column && self.drawing_visibility.support_extents {
                    if let (Some(&p), Some(section)) =
                        (c.points.first(), self.support_height.columns.get(&c.id))
                    {
                        handles.extend(crate::plan_trace::columns::section_corners(c, section));
                    }
                }
                for p in handles {
                    consider([p[0], p[1], z]);
                }
            }
        }
        if self.show_mesh && self.fe_mesh.is_some() && exclude.is_none() {
            if let Some((_, _, _, _, scene)) = &self.spatial.cache {
                for (n, (screen, world)) in projected.iter().zip(&scene.xyz).enumerate() {
                    if !self.spatial_node_view_visible(n, scene) || !self.sections.contains(*world)
                    {
                        continue;
                    }
                    let distance = screen.0.distance(pointer);
                    if distance <= 12. {
                        candidates.push((distance, *screen, *world));
                    }
                }
            }
        }
        candidates.sort_by(|a, b| a.0.total_cmp(&b.0).then_with(|| b.1 .1.total_cmp(&a.1 .1)));
        if candidates.is_empty() {
            return None;
        }
        let occlusion = self
            .spatial
            .cache
            .as_ref()
            .map(|s| spatial_results::Occlusion::new_visible(rect, &s.4, projected, self));
        candidates
            .into_iter()
            .find(|(_, screen, _)| {
                occlusion
                    .as_ref()
                    .is_none_or(|o| o.visible(screen.0, screen.1))
            })
            .map(|(_, _, world)| {
                (
                    snap::Snap {
                        point: [world[0], world[1]],
                        kind: snap::Kind::Node,
                    },
                    world,
                )
            })
    }
    pub(super) fn spatial_construction(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
    ) -> bool {
        let drawing = self.mode.is_some();
        let z = self.spatial_construction_z();
        let origin = project([0., 0., z]).0;
        let x = (project([1000., 0., z]).0 - origin) / 1000.;
        let y = (project([0., 1000., z]).0 - origin) / 1000.;
        let det = x.x * y.y - x.y * y.x;
        if det.abs() < 1e-20 || det.abs() < x.length() * y.length() * 1e-4 {
            if drawing {
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "Для построения поверните вид к плоскости текущего этажа",
                    egui::FontId::proportional(14.),
                    Color32::LIGHT_BLUE,
                );
            }
            self.dynamic_rect = None;
            return drawing;
        }
        let saved = self.camera;
        self.camera = Camera {
            center: [0., 0.],
            zoom: x.length().min(y.length()) as f64,
            angle: 0.,
            plane: Some([
                [origin.x as f64, origin.y as f64],
                [x.x as f64, x.y as f64],
                [y.x as f64, y.y as f64],
            ]),
        };
        let camera = self.camera;
        let hovered = response
            .hover_pos()
            .filter(|p| !self.toolbar_rect.is_some_and(|r| r.contains(*p)));
        let base_snap = if self.section_pick_origin.is_some() || self.grid_pick_origin {
            hovered.and_then(|p| self.spatial_vertex_snap(p, project, projected, rect, None))
        } else {
            None
        };
        if let Some((_, world)) = base_snap {
            let p = project(world).0;
            ui.painter()
                .circle_stroke(p, 6., egui::Stroke::new(2., Color32::YELLOW));
            ui.painter().circle_filled(p, 2., Color32::WHITE);
        }
        if let Some(i) = self.section_pick_origin {
            if let Some(p) = hovered {
                if response.clicked() {
                    let xy = camera.world(p, rect);
                    let world = self
                        .spatial_vertex_snap(p, project, projected, rect, None)
                        .map(|s| s.1)
                        .unwrap_or([xy[0], xy[1], z]);
                    if let Some(plane) = self.sections.planes.get_mut(i) {
                        plane.set_origin(i, world);
                    }
                    self.section_pick_origin = None;
                    self.section_geometry_changed();
                }
            }
            self.camera = saved;
            return true;
        }
        if self.grid_pick_origin {
            if let Some(p) = hovered {
                if response.clicked() {
                    self.construction_grid.center = base_snap
                        .map(|s| s.0.point)
                        .unwrap_or_else(|| camera.world(p, rect));
                    self.grid_pick_origin = false;
                }
            }
            self.camera = saved;
            return true;
        }
        self.dynamic_rect = Some(rect);
        self.hover_pick.surface = Some((rect, ui.layer_id()));
        let mut builder = spatial_depth::Builder::new(rect, projected);
        for (i, plane) in self
            .sections
            .planes
            .iter()
            .enumerate()
            .filter(|(_, p)| p.enabled)
        {
            let n = glam::DVec3::from_array(plane.normal(i));
            let u = if n.z.abs() > 0.5 {
                glam::DVec3::X
            } else {
                glam::DVec3::Z
            };
            let v = n.cross(u);
            let origin = n * plane.offset(i);
            let corners = [(-1., -1.), (1., -1.), (1., 1.), (-1., 1.)]
                .map(|(a, b)| (origin + u * a * 10000. + v * b * 10000.).to_array());
            let points = corners.map(project);
            for j in 0..4 {
                builder.line(
                    [points[j], points[(j + 1) % 4]],
                    2.,
                    Color32::from_rgb(50, 180, 240),
                );
            }
        }
        if drawing {
            if !ui.input(|i| i.modifiers.shift) {
                self.temporary_polar_anchor = None;
            }
            if let Some(pointer) = hovered {
                let raw = camera.world(pointer, rect);
                let cross_floor = self.spatial_vertex_snap(pointer, project, projected, rect, None);
                let node = cross_floor.map(|s| s.0).or_else(|| {
                    self.snap(raw, None)
                        .filter(|s| camera.screen(s.point, rect).distance(pointer) <= 12.)
                });
                let aim = cross_floor.map_or(raw, |s| s.0.point);
                if self.snap_settings.polar {
                    if let Some(s) = node.filter(|s| s.kind == snap::Kind::Node) {
                        let now = ui.input(|i| i.time);
                        match self.dwell_polar_candidate {
                            Some((old, since))
                                if camera
                                    .screen(old, rect)
                                    .distance(camera.screen(s.point, rect))
                                    < 1. =>
                            {
                                if now - since >= 1. {
                                    self.dwell_polar_anchor = Some(s.point);
                                }
                            }
                            _ => self.dwell_polar_candidate = Some((s.point, now)),
                        }
                        ui.ctx().request_repaint_after(Duration::from_millis(30));
                    } else {
                        self.dwell_polar_candidate = None;
                    }
                }
                if ui.input(|i| i.modifiers.shift) && self.temporary_polar_anchor.is_none() {
                    self.temporary_polar_anchor = Some(node.map_or(raw, |s| s.point));
                }
                let first = self.draft.first().or(self.curve_controls.first()).copied();
                let can_close = matches!(self.tool, Tool::Line | Tool::Spline | Tool::Arc)
                    && (self.can_finish_sketch()
                        || (self.tool == Tool::Arc
                            && self.curve_controls.len() == 2
                            && self.draft.len() > 1));
                let closes = self.snap_settings.geometry_nodes
                    && can_close
                    && first.is_some_and(|p| {
                        camera.screen(p, rect).distance(pointer) <= 12.
                            || cross_floor.is_some_and(|s| {
                                camera
                                    .screen(p, rect)
                                    .distance(camera.screen(s.0.point, rect))
                                    <= 12.
                            })
                    });
                let found = if closes {
                    first.map(|point| snap::Snap {
                        point,
                        kind: snap::Kind::Node,
                    })
                } else {
                    node.or_else(|| self.polar_snap(raw))
                };
                let mut target = found.map_or(raw, |s| s.point);
                if let Some(p) = self.dynamic_endpoint(aim) {
                    target = p;
                }
                // Dynamic entry invokes the same sketch_click/analytic curve path as 2D.
                self.dynamic_aim = Some(aim);
                self.dynamic_input(ui.ctx());
                self.dynamic_aim = None;
                let painter = ui.painter().with_clip_rect(rect);
                if self.snap_settings.polar {
                    for a in self.polar_anchors() {
                        builder.marker(project([a[0], a[1], z]), true);
                        let length = rect.size().length() as f64 / camera.zoom * 2.;
                        for i in 0..(360. / self.snap_settings.angle_step).round() as usize {
                            let angle = (i as f64 * self.snap_settings.angle_step).to_radians();
                            let b = [a[0] + length * angle.cos(), a[1] + length * angle.sin()];
                            if let Some([start, end]) =
                                clip_line(camera.screen64(a, rect), camera.screen64(b, rect), rect)
                            {
                                let len = start.distance(end);
                                let mut x = 0.;
                                while x < len {
                                    let at = |d: f32| {
                                        let p =
                                            camera.world(start + (end - start) * (d / len), rect);
                                        project([p[0], p[1], z])
                                    };
                                    builder.line(
                                        [at(x), at((x + 6.).min(len))],
                                        0.7,
                                        Color32::from_rgba_unmultiplied(170, 185, 210, 90),
                                    );
                                    x += 11.;
                                }
                            }
                        }
                    }
                }
                self.draw_dynamic_length(&painter, rect, Some(target));
                let points =
                    if self.mode == Some(Kind::Triangulation) && self.triangulation_grid.active {
                        self.triangulation_grid.points(target).unwrap_or_default()
                    } else {
                        self.sketch_preview(Some(target))
                    };
                let ink = self.mode.map_or(Color32::WHITE, |k| self.contour_color(k));
                for edge in points.windows(2) {
                    builder.line(
                        [edge[0], edge[1]].map(|p| project([p[0], p[1], z])),
                        2.,
                        ink,
                    );
                }
                for p in self.draft.iter().chain(&self.curve_controls) {
                    let q = project([p[0], p[1], z]);
                    builder.marker(q, true);
                }
                if let Some(s) = found {
                    let q = project([s.point[0], s.point[1], z]);
                    builder.marker(q, true);
                }
                if let Some((s, world)) =
                    cross_floor.filter(|(s, _)| found.is_some_and(|f| f.point == s.point))
                {
                    let source = project(world);
                    let target = project([s.point[0], s.point[1], z]);
                    builder.marker(source, true);
                    let length = source.0.distance(target.0);
                    let mut at = 0.;
                    while at < length {
                        let p = |d: f32| {
                            let t = (d / length) as f64;
                            project([world[0], world[1], world[2] + (z - world[2]) * t])
                        };
                        builder.line(
                            [p(at), p((at + 5.).min(length))],
                            0.8,
                            Color32::from_rgb(70, 235, 225),
                        );
                        at += 9.;
                    }
                }
                ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
                if response.clicked_by(egui::PointerButton::Primary) && self.reuse_mode.is_none() {
                    self.sketch_click(target, closes);
                }
                if response.double_clicked_by(egui::PointerButton::Primary)
                    && self.mode.is_some()
                    && matches!(self.tool, Tool::Line | Tool::Spline)
                    && self.can_finish_sketch()
                {
                    self.finish();
                }
                self.reuse_canvas(ui, response, rect, hovered);
            }
        }
        spatial_depth::paint_layer(
            ui.painter(),
            rect,
            std::sync::Arc::new(builder.vertices),
            spatial_depth::Layer::Construction,
        );
        self.camera = saved;
        if drawing {
            response.context_menu(|ui| self.scene_context_menu(ui));
        }
        drawing
    }
}
