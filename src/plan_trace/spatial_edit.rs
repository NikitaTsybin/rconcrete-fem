//! Edit linked objects on the storey plane and free beams in world coordinates.
use super::*;
pub(super) fn plane_camera(project: &impl Fn([f64; 3]) -> (Pos2, f64), z: f64) -> Option<Camera> {
    let o = project([0., 0., z]).0;
    let x = (project([1000., 0., z]).0 - o) / 1000.;
    let y = (project([0., 1000., z]).0 - o) / 1000.;
    let d = x.x * y.y - x.y * y.x;
    if d.abs() < 1e-20 || d.abs() < x.length() * y.length() * 1e-4 {
        return None;
    }
    Some(Camera {
        center: [0., 0.],
        zoom: x.length().min(y.length()) as f64,
        angle: 0.,
        plane: Some([
            [o.x as f64, o.y as f64],
            [x.x as f64, x.y as f64],
            [y.x as f64, y.y as f64],
        ]),
    })
}

fn on_view_plane(
    project: &impl Fn([f64; 3]) -> (Pos2, f64),
    anchor: [f64; 3],
    pointer: Pos2,
) -> Option<[f64; 3]> {
    let origin = project(anchor).0;
    let axes: [egui::Vec2; 3] = std::array::from_fn(|i| {
        let mut p = anchor;
        p[i] += 1000.;
        (project(p).0 - origin) / 1000.
    });
    let (xx, xy, yy) = axes.iter().fold((0_f64, 0_f64, 0_f64), |(xx, xy, yy), a| {
        let (x, y) = (f64::from(a.x), f64::from(a.y));
        (xx + x * x, xy + x * y, yy + y * y)
    });
    let det = xx * yy - xy * xy;
    if det <= 0. || !det.is_finite() {
        return None;
    }
    let delta = pointer - origin;
    let u = (yy * f64::from(delta.x) - xy * f64::from(delta.y)) / det;
    let v = (xx * f64::from(delta.y) - xy * f64::from(delta.x)) / det;
    Some(std::array::from_fn(|i| {
        anchor[i] + u * f64::from(axes[i].x) + v * f64::from(axes[i].y)
    }))
}

impl App {
    fn apply_spatial_drag(&mut self, c: Contour, moved: Result<(), String>) {
        match moved.and_then(|_| model::validate_candidate(&self.contours, &c)) {
            Ok(()) => {
                if self.contours.iter().find(|x| x.id == c.id) != Some(&c) {
                    if c.kind.affects_calculation() {
                        self.clear_mesh();
                    }
                    if let Some(old) = self.contours.iter_mut().find(|x| x.id == c.id) {
                        *old = c;
                    }
                    self.slab_cache = None;
                    self.invalidate_scene_buffers();
                    self.status.clear();
                }
            }
            Err(error) => self.status = error,
        }
    }

    fn activate_edit_floor(&mut self, id: u64) {
        if self.building.enabled {
            let floor = self.building.owner(id);
            if self.building.active != floor {
                self.building.active = floor;
                self.slab_cache = None;
                self.rebuild_index();
                if let Some(mesh) = &self.fe_mesh {
                    self.fe_snap_index = Index::mesh_floor(mesh, Some(floor));
                }
            }
        }
    }
    pub(super) fn spatial_edit(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
    ) -> bool {
        self.spatial.edit_hit = false;
        let movement_locked = if self.selected.is_some() || !self.selection_ids.is_empty() {
            self.movement_locked_contours()
        } else {
            Default::default()
        };
        if self.drag.is_none() && self.arc_placement.is_none() {
            if response.dragged_by(egui::PointerButton::Secondary)
                || response.dragged_by(egui::PointerButton::Middle)
                || (response.hovered() && ui.input(|i| i.smooth_scroll_delta.y != 0.))
            {
                return false;
            }
            if !self.contours.iter().any(|c| {
                self.is_selected(c.id)
                    && self.selectable(c)
                    && self.contour_editable(c.id)
                    && !movement_locked.contains(&c.id)
            }) {
                if response.clicked_by(egui::PointerButton::Secondary) {
                    self.menu_target = None;
                    self.arc_menu_target = None;
                }
                return false;
            }
        }
        if self.clipboard_op.is_some() || self.scaling_op.is_some() || self.mode.is_some() {
            return false;
        }
        let pointer = response
            .hover_pos()
            .filter(|p| !self.toolbar_rect.is_some_and(|r| r.contains(*p)));
        let occlusion = self
            .spatial
            .cache
            .as_ref()
            .map(|s| spatial_results::Occlusion::new_visible(rect, &s.4, projected, self));
        let mut builder = spatial_depth::Builder::new(rect, projected);
        // (distance, node-before-midpoint, depth, contour, handle/edge, XY, planeZ)
        let mut hit = None::<(f32, bool, f64, u64, usize, Point, f64)>;
        for c in self.contours.iter().filter(|c| {
            self.is_selected(c.id)
                && self.selectable(c)
                && self.contour_editable(c.id)
                && !movement_locked.contains(&c.id)
        }) {
            let handles = if c.is_free_beam() {
                let path = self.source_beam_path(c);
                path.iter()
                    .copied()
                    .enumerate()
                    .map(|(i, p)| (true, i, p))
                    .chain(path.windows(2).enumerate().map(|(i, edge)| {
                        (
                            false,
                            i,
                            std::array::from_fn(|k| (edge[0][k] + edge[1][k]) * 0.5),
                        )
                    }))
                    .collect::<Vec<_>>()
            } else {
                let vertices = c.handles();
                let edges = c.straight_edges();
                self.source_heights(c)
                    .into_iter()
                    .flat_map(|z| {
                        vertices
                            .iter()
                            .enumerate()
                            .map(move |(i, &[x, y])| (true, i, [x, y, z]))
                            .chain(edges.iter().map(move |&(i, a, b)| {
                                (false, i, [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, z])
                            }))
                    })
                    .collect()
            };
            for (vertex, index, [x, y, z]) in handles {
                let p = [x, y];
                if !self.sections.contains([x, y, z]) {
                    continue;
                }
                let q = project([p[0], p[1], z]);
                if !rect.contains(q.0) || occlusion.as_ref().is_some_and(|o| !o.visible(q.0, q.1)) {
                    continue;
                }
                let corners = if vertex {
                    [[-4., -4.], [4., -4.], [4., 4.], [-4., 4.]]
                } else {
                    [[0., -4.], [4., 0.], [0., 4.], [-4., 0.]]
                };
                for i in 0..4 {
                    let a = corners[i];
                    let b = corners[(i + 1) % 4];
                    builder.line(
                        [
                            (q.0 + egui::vec2(a[0], a[1]), q.1),
                            (q.0 + egui::vec2(b[0], b[1]), q.1),
                        ],
                        1.5,
                        Color32::YELLOW,
                    );
                }
                if let Some(pointer) = pointer {
                    let distance = q.0.distance(pointer);
                    if distance <= 10.
                        && hit.is_none_or(|old| {
                            (vertex && !old.1)
                                || (vertex == old.1
                                    && (distance < old.0 - 0.5
                                        || ((distance - old.0).abs() <= 0.5 && q.1 > old.2)))
                        })
                    {
                        hit = Some((distance, vertex, q.1, c.id, index, p, z));
                    }
                }
            }
        }
        self.spatial.edit_hit = hit.is_some();
        if response.clicked_by(egui::PointerButton::Secondary) {
            self.menu_target = hit.map(|(_, vertex, _, id, index, _, _)| (id, index, vertex));
            self.arc_menu_target = hit.filter(|h| !h.1).map(|h| (h.3, h.4));
            if let Some(h) = hit {
                self.spatial.edit_plane_z = h.6;
                self.activate_edit_floor(h.3);
            }
        }
        if self.drag.is_none()
            && self.arc_placement.is_none()
            && ui.input(|i| i.pointer.primary_pressed() && !i.modifiers.shift && !i.modifiers.ctrl)
        {
            if let Some((_, vertex, _, id, index, anchor, z)) = hit {
                if let Some(c) = self.contours.iter().find(|c| c.id == id).cloned() {
                    self.spatial.lock_edit_camera();
                    self.activate_edit_floor(id);
                    self.spatial.edit_plane_z = z;
                    self.drag = Some(VertexDrag {
                        id,
                        vertex: vertex.then_some(index),
                        edge: (!vertex).then_some(index),
                        anchor,
                        original: c,
                        before: self.contours.clone(),
                    });
                    self.spatial.frame_start = None;
                    self.dwell_polar_anchor = None;
                    self.dwell_polar_candidate = None;
                    self.temporary_polar_anchor = None;
                    if let Some(id) = ui.memory(|m| m.focused()) {
                        ui.memory_mut(|m| m.surrender_focus(id));
                    }
                }
            } else if let (Some(id), Some(pointer)) = (self.selected, pointer) {
                if let Some(c) = self
                    .contours
                    .iter()
                    .find(|c| {
                        c.id == id
                            && !movement_locked.contains(&id)
                            && self.contour_editable(id)
                            && self.selectable(c)
                    })
                    .cloned()
                {
                    let anchor = if c.is_free_beam() {
                        self.source_beam_path(&c)
                            .windows(2)
                            .filter_map(|edge| {
                                let a = project(edge[0]);
                                let b = project(edge[1]);
                                let d = b.0 - a.0;
                                let [lo, hi] =
                                    self.sections.segment_interval([edge[0], edge[1]])?;
                                let t = if d.length_sq() > 1e-10 {
                                    f64::from((pointer - a.0).dot(d) / d.length_sq()).clamp(lo, hi)
                                } else {
                                    if b.1 > a.1 {
                                        hi
                                    } else {
                                        lo
                                    }
                                };
                                let p = std::array::from_fn(|k| {
                                    edge[0][k] + (edge[1][k] - edge[0][k]) * t
                                });
                                let (screen, depth) = project(p);
                                let distance = pointer.distance(screen);
                                (distance <= 7.
                                    && occlusion.as_ref().is_none_or(|o| o.visible(screen, depth)))
                                .then_some((distance, depth, p))
                            })
                            .min_by(|a, b| a.0.total_cmp(&b.0).then_with(|| b.1.total_cmp(&a.1)))
                            .and_then(|(_, _, p)| on_view_plane(project, p, pointer))
                    } else {
                        let z = self.source_heights(&c)[0];
                        plane_camera(project, z).and_then(|camera| {
                            let p = camera.world(pointer, rect);
                            (self.hit_contour(&c, p, camera.zoom)
                                && occlusion
                                    .as_ref()
                                    .is_none_or(|o| o.visible(pointer, project([p[0], p[1], z]).1)))
                            .then_some([p[0], p[1], z])
                        })
                    };
                    if let Some([x, y, z]) = anchor {
                        self.spatial.lock_edit_camera();
                        self.activate_edit_floor(id);
                        self.spatial.edit_plane_z = z;
                        self.drag = Some(VertexDrag {
                            id,
                            vertex: None,
                            edge: None,
                            anchor: [x, y],
                            original: c,
                            before: self.contours.clone(),
                        });
                        self.spatial.frame_start = None;
                        self.dwell_polar_anchor = None;
                        self.dwell_polar_candidate = None;
                    }
                }
            }
        }
        if self.drag.is_some() && ui.input(|i| i.pointer.primary_pressed()) {
            if let Some(drag) = self.drag.as_ref().filter(|d| d.original.is_free_beam()) {
                let [x, y] = drag.anchor;
                let screen = project([x, y, self.spatial.edit_plane_z]).0;
                let offset = ui
                    .input(|i| i.pointer.press_origin())
                    .map_or(egui::Vec2::ZERO, |press| press - screen);
                ui.data_mut(|data| {
                    data.insert_temp(egui::Id::new("free_beam_drag_offset"), offset)
                });
            }
            self.dwell_polar_anchor = None;
            self.dwell_polar_candidate = None;
            self.temporary_polar_anchor = None;
            if let Some(id) = ui.memory(|m| m.focused()) {
                ui.memory_mut(|m| m.surrender_focus(id));
            }
        }
        let editing = self.drag.is_some() || self.arc_placement.is_some();
        if editing {
            self.spatial.lock_edit_camera();
            let saved = self.camera;
            if let Some(drag) = self.drag.as_ref().filter(|d| d.original.is_free_beam()) {
                if response.dragged_by(egui::PointerButton::Primary)
                    || response.drag_stopped_by(egui::PointerButton::Primary)
                {
                    if let Some(pointer) = ui.input(|i| i.pointer.interact_pos()) {
                        let anchor = [drag.anchor[0], drag.anchor[1], self.spatial.edit_plane_z];
                        let target = self
                            .spatial_vertex_snap(pointer, project, projected, rect, Some(drag.id))
                            .map(|(_, p)| p)
                            .or_else(|| {
                                let offset = ui
                                    .data(|data| {
                                        data.get_temp::<egui::Vec2>(egui::Id::new(
                                            "free_beam_drag_offset",
                                        ))
                                    })
                                    .unwrap_or_default();
                                on_view_plane(project, anchor, pointer - offset)
                            });
                        if let Some([x, y, z]) = target {
                            let mut c = drag.original.clone();
                            let heights = c.placement.beam_z_mm.as_mut().unwrap();
                            if let Some(vertex) = drag.vertex {
                                c.points[vertex] = [x, y];
                                heights[vertex] = z;
                            } else {
                                let range = drag.edge.map_or(0..c.points.len(), |e| e..e + 2);
                                for i in range {
                                    c.points[i][0] += x - anchor[0];
                                    c.points[i][1] += y - anchor[1];
                                    heights[i] += z - anchor[2];
                                }
                            }
                            self.apply_spatial_drag(c, Ok(()));
                        }
                    }
                }
            } else if let Some(camera) = plane_camera(project, self.spatial.edit_plane_z) {
                self.camera = camera;
                if let Some(pointer) = ui.input(|i| i.pointer.interact_pos()) {
                    let raw = camera.world(pointer, rect);
                    let exclude = self
                        .drag
                        .as_ref()
                        .map(|d| d.id)
                        .or_else(|| self.arc_placement.as_ref().map(|a| a.original.id));
                    let found = self
                        .locked_vertex_direction(ui, raw)
                        .or_else(|| {
                            self.snap(raw, exclude)
                                .filter(|s| camera.screen(s.point, rect).distance(pointer) <= 12.)
                        })
                        .or_else(|| self.polar_snap(raw));
                    let target = found.map_or(raw, |s| s.point);
                    if self.snap_settings.polar {
                        let own = self
                            .drag
                            .as_ref()
                            .into_iter()
                            .flat_map(|d| d.original.handles())
                            .filter(|p| camera.screen(*p, rect).distance(pointer) <= 9.)
                            .next();
                        let anchor = own.or_else(|| {
                            found
                                .filter(|s| s.kind == snap::Kind::Node)
                                .map(|s| s.point)
                        });
                        if let Some(anchor) = anchor {
                            let now = ui.input(|i| i.time);
                            match self.dwell_polar_candidate {
                                Some((old, since))
                                    if camera
                                        .screen(old, rect)
                                        .distance(camera.screen(anchor, rect))
                                        < 1. =>
                                {
                                    if now - since >= 1. {
                                        self.dwell_polar_anchor = Some(anchor);
                                    }
                                }
                                _ => self.dwell_polar_candidate = Some((anchor, now)),
                            }
                            ui.ctx().request_repaint_after(Duration::from_millis(30));
                        } else {
                            self.dwell_polar_candidate = None;
                        }
                        self.draw_polar_guides(
                            ui.painter(),
                            rect,
                            raw,
                            found.is_some_and(|s| s.kind == snap::Kind::Angle),
                        );
                    }
                    if self.arc_placement.is_some() {
                        self.preview_edge_arc(target);
                        if let Some(edit) = &self.arc_placement {
                            if let Some(c) = &edit.preview {
                                for edge in c.points.windows(2) {
                                    builder.line(
                                        [edge[0], edge[1]].map(|p| {
                                            project([p[0], p[1], self.spatial.edit_plane_z])
                                        }),
                                        2.,
                                        Color32::YELLOW,
                                    );
                                }
                            }
                            if let Some(radius) = edit.radius {
                                self.load_caption(
                                    ui.painter(),
                                    pointer + egui::vec2(35., -18.),
                                    format!("R {} мм", self.display_text.number(radius)),
                                );
                            }
                        }
                        if response.clicked_by(egui::PointerButton::Primary) {
                            self.accept_edge_arc();
                        }
                    } else if response.dragged_by(egui::PointerButton::Primary)
                        || response.drag_stopped_by(egui::PointerButton::Primary)
                    {
                        if let Some(drag) = &self.drag {
                            let mut c = drag.original.clone();
                            let delta = [target[0] - drag.anchor[0], target[1] - drag.anchor[1]];
                            let moved = if let Some(edge) = drag.edge {
                                c.move_edge(edge, delta)
                            } else if let Some(vertex) = drag.vertex {
                                c.move_handle(vertex, target)
                            } else {
                                c.translate(delta)
                            };
                            self.apply_spatial_drag(c, moved);
                        }
                    }
                }
                self.camera = saved;
            } else {
                self.status = "Поверните вид к плоскости редактирования".into();
            }
            if self.drag.is_some() && !ui.input(|i| i.pointer.primary_down()) {
                ui.ctx().data_mut(|data| {
                    data.remove::<(u64, Point, f64)>(egui::Id::new("locked_vertex_direction"));
                    data.remove::<egui::Vec2>(egui::Id::new("free_beam_drag_offset"));
                });
                let drag = self.drag.take().unwrap();
                if self.contours != drag.before {
                    self.history.push(drag.before);
                }
            }
        }
        spatial_depth::paint_layer(
            ui.painter(),
            rect,
            std::sync::Arc::new(builder.vertices),
            spatial_depth::Layer::Editing,
        );
        editing
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn spatial_vertex_drag_uses_plane_coordinates_and_one_undo() {
        for kind in [Kind::Slab, Kind::Support, Kind::Beam] {
            check_spatial_vertex_undo(kind);
        }
    }

    fn check_spatial_vertex_undo(kind: Kind) {
        let mut app = App::default();
        app.building.enabled = true;
        app.spatial.enabled = true;
        let fixed_bounds = ([500., 500., 0.], 1400.);
        app.spatial.focus_warning(fixed_bounds.0, fixed_bounds.1);
        app.begin(kind);
        app.choose_tool(if kind == Kind::Slab {
            Tool::Rectangle
        } else {
            Tool::Line
        });
        app.sketch_click([0., 0.], false);
        app.sketch_click([1000., 1000.], false);
        if matches!(kind, Kind::Support | Kind::Beam) {
            app.finish();
        }
        if kind == Kind::Beam {
            app.contours[0].placement.beam_z_mm = Some(vec![500., 1000.]);
        }
        app.sync_building_objects();
        let original = app.contours.clone();
        app.history.clear();
        app.snap_settings.polar = false;
        app.snap_settings.grid_nodes = false;
        app.snap_settings.geometry_nodes = false;
        app.snap_settings.nearest = false;
        let ctx = egui::Context::default();
        let project = |p: [f64; 3]| {
            if kind == Kind::Beam {
                return (
                    egui::pos2(200. + p[0] as f32 * 0.1, 450. - p[2] as f32 * 0.1),
                    p[1],
                );
            }
            (
                egui::pos2(200. + p[0] as f32 * 0.1, 400. - p[1] as f32 * 0.1),
                p[2],
            )
        };
        let mut frame = |app: &mut App, events: Vec<egui::Event>| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let (rect, response) = ui.allocate_exact_size(
                            ui.available_size(),
                            egui::Sense::click_and_drag(),
                        );
                        app.spatial_edit(
                            ui,
                            &response,
                            rect,
                            &project,
                            &[(egui::pos2(200., 400.), 0.), (egui::pos2(300., 300.), 1.)],
                        );
                    });
                },
            );
        };
        frame(&mut app, vec![]);
        let start = egui::pos2(200., 400.);
        let end = egui::pos2(220., 370.);
        frame(
            &mut app,
            vec![
                egui::Event::PointerMoved(start),
                egui::Event::PointerButton {
                    pos: start,
                    button: egui::PointerButton::Primary,
                    pressed: true,
                    modifiers: egui::Modifiers::NONE,
                },
            ],
        );
        assert!(app.drag.is_some());
        assert_eq!(app.spatial.edit_camera_bounds, Some(fixed_bounds));
        frame(&mut app, vec![egui::Event::PointerMoved(end)]);
        assert_eq!(app.spatial.edit_camera_bounds, Some(fixed_bounds));
        frame(
            &mut app,
            vec![egui::Event::PointerButton {
                pos: end,
                button: egui::PointerButton::Primary,
                pressed: false,
                modifiers: egui::Modifiers::NONE,
            }],
        );
        if kind == Kind::Beam {
            let [x, y, z] = app.source_beam_path(&app.contours[0])[0];
            assert!((x - 200.).abs() < 0.01 && y == 0. && (z - 800.).abs() < 0.01);
        } else {
            let p = app.contours[0].handles()[0];
            assert!(
                (p[0] - 200.).abs() < 0.01 && (p[1] - 300.).abs() < 0.01,
                "{p:?}"
            );
        }
        assert_eq!(app.history.len(), 1);
        app.press_undo_after_control_release();
        assert_eq!(app.contours, original);

        let id = app.contours[0].id;
        app.contours[0].placement.movement_locked = true;
        app.select_only(Some(id));
        frame(&mut app, vec![egui::Event::PointerMoved(start)]);
        frame(
            &mut app,
            vec![egui::Event::PointerButton {
                pos: start,
                button: egui::PointerButton::Primary,
                pressed: true,
                modifiers: egui::Modifiers::NONE,
            }],
        );
        assert!(app.drag.is_none());
    }
}
