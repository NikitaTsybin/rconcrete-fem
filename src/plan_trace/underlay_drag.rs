use super::*;
use crate::plan_trace::underlay::{map_label, Transform};
#[derive(Clone)]
pub(super) struct Change {
    pub id: u64,
    transform: Transform,
    pub(super) history: usize,
    deleted: Vec<(usize, crate::plan_trace::underlay::Underlay, Option<u64>)>,
    grouped: bool,
}
pub(super) struct Drag {
    before: Change,
    anchor: Point,
    corner: Option<usize>,
    bounds: [Point; 4],
}
impl App {
    pub(super) fn underlay_corners(&self, id: u64) -> Option<[Point; 4]> {
        let u = self
            .underlays
            .iter()
            .find(|u| u.id == id && u.visible && self.show_dxf)?;
        u.corners()
    }
    pub(super) fn select_underlay(&mut self, id: u64) {
        if self.underlay_corners(id).is_none()
            || !self
                .underlays
                .iter()
                .any(|u| u.id == id && self.underlay_selectable(u))
        {
            return;
        }
        self.cancel();
        self.select_scene([], [id], egui::Modifiers::default());
    }
    pub(super) fn hit_underlay(&self, p: Point) -> Option<u64> {
        if !self.show_dxf {
            return None;
        }
        self.underlays
            .iter()
            .rev()
            .find(|u| self.underlay_selectable(u) && u.contains(p))
            .map(|u| u.id)
    }
    pub(super) fn set_underlay_transform(&mut self, id: u64, transform: Transform) {
        let Some(u) = self.underlays.iter_mut().find(|u| u.id == id) else {
            return;
        };
        let previous = u.transform();
        if previous == transform {
            return;
        }
        u.set_transform(transform);
        // Keep the merged DXF display proxy current without rebuilding its snap
        // index per mouse event. The immutable source is resampled once on release.
        if let Some(d) = &mut self.drawing {
            let prefix = format!("#{id} · ");
            let point = |p| transform.point(previous.inverse(p));
            let vector = |p| transform.vector(previous.inverse_vector(p));
            for path in d.paths.iter_mut().filter(|p| p.layer.starts_with(&prefix)) {
                for p in &mut path.points {
                    *p = point(*p);
                }
            }
            for (triangles, _, layer) in &mut d.fills {
                if layer.starts_with(&prefix) {
                    for p in triangles.iter_mut().flatten() {
                        *p = point(*p);
                    }
                }
            }
            for (p, layer) in d.nodes.iter_mut().zip(&d.node_layers) {
                if layer.starts_with(&prefix) {
                    *p = point(*p);
                }
            }
            for label in d.labels.iter_mut().filter(|l| l.layer.starts_with(&prefix)) {
                map_label(label, point, vector);
            }
        }
    }
    pub(super) fn cancel_underlay_drag(&mut self) {
        if let Some(d) = self.underlay_drag.take() {
            self.set_underlay_transform(d.before.id, d.before.transform);
            self.sync_underlays();
        }
    }
    pub(super) fn remove_underlays(
        &mut self,
        ids: &std::collections::BTreeSet<u64>,
        grouped: bool,
    ) {
        if ids.is_empty() {
            return;
        }
        if self.input_locked || self.underlays_locked {
            self.status = "Подложки заблокированы".into();
            return;
        }
        let mut deleted = vec![];
        self.underlays = std::mem::take(&mut self.underlays)
            .into_iter()
            .enumerate()
            .filter_map(|(index, u)| {
                if ids.contains(&u.id) {
                    let floor = self.building.underlay_floors.get(&u.id).copied();
                    deleted.push((index, u, floor));
                    None
                } else {
                    Some(u)
                }
            })
            .collect();
        if deleted.is_empty() {
            return;
        }
        let id = deleted[0].1.id;
        self.underlay_history.push(Change {
            id,
            transform: Transform::default(),
            history: self.history.len(),
            deleted,
            grouped,
        });
        self.underlay_selection.retain(|id| !ids.contains(id));
        if self.selected_underlay.is_some_and(|id| ids.contains(&id)) {
            self.selected_underlay = self.underlay_selection.last().copied();
        }
        for id in ids {
            self.pdf_textures.remove(id);
        }
        self.sync_underlays();
        self.status = "Выделенные подложки удалены. Ctrl+Z — отменить".into();
    }
    pub(super) fn undo_underlay(&mut self) -> bool {
        if self
            .underlay_history
            .last()
            .is_none_or(|c| c.history < self.history.len())
        {
            return false;
        }
        if self.input_locked || self.underlays_locked {
            self.status = "Снимите блокировку подложек для отмены".into();
            return true;
        }
        if self.underlay_history.last().is_some_and(|c| c.grouped) && !self.undo_contour_edit() {
            return true;
        }
        let previous = self.underlay_history.pop().unwrap();
        if !previous.deleted.is_empty() {
            let mut ids = vec![];
            for (index, u, floor) in previous.deleted {
                if let Some(floor) = floor {
                    self.building.underlay_floors.insert(u.id, floor);
                }
                ids.push(u.id);
                self.underlays.insert(index.min(self.underlays.len()), u);
            }
            self.sync_underlays();
            self.select_scene([], ids, egui::Modifiers::NONE);
            self.status = "Удаление подложек отменено".into();
            return true;
        }
        if self.underlays.iter().any(|u| u.id == previous.id) {
            self.set_underlay_transform(previous.id, previous.transform);
            self.sync_underlays();
            self.selected_underlay = Some(previous.id);
        }
        true
    }
    pub(super) fn handle_underlay_drag(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> bool {
        if self.underlays_locked || self.input_locked {
            self.cancel_underlay_drag();
            return false;
        }
        if self.scaling_op.is_some()
            || self.mode.is_some()
            || self.measurements.active
            || self.clipboard_op.is_some()
        {
            return false;
        }
        if let Some(id) = self.selected_underlay {
            if let Some(bounds) = self.underlay_corners(id) {
                if let Some(p) = hovered {
                    let corner = bounds
                        .iter()
                        .position(|&q| self.camera.screen(q, rect).distance(p) <= 10.);
                    let world = self.camera.world(p, rect);
                    let u = self.underlays.iter().find(|u| u.id == id).unwrap();
                    let inside = u.contains(world);
                    if corner.is_some() || inside {
                        ui.ctx().set_cursor_icon(if corner.is_some() {
                            egui::CursorIcon::ResizeNwSe
                        } else {
                            egui::CursorIcon::Grab
                        });
                        if ui.input(|i| i.pointer.primary_pressed()) {
                            self.underlay_drag = Some(Drag {
                                before: Change {
                                    id,
                                    transform: u.transform(),
                                    history: self.history.len(),
                                    deleted: vec![],
                                    grouped: false,
                                },
                                anchor: world,
                                corner,
                                bounds,
                            });
                        }
                    }
                }
            } else {
                self.selected_underlay = None;
            }
        }
        let moving = self.underlay_drag.is_some()
            && (response.dragged_by(egui::PointerButton::Primary)
                || response.drag_stopped_by(egui::PointerButton::Primary));
        if moving {
            if let (Some(d), Some(p)) =
                (&self.underlay_drag, ui.input(|i| i.pointer.interact_pos()))
            {
                let target = self.camera.world(p, rect);
                let mut transform = d.before.transform;
                if let Some(corner) = d.corner {
                    let opposite = d.bounds[(corner + 2) % 4];
                    let v = [
                        d.bounds[corner][0] - opposite[0],
                        d.bounds[corner][1] - opposite[1],
                    ];
                    let length = v[0] * v[0] + v[1] * v[1];
                    let ratio = if length > 1e-24 {
                        ((target[0] - opposite[0]) * v[0] + (target[1] - opposite[1]) * v[1])
                            / length
                    } else {
                        1.
                    };
                    let scale_x = (transform.scale[0] * ratio.max(1e-6)).clamp(1e-9, 1e9);
                    let ratio = scale_x / transform.scale[0];
                    transform.origin = std::array::from_fn(|i| {
                        opposite[i] + (transform.origin[i] - opposite[i]) * ratio
                    });
                    transform.scale = transform.scale.map(|s| s * ratio);
                } else {
                    transform.origin =
                        std::array::from_fn(|i| transform.origin[i] + target[i] - d.anchor[i]);
                }
                if transform.validate().is_ok() {
                    let id = d.before.id;
                    self.set_underlay_transform(id, transform);
                }
            }
        }
        if !ui.input(|i| i.pointer.primary_down()) {
            if let Some(d) = self.underlay_drag.take() {
                if self
                    .underlays
                    .iter()
                    .any(|u| u.id == d.before.id && u.transform() != d.before.transform)
                {
                    self.underlay_history.push(d.before);
                    self.sync_underlays();
                }
            }
        }
        moving || self.underlay_drag.is_some()
    }
    pub(super) fn paint_underlay_selection(&self, painter: &egui::Painter, rect: Rect) {
        if self.mode.is_none()
            && self.drag.is_none()
            && self.clipboard_op.is_none()
            && self.scaling_op.is_none()
        {
            if let Some(pointer) = painter
                .ctx()
                .pointer_hover_pos()
                .filter(|p| rect.contains(*p) && !self.toolbar_rect.is_some_and(|r| r.contains(*p)))
            {
                if self.hover_pick.current().is_none() {
                    if let Some(id) = self.hit_underlay(self.camera.world(pointer, rect)) {
                        if let Some(corners) = self.underlay_corners(id) {
                            draw_path(
                                painter,
                                &corners,
                                true,
                                self.camera,
                                rect,
                                Stroke::new(1.5_f32, Color32::LIGHT_BLUE),
                            );
                        }
                    }
                }
            }
        }
        let ids = self
            .underlay_selection
            .iter()
            .copied()
            .chain(self.selected_underlay)
            .collect::<std::collections::BTreeSet<_>>();
        for id in ids {
            let Some(bounds) = self.underlay_corners(id) else {
                continue;
            };
            for i in 0..4 {
                if let Some(line) = clip_line(
                    self.camera.screen64(bounds[i], rect),
                    self.camera.screen64(bounds[(i + 1) % 4], rect),
                    rect,
                ) {
                    painter.extend(egui::Shape::dashed_line(
                        &line,
                        Stroke::new(1.5_f32, Color32::LIGHT_BLUE),
                        6.,
                        4.,
                    ));
                }
                let p = self.camera.screen(bounds[i], rect);
                if rect.contains(p) && !self.underlays_locked {
                    painter.rect_filled(
                        Rect::from_center_size(p, Vec2::splat(9.)),
                        0.,
                        Color32::LIGHT_BLUE,
                    );
                }
            }
        }
    }
}

#[cfg(test)]
mod delete_regression {
    use super::*;
    #[test]
    fn delete_pdf_undo_keeps_data_order_and_floor() {
        let mut app = App::default();
        app.underlays.push(crate::plan_trace::underlay::Underlay {
            id: 17,
            name: "test.pdf".into(),
            visible: true,
            transparency: 0.25,
            origin: [10., 20.],
            scale: 2.,
            scale_y: None,
            angle_rad: 0.,
            data: crate::plan_trace::underlay::Data::Pdf {
                width: 1,
                height: 1,
                rgba: vec![10, 20, 30, 255],
                size_mm: [210., 297.],
                page: 1,
            },
        });
        app.building.underlay_floors.insert(17, 1);
        app.select_scene([], [17], egui::Modifiers::NONE);
        app.remove();
        assert!(app.underlays.is_empty());
        app.undo();
        assert_eq!(app.underlays.len(), 1);
        assert_eq!(app.underlays[0].id, 17);
        assert_eq!(app.underlays[0].origin, [10., 20.]);
        assert_eq!(app.building.underlay_floors[&17], 1);
        assert!(app.selected_underlay == Some(17));
    }

    #[test]
    fn hit_rotated_pdf_uses_its_actual_corners() {
        use crate::plan_trace::underlay::{Data, Underlay};
        let mut app = App::default();
        let underlay = Underlay {
            id: 17,
            name: "rotated.pdf".into(),
            visible: true,
            transparency: 0.,
            origin: [300., 400.],
            scale: 2.,
            scale_y: Some(3.),
            angle_rad: std::f64::consts::FRAC_PI_4,
            data: Data::Pdf {
                width: 1,
                height: 1,
                rgba: vec![255; 4],
                size_mm: [210., 297.],
                page: 1,
            },
        };
        let center = underlay.point([105., 148.5]);
        app.underlays.push(underlay);
        assert_eq!(app.hit_underlay(center), Some(17));
        assert_eq!(app.hit_underlay([300., 800.]), Some(17));
        assert_eq!(app.hit_underlay([-200., 420.]), None);
    }
}
