use super::*;
#[derive(Default)]
pub(super) struct Hover {
    ids: Vec<u64>,
    index: usize,
    pub surface: Option<(Rect, egui::LayerId)>,
}
impl Hover {
    pub(super) fn current(&self) -> Option<u64> {
        self.ids.get(self.index).copied()
    }
}
impl App {
    pub(super) fn selected_vertex_hit(
        &self,
        pointer: Pos2,
        rect: Rect,
        locked: &std::collections::BTreeSet<u64>,
    ) -> bool {
        self.selected
            .and_then(|id| self.contours.iter().find(|c| c.id == id))
            .is_some_and(|c| {
                self.selectable(c)
                    && !locked.contains(&c.id)
                    && self.contour_editable(c.id)
                    && (c
                        .handles()
                        .iter()
                        .any(|&p| self.camera.screen(p, rect).distance(pointer) <= 9.)
                        || c.straight_edges().into_iter().any(|(_, a, b)| {
                            self.camera
                                .screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], rect)
                                .distance(pointer)
                                <= 9.
                        }))
            })
    }
    pub(super) fn paint_selected_contours(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        locked: &std::collections::BTreeSet<u64>,
    ) {
        if self.mode.is_some() {
            return;
        }
        let accent = Color32::from_rgb(100, 225, 255);
        for c in self
            .contours
            .iter()
            .filter(|c| self.is_selected(c.id) && self.selectable(c))
        {
            self.paint_source_contour_outline(
                painter,
                c,
                rect,
                Stroke::new(6_f32, Color32::from_rgba_unmultiplied(15, 25, 35, 220)),
            );
            self.paint_source_contour_outline(painter, c, rect, Stroke::new(3_f32, accent));
            if !locked.contains(&c.id) && self.contour_editable(c.id) {
                for point in c.handles() {
                    let at = self.camera.screen(point, rect);
                    painter.rect_filled(
                        Rect::from_center_size(at, Vec2::splat(10.)),
                        0.,
                        Color32::from_rgb(15, 25, 35),
                    );
                    painter.rect_filled(Rect::from_center_size(at, Vec2::splat(6.)), 0., accent);
                }
            }
        }
    }
    fn hover_available(&self) -> bool {
        !self.spatial.enabled
            && self.mode.is_none()
            && self.load_value_edit.is_none()
            && self.drag.is_none()
            && self.underlay_drag.is_none()
            && self.arc_placement.is_none()
            && self.clipboard_op.is_none()
            && self.scaling_op.is_none()
            && !self.measurements.active
            && self.reuse_mode.is_none()
    }
    pub(super) fn update_contour_hover(&mut self, pointer: Option<Pos2>, rect: Rect) {
        if self.drag.is_some() {
            return;
        }
        let ids = if self.hover_available() {
            pointer
                .filter(|p| rect.contains(*p))
                .map(|p| {
                    let world = self.camera.world(p, rect);
                    let mut ids = self
                        .contours
                        .iter()
                        .rev()
                        .filter(|c| c.kind.is_load())
                        .chain(self.contours.iter().rev().filter(|c| !c.kind.is_load()))
                        .filter(|c| self.hit_contour(c, world, self.camera.zoom))
                        .map(|c| c.id)
                        .collect::<Vec<_>>();
                    ids.sort_by_key(|id| {
                        let kind = self
                            .contours
                            .iter()
                            .find(|c| c.id == *id)
                            .map(|c| draw_order::category(c.kind));
                        self.draw_order
                            .items
                            .iter()
                            .position(|v| Some(*v) == kind)
                            .unwrap_or(usize::MAX)
                    });
                    ids
                })
                .unwrap_or_default()
        } else {
            vec![]
        };
        if ids != self.hover_pick.ids {
            self.hover_pick.ids = ids;
            self.hover_pick.index = 0;
        }
    }
    pub(super) fn contour_hover_raw_input(
        &mut self,
        ctx: &egui::Context,
        input: &mut egui::RawInput,
    ) {
        if self.spatial.enabled {
            // Consume before egui begins focus traversal, not during canvas painting.
            let pointer = input
                .events
                .iter()
                .rev()
                .find_map(|e| match e {
                    egui::Event::PointerMoved(p) => Some(*p),
                    _ => None,
                })
                .or_else(|| ctx.pointer_hover_pos());
            if self.mode.is_none()
                && !ctx.output(|o| o.ime.is_some())
                && self.spatial.hover_ids.len() > 1
                && self.spatial.hover_surface.is_some_and(|(rect, layer)| {
                    pointer.is_some_and(|p| rect.contains(p) && ctx.layer_id_at(p) == Some(layer))
                })
            {
                input.events.retain(|e| {
                    if let egui::Event::Key {
                        key: egui::Key::Tab,
                        pressed,
                        modifiers,
                        ..
                    } = e
                    {
                        if !modifiers.ctrl && !modifiers.alt && !modifiers.command {
                            if *pressed {
                                self.spatial.hover_cycle += if modifiers.shift { -1 } else { 1 };
                            }
                            return false;
                        }
                    }
                    true
                });
            }
            return;
        }
        if !self.hover_available() {
            return;
        }
        let Some((rect, layer)) = self.hover_pick.surface else {
            return;
        };
        let pointer = input
            .events
            .iter()
            .rev()
            .find_map(|e| {
                if let egui::Event::PointerMoved(p) = e {
                    Some(*p)
                } else {
                    None
                }
            })
            .or_else(|| ctx.pointer_hover_pos());
        let Some(p) = pointer.filter(|p| rect.contains(*p) && ctx.layer_id_at(*p) == Some(layer))
        else {
            return;
        };
        if !input.events.iter().any(|e|matches!(e,egui::Event::Key{key:egui::Key::Tab,pressed:true,modifiers,..}if !modifiers.ctrl&&!modifiers.alt&&!modifiers.command)){return;}
        self.update_contour_hover(Some(p), rect);
        if self.hover_pick.ids.len() < 2 {
            return;
        }
        input.events.retain(|e| {
            if let egui::Event::Key {
                key: egui::Key::Tab,
                pressed,
                modifiers,
                ..
            } = e
            {
                if !modifiers.ctrl && !modifiers.alt && !modifiers.command {
                    if *pressed {
                        let n = self.hover_pick.ids.len();
                        self.hover_pick.index =
                            (self.hover_pick.index + if modifiers.shift { n - 1 } else { 1 }) % n;
                    }
                    return false;
                }
            }
            true
        });
    }
    pub(super) fn paint_contour_hover(&self, painter: &egui::Painter, rect: Rect) {
        let Some(c) = self
            .hover_pick
            .current()
            .and_then(|id| self.contours.iter().find(|c| c.id == id))
        else {
            return;
        };
        // Visibility may change after hover candidates were computed (sidebar,
        // load case or a cached candidate during a drag). Never paint hidden IDs.
        if !self.hover_available() || !self.selectable(c) {
            return;
        }
        if !(self.show_results
            && (self.selected_result().is_some() || self.punching_view.is_some()))
            && !painter.ctx().input(|i| i.pointer.any_down())
        {
            if let Some(pointer) = painter.ctx().pointer_hover_pos().filter(|p| {
                rect.contains(*p)
                    && self
                        .hover_pick
                        .surface
                        .is_some_and(|(_, layer)| painter.ctx().layer_id_at(*p) == Some(layer))
            }) {
                let _ = pointer;
                egui::show_tooltip_at_pointer(
                    painter.ctx(),
                    egui::LayerId::new(egui::Order::Tooltip, egui::Id::new("source_2d_tooltip")),
                    egui::Id::new(("source_2d_object", c.id)),
                    |ui| {
                        ui.label(format!("{} · {}", c.kind.label(), c.id));
                        if let Some(value) = self.schema_hover(c.id) {
                            ui.label(value);
                        }
                        if !c.name.is_empty() {
                            ui.label(&c.name);
                        }
                        if self.building.enabled {
                            let floor = self.building.floor(self.building.owner(c.id));
                            ui.label(format!(
                                "Этаж: {} · Z={} мм",
                                floor.name,
                                self.display_text.number(floor.elevation_mm)
                            ));
                        }
                        if let Some(entry) = self
                            .stiffnesses
                            .assignments
                            .get(&c.id)
                            .and_then(|id| self.stiffnesses.entries.get(id))
                        {
                            ui.label(format!("Жёсткость: {}", entry.name));
                        }
                        let count = self.hover_pick.ids.len();
                        if count > 1 {
                            ui.weak(format!(
                                "{} / {} · ещё {} · Tab / Shift+Tab",
                                self.hover_pick.index + 1,
                                count,
                                count - 1
                            ));
                        }
                    },
                );
            }
        }
        self.paint_source_contour_outline(
            painter,
            c,
            rect,
            Stroke::new(5_f32, Color32::from_rgba_unmultiplied(255, 255, 255, 150)),
        );
        self.paint_source_contour_outline(
            painter,
            c,
            rect,
            Stroke::new(2_f32, self.object_ink(c.id, c.kind)),
        );
    }
}
