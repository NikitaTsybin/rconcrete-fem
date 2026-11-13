use super::*;
pub(super) struct Edit {
    id: u64,
    text: String,
    at: Pos2,
    factor: f64,
    units: crate::plan_trace::loads::ForceUnits,
    focus: bool,
    dimension: u8,
}
impl App {
    pub(super) fn load_value_canvas(
        &mut self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> bool {
        if self.load_value_edit.is_some() {
            return true;
        }
        if self.mode.is_some()
            || self.drag.is_some()
            || self.measurements.active
            || self.clipboard_op.is_some()
            || self.scaling_op.is_some()
            || self.arc_placement.is_some()
            || self.underlay_drag.is_some()
            || !self.load_values_visible()
            || (self.show_results
                && matches!(self.result_field, 7..=9)
                && self.selected_result().is_some())
        {
            return false;
        }
        let Some(pointer) = hovered else {
            return false;
        };
        let hit = self
            .contours
            .iter()
            .rev()
            .filter(|c| c.kind.is_load() && c.kind != Kind::LoadHole && self.contour_visible(c))
            .find_map(|c| {
                let value = self.displayed_load_value(c)?;
                let text = format!(
                    "{} {}",
                    self.display_text.number(value),
                    self.load_units.intensity_label(c.load_dimension())
                );
                let size = painter
                    .layout_no_wrap(
                        text,
                        egui::FontId::proportional(self.display_text.size),
                        self.display_text.ink(),
                    )
                    .size()
                    + egui::vec2(6., 3.);
                let hit = c.load_label_positions().into_iter().any(|p| {
                    let mut center = self.camera.screen(p, rect);
                    if c.kind == Kind::LoadPoint {
                        center += egui::vec2(45., -15.);
                    }
                    Rect::from_center_size(center, size).contains(pointer)
                });
                hit.then_some((c.id, value, c.load_dimension(), c.load.as_ref()?.case_id))
            });
        let Some((id, _, _, _)) = hit else {
            return false;
        };
        ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
        if ui.input(|i| i.pointer.primary_pressed()) {
            self.begin_load_value_edit(id, pointer);
        }
        true
    }
    fn begin_load_value_edit(&mut self, id: u64, pointer: Pos2) {
        self.selection_start = None;
        self.spatial.frame_start = None;
        if !self.contour_editable(id) {
            self.status =
                "Для изменения нагрузки снимите блокировку исходных данных и нагрузок".into();
            return;
        }
        let Some(c) = self.contours.iter().find(|c| c.id == id) else {
            return;
        };
        let Some(value) = self.displayed_load_value(c) else {
            return;
        };
        let Some(case) = c.load.as_ref().map(|l| l.case_id) else {
            return;
        };
        let dimension = c.load_dimension();
        let Some(factor) = self
            .displayed_load_factor(case)
            .filter(|v| v.is_finite() && *v != 0.)
        else {
            return;
        };
        self.select_only(Some(id));
        self.load_value_edit = Some(Edit {
            id,
            text: value.to_string(),
            at: pointer + egui::vec2(12., 12.),
            factor,
            units: self.load_units,
            focus: true,
            dimension,
        });
    }
    pub(super) fn load_value_spatial(&mut self, ui: &egui::Ui, hit: Option<u64>) -> bool {
        if self.load_value_edit.is_some() {
            return true;
        }
        if self.mode.is_some()
            || self.drag.is_some()
            || self.measurements.active
            || self.clipboard_op.is_some()
            || self.scaling_op.is_some()
            || self.arc_placement.is_some()
            || self.underlay_drag.is_some()
        {
            return false;
        }
        if !ui
            .ctx()
            .pointer_hover_pos()
            .is_some_and(|p| ui.ctx().layer_id_at(p) == Some(ui.layer_id()))
        {
            return false;
        }
        let Some(id) = hit else {
            return false;
        };
        ui.ctx().set_cursor_icon(egui::CursorIcon::Text);
        if ui.input(|i| i.pointer.primary_pressed()) {
            if let Some(pointer) = ui.input(|i| i.pointer.hover_pos()) {
                self.begin_load_value_edit(id, pointer);
            }
        }
        true
    }
    pub(super) fn load_value_dialog(&mut self, ctx: &egui::Context) {
        let Some(edit) = &mut self.load_value_edit else {
            return;
        };
        let mut open = true;
        let mut accept = false;
        let mut cancel = false;
        egui::Window::new("Значение нагрузки")
            .id(egui::Id::new("load_value_editor"))
            .default_pos(edit.at)
            .collapsible(false)
            .resizable(false)
            .open(&mut open)
            .show(ctx, |ui| {
                ui.label(format!(
                    "Интенсивность на текущем виде, {}",
                    edit.units.intensity_label(edit.dimension)
                ));
                let response = ui.text_edit_singleline(&mut edit.text);
                if edit.focus {
                    response.request_focus();
                    edit.focus = false;
                }
                ui.horizontal(|ui| {
                    accept = ui.button("Применить").clicked();
                    cancel = ui.button("Отмена").clicked();
                });
                accept |= ui.input(|i| i.key_pressed(egui::Key::Enter));
                cancel |= ui.input(|i| i.key_pressed(egui::Key::Escape));
            });
        if cancel || !open {
            self.load_value_edit = None;
            return;
        }
        if !accept {
            return;
        }
        let edit = self.load_value_edit.as_ref().unwrap();
        let value = match edit.text.trim().replace(',', ".").parse::<f64>() {
            Ok(value) if value.is_finite() && (value / edit.factor).is_finite() => {
                value / edit.factor
            }
            _ => {
                self.status = "Введите конечное числовое значение нагрузки".into();
                return;
            }
        };
        let id = edit.id;
        let units = edit.units;
        if !self.contour_editable(id) {
            self.status = "Нагрузка заблокирована".into();
            return;
        }
        let Some(index) = self
            .contours
            .iter()
            .position(|c| c.id == id && c.load.is_some())
        else {
            self.load_value_edit = None;
            return;
        };
        let magnitude = crate::plan_trace::loads::Magnitude { value, units };
        if self.contours[index].load.as_ref().unwrap().magnitude != magnitude {
            self.history.push(self.contours.clone());
            self.contours[index].load.as_mut().unwrap().magnitude = magnitude;
            self.analysis = None;
            self.analysis_input = None;
            self.analysis_job = None;
            self.combination_cache = None;
            self.load_scale_key = None;
            self.invalidate_scene_buffers();
            self.spatial.cache_reset();
            self.status = "Значение нагрузки изменено. Выполните расчёт заново".into();
        }
        self.load_value_edit = None;
    }
}
