use super::*;

impl App {
    pub(super) fn rectangle_raw_input(&mut self, ctx: &egui::Context, input: &mut egui::RawInput) {
        if self.rectangle_dynamic_anchor().is_none() {
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
        if pointer.is_none_or(|p| {
            !rect.contains(p)
                || ctx.layer_id_at(p) != Some(layer)
                || self.toolbar_rect.is_some_and(|t| t.contains(p))
        }) {
            return;
        }
        if input.modifiers.ctrl || input.modifiers.alt || input.modifiers.command {
            return;
        }
        let numeric = input.events.iter().any(|e| {
            matches!(e,egui::Event::Text(t)if t.chars().any(|c|c.is_ascii_digit()||c=='.'||c==','))
                || matches!(
                    e,
                    egui::Event::Key {
                        key: egui::Key::Tab | egui::Key::Enter | egui::Key::Backspace,
                        pressed: true,
                        ..
                    }
                )
        });
        if !numeric {
            return;
        }
        // Release stale sidebar focus before egui routes the text or Tab.
        if let Some(id) = ctx.memory(|m| m.focused()) {
            ctx.memory_mut(|m| m.surrender_focus(id));
        }
        input.events.retain(|e| {
            if let egui::Event::Key {
                key: egui::Key::Tab,
                pressed,
                ..
            } = e
            {
                if *pressed {
                    self.dynamic_rectangle_field = 1 - self.dynamic_rectangle_field;
                    self.dynamic_rectangle_replace = true;
                }
                false
            } else {
                true
            }
        });
    }
    pub(super) fn reset_dynamic(&mut self) {
        self.dynamic_aim = None;
        self.dynamic_length.clear();
        self.dynamic_rectangle = std::array::from_fn(|_| String::new());
        self.dynamic_rectangle_field = 0;
        self.dynamic_rectangle_replace = false;
    }
    fn rectangle_dynamic_anchor(&self) -> Option<Point> {
        if self.mode.is_some() && self.tool == Tool::Rectangle && !self.input_locked {
            self.rectangle_seed()
        } else {
            None
        }
    }
    fn rectangle_endpoint(&self, aim: Point) -> Option<Point> {
        let aim = self.polar_snap(aim).map_or(aim, |s| s.point);
        let a = self.rectangle_dynamic_anchor()?;
        let mut p = aim;
        for axis in 0..2 {
            let delta = aim[axis] - a[axis];
            let size = if self.dynamic_rectangle[axis].is_empty() {
                delta.abs()
            } else {
                self.dynamic_rectangle[axis]
                    .replace(',', ".")
                    .parse::<f64>()
                    .ok()?
            };
            if !size.is_finite() || size <= 0. {
                return None;
            }
            p[axis] = a[axis] + if delta < 0. { -size } else { size };
        }
        Some(p)
    }
    fn rectangle_dynamic_input(&mut self, ctx: &egui::Context) -> bool {
        if self.rectangle_dynamic_anchor().is_none() {
            return false;
        }
        if ctx.input(|i| i.modifiers.ctrl || i.modifiers.command || i.modifiers.alt) {
            return true;
        }
        let pointer = self.dynamic_rect.and_then(|r| {
            ctx.input(|i| i.pointer.hover_pos())
                .filter(|p| r.contains(*p) && !self.toolbar_rect.is_some_and(|t| t.contains(*p)))
                .map(|p| self.dynamic_aim.unwrap_or_else(|| self.camera.world(p, r)))
        });
        let Some(aim) = pointer else {
            if self.dynamic_rectangle.iter().any(|s| !s.is_empty()) {
                ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
            }
            return true;
        };
        // Tab belongs to drawing while the pointer is on the canvas. Undo egui's
        // generic focus navigation so it cannot move into a sidebar input.
        if ctx.input(|i| i.key_pressed(egui::Key::Tab)) {
            if let Some(id) = ctx.memory(|m| m.focused()) {
                ctx.memory_mut(|m| m.surrender_focus(id));
            }
        }
        if ctx.wants_keyboard_input() {
            return true;
        }
        for event in ctx.input(|i| i.events.clone()) {
            match event {
                egui::Event::Key {
                    key: egui::Key::Tab,
                    pressed: true,
                    ..
                } => {
                    ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Tab));
                    ctx.input_mut(|i| i.consume_key(egui::Modifiers::SHIFT, egui::Key::Tab));
                    self.dynamic_rectangle_field = 1 - self.dynamic_rectangle_field;
                    self.dynamic_rectangle_replace = true;
                }
                egui::Event::Text(text) => {
                    let chars = text
                        .chars()
                        .filter(|c| c.is_ascii_digit() || *c == '.' || *c == ',')
                        .collect::<String>();
                    if !chars.is_empty() {
                        let value = &mut self.dynamic_rectangle[self.dynamic_rectangle_field];
                        if self.dynamic_rectangle_replace {
                            value.clear();
                            self.dynamic_rectangle_replace = false;
                        }
                        for c in chars.chars() {
                            if value.len() < 32 {
                                value.push(c);
                            }
                        }
                    }
                }
                egui::Event::Key {
                    key: egui::Key::Backspace,
                    pressed: true,
                    ..
                } => {
                    if self.dynamic_rectangle.iter().any(|s| !s.is_empty()) {
                        ctx.input_mut(|i| {
                            i.consume_key(egui::Modifiers::NONE, egui::Key::Backspace)
                        });
                        let value = &mut self.dynamic_rectangle[self.dynamic_rectangle_field];
                        if self.dynamic_rectangle_replace {
                            value.clear();
                            self.dynamic_rectangle_replace = false;
                        } else {
                            value.pop();
                        }
                    }
                }
                egui::Event::Key {
                    key: egui::Key::Enter,
                    pressed: true,
                    ..
                } => {
                    ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
                    if let Some(p) = self.rectangle_endpoint(aim) {
                        self.sketch_click(p, false);
                        return true;
                    }
                    self.status = "Введите положительные b и h в мм; Tab — переключить поле".into();
                }
                _ => {}
            }
        }
        true
    }
    fn dynamic_anchor(&self) -> Option<Point> {
        (self.mode.is_some() && self.tool == Tool::Line && !self.input_locked)
            .then(|| self.draft.last().copied())
            .flatten()
    }
    pub(super) fn dynamic_endpoint(&self, aim: Point) -> Option<Point> {
        if self.rectangle_dynamic_anchor().is_some() {
            return if self.dynamic_rectangle.iter().all(|s| s.is_empty()) {
                None
            } else {
                self.rectangle_endpoint(aim)
            };
        }
        let aim = self.polar_snap(aim).map_or(aim, |s| s.point);
        let a = self.dynamic_anchor()?;
        let length = self.dynamic_length.replace(',', ".").parse::<f64>().ok()?;
        let d = [aim[0] - a[0], aim[1] - a[1]];
        let norm = d[0].hypot(d[1]);
        (length.is_finite() && length > 0. && norm > 1e-9)
            .then(|| [a[0] + d[0] * length / norm, a[1] + d[1] * length / norm])
    }
    pub(super) fn dynamic_input(&mut self, ctx: &egui::Context) {
        if self.reuse_mode.is_some() {
            return;
        }
        if self.rectangle_dynamic_input(ctx) {
            return;
        }
        if ctx.wants_keyboard_input() || self.dynamic_anchor().is_none() {
            return;
        }
        let Some(rect) = self.dynamic_rect else {
            return;
        };
        let Some(pointer) = ctx
            .input(|i| i.pointer.hover_pos())
            .filter(|p| rect.contains(*p) && !self.toolbar_rect.is_some_and(|r| r.contains(*p)))
        else {
            if !self.dynamic_length.is_empty() {
                ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter));
            }
            return;
        };
        if ctx.input(|i| i.modifiers.ctrl || i.modifiers.command || i.modifiers.alt) {
            return;
        }
        for event in ctx.input(|i| i.events.clone()) {
            if let egui::Event::Text(text) = event {
                for c in text
                    .chars()
                    .filter(|c| c.is_ascii_digit() || *c == '.' || *c == ',')
                {
                    if self.dynamic_length.len() < 32 {
                        self.dynamic_length.push(c);
                    }
                }
            }
        }
        if self.dynamic_length.is_empty() {
            return;
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Backspace)) {
            self.dynamic_length.pop();
        }
        if ctx.input_mut(|i| i.consume_key(egui::Modifiers::NONE, egui::Key::Enter)) {
            if let Some(p) = self.dynamic_endpoint(
                self.dynamic_aim
                    .unwrap_or_else(|| self.camera.world(pointer, rect)),
            ) {
                self.sketch_click(p, false);
                self.dynamic_length.clear();
            } else {
                self.status =
                    "Введите положительную длину в мм и задайте направление курсором".into();
            }
        }
    }
    pub(super) fn draw_dynamic_length(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        aim: Option<Point>,
    ) {
        if let (Some(a), Some(b)) = (self.rectangle_dynamic_anchor(), aim) {
            for axis in 0..2 {
                let name = if axis == 0 { "b" } else { "h" };
                let size = if self.dynamic_rectangle[axis].is_empty() {
                    format!("{:.1}", (b[axis] - a[axis]).abs())
                } else {
                    self.dynamic_rectangle[axis].clone()
                };
                let text = format!("{name}: {size} мм");
                let g = painter.layout_no_wrap(text, egui::FontId::monospace(13.), Color32::WHITE);
                let center = if axis == 0 {
                    [(a[0] + b[0]) * 0.5, a[1]]
                } else {
                    [b[0], (a[1] + b[1]) * 0.5]
                };
                let center = self.camera.screen(center, rect)
                    + if axis == 0 {
                        egui::vec2(0., -18.)
                    } else {
                        egui::vec2(35., 0.)
                    };
                let r = Rect::from_center_size(center, g.size() + egui::vec2(10., 6.));
                let color = if axis == self.dynamic_rectangle_field {
                    Color32::from_rgb(35, 100, 150)
                } else {
                    Color32::from_rgb(30, 50, 65)
                };
                painter.rect_filled(r, 3., color);
                painter.galley(r.min + egui::vec2(5., 3.), g, Color32::WHITE);
            }
            return;
        }
        let (Some(a), Some(b)) = (self.dynamic_anchor(), aim) else {
            return;
        };
        let text = if self.dynamic_length.is_empty() {
            format!("{:.1} мм", (b[0] - a[0]).hypot(b[1] - a[1]))
        } else {
            format!("{} мм ↵", self.dynamic_length)
        };
        let g = painter.layout_no_wrap(text, egui::FontId::monospace(13.), Color32::WHITE);
        let center = self
            .camera
            .screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], rect);
        let r = Rect::from_center_size(center, g.size() + egui::vec2(10., 6.));
        painter.rect_filled(r, 3., Color32::from_rgb(30, 50, 65));
        painter.galley(r.min + egui::vec2(5., 3.), g, Color32::WHITE);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rectangle_width_tab_height_enter_respects_cursor_quadrant_and_repeats() {
        let ctx = egui::Context::default();
        let mut a = App::default();
        a.snap_settings.polar = true;
        a.repeat_sketch = true;
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([100., 200.], false);
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        a.dynamic_rect = Some(rect);
        let pointer = a.camera.screen([70., 160.], rect);
        ctx.memory_mut(|m| m.request_focus(egui::Id::new("sidebar_input")));
        let key = |key| egui::Event::Key {
            key,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        };
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                events: vec![
                    egui::Event::PointerMoved(pointer),
                    egui::Event::Text("1500".into()),
                    key(egui::Key::Tab),
                    egui::Event::Text("800".into()),
                    key(egui::Key::Enter),
                ],
                ..Default::default()
            },
            |ctx| {
                a.dynamic_input(ctx);
                assert!(!ctx
                    .input(|i| i.key_pressed(egui::Key::Tab) || i.key_pressed(egui::Key::Enter)));
            },
        );
        assert_eq!(a.contours.len(), 1);
        let points = &a.contours[0].points;
        assert!(points.contains(&[-1400., -600.]));
        assert!(points.contains(&[100., 200.]));
        assert!((crate::plan_trace::geometry::area(points) - 1_200_000.).abs() < 1e-8);
        assert_eq!(a.mode, Some(Kind::Slab));
        assert!(a.dynamic_rectangle.iter().all(|s| s.is_empty()));
    }
    #[test]
    fn rectangle_partial_input_follows_cursor_and_tab_replaces_selected_dimension() {
        let ctx = egui::Context::default();
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.dynamic_rectangle[0] = "100".into();
        assert_eq!(a.dynamic_endpoint([-30., 40.]), Some([-100., 40.]));
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        a.dynamic_rect = Some(rect);
        let pointer = a.camera.screen([30., 40.], rect);
        let tab = || egui::Event::Key {
            key: egui::Key::Tab,
            physical_key: None,
            pressed: true,
            repeat: false,
            modifiers: egui::Modifiers::NONE,
        };
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                events: vec![
                    egui::Event::PointerMoved(pointer),
                    tab(),
                    egui::Event::Text("200".into()),
                    tab(),
                    egui::Event::Text("50".into()),
                ],
                ..Default::default()
            },
            |ctx| a.dynamic_input(ctx),
        );
        assert_eq!(a.dynamic_rectangle, ["50", "200"]);
        assert_eq!(a.dynamic_endpoint([30., 40.]), Some([50., 200.]));
        a.dynamic_rectangle[0] = "0".into();
        assert!(a.dynamic_endpoint([30., 40.]).is_none());
        a.cancel();
        assert!(a.dynamic_rectangle.iter().all(|s| s.is_empty()));
    }
    #[test]
    fn typed_length_and_enter_add_exact_endpoint_without_finishing_support() {
        let ctx = egui::Context::default();
        let mut app = App::default();
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        app.dynamic_rect = Some(rect);
        let pointer = app.camera.screen([30., 40.], rect);
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                events: vec![
                    egui::Event::PointerMoved(pointer),
                    egui::Event::Text("500".into()),
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
            |ctx| {
                app.dynamic_input(ctx);
                assert!(!ctx.input(|i| i.key_pressed(egui::Key::Enter)));
            },
        );
        assert_eq!(app.draft, vec![[0., 0.], [300., 400.]]);
        assert!(app.dynamic_length.is_empty());
        assert_eq!(app.mode, Some(Kind::Support));
    }
    #[test]
    fn length_follows_cursor_direction_in_model_units() {
        let mut a = App::default();
        a.begin(Kind::Support);
        a.sketch_click([100., 200.], false);
        a.dynamic_length = "500".into();
        assert_eq!(a.dynamic_endpoint([103., 204.]), Some([400., 600.]));
        a.dynamic_length = "0".into();
        assert!(a.dynamic_endpoint([103., 204.]).is_none());
        a.dynamic_length = "100".into();
        assert!(a.dynamic_endpoint([100., 200.]).is_none());
        a.input_locked = true;
        assert!(a.dynamic_endpoint([103., 204.]).is_none());
    }
}
