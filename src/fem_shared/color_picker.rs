use eframe::egui;

pub fn button(ui: &mut egui::Ui, rgb: &mut [u8; 3]) -> egui::Response {
    button_impl(ui, rgb, None)
}
pub fn square(ui: &mut egui::Ui, rgb: &mut [u8; 3]) -> egui::Response {
    let size = ui.spacing().icon_width;
    button_impl(ui, rgb, Some(size))
}
fn button_impl(ui: &mut egui::Ui, rgb: &mut [u8; 3], size: Option<f32>) -> egui::Response {
    let button = egui::Button::new(" ").fill(egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]));
    let mut response = if let Some(size) = size {
        let (slot, _) = ui.allocate_exact_size(egui::Vec2::splat(size), egui::Sense::hover());
        ui.put(slot, button.min_size(egui::Vec2::ZERO).small())
    } else {
        ui.add(button)
    };
    let id = response.id.with("palette_popup");
    let mut open = ui.data_mut(|d| d.get_temp::<bool>(id).unwrap_or(false));
    let commit_id = id.with("pending_color_commit");
    let mut custom_dirty = ui.data_mut(|d| d.get_temp::<bool>(commit_id).unwrap_or(false));
    if response.clicked() {
        open = !open;
        custom_dirty = false;
    }
    let mut changed = false;
    let mut picked = false;
    egui::Window::new("Выбор цвета")
        .id(id)
        .open(&mut open)
        .default_pos(response.rect.left_bottom())
        .default_width(383.)
        .resizable(false)
        .collapsible(false)
        .show(ui.ctx(), |ui| {
            let tab_id = id.with("tab");
            let mut custom = ui.data_mut(|d| d.get_temp::<bool>(tab_id).unwrap_or(false));
            ui.horizontal(|ui| {
                ui.selectable_value(&mut custom, false, "Палитра");
                ui.selectable_value(&mut custom, true, "Произвольный цвет");
            });
            ui.data_mut(|d| d.insert_temp(tab_id, custom));
            ui.separator();
            if custom {
                ui.spacing_mut().slider_width = ui.available_width().max(260.);
                let mut color = egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
                if egui::color_picker::color_picker_color32(
                    ui,
                    &mut color,
                    egui::color_picker::Alpha::Opaque,
                ) {
                    *rgb = [color.r(), color.g(), color.b()];
                    changed = true;
                    custom_dirty = true;
                }
            } else {
                ui.spacing_mut().item_spacing = egui::vec2(1., 1.);
                // Two blocks: dark-to-saturated colors and pale-to-dark colors.
                for row in 0..11 {
                    if row == 5 {
                        ui.add_space(6.);
                    }
                    ui.horizontal(|ui| {
                        for col in 0..24 {
                            let hue = col as f32 / 24.;
                            let (sat, value) = if row < 5 {
                                (1., 0.18 + row as f32 * 0.205)
                            } else {
                                (0.28 + (row - 5) as f32 * 0.11, 1. - (row - 5) as f32 * 0.15)
                            };
                            let color =
                                egui::Color32::from(egui::ecolor::Hsva::new(hue, sat, value, 1.));
                            let (rect, r) =
                                ui.allocate_exact_size(egui::vec2(15., 14.), egui::Sense::click());
                            ui.painter().rect_filled(rect, 0., color);
                            if r.hovered() || *rgb == [color.r(), color.g(), color.b()] {
                                ui.painter().rect_stroke(
                                    rect,
                                    0.,
                                    egui::Stroke::new(1.0_f32, egui::Color32::WHITE),
                                    egui::StrokeKind::Inside,
                                );
                            }
                            if r.clicked() {
                                *rgb = [color.r(), color.g(), color.b()];
                                changed = true;
                                picked = true;
                            }
                        }
                    });
                }
                ui.horizontal(|ui| {
                    for i in 0..24 {
                        let v = (i * 255 / 23) as u8;
                        if ui
                            .add_sized(
                                [15., 14.],
                                egui::Button::new("").fill(egui::Color32::from_gray(v)),
                            )
                            .clicked()
                        {
                            *rgb = [v; 3];
                            changed = true;
                            picked = true;
                        }
                    }
                });
            }
        });
    // Commit custom color after the whole drag/click, not on mouse-down;
    // numeric RGB entry commits on Enter rather than on its first digit.
    if picked
        || (custom_dirty
            && ui.input(|i| i.pointer.any_released() || i.key_pressed(egui::Key::Enter)))
    {
        open = false;
    }
    if !open {
        custom_dirty = false;
    }
    ui.data_mut(|d| {
        d.insert_temp(id, open);
        d.insert_temp(commit_id, custom_dirty);
    });
    if changed {
        response.mark_changed();
    }
    response
}
