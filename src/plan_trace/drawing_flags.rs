use super::*;
pub(super) fn default_contour_colors() -> [[u8; 3]; 3] {
    [[80, 210, 140], [255, 145, 75], [90, 185, 255]]
}
pub(super) fn material_color_default(id: u64) -> [u8; 3] {
    const PALETTE: [[u8; 3]; 8] = [
        [90, 175, 235],
        [245, 163, 88],
        [118, 202, 128],
        [211, 141, 218],
        [230, 196, 94],
        [90, 196, 190],
        [235, 129, 148],
        [168, 164, 230],
    ];
    PALETTE[(id.saturating_sub(1) % PALETTE.len() as u64) as usize]
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn material_colors_follow_each_contour_and_take_priority_over_the_section_color() {
        let mut app = App::default();
        for x in [0., 3000.] {
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([x, 0.], false);
            app.sketch_click([x + 2000., 2000.], false);
        }
        app.sync_stiffness_sources();
        let [first, second] = [app.contours[0].id, app.contours[1].id];
        let section = app.stiffnesses.assignments[&first];
        assert_eq!(app.stiffnesses.assignments[&second], section);
        let first_material = app.stiffnesses.material_id(first).unwrap();
        let second_material = app.stiffnesses.elastic.keys().next_back().unwrap() + 1;
        let material = app.stiffnesses.elastic[&first_material].clone();
        app.stiffnesses.elastic.insert(second_material, material);
        app.stiffnesses
            .contour_materials
            .insert(second, second_material);
        app.stiffnesses
            .material_colors
            .insert(first_material, [10, 30, 50]);
        app.stiffnesses
            .material_colors
            .insert(second_material, [90, 80, 60]);
        app.stiffnesses.colors.insert(section, [22, 44, 66]);
        app.drawing_visibility.stiffness_colors = true;
        app.drawing_visibility.material_colors = true;
        assert_eq!(
            app.object_ink(first, Kind::Slab),
            Color32::from_rgb(10, 30, 50)
        );
        assert_eq!(
            app.object_ink(second, Kind::Slab),
            Color32::from_rgb(90, 80, 60)
        );
        app.drawing_visibility.material_colors = false;
        assert_eq!(
            app.object_ink(first, Kind::Slab),
            Color32::from_rgb(22, 44, 66)
        );
        assert_eq!(
            app.object_ink(second, Kind::Slab),
            Color32::from_rgb(22, 44, 66)
        );
    }

    #[test]
    fn self_weight_overlay_has_values_respects_flags_and_never_adds_editable_loads() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        app.sync_stiffness_sources();
        app.load_cases[0].kind = crate::plan_trace::loads::LoadType::SelfWeight;
        let before = app.contours.clone();
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.));
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Middle,
                egui::Id::new("self_weight"),
            ));
            app.draw_loads(&painter, app.camera, rect);
        });
        assert!(out.shapes.iter().any(
            |s| matches!(&s.shape,egui::Shape::Text(t) if t.galley.job.text.contains("0.55"))
        ));
        assert_eq!(app.contours, before);
        app.show_loads = false;
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Middle,
                egui::Id::new("self_weight"),
            ));
            app.draw_loads(&painter, app.camera, rect);
        });
        assert!(out.shapes.is_empty());
    }
    #[test]
    fn load_type_visibility_and_colors_are_independent_and_do_not_change_model() {
        let ctx = egui::Context::default();
        let viewport = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        for (i, kind) in [Kind::LoadArea, Kind::LoadLine, Kind::LoadPoint]
            .into_iter()
            .enumerate()
        {
            let mut a = App::default();
            a.begin(kind);
            if kind == Kind::LoadArea {
                a.choose_tool(Tool::Rectangle);
            }
            a.sketch_click([0., 0.], false);
            if kind != Kind::LoadPoint {
                a.sketch_click([100., 100.], false);
                a.finish();
            }
            assert_eq!(a.contours.len(), 1);
            let original = a.contours.clone();
            a.drawing_visibility.load_colors[i] = [11, 22, 33];
            assert_eq!(a.contour_color(kind), Color32::from_rgb(11, 22, 33));
            assert!(a.contour_visible(&a.contours[0]));
            a.select_only(Some(a.contours[0].id));
            a.drawing_visibility.loads[i] = false;
            a.prune_selection();
            assert!(!a.contour_visible(&a.contours[0]));
            assert!(!a.selectable(&a.contours[0]));
            assert!(a.selected.is_none());
            assert!(a.snap([0., 0.], None).is_none());
            let out = ctx.run(egui::RawInput::default(), |ctx| {
                let painter = ctx.layer_painter(egui::LayerId::new(
                    egui::Order::Middle,
                    egui::Id::new("loads"),
                ));
                a.draw_loads(&painter, a.camera, viewport);
            });
            assert!(out.shapes.is_empty());
            assert_eq!(a.contours, original);
            a.drawing_visibility.loads[i] = true;
            assert!(a.contour_visible(&a.contours[0]));
            for j in 0..3 {
                if j != i {
                    assert!(a.drawing_visibility.loads[j]);
                }
            }
        }
    }
}
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct Visibility {
    pub dark_theme: bool,
    pub canvas_background: [u8; 3],
    pub load_opacity: f32,
    pub ground_opacity: f32,
    pub fill: bool,
    pub stiffness_colors: bool,
    pub material_colors: bool,
    pub support_extents: bool,
    #[serde(alias = "support_edges")]
    pub structure_edges: bool,
    pub support_storeys: u8,
    pub restraints: bool,
    pub punching: bool,
    pub foundation: bool,
    #[serde(default = "column_true")]
    pub columns: bool,
    #[serde(default = "column_true")]
    pub rigid_bodies: bool,
    #[serde(default = "column_color")]
    pub column_color: [u8; 3],
    #[serde(default = "rigid_color")]
    pub rigid_color: [u8; 3],
    pub beam: bool,
    pub triangulation: bool,
    pub triangulation_color: [u8; 3],
    pub beam_color: [u8; 3],
    pub contours: [bool; 3],
    pub loads: [bool; 3],
    pub load_colors: [[u8; 3]; 3],
}
impl Visibility {
    pub(super) fn exclusive_colors(mut self) -> Self {
        if self.material_colors {
            self.stiffness_colors = false;
        }
        self
    }
}
impl Default for Visibility {
    fn default() -> Self {
        Self {
            dark_theme: true,
            canvas_background: [28, 33, 41],
            load_opacity: 0.25,
            ground_opacity: 0.30,
            fill: true,
            stiffness_colors: false,
            material_colors: false,
            support_extents: false,
            structure_edges: false,
            support_storeys: 0,
            restraints: true,
            punching: true,
            foundation: true,
            columns: true,
            rigid_bodies: true,
            column_color: column_color(),
            rigid_color: rigid_color(),
            beam: true,
            triangulation: true,
            triangulation_color: [225, 120, 230],
            beam_color: [255, 210, 80],
            contours: [true; 3],
            loads: [true; 3],
            load_colors: [[245, 75, 75]; 3],
        }
    }
}
impl Visibility {
    pub fn border(self, kind: Kind) -> bool {
        match kind {
            Kind::Punching => self.punching,
            Kind::Foundation => self.foundation,
            Kind::Column => self.columns,
            Kind::Restraint | Kind::PointRestraint => self.restraints,
            Kind::RigidLink => self.rigid_bodies,
            Kind::Beam => self.beam,
            Kind::Triangulation => self.triangulation,
            Kind::Slab => self.contours[0],
            Kind::Hole => self.contours[1],
            Kind::Support => self.contours[2],
            Kind::LoadArea | Kind::LoadHole => self.loads[0],
            Kind::LoadLine => self.loads[1],
            Kind::LoadPoint => self.loads[2],
            _ => true,
        }
    }
}
#[derive(Clone, Copy, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(crate) struct TextStyle {
    pub color: [u8; 3],
    pub size: f32,
    pub decimals: usize,
}
impl Default for TextStyle {
    fn default() -> Self {
        Self {
            color: [255; 3],
            size: 12.,
            decimals: 2,
        }
    }
}
impl TextStyle {
    pub fn valid(self) -> bool {
        self.size.is_finite() && (6. ..=48.).contains(&self.size) && self.decimals <= 8
    }
    pub fn ink(self) -> Color32 {
        Color32::from_rgb(self.color[0], self.color[1], self.color[2])
    }
    pub fn number(self, value: f64) -> String {
        let text = format!("{:.*}", self.decimals.min(8), value);
        // A tiny negative value rounded to zero must not display a minus sign.
        if text.starts_with('-') && text.parse::<f64>().ok() == Some(0.) {
            text[1..].to_owned()
        } else {
            text
        }
    }
    pub fn numeric_text(self, raw: &str) -> String {
        raw.trim()
            .replace(',', ".")
            .parse::<f64>()
            .map(|v| self.number(v))
            .unwrap_or_else(|_| raw.to_owned())
    }
}
impl App {
    pub(super) fn property_ink(&self, id: u64, kind: Kind) -> Option<Color32> {
        use crate::plan_trace::stiffness::Family;
        if self.schema_analysis.mode.is_some()
            && (matches!(
                self.schema_analysis.mode,
                Some(schema_analysis::Mode::Floors | schema_analysis::Mode::MovementLocks)
            ) || matches!(kind, Kind::Slab | Kind::Support | Kind::Column | Kind::Beam))
        {
            return Some(self.schema_ink(id).unwrap_or(Color32::GRAY));
        }
        let family = match kind {
            Kind::Slab => Family::Slab,
            Kind::Support => Family::Wall,
            Kind::Beam => Family::Beam,
            Kind::Column => Family::Column,
            _ => return None,
        };
        if self.drawing_visibility.material_colors {
            if let Some(material) = self.stiffnesses.material_id(id) {
                let [r, g, b] = self
                    .stiffnesses
                    .material_colors
                    .get(&material)
                    .copied()
                    .unwrap_or_else(|| material_color_default(material));
                return Some(Color32::from_rgb(r, g, b));
            }
        }
        if !self.drawing_visibility.stiffness_colors {
            return None;
        }
        let property = self.stiffnesses.assignments.get(&id)?;
        let [r, g, b] = self
            .stiffnesses
            .colors
            .get(property)
            .copied()
            .unwrap_or_else(|| super::stiffness_ui::stiffness_color_default(family));
        Some(Color32::from_rgb(r, g, b))
    }
    pub(super) fn object_ink(&self, id: u64, kind: Kind) -> Color32 {
        self.property_ink(id, kind)
            .unwrap_or_else(|| self.contour_color(kind))
    }
    pub(super) fn slab_property_owner(&self, point: Point) -> Option<u64> {
        self.contours
            .iter()
            .filter(|c| c.kind == Kind::Slab && super::geometry::inside(point, &c.points))
            .filter(|c| {
                !self.contours.iter().any(|h| {
                    h.active_hole()
                        && h.parent == Some(c.id)
                        && super::geometry::inside(point, &h.points)
                })
            })
            .min_by(|a, b| {
                super::geometry::area(&a.points)
                    .abs()
                    .total_cmp(&super::geometry::area(&b.points).abs())
                    .then_with(|| b.id.cmp(&a.id))
            })
            .map(|c| c.id)
    }
    pub(super) fn contour_color(&self, kind: Kind) -> Color32 {
        if kind == Kind::RigidLink {
            let [r, g, b] = self.drawing_visibility.rigid_color;
            return Color32::from_rgb(r, g, b);
        }
        if kind == Kind::Triangulation {
            let c = self.drawing_visibility.triangulation_color;
            return Color32::from_rgb(c[0], c[1], c[2]);
        }
        if kind == Kind::Column {
            return Color32::from_rgb(
                self.drawing_visibility.column_color[0],
                self.drawing_visibility.column_color[1],
                self.drawing_visibility.column_color[2],
            );
        }
        if kind == Kind::Beam {
            let c = self.drawing_visibility.beam_color;
            return Color32::from_rgb(c[0], c[1], c[2]);
        }
        let index = match kind {
            Kind::Slab => 0,
            Kind::Hole => 1,
            Kind::Support => 2,
            Kind::LoadArea | Kind::LoadLine | Kind::LoadPoint => {
                let index = match kind {
                    Kind::LoadArea => 0,
                    Kind::LoadLine => 1,
                    _ => 2,
                };
                let c = self.drawing_visibility.load_colors[index];
                return Color32::from_rgb(c[0], c[1], c[2]);
            }
            _ => return kind.color(),
        };
        let [r, g, b] = self.contour_colors[index];
        Color32::from_rgb(r, g, b)
    }

    pub(super) fn appearance_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.vertical(|ui| {
                ui.label("Тема");
                egui::ComboBox::from_id_salt("appearance_theme")
                    .width(74.)
                    .selected_text(if self.drawing_visibility.dark_theme {
                        "Тёмная"
                    } else {
                        "Светлая"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut self.drawing_visibility.dark_theme,
                            true,
                            "Тёмная",
                        );
                        ui.selectable_value(
                            &mut self.drawing_visibility.dark_theme,
                            false,
                            "Светлая",
                        );
                    });
            });
            ui.vertical(|ui| {
                ui.label("Фон");
                crate::plan_trace::color_picker::button(
                    ui,
                    &mut self.drawing_visibility.canvas_background,
                );
            });
            self.units_controls(ui);
            ui.vertical(|ui| {
                ui.label("Знаков")
                    .on_hover_text("Число знаков после запятой");
                egui::ComboBox::from_id_salt("text_decimal_places")
                    .width(45.)
                    .selected_text(self.display_text.decimals.to_string())
                    .show_ui(ui, |ui| {
                        for value in 0..=6 {
                            ui.selectable_value(
                                &mut self.display_text.decimals,
                                value,
                                value.to_string(),
                            );
                        }
                    });
            });
        });
        self.apply_appearance(ui.ctx());
    }
    pub(super) fn apply_appearance(&self, ctx: &egui::Context) {
        if ctx.style().visuals.dark_mode != self.drawing_visibility.dark_theme {
            ctx.set_visuals(if self.drawing_visibility.dark_theme {
                egui::Visuals::dark()
            } else {
                egui::Visuals::light()
            });
        }
    }
    pub(super) fn canvas_background(&self) -> Color32 {
        let [r, g, b] = self.drawing_visibility.canvas_background;
        Color32::from_rgb(r, g, b)
    }
    pub(super) fn units_controls(&mut self, ui: &mut egui::Ui) {
        ui.vertical(|ui| {
            ui.label("Единицы");
            egui::ComboBox::from_id_salt("force_units")
                .width(56.)
                .truncate()
                .selected_text(self.load_units.label())
                .show_ui(ui, |ui| {
                    for units in [
                        crate::plan_trace::loads::ForceUnits::TonForce,
                        crate::plan_trace::loads::ForceUnits::KiloNewton,
                    ] {
                        ui.selectable_value(&mut self.load_units, units, units.label());
                    }
                });
        });
        ui.vertical(|ui| {
            ui.label("g, м/с²");
            ui.add_enabled(
                !self.input_locked,
                egui::DragValue::new(&mut self.gravity)
                    .range(0.001..=1000.)
                    .speed(0.01),
            );
        });
        self.sync_result_units();
    }
    fn text_style_controls(&mut self, ui: &mut egui::Ui, dimensions: bool) {
        ui.horizontal(|ui| {
            if dimensions {
                ui.checkbox(&mut self.measurements.show, "Размеры");
            }
            ui.vertical(|ui| {
                ui.label("Цвет");
                crate::plan_trace::color_picker::button(ui, &mut self.display_text.color);
            });
            ui.vertical(|ui| {
                ui.label("Кегль");
                ui.add(
                    egui::DragValue::new(&mut self.display_text.size)
                        .range(6. ..=48.)
                        .speed(0.5),
                );
            });
        });
    }
    pub(super) fn results_controls(&mut self, ui: &mut egui::Ui) {
        let has_fe = self.analysis.is_some() || self.fe_mesh.is_some();
        if !has_fe && !self.has_soil_pressure_results() {
            return;
        }
        egui::CollapsingHeader::new("Результаты")
            .default_open(false)
            .show(ui, |ui| {
                if has_fe {
                    self.result_view_controls(ui);
                } else {
                    self.soil_pressure_results(ui);
                }
            });
    }
    pub(super) fn drawing_flags_window(&mut self, ctx: &egui::Context) {
        if !self.drawing_flags_open {
            return;
        }
        let mut open = self.drawing_flags_open;
        egui::Window::new("Флаги рисования")
            .id(egui::Id::new("drawing_flags_window"))
            .order(egui::Order::Foreground)
            .open(&mut open)
            .default_width(350.)
            .resizable(true)
            .collapsible(false)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical()
                    .id_salt("drawing_flags_scroll")
                    .max_height((ctx.screen_rect().height() - 100.).max(80.))
                    .show(ui, |ui| self.drawing_flags_controls(ui));
            });
        self.drawing_flags_open = open;
    }
    pub(super) fn drawing_flags_controls(&mut self, ui: &mut egui::Ui) {
        if self
            .fe_mesh
            .as_ref()
            .is_some_and(|m| !m.elements.is_empty())
        {
            ui.checkbox(&mut self.show_mesh, "Показывать рёбра КЭ");
        }
        ui.checkbox(&mut self.show_support_nodes, "Узлы границ стен");
        ui.horizontal(|ui| {
            ui.label("Размер точки");
            ui.add(
                egui::DragValue::new(&mut self.point_size)
                    .range(1. ..=30.)
                    .speed(0.5)
                    .suffix(" px"),
            )
            .on_hover_text("Диаметр узловых маркеров результатов");
        });
        ui.push_id("drawing_text_style", |ui| {
            self.text_style_controls(ui, true)
        });
        ui.separator();
        if ui
            .checkbox(&mut self.show_dxf, "Подложки PDF / DXF")
            .changed()
        {
            self.rebuild_index();
        }
        ui.horizontal(|ui| {
            ui.checkbox(&mut self.show_text, "Текст DXF");
            ui.checkbox(&mut self.contrast_text, "Контрастно");
        });

        // Independent rows avoid Grid column-width retention and expanding child UIs.
        ui.horizontal(|ui| {
            visibility_prefix(
                ui,
                &mut self.drawing_visibility.fill,
                Some(&mut self.slab_color),
            );
            ui.label("Заливка плиты");
            opacity_control(ui, &mut self.slab_opacity);
        });
        if ui
            .checkbox(
                &mut self.drawing_visibility.stiffness_colors,
                "Цвета жёсткостей",
            )
            .changed()
            && self.drawing_visibility.stiffness_colors
        {
            self.drawing_visibility.material_colors = false;
        }
        if ui
            .checkbox(
                &mut self.drawing_visibility.material_colors,
                "Цвета материалов",
            )
            .changed()
            && self.drawing_visibility.material_colors
        {
            self.drawing_visibility.stiffness_colors = false;
        }
        for (i, label) in ["Границы плит", "Границы отверстий", "Границы стен"]
            .iter()
            .enumerate()
        {
            visibility_row(
                ui,
                &mut self.drawing_visibility.contours[i],
                Some(&mut self.contour_colors[i]),
                label,
            );
        }
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 5.;
            visibility_prefix(ui, &mut self.drawing_visibility.support_extents, None);
            ui.label("Габариты опор");
        });
        ui.checkbox(&mut self.drawing_visibility.structure_edges, "Рёбра")
            .on_hover_text("Границы конструкций и линии среза в 3D");
        ui.horizontal(|ui| {
            ui.label("Непрозрачность грунта");
            opacity_control(ui, &mut self.drawing_visibility.ground_opacity);
        });
        if self.building.enabled {
            visibility_row(ui, &mut self.drawing_visibility.restraints, None, "Опоры");
        }
        visibility_row(
            ui,
            &mut self.drawing_visibility.columns,
            Some(&mut self.drawing_visibility.column_color),
            "Колонны",
        );
        visibility_row(
            ui,
            &mut self.drawing_visibility.rigid_bodies,
            Some(&mut self.drawing_visibility.rigid_color),
            "АЖТ",
        );
        visibility_row(
            ui,
            &mut self.drawing_visibility.beam,
            Some(&mut self.drawing_visibility.beam_color),
            "Балки",
        );
        visibility_row(
            ui,
            &mut self.drawing_visibility.punching,
            None,
            "Области суммирования нагрузок",
        );
        visibility_row(
            ui,
            &mut self.drawing_visibility.triangulation,
            Some(&mut self.drawing_visibility.triangulation_color),
            "Ограничения триангуляции",
        );
        visibility_row(
            ui,
            &mut self.drawing_visibility.foundation,
            None,
            "Основания",
        );
        let mut show = self.show_loads;
        ui.horizontal(|ui| {
            ui.spacing_mut().item_spacing.x = 5.;
            visibility_prefix(ui, &mut show, None);
            ui.label("Нагрузки");
            ui.add_enabled(
                self.show_loads,
                egui::Checkbox::new(&mut self.show_load_values, "Значения"),
            );
            opacity_control(ui, &mut self.drawing_visibility.load_opacity);
        });
        if show != self.show_loads {
            self.set_load_visibility(show);
        }
        for (i, label) in ["Распределённые", "Линейные", "Сосредоточенные"]
            .iter()
            .enumerate()
        {
            visibility_row(
                ui,
                &mut self.drawing_visibility.loads[i],
                Some(&mut self.drawing_visibility.load_colors[i]),
                label,
            );
        }
        self.prune_selection();
        ui.separator();
        if ui.button("Вписать чертёж").clicked() {
            if self.spatial.enabled {
                self.spatial.fit_view();
            } else {
                self.fit = true;
            }
            ui.ctx().request_repaint();
        }
    }
}

fn visibility_prefix(ui: &mut egui::Ui, visible: &mut bool, color: Option<&mut [u8; 3]>) {
    let side = ui.spacing().icon_width;
    let (prefix, response) =
        ui.allocate_exact_size(egui::vec2(side * 2. + 5., side), egui::Sense::hover());
    let check = Rect::from_min_size(prefix.min, egui::Vec2::splat(side));
    let color_rect = Rect::from_min_size(
        prefix.min + egui::vec2(side + 5., 0.),
        egui::Vec2::splat(side),
    );
    let mut check_ui = ui.new_child(
        egui::UiBuilder::new()
            .id_salt(response.id.with("check"))
            .max_rect(check),
    );
    check_ui
        .add(egui::Checkbox::new(visible, ""))
        .on_hover_text("Показывать");
    if let Some(color) = color {
        let mut color_ui = ui.new_child(
            egui::UiBuilder::new()
                .id_salt(response.id.with("color"))
                .max_rect(color_rect),
        );
        crate::plan_trace::color_picker::square(&mut color_ui, color);
    }
}
fn visibility_row(ui: &mut egui::Ui, visible: &mut bool, color: Option<&mut [u8; 3]>, label: &str) {
    ui.horizontal(|ui| {
        ui.spacing_mut().item_spacing.x = 5.;
        visibility_prefix(ui, visible, color);
        ui.label(label);
    });
}

#[cfg(test)]
mod compact_row_regression {
    use super::*;
    #[test]
    fn colour_slot_is_checkbox_sized_and_labels_align_without_grid_padding() {
        let ctx = egui::Context::default();
        let mut positions = vec![];
        let mut enabled = true;
        let mut color = [80, 120, 160];
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(400., 300.))),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    let side = ui.spacing().icon_width;
                    for coloured in [true, false] {
                        ui.horizontal(|ui| {
                            ui.spacing_mut().item_spacing.x = 5.;
                            let start = ui.cursor().left();
                            visibility_prefix(
                                ui,
                                &mut enabled,
                                if coloured { Some(&mut color) } else { None },
                            );
                            let label = ui.label("Границы плит");
                            positions.push(label.rect.left());
                            assert!(
                                label.rect.left() - start <= 2. * side + 12.,
                                "prefix too wide: {}",
                                label.rect.left() - start
                            );
                        });
                    }
                });
            },
        );
        assert!((positions[0] - positions[1]).abs() < 0.1);
    }
}

pub(super) fn window_button(ui: &mut egui::Ui, open: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(30., 24.), egui::Sense::click());
    let style = ui.style().interact_selectable(&response, open);
    let painter = ui.painter();
    let ink = style.fg_stroke.color;
    painter.rect_filled(rect, 3., style.bg_fill);
    for (x, y) in [(6., 5.), (16., 8.)] {
        let top = rect.min + egui::vec2(x, y);
        painter.line_segment(
            [top, rect.min + egui::vec2(x, 19.)],
            Stroke::new(1.5_f32, ink),
        );
        painter.add(egui::Shape::convex_polygon(
            vec![top, top + egui::vec2(8., 2.5), top + egui::vec2(0., 5.)],
            ink,
            Stroke::NONE,
        ));
    }
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), "Флаги рисования")
    });
    response.on_hover_text("Флаги рисования")
}

pub(super) fn properties_button(ui: &mut egui::Ui, open: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(30., 24.), egui::Sense::click());
    let style = ui.style().interact_selectable(&response, open);
    let p = ui.painter();
    p.rect_filled(rect, 3., style.bg_fill);
    for (row, offset) in [(0, 7.), (1, 15.), (2, 10.)] {
        let y = rect.top() + 6. + row as f32 * 6.;
        p.line_segment(
            [
                egui::pos2(rect.left() + 5., y),
                egui::pos2(rect.right() - 5., y),
            ],
            Stroke::new(1.3_f32, style.fg_stroke.color),
        );
        p.rect_filled(
            Rect::from_center_size(egui::pos2(rect.left() + offset, y), egui::vec2(3., 5.)),
            1.,
            style.fg_stroke.color,
        );
    }
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), "Свойства")
    });
    response.on_hover_text("Свойства")
}

fn column_true() -> bool {
    true
}
fn column_color() -> [u8; 3] {
    [190, 160, 240]
}
fn rigid_color() -> [u8; 3] {
    [255, 155, 40]
}

fn opacity_control(ui: &mut egui::Ui, value: &mut f32) {
    ui.scope(|ui| {
        ui.spacing_mut().slider_width = 62.;
        ui.add(egui::Slider::new(value, 0.05..=1.).fixed_decimals(2))
            .on_hover_text("Непрозрачность");
    });
}
