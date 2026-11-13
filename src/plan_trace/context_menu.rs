use super::*;
pub(super) struct WallRestraints {
    walls: Vec<u64>,
    top: bool,
    mask: u8,
    error: String,
}
impl App {
    pub(super) fn automatic_restraint_dialog(&mut self, ctx: &egui::Context) {
        let id = egui::Id::new("automatic_restraint_prompt");
        let Some(mut state) = ctx.data(|d| d.get_temp::<[u8; 3]>(id)) else {
            return;
        };
        let mut generate = false;
        let mut cancel = false;
        let response = egui::Modal::new(id).show(ctx, |ui| {
            if state[0] == 0 {
                ui.heading("В модели нет опор");
                ui.label("Сгенерировать закрепления автоматически?");
                ui.horizontal(|ui| {
                    if ui.button("Да").clicked() {
                        state = [1, 7, 0];
                    }
                    cancel = ui.button("Отмена").clicked();
                });
            } else {
                ui.heading("Автоматические закрепления");
                ui.set_min_width(420.);
                ui.columns(2, |columns| {
                    for (i, title) in ["Низ нижнего этажа", "Верх верхнего этажа"]
                        .into_iter()
                        .enumerate()
                    {
                        let ui = &mut columns[i];
                        ui.label(title);
                        for (axis, label) in
                            ["X", "Y", "Z", "Ux", "Uy", "Uz"].into_iter().enumerate()
                        {
                            let bit = 1 << axis;
                            let mut enabled = state[i + 1] & bit != 0;
                            if ui.checkbox(&mut enabled, label).changed() {
                                state[i + 1] ^= bit;
                            }
                        }
                    }
                });
                ui.horizontal(|ui| {
                    generate = ui
                        .add_enabled(
                            state[1] | state[2] != 0,
                            egui::Button::new("Сгенерировать и рассчитать"),
                        )
                        .clicked();
                    cancel = ui.button("Отмена").clicked();
                });
            }
        });
        if generate || cancel || response.should_close() {
            ctx.data_mut(|d| d.remove::<[u8; 3]>(id));
        } else {
            ctx.data_mut(|d| d.insert_temp(id, state));
        }
        if generate {
            self.support_height.building_bottom_mask = state[1];
            self.support_height.building_top_mask = state[2];
            self.support_height.automatic_building_bottom = state[1] != 0;
            self.support_height.automatic_building_top = state[2] != 0;
            self.start_variant_analyses(ctx);
        }
    }
    pub(super) fn wall_restraint_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.wall_restraint_dialog.take() else {
            return;
        };
        let mut open = true;
        let mut create = false;
        egui::Window::new("Добавить опоры")
            .collapsible(false)
            .open(&mut open)
            .show(ctx, |ui| {
                egui::ComboBox::from_id_salt("wall_restraint_side")
                    .selected_text(if dialog.top {
                        "Сверху"
                    } else {
                        "Снизу"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut dialog.top, true, "Сверху");
                        ui.selectable_value(&mut dialog.top, false, "Снизу");
                    });
                for row in 0..2 {
                    ui.horizontal(|ui| {
                        for axis in 3 * row..3 * row + 3 {
                            let bit = 1 << axis;
                            let mut checked = dialog.mask & bit != 0;
                            if ui
                                .checkbox(&mut checked, ["X", "Y", "Z", "Ux", "Uy", "Uz"][axis])
                                .changed()
                            {
                                dialog.mask ^= bit;
                            }
                        }
                    });
                }
                if !dialog.error.is_empty() {
                    ui.colored_label(ui.visuals().error_fg_color, &dialog.error);
                }
                create = ui
                    .add_enabled(
                        dialog.mask != 0 && !self.is_kind_locked(Kind::Restraint),
                        egui::Button::new("Добавить"),
                    )
                    .clicked();
            });
        if create {
            match self.add_wall_restraints(&dialog) {
                Ok(()) => return,
                Err(error) => dialog.error = error,
            }
        }
        if open {
            self.wall_restraint_dialog = Some(dialog);
        }
    }

    fn add_wall_restraints(&mut self, dialog: &WallRestraints) -> Result<(), String> {
        if !self.building.enabled || self.is_kind_locked(Kind::Restraint) {
            return Err("Добавление опор недоступно".into());
        }
        let mut additions = Vec::new();
        for &id in &dialog.walls {
            let wall = self
                .contours
                .iter()
                .find(|c| c.id == id && c.kind == Kind::Support)
                .ok_or_else(|| format!("Стена {id} больше не существует"))?;
            let owner = self.building.owner(id);
            let floor = if dialog.top {
                let elevation = self.building.extent(owner)[1];
                self.building
                    .floors
                    .iter()
                    .find(|floor| {
                        floor.id != owner && (floor.elevation_mm - elevation).abs() < 1e-6
                    })
                    .ok_or_else(|| {
                        format!("Над стеной {id} нет этажа для размещения опоры. Выберите «Снизу».")
                    })?
                    .id
            } else {
                owner
            };
            let mut support = wall.clone();
            support.id = self
                .next_id
                .checked_add(additions.len() as u64)
                .ok_or("Исчерпаны ID")?;
            support.kind = Kind::Restraint;
            support.parent = None;
            support.restraint_mask = dialog.mask;
            model::validate_candidate(&self.contours, &support)?;
            additions.push((support, floor));
        }
        let count = additions.len();
        let next = self
            .next_id
            .checked_add(count as u64)
            .ok_or("Исчерпаны ID")?;
        self.history.push(self.contours.clone());
        for (support, floor) in additions {
            self.building.contour_floors.insert(support.id, floor);
            self.contours.push(support);
        }
        self.next_id = next;
        self.clear_mesh();
        self.invalidate_scene_buffers();
        self.status = format!("Добавлено опор: {count}");
        Ok(())
    }

    pub(super) fn scene_context_menu(&mut self, ui: &mut egui::Ui) {
        if ui.button("Свойства").clicked() {
            self.properties.open = true;
            ui.close_menu();
            return;
        }
        self.foundation_context_menu(ui);
        self.visibility_menu(ui);

        if self.scaling_op.is_some() {
            ui.label("Esc — отменить преобразование");
            return;
        }
        if self.measurements.active {
            ui.label("Esc — завершить измерение");
            return;
        }
        if self.clipboard_op.is_some() {
            ui.label("Esc — отменить копирование / вставку");
            return;
        }
        if (self.selected.is_some() || !self.selection_ids.is_empty())
            && ui.button("Копировать с базовой точкой").clicked()
        {
            self.copy_base_command();
            ui.close_menu();
            return;
        }
        self.dxf_contours_menu(ui);
        self.scaling_menu(ui);
        self.load_transfer_menu(ui);
        if self.building.enabled {
            let walls: Vec<_> = self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Support && self.is_selected(c.id))
                .map(|c| c.id)
                .collect();
            if let [wall] = walls.as_slice() {
                let columns = self
                    .contours
                    .iter()
                    .filter(|c| {
                        c.kind == Kind::Column && self.is_selected(c.id) && self.contour_visible(c)
                    })
                    .map(|c| c.id)
                    .collect::<Vec<_>>();
                if !columns.is_empty()
                    && ui.add_enabled(!self.is_kind_locked(Kind::RigidLink), egui::Button::new("Соединить АЖТ"))
                        .on_hover_text("Соединить оси выбранных колонн с ближайшими точками выбранной стены на общей высоте")
                        .clicked()
                {
                    if let Err(error) = self.add_bound_rigid_links(*wall, &columns) {
                        self.status = error;
                    }
                    ui.close_menu();
                    return;
                }
            }
            if !walls.is_empty()
                && ui
                    .add_enabled(
                        !self.is_kind_locked(Kind::Restraint),
                        egui::Button::new("Добавить опоры"),
                    )
                    .clicked()
            {
                self.wall_restraint_dialog = Some(WallRestraints {
                    walls,
                    top: false,
                    mask: 4,
                    error: String::new(),
                });
                ui.close_menu();
                return;
            }
        }

        if let Some(id) = self.support_menu_target {
            use punching::SlabSide;
            let label = "Создать области продавливания";
            let help = "Для стены — у свободных торцов, для колонны — замкнутая область вокруг сечения. Затем перестройте сетку.";
            if self.building.enabled {
                ui.add_enabled_ui(!self.input_locked, |ui| {
                    ui.menu_button(label, |ui| {
                        for (label, side) in [
                            ("В плите сверху", SlabSide::Above),
                            ("В плите снизу", SlabSide::Below),
                        ] {
                            if ui.button(label).clicked() {
                                self.generate_punching_contours(Some(id), side);
                                ui.close_menu();
                            }
                        }
                    })
                    .response
                    .on_hover_text(help);
                });
            } else if ui
                .add_enabled(!self.input_locked, egui::Button::new(label))
                .on_hover_text(help)
                .clicked()
            {
                self.generate_punching_contours(Some(id), SlabSide::Above);
                ui.close_menu();
            }
        }
        if let Some((id, edge)) = self.arc_menu_target {
            if ui
                .add_enabled(
                    self.contour_geometry_editable(id),
                    egui::Button::new("Добавить дугу"),
                )
                .clicked()
            {
                self.begin_edge_arc(id, edge);
                ui.close_menu();
            }
            ui.separator();
        }
        if self.arc_placement.is_some() && ui.button("Отменить дугу").clicked() {
            self.cancel();
            ui.close_menu();
        }
        if let Some((id, i, vertex)) = self.menu_target {
            if ui
                .add_enabled(
                    self.contour_geometry_editable(id)
                        && self
                            .contours
                            .iter()
                            .find(|c| c.id == id)
                            .is_some_and(|c| c.can_insert(i)),
                    egui::Button::new("Добавить вершину"),
                )
                .clicked()
            {
                self.change_vertex(id, i, true);
                ui.close_menu();
            }
            let can_remove = self.contour_geometry_editable(id)
                && vertex
                && self
                    .contours
                    .iter()
                    .find(|c| c.id == id)
                    .is_some_and(|c| c.can_remove(i));
            if ui
                .add_enabled(can_remove, egui::Button::new("Удалить вершину"))
                .clicked()
            {
                self.change_vertex(id, i, false);
                ui.close_menu();
            }
            ui.separator();
        }
        if ui.button("Вписать чертёж").clicked() {
            if self.spatial.enabled {
                self.spatial.fit_view();
            } else {
                self.fit = true;
            }
            ui.close_menu();
        }
        if ui
            .add_enabled(
                self.can_finish_sketch(),
                egui::Button::new(
                    if matches!(
                        self.mode,
                        Some(
                            Kind::LoadLine
                                | Kind::Support
                                | Kind::Auxiliary
                                | Kind::Beam
                                | Kind::Triangulation
                        )
                    ) {
                        "Завершить линию"
                    } else {
                        "Замкнуть границу"
                    },
                ),
            )
            .clicked()
        {
            self.finish();
            ui.close_menu();
        }
        if ui.button("Отменить обводку").clicked() {
            self.cancel();
            ui.close_menu();
        }
    }
}
