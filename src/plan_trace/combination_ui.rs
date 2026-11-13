use super::*;
use crate::plan_trace::{
    fem::CaseResult,
    load_combinations::{self, Combination, Variant},
};
use std::cell::OnceCell;
pub(super) struct Cache {
    key: (bool, u64, Vec<(u64, f64)>),
    full: OnceCell<Result<CaseResult, String>>,
    area: OnceCell<Result<CaseResult, String>>,
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        fem::Results,
        loads::{ForceUnits, LoadCase},
    };
    fn result(id: u64, value: f64) -> CaseResult {
        CaseResult {
            punching: Default::default(),
            shell: None,
            id,
            w_mm: vec![value],
            rotations_rad: vec![[0.; 3]; 0],
            moments: vec![[value; 3]],
            shears: vec![[value; 2]],
            reaction_kn: vec![-value],
            loads_kn_m2: vec![],
            foundation_pressure_kn_m2: vec![],
            applied_kn: value,
            balance_kn: 0.,
            balance_m_knm: [0.; 2],
            relative_residual: 0.,
        }
    }
    #[test]
    fn combination_names_and_deletion_keep_underlying_case_results() {
        let mut app = App::default();
        app.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![result(1, 10.)],
            free_dofs: 1,
        });
        app.active_load = 1;
        app.active_combination = app.combinations[0].id;
        app.result_by_combinations = true;
        app.sync_combination_result();
        app.combinations[0].name = "Основное".into();
        assert!(app.result_caption().contains("Сочетание: Основное"));
        assert_eq!(app.selected_result().unwrap().w_mm[0], 10.);
        let saved = serde_json::to_value(&app.combinations[0]).unwrap();
        let restored: Combination = serde_json::from_value(saved.clone()).unwrap();
        assert_eq!(restored.display_name(), "Основное");
        let mut old = saved;
        old.as_object_mut().unwrap().remove("name");
        let restored: Combination = serde_json::from_value(old).unwrap();
        assert_eq!(restored.display_name(), restored.id.to_string());
        app.combinations.push(Combination::new(2, [1]));
        app.delete_combination(1);
        assert_eq!(app.active_combination, 2);
        app.delete_combination(2);
        assert!(!app.result_by_combinations);
        assert!(app.combinations.is_empty());
        assert_eq!(app.analysis.as_ref().unwrap().cases[0].w_mm, vec![10.]);
    }
    #[test]
    fn single_and_combination_views_scale_loads_and_results_without_mutating_inputs() {
        let mut app = App::default();
        app.load_units = ForceUnits::KiloNewton;
        app.new_load.units = ForceUnits::KiloNewton;
        app.new_load.value = 10.;
        app.load_cases[0].gamma_f = 2.;
        app.load_cases[0].gamma_l = 0.3;
        app.begin(Kind::LoadPoint);
        app.sketch_click([100., 100.], false);
        app.load_cases.push(LoadCase::new(2));
        app.load_cases[1].gamma_f = 1.25;
        app.load_cases[1].gamma_l = 0.5;
        app.active_load = 2;
        app.new_load.value = 20.;
        app.begin(Kind::LoadPoint);
        app.sketch_click([200., 100.], false);
        app.active_load = 1;
        let original = app.contours.clone();
        app.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![result(1, 10.), result(2, 20.)],
            free_dofs: 1,
        });
        for (variant, value) in [
            (Variant::Design, 10.),
            (Variant::DesignLong, 3.),
            (Variant::Normative, 5.),
            (Variant::NormativeLong, 1.5),
        ] {
            app.combination_variant = variant;
            app.sync_combination_result();
            assert!((app.displayed_load_value(&app.contours[0]).unwrap() - value).abs() < 1e-12);
            assert!((app.selected_result().unwrap().w_mm[0] - value).abs() < 1e-12);
            assert!(app.contour_visible(&app.contours[0]));
            assert!(!app.contour_visible(&app.contours[1]));
        }
        app.result_by_combinations = true;
        app.combinations[0].coefficients.insert(1, 2.);
        app.combinations[0].coefficients.insert(2, -1.);
        app.sync_combination_result();
        assert!(app.contour_visible(&app.contours[0]) && app.contour_visible(&app.contours[1]));
        assert!((app.displayed_load_value(&app.contours[0]).unwrap() - 3.).abs() < 1e-12);
        assert!((app.displayed_load_value(&app.contours[1]).unwrap() + 8.).abs() < 1e-12);
        assert!((app.selected_result().unwrap().w_mm[0] + 5.).abs() < 1e-12);
        assert_eq!(app.contours, original);
        assert_eq!(app.analysis.as_ref().unwrap().cases[0].w_mm, vec![10.]);
        app.combinations[0].coefficients.insert(2, 0.);
        assert!(!app.contour_visible(&app.contours[1]));
        app.show_load_values = false;
        assert!(!app.load_values_visible());
        assert!(app.contour_visible(&app.contours[0]));
        app.show_load_values = true;
        assert!(app.load_values_visible());
        app.set_load_visibility(false);
        assert!(!app.load_values_visible());
        assert!(!app.contour_visible(&app.contours[0]));
        assert_eq!(app.contours, original);
    }

    #[test]
    fn distributed_load_view_combines_pressures_without_eager_structural_results() {
        let mut app = App::default();
        app.load_cases.push(LoadCase::new(2));
        let mut first = result(1, 10.);
        first.loads_kn_m2 = vec![2., 3.];
        let mut second = result(2, 20.);
        second.loads_kn_m2 = vec![1., 3.];
        app.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![first, second],
            free_dofs: 1,
        });
        app.result_by_combinations = true;
        app.result_field = 7;
        app.show_results = true;
        app.combinations[0].coefficients.insert(2, -0.5);
        app.sync_combination_result();
        assert_eq!(app.selected_result().unwrap().loads_kn_m2, [1.5, 1.5]);
        assert_eq!(app.result_metadata().unwrap().w_mm, [10.]);
        assert!(app.combination_cache.as_ref().unwrap().full.get().is_none());
        assert_eq!(app.selected_full_result().unwrap().w_mm, [0.]);

        app.combinations[0].coefficients.insert(2, 2.);
        app.sync_combination_result();
        assert_eq!(app.selected_result().unwrap().loads_kn_m2, [4., 9.]);
        assert!(app.combination_cache.as_ref().unwrap().full.get().is_none());
        app.result_field = 0;
        assert_eq!(app.selected_result().unwrap().w_mm, [50.]);
    }
}
impl App {
    fn active_combination_name(&self) -> String {
        self.combinations
            .iter()
            .find(|c| c.id == self.active_combination)
            .map_or_else(|| self.active_combination.to_string(), |c| c.display_name())
    }
    fn delete_combination(&mut self, id: u64) {
        self.combinations.retain(|c| c.id != id);
        if self.active_combination == id {
            self.active_combination = self.combinations.first().map_or(0, |c| c.id);
        }
        if self.combinations.is_empty() {
            self.result_by_combinations = false;
        }
        self.combination_cache = None;
        self.sync_combination_result();
    }
    pub(super) fn combination_controls(&mut self, ui: &mut egui::Ui) {
        // Stable load-case IDs prevent rename/reorder from changing a coefficient.
        for combo in &mut self.combinations {
            for case in &self.load_cases {
                combo.coefficients.entry(case.id).or_insert(1.);
            }
        }
        let mut remove = None;
        let mut coefficients_changed = false;
        egui::CollapsingHeader::new("Сочетания нагрузок")
            .default_open(false)
            .show(ui, |ui| {
                if ui.button("Добавить сочетание").clicked() {
                    self.combinations.push(Combination::new(
                        self.next_combination,
                        self.load_cases.iter().map(|c| c.id),
                    ));
                    self.next_combination += 1;
                }
                egui::ScrollArea::both()
                    .id_salt("combination_scroll")
                    .max_height(300.)
                    .auto_shrink([false, true])
                    .show(ui, |ui| {
                        let min_case_width = 115.;
                        let min_combination_width = 78.;
                        let count = self.combinations.len() as f32;
                        let cell_width = ui.available_width() - ui.spacing().item_spacing.x * count;
                        let case_width = if count == 0. {
                            cell_width.max(min_case_width)
                        } else {
                            (cell_width * 0.4)
                                .min(cell_width - min_combination_width * count)
                                .max(min_case_width)
                        };
                        let combination_width =
                            ((cell_width - case_width) / count.max(1.)).max(min_combination_width);
                        let row_height = ui.spacing().interact_size.y;
                        egui::Grid::new("combination_table")
                            .striped(true)
                            .show(ui, |ui| {
                                ui.allocate_space(egui::vec2(case_width, row_height));
                                for combo in &self.combinations {
                                    ui.push_id(("remove_combination", combo.id), |ui| {
                                        if ui
                                            .small_button("−")
                                            .on_hover_text("Удалить сочетание")
                                            .clicked()
                                        {
                                            remove = Some(combo.id);
                                        }
                                    });
                                }
                                ui.end_row();
                                ui.add_sized(
                                    [case_width, row_height],
                                    egui::Label::new("Загружение").halign(egui::Align::LEFT),
                                );
                                for combo in &mut self.combinations {
                                    let id = ui.id().with(("combination_name", combo.id));
                                    let mut name = if combo.name.is_empty()
                                        && !ui.memory(|m| m.has_focus(id))
                                    {
                                        combo.id.to_string()
                                    } else {
                                        combo.name.clone()
                                    };
                                    if ui
                                        .add_sized(
                                            [combination_width, row_height],
                                            egui::TextEdit::singleline(&mut name)
                                                .id(id)
                                                .char_limit(100)
                                                .desired_width(combination_width),
                                        )
                                        .changed()
                                    {
                                        combo.name = name;
                                    }
                                }
                                ui.end_row();
                                for case in &self.load_cases {
                                    ui.add_sized(
                                        [case_width, row_height],
                                        egui::Label::new(case.display_name())
                                            .halign(egui::Align::LEFT)
                                            .truncate(),
                                    );
                                    for combo in &mut self.combinations {
                                        ui.push_id((combo.id, case.id), |ui| {
                                            coefficients_changed |= ui
                                                .add_sized(
                                                    [combination_width, row_height],
                                                    egui::DragValue::new(
                                                        combo
                                                            .coefficients
                                                            .entry(case.id)
                                                            .or_insert(1.),
                                                    )
                                                    .speed(0.05),
                                                )
                                                .changed();
                                        });
                                    }
                                    ui.end_row();
                                }
                            });
                    });
            });
        if let Some(id) = remove {
            self.delete_combination(id);
        }
        self.sync_combination_result();
        if (coefficients_changed || remove.is_some())
            && self.area_combination_view()
            && self.analysis.is_none()
        {
            self.sync_source_view();
        }
    }
    pub(super) fn result_context_controls(&mut self, ui: &mut egui::Ui) {
        self.stiffness_variant_controls(ui);
        let previous_load = self.active_load;
        let previous_variant = self.combination_variant;
        let caption = if self.result_by_combinations {
            format!("Сочетание: {}", self.active_combination_name())
        } else {
            self.active_load_name()
        };
        bottom_selector_wrap(ui, 153.);
        egui::ComboBox::from_id_salt("result_load_or_combination")
            .width(133.)
            .truncate()
            .selected_text(caption)
            .show_ui(ui, |ui| {
                ui.strong("Загружения");
                for case in &self.load_cases {
                    if ui
                        .selectable_label(
                            !self.result_by_combinations && self.active_load == case.id,
                            case.display_name(),
                        )
                        .clicked()
                    {
                        self.active_load = case.id;
                        self.result_by_combinations = false;
                    }
                }
                if !self.combinations.is_empty() {
                    ui.separator();
                    ui.strong("Сочетания");
                    for combination in &self.combinations {
                        if ui
                            .selectable_label(
                                self.result_by_combinations
                                    && self.active_combination == combination.id,
                                format!("Сочетание: {}", combination.display_name()),
                            )
                            .clicked()
                        {
                            self.active_combination = combination.id;
                            self.result_by_combinations = true;
                        }
                    }
                }
            });
        bottom_selector_wrap(ui, 150.);
        egui::ComboBox::from_id_salt("result_combo_variant")
            .width(129.5)
            .truncate()
            .selected_text(self.combination_variant.label())
            .show_ui(ui, |ui| {
                for variant in Variant::ALL {
                    ui.selectable_value(&mut self.combination_variant, variant, variant.label());
                }
            });
        if previous_variant != self.combination_variant {
            self.variant_scale_pending = [true; crate::plan_trace::fem::FIELD_COUNT];
        }
        if previous_load != self.active_load && self.mode.is_some_and(|k| k.is_load()) {
            self.cancel();
        }
        self.sync_combination_result();
        if !self.combination_error.is_empty() {
            ui.add(egui::Label::new(&self.combination_error).wrap());
        }
    }
    fn view_weights(&self) -> Result<Vec<(u64, f64)>, String> {
        if self.result_by_combinations {
            self.combinations
                .iter()
                .find(|c| c.id == self.active_combination)
                .ok_or("Не найдено сочетание")?
                .weights(&self.load_cases, self.combination_variant)
        } else {
            let case = self
                .load_cases
                .iter()
                .find(|c| c.id == self.active_load)
                .ok_or("Не найдено загружение")?;
            Ok(vec![(case.id, self.combination_variant.weight(case, 1.)?)])
        }
    }
    pub(super) fn displayed_load_factor(&self, case_id: u64) -> Option<f64> {
        let case = self.load_cases.iter().find(|c| c.id == case_id)?;
        let coefficient = if self.result_by_combinations {
            *self
                .combinations
                .iter()
                .find(|c| c.id == self.active_combination)?
                .coefficients
                .get(&case_id)
                .unwrap_or(&1.)
        } else if case_id == self.active_load {
            1.
        } else {
            return None;
        };
        let factor = self.combination_variant.weight(case, coefficient).ok()?;
        (factor != 0.).then_some(factor)
    }
    pub(super) fn displayed_load_value(&self, c: &Contour) -> Option<f64> {
        let data = c.load.as_ref()?;
        let v = data.magnitude.in_units(self.load_units, self.gravity)
            * self.displayed_load_factor(data.case_id)?;
        v.is_finite().then_some(v)
    }
    pub(super) fn load_values_visible(&self) -> bool {
        self.show_loads && self.show_load_values
    }
    pub(super) fn sync_combination_result(&mut self) {
        let weights = match self.view_weights() {
            Ok(w) => w,
            Err(e) => {
                self.combination_error = e;
                self.combination_cache = None;
                return;
            }
        };
        self.combination_error.clear();
        let Some(results) = &self.analysis else {
            self.combination_cache = None;
            return;
        };
        if !self.result_by_combinations && self.combination_variant == Variant::Design {
            self.combination_cache = None;
            return;
        }
        let id = if self.result_by_combinations {
            self.active_combination
        } else {
            self.active_load
        };
        let key = (self.result_by_combinations, id, weights);
        let area_view = self.area_combination_view();
        if self.combination_cache.as_ref().is_none_or(|c| c.key != key) {
            self.result_generation = self.result_generation.wrapping_add(1);
            self.combination_cache = Some(Cache {
                key,
                full: OnceCell::new(),
                area: OnceCell::new(),
            });
        }
        let cache = self.combination_cache.as_ref().unwrap();
        let result = if area_view {
            match cache.full.get() {
                Some(result @ Ok(_)) => result,
                _ => cache.area.get_or_init(|| {
                    load_combinations::combine_area_loads(results, id, &cache.key.2)
                }),
            }
        } else {
            cache
                .full
                .get_or_init(|| load_combinations::combine(results, id, &cache.key.2))
        };
        if let Err(error) = result {
            self.combination_error.clone_from(error);
        }
    }
    fn area_combination_view(&self) -> bool {
        self.show_results && self.result_field == 7 && self.punching_view.is_none()
    }
    pub(super) fn selected_result(&self) -> Option<&CaseResult> {
        if self.area_combination_view()
            && self.analysis.is_some()
            && (self.result_by_combinations || self.combination_variant != Variant::Design)
        {
            let cache = self.combination_cache.as_ref()?;
            if let Some(Ok(result)) = cache.full.get() {
                return Some(result);
            }
            return cache.area.get()?.as_ref().ok();
        }
        self.selected_full_result()
    }
    pub(super) fn result_metadata(&self) -> Option<&CaseResult> {
        let result = self.selected_result()?;
        if self.area_combination_view() {
            self.analysis
                .as_ref()
                .and_then(|analysis| analysis.cases.first())
                .or(Some(result))
        } else {
            Some(result)
        }
    }
    pub(super) fn selected_full_result(&self) -> Option<&CaseResult> {
        if self.analysis.is_none() || self.result_field == 33 {
            return self.source_view_result();
        }
        let analysis = self.analysis.as_ref()?;
        if self.result_by_combinations || self.combination_variant != Variant::Design {
            let cache = self.combination_cache.as_ref()?;
            cache
                .full
                .get_or_init(|| load_combinations::combine(analysis, cache.key.1, &cache.key.2))
                .as_ref()
                .ok()
        } else {
            analysis.cases.iter().find(|c| c.id == self.active_load)
        }
    }
    pub(super) fn result_caption(&self) -> String {
        if let Some(mode) = self.schema_analysis.mode {
            return format!("Анализ схемы · {}", mode.label());
        }

        let mut text = self.base_result_caption();
        if let Some(base) = self
            .analysis
            .as_ref()
            .and_then(|r| r.nonlinear.as_ref())
            .filter(|_| self.show_results && self.selected_result().is_some())
        {
            let exact_base = self.view_weights().is_ok_and(|weights| {
                let normalize = |values: &[(u64, f64)]| {
                    values
                        .iter()
                        .copied()
                        .filter(|(_, w)| *w != 0.)
                        .collect::<BTreeMap<_, _>>()
                };
                normalize(&weights) == normalize(&base.weights)
            });
            text = if exact_base {
                format!("{text} · нелинейное решение")
            } else {
                format!("{text} · касательная к базе {}", base.caption)
            };
        }
        if self.stiffnesses.alternative_enabled {
            format!("Жёсткости {} · {text}", self.stiffness_variant + 1)
        } else {
            text
        }
    }
    fn base_result_caption(&self) -> String {
        if self.length_view != 0
            && !self.show_results
            && self.analysis.as_ref().is_some_and(|r| r.lengths.is_some())
        {
            return if self.length_view == 1 {
                "Расчётные длины · L₀, м"
            } else {
                "Коэффициенты расчётной длины · μ"
            }
            .into();
        }
        if !self.show_results || self.selected_result().is_none() {
            let context = if self.result_by_combinations {
                format!("Сочетание: {}", self.active_combination_name())
            } else {
                self.active_load_name()
            };
            return format!(
                "{context} · {} · Расчётная схема",
                self.combination_variant.label()
            );
        }
        if let Some(field) = self.punching_view {
            let context = if self.result_by_combinations {
                format!("Сочетание: {}", self.active_combination_name())
            } else {
                self.active_load_name()
            };
            return format!(
                "{context} · {} · Суммирование нагрузок · {} · {} · {}",
                self.combination_variant.label(),
                self.summation_method.label(),
                ["F", "Mx", "My", "F | Mx | My", "Fi"][field],
                if field == 0 || field == 4 {
                    self.load_units.label().to_owned()
                } else if field < 3 {
                    format!("{}·м", self.load_units.label())
                } else {
                    format!(
                        "{} / {}·м",
                        self.load_units.label(),
                        self.load_units.label()
                    )
                }
            );
        }

        if self.result_field == 33 {
            return self.result_field_label(33);
        }
        if (26..=32).contains(&self.result_field) {
            return format!(
                "Все сочетания · {}{}",
                self.result_field_label(self.result_field),
                if (26..=29).contains(&self.result_field) {
                    format!(" · {}", self.reinforcement_area_view.label())
                } else {
                    String::new()
                }
            );
        }

        let caption = if self.result_by_combinations {
            format!(
                "Сочетание: {} · {} · {}",
                self.active_combination_name(),
                self.combination_variant.label(),
                self.result_field_label(self.result_field)
            )
        } else {
            format!(
                "{} · {} · {}",
                self.active_load_name(),
                self.combination_variant.label(),
                self.result_field_label(self.result_field)
            )
        };
        if crate::plan_trace::fem::displacement_field(self.result_field) {
            format!(
                "{caption} · {}",
                if self.iso_displacements {
                    "Изополя"
                } else {
                    "Мозаика"
                }
            )
        } else if self.result_has_element_field() {
            format!(
                "{caption} · {}",
                if self.iso_efforts {
                    "Изополя"
                } else {
                    "Мозаика"
                }
            )
        } else {
            caption
        }
    }
}
