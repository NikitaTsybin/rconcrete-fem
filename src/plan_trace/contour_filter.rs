use super::*;
use std::collections::{BTreeMap, BTreeSet};

#[derive(Clone, Copy, PartialEq)]
enum Comparison {
    Less,
    Greater,
}

impl Comparison {
    fn label(self) -> &'static str {
        match self {
            Self::Less => "Меньше",
            Self::Greater => "Больше",
        }
    }

    fn matches(self, actual: f64, limit: f64) -> bool {
        match self {
            Self::Less => actual < limit,
            Self::Greater => actual > limit,
        }
    }
}

enum ChoiceFilter {
    Single(u64),
    Multiple(BTreeSet<u64>),
}

impl ChoiceFilter {
    fn contains(&self, id: u64) -> bool {
        match self {
            Self::Single(selected) => *selected == id,
            Self::Multiple(selected) => selected.contains(&id),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
enum Criterion {
    Floor,
    Section,
    Material,
    Design,
    RigidOffset,
    LoadMeshConstraint,
}

impl Criterion {
    const ALL: [Self; 6] = [
        Self::Floor,
        Self::Section,
        Self::Material,
        Self::Design,
        Self::RigidOffset,
        Self::LoadMeshConstraint,
    ];

    fn label(self) -> &'static str {
        match self {
            Self::Floor => "Этаж",
            Self::Section => "Сечение",
            Self::Material => "Материал",
            Self::Design => "Конструирование",
            Self::RigidOffset => "Жёсткие вставки",
            Self::LoadMeshConstraint => crate::plan_trace::loads::MESH_CONSTRAINT_LABEL,
        }
    }

    fn applies(self, kind: Kind) -> bool {
        match self {
            Self::Floor => true,
            Self::LoadMeshConstraint => kind.is_load() && kind != Kind::LoadHole,
            _ => matches!(kind, Kind::Slab | Kind::Support),
        }
    }

    fn value(self, app: &App, contour: &Contour) -> u64 {
        let library = &app.stiffnesses;
        let section = || {
            library
                .assignments
                .get(&contour.id)
                .copied()
                .filter(|id| library.entries.get(id).is_some_and(|e| e.family.is_plate()))
        };
        match self {
            Self::Floor => app.building.owner(contour.id),
            Self::Section => section().unwrap_or(0),
            Self::Material => library
                .material_id(contour.id)
                .or_else(|| {
                    library
                        .elastic_assignments
                        .get(&section()?)
                        .copied()
                        .filter(|id| library.elastic.contains_key(id))
                })
                .unwrap_or(0),
            Self::Design => library
                .design_id(contour.id)
                .filter(|id| library.designs.get(id).is_some_and(|d| !d.bar))
                .unwrap_or(0),
            Self::RigidOffset if contour.kind == Kind::Slab => {
                offset_key(contour.placement.slab_rigid_z_mm)
            }
            Self::RigidOffset => 0,
            Self::LoadMeshConstraint => u64::from(
                contour
                    .load
                    .as_ref()
                    .is_some_and(|load| load.mesh_constraint),
            ),
        }
    }
}

fn offset_key(mm: f64) -> u64 {
    if mm == 0. {
        0
    } else {
        mm.to_bits()
    }
}

struct Choice {
    id: u64,
    name: String,
    elevation_mm: Option<f64>,
}

impl Choice {
    fn named(id: u64, name: impl Into<String>) -> Self {
        Self {
            id,
            name: name.into(),
            elevation_mm: None,
        }
    }
}

pub(super) struct State {
    pub open: bool,
    kind: Kind,
    use_area: bool,
    comparison: Comparison,
    value: String,
    choices: BTreeMap<Criterion, ChoiceFilter>,
    draft: Option<(Criterion, BTreeSet<u64>)>,
    matched: Option<usize>,
}

impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            kind: Kind::Hole,
            use_area: true,
            comparison: Comparison::Less,
            value: String::new(),
            choices: BTreeMap::new(),
            draft: None,
            matched: None,
        }
    }
}

impl State {
    fn area_active(&self) -> bool {
        self.kind == Kind::Hole && self.use_area
    }

    fn area_limit(&self) -> Option<f64> {
        self.value
            .trim()
            .replace(',', ".")
            .parse::<f64>()
            .ok()
            .filter(|limit| limit.is_finite() && *limit >= 0.)
    }

    fn area_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Площадь отверстия, м²");
            if ui
                .small_button("×")
                .on_hover_text("Удалить фильтр площади")
                .clicked()
            {
                self.use_area = false;
                self.matched = None;
            }
        });
        ui.horizontal(|ui| {
            let previous = self.comparison;
            egui::ComboBox::from_id_salt("area_comparison")
                .width(68.)
                .selected_text(self.comparison.label())
                .show_ui(ui, |ui| {
                    for comparison in [Comparison::Less, Comparison::Greater] {
                        ui.selectable_value(&mut self.comparison, comparison, comparison.label());
                    }
                });
            let response = ui.add(
                egui::TextEdit::singleline(&mut self.value)
                    .desired_width(64.)
                    .hint_text("м²"),
            );
            if response.changed() || previous != self.comparison {
                self.matched = None;
            }
        });
    }

    fn choice_controls(&mut self, ui: &mut egui::Ui, criterion: Criterion, options: &[Choice]) {
        let Some(filter) = self.choices.get(&criterion) else {
            return;
        };
        let multiple = matches!(filter, ChoiceFilter::Multiple(_));
        let selected = match filter {
            ChoiceFilter::Single(id) => options.iter().find(|option| option.id == *id).map_or_else(
                || "Значение удалено".to_owned(),
                |option| option.name.clone(),
            ),
            ChoiceFilter::Multiple(_) => {
                let count = options
                    .iter()
                    .filter(|option| filter.contains(option.id))
                    .count();
                format!("Несколько ({count})")
            }
        };
        let mut choose_multiple = false;
        let mut single = None;
        let remove = ui
            .horizontal(|ui| {
                ui.label(criterion.label());
                egui::ComboBox::from_id_salt(("contour_filter_choice", criterion))
                    .width(90.)
                    .truncate()
                    .selected_text(selected)
                    .show_ui(ui, |ui| {
                        if criterion != Criterion::LoadMeshConstraint {
                            choose_multiple = ui.selectable_label(multiple, "Несколько…").clicked();
                        }
                        for option in options {
                            let name = option.elevation_mm.map_or_else(
                                || option.name.clone(),
                                |z| format!("{} · {z:.0} мм", option.name),
                            );
                            if ui
                                .selectable_label(!multiple && filter.contains(option.id), name)
                                .clicked()
                            {
                                single = Some(option.id);
                            }
                        }
                    });
                ui.small_button("×")
                    .on_hover_text("Удалить фильтр")
                    .clicked()
            })
            .inner;
        if remove {
            self.choices.remove(&criterion);
            self.matched = None;
        } else if let Some(id) = single {
            self.choices.insert(criterion, ChoiceFilter::Single(id));
            self.matched = None;
        } else if choose_multiple
            || multiple
                && ui
                    .button(if criterion == Criterion::Floor {
                        "Выбрать этажи…"
                    } else {
                        "Выбрать значения…"
                    })
                    .clicked()
        {
            self.draft = Some((
                criterion,
                options
                    .iter()
                    .filter(|option| filter.contains(option.id))
                    .map(|option| option.id)
                    .collect(),
            ));
        }
    }

    fn choices_dialog(&mut self, ctx: &egui::Context, options: &[Choice]) {
        let Some((criterion, selected)) = &mut self.draft else {
            return;
        };
        let criterion = *criterion;
        let mut confirm = false;
        let mut cancel = false;
        let response =
            egui::Modal::new(egui::Id::new(("contour_filter_values", criterion))).show(ctx, |ui| {
                if criterion == Criterion::Floor {
                    ui.heading("Этажи для фильтра");
                } else {
                    ui.heading(criterion.label());
                }
                ui.set_min_width(220.);
                egui::ScrollArea::vertical()
                    .max_height(260.)
                    .show(ui, |ui| {
                        egui::Grid::new(("contour_filter_choice_table", criterion))
                            .striped(true)
                            .show(ui, |ui| {
                                let count = options
                                    .iter()
                                    .filter(|option| selected.contains(&option.id))
                                    .count();
                                let mut all_checked = !options.is_empty() && count == options.len();
                                let mixed = count > 0 && !all_checked;
                                if ui
                                    .add_enabled(
                                        !options.is_empty(),
                                        egui::Checkbox::new(&mut all_checked, "")
                                            .indeterminate(mixed),
                                    )
                                    .on_hover_text("Отметить / снять все")
                                    .changed()
                                {
                                    selected.clear();
                                    if all_checked {
                                        selected.extend(options.iter().map(|option| option.id));
                                    }
                                }
                                ui.strong(criterion.label());
                                if criterion == Criterion::Floor {
                                    ui.strong("Z, мм");
                                }
                                ui.end_row();
                                for option in options {
                                    let mut checked = selected.contains(&option.id);
                                    if ui
                                        .push_id(option.id, |ui| ui.checkbox(&mut checked, ""))
                                        .inner
                                        .changed()
                                    {
                                        if checked {
                                            selected.insert(option.id);
                                        } else {
                                            selected.remove(&option.id);
                                        }
                                    }
                                    ui.label(&option.name);
                                    if let Some(z) = option.elevation_mm {
                                        ui.label(format!("{z:.0}"));
                                    }
                                    ui.end_row();
                                }
                            });
                    });
                ui.horizontal(|ui| {
                    confirm = ui
                        .add_enabled(
                            options.iter().any(|option| selected.contains(&option.id)),
                            egui::Button::new("Готово"),
                        )
                        .clicked();
                    cancel = ui.button("Отмена").clicked();
                });
            });
        if confirm {
            let (_, ids) = self.draft.take().unwrap();
            self.choices.insert(criterion, ChoiceFilter::Multiple(ids));
            self.matched = None;
        } else if cancel || response.should_close() {
            self.draft = None;
        }
    }
}

pub(super) fn window_button(ui: &mut egui::Ui, open: bool) -> egui::Response {
    let (rect, response) = ui.allocate_exact_size(egui::vec2(30., 24.), egui::Sense::click());
    let style = ui.style().interact_selectable(&response, open);
    let center = rect.center();
    let funnel = [
        [-8., -6.],
        [8., -6.],
        [2., 0.],
        [2., 5.],
        [-2., 7.],
        [-2., 0.],
    ]
    .map(|[x, y]| center + egui::vec2(x, y));
    ui.painter().rect_filled(rect, 3., style.bg_fill);
    ui.painter().add(egui::Shape::closed_line(
        funnel.to_vec(),
        Stroke::new(1.5_f32, style.fg_stroke.color),
    ));
    response.widget_info(|| {
        egui::WidgetInfo::labeled(egui::WidgetType::Button, ui.is_enabled(), "Фильтр объектов")
    });
    response.on_hover_text("Фильтр объектов")
}

impl App {
    fn contour_filter_choices(&self, kind: Kind, criterion: Criterion) -> Vec<Choice> {
        let library = &self.stiffnesses;
        match criterion {
            Criterion::LoadMeshConstraint => vec![Choice::named(1, "Да"), Choice::named(0, "Нет")],
            Criterion::Floor => self
                .building
                .floors
                .iter()
                .map(|f| Choice {
                    id: f.id,
                    name: f.name.clone(),
                    elevation_mm: Some(f.elevation_mm),
                })
                .collect(),
            Criterion::Section => std::iter::once(Choice::named(0, "Без сечения"))
                .chain(
                    library
                        .entries
                        .iter()
                        .filter(|(_, entry)| entry.family.is_plate())
                        .map(|(&id, entry)| Choice::named(id, format!("{} · №{id}", entry.name))),
                )
                .collect(),
            Criterion::Material => {
                std::iter::once(Choice::named(0, "Без материала"))
                    .chain(library.elastic.iter().map(|(&id, material)| {
                        Choice::named(id, format!("{} · №{id}", material.name))
                    }))
                    .collect()
            }
            Criterion::Design => std::iter::once(Choice::named(0, "Без конструирования"))
                .chain(
                    library
                        .designs
                        .iter()
                        .filter(|(_, design)| !design.bar)
                        .map(|(&id, design)| Choice::named(id, format!("{} · №{id}", design.name))),
                )
                .collect(),
            Criterion::RigidOffset => {
                let mut options = vec![Choice::named(0, "Без вставки (0 мм)")];
                if kind == Kind::Slab {
                    let mut offsets = self
                        .contours
                        .iter()
                        .filter(|c| c.kind == Kind::Slab)
                        .map(|c| c.placement.slab_rigid_z_mm)
                        .filter(|mm| mm.is_finite() && *mm != 0.)
                        .collect::<Vec<_>>();
                    offsets.sort_unstable_by(f64::total_cmp);
                    offsets.dedup();
                    options.extend(
                        offsets
                            .into_iter()
                            .map(|mm| Choice::named(offset_key(mm), format!("{mm} мм по Z"))),
                    );
                }
                options
            }
        }
    }

    fn filtered_contour_ids(
        &self,
        kind: Kind,
        area: Option<(Comparison, f64)>,
        choices: &BTreeMap<Criterion, ChoiceFilter>,
    ) -> Vec<u64> {
        self.contours
            .iter()
            .filter(|c| {
                c.kind == kind
                    && choices.iter().all(|(&criterion, filter)| {
                        !criterion.applies(kind) || filter.contains(criterion.value(self, c))
                    })
                    && self.selectable(c)
            })
            .filter(|c| {
                area.is_none_or(|(comparison, limit)| {
                    let area_m2 = geometry::area(&c.points) / 1e6;
                    c.points.len() >= 3 && area_m2.is_finite() && comparison.matches(area_m2, limit)
                })
            })
            .map(|c| c.id)
            .collect()
    }

    pub(super) fn contour_filter_window(&mut self, ctx: &egui::Context) {
        if !self.contour_filter.open {
            self.contour_filter.draft = None;
            return;
        }

        let mut open = self.contour_filter.open;
        let mut apply = None;
        let mut filter = std::mem::take(&mut self.contour_filter);
        egui::Window::new("Фильтр объектов")
            .open(&mut open)
            .collapsible(false)
            .default_size(egui::vec2(385., 280.))
            .min_size(egui::vec2(320., 174.))
            .resizable(true)
            .show(ctx, |ui| {
                let types_width = (0.43 * ui.available_width()).clamp(128., 260.);
                let types_height = (ui.available_height() - ui.spacing().interact_size.y).max(100.);
                ui.horizontal_top(|ui| {
                    ui.vertical(|ui| {
                        ui.set_width(types_width);
                        ui.strong("Тип объекта");
                        egui::ScrollArea::vertical()
                            .max_height(types_height)
                            .show(ui, |ui| {
                                for kind in selection::KINDS
                                    .into_iter()
                                    .chain([Kind::Column, Kind::Foundation, Kind::PointRestraint])
                                {
                                    if kind == Kind::RigidLink && !self.building.enabled {
                                        continue;
                                    }
                                    let button = egui::Button::new(kind.label())
                                        .frame(false)
                                        .selected(filter.kind == kind)
                                        .wrap();
                                    if ui.add(button).clicked() && filter.kind != kind {
                                        filter.kind = kind;
                                        filter.choices.remove(&Criterion::RigidOffset);
                                        filter.draft = None;
                                        filter.matched = None;
                                    }
                                }
                            });
                    });
                    ui.separator();
                    ui.vertical(|ui| {
                        ui.strong("Параметр и значение");
                        if filter.area_active() {
                            filter.area_controls(ui);
                        }
                        let mut invalid_choice = None;
                        let mut any_choice = false;
                        for criterion in Criterion::ALL {
                            if !criterion.applies(filter.kind) || !filter.choices.contains_key(&criterion) {
                                continue;
                            }
                            let options = self.contour_filter_choices(filter.kind, criterion);
                            filter.choice_controls(ui, criterion, &options);
                            if let Some(choice) = filter.choices.get(&criterion) {
                                any_choice = true;
                                if !options.iter().any(|option| choice.contains(option.id)) {
                                    invalid_choice = Some(criterion);
                                }
                            }
                        }
                        if !filter.area_active() && !any_choice {
                            ui.label("По типу объекта");
                        }
                        ui.menu_button("Добавить фильтр", |ui| {
                            for criterion in Criterion::ALL {
                                if !criterion.applies(filter.kind) {
                                    continue;
                                }
                                if ui.add_enabled(!filter.choices.contains_key(&criterion), egui::Button::new(criterion.label())).clicked() {
                                    let value = match criterion {
                                        Criterion::Floor => self.building.active,
                                        Criterion::LoadMeshConstraint => 1,
                                        _ => 0,
                                    };
                                    filter.choices.insert(criterion, ChoiceFilter::Single(value));
                                    filter.matched = None;
                                    ui.close_menu();
                                }
                            }
                            if filter.kind == Kind::Hole && ui.add_enabled(!filter.use_area, egui::Button::new("Площадь отверстия")).clicked() {
                                filter.use_area = true;
                                filter.matched = None;
                                ui.close_menu();
                            }
                        });
                        let area = filter.area_active()
                            .then(|| filter.area_limit())
                            .flatten()
                            .map(|limit| (filter.comparison, limit));
                        let invalid = if filter.area_active() && area.is_none() {
                            Some("Введите неотрицательную площадь в м²".into())
                        } else {
                            invalid_choice.map(|criterion| format!("Выберите значения фильтра «{}»", criterion.label()))
                        };
                        let response = ui.add_enabled(invalid.is_none(), egui::Button::new("Выделить"));
                        if response
                            .on_hover_text("Учитываются видимость объектов и текущая область выделения по этажам")
                            .on_disabled_hover_text(invalid.unwrap_or_default())
                            .clicked()
                        {
                            apply = Some((filter.kind, area));
                        }
                        if let Some(count) = filter.matched {
                            ui.label(format!("Найдено: {count}"));
                        }
                    });
                });
            });
        filter.open = open;
        if open {
            if let Some((criterion, _)) = &filter.draft {
                let options = self.contour_filter_choices(filter.kind, *criterion);
                filter.choices_dialog(ctx, &options);
            }
        } else {
            filter.draft = None;
        }
        if let Some((kind, area)) = apply {
            let ids = self.filtered_contour_ids(kind, area, &filter.choices);
            filter.matched = Some(ids.len());
            self.status = format!("Выделено объектов: {}", ids.len());
            if !ids.is_empty() {
                self.select_ids(ids, egui::Modifiers::NONE);
            }
        }
        self.contour_filter = filter;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rectangle(id: u64, kind: Kind, width: f64) -> Contour {
        Contour {
            id,
            kind,
            points: vec![[0., 0.], [width, 0.], [width, 1000.], [0., 1000.]],
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
            parent: None,
            curves: vec![],
        }
    }

    #[test]
    fn load_mesh_constraint_filter_combines_with_the_floor_and_visibility() {
        use crate::plan_trace::loads::{ForceUnits, LoadData, Magnitude};

        let mut app = App::default();
        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.spatial.enabled = true;
        app.selection_all_floors = true;
        let criterion = Criterion::LoadMeshConstraint;
        for kind in [Kind::LoadPoint, Kind::LoadLine, Kind::LoadArea] {
            assert!(criterion.applies(kind));
            app.contours = (1..=4)
                .map(|id| {
                    let mut contour = rectangle(id, kind, 1000.);
                    if kind != Kind::LoadArea {
                        contour
                            .points
                            .truncate(if kind == Kind::LoadPoint { 1 } else { 2 });
                    }
                    contour.load = Some(LoadData {
                        direction: Default::default(),
                        moment: false,
                        additive: false,
                        name: String::new(),
                        mesh_constraint: id != 2,
                        case_id: 1,
                        magnitude: Magnitude {
                            value: 5.,
                            units: ForceUnits::TonForce,
                        },
                        closed: kind == Kind::LoadArea,
                    });
                    contour
                })
                .collect();
            app.building.contour_floors.insert(3, 2);
            app.view_visibility.contours.insert(4);
            let mut filters = BTreeMap::from([(criterion, ChoiceFilter::Single(1))]);
            assert_eq!(app.filtered_contour_ids(kind, None, &filters), vec![1, 3]);
            filters.insert(Criterion::Floor, ChoiceFilter::Single(2));
            assert_eq!(app.filtered_contour_ids(kind, None, &filters), vec![3]);
            filters.remove(&Criterion::Floor);
            filters.insert(criterion, ChoiceFilter::Single(0));
            assert_eq!(app.filtered_contour_ids(kind, None, &filters), vec![2]);
        }
        assert!(!criterion.applies(Kind::LoadHole));
        assert!(!criterion.applies(Kind::Slab));
    }

    #[test]
    fn hole_area_uses_strict_comparison_and_current_selection_scope() {
        let mut app = App::default();
        app.contours = vec![
            rectangle(1, Kind::Hole, 500.),
            rectangle(2, Kind::Hole, 1250.),
            rectangle(3, Kind::Hole, 2000.),
            rectangle(4, Kind::Hole, 2500.),
            rectangle(5, Kind::Hole, 3000.),
            rectangle(6, Kind::Slab, 4000.),
        ];
        app.contours[2].points.reverse();
        app.view_visibility.contours.insert(4);
        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors.insert(5, 2);
        app.contour_filter.value = "1,25".into();
        let limit = app.contour_filter.area_limit().unwrap();
        let empty = BTreeMap::new();
        let holes = |app: &App, comparison| {
            app.filtered_contour_ids(Kind::Hole, Some((comparison, limit)), &empty)
        };
        assert_eq!(holes(&app, Comparison::Less), vec![1]);
        assert_eq!(holes(&app, Comparison::Greater), vec![3]);
        assert_eq!(app.filtered_contour_ids(Kind::Slab, None, &empty), vec![6]);

        app.spatial.enabled = true;
        assert_eq!(holes(&app, Comparison::Greater), vec![3]);
        let single = BTreeMap::from([(Criterion::Floor, ChoiceFilter::Single(2))]);
        assert!(app
            .filtered_contour_ids(Kind::Hole, None, &single)
            .is_empty());
        app.selection_all_floors = true;
        assert_eq!(holes(&app, Comparison::Greater), vec![3, 5]);
        assert_eq!(app.filtered_contour_ids(Kind::Hole, None, &single), vec![5]);
        let multiple = BTreeMap::from([(
            Criterion::Floor,
            ChoiceFilter::Multiple(BTreeSet::from([1, 2])),
        )]);
        let area = Some((Comparison::Greater, limit));
        assert_eq!(
            app.filtered_contour_ids(Kind::Hole, area, &multiple),
            vec![3, 5]
        );
        assert_eq!(
            app.filtered_contour_ids(Kind::Slab, None, &multiple),
            vec![6]
        );
        app.building.hidden_floors.insert(2);
        assert_eq!(holes(&app, Comparison::Greater), vec![3]);
        assert!(app
            .filtered_contour_ids(Kind::Hole, None, &single)
            .is_empty());
        app.selection_filter[1] = false;
        assert!(holes(&app, Comparison::Greater).is_empty());
    }

    #[test]
    fn slab_filters_intersect_and_follow_current_assignments() {
        use crate::plan_trace::loads::{ForceUnits, Magnitude};
        use crate::plan_trace::stiffness::{Design, ElasticMaterial, NamedMaterial};
        let mut app = App::default();
        app.contours = (1..=5).map(|id| rectangle(id, Kind::Slab, 1000.)).collect();
        app.contours[0].placement.slab_rigid_z_mm = 100.;
        app.contours[1].placement.slab_rigid_z_mm = -125.;
        app.contours[2].placement.slab_rigid_z_mm = 100.;
        app.contours[3].placement.slab_rigid_z_mm = -0.;
        app.contours[4].placement.slab_rigid_z_mm = 100.;
        app.view_visibility.contours.insert(5);
        app.stiffnesses.assignments = BTreeMap::from([(1, 1), (2, 2), (3, 1), (4, 2), (5, 2)]);
        let elastic = ElasticMaterial {
            e_mpa: 30000.,
            nu: 0.2,
            rho: Magnitude {
                value: 2.5,
                units: ForceUnits::TonForce,
            },
        };
        for id in [10, 11] {
            app.stiffnesses.elastic.insert(
                id,
                NamedMaterial {
                    name: "Бетон".into(),
                    primary: elastic,
                    alternative: elastic,
                    concrete: None,
                },
            );
        }
        app.stiffnesses.elastic_assignments.insert(1, 10);
        app.stiffnesses.contour_materials = BTreeMap::from([(2, 10), (3, 11), (5, 10)]);
        for id in [20, 21] {
            app.stiffnesses.designs.insert(
                id,
                Design {
                    name: "Плита".into(),
                    bar: false,
                    settings: Default::default(),
                },
            );
        }
        app.stiffnesses.design_assignments.insert(1, 20);
        app.stiffnesses.contour_designs = BTreeMap::from([(2, 20), (3, 21), (5, 20)]);
        let choices = BTreeMap::from([
            (
                Criterion::Section,
                ChoiceFilter::Multiple(BTreeSet::from([1, 2])),
            ),
            (Criterion::Material, ChoiceFilter::Single(10)),
            (Criterion::Design, ChoiceFilter::Single(20)),
            (
                Criterion::RigidOffset,
                ChoiceFilter::Multiple(BTreeSet::from([offset_key(100.), offset_key(-125.)])),
            ),
        ]);
        assert_eq!(app.filtered_contour_ids(Kind::Slab, None, &choices), [1, 2]);

        let absent = BTreeMap::from([
            (Criterion::Material, ChoiceFilter::Single(0)),
            (Criterion::Design, ChoiceFilter::Single(0)),
            (Criterion::RigidOffset, ChoiceFilter::Single(0)),
        ]);
        assert_eq!(app.filtered_contour_ids(Kind::Slab, None, &absent), [4]);
        assert_eq!(
            app.contour_filter_choices(Kind::Slab, Criterion::RigidOffset)
                .iter()
                .map(|option| option.id)
                .collect::<Vec<_>>(),
            [0, offset_key(-125.), offset_key(100.)]
        );
    }

    #[test]
    fn wall_has_no_insert_and_other_contours_ignore_plate_filters() {
        let mut app = App::default();
        app.contours = vec![
            rectangle(1, Kind::Support, 1000.),
            rectangle(2, Kind::Hole, 1000.),
        ];
        app.stiffnesses.assignments.insert(1, 2);
        let wall = BTreeMap::from([
            (Criterion::Section, ChoiceFilter::Single(2)),
            (Criterion::RigidOffset, ChoiceFilter::Single(0)),
        ]);
        assert_eq!(app.filtered_contour_ids(Kind::Support, None, &wall), [1]);
        assert_eq!(
            app.contour_filter_choices(Kind::Support, Criterion::RigidOffset)
                .len(),
            1
        );
        assert_eq!(app.filtered_contour_ids(Kind::Hole, None, &wall), [2]);
    }
}
