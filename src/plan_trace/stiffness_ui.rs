use super::*;
#[derive(Clone, Copy)]
enum StiffnessFilter {
    Material(u64),
    Section(u64),
    Design(u64),
}

fn select_visible_menu(
    response: &egui::Response,
    target: StiffnessFilter,
    action: &mut Option<StiffnessFilter>,
) {
    response.context_menu(|ui| {
        if ui.button("Выделить видимые").clicked() {
            *action = Some(target);
            ui.close_menu();
        }
    });
}
use crate::plan_trace::{fem::element::Material, stiffness::Family};

#[derive(Default)]
pub(super) struct SourceSync {
    library: Option<crate::plan_trace::stiffness::Library>,
    contours: Vec<(u64, Kind)>,
    variant: usize,
    pub(super) revision: u64,
}

fn stiffness_contours(contours: &[Contour]) -> impl Iterator<Item = (u64, Kind)> + '_ {
    contours
        .iter()
        .filter(|c| {
            matches!(
                c.kind,
                Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
            )
        })
        .map(|c| (c.id, c.kind))
}

#[cfg(all(test, not(target_arch = "wasm32")))]
#[test]
#[ignore = "requires MINI_FEM_LIRA_WALL_PROJECT pointing to a local slab project"]
fn local_project_removes_materials_referenced_only_by_unused_sections() {
    let path = std::env::var_os("MINI_FEM_LIRA_WALL_PROJECT").expect("fixture path");
    let mut app = App::default();
    app.load_project_path(path.into()).unwrap();
    let physical = app.materials.clone();
    let walls = app.support_height.materials.clone();
    for id in [1, 2] {
        assert_eq!(app.stiffnesses.elastic[&id].name, format!("Материал {id}"));
        assert!(!app.used_material_ids().contains(&id));
        assert!(app.stiffnesses.remove_unused_material(id));
    }
    app.sync_stiffness_sources();
    assert!([1, 2]
        .iter()
        .all(|id| !app.stiffnesses.elastic.contains_key(id)));
    assert_eq!(app.materials, physical);
    assert_eq!(app.support_height.materials, walls);
}

impl App {
    pub(super) fn sync_stiffness_sources(&mut self) {
        let synced = &self.stiffness_sources;
        if synced.variant == self.stiffness_variant
            && synced.library.as_ref() == Some(&self.stiffnesses)
            && synced
                .contours
                .iter()
                .copied()
                .eq(stiffness_contours(&self.contours))
        {
            return;
        }
        self.stiffnesses.reconcile(
            &self.contours,
            &self.materials,
            &self.support_height.materials,
        );
        self.stiffnesses.split_legacy_entries();
        let active = self.stiffnesses.for_variant(self.stiffness_variant);
        self.materials = active.resolve(&self.contours, Kind::Slab);
        self.support_height.materials = active.resolve(&self.contours, Kind::Support);
        self.support_height.beams = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Beam)
            .filter_map(|c| {
                let entry = active.resolved_entry(c.id)?;
                Some((
                    c.id,
                    crate::plan_trace::fem::beam::Section {
                        material: entry.material,
                        width_mm: entry.width_mm,
                    },
                ))
            })
            .collect();
        self.support_height.columns = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column)
            .filter_map(|c| {
                let e = active.resolved_entry(c.id)?;
                Some((
                    c.id,
                    crate::plan_trace::fem::beam::Section {
                        material: e.material,
                        width_mm: e.width_mm,
                    },
                ))
            })
            .collect();
        let synced = &mut self.stiffness_sources;
        synced.revision = synced.revision.wrapping_add(1);
        synced.variant = self.stiffness_variant;
        synced.library = Some(self.stiffnesses.clone());
        synced.contours.clear();
        synced.contours.extend(stiffness_contours(&self.contours));
    }
    pub(super) fn stiffness_controls(&mut self, ui: &mut egui::Ui) {
        let mut enabled = self.stiffnesses.alternative_enabled;
        let mut select = None;

        let editor = egui::CollapsingHeader::new("Материалы и жёсткости").id_salt("Жёсткости").default_open(false).show(ui,|ui|{
                egui::CollapsingHeader::new("Материалы").show(ui, |ui| {
                    let mut add_material = None;
                    ui.add_enabled_ui(!self.input_locked, |ui| {
                        ui.checkbox(&mut enabled, "Альтернативные материалы");
                        ui.horizontal_wrapped(|ui| {
                            if ui.button("+ Материал").clicked() {
                                add_material = Some(None);
                            }
                            ui.menu_button("Материал из базы", |ui| {
                                if ui.button("Бетон").clicked() {
                                    add_material = Some(Some(crate::plan_trace::stiffness::DatabaseConcrete::default()));
                                    ui.close_menu();
                                }
                            });
                        });
                    });
                    if let Some(concrete) = add_material {
                        let id=self.stiffnesses.elastic.keys().next_back().copied().unwrap_or(0)+1;
                        let primary=crate::plan_trace::stiffness::ElasticMaterial {
                            e_mpa:30000.,nu:0.2,rho:Material::default().weight,
                        };
                        let name=if concrete.is_some() { format!("Бетон {id}") } else { format!("Материал {id}") };
                        self.stiffnesses.elastic.insert(id,crate::plan_trace::stiffness::NamedMaterial { name, primary, alternative:primary, concrete });
                    }
                    let used = self.used_material_ids();
                    if ui.add_enabled(
                        !self.input_locked && self.stiffnesses.elastic.keys().any(|id| !used.contains(id)),
                        egui::Button::new("Удалить неиспользуемые"),
                    ).on_hover_text("Удаляет материалы, не назначенные объектам").clicked() {
                        let unused = self.stiffnesses.elastic.keys().filter(|id| !used.contains(id)).copied().collect::<Vec<_>>();
                        for &id in &unused { self.stiffnesses.remove_unused_material(id); }
                        self.status = format!("Удалено материалов: {}", unused.len());
                    }
                    let mut remove = None;
                    for (&id, material) in &mut self.stiffnesses.elastic {
                        let collapsible = ui.make_persistent_id(("elastic", id));
                        let unused = !used.contains(&id);
                        let mut toggle = false;
                        let mut header = egui::collapsing_header::CollapsingState::load_with_default_open(
                            ui.ctx(), collapsible, false,
                        ).show_header(ui, |ui| {
                            let label = egui::Label::new(material.name.as_str())
                                .sense(egui::Sense::click()).truncate();
                            let button_size = egui::vec2(24., 20.);
                            let title = if unused {
                                let width = (ui.available_width() - button_size.x - ui.spacing().item_spacing.x).max(1.);
                                ui.add_sized([width, button_size.y], label)
                            } else {
                                ui.add(label)
                            };
                            select_visible_menu(&title, StiffnessFilter::Material(id), &mut select);
                            toggle = title.on_hover_text(material.name.as_str()).clicked();
                            if unused && ui.add_enabled(!self.input_locked, egui::Button::new("−").min_size(button_size))
                                .on_hover_text("Удалить материал").clicked()
                            {
                                remove = Some(id);
                            }
                        });
                        if toggle {
                            header.toggle();
                        }
                        header.body(|ui| ui.add_enabled_ui(!self.input_locked, |ui| {
                            let mut color = self.stiffnesses.material_colors.get(&id).copied()
                                .unwrap_or_else(|| drawing_flags::material_color_default(id));
                            ui.horizontal(|ui| {
                                ui.spacing_mut().interact_size.x = 20.;
                                ui.spacing_mut().item_spacing.x = 4.;
                                if ui.color_edit_button_srgb(&mut color).on_hover_text("Цвет материала").changed() {
                                    self.stiffnesses.material_colors.insert(id, color);
                                }
                                let fields_width = match (enabled, material.concrete.is_some()) {
                                    (true, _) => 157.,
                                    (false, true) => 100.,
                                    (false, false) => 145.,
                                };
                                let name_width = (ui.available_width() - fields_width).max(30.);
                                ui.add(egui::TextEdit::singleline(&mut material.name).desired_width(name_width))
                                    .on_hover_text("Имя материала");
                                if let Some(concrete) = &mut material.concrete {
                                    let classes = &crate::plan_trace::reinforcement::CONCRETE;
                                    egui::ComboBox::from_id_salt(("material_concrete_class", id))
                                        .width(38.)
                                        .selected_text(classes[concrete.class.min(classes.len() - 1)].0)
                                        .show_ui(ui, |ui| {
                                            for (index, class) in classes.iter().enumerate() {
                                                ui.selectable_value(&mut concrete.class, index, class.0);
                                            }
                                        }).response.on_hover_text("Класс бетона");
                                    ui.add(egui::DragValue::new(&mut concrete.factor).prefix(if enabled { "kE₁ " } else { "kE " }).range(0.001..=10.).speed(0.01).update_while_editing(false))
                                        .on_hover_text("Коэффициент к начальному модулю упругости");
                                    if enabled {
                                        ui.add(egui::DragValue::new(&mut concrete.alternative_factor).prefix("kE₂ ").range(0.001..=10.).speed(0.01).update_while_editing(false))
                                            .on_hover_text("Коэффициент к начальному модулю для альтернативного материала");
                                    }
                                } else {
                                    elastic_controls(ui, &mut material.primary, self.load_units, self.gravity, enabled.then_some("1"));
                                }
                            });
                            if material.concrete.is_none() && enabled {
                                ui.horizontal(|ui| {
                                    ui.spacing_mut().interact_size.x = 20.;
                                    ui.spacing_mut().item_spacing.x = 4.;
                                    elastic_controls(ui, &mut material.alternative, self.load_units, self.gravity, Some("2"));
                                });
                            }
                        }));
                    }
                    if let Some(id) = remove {
                        self.stiffnesses.remove_unused_material(id);
                    }
                });
            ui.scope(|ui|{
                let mut remove=None;
                let base_beam=self.stiffnesses.entries.iter().find(|(_,e)|matches!(e.family,Family::Beam|Family::Column)).map(|(&id,_)|id);
                let used=self.stiffnesses.assignments.values().copied().collect::<std::collections::BTreeSet<_>>();
                egui::ScrollArea::vertical().id_salt("stiffness_library_scroll").max_height(320.).show(ui,|ui|{
                    for (label, families) in [
                        ("Жёсткости пластин", [Family::Slab, Family::Wall]),
                        ("Жёсткости стержней", [Family::Beam, Family::Column]),
                    ] {
                    let bar = matches!(families[0], Family::Beam);
                    egui::CollapsingHeader::new(label).default_open(false).show(ui, |ui| {
                    ui.add_enabled_ui(!self.input_locked, |ui| ui.horizontal_wrapped(|ui| {
                        if bar {
                            if ui.button("+ Колонна").clicked() {
                                self.stiffnesses.add(Family::Column, crate::plan_trace::stiffness::column_material());
                            }
                            if ui.button("+ Балка").clicked() {
                                self.stiffnesses.add(Family::Beam, crate::plan_trace::stiffness::beam_material());
                            }
                        } else if ui.button("+ Сечение").clicked() {
                            let id = self.stiffnesses.add(Family::Slab, Material::default());
                            self.stiffnesses.entries.get_mut(&id).unwrap().name = format!("Пластина {id}");
                        }
                    }));
                    for (&id,e) in self.stiffnesses.entries.iter_mut().filter(|(_, e)| families.contains(&e.family)) {
                        let state_id = ui.make_persistent_id(egui::Id::new(("stiffness", id)));
                        let mut toggle = false;
                        let mut header = egui::collapsing_header::CollapsingState::load_with_default_open(ui.ctx(), state_id, false).show_header(ui, |ui| {
                            let title = ui.add(egui::Label::new(e.name.as_str()).truncate().sense(egui::Sense::click()));
                            select_visible_menu(&title, StiffnessFilter::Section(id), &mut select);
                            toggle = title.on_hover_text(e.name.as_str()).clicked();
                        });
                        if toggle {
                            header.toggle();
                        }
                        header.body(|ui| {
                            let default = stiffness_color_default(e.family);
                            let mut color = self.stiffnesses.colors.get(&id).copied().unwrap_or(default);
                            ui.horizontal(|ui| {
                                ui.spacing_mut().interact_size.x = 20.;
                                ui.spacing_mut().item_spacing.x = 4.;
                                if ui.color_edit_button_srgb(&mut color).on_hover_text("Цвет сечения").changed() {
                                    self.stiffnesses.colors.insert(id, color);
                                }
                                ui.add_enabled_ui(!self.input_locked, |ui| {
                                let delete_size = egui::vec2(24., 20.);
                                let margin = egui::Margin::symmetric(4, 2);
                                let gap = ui.spacing().item_spacing.x;
                                let fields_width = delete_size.x + gap + if bar { 0. } else { 72. + gap };
                                let name_width = (ui.available_width() - fields_width - margin.sum().x).max(24.);
                                ui.add(egui::TextEdit::singleline(&mut e.name).margin(margin).desired_width(name_width))
                                    .on_hover_text("Имя сечения");
                                if !bar {
                                    ui.add(egui::DragValue::new(&mut e.material.thickness_mm).prefix("h ").suffix(" мм").update_while_editing(false).range(1. ..=10000.))
                                        .on_hover_text("Толщина пластины");
                                }
                                if ui.add_enabled(id>2&&Some(id)!=base_beam&&!used.contains(&id), egui::Button::new("−").min_size(delete_size))
                                    .on_hover_text("Удалить жёсткость")
                                    .on_disabled_hover_text("Базовые жёсткости и наборы, связанные с объектами или их историей, удалить нельзя")
                                    .clicked() { remove=Some(id); }
                                });
                            });
                            ui.add_enabled_ui(!self.input_locked,|ui|{
                            if bar {
                                ui.horizontal_wrapped(|ui| {
                                    ui.weak(e.family.label());
                                    ui.add(egui::DragValue::new(&mut e.width_mm).prefix("b ").suffix(" мм").update_while_editing(false).range(1. ..=10000.))
                                        .on_hover_text("Ширина сечения");
                                    ui.add(egui::DragValue::new(&mut e.material.thickness_mm).prefix("h ").suffix(" мм").update_while_editing(false).range(1. ..=10000.))
                                        .on_hover_text("Высота сечения");
                                });
                            }
                            });
                        });
                    }
                    });
                    }
                });
                if let Some(id)=remove{self.stiffnesses.entries.remove(&id);self.stiffnesses.alternatives.remove(&id);self.stiffnesses.elastic_assignments.remove(&id);self.stiffnesses.design_assignments.remove(&id);self.stiffnesses.colors.remove(&id);self.stiffnesses.assignments.retain(|_,p|*p!=id);}
            });
            egui::CollapsingHeader::new("Параметры конструирования").id_salt("Конструирование пластин").show(ui, |ui| {
                if ui.add_enabled(!self.input_locked, egui::Button::new("+ Набор параметров")).clicked() {
                    let id=self.stiffnesses.designs.keys().next_back().copied().unwrap_or(0)+1;
                    self.stiffnesses.designs.insert(id,crate::plan_trace::stiffness::Design {
                        name:format!("Конструирование {id}"),bar:false,settings:Default::default(),
                    });
                }
                for (&id,design) in self.stiffnesses.designs.iter_mut().filter(|(_,d)| !d.bar) {
                    let row = egui::CollapsingHeader::new(&design.name).id_salt(("design",id)).show(ui,|ui| ui.add_enabled_ui(!self.input_locked, |ui| {
                        ui.text_edit_singleline(&mut design.name);
                        reinforcement_ui::property_controls(ui,id,&mut design.settings,200.);
                    }));
                    select_visible_menu(&row.header_response, StiffnessFilter::Design(id), &mut select);
                }
            });
        });

        if editor.body_returned.is_none() {
            return;
        }
        if let Some(filter) = select {
            let ids = self
                .contours
                .iter()
                .filter(|c| self.selectable(c))
                .filter(|c| {
                    !self.spatial.enabled || !self.sections.active() || {
                        let heights = self.source_heights(c);
                        self.sections.intersects(
                            c.points
                                .iter()
                                .flat_map(|p| heights.iter().map(|&z| [p[0], p[1], z])),
                        )
                    }
                })
                .filter(|c| match filter {
                    StiffnessFilter::Material(id) => self.stiffnesses.material_id(c.id) == Some(id),
                    StiffnessFilter::Section(id) => {
                        self.stiffnesses.assignments.get(&c.id) == Some(&id)
                    }
                    StiffnessFilter::Design(id) => self.stiffnesses.design_id(c.id) == Some(id),
                })
                .map(|c| c.id)
                .collect::<Vec<_>>();
            self.status = format!("Выделено объектов: {}", ids.len());
            if !ids.is_empty() {
                self.select_ids(ids, egui::Modifiers::default());
            }
        }

        if enabled != self.stiffnesses.alternative_enabled {
            if !enabled && self.stiffness_variant == 1 {
                self.switch_stiffness_variant(0);
            }
            self.stiffnesses.alternative_enabled = enabled;
        }
        self.stiffnesses
            .alternatives
            .retain(|id, _| self.stiffnesses.entries.contains_key(id));
        self.sync_stiffness_sources();
    }
    pub(super) fn contour_stiffness_controls(&mut self, ui: &mut egui::Ui) {
        let Some((id, kind)) = self.selected.and_then(|id| {
            self.contours
                .iter()
                .find(|c| c.id == id)
                .map(|c| (c.id, c.kind))
        }) else {
            return;
        };
        if !matches!(kind, Kind::Slab | Kind::Support | Kind::Beam | Kind::Column) {
            return;
        }
        let family = match kind {
            Kind::Slab => Family::Slab,
            Kind::Support => Family::Wall,
            Kind::Column => Family::Column,
            _ => Family::Beam,
        };
        let default_id = self
            .stiffnesses
            .entries
            .iter()
            .find(|(_, e)| e.family == family)
            .map_or(1, |(&id, _)| id);
        let mut property = self
            .stiffnesses
            .assignments
            .get(&id)
            .copied()
            .unwrap_or(default_id);
        let locked = self.is_kind_locked(kind);
        let old = property;
        let old_material = self.stiffnesses.material_id(id).unwrap_or(0);
        let mut material = old_material;
        ui.add_enabled_ui(!locked, |ui| {
            ui.horizontal(|ui| {
                ui.label("Жёсткость");
                egui::ComboBox::from_id_salt(("contour_stiffness", id))
                    .selected_text(
                        self.stiffnesses
                            .entries
                            .get(&property)
                            .map_or("Не назначена", |e| e.name.as_str()),
                    )
                    .show_ui(ui, |ui| {
                        for (&id, e) in &self.stiffnesses.entries {
                            if e.family.section_family() != family.section_family() {
                                continue;
                            }
                            ui.selectable_value(&mut property, id, &e.name);
                        }
                    });
            });
            ui.horizontal(|ui| {
                ui.label("Материал");
                self.material_combo(ui, ("contour_material", id), &mut material);
            });
        });
        let item = [object_properties::Item::Contour(id)];
        if old != property {
            self.property_apply(
                &item,
                object_properties::Change::Stiffness(property),
                ui.id().with("section"),
            );
        }
        if old_material != material {
            self.property_apply(
                &item,
                object_properties::Change::Material(material),
                ui.id().with("material"),
            );
        }
    }

    pub(super) fn material_combo(
        &self,
        ui: &mut egui::Ui,
        id: impl std::hash::Hash,
        material: &mut u64,
    ) -> egui::Response {
        egui::ComboBox::from_id_salt(id)
            .selected_text(
                self.stiffnesses
                    .elastic
                    .get(material)
                    .map_or("Различные", |m| m.name.as_str()),
            )
            .show_ui(ui, |ui| {
                for (&id, value) in &self.stiffnesses.elastic {
                    ui.selectable_value(material, id, &value.name);
                }
            })
            .response
    }
}

pub(super) fn stiffness_color_default(family: Family) -> [u8; 3] {
    match family {
        Family::Slab => [75, 205, 135],
        Family::Wall => [85, 180, 255],
        Family::Beam => [255, 210, 80],
        Family::Column => [186, 155, 238],
    }
}

fn elastic_controls(
    ui: &mut egui::Ui,
    m: &mut crate::plan_trace::stiffness::ElasticMaterial,
    units: crate::plan_trace::loads::ForceUnits,
    gravity: f64,
    variant: Option<&str>,
) {
    if let Some(variant) = variant {
        ui.small(variant).on_hover_text(if variant == "1" {
            "Основной материал"
        } else {
            "Альтернативный материал"
        });
    }
    ui.add(
        egui::DragValue::new(&mut m.e_mpa)
            .prefix("E ")
            .update_while_editing(false)
            .range(1. ..=1e6),
    )
    .on_hover_text("Модуль упругости, МПа");
    ui.add(
        egui::DragValue::new(&mut m.nu)
            .prefix("ν ")
            .update_while_editing(false)
            .range(-0.99..=0.49),
    )
    .on_hover_text("Коэффициент Пуассона");
    let mut rho = m.rho.in_units(units, gravity);
    if ui
        .add(
            egui::DragValue::new(&mut rho)
                .prefix("ρ ")
                .update_while_editing(false)
                .range(0. ..=1e6),
        )
        .on_hover_text(format!("Объёмный вес, {}/м³", units.label()))
        .changed()
    {
        m.rho = crate::plan_trace::loads::Magnitude { value: rho, units };
    }
}
