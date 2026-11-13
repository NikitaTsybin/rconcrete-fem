//! Source-only building composition. Floor IDs never depend on scene/FE indices.
use super::*;
use std::collections::BTreeSet;
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub(super) struct Floor {
    pub id: u64,
    pub name: String,
    pub elevation_mm: f64,
    pub height_mm: f64,
}
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct Building {
    pub enabled: bool,
    pub floors: Vec<Floor>,
    pub active: u64,
    pub hidden_floors: BTreeSet<u64>,
    pub contour_floors: BTreeMap<u64, u64>,
    pub underlay_floors: BTreeMap<u64, u64>,
}
impl Default for Building {
    fn default() -> Self {
        Self {
            enabled: false,
            floors: vec![Floor {
                id: 1,
                name: "Этаж 1".into(),
                elevation_mm: 0.,
                height_mm: 3000.,
            }],
            active: 1,
            hidden_floors: Default::default(),
            contour_floors: BTreeMap::new(),
            underlay_floors: BTreeMap::new(),
        }
    }
}
impl Building {
    fn renumber_auto_names(&mut self) {
        let mut order = (0..self.floors.len()).collect::<Vec<_>>();
        order.sort_by(|a, b| {
            self.floors[*a]
                .elevation_mm
                .total_cmp(&self.floors[*b].elevation_mm)
        });
        for (number, index) in order.into_iter().enumerate() {
            let floor = &mut self.floors[index];
            if floor
                .name
                .strip_prefix("Этаж ")
                .and_then(|v| v.parse::<usize>().ok())
                .is_some()
            {
                floor.name = format!("Этаж {}", number + 1);
            }
        }
    }
    pub fn floor_visible(&self, id: u64) -> bool {
        !self.enabled || !self.hidden_floors.contains(&id)
    }
    pub fn validate(&self) -> Result<(), String> {
        if self.floors.is_empty()
            || self.floors.len() > 500
            || !self.floors.iter().any(|f| f.id == self.active)
        {
            return Err("Некорректный список этажей".into());
        }
        let mut ids = std::collections::BTreeSet::new();
        let mut levels = vec![];
        for f in &self.floors {
            if f.id == 0
                || f.id == u64::MAX
                || !ids.insert(f.id)
                || f.name.len() > 1024
                || !f.elevation_mm.is_finite()
                || f.elevation_mm.abs() > 1e8
                || !f.height_mm.is_finite()
                || !(100. ..=1e6).contains(&f.height_mm)
            {
                return Err("Некорректные параметры этажа".into());
            }
            levels.push(f.elevation_mm);
        }
        levels.sort_by(f64::total_cmp);
        if levels.windows(2).any(|w| w[1] - w[0] < 100.) {
            return Err("Отметки этажей должны различаться не менее чем на 100 мм".into());
        }
        if self
            .contour_floors
            .values()
            .chain(self.underlay_floors.values())
            .any(|id| !ids.contains(id))
        {
            return Err("Объект ссылается на отсутствующий этаж".into());
        }
        Ok(())
    }
    pub fn floor(&self, id: u64) -> &Floor {
        self.floors
            .iter()
            .find(|f| f.id == id)
            .unwrap_or(&self.floors[0])
    }
    pub fn owner(&self, id: u64) -> u64 {
        self.contour_floors
            .get(&id)
            .copied()
            .unwrap_or(self.floors[0].id)
    }
    pub fn extent(&self, id: u64) -> [f64; 2] {
        let f = self.floor(id);
        [
            f.elevation_mm,
            self.floors
                .iter()
                .filter(|g| g.elevation_mm > f.elevation_mm)
                .map(|g| g.elevation_mm)
                .min_by(f64::total_cmp)
                .unwrap_or(f.elevation_mm + f.height_mm),
        ]
    }
    pub fn snapshot(
        &self,
        contours: &[Contour],
        underlays: &[crate::plan_trace::underlay::Underlay],
    ) -> Self {
        let mut out = self.clone();
        let contour_ids = contours.iter().map(|c| c.id).collect::<BTreeSet<_>>();
        let underlay_ids = underlays.iter().map(|u| u.id).collect::<BTreeSet<_>>();
        out.contour_floors.retain(|id, _| contour_ids.contains(id));
        out.underlay_floors
            .retain(|id, _| underlay_ids.contains(id));
        out
    }
}

#[derive(Default)]
pub(super) struct ObjectSync {
    initialized: bool,
    pub(super) revision: u64,
    enabled: bool,
    floors: Vec<(u64, u64, u64)>,
    contours: Vec<(u64, Option<u64>, Option<[u64; 2]>)>,
    underlays: Vec<u64>,
    owners: BTreeMap<u64, u64>,
    underlay_owners: BTreeMap<u64, u64>,
    linked_geometry: Vec<(usize, Contour)>,
    mesh_size: f64,
}

fn floor_geometry(floor: &Floor) -> (u64, u64, u64) {
    (
        floor.id,
        floor.elevation_mm.to_bits(),
        floor.height_mm.to_bits(),
    )
}

fn contour_ownership(contour: &Contour) -> (u64, Option<u64>, Option<[u64; 2]>) {
    (
        contour.id,
        contour.parent,
        contour.placement.rigid_link_targets,
    )
}

impl App {
    pub(super) fn sync_building_objects(&mut self) {
        let synced = &self.building_sync;
        if synced.initialized
            && synced.enabled == self.building.enabled
            && synced
                .floors
                .iter()
                .copied()
                .eq(self.building.floors.iter().map(floor_geometry))
            && self
                .building
                .floors
                .iter()
                .any(|f| f.id == self.building.active)
            && synced
                .contours
                .iter()
                .copied()
                .eq(self.contours.iter().map(contour_ownership))
            && synced
                .underlays
                .iter()
                .copied()
                .eq(self.underlays.iter().map(|u| u.id))
            && synced.owners == self.building.contour_floors
            && synced.underlay_owners == self.building.underlay_floors
            && (synced.linked_geometry.is_empty()
                || (synced.mesh_size == self.mesh_size
                    && synced
                        .linked_geometry
                        .iter()
                        .all(|(i, contour)| self.contours.get(*i) == Some(contour))))
        {
            return;
        }
        let floor_ids = self
            .building
            .floors
            .iter()
            .map(|f| f.id)
            .collect::<std::collections::BTreeSet<_>>();
        if !floor_ids.contains(&self.building.active) {
            if let Some(f) = self.building.floors.first() {
                self.building.active = f.id;
            }
        }
        let live = self
            .contours
            .iter()
            .enumerate()
            .map(|(i, c)| (c.id, i))
            .collect::<BTreeMap<_, _>>();
        let underlays = self
            .underlays
            .iter()
            .map(|u| u.id)
            .collect::<std::collections::BTreeSet<_>>();
        self.building
            .contour_floors
            .retain(|id, _| live.contains_key(id));
        self.building
            .underlay_floors
            .retain(|id, _| underlays.contains(id));
        for floor in self
            .building
            .contour_floors
            .values_mut()
            .chain(self.building.underlay_floors.values_mut())
        {
            if !floor_ids.contains(floor) {
                *floor = self.building.active;
            }
        }
        let underlay_count = self.building.underlay_floors.len();
        let mut bound_links = Vec::new();
        for (i, c) in self.contours.iter().enumerate() {
            if c.placement.rigid_link_targets.is_some() {
                bound_links.push(i);
            }
            let inherited = c
                .parent
                .and_then(|id| self.building.contour_floors.get(&id).copied())
                .unwrap_or(self.building.active);
            self.building
                .contour_floors
                .entry(c.id)
                .or_insert(inherited);
        }
        if !bound_links.is_empty() {
            self.sync_bound_rigid_links(&live, bound_links);
        }
        for u in &self.underlays {
            self.building
                .underlay_floors
                .entry(u.id)
                .or_insert(self.building.active);
        }
        self.sync_mesh_object_owners();
        if self.building.enabled && underlay_count != self.building.underlay_floors.len() {
            self.rebuild_index();
        }
        let linked_ids = self
            .contours
            .iter()
            .filter_map(|c| c.placement.rigid_link_targets.map(|[a, b]| [c.id, a, b]))
            .flatten()
            .collect::<BTreeSet<_>>();
        let synced = &mut self.building_sync;
        synced.initialized = true;
        synced.revision = synced.revision.wrapping_add(1);
        synced.enabled = self.building.enabled;
        synced.floors.clear();
        synced
            .floors
            .extend(self.building.floors.iter().map(floor_geometry));
        synced.contours.clear();
        synced
            .contours
            .extend(self.contours.iter().map(contour_ownership));
        synced.underlays.clear();
        synced.underlays.extend(self.underlays.iter().map(|u| u.id));
        synced.owners.clone_from(&self.building.contour_floors);
        synced
            .underlay_owners
            .clone_from(&self.building.underlay_floors);
        synced.linked_geometry.clear();
        if !linked_ids.is_empty() {
            synced.linked_geometry.extend(
                self.contours
                    .iter()
                    .enumerate()
                    .filter(|(_, c)| linked_ids.contains(&c.id))
                    .map(|(i, c)| (i, c.clone())),
            );
        }
        synced.mesh_size = self.mesh_size;
    }
    pub(super) fn building_layout(&self) -> Option<crate::plan_trace::building_mesh::Layout> {
        self.building
            .enabled
            .then(|| crate::plan_trace::building_mesh::Layout {
                legacy_wall_extent: None,
                levels: self
                    .building
                    .floors
                    .iter()
                    .map(|f| {
                        let [z, top] = self.building.extent(f.id);
                        crate::plan_trace::building_mesh::Level { id: f.id, z, top }
                    })
                    .collect(),
                owners: self
                    .contours
                    .iter()
                    .map(|c| (c.id, self.building.owner(c.id)))
                    .collect(),
            })
    }
    pub(super) fn mesh_node_on_floor(&self, n: usize) -> bool {
        (!self.building.enabled
            || self
                .fe_mesh
                .as_ref()
                .and_then(|m| m.building.as_ref())
                .is_none_or(|b| b.node_floors.get(n) == Some(&self.building.active)))
            && self.mesh_node_view_visible(n)
    }
    pub(super) fn mesh_element_on_floor(&self, i: usize) -> bool {
        (!self.building.enabled
            || self
                .fe_mesh
                .as_ref()
                .and_then(|m| m.building.as_ref())
                .is_none_or(|b| b.element_floors.get(i) == Some(&self.building.active)))
            && !self.mesh_element_hidden(i)
    }
    pub(super) fn source_heights(&self, c: &Contour) -> Vec<f64> {
        self.source_heights_in(c, &self.building)
    }
    pub(super) fn source_beam_path(&self, c: &Contour) -> Vec<[f64; 3]> {
        c.beam_path(self.source_beam_extent(c, &self.building))
    }
    fn source_beam_extent(&self, c: &Contour, building: &Building) -> [f64; 2] {
        if building.enabled {
            building.extent(building.owner(c.id))
        } else {
            [0., 3000.]
        }
    }
    fn source_heights_in(&self, c: &Contour, building: &Building) -> Vec<f64> {
        let c = if matches!(
            c.kind,
            Kind::Hole | Kind::MeshRegion | Kind::LoadHole | Kind::Punching
        ) {
            c.parent.and_then(|id| self.view_contour(id)).unwrap_or(c)
        } else {
            c
        };
        if c.kind == Kind::Beam {
            let extent = self.source_beam_extent(c, building);
            if c.is_free_beam() {
                let mut heights = c.beam_path(extent).into_iter().map(|p| p[2]);
                if let Some(z) = heights.next() {
                    let [lo, hi] = heights.fold([z, z], |[lo, hi], z| [lo.min(z), hi.max(z)]);
                    return if lo == hi { vec![lo] } else { vec![lo, hi] };
                }
            }
            return vec![c.placement.slab.z(extent)];
        }
        let extent = if building.enabled {
            building.extent(building.owner(c.id))
        } else {
            [0., building.floors[0].height_mm]
        };
        if c.kind == Kind::Support {
            if !building.enabled {
                return self.support_height.extent().to_vec();
            }
            return c.placement.wall(extent).to_vec();
        }
        if matches!(c.kind, Kind::Column | Kind::RigidLink) {
            return if building.enabled {
                c.placement.wall(extent).to_vec()
            } else {
                self.support_height.extent().to_vec()
            };
        }
        vec![if building.enabled {
            c.placement.slab.z(extent)
        } else {
            0.
        }]
    }
    fn floor_changes_move_locked_contours(
        &self,
        before: &Building,
        after: &Building,
        removed: &BTreeSet<u64>,
    ) -> bool {
        let locked = self.movement_locked_contours();
        self.contours.iter().any(|c| {
            locked.contains(&c.id)
                && !removed.contains(&c.id)
                && self.source_heights_in(c, before) != self.source_heights_in(c, after)
        })
    }
    pub(super) fn floor_underlay_visible(&self, id: u64) -> bool {
        !self.underlay_hidden(id)
            && (!self.building.enabled
                || self
                    .building
                    .underlay_floors
                    .get(&id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id)
                    == self.building.active)
    }
    pub(super) fn floor_layer_visible(&self, name: &str) -> bool {
        name.strip_prefix('#')
            .and_then(|s| s.split_once(" · "))
            .and_then(|s| s.0.parse::<u64>().ok())
            .is_none_or(|id| self.floor_underlay_visible(id))
    }
    pub(super) fn on_view_floor(&self, c: &Contour) -> bool {
        if !self.building.floor_visible(self.building.owner(c.id)) {
            return false;
        }
        if !self.building.enabled || self.spatial.enabled {
            return true;
        }
        if !matches!(c.kind, Kind::Support | Kind::Column) {
            return self.on_active_floor(c);
        }
        let owner = self.building.owner(c.id);
        let active = self.building.active;
        let z = self.building.floor(active).elevation_mm;
        let previous = (self.building.extent(owner)[1] - z).abs() < 1e-6 && owner != active;
        match self.drawing_visibility.support_storeys {
            1 => previous,
            2 => owner == active || previous,
            _ => owner == active,
        }
    }
    pub(super) fn support_storey_selector(&mut self, ui: &mut egui::Ui) {
        if !self.building.enabled || self.spatial.enabled {
            return;
        }
        bottom_selector_wrap(ui, 145.);
        let before = self.drawing_visibility.support_storeys;
        egui::ComboBox::from_id_salt("support_storeys_2d")
            .width(125.)
            .truncate()
            .selected_text(format!(
                "Опоры: {}",
                match before {
                    1 => "Предыдущий",
                    2 => "Оба",
                    _ => "Текущий",
                }
            ))
            .show_ui(ui, |ui| {
                ui.selectable_value(&mut self.drawing_visibility.support_storeys, 0, "Текущий");
                ui.selectable_value(
                    &mut self.drawing_visibility.support_storeys,
                    1,
                    "Предыдущий",
                );
                ui.selectable_value(&mut self.drawing_visibility.support_storeys, 2, "Оба");
            });
        if before != self.drawing_visibility.support_storeys {
            self.prune_selection();
            self.invalidate_scene_buffers();
            self.rebuild_index();
        }
    }
    pub(super) fn on_active_floor(&self, c: &Contour) -> bool {
        !self.building.enabled || self.building.owner(c.id) == self.building.active
    }
    pub(super) fn floor_contours(&self) -> Vec<Contour> {
        self.contours
            .iter()
            .filter(|c| self.on_active_floor(c) && !self.object_hidden(c))
            .cloned()
            .collect()
    }
    pub(super) fn on_list_floor(&self, c: &Contour) -> bool {
        self.floor_list
            .is_none_or(|id| self.building.owner(c.id) == id)
    }
    pub(super) fn building_changed(&mut self) {
        self.spatial.cache_reset();
        self.slab_cache = None;
        self.invalidate_scene_buffers();
        self.clear_mesh();
        self.rebuild_index();
        self.analysis = None;
        self.analysis_input = None;
        self.combination_cache = None;
    }
    pub(super) fn change_floor(&mut self, id: u64) {
        if self.building.active == id {
            return;
        }
        let operation = self.clipboard_op.take();
        self.cancel();
        self.clipboard_op = operation;
        self.building.active = id;
        if self.active_floor_only() {
            self.view_visibility_changed();
        }
        self.select_only(None);
        self.slab_cache = None;
        self.invalidate_scene_buffers();
        self.spatial.frame_start = None;
        if let Some(mesh) = &self.fe_mesh {
            self.fe_snap_index = Index::mesh_floor(mesh, self.building.enabled.then_some(id));
        }
        self.rebuild_index();
    }
    pub(super) fn floor_selector(&mut self, ui: &mut egui::Ui) {
        if !self.building.enabled {
            return;
        }
        let mut id = self.building.active;
        bottom_selector_wrap(ui, 145.);
        egui::ComboBox::from_id_salt("active_building_floor")
            .width(125.)
            .truncate()
            .selected_text(&self.building.floor(id).name)
            .show_ui(ui, |ui| {
                for f in &self.building.floors {
                    ui.selectable_value(
                        &mut id,
                        f.id,
                        format!("{} · {:.0} мм", f.name, f.elevation_mm),
                    );
                }
            });
        self.change_floor(id);
    }
    fn request_delete_floor(&mut self, id: u64) {
        if self.input_locked || self.building.floors.len() <= 1 {
            return;
        }
        let name = self.building.floor(id).name.clone();
        if rfd::MessageDialog::new()
            .set_title("Удаление этажа")
            .set_description(format!(
                "Удалить «{name}» и все его объекты? Сетка и результаты будут сброшены."
            ))
            .set_level(rfd::MessageLevel::Warning)
            .set_buttons(rfd::MessageButtons::YesNo)
            .show()
            != rfd::MessageDialogResult::Yes
        {
            return;
        }
        let mut ids = self
            .contours
            .iter()
            .filter(|c| self.building.owner(c.id) == id)
            .map(|c| c.id)
            .collect::<BTreeSet<_>>();
        loop {
            let count = ids.len();
            for c in &self.contours {
                if c.parent.is_some_and(|p| ids.contains(&p))
                    || c.placement
                        .rigid_link_targets
                        .is_some_and(|targets| targets.iter().any(|id| ids.contains(id)))
                {
                    ids.insert(c.id);
                }
            }
            if ids.len() == count {
                break;
            }
        }
        let mut remaining = self.building.clone();
        remaining.floors.retain(|f| f.id != id);
        if self.floor_changes_move_locked_contours(&self.building, &remaining, &ids) {
            self.status = "Удаление этажа изменит отметку заблокированного объекта.".into();
            return;
        }
        let underlays = self
            .underlays
            .iter()
            .filter(|u| {
                self.building
                    .underlay_floors
                    .get(&u.id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id)
                    == id
            })
            .map(|u| u.id)
            .collect::<Vec<_>>();
        for uid in underlays {
            self.underlays.retain(|u| u.id != uid);
            self.pdf_textures.remove(&uid);
            self.building.underlay_floors.remove(&uid);
        }
        self.contours.retain(|c| !ids.contains(&c.id));
        self.building
            .contour_floors
            .retain(|cid, _| !ids.contains(cid));
        self.stiffnesses
            .assignments
            .retain(|cid, _| !ids.contains(cid));
        self.building.floors.retain(|f| f.id != id);
        self.building.renumber_auto_names();
        self.building.hidden_floors.remove(&id);
        if self.building.active == id {
            self.building.active = self.building.floors[0].id;
        }
        self.history.clear();
        self.underlay_history.clear();
        self.scaling_history.clear();
        self.cancel();
        self.sync_underlays();
        self.properties = Default::default();
        self.floor_list = None;
        self.select_only(None);
        self.selected_underlay = None;
        self.underlay_selection.clear();
        self.sync_stiffness_sources();
        self.sync_building_objects();
        self.building_changed();
        self.status = format!("Этаж «{name}» удалён");
    }
    pub(super) fn floor_controls(&mut self, ui: &mut egui::Ui) {
        if !self.building.enabled {
            return;
        }
        egui::CollapsingHeader::new("Этажи")
            .default_open(false)
            .show(ui, |ui| {
                ui.add_enabled_ui(true, |ui| {
                    if ui
                        .add_enabled(
                            !self.input_locked && self.building.floors.len() < 500,
                            egui::Button::new("Добавить этаж"),
                        )
                        .clicked()
                    {
                        let id = self.building.floors.iter().map(|f| f.id).max().unwrap_or(0) + 1;
                        let last = self
                            .building
                            .floors
                            .iter()
                            .max_by(|a, b| a.elevation_mm.total_cmp(&b.elevation_mm))
                            .unwrap();
                        let z = last.elevation_mm + last.height_mm;
                        self.building.floors.push(Floor {
                            id,
                            name: format!("Этаж {}", self.building.floors.len() + 1),
                            elevation_mm: z,
                            height_mm: last.height_mm,
                        });
                        self.building_changed();
                        self.change_floor(id);
                    }
                    let previous = self.building.floors.clone();
                    let top = self
                        .building
                        .floors
                        .iter()
                        .max_by(|a, b| a.elevation_mm.total_cmp(&b.elevation_mm))
                        .unwrap()
                        .id;
                    let heights = self
                        .building
                        .floors
                        .iter()
                        .map(|f| {
                            let e = self.building.extent(f.id);
                            (f.id, e[1] - e[0])
                        })
                        .collect::<BTreeMap<_, _>>();
                    let mut chosen = self.building.active;
                    let can_remove = !self.input_locked && self.building.floors.len() > 1;
                    let mut remove = None;
                    let mut visibility_changed = false;
                    let mut floor_visibility_action = None;
                    let mut all_visibility_action = None;
                    egui::Grid::new("building_floor_table").show(ui, |ui| {
                        let visible_count = self
                            .building
                            .floors
                            .iter()
                            .filter(|f| !self.building.hidden_floors.contains(&f.id))
                            .count();
                        let mut all_visible = visible_count == self.building.floors.len();
                        let mixed = visible_count > 0 && !all_visible;
                        if ui
                            .add(egui::Checkbox::new(&mut all_visible, "").indeterminate(mixed))
                            .on_hover_text("Показать / скрыть все этажи")
                            .changed()
                        {
                            all_visibility_action = Some(all_visible);
                        }
                        ui.label("Этаж");
                        ui.label("Z, мм");
                        ui.label("H, мм");
                        ui.label("");
                        ui.end_row();
                        for f in &mut self.building.floors {
                            let mut visible = !self.building.hidden_floors.contains(&f.id);
                            if ui
                                .checkbox(&mut visible, "")
                                .on_hover_text("Показывать этаж")
                                .changed()
                            {
                                if visible {
                                    self.building.hidden_floors.remove(&f.id);
                                } else {
                                    self.building.hidden_floors.insert(f.id);
                                }
                                visibility_changed = true;
                            }
                            ui.horizontal(|ui| {
                                let row = ui.selectable_label(chosen == f.id, format!("{}", f.id));
                                if row.clicked() {
                                    chosen = f.id;
                                }
                                row.context_menu(|ui| {
                                    if ui.button("Сделать активным").clicked() {
                                        chosen = f.id;
                                        ui.close_menu();
                                    }
                                    if ui.button("Показать").clicked() {
                                        floor_visibility_action = Some((f.id, true));
                                        ui.close_menu();
                                    }
                                    if ui.button("Скрыть").clicked() {
                                        floor_visibility_action = Some((f.id, false));
                                        ui.close_menu();
                                    }
                                });
                                let name = ui.add_enabled(
                                    !self.input_locked,
                                    egui::TextEdit::singleline(&mut f.name).desired_width(130.),
                                );
                                name.context_menu(|ui| {
                                    if ui.button("Сделать активным").clicked() {
                                        chosen = f.id;
                                        ui.close_menu();
                                    }
                                    if ui.button("Показать").clicked() {
                                        floor_visibility_action = Some((f.id, true));
                                        ui.close_menu();
                                    }
                                    if ui.button("Скрыть").clicked() {
                                        floor_visibility_action = Some((f.id, false));
                                        ui.close_menu();
                                    }
                                });
                            });
                            ui.add_enabled(
                                !self.input_locked,
                                egui::DragValue::new(&mut f.elevation_mm)
                                    .update_while_editing(false)
                                    .speed(100.),
                            );
                            if f.id == top {
                                ui.add_enabled(
                                    !self.input_locked,
                                    egui::DragValue::new(&mut f.height_mm)
                                        .range(100. ..=1e6)
                                        .speed(100.),
                                );
                            } else {
                                let mut height = heights[&f.id];
                                ui.add_enabled_ui(false, |ui| {
                                    ui.add_sized(
                                        [55., ui.spacing().interact_size.y],
                                        egui::DragValue::new(&mut height).max_decimals(1),
                                    )
                                })
                                .response
                                .on_hover_text("Вычислено по отметкам этажей");
                            }
                            if ui
                                .add_enabled(can_remove, egui::Button::new("−"))
                                .on_hover_text("Удалить этаж")
                                .clicked()
                            {
                                remove = Some(f.id);
                            }
                            ui.end_row();
                        }
                    });
                    if let Some(show) = all_visibility_action {
                        if self.active_floor_only() {
                            self.toggle_active_floor_only();
                        }
                        if show {
                            self.building.hidden_floors.clear();
                        } else {
                            self.building.hidden_floors =
                                self.building.floors.iter().map(|f| f.id).collect();
                        }
                        visibility_changed = true;
                    }
                    if previous != self.building.floors {
                        let previous_building = Building {
                            floors: previous.clone(),
                            ..self.building.clone()
                        };
                        if let Err(e) = self.building.validate() {
                            self.building.floors = previous;
                            self.status = e;
                        } else if self.floor_changes_move_locked_contours(
                            &previous_building,
                            &self.building,
                            &BTreeSet::new(),
                        ) {
                            self.building.floors = previous;
                            self.status =
                                "Изменение этажа сдвинет объект с заблокированным перемещением."
                                    .into();
                        } else {
                            self.building_changed();
                            self.sync_building_objects();
                        }
                    }
                    if visibility_changed {
                        self.view_visibility_changed();
                    }
                    self.change_floor(chosen);
                    if let Some((id, show)) = floor_visibility_action {
                        self.set_floor_visibility_from_menu(id, show);
                    }
                    if let Some(id) = remove {
                        self.request_delete_floor(id);
                    }
                });
            });
    }
}

fn segment_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let d = b - a;
    let t = if d.length_sq() > 1e-10 {
        ((p - a).dot(d) / d.length_sq()).clamp(0., 1.)
    } else {
        0.
    };
    (p - (a + d * t)).length()
}
fn face_hit(points: &[Pos2], p: Pos2) -> bool {
    points
        .windows(2)
        .any(|w| segment_distance(p, w[0], w[1]) < 7.)
        || (points.len() > 2
            && geometry::inside(
                [p.x as f64, p.y as f64],
                &points
                    .iter()
                    .map(|p| [p.x as f64, p.y as f64])
                    .collect::<Vec<_>>(),
            ))
}
fn line_frame(a: Pos2, b: Pos2, r: Rect) -> bool {
    let mut lo = 0_f32;
    let mut hi = 1_f32;
    for (p, d, min, max) in [
        (a.x, b.x - a.x, r.left(), r.right()),
        (a.y, b.y - a.y, r.top(), r.bottom()),
    ] {
        if d.abs() < 1e-8 {
            if p < min || p > max {
                return false;
            }
        } else {
            let x = (min - p) / d;
            let y = (max - p) / d;
            lo = lo.max(x.min(y));
            hi = hi.min(x.max(y));
            if lo > hi {
                return false;
            }
        }
    }
    true
}
pub(super) fn face_frame(points: &[Pos2], r: Rect, window: bool) -> bool {
    if points.is_empty() {
        return false;
    }
    if window {
        return points.iter().all(|p| r.contains(*p));
    }
    if !Rect::from_points(points).intersects(r) {
        return false;
    }
    points.iter().any(|p| r.contains(*p))
        || points
            .iter()
            .zip(points.iter().cycle().skip(1))
            .any(|(&a, &b)| line_frame(a, b, r))
        || (points.len() > 2
            && geometry::inside(
                [r.center().x as f64, r.center().y as f64],
                &points
                    .iter()
                    .map(|p| [p.x as f64, p.y as f64])
                    .collect::<Vec<_>>(),
            ))
}

impl App {
    pub(super) fn spatial_source_interaction(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        _center_z: f64,
        projected: &[(Pos2, f64)],
        occlusion: Option<&spatial_results::Occlusion>,
    ) {
        self.spatial.hover_surface = Some((rect, ui.layer_id()));
        let at = response.hover_pos();
        let active_z = if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        };
        let plane = at.and_then(|at| {
            let origin = project([0., 0., active_z]).0;
            let x = project([1000., 0., active_z]).0 - origin;
            let y = project([0., 1000., active_z]).0 - origin;
            let d = x.x * y.y - x.y * y.x;
            if d.abs() < 1e-5 {
                return None;
            }
            let p = at - origin;
            Some([
                1000. * ((p.x * y.y - p.y * y.x) / d) as f64,
                1000. * ((x.x * p.y - x.y * p.x) / d) as f64,
            ])
        });
        if self.scaling_op.is_some() {
            if let Some(camera) = spatial_edit::plane_camera(project, active_z) {
                let saved = self.camera;
                let source =
                    at.and_then(|p| self.spatial_vertex_snap(p, project, projected, rect, None));
                self.transform_snap = source.map(|s| s.0);
                self.camera = camera;
                let preview = self.scaling_canvas(ui, response, rect, at);
                self.transform_snap = None;
                self.camera = saved;
                if let Some(preview) = preview {
                    self.spatial_transform_preview(
                        &ui.painter().with_clip_rect(rect),
                        project,
                        preview,
                    );
                }
                if let Some((_, world)) = source {
                    ui.painter().circle_stroke(
                        project(world).0,
                        5.,
                        Stroke::new(1.7_f32, Color32::from_rgb(70, 235, 225)),
                    );
                }
            } else {
                ui.colored_label(
                    Color32::LIGHT_BLUE,
                    "Для преобразования поверните вид к плоскости этажа",
                );
            }
            response.context_menu(|ui| self.scene_context_menu(ui));
            return;
        }
        if self.clipboard_op.is_some() {
            let snapped =
                at.and_then(|p| self.spatial_vertex_snap(p, project, projected, rect, None));
            if let Some(p) = snapped.map(|s| s.0.point).or(plane) {
                self.spatial_clipboard(ui, response, p, project, active_z, snapped.map(|s| s.1));
            } else if at.is_some() {
                ui.painter().text(
                    rect.center(),
                    egui::Align2::CENTER_CENTER,
                    "Выберите видимый узел или поверните вид к плоскости этажа",
                    egui::FontId::proportional(14.),
                    Color32::LIGHT_BLUE,
                );
            }
            return;
        }
        if self.building.enabled {
            for c in self.contours.iter().filter(|c| {
                self.contour_visible(c)
                    && !punching::is_punching(c)
                    && !c.kind.is_load()
                    && c.kind != Kind::Triangulation
                    && !matches!(
                        c.kind,
                        Kind::Slab
                            | Kind::Hole
                            | Kind::Support
                            | Kind::Beam
                            | Kind::Restraint
                            | Kind::RigidLink
                            | Kind::Foundation
                    )
            }) {
                let z = self.source_heights(c)[0];
                let ink = self.object_ink(c.id, c.kind);
                let mut path = c.points.clone();
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                for edge in path.windows(2) {
                    ui.painter().line_segment(
                        [
                            project([edge[0][0], edge[0][1], z]).0,
                            project([edge[1][0], edge[1][1], z]).0,
                        ],
                        Stroke::new(1.5_f32, ink),
                    );
                }
                if c.kind == Kind::LoadPoint {
                    if let Some(p) = path.first() {
                        ui.painter().circle_stroke(
                            project([p[0], p[1], z]).0,
                            5.,
                            Stroke::new(1.5_f32, ink),
                        );
                    }
                }
            }
        }
        let mut polygons = vec![];
        for c in self.contours.iter().filter(|c| self.selectable(c)) {
            let z = self.source_heights(c)[0];
            if matches!(c.kind, Kind::LoadArea | Kind::LoadLine) {
                let mut path = c.points.clone();
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                for e in path.windows(2) {
                    let [dx, dy, dz] = spatial_loads::offset(self, c);
                    polygons.push((
                        c.id,
                        [
                            [e[0][0], e[0][1], z],
                            [e[1][0], e[1][1], z],
                            [e[1][0] + dx, e[1][1] + dy, z + dz],
                            [e[0][0] + dx, e[0][1] + dy, z + dz],
                            [e[0][0], e[0][1], z],
                        ]
                        .map(project)
                        .to_vec(),
                    ));
                }
            } else if c.kind == Kind::LoadPoint {
                if let Some(p) = c.points.first() {
                    let anchor = [p[0], p[1], z];
                    let delta = spatial_loads::offset(self, c);
                    if let Some(load) = c.load.as_ref().filter(|l| l.moment) {
                        let axis = load.direction.index();
                        let tail = if axis == 2 { -delta[axis] } else { delta[axis] };
                        spatial_loads::arrow(anchor, axis, tail, true, |triangle| {
                            polygons.push((
                                c.id,
                                [triangle[0], triangle[1], triangle[2], triangle[0]]
                                    .map(project)
                                    .to_vec(),
                            ));
                        });
                    } else {
                        let tail = std::array::from_fn(|axis| anchor[axis] + delta[axis]);
                        polygons.push((c.id, vec![project(anchor), project(tail)]));
                    }
                }
            }
            let mut path = c.points.clone();
            if c.closed() && !path.is_empty() {
                path.push(path[0]);
            }
            if c.kind == Kind::Column {
                if let (Some(&p), Some(section)) =
                    (c.points.first(), self.support_height.columns.get(&c.id))
                {
                    let q = crate::plan_trace::columns::section_corners(c, section);
                    let heights = self.source_heights(c);
                    let lo = heights[0];
                    let hi = *heights.last().unwrap();
                    for i in 0..4 {
                        let a = q[i];
                        let b = q[(i + 1) % 4];
                        polygons.push((
                            c.id,
                            [
                                [a[0], a[1], lo],
                                [b[0], b[1], lo],
                                [b[0], b[1], hi],
                                [a[0], a[1], hi],
                                [a[0], a[1], lo],
                            ]
                            .map(project)
                            .to_vec(),
                        ));
                    }
                    for z in [lo, hi] {
                        let mut path = q.to_vec();
                        path.push(q[0]);
                        polygons.push((
                            c.id,
                            path.into_iter().map(|p| project([p[0], p[1], z])).collect(),
                        ));
                    }
                }
            } else if c.kind == Kind::Beam {
                let mut path = self.source_beam_path(c);
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                polygons.extend(
                    path.windows(2)
                        .map(|e| (c.id, vec![project(e[0]), project(e[1])])),
                );
            } else if matches!(c.kind, Kind::Support | Kind::RigidLink) {
                let heights = self.source_heights(c);
                let [lo, hi] = [heights[0], heights[1]];
                for edge in path.windows(2) {
                    let world = [
                        [edge[0][0], edge[0][1], lo],
                        [edge[1][0], edge[1][1], lo],
                        [edge[1][0], edge[1][1], hi],
                        [edge[0][0], edge[0][1], hi],
                        [edge[0][0], edge[0][1], lo],
                    ];
                    polygons.push((c.id, world.map(project).to_vec()));
                }
            } else {
                let points = path
                    .iter()
                    .map(|p| project([p[0], p[1], z]))
                    .collect::<Vec<_>>();
                if c.closed() || points.len() < 2 {
                    polygons.push((c.id, points));
                } else {
                    for edge in points.windows(2) {
                        polygons.push((c.id, edge.to_vec()));
                    }
                }
            }
        }
        if self.sections.active() {
            polygons = polygons
                .into_iter()
                .map(|(id, p)| (id, spatial_depth::clip_shape(p)))
                .filter(|(_, p)| !p.is_empty())
                .collect();
        }
        let mut selected_outline = spatial_depth::Builder::new(rect, projected);
        let mut candidates = BTreeMap::<u64, f64>::new();
        for (id, face) in &polygons {
            let points = face.iter().map(|p| p.0).collect::<Vec<_>>();
            if let Some(at) = at {
                if face_hit(&points, at)
                    && !self
                        .view_contour(*id)
                        .is_some_and(|c| c.kind.is_restraint())
                {
                    let depth = face.iter().map(|p| p.1).sum::<f64>() / face.len().max(1) as f64;
                    candidates
                        .entry(*id)
                        .and_modify(|d| *d = d.max(depth))
                        .or_insert(depth);
                }
            }
            if self.is_selected(*id)
                && !self.view_contour(*id).is_some_and(|c| {
                    punching::is_punching(c)
                        || c.kind.is_load()
                        || matches!(c.kind, Kind::Triangulation | Kind::Restraint)
                })
            {
                for edge in face.windows(2) {
                    selected_outline.line([edge[0], edge[1]], 2., Color32::YELLOW);
                }
            }
        }
        // Pick visible restraint lines ahead of their supporting plate face.
        spatial_depth::paint_layer(
            ui.painter(),
            rect,
            std::sync::Arc::new(selected_outline.vertices),
            spatial_depth::Layer::Restraints,
        );
        let mut restraint_hit = None::<(u64, f32, f64)>;
        if let Some(cursor) = at {
            for (id, face) in &polygons {
                if !self
                    .view_contour(*id)
                    .is_some_and(|c| c.kind.is_restraint())
                {
                    continue;
                }
                let point_edge;
                let face = if face.len() == 1 {
                    point_edge = [face[0]; 2];
                    &point_edge[..]
                } else {
                    face.as_slice()
                };
                for e in face.windows(2) {
                    let d = e[1].0 - e[0].0;
                    let t = if d.length_sq() > 1e-10 {
                        ((cursor - e[0].0).dot(d) / d.length_sq()).clamp(0., 1.)
                    } else {
                        0.
                    };
                    let p = e[0].0 + d * t;
                    let depth = e[0].1 + (e[1].1 - e[0].1) * t as f64;
                    let distance = p.distance(cursor);
                    if distance <= 7. && occlusion.is_none_or(|o| o.visible(p, depth)) {
                        candidates
                            .entry(*id)
                            .and_modify(|d| *d = d.max(depth))
                            .or_insert(depth);
                        if restraint_hit.is_none_or(|h| {
                            distance < h.1 || (distance - h.1).abs() < 0.1 && depth > h.2
                        }) {
                            restraint_hit = Some((*id, distance, depth));
                        }
                    }
                }
            }
        }
        if let Some((id, _, depth)) = restraint_hit {
            candidates.insert(id, depth);
        }
        let mut candidates = candidates.into_iter().collect::<Vec<_>>();
        candidates.sort_by(|a, b| {
            let restraint = restraint_hit.map(|h| h.0);
            (restraint == Some(b.0))
                .cmp(&(restraint == Some(a.0)))
                .then_with(|| b.1.total_cmp(&a.1))
                .then_with(|| a.0.cmp(&b.0))
        });
        let ids = candidates.iter().map(|c| c.0).collect::<Vec<_>>();
        if self.spatial.hover_ids != ids {
            self.spatial.hover_ids = ids;
            self.spatial.hover_index = 0;
        }
        let cycle = std::mem::take(&mut self.spatial.hover_cycle);
        if !candidates.is_empty() && response.hovered() && cycle != 0 {
            self.spatial.hover_index = (self.spatial.hover_index as isize + cycle)
                .rem_euclid(candidates.len() as isize)
                as usize;
            ui.ctx().request_repaint();
        }
        let mut hit = candidates.get(self.spatial.hover_index).copied();
        let hover = hit
            .filter(|h| {
                self.view_contour(h.0)
                    .is_some_and(|c| c.kind.is_restraint())
            })
            .map(|h| h.0);
        if self.spatial.restraint_hover != hover {
            self.spatial.restraint_hover = hover;
            ui.ctx().request_repaint();
        }
        let underlay_faces = self
            .underlays
            .iter()
            .filter(|u| self.underlay_selectable(u))
            .filter_map(|u| {
                let z = if self.building.enabled {
                    self.building
                        .floor(
                            self.building
                                .underlay_floors
                                .get(&u.id)
                                .copied()
                                .unwrap_or(self.building.floors[0].id),
                        )
                        .elevation_mm
                } else {
                    0.
                };
                Some((
                    u.id,
                    self.underlay_corners(u.id)?
                        .map(|p| project([p[0], p[1], z])),
                ))
            })
            .collect::<Vec<_>>();
        let mut underlay_hit = None::<(u64, f64)>;
        if let Some(at) = at {
            for (id, q) in &underlay_faces {
                for triangle in [[q[0], q[1], q[2]], [q[0], q[2], q[3]]] {
                    if let Some((_, depth)) = spatial_results::pick_triangle(at, triangle) {
                        if underlay_hit.is_none_or(|h| depth >= h.1) {
                            underlay_hit = Some((*id, depth));
                        }
                    }
                }
            }
            if let Some((_, depth)) = underlay_hit {
                if let Some((_, _, _, _, scene)) = &self.spatial.cache {
                    if projected.len() >= scene.xyz.len()
                        && !spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                            .visible(at, depth)
                    {
                        underlay_hit = None;
                    }
                }
            }
        }
        if underlay_hit.is_some_and(|u| hit.is_none_or(|h| u.1 > h.1 + 1e-7)) {
            hit = None;
        } else {
            underlay_hit = None;
        }
        if let Some((id, _)) = hit.filter(|_| {
            response.hovered()
                && !ui.input(|i| i.pointer.any_down())
                && self.mode.is_none()
                && !(self.show_results
                    && (self.selected_result().is_some() || self.punching_view.is_some()))
        }) {
            if let Some(c) = self.view_contour(id) {
                egui::show_tooltip_at_pointer(
                    ui.ctx(),
                    egui::LayerId::new(egui::Order::Tooltip, egui::Id::new("source_3d_tooltip")),
                    egui::Id::new(("source_3d_object", id)),
                    |ui| {
                        ui.label(format!("{} · {}", c.kind.label(), c.id));
                        if self.building.enabled {
                            let floor = self.building.floor(self.building.owner(c.id));
                            ui.label(format!(
                                "Этаж: {} · Z={} мм",
                                floor.name,
                                self.display_text.number(floor.elevation_mm)
                            ));
                        }
                        if !c.name.is_empty() {
                            ui.label(&c.name);
                        }
                        if let Some(entry) = self
                            .stiffnesses
                            .assignments
                            .get(&id)
                            .and_then(|id| self.stiffnesses.entries.get(id))
                        {
                            ui.label(format!("Жёсткость: {}", entry.name));
                            if let Some(value) = self.schema_hover(c.id) {
                                ui.label(value);
                            }
                        }
                        if candidates.len() > 1 {
                            ui.weak(format!(
                                "{} / {} · Tab / Shift+Tab",
                                self.spatial.hover_index + 1,
                                candidates.len()
                            ));
                        }
                    },
                );
            }
        }
        let hover = underlay_hit.map(|h| h.0);
        if self.spatial.underlay_hover != hover {
            self.spatial.underlay_hover = hover;
            ui.ctx().request_repaint();
        }
        if let Some((id, _)) = hit.filter(|(id, _)| {
            !self
                .view_contour(*id)
                .is_some_and(|c| c.kind.is_restraint())
        }) {
            for (_, face) in polygons.iter().filter(|p| p.0 == id) {
                for e in face.windows(2) {
                    for i in 0..8 {
                        let t0 = i as f32 / 8.;
                        let t1 = (i + 1) as f32 / 8.;
                        let mid = (t0 + t1) * 0.5;
                        let p = e[0].0.lerp(e[1].0, mid);
                        let depth = e[0].1 + (e[1].1 - e[0].1) * mid as f64;
                        if occlusion.is_none_or(|o| o.visible(p, depth)) {
                            ui.painter().line_segment(
                                [e[0].0.lerp(e[1].0, t0), e[0].0.lerp(e[1].0, t1)],
                                Stroke::new(1.5_f32, Color32::LIGHT_BLUE),
                            );
                        }
                    }
                }
            }
        }
        if !self.spatial.edit_hit && response.drag_started_by(egui::PointerButton::Primary) {
            self.spatial.frame_start = ui.input(|i| i.pointer.press_origin());
        }
        if let (Some(start), Some(end)) = (
            self.spatial.frame_start,
            ui.input(|i| i.pointer.interact_pos()),
        ) {
            let frame = Rect::from_two_pos(start, end);
            let window = start.x <= end.x;
            ui.painter().rect(
                frame,
                0.,
                Color32::from_rgba_unmultiplied(80, 160, 255, 25),
                Stroke::new(1.0_f32, Color32::LIGHT_BLUE),
                egui::StrokeKind::Inside,
            );
            if response.drag_stopped_by(egui::PointerButton::Primary) {
                self.fe_frame_3d(frame, window, projected, ui.input(|i| i.modifiers));
                let mut ids = std::collections::BTreeMap::<u64, Vec<Pos2>>::new();
                for (id, face) in &polygons {
                    ids.entry(*id).or_default().extend(face.iter().map(|p| p.0));
                }
                let selected = ids
                    .iter()
                    .filter(|(_, p)| {
                        if window {
                            face_frame(p, frame, true)
                        } else {
                            false
                        }
                    })
                    .map(|(id, _)| *id)
                    .chain(
                        polygons
                            .iter()
                            .filter(|(_, p)| {
                                !window
                                    && face_frame(
                                        &p.iter().map(|p| p.0).collect::<Vec<_>>(),
                                        frame,
                                        false,
                                    )
                            })
                            .map(|p| p.0),
                    )
                    .collect::<std::collections::BTreeSet<_>>();
                let underlays = underlay_faces
                    .iter()
                    .filter(|(_, q)| face_frame(&q.map(|p| p.0), frame, window))
                    .map(|(id, _)| *id)
                    .collect::<Vec<_>>();
                self.select_scene(selected, underlays, ui.input(|i| i.modifiers));
                self.spatial.frame_start = None;
            }
        }
        if !self.spatial.edit_hit && response.clicked_by(egui::PointerButton::Primary) {
            if let Some((id, _)) = underlay_hit {
                self.select_scene([], [id], ui.input(|i| i.modifiers));
            } else if let Some((id, _)) = hit {
                self.select_ids([id], ui.input(|i| i.modifiers));
            }
        }
        if response.clicked_by(egui::PointerButton::Secondary) {
            self.dxf_menu_target = underlay_hit.map(|h| h.0);
            self.load_menu_target = hit.and_then(|h| {
                self.contours
                    .iter()
                    .find(|c| c.id == h.0 && c.kind.is_load())
                    .map(|c| c.id)
            });
            self.support_menu_target = hit.and_then(|h| {
                self.contours
                    .iter()
                    .find(|c| c.id == h.0 && matches!(c.kind, Kind::Support | Kind::Column))
                    .map(|c| c.id)
            });
            if self.selected.is_none()
                && self.selection_ids.is_empty()
                && self.selected_underlay.is_none()
                && self.underlay_selection.is_empty()
                && self.measurements.selected.is_empty()
            {
                if let Some((id, _)) = hit {
                    self.select_only(Some(id));
                } else if let Some((id, _)) = underlay_hit {
                    self.select_scene([], [id], egui::Modifiers::NONE);
                }
            }
        }
        response.context_menu(|ui| self.scene_context_menu(ui));
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn two_floors() -> Building {
        let mut b = Building::default();
        b.enabled = true;
        b.floors.push(Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3200.,
            height_mm: 2800.,
        });
        b
    }
    #[test]
    fn floor_order_extents_validation_and_stable_ids() {
        let mut b = two_floors();
        b.floors.reverse();
        assert_eq!(b.extent(1), [0., 3200.]);
        assert_eq!(b.extent(2), [3200., 6000.]);
        assert!(b.validate().is_ok());
        b.floor(1);
        b.floors[0].elevation_mm = 0.;
        assert!(b.validate().is_err());
    }
    #[test]
    fn active_floor_filters_source_and_selection_but_3d_keeps_other_levels_visible() {
        let mut app = App::default();
        app.building = two_floors();
        for _ in 0..2 {
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([0., 0.], false);
            app.sketch_click([2000., 1500.], false);
            app.sync_building_objects();
            app.change_floor(2);
        }
        let first = app.contours[0].id;
        let second = app.contours[1].id;
        assert_eq!(app.building.owner(first), 1);
        assert_eq!(app.building.owner(second), 2);
        assert!(!app.contour_visible(&app.contours[0]));
        assert!(app.contour_visible(&app.contours[1]));
        assert_eq!(app.floor_contours().len(), 1);
        app.spatial.enabled = true;
        assert!(app.contour_visible(&app.contours[0]));
        app.select_only(Some(second));
        app.change_floor(1);
        assert!(app.selection_ids.is_empty());
        assert_eq!(app.source_heights(&app.contours[0]), vec![0.]);
        assert_eq!(app.source_heights(&app.contours[1]), vec![3200.]);
    }
    #[test]
    fn spatial_click_and_frame_select_only_active_floor_and_empty_click_preserves_selection() {
        let mut app = App::default();
        app.building = two_floors();
        app.spatial.enabled = true;
        for _ in 0..2 {
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([50., 50.], false);
            app.sketch_click([200., 150.], false);
            app.sync_building_objects();
            app.change_floor(2);
        }
        app.change_floor(1);
        app.select_only(None);
        let id = app.contours[0].id;
        let ctx = egui::Context::default();
        let mut frame = |app: &mut App, events: Vec<egui::Event>| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let (rect, response) = ui.allocate_exact_size(
                            ui.available_size(),
                            egui::Sense::click_and_drag(),
                        );
                        app.spatial_source_interaction(
                            ui,
                            &response,
                            rect,
                            &|p| (Pos2::new(p[0] as f32, p[1] as f32), p[2]),
                            0.,
                            &[],
                            None,
                        );
                    });
                },
            );
        };
        let button = |pos: Pos2, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: Default::default(),
        };
        frame(&mut app, vec![]);
        let at = Pos2::new(100., 100.);
        frame(
            &mut app,
            vec![egui::Event::PointerMoved(at), button(at, true)],
        );
        frame(&mut app, vec![button(at, false)]);
        assert!(app.is_selected(id));
        let empty = Pos2::new(350., 350.);
        frame(
            &mut app,
            vec![egui::Event::PointerMoved(empty), button(empty, true)],
        );
        frame(&mut app, vec![button(empty, false)]);
        assert!(app.is_selected(id));
        app.select_only(None);
        let from = Pos2::new(20., 20.);
        let to = Pos2::new(240., 200.);
        frame(
            &mut app,
            vec![egui::Event::PointerMoved(from), button(from, true)],
        );
        frame(&mut app, vec![egui::Event::PointerMoved(to)]);
        frame(&mut app, vec![button(to, false)]);
        assert_eq!(
            app.selection_ids.iter().copied().collect::<Vec<_>>(),
            vec![id]
        );
    }
    #[test]
    fn spatial_window_and_crossing_respect_whole_object_and_segment_limits() {
        let frame = Rect::from_min_max(Pos2::new(10., 10.), Pos2::new(20., 20.));
        let crossing = [Pos2::new(0., 15.), Pos2::new(30., 15.)];
        assert!(face_frame(&crossing, frame, false));
        assert!(!face_frame(&crossing, frame, true));
        assert!(!face_frame(
            &[Pos2::new(0., 15.), Pos2::new(5., 15.)],
            frame,
            false
        ));
        assert!(face_frame(
            &[Pos2::new(11., 11.), Pos2::new(19., 19.)],
            frame,
            true
        ));
    }
}
