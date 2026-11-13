//! Common and category-specific properties of the current selection.
use super::*;
use crate::plan_trace::{
    model::{LevelAnchor, LevelRef},
    stiffness::Family,
    underlay::Transform,
};
#[derive(Clone, Copy)]
pub(super) enum Item {
    Contour(u64),
    Underlay(u64),
    Dimension(u64),
}
impl Item {
    fn id(self) -> u64 {
        match self {
            Self::Contour(id) | Self::Underlay(id) | Self::Dimension(id) => id,
        }
    }
}
#[derive(Default)]
pub(super) struct State {
    pub open: bool,
    category: String,
    undo: BTreeMap<usize, Snapshot>,
    last_edit: Option<(egui::Id, usize)>,
    indices: BTreeMap<u64, usize>,
}
struct Snapshot {
    building: building::Building,
    assignments: BTreeMap<u64, u64>,
    materials: BTreeMap<u64, u64>,
    designs: BTreeMap<u64, u64>,
    keep_mesh: bool,
    contour_removal: bool,
    underlay_names: BTreeMap<u64, String>,
    underlay_transforms: BTreeMap<u64, Transform>,
}
#[derive(Clone, Copy)]
pub(super) enum UnderlayProperty {
    Insertion(usize),
    Scale(usize),
    Rotation,
}
impl UnderlayProperty {
    fn read(self, transform: Transform) -> f64 {
        match self {
            Self::Insertion(axis) => transform.origin[axis],
            Self::Scale(axis) => transform.scale[axis],
            Self::Rotation => transform.angle.to_degrees(),
        }
    }
    fn write(self, transform: &mut Transform, value: f64) {
        match self {
            Self::Insertion(axis) => transform.origin[axis] = value,
            Self::Scale(axis) => transform.scale[axis] = value,
            Self::Rotation => transform.angle = value.to_radians(),
        }
    }
}
pub(super) enum Change {
    LoadDirection(crate::plan_trace::loads::Direction),
    RestraintDof(u8, bool),
    Name(String),
    Floor(u64),
    Stiffness(u64),
    Material(u64),
    Design(u64),
    MeshSize(f64),
    EffectiveLength(bool),
    Anchor(usize, LevelAnchor),
    BeamLevel(Option<LevelAnchor>),
    Offset(usize, f64),
    LoadMagnitude(crate::plan_trace::loads::Magnitude),
    LoadAdditive(bool),
    LoadCase(u64),
    LoadMeshConstraint(bool),
    SlabRigidZ(f64),
    IgnoreOpening(bool),
    RigidOffset(usize, f64),
    CutOtherSlabs(bool),
    MovementLock(bool),
    UnderlayTransform(UnderlayProperty, f64),
}
fn common<T: PartialEq + Clone>(mut values: impl Iterator<Item = T>) -> Option<T> {
    let value = values.next()?;
    values.all(|v| v == value).then_some(value)
}
fn family(kind: Kind) -> Option<Family> {
    match kind {
        Kind::Slab => Some(Family::Slab),
        Kind::Support => Some(Family::Wall),
        Kind::Beam => Some(Family::Beam),
        Kind::Column => Some(Family::Column),
        _ => None,
    }
}
#[derive(Clone, Copy)]
struct Quantities {
    length_m: f64,
    area_m2: f64,
    volume_m3: f64,
    height_mm: Option<f64>,
}
fn path_length(points: &[Point], closed: bool) -> f64 {
    let open = points
        .windows(2)
        .map(|p| (p[1][0] - p[0][0]).hypot(p[1][1] - p[0][1]))
        .sum::<f64>();
    open + if closed && points.len() > 2 {
        let a = points[0];
        let b = *points.last().unwrap();
        (a[0] - b[0]).hypot(a[1] - b[1])
    } else {
        0.
    }
}
impl App {
    fn property_snapshot(&self, keep_mesh: bool) -> Snapshot {
        Snapshot {
            building: self.building.clone(),
            assignments: self.stiffnesses.assignments.clone(),
            materials: self.stiffnesses.contour_materials.clone(),
            designs: self.stiffnesses.contour_designs.clone(),
            keep_mesh,
            contour_removal: false,
            underlay_names: self
                .underlays
                .iter()
                .map(|u| (u.id, u.name.clone()))
                .collect(),
            underlay_transforms: BTreeMap::new(),
        }
    }
    pub(super) fn push_contour_property_undo(&mut self) {
        let snapshot = self.property_snapshot(false);
        self.push_property_snapshot(snapshot);
    }
    pub(super) fn push_contour_removal_undo(&mut self, affects_mesh: bool) {
        let mut snapshot = self.property_snapshot(!affects_mesh);
        snapshot.contour_removal = true;
        self.push_property_snapshot(snapshot);
    }
    fn push_property_snapshot(&mut self, snapshot: Snapshot) {
        self.properties.undo.insert(self.history.len(), snapshot);
        self.history.push(self.contours.clone());
        self.properties.last_edit = None;
    }
    fn remember_property_edit(&mut self, widget: egui::Id, keep_mesh: bool) -> usize {
        let index = self.history.len();
        if let Some((last_widget, last_index)) = self.properties.last_edit {
            if widget == last_widget && index == last_index + 1 {
                return last_index;
            }
        }
        self.properties
            .undo
            .insert(index, self.property_snapshot(keep_mesh));
        self.history.push(self.contours.clone());
        self.properties.last_edit = Some((widget, index));
        index
    }
    fn apply_underlay_property(
        &mut self,
        items: &[Item],
        property: UnderlayProperty,
        value: f64,
        widget: egui::Id,
    ) {
        let mut changes = Vec::with_capacity(items.len());
        for item in items {
            let Item::Underlay(id) = *item else {
                return;
            };
            let Some(index) = self.underlays.iter().position(|underlay| underlay.id == id) else {
                return;
            };
            let before = self.underlays[index].transform();
            let mut after = before;
            property.write(&mut after, value);
            if let Err(error) = after.validate() {
                self.status = error;
                return;
            }
            if after != before {
                changes.push((index, id, before, after));
            }
        }
        if changes.is_empty() {
            return;
        }
        let index = self.remember_property_edit(widget, true);
        let snapshot = self.properties.undo.get_mut(&index).unwrap();
        for (index, id, before, after) in changes {
            snapshot.underlay_transforms.entry(id).or_insert(before);
            self.underlays[index].set_transform(after);
        }
        self.sync_underlays();
    }
    fn property_quantities(&self, c: &Contour) -> Option<Quantities> {
        let thickness = match c.kind {
            Kind::Slab => {
                self.materials
                    .get(&c.id)
                    .copied()
                    .unwrap_or_default()
                    .thickness_mm
            }
            Kind::Support => {
                self.support_height
                    .materials
                    .get(&c.id)
                    .copied()
                    .unwrap_or_else(crate::plan_trace::fem::element::Material::wall_default)
                    .thickness_mm
            }
            _ => return None,
        };
        if c.kind == Kind::Slab {
            use mini_fem::geo::{Area, BooleanOps, LineString, MultiPolygon, Polygon};
            let polygon = |points: &[Point]| {
                Polygon::new(
                    LineString::from(
                        points
                            .iter()
                            .copied()
                            .map(|p| (p[0], p[1]))
                            .collect::<Vec<_>>(),
                    ),
                    vec![],
                )
            };
            let mut shape = MultiPolygon(vec![polygon(&c.points)]);
            for hole in self
                .contours
                .iter()
                .filter(|h| h.active_hole() && h.parent == Some(c.id))
            {
                shape = shape.difference(&polygon(&hole.points));
            }
            let mut perimeter = 0.;
            for polygon in &shape.0 {
                for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
                    perimeter += path_length(
                        &ring.0.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>(),
                        false,
                    );
                }
            }
            let area_m2 = shape.unsigned_area() / 1e6;
            Some(Quantities {
                length_m: perimeter / 1000.,
                area_m2,
                volume_m3: area_m2 * thickness / 1000.,
                height_mm: None,
            })
        } else {
            let heights = self.source_heights(c);
            let height = heights[1] - heights[0];
            let length = path_length(&c.points, c.closed());
            Some(Quantities {
                length_m: length / 1000.,
                area_m2: length * height.max(0.) / 1e6,
                volume_m3: length * height.max(0.) * thickness / 1e9,
                height_mm: Some(height),
            })
        }
    }
    fn property_quantity_rows(&self, ui: &mut egui::Ui, items: &[Item]) {
        let contours = items
            .iter()
            .filter_map(|i| match i {
                Item::Contour(id) => self.property_contour(*id),
                _ => None,
            })
            .collect::<Vec<_>>();
        if contours.len() != items.len() || contours.is_empty() {
            return;
        }
        if let Some(kind) = common(contours.iter().map(|c| c.kind))
            .filter(|k| matches!(k, Kind::LoadArea | Kind::LoadLine))
        {
            use mini_fem::geo::Area;
            let value = contours
                .iter()
                .map(|c| {
                    if kind == Kind::LoadArea {
                        crate::plan_trace::model::load_area_material(&self.contours, c)
                            .unsigned_area()
                            / 1e6
                    } else {
                        path_length(
                            &c.mesh_boundary(self.mesh_size)
                                .unwrap_or_else(|_| c.points.clone()),
                            c.closed(),
                        ) / 1000.
                    }
                })
                .sum::<f64>();
            ui.horizontal(|ui| {
                ui.label(if kind == Kind::LoadArea {
                    "Площадь, м²"
                } else {
                    "Длина, м"
                });
                ui.label(self.display_text.number(value));
            });
            return;
        }
        let values = contours
            .iter()
            .map(|c| self.property_quantities(c))
            .collect::<Option<Vec<_>>>();
        let Some(values) = values else {
            return;
        };
        let same_kind = common(contours.iter().map(|c| c.kind));
        let total = |f: fn(&Quantities) -> f64| values.iter().map(f).sum::<f64>();
        let sum = if values.len() > 1 {
            " (сумма)"
        } else {
            ""
        };
        egui::Grid::new("object_quantities")
            .num_columns(2)
            .spacing([12., 8.])
            .show(ui, |ui| {
                if let Some(kind) = same_kind {
                    ui.label(format!(
                        "{}{}, м",
                        if kind == Kind::Slab {
                            "Периметр"
                        } else {
                            "Длина"
                        },
                        sum
                    ))
                    .on_hover_text(if kind == Kind::Slab {
                        "Внешняя граница и границы отверстий"
                    } else {
                        "Длина открытой или замкнутой оси стены"
                    });
                    ui.label(self.display_text.number(total(|v| v.length_m)));
                    ui.end_row();
                    if kind == Kind::Support {
                        ui.label("Высота, мм");
                        let height = common(values.iter().map(|v| v.height_mm));
                        ui.label(
                            height.flatten().map_or_else(
                                || "Различные".into(),
                                |h| self.display_text.number(h),
                            ),
                        );
                        ui.end_row();
                    }
                    ui.label(format!(
                        "{}{}, м²",
                        if kind == Kind::Slab {
                            "Площадь"
                        } else {
                            "Площадь поверхности"
                        },
                        sum
                    ))
                    .on_hover_text(if kind == Kind::Slab {
                        "Площадь плиты за вычетом отверстий"
                    } else {
                        "Длина × фактическая высота; одна срединная поверхность стены"
                    });
                    ui.label(self.display_text.number(total(|v| v.area_m2)));
                    ui.end_row();
                }
                ui.label(format!("Объём{sum}, м³"));
                ui.label(self.display_text.number(total(|v| v.volume_m3)));
                ui.end_row();
            });
    }
    fn property_contour(&self, id: u64) -> Option<&Contour> {
        self.properties
            .indices
            .get(&id)
            .and_then(|&i| self.contours.get(i))
            .filter(|c| c.id == id)
            .or_else(|| self.contours.iter().find(|c| c.id == id))
    }
    pub(super) fn property_undo_locked(&self, index: usize) -> bool {
        let Some(snapshot) = self.properties.undo.get(&index) else {
            return false;
        };
        if !snapshot.underlay_transforms.is_empty() {
            return self.input_locked || self.underlays_locked;
        }
        self.input_locked
            || self.contours.iter().any(|c| {
                self.is_kind_locked(c.kind)
                    && (self.building.owner(c.id) != snapshot.building.owner(c.id)
                        || self.stiffnesses.assignments.get(&c.id)
                            != snapshot.assignments.get(&c.id)
                        || self.stiffnesses.contour_materials.get(&c.id)
                            != snapshot.materials.get(&c.id)
                        || self.stiffnesses.contour_designs.get(&c.id)
                            != snapshot.designs.get(&c.id))
            })
    }
    pub(super) fn used_material_ids(&self) -> std::collections::BTreeSet<u64> {
        self.stiffnesses
            .contour_materials
            .values()
            .copied()
            .chain(
                self.properties
                    .undo
                    .values()
                    .flat_map(|s| s.materials.values().copied()),
            )
            .collect()
    }
    fn property_name(&self, item: Item) -> String {
        match item {
            Item::Contour(id) => self.property_contour(id).unwrap().display_name(),
            Item::Underlay(id) => self
                .underlays
                .iter()
                .find(|u| u.id == id)
                .unwrap()
                .name
                .clone(),
            Item::Dimension(id) => format!("Размер {id}"),
        }
    }
    fn property_floor(&self, item: Item) -> Option<u64> {
        match item {
            Item::Contour(id) => Some(self.building.owner(id)),
            Item::Underlay(id) => Some(
                self.building
                    .underlay_floors
                    .get(&id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id),
            ),
            Item::Dimension(_) => None,
        }
    }
    fn property_items(&self) -> Vec<Item> {
        self.contours
            .iter()
            .filter(|c| self.is_selected(c.id))
            .map(|c| Item::Contour(c.id))
            .chain(
                self.underlays
                    .iter()
                    .filter(|u| {
                        self.selected_underlay == Some(u.id)
                            || self.underlay_selection.contains(&u.id)
                    })
                    .map(|u| Item::Underlay(u.id)),
            )
            .chain(
                self.measurements
                    .items
                    .iter()
                    .filter(|m| self.measurements.selected.contains(&m.id))
                    .map(|m| Item::Dimension(m.id)),
            )
            .collect()
    }
    fn property_category(&self, item: Item) -> String {
        match item {
            Item::Contour(id) => self.property_contour(id).unwrap().kind.label().into(),
            Item::Underlay(id) => if matches!(
                self.underlays.iter().find(|u| u.id == id).unwrap().data,
                crate::plan_trace::underlay::Data::Dxf(_)
            ) {
                "Подложки DXF"
            } else {
                "Подложки PDF"
            }
            .into(),
            Item::Dimension(_) => "Размеры".into(),
        }
    }
    fn property_editable(&self, items: &[Item]) -> bool {
        !self.input_locked
            && self.mesh_job.is_none()
            && self.analysis_job.is_none()
            && self.alternative_job.is_none()
            && items.iter().all(|item| match *item {
                Item::Contour(id) => self
                    .property_contour(id)
                    .is_some_and(|c| !self.is_kind_locked(c.kind)),
                Item::Underlay(_) => !self.underlays_locked,
                Item::Dimension(_) => false,
            })
    }
    pub(super) fn property_apply(&mut self, items: &[Item], change: Change, widget: egui::Id) {
        if items.is_empty() || !self.property_editable(items) {
            return;
        }
        if matches!(
            change,
            Change::Floor(_)
                | Change::Anchor(..)
                | Change::BeamLevel(_)
                | Change::Offset(..)
                | Change::SlabRigidZ(_)
                | Change::RigidOffset(..)
        ) {
            let locked = self.movement_locked_contours();
            if items
                .iter()
                .any(|item| matches!(item, Item::Contour(id) if locked.contains(id)))
            {
                self.status = "У выбранного объекта заблокировано перемещение.".into();
                return;
            }
        }
        if matches!(change, Change::MovementLock(_))
            && !items.iter().all(|item| matches!(item, Item::Contour(_)))
        {
            return;
        }
        if let Change::UnderlayTransform(property, value) = change {
            self.apply_underlay_property(items, property, value, widget);
            return;
        }
        if matches!(&change, Change::Name(name) if name.len() > 1024) {
            return;
        }
        if let Change::MeshSize(v) = change {
            if !v.is_finite() || v != 0. && !(100. ..=1e6).contains(&v) {
                return;
            }
        }
        if let Change::Offset(slot, v) = change {
            if !v.is_finite()
                || v.abs() > 1e6
                || !items.iter().any(|item| {
                    matches!(item, Item::Contour(id) if self.property_contour(*id)
                        .is_some_and(|c| level_ref_read(c, slot).offset_mm != v))
                })
            {
                return;
            }
        }
        if let Change::BeamLevel(anchor) = change {
            if !self.building.enabled
                || items.iter().any(|item| {
                    let Item::Contour(id) = *item else {
                        return true;
                    };
                    self.property_contour(id).is_none_or(|c| {
                        c.kind != Kind::Beam
                            || (anchor.is_none() && (c.closed() || !c.curves.is_empty()))
                            || (anchor.is_some()
                                && c.is_free_beam()
                                && c.points
                                    .windows(2)
                                    .any(|p| (p[0][0] - p[1][0]).hypot(p[0][1] - p[1][1]) < 1e-6))
                    })
                })
            {
                self.status = "Свободная балка должна быть открытой ломаной. Для привязки к этажу соседние вершины должны различаться в плане.".into();
                return;
            }
        }
        if let Change::Floor(floor) = change {
            if !self.building.floors.iter().any(|f| f.id == floor) {
                return;
            }
        }
        if let Change::Stiffness(id) = change {
            let Some(entry) = self.stiffnesses.entries.get(&id) else {
                return;
            };
            if items.iter().any(|item| match item {
                Item::Contour(id) => {
                    self.property_contour(*id)
                        .and_then(|c| family(c.kind))
                        .map(Family::section_family)
                        != Some(entry.family.section_family())
                }
                _ => true,
            }) {
                return;
            }
        }
        if let Change::Material(id) = change {
            if !self.stiffnesses.elastic.contains_key(&id) || items.iter().any(|item| {
                !matches!(item, Item::Contour(id) if self.property_contour(*id).is_some_and(|c| family(c.kind).is_some()))
            }) { return; }
        }
        if let Change::Design(id) = change {
            if !self.stiffnesses.designs.get(&id).is_some_and(|d| !d.bar)
                || items.iter().any(|item| {
                    !matches!(item, Item::Contour(id) if self.property_contour(*id)
                        .is_some_and(|c| matches!(c.kind, Kind::Slab | Kind::Support)))
                })
            {
                return;
            }
        }
        if matches!(&change, Change::SlabRigidZ(v) if !v.is_finite() || v.abs() > 1e6) {
            return;
        }
        if let Change::LoadCase(case) = &change {
            if !self.load_cases.iter().any(|c| c.id == *case)
                || items.iter().any(|item| !matches!(item, Item::Contour(id)
                    if self.property_contour(*id).is_some_and(|c| c.load.is_some() && c.parent.is_none())))
                || items.iter().any(|item| matches!(item, Item::Contour(id)
                    if self.contours.iter().any(|c| c.parent == Some(*id) && self.is_kind_locked(c.kind))))
            {
                return;
            }
        }
        if let Change::LoadMagnitude(value) = &change {
            if !value.value.is_finite()
                || !value
                    .in_units(
                        crate::plan_trace::loads::ForceUnits::KiloNewton,
                        self.gravity,
                    )
                    .is_finite()
            {
                return;
            }
        }
        if let Change::LoadMeshConstraint(enabled) = change {
            let mut changed = false;
            for item in items {
                let Item::Contour(id) = *item else {
                    return;
                };
                let Some(load) = self
                    .property_contour(id)
                    .filter(|c| c.kind.is_load() && c.kind != Kind::LoadHole)
                    .and_then(|c| c.load.as_ref())
                else {
                    return;
                };
                changed |= load.mesh_constraint != enabled;
            }
            if !changed {
                return;
            }
        }
        if matches!(change, Change::MovementLock(_)) {
            self.push_property_snapshot(self.property_snapshot(true));
        } else if matches!(
            change,
            Change::Anchor(_, _) | Change::BeamLevel(_) | Change::LoadMeshConstraint(_)
        ) {
            self.push_contour_property_undo();
        } else {
            self.remember_property_edit(widget, matches!(change, Change::Design(_)));
        }
        let mut ids = items
            .iter()
            .filter_map(|item| match item {
                Item::Contour(id) => Some(*id),
                _ => None,
            })
            .collect::<std::collections::BTreeSet<_>>();
        if matches!(change, Change::Floor(_) | Change::LoadCase(_)) {
            loop {
                let old = ids.len();
                for c in &self.contours {
                    if c.parent.is_some_and(|id| ids.contains(&id)) {
                        ids.insert(c.id);
                    }
                }
                if ids.len() == old {
                    break;
                }
            }
        }
        for c in self.contours.iter_mut().filter(|c| ids.contains(&c.id)) {
            match &change {
                Change::RestraintDof(bit, enabled) => {
                    if c.kind.is_restraint() {
                        if *enabled {
                            c.restraint_mask |= bit;
                        } else {
                            c.restraint_mask &= !bit;
                        }
                    }
                }
                Change::Name(name) => {
                    c.name = name.clone();
                    if let Some(load) = &mut c.load {
                        load.name = name.clone();
                    }
                }
                Change::Floor(floor) => {
                    self.building.contour_floors.insert(c.id, *floor);
                }
                Change::SlabRigidZ(value) if c.kind == Kind::Slab => {
                    c.placement.slab_rigid_z_mm = *value
                }
                Change::IgnoreOpening(value) if c.kind == Kind::Hole => {
                    c.placement.ignore_opening = *value
                }
                Change::SlabRigidZ(_) | Change::IgnoreOpening(_) => {}
                Change::LoadDirection(direction) => {
                    if let Some(load) = &mut c.load {
                        load.direction = *direction;
                    }
                }
                Change::LoadCase(case) => {
                    if let Some(load) = &mut c.load {
                        load.case_id = *case;
                    }
                }
                Change::LoadMagnitude(value) => {
                    if let Some(load) = &mut c.load {
                        load.magnitude = *value;
                    }
                }
                Change::LoadAdditive(value) => {
                    if c.kind == Kind::LoadArea {
                        if let Some(load) = &mut c.load {
                            load.additive = *value;
                        }
                    }
                }
                Change::LoadMeshConstraint(enabled) => {
                    if let Some(load) = &mut c.load {
                        load.mesh_constraint = *enabled;
                    }
                }
                Change::MeshSize(value) => c.mesh_size_mm = *value,
                Change::CutOtherSlabs(v) => {
                    if c.kind == Kind::Slab {
                        c.placement.cut_other_slabs = *v;
                    }
                }
                Change::MovementLock(locked) => c.placement.movement_locked = *locked,
                Change::RigidOffset(axis, value) => c.placement.rigid_yz_mm[*axis] = *value,
                Change::EffectiveLength(value) => {
                    if matches!(c.kind, Kind::Support | Kind::Column) {
                        c.calculate_effective_length = *value;
                    }
                }
                Change::Stiffness(id) => {
                    if matches!(c.kind, Kind::Slab | Kind::Support) {
                        if let Some(design) = self.stiffnesses.design_id(c.id) {
                            self.stiffnesses
                                .contour_designs
                                .entry(c.id)
                                .or_insert(design);
                        }
                    }
                    self.stiffnesses.assignments.insert(c.id, *id);
                }
                Change::Material(id) => {
                    self.stiffnesses.contour_materials.insert(c.id, *id);
                }
                Change::Design(id) => {
                    self.stiffnesses.contour_designs.insert(c.id, *id);
                }
                Change::Anchor(slot, value) => level_ref(c, *slot).anchor = *value,
                Change::BeamLevel(anchor) => {
                    let extent = self.building.extent(self.building.owner(c.id));
                    if let Some(anchor) = anchor {
                        if let Some(z) = c.placement.beam_z_mm.take() {
                            let origin = LevelRef {
                                anchor: *anchor,
                                offset_mm: 0.,
                            }
                            .z(extent);
                            c.placement.slab.offset_mm = z[0] - origin;
                        }
                        c.placement.slab.anchor = *anchor;
                    } else if !c.is_free_beam() {
                        c.placement.beam_z_mm =
                            Some(vec![c.placement.slab.z(extent); c.points.len()]);
                    }
                }
                Change::Offset(slot, value) => level_ref(c, *slot).offset_mm = *value,
                Change::UnderlayTransform(_, _) => {}
            }
        }
        for item in items {
            if let Item::Underlay(id) = *item {
                match &change {
                    Change::Name(name) => {
                        if let Some(u) = self.underlays.iter_mut().find(|u| u.id == id) {
                            u.name = name.clone();
                        }
                    }
                    Change::Floor(floor) => {
                        self.building.underlay_floors.insert(id, *floor);
                    }
                    _ => {}
                }
            }
        }
        if matches!(change, Change::RestraintDof(..)) {
            self.poll_analysis();
            self.alternative_cache = Default::default();
            self.spatial.cache_reset();
        } else if matches!(
            change,
            Change::LoadMagnitude(_)
                | Change::LoadAdditive(_)
                | Change::LoadCase(_)
                | Change::LoadDirection(_)
        ) {
            self.analysis = None;
            self.analysis_input = None;
            self.combination_cache = None;
            self.source_view = None;
            self.load_scale_key = None;
            self.spatial.cache_reset();
        } else if matches!(change, Change::LoadMeshConstraint(_)) {
            self.clear_mesh();
        } else if !matches!(
            change,
            Change::Name(_)
                | Change::EffectiveLength(_)
                | Change::Design(_)
                | Change::MovementLock(_)
        ) {
            if matches!(change, Change::Floor(_)) {
                self.sync_building_objects();
            }
            self.sync_stiffness_sources();
            self.clear_mesh();
            self.rebuild_index();
        }
        self.slab_cache = None;
        self.invalidate_scene_buffers();
    }
    pub(super) fn property_undo(&mut self, index: usize) -> bool {
        let Some(snapshot) = self.properties.undo.remove(&index) else {
            return false;
        };
        self.properties.last_edit = None;
        if !snapshot.underlay_transforms.is_empty() {
            for underlay in &mut self.underlays {
                if let Some(&transform) = snapshot.underlay_transforms.get(&underlay.id) {
                    underlay.set_transform(transform);
                }
            }
            self.sync_underlays();
            return false;
        }
        if snapshot.contour_removal {
            self.building.contour_floors = snapshot.building.contour_floors;
        } else {
            self.building = snapshot.building;
        }
        self.stiffnesses.assignments = snapshot.assignments;
        self.stiffnesses.contour_materials = snapshot.materials;
        self.stiffnesses.contour_designs = snapshot.designs;
        if !snapshot.contour_removal {
            for u in &mut self.underlays {
                if let Some(name) = snapshot.underlay_names.get(&u.id) {
                    u.name = name.clone();
                }
            }
        }
        !snapshot.keep_mesh
    }
    pub(super) fn object_properties_window(&mut self, ctx: &egui::Context) {
        if !self.properties.open {
            return;
        }
        self.properties.indices = self
            .contours
            .iter()
            .enumerate()
            .map(|(i, c)| (c.id, i))
            .collect();
        let items = self.property_items();
        let mut groups = BTreeMap::<String, Vec<Item>>::new();
        for &item in &items {
            groups
                .entry(self.property_category(item))
                .or_default()
                .push(item);
        }
        if !self.properties.category.is_empty() && !groups.contains_key(&self.properties.category) {
            self.properties.category.clear();
        }
        let mut open = true;
        egui::Window::new(format!("Свойства · {} объектов", items.len()))
            .id(egui::Id::new("object_properties_window"))
            .order(egui::Order::Foreground)
            .open(&mut open)
            .default_width(420.)
            .resizable(true)
            .vscroll(true)
            .show(ctx, |ui| {
                if items.is_empty() {
                    ui.label("Выберите объект или выделите несколько рамкой.");
                    return;
                }
                if groups.len() > 1 {
                    egui::ComboBox::from_id_salt("property_category")
                        .selected_text(if self.properties.category.is_empty() {
                            "Все"
                        } else {
                            &self.properties.category
                        })
                        .show_ui(ui, |ui| {
                            ui.selectable_value(
                                &mut self.properties.category,
                                String::new(),
                                "Все",
                            );
                            for (name, items) in &groups {
                                ui.selectable_value(
                                    &mut self.properties.category,
                                    name.clone(),
                                    format!("{name} ({})", items.len()),
                                );
                            }
                        });
                }
                if !self.properties.category.is_empty() {
                    let selected = groups[&self.properties.category].clone();
                    self.property_rows(ui, &selected, true, true);
                } else if groups.len() == 1 {
                    self.property_rows(ui, &items, true, true);
                } else {
                    self.property_rows(ui, &items, true, false);
                    for (name, selected) in groups {
                        ui.collapsing(format!("{name} ({})", selected.len()), |ui| {
                            self.property_rows(ui, &selected, false, true)
                        });
                    }
                }
            });
        self.properties.open = open;
    }
    fn underlay_property_rows(
        &self,
        ui: &mut egui::Ui,
        items: &[Item],
        patches: &mut Vec<(Change, egui::Id)>,
    ) {
        let transforms = items
            .iter()
            .filter_map(|item| match *item {
                Item::Underlay(id) => self
                    .underlays
                    .iter()
                    .find(|underlay| underlay.id == id)
                    .map(|underlay| underlay.transform()),
                _ => None,
            })
            .collect::<Vec<_>>();
        if transforms.is_empty() || transforms.len() != items.len() {
            return;
        }
        for (property, label, speed, default) in [
            (
                UnderlayProperty::Insertion(0),
                "Точка вставки X, мм",
                1.,
                0.,
            ),
            (
                UnderlayProperty::Insertion(1),
                "Точка вставки Y, мм",
                1.,
                0.,
            ),
            (UnderlayProperty::Scale(0), "Масштаб по X", 0.01, 1.),
            (UnderlayProperty::Scale(1), "Масштаб по Y", 0.01, 1.),
            (UnderlayProperty::Rotation, "Угол поворота, °", 0.5, 0.),
        ] {
            ui.label(label);
            let shared = common(transforms.iter().copied().map(|t| property.read(t)));
            let mut value = shared.unwrap_or(default);
            let mut control = egui::DragValue::new(&mut value)
                .speed(speed)
                .update_while_editing(false)
                .custom_formatter(move |number, decimals| {
                    shared.map_or_else(
                        || "Различные".into(),
                        |_| egui::emath::format_with_decimals_in_range(number, decimals),
                    )
                });
            if matches!(property, UnderlayProperty::Scale(_)) {
                control = control.range(1e-9..=1e9);
            }
            let response = ui.add(control);
            if response.changed() {
                patches.push((Change::UnderlayTransform(property, value), response.id));
            }
            ui.end_row();
        }
    }
    fn property_rows(&mut self, ui: &mut egui::Ui, items: &[Item], general: bool, specific: bool) {
        let ids = items.iter().map(|item| item.id()).collect::<Vec<_>>();
        let locked = self.movement_locked_contours();
        let can_move = items
            .iter()
            .all(|item| !matches!(item, Item::Contour(id) if locked.contains(id)));
        ui.push_id((&ids,general,specific),|ui|{
            let editable=self.property_editable(items);let mut patches=vec![];
            let mut finished_offset_edits = vec![];
            ui.add_enabled_ui(editable,|ui|{
                egui::Grid::new("object_property_grid").num_columns(2).spacing([12.,8.]).show(ui,|ui|{
                    if general {
                        ui.label("Наименование");
                        let value=common(items.iter().map(|&item|self.property_name(item)));
                        let draft_id=ui.id().with("name_draft");
                        let(mut draft,mut dirty)=ui.ctx().data(|d|d.get_temp::<(String,bool)>(draft_id)).unwrap_or_else(||(value.clone().unwrap_or_default(),false));
                        if !dirty {draft=value.clone().unwrap_or_default();}
                        let response=ui.add(egui::TextEdit::singleline(&mut draft).hint_text("Различные").char_limit(1024));
                        dirty|=response.changed();
                        if dirty&&(response.lost_focus()||response.has_focus()&&ui.input(|i|i.key_pressed(egui::Key::Enter))) {patches.push((Change::Name(draft.clone()),response.id));dirty=false;}
                        ui.ctx().data_mut(|d|d.insert_temp(draft_id,(draft,dirty)));ui.end_row();
                        ui.label("Этаж");
                        if items.iter().all(|&i|self.property_floor(i).is_some()) {
                            let common=common(items.iter().map(|&i|self.property_floor(i).unwrap()));let mut value=common.unwrap_or(0);
                            let allow=can_move&&items.iter().all(|item|match item {Item::Contour(id)=>{
                                let contour = self.property_contour(*id).unwrap();
                                contour.placement.rigid_link_targets.is_none() && contour.parent.is_none_or(|p|items.iter().any(|i|matches!(i,Item::Contour(id) if *id==p)))
                            },_=>true});
                            let response=ui.add_enabled_ui(allow,|ui|egui::ComboBox::from_id_salt("floor").selected_text(self.building.floors.iter().find(|f|f.id==value).map_or("Различные",|f|f.name.as_str())).show_ui(ui,|ui|{
                                for f in &self.building.floors {ui.selectable_value(&mut value,f.id,&f.name);}
                            })).inner.response;
                            if value!=common.unwrap_or(0){patches.push((Change::Floor(value),response.id));}
                        }else{ui.label("Не задан");}ui.end_row();
                        if items.iter().all(|item| matches!(item, Item::Contour(_))) {
                            let value = common(ids.iter().map(|id| self.property_contour(*id).unwrap().placement.movement_locked));
                            let mut movement_locked = value.unwrap_or(false);
                            ui.label("Блокировать перемещение");
                            let response = ui.add(egui::Checkbox::new(&mut movement_locked, "").indeterminate(value.is_none()))
                                .on_hover_text("Фиксирует положение объекта и его вершин при редактировании и округлении координат");
                            if response.changed() { patches.push((Change::MovementLock(movement_locked), response.id)); }
                            ui.end_row();
                        }
                    }
                    self.underlay_property_rows(ui, items, &mut patches);
                    let contours=items.iter().filter_map(|item|match item {Item::Contour(id)=>self.property_contour(*id),_=>None}).collect::<Vec<_>>();
                    if contours.len()==items.len()&&!contours.is_empty() {
                        if contours.iter().all(|c| c.load.is_some()) {
                            ui.label("Загружение");
                            let shared = common(contours.iter().map(|c| c.load.as_ref().unwrap().case_id));
                            let mut value = shared;
                            let response = ui.add_enabled_ui(contours.iter().all(|c| c.parent.is_none()), |ui| {
                                egui::ComboBox::from_id_salt("property_load_case")
                                    .selected_text(self.load_cases.iter().find(|c| Some(c.id) == value)
                                        .map_or_else(|| "Различные".into(), |c| c.display_name()))
                                    .show_ui(ui, |ui| {
                                        for case in &self.load_cases {
                                            ui.selectable_value(&mut value, Some(case.id), case.display_name());
                                        }
                                    })
                            }).inner.response;
                            if let Some(case) = value.filter(|_| value != shared) {
                                patches.push((Change::LoadCase(case), response.id));
                            }
                            ui.end_row();
                            if contours.iter().all(|c| c.kind.is_load() && c.kind != Kind::LoadHole) {
                                use crate::plan_trace::loads::{MESH_CONSTRAINT_HELP, MESH_CONSTRAINT_LABEL};

                                ui.label(MESH_CONSTRAINT_LABEL);
                                let shared = common(contours.iter().map(|c| c.load.as_ref().unwrap().mesh_constraint));
                                let mut enabled = shared.unwrap_or(false);
                                let response = ui.add(egui::Checkbox::without_text(&mut enabled).indeterminate(shared.is_none()))
                                    .on_hover_text(MESH_CONSTRAINT_HELP);
                                if response.changed() {
                                    patches.push((Change::LoadMeshConstraint(enabled), response.id));
                                }
                                ui.end_row();
                            }
                            if let Some(dimension) = common(contours.iter().map(|c| c.load_dimension()))
                                .filter(|_| contours.iter().all(|c| c.kind != Kind::LoadHole)) {
                                ui.label(format!("Интенсивность (расчётная), {}", self.load_units.intensity_label(dimension)));
                                let value=common(contours.iter().map(|c|c.load.as_ref().unwrap().magnitude.in_units(self.load_units,self.gravity)));
                                let mut number=value.unwrap_or(0.);
                                let response=ui.add(egui::DragValue::new(&mut number).speed(0.1).update_while_editing(false)
                                    .custom_formatter(move|v,_|value.map_or_else(||"Различные".into(),|_|format!("{v:.4}"))));
                                if response.changed() {patches.push((Change::LoadMagnitude(crate::plan_trace::loads::Magnitude {value:number,units:self.load_units}),response.id));}ui.end_row();
                                if dimension == 2 {
                                    ui.label("Наложение");
                                    let value=common(contours.iter().map(|c|c.load.as_ref().unwrap().additive));let mut enabled=value.unwrap_or(false);
                                    let response=ui.add(egui::Checkbox::new(&mut enabled,"").indeterminate(value.is_none())).on_hover_text("Прибавлять эту нагрузку к максимальной из обычных нагрузок того же загружения. Несколько отмеченных нагрузок суммируются.");
                                    if response.changed() {patches.push((Change::LoadAdditive(enabled),response.id));}ui.end_row();
                                }
                            }
                        }
                        if contours.iter().all(|c| c.kind == Kind::Hole) {
                            ui.label("Не учитывать");
                            let shared = common(contours.iter().map(|c| c.placement.ignore_opening));
                            let mut value = shared.unwrap_or(false);
                            let r = ui.add(egui::Checkbox::new(&mut value, "").indeterminate(shared.is_none()));
                            if r.changed() { patches.push((Change::IgnoreOpening(value), r.id)); }
                            ui.end_row();
                        }
                        if contours.iter().all(|c| c.kind == Kind::Slab) {
                            ui.label("Абсолютно жёсткая вставка Z, мм");
                            let mut value = common(contours.iter().map(|c| c.placement.slab_rigid_z_mm)).unwrap_or(0.);
                            let r = ui.add_enabled(can_move, egui::DragValue::new(&mut value).range(-1e6..=1e6).update_while_editing(false));
                            if r.changed() { patches.push((Change::SlabRigidZ(value), r.id)); }
                            ui.end_row();
                        }
                        if contours.iter().all(|c|family(c.kind).is_some()) {
                            ui.label("Жёсткость");
                            let kinds=common(contours.iter().map(|c|family(c.kind).unwrap().section_family()));
                            let shared=common(contours.iter().map(|c|self.stiffnesses.assignments.get(&c.id).copied().unwrap_or(0)));let mut value=shared.unwrap_or(0);
                            let response=ui.add_enabled_ui(kinds.is_some(),|ui|egui::ComboBox::from_id_salt("stiffness").selected_text(self.stiffnesses.entries.get(&value).map_or("Различные",|e|e.name.as_str())).show_ui(ui,|ui|{
                                for(&id,e) in self.stiffnesses.entries.iter().filter(|(_,e)|Some(e.family.section_family())==kinds){ui.selectable_value(&mut value,id,&e.name);}
                            })).inner.response;
                            if value!=shared.unwrap_or(0){patches.push((Change::Stiffness(value),response.id));}ui.end_row();
                            ui.label("Материал");
                            let shared = common(contours.iter().map(|c| self.stiffnesses.material_id(c.id).unwrap_or(0)));
                            let mut value = shared.unwrap_or(0);
                            let response = self.material_combo(ui, "material", &mut value);
                            if value != shared.unwrap_or(0) { patches.push((Change::Material(value), response.id)); }
                            ui.end_row();
                        }
                        if contours.iter().all(|c| matches!(c.kind, Kind::Slab | Kind::Support)) {
                            ui.label("Конструирование");
                            let shared = common(contours.iter().map(|c| self.stiffnesses.design_id(c.id)));
                            let mut value = shared.flatten();
                            let response = egui::ComboBox::from_id_salt("property_plate_design")
                                .selected_text(value.and_then(|id| self.stiffnesses.designs.get(&id))
                                    .map_or("Различные", |d| d.name.as_str()))
                                .show_ui(ui, |ui| {
                                    for (&id, design) in self.stiffnesses.designs.iter().filter(|(_, d)| !d.bar) {
                                        ui.selectable_value(&mut value, Some(id), &design.name);
                                    }
                                }).response;
                            if let Some(id) = value.filter(|_| value != shared.flatten()) {
                                patches.push((Change::Design(id), response.id));
                            }
                            ui.end_row();
                        }
                        if contours.iter().all(|c|c.kind==Kind::Slab) {
                            ui.label("Вырезать из других плит");
                            let shared=common(contours.iter().map(|c|c.placement.cut_other_slabs));let mut value=shared.unwrap_or(false);
                            let r=ui.add(egui::Checkbox::without_text(&mut value).indeterminate(shared.is_none())).on_hover_text("Автоматически создаёт связанные отверстия в содержащих плитах на той же отметке. Перемещаются и удаляются вместе с этой плитой.");
                            if r.changed(){patches.push((Change::CutOtherSlabs(value),r.id));}ui.end_row();
                        }
                        if contours.iter().all(|c|matches!(c.kind,Kind::Support|Kind::Column)) {
                            ui.label("Расчётная длина");
                            let shared=common(contours.iter().map(|c|c.calculate_effective_length));
                            let mut value=shared.unwrap_or(false);
                            let response=ui.add(egui::Checkbox::new(&mut value,"Вычислять").indeterminate(shared.is_none()))
                                .on_hover_text("При МКЭ-расчёте определять расчётные длины только отмеченных стен и колонн. Сетка не изменяется.");
                            if response.changed(){patches.push((Change::EffectiveLength(value),response.id));}ui.end_row();
                        }
                        if contours.iter().all(|c| matches!(c.kind,Kind::Beam|Kind::Column)) {
                            for (axis,label) in [(0,"Жёсткая вставка Y, мм"),(1,"Жёсткая вставка Z, мм")] {
                                ui.label(label);
                                let shared=common(contours.iter().map(|c|c.placement.rigid_yz_mm[axis]));
                                let mut value=shared.unwrap_or(0.);
                                let response=ui.add_enabled(can_move, egui::DragValue::new(&mut value).range(-1e6..=1e6).speed(1.).update_while_editing(false))
                                    .on_hover_text("От узлов схемы к упругой оси стержня, по местным осям сечения с учётом поворота. Одинаково на обоих концах каждого КЭ. Плюс — по направлению оси.");
                                if response.changed(){patches.push((Change::RigidOffset(axis,value),response.id));}ui.end_row();
                            }
                        }
                        if contours.iter().all(|c| matches!(c.kind,Kind::Slab|Kind::Support)) {
                            ui.label("Размер КЭ, мм");
                            let value=common(contours.iter().map(|c|c.mesh_size_mm));let mut size=value.unwrap_or(0.);
                            let response=ui.add(egui::DragValue::new(&mut size).range(0. ..=1e6).speed(50.).custom_formatter(move|v,_|value.map_or_else(||"Различные".into(), |_|format!("{v:.0}")))).on_hover_text("0 — размер КЭ из проекта; индивидуальный размер не менее 100 мм");
                            if response.changed(){patches.push((Change::MeshSize(if size>0. {size.max(100.)} else {0.}),response.id));}ui.end_row();
                        }
                        if contours.iter().all(|c| c.kind.is_restraint()) {
                            for (row, label) in ["Перемещения", "Повороты"].into_iter().enumerate() {
                                ui.label(label);
                                ui.horizontal(|ui| {
                                    for (axis, name) in ["X", "Y", "Z"].into_iter().enumerate() {
                                        let bit = 1 << (row * 3 + axis);
                                        let state = common(contours.iter().map(|c| c.restraint_mask & bit != 0));
                                        let caption = if state.is_none() { format!("{name} ·") } else { name.into() };
                                        let response = ui.selectable_label(state == Some(true), caption);
                                        if response.clicked() {
                                            patches.push((Change::RestraintDof(bit, state != Some(true)), response.id));
                                        }
                                    }
                                });
                                ui.end_row();
                            }
                        }
                        let same_kind = common(contours.iter().map(|c| c.kind));
                        if contours.iter().all(|c| c.load.is_some() && c.kind != Kind::LoadHole) {
                            let moment = common(contours.iter().map(|c| c.load.as_ref().unwrap().moment));
                            if let Some(moment) = moment {
                                ui.label(if moment { "Ось момента" } else { "Направление силы" });
                                let axis = common(contours.iter().map(|c| c.load.as_ref().unwrap().direction));
                                let mut selected = axis;
                                let response = egui::ComboBox::from_id_salt("load_axis").selected_text(axis.map_or("Различные", |a| a.label(moment))).show_ui(ui, |ui| {
                                    for axis in [crate::plan_trace::loads::Direction::X, crate::plan_trace::loads::Direction::Y, crate::plan_trace::loads::Direction::Z] {
                                        ui.selectable_value(&mut selected, Some(axis), axis.label(moment));
                                    }
                                }).response;
                                if selected != axis {
                                    if let Some(axis) = selected { patches.push((Change::LoadDirection(axis), response.id)); }
                                }
                                ui.end_row();
                            }
                        }
                        let load_levels = contours.iter().all(|c| matches!(c.kind, Kind::LoadPoint | Kind::LoadLine | Kind::LoadArea));
                        if specific && same_kind == Some(Kind::LoadHole) && self.building.enabled {
                            ui.label("Уровень отверстия нагрузки");
                            let z = common(contours.iter().map(|c| self.source_heights(c)[0]));
                            ui.label(z.map_or_else(|| "Различные".into(), |z| format!("{z:.1} мм · по нагрузке")));
                            ui.end_row();
                        }
                        if specific || load_levels {
                            let bound_rigid_link = same_kind == Some(Kind::RigidLink) && contours.iter().any(|c| c.placement.rigid_link_targets.is_some());
                            if bound_rigid_link {
                                ui.label("Привязка АЖТ");
                                if let Some([column, wall]) = common(contours.iter().map(|c| c.placement.rigid_link_targets)).flatten() {
                                    ui.label(format!("Колонна {column} · стена {wall}"));
                                } else {
                                    ui.label("По связанным объектам");
                                }
                                ui.end_row();
                            }
                            let slots: &[(usize, &str)] = if load_levels {
                                &[(0, "Уровень нагрузки")]
                            } else {
                                match same_kind {
                                    Some(Kind::Slab) => &[(0, "Уровень плиты")],
                                    Some(Kind::Beam) if self.building.enabled => &[(0, "Привязка уровня")],
                                    Some(Kind::Foundation) => &[(0, "Уровень основания")],
                                    Some(Kind::PointRestraint | Kind::Restraint) => &[(0, "Уровень опоры")],
                                    Some(Kind::Support) => &[(1, "Низ стены"), (2, "Верх стены")],
                                    Some(Kind::Column) => &[(1, "Низ колонны"), (2, "Верх колонны")],
                                    Some(Kind::RigidLink) => &[(1, "Низ АЖТ"), (2, "Верх АЖТ")],
                                    _ => &[],
                                }
                            };
                            if !slots.is_empty() && !self.building.enabled {ui.label("Привязки уровней");ui.label("Включите режим здания");ui.end_row();}
                            for &(slot,label) in slots.iter().filter(|_|self.building.enabled) {
                                ui.label(label);
                                if bound_rigid_link {
                                    let z = common(contours.iter().map(|c| self.source_heights(c)[slot - 1]));
                                    ui.label(z.map_or_else(|| "Различные".into(), |z| format!("{z:.1} мм")));
                                    ui.end_row();
                                    continue;
                                }
                                let beam = same_kind == Some(Kind::Beam);
                                ui.horizontal(|ui| {
                                    let anchor = common(contours.iter().map(|c| {
                                        (!c.is_free_beam()).then_some(level_ref_read(c, slot).anchor)
                                    }));
                                    let mut selected = anchor;
                                    ui.add_enabled_ui(can_move, |ui| {
                                        let response=egui::ComboBox::from_id_salt(("anchor",slot)).width(145.).selected_text(anchor.map_or("Различные", |a| a.map_or("Свободная", LevelAnchor::label))).show_ui(ui,|ui|{
                                            for a in [LevelAnchor::Bottom,LevelAnchor::Top] {ui.selectable_value(&mut selected,Some(Some(a)),a.label());}
                                            if beam { ui.selectable_value(&mut selected, Some(None), "Свободная"); }
                                        }).response;
                                        if selected != anchor {
                                            if let Some(selected) = selected {
                                                if beam { patches.push((Change::BeamLevel(selected), response.id)); }
                                                else if let Some(selected) = selected { patches.push((Change::Anchor(slot, selected), response.id)); }
                                            }
                                        }
                                    });
                                    if beam && contours.iter().any(|c| c.is_free_beam()) { return; }
                                    let value=common(contours.iter().map(|c|level_ref_read(c,slot).offset_mm));
                                    let original=value.unwrap_or(0.);
                                    let mut offset=original;
                                    let response=ui.add_enabled(can_move, egui::DragValue::new(&mut offset).speed(10.).range(-1e6..=1e6).update_while_editing(false).custom_formatter(move|v,_|if value.is_none(){"Различные".into()}else{format!("{v:.1}")}))
                                        .on_hover_text("Смещение по Z, мм");
                                    ui.label("мм");
                                    let applied=offset!=original || value.is_none() && response.changed() && response.lost_focus();
                                    if response.drag_stopped() || response.lost_focus() || (applied && !response.dragged()) {
                                        finished_offset_edits.push(response.id);
                                    }
                                    if applied{patches.push((Change::Offset(slot,offset),response.id));}
                                });
                                ui.end_row();
                            }
                        }
                    }
                });
            });
            if specific && items.iter().all(|i|matches!(i,Item::Contour(_))) {self.foundation_property_rows(ui, &ids);}
            self.property_quantity_rows(ui,items);
            if !editable {ui.colored_label(Color32::LIGHT_BLUE,"Свойства доступны для просмотра. Для изменения снимите блокировку исходных данных/категории.");}
            for (patch,widget) in patches {self.property_apply(items,patch,widget);}
            if self.properties.last_edit.is_some_and(|(widget, _)| finished_offset_edits.contains(&widget)) {
                self.properties.last_edit = None;
            }
            for item in items {if let Item::Dimension(id)=*item {if let Some(m)=self.measurements.items.iter().find(|m|m.id==id){ui.label(format!("Размер {id} · {:.1} мм",m.length()));}}}
        });
    }
}
fn level_ref(c: &mut Contour, slot: usize) -> &mut LevelRef {
    match slot {
        0 => &mut c.placement.slab,
        1 => &mut c.placement.bottom,
        _ => &mut c.placement.top,
    }
}
fn level_ref_read(c: &Contour, slot: usize) -> &LevelRef {
    match slot {
        0 => &c.placement.slab,
        1 => &c.placement.bottom,
        _ => &c.placement.top,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn beam_level_can_be_detached_and_reattached_with_undo() {
        let mut app = App::default();
        app.building.enabled = true;
        let mut beam = c(21, Kind::Beam);
        beam.points.truncate(2);
        beam.support_open = true;
        app.contours.push(beam);
        let items = [Item::Contour(21)];
        let widget = egui::Id::new("beam_level_test");
        app.property_apply(&items, Change::BeamLevel(Some(LevelAnchor::Top)), widget);
        app.property_apply(&items, Change::Offset(0, -250.), widget);
        let z = app.building.extent(app.building.active)[1] - 250.;
        app.property_apply(&items, Change::BeamLevel(None), widget);
        assert_eq!(
            app.contours[0].placement.beam_z_mm.as_deref(),
            Some(&[z, z][..])
        );
        let beam = &mut app.contours[0];
        beam.change_handle(0, true).unwrap();
        assert_eq!(beam.placement.beam_z_mm.as_deref(), Some(&[z, z, z][..]));
        beam.change_handle(1, false).unwrap();
        beam.placement.beam_z_mm.as_mut().unwrap()[1] += 500.;
        let persisted = serde_json::to_string(beam).unwrap();
        let restored: Contour = serde_json::from_str(&persisted).unwrap();
        assert_eq!(restored, *beam);
        app.property_apply(&items, Change::BeamLevel(Some(LevelAnchor::Bottom)), widget);
        assert_eq!(app.source_beam_path(&app.contours[0])[1][2], z);
        assert!(app.undo_contour_edit());
        assert_eq!(app.source_beam_path(&app.contours[0])[1][2], z + 500.);
    }
    #[test]
    fn load_level_properties_apply_to_mixed_types_and_holes_follow_parent() {
        let mut app = App::new_project();
        app.contours = vec![
            c(1, Kind::LoadArea),
            c(2, Kind::LoadPoint),
            c(3, Kind::LoadHole),
        ];
        app.contours[1].points.truncate(1);
        app.contours[2].parent = Some(1);
        let items = [Item::Contour(1), Item::Contour(2)];
        app.property_apply(
            &items,
            Change::Anchor(0, LevelAnchor::Top),
            egui::Id::new("load_anchor"),
        );
        app.property_apply(
            &items,
            Change::Offset(0, -125.),
            egui::Id::new("load_offset"),
        );
        assert!(app
            .contours
            .iter()
            .all(|c| app.source_heights(c) == [2875.]));
        assert_eq!(app.contours[2].placement.slab, LevelRef::default());
        app.undo();
        assert!(app
            .contours
            .iter()
            .all(|c| app.source_heights(c) == [3000.]));
    }

    fn c(id: u64, kind: Kind) -> Contour {
        Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: kind == Kind::Support,
            load: None,
            refinement: None,
            id,
            parent: None,
            kind,
            points: if kind == Kind::Support {
                vec![[0., 0.], [1000., 0.]]
            } else {
                vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]]
            },
            curves: vec![],
        }
    }

    #[test]
    fn mixed_load_mesh_constraint_is_shared_with_the_mesher_and_undoable() {
        use crate::plan_trace::loads::{ForceUnits, LoadData, Magnitude};
        use crate::plan_trace::model::mesh_sources;

        let mut app = App::default();
        app.contours = [
            Kind::LoadPoint,
            Kind::LoadLine,
            Kind::LoadArea,
            Kind::LoadHole,
            Kind::LoadArea,
        ]
        .into_iter()
        .enumerate()
        .map(|(i, kind)| {
            let mut contour = c(i as u64 + 1, kind);
            let closed = matches!(kind, Kind::LoadArea | Kind::LoadHole);
            contour.parent = (kind == Kind::LoadHole).then_some(3);
            if !closed {
                contour
                    .points
                    .truncate(if kind == Kind::LoadPoint { 1 } else { 2 });
            }
            contour.load = Some(LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                name: String::new(),
                mesh_constraint: kind == Kind::LoadLine,
                case_id: 1,
                magnitude: Magnitude {
                    value: 5.,
                    units: ForceUnits::TonForce,
                },
                closed,
            });
            contour
        })
        .collect();
        let items = [Item::Contour(1), Item::Contour(2), Item::Contour(3)];
        let widget = egui::Id::new("load_mesh_constraint");
        let mesh_ids = |app: &App| {
            mesh_sources(&app.contours)
                .map(|c| c.id)
                .collect::<Vec<_>>()
        };

        assert_eq!(mesh_ids(&app), vec![2]);
        app.property_apply(&items, Change::LoadMeshConstraint(true), widget);
        assert_eq!(mesh_ids(&app), vec![1, 2, 3, 4]);
        assert!(!app.contours[3].load.as_ref().unwrap().mesh_constraint);
        app.property_apply(&items, Change::LoadMeshConstraint(true), widget);
        app.property_apply(&items, Change::LoadMeshConstraint(false), widget);
        assert!(mesh_ids(&app).is_empty());
        assert_eq!(app.history.len(), 2);
        assert!(app.undo_contour_edit());
        assert_eq!(mesh_ids(&app), vec![1, 2, 3, 4]);
        assert!(app.undo_contour_edit());
        assert_eq!(mesh_ids(&app), vec![2]);
        assert_eq!(app.contours[0].load.as_ref().unwrap().magnitude.value, 5.);
    }

    #[test]
    fn one_plate_section_can_be_assigned_to_both_slabs_and_walls() {
        let mut app = App::default();
        app.contours = vec![c(1, Kind::Slab), c(2, Kind::Support)];
        app.sync_stiffness_sources();
        let section = app.stiffnesses.add(
            Family::Slab,
            crate::plan_trace::fem::element::Material {
                thickness_mm: 275.,
                ..Default::default()
            },
        );
        app.property_apply(
            &[Item::Contour(1), Item::Contour(2)],
            Change::Stiffness(section),
            egui::Id::new("common_plate_section"),
        );
        assert_eq!(app.stiffnesses.assignments[&1], section);
        assert_eq!(app.stiffnesses.assignments[&2], section);
        assert_eq!(app.materials[&1].thickness_mm, 275.);
        assert_eq!(app.support_height.materials[&2].thickness_mm, 275.);
    }

    #[test]
    fn design_can_change_for_one_plate_without_changing_its_shared_section() {
        let mut app = App::default();
        app.contours = vec![c(1, Kind::Slab), c(2, Kind::Slab)];
        app.sync_stiffness_sources();
        let inherited = app.stiffnesses.design_id(1);
        app.stiffnesses.designs.insert(
            50,
            crate::plan_trace::stiffness::Design {
                name: "Индивидуальный".into(),
                bar: false,
                settings: crate::plan_trace::reinforcement::Settings {
                    minimum_percent: 0.4,
                    ..Default::default()
                },
            },
        );
        app.property_apply(
            &[Item::Contour(1)],
            Change::Design(50),
            egui::Id::new("design"),
        );
        assert_eq!(
            app.stiffnesses.assignments[&1],
            app.stiffnesses.assignments[&2]
        );
        assert_eq!(app.stiffnesses.design_id(1), Some(50));
        assert_eq!(app.stiffnesses.design_id(2), inherited);
        assert_eq!(
            app.stiffnesses
                .resolved_entry(1)
                .unwrap()
                .reinforcement
                .minimum_percent,
            0.4
        );
        let snapshot = app.stiffnesses.snapshot(&app.contours);
        let saved = serde_json::to_vec(&snapshot).unwrap();
        let restored: crate::plan_trace::stiffness::Library =
            serde_json::from_slice(&saved).unwrap();
        assert_eq!(restored.design_id(1), Some(50));
        app.undo();
        assert_eq!(app.stiffnesses.design_id(1), inherited);
    }

    #[test]
    fn quantities_use_holes_actual_wall_height_and_assigned_thickness() {
        let mut app = App::default();
        let slab = c(1, Kind::Slab);
        let mut hole = c(2, Kind::Hole);
        hole.parent = Some(1);
        hole.points = vec![[250., 250.], [750., 250.], [750., 750.], [250., 750.]];
        let mut wall = c(3, Kind::Support);
        wall.placement.bottom.offset_mm = 200.;
        wall.placement.top.offset_mm = -300.;
        app.contours = vec![slab, hole, wall];
        app.building.enabled = true;
        app.sync_stiffness_sources();
        let q = app.property_quantities(&app.contours[0]).unwrap();
        assert!((q.area_m2 - 0.75).abs() < 1e-10);
        assert!((q.length_m - 6.).abs() < 1e-10);
        assert!((q.volume_m3 - 0.15).abs() < 1e-10);
        let q = app.property_quantities(&app.contours[2]).unwrap();
        assert_eq!(q.height_mm, Some(2500.));
        assert_eq!(q.length_m, 1.);
        assert!((q.area_m2 - 2.5).abs() < 1e-10);
        assert!((q.volume_m3 - 0.5).abs() < 1e-10);
    }
    #[test]
    fn common_edit_floor_stiffness_and_levels_are_atomic_and_undoable() {
        let mut app = App::default();
        app.contours = vec![c(1, Kind::Slab), c(2, Kind::Slab), c(3, Kind::Support)];
        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.sync_stiffness_sources();
        let items = [Item::Contour(1), Item::Contour(2)];
        app.property_apply(&items, Change::Name("Перекрытие".into()), egui::Id::new(1));
        assert!(app.contours[..2].iter().all(|c| c.name == "Перекрытие"));
        app.property_apply(&items, Change::Floor(2), egui::Id::new(2));
        assert_eq!(app.building.owner(1), 2);
        app.property_apply(
            &items,
            Change::Anchor(0, LevelAnchor::Top),
            egui::Id::new(3),
        );
        app.property_apply(&items, Change::Offset(0, -200.), egui::Id::new(4));
        assert_eq!(app.source_heights(&app.contours[0]), vec![5800.]);
        app.press_undo_after_control_release();
        assert_eq!(app.source_heights(&app.contours[0]), vec![6000.]);
        app.undo();
        app.undo();
        assert_eq!(app.building.owner(1), 1);
        app.undo();
        assert!(app.contours[0].name.is_empty());
        let id = app.stiffnesses.add(
            Family::Slab,
            crate::plan_trace::fem::element::Material {
                thickness_mm: 350.,
                ..Default::default()
            },
        );
        app.property_apply(&items, Change::Stiffness(id), egui::Id::new(5));
        assert_eq!(app.materials[&1].thickness_mm, 350.);
        app.undo();
        assert_eq!(app.materials[&1].thickness_mm, 200.);
        app.input_locked = true;
        app.property_apply(&items, Change::Offset(0, 100.), egui::Id::new(6));
        assert_eq!(app.contours[0].placement.slab.offset_mm, 0.);
    }

    #[test]
    fn undo_wall_bottom_and_top_offsets_in_separate_steps() {
        let mut app = App::default();
        app.building.enabled = true;
        app.contours.push(c(1, Kind::Support));
        let original = app.source_heights(&app.contours[0]);
        let wall = [Item::Contour(1)];
        app.property_apply(&wall, Change::Offset(1, 100.), egui::Id::new("bottom"));
        app.property_apply(&wall, Change::Offset(2, -300.), egui::Id::new("top"));
        app.property_apply(&wall, Change::Offset(1, 100.), egui::Id::new("bottom"));
        assert_eq!(app.history.len(), 2);
        assert_eq!(
            app.source_heights(&app.contours[0]),
            [original[0] + 100., original[1] - 300.]
        );
        app.press_undo_after_control_release();
        assert_eq!(
            app.source_heights(&app.contours[0]),
            [original[0] + 100., original[1]]
        );
        app.press_undo_after_control_release();
        assert_eq!(app.source_heights(&app.contours[0]), original);
    }

    #[test]
    fn movement_lock_is_persisted_undoable_and_leaves_calculation_inputs_intact() {
        let mut app = App::default();
        app.building.enabled = true;
        app.contours.push(c(1, Kind::Slab));
        let before = app.contours.clone();
        let key = app.analysis_key();
        let slab = [Item::Contour(1)];
        app.property_apply(&slab, Change::MovementLock(true), egui::Id::new("lock"));
        assert!(app.contours[0].placement.movement_locked);
        assert!(model::same_mesh_input(&before, &app.contours));
        assert!(app.analysis_key() == key);
        let json = serde_json::to_value(&app.contours[0]).unwrap();
        let decoded: Contour = serde_json::from_value(json).unwrap();
        assert!(decoded.placement.movement_locked);

        app.property_apply(&slab, Change::Offset(0, 500.), egui::Id::new("level"));
        app.change_vertex(1, 0, true);
        assert_eq!(app.contours[0], decoded);
        assert_eq!(app.history.len(), 1);

        app.press_undo_after_control_release();
        assert_eq!(app.contours, before);
    }

    #[test]
    fn mixed_underlay_properties_are_atomic_and_undo_preserves_mesh() {
        let mut app = App::default();
        app.contours.push(c(1, Kind::Slab));
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap());
        let make_underlay = |id, origin, scale, data| {
            serde_json::from_value(serde_json::json!({
                "id": id,
                "name": "",
                "visible": true,
                "origin": origin,
                "scale": scale,
                "data": data,
            }))
            .unwrap()
        };
        app.underlays = vec![
            make_underlay(
                1,
                [20., 50.],
                2.,
                serde_json::json!({"Pdf": {
                    "width": 1, "height": 1, "rgba": [0, 0, 0, 255],
                    "size_mm": [100., 150.], "page": 1
                }}),
            ),
            make_underlay(
                2,
                [100., 200.],
                4.,
                serde_json::json!({"Dxf": crate::plan_trace::import::Drawing {
                    nodes: vec![[10., 0.]],
                    ..Default::default()
                }}),
            ),
        ];
        app.sync_underlays();
        let items = [Item::Underlay(1), Item::Underlay(2)];
        let scale_widget = egui::Id::new("scale_x");
        for scale in [2.5, 3.] {
            app.property_apply(
                &items,
                Change::UnderlayTransform(UnderlayProperty::Scale(0), scale),
                scale_widget,
            );
        }
        assert_eq!(app.history.len(), 1);
        assert_eq!(app.underlays[0].transform().scale, [3., 2.]);
        assert_eq!(app.underlays[1].transform().scale, [3., 4.]);
        app.property_apply(
            &items,
            Change::UnderlayTransform(UnderlayProperty::Scale(1), 0.),
            egui::Id::new("scale_y"),
        );
        assert_eq!(app.history.len(), 1);
        assert_eq!(app.underlays[1].transform().scale, [3., 4.]);
        app.property_apply(
            &items,
            Change::UnderlayTransform(UnderlayProperty::Rotation, 90.),
            egui::Id::new("angle"),
        );
        assert!((app.underlays[0].transform().angle - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let [x, y] = app.drawing.as_ref().unwrap().nodes[0];
        assert!((x - 100.).abs() < 1e-12);
        assert!((y - 230.).abs() < 1e-12);
        assert!(app.fe_mesh.is_some());
        app.underlays_locked = true;
        app.undo();
        assert_eq!(app.history.len(), 2);
        app.underlays_locked = false;
        app.undo();
        assert_eq!(app.underlays[0].transform().angle, 0.);
        assert_eq!(app.drawing.as_ref().unwrap().nodes[0], [130., 200.]);
        app.undo();
        assert_eq!(app.underlays[0].transform().scale, [2., 2.]);
        assert_eq!(app.underlays[1].transform().scale, [4., 4.]);
        assert_eq!(app.drawing.as_ref().unwrap().nodes[0], [140., 200.]);
        assert!(app.fe_mesh.is_some());
    }
}
