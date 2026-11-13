use super::*;
use crate::plan_trace::fem::element::Material;
use crate::plan_trace::stiffness::{Design, Entry, NamedMaterial};
use rstar::{primitives::GeomWithData, RTree, AABB};
use std::collections::{BTreeMap, BTreeSet};
const PREFIX: &str = "mini-fem-slab-objects-v1\n";
const PASTE_TOLERANCE_MM: f64 = crate::plan_trace::building_mesh::WELD_TOLERANCE_MM;
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct CopiedMaterial {
    material: NamedMaterial,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    color: Option<[u8; 3]>,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct Bundle {
    #[serde(default)]
    source_floor: Option<u64>,
    #[serde(default)]
    preserve_load_cases: bool,
    contours: Vec<Contour>,
    materials: BTreeMap<u64, Material>,
    #[serde(default)]
    properties: BTreeMap<u64, Entry>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    named_materials: BTreeMap<u64, CopiedMaterial>,
    #[serde(default, skip_serializing_if = "BTreeMap::is_empty")]
    named_designs: BTreeMap<u64, Design>,
    base: Point,
}
#[derive(Clone)]
pub(super) struct Pending {
    repeat: bool,
    bundle: Bundle,
    parent: Option<u64>,
}
pub(super) enum Operation {
    Base(Pending),
    Place(Pending),
}
impl Bundle {
    fn text(&self) -> Result<String, String> {
        if self.contours.len() > 10000 {
            return Err("Слишком много объектов для буфера".into());
        }
        serde_json::to_string(self)
            .map_err(|e| e.to_string())
            .and_then(|s| {
                if s.len() > 8 * 1024 * 1024 {
                    Err("Слишком большой буфер объектов".into())
                } else {
                    Ok(format!("{PREFIX}{s}"))
                }
            })
    }
    fn read(text: &str) -> Result<Option<Self>, String> {
        let Some(json) = text.strip_prefix(PREFIX) else {
            return Ok(None);
        };
        if json.len() > 8 * 1024 * 1024 {
            return Err("Слишком большой буфер объектов".into());
        }
        let mut data: Self =
            serde_json::from_str(json).map_err(|e| format!("Буфер объектов: {e}"))?;
        if data.contours.is_empty()
            || data.contours.len() > 10000
            || data.base.iter().any(|v| !v.is_finite())
        {
            return Err("Некорректный буфер объектов".into());
        }
        let mut ids = BTreeSet::new();
        for c in &mut data.contours {
            if c.id == 0
                || c.id == u64::MAX
                || !ids.insert(c.id)
                || c.points.len() > 4096
                || c.curves.len() > 4096
                || c.points.iter().flatten().any(|v| !v.is_finite())
                || c.curves
                    .iter()
                    .any(|curve| matches!(curve,Curve::Spline{controls,..} if controls.len()>512))
                || c.kind.is_load() != c.load.is_some()
                || (matches!(c.kind, Kind::MeshRegion | Kind::Punching) && c.refinement.is_none())
                || (!matches!(c.kind, Kind::MeshRegion | Kind::Punching | Kind::Auxiliary)
                    && c.refinement.is_some())
                || (!matches!(
                    c.kind,
                    Kind::Hole | Kind::MeshRegion | Kind::LoadHole | Kind::Punching
                ) && c.parent.is_some())
            {
                return Err("Некорректная геометрия в буфере".into());
            }
            if !c.column_angle.is_finite() {
                return Err("Некорректный угол колонны в буфере".into());
            }
            c.rebuild()?;
        }
        for (id, m) in &data.materials {
            if !data
                .contours
                .iter()
                .any(|c| c.id == *id && c.kind == Kind::Slab)
            {
                return Err("Материал без плиты в буфере".into());
            }
            m.validate()?;
        }
        for (id, e) in &data.properties {
            if !data.contours.iter().any(|c| {
                c.id == *id
                    && matches!(
                        c.kind,
                        Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
                    )
            }) || e.name.trim().is_empty()
                || e.name.len() > 1024
            {
                return Err("Некорректная жёсткость в буфере".into());
            }
            e.material.validate()?;
            e.reinforcement.validate_source()?;
        }
        for (id, copied) in &data.named_materials {
            let m = &copied.material;
            if !data.properties.contains_key(id)
                || m.concrete.is_some_and(|c| {
                    ![c.factor, c.alternative_factor]
                        .into_iter()
                        .all(|value| value.is_finite() && value > 0.)
                })
            {
                return Err("Некорректный именованный материал в буфере".into());
            }
            for elastic in [m.primary, m.alternative] {
                Material {
                    e_mpa: elastic.e_mpa,
                    nu: elastic.nu,
                    weight: elastic.rho,
                    ..Default::default()
                }
                .validate()?;
            }
        }
        if !data.named_designs.is_empty() {
            let plates = data
                .contours
                .iter()
                .filter(|c| matches!(c.kind, Kind::Slab | Kind::Support))
                .map(|c| c.id)
                .collect::<BTreeSet<_>>();
            for (id, design) in &data.named_designs {
                if design.bar || !plates.contains(id) || !data.properties.contains_key(id) {
                    return Err("Некорректное конструирование пластины в буфере".into());
                }
                design.settings.validate_source()?;
            }
        }
        for c in &data.contours {
            if c.parent.is_some_and(|id| {
                data.contours.iter().any(|p| {
                    p.id == id
                        && p.kind
                            != if c.kind == Kind::LoadHole {
                                Kind::LoadArea
                            } else {
                                Kind::Slab
                            }
                })
            }) {
                return Err("Некорректная связь вложенного объекта".into());
            }
            if matches!(c.kind, Kind::Hole | Kind::MeshRegion | Kind::LoadHole) {
                if c.parent.is_none_or(|id| id == 0 || id == u64::MAX) {
                    return Err("Не задана плита вложенного объекта".into());
                }
                model::valid(&c.points)?;
                if c.refinement
                    .as_ref()
                    .is_some_and(|r| !r.size.is_finite() || r.size < 100.)
                {
                    return Err("Некорректный размер КЭ в буфере".into());
                }
            } else {
                model::validate_candidate(&data.contours, c)?;
            }
        }
        Ok(Some(data))
    }
    fn moved(&self, target: Point) -> Result<Vec<Contour>, String> {
        if target.iter().any(|v| !v.is_finite()) {
            return Err("Некорректная точка вставки".into());
        }
        let delta = [target[0] - self.base[0], target[1] - self.base[1]];
        if delta.iter().any(|v| !v.is_finite()) {
            return Err("Слишком большое смещение".into());
        }
        self.contours
            .iter()
            .map(|c| {
                let mut c = c.clone();
                c.translate(delta)?;
                if c.points.iter().flatten().any(|v| !v.is_finite()) {
                    return Err("Слишком большие координаты".into());
                }
                Ok(c)
            })
            .collect()
    }

    fn matches_existing_properties(&self, source: u64, app: &App, existing: u64) -> bool {
        let entry = app.stiffnesses.resolved_entry_for(existing, 0);
        let material = self.named_materials.get(&source);
        let design = self.named_designs.get(&source);
        if let Some(property) = self.properties.get(&source) {
            if !entry.as_ref().is_some_and(|e| {
                e.family.section_family() == property.family.section_family()
                    && e.material.thickness_mm == property.material.thickness_mm
                    && (material.is_some() || e.material == property.material)
                    && e.width_mm == property.width_mm
                    && (design.is_some() || e.reinforcement == property.reinforcement)
            }) {
                return false;
            }
        } else if let Some(material) = self.materials.get(&source) {
            if entry
                .map(|e| e.material)
                .or_else(|| app.materials.get(&existing).copied())
                != Some(*material)
            {
                return false;
            }
        }
        material.is_none_or(|copied| {
            app.stiffnesses
                .material_id(existing)
                .and_then(|id| app.stiffnesses.elastic.get(&id))
                == Some(&copied.material)
        }) && design.is_none_or(|copied| {
            app.stiffnesses
                .design_id(existing)
                .and_then(|id| app.stiffnesses.designs.get(&id))
                == Some(copied)
        })
    }

    fn matches_staged_properties(&self, a: u64, b: u64) -> bool {
        self.materials.get(&a) == self.materials.get(&b)
            && self.properties.get(&a) == self.properties.get(&b)
            && self.named_materials.get(&a).map(|m| &m.material)
                == self.named_materials.get(&b).map(|m| &m.material)
            && self.named_designs.get(&a) == self.named_designs.get(&b)
    }
}

fn paste_index_point(c: &Contour) -> [f64; 3] {
    let [x, y] = c.points.iter().fold([f64::INFINITY; 2], |min, p| {
        [min[0].min(p[0]), min[1].min(p[1])]
    });
    [x, y, f64::from(c.kind as u8)]
}

fn same_paste_contour(a: &Contour, b: &Contour) -> bool {
    if a.kind != b.kind
        || a.parent != b.parent
        || !a.placement.same_except_movement_lock(&b.placement)
        || a.closed() != b.closed()
        || a.points.len() != b.points.len()
        || a.curves != b.curves
        || a.grid != b.grid
        || a.foundation != b.foundation
        || a.mesh_size_mm != b.mesh_size_mm
        || a.column_angle != b.column_angle
        || a.calculate_effective_length != b.calculate_effective_length
        || a.restraint_mask != b.restraint_mask
        || a.use_foundation != b.use_foundation
        || a.load.as_ref().map(|l| {
            (
                l.case_id,
                l.magnitude,
                l.additive,
                l.mesh_constraint,
                l.direction,
                l.moment,
            )
        }) != b.load.as_ref().map(|l| {
            (
                l.case_id,
                l.magnitude,
                l.additive,
                l.mesh_constraint,
                l.direction,
                l.moment,
            )
        })
        || a.refinement.as_ref().map(|r| (r.size, &r.inactive_edges))
            != b.refinement.as_ref().map(|r| (r.size, &r.inactive_edges))
    {
        return false;
    }

    let same_point = |p: &Point, q: &Point| {
        (p[0] - q[0]).abs() <= PASTE_TOLERANCE_MM && (p[1] - q[1]).abs() <= PASTE_TOLERANCE_MM
    };
    if a.points
        .iter()
        .zip(&b.points)
        .all(|(p, q)| same_point(p, q))
    {
        return true;
    }
    // Wall and beam order defines local axes. C1 values and inactive edges
    // are indexed by vertex or edge. Keep those contours in their stored order.
    if matches!(
        a.kind,
        Kind::Support | Kind::Beam | Kind::Foundation | Kind::RigidLink
    ) || !a.curves.is_empty()
        || a.refinement
            .as_ref()
            .is_some_and(|r| !r.inactive_edges.is_empty())
    {
        return false;
    }
    if !a.closed() {
        return a
            .points
            .iter()
            .zip(b.points.iter().rev())
            .all(|(p, q)| same_point(p, q));
    }
    let n = a.points.len();
    b.points.iter().enumerate().any(|(start, p)| {
        same_point(&a.points[0], p)
            && ((1..n).all(|i| same_point(&a.points[i], &b.points[(start + i) % n]))
                || (1..n).all(|i| same_point(&a.points[i], &b.points[(start + n - i) % n])))
    })
}
impl App {
    fn copy_bundle(&mut self) -> Option<Bundle> {
        self.sync_stiffness_sources();
        self.prune_selection();
        let mut ids = self.selection_ids.clone();
        ids.extend(self.selected);
        if ids.is_empty() {
            return None;
        }
        let slabs = self
            .contours
            .iter()
            .filter(|c| ids.contains(&c.id) && matches!(c.kind, Kind::Slab | Kind::LoadArea))
            .map(|c| c.id)
            .collect::<BTreeSet<_>>();
        // A copied slab retains its actual openings and local meshing regions.
        ids.extend(
            self.contours
                .iter()
                .filter(|c| c.parent.is_some_and(|id| slabs.contains(&id)))
                .map(|c| c.id),
        );
        let bound_links = self
            .contours
            .iter()
            .filter_map(|c| {
                c.placement
                    .rigid_link_targets
                    .filter(|targets| targets.iter().all(|id| ids.contains(id)))
                    .map(|_| c.id)
            })
            .collect::<BTreeSet<_>>();
        let contours = self
            .contours
            .iter()
            .filter(|c| {
                if c.placement.rigid_link_targets.is_some() {
                    bound_links.contains(&c.id)
                } else {
                    ids.contains(&c.id)
                }
            })
            .cloned()
            .collect::<Vec<_>>();
        if contours.is_empty() {
            return None;
        }
        let bounds = contours.iter().flat_map(|c| &c.points).fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |b, p| {
                [
                    b[0].min(p[0]),
                    b[1].min(p[1]),
                    b[2].max(p[0]),
                    b[3].max(p[1]),
                ]
            },
        );
        let properties = contours
            .iter()
            .filter_map(|c| {
                let id = self.stiffnesses.assignments.get(&c.id)?;
                let mut entry = self.stiffnesses.entries.get(id)?.clone();
                // Keep the section's legacy data separate from contour overrides.
                entry.material = self.stiffnesses.resolved_entry_for(c.id, 0)?.material;
                Some((c.id, entry))
            })
            .collect::<BTreeMap<_, _>>();
        let named_materials = contours
            .iter()
            .filter_map(|c| {
                let id = self.stiffnesses.material_id(c.id)?;
                let material = self.stiffnesses.elastic.get(&id)?.clone();
                let color = self.stiffnesses.material_colors.get(&id).copied();
                Some((c.id, CopiedMaterial { material, color }))
            })
            .collect();
        let named_designs = contours
            .iter()
            .filter(|c| {
                matches!(c.kind, Kind::Slab | Kind::Support) && properties.contains_key(&c.id)
            })
            .filter_map(|c| {
                let design = self.stiffnesses.design_id(c.id)?;
                self.stiffnesses
                    .designs
                    .get(&design)
                    .filter(|design| !design.bar)
                    .map(|design| (c.id, design.clone()))
            })
            .collect();
        let source_floor = if self.building.enabled {
            let floors = contours
                .iter()
                .map(|c| self.building.owner(c.id))
                .collect::<BTreeSet<_>>();
            (floors.len() == 1).then(|| *floors.first().unwrap())
        } else {
            None
        };
        Some(Bundle {
            preserve_load_cases: self.result_by_combinations,
            source_floor,
            properties,
            named_materials,
            named_designs,
            contours,
            base: [
                bounds[0] + (bounds[2] - bounds[0]) * 0.5,
                bounds[1] + (bounds[3] - bounds[1]) * 0.5,
            ],
            materials: slabs
                .into_iter()
                .filter(|id| {
                    self.contours
                        .iter()
                        .any(|c| c.id == *id && c.kind == Kind::Slab)
                })
                .map(|id| (id, self.materials.get(&id).copied().unwrap_or_default()))
                .collect(),
        })
    }
    pub(super) fn clipboard_keys(&mut self, ctx: &egui::Context) {
        if ctx.wants_keyboard_input() {
            return;
        }
        for event in ctx.input(|i| i.events.clone()) {
            match event {
                egui::Event::Copy => {
                    if let Some(data) = self.copy_bundle() {
                        match data.text() {
                            Ok(text) => {
                                ctx.copy_text(text);
                                self.status =
                                    format!("Скопировано объектов: {}", data.contours.len());
                            }
                            Err(e) => self.status = e,
                        }
                    }
                }
                egui::Event::Paste(text) => match Bundle::read(&text) {
                    Ok(Some(data)) => {
                        let parent = self.selected_load_area().or_else(|| self.selected_slab());
                        let result = self.paste_bundle(data, parent);

                        if let Err(e) = result {
                            self.status = e;
                        }
                        ctx.input_mut(|i| i.events.retain(|e| !matches!(e, egui::Event::Paste(_))));
                    }
                    Err(e) => self.status = e,
                    _ => {}
                },
                _ => {}
            }
        }
    }
    pub(super) fn copy_base_command(&mut self) {
        let Some(bundle) = self.copy_bundle() else {
            return;
        };
        let parent = self.selected_load_area().or_else(|| self.selected_slab());
        let ids = self.selection_ids.clone();
        let selected = self.selected;
        self.cancel();
        self.selection_ids = ids;
        self.selected = selected;
        self.clipboard_op = Some(Operation::Base(Pending {
            bundle,
            parent,
            repeat: true,
        }));
        self.status = "Укажите базовую точку копирования. Esc — отменить".into();
    }
    fn paste_bundle(&mut self, data: Bundle, parent: Option<u64>) -> Result<(), String> {
        let same_xy = self.building.enabled
            && data
                .source_floor
                .is_some_and(|id| id != self.building.active);
        let target = data.base;
        self.start_paste(data, parent)?;
        if same_xy {
            self.commit_paste(target)
        } else {
            Ok(())
        }
    }
    fn start_paste(&mut self, bundle: Bundle, parent: Option<u64>) -> Result<(), String> {
        if bundle.contours.iter().any(|c| self.is_kind_locked(c.kind)) {
            return Err("Вставка заблокирована. Снимите блокировку соответствующих типов объектов или исходных данных".into());
        }
        self.cancel();
        self.clipboard_op = Some(Operation::Place(Pending {
            bundle,
            parent,
            repeat: false,
        }));
        self.status = "Укажите точку вставки. Esc — отменить".into();
        Ok(())
    }
    fn commit_paste(&mut self, target: Point) -> Result<(), String> {
        let Some(Operation::Place(pending)) = &self.clipboard_op else {
            return Ok(());
        };
        if pending
            .bundle
            .contours
            .iter()
            .any(|c| self.is_kind_locked(c.kind))
        {
            return Err("Вставка заблокирована".into());
        }
        let repeat = pending.repeat.then(|| pending.clone());
        let start = self.next_id.max(
            self.contours
                .iter()
                .map(|c| c.id)
                .max()
                .unwrap_or(0)
                .checked_add(1)
                .ok_or("Исчерпаны ID")?,
        );
        let end = start
            .checked_add(pending.bundle.contours.len() as u64)
            .ok_or("Исчерпаны ID")?;
        let map = pending
            .bundle
            .contours
            .iter()
            .enumerate()
            .map(|(i, c)| (c.id, start + i as u64))
            .collect::<BTreeMap<_, _>>();
        let mut candidates = pending.bundle.moved(target)?;
        let preserve_load_cases = pending.bundle.preserve_load_cases;
        if preserve_load_cases {
            if candidates
                .iter()
                .filter_map(|c| c.load.as_ref())
                .any(|l| !self.load_cases.iter().any(|case| case.id == l.case_id))
            {
                return Err("В проекте отсутствует исходное загружение копируемой нагрузки".into());
            }
        } else if candidates.iter().any(|c| c.kind.is_load())
            && !self.load_cases.iter().any(|c| c.id == self.active_load)
        {
            return Err("Не найдено активное загружение".into());
        }
        for c in &mut candidates {
            c.id = map[&c.id];
            if let Some(source) = c.placement.cut_source {
                c.placement.cut_source = Some(map.get(&source).copied().unwrap_or(source));
            }
            if let Some([column, wall]) = c.placement.rigid_link_targets {
                c.placement.rigid_link_targets = Some([
                    *map.get(&column)
                        .ok_or("Для вставки АЖТ нужна связанная колонна")?,
                    *map.get(&wall)
                        .ok_or("Для вставки АЖТ нужна связанная стена")?,
                ]);
            }
            if !preserve_load_cases {
                if let Some(load) = &mut c.load {
                    load.case_id = self.active_load;
                }
            }
            if let Some(region) = &mut c.refinement {
                region.name.push_str(" (копия)");
            }
            if let Some(parent) = c.parent {
                if let Some(&new) = map.get(&parent) {
                    c.parent = Some(new);
                } else {
                    let parents = pending
                        .parent
                        .filter(|id| {
                            !self.building.enabled
                                || self.building.owner(*id) == self.building.active
                        })
                        .into_iter()
                        .chain(
                            self.contours
                                .iter()
                                .filter(|p| {
                                    self.on_active_floor(p)
                                        && p.kind
                                            == if c.kind == Kind::LoadHole {
                                                Kind::LoadArea
                                            } else {
                                                Kind::Slab
                                            }
                                })
                                .map(|p| p.id),
                        );
                    let parent = parents
                        .into_iter()
                        .find(|&id| {
                            let mut probe = c.clone();
                            probe.parent = Some(id);
                            if c.kind == Kind::LoadHole {
                                probe.load.as_mut().unwrap().case_id = self
                                    .contours
                                    .iter()
                                    .find(|p| p.id == id)
                                    .and_then(|p| p.load.as_ref())
                                    .map_or(0, |l| l.case_id);
                            }
                            model::validate_candidate(&self.contours, &probe).is_ok()
                        })
                        .ok_or("Вложенный объект нужно вставить внутрь плиты")?;
                    c.parent = Some(parent);
                    if c.kind == Kind::LoadHole {
                        c.load.as_mut().unwrap().case_id = self
                            .contours
                            .iter()
                            .find(|p| p.id == parent)
                            .and_then(|p| p.load.as_ref())
                            .unwrap()
                            .case_id;
                    }
                }
            }
        }
        let existing_count = self.contours.len();
        let mut index = RTree::bulk_load(
            self.contours
                .iter()
                .enumerate()
                .filter(|(_, c)| self.on_active_floor(c) && !c.points.is_empty())
                .map(|(i, c)| GeomWithData::new(paste_index_point(c), i))
                .collect(),
        );
        let mut order: Vec<_> = (0..candidates.len()).collect();
        // Resolve reused parent and target IDs before comparing dependent contours.
        order.sort_by_key(|&i| {
            let c = &candidates[i];
            c.parent.is_some() || c.placement.rigid_link_targets.is_some()
        });
        let mut duplicates = BTreeMap::new();
        for i in order {
            let c = &mut candidates[i];
            let resolved = |id| duplicates.get(&id).copied().unwrap_or(id);
            c.parent = c.parent.map(resolved);
            c.placement.cut_source = c.placement.cut_source.map(resolved);
            c.placement.rigid_link_targets =
                c.placement.rigid_link_targets.map(|ids| ids.map(resolved));
            let c = &candidates[i];
            let [x, y, kind] = paste_index_point(c);
            let eps = PASTE_TOLERANCE_MM;
            let area = AABB::from_corners([x - eps, y - eps, kind], [x + eps, y + eps, kind]);
            let source = pending.bundle.contours[i].id;
            let duplicate = index
                .locate_in_envelope_intersecting(&area)
                .map(|item| item.data)
                .filter(|&j| {
                    if j < existing_count {
                        let existing = &self.contours[j];
                        same_paste_contour(c, existing)
                            && pending
                                .bundle
                                .matches_existing_properties(source, self, existing.id)
                    } else {
                        let j = j - existing_count;
                        same_paste_contour(c, &candidates[j])
                            && pending
                                .bundle
                                .matches_staged_properties(source, pending.bundle.contours[j].id)
                    }
                })
                .min();
            if let Some(j) = duplicate {
                let id = if j < existing_count {
                    self.contours[j].id
                } else {
                    candidates[j - existing_count].id
                };
                duplicates.insert(c.id, id);
            } else {
                index.insert(GeomWithData::new([x, y, kind], existing_count + i));
            }
        }
        let skipped = duplicates.len();
        let suffix = if repeat.is_some() {
            "Укажите следующую точку вставки. Esc — завершить"
        } else {
            "Ctrl+Z — отменить"
        };
        candidates.retain(|c| !duplicates.contains_key(&c.id));
        if candidates.is_empty() {
            self.clipboard_op = repeat.map(Operation::Place);
            self.status = format!("Пропущено совпадающих объектов: {skipped}");
            if self.clipboard_op.is_some() {
                self.status.push_str(". ");
                self.status.push_str(suffix);
            }
            return Ok(());
        }
        let mut combined = self.contours.clone();
        combined.extend(candidates.iter().cloned());
        for c in &candidates {
            model::validate_candidate(&combined, c)?;
        }
        let materials = pending
            .bundle
            .materials
            .iter()
            .filter(|(id, _)| !duplicates.contains_key(&map[id]))
            .map(|(id, m)| (map[id], *m))
            .collect::<Vec<_>>();
        let ids = candidates.iter().map(|c| c.id).collect::<Vec<_>>();
        let Some(Operation::Place(pending)) = self.clipboard_op.take() else {
            return Ok(());
        };
        if pending.bundle.named_designs.is_empty()
            && !candidates
                .iter()
                .any(|c| c.placement.rigid_link_targets.is_some())
        {
            self.history.push(self.contours.clone());
        } else {
            self.push_contour_property_undo();
        }
        for &id in &ids {
            self.building
                .contour_floors
                .insert(id, self.building.active);
        }
        let geometry = candidates.iter().any(|c| c.is_mesh_input());
        let calculation = candidates.iter().any(|c| c.kind.affects_calculation());
        self.contours = combined;
        self.materials.extend(materials);
        for (old, property) in &pending.bundle.properties {
            let new = map[old];
            if duplicates.contains_key(&new) {
                continue;
            }
            let named = pending.bundle.named_materials.get(old);
            let has_design = pending.bundle.named_designs.contains_key(old);
            let found = self
                .stiffnesses
                .entries
                .iter()
                .find(|(_, e)| {
                    if named.is_none() {
                        *e == property
                    } else {
                        e.name == property.name
                            && e.family == property.family
                            && e.material.thickness_mm == property.material.thickness_mm
                            && e.width_mm == property.width_mm
                            && (has_design || e.reinforcement == property.reinforcement)
                    }
                })
                .map(|(&id, _)| id);
            let id = found.unwrap_or_else(|| {
                let id = self.stiffnesses.add(property.family, property.material);
                let mut e = property.clone();
                let mut n = 1;
                while self
                    .stiffnesses
                    .entries
                    .iter()
                    .any(|(&other, entry)| other != id && entry.name == e.name)
                {
                    e.name = format!("{} ({n})", property.name);
                    n += 1;
                }
                self.stiffnesses.entries.insert(id, e);
                id
            });
            self.stiffnesses.assignments.insert(new, id);
            if let Some(named) = named {
                let material = self
                    .stiffnesses
                    .import_named_material(named.material.clone(), named.color);
                self.stiffnesses.contour_materials.insert(new, material);
            }
        }
        for (old, design) in pending.bundle.named_designs {
            let new = map[&old];
            if duplicates.contains_key(&new) {
                continue;
            }
            let library = &mut self.stiffnesses;
            let id = if let Some((&id, _)) = library.designs.iter().find(|(_, d)| **d == design) {
                id
            } else {
                let id = library
                    .designs
                    .keys()
                    .next_back()
                    .and_then(|id| id.checked_add(1))
                    .unwrap_or_else(|| (1..).find(|id| !library.designs.contains_key(id)).unwrap());
                library.designs.insert(id, design);
                id
            };
            library.contour_designs.insert(new, id);
        }
        self.sync_stiffness_sources();
        self.next_id = end;
        self.clipboard_op = repeat.map(Operation::Place);
        self.select_ids(ids, egui::Modifiers::NONE);
        self.prune_selection();
        if geometry {
            self.fe_mesh = None;
            self.discard_mesh_job();
            self.slab_cache = None;
        }
        if calculation {
            self.analysis = None;
            self.analysis_input = None;
            self.analysis_job = None;
            self.combination_cache = None;
        }
        self.status = if skipped == 0 {
            format!("Копии вставлены. {suffix}")
        } else {
            format!(
                "Вставлено объектов: {}. Пропущено совпадающих: {skipped}. {suffix}",
                candidates.len()
            )
        };
        Ok(())
    }
    pub(super) fn clipboard_canvas(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) {
        let Some(op) = &self.clipboard_op else {
            return;
        };
        let Some(pointer) = hovered else {
            return;
        };
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        let raw = self.camera.world(pointer, rect);
        let target = self.snap(raw, None).map_or(raw, |s| s.point);
        let picking = matches!(op, Operation::Base(_));
        let painter = ui.painter().with_clip_rect(rect);
        if let Operation::Place(pending) = op {
            let alpha = if (ui.input(|i| i.time) * 3.).floor() as i64 % 2 == 0 {
                0.85
            } else {
                0.35
            };
            // Move only the preview camera: keep analytic geometry and its
            // sampled display cache unchanged while following the pointer.
            let mut preview = self.camera;
            for axis in 0..2 {
                preview.center[axis] -= target[axis] - pending.bundle.base[axis];
            }
            if preview.center.iter().all(|v| v.is_finite()) {
                for c in &pending.bundle.contours {
                    let color = self.object_ink(c.id, c.kind).gamma_multiply(alpha);
                    if c.kind == Kind::LoadPoint {
                        let p = preview.screen(c.points[0], rect);
                        if !rect.contains(p) {
                            continue;
                        }
                        painter.circle_stroke(p, 7., Stroke::new(2_f32, color));
                        for d in [egui::vec2(5., 5.), egui::vec2(5., -5.)] {
                            painter.line_segment([p - d, p + d], Stroke::new(1.5_f32, color));
                        }
                    } else {
                        draw_path(
                            &painter,
                            &c.points,
                            c.closed(),
                            preview,
                            rect,
                            Stroke::new(2.5_f32, color),
                        );
                    }
                }
            }
            ui.ctx().request_repaint_after(Duration::from_millis(150));
        }
        let p = self.camera.screen(target, rect);
        painter.circle_stroke(p, 5., Stroke::new(1.5_f32, Color32::LIGHT_BLUE));
        if picking {
            painter.text(
                pointer + egui::vec2(14., 14.),
                egui::Align2::LEFT_TOP,
                "Базовая точка",
                egui::FontId::proportional(13.),
                Color32::LIGHT_BLUE,
            );
        }
        if response.clicked_by(egui::PointerButton::Primary) {
            if picking {
                let Some(Operation::Base(mut pending)) = self.clipboard_op.take() else {
                    return;
                };
                pending.bundle.base = target;
                if let Ok(text) = pending.bundle.text() {
                    ui.ctx().copy_text(text);
                }
                if let Err(e) = self.start_paste(pending.bundle, pending.parent) {
                    self.status = e;
                } else if let Some(Operation::Place(placement)) = &mut self.clipboard_op {
                    placement.repeat = true;
                }
            } else if let Err(e) = self.commit_paste(target) {
                self.status = e;
            }
        }
    }
    pub(super) fn spatial_clipboard(
        &mut self,
        ui: &mut egui::Ui,
        response: &egui::Response,
        target: Point,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        z: f64,
        snapped: Option<[f64; 3]>,
    ) {
        let Some(op) = &self.clipboard_op else {
            return;
        };
        let (pending, base) = match op {
            Operation::Base(p) => (p, true),
            Operation::Place(p) => (p, false),
        };
        let ink = Color32::from_rgb(90, 210, 255);
        let blink = (ui.input(|i| i.time) * 4.).floor() as i64 % 2 == 0;
        if blink && !base {
            if let Ok(contours) = pending.bundle.moved(target) {
                for c in contours {
                    let support = c.kind == Kind::Support;
                    let closed = c.closed();
                    let mut points = c.points;
                    if closed && !points.is_empty() {
                        points.push(points[0]);
                    }
                    if support {
                        let top = if self.building.enabled {
                            self.building.extent(self.building.active)[1]
                        } else {
                            z + self.support_height.height_mm
                        };
                        for p in &points {
                            ui.painter().line_segment(
                                [project([p[0], p[1], z]).0, project([p[0], p[1], top]).0],
                                Stroke::new(2.0_f32, ink),
                            );
                        }
                        for edge in points.windows(2) {
                            ui.painter().line_segment(
                                [
                                    project([edge[0][0], edge[0][1], top]).0,
                                    project([edge[1][0], edge[1][1], top]).0,
                                ],
                                Stroke::new(2.0_f32, ink),
                            );
                        }
                    }
                    for edge in points.windows(2) {
                        ui.painter().line_segment(
                            [
                                project([edge[0][0], edge[0][1], z]).0,
                                project([edge[1][0], edge[1][1], z]).0,
                            ],
                            Stroke::new(2.0_f32, ink),
                        );
                    }
                }
            }
        }
        let at = project(snapped.unwrap_or([target[0], target[1], z])).0;
        let painter = ui.painter().with_clip_rect(response.rect);
        if snapped.is_some() {
            painter.circle_stroke(at, 6., Stroke::new(2., Color32::YELLOW));
            painter.circle_filled(at, 2., Color32::WHITE);
        } else {
            painter.circle_stroke(at, 5., Stroke::new(1.5_f32, ink));
        }
        painter.text(
            at + egui::vec2(14., 14.),
            egui::Align2::LEFT_TOP,
            if base {
                "Базовая точка"
            } else {
                "Точка вставки · активный этаж"
            },
            egui::FontId::proportional(13.),
            ink,
        );
        ui.ctx().request_repaint_after(Duration::from_millis(150));
        if response.clicked_by(egui::PointerButton::Primary) {
            if base {
                let Some(Operation::Base(mut pending)) = self.clipboard_op.take() else {
                    return;
                };
                pending.bundle.base = target;
                if let Ok(text) = pending.bundle.text() {
                    ui.ctx().copy_text(text);
                }
                if let Err(e) = self.start_paste(pending.bundle, pending.parent) {
                    self.status = e;
                } else if let Some(Operation::Place(p)) = &mut self.clipboard_op {
                    p.repeat = true;
                }
            } else if let Err(e) = self.commit_paste(target) {
                self.status = e;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> App {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([6000., 4000.], false);
        a.begin(Kind::Hole);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([1000., 1000.], false);
        a.sketch_click([2000., 2000.], false);
        a.begin(Kind::MeshRegion);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([3000., 1000.], false);
        a.sketch_click([4000., 2000.], false);
        a.materials.entry(1).or_default().thickness_mm = 350.;
        a.begin(Kind::Support);
        a.sketch_click([0., 500.], false);
        a.sketch_click([6000., 500.], false);
        a.finish();
        a
    }
    #[test]
    fn normal_paste_to_other_floor_keeps_exact_xy_and_same_floor_still_places() {
        let mut a = sample();
        a.building.enabled = true;
        a.sync_building_objects();
        let ids = a.contours.iter().map(|c| c.id).collect::<Vec<_>>();
        a.select_ids(ids.clone(), egui::Modifiers::NONE);
        let bundle = Bundle::read(&a.copy_bundle().unwrap().text().unwrap())
            .unwrap()
            .unwrap();
        let originals = bundle.contours.clone();
        a.building.floors.push(building::Floor {
            id: 2,
            name: "2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        a.change_floor(2);
        a.paste_bundle(bundle.clone(), None).unwrap();
        assert!(a.clipboard_op.is_none());
        let copied = a
            .contours
            .iter()
            .filter(|c| !ids.contains(&c.id))
            .collect::<Vec<_>>();
        assert_eq!(copied.len(), originals.len());
        for (source, copy) in originals.iter().zip(copied) {
            assert_eq!(source.points, copy.points);
            assert_eq!(a.building.owner(copy.id), 2);
        }
        let after_first = a.contours.clone();
        let history = a.history.len();
        let next_id = a.next_id;
        a.paste_bundle(bundle.clone(), None).unwrap();
        assert_eq!(a.contours, after_first);
        assert_eq!(a.history.len(), history);
        assert_eq!(a.next_id, next_id);
        assert!(a.clipboard_op.is_none());
        assert!(a.status.contains("Пропущено совпадающих объектов: 4"));
        a.undo();
        assert_eq!(a.contours, originals);
        a.change_floor(1);
        a.paste_bundle(bundle, None).unwrap();
        assert!(a.clipboard_op.is_some());
    }

    #[test]
    fn paste_reuses_a_matching_slab_and_keeps_new_children() {
        let mut app = sample();
        app.select_only(Some(1));
        let bundle = app.copy_bundle().unwrap();
        let mut slab_only = bundle.clone();
        slab_only.contours.retain(|c| c.kind == Kind::Slab);
        let target = [13000., 7000.];
        app.start_paste(slab_only, None).unwrap();
        app.commit_paste(target).unwrap();
        let slab = app.contours.last_mut().unwrap();
        let slab_id = slab.id;
        slab.points.rotate_left(1);

        let before = app.contours.len();
        app.start_paste(bundle.clone(), None).unwrap();
        app.commit_paste(target).unwrap();
        assert_eq!(app.contours.len(), before + 2);
        assert!(app.contours[before..]
            .iter()
            .all(|c| c.parent == Some(slab_id)));
        assert!(app.status.contains("Пропущено совпадающих: 1"));

        app.start_paste(bundle, None).unwrap();
        app.commit_paste(target).unwrap();
        assert_eq!(app.contours.len(), before + 2);
        app.undo();
        assert_eq!(app.contours.len(), before);
    }

    #[test]
    fn coincident_loads_with_different_cases_elevations_or_intensities_are_distinct() {
        let mut app = sample();
        app.begin(Kind::LoadPoint);
        let point = [2500., 2500.];
        app.sketch_click(point, false);
        let data = app.copy_bundle().unwrap();
        let before = app.contours.len();
        app.load_cases
            .push(crate::plan_trace::loads::LoadCase::new(2));
        app.active_load = 2;

        for (height, magnitude) in [(0., 1.), (100., 1.), (100., 2.), (100., 2.)] {
            let mut data = data.clone();
            let load = &mut data.contours[0];
            load.placement.slab.offset_mm = height;
            load.load.as_mut().unwrap().magnitude.value = magnitude;
            app.start_paste(data, None).unwrap();
            app.commit_paste(point).unwrap();
        }
        assert_eq!(app.contours.len(), before + 3);
        assert!(app.contours[before..]
            .iter()
            .all(|c| c.load.as_ref().unwrap().case_id == 2));
    }
    #[test]
    fn copying_between_floors_keeps_nested_parent_ids_materials_and_undo() {
        let mut a = sample();
        a.building.enabled = true;
        a.sync_building_objects();
        let old_ids = a.contours.iter().map(|c| c.id).collect::<Vec<_>>();
        a.select_ids(old_ids.clone(), egui::Modifiers::default());
        let bundle = a.copy_bundle().unwrap();
        a.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        a.change_floor(2);
        a.start_paste(bundle, None).unwrap();
        a.commit_paste([3000., 2000.]).unwrap();
        let copies = a
            .contours
            .iter()
            .filter(|c| !old_ids.contains(&c.id))
            .collect::<Vec<_>>();
        assert!(!copies.is_empty());
        for c in &copies {
            assert_eq!(a.building.owner(c.id), 2);
            if let Some(parent) = c.parent {
                assert!(copies.iter().any(|p| p.id == parent));
            }
        }
        assert!(copies
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .all(|c| a.materials.contains_key(&c.id)));
        a.undo();
        assert_eq!(a.contours.iter().map(|c| c.id).collect::<Vec<_>>(), old_ids);
    }
    #[test]
    fn group_copy_keeps_children_materials_new_ids_and_is_one_undo() {
        let mut a = sample();
        a.select_ids([1, 4], egui::Modifiers::NONE);
        let before = a.contours.clone();
        let history = a.history.len();
        let data = a.copy_bundle().unwrap();
        assert_eq!(data.contours.len(), 4);
        assert_eq!(data.base, [3000., 2000.]);
        let data = Bundle::read(&data.text().unwrap()).unwrap().unwrap();
        a.start_paste(data, None).unwrap();
        assert_eq!(a.contours, before);
        a.commit_paste([13000., 7000.]).unwrap();
        assert_eq!(a.contours.len(), 8);
        assert_eq!(a.history.len(), history + 1);
        assert_eq!(&a.contours[..4], before.as_slice());
        let slab = a.contours[4..]
            .iter()
            .find(|c| c.kind == Kind::Slab)
            .unwrap();
        assert_ne!(slab.id, 1);
        assert_eq!(a.materials[&slab.id].thickness_mm, 350.);
        assert!(a.contours[4..]
            .iter()
            .filter(|c| matches!(c.kind, Kind::Hole | Kind::MeshRegion | Kind::LoadHole))
            .all(|c| c.parent == Some(slab.id)));
        for original in &before {
            let copy = a.contours[4..]
                .iter()
                .find(|c| c.kind == original.kind)
                .unwrap();
            let mut expected = original.clone();
            expected.translate([10000., 5000.]).unwrap();
            assert_eq!(copy.points, expected.points);
            assert_eq!(copy.curves, expected.curves);
        }
        a.undo();
        assert_eq!(a.contours, before);
    }
    #[test]
    fn clipboard_preserves_contour_material_and_color_without_duplicating_section() {
        let mut source = sample();
        source.sync_stiffness_sources();
        let section = source.stiffnesses.assignments[&4];
        let old = source.stiffnesses.material_id(4).unwrap();
        let mut material = source.stiffnesses.elastic[&old].clone();
        material.name = "Материал стены из буфера".into();
        material.primary.e_mpa = 28000.;
        material.alternative.e_mpa = 24000.;
        let color = [12, 34, 56];
        let override_id = source
            .stiffnesses
            .import_named_material(material.clone(), Some(color));
        source.stiffnesses.contour_materials.insert(4, override_id);
        source.select_only(Some(4));
        let text = source.copy_bundle().unwrap().text().unwrap();

        let mut target = sample();
        target.sync_stiffness_sources();
        let before = target.stiffnesses.entries.len();
        target
            .start_paste(Bundle::read(&text).unwrap().unwrap(), None)
            .unwrap();
        target.commit_paste([13000., 500.]).unwrap();
        let id = target.contours.last().unwrap().id;
        let imported = target.stiffnesses.material_id(id).unwrap();
        assert_eq!(target.stiffnesses.entries.len(), before);
        assert_eq!(target.stiffnesses.assignments[&id], section);
        assert_eq!(target.stiffnesses.elastic[&imported], material);
        assert_eq!(target.stiffnesses.material_colors[&imported], color);
    }
    #[test]
    fn clipboard_preserves_explicit_and_inherited_plate_designs_independently() {
        let mut source = sample();
        source.sync_stiffness_sources();
        let inherited =
            source.stiffnesses.designs[&source.stiffnesses.design_id(1).unwrap()].clone();
        let design = Design {
            name: "Конструирование стены из буфера".into(),
            bar: false,
            settings: crate::plan_trace::reinforcement::Settings {
                minimum_percent: 0.25,
                ..Default::default()
            },
        };
        let override_id = source.stiffnesses.designs.keys().next_back().unwrap() + 1;
        source
            .stiffnesses
            .designs
            .insert(override_id, design.clone());
        source.stiffnesses.contour_designs.insert(4, override_id);
        source.select_ids([1, 4], egui::Modifiers::NONE);
        let data = Bundle::read(&source.copy_bundle().unwrap().text().unwrap())
            .unwrap()
            .unwrap();
        assert_eq!(data.named_designs[&1], inherited);
        assert_eq!(data.named_designs[&4], design);

        let mut target = sample();
        target.sync_stiffness_sources();
        let section_count = target.stiffnesses.entries.len();
        target.start_paste(data, None).unwrap();
        target.commit_paste([13000., 7000.]).unwrap();
        assert_eq!(target.stiffnesses.entries.len(), section_count);
        for (kind, original, expected) in [(Kind::Slab, 1, inherited), (Kind::Support, 4, design)] {
            let copy = target.contours[4..]
                .iter()
                .find(|c| c.kind == kind)
                .unwrap();
            let id = target.stiffnesses.contour_designs[&copy.id];
            assert_eq!(target.stiffnesses.designs[&id], expected);
            assert_eq!(
                target.stiffnesses.assignments[&copy.id],
                target.stiffnesses.assignments[&original]
            );
        }
        target.undo();
        assert_eq!(target.contours.len(), 4);
        assert!(target.stiffnesses.contour_designs.is_empty());
    }
    #[test]
    fn standalone_child_reparents_at_destination_and_invalid_place_is_atomic() {
        let mut a = sample();
        a.select_only(Some(2));
        let data = a.copy_bundle().unwrap();
        a.start_paste(data, None).unwrap();
        let before = a.contours.clone();
        let history = a.history.len();
        let id = a.next_id;
        assert!(a.commit_paste([10000., 10000.]).is_err());
        assert_eq!(a.contours, before);
        assert_eq!(a.history.len(), history);
        assert_eq!(a.next_id, id);
        assert!(a.clipboard_op.is_some());
        a.commit_paste([4500., 3000.]).unwrap();
        let last = a.contours.last().unwrap();
        assert_eq!(last.kind, Kind::Hole);
        assert_eq!(last.parent, Some(1));
    }
    #[test]
    fn clipboard_events_keep_text_editing_and_paste_respects_locks_and_cancel() {
        let ctx = egui::Context::default();
        let mut a = sample();
        a.select_only(Some(4));
        let out = ctx.run(
            egui::RawInput {
                events: vec![egui::Event::Copy],
                ..Default::default()
            },
            |ctx| a.clipboard_keys(ctx),
        );
        let text = out
            .platform_output
            .commands
            .iter()
            .find_map(|c| {
                if let egui::OutputCommand::CopyText(s) = c {
                    Some(s.clone())
                } else {
                    None
                }
            })
            .expect("clipboard text");
        let data = Bundle::read(&text).unwrap().unwrap();
        let before = a.contours.clone();
        a.input_locked = true;
        assert!(a.start_paste(data.clone(), None).is_err());
        assert_eq!(a.contours, before);
        a.input_locked = false;
        let _ = ctx.run(
            egui::RawInput {
                events: vec![egui::Event::Paste(text.clone())],
                ..Default::default()
            },
            |ctx| a.clipboard_keys(ctx),
        );
        assert!(matches!(a.clipboard_op, Some(Operation::Place(_))));
        a.undo();
        assert!(a.clipboard_op.is_none());
        assert_eq!(a.contours, before);
        // Text input focus must retain standard native clipboard handling.
        let _ = ctx.run(
            egui::RawInput {
                events: vec![],
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    let r = ui.text_edit_singleline(&mut a.status);
                    r.request_focus();
                });
            },
        );
        let _ = ctx.run(
            egui::RawInput {
                events: vec![egui::Event::Paste(text)],
                ..Default::default()
            },
            |ctx| a.clipboard_keys(ctx),
        );
        assert!(a.clipboard_op.is_none());
        a.start_paste(data, None).unwrap();
        a.cancel();
        assert_eq!(a.contours, before);
    }
    #[test]
    fn base_point_changes_pivot_and_invalid_clipboard_never_starts_preview() {
        let mut a = sample();
        a.select_only(Some(4));
        let before = a.contours.clone();
        a.copy_base_command();
        let Some(Operation::Base(mut pending)) = a.clipboard_op.take() else {
            panic!("choose base")
        };
        pending.bundle.base = [0., 500.];
        let text = pending.bundle.text().unwrap();
        a.start_paste(Bundle::read(&text).unwrap().unwrap(), None)
            .unwrap();
        a.commit_paste([1000., 1500.]).unwrap();
        assert_eq!(
            a.contours.last().unwrap().points,
            vec![[1000., 1500.], [7000., 1500.]]
        );
        assert_eq!(&a.contours[..before.len()], before.as_slice());
        assert!(Bundle::read("unrelated clipboard text").unwrap().is_none());
        assert!(Bundle::read(&format!("{PREFIX}{{}}")).is_err());
        let mut data = a.copy_bundle().unwrap();
        data.contours[0].points.clear();
        data.contours[0].curves.clear();
        assert!(Bundle::read(&data.text().unwrap()).is_err());
    }
    #[test]
    fn spatial_copy_base_snaps_to_a_visible_node_at_its_actual_height() {
        let mut app = sample();
        app.building.enabled = true;
        app.building.floors[0].height_mm = 3000.;
        app.spatial.enabled = true;
        app.snap_settings.geometry_nodes = true;
        app.select_only(Some(4));
        app.copy_base_command();
        let project = |[x, y, z]: [f64; 3]| {
            (
                egui::pos2(
                    (50. + x * 0.1 + z * 0.03) as f32,
                    (60. + y * 0.1 + z * 0.04) as f32,
                ),
                z,
            )
        };
        let node = project([0., 500., 3000.]).0;
        let pointer = node + egui::vec2(3., 0.);
        let ctx = egui::Context::default();
        let frame = |app: &mut App, events| {
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let (rect, response) =
                            ui.allocate_exact_size(ui.available_size(), egui::Sense::click());
                        app.spatial_source_interaction(
                            ui,
                            &response,
                            rect,
                            &project,
                            0.,
                            &[],
                            None,
                        );
                    });
                },
            )
        };
        frame(&mut app, vec![]);
        let hover = frame(&mut app, vec![egui::Event::PointerMoved(pointer)]);
        assert!(hover.shapes.iter().any(|shape| {
            matches!(&shape.shape, egui::Shape::Circle(circle) if circle.center == node && circle.stroke.color == Color32::YELLOW)
        }));
        for pressed in [true, false] {
            frame(
                &mut app,
                vec![egui::Event::PointerButton {
                    pos: pointer,
                    button: egui::PointerButton::Primary,
                    pressed,
                    modifiers: egui::Modifiers::NONE,
                }],
            );
        }
        let Some(Operation::Place(pending)) = &app.clipboard_op else {
            panic!("choose placement")
        };
        assert_eq!(pending.bundle.base, [0., 500.]);
    }
    #[test]
    fn base_point_mouse_then_place_click_does_not_move_sources_or_start_a_frame() {
        fn frame(ctx: &egui::Context, a: &mut App, events: Vec<egui::Event>) -> Rect {
            let mut rect = Rect::NOTHING;
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        rect = ui.available_rect_before_wrap();
                        a.canvas(ui);
                    });
                },
            );
            rect
        }
        let button = |pos, pressed| egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        };
        let ctx = egui::Context::default();
        let mut a = sample();
        a.fit = false;
        a.camera = Camera {
            center: [3000., 2000.],
            zoom: 0.1,
            angle: 0.,
            plane: None,
        };
        a.select_only(Some(4));
        a.copy_base_command();
        let before = a.contours.clone();
        let history = a.history.len();
        let rect = frame(&ctx, &mut a, vec![]);
        frame(&ctx, &mut a, vec![]);
        let base = a.camera.screen([0., 500.], rect);
        frame(
            &ctx,
            &mut a,
            vec![egui::Event::PointerMoved(base), button(base, true)],
        );
        frame(&ctx, &mut a, vec![button(base, false)]);
        assert!(matches!(a.clipboard_op, Some(Operation::Place(_))));
        assert_eq!(a.contours, before);
        assert!(a.drag.is_none() && a.selection_start.is_none());
        let target = a.camera.screen([2000., 1500.], rect);
        frame(&ctx, &mut a, vec![egui::Event::PointerMoved(target)]);
        assert_eq!(a.contours, before);
        frame(&ctx, &mut a, vec![button(target, true)]);
        frame(&ctx, &mut a, vec![button(target, false)]);
        assert!(matches!(a.clipboard_op, Some(Operation::Place(_))));
        assert_eq!(a.contours.len(), before.len() + 1);
        assert_eq!(
            a.contours.last().unwrap().points,
            vec![[2000., 1500.], [8000., 1500.]]
        );
        assert_eq!(&a.contours[..before.len()], before.as_slice());
        assert_eq!(a.history.len(), history + 1);
    }
    #[test]
    fn copied_load_uses_active_case_preserves_units_and_keeps_mesh() {
        let mut a = sample();
        a.begin(Kind::LoadPoint);
        a.sketch_click([2500., 2500.], false);
        let data = a.copy_bundle().unwrap();
        a.load_cases
            .push(super::super::super::loads::LoadCase::new(2));
        a.active_load = 2;
        a.fe_mesh = Some(super::super::super::meshing::Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![],
            elements: vec![],
            edges: vec![],
            support_boundary: Default::default(),
        });
        let magnitude = data.contours[0].load.as_ref().unwrap().magnitude;
        a.start_paste(data, None).unwrap();
        a.commit_paste([3000., 3000.]).unwrap();
        let data = a.contours.last().unwrap().load.as_ref().unwrap();
        assert_eq!(data.case_id, 2);
        assert_eq!(data.magnitude, magnitude);
        assert!(a.fe_mesh.is_some());
    }
}
