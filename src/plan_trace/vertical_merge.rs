//! Join isolated vertical members without changing their physical geometry.
use super::*;
use mini_fem::geo::{Intersects, LineString, Polygon};
const EPS: f64 = 0.05;
fn same_path(a: &Contour, b: &Contour) -> bool {
    if a.kind != b.kind || a.closed() != b.closed() {
        return false;
    }
    let closed = a.closed();
    let a = if a.kind == Kind::Column {
        a.points.clone()
    } else {
        model::without_collinear_spurs(a.points.clone())
    };
    let b = if b.kind == Kind::Column {
        b.points.clone()
    } else {
        model::without_collinear_spurs(b.points.clone())
    };
    if a.len() != b.len() || a.is_empty() {
        return false;
    }
    let same = |p: Point, q: Point| (p[0] - q[0]).hypot(p[1] - q[1]) <= EPS;
    if a.len() == 1 {
        return same(a[0], b[0]);
    }
    if !a.is_empty() {
        for start in 0..b.len() {
            for reverse in [false, true] {
                if !closed && ((!reverse && start != 0) || (reverse && start != b.len() - 1)) {
                    continue;
                }
                if a.iter().enumerate().all(|(i, p)| {
                    same(
                        *p,
                        b[if reverse {
                            (start + b.len() - i) % b.len()
                        } else {
                            (start + i) % b.len()
                        }],
                    )
                }) {
                    return true;
                }
            }
        }
    }
    false
}
fn path(c: &Contour) -> LineString<f64> {
    let mut p = c.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
    if c.closed() && p.len() > 1 {
        p.push(p[0]);
    }
    LineString::from(p)
}
impl App {
    fn same_vertical_stiffness(&self, a: &Contour, b: &Contour) -> bool {
        if self.stiffnesses.material_id(a.id) != self.stiffnesses.material_id(b.id) {
            return false;
        }
        let equal = |variant| match (
            self.stiffnesses.resolved_entry_for(a.id, variant),
            self.stiffnesses.resolved_entry_for(b.id, variant),
        ) {
            (Some(a), Some(b)) => {
                a.family == b.family && a.material == b.material && a.width_mm == b.width_mm
            }
            _ => false,
        };
        equal(0)
            && (!self.stiffnesses.alternative_enabled || equal(1))
            && a.mesh_size_mm == b.mesh_size_mm
            && (a.column_angle - b.column_angle).sin().abs() < 1e-8
    }
    fn vertical_separator(&self, all: &[Contour], a: &Contour, b: &Contour, z: f64) -> bool {
        let footprint = if a.kind == Kind::Column {
            self.support_height.columns.get(&a.id).map(|s| {
                let mut p = crate::plan_trace::columns::section_corners(a, s)
                    .iter()
                    .map(|p| (p[0], p[1]))
                    .collect::<Vec<_>>();
                p.push(p[0]);
                Polygon::new(LineString::from(p), vec![])
            })
        } else {
            None
        };
        let axis = path(a);
        all.iter()
            .filter(|c| c.id != a.id && c.id != b.id)
            .any(|c| {
                let h = self.source_heights(c);
                match c.kind {
                    Kind::Slab if (h[0] - z).abs() <= EPS => {
                        let mut ring = c.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
                        if ring.is_empty() {
                            return false;
                        }
                        ring.push(ring[0]);
                        let holes = all
                            .iter()
                            .filter(|h| h.active_hole() && h.parent == Some(c.id))
                            .map(|h| {
                                let mut p =
                                    h.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
                                if !p.is_empty() {
                                    p.push(p[0]);
                                }
                                LineString::from(p)
                            })
                            .collect();
                        let poly = Polygon::new(LineString::from(ring), holes);
                        footprint
                            .as_ref()
                            .map_or_else(|| poly.intersects(&axis), |p| poly.intersects(p))
                    }
                    Kind::Support if h[0] <= z + EPS && h[1] >= z - EPS => {
                        let neighbor = path(c);
                        if !footprint
                            .as_ref()
                            .map_or_else(|| neighbor.intersects(&axis), |p| p.intersects(&neighbor))
                        {
                            return false;
                        }
                        if a.kind != Kind::Support {
                            return true;
                        }
                        // A neighboring wall split at the same level is not a
                        // separator when its matching half continues the joint.
                        !all.iter().any(|other| {
                            if other.id == a.id
                                || other.id == b.id
                                || other.id == c.id
                                || !same_path(c, other)
                                || !self.same_vertical_stiffness(c, other)
                            {
                                return false;
                            }
                            let other_h = self.source_heights(other);
                            ((h[1] - z).abs() <= EPS && (other_h[0] - z).abs() <= EPS)
                                || ((h[0] - z).abs() <= EPS && (other_h[1] - z).abs() <= EPS)
                        })
                    }
                    Kind::Beam | Kind::Restraint if (h[0] - z).abs() <= EPS => footprint
                        .as_ref()
                        .map_or_else(|| path(c).intersects(&axis), |p| p.intersects(&path(c))),
                    _ => false,
                }
            })
    }
    pub(super) fn vertical_merge_ready(&self) -> bool {
        self.building.enabled
            && !self.input_locked
            && self.mode.is_none()
            && self.drag.is_none()
            && self.arc_placement.is_none()
            && self.clipboard_op.is_none()
            && !self.measurements.active
            && self
                .contours
                .iter()
                .filter(|c| {
                    self.operation_target(c.id) && matches!(c.kind, Kind::Support | Kind::Column)
                })
                .all(|c| !self.is_kind_locked(c.kind))
    }
    pub(super) fn merge_vertical_members(&mut self) -> Result<usize, String> {
        if !self.vertical_merge_ready() {
            return Err("Включите этажи, снимите блокировки и завершите редактирование".into());
        }
        let movement_locked = self.movement_locked_contours();
        let mut all = self.contours.clone();
        let mut count = 0;
        let mut by_floor = BTreeMap::<u64, Vec<usize>>::new();
        for (i, c) in all.iter().enumerate().filter(|(_, c)| {
            self.operation_target(c.id) && matches!(c.kind, Kind::Support | Kind::Column)
        }) {
            by_floor
                .entry(self.building.owner(c.id))
                .or_default()
                .push(i);
        }
        let mut order = by_floor.values().flatten().copied().collect::<Vec<_>>();
        order.sort_by(|&i, &j| {
            self.source_heights(&all[i])[0].total_cmp(&self.source_heights(&all[j])[0])
        });
        let mut removed = std::collections::BTreeSet::new();
        for i in order {
            if removed.contains(&i) || movement_locked.contains(&all[i].id) {
                continue;
            }
            loop {
                let a = &all[i];
                let top = *self.source_heights(a).last().unwrap();
                let Some(floor) = self.building.floors.iter().find(|f| {
                    (f.elevation_mm - top).abs() <= EPS
                        && f.elevation_mm
                            > self.building.floor(self.building.owner(a.id)).elevation_mm + EPS
                }) else {
                    break;
                };
                let mut candidates = by_floor
                    .get(&floor.id)
                    .into_iter()
                    .flatten()
                    .copied()
                    .filter(|&j| {
                        !removed.contains(&j)
                            && all[j].id != a.id
                            && (self.source_heights(&all[j])[0] - top).abs() <= EPS
                            && same_path(a, &all[j])
                            && self.same_vertical_stiffness(a, &all[j])
                    });
                let Some(j) = candidates.next() else {
                    break;
                };
                if candidates.next().is_some() || movement_locked.contains(&all[j].id) {
                    break;
                }
                // Use original joints so merging one wall does not change
                // whether its neighbor can merge in the same operation.
                if self.vertical_separator(&self.contours, a, &all[j], top) {
                    break;
                }
                let new_top = *self.source_heights(&all[j]).last().unwrap();
                let base = self.building.floor(self.building.owner(a.id)).elevation_mm;
                all[i].placement.top = model::LevelRef {
                    anchor: model::LevelAnchor::Bottom,
                    offset_mm: new_top - base,
                };
                all[i].calculate_effective_length |= all[j].calculate_effective_length;
                all[i].placement.validate()?;
                removed.insert(j);
                count += 1;
            }
        }
        all = all
            .into_iter()
            .enumerate()
            .filter(|(i, _)| !removed.contains(i))
            .map(|(_, c)| c)
            .collect();
        if count > 0 {
            self.history.push(self.contours.clone());
            self.contours = all;
            // Keep stable owner/property maps for removed IDs: undo restores the
            // original contours with their assignments. Project snapshots filter
            // property maps by live IDs, as for ordinary contour deletion.
            self.select_only(None);
            self.clear_mesh();
            self.slab_cache = None;
            self.spatial.cache_reset();
            self.rebuild_index();
        }
        self.status = format!("Объединено вертикальных конструкций: {count}. Ctrl+Z — отменить");
        Ok(count)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn fixture(kind: &str) -> App {
        let mut a = App::default();
        a.building.enabled = true;
        a.building.floors[0].elevation_mm = 0.;
        a.building.floors[0].height_mm = 3000.;
        a.building.floors.push(building::Floor {
            id: 2,
            name: "Second".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        let c = |id| {
            serde_json::from_value(serde_json::json!({"id":id,"kind":kind,"points":if kind=="Column"{vec![[1000.,1000.]]}else{vec![[0.,1000.],[2000.,1000.]]},"parent":null,"support_open":true,"load":null,"refinement":null,"curves":[]})).unwrap()
        };
        a.contours = vec![c(1), c(2)];
        a.building.contour_floors = BTreeMap::from([(1, 1), (2, 2)]);
        a.sync_stiffness_sources();
        a
    }
    #[test]
    fn isolated_columns_and_walls_merge_physical_height_and_undo() {
        for kind in ["Column", "Support"] {
            let mut a = fixture(kind);
            let old = a.contours.clone();
            assert_eq!(a.merge_vertical_members().unwrap(), 1);
            assert_eq!(a.contours.len(), 1);
            assert_eq!(a.source_heights(&a.contours[0]), vec![0., 6000.]);
            let scene =
                spatial_ui::building_source_scene(&a.contours, &a.support_height, &a.building)
                    .unwrap();
            assert!(scene.xyz.iter().any(|p| p[2] == 6000.));
            a.undo();
            assert_eq!(a.contours, old);
        }
    }
    #[test]
    fn floor_plate_and_changed_stiffness_prevent_merging() {
        let mut a = fixture("Column");
        let slab=serde_json::from_value(serde_json::json!({"id":3,"kind":"Slab","points":[[0.,0.],[3000.,0.],[3000.,3000.],[0.,3000.]],"parent":null,"support_open":false,"load":null,"refinement":null,"curves":[]})).unwrap();
        a.contours.push(slab);
        a.building.contour_floors.insert(3, 2);
        a.sync_stiffness_sources();
        assert_eq!(a.merge_vertical_members().unwrap(), 0);
        let mut a = fixture("Support");
        let mut e = a.stiffnesses.entries[&2].clone();
        e.material.thickness_mm += 100.;
        a.stiffnesses.entries.insert(5, e);
        a.stiffnesses.assignments.insert(2, 5);
        assert_eq!(a.merge_vertical_members().unwrap(), 0);
    }
    #[test]
    fn matching_wall_returns_merge_without_a_separator() {
        let mut a = fixture("Support");
        for (id, floor) in [(3, 1), (4, 2)] {
            let mut wall = a.contours[0].clone();
            wall.id = id;
            wall.points = vec![[0., 1000.], [0., 2000.]];
            a.contours.push(wall);
            a.building.contour_floors.insert(id, floor);
        }
        a.sync_stiffness_sources();
        let mut interrupted = fixture("Support");
        interrupted.contours.push(a.contours[2].clone());
        interrupted.building.contour_floors.insert(3, 1);
        interrupted.sync_stiffness_sources();
        assert_eq!(interrupted.merge_vertical_members().unwrap(), 0);
        assert_eq!(a.merge_vertical_members().unwrap(), 2);
        assert!(a
            .contours
            .iter()
            .all(|c| a.source_heights(c) == vec![0., 6000.]));
    }
    #[test]
    fn identical_sections_with_different_contour_materials_stay_separate() {
        let mut a = fixture("Support");
        assert_eq!(a.stiffnesses.assignments[&1], a.stiffnesses.assignments[&2]);
        let old = a.stiffnesses.material_id(1).unwrap();
        let mut material = a.stiffnesses.elastic[&old].clone();
        material.name = "Другой материал стены".into();
        let other = a.stiffnesses.import_named_material(material, None);
        a.stiffnesses.contour_materials.insert(2, other);
        assert_eq!(a.merge_vertical_members().unwrap(), 0);
    }
}
