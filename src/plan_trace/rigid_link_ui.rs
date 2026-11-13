use super::*;
use crate::plan_trace::{
    building_mesh::WELD_TOLERANCE_MM,
    model::{LevelAnchor, LevelRef, Placement},
};

fn link_geometry(
    building: &building::Building,
    column: &Contour,
    wall: &Contour,
    size: f64,
) -> Result<([Point; 2], [LevelRef; 2]), String> {
    let [axis] = column.points.as_slice() else {
        return Err("не задана ось".into());
    };
    let column_floor = building.extent(building.owner(column.id));
    let column_levels = column.placement.wall(column_floor);
    let wall_levels = wall
        .placement
        .wall(building.extent(building.owner(wall.id)));
    let low = column_levels[0].max(wall_levels[0]);
    let high = column_levels[1].min(wall_levels[1]);
    if high - low <= WELD_TOLERANCE_MM {
        return Err("со стеной нет общего участка по высоте".into());
    }

    let sampled;
    let boundary = if wall.curves.is_empty() {
        &wall.points
    } else {
        sampled = wall.mesh_boundary(wall.mesh_step(size))?;
        &sampled
    };
    let edge_count = if wall.closed() {
        boundary.len()
    } else {
        boundary.len().saturating_sub(1)
    };
    let (distance, closest) = boundary
        .iter()
        .zip(boundary.iter().cycle().skip(1))
        .take(edge_count)
        .filter_map(|(a, b)| {
            let [dx, dy] = [b[0] - a[0], b[1] - a[1]];
            let length_squared = dx * dx + dy * dy;
            if length_squared <= WELD_TOLERANCE_MM.powi(2) {
                return None;
            }
            let t =
                (((axis[0] - a[0]) * dx + (axis[1] - a[1]) * dy) / length_squared).clamp(0., 1.);
            let point = [a[0] + t * dx, a[1] + t * dy];
            let distance = (axis[0] - point[0]).hypot(axis[1] - point[1]);
            distance.is_finite().then_some((distance, point))
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .ok_or("у стены нет подходящих рёбер")?;
    if distance <= WELD_TOLERANCE_MM {
        return Err("ось уже проходит по стене".into());
    }
    let bottom = LevelRef {
        anchor: LevelAnchor::Bottom,
        offset_mm: low - column_floor[0],
    };
    let top = LevelRef {
        anchor: LevelAnchor::Top,
        offset_mm: high - column_floor[1],
    };
    Ok(([*axis, closest], [bottom, top]))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn contour(id: u64, kind: Kind, points: &[Point]) -> Contour {
        serde_json::from_value(serde_json::json!({
            "id": id, "kind": kind, "points": points, "support_open": true, "curves": []
        }))
        .unwrap()
    }

    #[test]
    fn automatic_rigid_links_follow_the_column_and_wall_and_undo_removal() {
        let mut app = App::new_project();
        app.contours = vec![
            contour(1, Kind::Support, &[[0., 0.], [2000., 0.]]),
            contour(2, Kind::Column, &[[800., 500.]]),
            contour(3, Kind::Column, &[[2500., 400.]]),
        ];
        app.contours[0].placement.bottom.offset_mm = 250.;
        app.contours[1].placement.top.offset_mm = -350.;
        app.next_id = 4;
        app.sync_building_objects();
        app.add_bound_rigid_links(1, &[2, 3]).unwrap();
        assert!(app.add_bound_rigid_links(1, &[2, 3]).is_err());
        assert_eq!(app.contours[3].placement.rigid_link_targets, Some([2, 1]));
        assert_eq!(app.contours[3].points, [[800., 500.], [800., 0.]]);
        assert_eq!(app.source_heights(&app.contours[3]), [250., 2650.]);
        assert_eq!(app.contours[4].points, [[2500., 400.], [2000., 0.]]);
        app.select_only(Some(4));
        assert_eq!(
            app.operation_sources()
                .iter()
                .map(|c| c.id)
                .collect::<Vec<_>>(),
            [1, 2, 4]
        );

        app.contours[1].translate([100., 200.]).unwrap();
        app.sync_building_objects();
        assert_eq!(app.contours[3].points, [[900., 700.], [900., 0.]]);
        let serialized = serde_json::to_vec(&app.project_snapshot()).unwrap();
        let project = serde_json::from_slice(&serialized).unwrap();
        let mut app = App::new_project();
        app.apply_project(project, "automatic-rigid-links.slab.zip".into())
            .unwrap();
        app.sync_building_objects();
        assert_eq!(app.contours[3].points, [[900., 700.], [900., 0.]]);

        app.select_only(Some(2));
        app.remove();
        app.sync_building_objects();
        assert!(!app.contours.iter().any(|c| [2, 4].contains(&c.id)));
        assert!(app.contours.iter().any(|c| c.id == 5));
        app.undo();
        app.sync_building_objects();
        assert_eq!(app.contours[3].placement.rigid_link_targets, Some([2, 1]));
        assert_eq!(app.contours[3].points, [[900., 700.], [900., 0.]]);
    }
}

impl App {
    pub(super) fn add_bound_rigid_links(
        &mut self,
        wall_id: u64,
        column_ids: &[u64],
    ) -> Result<(), String> {
        if !self.building.enabled || self.is_kind_locked(Kind::RigidLink) {
            return Err("Добавление АЖТ недоступно".into());
        }
        let wall = self
            .contours
            .iter()
            .find(|c| c.kind == Kind::Support && c.id == wall_id)
            .ok_or("Не найдена выбранная стена")?;
        let existing = self
            .contours
            .iter()
            .filter_map(|c| c.placement.rigid_link_targets)
            .collect::<std::collections::BTreeSet<_>>();
        let mut additions = Vec::new();
        let mut skipped = Vec::new();
        for column in self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column && column_ids.contains(&c.id))
        {
            let targets = [column.id, wall.id];
            if existing.contains(&targets) {
                skipped.push(format!("колонна {} уже соединена", column.id));
                continue;
            }
            let (points, [bottom, top]) =
                match link_geometry(&self.building, column, wall, self.mesh_size) {
                    Ok(geometry) => geometry,
                    Err(error) => {
                        skipped.push(format!("колонна {}: {error}", column.id));
                        continue;
                    }
                };
            let mut link = column.clone();
            link.id = self
                .next_id
                .checked_add(additions.len() as u64)
                .ok_or("Исчерпаны ID")?;
            link.kind = Kind::RigidLink;
            link.name = format!("АЖТ · колонна {} к стене {}", column.id, wall.id);
            link.placement = Placement {
                bottom,
                top,
                rigid_link_targets: Some(targets),
                ..Default::default()
            };
            link.mesh_size_mm = 0.;
            link.column_angle = 0.;
            link.calculate_effective_length = false;
            link.points = points.to_vec();
            link.curves.clear();
            link.support_open = true;
            link.placement.validate()?;
            model::validate_candidate(&self.contours, &link)?;
            additions.push((link, self.building.owner(column.id)));
        }
        let count = additions.len();
        if count == 0 {
            return Err(skipped.first().map_or_else(
                || "Выберите хотя бы одну колонну".into(),
                |reason| format!("АЖТ: {reason}"),
            ));
        }
        let next = self
            .next_id
            .checked_add(count as u64)
            .ok_or("Исчерпаны ID")?;
        self.push_contour_property_undo();
        for (link, floor) in additions {
            self.building.contour_floors.insert(link.id, floor);
            self.contours.push(link);
        }
        self.next_id = next;
        self.clear_mesh();
        self.status = if let Some(reason) = skipped.first() {
            format!(
                "Добавлено АЖТ: {count} · пропущено {} · {reason}",
                skipped.len()
            )
        } else {
            format!("Добавлено АЖТ: {count}")
        };
        Ok(())
    }

    pub(super) fn sync_bound_rigid_links(
        &mut self,
        live: &BTreeMap<u64, usize>,
        bound: Vec<usize>,
    ) {
        let mut removed = std::collections::BTreeSet::new();
        let mut changed = false;
        for index in bound {
            let link = &self.contours[index];
            let [column_id, wall_id] = link.placement.rigid_link_targets.unwrap();
            let column = live
                .get(&column_id)
                .map(|&i| &self.contours[i])
                .filter(|c| c.kind == Kind::Column);
            let wall = live
                .get(&wall_id)
                .map(|&i| &self.contours[i])
                .filter(|c| c.kind == Kind::Support);
            let (Some(column), Some(wall)) = (column, wall) else {
                removed.insert(link.id);
                continue;
            };
            let floor = self.building.owner(column.id);
            if self.building.contour_floors.get(&link.id) != Some(&floor) {
                self.building.contour_floors.insert(link.id, floor);
                changed = true;
            }
            let Ok((points, [bottom, top])) =
                link_geometry(&self.building, column, wall, self.mesh_size)
            else {
                // Keep a recoverable contour for editing. The mesher validates the current targets.
                continue;
            };
            if link.points != points || link.placement.bottom != bottom || link.placement.top != top
            {
                let link = &mut self.contours[index];
                link.points = points.to_vec();
                link.placement.bottom = bottom;
                link.placement.top = top;
                changed = true;
            }
        }
        if !removed.is_empty() {
            self.contours.retain(|c| !removed.contains(&c.id));
            self.building
                .contour_floors
                .retain(|id, _| !removed.contains(id));
            self.prune_selection();
            changed = true;
        }
        if changed {
            self.slab_cache = None;
            self.spatial.cache_reset();
            self.invalidate_scene_buffers();
        }
    }
}
