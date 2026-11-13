use super::*;

fn joined_path(first: &Contour, second: &Contour, prepend: bool) -> Result<Contour, String> {
    let (left, right) = if prepend {
        (second, first)
    } else {
        (first, second)
    };
    let mut merged = first.clone();
    if left.curves.is_empty() && right.curves.is_empty() {
        merged.points = left
            .points
            .iter()
            .chain(&right.points[1..])
            .copied()
            .collect();
        if let (Some(left_z), Some(right_z)) =
            (&left.placement.beam_z_mm, &right.placement.beam_z_mm)
        {
            merged.placement.beam_z_mm =
                Some(left_z.iter().chain(&right_z[1..]).copied().collect());
        }
    } else {
        let segments = |c: &Contour| {
            if c.curves.is_empty() {
                contour_reuse::curves(&c.points, false)
            } else {
                c.curves.clone()
            }
        };
        merged.curves = segments(left);
        merged.curves.extend(segments(right));
    }
    merged.rebuild()?;
    if merged.points.len() > 4096 || merged.curves.len() > 4096 {
        return Err("Объединённый объект превышает лимит 4096 вершин или сегментов".into());
    }
    Ok(merged)
}

impl App {
    fn contour_merge_selection(&self) -> Option<[&Contour; 2]> {
        let mut ids = self
            .selection_ids
            .iter()
            .copied()
            .chain(self.selected.filter(|id| !self.selection_ids.contains(id)));
        let first = self.view_contour(ids.next()?)?;
        let second = self.view_contour(ids.next()?)?;
        (ids.next().is_none() && self.selectable(first) && self.selectable(second))
            .then_some([first, second])
    }

    fn contour_merge_ready(&self) -> bool {
        !self.input_locked
            && self.mode.is_none()
            && self.drag.is_none()
            && self.arc_placement.is_none()
            && self.clipboard_op.is_none()
            && self.scaling_op.is_none()
            && !self.measurements.active
            && self.contour_merge_selection().is_some()
    }

    pub(super) fn contour_merge_controls(&mut self, ui: &mut egui::Ui) {
        if ui
            .add_enabled(self.contour_merge_ready(), egui::Button::new("Объединить объекты"))
            .on_hover_text("Соединить два выделенных незамкнутых объекта одного типа по общей конечной вершине. Ctrl+Z — отменить.")
            .on_disabled_hover_text("Выделите два объекта и завершите текущее редактирование.")
            .clicked()
        {
            if let Err(error) = self.merge_open_contours() {
                self.status = error;
            }
        }
    }

    fn merge_open_contours(&mut self) -> Result<(), String> {
        if !self.contour_merge_ready() {
            return Err("Выделите два объекта и завершите текущее редактирование".into());
        }
        let [a, b] = self.contour_merge_selection().unwrap();
        if a.kind != b.kind || [a, b].into_iter().any(|c| c.closed() || c.points.len() < 2) {
            return Err("Выберите два незамкнутых объекта одного типа".into());
        }
        if [a, b].into_iter().any(|c| self.is_kind_locked(c.kind)) {
            return Err("Снимите блокировку выбранных объектов".into());
        }
        let movement_locked = self.movement_locked_contours();
        if [a, b].into_iter().any(|c| movement_locked.contains(&c.id)) {
            return Err("Снимите блокировку перемещения выбранных объектов".into());
        }
        if a.kind == Kind::RigidLink || a.grid.is_some() || b.grid.is_some() {
            return Err("Объединение доступно для ломаных и открытых кривых. Для АЖТ и параметрических сеток используется отдельная геометрия".into());
        }
        if self.contours.iter().any(|c| {
            let selected = |id| id == a.id || id == b.id;
            c.parent.is_some_and(selected)
                || c.placement.cut_source.is_some_and(selected)
                || c.placement
                    .rigid_link_targets
                    .is_some_and(|ids| ids.into_iter().any(selected))
        }) {
            return Err("Один из объектов имеет зависимые объекты или связанные АЖТ".into());
        }
        if self.building.owner(a.id) != self.building.owner(b.id) {
            return Err("Выбранные объекты должны находиться на одном этаже".into());
        }
        let ends = |c: &Contour| {
            if c.kind == Kind::Beam {
                let path = self.source_beam_path(c);
                [path[0], *path.last().unwrap()]
            } else {
                [c.points[0], *c.points.last().unwrap()].map(|[x, y]| [x, y, 0.])
            }
        };
        let [start_a, end_a] = ends(a);
        let [start_b, end_b] = ends(b);
        let matches = [
            (start_a == start_b, true, true),
            (start_a == end_b, true, false),
            (end_a == start_b, false, false),
            (end_a == end_b, false, true),
        ];
        let mut joints = matches.into_iter().filter(|(common, _, _)| *common);
        let Some((_, prepend, reverse)) = joints.next().filter(|_| joints.next().is_none()) else {
            return Err("Для объединения нужна ровно одна общая конечная вершина".into());
        };
        let mut second = b.clone();
        if reverse {
            second.points.reverse();
            if let Some(z) = &mut second.placement.beam_z_mm {
                z.reverse();
            }
            second.curves.reverse();
            second.curves.iter_mut().for_each(contour_reuse::reverse);
            if second.kind == Kind::Beam {
                second.column_angle = -second.column_angle;
                second.placement.rigid_yz_mm[0] = -second.placement.rigid_yz_mm[0];
            }
        }
        let same_load = match (&a.load, &second.load) {
            (Some(a), Some(b)) => {
                a.case_id == b.case_id
                    && a.magnitude == b.magnitude
                    && a.mesh_constraint == b.mesh_constraint
                    && a.additive == b.additive
            }
            (None, None) => true,
            _ => false,
        };
        let placement_matches = if a.is_free_beam() && second.is_free_beam() {
            let mut first = a.placement.clone();
            first.slab = second.placement.slab;
            first.beam_z_mm.clone_from(&second.placement.beam_z_mm);
            first == second.placement
        } else {
            a.placement == second.placement
        };
        if a.parent != second.parent
            || !placement_matches
            || a.mesh_size_mm != second.mesh_size_mm
            || a.restraint_mask != second.restraint_mask
            || a.calculate_effective_length != second.calculate_effective_length
            || a.refinement != second.refinement
            || a.column_angle != second.column_angle
            || !same_load
            || self.stiffnesses.assignments.get(&a.id) != self.stiffnesses.assignments.get(&b.id)
            || self.stiffnesses.material_id(a.id) != self.stiffnesses.material_id(b.id)
            || self.stiffnesses.design_id(a.id) != self.stiffnesses.design_id(b.id)
        {
            return Err(
                "Свойства выбранных объектов различаются. Согласуйте их перед объединением".into(),
            );
        }
        let merged = joined_path(a, &second, prepend)?;
        model::validate_candidate(&self.contours, &merged)?;
        let (keep, remove) = (merged.id, b.id);
        self.push_contour_property_undo();
        *self.contours.iter_mut().find(|c| c.id == keep).unwrap() = merged;
        self.contours.retain(|c| c.id != remove);
        self.clear_mesh();
        self.slab_cache = None;
        self.spatial.cache_reset();
        self.rebuild_index();
        self.select_only(Some(keep));
        self.status = format!("Объекты {keep} и {remove} объединены. Ctrl+Z — отменить");
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn path(id: u64, points: [Point; 2]) -> Contour {
        serde_json::from_value(serde_json::json!({
            "id": id, "kind": "Auxiliary", "points": points, "curves": [],
            "parent": null, "support_open": true, "load": null, "refinement": null
        }))
        .unwrap()
    }

    #[test]
    fn joins_reversed_arc_and_undo_restores_nonactive_floor() {
        let mut app = App::default();
        app.building.enabled = true;
        app.spatial.enabled = true;
        app.selection_all_floors = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        let start = [0., 0.];
        let joint = [1000., 0.];
        let end = [2000., 0.];
        let through = [1500., 400.];
        let mut arc = path(2, [end, joint]);
        arc.curves.push(Curve::Arc([end, through, joint]));
        arc.rebuild().unwrap();
        app.contours = vec![path(1, [start, joint]), arc];
        app.building.contour_floors = BTreeMap::from([(1, 2), (2, 2)]);
        let original = app.contours.clone();
        app.select_ids([1, 2], egui::Modifiers::NONE);
        app.merge_open_contours().unwrap();
        assert_eq!(app.contours.len(), 1);
        let joined = &app.contours[0];
        assert_eq!(joined.id, 1);
        assert_eq!(
            joined.curves,
            [Curve::Line(start, joint), Curve::Arc([joint, through, end])]
        );
        assert_eq!(joined.points.iter().filter(|&&p| p == joint).count(), 1);
        assert_eq!(app.selected, Some(1));
        app.sync_building_objects();
        assert!(!app.building.contour_floors.contains_key(&2));
        app.undo();
        assert_eq!(app.contours, original);
        assert_eq!(app.building.owner(2), 2);
    }

    #[test]
    fn different_line_load_intensities_leave_both_contours_unchanged() {
        use crate::plan_trace::loads::{ForceUnits, LoadData, Magnitude};
        let mut app = App::default();
        app.contours = vec![
            path(1, [[0., 0.], [1000., 0.]]),
            path(2, [[1000., 0.], [1000., 1000.]]),
        ];
        for c in &mut app.contours {
            c.kind = Kind::LoadLine;
            c.load = Some(LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                name: format!("Линия {}", c.id),
                mesh_constraint: false,
                case_id: 1,
                magnitude: Magnitude {
                    value: c.id as f64,
                    units: ForceUnits::TonForce,
                },
                closed: false,
            });
        }
        let original = app.contours.clone();
        app.select_ids([1, 2], egui::Modifiers::NONE);
        assert!(app
            .merge_open_contours()
            .unwrap_err()
            .contains("различаются"));
        assert_eq!(app.contours, original);
        assert!(app.history.is_empty());
    }
}
