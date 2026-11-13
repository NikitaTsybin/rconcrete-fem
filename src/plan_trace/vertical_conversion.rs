use super::*;
use crate::plan_trace::stiffness::{Family, Library};

fn straight_span(c: &Contour) -> Option<[Point; 2]> {
    if c.closed()
        || c.grid.is_some()
        || c.curves
            .iter()
            .any(|curve| !matches!(curve, Curve::Line(..)))
    {
        return None;
    }
    let ends = [*c.points.first()?, *c.points.last()?];
    let d = std::array::from_fn::<_, 2, _>(|i| ends[1][i] - ends[0][i]);
    let length = d[0].hypot(d[1]);
    if length <= 1e-5 {
        return None;
    }
    let mut last = -1e-5;
    for p in &c.points {
        let q = [p[0] - ends[0][0], p[1] - ends[0][1]];
        let along = (q[0] * d[0] + q[1] * d[1]) / length;
        if (q[0] * d[1] - q[1] * d[0]).abs() / length > 1e-5
            || along < last - 1e-5
            || along > length + 1e-5
        {
            return None;
        }
        last = along;
    }
    Some(ends)
}

fn section_for(library: &mut Library, old: u64, family: Family, b: f64, h: f64) -> u64 {
    let color = library.colors.get(&old).copied();
    if let Some((&id, _)) = library.entries.iter().find(|(id, e)| {
        e.family == family
            && e.material.thickness_mm == h
            && (family == Family::Wall || e.width_mm == b)
            && library.colors.get(id).copied() == color
    }) {
        return id;
    }
    let mut section = library.entries[&old].material;
    section.thickness_mm = h;
    let id = library.add(family, section);
    library.entries.get_mut(&id).unwrap().width_mm = b;
    if let Some(color) = color {
        library.colors.insert(id, color);
    }
    id
}

impl App {
    fn vertical_conversion_ready(&self, from: Kind, to: Kind) -> bool {
        !self.input_locked
            && !self.is_kind_locked(from)
            && !self.is_kind_locked(to)
            && self.mode.is_none()
            && self.drag.is_none()
            && self.arc_placement.is_none()
            && self.clipboard_op.is_none()
            && self.scaling_op.is_none()
            && !self.measurements.active
            && self
                .contours
                .iter()
                .any(|c| c.kind == from && self.operation_target(c.id))
    }

    pub(super) fn vertical_conversion_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            for (from, to, label) in [
                (Kind::Support, Kind::Column, "Стены в колонны"),
                (Kind::Column, Kind::Support, "Колонны в стены"),
            ] {
                if ui.add_enabled(self.vertical_conversion_ready(from, to), egui::Button::new(label))
                    .on_hover_text("Действует на выбранные объекты либо на все этого типа. Сохраняет этаж, уровни и материал. Ctrl+Z — отмена.")
                    .clicked() {
                    if let Err(error) = self.convert_vertical(from, to) { self.status = error; }
                }
            }
        });
    }

    fn convert_vertical(&mut self, from: Kind, to: Kind) -> Result<usize, String> {
        if !self.vertical_conversion_ready(from, to) {
            return Err("Преобразование сейчас недоступно".into());
        }
        let movement_locked = self.movement_locked_contours();
        self.sync_stiffness_sources();
        let mut stiffness = self.stiffnesses.clone();
        let mut contours = self.contours.clone();
        let mut converted = Vec::new();
        let mut skipped = Vec::new();
        for c in contours.iter_mut().filter(|c| {
            c.kind == from && self.operation_target(c.id) && !movement_locked.contains(&c.id)
        }) {
            let old = stiffness.assignments[&c.id];
            let section = &stiffness.entries[&old];
            let h = section.material.thickness_mm;
            let (family, b, h) = if from == Kind::Support {
                let Some([a, b]) = straight_span(c) else {
                    skipped.push(c.id);
                    continue;
                };
                let d = [b[0] - a[0], b[1] - a[1]];
                let length = d[0].hypot(d[1]);
                let angle = d[1].atan2(d[0]);
                c.column_angle = if length >= h {
                    angle - std::f64::consts::FRAC_PI_2
                } else {
                    angle
                };
                c.points = vec![std::array::from_fn(|i| (a[i] + b[i]) * 0.5)];
                (Family::Column, h.min(length), h.max(length))
            } else {
                if c.placement.rigid_yz_mm != [0.; 2] {
                    skipped.push(c.id);
                    continue;
                }
                let b = section.width_mm;
                let center = c.points[0];
                let angle = c.column_angle
                    + if h >= b {
                        std::f64::consts::FRAC_PI_2
                    } else {
                        0.
                    };
                let d = [angle.cos() * h.max(b) * 0.5, angle.sin() * h.max(b) * 0.5];
                c.points = [-1., 1.]
                    .map(|s| [center[0] + s * d[0], center[1] + s * d[1]])
                    .to_vec();
                c.column_angle = 0.;
                (Family::Wall, h.min(b), h.min(b))
            };
            c.kind = to;
            c.support_open = true;
            c.curves.clear();
            let id = section_for(&mut stiffness, old, family, b, h);
            stiffness.assignments.insert(c.id, id);
            converted.push(c.id);
        }
        let converted_ids = converted
            .iter()
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        for c in contours.iter().filter(|c| converted_ids.contains(&c.id)) {
            model::validate_candidate(&contours, c).map_err(|e| format!("Объект {}: {e}", c.id))?;
        }
        if !converted.is_empty() {
            self.push_contour_property_undo();
            self.stiffnesses = stiffness;
            self.contours = contours;
            self.sync_stiffness_sources();
            self.clear_mesh();
            self.slab_cache = None;
            self.spatial.cache_reset();
            self.rebuild_index();
            self.prune_selection();
        }
        let mut message = format!("Преобразовано: {}", converted.len());
        if !skipped.is_empty() {
            message.push_str(&format!(
                " · пропущено: {} (ломаные/кривые стены или колонны с жёсткими вставками)",
                skipped.len()
            ));
        }
        self.status = message;
        Ok(converted.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn rotated_wall_column_roundtrip_and_undo_preserve_material_and_levels() {
        let mut app = App::default();
        app.building.enabled = true;
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        app.sketch_click([1200., 900.], false);
        app.finish();
        app.cancel();
        app.sync_stiffness_sources();
        let wall = app.contours[0].clone();
        let section = app.stiffnesses.assignments[&wall.id];
        let material = app.stiffnesses.material_id(wall.id).unwrap();
        let h = app.stiffnesses.entries[&section].material.thickness_mm;
        assert_eq!(
            app.convert_vertical(Kind::Support, Kind::Column).unwrap(),
            1
        );
        let column = app.contours[0].clone();
        assert_eq!(column.id, wall.id);
        assert_eq!(column.points, [[600., 450.]]);
        assert_eq!(column.placement, wall.placement);
        let new = app.stiffnesses.assignments[&wall.id];
        assert_eq!(app.stiffnesses.material_id(wall.id), Some(material));
        assert_eq!(app.stiffnesses.entries[&new].material.thickness_mm, 1500.);
        assert_eq!(app.stiffnesses.entries[&new].width_mm, h);
        assert_eq!(
            app.convert_vertical(Kind::Column, Kind::Support).unwrap(),
            1
        );
        for (a, b) in app.contours[0].points.iter().zip(&wall.points) {
            assert!((a[0] - b[0]).hypot(a[1] - b[1]) < 1e-8);
        }
        app.undo();
        assert_eq!(app.contours[0], column);
        assert_eq!(app.stiffnesses.assignments[&wall.id], new);
        app.undo();
        assert_eq!(app.contours[0], wall);
        assert_eq!(app.stiffnesses.assignments[&wall.id], section);
        assert_eq!(app.stiffnesses.material_id(wall.id), Some(material));
    }
}
