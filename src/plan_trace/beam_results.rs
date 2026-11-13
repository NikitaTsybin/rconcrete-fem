use super::*;
impl App {
    pub(super) fn beam_result_visible(&self, id: u64) -> bool {
        self.beam_view_visible(id)
            && self
                .view_contour(id)
                .is_none_or(|c| self.on_view_floor(c) && !self.object_hidden(c))
    }
    pub(super) fn beam_result_active(&self, id: u64) -> bool {
        self.show_results
            && self.punching_view.is_none()
            && (17..=22).contains(&self.result_field)
            && self
                .selected_result()
                .and_then(|r| r.shell.as_ref())
                .is_some_and(|s| s.beam_forces.iter().any(|b| b.id == id))
    }
    pub(super) fn beam_field_values(
        &self,
        section_scene: Option<&crate::plan_trace::fem::shell::Scene>,
    ) -> Vec<f64> {
        let Some(forces) = self
            .selected_result()
            .and_then(|r| r.shell.as_ref())
            .map(|s| &s.beam_forces)
        else {
            return vec![];
        };
        let component = self.result_field.saturating_sub(17);
        if component >= 6 {
            return vec![];
        }
        let factor = self.force_result_factor();
        forces
            .iter()
            .enumerate()
            .filter(|(i, b)| self.fe_item_visible((1, *i)) && self.beam_result_visible(b.id))
            .filter(|(_, b)| {
                !self.building.enabled
                    || self.spatial.enabled
                    || self.building.owner(b.id) == self.building.active
            })
            .filter_map(|(index, beam)| {
                let [lo, hi] = if let Some(scene) = section_scene {
                    let geometry = scene.beams.get(index).filter(|g| g.id == beam.id)?;
                    let arm = geometry.rigid_arm(&scene.xyz).unwrap_or([0.; 3]);
                    let points = geometry
                        .nodes
                        .map(|n| std::array::from_fn(|axis| scene.xyz[n][axis] + arm[axis]));
                    self.sections.segment_interval(points)?
                } else {
                    [0., 1.]
                };
                let shear = match component {
                    2 => Some(5),
                    3 => Some(4),
                    _ => None,
                };
                let stationary = shear.and_then(|shear| {
                    let delta = beam.ends[1][shear] - beam.ends[0][shear];
                    (delta.abs() > 1e-15)
                        .then(|| -beam.ends[0][shear] / delta)
                        .filter(|&t| t > lo && t < hi)
                });
                Some((
                    beam,
                    [Some(lo), Some((lo + hi) * 0.5), Some(hi), stationary],
                ))
            })
            .flat_map(|(beam, points)| {
                points
                    .into_iter()
                    .flatten()
                    .map(move |t| beam.at(t)[component] * factor)
            })
            .collect()
    }
    pub(super) fn paint_beam_results(&self, painter: &egui::Painter, rect: Rect) {
        if !self.show_results || !(17..=22).contains(&self.result_field) {
            return;
        }
        let Some(shell) = self.selected_result().and_then(|r| r.shell.as_ref()) else {
            return;
        };
        let Some(range) = self.result_range_cached() else {
            return;
        };
        let scale = &self.result_scales[self.result_field];
        let component = self.result_field - 17;
        for (_, beam) in shell
            .beam_forces
            .iter()
            .enumerate()
            .filter(|(i, b)| self.fe_item_visible((1, *i)) && self.beam_result_visible(b.id))
        {
            if self.building.enabled && self.building.owner(beam.id) != self.building.active {
                continue;
            }
            for index in 0..16 {
                let start = index as f64 / 16.;
                let end = (index + 1) as f64 / 16.;
                let value = beam.at((start + end) * 0.5)[component] * self.force_result_factor();
                let Some(color) = scale.color(value, range) else {
                    continue;
                };
                let point = |t: f64| {
                    std::array::from_fn(|a| {
                        beam.points[0][a] + t * (beam.points[1][a] - beam.points[0][a])
                    })
                };
                if let Some(line) = clip_line(
                    self.camera.screen64(point(start), rect),
                    self.camera.screen64(point(end), rect),
                    rect,
                ) {
                    painter.line_segment(line, Stroke::new(5_f32, color));
                }
            }
        }
    }
    pub(super) fn paint_beam_annotations(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pointer: Option<Pos2>,
    ) {
        let Some(shell) = self.selected_result().and_then(|r| r.shell.as_ref()) else {
            return;
        };
        let Some(range) = self.result_range_cached() else {
            return;
        };
        let scale = &self.result_scales[self.result_field];
        let component = self.result_field - 17;
        if self.show_result_values {
            let mut occupied = vec![];
            for (_, beam) in
                shell.beam_forces.iter().enumerate().filter(|(i, b)| {
                    self.fe_item_visible((1, *i)) && self.beam_result_visible(b.id)
                })
            {
                if self.building.enabled && self.building.owner(beam.id) != self.building.active {
                    continue;
                }
                if self.building.enabled && self.building.owner(beam.id) != self.building.active {
                    continue;
                }
                for (t, section) in beam.sections() {
                    let p = std::array::from_fn(|i| {
                        beam.points[0][i] + t * (beam.points[1][i] - beam.points[0][i])
                    });
                    let at = self.camera.screen(p, rect);
                    if !rect.contains(at) || occupied.iter().any(|r: &Rect| r.contains(at)) {
                        continue;
                    }
                    let value = section[component] * self.force_result_factor();
                    if scale.color(value, range).is_none() {
                        continue;
                    }
                    occupied.push(fem_ui::result_label(
                        painter,
                        at,
                        self.display_text.number(value),
                        rect,
                        self.display_text,
                    ));
                }
            }
        }
        if self.show_results {
            scale.legend(painter, rect, range, self.display_text);
        }
        if let Some(pointer) = pointer {
            let world = self.camera.world(pointer, rect);
            let hit = shell
                .beam_forces
                .iter()
                .enumerate()
                .filter(|(i, b)| self.fe_item_visible((1, *i)) && self.beam_result_visible(b.id))
                .filter_map(|(_, beam)| {
                    let d = [
                        beam.points[1][0] - beam.points[0][0],
                        beam.points[1][1] - beam.points[0][1],
                    ];
                    let length = d[0] * d[0] + d[1] * d[1];
                    if length <= 0. {
                        return None;
                    }
                    let t = (((world[0] - beam.points[0][0]) * d[0]
                        + (world[1] - beam.points[0][1]) * d[1])
                        / length)
                        .clamp(0., 1.);
                    let distance = (world[0] - beam.points[0][0] - t * d[0])
                        .hypot(world[1] - beam.points[0][1] - t * d[1])
                        * self.camera.zoom;
                    (distance <= 9.).then_some((beam, t, distance))
                })
                .min_by(|a, b| a.2.total_cmp(&b.2));
            if let Some((beam, t, _)) = hit {
                let value = beam.at(t)[component] * self.force_result_factor();
                fem_ui::result_label(
                    painter,
                    pointer + egui::vec2(16., 18.),
                    format!(
                        "{}\nБалка {} · {}",
                        self.result_caption(),
                        beam.id,
                        self.display_text.number(value)
                    ),
                    rect,
                    self.display_text,
                );
            }
        }
    }
}
