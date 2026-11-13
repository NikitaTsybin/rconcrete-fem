//! Geometric pilot: U-shaped control lines at free ends of open wall axes.
use super::*;
const TAG: &str = "Продавливание: торец";
#[derive(Clone, Copy)]
pub(super) enum SlabSide {
    Above,
    Below,
}
impl SlabSide {
    fn level(self, heights: &[f64]) -> f64 {
        match self {
            Self::Above => *heights.last().unwrap(),
            Self::Below => heights[0],
        }
    }
    fn adjoining_wall(self, heights: &[f64], level: f64) -> bool {
        const EPS: f64 = 1e-5;
        match self {
            Self::Above => heights[0] < level - EPS && heights[1] >= level - EPS,
            Self::Below => heights[0] <= level + EPS && heights[1] > level + EPS,
        }
    }
}
pub(super) fn is_punching(c: &Contour) -> bool {
    super::super::fem::cut_forces::is_cut(c)
}
fn cross(a: Point, b: Point) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}
fn sub(a: Point, b: Point) -> Point {
    [a[0] - b[0], a[1] - b[1]]
}
impl App {
    pub(super) fn refresh_summation_results(&mut self) {
        let mut changed = false;
        for results in [&mut self.analysis, &mut self.alternative_cache.analysis]
            .into_iter()
            .flatten()
        {
            for case in &mut results.cases {
                for c in self.contours.iter().filter(|c| c.kind == Kind::Punching) {
                    if let Some(r) = case.punching.get_mut(&c.id) {
                        let inactive = c
                            .refinement
                            .as_ref()
                            .map_or(&[][..], |r| r.inactive_edges.as_slice());
                        changed |= r.select_method(self.summation_method);
                        changed |= r.select_edges(inactive);
                    }
                }
            }
        }
        if changed {
            self.combination_cache = None;
            self.sync_combination_result();
            self.result_generation = self.result_generation.wrapping_add(1);
            self.invalidate_scene_buffers();
        }
    }
    pub(super) fn punching_edge_buttons(&mut self, ui: &mut egui::Ui, rect: Rect) {
        if self.mode.is_some() || self.drag.is_some() || self.spatial.enabled {
            return;
        }
        let mut toggle = None;
        for c in self
            .contours
            .iter()
            .filter(|c| is_punching(c) && self.is_selected(c.id) && self.contour_visible(c))
        {
            let working = super::super::fem::cut_forces::control_edges(c, &self.contours);
            for i in 0..c.points.len() {
                let a = c.points[i];
                let b = c.points[(i + 1) % c.points.len()];
                let center = self
                    .camera
                    .screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], rect);
                if !rect.contains(center) {
                    continue;
                }
                let enabled = working.contains(&[a, b]);
                egui::Area::new(egui::Id::new(("punching_edge", c.id, i)))
                    .order(egui::Order::Foreground)
                    .fixed_pos(center + egui::vec2(10., 10.))
                    .show(ui.ctx(), |ui| {
                        let (r, response) =
                            ui.allocate_exact_size(egui::vec2(20., 20.), egui::Sense::click());
                        let color = if enabled {
                            Color32::from_rgb(80, 210, 140)
                        } else {
                            ui.visuals().text_color()
                        };
                        ui.painter().rect_filled(r, 3., ui.visuals().panel_fill);
                        let body = Rect::from_center_size(
                            r.center() + egui::vec2(0., 3.),
                            egui::vec2(10., 8.),
                        );
                        ui.painter().rect_stroke(
                            body,
                            1.,
                            Stroke::new(1.5, color),
                            egui::StrokeKind::Inside,
                        );
                        let x = if enabled { 0. } else { 3. };
                        ui.painter().line_segment(
                            [
                                r.center() + egui::vec2(-4. + x, -1.),
                                r.center() + egui::vec2(-4. + x, -6.),
                            ],
                            Stroke::new(1.5, color),
                        );
                        ui.painter().line_segment(
                            [
                                r.center() + egui::vec2(-4. + x, -6.),
                                r.center() + egui::vec2(4. + x, -6.),
                            ],
                            Stroke::new(1.5, color),
                        );
                        if enabled {
                            ui.painter().line_segment(
                                [
                                    r.center() + egui::vec2(4., -6.),
                                    r.center() + egui::vec2(4., -1.),
                                ],
                                Stroke::new(1.5, color),
                            );
                        }
                        if response
                            .on_hover_text(if enabled {
                                "Рабочее ребро: собирает усилия. Нажмите, чтобы исключить"
                            } else {
                                "Только замыкание и триангуляция. Нажмите, чтобы включить сбор"
                            })
                            .clicked()
                        {
                            toggle = Some((c.id, i));
                        }
                    });
            }
        }
        if let Some((id, i)) = toggle {
            self.history.push(self.contours.clone());
            if let Some(r) = self
                .contours
                .iter_mut()
                .find(|c| c.id == id)
                .and_then(|c| c.refinement.as_mut())
            {
                if r.inactive_edges.contains(&i) {
                    r.inactive_edges.retain(|j| *j != i);
                } else {
                    r.inactive_edges.push(i);
                }
            }
            self.refresh_summation_results();
            self.invalidate_scene_buffers();
            self.status = if self.analysis.is_some() {
                "Рабочие рёбра изменены. Центр и усилия обновлены без повторного расчёта"
            } else {
                "Рабочие рёбра изменены. Будут учтены при расчёте"
            }
            .into();
        }
    }
    pub(super) fn paint_source_contour_outline(
        &self,
        painter: &egui::Painter,
        c: &Contour,
        rect: Rect,
        stroke: Stroke,
    ) {
        if !is_punching(c) {
            draw_path(painter, &c.points, c.closed(), self.camera, rect, stroke);
            return;
        }
        let working = super::super::fem::cut_forces::control_edges(c, &self.contours);
        for edge in c
            .points
            .iter()
            .copied()
            .zip(c.points.iter().copied().cycle().skip(1))
            .take(c.points.len())
            .map(|(a, b)| [a, b])
        {
            let screen = edge.map(|p| self.camera.screen(p, rect));
            if working.contains(&edge) {
                painter.line_segment(screen, stroke);
            } else {
                painter.extend(egui::Shape::dashed_line(&screen, stroke, 6., 4.));
            }
        }
    }

    pub(super) fn punching_results_controls(&mut self, ui: &mut egui::Ui) {
        if !self.contours.iter().any(is_punching) {
            return;
        }
        if self.result_metadata().is_none_or(|r| r.punching.is_empty()) {
            egui::CollapsingHeader::new("Области суммирования нагрузок").default_open(false).show(ui,|ui|{ui.label("Данных нет. Повторите расчёт после обновления приложения; при изменении объектов сначала перестройте сетку.");});
            return;
        }
        egui::CollapsingHeader::new("Области суммирования нагрузок")
            .default_open(false)
            .show(ui, |ui| {
                let previous = self.summation_method;
                egui::ComboBox::from_id_salt("summation_method")
                    .selected_text(self.summation_method.label())
                    .show_ui(ui, |ui| {
                        for method in [
                            super::super::fem::cut_forces::Method::NodalEquilibrium,
                            super::super::fem::cut_forces::Method::ElementEdges,
                        ] {
                            ui.selectable_value(&mut self.summation_method, method, method.label());
                        }
                    });
                if previous != self.summation_method {
                    self.refresh_summation_results();
                }
                ui.horizontal(|ui| {
                    for (i, label) in ["F", "Mx", "My", "F | Mx | My", "Fi"].iter().enumerate() {
                        if ui
                            .selectable_label(
                                self.punching_view == Some(i) && self.show_results,
                                *label,
                            )
                            .clicked()
                        {
                            self.punching_view =
                                if self.punching_view == Some(i) && self.show_results {
                                    None
                                } else {
                                    Some(i)
                                };
                            self.show_results = self.punching_view.is_some();
                            if self.show_results {
                                self.show_result_values = true;
                            }
                        }
                    }
                });
            });
    }
    fn punching_number(&self, v: f64) -> String {
        self.display_text.number(v * self.force_result_factor())
    }
    fn summation_scale_index(field: usize) -> Option<usize> {
        match field {
            0..=2 => Some(field),
            4 => Some(3),
            _ => None,
        }
    }
    pub(super) fn punching_range(&self, field: usize) -> [f64; 2] {
        let mut values = vec![];
        if let Some(case) = self.selected_result() {
            for (&id, r) in &case.punching {
                if !r.diagnostic.is_empty()
                    || self
                        .view_contour(id)
                        .is_none_or(|c| !self.contour_visible(c))
                {
                    continue;
                }
                if field == 4 {
                    values.extend(
                        r.edge_forces
                            .iter()
                            .filter(|e| e.active)
                            .map(|e| e.force * self.force_result_factor()),
                    );
                } else if field < 3 {
                    values.push([r.force, r.mx, r.my][field] * self.force_result_factor());
                }
            }
        }
        super::super::result_scale::range(values.into_iter())
    }
    pub(super) fn punching_legend_selection(&self, rect: Rect, pointer: Pos2) -> Option<Vec<u64>> {
        if !self.show_results {
            return None;
        }
        let field = self.punching_view?;
        let scale = &self.punching_scales[Self::summation_scale_index(field)?];
        let range = self.punching_range(field);
        let band = scale.legend_band_at(rect, pointer, range, self.display_text.size)?;
        let matches = |v| scale.value_band(v * self.force_result_factor(), range) == Some(band);
        Some(
            self.selected_result()?
                .punching
                .iter()
                .filter(|(id, r)| {
                    scale.band_enabled(band)
                        && r.diagnostic.is_empty()
                        && self.view_contour(**id).is_some_and(|c| self.selectable(c))
                })
                .filter(|(_, r)| {
                    if field == 4 {
                        r.edge_forces.iter().any(|e| e.active && matches(e.force))
                    } else {
                        matches([r.force, r.mx, r.my][field])
                    }
                })
                .map(|(&id, _)| id)
                .collect(),
        )
    }
    pub(super) fn punching_scale_controls(&mut self, ui: &mut egui::Ui) {
        if let Some((field, index)) = self
            .punching_view
            .and_then(|field| Self::summation_scale_index(field).map(|index| (field, index)))
        {
            let range = self.punching_range(field);
            egui::CollapsingHeader::new("Шкала")
                .default_open(false)
                .show(ui, |ui| {
                    let mut scale = std::mem::take(&mut self.punching_scales[index]);
                    scale.ui_with_header(ui, range, self.display_text, |ui| {
                        self.result_display_controls(ui)
                    });
                    self.punching_scales[index] = scale;
                });
        } else {
            egui::CollapsingHeader::new("Шкала")
                .default_open(false)
                .show(ui, |ui| {
                    self.result_display_controls(ui);
                });
        }
    }
    fn paint_summation_edge_forces(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        id: u64,
        r: &super::super::fem::cut_forces::CutResult,
        range: [f64; 2],
    ) {
        let pointer = painter.ctx().pointer_hover_pos();
        for edge in r.edge_forces.iter().filter(|e| e.active) {
            let value = edge.force * self.force_result_factor();
            let Some(color) = self.punching_scales[3].color(value, range) else {
                continue;
            };
            let [a, b] = edge.ends.map(|p| self.camera.screen(p, rect));
            painter.line_segment([a, b], Stroke::new(3_f32, color));
            let center = a + (b - a) * 0.5;
            let text = format!(
                "F{}={} {}",
                edge.index + 1,
                self.punching_number(edge.force),
                self.load_units.label()
            );
            if self.show_result_values {
                let galley = painter.layout_no_wrap(
                    text.clone(),
                    egui::FontId::proportional(self.display_text.size),
                    self.display_text.ink(),
                );
                let position = center - galley.size() * 0.5;
                painter.rect_filled(
                    Rect::from_min_size(position, galley.size()).expand(3.),
                    3.,
                    self.canvas_background().gamma_multiply(0.95),
                );
                painter.galley(position, galley, self.display_text.ink());
            }
            let hover = pointer.is_some_and(|p| {
                let d = b - a;
                let t = ((p - a).dot(d) / d.length_sq().max(1e-12)).clamp(0., 1.);
                p.distance(a + d * t) < 8.
            });
            if hover {
                egui::show_tooltip_at_pointer(
                    painter.ctx(),
                    egui::LayerId::new(
                        egui::Order::Tooltip,
                        egui::Id::new("summation_edge_tooltip"),
                    ),
                    egui::Id::new(("summation_edge_value", id, edge.index)),
                    |ui| {
                        ui.label(format!(
                            "Область суммирования нагрузок {id} · ребро {}\n{text}",
                            edge.index + 1
                        ));
                    },
                );
            }
        }
    }
    pub(super) fn paint_punching_results(&self, painter: &egui::Painter, rect: Rect) {
        let Some(field) = self.punching_view else {
            return;
        };
        let Some(case) = self.selected_result() else {
            return;
        };
        let range = if Self::summation_scale_index(field).is_some() {
            self.punching_range(field)
        } else {
            [0., 1.]
        };
        let pointer = painter.ctx().pointer_hover_pos();
        for (id, r) in &case.punching {
            let Some(contour) = self
                .contours
                .iter()
                .find(|c| c.id == *id && self.contour_visible(c))
            else {
                continue;
            };
            let center = self.camera.screen(r.center, rect);
            if !r.diagnostic.is_empty() {
                painter.text(
                    center,
                    egui::Align2::CENTER_CENTER,
                    "!",
                    egui::FontId::proportional(16.),
                    Color32::RED,
                );
                if pointer.is_some_and(|p| p.distance(center) < 18.) {
                    egui::show_tooltip_at_pointer(
                        painter.ctx(),
                        egui::LayerId::new(
                            egui::Order::Tooltip,
                            egui::Id::new("summation_diagnostic"),
                        ),
                        egui::Id::new(("summation_error", id)),
                        |ui| {
                            ui.label(format!(
                                "Область суммирования нагрузок {id}\n{}",
                                r.diagnostic
                            ));
                        },
                    );
                }
                continue;
            }
            if field == 4 {
                self.paint_summation_edge_forces(painter, rect, *id, r, range);
                continue;
            }
            let color = if field < 3 {
                let value = [r.force, r.mx, r.my][field] * self.force_result_factor();
                let Some(color) = self.punching_scales[field].color(value, range) else {
                    continue;
                };
                color
            } else {
                self.contour_color(Kind::Support)
            };
            for [a, b] in super::super::fem::cut_forces::control_edges(contour, &self.contours) {
                painter.line_segment(
                    [self.camera.screen(a, rect), self.camera.screen(b, rect)],
                    Stroke::new(3_f32, color),
                );
            }
            let values = [r.force, r.mx, r.my];
            let names = ["F", "Mx", "My"];
            let text = if field < 3 {
                format!(
                    "{}={} {}{}",
                    names[field],
                    self.punching_number(values[field]),
                    self.load_units.label(),
                    if field == 0 { "" } else { "·м" }
                )
            } else {
                format!(
                    "F={} {}\nMx={} {}·м\nMy={} {}·м",
                    self.punching_number(r.force),
                    self.load_units.label(),
                    self.punching_number(r.mx),
                    self.load_units.label(),
                    self.punching_number(r.my),
                    self.load_units.label()
                )
            };
            if self.show_result_values {
                let galley = painter.layout_no_wrap(
                    text,
                    egui::FontId::proportional(self.display_text.size),
                    self.display_text.ink(),
                );
                let pos = center + egui::vec2(7., -galley.size().y * 0.5);
                let label = Rect::from_min_size(pos, galley.size()).expand(3.);
                painter.rect_filled(label, 3., self.canvas_background().gamma_multiply(0.95));
                painter.galley(pos, galley, self.display_text.ink());
            }
            painter.circle_filled(center, 3., color);
            if pointer.is_some_and(|p| p.distance(center) < 16.) {
                egui::show_tooltip_at_pointer(
                    painter.ctx(),
                    egui::LayerId::new(egui::Order::Tooltip, egui::Id::new("punching_tooltip")),
                    egui::Id::new(("punching_value", id)),
                    |ui| {
                        ui.label(format!(
                            "Область суммирования нагрузок {id}\nF={} {}\nMx={} {}·м\nMy={} {}·м",
                            self.punching_number(r.force),
                            self.load_units.label(),
                            self.punching_number(r.mx),
                            self.load_units.label(),
                            self.punching_number(r.my),
                            self.load_units.label()
                        ));
                    },
                );
            }
        }
        if let Some(index) = Self::summation_scale_index(field) {
            self.punching_scales[index].legend(painter, rect, range, self.display_text);
        }
    }
    fn slab_at_support_level(&self, slab: &Contour, z: f64) -> bool {
        !self.building.enabled
            || super::super::foundation::same_elevation(self.source_heights(slab)[0], z)
    }
    fn generate_column_punching_contours(&mut self, id: u64, side: SlabSide) {
        self.sync_stiffness_sources();
        let Some(column) = self
            .contours
            .iter()
            .find(|c| c.id == id && c.kind == Kind::Column)
            .cloned()
        else {
            return;
        };
        let Some(section) = self.support_height.columns.get(&id).cloned() else {
            return;
        };
        let Some(&center) = column.points.first() else {
            return;
        };
        let level = side.level(&self.source_heights(&column));
        let slabs = self.stiffnesses.resolve(&self.contours, Kind::Slab);
        use mini_fem::geo::{BooleanOps, LineString, Polygon};
        let polygon = |points: &[Point]| {
            let mut p = points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
            if p.first() != p.last() {
                if let Some(&first) = p.first() {
                    p.push(first);
                }
            }
            Polygon::new(LineString::from(p), vec![])
        };
        let mut generated = vec![];
        for slab in self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab && geometry::inside(center, &c.points))
        {
            if !self.slab_at_support_level(slab, level) {
                continue;
            }
            let settings = self
                .stiffnesses
                .assignments
                .get(&slab.id)
                .and_then(|id| self.stiffnesses.entries.get(id))
                .map(|e| e.reinforcement)
                .unwrap_or_default();
            let h0 = slabs[&slab.id].thickness_mm
                - settings
                    .asbx
                    .max(settings.asby)
                    .max(settings.astx)
                    .max(settings.asty);
            if h0 <= 0. {
                continue;
            }
            let mut expanded = section.clone();
            expanded.width_mm += h0;
            expanded.material.thickness_mm += h0;
            let outline = crate::plan_trace::columns::section_corners(&column, &expanded);
            let mut plate = polygon(&slab.points);
            for hole in self
                .contours
                .iter()
                .filter(|c| c.active_hole() && c.parent == Some(slab.id))
            {
                plate.interiors_push(polygon(&hole.points).exterior().clone());
            }
            for part in polygon(&outline).intersection(&plate).0 {
                // A contour cannot encode polygon holes: do not silently fill one.
                if !part.interiors().is_empty() {
                    continue;
                }
                let points = part
                    .exterior()
                    .0
                    .iter()
                    .take(part.exterior().0.len().saturating_sub(1))
                    .map(|p| [p.x, p.y])
                    .collect::<Vec<_>>();
                if points.len() < 3 {
                    continue;
                }
                generated.push(Contour {
                    name: String::new(),
                    placement: model::Placement {
                        slab: slab.placement.slab,
                        ..Default::default()
                    },
                    mesh_size_mm: 0.,
                    column_angle: 0.,
                    calculate_effective_length: false,
                    restraint_mask: 4,
                    use_foundation: false,
                    foundation: None,
                    grid: None,
                    support_open: false,
                    load: None,
                    refinement: Some(model::Refinement {
                        inactive_edges: vec![],
                        name: String::new(),
                        size: h0,
                    }),
                    id: 0,
                    parent: Some(slab.id),
                    kind: Kind::Punching,
                    points,
                    curves: vec![],
                });
            }
        }
        if generated.is_empty() {
            self.status =
                "Для колонны не найден область внутри плиты с допустимой рабочей высотой".into();
            return;
        }
        self.history.push(self.contours.clone());
        self.clear_mesh();
        let count = generated.len();
        for mut c in generated {
            c.id = self.next_id;
            self.next_id += 1;
            c.refinement.as_mut().unwrap().name = format!("Область суммирования нагрузок {}", c.id);
            if self.building.enabled {
                self.building
                    .contour_floors
                    .insert(c.id, self.building.owner(c.parent.unwrap()));
            }
            self.contours.push(c);
        }
        self.invalidate_scene_buffers();
        self.status =
            format!("Созданы замкнутые области вокруг колонны: {count}. Перестройте сетку.");
    }

    pub(super) fn generate_punching_contours(&mut self, support: Option<u64>, side: SlabSide) {
        if self.input_locked {
            self.status = "Снимите блокировку исходных данных".into();
            return;
        }
        if let Some(id) = support.filter(|id| {
            self.contours
                .iter()
                .any(|c| c.id == *id && c.kind == Kind::Column)
        }) {
            self.generate_column_punching_contours(id, side);
            return;
        }
        let walls = self.stiffnesses.resolve(&self.contours, Kind::Support);
        let slabs = self.stiffnesses.resolve(&self.contours, Kind::Slab);
        let mut generated = vec![];
        let mut skipped = 0;
        for wall in self.contours.iter().filter(|c| {
            c.kind == Kind::Support && !c.closed() && support.is_none_or(|id| c.id == id)
        }) {
            if wall.points.len() < 2 {
                continue;
            }
            let level = side.level(&self.source_heights(wall));
            for (tip, near) in [
                (wall.points[0], wall.points[1]),
                (
                    *wall.points.last().unwrap(),
                    wall.points[wall.points.len() - 2],
                ),
            ] {
                let connected = self
                    .contours
                    .iter()
                    .filter(|c| c.kind == Kind::Support && c.id != wall.id)
                    .any(|c| {
                        if self.building.enabled
                            && !side.adjoining_wall(&self.source_heights(c), level)
                        {
                            return false;
                        }
                        let count = c.points.len().saturating_sub(usize::from(!c.closed()));
                        c.points
                            .iter()
                            .zip(c.points.iter().cycle().skip(1))
                            .take(count)
                            .any(|(&a, &b)| {
                                let d = sub(b, a);
                                let len = d[0] * d[0] + d[1] * d[1];
                                if len < 1e-12 {
                                    return false;
                                }
                                let q = sub(tip, a);
                                let t = ((q[0] * d[0] + q[1] * d[1]) / len).clamp(0., 1.);
                                (tip[0] - a[0] - t * d[0]).hypot(tip[1] - a[1] - t * d[1]) < 1e-4
                            })
                    });
                if connected {
                    continue;
                }
                let d = sub(tip, near);
                let len = d[0].hypot(d[1]);
                if len < 1e-6 {
                    continue;
                }
                let u = [d[0] / len, d[1] / len];
                let n = [-u[1], u[0]];
                for slab in self.contours.iter().filter(|c| {
                    c.kind == Kind::Slab
                        && geometry::inside(tip, &c.points)
                        && self.slab_at_support_level(c, level)
                }) {
                    let h = slabs[&slab.id].thickness_mm;
                    let settings = self
                        .stiffnesses
                        .assignments
                        .get(&slab.id)
                        .and_then(|id| self.stiffnesses.entries.get(id))
                        .map(|e| e.reinforcement)
                        .unwrap_or_default();
                    let cover = settings
                        .asbx
                        .max(settings.asby)
                        .max(settings.astx)
                        .max(settings.asty);
                    let h0 = h - cover;
                    if h0 <= 0. {
                        skipped += 1;
                        continue;
                    }
                    let r = h0 * 0.5;
                    let width = walls[&wall.id].thickness_mm * 0.5 + r;
                    let at = |x: f64, y: f64| {
                        [tip[0] + u[0] * x + n[0] * y, tip[1] + u[1] * x + n[1] * y]
                    };
                    // SP 430 6.3.8: all three control segments have length a1+h0.
                    let a = 2. * width;
                    let path = [
                        at(r - a, -width),
                        at(r, -width),
                        at(r, width),
                        at(r - a, width),
                    ];
                    let holes: Vec<_> = self
                        .contours
                        .iter()
                        .filter(|c| c.active_hole() && c.parent == Some(slab.id))
                        .collect();
                    use mini_fem::geo::{BooleanOps, LineString, Polygon};
                    let polygon = |points: &[Point]| {
                        let mut xy: Vec<_> = points.iter().map(|p| (p[0], p[1])).collect();
                        if xy.first() != xy.last() {
                            xy.push(xy[0]);
                        }
                        Polygon::new(LineString::from(xy), vec![])
                    };
                    let mut plate = polygon(&slab.points);
                    for hole in &holes {
                        plate.interiors_push(polygon(&hole.points).exterior().clone());
                    }
                    let clipped = polygon(&path).intersection(&plate);
                    let parts: Vec<Vec<Point>> = clipped
                        .0
                        .iter()
                        .map(|p| {
                            p.exterior()
                                .0
                                .iter()
                                .take(p.exterior().0.len().saturating_sub(1))
                                .map(|p| [p.x, p.y])
                                .collect()
                        })
                        .filter(|p: &Vec<Point>| p.len() >= 3)
                        .collect();
                    for points in parts {
                        generated.push(Contour {
                            name: String::new(),
                            placement: model::Placement {
                                slab: slab.placement.slab,
                                ..Default::default()
                            },
                            mesh_size_mm: 0.,
                            column_angle: 0.,
                            calculate_effective_length: false,
                            restraint_mask: 4,
                            use_foundation: false,
                            foundation: None,
                            grid: None,
                            support_open: false,
                            load: None,
                            refinement: Some(model::Refinement {
                                inactive_edges: vec![],

                                name: format!("{TAG} стены {} · h₀={h0:.1} мм", wall.id),
                                size: h0,
                            }),
                            id: 0,
                            parent: Some(slab.id),
                            kind: Kind::Punching,
                            points,
                            curves: vec![],
                        });
                    }
                }
            }
        }
        if support.is_some() && generated.is_empty() {
            self.status = "Для этой опоры не найдены свободные торцы внутри плиты с допустимой рабочей высотой".into();
            return;
        }
        self.history.push(self.contours.clone());
        self.clear_mesh();
        if support.is_none() {
            self.contours.retain(|c| !is_punching(c));
        }
        for c in &mut generated {
            c.kind = Kind::Auxiliary;
            let active = super::super::fem::cut_forces::control_edges(c, &self.contours);
            c.refinement.as_mut().unwrap().inactive_edges = (0..c.points.len())
                .filter(|&i| !active.contains(&[c.points[i], c.points[(i + 1) % c.points.len()]]))
                .collect();
            c.kind = Kind::Punching;
        }
        let count = generated.len();
        for mut c in generated {
            c.id = self.next_id;
            if let Some(r) = &mut c.refinement {
                r.name = format!("Область суммирования нагрузок {}", c.id);
            }
            self.next_id += 1;
            if self.building.enabled {
                if let Some(parent) = c.parent {
                    self.building
                        .contour_floors
                        .insert(c.id, self.building.owner(parent));
                }
            }
            self.contours.push(c);
        }
        self.invalidate_scene_buffers();
        self.status=format!("Области суммирования нагрузок: {count}. Недопустимая рабочая высота: {skipped}. Перестройте сетку и выполните расчёт для сбора F/Mx/My.");
    }
}
