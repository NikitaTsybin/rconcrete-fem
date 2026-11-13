use super::*;
use std::sync::Arc;

#[derive(Clone, Debug)]
pub(super) enum Geometry {
    Line([Point; 2]),
    Point(Point),
}
#[derive(Clone, Debug)]
pub(super) struct Shape {
    pub z: f64,
    pub end_z: Option<f64>,
    pub geometry: Geometry,
    pub value: f64,
}
const EPS: f64 = 1e-6; // display-only coincidence in mm; never alters model/RHS

fn combine_lines(lines: Vec<([Point; 2], f64)>) -> Vec<Shape> {
    let mut groups = BTreeMap::<(i64, i64, i64), Vec<([Point; 2], f64)>>::new();
    for ([a, b], v) in lines {
        let mut u = [b[0] - a[0], b[1] - a[1]];
        let length = u[0].hypot(u[1]);
        if length <= EPS {
            continue;
        }
        u = u.map(|x| x / length);
        if u[0] < 0. || (u[0] == 0. && u[1] < 0.) {
            u = u.map(|x| -x);
        }
        let offset = -u[1] * a[0] + u[0] * a[1];
        groups
            .entry((
                (u[0] * 1e10).round() as i64,
                (u[1] * 1e10).round() as i64,
                (offset / EPS).round() as i64,
            ))
            .or_default()
            .push(([a, b], v));
    }
    let mut out = vec![];
    for lines in groups.into_values() {
        let [origin, b] = lines[0].0;
        let d = [b[0] - origin[0], b[1] - origin[1]];
        let len = d[0].hypot(d[1]);
        let u = d.map(|x| x / len);
        let mut events = vec![];
        for ([a, b], v) in lines {
            let t = |p: Point| (p[0] - origin[0]) * u[0] + (p[1] - origin[1]) * u[1];
            let (a, b) = (t(a), t(b));
            events.push((a.min(b), v));
            events.push((a.max(b), -v));
        }
        events.sort_by(|a, b| a.0.total_cmp(&b.0));
        let tolerance = events.iter().map(|e| e.1.abs()).sum::<f64>().max(1.) * 1e-12;
        let mut i = 0;
        let mut sum = 0.;
        while i < events.len() {
            let start = events[i].0;
            while i < events.len() && (events[i].0 - start).abs() <= EPS {
                sum += events[i].1;
                i += 1;
            }
            if i == events.len() {
                break;
            }
            let end = events[i].0;
            if end - start > EPS && sum.abs() > tolerance {
                let point = |t: f64| [origin[0] + u[0] * t, origin[1] + u[1] * t];
                out.push(Shape {
                    z: 0.,
                    end_z: None,
                    geometry: Geometry::Line([point(start), point(end)]),
                    value: sum,
                });
            }
        }
    }
    out
}

impl App {
    pub(super) fn has_linear_load_results(&self) -> bool {
        self.contours.iter().any(|c| c.kind == Kind::LoadLine)
            || (self
                .load_cases
                .iter()
                .any(|c| c.kind == crate::plan_trace::loads::LoadType::SelfWeight)
                && self.contours.iter().any(|c| {
                    let section = match c.kind {
                        Kind::Beam => self.support_height.beams.get(&c.id),
                        Kind::Column => self.support_height.columns.get(&c.id),
                        _ => None,
                    };
                    section.is_some_and(|s| s.material.weight.value > 0.)
                }))
    }
    pub(super) fn load_mosaic_shapes(&self) -> Arc<Vec<Shape>> {
        if !matches!(self.result_field, 8 | 9) || self.selected_result().is_none() {
            return Arc::new(vec![]);
        }
        let key = self.view_key();
        if let Some((old, shapes)) = self.perf.load_shapes.borrow().as_ref() {
            if *old == key {
                return shapes.clone();
            }
        }
        let kind = if self.result_field == 8 {
            Kind::LoadLine
        } else {
            Kind::LoadPoint
        };
        let floors = if self.building.enabled {
            if self.spatial.enabled {
                self.building.floors.iter().map(|f| f.id).collect()
            } else {
                vec![self.building.active]
            }
        } else {
            vec![0]
        };
        let mut all_shapes = vec![];
        for floor in floors {
            let mut lines = vec![];
            let mut points = BTreeMap::<(i64, i64), (Point, f64)>::new();
            for c in self.contours.iter().filter(|c| {
                !self.object_hidden(c)
                    && c.kind == kind
                    && c.load
                        .as_ref()
                        .is_some_and(|l| !l.moment && l.direction.index() == 2)
                    && (!self.building.enabled || self.building.owner(c.id) == floor)
            }) {
                let Some(value) = self.displayed_load_value(c) else {
                    continue;
                };
                if kind == Kind::LoadPoint {
                    if let Some(&p) = c.points.first() {
                        let e = points
                            .entry(((p[0] / EPS).round() as i64, (p[1] / EPS).round() as i64))
                            .or_insert((p, 0.));
                        e.1 += value;
                    }
                } else {
                    lines.extend(c.points.windows(2).map(|p| ([p[0], p[1]], value)));
                    if c.closed() && c.points.len() > 1 {
                        lines.push(([*c.points.last().unwrap(), c.points[0]], value));
                    }
                }
            }
            let mut floor_shapes = if kind == Kind::LoadLine {
                combine_lines(lines)
            } else {
                points
                    .into_values()
                    .filter(|(_, v)| *v != 0.)
                    .map(|(p, value)| Shape {
                        z: 0.,
                        end_z: None,
                        geometry: Geometry::Point(p),
                        value,
                    })
                    .collect()
            };

            for shape in &mut floor_shapes {
                shape.z = if self.building.enabled {
                    self.building.floor(floor).elevation_mm
                } else {
                    0.
                };
            }
            if kind == Kind::LoadLine {
                let factor: f64 = self
                    .load_cases
                    .iter()
                    .filter(|case| case.kind == crate::plan_trace::loads::LoadType::SelfWeight)
                    .filter_map(|case| self.displayed_load_factor(case.id))
                    .sum();
                if factor != 0. {
                    for c in self.contours.iter().filter(|c| {
                        matches!(c.kind, Kind::Beam | Kind::Column)
                            && !self.object_hidden(c)
                            && (!self.building.enabled || self.building.owner(c.id) == floor)
                    }) {
                        let section = if c.kind == Kind::Column {
                            self.support_height.columns.get(&c.id)
                        } else {
                            self.support_height.beams.get(&c.id)
                        };
                        let Some(section) = section else {
                            continue;
                        };
                        let value = section
                            .material
                            .weight
                            .in_units(self.load_units, self.gravity)
                            * section.area()
                            * factor;
                        if value == 0. {
                            continue;
                        }
                        let heights = self.source_heights(c);
                        if c.kind == Kind::Column {
                            if let Some(&p) = c.points.first() {
                                floor_shapes.push(Shape {
                                    z: heights[0],
                                    end_z: heights.last().copied(),
                                    geometry: Geometry::Line([p, p]),
                                    value,
                                });
                            }
                        } else {
                            let mut own = combine_lines(
                                c.points.windows(2).map(|p| ([p[0], p[1]], value)).collect(),
                            );
                            for shape in &mut own {
                                shape.z = heights[0];
                            }
                            floor_shapes.extend(own);
                        }
                    }
                }
            }
            if kind == Kind::LoadLine {
                let mut horizontal = BTreeMap::<u64, Vec<([Point; 2], f64)>>::new();
                let mut vertical = vec![];
                for shape in floor_shapes {
                    if shape.end_z.is_some() {
                        vertical.push(shape);
                    } else if let Geometry::Line(line) = shape.geometry {
                        horizontal
                            .entry(shape.z.to_bits())
                            .or_default()
                            .push((line, shape.value));
                    }
                }
                floor_shapes = vertical;
                for (z, lines) in horizontal {
                    let mut combined = combine_lines(lines);
                    for shape in &mut combined {
                        shape.z = f64::from_bits(z);
                    }
                    floor_shapes.extend(combined);
                }
            }
            all_shapes.extend(floor_shapes);
        }
        let shapes = Arc::new(all_shapes);
        *self.perf.load_shapes.borrow_mut() = Some((key, shapes.clone()));
        shapes
    }
    pub(super) fn paint_load_mosaics(&self, painter: &egui::Painter, rect: Rect) {
        if !self.show_results || self.punching_view.is_some() || !matches!(self.result_field, 8 | 9)
        {
            return;
        }
        let Some(auto) = self.result_range_cached() else {
            return;
        };
        let scale = &self.result_scales[self.result_field];
        for shape in self.load_mosaic_shapes().iter() {
            let Some(color) = scale.color(shape.value, auto) else {
                continue;
            };
            match shape.geometry {
                Geometry::Line([a, b]) => draw_path(
                    painter,
                    &[a, b],
                    false,
                    self.camera,
                    rect,
                    Stroke::new(3_f32, color),
                ),
                Geometry::Point(p) => {
                    let p = self.camera.screen(p, rect);
                    if !rect.expand(8.).contains(p) {
                        continue;
                    }
                    painter.circle_filled(p, 7., color);
                    let ink = if color.r() as u16 + color.g() as u16 + color.b() as u16 > 380 {
                        Color32::BLACK
                    } else {
                        Color32::WHITE
                    };
                    for sign in [-1., 1.] {
                        painter.line_segment(
                            [
                                p + egui::vec2(-4., sign * 4.),
                                p + egui::vec2(4., -sign * 4.),
                            ],
                            Stroke::new(1.2_f32, ink),
                        );
                    }
                }
            }
        }
    }
    pub(super) fn load_mosaic_hover(&self, p: Pos2, rect: Rect) -> Option<String> {
        let auto = self.result_range_cached()?;
        let scale = &self.result_scales[self.result_field];
        let shapes = self.load_mosaic_shapes();
        let hit = shapes
            .iter()
            .filter(|s| scale.color(s.value, auto).is_some())
            .filter_map(|s| {
                let distance = match s.geometry {
                    Geometry::Point(q) => self.camera.screen(q, rect).distance(p),
                    Geometry::Line([a, b]) => {
                        let a = self.camera.screen(a, rect);
                        let b = self.camera.screen(b, rect);
                        let d = b - a;
                        let t = ((p - a).dot(d) / d.length_sq().max(1e-20)).clamp(0., 1.);
                        (a + d * t).distance(p)
                    }
                };
                (distance <= 9.).then_some((distance, s.value))
            })
            .min_by(|a, b| a.0.total_cmp(&b.0))?;
        let world = self.camera.world(p, rect);
        Some(format!(
            "{}\nX {:.1} · Y {:.1} мм\nЗначение: {}",
            self.result_caption(),
            world[0],
            world[1],
            self.display_text.number(hit.1)
        ))
    }
    pub(super) fn paint_load_mosaic_annotations(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pointer: Option<Pos2>,
    ) {
        let Some(auto) = self.result_range_cached() else {
            return;
        };
        let scale = &self.result_scales[self.result_field];
        let mut occupied = vec![];
        if self.show_result_values {
            for s in self
                .load_mosaic_shapes()
                .iter()
                .filter(|s| scale.color(s.value, auto).is_some())
            {
                let p = match s.geometry {
                    Geometry::Point(p) => self.camera.screen(p, rect) + egui::vec2(12., -12.),
                    Geometry::Line([a, b]) => self
                        .camera
                        .screen([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5], rect),
                };
                if !rect.contains(p) || occupied.iter().any(|r: &Rect| r.contains(p)) {
                    continue;
                }
                let r = super::fem_ui::result_label(
                    painter,
                    p,
                    self.display_text.number(s.value),
                    rect,
                    self.display_text,
                );
                occupied.push(r.expand(3.));
            }
        }
        if self.show_results {
            scale.legend(painter, rect, auto, self.display_text);
        }
        if let Some(p) = pointer {
            if let Some(text) = self.load_mosaic_hover(p, rect) {
                super::fem_ui::result_label(
                    painter,
                    p + egui::vec2(16., 18.),
                    text,
                    rect,
                    self.display_text,
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn point_mosaic_sums_coincident_sources_with_case_variant_units_and_cache() {
        use crate::plan_trace::{
            fem::{CaseResult, Results},
            load_combinations::Variant,
            loads::{ForceUnits, LoadCase, Magnitude},
        };
        let mut a = App::default();
        a.display_text.decimals = 4;
        a.load_units = ForceUnits::KiloNewton;
        a.load_cases.push(LoadCase::new(2));
        a.load_cases[0].gamma_f = 2.;
        a.load_cases[1].gamma_f = 4.;
        for (case, value) in [(1, 2.), (2, 3.)] {
            a.active_load = case;
            a.new_load = Magnitude {
                value,
                units: ForceUnits::KiloNewton,
            };
            a.begin(Kind::LoadPoint);
            a.sketch_click([0., 0.], false);
        }
        let result = |id| CaseResult {
            punching: Default::default(),
            shell: None,
            id,
            w_mm: vec![],
            rotations_rad: vec![[0.; 3]; 0],
            moments: vec![],
            shears: vec![],
            reaction_kn: vec![],
            loads_kn_m2: vec![0., id as f64 + 1., id as f64 + 1.],
            foundation_pressure_kn_m2: vec![],
            applied_kn: 0.,
            balance_kn: 0.,
            balance_m_knm: [0.; 2],
            relative_residual: 0.,
        };
        a.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![result(1), result(2)],
            free_dofs: 0,
        });
        a.active_load = 1;
        a.result_field = 9;
        let first = a.load_mosaic_shapes();
        assert_eq!(first.len(), 1);
        assert_eq!(first[0].value, 2.);
        a.sync_load_scale();
        assert_eq!(
            serde_json::to_value(a.result_scales[9].state()).unwrap()["discrete"],
            serde_json::json!([2.])
        );
        let signature = a.result_scales[9].signature();
        a.sync_load_scale();
        assert_eq!(a.result_scales[9].signature(), signature);
        assert!(Arc::ptr_eq(&first, &a.load_mosaic_shapes()));
        a.result_by_combinations = true;
        a.combinations[0].coefficients.insert(1, 2.);
        a.combinations[0].coefficients.insert(2, -1.);
        a.sync_combination_result();
        assert_eq!(a.load_mosaic_shapes()[0].value, 1.);
        a.combination_variant = Variant::Normative;
        a.sync_combination_result();
        assert_eq!(a.load_mosaic_shapes()[0].value, 1.25);
        a.sync_load_scale();
        assert_eq!(
            serde_json::to_value(a.result_scales[9].state()).unwrap()["discrete"],
            serde_json::json!([1.25])
        );
        a.load_units = ForceUnits::TonForce;
        a.sync_result_units();
        a.show_loads = false;
        assert_eq!(a.load_mosaic_shapes()[0].value, 0.125);
        a.sync_load_scale();
        assert_eq!(
            serde_json::to_value(a.result_scales[9].state()).unwrap()["discrete"],
            serde_json::json!([0.125])
        );
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        assert!(a
            .load_mosaic_hover(a.camera.screen([0., 0.], rect), rect)
            .unwrap()
            .ends_with("0.1250"));
        a.result_field = 8;
        assert!(a.load_mosaic_shapes().is_empty());
        assert_eq!(a.result_extrema_cached(), Some([0., 0.]));
        a.sync_load_scale();
        assert_eq!(
            serde_json::to_value(a.result_scales[8].state()).unwrap()["discrete"],
            serde_json::json!([])
        );
        a.result_field = 7;
        a.sync_load_scale();
        assert_eq!(
            serde_json::to_value(a.result_scales[7].state()).unwrap()["discrete"],
            serde_json::json!([0., 0.125])
        );
        a.result_by_combinations = false;
        a.load_units = ForceUnits::KiloNewton;
        a.combination_variant = Variant::Design;
        // Self-weight alone must expose ql and produce a physical beam intensity.
        let mut beam = a.contours[0].clone();
        beam.id = 999;
        beam.kind = Kind::Beam;
        beam.load = None;
        beam.points = vec![[0., 0.], [1000., 0.]];
        a.contours.push(beam);
        a.load_cases[0].kind = crate::plan_trace::loads::LoadType::SelfWeight;
        a.support_height.beams.insert(
            999,
            crate::plan_trace::fem::beam::Section {
                material: crate::plan_trace::fem::element::Material {
                    thickness_mm: 550.,
                    weight: Magnitude {
                        value: 2.5,
                        units: ForceUnits::TonForce,
                    },
                    ..Default::default()
                },
                width_mm: 200.,
            },
        );
        assert!(a.has_linear_load_results());
        a.result_field = 8;
        let lines = a.load_mosaic_shapes();
        assert_eq!(lines.len(), 1);
        assert!((lines[0].value - 0.275 * a.gravity).abs() < 1e-10);
    }
    #[test]
    fn collinear_partial_overlap_sums_only_over_shared_length_and_preserves_resultant() {
        let shapes = combine_lines(vec![
            ([[0., 0.], [10., 0.]], 2.),
            ([[15., 0.], [5., 0.]], 3.),
        ]);
        assert_eq!(shapes.len(), 3);
        let mut values = shapes.iter().map(|s| s.value).collect::<Vec<_>>();
        values.sort_by(f64::total_cmp);
        assert_eq!(values, vec![2., 3., 5.]);
        let force: f64 = shapes
            .iter()
            .map(|s| {
                if let Geometry::Line([a, b]) = s.geometry {
                    (b[0] - a[0]).hypot(b[1] - a[1]) * s.value
                } else {
                    0.
                }
            })
            .sum();
        assert!((force - 50.).abs() < 1e-10);
        assert!(combine_lines(vec![
            ([[0., 0.], [10., 0.]], 2.),
            ([[10., 0.], [0., 0.]], -2.)
        ])
        .is_empty());
    }
}
