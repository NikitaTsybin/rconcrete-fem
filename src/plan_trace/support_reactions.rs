use super::*;
use std::{collections::BTreeSet, sync::Arc};
#[derive(Clone)]
pub(super) struct Total {
    pub id: u64,
    pub point: Point,
    pub value: f64,
}
impl App {
    pub(super) fn support_reaction_totals(&self) -> Arc<Vec<Total>> {
        let Some(mesh) = &self.fe_mesh else {
            return Arc::new(vec![]);
        };
        let Some(case) = self.selected_result() else {
            return Arc::new(vec![]);
        };
        let key = self.view_key();
        if let Some((old, totals)) = self.perf.reactions.borrow().as_ref() {
            if *old == key {
                return totals.clone();
            }
        }
        let groups = mesh
            .support_boundary
            .iter()
            .filter(|(_, nodes)| {
                self.spatial.enabled || nodes.iter().any(|&n| self.mesh_node_on_floor(n))
            })
            .map(|(&id, nodes)| {
                (
                    id,
                    nodes
                        .iter()
                        .copied()
                        .filter(|&n| {
                            n < mesh.nodes.len()
                                && n < case.reaction_kn.len()
                                && (self.spatial.enabled || self.mesh_node_on_floor(n))
                        })
                        .collect::<BTreeSet<_>>(),
                )
            })
            .collect::<Vec<_>>();
        let mut membership = vec![0usize; mesh.nodes.len()];
        for (_, nodes) in &groups {
            for &n in nodes {
                membership[n] += 1;
            }
        }
        let totals = groups
            .into_iter()
            .filter(|(_, nodes)| !nodes.is_empty())
            .map(|(id, nodes)| {
                let value = case
                    .shell
                    .as_ref()
                    .filter(|_| {
                        !self.building.enabled || self.support_height.columns.contains_key(&id)
                    })
                    .and_then(|s| s.support_reactions_kn.get(&id))
                    .map(|r| -r[2] * self.force_result_factor())
                    .unwrap_or_else(|| {
                        nodes
                            .iter()
                            .map(|&n| case.reaction_kn[n] / membership[n] as f64)
                            .sum::<f64>()
                            * self.force_result_factor()
                    });
                let contour = self
                    .contours
                    .iter()
                    .find(|c| c.id == id && matches!(c.kind, Kind::Support | Kind::Column));
                let point = contour.and_then(line_center).unwrap_or_else(|| {
                    [
                        nodes
                            .iter()
                            .map(|&n| mesh.nodes[n].xy[0] / nodes.len() as f64)
                            .sum(),
                        nodes
                            .iter()
                            .map(|&n| mesh.nodes[n].xy[1] / nodes.len() as f64)
                            .sum(),
                    ]
                });
                Total { id, point, value }
            })
            .collect::<Vec<_>>();
        let totals = Arc::new(totals);
        *self.perf.reactions.borrow_mut() = Some((key, totals.clone()));
        totals
    }
    fn reaction_screen_anchor(&self, total: &Total, rect: Rect) -> Option<Pos2> {
        let p = self.camera.screen(total.point, rect);
        if rect.contains(p) {
            return Some(p);
        }
        let c = self
            .contours
            .iter()
            .find(|c| c.id == total.id && matches!(c.kind, Kind::Support | Kind::Column))?;
        edges(c)
            .filter_map(|[a, b]| {
                clip_line(
                    self.camera.screen64(a, rect),
                    self.camera.screen64(b, rect),
                    rect,
                )
            })
            .max_by(|a, b| a[0].distance(a[1]).total_cmp(&b[0].distance(b[1])))
            .map(|[a, b]| a + (b - a) * 0.5)
    }
    pub(super) fn paint_support_reactions(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        pointer: Option<Pos2>,
    ) {
        let totals = self.support_reaction_totals();
        let Some(auto) = self.result_range_cached() else {
            return;
        };
        let scale = &self.result_scales[6];
        let Some(mesh) = &self.fe_mesh else {
            return;
        };
        let mut shared_label_rows = BTreeMap::<usize, usize>::new();
        for total in totals.iter() {
            let Some(color) = scale.color(total.value, auto) else {
                continue;
            };
            let Some(nodes) = mesh.support_boundary.get(&total.id) else {
                continue;
            };
            let nodes = nodes.iter().copied().collect::<BTreeSet<_>>();
            for node in nodes {
                let Some(node_data) = mesh.nodes.get(node) else {
                    continue;
                };
                let at = self.camera.screen(node_data.xy, rect);
                if !rect.contains(at) {
                    continue;
                }
                if self.show_results {
                    painter.circle_filled(at, self.point_size * 0.5, color);
                }
                if self.show_result_values {
                    let row = shared_label_rows.entry(node).or_default();
                    fem_ui::result_label(
                        painter,
                        at + egui::vec2(8., -12. + *row as f32 * (self.display_text.size + 8.)),
                        self.display_text.number(total.value),
                        rect,
                        self.display_text,
                    );
                    *row += 1;
                }
            }
        }
        if self.show_results {
            scale.legend(painter, rect, auto, self.display_text);
        }
        if let Some(p) = pointer {
            if let Some(text) = self.support_reaction_hover(p, rect) {
                fem_ui::result_label(
                    painter,
                    p + egui::vec2(16., 18.),
                    text,
                    rect,
                    self.display_text,
                );
            }
        }
    }
    pub(super) fn support_reaction_hover(&self, p: Pos2, rect: Rect) -> Option<String> {
        let totals = self.support_reaction_totals();
        let world = self.camera.world(p, rect);
        let total = totals
            .iter()
            .filter_map(|total| {
                let mut distance = self
                    .reaction_screen_anchor(total, rect)
                    .map_or(f32::INFINITY, |q| q.distance(p));
                if let Some(c) = self
                    .contours
                    .iter()
                    .find(|c| c.id == total.id && matches!(c.kind, Kind::Support | Kind::Column))
                {
                    for [a, b] in edges(c) {
                        let d = [b[0] - a[0], b[1] - a[1]];
                        let len = d[0] * d[0] + d[1] * d[1];
                        if len <= 0. {
                            continue;
                        }
                        let t = (((world[0] - a[0]) * d[0] + (world[1] - a[1]) * d[1]) / len)
                            .clamp(0., 1.);
                        let value = (world[0] - a[0] - t * d[0]).hypot(world[1] - a[1] - t * d[1])
                            * self.camera.zoom;
                        distance = distance.min(value as f32);
                    }
                }
                (distance <= 9.).then_some((total, distance))
            })
            .min_by(|a, b| a.1.total_cmp(&b.1))?
            .0;
        Some(format!(
            "{}\nОпора {}\nСумма Rz: {} {}\nОбщие узлы разделены поровну между опорами",
            self.result_caption(),
            total.id,
            self.display_text.number(total.value),
            self.load_units.label()
        ))
    }
}
fn edges(c: &Contour) -> impl Iterator<Item = [Point; 2]> + '_ {
    c.points
        .iter()
        .copied()
        .zip(c.points.iter().copied().cycle().skip(1))
        .take(if c.closed() {
            c.points.len()
        } else {
            c.points.len().saturating_sub(1)
        })
        .map(|(a, b)| [a, b])
}
fn line_center(c: &Contour) -> Option<Point> {
    let mut length = 0.;
    let mut sum = [0.; 2];
    for [a, b] in edges(c) {
        let weight = (b[0] - a[0]).hypot(b[1] - a[1]);
        length += weight;
        for axis in 0..2 {
            sum[axis] += (a[axis] + b[axis]) * 0.5 * weight;
        }
    }
    (length > 0.).then(|| [sum[0] / length, sum[1] / length])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        fem::{CaseResult, Results},
        load_combinations::Variant,
        loads::{ForceUnits, LoadCase},
        meshing::{Mesh, Node},
    };
    fn sample() -> App {
        let mut a = App::default();
        a.result_field = 6;
        a.sum_reactions = true;
        a.fit = false;
        a.camera.zoom = 0.2;
        a.fe_mesh = Some(Mesh {
            building: None,
            quads: vec![],
            constrained_nodes: Default::default(),
            constrained_edges: Default::default(),
            nodes: vec![
                Node {
                    id: 1,
                    xy: [0., 0.],
                },
                Node {
                    id: 2,
                    xy: [1000., 0.],
                },
                Node {
                    id: 3,
                    xy: [1000., 1000.],
                },
                Node {
                    id: 4,
                    xy: [500., 500.],
                },
            ],
            elements: vec![],
            edges: vec![],
            support_boundary: BTreeMap::from([(10, vec![0, 1, 1]), (20, vec![1, 2]), (30, vec![])]),
        });
        for (id, points) in [
            (10, vec![[0., 0.], [1000., 0.]]),
            (20, vec![[1000., 0.], [1000., 1000.]]),
        ] {
            a.contours.push(Contour {
                name: String::new(),
                placement: Default::default(),
                mesh_size_mm: 0.,
                column_angle: 0.,
                calculate_effective_length: false,
                restraint_mask: 4,
                use_foundation: false,
                foundation: None,
                id,
                kind: Kind::Support,
                parent: None,
                points,
                curves: vec![],
                grid: None,
                support_open: true,
                refinement: None,
                load: None,
            });
        }
        a.analysis = Some(Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            free_dofs: 1,
            cases: vec![CaseResult {
                punching: Default::default(),
                shell: None,
                id: 1,
                w_mm: vec![0.; 4],
                rotations_rad: vec![[0.; 3]; 0],
                moments: vec![],
                shears: vec![],
                reaction_kn: vec![-10., -6., -20., 123.],
                loads_kn_m2: vec![],
                foundation_pressure_kn_m2: vec![],
                applied_kn: 36.,
                balance_kn: 0.,
                balance_m_knm: [0.; 2],
                relative_residual: 0.,
            }],
        });
        a
    }
    #[test]
    fn shared_nodes_are_divided_once_preserving_balance_units_and_range_cache() {
        let mut a = sample();
        let totals = a.support_reaction_totals();
        assert_eq!(totals.len(), 2);
        assert!((totals[0].value + 1.3).abs() < 1e-12);
        assert!((totals[1].value + 2.3).abs() < 1e-12);
        assert!((totals.iter().map(|r| r.value).sum::<f64>() + 3.6).abs() < 1e-12);
        assert!(Arc::ptr_eq(&totals, &a.support_reaction_totals()));
        let extrema = a.result_extrema_cached().unwrap();
        assert!((extrema[0] + 2.3).abs() < 1e-12);
        a.sum_reactions = false;
        let extrema = a.result_extrema_cached().unwrap();
        assert!((extrema[0] + 2.).abs() < 1e-12);
        a.sum_reactions = true;
        a.load_units = ForceUnits::KiloNewton;
        a.sync_result_units();
        let totals = a.support_reaction_totals();
        assert_eq!(
            totals.iter().map(|r| r.value).collect::<Vec<_>>(),
            vec![-13., -23.]
        );
        assert!(a.result_caption().contains("Суммарные реакции"));
        assert_eq!(
            a.analysis.as_ref().unwrap().cases[0].reaction_kn,
            vec![-10., -6., -20., 123.]
        );
    }
    #[test]
    fn totals_follow_variants_and_combinations_without_changing_base_results() {
        let mut a = sample();
        a.load_units = ForceUnits::KiloNewton;
        a.load_cases[0].gamma_f = 2.;
        a.load_cases[0].gamma_l = 0.4;
        a.combination_variant = Variant::NormativeLong;
        a.sync_combination_result();
        let totals = a.support_reaction_totals();
        assert!((totals[0].value + 2.6).abs() < 1e-12);
        assert!((totals[1].value + 4.6).abs() < 1e-12);
        let mut second = a.analysis.as_ref().unwrap().cases[0].clone();
        second.id = 2;
        for v in &mut second.reaction_kn {
            *v *= -2.;
        }
        a.analysis.as_mut().unwrap().cases.push(second);
        a.load_cases.push(LoadCase::new(2));
        a.result_by_combinations = true;
        a.combination_variant = Variant::Design;
        a.combinations[0].coefficients.insert(1, 1.);
        a.combinations[0].coefficients.insert(2, 0.25);
        a.sync_combination_result();
        let totals = a.support_reaction_totals();
        assert!((totals.iter().map(|r| r.value).sum::<f64>() + 18.).abs() < 1e-12);
        assert_eq!(a.analysis.as_ref().unwrap().cases[0].reaction_kn[0], -10.);
    }
    #[test]
    fn summed_reactions_draw_values_without_nodal_value_flag_and_hover_identifies_support() {
        let mut a = sample();
        a.show_result_values = true;
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.));
        let p = a.camera.screen([500., 0.], rect);
        let text = a.support_reaction_hover(p, rect).unwrap();
        assert!(text.contains("Опора 10") && text.contains("-1.30"));
        let ctx = egui::Context::default();
        let out = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    a.paint_support_reactions(ui.painter(), rect, None)
                });
            },
        );
        assert!(out
            .shapes
            .iter()
            .any(|s| matches!(&s.shape,egui::Shape::Text(t) if t.galley.job.text == "-1.30")));
        assert!(out
            .shapes
            .iter()
            .any(|s| matches!(&s.shape,egui::Shape::Text(t) if t.galley.job.text == "-2.30")));
        let empty = App::default();
        assert!(empty.support_reaction_totals().is_empty());
    }
}
