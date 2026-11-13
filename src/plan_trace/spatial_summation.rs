//! Summation contours and their result fields at the owning storey elevation.
use super::*;
impl App {
    pub(super) fn spatial_summation(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        scene: &crate::plan_trace::fem::shell::Scene,
    ) -> Option<(usize, [f64; 2])> {
        let field = self.punching_view.filter(|_| self.show_results);
        let case = field.and_then(|_| self.selected_result());
        let range = field.filter(|f| *f != 3).map(|f| self.punching_range(f));
        let scale_index = field.and_then(|f| match f {
            0..=2 => Some(f),
            4 => Some(3),
            _ => None,
        });
        let mut builder = spatial_depth::Builder::new(rect, projected);
        let mut labels = Vec::<((Pos2, f64), String, bool)>::new();
        let mut hover = None::<(f64, String)>;
        let pointer = painter
            .ctx()
            .pointer_hover_pos()
            .filter(|p| rect.contains(*p));
        let mut occlusion = None;
        let number = |v: f64| self.display_text.number(v * self.force_result_factor());
        for c in self
            .contours
            .iter()
            .filter(|c| punching::is_punching(c) && self.contour_visible(c))
        {
            let z = self.source_heights(c)[0];
            let plane = [[0., 0., z], [1000., 0., z], [0., 1000., z]].map(project);
            let working = crate::plan_trace::fem::cut_forces::control_edges(c, &self.contours);
            let result = case.and_then(|r| r.punching.get(&c.id));
            let valid = result.filter(|r| r.diagnostic.is_empty());
            for (index, (a, b)) in c
                .points
                .iter()
                .copied()
                .zip(c.points.iter().copied().cycle().skip(1))
                .take(c.points.len())
                .enumerate()
            {
                let ends = [a, b].map(|p| project([p[0], p[1], z]));
                if !working.contains(&[a, b]) {
                    let length = ends[0].0.distance(ends[1].0);
                    let mut x = 0.;
                    while x < length {
                        let at = |d: f32| {
                            let t = (d / length) as f64;
                            project([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, z])
                        };
                        builder.line_on_face(
                            [at(x), at((x + 6.).min(length))],
                            1.5,
                            self.contour_color(c.kind),
                            plane,
                        );
                        x += 10.;
                    }
                    continue;
                }
                let mut ink = if self.is_selected(c.id) {
                    Color32::YELLOW
                } else {
                    self.contour_color(c.kind)
                };
                let mut edge_text = None;
                if let (Some(field), Some(r)) = (field, valid) {
                    if field == 4 {
                        if let Some(edge) =
                            r.edge_forces.iter().find(|e| e.index == index && e.active)
                        {
                            let Some(color) = self.punching_scales[3]
                                .color(edge.force * self.force_result_factor(), range.unwrap())
                            else {
                                continue;
                            };
                            ink = color;
                            edge_text = Some(format!(
                                "F{}={} {}",
                                index + 1,
                                number(edge.force),
                                self.load_units.label()
                            ));
                        }
                    } else if field < 3 {
                        let value = [r.force, r.mx, r.my][field] * self.force_result_factor();
                        let Some(color) = self.punching_scales[field].color(value, range.unwrap())
                        else {
                            continue;
                        };
                        ink = color;
                    }
                }
                builder.line_on_face(ends, if field.is_some() { 3. } else { 2. }, ink, plane);
                let mid = project([(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5, z]);
                if let Some(text) = edge_text {
                    if self.show_result_values {
                        labels.push((mid, text.clone(), false));
                    }
                    if pointer.is_some_and(|p| {
                        let d = ends[1].0 - ends[0].0;
                        let t = ((p - ends[0].0).dot(d) / d.length_sq().max(1e-12)).clamp(0., 1.);
                        p.distance(ends[0].0 + d * t) <= 8.
                    }) && occlusion
                        .get_or_insert_with(|| {
                            spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                        })
                        .visible(mid.0, mid.1)
                    {
                        if hover.as_ref().is_none_or(|h| mid.1 > h.0) {
                            hover = Some((
                                mid.1,
                                format!(
                                    "Область суммирования нагрузок {} · ребро {}\n{}",
                                    c.id,
                                    index + 1,
                                    text
                                ),
                            ));
                        }
                    }
                }
            }
            if let (Some(field), Some(r)) = (field, result) {
                if field < 3
                    && r.diagnostic.is_empty()
                    && self.punching_scales[field]
                        .color(
                            [r.force, r.mx, r.my][field] * self.force_result_factor(),
                            range.unwrap(),
                        )
                        .is_none()
                {
                    continue;
                }
                let at = project([r.center[0], r.center[1], z]);
                let all = format!(
                    "F={} {}\nMx={} {}·м\nMy={} {}·м",
                    number(r.force),
                    self.load_units.label(),
                    number(r.mx),
                    self.load_units.label(),
                    number(r.my),
                    self.load_units.label()
                );
                if !r.diagnostic.is_empty() {
                    labels.push((at, "!".into(), true));
                } else if field != 4 && self.show_result_values {
                    let text = if field < 3 {
                        format!(
                            "{}={} {}{}",
                            ["F", "Mx", "My"][field],
                            number([r.force, r.mx, r.my][field]),
                            self.load_units.label(),
                            if field == 0 { "" } else { "·м" }
                        )
                    } else {
                        all.clone()
                    };
                    labels.push((at, text, false));
                }
                if pointer.is_some_and(|p| p.distance(at.0) <= 16.)
                    && occlusion
                        .get_or_insert_with(|| {
                            spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                        })
                        .visible(at.0, at.1)
                {
                    if hover.as_ref().is_none_or(|h| at.1 > h.0) {
                        hover = Some((
                            at.1,
                            format!(
                                "Область суммирования нагрузок {}\n{}",
                                c.id,
                                if r.diagnostic.is_empty() {
                                    all
                                } else {
                                    r.diagnostic.clone()
                                }
                            ),
                        ));
                    }
                }
            }
        }
        spatial_depth::paint_layer(
            painter,
            rect,
            std::sync::Arc::new(builder.vertices),
            spatial_depth::Layer::Summation,
        );
        for (at, text, error) in labels {
            if rect.contains(at.0)
                && occlusion
                    .get_or_insert_with(|| {
                        spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                    })
                    .visible(at.0, at.1)
            {
                if error {
                    painter.text(
                        at.0,
                        egui::Align2::CENTER_CENTER,
                        text,
                        egui::FontId::proportional(16.),
                        Color32::RED,
                    );
                } else {
                    self.load_caption(painter, at.0 + egui::vec2(8., 0.), text);
                }
            }
        }
        if let (Some(pointer), Some((_, text))) = (pointer, hover) {
            fem_ui::result_label(
                painter,
                pointer + egui::vec2(16., 18.),
                text,
                rect,
                self.display_text,
            );
        }
        scale_index.zip(range)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn source_and_force_fields_draw_at_storey_height_and_visibility_flag_is_respected() {
        let mut app = App::default();
        app.building.enabled = true;
        app.building.floors[0].elevation_mm = 3000.;
        app.contours = vec![Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 1,
            kind: Kind::Punching,
            parent: None,
            points: vec![[100., 100.], [200., 100.], [200., 200.], [100., 200.]],
            curves: vec![],
            grid: None,
            support_open: false,
            load: None,
            refinement: Some(model::Refinement {
                name: "Контур 1".into(),
                size: 100.,
                inactive_edges: vec![3],
            }),
        }];
        let mut cut = crate::plan_trace::fem::cut_forces::CutResult {
            center: [150., 150.],
            force: 20.,
            mx: 2.,
            my: -3.,
            ..Default::default()
        };
        cut.edge_forces = app.contours[0]
            .points
            .iter()
            .copied()
            .zip(app.contours[0].points.iter().copied().cycle().skip(1))
            .take(4)
            .enumerate()
            .map(
                |(index, (a, b))| crate::plan_trace::fem::cut_forces::EdgeForce {
                    index,
                    ends: [a, b],
                    force: 5.,
                    mx_origin: 0.,
                    my_origin: 0.,
                    active: index != 3,
                    diagnostic: String::new(),
                },
            )
            .collect();
        app.analysis = Some(crate::plan_trace::fem::Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            free_dofs: 0,
            cases: vec![crate::plan_trace::fem::CaseResult {
                punching: BTreeMap::from([(1, cut)]),
                id: 1,
                shell: None,
                w_mm: vec![],
                rotations_rad: vec![],
                moments: vec![],
                shears: vec![],
                reaction_kn: vec![],
                loads_kn_m2: vec![],
                foundation_pressure_kn_m2: vec![],
                applied_kn: 0.,
                balance_kn: 0.,
                balance_m_knm: [0.; 2],
                relative_residual: 0.,
            }],
        });
        let scene = crate::plan_trace::fem::shell::Scene {
            xyz: vec![],
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let ctx = egui::Context::default();
        let project = |p: [f64; 3]| {
            assert_eq!(p[2], 3000.);
            (Pos2::new(p[0] as f32, p[1] as f32), p[2])
        };
        let draw = |app: &App| {
            ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(400., 300.))),
                    ..Default::default()
                },
                |ctx| {
                    let painter = ctx.layer_painter(egui::LayerId::new(
                        egui::Order::Middle,
                        egui::Id::new("sum-test"),
                    ));
                    app.spatial_summation(
                        &painter,
                        Rect::from_min_size(Pos2::ZERO, egui::vec2(400., 300.)),
                        &project,
                        &[(Pos2::ZERO, 2900.), (Pos2::new(400., 300.), 3100.)],
                        &scene,
                    );
                },
            )
        };
        assert!(draw(&app)
            .shapes
            .iter()
            .any(|s| matches!(s.shape, egui::Shape::Callback(_))));
        app.show_results = true;
        app.show_result_values = true;
        fn texts(shape: &egui::Shape, out: &mut String) {
            match shape {
                egui::Shape::Text(t) => out.push_str(&t.galley.job.text),
                egui::Shape::Vec(v) => {
                    for s in v {
                        texts(s, out)
                    }
                }
                _ => {}
            }
        }
        for field in 0..5 {
            app.punching_view = Some(field);
            let mut text = String::new();
            for s in draw(&app).shapes {
                texts(&s.shape, &mut text);
            }
            assert!(
                text.contains(if field == 4 {
                    "F1="
                } else {
                    ["F=", "Mx=", "My=", "F="][field]
                }),
                "field {field}: {text}"
            );
        }
        app.punching_view = None;
        app.drawing_visibility.punching = false;
        assert!(!draw(&app)
            .shapes
            .iter()
            .any(|s| matches!(s.shape, egui::Shape::Callback(_))));
    }
}
