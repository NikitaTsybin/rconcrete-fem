use super::*;
#[derive(Clone, Copy, PartialEq)]
pub(super) enum Mode {
    Element,
    Whole,
}
#[derive(Clone)]
struct Candidate {
    points: Vec<Point>,
    curves: Vec<Curve>,
    closed: bool,
}
fn distance(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len = d[0] * d[0] + d[1] * d[1];
    let t = if len > 0. {
        (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len).clamp(0., 1.)
    } else {
        0.
    };
    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1])
}
pub(super) fn curves(points: &[Point], closed: bool) -> Vec<Curve> {
    points
        .iter()
        .copied()
        .zip(points.iter().copied().cycle().skip(1))
        .take(if closed {
            points.len()
        } else {
            points.len().saturating_sub(1)
        })
        .map(|(a, b)| Curve::Line(a, b))
        .collect()
}
pub(super) fn reverse(c: &mut Curve) {
    match c {
        Curve::Line(a, b) => std::mem::swap(a, b),
        Curve::Arc(p) => p.swap(0, 2),
        Curve::Spline { controls, .. } => controls.reverse(),
        Curve::Ellipse(_) => {}
    }
}
impl App {
    pub(super) fn reuse_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            for (mode, label) in [
                (Mode::Element, "По ребру"),
                (Mode::Whole, "По всей границе"),
            ] {
                if ui
                    .selectable_label(self.reuse_mode == Some(mode), label)
                    .clicked()
                {
                    let previous = self.reuse_mode;
                    self.choose_tool(Tool::Line);
                    if self.tool == Tool::Line {
                        self.reuse_mode = if previous == Some(mode) {
                            None
                        } else {
                            Some(mode)
                        };
                    }
                }
            }
        });
    }
    fn reuse_hit(&self, p: Point, mode: Mode) -> Option<Candidate> {
        let mut best = 10. / self.camera.zoom;
        let mut result = None;
        for c in self
            .contours
            .iter()
            .filter(|c| self.contour_visible(c) && self.on_active_floor(c) && c.points.len() > 1)
        {
            let mut source = if c.curves.is_empty() {
                curves(&c.points, c.closed())
            } else {
                c.curves.clone()
            };
            if c.closed() && !c.curves.is_empty() {
                let a = source.first()?.endpoints().0;
                let b = source.last()?.endpoints().1;
                if a != b {
                    source.push(Curve::Line(b, a));
                }
            }
            for curve in &source {
                let Ok(points) = curve.sample() else {
                    continue;
                };
                let closed = matches!(
                    curve,
                    Curve::Ellipse(_) | Curve::Spline { closed: true, .. }
                );
                let mut d = points
                    .windows(2)
                    .map(|s| distance(p, s[0], s[1]))
                    .fold(f64::INFINITY, f64::min);
                if closed && points.len() > 1 {
                    d = d.min(distance(p, *points.last()?, points[0]));
                }
                if d <= best {
                    best = d;
                    result = Some(if mode == Mode::Whole {
                        Candidate {
                            points: c.points.clone(),
                            curves: source.clone(),
                            closed: c.closed(),
                        }
                    } else {
                        Candidate {
                            points,
                            curves: vec![curve.clone()],
                            closed,
                        }
                    });
                }
            }
        }
        if let Some((path, index, d)) = self.index.path_hit(p, self.camera.zoom) {
            if d < best {
                let source = self.drawing.as_ref()?.paths.get(path)?;
                result = Some(if mode == Mode::Whole {
                    Candidate {
                        points: source.points.clone(),
                        curves: curves(&source.points, source.closed),
                        closed: source.closed,
                    }
                } else {
                    let a = source.points[index];
                    let b = source.points[(index + 1) % source.points.len()];
                    Candidate {
                        points: vec![a, b],
                        curves: vec![Curve::Line(a, b)],
                        closed: false,
                    }
                });
            }
        }
        result
    }
    fn add_reused(&mut self, mut candidate: Candidate) -> Result<(), String> {
        if self.mode.is_none() || self.mode == Some(Kind::LoadPoint) {
            return Err("Сначала начните построение объекта".into());
        }
        if candidate.closed && !self.draft.is_empty() {
            return Err(
                "Замкнутую границу можно добавить только в новое построение. Завершите текущее"
                    .into(),
            );
        }
        if self.draft.len() + candidate.points.len() > 4096 {
            return Err("Объект превышает лимит 4096 вершин".into());
        }
        if candidate.points.len() < 2 {
            return Err("У элемента недостаточно точек".into());
        }
        if let Some(&last) = self.draft.last() {
            let a = candidate.points[0];
            let b = *candidate.points.last().unwrap();
            if (last[0] - b[0]).hypot(last[1] - b[1]) < (last[0] - a[0]).hypot(last[1] - a[1]) {
                candidate.points.reverse();
                candidate.curves.reverse();
                for c in &mut candidate.curves {
                    reverse(c);
                }
            }
        }
        self.sketch_snapshot();
        let first = candidate.points[0];
        if let Some(&last) = self.draft.last() {
            if last != first {
                self.draft_segments.push(Curve::Line(last, first));
                self.draft.push(first);
            }
        } else {
            self.draft.push(first);
        }
        self.draft.extend(candidate.points.iter().copied().skip(1));
        self.draft_segments.extend(candidate.curves);
        self.curve_controls.clear();
        self.tool = Tool::Line;
        let closes = self.draft.len() > 2 && self.draft.first() == self.draft.last();
        if candidate.closed || closes {
            self.draft_line_closed = true;
            self.finish();
        } else {
            self.status = "Элемент добавлен. Продолжайте выбор или завершите построение".into();
        }
        Ok(())
    }
    pub(super) fn reuse_canvas(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) {
        let Some(mode) = self.reuse_mode else {
            return;
        };
        let Some(p) = hovered else {
            return;
        };
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        if let Some(candidate) = self.reuse_hit(self.camera.world(p, rect), mode) {
            draw_path(
                ui.painter(),
                &candidate.points,
                candidate.closed,
                self.camera,
                rect,
                Stroke::new(3_f32, Color32::WHITE),
            );
            if response.clicked_by(egui::PointerButton::Primary) {
                if let Err(e) = self.add_reused(candidate) {
                    self.status = e;
                }
            }
        }
    }
}
