use super::*;
fn offsets(points: &[Point], closed: bool, r: f64) -> Vec<Vec<Point>> {
    let mut p = Vec::new();
    for &v in points {
        if p.last()
            .is_none_or(|q: &Point| (v[0] - q[0]).hypot(v[1] - q[1]) > 1e-6)
        {
            p.push(v)
        }
    }
    if closed && p.len() > 1 && p.first() == p.last() {
        p.pop();
    }
    if p.len() < 2 || !r.is_finite() || r <= 0. {
        return vec![];
    }
    let count = p.len();
    let normal = |a: Point, b: Point| {
        let x = b[0] - a[0];
        let y = b[1] - a[1];
        let l = x.hypot(y);
        [-y / l, x / l]
    };
    let mut paths: Vec<Vec<Point>> = [-1., 1.]
        .into_iter()
        .map(|sign| {
            let mut path = vec![];
            for i in 0..count {
                let n0 = normal(
                    p[if i == 0 {
                        if closed {
                            count - 1
                        } else {
                            0
                        }
                    } else {
                        i - 1
                    }],
                    p[if i == 0 && !closed { 1 } else { i }],
                );
                let n1 = normal(
                    p[if i == count - 1 && !closed {
                        count - 2
                    } else {
                        i
                    }],
                    p[if i + 1 == count {
                        if closed {
                            0
                        } else {
                            count - 1
                        }
                    } else {
                        i + 1
                    }],
                );
                let n = if !closed && i == 0 {
                    n1
                } else if !closed && i + 1 == count {
                    n0
                } else {
                    let dot = 1. + n0[0] * n1[0] + n0[1] * n1[1];
                    if dot > 0.125 {
                        [(n0[0] + n1[0]) / dot, (n0[1] + n1[1]) / dot]
                    } else {
                        n1
                    }
                };
                path.push([p[i][0] + sign * r * n[0], p[i][1] + sign * r * n[1]]);
            }
            if closed {
                path.push(path[0]);
            }
            path
        })
        .collect();
    if !closed {
        let left = &paths[0];
        let right = &paths[1];
        let first = vec![left[0], right[0]];
        let last = vec![*left.last().unwrap(), *right.last().unwrap()];
        paths.push(first);
        paths.push(last);
    }
    paths
}
impl App {
    pub(super) fn support_extent_paths(&self, exclude: Option<u64>) -> Vec<Vec<Point>> {
        self.support_extent_objects(exclude)
            .into_iter()
            .map(|(_, _, path)| path)
            .collect()
    }
    fn support_extent_objects(&self, exclude: Option<u64>) -> Vec<(u64, Kind, Vec<Point>)> {
        if !self.drawing_visibility.support_extents {
            return vec![];
        }
        let materials = self.stiffnesses.resolve(&self.contours, Kind::Support);
        self.contours
            .iter()
            .filter(|c| c.kind == Kind::Support && Some(c.id) != exclude && self.contour_visible(c))
            .flat_map(|c| {
                offsets(&c.points, c.closed(), materials[&c.id].thickness_mm * 0.5)
                    .into_iter()
                    .map(move |path| (c.id, c.kind, path))
            })
            .chain(
                self.contours
                    .iter()
                    .filter(|c| {
                        c.kind == Kind::Column && Some(c.id) != exclude && self.contour_visible(c)
                    })
                    .filter_map(|c| {
                        let &p = c.points.first()?;
                        let section = self.support_height.columns.get(&c.id)?;
                        let mut path =
                            crate::plan_trace::columns::section_corners(c, section).to_vec();
                        path.push(path[0]);
                        Some((c.id, c.kind, path))
                    }),
            )
            .collect()
    }
    pub(super) fn paint_support_extents(&self, painter: &egui::Painter, rect: Rect) {
        let start = self.draw_order.start(painter);
        for (id, kind, path) in self.support_extent_objects(None) {
            let screen: Vec<_> = path
                .into_iter()
                .map(|p| self.camera.screen(p, rect))
                .collect();
            painter.extend(egui::Shape::dashed_line(
                &screen,
                Stroke::new(
                    1.,
                    self.property_ink(id, kind)
                        .unwrap_or_else(|| self.contour_color(Kind::Support)),
                ),
                6.,
                4.,
            ));
        }
        self.draw_order
            .finish(painter, draw_order::category(Kind::Support), start);
    }
}
