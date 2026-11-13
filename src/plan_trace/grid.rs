//! Construction-only XY grid. It never contributes nodes or elements to the FE model.
use super::geometry::Point;
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy)]
#[serde(default)]
pub struct Grid {
    pub color: [u8; 3],
    pub center: Point,
    pub width_mm: f64,
    pub height_mm: f64,
    pub enabled: bool,
    pub dx: f64,
    pub dy: f64,
}
impl Default for Grid {
    fn default() -> Self {
        Self {
            color: [70, 75, 85],
            center: [0., 0.],
            width_mm: 20000.,
            height_mm: 20000.,
            enabled: false,
            dx: 1000.,
            dy: 1000.,
        }
    }
}
impl Grid {
    fn valid(self) -> bool {
        self.enabled && self.parameters_valid()
    }
    pub fn parameters_valid(self) -> bool {
        self.dx.is_finite()
            && self.dy.is_finite()
            && self.dx > 0.
            && self.dy > 0.
            && self.width_mm.is_finite()
            && self.height_mm.is_finite()
            && self.width_mm > 0.
            && self.height_mm > 0.
            && self.center.iter().all(|v| v.is_finite())
    }
    fn bounds(self) -> [f64; 4] {
        [
            self.center[0] - self.width_mm / 2.,
            self.center[1] - self.height_mm / 2.,
            self.center[0] + self.width_mm / 2.,
            self.center[1] + self.height_mm / 2.,
        ]
    }
    pub fn snap(self, p: Point, zoom: f64) -> Option<Point> {
        if !self.valid() || !zoom.is_finite() || zoom <= 0. || p.iter().any(|v| !v.is_finite()) {
            return None;
        }
        let q = [
            ((p[0] - self.center[0]) / self.dx).round() * self.dx + self.center[0],
            ((p[1] - self.center[1]) / self.dy).round() * self.dy + self.center[1],
        ];
        let b = self.bounds();
        if q[0] < b[0] || q[1] < b[1] || q[0] > b[2] || q[1] > b[3] {
            return None;
        }
        (((q[0] - p[0]).hypot(q[1] - p[1]) * zoom) <= 10.).then_some(q)
    }
    pub fn lines(self, bounds: [f64; 4], zoom: f64) -> Vec<[Point; 2]> {
        if !self.valid() || !zoom.is_finite() || zoom <= 0. || bounds.iter().any(|v| !v.is_finite())
        {
            return vec![];
        }
        let region = self.bounds();
        let bounds = [
            bounds[0].max(region[0]),
            bounds[1].max(region[1]),
            bounds[2].min(region[2]),
            bounds[3].min(region[3]),
        ];
        if bounds[0] > bounds[2] || bounds[1] > bounds[3] {
            return vec![];
        }
        let visible_step = |step: f64| {
            let scale = (8. / (step * zoom)).log10().ceil().max(0.);
            step * 10_f64.powf(scale)
        };
        let dx = visible_step(self.dx);
        let dy = visible_step(self.dy);
        let mut lines = vec![];
        for axis in 0..2 {
            let step = if axis == 0 { dx } else { dy };
            if !step.is_finite() {
                continue;
            }
            let start =
                ((bounds[axis] - self.center[axis]) / step).ceil() * step + self.center[axis];
            let count = (((bounds[axis + 2] - start) / step).floor() + 1.).max(0.) as usize;
            for i in 0..count.min(2048) {
                let v = start + i as f64 * step;
                lines.push(if axis == 0 {
                    [[v, bounds[1]], [v, bounds[3]]]
                } else {
                    [[bounds[0], v], [bounds[2], v]]
                });
            }
        }
        lines
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finite_grid_has_no_lines_or_snaps_outside_its_configured_area() {
        let g = Grid {
            enabled: true,
            ..Default::default()
        };
        assert!(g.lines([20000., 20000., 30000., 30000.], 1.).is_empty());
        assert_eq!(g.snap([20000., 20000.], 1.), None);
        let lines = g.lines([-1e6, -1e6, 1e6, 1e6], 1.);
        assert!(lines
            .iter()
            .flatten()
            .all(|p| p[0].abs() <= 10000. && p[1].abs() <= 10000.));
    }
    #[test]
    fn spacing_negative_coordinates_and_disabled_snaps() {
        let g = Grid {
            enabled: true,
            dx: 100.,
            dy: 250.,
            ..Default::default()
        };
        assert_eq!(g.snap([-98., 247.], 1.), Some([-100., 250.]));
        assert_eq!(g.snap([45., 100.], 1.), None);
        assert_eq!(Grid::default().snap([0., 0.], 1.), None);
        let lines = g.lines([-200., -500., 200., 500.], 1.);
        assert_eq!(lines.len(), 10);
        assert!(lines.contains(&[[0., -500.], [0., 500.]]));
        let far = g.lines([-1e12, -1e12, 1e12, 1e12], 1e-10);
        assert!(far.len() < 1000);
        assert!(Grid {
            enabled: true,
            dx: 0.,
            dy: 100.,
            ..Default::default()
        }
        .lines([0., 0., 100., 100.], 1.)
        .is_empty());
    }
}
