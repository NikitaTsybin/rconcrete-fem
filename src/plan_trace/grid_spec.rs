//! Persistent parametric grid geometry, independent of viewport and FEM IDs.
use super::Point;
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct GridSpec {
    pub origin: Point,
    pub ex: Point,
    pub ey: Point,
    pub nx: usize,
    pub ny: usize,
}
impl GridSpec {
    pub fn dx(&self) -> f64 {
        self.ex[0].hypot(self.ex[1])
    }
    pub fn dy(&self) -> f64 {
        self.ey[0].hypot(self.ey[1])
    }
    pub fn cell_area(&self) -> f64 {
        (self.ex[0] * self.ey[1] - self.ex[1] * self.ey[0]).abs()
    }
    pub fn at(&self, x: f64, y: f64) -> Point {
        std::array::from_fn(|a| self.origin[a] + x * self.ex[a] + y * self.ey[a])
    }
    pub fn coordinates(&self, p: Point) -> [f64; 2] {
        let r = [p[0] - self.origin[0], p[1] - self.origin[1]];
        let d = self.ex[0] * self.ey[1] - self.ex[1] * self.ey[0];
        [
            (r[0] * self.ey[1] - r[1] * self.ey[0]) / d,
            (self.ex[0] * r[1] - self.ex[1] * r[0]) / d,
        ]
    }
    pub fn nearest_node(&self, p: Point) -> Point {
        let [x, y] = self.coordinates(p);
        self.at(
            x.round().clamp(0., self.nx as f64),
            y.round().clamp(0., self.ny as f64),
        )
    }
    pub fn contains(&self, p: Point) -> bool {
        let r = [p[0] - self.origin[0], p[1] - self.origin[1]];
        let d = self.ex[0] * self.ey[1] - self.ex[1] * self.ey[0];
        if d.abs() < 1e-12 {
            return false;
        }
        let x = (r[0] * self.ey[1] - r[1] * self.ey[0]) / d;
        let y = (self.ex[0] * r[1] - self.ex[1] * r[0]) / d;
        x >= -1e-8 && y >= -1e-8 && x <= self.nx as f64 + 1e-8 && y <= self.ny as f64 + 1e-8
    }
    pub fn points(&self) -> Result<Vec<Point>, String> {
        if self.nx == 0
            || self.ny == 0
            || self.nx > 500
            || self.ny > 500
            || (self.nx + 1) * (self.ny + 1) > 100_000
            || self
                .origin
                .iter()
                .chain(&self.ex)
                .chain(&self.ey)
                .any(|v| !v.is_finite())
            || self.dx() < 1e-6
            || self.dy() < 1e-6
            || (self.ex[0] * self.ey[0] + self.ex[1] * self.ey[1]).abs()
                > self.dx() * self.dy() * 1e-8
        {
            return Err("Некорректные параметры прямоугольной сетки".into());
        }
        let mut points = Vec::new();
        let mut push = |i: f64, j: f64| {
            let p = self.at(i, j);
            if points.last() != Some(&p) {
                points.push(p);
            }
        };
        for i in 0..=self.nx {
            if i % 2 == 0 {
                push(i as f64, 0.);
                push(i as f64, self.ny as f64);
            } else {
                push(i as f64, self.ny as f64);
                push(i as f64, 0.);
            }
        }
        if self.nx % 2 == 0 {
            push(self.nx as f64, 0.);
        }
        push(0., 0.);
        for j in 0..=self.ny {
            if j % 2 == 0 {
                push(0., j as f64);
                push(self.nx as f64, j as f64);
            } else {
                push(self.nx as f64, j as f64);
                push(0., j as f64);
            }
        }
        if points.iter().flatten().any(|v| !v.is_finite()) {
            return Err("Слишком большие координаты сетки".into());
        }
        Ok(points)
    }
    /// Recognize exactly the old generated walk; do not reinterpret arbitrary paths.
    pub fn infer(points: &[Point]) -> Option<Self> {
        if points.len() < 8 {
            return None;
        }
        let axis = |a: usize| {
            let mut v = points.iter().map(|p| p[a]).collect::<Vec<_>>();
            v.sort_by(f64::total_cmp);
            v.dedup_by(|a, b| (*a - *b).abs() < 1e-6);
            v
        };
        let x = axis(0);
        let y = axis(1);
        if x.len() < 2 || y.len() < 2 {
            return None;
        }
        let spec = Self {
            origin: points[0],
            ex: [x[1] - x[0], 0.],
            ey: [0., y[1] - y[0]],
            nx: x.len() - 1,
            ny: y.len() - 1,
        };
        let expected = spec.points().ok()?;
        (expected.len() == points.len()
            && expected
                .iter()
                .zip(points)
                .all(|(a, b)| (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-6))
        .then_some(spec)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn parameter_roundtrip_legacy_recognition_and_cell_domain() {
        let grid = GridSpec {
            origin: [300., 700.],
            ex: [1000., 0.],
            ey: [0., 800.],
            nx: 3,
            ny: 2,
        };
        let points = grid.points().unwrap();
        assert_eq!(GridSpec::infer(&points), Some(grid.clone()));
        assert!(grid.contains(grid.at(1.5, 1.5)));
        assert!(!grid.contains(grid.at(3.1, 0.5)));
        let restored: GridSpec =
            serde_json::from_slice(&serde_json::to_vec(&grid).unwrap()).unwrap();
        assert_eq!(restored, grid);
    }
}
