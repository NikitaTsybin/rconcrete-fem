//! Curve tools are discretized in model coordinates (millimetres), never in pixels.
use super::geometry::Point;
const TOLERANCE: f64 = 1.;
const LIMIT: usize = 4096;
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum Tool {
    #[default]
    Line,
    Spline,
    Arc,
    Ellipse,
    Rectangle,
}
impl Tool {
    pub const ALL: [Self; 5] = [
        Self::Line,
        Self::Spline,
        Self::Arc,
        Self::Ellipse,
        Self::Rectangle,
    ];
    pub fn name(self) -> &'static str {
        match self {
            Self::Line => "Отрезок",
            Self::Spline => "Сплайн",
            Self::Arc => "Дуга",
            Self::Ellipse => "Эллипс",
            Self::Rectangle => "Прямоугольник",
        }
    }
    pub fn hint(self) -> &'static str {
        match self {
        Self::Line=>"Указывайте вершины. Щелчок по первой или Enter — замкнуть.",
        Self::Spline=>"Указывайте точки сплайна. Первая точка или Enter — замкнуть; выбор отрезка/дуги завершает участок сплайна.",
        Self::Arc=>"Дуга: начало, промежуточная точка, конец. Можно продолжить следующую дугу или перейти к отрезкам.",
        Self::Ellipse=>"Эллипс: центр, конец первой полуоси, точка для второй полуоси. Третий щелчок завершает построение.",
        Self::Rectangle=>"Прямоугольник: два противоположных угла. Второй щелчок завершает построение.",
    }
    }
}
fn dist(a: Point, b: Point) -> f64 {
    (a[0] - b[0]).hypot(a[1] - b[1])
}
fn mid(a: Point, b: Point) -> Point {
    [(a[0] + b[0]) * 0.5, (a[1] + b[1]) * 0.5]
}
pub fn rectangle(a: Point, b: Point) -> Result<Vec<Point>, String> {
    if (a[0] - b[0]).abs() < 1e-6 || (a[1] - b[1]).abs() < 1e-6 {
        return Err("Прямоугольник имеет нулевую ширину или высоту".into());
    }
    Ok(vec![a, [b[0], a[1]], b, [a[0], b[1]]])
}
pub fn ellipse(c: Point, a: Point, b: Point) -> Result<Vec<Point>, String> {
    let r = dist(c, a);
    if r < 1e-6 {
        return Err("Укажите ненулевую первую полуось".into());
    }
    let u = [(a[0] - c[0]) / r, (a[1] - c[1]) / r];
    let v = [-u[1], u[0]];
    let s = ((b[0] - c[0]) * v[0] + (b[1] - c[1]) * v[1]).abs();
    if s < 1e-6 {
        return Err("Укажите ненулевую вторую полуось".into());
    }
    let n = ((std::f64::consts::TAU / (2. * (1. - TOLERANCE / r.max(s)).clamp(-1., 1.).acos()))
        .ceil() as usize)
        .max(32);
    if n > LIMIT {
        return Err("Слишком большой эллипс для точности 1 мм".into());
    }
    Ok((0..n)
        .map(|i| {
            let t = std::f64::consts::TAU * i as f64 / n as f64;
            [
                c[0] + r * t.cos() * u[0] + s * t.sin() * v[0],
                c[1] + r * t.cos() * u[1] + s * t.sin() * v[1],
            ]
        })
        .collect())
}
pub fn arc(a: Point, b: Point, c: Point) -> Result<Vec<Point>, String> {
    let u = [b[0] - a[0], b[1] - a[1]];
    let v = [c[0] - a[0], c[1] - a[1]];
    let d = 2. * (u[0] * v[1] - u[1] * v[0]);
    if d.abs() < 1e-9 * dist(a, b) * dist(a, c) {
        return Err("Три точки дуги не должны лежать на одной прямой".into());
    }
    let uu = u[0] * u[0] + u[1] * u[1];
    let vv = v[0] * v[0] + v[1] * v[1];
    let o = [
        a[0] + (uu * v[1] - vv * u[1]) / d,
        a[1] + (u[0] * vv - v[0] * uu) / d,
    ];
    let r = dist(a, o);
    if !r.is_finite() || r < 1e-6 {
        return Err("Некорректная дуга".into());
    }
    let angle = |p: Point| (p[1] - o[1]).atan2(p[0] - o[0]);
    let t = angle(a);
    let end = (angle(c) - t).rem_euclid(std::f64::consts::TAU);
    let sweep = if (angle(b) - t).rem_euclid(std::f64::consts::TAU) <= end {
        end
    } else {
        end - std::f64::consts::TAU
    };
    let n =
        ((sweep.abs() / (2. * (1. - TOLERANCE / r).clamp(-1., 1.).acos())).ceil() as usize).max(2);
    if n >= LIMIT {
        return Err("Слишком большая дуга для точности 1 мм".into());
    }
    let mut points = (0..=n)
        .map(|i| {
            let q = t + sweep * i as f64 / n as f64;
            [o[0] + r * q.cos(), o[1] + r * q.sin()]
        })
        .collect::<Vec<_>>();
    points[0] = a;
    points[n] = c;
    Ok(points)
}
fn segment_distance(p: Point, a: Point, b: Point) -> f64 {
    let d = [b[0] - a[0], b[1] - a[1]];
    let len = d[0] * d[0] + d[1] * d[1];
    if len < 1e-20 {
        return dist(p, a);
    }
    let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len).clamp(0., 1.);
    dist(p, [a[0] + t * d[0], a[1] + t * d[1]])
}
fn bezier(p: [Point; 4], depth: usize, out: &mut Vec<Point>) -> Result<(), String> {
    if out.len() >= LIMIT {
        return Err("Слишком сложный сплайн (лимит 4096 точек)".into());
    }
    if segment_distance(p[1], p[0], p[3]).max(segment_distance(p[2], p[0], p[3])) <= TOLERANCE {
        out.push(p[3]);
        return Ok(());
    }
    if depth >= 20 {
        return Err("Не удалось аппроксимировать сплайн с точностью 1 мм".into());
    }
    let a = mid(p[0], p[1]);
    let b = mid(p[1], p[2]);
    let c = mid(p[2], p[3]);
    let d = mid(a, b);
    let e = mid(b, c);
    let f = mid(d, e);
    bezier([p[0], a, d, f], depth + 1, out)?;
    bezier([f, e, c, p[3]], depth + 1, out)
}
pub fn spline(p: &[Point], closed: bool) -> Result<Vec<Point>, String> {
    if p.len() < if closed { 3 } else { 2 } {
        return Err("Недостаточно точек сплайна".into());
    }
    let mut out = vec![p[0]];
    for i in 0..if closed { p.len() } else { p.len() - 1 } {
        let a = p[i];
        let b = p[(i + 1) % p.len()];
        let prev = if i > 0 {
            p[i - 1]
        } else if closed {
            p[p.len() - 1]
        } else {
            [2. * a[0] - b[0], 2. * a[1] - b[1]]
        };
        let next = if i + 2 < p.len() {
            p[i + 2]
        } else if closed {
            p[(i + 2) % p.len()]
        } else {
            [2. * b[0] - a[0], 2. * b[1] - a[1]]
        };
        bezier(
            [
                a,
                [a[0] + (b[0] - prev[0]) / 6., a[1] + (b[1] - prev[1]) / 6.],
                [b[0] - (next[0] - a[0]) / 6., b[1] - (next[1] - a[1]) / 6.],
                b,
            ],
            0,
            &mut out,
        )?;
    }
    if closed {
        out.pop();
    }
    Ok(out)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn arc_orientation_endpoints_and_error_bound() {
        for b in [[0., 1000.], [0., -1000.]] {
            let p = arc([-1000., 0.], b, [1000., 0.]).unwrap();
            assert_eq!(p[0], [-1000., 0.]);
            assert_eq!(*p.last().unwrap(), [1000., 0.]);
            assert!(p.iter().all(|v| v[1] * b[1] >= -1e-6));
            for s in p.windows(2) {
                assert!(1000. - dist([0., 0.], mid(s[0], s[1])) <= 1.001);
            }
        }
        assert!(arc([0., 0.], [1., 0.], [2., 0.]).is_err());
    }
    #[test]
    fn closed_shapes_and_interpolating_spline_are_valid() {
        let e = ellipse([0., 0.], [1000., 1000.], [-500., 500.]).unwrap();
        super::super::model::valid(&e).unwrap();
        assert_eq!(rectangle([0., 0.], [100., 200.]).unwrap().len(), 4);
        assert!(ellipse([0., 0.], [100., 0.], [50., 0.]).is_err());
        let controls = vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]];
        let p = spline(&controls, true).unwrap();
        for c in controls {
            assert!(p.contains(&c));
        }
        assert_ne!(p[0], *p.last().unwrap());
        super::super::model::valid(&p).unwrap();
    }
}

/// Minor circular arc with fixed endpoints and radius; cursor selects the bulge side.
pub fn arc_by_radius(a: Point, b: Point, radius: f64, side: Point) -> Result<[Point; 3], String> {
    let length = dist(a, b);
    if !radius.is_finite() || radius <= 0. || length < 1e-6 {
        return Err("Укажите разные концы дуги и положительный радиус".into());
    }
    let half = length * 0.5;
    if radius < half {
        return Err(format!("Радиус должен быть не меньше {:.3} мм", half));
    }
    let middle = mid(a, b);
    let normal = [-(b[1] - a[1]) / length, (b[0] - a[0]) / length];
    let sign = if (side[0] - middle[0]) * normal[0] + (side[1] - middle[1]) * normal[1] < 0. {
        -1.
    } else {
        1.
    };
    let height = (radius - half).sqrt() * (radius + half).sqrt();
    // Stable sagitta even for a very large radius.
    let sagitta = half * (half / (radius + height));
    Ok([
        a,
        [
            middle[0] + sign * normal[0] * sagitta,
            middle[1] + sign * normal[1] * sagitta,
        ],
        b,
    ])
}

#[cfg(test)]
mod radius_tests {
    use super::*;
    #[test]
    fn radius_arc_preserves_chord_and_radius_on_both_sides() {
        for side in [-1., 1.] {
            let q = arc_by_radius([0., 0.], [600., 0.], 500., [300., side * 200.]).unwrap();
            assert_eq!(q[0], [0., 0.]);
            assert_eq!(q[2], [600., 0.]);
            assert!((q[1][1] - side * 100.).abs() < 1e-10);
            let points = arc(q[0], q[1], q[2]).unwrap();
            for p in points {
                assert!(((p[0] - 300.).hypot(p[1] + side * 400.) - 500.).abs() < 1e-7);
            }
        }
        assert!(arc_by_radius([0., 0.], [600., 0.], 299., [300., 100.]).is_err());
        let q = arc_by_radius([0., 0.], [600., 0.], 300., [300., 100.]).unwrap();
        assert_eq!(q[1], [300., 300.]);
    }
}
