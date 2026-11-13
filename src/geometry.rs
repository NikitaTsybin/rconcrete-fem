pub type Point = [f64; 2];
pub fn cross(a: Point, b: Point, c: Point) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}
pub fn area(poly: &[Point]) -> f64 {
    if poly.len() < 3 {
        return 0.;
    }
    // Translate before summation: retain accuracy for drawings far from global origin.
    (1..poly.len() - 1)
        .map(|i| cross(poly[0], poly[i], poly[i + 1]))
        .sum::<f64>()
        .abs()
        * 0.5
}
pub fn inside(point: Point, poly: &[Point]) -> bool {
    if poly.len() < 3 {
        return false;
    }
    let mut yes = false;
    for i in 0..poly.len() {
        let a = poly[i];
        let b = poly[(i + 1) % poly.len()];
        if (a[1] > point[1]) != (b[1] > point[1])
            && point[0] < (b[0] - a[0]) * (point[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            yes = !yes;
        }
    }
    yes
}
pub fn in_rings(p: Point, rings: &[Vec<Point>]) -> bool {
    rings.iter().filter(|r| inside(p, r)).count() % 2 == 1
}
pub fn signs(rings: &[Vec<Point>]) -> Vec<f64> {
    rings
        .iter()
        .enumerate()
        .map(|(i, r)| {
            let depth = rings
                .iter()
                .enumerate()
                .filter(|(j, other)| *j != i && inside(r[0], other))
                .count();
            if depth % 2 == 0 {
                1.
            } else {
                -1.
            }
        })
        .collect()
}
pub fn bounds(rings: &[Vec<Point>]) -> [f64; 4] {
    let mut b = [
        f64::INFINITY,
        f64::INFINITY,
        f64::NEG_INFINITY,
        f64::NEG_INFINITY,
    ];
    for p in rings.iter().flatten() {
        b[0] = b[0].min(p[0]);
        b[1] = b[1].min(p[1]);
        b[2] = b[2].max(p[0]);
        b[3] = b[3].max(p[1]);
    }
    b
}
pub fn intersection_area(rings: &[Vec<Point>], signs: &[f64], rect: [f64; 4]) -> f64 {
    let mut total = 0.;
    for (ring, sign) in rings.iter().zip(signs) {
        let mut polygon = ring.clone();
        for (axis, limit, lower) in [
            (0, rect[0], true),
            (0, rect[2], false),
            (1, rect[1], true),
            (1, rect[3], false),
        ] {
            if polygon.is_empty() {
                break;
            }
            let mut clipped = Vec::with_capacity(polygon.len() + 4);
            for i in 0..polygon.len() {
                let a = polygon[i];
                let b = polygon[(i + 1) % polygon.len()];
                let ia = if lower {
                    a[axis] >= limit
                } else {
                    a[axis] <= limit
                };
                let ib = if lower {
                    b[axis] >= limit
                } else {
                    b[axis] <= limit
                };
                if ia {
                    clipped.push(a);
                }
                if ia != ib {
                    let t = (limit - a[axis]) / (b[axis] - a[axis]);
                    let mut p = [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])];
                    p[axis] = limit;
                    clipped.push(p);
                }
            }
            polygon = clipped;
        }
        total += sign * area(&polygon);
    }
    total.max(0.)
}
/// Independent triangulation for drawing. Each exterior includes only its immediate holes.
pub fn triangulate(rings: &[Vec<Point>]) -> Result<Vec<[Point; 3]>, String> {
    let signs = signs(rings);
    let mut triangles = Vec::new();
    for (i, outer) in rings.iter().enumerate().filter(|(i, _)| signs[*i] > 0.) {
        let mut points = outer.clone();
        let mut holes = Vec::new();
        for (j, hole) in rings.iter().enumerate().filter(|(j, _)| signs[*j] < 0.) {
            if !inside(hole[0], outer) {
                continue;
            }
            if rings.iter().enumerate().any(|(k, r)| {
                k != i && k != j && signs[k] > 0. && inside(hole[0], r) && inside(r[0], outer)
            }) {
                continue;
            }
            holes.push(points.len());
            points.extend_from_slice(hole);
        }
        let flat: Vec<_> = points.iter().flat_map(|p| p.iter().copied()).collect();
        let indices = earcutr::earcut(&flat, &holes, 2)
            .map_err(|e| format!("Не удалось триангулировать контур: {e:?}"))?;
        triangles.extend(
            indices
                .chunks_exact(3)
                .map(|t| [points[t[0]], points[t[1]], points[t[2]]]),
        );
    }
    Ok(triangles)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn hole_is_not_filled_and_orientation_does_not_matter() {
        let rings = vec![
            vec![[0., 0.], [4., 0.], [4., 4.], [0., 4.]],
            vec![[1., 1.], [3., 1.], [3., 3.], [1., 3.]],
        ];
        let a = intersection_area(&rings, &signs(&rings), [0., 0., 4., 4.]);
        assert!((a - 12.).abs() < 1e-9);
        assert!(!in_rings([2., 2.], &rings));
        let sum: f64 = triangulate(&rings).unwrap().iter().map(|t| area(t)).sum();
        assert!((sum - 12.).abs() < 1e-9);
        assert_eq!(
            intersection_area(&rings, &signs(&rings), [1.5, 1.5, 2.5, 2.5]),
            0.
        );
    }
}
