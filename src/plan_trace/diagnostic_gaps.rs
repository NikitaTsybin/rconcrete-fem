//! Short positive distances between physical contour edges, in millimetres.
use super::*;
use glam::DVec3;
struct Edge {
    id: u64,
    p: [DVec3; 2],
    bbox: AABB<[f64; 3]>,
}
// Closest points on two finite segments, including zero-length column/point edges.
fn closest(a: [DVec3; 2], b: [DVec3; 2]) -> (f64, [[f64; 3]; 2]) {
    let u = a[1] - a[0];
    let v = b[1] - b[0];
    let w = a[0] - b[0];
    let aa = u.length_squared();
    let bb = u.dot(v);
    let cc = v.length_squared();
    let dd = u.dot(w);
    let ee = v.dot(w);
    let mut candidates = vec![
        (0., if cc > 0. { (ee / cc).clamp(0., 1.) } else { 0. }),
        (
            1.,
            if cc > 0. {
                ((ee + bb) / cc).clamp(0., 1.)
            } else {
                0.
            },
        ),
        (
            if aa > 0. {
                (-dd / aa).clamp(0., 1.)
            } else {
                0.
            },
            0.,
        ),
        (
            if aa > 0. {
                ((bb - dd) / aa).clamp(0., 1.)
            } else {
                0.
            },
            1.,
        ),
    ];
    let det = aa * cc - bb * bb;
    if det > 1e-12 * aa * cc {
        let s = (bb * ee - cc * dd) / det;
        let t = (aa * ee - bb * dd) / det;
        if (0. ..=1.).contains(&s) && (0. ..=1.).contains(&t) {
            candidates.push((s, t));
        }
    }
    candidates
        .into_iter()
        .map(|(s, t)| {
            let p = a[0] + u * s;
            let q = b[0] + v * t;
            (p.distance(q), [p.to_array(), q.to_array()])
        })
        .min_by(|a, b| a.0.total_cmp(&b.0))
        .unwrap()
}
pub(in crate::plan_trace) fn nearby(
    all: &[Contour],
    heights: &BTreeMap<u64, Vec<f64>>,
    step: f64,
    threshold: f64,
) -> Result<Vec<Issue>, String> {
    if !threshold.is_finite() || threshold <= 0. {
        return Ok(vec![]);
    }
    let mut edges = vec![];
    for c in all.iter().filter(|c| {
        matches!(
            c.kind,
            Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
        )
    }) {
        let p = c.mesh_boundary(step)?;
        let Some(h) = heights.get(&c.id).filter(|h| !h.is_empty()) else {
            continue;
        };
        if p.is_empty() {
            continue;
        }
        let lo = h[0].min(*h.last().unwrap());
        let hi = h[0].max(*h.last().unwrap());
        let mut add = |a: Point, za: f64, b: Point, zb: f64| {
            let p = [DVec3::new(a[0], a[1], za), DVec3::new(b[0], b[1], zb)];
            edges.push(Edge {
                id: c.id,
                p,
                bbox: AABB::from_corners(p[0].min(p[1]).to_array(), p[0].max(p[1]).to_array()),
            });
        };
        if c.kind == Kind::Column {
            add(p[0], lo, p[0], hi);
            continue;
        }
        let mut path = p;
        if c.closed() {
            path.push(path[0]);
        }
        for e in path.windows(2) {
            add(e[0], lo, e[1], lo);
            if c.kind == Kind::Support {
                add(e[0], hi, e[1], hi);
                add(e[0], lo, e[0], hi);
                add(e[1], lo, e[1], hi);
            }
        }
    }
    let tree = RTree::bulk_load(
        edges
            .iter()
            .enumerate()
            .map(|(i, e)| Index { i, bbox: e.bbox })
            .collect(),
    );
    let mut pairs = BTreeMap::<[u64; 2], (f64, [[f64; 3]; 2])>::new();
    for (i, a) in edges.iter().enumerate() {
        let query = AABB::from_corners(
            a.bbox.lower().map(|v| v - threshold),
            a.bbox.upper().map(|v| v + threshold),
        );
        for j in tree.locate_in_envelope_intersecting(&query) {
            let b = &edges[j.i];
            if j.i <= i || a.id == b.id {
                continue;
            }
            let ids = [a.id.min(b.id), a.id.max(b.id)];
            if pairs.get(&ids).is_some_and(|v| v.0 <= 1e-5) {
                continue;
            }
            let found = closest(a.p, b.p);
            if found.0 < threshold && pairs.get(&ids).is_none_or(|v| found.0 < v.0) {
                pairs.insert(ids, found);
            }
        }
    }
    Ok(pairs
        .into_iter()
        .filter(|(_, v)| v.0 > 1e-5)
        .take(1000)
        .map(|(ids, (d, p))| Issue {
            ids: ids.to_vec(),
            points: p.to_vec(),
            message: format!(
                "Близкие объекты {} и {}: зазор {:.3} мм (порог {:.3} мм)",
                ids[0], ids[1], d, threshold
            ),
        })
        .collect())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn finite_segments_include_skew_interior_and_endpoint_gaps() {
        let v = |x, y, z| DVec3::new(x, y, z);
        assert!(
            (closest(
                [v(0., 0., 0.), v(100., 0., 0.)],
                [v(50., -50., 10.), v(50., 50., 10.)]
            )
            .0 - 10.)
                .abs()
                < 1e-9
        );
        assert!(
            (closest(
                [v(0., 0., 0.), v(100., 0., 0.)],
                [v(110., 0., 0.), v(200., 0., 0.)]
            )
            .0 - 10.)
                .abs()
                < 1e-9
        );
    }
    #[test]
    fn nearby_uses_xyz_strict_threshold_and_excludes_connected_pairs() {
        let c = |id, p| {
            serde_json::from_value(serde_json::json!({"id":id,"kind":"Beam","points":p,"support_open":true,"curves":[]})).unwrap()
        };
        let all = vec![
            c(1, vec![[0., 0.], [100., 0.]]),
            c(2, vec![[110., 0.], [200., 0.]]),
            c(3, vec![[100., 0.], [100., 100.]]),
        ];
        let h = BTreeMap::from([(1, vec![0.]), (2, vec![0.]), (3, vec![0.])]);
        let r = nearby(&all, &h, 100., 20.).unwrap();
        assert!(r.iter().any(|i| i.ids == vec![1, 2]));
        assert!(!r.iter().any(|i| i.ids == vec![1, 3]));
        assert!(nearby(&all, &h, 100., 10.).unwrap().is_empty());
        let h = BTreeMap::from([(1, vec![0.]), (2, vec![3000.]), (3, vec![0.])]);
        assert!(nearby(&all, &h, 100., 20.).unwrap().is_empty());
    }
}
