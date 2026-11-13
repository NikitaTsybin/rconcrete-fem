//! Read-only source overlaps. Work on a snapshot off the UI thread.
use super::*;
use mini_fem::geo::{Area, BooleanOps, LineString, MultiPolygon, Polygon};
use rstar::{RTree, RTreeObject, AABB};
use std::collections::BTreeSet;
#[path = "diagnostic_gaps.rs"]
mod gaps;
pub(super) use gaps::nearby;
#[derive(Clone)]
pub(super) struct Issue {
    pub ids: Vec<u64>,
    pub points: Vec<[f64; 3]>,
    pub message: String,
}
struct Item {
    id: u64,
    kind: Kind,
    p: [Point; 2],
    z: [f64; 2],
    bbox: AABB<[f64; 3]>,
    area: Option<MultiPolygon<f64>>,
}
#[derive(Clone)]
struct Index {
    i: usize,
    bbox: AABB<[f64; 3]>,
}
impl RTreeObject for Index {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        self.bbox
    }
}
fn polygon(p: &[Point]) -> Polygon<f64> {
    let mut p = p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
    if let Some(&a) = p.first() {
        p.push(a);
    }
    Polygon::new(LineString::from(p), vec![])
}
fn bbox(p: &[Point], z: [f64; 2]) -> AABB<[f64; 3]> {
    AABB::from_corners(
        [
            p.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
            p.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
            z[0].min(z[1]),
        ]
        .map(|v| v - 1e-5),
        [
            p.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
            p.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
            z[0].max(z[1]),
        ]
        .map(|v| v + 1e-5),
    )
}
fn overlap(a: [Point; 2], b: [Point; 2]) -> Option<[Point; 2]> {
    let d = [a[1][0] - a[0][0], a[1][1] - a[0][1]];
    let l = d[0].hypot(d[1]);
    if l < 1e-5 {
        return None;
    }
    if b.iter()
        .any(|p| ((p[0] - a[0][0]) * d[1] - (p[1] - a[0][1]) * d[0]).abs() / l > 1e-5)
    {
        return None;
    }
    let t = b.map(|p| ((p[0] - a[0][0]) * d[0] + (p[1] - a[0][1]) * d[1]) / (l * l));
    let lo = t[0].min(t[1]).max(0.);
    let hi = t[0].max(t[1]).min(1.);
    if (hi - lo) * l <= 1e-5 {
        return None;
    }
    Some([lo, hi].map(|t| [a[0][0] + t * d[0], a[0][1] + t * d[1]]))
}
pub(super) fn find(
    all: &[Contour],
    heights: &BTreeMap<u64, Vec<f64>>,
    step: f64,
) -> Result<Vec<Issue>, String> {
    let mut out = vec![];
    let mut items = vec![];
    for c in all.iter().filter(|c| {
        matches!(
            c.kind,
            Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
        )
    }) {
        let p = c.mesh_boundary(step)?;
        let h = &heights[&c.id];
        if p.is_empty() || h.is_empty() {
            continue;
        }
        let z = [h[0], *h.last().unwrap()];
        if c.kind == Kind::Slab {
            let mut area = MultiPolygon(vec![polygon(&p)]);
            for hole in all
                .iter()
                .filter(|h| h.active_hole() && h.parent == Some(c.id))
            {
                area = area.difference(&polygon(&hole.mesh_boundary(step)?));
            }
            items.push(Item {
                id: c.id,
                kind: c.kind,
                p: [[0.; 2]; 2],
                z,
                bbox: bbox(&p, z),
                area: Some(area),
            });
        } else if c.kind == Kind::Column {
            items.push(Item {
                id: c.id,
                kind: c.kind,
                p: [p[0]; 2],
                z,
                bbox: bbox(&p[..1], z),
                area: None,
            });
        } else {
            let mut path = p;
            if c.closed() {
                path.push(path[0]);
            }
            for e in path.windows(2) {
                let p = [e[0], e[1]];
                items.push(Item {
                    id: c.id,
                    kind: c.kind,
                    p,
                    z,
                    bbox: bbox(&p, z),
                    area: None,
                });
            }
        }
    }
    let tree = RTree::bulk_load(
        items
            .iter()
            .enumerate()
            .map(|(i, v)| Index { i, bbox: v.bbox })
            .collect(),
    );
    let mut seen = BTreeSet::new();
    for (i, a) in items.iter().enumerate() {
        for index in tree.locate_in_envelope_intersecting(&a.bbox) {
            if index.i <= i {
                continue;
            }
            let b = &items[index.i];
            if a.kind != b.kind {
                continue;
            }
            let ids = vec![a.id.min(b.id), a.id.max(b.id)];
            let key = (App::kind_index(a.kind), ids[0], ids[1]);
            if seen.contains(&key) {
                continue;
            }
            let lo = a.z[0].max(b.z[0]);
            let hi = a.z[1].min(b.z[1]);
            let found = if a.kind == Kind::Slab && a.id != b.id && (a.z[0] - b.z[0]).abs() < 1e-5 {
                let area = a
                    .area
                    .as_ref()
                    .unwrap()
                    .intersection(b.area.as_ref().unwrap());
                let value = area.unsigned_area();
                (value>1e-3).then(|| {let p=area.0.iter().flat_map(|p|p.exterior().0.iter().map(|p|[p.x,p.y,a.z[0]])).collect();
      (p,format!("Наложение плит {} и {}: {:.6} м², Z={:.3} мм. Вложенные участки/капители могут быть намеренными",a.id,b.id,value/1e6,a.z[0]))})
            } else if a.kind == Kind::Column
                && a.id != b.id
                && (a.p[0][0] - b.p[0][0]).hypot(a.p[0][1] - b.p[0][1]) < 1e-5
                && hi - lo > 1e-5
            {
                Some((
                    vec![[a.p[0][0], a.p[0][1], lo], [a.p[0][0], a.p[0][1], hi]],
                    format!(
                        "Совпадают колонны {} и {}: {:.3} мм по высоте",
                        a.id,
                        b.id,
                        hi - lo
                    ),
                ))
            } else if matches!(a.kind, Kind::Beam | Kind::Support) {
                overlap(a.p,b.p).and_then(|[p,q]|{
     let length=(q[0]-p[0]).hypot(q[1]-p[1]);
     if a.kind==Kind::Beam&&(a.z[0]-b.z[0]).abs()<1e-5 {Some((vec![[p[0],p[1],lo],[q[0],q[1],lo]],format!("Совпадают балки {} и {}: {:.6} мм, Z={:.3} мм",a.id,b.id,length,lo)))}
     else if a.kind==Kind::Support&&hi-lo>1e-5 {Some((vec![[p[0],p[1],lo],[q[0],q[1],lo],[q[0],q[1],hi],[p[0],p[1],hi]],format!("Наложение стен {} и {}: {:.6} м² (длина {:.3} мм, высота {:.3} мм)",a.id,b.id,length*(hi-lo)/1e6,length,hi-lo)))}else{None}
    })
            } else {
                None
            };
            if let Some((points, message)) = found {
                seen.insert(key);
                out.push(Issue {
                    ids,
                    points,
                    message,
                });
                if out.len() >= 1000 {
                    return Ok(out);
                }
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn c(id: u64, kind: &str, parent: Option<u64>, points: Vec<Point>) -> Contour {
        serde_json::from_value(serde_json::json!({"id":id,"kind":kind,"parent":parent,"points":points,"support_open":kind=="Beam"||kind=="Support","load":null,"refinement":null,"curves":[]})).unwrap()
    }
    #[test]
    fn finds_short_beam_overlap_but_not_a_shared_endpoint() {
        let all = vec![
            c(4015, "Beam", None, vec![[77880., 61000.], [77880., 60670.]]),
            c(4019, "Beam", None, vec![[77880., 60680.], [77880., 60000.]]),
            c(4020, "Beam", None, vec![[77880., 60000.], [77880., 59000.]]),
        ];
        let heights = all.iter().map(|c| (c.id, vec![38600.])).collect();
        let issues = find(&all, &heights, 600.).unwrap();
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].ids, vec![4015, 4019]);
        assert!(issues[0].message.contains("10.000000 мм"));
    }
    #[test]
    fn slabs_subtract_only_own_holes_and_skip_other_elevations() {
        let all = vec![
            c(
                1,
                "Slab",
                None,
                vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
            ),
            c(
                2,
                "Hole",
                Some(1),
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
            c(
                3,
                "Slab",
                None,
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
            c(
                4,
                "Slab",
                None,
                vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
            ),
        ];
        let heights = BTreeMap::from([
            (1, vec![0.]),
            (2, vec![0.]),
            (3, vec![0.]),
            (4, vec![3000.]),
        ]);
        assert!(find(&all, &heights, 500.).unwrap().is_empty());
        let mut heights = heights;
        heights.insert(4, vec![0.]);
        let issues = find(&all, &heights, 500.).unwrap();
        assert!(issues.iter().any(|i| i.ids == vec![1, 4]));
    }
    #[test]
    fn walls_require_collinear_plan_and_overlapping_vertical_ranges() {
        let all = vec![
            c(1, "Support", None, vec![[0., 0.], [1000., 0.]]),
            c(2, "Support", None, vec![[500., 0.], [1500., 0.]]),
            c(3, "Support", None, vec![[750., -1000.], [750., 1000.]]),
        ];
        let heights = BTreeMap::from([
            (1, vec![0., 3000.]),
            (2, vec![2000., 5000.]),
            (3, vec![0., 3000.]),
        ]);
        let issues = find(&all, &heights, 500.).unwrap();
        assert_eq!(issues.len(), 1);
        assert_eq!(issues[0].ids, vec![1, 2]);
        assert!(issues[0].message.contains("0.500000 м²"));
    }
}
