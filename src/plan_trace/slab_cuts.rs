//! Linked subtraction contours; geometry is regenerated from the owning slab.
use super::*;
use mini_fem::geo::{Area, BooleanOps, Contains, LineString, Polygon};
#[derive(Default)]
pub(super) struct State {
    input: Vec<(Contour, u64, u64)>,
    generated: Vec<Contour>,
    size: f64,
}
fn poly(p: &[Point]) -> Polygon<f64> {
    let mut p = p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
    if let Some(&a) = p.first() {
        p.push(a);
    }
    Polygon::new(LineString::from(p), vec![])
}
impl App {
    pub(super) fn sync_slab_cuts(&mut self) {
        if !self
            .contours
            .iter()
            .any(|c| c.placement.cut_other_slabs || c.placement.cut_source.is_some())
        {
            self.slab_cuts.input.clear();
            return;
        }
        let inputs = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .map(|c| {
                (
                    c,
                    self.building.owner(c.id),
                    self.source_heights(c)[0].to_bits(),
                )
            })
            .collect::<Vec<_>>();
        if self.mesh_size == self.slab_cuts.size
            && self
                .contours
                .iter()
                .filter(|c| c.placement.cut_source.is_some())
                .eq(self.slab_cuts.generated.iter())
            && inputs.len() == self.slab_cuts.input.len()
            && inputs
                .iter()
                .zip(&self.slab_cuts.input)
                .all(|((a, f, z), (b, g, w))| *a == b && f == g && z == w)
        {
            return;
        }
        let inputs = inputs
            .into_iter()
            .map(|(c, f, z)| (c.clone(), f, z))
            .collect::<Vec<_>>();
        let old = self
            .contours
            .iter()
            .filter_map(|c| {
                c.placement.cut_source.zip(c.parent).map(|(s, p)| {
                    (
                        (s, p, c.placement.cut_part),
                        (c.id, c.placement.movement_locked),
                    )
                })
            })
            .collect::<BTreeMap<_, _>>();
        let mut holes = vec![];
        for (source, _, z) in inputs
            .iter()
            .filter(|(c, _, _)| c.placement.cut_other_slabs)
        {
            let Ok(boundary) = source.mesh_boundary(source.mesh_step(self.mesh_size)) else {
                continue;
            };
            let area = poly(&boundary);
            let size = area.unsigned_area();
            for (host, floor, hz) in inputs.iter().filter(|(c, _, h)| {
                c.id != source.id && (f64::from_bits(*h) - f64::from_bits(*z)).abs() < 1e-5
            }) {
                let Ok(boundary) = host.mesh_boundary(host.mesh_step(self.mesh_size)) else {
                    continue;
                };
                let host_area = poly(&boundary);
                let host_size = host_area.unsigned_area();
                if size > host_size + 1e-3
                    || ((size - host_size).abs() < 1e-3
                        && host.placement.cut_other_slabs
                        && source.id < host.id)
                {
                    continue;
                }
                let contained =
                    host_area.contains(&area) || area.difference(&host_area).unsigned_area() < 1e-6;
                let common = area.intersection(&host_area);
                if common.unsigned_area() < size * 0.99 || common.unsigned_area() < 1e-3 {
                    continue;
                }
                for (part, region) in common.0.iter().enumerate() {
                    let mut points = region
                        .exterior()
                        .0
                        .iter()
                        .map(|p| [p.x, p.y])
                        .collect::<Vec<_>>();
                    if points.first() == points.last() {
                        points.pop();
                    }
                    if points.len() < 3 {
                        continue;
                    }
                    let (id, movement_locked) = old
                        .get(&(source.id, host.id, part))
                        .copied()
                        .unwrap_or_else(|| {
                            let id = self.next_id;
                            self.next_id += 1;
                            (id, false)
                        });
                    let mut hole = source.clone();
                    hole.id = id;
                    hole.kind = Kind::Hole;
                    hole.parent = Some(host.id);
                    hole.name = format!("Вырез плиты {}", source.id);
                    hole.placement = Default::default();
                    hole.placement.movement_locked = movement_locked;
                    hole.placement.cut_source = Some(source.id);
                    hole.placement.cut_part = part;
                    hole.points = points;
                    hole.curves.clear();
                    // A fully contained cut must share the SAME analytic edge
                    // and sampling step as its source, not a Boolean approximation.
                    if contained {
                        hole.points = source.points.clone();
                        hole.curves = source.curves.clone();
                    }
                    hole.grid = None;
                    hole.foundation = None;
                    hole.refinement = None;
                    hole.load = None;
                    hole.support_open = false;
                    holes.push((hole, *floor));
                }
            }
        }
        self.contours.retain(|c| c.placement.cut_source.is_none());
        let ids = self
            .contours
            .iter()
            .map(|c| c.id)
            .collect::<std::collections::BTreeSet<_>>();
        self.building
            .contour_floors
            .retain(|id, _| ids.contains(id));
        for (hole, floor) in holes {
            self.building.contour_floors.insert(hole.id, floor);
            self.contours.push(hole);
        }
        self.slab_cuts.generated = self
            .contours
            .iter()
            .filter(|c| c.placement.cut_source.is_some())
            .cloned()
            .collect();
        self.slab_cuts.input = inputs;
        self.slab_cuts.size = self.mesh_size;
        self.slab_cache = None;
        self.spatial.cache_reset();
        self.invalidate_scene_buffers();
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn curved_capital_and_linked_hole_share_analytic_edges_and_mesh_step() {
        let mut a = App::default();
        let c = |id, p| {
            serde_json::from_value(serde_json::json!({"id":id,"kind":"Slab","points":p,"support_open":false,"curves":[]})).unwrap()
        };
        a.contours = vec![
            c(1, vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]]),
            c(
                2,
                vec![
                    [1100., 1500.],
                    [1500., 1900.],
                    [1900., 1500.],
                    [1500., 1100.],
                ],
            ),
        ];
        a.contours[1].curves = vec![
            model::Curve::Arc([[1100., 1500.], [1500., 1900.], [1900., 1500.]]),
            model::Curve::Arc([[1900., 1500.], [1500., 1100.], [1100., 1500.]]),
        ];
        a.contours[1].mesh_size_mm = 100.;
        a.contours[1].placement.cut_other_slabs = true;
        a.contours[1].rebuild().unwrap();
        a.next_id = 3;
        a.sync_slab_cuts();
        assert_eq!(a.contours.len(), 3);
        assert_eq!(a.contours[2].curves, a.contours[1].curves);
        assert_eq!(
            a.contours[2].mesh_boundary(100.).unwrap(),
            a.contours[1].mesh_boundary(100.).unwrap()
        );
        let mesh = super::super::super::meshing::generate(&a.contours, 1000.).unwrap();
        let area: f64 = mesh
            .elements
            .iter()
            .map(|e| {
                let p = e.nodes.map(|n| mesh.nodes[n].xy);
                ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                    .abs()
                    / 2.
            })
            .sum();
        assert!((area - 16e6).abs() < 0.1);
    }
    #[test]
    fn linked_cuts_move_with_capital_and_do_not_affect_other_storeys() {
        let mut a = App::default();
        let c = |id, p| {
            serde_json::from_value(serde_json::json!({"id":id,"kind":"Slab","points":p,"support_open":false,"curves":[]})).unwrap()
        };
        a.contours = vec![
            c(1, vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]]),
            c(
                2,
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
        ];
        let mut upper = a.contours[0].clone();
        upper.id = 3;
        a.contours.push(upper);
        a.building.enabled = true;
        a.building.floors.push(building::Floor {
            id: 2,
            name: "Верхний".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        a.building.contour_floors = BTreeMap::from([(1, 1), (2, 1), (3, 2)]);
        a.next_id = 4;
        a.contours[1].placement.cut_other_slabs = true;
        a.sync_slab_cuts();
        assert_eq!(a.contours.len(), 4);
        let id = a.contours[3].id;
        assert_eq!(a.contours[3].parent, Some(1));
        assert_eq!(a.contours[3].placement.cut_source, Some(2));
        let mesh = super::super::super::building_mesh::generate(
            &a.contours,
            1000.,
            a.building_layout().unwrap(),
            Default::default(),
        )
        .unwrap();
        let areas = mesh
            .elements
            .iter()
            .zip(&mesh.building.as_ref().unwrap().element_slabs)
            .fold(BTreeMap::<u64, f64>::new(), |mut areas, (e, id)| {
                let p = e.nodes.map(|n| mesh.nodes[n].xy);
                *areas.entry(*id).or_default() += ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                    .abs()
                    / 2.;
                areas
            });
        assert!((areas[&1] - 15e6).abs() < 0.1);
        assert!((areas[&2] - 1e6).abs() < 0.1);
        assert!((areas[&3] - 16e6).abs() < 0.1);
        a.contours[1].translate([100., 0.]).unwrap();
        a.sync_slab_cuts();
        assert_eq!(a.contours[3].id, id);
        assert!(a.contours[3].points.iter().all(|p| p[0] >= 1100.));
        a.contours[1].placement.cut_other_slabs = false;
        a.sync_slab_cuts();
        assert_eq!(a.contours.len(), 3);
    }
}
