use super::{geometry::Point, import::Drawing};
use std::collections::{HashMap, HashSet};
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Node,
    Line,
    Angle,
}
#[derive(Clone, Copy, Debug)]
pub struct Snap {
    pub point: Point,
    pub kind: Kind,
}
pub(super) fn axis_intersection(
    anchor: Point,
    direction: Point,
    [a, b]: [Point; 2],
) -> Option<Point> {
    let side = |p: Point| direction[0] * (p[1] - anchor[1]) - direction[1] * (p[0] - anchor[0]);
    let a_side = side(a);
    let b_side = side(b);
    let denominator = a_side - b_side;
    if denominator.abs() <= 1e-10 {
        return None;
    }
    let fraction = a_side / denominator;
    (0. ..=1.).contains(&fraction).then(|| {
        [
            a[0] + fraction * (b[0] - a[0]),
            a[1] + fraction * (b[1] - a[1]),
        ]
    })
}
#[derive(Default)]
pub struct Index {
    vertices: Vec<Point>,
    path_tags: Vec<(usize, usize)>,
    edges: Vec<[Point; 2]>,
    points: HashMap<(i64, i64), Vec<usize>>,
    lines: HashMap<(i64, i64), Vec<usize>>,
    large: Vec<usize>,
    cell: f64,
}
impl Index {
    fn key(&self, p: Point) -> (i64, i64) {
        (
            (p[0] / self.cell.max(1.)).floor() as i64,
            (p[1] / self.cell.max(1.)).floor() as i64,
        )
    }
    #[cfg(test)]
    pub fn new(d: &Drawing) -> Self {
        Self::visible(d, |_| true)
    }
    pub fn visible(d: &Drawing, visible: impl Fn(&str) -> bool) -> Self {
        let b = d.bounds().unwrap_or([0., 0., 100., 100.]);
        let mut s = Self {
            cell: ((b[2] - b[0]).max(b[3] - b[1]) / 128.).max(1.),
            ..Default::default()
        };
        let mut unique = HashSet::new();
        for (node, &p) in d.nodes.iter().enumerate() {
            if !visible(d.node_layers.get(node).map_or("0", String::as_str)) {
                continue;
            }
            if unique.insert(((p[0] * 1e6).round() as i64, (p[1] * 1e6).round() as i64)) {
                let i = s.vertices.len();
                s.vertices.push(p);
                s.points.entry(s.key(p)).or_default().push(i);
            }
        }
        for (path_index, path) in d
            .paths
            .iter()
            .enumerate()
            .filter(|(_, p)| visible(&p.layer))
        {
            let n = if path.closed {
                path.points.len()
            } else {
                path.points.len().saturating_sub(1)
            };
            for i in 0..n {
                let a = path.points[i];
                let b = path.points[(i + 1) % path.points.len()];
                let j = s.edges.len();
                s.edges.push([a, b]);
                s.path_tags.push((path_index, i));
                let lo = s.key([a[0].min(b[0]), a[1].min(b[1])]);
                let hi = s.key([a[0].max(b[0]), a[1].max(b[1])]);
                if (hi.0 as i128 - lo.0 as i128 + 1) * (hi.1 as i128 - lo.1 as i128 + 1) > 4096 {
                    s.large.push(j);
                } else {
                    for x in lo.0..=hi.0 {
                        for y in lo.1..=hi.1 {
                            s.lines.entry((x, y)).or_default().push(j);
                        }
                    }
                }
            }
        }
        s
    }
    pub fn path_hit(&self, p: Point, zoom: f64) -> Option<(usize, usize, f64)> {
        let mut ids = self.nearby(p, 10. / zoom, &self.lines, self.edges.len());
        ids.extend(&self.large);
        ids.into_iter()
            .filter_map(|i| {
                let &(path, segment) = self.path_tags.get(i)?;
                let [a, b] = self.edges[i];
                let d = [b[0] - a[0], b[1] - a[1]];
                let len = d[0] * d[0] + d[1] * d[1];
                let t = if len > 0. {
                    (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / len).clamp(0., 1.)
                } else {
                    0.
                };
                let distance = (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]);
                (distance * zoom <= 10.).then_some((path, segment, distance))
            })
            .min_by(|a, b| a.2.total_cmp(&b.2))
    }
    pub fn mesh(mesh: &super::meshing::Mesh) -> Self {
        Self::mesh_floor(mesh, None)
    }
    pub fn mesh_floor(mesh: &super::meshing::Mesh, floor: Option<u64>) -> Self {
        let visible = |n: usize| {
            floor.is_none_or(|f| mesh.building.as_ref().is_none_or(|b| b.node_floors[n] == f))
        };
        let mut s = Self {
            vertices: mesh
                .nodes
                .iter()
                .enumerate()
                .filter(|(i, _)| visible(*i))
                .map(|(_, n)| n.xy)
                .collect(),
            edges: mesh
                .visible_edges()
                .iter()
                .filter(|e| visible(e[0]) && visible(e[1]))
                .map(|e| e.map(|n| mesh.nodes[n].xy))
                .collect(),
            ..Default::default()
        };
        let b = s.vertices.iter().fold(
            [
                f64::INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::NEG_INFINITY,
            ],
            |b, p| {
                [
                    b[0].min(p[0]),
                    b[1].min(p[1]),
                    b[2].max(p[0]),
                    b[3].max(p[1]),
                ]
            },
        );
        s.cell = ((b[2] - b[0]).max(b[3] - b[1]) / 128.).max(1.);
        for i in 0..s.vertices.len() {
            s.points.entry(s.key(s.vertices[i])).or_default().push(i);
        }
        for j in 0..s.edges.len() {
            let [a, b] = s.edges[j];
            let lo = s.key([a[0].min(b[0]), a[1].min(b[1])]);
            let hi = s.key([a[0].max(b[0]), a[1].max(b[1])]);
            if (hi.0 as i128 - lo.0 as i128 + 1) * (hi.1 as i128 - lo.1 as i128 + 1) > 4096 {
                s.large.push(j);
            } else {
                for x in lo.0..=hi.0 {
                    for y in lo.1..=hi.1 {
                        s.lines.entry((x, y)).or_default().push(j);
                    }
                }
            }
        }
        s
    }
    fn nearby(
        &self,
        p: Point,
        r: f64,
        buckets: &HashMap<(i64, i64), Vec<usize>>,
        count: usize,
    ) -> Vec<usize> {
        let lo = self.key([p[0] - r, p[1] - r]);
        let hi = self.key([p[0] + r, p[1] + r]);
        if (hi.0 as i128 - lo.0 as i128 + 1) * (hi.1 as i128 - lo.1 as i128 + 1) > 4096 {
            return (0..count).collect();
        }
        let mut found = HashSet::new();
        for x in lo.0..=hi.0 {
            for y in lo.1..=hi.1 {
                if let Some(ids) = buckets.get(&(x, y)) {
                    found.extend(ids);
                }
            }
        }
        let mut ids = found.into_iter().collect::<Vec<_>>();
        ids.sort_unstable();
        ids
    }
    pub(super) fn nearby_edges(
        &self,
        p: Point,
        radius: f64,
    ) -> impl Iterator<Item = [Point; 2]> + '_ {
        let mut ids = self.nearby(p, radius, &self.lines, self.edges.len());
        ids.extend(&self.large);
        ids.into_iter().map(|i| self.edges[i])
    }
    #[cfg(test)]
    pub fn query(
        &self,
        p: Point,
        scale: f64,
        extra_points: &[Point],
        extra_edges: &[[Point; 2]],
    ) -> Option<Snap> {
        self.query_modes(p, scale, extra_points, extra_edges, true, true)
    }
    pub fn query_modes(
        &self,
        p: Point,
        scale: f64,
        extra_points: &[Point],
        extra_edges: &[[Point; 2]],
        nodes: bool,
        nearest: bool,
    ) -> Option<Snap> {
        let scale = scale.max(1e-12);
        let distance = |q: Point| (p[0] - q[0]).hypot(p[1] - q[1]) * scale;
        let mut node: Option<(f64, Point)> = None;
        if nodes {
            for q in self
                .nearby(p, 12. / scale, &self.points, self.vertices.len())
                .into_iter()
                .map(|i| self.vertices[i])
                .chain(extra_points.iter().copied())
            {
                let d = distance(q);
                if d <= 12. && node.is_none_or(|n| d < n.0) {
                    node = Some((d, q));
                }
            }
            if let Some((_, point)) = node {
                return Some(Snap {
                    point,
                    kind: Kind::Node,
                });
            }
        }
        if !nearest {
            return None;
        }
        let mut line: Option<(f64, Point)> = None;
        let mut ids = self.nearby(p, 8. / scale, &self.lines, self.edges.len());
        ids.extend(&self.large);
        for [a, b] in ids
            .into_iter()
            .map(|i| self.edges[i])
            .chain(extra_edges.iter().copied())
        {
            let dx = b[0] - a[0];
            let dy = b[1] - a[1];
            let l = dx * dx + dy * dy;
            if l < 1e-20 {
                continue;
            }
            let t = (((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / l).clamp(0., 1.);
            let q = [a[0] + t * dx, a[1] + t * dy];
            let d = distance(q);
            if d <= 8. && line.is_none_or(|n| d < n.0) {
                line = Some((d, q));
            }
        }
        line.map(|(_, point)| Snap {
            point,
            kind: Kind::Line,
        })
    }
}
#[cfg(test)]
mod tests {
    use super::super::{import::Path2, source_dxf::Color};
    use super::*;
    #[test]
    fn node_priority_and_line_projection_use_screen_radius() {
        let d = Drawing {
            nodes: vec![[0., 0.], [100., 0.]],
            paths: vec![Path2 {
                layer: "0".into(),
                points: vec![[0., 0.], [100., 0.]],
                closed: false,
                color: Color::Aci(1),
            }],
            ..Default::default()
        };
        let s = Index::new(&d);
        for _ in 0..50 {
            assert_eq!(s.query([50., 0.], 0.2, &[], &[]).unwrap().point, [0., 0.]);
        }
        assert_eq!(s.query([7., 2.], 1., &[], &[]).unwrap().kind, Kind::Node);
        assert_eq!(s.query([50., 4.], 1., &[], &[]).unwrap().point, [50., 0.]);
        assert!(s.query([50., 5.], 2., &[], &[]).is_none());
        assert_eq!(
            s.query([50., 4.], 1., &[[50., 10.]], &[]).unwrap().kind,
            Kind::Node
        );
    }
}
