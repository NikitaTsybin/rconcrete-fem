//! Physical attachment stations for explicit rigid-link contours.
use super::{
    building_mesh::{Layout, WELD_TOLERANCE_MM},
    geometry::Point,
    model::{Contour, Kind},
};
use mini_fem::geo::{
    line_intersection::{line_intersection, LineIntersection},
    BooleanOps, Intersects, Line, LineString, MultiPolygon, Polygon,
};
use rstar::{RTree, RTreeObject, AABB};

const EPS: f64 = WELD_TOLERANCE_MM;

#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub(super) struct Attachment {
    pub owner: u64,
    pub xyz: [f64; 3],
}

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(super) struct Station {
    pub contour: u64,
    pub master: Attachment,
    pub slaves: Vec<Attachment>,
}

#[derive(Clone, Copy)]
struct Track {
    owner: u64,
    origin: [f64; 3],
    direction: [f64; 3],
}

impl Track {
    fn at(self, s: f64) -> Attachment {
        Attachment {
            owner: self.owner,
            xyz: std::array::from_fn(|i| self.origin[i] + s * self.direction[i]),
        }
    }
}

#[derive(Clone, Copy)]
struct Relation {
    master: Track,
    slave: Track,
    span: [f64; 2],
    step: f64,
}

#[derive(Clone)]
struct PhysicalContact([[f64; 3]; 2]);

impl RTreeObject for PhysicalContact {
    type Envelope = AABB<[f64; 3]>;

    fn envelope(&self) -> Self::Envelope {
        let [a, b] = self.0;
        AABB::from_corners(
            std::array::from_fn(|i| a[i].min(b[i])),
            std::array::from_fn(|i| a[i].max(b[i])),
        )
    }
}

fn vertical_contact_levels(
    track: Track,
    span: [f64; 2],
    contacts: &RTree<PhysicalContact>,
    levels: &mut Vec<f64>,
) {
    let [x, y, _] = track.origin;
    let query = AABB::from_corners([x - EPS, y - EPS, span[0]], [x + EPS, y + EPS, span[1]]);
    let in_span = |z: f64| z > span[0] + EPS && z < span[1] - EPS;
    for contact in contacts.locate_in_envelope_intersecting(&query) {
        let [a, b] = contact.0;
        let xy = [[a[0], a[1]], [b[0], b[1]]];
        if same_point(xy[0], xy[1]) && same_point([x, y], xy[0]) {
            levels.extend([a[2], b[2]].into_iter().filter(|&z| in_span(z)));
        } else if super::meshing::on_segment([x, y], xy) {
            let delta = [b[0] - a[0], b[1] - a[1]];
            let t = ((x - a[0]) * delta[0] + (y - a[1]) * delta[1])
                / (delta[0].powi(2) + delta[1].powi(2));
            let z = a[2] + t.clamp(0., 1.) * (b[2] - a[2]);
            if in_span(z) {
                levels.push(z);
            }
        }
    }
}

struct Path<'a> {
    contour: &'a Contour,
    extent: [f64; 2],
    points: Vec<Point>,
}

struct Slab<'a> {
    contour: &'a Contour,
    z: f64,
    domain: MultiPolygon<f64>,
}

fn polygon(points: &[Point]) -> Polygon<f64> {
    Polygon::new(
        LineString::from(
            points
                .iter()
                .chain(points.first())
                .map(|p| (p[0], p[1]))
                .collect::<Vec<_>>(),
        ),
        vec![],
    )
}

fn same_point<const D: usize>(a: [f64; D], b: [f64; D]) -> bool {
    a.into_iter().zip(b).all(|(a, b)| (a - b).abs() < EPS)
}

fn project(p: Point, origin: Point, axis: Point) -> f64 {
    (p[0] - origin[0]) * axis[0] + (p[1] - origin[1]) * axis[1]
}

fn point_on_wall(point: Point, wall: &Path<'_>) -> Option<Point> {
    wall.points.windows(2).find_map(|edge| {
        if let Some(&end) = edge.iter().find(|&&end| same_point(point, end)) {
            return Some(end);
        }
        if !super::meshing::on_segment(point, [edge[0], edge[1]]) {
            return None;
        }
        let delta = [edge[1][0] - edge[0][0], edge[1][1] - edge[0][1]];
        let length = delta[0].hypot(delta[1]);
        let axis = delta.map(|d| d / length);
        let s = project(point, edge[0], axis).clamp(0., length);
        Some(std::array::from_fn(|i| edge[0][i] + s * axis[i]))
    })
}

fn vertical_between(
    points: [Point; 2],
    extent: [f64; 2],
    master: &Path<'_>,
    wall: &Path<'_>,
    step: f64,
) -> Option<Relation> {
    let span = [
        extent[0].max(master.extent[0]).max(wall.extent[0]),
        extent[1].min(master.extent[1]).min(wall.extent[1]),
    ];
    if master.contour.id == wall.contour.id || span[1] - span[0] <= EPS {
        return None;
    }
    let start = if master.contour.kind == Kind::Column {
        *master
            .points
            .first()
            .filter(|&&p| same_point(p, points[0]))?
    } else {
        point_on_wall(points[0], master)?
    };
    let end = point_on_wall(points[1], wall)?;
    if same_point(start, end) {
        return None;
    }
    let vertical = |owner, p: Point| Track {
        owner,
        origin: [p[0], p[1], 0.],
        direction: [0., 0., 1.],
    };
    Some(Relation {
        master: vertical(master.contour.id, start),
        slave: vertical(wall.contour.id, end),
        span,
        step: step
            .min(master.contour.mesh_step(step))
            .min(wall.contour.mesh_step(step)),
    })
}

fn vertical_relations(
    points: [Point; 2],
    extent: [f64; 2],
    masters: &[Path<'_>],
    walls: &[Path<'_>],
    step: f64,
) -> Vec<Relation> {
    masters
        .iter()
        .flat_map(|master| {
            walls
                .iter()
                .filter_map(move |wall| vertical_between(points, extent, master, wall, step))
        })
        .collect()
}

fn slab_spans(track: Track, span: [f64; 2], slab: &Slab<'_>) -> Vec<[f64; 2]> {
    let [p, q] = span.map(|s| track.at(s).xyz);
    let line = Line::new((p[0], p[1]), (q[0], q[1]));
    let mut ends = span.to_vec();
    let axis = if track.direction[0].abs() >= track.direction[1].abs() {
        0
    } else {
        1
    };
    let mut add = |p: mini_fem::geo::Coord<f64>| {
        let s = ([p.x, p.y][axis] - track.origin[axis]) / track.direction[axis];
        ends.push(s.clamp(span[0], span[1]));
    };
    for p in &slab.domain.0 {
        for ring in std::iter::once(p.exterior()).chain(p.interiors()) {
            for e in ring.0.windows(2) {
                match line_intersection(line, Line::new(e[0], e[1])) {
                    Some(LineIntersection::SinglePoint { intersection, .. }) => add(intersection),
                    Some(LineIntersection::Collinear { intersection }) => {
                        add(intersection.start);
                        add(intersection.end);
                    }
                    None => {}
                }
            }
        }
    }
    ends.sort_by(f64::total_cmp);
    ends.dedup_by(|a, b| (*a - *b).abs() < EPS);
    ends.windows(2)
        .filter_map(|w| {
            let p = track.at((w[0] + w[1]) * 0.5).xyz;
            (w[1] - w[0] > EPS
                && slab
                    .domain
                    .intersects(&mini_fem::geo::Point::new(p[0], p[1])))
            .then_some([w[0], w[1]])
        })
        .collect()
}

fn beam_slab(
    points: [Point; 2],
    length: f64,
    extent: [f64; 2],
    beams: &[Path<'_>],
    slabs: &[Slab<'_>],
    step: f64,
) -> Vec<Relation> {
    let origin = points[0];
    let axis = std::array::from_fn(|i| (points[1][i] - origin[i]) / length);
    let normal = [-axis[1], axis[0]];
    let within = |z| z >= extent[0] - EPS && z <= extent[1] + EPS;
    let mut relations = vec![];
    for b in beams.iter().filter(|b| within(b.extent[0])) {
        for e in b.points.windows(2) {
            if e.iter().any(|&p| project(p, origin, normal).abs() >= EPS) {
                continue;
            }
            let [a, c] = [e[0], e[1]].map(|p| project(p, origin, axis));
            let span = [a.min(c).max(0.), a.max(c).min(length)];
            if span[1] - span[0] <= EPS {
                continue;
            }
            let direction = [
                (e[1][0] - e[0][0]) / (c - a),
                (e[1][1] - e[0][1]) / (c - a),
                0.,
            ];
            let master = Track {
                owner: b.contour.id,
                origin: [
                    e[0][0] - a * direction[0],
                    e[0][1] - a * direction[1],
                    b.extent[0],
                ],
                direction,
            };
            for slab in slabs
                .iter()
                .filter(|s| within(s.z) && (s.z - b.extent[0]).abs() > EPS)
            {
                let slave = Track {
                    owner: slab.contour.id,
                    origin: [master.origin[0], master.origin[1], slab.z],
                    direction,
                };
                relations.extend(slab_spans(slave, span, slab).into_iter().map(|span| {
                    Relation {
                        master,
                        slave,
                        span,
                        step: step
                            .min(b.contour.mesh_step(step))
                            .min(slab.contour.mesh_step(step)),
                    }
                }));
            }
        }
    }
    relations
}

fn sample(
    contour: u64,
    relations: &[Relation],
    contacts: &RTree<PhysicalContact>,
    stations: &mut Vec<Station>,
    attachment_count: &mut usize,
) -> Result<(), String> {
    let error = |reason| format!("АЖТ {contour}: {reason}");
    let mut ends = relations.iter().flat_map(|r| r.span).collect::<Vec<_>>();
    // Use the same physical levels at both ends of a vertical link before filling
    // the remaining intervals. Sampling its full height independently leaves a
    // second near-coincident node whenever an existing contact falls near a station.
    for relation in relations.iter().filter(|r| r.master.direction[2] != 0.) {
        for track in [relation.master, relation.slave] {
            vertical_contact_levels(track, relation.span, contacts, &mut ends);
        }
    }
    ends.sort_by(f64::total_cmp);
    ends.dedup_by(|a, b| (*a - *b).abs() < EPS);
    let covers = |r: &Relation, s| s >= r.span[0] - EPS && s <= r.span[1] + EPS;
    let mut append = |s, active: &[&Relation]| -> Result<(), String> {
        let Some(first) = active.first() else {
            return Ok(());
        };
        let master = first.master.at(s);
        if active
            .iter()
            .any(|r| !same_point(r.master.at(s).xyz, master.xyz))
        {
            return Err(error("на одной станции найдены разные ведущие узлы"));
        }
        let mut slaves: Vec<Attachment> = vec![];
        for r in active {
            let slave = r.slave.at(s);
            if !same_point(slave.xyz, master.xyz)
                && !slaves.iter().any(|p| same_point(p.xyz, slave.xyz))
            {
                slaves.push(slave);
            }
        }
        if !slaves.is_empty() {
            *attachment_count = attachment_count.saturating_add(1 + slaves.len());
            if *attachment_count > super::limits::MAX_NODES {
                return Err(error("превышен лимит узлов станций"));
            }
            stations.push(Station {
                contour,
                master,
                slaves,
            });
        }
        Ok(())
    };
    for (i, &a) in ends.iter().enumerate() {
        let at_end = relations
            .iter()
            .filter(|r| covers(r, a))
            .collect::<Vec<_>>();
        append(a, &at_end)?;
        let Some(&b) = ends.get(i + 1) else { break };
        let middle = (a + b) * 0.5;
        let active = relations
            .iter()
            .filter(|r| covers(r, middle))
            .collect::<Vec<_>>();
        let Some(step) = active.iter().map(|r| r.step).min_by(f64::total_cmp) else {
            continue;
        };
        if !step.is_finite() || step <= 0. {
            return Err(error("некорректный шаг станций"));
        }
        let count = ((b - a - EPS) / step).ceil().max(1.) as usize;
        if count > super::limits::MAX_NODES {
            return Err(error("превышен лимит станций"));
        }
        for k in 1..count {
            append(a + (b - a) * k as f64 / count as f64, &active)?;
        }
    }
    Ok(())
}

pub(super) fn prepare(
    all: &[Contour],
    layout: &Layout,
    size: f64,
    physical_contacts: &[[[f64; 3]; 2]],
) -> Result<Vec<Station>, String> {
    if !all.iter().any(|c| c.kind == Kind::RigidLink) {
        return Ok(vec![]);
    }
    layout.validate()?;
    if !size.is_finite() || size < 100. {
        return Err("Размер КЭ должен быть не менее 100 мм".into());
    }
    let mut columns = vec![];
    let mut walls = vec![];
    let mut beams = vec![];
    let mut slabs = vec![];
    for c in all {
        if layout.owner(c.id) == 0 {
            return Err(format!("Не назначен этаж объекта {}", c.id));
        }
        // This relation maps a horizontal beam to another level. A free
        // spatial axis is connected by physical contacts in the mesh.
        if c.is_free_beam() {
            continue;
        }
        match c.kind {
            Kind::Column | Kind::Support | Kind::Beam => {
                let mut points = c.mesh_boundary(c.mesh_step(size))?;
                if c.closed() && !points.is_empty() {
                    points.push(points[0]);
                }
                let path = Path {
                    contour: c,
                    extent: layout.extent(c),
                    points,
                };
                match c.kind {
                    Kind::Column => columns.push(path),
                    Kind::Support => walls.push(path),
                    _ => beams.push(path),
                }
            }
            Kind::Slab => {
                let mut domain = MultiPolygon(vec![polygon(&c.mesh_boundary(c.mesh_step(size))?)]);
                for h in all
                    .iter()
                    .filter(|h| h.active_hole() && h.parent == Some(c.id))
                {
                    domain = domain.difference(&polygon(&h.mesh_boundary(size)?));
                }
                slabs.push(Slab {
                    contour: c,
                    z: layout.extent(c)[0],
                    domain,
                });
            }
            _ => {}
        }
    }
    let mut stations = vec![];
    let mut attachment_count = 0;
    let contacts = RTree::bulk_load(
        physical_contacts
            .iter()
            .copied()
            .map(PhysicalContact)
            .collect(),
    );
    for c in all.iter().filter(|c| c.kind == Kind::RigidLink) {
        let [a, b] = c.points.as_slice() else {
            return Err(format!("АЖТ {}: нужны две точки", c.id));
        };
        let length = (b[0] - a[0]).hypot(b[1] - a[1]);
        let extent = layout.extent(c);
        if !length.is_finite()
            || length <= EPS
            || extent.iter().any(|z| !z.is_finite())
            || extent[1] - extent[0] <= EPS
        {
            return Err(format!("АЖТ {}: проверьте длину и отметки", c.id));
        }
        let step = c.mesh_step(size).min(size);
        let points = [*a, *b];
        if let Some([column, wall]) = c.placement.rigid_link_targets {
            let master = columns.iter().find(|path| path.contour.id == column);
            let slave = walls.iter().find(|path| path.contour.id == wall);
            let (Some(master), Some(slave)) = (master, slave) else {
                return Err(format!(
                    "АЖТ {}: в сетку должны входить колонна {column} и стена {wall}",
                    c.id
                ));
            };
            let relation = vertical_between(points, extent, master, slave, step).ok_or_else(|| format!(
                "АЖТ {}: проверьте точки привязки и общий диапазон высот колонны {column} и стены {wall}", c.id
            ))?;
            sample(
                c.id,
                &[relation],
                &contacts,
                &mut stations,
                &mut attachment_count,
            )?;
            continue;
        }
        let mut vertical = vertical_relations(points, extent, &columns, &walls, step);
        vertical.extend(vertical_relations([*b, *a], extent, &columns, &walls, step));
        if vertical.is_empty() {
            vertical = vertical_relations(points, extent, &walls, &walls, step);
        }
        let longitudinal = beam_slab([*a, *b], length, extent, &beams, &slabs, step);
        let relations = match (vertical.is_empty(), longitudinal.is_empty()) {
            (false, true) => &vertical,
            (true, false) => &longitudinal,
            (true, true) => {
                return Err(format!(
                "АЖТ {}: не найдена пара колонна–стена, стена–стена или балка–плита в заданных границах",
                c.id
            ))
            }
            (false, false) => {
                return Err(format!(
                    "АЖТ {}: найдены оба вида связей, уточните границы",
                    c.id
                ))
            }
        };
        sample(
            c.id,
            relations,
            &contacts,
            &mut stations,
            &mut attachment_count,
        )?;
    }
    Ok(stations)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{building_mesh::Level, model::LevelAnchor};

    fn contour(id: u64, kind: Kind, points: Vec<Point>) -> Contour {
        Contour {
            id,
            kind,
            points,
            support_open: true,
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            parent: None,
            curves: vec![],
            grid: None,
            refinement: None,
            load: None,
        }
    }

    fn layout(all: &[Contour]) -> Layout {
        Layout {
            levels: vec![Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: all.iter().map(|c| (c.id, 1)).collect(),
            legacy_wall_extent: None,
        }
    }

    #[test]
    fn vertical_stations_are_clipped_to_the_common_height() {
        let mut column = contour(1, Kind::Column, vec![[1000., 0.]]);
        column.placement.bottom.offset_mm = 1000.;
        column.mesh_size_mm = 600.;
        let mut wall = contour(2, Kind::Support, vec![[0., -2000.], [0., 2000.]]);
        wall.placement.top.anchor = LevelAnchor::Bottom;
        wall.placement.top.offset_mm = 2200.;
        wall.mesh_size_mm = 600.;
        let mut link = contour(3, Kind::RigidLink, vec![[0., 0.], [1000., 0.]]);
        link.mesh_size_mm = 400.;
        let all = [column, wall, link];
        let stations = prepare(&all, &layout(&all), 1000., &[]).unwrap();
        assert_eq!(stations.len(), 4);
        for (station, z) in stations.iter().zip([1000., 1400., 1800., 2200.]) {
            assert_eq!(station.master.xyz, [1000., 0., z]);
            assert_eq!(station.master.owner, 1);
            assert_eq!(station.slaves[0].xyz, [0., 0., z]);
            assert_eq!(station.slaves[0].owner, 2);
        }
    }

    #[test]
    fn wall_wall_stations_use_contact_levels_at_both_ends() {
        let first = contour(1, Kind::Support, vec![[0., -1000.], [0., 1000.]]);
        let second = contour(2, Kind::Support, vec![[1000., -1000.], [1000., 1000.]]);
        let link = contour(3, Kind::RigidLink, vec![[0., 0.], [1000., 0.]]);
        let all = [first, second, link];
        let contacts = [
            [[0., -500., 999.], [0., 500., 999.]],
            [[1000., 0., 0.], [1000., 0., 2001.]],
        ];
        let stations = prepare(&all, &layout(&all), 1000., &contacts).unwrap();
        let levels = stations.iter().map(|s| s.master.xyz[2]).collect::<Vec<_>>();
        assert_eq!(levels, [0., 999., 1500., 2001., 3000.]);
        for (s, z) in stations.iter().zip(levels) {
            assert_eq!((s.master.owner, s.master.xyz), (1, [0., 0., z]));
            assert_eq!((s.slaves[0].owner, s.slaves[0].xyz), (2, [1000., 0., z]));
        }
    }

    #[test]
    fn bound_link_uses_only_its_named_column_and_wall() {
        let column = contour(1, Kind::Column, vec![[0., 0.]]);
        let first = contour(2, Kind::Support, vec![[1000., -1000.], [1000., 1000.]]);
        let target = contour(3, Kind::Support, vec![[500., 0.], [1500., 0.]]);
        let mut link = contour(4, Kind::RigidLink, vec![[0., 0.], [1000., 0.]]);
        link.placement.rigid_link_targets = Some([1, 3]);
        let all = [column, first, target, link];
        let stations = prepare(&all, &layout(&all), 1000., &[]).unwrap();
        assert!(stations
            .iter()
            .all(|s| s.master.owner == 1 && s.slaves[0].owner == 3));
    }

    #[test]
    fn beam_stations_match_offset_slab_and_leave_its_opening_empty() {
        let beam = contour(1, Kind::Beam, vec![[0., 0.], [4000., 0.]]);
        let mut slab = contour(
            2,
            Kind::Slab,
            vec![[0., -1000.], [4000., -1000.], [4000., 1000.], [0., 1000.]],
        );
        slab.placement.slab.offset_mm = 300.;
        let mut hole = contour(
            3,
            Kind::Hole,
            vec![[1000., -500.], [3000., -500.], [3000., 500.], [1000., 500.]],
        );
        hole.parent = Some(2);
        let link = contour(4, Kind::RigidLink, vec![[0., 0.], [4000., 0.]]);
        let all = [beam, slab, hole, link];
        let stations = prepare(&all, &layout(&all), 500., &[]).unwrap();
        assert_eq!(stations.len(), 6);
        for (station, x) in stations.iter().zip([0., 500., 1000., 3000., 3500., 4000.]) {
            assert!(same_point(station.master.xyz, [x, 0., 0.]));
            assert_eq!(station.master.owner, 1);
            assert!(same_point(station.slaves[0].xyz, [x, 0., 300.]));
            assert_eq!(station.slaves[0].owner, 2);
        }
    }
}
