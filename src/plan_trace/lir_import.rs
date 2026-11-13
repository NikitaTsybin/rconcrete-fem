//! Editable geometry and supported physical loads reconstructed from verified LIR topology.
use super::*;
use std::collections::BTreeSet;
use std::sync::Arc;
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(super) struct Source {
    pub name: String,
    pub nodes: usize,
    pub elements: usize,
    pub inactive: usize,
    pub inferred_types: usize,
    pub unit_mm: f64,
    pub gravity: f64,
    pub force_units: crate::plan_trace::loads::ForceUnits,
    pub contour_elements: BTreeMap<u64, Vec<u32>>,
    pub notes: Vec<String>,
    #[serde(default)]
    pub foundation_assignments: BTreeMap<u32, SoilSource>,
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub(super) struct SoilSource {
    pub c1: f64,
    pub c2: f64,
    pub rz_text: String,
}
pub(super) struct Recovered {
    pub contours: Vec<Contour>,
    pub building: building::Building,
    pub library: crate::plan_trace::stiffness::Library,
    pub source: Source,
    pub load_cases: Vec<crate::plan_trace::loads::LoadCase>,
    candidate_levels: Vec<f64>,
    #[cfg(test)]
    plate_areas: BTreeMap<i64, f64>,
}
const MIN_LIR_FLOOR_GAP: i64 = 10_000; // 100 mm in hundredths of a millimetre.

fn inclined_beam_roll(a: [f64; 3], b: [f64; 3], lira_roll: f64) -> f64 {
    let d = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
    let horizontal = d[0].hypot(d[1]);
    let length = horizontal.hypot(d[2]);
    // For this direction the LIRA and FEM zero-angle bases already agree.
    if d[2].abs() / length < 0.95 {
        return lira_roll;
    }

    // LIRA uses global Z as the reference for every nonvertical bar. The FEM
    // kernel switches to global Y near vertical, so preserve the width axis.
    let x = d.map(|v| v / length);
    let lira_y = [-d[1] / horizontal, d[0] / horizontal, 0.];
    let norm = x[2].hypot(x[0]);
    let fem_y = [x[2] / norm, 0., -x[0] / norm];
    let fem_z = [
        x[1] * fem_y[2] - x[2] * fem_y[1],
        x[2] * fem_y[0] - x[0] * fem_y[2],
        x[0] * fem_y[1] - x[1] * fem_y[0],
    ];
    let dot = |axis: [f64; 3]| (0..3).map(|i| lira_y[i] * axis[i]).sum::<f64>();
    lira_roll + dot(fem_z).atan2(dot(fem_y))
}

fn validate_floor_gap(levels: impl IntoIterator<Item = i64>) -> Result<(), String> {
    let mut levels = levels.into_iter();
    let Some(mut previous) = levels.next() else {
        return Err("Выберите хотя бы одну отметку для создания этажа".into());
    };
    for z in levels {
        if z.saturating_sub(previous) < MIN_LIR_FLOOR_GAP {
            return Err(format!(
                "Между этажами должно быть не менее 100 мм. Отметки {:.2} и {:.2} мм находятся ближе.",
                previous as f64 / 100., z as f64 / 100.
            ));
        }
        previous = z;
    }
    Ok(())
}

fn default_floor_levels(levels: &BTreeSet<i64>, plate_areas: &BTreeMap<i64, f64>) -> BTreeSet<i64> {
    let mut floors = levels.first().copied().into_iter().collect::<BTreeSet<_>>();
    let mut by_area = levels.iter().copied().skip(1).collect::<Vec<_>>();
    let area_at = |z: &i64| plate_areas.get(z).copied().unwrap_or(0.);
    by_area.sort_by(|a, b| area_at(b).total_cmp(&area_at(a)).then(a.cmp(b)));
    for z in by_area {
        let radius = MIN_LIR_FLOOR_GAP - 1;
        if floors
            .range(z.saturating_sub(radius)..=z.saturating_add(radius))
            .next()
            .is_none()
        {
            floors.insert(z);
        }
    }
    floors
}
type Key = [i64; 2];
// Merge sub-millimetre elevation noise around a dominant horizontal plane.
const SLAB_Z_NOISE_MM: f64 = 1.;

fn slab_plane(lo: f64, hi: f64, areas: &BTreeMap<i64, f64>) -> Option<i64> {
    let mid = ((lo + hi) * 50.).round() as i64;
    let min_plane = ((hi - SLAB_Z_NOISE_MM) * 100.).ceil() as i64;
    let max_plane = ((lo + SLAB_Z_NOISE_MM) * 100.).floor() as i64;
    let dominant = (min_plane <= max_plane && hi - lo <= SLAB_Z_NOISE_MM)
        .then(|| {
            areas
                .range(min_plane..=max_plane)
                .max_by(|(az, a), (bz, b)| {
                    a.total_cmp(b)
                        .then_with(|| (*bz - mid).abs().cmp(&(*az - mid).abs()))
                })
        })
        .flatten()
        .map(|(&z, _)| z);
    dominant.or_else(|| (hi - lo < 0.05).then_some(mid))
}

fn key(p: Point) -> Key {
    p.map(|v| (v * 100.).round() as i64)
}
fn point(k: Key) -> Point {
    k.map(|v| v as f64 / 100.)
}
fn area(p: &[Point]) -> f64 {
    p.iter()
        .zip(p.iter().cycle().skip(1))
        .take(p.len())
        .map(|(a, b)| a[0] * b[1] - a[1] * b[0])
        .sum::<f64>()
        * 0.5
}
/// Repeatedly remove the middle vertex of a nearly collinear neighbour triple.
/// Distances are in mm. Only forward, between-neighbours points may disappear.
#[cfg(test)]
fn simple(p: Vec<Point>) -> Vec<Point> {
    simple_with(p, 0.1)
}
fn simple_with(p: Vec<Point>, tolerance: f64) -> Vec<Point> {
    let p = model::without_collinear_spurs(p);
    use mini_fem::geo::{algorithm::Validation, LineString, Polygon};
    if p.len() <= 3 {
        return p;
    }
    let mut result = p.clone();
    loop {
        let before = result.len();
        let mut i = 0;
        while i < result.len() && result.len() > 3 {
            let n = result.len();
            let a = result[(i + n - 1) % n];
            let b = result[i];
            let c = result[(i + 1) % n];
            let d = [c[0] - a[0], c[1] - a[1]];
            let length2 = d[0] * d[0] + d[1] * d[1];
            let projection = (b[0] - a[0]) * d[0] + (b[1] - a[1]) * d[1];
            let cross = (b[0] - a[0]) * d[1] - (b[1] - a[1]) * d[0];
            let forward = (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]);
            if length2 > 1e-16
                && projection >= 0.
                && projection <= length2
                && forward >= 0.
                && cross * cross <= tolerance.powi(2) * length2
            {
                result.remove(i);
                // The new vertex at i is checked against its updated neighbours.
            } else {
                i += 1;
            }
        }
        if result.len() == before {
            break;
        }
    }
    if result.len() < 3 || area(&result) * area(&p) <= 0. {
        return p;
    }
    let mut ring = result.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
    ring.push(ring[0]);
    if Polygon::new(LineString::from(ring), vec![]).is_valid() {
        result
    } else {
        p
    }
}
fn simple_open(mut p: Vec<Point>, tolerance: f64) -> Vec<Point> {
    loop {
        let before = p.len();
        let mut i = 1;
        while i + 1 < p.len() {
            let (a, b, c) = (p[i - 1], p[i], p[i + 1]);
            let d = [c[0] - a[0], c[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            let projection = (b[0] - a[0]) * d[0] + (b[1] - a[1]) * d[1];
            let cross = (b[0] - a[0]) * d[1] - (b[1] - a[1]) * d[0];
            let forward = (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]);
            if l2 > 1e-16
                && projection >= 0.
                && projection <= l2
                && forward >= 0.
                && cross * cross <= tolerance * tolerance * l2
            {
                p.remove(i);
            } else {
                i += 1;
            }
        }
        if p.len() == before {
            return p;
        }
    }
}
/// Trace degree-two chains only. Material/level groups are supplied by caller;
/// junctions remain endpoints, so merging never removes a branch connection.
fn beam_chains(
    segments: Vec<([Point; 2], u32)>,
    tolerance: f64,
) -> Vec<(Vec<Point>, Vec<u32>, bool)> {
    let mut adjacent = BTreeMap::<Key, Vec<usize>>::new();
    let mut coordinates = BTreeMap::new();
    let edges = segments.iter().map(|(p, _)| p.map(key)).collect::<Vec<_>>();
    for (i, (ends, _)) in segments.iter().enumerate() {
        for &p in ends {
            coordinates.entry(key(p)).or_insert(p);
            adjacent.entry(key(p)).or_default().push(i);
        }
    }
    let mut used = vec![false; segments.len()];
    let mut output = vec![];
    let starts = adjacent
        .iter()
        .filter(|(_, v)| v.len() != 2)
        .flat_map(|(&p, v)| v.iter().map(move |&e| (p, e)))
        .chain(edges.iter().enumerate().map(|(i, e)| (e[0], i)))
        .collect::<Vec<_>>();
    for (start, first) in starts {
        if used[first] {
            continue;
        }
        let mut path = vec![coordinates[&start]];
        let mut ids = vec![];
        let mut at = start;
        let mut e = first;
        loop {
            used[e] = true;
            ids.push(segments[e].1);
            let next = if edges[e][0] == at {
                edges[e][1]
            } else {
                edges[e][0]
            };
            path.push(coordinates[&next]);
            if next == start || adjacent[&next].len() != 2 {
                break;
            }
            let Some(&following) = adjacent[&next].iter().find(|&&i| !used[i]) else {
                break;
            };
            at = next;
            e = following;
        }
        let closed = path.len() > 2 && path.first() == path.last();
        let path = if closed {
            path.pop();
            simple_with(path, tolerance)
        } else {
            simple_open(path, tolerance)
        };
        output.push((path, ids, closed));
    }
    output
}
fn contour(id: u64, kind: Kind, points: Vec<Point>, parent: Option<u64>) -> Contour {
    Contour {
        id,
        kind,
        points,
        parent,
        name: String::new(),
        placement: Default::default(),
        mesh_size_mm: 0.,
        column_angle: 0.,
        calculate_effective_length: false,
        restraint_mask: 4,
        use_foundation: false,
        foundation: None,
        grid: None,
        support_open: kind == Kind::Support,
        load: None,
        refinement: None,
        curves: vec![],
    }
}
fn wall_contour(id: u64, direction: Point, offset: f64, [lo, hi]: [f64; 2]) -> Option<Contour> {
    let points = [lo, hi].map(|t| {
        [
            direction[0] * t - direction[1] * offset,
            direction[1] * t + direction[0] * offset,
        ]
    });
    let wall = contour(id, Kind::Support, points.to_vec(), None);
    model::validate_candidate(&[], &wall).ok()?;
    Some(wall)
}
#[cfg(test)]
fn union_loops(cells: &[Vec<Point>]) -> Vec<Vec<Point>> {
    union_loops_with(cells, 0.1)
}
fn union_loops_with(cells: &[Vec<Point>], tolerance: f64) -> Vec<Vec<Point>> {
    union_polygons_with(cells, tolerance)
        .into_iter()
        .flat_map(|(outer, holes)| std::iter::once(outer).chain(holes))
        .collect()
}
fn union_polygons_with(cells: &[Vec<Point>], tolerance: f64) -> Vec<(Vec<Point>, Vec<Vec<Point>>)> {
    use mini_fem::geo::{
        algorithm::orient::Direction, unary_union, Coord, LineString, Orient, Polygon,
    };
    let polygons = cells
        .iter()
        .filter(|p| p.len() >= 3)
        .map(|p| {
            let mut p = p
                .iter()
                .map(|&p| point(key(p)))
                .map(|p| Coord { x: p[0], y: p[1] })
                .collect::<Vec<_>>();
            p.push(p[0]);
            Polygon::new(LineString::new(p), vec![]).orient(Direction::Default)
        })
        .collect::<Vec<_>>();
    let ring = |line: &LineString<f64>, positive| {
        let mut points = line.0.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>();
        if points.first() == points.last() {
            points.pop();
        }
        let mut points = simple_with(points, tolerance);
        let signed_area = area(&points);
        if (signed_area > 0.) != positive {
            points.reverse();
        }
        (points.len() >= 3 && signed_area.abs() > 1.).then_some(points)
    };
    unary_union(&polygons)
        .0
        .into_iter()
        .filter_map(|polygon| {
            Some((
                ring(polygon.exterior(), true)?,
                polygon
                    .interiors()
                    .iter()
                    .filter_map(|line| ring(line, false))
                    .collect(),
            ))
        })
        .collect()
}
#[derive(Default)]
struct Plate {
    ids: Vec<(u32, Point)>,
    cells: Vec<Vec<Point>>,
}
impl Plate {
    fn add(&mut self, mut p: Vec<Point>, id: u32) {
        if area(&p) < 0. {
            p.reverse();
        }
        self.ids.push((
            id,
            [
                p.iter().map(|v| v[0]).sum::<f64>() / p.len() as f64,
                p.iter().map(|v| v[1]).sum::<f64>() / p.len() as f64,
            ],
        ));
        self.cells.push(p);
    }
    #[cfg(test)]
    fn loops(&self) -> Result<Vec<Vec<Point>>, String> {
        self.loops_with(0.1)
    }
    fn loops_with(&self, tolerance: f64) -> Result<Vec<Vec<Point>>, String> {
        let mut counts = BTreeMap::<_, (usize, _)>::new();
        for p in &self.cells {
            for (a, b) in p.iter().zip(p.iter().cycle().skip(1)).take(p.len()) {
                let (a, b) = (key(*a), key(*b));
                if a == b {
                    continue;
                }
                let edge = if a < b { (a, b) } else { (b, a) };
                counts.entry(edge).or_insert((0, (a, b))).0 += 1;
            }
        }
        let mut edges = std::collections::BTreeSet::new();
        let mut next = BTreeMap::<Key, Vec<Key>>::new();
        let edge_count = counts.len();
        for (count, (a, b)) in counts.into_values() {
            if count > 2 {
                return Err("Наложение или неоднозначная граница пластин LIR".into());
            }
            if count == 1 {
                edges.insert((a, b));
                next.entry(a).or_default().push(b);
            }
        }
        let mut loops = vec![];
        while let Some(&(start, first)) = edges.first() {
            let mut path = vec![start];
            let mut a = start;
            let mut b = first;
            loop {
                if !edges.remove(&(a, b)) {
                    return Err("Повторная граница LIR".into());
                }
                if b == start {
                    break;
                }
                path.push(b);
                let reverse = ((a[1] - b[1]) as f64).atan2((a[0] - b[0]) as f64);
                let c = next
                    .get(&b)
                    .into_iter()
                    .flatten()
                    .copied()
                    .filter(|&c| edges.contains(&(b, c)))
                    .min_by(|c, d| {
                        let turn = |p: Key| {
                            (reverse - ((p[1] - b[1]) as f64).atan2((p[0] - b[0]) as f64))
                                .rem_euclid(std::f64::consts::TAU)
                        };
                        turn(*c).total_cmp(&turn(*d))
                    })
                    .ok_or("Незамкнутая граница плит LIR")?;
                a = b;
                b = c;
                if path.len() > edge_count {
                    return Err("Цикл границы LIR".into());
                }
            }
            // Split a weakly simple walk at repeated contact vertices.
            path.push(start);
            let mut ring = Vec::<Key>::new();
            for vertex in path {
                if let Some(i) = ring.iter().position(|&p| p == vertex) {
                    let p = simple_with(ring[i..].iter().map(|&p| point(p)).collect(), tolerance);
                    if p.len() >= 3 && area(&p).abs() > 1. {
                        loops.push(p);
                    }
                    ring.truncate(i + 1);
                } else {
                    ring.push(vertex);
                }
            }
        }
        Ok(loops)
    }
}
fn wall_rectangles_sweep(
    loops: &[Vec<Point>],
    keep_inscribed: bool,
) -> Result<Vec<[f64; 4]>, String> {
    let mut zs = vec![];
    for ring in loops {
        for (a, b) in ring
            .iter()
            .zip(ring.iter().cycle().skip(1))
            .take(ring.len())
        {
            if !keep_inscribed && (a[0] - b[0]).abs() > 0.05 && (a[1] - b[1]).abs() > 0.05 {
                return Err("Непрямоугольная граница стены пока не поддержана".into());
            }
            zs.push(a[1]);
        }
    }
    zs.sort_by(f64::total_cmp);
    zs.dedup_by(|a, b| (*a - *b).abs() < 0.05);
    let mut rectangles = Vec::<[f64; 4]>::new();
    for z in zs.windows(2) {
        let mid = (z[0] + z[1]) * 0.5;
        let mut xs = vec![];
        for ring in loops {
            for (a, b) in ring
                .iter()
                .zip(ring.iter().cycle().skip(1))
                .take(ring.len())
            {
                if (a[1] <= mid && b[1] > mid) || (b[1] <= mid && a[1] > mid) {
                    let at = |y: f64| a[0] + (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]);
                    let x = at(mid);
                    let (mut lo, mut hi) = (x, x);
                    if keep_inscribed {
                        let [start, end] = [at(z[0]), at(z[1])];
                        lo = start.min(end);
                        hi = start.max(end);
                    }
                    xs.push((x, lo, hi));
                }
            }
        }
        xs.sort_by(|a, b| a.0.total_cmp(&b.0));
        if xs.len() % 2 != 0 {
            return Err("Неоднозначная геометрия стены".into());
        }
        for x in xs.chunks_exact(2) {
            // On sloping edges retain only the rectangle contained in this strip.
            let (lo, hi) = (x[0].2, x[1].1);
            if hi - lo < 0.05 {
                continue;
            }
            if let Some(r) = rectangles.iter_mut().find(|r| {
                (r[0] - lo).abs() < 0.05 && (r[1] - hi).abs() < 0.05 && (r[3] - z[0]).abs() < 0.05
            }) {
                r[3] = z[1];
            } else {
                rectangles.push([lo, hi, z[0], z[1]]);
            }
        }
    }
    Ok(rectangles)
}

fn align_wall_ring(ring: &mut [Point], tolerance: f64) {
    let original = ring.to_vec();
    let n = ring.len();
    for axis in 0..2 {
        let follows = |i: usize| {
            let a = original[i];
            let b = original[(i + 1) % n];
            let minor = (a[axis] - b[axis]).abs();
            let major = (a[1 - axis] - b[1 - axis]).abs();
            minor <= tolerance && minor <= major * 0.25
        };
        let Some(first) = (0..n).find(|&i| !follows(i)) else {
            continue;
        };
        let order = (1..=n).map(|i| (first + i) % n);
        let mut start = 1;
        let (mut low, mut high) = (f64::INFINITY, f64::NEG_INFINITY);
        for (step, i) in order.enumerate() {
            low = low.min(original[i][axis]);
            high = high.max(original[i][axis]);
            if !follows(i) {
                let end = step + 1;
                if high - low <= 2. * tolerance {
                    let aligned = low + (high - low) * 0.5;
                    for k in start..=end {
                        ring[(first + k) % n][axis] = aligned;
                    }
                }
                start = end + 1;
                (low, high) = (f64::INFINITY, f64::NEG_INFINITY);
            }
        }
    }
}

fn wall_component_rectangles(
    outer: &[Point],
    holes: &[Vec<Point>],
    tolerance: f64,
) -> Result<(Vec<[f64; 4]>, bool), String> {
    use mini_fem::geo::{algorithm::Validation, LineString, Polygon};
    let mut loops = std::iter::once(outer)
        .chain(holes.iter().map(Vec::as_slice))
        .map(|ring| ring.iter().map(|p| [p[1], p[0]]).collect::<Vec<_>>())
        .collect::<Vec<_>>();
    let (rects, partial) = wall_rectangles_sweep(&loops, false)
        .map(|r| (r, false))
        .or_else(|strict_error| {
            let mut aligned = loops.clone();
            for ring in &mut aligned {
                align_wall_ring(ring, tolerance);
            }
            let line = |ring: &Vec<Point>| {
                LineString::from(
                    ring.iter()
                        .chain(ring.first())
                        .map(|p| (p[0], p[1]))
                        .collect::<Vec<_>>(),
                )
            };
            if Polygon::new(line(&aligned[0]), aligned[1..].iter().map(line).collect()).is_valid() {
                loops = aligned;
            }
            if let Ok(rects) = wall_rectangles_sweep(&loops, false) {
                return Ok((rects, false));
            }
            let rects = wall_rectangles_sweep(&loops, true)?;
            if rects.is_empty() {
                Err(strict_error)
            } else {
                Ok((rects, true))
            }
        })?;
    Ok((
        rects
            .into_iter()
            .map(|r| [r[2], r[3], r[0], r[1]])
            .collect(),
        partial,
    ))
}

fn snap_wall_cells(cells: &mut [Vec<Point>], extent: [f64; 2], tolerance: f64) {
    for axis in 0..2 {
        let tolerance = if axis == 1 {
            tolerance.min((extent[1] - extent[0]) / 3.)
        } else {
            tolerance
        };
        let mut vertices = cells
            .iter()
            .enumerate()
            .flat_map(|(cell, points)| {
                points
                    .iter()
                    .enumerate()
                    .map(move |(vertex, p)| (p[axis], cell, vertex))
            })
            .collect::<Vec<_>>();
        vertices.sort_unstable_by(|a, b| a.0.total_cmp(&b.0));
        let mut remaining = vertices.as_slice();
        while let Some(&(first, _, _)) = remaining.first() {
            let count = remaining.partition_point(|&(v, _, _)| v - first <= tolerance);
            let last = remaining[count - 1].0;
            let mut value = first + (last - first) / 2.;
            if axis == 1 {
                // The intersection with the floor cut must stay on that exact level.
                if let Some(level) = extent.into_iter().find(|z| (first..=last).contains(z)) {
                    value = level;
                }
            }
            for &(_, cell, vertex) in &remaining[..count] {
                cells[cell][vertex][axis] = value;
            }
            remaining = &remaining[count..];
        }
    }
}

#[derive(Default)]
struct Wall {
    direction: Point,
    offset: f64,
    ids: Vec<(u32, Point)>,
    cells: Vec<Vec<Point>>,
    source: Vec<(u32, Vec<[f64; 3]>)>,
}
fn clip_wall_height(points: &[[f64; 3]], low: f64, high: f64) -> Vec<[f64; 3]> {
    let mut out = points.to_vec();
    for (z, lower) in [(low, true), (high, false)] {
        let input = std::mem::take(&mut out);
        if input.is_empty() {
            break;
        }
        for i in 0..input.len() {
            let a = input[i];
            let b = input[(i + 1) % input.len()];
            let inside = |p: [f64; 3]| if lower { p[2] >= z } else { p[2] <= z };
            if inside(a) {
                out.push(a);
            }
            if inside(a) != inside(b) {
                let t = (z - a[2]) / (b[2] - a[2]);
                out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1]), z]);
            }
        }
        out.dedup_by(|a, b| a.iter().zip(b.iter()).all(|(a, b)| (a - b).abs() < 1e-8));
        if out.len() > 1 && out.first() == out.last() {
            out.pop();
        }
    }
    out
}
fn wall_plane(points: &[[f64; 3]]) -> (Point, f64) {
    let n = points.len() as f64;
    let center = [
        points.iter().map(|p| p[0] / n).sum::<f64>(),
        points.iter().map(|p| p[1] / n).sum::<f64>(),
    ];
    let (mut xx, mut yy, mut xy) = (0., 0., 0.);
    for p in points {
        let x = p[0] - center[0];
        let y = p[1] - center[1];
        xx += x * x;
        yy += y * y;
        xy += x * y;
    }
    let angle = 0.5 * (2. * xy).atan2(xx - yy);
    let mut d = [angle.cos(), angle.sin()];
    if d[0] < -1e-8 || (d[0].abs() <= 1e-8 && d[1] < 0.) {
        d = d.map(|v| -v);
    }
    (d, -d[1] * center[0] + d[0] * center[1])
}
impl Wall {
    fn contains_plane(&self, points: &[[f64; 3]], direction: Point, tolerance: f64) -> bool {
        if (self.direction[0] * direction[1] - self.direction[1] * direction[0]).abs() > 0.05 {
            return false;
        }
        // Keep the initial plane as an anchor. A fit through distant nearly
        // parallel panels could otherwise rotate their axes and merge them.
        if points.iter().any(|p| {
            (-self.direction[1] * p[0] + self.direction[0] * p[1] - self.offset).abs() > tolerance
        }) {
            return false;
        }
        if self.source.is_empty() {
            return true;
        }
        let all = self
            .source
            .iter()
            .flat_map(|(_, p)| p.iter().copied())
            .chain(points.iter().copied())
            .collect::<Vec<_>>();
        let (d, offset) = wall_plane(&all);
        all.iter()
            .all(|p| (-d[1] * p[0] + d[0] * p[1] - offset).abs() <= tolerance)
    }
}
pub(super) fn recover(
    model: &lir_asp_parser::Model,
    name: String,
    unit_mm: f64,
    units: crate::plan_trace::loads::ForceUnits,
    g: f64,
) -> Result<Recovered, String> {
    recover_options(model, name, unit_mm, units, g, None, None, 1.)
}
fn recover_options(
    model: &lir_asp_parser::Model,
    name: String,
    unit_mm: f64,
    units: crate::plan_trace::loads::ForceUnits,
    g: f64,
    round_mm: Option<f64>,
    selected_levels: Option<Vec<f64>>,
    merge_tolerance_mm: f64,
) -> Result<Recovered, String> {
    recover_with_vertical_tolerance(
        model,
        name,
        unit_mm,
        units,
        g,
        round_mm,
        selected_levels,
        merge_tolerance_mm,
        5.,
    )
}
fn recover_with_vertical_tolerance(
    model: &lir_asp_parser::Model,
    name: String,
    unit_mm: f64,
    units: crate::plan_trace::loads::ForceUnits,
    g: f64,
    round_mm: Option<f64>,
    selected_levels: Option<Vec<f64>>,
    merge_tolerance_mm: f64,
    vertical_tolerance_mm: f64,
) -> Result<Recovered, String> {
    if !vertical_tolerance_mm.is_finite() || vertical_tolerance_mm < 0. {
        return Err("Проверьте допуск отклонения от вертикали".into());
    }
    if !unit_mm.is_finite() || unit_mm <= 0. || !g.is_finite() || g <= 0. {
        return Err("Проверьте единицы и g".into());
    }
    if round_mm.is_some_and(|v| !v.is_finite() || v <= 0.) {
        return Err("Проверьте шаг округления".into());
    }
    if !merge_tolerance_mm.is_finite() || merge_tolerance_mm <= 0. {
        return Err("Проверьте допуск объединения".into());
    }
    let tolerance = round_mm.map_or(merge_tolerance_mm, |step| {
        (step / 2_f64.sqrt()).max(merge_tolerance_mm)
    });
    let mut notes = model.section_notes.clone();
    notes.push(format!(
        "Допуск объединения вершин и плоскостей: {tolerance:.3} мм"
    ));
    notes.push(format!("Допуск отклонения от вертикали: {vertical_tolerance_mm:.3} мм; колонны выпрямляются по нижней точке, стены — на вертикальную плоскость."));
    let section_id = |e: &lir_asp_parser::ModelElement| {
        model
            .section_assignments
            .get(e.id as usize - 1)
            .copied()
            .unwrap_or(e.definition)
    };
    // LIR element definitions describe topology and can differ between T3 and
    // Q4 cells of the same shell. Only verified section assignments split it.
    let shell_section_id = |e: &lir_asp_parser::ModelElement| {
        model
            .section_assignments
            .get(e.id as usize - 1)
            .copied()
            .filter(|id| model.sections.get(id).is_some_and(|s| s.kind == 36))
            .unwrap_or(0)
    };

    if let Some(step) = round_mm {
        notes.push(format!(
            "Координаты округлены до {step} мм перед восстановлением."
        ));
    }
    let xyz = model
        .nodes
        .iter()
        .map(|p| {
            p.map(|v| {
                let v = v * unit_mm;
                round_mm.map_or(v, |step| (v / step).round() * step)
            })
        })
        .collect::<Vec<_>>();
    let mut levels = std::collections::BTreeSet::new();
    let mut plate_areas = BTreeMap::<i64, f64>::new();
    let mut wall_min = f64::INFINITY;
    let mut wall_max = f64::NEG_INFINITY;
    for e in model
        .elements
        .iter()
        .filter(|e| matches!(e.kind, 41 | 42 | 44))
    {
        let lo = e
            .nodes
            .iter()
            .map(|&n| xyz[n as usize - 1][2])
            .fold(f64::INFINITY, f64::min);
        let hi = e
            .nodes
            .iter()
            .map(|&n| xyz[n as usize - 1][2])
            .fold(f64::NEG_INFINITY, f64::max);
        if hi - lo < 0.05 {
            let z = ((hi + lo) * 50.).round() as i64;
            levels.insert(z);
            let mut xy = [[0.; 2]; 4];
            let n = e.nodes.len().min(4);
            for (p, &node) in xy.iter_mut().zip(&e.nodes) {
                let [x, y, _] = xyz[node as usize - 1];
                *p = [x, y];
            }
            if matches!(e.kind, 41 | 44) && n == 4 {
                xy.swap(2, 3);
            }
            *plate_areas.entry(z).or_default() += area(&xy[..n]).abs();
        } else {
            wall_min = wall_min.min(lo);
            wall_max = wall_max.max(hi);
        }
    }
    if levels.is_empty() {
        return Err("В LIR не найдены горизонтальные плиты для определения этажей".into());
    }
    let original_areas = std::mem::take(&mut plate_areas);
    levels.clear();
    for (&z, &size) in &original_areas {
        let z = slab_plane(z as f64 / 100., z as f64 / 100., &original_areas).unwrap_or(z);
        levels.insert(z);
        *plate_areas.entry(z).or_default() += size;
    }
    let first = *levels.first().unwrap() as f64 / 100.;
    if wall_min < first - 100. {
        levels.insert((wall_min * 100.).round() as i64);
    }
    let candidate_levels = levels.iter().map(|&z| z as f64 / 100.).collect();
    if let Some(selected) = selected_levels {
        let selected = selected
            .into_iter()
            .map(|z| (z * 100.).round() as i64)
            .collect::<std::collections::BTreeSet<_>>();
        if !selected.is_subset(&levels) {
            return Err("Выберите этажи из найденных отметок".into());
        }
        validate_floor_gap(selected.iter().copied())?;
        levels = selected;
    } else {
        let defaults = default_floor_levels(&levels, &plate_areas);
        let nearby = levels.len() - defaults.len();
        if nearby > 0 {
            notes.push(format!("Близкие отметки ({nearby}) сохранены как промежуточные уровни, список этажей можно изменить в предпросмотре."));
        }
        levels = defaults;
    }
    let levels = levels
        .into_iter()
        .map(|z| z as f64 / 100.)
        .collect::<Vec<_>>();
    let mut building = building::Building::default();
    building.enabled = true;
    building.floors = levels
        .iter()
        .enumerate()
        .map(|(i, &z)| building::Floor {
            id: i as u64 + 1,
            name: format!("Этаж {}", i + 1),
            elevation_mm: z,
            height_mm: levels.get(i + 1).map_or_else(
                || {
                    if wall_max > z + 100. {
                        wall_max - z
                    } else {
                        3000.
                    }
                },
                |next| next - z,
            ),
        })
        .collect();
    building.active = building.floors[0].id;
    building.validate()?;
    let floor_at = |z: f64| levels.partition_point(|&v| v <= z + 0.05).saturating_sub(1);
    let mut plates = BTreeMap::<(usize, i64, u16), Plate>::new();
    let mut slabs_without_sections = 0;
    let mut walls = BTreeMap::<(usize, u16, usize), Wall>::new();
    let mut walls_without_sections = 0;
    let plane_tolerance = tolerance.max(vertical_tolerance_mm);
    for e in model
        .elements
        .iter()
        .filter(|e| matches!(e.kind, 41 | 42 | 44))
    {
        let mut p = e
            .nodes
            .iter()
            .map(|&n| xyz[n as usize - 1])
            .collect::<Vec<_>>();
        if matches!(e.kind, 41 | 44) {
            p.swap(2, 3);
        }
        let unique = p
            .iter()
            .map(|p| p.map(f64::to_bits))
            .collect::<std::collections::BTreeSet<_>>();
        if unique.len() < 3 {
            notes.push(format!("КЭ {} вырожден после округления, пропущен", e.id));
            continue;
        }
        let lo = p.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
        let hi = p.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        if let Some(plane) = slab_plane(lo, hi, &plate_areas) {
            if area(&p.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>()).abs() < 1. {
                notes.push(format!(
                    "КЭ {}: нулевая площадь после округления, пропущен",
                    e.id
                ));
                continue;
            }
            let property = shell_section_id(e);
            slabs_without_sections += usize::from(property == 0);
            plates
                .entry((floor_at(plane as f64 / 100.), plane, property))
                .or_default()
                .add(p.iter().map(|p| [p[0], p[1]]).collect(), e.id);
        } else {
            let a = p[0];
            let Some(&_b) = p.iter().find(|b| (b[0] - a[0]).hypot(b[1] - a[1]) > 0.05) else {
                notes.push(format!(
                    "КЭ {}: вырожденная вертикальная пластина, пропущен",
                    e.id
                ));
                continue;
            };
            let (d, offset) = wall_plane(&p);
            if p.iter()
                .any(|p| (-d[1] * p[0] + d[0] * p[1] - offset).abs() > plane_tolerance)
            {
                notes.push(format!(
                    "КЭ {}: наклонную пластину нельзя представить стеной, пропущен",
                    e.id
                ));
                continue;
            }
            let property = shell_section_id(e);
            walls_without_sections += usize::from(property == 0);
            for f in floor_at(lo)..=floor_at(hi - 0.05) {
                let extent = building.extent(f as u64 + 1);
                let bottom = if f == 0 { lo } else { extent[0] };
                let part = clip_wall_height(&p, bottom, extent[1]);
                if part.len() < 3 {
                    continue;
                }
                let existing = walls.iter().find_map(|(key, wall)| {
                    (key.0 == f
                        && key.1 == property
                        && wall.contains_plane(&part, d, plane_tolerance))
                    .then_some(*key)
                });
                let key = existing.unwrap_or((f, property, walls.len()));
                let w = walls.entry(key).or_insert_with(|| Wall {
                    direction: d,
                    offset,
                    ..Default::default()
                });
                w.source.push((e.id, part));
            }
        }
    }
    // Fit one common plane to all original vertices, then project every cell in
    // that same basis. Reprojection never uses already rounded/projected data.
    for (&(floor, _, _), w) in &mut walls {
        let points = w
            .source
            .iter()
            .flat_map(|(_, p)| p.iter().copied())
            .collect::<Vec<_>>();
        (w.direction, w.offset) = wall_plane(&points);
        w.cells = w
            .source
            .iter()
            .map(|(_, p)| {
                p.iter()
                    .map(|p| [p[0] * w.direction[0] + p[1] * w.direction[1], p[2]])
                    .collect::<Vec<_>>()
            })
            .collect();
        snap_wall_cells(&mut w.cells, building.extent(floor as u64 + 1), tolerance);
        for ((id, _), cell) in w.source.iter().zip(&w.cells) {
            w.ids.push((
                *id,
                [
                    cell.iter().map(|p| p[0] / cell.len() as f64).sum(),
                    cell.iter().map(|p| p[1] / cell.len() as f64).sum(),
                ],
            ));
        }
    }
    let mut contours = vec![];
    let mut origins = BTreeMap::new();
    let mut definitions = BTreeMap::new();
    let mut id = 1;
    if slabs_without_sections != 0 {
        notes.push(format!(
            "Для {slabs_without_sections} горизонтальных КЭ жёсткость LIR не подтверждена. Они объединены по геометрии на каждой отметке с временной жёсткостью. Проверьте сечения и материалы перед расчётом."
        ));
    }
    if walls_without_sections != 0 {
        notes.push(format!(
            "Для {walls_without_sections} стеновых КЭ жёсткость ЛИРА не подтверждена. Они объединены по геометрии с временной жёсткостью. Проверьте сечения и материалы перед расчётом."
        ));
    }
    for ((floor, plane, property), plate) in plates {
        let offset = plane as f64 / 100. - levels[floor];
        if offset.abs() > 0.05 {
            building.enabled = true;
        }
        // Exact-edge cancellation alone leaves seams where the two sides of a
        // shared boundary have different subdivisions in the source mesh.
        let regions = union_polygons_with(&plate.cells, tolerance);
        let single_region = regions.len() == 1;
        for (outer, holes) in regions {
            let mut slab = contour(id, Kind::Slab, outer, None);
            slab.placement.slab.offset_mm = offset;
            building.contour_floors.insert(id, floor as u64 + 1);
            origins.insert(
                id,
                plate
                    .ids
                    .iter()
                    .filter(|(_, at)| {
                        single_region
                            || geometry::inside(*at, &slab.points)
                                && !holes.iter().any(|hole| geometry::inside(*at, hole))
                    })
                    .map(|(id, _)| *id)
                    .collect::<Vec<_>>(),
            );
            definitions.insert(id, property);
            contours.push(slab);
            let parent = id;
            id += 1;
            for hole in holes {
                contours.push(contour(id, Kind::Hole, hole, Some(parent)));
                building.contour_floors.insert(id, floor as u64 + 1);
                id += 1;
            }
        }
    }
    let mut degenerate_walls = 0;
    for ((floor, property, _), w) in walls {
        let [floor_bottom, floor_top] = building.extent(floor as u64 + 1);
        // Summed cell areas can match the full extent when overlaps offset an opening.
        // The union preserves the actual uncovered area.
        for (outer, holes) in union_polygons_with(&w.cells, tolerance) {
            match wall_component_rectangles(&outer, &holes, tolerance) {
                Ok((rectangles, partial)) => {
                    if partial {
                        let source_area =
                            area(&outer).abs() - holes.iter().map(|h| area(h).abs()).sum::<f64>();
                        let restored_area = rectangles
                            .iter()
                            .filter(|r| r[3] - r[2] >= 100. - 1e-5)
                            .map(|r| (r[1] - r[0]) * (r[3] - r[2]))
                            .sum::<f64>();
                        notes.push(format!("Стена этажа {}: наклонная кромка, восстановлены вписанные прямоугольные участки, пропущено {:.3} м²", floor + 1, (source_area - restored_area).max(0.) / 1e6));
                    }
                    for [lo, hi, z0, z1] in rectangles {
                        if z1 - z0 < 100. - 1e-5 {
                            notes.push(format!(
                                "Стена этажа {}: фрагмент высотой менее 100 мм пропущен",
                                floor + 1
                            ));
                            continue;
                        }
                        let Some(mut c) = wall_contour(id, w.direction, w.offset, [lo, hi]) else {
                            degenerate_walls += 1;
                            continue;
                        };
                        if (z0 - floor_bottom).abs() > 0.005 {
                            c.placement.bottom.offset_mm = z0 - floor_bottom;
                        }
                        if (z1 - floor_top).abs() > 0.005 {
                            c.placement.top.anchor = crate::plan_trace::model::LevelAnchor::Bottom;
                            c.placement.top.offset_mm = z1 - floor_bottom;
                        }
                        contours.push(c);
                        building.contour_floors.insert(id, floor as u64 + 1);
                        definitions.insert(id, property);
                        origins.insert(
                            id,
                            w.ids
                                .iter()
                                .filter(|(_, p)| {
                                    p[0] >= lo - 0.05
                                        && p[0] <= hi + 0.05
                                        && p[1] >= z0 - 0.05
                                        && p[1] <= z1 + 0.05
                                })
                                .map(|(id, _)| *id)
                                .collect(),
                        );
                        id += 1;
                    }
                }
                Err(e) => {
                    let skipped = w
                        .ids
                        .iter()
                        .filter(|(_, p)| {
                            geometry::inside(*p, &outer)
                                && !holes.iter().any(|hole| geometry::inside(*p, hole))
                        })
                        .count();
                    notes.push(format!(
                        "Стена этажа {}: {e}; пропущено КЭ в этой части: {skipped}",
                        floor + 1
                    ));
                }
            }
        }
    }
    // Vertical two-node bars can be recovered as storey columns. Retain their
    // source IDs; geometry and section decoding are independent capabilities.
    let mut column_groups = BTreeMap::<(usize, u16, Key, i64), Vec<(f64, f64, u32)>>::new();
    let mut beam_groups = BTreeMap::<(usize, i64, u16, i64, i64), Vec<([Point; 2], u32)>>::new();
    let mut skipped_bars = 0;
    for e in model.elements.iter().filter(|e| e.kind == 10) {
        let [a, b] = [xyz[e.nodes[0] as usize - 1], xyz[e.nodes[1] as usize - 1]];
        let angle = model
            .bar_angles_deg
            .get(&e.id)
            .copied()
            .unwrap_or(0.)
            .to_radians();
        let canonical_roll = if (a[0], a[1]) <= (b[0], b[1]) {
            angle
        } else {
            -angle
        };
        let roll_key = ((canonical_roll.rem_euclid(std::f64::consts::PI)) * 1e8).round() as i64;

        let horizontal = (a[0] - b[0]).hypot(a[1] - b[1]);
        let rise = (a[2] - b[2]).abs();
        if rise <= 0.05 && horizontal > 0.05 {
            let z = (a[2] + b[2]) * 0.5;
            let floor = floor_at(z);
            let property = section_id(e);
            // A rolled bent chain cannot carry one constant section angle when
            // its traversal reverses the canonical local X. Merge straight runs
            // only for non-zero roll, retaining every source member's basis.
            let bearing = if roll_key == 0 {
                0
            } else {
                ((b[1] - a[1])
                    .atan2(b[0] - a[0])
                    .rem_euclid(std::f64::consts::PI)
                    * 1e8)
                    .round() as i64
            };
            beam_groups
                .entry((
                    floor,
                    (z * 100.).round() as i64,
                    property,
                    roll_key,
                    bearing,
                ))
                .or_default()
                .push(([[a[0], a[1]], [b[0], b[1]]], e.id));
            continue;
        }
        if horizontal > vertical_tolerance_mm.max(0.05) || rise < 100. {
            if horizontal <= 0.05 {
                skipped_bars += 1;
                continue;
            }
            let mut beam = contour(id, Kind::Beam, vec![[a[0], a[1]], [b[0], b[1]]], None);
            beam.support_open = true;
            beam.column_angle = inclined_beam_roll(a, b, angle);
            beam.name = format!("Балка LIR {}", e.id);
            beam.placement.beam_z_mm = Some(vec![a[2], b[2]]);
            building.enabled = true;
            building
                .contour_floors
                .insert(id, floor_at(a[2].min(b[2])) as u64 + 1);
            definitions.insert(id, section_id(e));
            origins.insert(id, vec![e.id]);
            contours.push(beam);
            id += 1;
            continue;
        }
        let lo = a[2].min(b[2]);
        let hi = a[2].max(b[2]);
        // A source member can cross several selected storeys (including
        // intermediate slab levels). Split only the recovered geometry, keeping
        // the source element ID and section on every fragment.
        let angle_key = (((if b[2] > a[2] { angle } else { -angle }) - std::f64::consts::FRAC_PI_2)
            .rem_euclid(std::f64::consts::PI)
            * 1e8)
            .round() as i64;
        for floor in floor_at(lo)..=floor_at(hi - 0.05) {
            let start = if floor == 0 {
                lo
            } else {
                lo.max(levels[floor])
            };
            let end = levels.get(floor + 1).map_or(hi, |&level| hi.min(level));
            if end - start > 0.05 {
                column_groups
                    .entry((
                        floor,
                        section_id(e),
                        key(if a[2] <= b[2] {
                            [a[0], a[1]]
                        } else {
                            [b[0], b[1]]
                        }),
                        angle_key,
                    ))
                    .or_default()
                    .push((start, end, e.id));
            }
        }
    }
    for ((floor, z, property, roll, _), segments) in beam_groups {
        for (points, source_ids, closed) in beam_chains(segments, tolerance) {
            let mut beam = contour(id, Kind::Beam, points, None);
            beam.support_open = !closed;
            let forward =
                (beam.points[0][0], beam.points[0][1]) <= (beam.points[1][0], beam.points[1][1]);
            beam.column_angle = if forward {
                roll as f64 / 1e8
            } else {
                -(roll as f64 / 1e8)
            };
            beam.name = format!("Балка LIR {}", source_ids[0]);
            beam.placement.slab.offset_mm = z as f64 / 100. - levels[floor];
            if beam.placement.slab.offset_mm.abs() > 1e-5 {
                building.enabled = true;
            }
            building.contour_floors.insert(id, floor as u64 + 1);
            definitions.insert(id, property);
            origins.insert(id, source_ids);
            contours.push(beam);
            id += 1;
        }
    }
    for ((floor, property, xy, angle), mut spans) in column_groups {
        spans.sort_by(|a, b| a.0.total_cmp(&b.0));
        let floor_id = floor as u64 + 1;
        let [bottom, top] = building.extent(floor_id);
        let mut remaining = spans.as_slice();
        while let Some(&(bottom_z, mut top_z, _)) = remaining.first() {
            let mut count = 1;
            while let Some(&(start, end, _)) = remaining.get(count) {
                if start > top_z + 0.05 {
                    break;
                }
                top_z = top_z.max(end);
                count += 1;
            }
            let (connected, next) = remaining.split_at(count);
            remaining = next;

            let mut column = contour(id, Kind::Column, vec![point(xy)], None);
            column.column_angle = angle as f64 / 1e8;
            if (bottom_z - bottom).abs() > 0.05 {
                column.placement.bottom.offset_mm = bottom_z - bottom;
            }
            if (top_z - top).abs() > 0.05 {
                column.placement.top.offset_mm = top_z - top;
            }
            building.enabled |= !column.placement.levels_default();
            contours.push(column);
            building.contour_floors.insert(id, floor_id);
            definitions.insert(id, property);
            origins.insert(
                id,
                connected
                    .iter()
                    .map(|s| s.2)
                    .collect::<BTreeSet<_>>()
                    .into_iter()
                    .collect(),
            );
            id += 1;
        }
    }
    let missing_angles = model
        .elements
        .iter()
        .filter(|e| e.kind == 10 && !model.bar_angles_deg.contains_key(&e.id))
        .count();
    if missing_angles > 0 {
        notes.push(format!("У {missing_angles} стержней угол сечения не распознан: использовано исходное нулевое направление ЛИРА; проверьте ориентацию."));
    }
    if skipped_bars > 0 {
        notes.push(format!(
            "Пропущено стержневых КЭ: {skipped_bars} (совпавшие узлы или вертикальный стержень высотой менее 100 мм)."
        ));
    }
    let mut library = crate::plan_trace::stiffness::Library::default();
    let mut assigned = BTreeMap::new();
    if model.skipped_invalid > 0 {
        notes.push(format!(
            "Пропущено КЭ с некорректными связями узлов: {}",
            model.skipped_invalid
        ));
    }
    for (definition, count) in &model.skipped_unknown {
        notes.push(format!(
            "Пропущено КЭ с нераспознанным типом: {count} (определение {definition})."
        ));
    }
    let unsupported = model
        .elements
        .iter()
        .filter(|e| !matches!(e.kind, 10 | 41 | 42 | 44))
        .count();
    if unsupported > 0 {
        notes.push(format!("Пропущено {unsupported} КЭ: их тип распознан, но восстановление таких объектов пока не поддержано."));
    }

    for c in contours.iter().filter(|c| {
        matches!(
            c.kind,
            Kind::Slab | Kind::Support | Kind::Column | Kind::Beam
        )
    }) {
        let property = definitions[&c.id];
        let family = match c.kind {
            Kind::Column => crate::plan_trace::stiffness::Family::Column,
            Kind::Beam => crate::plan_trace::stiffness::Family::Beam,
            Kind::Slab => crate::plan_trace::stiffness::Family::Slab,
            _ => crate::plan_trace::stiffness::Family::Wall,
        };
        let family_key = family as u8;
        let entry = if let Some(&entry) = assigned.get(&(family_key, property)) {
            entry
        } else {
            let known = model.sections.get(&property).filter(|p| match c.kind {
                Kind::Column | Kind::Beam => p.kind == 0,
                _ => p.kind == 36,
            });
            let mut material = if let Some(p) = known {
                crate::plan_trace::fem::element::Material {
                    e_mpa: p.parameters[1] as f64
                        * if units == crate::plan_trace::loads::ForceUnits::TonForce {
                            g / 1000.
                        } else {
                            0.001
                        },
                    nu: if p.kind == 36 {
                        p.parameters[2] as f64
                    } else {
                        0.2
                    },
                    thickness_mm: p.parameters[3] as f64 * 10.,
                    weight: crate::plan_trace::loads::Magnitude {
                        value: p.parameters[0] as f64,
                        units,
                    },
                }
            } else {
                if property != 0 || c.kind != Kind::Slab {
                    notes.push(format!("Жёсткость LIR {property}: параметры не распознаны, требуется проверить временные значения"));
                }
                if matches!(c.kind, Kind::Column | Kind::Beam) {
                    library.entries[&if c.kind == Kind::Column { 4 } else { 3 }].material
                } else {
                    Default::default()
                }
            };
            if let Err(e) = material.validate() {
                notes.push(format!(
                    "Жёсткость {property}: {e}; временные параметры требуют проверки"
                ));
                material = Default::default();
            }
            let entry = library.add(family, material);
            if let Some(p) = known {
                let label = if p.name.is_empty() {
                    family.label()
                } else {
                    &p.name
                };
                library.entries.get_mut(&entry).unwrap().name = format!("{label} · LIR {property}");
                library.colors.insert(entry, p.color);
                if matches!(c.kind, Kind::Column | Kind::Beam) {
                    library.entries.get_mut(&entry).unwrap().width_mm =
                        p.parameters[2] as f64 * 10.;
                    notes.push("Сечения стержней прочитаны из LIR (размеры исходных определений в см). Повороты сечений прочитаны только для подтверждённых записей LIR; нераспознанные требуют проверки. Коэффициент Пуассона стержней временно ν=0,2.".into());
                }
            } else {
                library.entries.get_mut(&entry).unwrap().name =
                    if property == 0 && matches!(c.kind, Kind::Slab | Kind::Support) {
                        format!("{} LIR · временная жёсткость", family.label())
                    } else {
                        format!("{} LIR {property}", family.label())
                    };
                if c.kind == Kind::Column {
                    library.entries.get_mut(&entry).unwrap().width_mm = 250.;
                    notes.push(format!("Колонны LIR {property}: сечение и ориентация не распознаны; временно 250×750 мм. Перед расчётом задайте фактическое сечение."));
                }
            }
            assigned.insert((family_key, property), entry);
            entry
        };
        library.assignments.insert(c.id, entry);
    }
    let mut soil_count = 0usize;
    let mut imported_cells = 0usize;
    let mut next_soil = contours.iter().map(|c| c.id).max().unwrap_or(0) + 1;
    let mut slab_owners = BTreeMap::<u32, Vec<usize>>::new();
    for (index, c) in contours
        .iter()
        .enumerate()
        .filter(|(_, c)| c.kind == Kind::Slab)
    {
        for &id in origins.get(&c.id).into_iter().flatten() {
            if model.foundations.contains_key(&id) {
                slab_owners.entry(id).or_default().push(index);
            }
        }
    }
    let mut plane_regions = BTreeMap::<(u64, i64), Vec<(u32, Vec<Point>, f64)>>::new();
    for e in &model.elements {
        let Some(soil) = model.foundations.get(&e.id) else {
            continue;
        };
        let Some(owners) = slab_owners.get(&e.id) else {
            continue;
        };
        let mut points = e
            .nodes
            .iter()
            .map(|&n| xyz[n as usize - 1])
            .collect::<Vec<_>>();
        if points.len() == 4 {
            points.swap(2, 3);
        }
        if points.len() < 3
            || points
                .iter()
                .any(|p| (p[2] - points[0][2]).abs() > tolerance)
        {
            continue;
        }
        let mut planes = BTreeSet::new();
        for &index in owners {
            let c = &mut contours[index];
            c.use_foundation = true;
            let floor = building.owner(c.id);
            planes.insert((
                floor,
                (c.placement.slab.z(building.extent(floor)) * 100.).round() as i64,
            ));
        }
        let points = points.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
        for plane in planes {
            plane_regions
                .entry(plane)
                .or_default()
                .push((e.id, points.clone(), soil.c1));
        }
        imported_cells += 1;
    }
    for ((floor, plane), regions) in plane_regions {
        let polygons = regions
            .iter()
            .map(|(_, p, _)| p.clone())
            .collect::<Vec<_>>();
        for (points, _) in union_polygons_with(&polygons, 0.01) {
            let points = foundation_boundary_nodes(&points, &polygons);
            let mut c = contour(next_soil, Kind::Foundation, points, None);
            c.placement.slab.offset_mm = plane as f64 / 100. - building.extent(floor)[0];
            let data = imported_foundation_data(&c.points, &regions, units);
            c.name = "Основание LIR".into();
            building.contour_floors.insert(c.id, floor);
            origins.insert(
                c.id,
                regions
                    .iter()
                    .filter(|(_, p, _)| data.covers(&c.points, cell_center(p)))
                    .map(|(id, _, _)| *id)
                    .collect(),
            );
            c.foundation = Some(data);
            contours.push(c);
            next_soil += 1;
            soil_count += 1;
        }
    }
    if soil_count > 0 {
        notes.push(format!("Основание C1: {imported_cells} исходных КЭ объединены в {soil_count} областей на фактических отметках плит. C2 и Rz сохранены как метаданные LIR; их единицы отдельно не подтверждены, в расчёте они пока не применяются."));
    }
    notes.sort();
    notes.dedup();
    if notes.len() > 50 {
        let count = notes.len();
        notes.truncate(50);
        notes.push(format!(
            "Ещё {} сообщений о пропущенных/неподдержанных объектах",
            count - 50
        ));
    }
    if degenerate_walls > 0 {
        notes.insert(
            0,
            format!("После проекции пропущено вырожденных фрагментов стен: {degenerate_walls}."),
        );
    }
    let source = Source {
        name,
        nodes: model.nodes.len(),
        elements: model.elements.len(),
        inactive: model.inactive_elements,
        inferred_types: model.inferred_types,
        unit_mm,
        gravity: g,
        force_units: units,
        contour_elements: origins,
        foundation_assignments: model
            .foundations
            .iter()
            .map(|(&id, s)| {
                (
                    id,
                    SoilSource {
                        c1: s.c1,
                        c2: s.c2,
                        rz_text: s.rz_text.clone(),
                    },
                )
            })
            .collect(),
        notes,
    };
    Ok(Recovered {
        candidate_levels,
        #[cfg(test)]
        plate_areas,
        contours,
        building,
        library,
        source,
        load_cases: vec![],
    })
}

fn foundation_boundary_nodes(ring: &[Point], cells: &[Vec<Point>]) -> Vec<Point> {
    let mut out = Vec::new();
    for (&a, &b) in ring
        .iter()
        .zip(ring.iter().cycle().skip(1))
        .take(ring.len())
    {
        let d = [b[0] - a[0], b[1] - a[1]];
        let l = d[0] * d[0] + d[1] * d[1];
        if l < 1e-12 {
            continue;
        }
        let mut nodes = vec![(0., a)];
        for &p in cells.iter().flatten() {
            let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l;
            if t > 1e-8
                && t < 1. - 1e-8
                && (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]) < 0.02
            {
                nodes.push((t, [a[0] + t * d[0], a[1] + t * d[1]]));
            }
        }
        nodes.sort_by(|a, b| a.0.total_cmp(&b.0));
        nodes.dedup_by(|a, b| (a.0 - b.0).abs() * l.sqrt() < 0.02);
        out.extend(nodes.into_iter().map(|(_, p)| p));
    }
    out
}

fn cell_center(p: &[Point]) -> Point {
    std::array::from_fn(|a| p.iter().map(|p| p[a]).sum::<f64>() / p.len() as f64)
}
fn imported_foundation_data(
    points: &[Point],
    cells: &[(u32, Vec<Point>, f64)],
    units: crate::plan_trace::loads::ForceUnits,
) -> crate::plan_trace::foundation::Data {
    use crate::plan_trace::{
        foundation::{Data, Sample},
        loads::Magnitude,
    };
    let value = |p: Point| {
        let mut sum = 0.;
        let mut count = 0.;
        for (_, cell, c1) in cells {
            if cell
                .iter()
                .zip(cell.iter().cycle().skip(1))
                .take(cell.len())
                .any(|(a, b)| {
                    let d = [b[0] - a[0], b[1] - a[1]];
                    let l = d[0] * d[0] + d[1] * d[1];
                    let t = if l > 0. {
                        ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l
                    } else {
                        0.
                    }
                    .clamp(0., 1.);
                    (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]) <= 0.02
                })
            {
                sum += c1;
                count += 1.;
            }
        }
        if count > 0. {
            sum / count
        } else {
            cells
                .iter()
                .min_by(|a, b| {
                    let dist = |c: &(u32, Vec<Point>, f64)| {
                        let q = cell_center(&c.1);
                        (p[0] - q[0]).hypot(p[1] - q[1])
                    };
                    dist(a).total_cmp(&dist(b))
                })
                .map(|c| c.2)
                .unwrap_or(0.)
        }
    };
    let mut data = Data {
        values: points
            .iter()
            .map(|&p| Magnitude {
                value: value(p),
                units,
            })
            .collect(),
        ..Default::default()
    };
    data.interior = cells
        .iter()
        .filter_map(|(_, cell, c1)| {
            let xy = cell_center(cell);
            data.covers(points, xy).then_some(Sample {
                xy,
                c1: Magnitude { value: *c1, units },
            })
        })
        .collect();
    data
}

fn stamp_contour_parts(
    stamp: &lir_asp_parser::stamp_loads::Stamp,
) -> Result<Vec<(lir_asp_parser::stamp_loads::Stamp, Vec<Vec<[f64; 3]>>)>, String> {
    use mini_fem::geo::{BooleanOps, LineString, MultiPolygon, Polygon};
    if stamp.contours.len() <= 1 {
        return Ok(vec![(stamp.clone(), vec![])]);
    }
    if stamp.kind == 2 {
        return Ok(stamp
            .contours
            .iter()
            .map(|p| {
                let mut c = stamp.clone();
                c.points = p.clone();
                (c, vec![])
            })
            .collect());
    }
    let z = stamp.points[0][2];
    if stamp
        .contours
        .iter()
        .flatten()
        .any(|p| (p[2] - z).abs() > 1e-8)
    {
        return Err("Части одного штампа находятся на разных отметках".into());
    }
    // Source ring order is arbitrary (a hole may precede its outer contour).
    // Even/odd filling preserves nested holes and disconnected outer islands.
    let mut area = MultiPolygon(vec![]);
    for p in &stamp.contours {
        area = area.xor(&Polygon::new(
            LineString::from(p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
            vec![],
        ));
    }
    Ok(area
        .0
        .into_iter()
        .map(|p| {
            let xyz = |r: &LineString<f64>| r.0.iter().map(|p| [p.x, p.y, z]).collect::<Vec<_>>();
            let mut c = stamp.clone();
            c.points = xyz(p.exterior());
            (c, p.interiors().iter().map(xyz).collect())
        })
        .collect())
}
fn append_stamp_loads(
    recovered: &mut Recovered,
    report: &lir_asp_parser::stamp_loads::Report,
    round_mm: Option<f64>,
) {
    use crate::plan_trace::loads::{LoadCase, LoadData, Magnitude};
    recovered.source.notes.extend(report.notes.iter().cloned());
    let mut next_id = recovered.contours.iter().map(|c| c.id).max().unwrap_or(0) + 1;
    let mut cases = std::collections::BTreeSet::new();
    let mut sequence_assigned = 0;
    let mut imported_stamps = BTreeMap::<u16, (usize, usize)>::new();
    let mut parts = Vec::new();
    for stamp in &report.stamps {
        match stamp_contour_parts(stamp) {
            Ok(p) => parts.extend(p),
            Err(e) => recovered
                .source
                .notes
                .push(format!("Штамп {}: {e}; пропущен", stamp.id)),
        }
    }
    for (stamp, holes) in parts {
        if stamp.value != stamp.end_value {
            recovered.source.notes.push(format!("Штамп {}: переменная интенсивность {}…{} пока не поддержана, не заменяется равномерной",stamp.id,stamp.value,stamp.end_value));
            continue;
        }
        // Source contours remain independent of the old FE projection. The
        // 1-based source sequence identifies its case metadata; different old
        // projection elevation, clipping or incomplete coverage is not a gate.
        let fallback = stamp
            .attachment_case_id
            .filter(|_| stamp.attachment_direction_code == Some(153));
        if stamp.case_id.is_none() && fallback.is_some() {
            sequence_assigned += 1;
        }
        let Some(case) = stamp
            .case_id
            .filter(|_| stamp.direction_code == Some(153))
            .or(fallback)
        else {
            recovered.source.notes.push(format!(
                "Штамп {} «{}»: загружение или вертикальное направление не подтверждено, пропущен",
                stamp.id, stamp.name
            ));
            continue;
        };
        let points = stamp
            .points
            .iter()
            .map(|p| {
                p.map(|v| {
                    let mm = v * recovered.source.unit_mm;
                    round_mm.map_or(mm, |step| (mm / step).round() * step)
                })
            })
            .collect::<Vec<_>>();
        let Some(first) = points.first() else {
            continue;
        };
        let z = first[2];
        if points.iter().any(|p| (p[2] - z).abs() > 0.05) {
            recovered.source.notes.push(format!(
                "Штамп {}: геометрия не горизонтальна, пропущен",
                stamp.id
            ));
            continue;
        }
        let floor_index = recovered
            .building
            .floors
            .partition_point(|f| f.elevation_mm <= z + 0.05)
            .saturating_sub(1);
        let floor = &recovered.building.floors[floor_index];
        let kind = match stamp.kind {
            4 => Kind::LoadArea,
            2 => Kind::LoadLine,
            _ => continue,
        };
        let mut xy = points.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>();
        xy.dedup();
        let closed = xy.len() > 2 && xy.first() == xy.last();
        if closed {
            xy.pop();
        }
        let mut c = contour(next_id, kind, xy, None);
        c.name = stamp.name.clone();
        c.placement.slab.offset_mm = z - floor.elevation_mm;
        c.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: stamp.name.clone(),
            mesh_constraint: false,
            case_id: case as u64,
            magnitude: Magnitude {
                value: stamp.value,
                units: recovered.source.force_units,
            },
            closed: kind == Kind::LoadArea || closed,
        });
        if let Err(error) = model::validate_candidate(&recovered.contours, &c) {
            recovered
                .source
                .notes
                .push(format!("Штамп {}: {error}; пропущен", stamp.id));
            continue;
        }
        if c.placement.slab.offset_mm.abs() > 0.05 {
            recovered.building.enabled = true;
        }
        let mut added_holes = Vec::new();
        for ring in holes {
            let mut xy = ring
                .iter()
                .map(|p| {
                    [p[0], p[1]].map(|v| {
                        let mm = v * recovered.source.unit_mm;
                        round_mm.map_or(mm, |step| (mm / step).round() * step)
                    })
                })
                .collect::<Vec<_>>();
            xy.dedup();
            if xy.first() == xy.last() {
                xy.pop();
            }
            let mut h = contour(
                next_id + 1 + added_holes.len() as u64,
                Kind::LoadHole,
                xy,
                Some(c.id),
            );
            h.name = format!("Отверстие штампа {}", stamp.id);
            h.load = c.load.clone();
            added_holes.push(h);
        }
        let mut candidate = vec![c.clone()];
        candidate.extend(added_holes.iter().cloned());
        if let Some(error) = added_holes
            .iter()
            .find_map(|h| model::validate_candidate(&candidate, h).err())
        {
            recovered.source.notes.push(format!(
                "Штамп {}: некорректное отверстие: {error}; штамп целиком пропущен",
                stamp.id
            ));
            continue;
        }
        recovered.building.contour_floors.insert(c.id, floor.id);
        recovered.contours.push(c);
        next_id += 1 + added_holes.len() as u64;
        for h in added_holes {
            recovered.building.contour_floors.insert(h.id, floor.id);
            recovered.contours.push(h);
        }
        let counts = imported_stamps.entry(case).or_default();
        if kind == Kind::LoadArea {
            counts.0 += 1;
        } else {
            counts.1 += 1;
        }
        cases.insert(case as u64);
    }
    recovered.source.notes.push(format!("Штампы LIR: привязаны по порядковому номеру исходного штампа {sequence_assigned}; интенсивности перенесены без масштабирования. Расхождения интегралов в отчёте не скрываются."));
    recovered.source.notes.push(format!(
        "Фактически перенесено исходных штампов: {}. По загружениям: {}.",
        imported_stamps.values().map(|(a, l)| a + l).sum::<usize>(),
        imported_stamps
            .iter()
            .map(|(case, (a, l))| format!("{case}: площадных {a}, линейных {l}"))
            .collect::<Vec<_>>()
            .join("; ")
    ));
    recovered.load_cases = cases.into_iter().map(LoadCase::new).collect();
    if !recovered.load_cases.is_empty() {
        recovered.source.notes.push("Импортированы исходные вертикальные штампы. Их дискретизация на КЭ повторно не импортируется. Для загружений установлены γf=γl=1. Проверьте коэффициенты перед расчётом.".into());
    }
}
fn append_uniform_loads(
    recovered: &mut Recovered,
    model: &lir_asp_parser::Model,
    report: &lir_asp_parser::physical_loads::Report,
    round_mm: Option<f64>,
) {
    use crate::plan_trace::loads::{LoadCase, LoadData, Magnitude};
    use lir_asp_parser::physical_loads::TargetKind;
    recovered.source.notes.extend(report.notes.iter().cloned());
    for case in &report.cases {
        if case.id == 0 {
            continue;
        }
        if let Some(c) = recovered
            .load_cases
            .iter_mut()
            .find(|c| c.id == case.id as u64)
        {
            if !case.name.trim().is_empty() {
                c.name = case.name.clone();
            }
        } else {
            let mut c = LoadCase::new(case.id as u64);
            if !case.name.trim().is_empty() {
                c.name = case.name.clone();
            }
            recovered.load_cases.push(c);
        }
    }
    recovered.load_cases.sort_by_key(|c| c.id);
    let xyz = model
        .nodes
        .iter()
        .map(|p| {
            p.map(|v| {
                let v = v * recovered.source.unit_mm;
                round_mm.map_or(v, |step| (v / step).round() * step)
            })
        })
        .collect::<Vec<_>>();
    let elements = model
        .elements
        .iter()
        .map(|e| (e.id, e))
        .collect::<BTreeMap<_, _>>();
    let eligible = recovered
        .contours
        .iter()
        .filter(|c| matches!(c.kind, Kind::Slab | Kind::Beam))
        .flat_map(|c| {
            recovered
                .source
                .contour_elements
                .get(&c.id)
                .into_iter()
                .flatten()
                .copied()
        })
        .collect::<std::collections::BTreeSet<_>>();
    // Sum source contributions on each physical FE BEFORE reconstructing zones.
    // Different zones then tile the source mesh instead of relying on max-q to
    // combine independent overlapping source actions.
    let mut intensities = BTreeMap::<(u16, u32), f64>::new();
    let mut nodal = BTreeMap::<(u16, u32), f64>::new();
    for target in &report.records {
        let r = &target.record;
        if !r.payload.first().is_some_and(|v| v.is_finite()) {
            continue;
        }
        let map = match target.target_kind {
            TargetKind::Element if matches!(r.code, 63 | 163) => &mut intensities,
            TargetKind::Node if r.code == 103 => &mut nodal,
            _ => continue,
        };
        *map.entry((r.case_id, target.target_id)).or_default() += r.payload[0] as f64;
    }
    let mut zones = BTreeMap::<(usize, i64, u16, u64), Plate>::new();
    let mut lines = BTreeMap::<(usize, i64, u16, u64), Vec<([Point; 2], u32)>>::new();
    let floor_at = |z: f64| {
        recovered
            .building
            .floors
            .partition_point(|f| f.elevation_mm <= z + 0.05)
            .saturating_sub(1)
    };
    let mut skipped = 0usize;
    for ((case, id), q) in intensities {
        if q == 0. {
            continue;
        }
        if !eligible.contains(&id) {
            skipped += 1;
            continue;
        }
        let Some(e) = elements.get(&id) else {
            skipped += 1;
            continue;
        };
        let mut p = e
            .nodes
            .iter()
            .map(|&n| xyz[n as usize - 1])
            .collect::<Vec<_>>();
        let lo = p.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
        let hi = p.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
        if hi - lo > 0.05 {
            skipped += 1;
            continue;
        }
        let z = (lo + hi) * 0.5;
        let floor = floor_at(z);
        let key = (floor, (z * 100.).round() as i64, case, q.to_bits());
        if e.kind == 10 && p.len() == 2 {
            lines
                .entry(key)
                .or_default()
                .push(([[p[0][0], p[0][1]], [p[1][0], p[1][1]]], id));
        } else if matches!(e.kind, 41 | 42 | 44) {
            if matches!(e.kind, 41 | 44) {
                p.swap(2, 3);
            }
            zones
                .entry(key)
                .or_default()
                .add(p.iter().map(|p| [p[0], p[1]]).collect(), id);
        } else {
            skipped += 1;
        }
    }
    let mut next = recovered.contours.iter().map(|c| c.id).max().unwrap_or(0) + 1;
    let initial = next;
    let mut imported_area = 0usize;
    let mut imported_line = 0usize;
    let data = |case: u16, q: f64, name: String, closed: bool| LoadData {
        direction: Default::default(),
        moment: false,
        additive: false,
        name,
        mesh_constraint: false,
        case_id: case as u64,
        magnitude: Magnitude {
            value: q,
            units: recovered.source.force_units,
        },
        closed,
    };
    for ((floor, z, case, bits), plate) in zones {
        // Preserve loaded-mesh boundaries: no engineering-tolerance smoothing
        // between adjacent zones with different pressures.
        let rings = plate
            .loops_with(1e-6)
            .unwrap_or_else(|_| union_loops_with(&plate.cells, 1e-6));
        let mut outers = vec![];
        for ring in rings.iter().filter(|r| area(r) > 0.) {
            let name = format!("Распределённая LIR · загружение {case}");
            let mut c = contour(next, Kind::LoadArea, ring.clone(), None);
            c.name = name.clone();
            c.placement.slab.offset_mm =
                z as f64 / 100. - recovered.building.floors[floor].elevation_mm;
            c.load = Some(data(case, f64::from_bits(bits), name, true));
            if let Err(error) = model::validate_candidate(&recovered.contours, &c) {
                recovered
                    .source
                    .notes
                    .push(format!("Нагрузка LIR: {error}; пропущен"));
                continue;
            }
            recovered.source.contour_elements.insert(
                next,
                plate
                    .ids
                    .iter()
                    .filter(|(_, at)| geometry::inside(*at, ring))
                    .map(|(id, _)| *id)
                    .collect(),
            );
            recovered
                .building
                .contour_floors
                .insert(next, recovered.building.floors[floor].id);
            if c.placement.slab.offset_mm.abs() > 0.05 {
                recovered.building.enabled = true;
            }
            outers.push((next, ring.clone(), c.placement.clone()));
            recovered.contours.push(c);
            next += 1;
            imported_area += 1;
        }
        for ring in rings.into_iter().filter(|r| area(r) < 0.) {
            let Some((parent, _, placement)) = outers
                .iter()
                .filter(|(_, p, _)| geometry::inside(ring[0], p))
                .min_by(|a, b| area(&a.1).abs().total_cmp(&area(&b.1).abs()))
            else {
                continue;
            };
            let mut c = contour(next, Kind::LoadHole, ring, Some(*parent));
            c.placement = placement.clone();
            c.load = Some(data(
                case,
                f64::from_bits(bits),
                "Отверстие нагрузки LIR".into(),
                true,
            ));
            if model::validate_candidate(&recovered.contours, &c).is_ok() {
                recovered
                    .building
                    .contour_floors
                    .insert(next, recovered.building.floors[floor].id);
                recovered.contours.push(c);
                next += 1;
            }
        }
    }
    for ((floor, z, case, bits), segments) in lines {
        for (points, ids, closed) in beam_chains(segments, 1e-6) {
            let name = format!("Линейная LIR · загружение {case}");
            let mut c = contour(next, Kind::LoadLine, points, None);
            c.name = name.clone();
            c.placement.slab.offset_mm =
                z as f64 / 100. - recovered.building.floors[floor].elevation_mm;
            c.load = Some(data(case, f64::from_bits(bits), name, closed));
            if model::validate_candidate(&recovered.contours, &c).is_err() {
                skipped += ids.len();
                continue;
            }
            recovered.source.contour_elements.insert(next, ids);
            recovered
                .building
                .contour_floors
                .insert(next, recovered.building.floors[floor].id);
            if c.placement.slab.offset_mm.abs() > 0.05 {
                recovered.building.enabled = true;
            }
            recovered.contours.push(c);
            next += 1;
            imported_line += 1;
        }
    }
    let mut imported_point = 0;
    for ((case, id), q) in nodal {
        if q == 0. {
            continue;
        }
        let Some(p) = id.checked_sub(1).and_then(|n| xyz.get(n as usize)) else {
            skipped += 1;
            continue;
        };
        let floor = floor_at(p[2]);
        let supported = recovered
            .contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab
                    && recovered.building.owner(c.id) == recovered.building.floors[floor].id
            })
            .any(|c| {
                (c.placement
                    .slab
                    .z(recovered.building.extent(recovered.building.owner(c.id)))
                    - p[2])
                    .abs()
                    < 0.05
                    && geometry::inside([p[0], p[1]], &c.points)
            });
        if !supported {
            skipped += 1;
            continue;
        }
        let mut c = contour(next, Kind::LoadPoint, vec![[p[0], p[1]]], None);
        c.name = format!("Узловая LIR {id}");
        c.placement.slab.offset_mm = p[2] - recovered.building.floors[floor].elevation_mm;
        c.load = Some(data(case, q, c.name.clone(), false));
        recovered
            .building
            .contour_floors
            .insert(next, recovered.building.floors[floor].id);
        if c.placement.slab.offset_mm.abs() > 0.05 {
            recovered.building.enabled = true;
        }
        recovered.contours.push(c);
        next += 1;
        imported_point += 1;
    }
    recovered.source.notes.push(format!("Физические нагрузки LIR: площадных {imported_area}, линейных {imported_line}, сосредоточенных {imported_point}; неподдержанных по геометрии/отметке записей {skipped}."));
    if !report.unsupported_counts.is_empty() {
        recovered.source.notes.push(format!("Физические записи иных типов/направлений не преобразованы в обычные нагрузки: {}. Код 153 обрабатывается только через подтверждённые исходные штампы.",report.unsupported_counts.iter().map(|(code,n)|format!("{code}: {n}")).collect::<Vec<_>>().join(", ")));
    }
    if next > initial {
        recovered.source.notes.push("Соседние КЭ с одинаковой суммарной интенсивностью объединены по областям. Типы загружений и γ пока требуют проверки (γf=γl=1); перенесены числовые значения LIR без повторного применения коэффициентов.".into());
    }
}
/// Net footprint at each candidate level; overlap of materials/capitals counted once.
fn floor_plate_areas(preview: &Recovered) -> BTreeMap<i64, f64> {
    use mini_fem::geo::{Area, BooleanOps, LineString, MultiPolygon, Polygon};
    let polygon = |points: &[Point]| {
        let mut ring = points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
        if let Some(first) = ring.first().copied() {
            ring.push(first);
        }
        Polygon::new(LineString::from(ring), vec![])
    };
    let mut holes = BTreeMap::<u64, Vec<&Contour>>::new();
    for hole in preview.contours.iter().filter(|c| c.active_hole()) {
        if let Some(parent) = hole.parent {
            holes.entry(parent).or_default().push(hole);
        }
    }
    let mut totals = BTreeMap::<i64, MultiPolygon<f64>>::new();
    for slab in preview.contours.iter().filter(|c| c.kind == Kind::Slab) {
        let floor = preview.building.owner(slab.id);
        let z = slab.placement.slab.z(preview.building.extent(floor));
        let mut part = MultiPolygon(vec![polygon(&slab.points)]);
        for hole in holes.get(&slab.id).into_iter().flatten() {
            part = part.difference(&polygon(&hole.points));
        }
        let total = totals
            .entry((z * 100.).round() as i64)
            .or_insert_with(|| MultiPolygon(vec![]));
        *total = total.union(&part);
    }
    totals
        .into_iter()
        .map(|(z, total)| (z, total.unsigned_area() / 1e6))
        .collect()
}

enum Event {
    Parsed(
        Result<
            (
                lir_asp_parser::Model,
                lir_asp_parser::stamp_loads::Report,
                lir_asp_parser::physical_loads::Report,
            ),
            String,
        >,
    ),
    Recovered(Result<Recovered, String>),
}

#[derive(Clone, Copy, Default, PartialEq, Eq)]
enum FloorSort {
    #[default]
    ElevationAscending,
    ElevationDescending,
    AreaDescending,
    AreaAscending,
}

impl FloorSort {
    fn controls(&mut self, ui: &mut egui::Ui) {
        let by_elevation = matches!(self, Self::ElevationAscending | Self::ElevationDescending);
        let (z, a) = match self {
            Self::ElevationAscending => ("Z возр.", "A"),
            Self::ElevationDescending => ("Z убыв.", "A"),
            Self::AreaDescending => ("Z", "A убыв."),
            Self::AreaAscending => ("Z", "A возр."),
        };
        ui.horizontal(|ui| {
            if ui
                .add(egui::Button::new(z).selected(by_elevation))
                .on_hover_text("Сортировать по отметке Z, мм")
                .clicked()
            {
                *self = if *self == Self::ElevationAscending {
                    Self::ElevationDescending
                } else {
                    Self::ElevationAscending
                };
            }
            if ui
                .add(egui::Button::new(a).selected(!by_elevation))
                .on_hover_text("Сортировать по площади плит A, м²")
                .clicked()
            {
                *self = if *self == Self::AreaDescending {
                    Self::AreaAscending
                } else {
                    Self::AreaDescending
                };
            }
        });
    }

    fn rows(self, choices: &[(f64, bool, f64)]) -> Vec<usize> {
        let mut rows = (0..choices.len()).collect::<Vec<_>>();
        rows.sort_unstable_by(|&i, &j| {
            let (left_z, _, left_area) = choices[i];
            let (right_z, _, right_area) = choices[j];
            let by_elevation = left_z.total_cmp(&right_z);
            match self {
                Self::ElevationAscending => by_elevation,
                Self::ElevationDescending => by_elevation.reverse(),
                Self::AreaDescending => right_area.total_cmp(&left_area).then(by_elevation),
                Self::AreaAscending => left_area.total_cmp(&right_area).then(by_elevation),
            }
        });
        rows
    }
}

pub(super) struct Dialog {
    name: String,
    unit_mm: f64,
    units: crate::plan_trace::loads::ForceUnits,
    gravity: f64,
    round_coordinates: bool,
    round_mm: f64,
    merge_tolerance_mm: f64,
    vertical_tolerance_mm: f64,
    raw: Option<Arc<lir_asp_parser::Model>>,
    stamps: Option<Arc<lir_asp_parser::stamp_loads::Report>>,
    import_stamps: bool,
    physical: Option<Arc<lir_asp_parser::physical_loads::Report>>,
    preview: Option<Recovered>,
    floor_choices: Option<Vec<(f64, bool, f64)>>,
    floor_sort: FloorSort,
    job: Option<mpsc::Receiver<Event>>,
    error: String,
}
impl App {
    pub(super) fn load_lir_path(&mut self, path: PathBuf, ctx: &egui::Context) {
        let name = path
            .file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned();
        let (tx, rx) = mpsc::channel();
        let ctx = ctx.clone();
        crate::plan_trace::platform::spawn(move || {
            let result = crate::plan_trace::platform::read(&path)
                .map_err(|e| e.to_string())
                .and_then(|b| {
                    let model = lir_asp_parser::read_model(&b)?;
                    let stamps = lir_asp_parser::stamp_loads::read(&b).unwrap_or_else(|error| {
                        lir_asp_parser::stamp_loads::Report {
                            stamps: vec![],
                            notes: vec![format!("Нагрузки LIR не прочитаны: {error}")],
                        }
                    });
                    let physical =
                        lir_asp_parser::physical_loads::read_uniform(&b).unwrap_or_else(|error| {
                            lir_asp_parser::physical_loads::Report::empty(format!(
                                "Физические нагрузки LIR не прочитаны: {error}"
                            ))
                        });
                    Ok((model, stamps, physical))
                });
            let _ = tx.send(Event::Parsed(result));
            ctx.request_repaint();
        });
        self.lir_dialog = Some(Dialog {
            name,
            unit_mm: 1000.,
            units: crate::plan_trace::loads::ForceUnits::TonForce,
            gravity: self.gravity,
            round_coordinates: false,
            round_mm: 10.,
            merge_tolerance_mm: 1.,
            vertical_tolerance_mm: 5.,
            raw: None,
            stamps: None,
            import_stamps: true,
            physical: None,
            preview: None,
            floor_choices: None,
            floor_sort: FloorSort::default(),
            job: Some(rx),
            error: String::new(),
        });
    }
    pub(super) fn lir_import_button(&mut self, ui: &mut egui::Ui) {
        if ui
            .add_enabled(
                self.lir_dialog.is_none()
                    && self.mesh_job.is_none()
                    && self.analysis_job.is_none()
                    && self.alternative_job.is_none(),
                egui::Button::new("Импорт *.LIR"),
            )
            .on_hover_text("Восстановить этажи, объекты и жёсткости из LIR без ASP. Поддержанные варианты формата распознаются автоматически.")
            .clicked()
        {
            if let Some(path) = rfd::FileDialog::new()
                .add_filter("LIR", &["lir"])
                .pick_file()
            {
                self.load_lir_path(path, ui.ctx());
            }
        }
    }
    pub(super) fn accept_lir(&mut self, recovered: Recovered) {
        let mut next = App::default();
        next.saved_project_state = Some(vec![]);
        next.gravity = recovered.source.gravity;
        next.load_units = recovered.source.force_units;
        next.contours = recovered.contours;
        next.building = recovered.building;
        if !recovered.load_cases.is_empty() {
            next.load_cases = recovered.load_cases;
            next.active_load = next.load_cases[0].id;
            next.next_load = next.load_cases.iter().map(|c| c.id).max().unwrap() + 1;
            next.combinations = vec![crate::plan_trace::load_combinations::Combination::new(
                1,
                next.load_cases.iter().map(|c| c.id),
            )];
        }
        next.stiffnesses = recovered.library;
        next.next_id = next.contours.iter().map(|c| c.id).max().unwrap_or(0) + 1;
        next.support_height.enabled =
            next.building.enabled || next.contours.iter().any(|c| c.kind == Kind::Support);
        next.support_height.automatic_building_bottom = false;
        next.support_height.automatic_building_top = false;
        next.show_results = false;
        next.show_result_values = false;
        next.show_loads = false;
        next.show_load_values = false;
        next.spatial.enabled = next.support_height.enabled;
        next.fit = true;
        next.lir_source = Some(recovered.source);
        next.sync_stiffness_sources();
        next.status = format!(
            "LIR: {} этажей, {} объектов. Импортировано нагрузок: {}. Показ нагрузок выключен.",
            next.building.floors.len(),
            next.contours.len(),
            next.contours.iter().filter(|c| c.kind.is_load()).count()
        );
        *self = next;
    }
    pub(super) fn lir_import_window(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.lir_dialog.take() else {
            return;
        };
        if let Some(rx) = &dialog.job {
            match rx.try_recv() {
                Ok(event) => {
                    dialog.job = None;
                    match event {
                        Event::Parsed(Ok((raw, stamps, physical))) => {
                            dialog.raw = Some(Arc::new(raw));
                            dialog.stamps = Some(Arc::new(stamps));
                            dialog.physical = Some(Arc::new(physical));
                        }
                        Event::Recovered(Ok(preview)) => {
                            if dialog.floor_choices.is_none() {
                                let selected = preview
                                    .building
                                    .floors
                                    .iter()
                                    .map(|f| (f.elevation_mm * 100.).round() as i64)
                                    .collect::<BTreeSet<_>>();
                                let areas = floor_plate_areas(&preview);
                                dialog.floor_choices = Some(
                                    preview
                                        .candidate_levels
                                        .iter()
                                        .map(|&z| {
                                            let key = (z * 100.).round() as i64;
                                            (
                                                z,
                                                selected.contains(&key),
                                                areas.get(&key).copied().unwrap_or(0.),
                                            )
                                        })
                                        .collect(),
                                );
                            }
                            dialog.preview = Some(preview);
                        }
                        Event::Parsed(Err(e)) | Event::Recovered(Err(e)) => dialog.error = e,
                    }
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    dialog.job = None;
                    dialog.error = "Поток импорта прерван".into();
                }
                Err(mpsc::TryRecvError::Empty) => {}
            }
        }
        let mut close = false;
        let mut accept = false;
        let response = egui::Modal::new(egui::Id::new("lir_reconstruction")).show(ctx,|ui| {
            let width=(ctx.screen_rect().width()-64.).clamp(220.,720.);
            ui.set_width(width);
            ui.horizontal(|ui| {
                ui.heading("Импорт LIR");
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center),|ui| {
                    if ui.button("Закрыть").clicked(){close=true;}
                });
            });
            if dialog.job.is_some(){ui.horizontal(|ui|{ui.spinner();ui.label("Чтение и восстановление геометрии…");});}
            else if dialog.preview.is_some(){ui.colored_label(Color32::LIGHT_BLUE,"Восстановление завершено. Проверьте отчёт и нажмите «Создать схему».");}
            else if !dialog.error.is_empty(){ui.colored_label(Color32::LIGHT_RED,"Восстановление не завершено. Подробности ниже.");}
            else{ui.label("Проверьте параметры и нажмите «Восстановить объекты».");}
            ui.separator();
            egui::ScrollArea::vertical().id_salt("lir_import_body").max_height((ctx.screen_rect().height()-220.).clamp(80.,560.)).auto_shrink([false,true]).show(ui,|ui| {
                ui.label(&dialog.name);
                if let Some(raw)=&dialog.raw {
                    ui.label(format!("Узлов: {} · распознанных КЭ: {} · отключённых: {}",raw.nodes.len(),raw.elements.len(),raw.inactive_elements));
                    let skipped:usize=raw.skipped_unknown.values().sum();
                    if skipped>0 {ui.label(format!("Пропущено КЭ с нераспознанным типом: {skipped}"));}
                }
                ui.add_enabled_ui(dialog.job.is_none(),|ui| {
                    let old=(dialog.unit_mm,dialog.units,dialog.gravity,dialog.round_coordinates,dialog.round_mm,dialog.merge_tolerance_mm,dialog.vertical_tolerance_mm);
                    ui.horizontal_wrapped(|ui| {
                        egui::ComboBox::from_label("Координаты").selected_text(if dialog.unit_mm==1000. {"м"}else if dialog.unit_mm==10. {"см"}else{"мм"}).show_ui(ui,|ui|{for(label,scale)in[("м",1000.),("см",10.),("мм",1.)]{ui.selectable_value(&mut dialog.unit_mm,scale,label);}});
                        egui::ComboBox::from_label("Усилия LIR").selected_text(dialog.units.label()).show_ui(ui,|ui|{for units in [crate::plan_trace::loads::ForceUnits::TonForce,crate::plan_trace::loads::ForceUnits::KiloNewton]{ui.selectable_value(&mut dialog.units,units,units.label());}});
                        ui.label("g");ui.add(egui::DragValue::new(&mut dialog.gravity).range(0.01..=100.));
                    });
                    ui.horizontal_wrapped(|ui|{ui.checkbox(&mut dialog.round_coordinates,"Округлить координаты");ui.add_enabled(dialog.round_coordinates,egui::DragValue::new(&mut dialog.round_mm).range(0.01..=1e6).suffix(" мм"));});
                    ui.horizontal(|ui|{ui.label("Допуск объединения, мм");ui.add(egui::DragValue::new(&mut dialog.merge_tolerance_mm).range(0.01..=100.).speed(0.1));});
                    ui.horizontal(|ui|{ui.label("Отклонение от вертикали, мм");ui.add(egui::DragValue::new(&mut dialog.vertical_tolerance_mm).range(0.05..=1000.).speed(0.1));}).response.on_hover_text("Допустимое смещение вертикальных конструкций: колонны выпрямляются по нижней точке, стены проецируются на вертикальную плоскость. Округление координат не требуется.");
                    if old!=(dialog.unit_mm,dialog.units,dialog.gravity,dialog.round_coordinates,dialog.round_mm,dialog.merge_tolerance_mm,dialog.vertical_tolerance_mm){dialog.preview=None;dialog.floor_choices=None;}
                    let mut invalid_floor_selection = None;
                    if let Some(choices)=&mut dialog.floor_choices {
                        let mut changed=false;
                        ui.collapsing("Этажи",|ui| {
                            ui.small("Отметьте уровни этажей. Объекты ниже первой выбранной отметки попадут на первый этаж со своим смещением.");
                            dialog.floor_sort.controls(ui);
                            for i in dialog.floor_sort.rows(choices) {
                                let (z, enabled, area) = &mut choices[i];
                                let fill = if *enabled {
                                    ui.visuals().selection.bg_fill.gamma_multiply(0.6)
                                } else {
                                    Color32::TRANSPARENT
                                };
                                let width = (ui.available_width() - 8.).max(0.);
                                changed |= ui.push_id(i, |ui| {
                                    egui::Frame::new()
                                        .fill(fill)
                                        .inner_margin(egui::Margin::symmetric(4, 1))
                                        .show(ui, |ui| {
                                            ui.set_min_width(width);
                                            ui.checkbox(enabled, format!("Z = {z:.2} мм · {area:.2} м² плит"))
                                        }).inner.changed()
                                }).inner;
                            }
                        });
                        if changed { dialog.preview=None; }
                        invalid_floor_selection = validate_floor_gap(choices.iter().filter(|row| row.1).map(|row| (row.0 * 100.).round() as i64)).err();
                    }
                    if let Some(error) = &invalid_floor_selection {
                        ui.colored_label(Color32::LIGHT_RED, error);
                    }
                    ui.small("Единицы LIR задаются явно. Толщина распознанных жёсткостей типа 36 хранится в см; E — в выбранных единицах усилий/м².");
                    if ui.checkbox(&mut dialog.import_stamps,"Импортировать нагрузки и загружения").changed(){dialog.preview=None;}
                    if let Some(stamps)=&dialog.stamps {ui.small(format!("Найдено исходных штампов: {}",stamps.stamps.len()));}
                    if let Some(physical)=&dialog.physical {ui.small(format!("Поддержанных физических записей: {} · загружений: {}",physical.records.len(),physical.cases.len()));}
                    if dialog.raw.is_some()&&ui.add_enabled(invalid_floor_selection.is_none(), egui::Button::new("Восстановить объекты")).clicked(){
                        let raw=dialog.raw.as_ref().unwrap().clone();let name=dialog.name.clone();let unit=dialog.unit_mm;let units=dialog.units;let g=dialog.gravity;let rounding=dialog.round_coordinates.then_some(dialog.round_mm);let selected=dialog.floor_choices.as_ref().map(|v|v.iter().filter(|v|v.1).map(|v|v.0).collect());
                        let tolerance=dialog.merge_tolerance_mm;let vertical_tolerance=dialog.vertical_tolerance_mm;let stamps=dialog.stamps.clone().filter(|_|dialog.import_stamps);let physical=dialog.physical.clone().filter(|_|dialog.import_stamps);
                        let(tx,rx)=mpsc::channel();dialog.job=Some(rx);dialog.preview=None;dialog.error.clear();let ctx=ctx.clone();
                        crate::plan_trace::platform::spawn(move||{let _=tx.send(Event::Recovered(recover_with_vertical_tolerance(&raw,name,unit,units,g,rounding,selected,tolerance,vertical_tolerance).map(|mut result|{if let Some(stamps)=stamps {append_stamp_loads(&mut result,&stamps,rounding);}if let Some(physical)=physical {append_uniform_loads(&mut result,&raw,&physical,rounding);}result})));ctx.request_repaint();});
                    }
                });
                if let Some(preview)=&dialog.preview {
                    ui.separator();
                    ui.label(format!("Будет создано: этажей {} · плит {} · отверстий {} · стен {} · колонн {} · балок {}",preview.building.floors.len(),preview.contours.iter().filter(|c|c.kind==Kind::Slab).count(),preview.contours.iter().filter(|c|c.kind==Kind::Hole).count(),preview.contours.iter().filter(|c|c.kind==Kind::Support).count(),preview.contours.iter().filter(|c|c.kind==Kind::Column).count(),preview.contours.iter().filter(|c|c.kind==Kind::Beam).count()));
                    ui.collapsing("Жёсткости и созданные этажи",|ui| {
                        for f in &preview.building.floors{ui.label(format!("{}: {:.0} мм",f.name,f.elevation_mm));}
                        for e in preview.library.snapshot(&preview.contours).entries.values(){ui.label(format!("{}: {} · E={:.1} МПа · ν={:.3}",e.name,if matches!(e.family,crate::plan_trace::stiffness::Family::Column|crate::plan_trace::stiffness::Family::Beam){format!("b×h={:.0}×{:.0} мм",e.width_mm,e.material.thickness_mm)}else{format!("h={:.1} мм",e.material.thickness_mm)},e.material.e_mpa,e.material.nu));}
                    });
                    if !preview.source.notes.is_empty(){ui.collapsing(format!("Замечания импорта ({})",preview.source.notes.len()),|ui|{for note in &preview.source.notes{ui.label(note);}});}
                    if preview.source.inferred_types>0{ui.small(format!("Тип {} граничных записей подтверждён по однозначным определениям остальных КЭ файла.",preview.source.inferred_types));}
                    ui.label(format!("Нагрузки: {}. Закрепления, КЭ-сетка и результаты не импортируются.",preview.contours.iter().filter(|c|c.kind.is_load()).count()));
                }
                if !dialog.error.is_empty(){ui.colored_label(Color32::LIGHT_RED,&dialog.error);}
            });
            ui.separator();
            // Actions stay outside the report scroll area, regardless of its length.
            ui.horizontal_wrapped(|ui| {
                if dialog.preview.is_some() {
                    if !self.contours.is_empty() && ui.button("Сохранить текущую и создать схему").clicked()&&self.save_project(false){accept=true;}
                    if ui.button(if self.contours.is_empty(){"Создать схему"}else{"Заменить текущую схему"}).clicked(){accept=true;}
                }
                if ui.button("Отмена").clicked(){close=true;}
            });
            if dialog.preview.is_some()&&!self.contours.is_empty(){ui.small("Текущая схема будет заменена.");}
        });
        close |= response.should_close();
        if close {
            accept = false;
        }
        if accept {
            self.accept_lir(dialog.preview.take().unwrap());
        } else if !close {
            self.lir_dialog = Some(dialog);
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn nearby_lir_levels_default_to_larger_plate_area_and_report_manual_conflict() {
        let candidates = BTreeSet::from([0, 100_000, 102_000, 112_000, 300_000]);
        let areas = BTreeMap::from([(100_000, 5.), (102_000, 860.), (112_000, 7.)]);
        let chosen = default_floor_levels(&candidates, &areas);
        assert_eq!(chosen, BTreeSet::from([0, 102_000, 112_000, 300_000]));
        assert!(validate_floor_gap(chosen).is_ok());
        let error = validate_floor_gap(candidates).unwrap_err();
        assert!(error.contains("1000.00 и 1020.00"));
        assert!(validate_floor_gap([]).is_err());
    }

    #[test]
    fn floor_sort_reorders_rows_without_reassigning_elevations_or_checks() {
        let mut choices = [(-1000., true, 0.), (0., true, 25.), (3000., false, 25.)];
        assert_eq!(FloorSort::default().rows(&choices), [0, 1, 2]);
        assert_eq!(FloorSort::ElevationDescending.rows(&choices), [2, 1, 0]);
        assert_eq!(FloorSort::AreaAscending.rows(&choices), [0, 1, 2]);
        let rows = FloorSort::AreaDescending.rows(&choices);
        assert_eq!(rows, [1, 2, 0]);
        assert!(!choices[rows[1]].1);
        assert_eq!(rows[2], 0);
        choices[rows[1]].1 = true;
        assert_eq!(
            choices,
            [(-1000., true, 0.), (0., true, 25.), (3000., true, 25.)]
        );
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_STAMPS control file"]
    fn real_stamp_contours_become_editable_loads_without_physical_duplicates() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_STAMPS").expect("stamp LIR path"));
        let bytes = std::fs::read(path).unwrap();
        let model = lir_asp_parser::read_model(&bytes).unwrap();
        let mut report = lir_asp_parser::stamp_loads::read(&bytes).unwrap();
        // A changed/clipped old projection must not discard the source contour.
        // Also catch a parser regression forgetting the independently stored case.
        for stamp in &mut report.stamps {
            assert_eq!(stamp.attachment_case_id, Some(1));
            assert_eq!(stamp.attachment_direction_code, Some(153));
            stamp.case_id = None;
            stamp.direction_code = None;
            stamp.physical_resultant = Some(0.);
        }
        let mut recovered = recover(
            &model,
            "stamps.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        append_stamp_loads(&mut recovered, &report, None);
        let loads = recovered
            .contours
            .iter()
            .filter(|c| c.kind.is_load())
            .collect::<Vec<_>>();
        assert_eq!(loads.len(), 2);
        let area = loads.iter().find(|c| c.kind == Kind::LoadArea).unwrap();
        assert_eq!(area.points.len(), 3);
        assert!((super::area(&area.points).abs() / 1e6 - 1.86).abs() < 1e-9);
        assert_eq!(area.load.as_ref().unwrap().magnitude.value, 1.5);
        let line = loads.iter().find(|c| c.kind == Kind::LoadLine).unwrap();
        assert_eq!(line.points.len(), 2);
        assert!((line.points[1][0] - 2000.).abs() < 1e-9);
        assert_eq!(line.load.as_ref().unwrap().magnitude.value, 2.);
        assert!(loads.iter().all(|c| c.load.as_ref().unwrap().case_id == 1));
        assert_eq!(recovered.load_cases.len(), 1);
        let mut app = App::default();
        app.accept_lir(recovered);
        assert_eq!(app.load_cases[0].id, 1);
        assert_eq!(app.combinations[0].coefficients[&1], 1.);
        assert_eq!(app.contours.iter().filter(|c| c.kind.is_load()).count(), 2);
    }
    #[test]
    fn physical_pressures_merge_and_preserve_cases_without_stamp_duplicates() {
        use lir_asp_parser::physical_loads::{Case, Record, Report, TargetKind, TargetRecord};
        use lir_asp_parser::{Model, ModelElement};
        let model = Model {
            foundations: BTreeMap::new(),
            nodes: vec![[0., 0., 0.], [1., 0., 0.], [1., 1., 0.], [0., 1., 0.]],
            elements: vec![
                ModelElement {
                    id: 1,
                    definition: 2,
                    kind: 42,
                    nodes: vec![1, 2, 3],
                },
                ModelElement {
                    id: 2,
                    definition: 2,
                    kind: 42,
                    nodes: vec![1, 3, 4],
                },
            ],
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: vec![],
            section_notes: vec![],
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        let mut recovered = recover(
            &model,
            "uniform.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let record = |id, code, value| TargetRecord {
            target_kind: TargetKind::Element,
            target_id: id,
            record: Record {
                offset: 0,
                tag: 12,
                code,
                case_id: 2,
                group_raw: 0,
                payload: vec![value],
            },
        };
        let report = Report {
            cases: vec![
                Case {
                    id: 2,
                    name: "Полезная".into(),
                },
                Case {
                    id: 7,
                    name: "Пустое".into(),
                },
            ],
            records: vec![record(1, 63, 1.5), record(2, 63, 1.5), record(1, 153, 50.)],
            ..Default::default()
        };
        append_uniform_loads(&mut recovered, &model, &report, None);
        let loads = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::LoadArea)
            .collect::<Vec<_>>();
        assert_eq!(loads.len(), 1);
        assert!((area(&loads[0].points).abs() - 1e6).abs() < 1e-5);
        assert_eq!(loads[0].load.as_ref().unwrap().magnitude.value, 1.5);
        assert_eq!(loads[0].load.as_ref().unwrap().case_id, 2);
        assert_eq!(recovered.load_cases.len(), 2);
        assert_eq!(recovered.load_cases[0].name, "Полезная");
    }
    #[test]
    fn straight_imported_runs_collapse_without_losing_corners() {
        // Many almost collinear vertices on a diagonal, followed by a genuine notch.
        let mut ring = (0..=200)
            .map(|i| {
                [
                    i as f64 * 10.,
                    i as f64 * 3. + if i % 2 == 0 { 0.01 } else { -0.01 },
                ]
            })
            .collect::<Vec<_>>();
        ring.extend([
            [2000., 1000.],
            [1500., 1000.],
            [1500., 900.],
            [1400., 900.],
            [1400., 1000.],
            [0., 1000.],
        ]);
        let simplified = simple(ring.clone());
        assert_eq!(simplified.len(), 8);
        assert!(simplified.contains(&[1500., 900.]));
        assert!(simplified.contains(&[1400., 900.]));
        ring.reverse();
        let reversed = simple(ring);
        assert_eq!(reversed.len(), 8);
        assert!(area(&simplified) * area(&reversed) < 0.);
    }
    #[test]
    #[ignore = "requires MINI_FEM_LIR_V15"]
    fn real_v15_names_and_load_display_after_import() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_V15").expect("V15 LIR path"));
        let bytes = std::fs::read(&path).unwrap();
        let now = std::time::Instant::now();
        let model = lir_asp_parser::read_model(&bytes).unwrap();
        let stamps = lir_asp_parser::stamp_loads::read(&bytes).unwrap();
        let physical = lir_asp_parser::physical_loads::read_uniform(&bytes).unwrap();
        eprintln!("LIR parse: {:?}", now.elapsed());
        assert_eq!(model.sections.len(), 99);
        let now = std::time::Instant::now();
        let mut recovered = recover(
            &model,
            path.file_name().unwrap().to_string_lossy().into_owned(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        append_stamp_loads(&mut recovered, &stamps, None);
        append_uniform_loads(&mut recovered, &model, &physical, None);
        eprintln!("LIR reconstruction: {:?}", now.elapsed());
        let column_members = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column)
            .flat_map(|c| recovered.source.contour_elements[&c.id].iter().copied())
            .collect::<BTreeSet<_>>();
        assert_eq!(column_members.len(), 268);
        let typical_floor = recovered
            .building
            .floors
            .iter()
            .find(|f| f.elevation_mm == 9120.)
            .unwrap();
        let typical_slabs = recovered
            .contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab
                    && recovered.building.owner(c.id) == typical_floor.id
                    && c.placement.slab.offset_mm.abs() < 1e-5
            })
            .collect::<Vec<_>>();
        assert_eq!(typical_slabs.len(), 1);
        assert_eq!(
            recovered.source.contour_elements[&typical_slabs[0].id].len(),
            8830
        );
        let typical_holes = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Hole && c.parent == Some(typical_slabs[0].id))
            .count();
        assert_eq!(typical_holes, 53);
        eprintln!(
            "Z=9120 mm: {} slab, {typical_holes} openings",
            typical_slabs.len()
        );
        eprintln!(
            "Recovered geometry: {} slabs, {} openings, {} columns, {} load contours",
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Slab)
                .count(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Hole)
                .count(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Column)
                .count(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind.is_load())
                .count(),
        );
        let mut app = App::default();
        app.accept_lir(recovered);
        assert_eq!(app.load_cases.len(), 13);
        assert_eq!(app.load_cases[0].name, "Собственный вес (1.1)");
        assert_eq!(app.load_cases[4].name, "Перегородки (1.2)");
        assert_eq!(app.load_cases[12].name, "Подъездные пути/проезды");
        assert!(!app.show_loads && !app.show_load_values);
        assert!(app.contours.iter().any(|c| c.kind == Kind::LoadArea));
        let ctx = egui::Context::default();
        let draw = |app: &mut App| {
            let start = std::time::Instant::now();
            app.sync_building_objects();
            app.start_load_fill_frame();
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1100., 750.))),
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| app.spatial_canvas(ui));
                },
            );
            start.elapsed()
        };
        eprintln!("First scene: {:?}", draw(&mut app));
        assert!(app.spatial.cache.is_some());
        app.show_loads = true;
        app.active_load = app.load_cases[4].id;
        eprintln!("Load overlay: {:?}", draw(&mut app));
        assert!(!app.spatial_load_sources().is_empty());
        eprintln!("Warm load overlay: {:?}", draw(&mut app));
        app.show_load_values = true;
        eprintln!("Load labels: {:?}", draw(&mut app));
        app.show_loads = false;
        eprintln!("Load overlay hidden: {:?}", draw(&mut app));
        assert!(app.spatial_load_sources().is_empty());
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_C1 control file"]
    fn real_c1_keeps_the_point_force_and_opens_in_3d() {
        let path = std::env::var_os("MINI_FEM_LIR_C1").unwrap();
        let bytes = std::fs::read(path).unwrap();
        let model = lir_asp_parser::read_model(&bytes).unwrap();
        let report = lir_asp_parser::physical_loads::read_uniform(&bytes).unwrap();
        let mut recovered = recover(
            &model,
            "C1.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        append_uniform_loads(&mut recovered, &model, &report, None);
        let mut app = App::default();
        app.accept_lir(recovered);
        assert_eq!(app.building.floors.len(), 1);
        assert!(app.building.enabled && app.spatial.enabled);
        let point = app
            .contours
            .iter()
            .find(|c| c.kind == Kind::LoadPoint && c.points == [[6000., 4500.]])
            .unwrap();
        let load = point.load.as_ref().unwrap();
        assert_eq!(load.case_id, 1);
        assert_eq!(load.magnitude.value, 20.);
        assert_eq!(app.source_heights(point), [0.]);
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_MODEL"]
    fn reconstruct_real_lir_and_save_editable_project() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_MODEL").expect("LIR path"));
        let bytes = std::fs::read(&path).unwrap();
        let model = lir_asp_parser::read_model(&bytes).unwrap();
        let recovered = recover_options(
            &model,
            path.file_name().unwrap().to_string_lossy().into_owned(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
            None,
            None,
            1.,
        )
        .unwrap();
        eprintln!(
            "floors={} slabs={} holes={} walls={} source_ids={}",
            recovered.building.floors.len(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Slab)
                .count(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Hole)
                .count(),
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Support)
                .count(),
            recovered
                .source
                .contour_elements
                .values()
                .map(Vec::len)
                .sum::<usize>()
        );
        let retained = recovered
            .source
            .contour_elements
            .values()
            .flatten()
            .copied()
            .collect::<std::collections::BTreeSet<_>>();
        let known = model
            .elements
            .iter()
            .map(|e| e.id)
            .collect::<std::collections::BTreeSet<_>>();
        assert!(retained.is_subset(&known));
        let mut expected = BTreeMap::<(i64, u16), f64>::new();
        let by_id = model
            .elements
            .iter()
            .map(|e| (e.id, e))
            .collect::<BTreeMap<_, _>>();
        let planes = &recovered.plate_areas;
        let section = |e: &lir_asp_parser::ModelElement| {
            model
                .section_assignments
                .get(e.id as usize - 1)
                .copied()
                .filter(|id| model.sections.get(id).is_some_and(|s| s.kind == 36))
                .unwrap_or(0)
        };
        for e in model
            .elements
            .iter()
            .filter(|e| matches!(e.kind, 41 | 42 | 44))
        {
            let mut p = e
                .nodes
                .iter()
                .map(|n| model.nodes[*n as usize - 1].map(|v| v * 1000.))
                .collect::<Vec<_>>();
            let z0 = p.iter().map(|p| p[2]).fold(f64::INFINITY, f64::min);
            let z1 = p.iter().map(|p| p[2]).fold(f64::NEG_INFINITY, f64::max);
            let Some(plane) = slab_plane(z0, z1, planes) else {
                continue;
            };
            if matches!(e.kind, 41 | 44) {
                p.swap(2, 3);
            }
            let size = area(&p.iter().map(|p| [p[0], p[1]]).collect::<Vec<_>>()).abs();
            if size < 1. {
                continue;
            }
            assert!(
                retained.contains(&e.id),
                "horizontal element {} was lost",
                e.id
            );
            let section = section(e);
            *expected.entry((plane, section)).or_default() += size;
        }
        let mut actual = BTreeMap::<(i64, u16), f64>::new();
        for c in recovered.contours.iter().filter(|c| c.kind == Kind::Slab) {
            let source = recovered.source.contour_elements[&c.id]
                .first()
                .expect("slab must own source elements");
            let e = by_id[source];
            let section = section(e);
            let z = c
                .placement
                .slab
                .z(recovered.building.extent(recovered.building.owner(c.id)));
            let holes = recovered
                .contours
                .iter()
                .filter(|h| h.kind == Kind::Hole && h.parent == Some(c.id))
                .map(|h| area(&h.points).abs())
                .sum::<f64>();
            *actual
                .entry(((z * 100.).round() as i64, section))
                .or_default() += area(&c.points).abs() - holes;
        }
        for (key, wanted) in expected {
            let got = actual.get(&key).copied().unwrap_or(0.);
            eprintln!(
                "plate group {:?}: source={}m2 restored={}m2",
                key,
                wanted / 1e6,
                got / 1e6
            );
            assert!(
                (got - wanted).abs() < wanted * 1e-4 + 10.,
                "plate area mismatch {:?}",
                key
            );
        }
        let mut app = App::default();
        app.accept_lir(recovered);
        app.write_lir_preview(std::path::Path::new("target/lir_import_recovered.slab.zip"))
            .unwrap();
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_DFM control file"]
    fn dfm_import_assigns_rectangular_beams_from_the_mixed_section_table() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_DFM").expect("DFM LIR path"));
        let model = lir_asp_parser::read_model(&std::fs::read(&path).unwrap()).unwrap();
        assert_eq!(model.sections.len(), 51);
        assert_eq!(model.section_assignments.len(), 663_060);
        assert_eq!(model.section_assignments[0], 2);
        assert_eq!(model.sections[&3].parameters[2], 0.5);
        let imported = recover(
            &model,
            path.file_name().unwrap().to_string_lossy().into_owned(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let beam = imported
            .contours
            .iter()
            .find(|c| c.kind == Kind::Beam && imported.source.contour_elements[&c.id].contains(&1))
            .expect("beam from LIR element 1");
        let section = &imported.library.entries[&imported.library.assignments[&beam.id]];
        assert_eq!(section.name, "БАЛКА_В30 · LIR 2");
        assert_eq!(
            [section.width_mm, section.material.thickness_mm],
            [300., 500.]
        );
        assert_eq!(section.material.e_mpa, 6500.);
        assert_eq!(section.material.weight.value, 2.5);
        let mut counts = BTreeMap::<_, usize>::new();
        for beam in imported.contours.iter().filter(|c| c.kind == Kind::Beam) {
            let section_id = imported.library.assignments[&beam.id];
            let section = &imported.library.entries[&section_id];
            *counts.entry(section.name.as_str()).or_default() += 1;
        }
        assert!(counts.contains_key("ФАС · LIR 1") && counts.contains_key("ФАС · LIR 14"));
        eprintln!("Beam contours by LIR section: {counts:?}");
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_MIG6 control file"]
    fn mig6_import_retains_intermediate_plate_elevation() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_MIG6").expect("MiG-6 LIR path"));
        let model = lir_asp_parser::read_model(&std::fs::read(&path).unwrap()).unwrap();
        let recovered = recover(
            &model,
            path.file_name().unwrap().to_string_lossy().into_owned(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let storeys = &recovered.building;
        storeys.validate().unwrap();
        assert!(recovered.candidate_levels.contains(&11730.));
        assert!(recovered.candidate_levels.contains(&11750.));
        assert!(storeys.floors.iter().any(|f| f.elevation_mm == 11730.));
        assert!(!storeys.floors.iter().any(|f| f.elevation_mm == 11750.));
        let intermediate = recovered
            .contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab
                    && (c.placement.slab.z(storeys.extent(storeys.owner(c.id))) - 11750.).abs()
                        < 0.005
            })
            .count();
        assert!(intermediate > 0);
        let walls = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Support);
        let wall_count = walls.clone().count();
        let narrow = walls
            .filter(|c| {
                let [a, b] = [c.points[0], c.points[1]];
                (b[0] - a[0]).hypot(b[1] - a[1]) < 10.
            })
            .count();
        assert!(
            wall_count < 12_000 && narrow < 200,
            "wall count {wall_count}, narrower than 10 mm: {narrow}"
        );
        let floor = storeys
            .floors
            .iter()
            .find(|f| f.elevation_mm == 76530.)
            .unwrap();
        let wall_line = recovered
            .contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Support
                    && storeys.owner(c.id) == floor.id
                    && c.points.iter().all(|p| (p[1] - 8200.).abs() < 0.01)
            })
            .collect::<Vec<_>>();
        for [left, right] in [[10950., 12150.], [22000., 23200.], [29000., 30200.]] {
            let mid = (left + right) * 0.5;
            let over_opening = wall_line
                .iter()
                .filter(|c| c.points[0][0] < mid && mid < c.points[1][0])
                .map(|c| c.placement.wall(storeys.extent(floor.id)))
                .collect::<Vec<_>>();
            assert!(
                matches!(over_opening.as_slice(), [[bottom, top]]
                    if (*bottom - 77030.).abs() < 0.05 && (*top - 78330.).abs() < 0.05),
                "opening X={left}..{right}: {over_opening:?}"
            );
        }
        eprintln!(
            "{} candidates, {} storeys, {} contours; {intermediate} slabs at the intermediate Z=11750 mm; {wall_count} walls, {narrow} narrower than 10 mm",
            recovered.candidate_levels.len(),
            storeys.floors.len(),
            recovered.contours.len(),
        );
        for note in recovered
            .source
            .notes
            .iter()
            .filter(|s| s.contains("вырожденных фрагментов стен"))
        {
            eprintln!("{note}");
        }
        let invalid = recovered
            .contours
            .iter()
            .filter_map(|c| {
                model::validate_candidate(&recovered.contours, c)
                    .err()
                    .filter(|error| {
                        c.kind == Kind::Support
                            || error != "Граница пересекает или касается сама себя"
                    })
                    .map(|error| (c.id, c.kind, error))
            })
            .take(10)
            .collect::<Vec<_>>();
        assert!(invalid.is_empty(), "invalid imported contours: {invalid:?}");
        let mut app = App::default();
        app.accept_lir(recovered);
        app.write_lir_preview(std::path::Path::new(
            "target/qa-mig6-recovered-1mm.slab.zip",
        ))
        .unwrap();
    }
}

#[cfg(test)]
mod touching_boundaries_tests {
    use super::*;
    #[test]
    fn beam_segments_merge_keep_bends_and_stop_at_junctions() {
        let segments = vec![
            ([[0., 0.], [1000., 0.]], 10),
            ([[1000., 0.], [2000., 0.]], 11),
            ([[2000., 0.], [3000., 0.]], 12),
            ([[2000., 0.], [2000., 1000.]], 13),
            ([[2000., 1000.], [3000., 1000.]], 14),
        ];
        let chains = beam_chains(segments, 1.);
        assert_eq!(chains.len(), 3);
        let first = chains.iter().find(|(_, ids, _)| ids.contains(&10)).unwrap();
        assert_eq!(first.0, vec![[0., 0.], [2000., 0.]]);
        assert_eq!(first.1, vec![10, 11]);
        let bent = chains.iter().find(|(_, ids, _)| ids.contains(&13)).unwrap();
        assert_eq!(bent.0.len(), 3);
        assert!(bent.0.contains(&[2000., 1000.]));
        let ids = chains
            .iter()
            .flat_map(|(_, ids, _)| ids.iter().copied())
            .collect::<std::collections::BTreeSet<_>>();
        assert_eq!(ids, (10..=14).collect());
    }
    #[test]
    fn millimetre_tolerance_removes_source_coordinate_staircase() {
        let mut path = (0..50)
            .map(|i| {
                [
                    i as f64 * 300.,
                    i as f64 * 90. + if i % 2 == 0 { 0.4 } else { -0.4 },
                ]
            })
            .collect::<Vec<_>>();
        path.extend([[14700., 10000.], [0., 10000.]]);
        let fine = simple_with(path.clone(), 0.1);
        let result = simple_with(path, 1.);
        assert!(fine.len() > 20);
        assert_eq!(result.len(), 4);
    }
    #[test]
    fn plane_fit_uses_all_original_cells_without_chained_drift() {
        let at = |t: f64, n: f64, z: f64| [0.6 * t - 0.8 * n, 0.8 * t + 0.6 * n, z];
        let first = vec![
            at(0., 0.4, 0.),
            at(300., -0.4, 0.),
            at(300., 0.4, 300.),
            at(0., -0.4, 300.),
        ];
        let second = vec![
            at(300., -0.4, 0.),
            at(600., 0.4, 0.),
            at(600., -0.4, 300.),
            at(300., 0.4, 300.),
        ];
        let (direction, offset) = wall_plane(&first);
        let wall = Wall {
            direction,
            offset,
            source: vec![(1, first)],
            ..Default::default()
        };
        let (d, _) = wall_plane(&second);
        assert!(wall.contains_plane(&second, d, 1.));
        let shifted = second
            .iter()
            .map(|p| [p[0] - 8., p[1] + 6., p[2]])
            .collect::<Vec<_>>();
        assert!(!wall.contains_plane(&shifted, d, 1.));
    }
    #[test]
    fn wall_plane_rejects_distant_parallel_panel_that_would_rotate_the_fit() {
        let panel = |x, y| {
            [
                [x, y, 0.],
                [x, y + 400., 0.],
                [x, y + 400., 300.],
                [x, y, 300.],
            ]
        };
        let first = panel(0., 0.);
        let second = panel(100., 10000.);
        let (direction, offset) = wall_plane(&first);
        let wall = Wall {
            direction,
            offset,
            source: vec![(1, first.to_vec())],
            ..Default::default()
        };
        assert!(!wall.contains_plane(&second, direction, 5.));
        assert!(wall.contains_plane(&panel(0., 10000.), direction, 5.));
    }
    #[test]
    fn oblique_wall_plane_accepts_float_noise_but_not_nearby_parallel_wall() {
        let d = [0.6, 0.8];
        let wall = Wall {
            direction: d,
            offset: 1200.,
            ..Default::default()
        };
        let at = |t: f64, normal: f64, z: f64| {
            [
                d[0] * t - d[1] * (1200. + normal),
                d[1] * t + d[0] * (1200. + normal),
                z,
            ]
        };
        let cell = [
            at(5000., 0.002, 0.),
            at(5500., -0.002, 0.),
            at(5500., -0.002, 500.),
            at(5000., 0.002, 500.),
        ];
        // Separate element normals/offsets have different exact quantized keys.
        let noisy_direction = [d[0] + 0.0000064, d[1] - 0.0000048];
        assert!(wall.contains_plane(&cell, noisy_direction, 0.1));
        let other = cell.map(|mut p| {
            p[0] -= d[1] * 2.;
            p[1] += d[0] * 2.;
            p
        });
        assert!(!wall.contains_plane(&other, d, 0.1));
        assert!(!wall.contains_plane(&cell, [0.8, -0.6], 0.1));
    }
    #[test]
    fn selected_storeys_keep_intermediate_slabs_at_original_elevations() {
        use lir_asp_parser::{Model, ModelElement};
        let mut model = Model {
            foundations: BTreeMap::new(),
            nodes: vec![],
            elements: vec![],
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: vec![],
            section_notes: vec![],
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        for (i, z) in [0., 1500., 3000., 4500., 6000.].into_iter().enumerate() {
            let n = model.nodes.len() as u32;
            model.nodes.extend([
                [0., 0., z],
                [1000., 0., z],
                [0., 1000., z],
                [1000., 1000., z],
            ]);
            model.elements.push(ModelElement {
                id: i as u32 + 1,
                definition: 1,
                kind: 44,
                nodes: vec![n + 1, n + 2, n + 3, n + 4],
            });
            if i < 3 {
                model.foundations.insert(
                    i as u32 + 1,
                    lir_asp_parser::FoundationAssignment {
                        c1: 500. + 100. * i as f64,
                        c2: 0.,
                        rz_text: String::new(),
                    },
                );
            }
        }
        let result = recover_options(
            &model,
            "steps.lir".into(),
            1.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
            None,
            Some(vec![0., 3000., 6000.]),
            1.,
        )
        .unwrap();
        assert_eq!(result.building.floors.len(), 3);
        let areas = floor_plate_areas(&result);
        assert_eq!(areas.len(), 5);
        assert_eq!(areas.get(&150_000), Some(&1.));
        let slabs = result
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Slab)
            .collect::<Vec<_>>();
        assert_eq!(slabs.len(), 5);
        for c in slabs {
            let source = result.source.contour_elements[&c.id][0] as usize - 1;
            let z = source as f64 * 1500.;
            let floor = result.building.floor(result.building.owner(c.id));
            assert_eq!(floor.elevation_mm + c.placement.slab.offset_mm, z);
            assert_eq!(floor.elevation_mm, (z / 3000.).floor() * 3000.);
            assert_eq!(c.use_foundation, source < 3);
        }
        let foundations = result
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Foundation)
            .collect::<Vec<_>>();
        assert_eq!(foundations.len(), 3);
        for (index, c) in foundations.into_iter().enumerate() {
            let floor = result.building.owner(c.id);
            assert_eq!(
                c.placement.slab.z(result.building.extent(floor)),
                index as f64 * 1500.
            );
            assert_eq!(
                c.foundation.as_ref().unwrap().interior[0].c1.value,
                500. + 100. * index as f64
            );
        }
    }
    fn model_with_beam() -> lir_asp_parser::Model {
        use lir_asp_parser::{Model, ModelElement, SectionDefinition};
        let section = |id, kind, p| SectionDefinition {
            id,
            kind,
            flag: 1,
            name: format!("section{id}"),
            color: [1, 2, 3],
            parameters: p,
        };
        Model {
            nodes: vec![
                [0., 0., 0.],
                [1., 0., 0.],
                [0., 1., 0.],
                [1., 1., 0.],
                [0., 0., 0.5],
                [1., 0., 0.5],
            ],
            elements: vec![
                ModelElement {
                    id: 1,
                    definition: 2,
                    kind: 44,
                    nodes: vec![1, 2, 3, 4],
                },
                ModelElement {
                    id: 2,
                    definition: 0,
                    kind: 10,
                    nodes: vec![5, 6],
                },
            ],
            properties: Default::default(),
            sections: BTreeMap::from([
                (1, section(1, 36, [2.5, 3000000., 0.2, 20., 0., 0.])),
                (2, section(2, 0, [2.5, 3000000., 30., 60., 0., 0.])),
            ]),
            section_assignments: vec![1, 2],
            section_notes: vec![],
            foundations: BTreeMap::new(),
            bar_angles_deg: BTreeMap::from([(2, 30.)]),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        }
    }

    #[test]
    fn horizontal_bars_become_beams_with_actual_height_and_rectangular_section() {
        let result = recover(
            &model_with_beam(),
            "beams.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let beam = result
            .contours
            .iter()
            .find(|c| c.kind == Kind::Beam)
            .unwrap();
        assert!(beam.support_open);
        assert_eq!(beam.points, vec![[0., 0.], [1000., 0.]]);
        assert_eq!(beam.placement.slab.offset_mm, 500.);
        assert!((beam.column_angle - 30_f64.to_radians()).abs() < 1e-7);
        assert!(result.building.enabled);
        let entry = &result.library.entries[&result.library.assignments[&beam.id]];
        assert_eq!(entry.family, crate::plan_trace::stiffness::Family::Beam);
        assert_eq!(entry.width_mm, 300.);
        assert_eq!(entry.material.thickness_mm, 600.);
        assert_eq!(result.source.contour_elements[&beam.id], vec![2]);
    }

    #[test]
    fn unchecked_lowest_lir_level_keeps_pit_wall_and_stamp_at_original_z() {
        use lir_asp_parser::stamp_loads::{Report, Stamp};
        use lir_asp_parser::ModelElement;

        let mut model = model_with_beam();
        model.nodes.extend([
            [0., 0., -1.2],
            [1., 0., -1.2],
            [0., 1., -1.2],
            [1., 1., -1.2],
        ]);
        model.elements.extend([
            ModelElement {
                id: 3,
                definition: 2,
                kind: 44,
                nodes: vec![7, 8, 9, 10],
            },
            ModelElement {
                id: 4,
                definition: 2,
                kind: 44,
                nodes: vec![7, 8, 1, 2],
            },
        ]);
        model.section_assignments.extend([1, 1]);

        let mut result = recover_options(
            &model,
            "pit.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
            None,
            Some(vec![0.]),
            1.,
        )
        .unwrap();
        assert_eq!(result.building.floors.len(), 1);
        let first = &result.building.floors[0];
        assert_eq!(first.elevation_mm, 0.);
        let owner = first.id;
        let extent = result.building.extent(owner);
        let from_element = |id| {
            result
                .contours
                .iter()
                .find(|c| result.source.contour_elements[&c.id].contains(&id))
                .unwrap()
        };
        let slab = from_element(3);
        assert_eq!(slab.kind, Kind::Slab);
        assert_eq!(result.building.owner(slab.id), owner);
        assert_eq!(slab.placement.slab.z(extent), -1200.);
        let wall = from_element(4);
        assert_eq!(wall.kind, Kind::Support);
        assert_eq!(result.building.owner(wall.id), owner);
        assert_eq!(wall.placement.wall(extent), [-1200., 0.]);

        let points = vec![[0.1, 0.2, -1.2], [0.9, 0.2, -1.2]];
        let report = Report {
            notes: vec![],
            stamps: vec![Stamp {
                id: 1,
                kind: 2,
                raw_kind: 770,
                source_group: 0,
                source_direction: [0., 0., -1.],
                end_value: 2.,
                physical_resultant: None,
                attachment_case_id: None,
                attachment_direction_code: None,
                name: "Штамп приямка".into(),
                contours: vec![points.clone()],
                points,
                value: 2.,
                case_id: Some(1),
                direction_code: Some(153),
            }],
        };
        append_stamp_loads(&mut result, &report, None);
        let load = result
            .contours
            .iter()
            .find(|c| c.kind == Kind::LoadLine)
            .unwrap();
        assert_eq!(result.building.owner(load.id), owner);
        assert_eq!(load.placement.slab.z(extent), -1200.);
        assert_eq!(load.load.as_ref().unwrap().magnitude.value, 2.);
    }

    #[test]
    fn inclined_lir_beams_keep_xyz_section_and_physical_roll_in_both_directions() {
        for (ends, expected_roll_deg) in [
            ([[0., 0., -0.5], [1., 0., 0.5]], 30.),
            ([[0., 0., -0.5], [0.1, 0., 0.5]], 120.),
            ([[0.1, 0., 0.5], [0., 0., -0.5]], -60.),
        ] {
            let mut model = model_with_beam();
            model.nodes[4..6].copy_from_slice(&ends);
            let result = recover(
                &model,
                "inclined.lir".into(),
                1000.,
                crate::plan_trace::loads::ForceUnits::TonForce,
                10.,
            )
            .unwrap();
            let beam = result
                .contours
                .iter()
                .find(|c| c.kind == Kind::Beam)
                .unwrap();
            let floor = result.building.owner(beam.id);
            assert!(result.building.enabled);
            assert!(beam.support_open);
            assert!(beam.is_free_beam());
            assert_eq!(
                beam.beam_path(result.building.extent(floor)),
                ends.map(|p| p.map(|v| v * 1000.)).to_vec()
            );
            assert!((beam.column_angle.to_degrees() - expected_roll_deg).abs() < 1e-10);
            let entry = &result.library.entries[&result.library.assignments[&beam.id]];
            assert_eq!(entry.width_mm, 300.);
            assert_eq!(entry.material.thickness_mm, 600.);
            assert_eq!(result.source.contour_elements[&beam.id], vec![2]);
        }
    }
    #[test]
    fn union_fallback_counts_overlapping_slab_cells_once() {
        let cells = vec![
            vec![[0., 0.], [2000., 0.], [2000., 1000.], [0., 1000.]],
            vec![[1000., 0.], [3000., 0.], [3000., 1000.], [1000., 1000.]],
        ];
        let rings = union_loops(&cells);
        assert!((rings.iter().map(|p| area(p)).sum::<f64>() - 3e6).abs() < 1.);
    }
    #[test]
    fn point_touching_material_islands_remain_separate_and_capital_keeps_hole() {
        let mut plate = Plate::default();
        plate.add(vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]], 1);
        plate.add(
            vec![
                [1000., 1000.],
                [2000., 1000.],
                [2000., 2000.],
                [1000., 2000.],
            ],
            2,
        );
        let loops = plate.loops().unwrap();
        assert_eq!(loops.len(), 2);
        assert!((loops.iter().map(|p| area(p)).sum::<f64>() - 2e6).abs() < 1e-6);
        let mut ring = Plate::default();
        for p in [
            vec![[0., 0.], [3000., 0.], [3000., 1000.], [0., 1000.]],
            vec![[0., 2000.], [3000., 2000.], [3000., 3000.], [0., 3000.]],
            vec![[0., 1000.], [1000., 1000.], [1000., 2000.], [0., 2000.]],
            vec![
                [2000., 1000.],
                [3000., 1000.],
                [3000., 2000.],
                [2000., 2000.],
            ],
        ] {
            // Split horizontal strip edges at capital corners to match FE topology.
            let mut refined = vec![];
            for i in 0..p.len() {
                let a: Point = p[i];
                let b: Point = p[(i + 1) % p.len()];
                refined.push(a);
                if (a[0] - b[0]).abs() > 2000. {
                    for t in [1. / 3., 2. / 3.] {
                        refined.push([a[0] + (b[0] - a[0]) * t, a[1]]);
                    }
                }
            }
            ring.add(refined, 3);
        }
        let loops = ring.loops().unwrap();
        assert_eq!(loops.iter().filter(|p| area(p) > 0.).count(), 1);
        assert_eq!(loops.iter().filter(|p| area(p) < 0.).count(), 1);
    }
    #[test]
    fn slab_union_joins_edges_with_different_splits_and_winding() {
        let mut plate = Plate::default();
        plate.add(vec![[0., 0.], [1000., 0.], [1000., 2000.], [0., 2000.]], 1);
        plate.add(
            vec![[1000., 500.], [1000., 1000.], [2000., 1000.], [2000., 500.]],
            2,
        );
        plate.add(
            vec![
                [1000., 1000.],
                [2000., 1000.],
                [2000., 1500.],
                [1000., 1500.],
            ],
            3,
        );
        assert_eq!(plate.loops().unwrap().len(), 2);
        let loops = union_loops_with(&plate.cells, 0.1);
        assert_eq!(loops.len(), 1);
        assert_eq!(loops[0].len(), 8);
        assert!((area(&loops[0]) - 3e6).abs() < 1e-6);
        let mut reversed = plate.cells;
        reversed[0].reverse();
        let loops = union_loops_with(&reversed, 0.1);
        assert_eq!(loops.len(), 1);
        assert!((area(&loops[0]) - 3e6).abs() < 1e-6);
    }
    #[test]
    fn slab_import_uses_verified_sections_instead_of_cell_definitions() {
        use lir_asp_parser::{Model, ModelElement, SectionDefinition};
        let mut model = Model {
            nodes: vec![
                [0., 0., 0.],
                [1., 0., 0.],
                [0., 1., 0.],
                [1., 1., 0.],
                [2., 0., 0.],
                [2., 1., 0.],
            ],
            elements: vec![
                ModelElement {
                    id: 1,
                    definition: 1,
                    kind: 42,
                    nodes: vec![1, 2, 3],
                },
                ModelElement {
                    id: 2,
                    definition: 1,
                    kind: 42,
                    nodes: vec![2, 4, 3],
                },
                ModelElement {
                    id: 3,
                    definition: 2,
                    kind: 44,
                    nodes: vec![2, 5, 4, 6],
                },
            ],
            foundations: Default::default(),
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: vec![],
            section_notes: vec![],
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        let reconstruct = |model: &Model| {
            recover(
                model,
                "slab.lir".into(),
                1000.,
                crate::plan_trace::loads::ForceUnits::TonForce,
                10.,
            )
            .unwrap()
        };
        let result = reconstruct(&model);
        assert_eq!(result.contours.len(), 1);
        let slab = &result.contours[0];
        assert_eq!(result.source.contour_elements[&slab.id], [1, 2, 3]);
        assert!((area(&slab.points) - 2e6).abs() < 1e-6);

        for id in [1, 2] {
            model.sections.insert(
                id,
                SectionDefinition {
                    id,
                    kind: 36,
                    flag: 1,
                    name: String::new(),
                    color: [0; 3],
                    parameters: [2.5, 3_000_000., 0.2, 10. * f32::from(id), 0., 0.],
                },
            );
        }
        model.section_assignments = vec![1, 1, 2];
        let result = reconstruct(&model);
        assert_eq!(
            result
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Slab)
                .count(),
            2
        );
    }
    #[test]
    fn wall_import_joins_mixed_cells_and_preserves_door_and_verified_sections() {
        use lir_asp_parser::{Model, ModelElement, SectionDefinition};
        let mut model = Model {
            nodes: vec![
                [0., 0., 0.],
                [1000., 0., 0.],
                [0., 1000., 0.],
                [0., 0., 3000.],
                [1000., 0., 3000.],
                [0., 1000., 3000.],
                [0., 0., 1500.],
                [1000., 0., 1500.],
                [2000., 0., 0.],
                [3000., 0., 0.],
                [2000., 0., 3000.],
                [3000., 0., 3000.],
                [1000., 0., 2200.],
                [2000., 0., 2200.],
            ],
            elements: [
                (1, 42, vec![1, 2, 3]),
                (1, 42, vec![4, 5, 6]),
                (1, 42, vec![1, 2, 7]),
                (1, 42, vec![2, 8, 7]),
                (2, 44, vec![7, 8, 4, 5]),
                (2, 44, vec![9, 10, 11, 12]),
                (2, 44, vec![13, 14, 5, 11]),
            ]
            .into_iter()
            .enumerate()
            .map(|(i, (definition, kind, nodes))| ModelElement {
                id: i as u32 + 1,
                definition,
                kind,
                nodes,
            })
            .collect(),
            foundations: Default::default(),
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: Vec::new(),
            section_notes: Vec::new(),
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        let reconstruct = |model: &Model| {
            recover_options(
                model,
                "wall.lir".into(),
                1.,
                crate::plan_trace::loads::ForceUnits::TonForce,
                10.,
                Some(1.),
                None,
                1.,
            )
            .unwrap()
        };
        let wall_rects = |result: &Recovered| {
            result
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Support)
                .map(|c| {
                    let [lo, hi] = c
                        .placement
                        .wall(result.building.extent(result.building.owner(c.id)));
                    [c.points[0][0], c.points[1][0], lo, hi]
                })
                .collect::<Vec<_>>()
        };
        let result = reconstruct(&model);
        let rects = wall_rects(&result);
        assert_eq!(rects.len(), 3, "{rects:?}");
        assert!(rects.contains(&[0., 1000., 0., 3000.]));
        assert!(rects.contains(&[1000., 2000., 2200., 3000.]));
        assert!(rects.contains(&[2000., 3000., 0., 3000.]));
        assert!(result
            .source
            .contour_elements
            .values()
            .any(|ids| ids == &[3, 4, 5]));

        // The extra cell overlaps the left pier by exactly the area of the door.
        // The sum of cell areas matches a full wall, but the door is still open.
        model.nodes.push([0., 0., 2200.]);
        model.elements.push(ModelElement {
            id: 8,
            definition: 2,
            kind: 44,
            nodes: vec![1, 2, 15, 13],
        });
        assert_eq!(wall_rects(&reconstruct(&model)), rects);
        model.elements.pop();

        for id in [1, 2] {
            model.sections.insert(
                id,
                SectionDefinition {
                    id,
                    kind: 36,
                    flag: 1,
                    name: String::new(),
                    color: [0; 3],
                    parameters: [2.5, 3_000_000., 0.2, 10. * f32::from(id), 0., 0.],
                },
            );
        }
        model.section_assignments = vec![1, 1, 1, 1, 2, 2, 2];
        let result = reconstruct(&model);
        let rects = wall_rects(&result);
        assert_eq!(rects.len(), 4, "{rects:?}");
        assert!(rects.contains(&[0., 1000., 0., 1500.]));
        for (source, thickness) in [(3, 100.), (5, 200.)] {
            let wall = result
                .contours
                .iter()
                .find(|c| {
                    c.kind == Kind::Support
                        && result.source.contour_elements[&c.id].contains(&source)
                })
                .unwrap();
            let section = result.library.assignments[&wall.id];
            assert_eq!(
                result.library.entries[&section].material.thickness_mm,
                thickness
            );
        }
    }
    #[test]
    fn vertical_lir_member_crossing_intermediate_levels_recovers_each_storey() {
        use lir_asp_parser::{Model, ModelElement};
        let model = Model {
            foundations: BTreeMap::new(),
            nodes: vec![
                [0., 0., 0.],
                [2., 0., 0.],
                [0., 2., 0.],
                [0., 0., 1.],
                [2., 0., 1.],
                [0., 2., 1.],
                [0., 0., 2.],
                [2., 0., 2.],
                [0., 2., 2.],
                [1., 1., -0.2],
                [1., 1., 0.25],
                [1., 1., 0.45],
                [1., 1., 0.60],
                [1., 1., 0.85],
            ],
            elements: vec![
                ModelElement {
                    id: 1,
                    definition: 0,
                    kind: 42,
                    nodes: vec![1, 2, 3],
                },
                ModelElement {
                    id: 2,
                    definition: 0,
                    kind: 42,
                    nodes: vec![4, 5, 6],
                },
                ModelElement {
                    id: 3,
                    definition: 0,
                    kind: 42,
                    nodes: vec![7, 8, 9],
                },
                ModelElement {
                    id: 4,
                    definition: 0,
                    kind: 10,
                    nodes: vec![1, 7],
                },
                ModelElement {
                    id: 5,
                    definition: 0,
                    kind: 10,
                    nodes: vec![10, 11],
                },
                ModelElement {
                    id: 6,
                    definition: 0,
                    kind: 10,
                    nodes: vec![11, 12],
                },
                ModelElement {
                    id: 7,
                    definition: 0,
                    kind: 10,
                    nodes: vec![13, 14],
                },
            ],
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: vec![],
            section_notes: vec![],
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        let result = recover(
            &model,
            "crossing.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let columns = result
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column)
            .collect::<Vec<_>>();
        assert_eq!(columns.len(), 4);
        let recovered_spans = columns
            .iter()
            .map(|c| {
                let floor = result.building.owner(c.id);
                (
                    floor,
                    c.placement.wall(result.building.extent(floor)),
                    result.source.contour_elements[&c.id].as_slice(),
                )
            })
            .collect::<Vec<_>>();
        assert_eq!(
            recovered_spans,
            [
                (1, [0., 1000.], &[4][..]),
                (1, [-200., 450.], &[5, 6][..]),
                (1, [600., 850.], &[7][..]),
                (2, [1000., 2000.], &[4][..]),
            ]
        );
    }
    #[test]
    fn wall_panel_crossing_storeys_is_clipped_to_level_planes() {
        let cut = clip_wall_height(
            &[
                [0., 0., 0.],
                [1000., 0., 0.],
                [1000., 0., 3000.],
                [0., 0., 3000.],
            ],
            1000.,
            2000.,
        );
        assert_eq!(cut.len(), 4);
        assert!(cut.iter().all(|p| (1000. ..=2000.).contains(&p[2])));
    }

    #[test]
    fn wall_recovery_joins_rounding_gaps_and_keeps_the_rectangular_core_of_a_slope() {
        let mut cells = [
            vec![[0., -30.], [1000., -30.], [1000., 2200.], [0., 2200.]],
            vec![
                [1000.6, -29.6],
                [2000., -29.6],
                [2000., 2200.],
                [1000.6, 2200.],
            ],
        ];
        snap_wall_cells(&mut cells, [-30., 3000.], 1.);
        assert_eq!(cells[0][1], cells[1][0]);
        assert_eq!(cells[1][1][1], -30.);
        let polygons = union_polygons_with(&cells, 1.);
        assert_eq!(polygons.len(), 1);
        let (rects, partial) = wall_component_rectangles(&polygons[0].0, &[], 1.).unwrap();
        assert!(!partial);
        assert_eq!(rects.len(), 1);

        let sloped = [
            [0., 0.],
            [2000., 0.],
            [2000., 2200.],
            [1000., 2200.],
            [900., 3000.],
            [0., 3000.],
        ];
        let (rects, partial) = wall_component_rectangles(&sloped, &[], 1.).unwrap();
        assert!(partial);
        assert_eq!(rects, [[0., 900., 0., 3000.], [900., 2000., 0., 2200.]]);
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_V15 control file"]
    fn lir_v15_recovers_two_walls_on_minus_30_with_one_mm_rounding() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_V15").unwrap());
        let model = lir_asp_parser::read_model(&std::fs::read(path).unwrap()).unwrap();
        let recovered = recover_options(
            &model,
            "V15.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
            Some(1.),
            None,
            1.,
        )
        .unwrap();
        let floor = recovered
            .building
            .floors
            .iter()
            .find(|f| f.elevation_mm == -30.)
            .unwrap();
        let walls = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Support && recovered.building.owner(c.id) == floor.id)
            .collect::<Vec<_>>();
        for (source, x, min_y, max_y) in [
            (525390, 3267., -88517., -86702.),
            (55621, 25282., -95132., -93332.),
        ] {
            let matching = walls
                .iter()
                .copied()
                .filter(|c| {
                    c.points.iter().all(|p| (p[0] - x).abs() < 6.)
                        && c.points[0][1].max(c.points[1][1]) > min_y
                        && c.points[0][1].min(c.points[1][1]) < max_y
                })
                .collect::<Vec<_>>();
            let mut area_mm2 = 0.;
            for c in &matching {
                let [lo, hi] = c.placement.wall(recovered.building.extent(floor.id));
                let [a, b] = [c.points[0][1], c.points[1][1]];
                area_mm2 += (a.max(b).min(max_y) - a.min(b).max(min_y)) * (hi - lo);
                eprintln!("  wall {}: {:?}; Z={lo}..{hi}", c.id, c.points);
            }
            eprintln!(
                "Wall X={x:.0}: {} contours, {:.4} m² recovered",
                matching.len(),
                area_mm2 / 1e6
            );
            assert!(
                matching
                    .iter()
                    .any(|c| recovered.source.contour_elements[&c.id].contains(&source)),
                "source KЭ {source}; found on contours {:?}",
                recovered
                    .contours
                    .iter()
                    .filter(|c| c.kind == Kind::Support
                        && recovered.source.contour_elements[&c.id].contains(&source))
                    .map(|c| (recovered.building.owner(c.id), &c.points))
                    .collect::<Vec<_>>()
            );
            assert!(matching.iter().any(|c| (c
                .placement
                .wall(recovered.building.extent(floor.id))[0]
                + 30.)
                .abs()
                < 0.02));
            assert!(
                area_mm2 > 7.75e6,
                "wall X={x}: recovered area {area_mm2} mm²"
            );
        }
    }

    #[test]
    fn wall_import_keeps_noisy_rectangular_component_beside_unsupported_slope() {
        use lir_asp_parser::{Model, ModelElement};
        let model = Model {
            nodes: vec![
                [0., 0., 0.],
                [2000., 0., 0.],
                [0., 2000., 0.],
                [0., 0., 3000.],
                [2000., 0., 3000.],
                [0., 2000., 3000.],
                [10000., 500., 100.],
                [12000., 500., 100.],
                [10000., 500., 1900.],
                [12000., 500., 1900.8],
                [14000., 500., 100.],
                [16000., 500., 100.],
                [14000., 500., 1800.],
            ],
            elements: [
                (42, vec![1, 2, 3]),
                (42, vec![4, 5, 6]),
                (44, vec![7, 8, 9, 10]),
                (42, vec![11, 12, 13]),
            ]
            .into_iter()
            .enumerate()
            .map(|(i, (kind, nodes))| ModelElement {
                id: i as u32 + 1,
                definition: 1,
                kind,
                nodes,
            })
            .collect(),
            foundations: Default::default(),
            properties: Default::default(),
            sections: Default::default(),
            section_assignments: vec![],
            section_notes: vec![],
            bar_angles_deg: Default::default(),
            inactive_elements: 0,
            inferred_types: 0,
            skipped_unknown: Default::default(),
            skipped_invalid: 0,
        };
        let result = recover(
            &model,
            "wall.lir".into(),
            1.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let walls: Vec<_> = result
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Support)
            .collect();
        assert_eq!(walls.len(), 1);
        let wall = walls[0];
        assert_eq!(result.source.contour_elements[&wall.id], [3]);
        let [lo, hi] = wall.placement.wall([0., 3000.]);
        assert!(
            (lo - 100.).abs() < 0.02 && (hi - 1900.4).abs() < 0.02,
            "recovered elevation range: {lo}..{hi} mm"
        );
        assert!(result
            .source
            .notes
            .iter()
            .any(|s| s.contains("пропущено КЭ в этой части: 1")));
    }
}

#[cfg(test)]
mod foundation_regions_regression {
    use super::*;
    #[test]
    #[ignore = "requires MINI_FEM_LIR_V15 control file"]
    fn lir_v15_recovers_one_foundation_without_holes() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_V15").unwrap());
        let model = lir_asp_parser::read_model(&std::fs::read(path).unwrap()).unwrap();
        let recovered = recover(
            &model,
            "V15.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let regions: Vec<_> = recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Foundation)
            .collect();
        assert_eq!(regions.len(), 1);
        let c = regions[0];
        let data = c.foundation.as_ref().unwrap();
        assert!(data.exclusions.is_empty());
        assert_eq!(data.interior.len(), 9077);
        assert_eq!(recovered.source.contour_elements[&c.id].len(), 9077);
        let z = c
            .placement
            .slab
            .z(recovered.building.extent(recovered.building.owner(c.id)));
        assert!((z + 6030.).abs() < 0.01);
        let field = crate::plan_trace::foundation::Field::new(c, 10.).unwrap();
        assert!(field.at([25100., -89700.]).is_some());
        assert!(data.interior.iter().all(|p| {
            field
                .at(p.xy)
                .is_some_and(|c1| (c1 - 10. * p.c1.value).abs() < 1e-8)
        }));
    }

    #[test]
    #[ignore = "requires MINI_FEM_LIR_FOUNDATION_0351 control file"]
    fn lir_0351_recovers_foundation_on_both_physical_planes() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_LIR_FOUNDATION_0351").unwrap());
        let model = lir_asp_parser::read_model(&std::fs::read(&path).unwrap()).unwrap();
        let recovered = recover(
            &model,
            "0351.lir".into(),
            1000.,
            crate::plan_trace::loads::ForceUnits::TonForce,
            10.,
        )
        .unwrap();
        let mut planes = BTreeMap::<i64, BTreeSet<u32>>::new();
        for c in recovered
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Foundation)
        {
            let z = c
                .placement
                .slab
                .z(recovered.building.extent(recovered.building.owner(c.id)));
            planes
                .entry(z.round() as i64)
                .or_default()
                .extend(recovered.source.contour_elements[&c.id].iter().copied());
        }
        eprintln!(
            "C1 regions={}, planes and source cells={:?}",
            recovered
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Foundation)
                .count(),
            planes
                .iter()
                .map(|(z, ids)| (*z, ids.len()))
                .collect::<Vec<_>>()
        );
        assert_eq!(planes[&-7380].len(), 128);
        assert_eq!(planes[&-6180].len(), 8320);
        let restored = planes.values().flatten().copied().collect::<BTreeSet<_>>();
        assert_eq!(restored, model.foundations.keys().copied().collect());
    }
    #[test]
    fn two_zone_control_keeps_frame_hole_and_constant_values_at_all_cell_centres() {
        let mut central = vec![];
        let mut frame = vec![];
        for y in 0..20 {
            for x in 0..20 {
                let (a, b) = (x as f64 * 500., y as f64 * 500.);
                let p = vec![[a, b], [a + 500., b], [a + 500., b + 500.], [a, b + 500.]];
                if (2..18).contains(&x) && (2..18).contains(&y) {
                    central.push(p);
                } else {
                    frame.push(p);
                }
            }
        }
        let mut contours = vec![];
        for (cells, value) in [(&central, 500.), (&frame, 1000.)] {
            let parts = union_polygons_with(cells, 0.01);
            let total = parts
                .iter()
                .map(|(p, holes)| area(p).abs() - holes.iter().map(|p| area(p).abs()).sum::<f64>())
                .sum::<f64>();
            assert!((total - cells.len() as f64 * 250000.).abs() < 1.);
            for (points, exclusions) in parts {
                let mut c = contour(contours.len() as u64 + 1, Kind::Foundation, points, None);
                c.foundation = Some(crate::plan_trace::foundation::Data {
                    values: vec![
                        crate::plan_trace::loads::Magnitude {
                            value,
                            units: crate::plan_trace::loads::ForceUnits::TonForce
                        };
                        c.points.len()
                    ],
                    exclusions,
                    ..Default::default()
                });
                contours.push(c);
            }
        }
        assert_eq!(
            contours.len(),
            2,
            "central rectangle and a single frame with a hole"
        );
        let fields = crate::plan_trace::foundation::fields(&contours, 10.).unwrap();
        for y in 0..20 {
            for x in 0..20 {
                let p = [(x as f64 + 0.5) * 500., (y as f64 + 0.5) * 500.];
                let hits = fields.iter().filter_map(|f| f.at(p)).collect::<Vec<_>>();
                assert_eq!(hits.len(), 1, "no duplicate or missing region at {p:?}");
                let expected = if (2..18).contains(&x) && (2..18).contains(&y) {
                    5000.
                } else {
                    10000.
                };
                assert!((hits[0] - expected).abs() < 1e-6);
            }
        }
    }
}

#[cfg(test)]
mod merged_soil_samples_test {
    use super::*;
    #[test]
    fn different_c1_cells_share_one_boundary_and_keep_centers() {
        let cells = vec![
            (
                1,
                vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
                500.,
            ),
            (
                2,
                vec![[1000., 0.], [2000., 0.], [2000., 1000.], [1000., 1000.]],
                1000.,
            ),
        ];
        let polygons = cells.iter().map(|(_, p, _)| p.clone()).collect::<Vec<_>>();
        let parts = union_polygons_with(&polygons, 0.01);
        assert_eq!(parts.len(), 1);
        let p = foundation_boundary_nodes(&parts[0].0, &polygons);
        assert!(p.contains(&[1000., 0.]));
        let d =
            imported_foundation_data(&p, &cells, crate::plan_trace::loads::ForceUnits::TonForce);
        assert_eq!(d.interior.len(), 2);
        assert_eq!(d.interior[0].c1.value, 500.);
        assert_eq!(d.interior[1].c1.value, 1000.);
        let i = p.iter().position(|p| *p == [1000., 0.]).unwrap();
        assert_eq!(d.values[i].value, 750.);
    }
}
