//! Geometric distance/nearest-support catchments. No structural load redistribution.
use super::*;
use std::sync::Arc;
#[derive(Clone)]
struct Segment {
    a: Point,
    b: Point,
    id: u64,
}
struct Tree {
    bounds: [f64; 4],
    segments: Vec<Segment>,
    children: Option<Box<[Tree; 2]>>,
}
impl Tree {
    fn build(mut segments: Vec<Segment>) -> Self {
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for s in &segments {
            for p in [s.a, s.b] {
                for a in 0..2 {
                    bounds[a] = bounds[a].min(p[a]);
                    bounds[a + 2] = bounds[a + 2].max(p[a]);
                }
            }
        }
        let children = if segments.len() > 8 {
            let axis = usize::from(bounds[3] - bounds[1] > bounds[2] - bounds[0]);
            segments.sort_by(|a, b| {
                ((a.a[axis] + a.b[axis]) * 0.5).total_cmp(&((b.a[axis] + b.b[axis]) * 0.5))
            });
            let right = segments.split_off(segments.len() / 2);
            Some(Box::new([
                Self::build(std::mem::take(&mut segments)),
                Self::build(right),
            ]))
        } else {
            None
        };
        Self {
            bounds,
            segments,
            children,
        }
    }
    fn bound(&self, p: Point) -> f64 {
        (0..2)
            .map(|a| {
                (self.bounds[a] - p[a])
                    .max(0.)
                    .max(p[a] - self.bounds[a + 2])
                    .powi(2)
            })
            .sum()
    }
    fn nearest(&self, p: Point, best: &mut (f64, u64)) {
        if self.bound(p) > best.0 {
            return;
        }
        if let Some(c) = &self.children {
            let first = usize::from(c[1].bound(p) < c[0].bound(p));
            c[first].nearest(p, best);
            c[1 - first].nearest(p, best);
        } else {
            for s in &self.segments {
                let d = [s.b[0] - s.a[0], s.b[1] - s.a[1]];
                let length = d[0] * d[0] + d[1] * d[1];
                let t = if length > 0. {
                    ((p[0] - s.a[0]) * d[0] + (p[1] - s.a[1]) * d[1]) / length
                } else {
                    0.
                };
                let t = t.clamp(0., 1.);
                let distance =
                    (p[0] - s.a[0] - t * d[0]).powi(2) + (p[1] - s.a[1] - t * d[1]).powi(2);
                if distance < best.0 || (distance == best.0 && s.id < best.1) {
                    *best = (distance, s.id);
                }
            }
        }
    }
    fn query(&self, p: Point) -> (f64, u64) {
        let mut best = (f64::INFINITY, u64::MAX);
        self.nearest(p, &mut best);
        (best.0.sqrt(), best.1)
    }
}
pub(super) struct Data {
    generation: u64,
    pub distances: Vec<f64>,
    pub owners: Vec<u64>,
    pub areas: BTreeMap<u64, f64>,
    pub centers: BTreeMap<u64, Point>,
    borders: Vec<([Point; 2], u64)>,
}
impl App {
    pub(super) fn support_distance_data(&self) -> Option<Arc<Data>> {
        let mesh = self.fe_mesh.as_ref()?;
        if let Some(data) = self.support_distance_cache.borrow().as_ref() {
            if data.generation == self.mesh_generation && data.distances.len() == mesh.nodes.len() {
                return Some(data.clone());
            }
        }
        let mut segments = vec![];
        for support in self.contours.iter().filter(|c| c.kind == Kind::Support) {
            let count = if support.closed() {
                support.points.len()
            } else {
                support.points.len().saturating_sub(1)
            };
            for i in 0..count {
                segments.push(Segment {
                    a: support.points[i],
                    b: support.points[(i + 1) % support.points.len()],
                    id: support.id,
                });
            }
        }
        for column in self.contours.iter().filter(|c| c.kind == Kind::Column) {
            if let Some(&p) = column.points.first() {
                segments.push(Segment {
                    a: p,
                    b: p,
                    id: column.id,
                });
            }
        }
        if segments.is_empty() {
            return None;
        }
        let mut floor_segments = BTreeMap::<u64, Vec<Segment>>::new();
        for segment in segments {
            if let Some(data) = &mesh.building {
                let wall = data
                    .layout
                    .levels
                    .iter()
                    .find(|l| l.id == mesh.contour_floor(segment.id));
                if let Some(wall) = wall {
                    for level in &data.layout.levels {
                        if (level.z - wall.z).abs() < 1e-5 || (level.z - wall.top).abs() < 1e-5 {
                            floor_segments
                                .entry(level.id)
                                .or_default()
                                .push(segment.clone());
                        }
                    }
                }
            } else {
                floor_segments.entry(0).or_default().push(segment);
            }
        }
        let trees = floor_segments
            .into_iter()
            .map(|(floor, segments)| (floor, Tree::build(segments)))
            .collect::<BTreeMap<_, _>>();
        let query = |floor: u64, p: Point| {
            trees
                .get(&floor)
                .map_or((f64::NAN, 0), |tree| tree.query(p))
        };
        let distances = mesh
            .nodes
            .iter()
            .enumerate()
            .map(|(i, n)| query(mesh.building.as_ref().map_or(0, |b| b.node_floors[i]), n.xy).0)
            .collect();
        let mut owners = mesh
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let p = e.nodes.map(|n| mesh.nodes[n].xy);
                query(
                    mesh.element_floor(i),
                    [
                        (p[0][0] + p[1][0] + p[2][0]) / 3.,
                        (p[0][1] + p[1][1] + p[2][1]) / 3.,
                    ],
                )
                .1
            })
            .collect::<Vec<_>>();
        let ids = mesh
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect::<BTreeMap<_, _>>();
        for q in &mesh.quads {
            let p = q.nodes.map(|n| mesh.nodes[n].xy);
            let center = [
                p.iter().map(|p| p[0]).sum::<f64>() / 4.,
                p.iter().map(|p| p[1]).sum::<f64>() / 4.,
            ];
            let owner = query(mesh.element_floor(ids[&q.display_triangles[0]]), center).1;
            for id in q.display_triangles {
                if let Some(&i) = ids.get(&id) {
                    owners[i] = owner;
                }
            }
        }
        let mut areas = BTreeMap::<u64, f64>::new();
        for (i, e) in mesh.elements.iter().enumerate() {
            let points = e.nodes.map(|n| mesh.nodes[n].xy);
            if owners[i] != 0 {
                *areas.entry(owners[i]).or_default() += geometry::area(&points) / 1_000_000.;
            }
        }
        let mut centers = BTreeMap::<u64, Point>::new();
        let mut edge_owners = BTreeMap::<[usize; 2], u64>::new();
        let mut borders = vec![];
        for (i, e) in mesh.elements.iter().enumerate() {
            if owners[i] == 0 {
                continue;
            }
            let points = e.nodes.map(|n| mesh.nodes[n].xy);
            let area = geometry::area(&points) / 1_000_000.;
            let center = centers.entry(owners[i]).or_insert([0.; 2]);
            for axis in 0..2 {
                center[axis] += area * points.iter().map(|p| p[axis]).sum::<f64>() / 3.;
            }
            for side in 0..3 {
                let a = e.nodes[side];
                let b = e.nodes[(side + 1) % 3];
                let edge = [a.min(b), a.max(b)];
                if let Some(old) = edge_owners.insert(edge, owners[i]) {
                    if old != owners[i] {
                        borders.push((edge.map(|n| mesh.nodes[n].xy), mesh.element_floor(i)));
                    }
                }
            }
        }
        for (id, center) in &mut centers {
            if let Some(area) = areas.get(id) {
                for value in center {
                    *value /= area;
                }
            }
        }
        let data = Arc::new(Data {
            generation: self.mesh_generation,
            distances,
            owners,
            areas,
            centers,
            borders,
        });
        *self.support_distance_cache.borrow_mut() = Some(data.clone());
        Some(data)
    }
    pub(super) fn result_node_value(
        &self,
        case: &crate::plan_trace::fem::CaseResult,
        node: usize,
    ) -> f64 {
        if self.result_field == 15 {
            self.support_distance_data()
                .map_or(0., |data| data.distances[node])
        } else if self.result_field == crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD {
            let xyz = case.shell.as_ref().map_or_else(
                || case.w_mm.get(node).map(|&w| [0., 0., w]),
                |shell| shell.translations_mm.get(node).copied(),
            );
            xyz.map_or(f64::NAN, |xyz| {
                crate::plan_trace::fem::displacement_norm(xyz, self.displacement_axes)
            })
        } else {
            case.node_value(self.result_field, node)
        }
    }
    pub(super) fn result_element_value(
        &self,
        case: &crate::plan_trace::fem::CaseResult,
        element: usize,
    ) -> f64 {
        if self.result_field == 34 {
            return case
                .foundation_pressure_kn_m2
                .get(element)
                .copied()
                .unwrap_or(f64::NAN);
        }
        if self.result_field == 33 {
            return case.loads_kn_m2.get(element).copied().unwrap_or(f64::NAN);
        }
        if (26..=32).contains(&self.result_field) {
            return self.reinforcement.result.as_ref().map_or(f64::NAN, |r| {
                r.value_for(self.result_field, element, self.reinforcement_area_view)
            });
        }
        if self.result_field == 16 {
            self.support_distance_data().map_or(0., |data| {
                data.areas
                    .get(&data.owners[element])
                    .copied()
                    .unwrap_or(f64::NAN)
            })
        } else {
            let value = case.element_value(self.result_field, element);
            if self.membrane_area && matches!(self.result_field, 12..=14) {
                value
                    / self
                        .membrane_element_thickness(case)
                        .get(element)
                        .copied()
                        .unwrap_or(f64::NAN)
            } else {
                value
            }
        }
    }
    fn membrane_element_thickness(
        &self,
        case: &crate::plan_trace::fem::CaseResult,
    ) -> std::sync::Arc<Vec<f64>> {
        let key = (self.mesh_generation, self.result_generation);
        if let Some((old, values)) = self.perf.membrane_thickness.borrow().as_ref() {
            if *old == key {
                return values.clone();
            }
        }
        // Surface results retain the actual material used in this stiffness variant.
        // Match root node IDs, not XY: coincident plans on different floors are distinct.
        let mut by_nodes = BTreeMap::new();
        if let Some(shell) = &case.shell {
            for face in shell.surface_forces.iter().filter(|f| f.support.is_none()) {
                let h = face.material.thickness_mm / 1000.;
                if !h.is_finite() || h <= 0. {
                    continue;
                }
                for i in 0..face.nodes.len() {
                    for j in i + 1..face.nodes.len() {
                        for k in j + 1..face.nodes.len() {
                            let mut nodes = [face.nodes[i], face.nodes[j], face.nodes[k]];
                            nodes.sort_unstable();
                            by_nodes.insert(nodes, h);
                        }
                    }
                }
            }
        }
        let values = std::sync::Arc::new(self.fe_mesh.as_ref().map_or_else(Vec::new, |mesh| {
            mesh.elements
                .iter()
                .map(|e| {
                    let mut nodes = e.nodes;
                    nodes.sort_unstable();
                    by_nodes.get(&nodes).copied().unwrap_or(f64::NAN)
                })
                .collect()
        }));
        *self.perf.membrane_thickness.borrow_mut() = Some((key, values.clone()));
        values
    }
    pub(super) fn result_display_factor(&self) -> f64 {
        if matches!(self.result_field, 15 | 16 | 26..=32)
            || crate::plan_trace::fem::displacement_field(self.result_field)
        {
            1.
        } else {
            self.force_result_factor()
        }
    }
    pub(super) fn paint_support_area_borders(&self, painter: &egui::Painter, rect: Rect) {
        if self.result_field != 16 || !self.show_results {
            return;
        }
        if let Some(data) = self.support_distance_data() {
            for (line, floor) in &data.borders {
                if self.building.enabled && *floor != self.building.active {
                    continue;
                }
                if let Some(line) = clip_line(
                    self.camera.screen64(line[0], rect),
                    self.camera.screen64(line[1], rect),
                    rect,
                ) {
                    painter.line_segment(line, Stroke::new(1.5_f32, Color32::from_rgb(30, 35, 40)));
                }
            }
        }
    }
    pub(super) fn support_area_controls(&mut self, ui: &mut egui::Ui) {
        if !matches!(self.result_field, 0 | 10 | 11 | 15 | 16 | 23..=32) {
            return;
        }
        ui.small("Расстояние в плане до линии опоры. Площади — геометрическая оценка по ближайшей опоре, с точностью КЭ-сетки.");
        if let Some(data) = self.support_distance_data() {
            egui::CollapsingHeader::new("Грузовые площади опор, м²").show(ui, |ui| {
                egui::Grid::new("support_catchment_areas")
                    .num_columns(2)
                    .show(ui, |ui| {
                        for (id, area) in &data.areas {
                            if self.building.enabled
                                && self.building.owner(*id) != self.building.active
                            {
                                continue;
                            }
                            ui.label(format!("Опора {id}"));
                            ui.label(self.display_text.number(*area));
                            ui.end_row();
                        }
                    });
            });
        }
    }
}
