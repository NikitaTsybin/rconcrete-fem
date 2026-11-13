use super::*;
use crate::plan_trace::intersections::{self, Graph};
use std::collections::BTreeSet;
#[derive(Default)]
struct PlanCache {
    world_key: Vec<u64>,
    lines: Vec<[Point; 2]>,
    points: Vec<Point>,
    screen_key: Vec<u64>,
    mesh: std::sync::Arc<egui::Mesh>,
}
pub(super) struct State {
    plan: std::cell::RefCell<PlanCache>,
    spatial: std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<Vec<spatial_depth::Vertex>>)>>,
    revision: u64,
    pub show: bool,
    pub tolerance: f64,
    pub graph: Graph,
    pub current: bool,
    source: Vec<Contour>,
    layout: Option<crate::plan_trace::building_mesh::Layout>,
    legacy: [f64; 2],
    job: Option<mpsc::Receiver<Result<Graph, String>>>,
    ready: bool,
    size: f64,
}
impl Default for State {
    fn default() -> Self {
        Self {
            plan: Default::default(),
            spatial: Default::default(),
            revision: 0,
            show: false,
            tolerance: 10.,
            graph: Graph::default(),
            current: false,
            source: vec![],
            layout: None,
            legacy: [0.; 2],
            job: None,
            ready: false,
            size: 0.,
        }
    }
}
impl State {
    pub(super) fn with_tolerance(tolerance: f64) -> Self {
        Self {
            tolerance,
            ..Default::default()
        }
    }
}
impl App {
    fn refresh_intersection_state(&mut self) {
        self.intersections.current = self.intersections.ready
            && self.intersections.size == self.mesh_size
            && self.intersections.legacy == self.support_height.extent()
            && self.intersections.layout == self.building_layout()
            && model::same_mesh_input(&self.intersections.source, &self.operation_sources());
    }

    pub(super) fn poll_intersections(&mut self, ctx: &egui::Context) {
        if let Some(rx) = &self.intersections.job {
            match rx.try_recv() {
                Ok(result) => {
                    self.intersections.job = None;
                    match result {
                        Ok(graph) => {
                            self.status = format!(
                                "Найдены пересечения: линий {}, точек {}",
                                graph.lines.len(),
                                graph.points.len()
                            );
                            self.intersections.revision =
                                self.intersections.revision.wrapping_add(1);
                            self.intersections.graph = graph;
                            self.intersections.ready = true;
                            self.intersections.show = true;
                            ctx.request_repaint();
                        }
                        Err(e) => self.status = e,
                    }
                }
                Err(mpsc::TryRecvError::Disconnected) => {
                    self.intersections.job = None;
                    self.status = "Построение пересечений прервано".into();
                }
                Err(mpsc::TryRecvError::Empty) => {
                    ctx.request_repaint_after(Duration::from_millis(50));
                }
            }
        }
        if self.intersections.show {
            self.refresh_intersection_state();
        }
    }

    pub(super) fn intersection_controls(&mut self, ui: &mut egui::Ui) {
        if !self.intersections.show {
            self.refresh_intersection_state();
        }
        if ui
            .add_enabled(
                !self.input_locked
                    && self.intersections.job.is_none()
                    && self.mode.is_none()
                    && !self.contours.is_empty(),
                egui::Button::new("Пересечь"),
            )
            .clicked()
        {
            let source = self.operation_sources();
            let layout = self.building_layout();
            let legacy = self.support_height.extent();
            self.intersections.source = source.clone();
            self.intersections.layout = layout.clone();
            self.intersections.legacy = legacy;
            self.intersections.ready = false;
            self.intersections.current = false;
            self.intersections.size = self.mesh_size;
            let size = self.mesh_size;
            let (tx, rx) = mpsc::channel();
            self.intersections.job = Some(rx);
            let ctx = ui.ctx().clone();
            crate::plan_trace::platform::spawn(move || {
                let _ = tx.send(intersections::build_sampled(
                    &source,
                    layout.as_ref(),
                    legacy,
                    size,
                ));
                ctx.request_repaint();
            });
        }
        ui.checkbox(&mut self.intersections.show, "Показать пересечения");
        if self.intersections.job.is_some() {
            ui.spinner();
        }
    }
    pub(super) fn intersection_owner_visible(&self, id: u64) -> bool {
        self.view_contour(id)
            .is_some_and(|c| !self.object_hidden(c) && self.on_view_floor(c))
    }
    pub(super) fn paint_intersections(&self, painter: &egui::Painter, rect: Rect, camera: Camera) {
        if self.fe_isolated() || !self.intersections.show || !self.intersections.current {
            return;
        }
        let world_key = vec![
            self.intersections.revision,
            self.intersections.graph.lines.as_ptr() as u64,
            self.intersections.graph.lines.len() as u64,
            self.intersections.graph.points.len() as u64,
            self.view_visibility.revision as u64,
            self.building.active,
            self.building.enabled as u64,
            self.drawing_visibility.support_storeys as u64,
        ];
        let mut cache = self.intersections.plan.borrow_mut();
        if cache.world_key != world_key {
            let visible = self
                .contours
                .iter()
                .filter(|c| !self.object_hidden(c) && self.on_view_floor(c))
                .map(|c| c.id)
                .collect::<std::collections::BTreeSet<_>>();
            let mut groups = BTreeMap::<[i64; 3], ([f64; 2], f64, Vec<[f64; 2]>)>::new();
            let mut points = BTreeMap::<[i64; 2], Point>::new();
            for line in &self.intersections.graph.lines {
                if !line.owners.iter().any(|id| visible.contains(id)) {
                    continue;
                }
                let e = line.ends.map(|p| [p[0], p[1]]);
                let d = [e[1][0] - e[0][0], e[1][1] - e[0][1]];
                let length = d[0].hypot(d[1]);
                if length < 1e-5 {
                    points
                        .entry(e[0].map(|v| (v * 1e5).round() as i64))
                        .or_insert(e[0]);
                    continue;
                }
                let mut axis = d.map(|v| v / length);
                if axis[0] < -1e-10 || axis[0].abs() < 1e-10 && axis[1] < 0. {
                    axis = axis.map(|v| -v);
                }
                let offset = -axis[1] * e[0][0] + axis[0] * e[0][1];
                let key = [
                    (axis[0] * 1e9).round() as i64,
                    (axis[1] * 1e9).round() as i64,
                    (offset * 1e4).round() as i64,
                ];
                let group = groups.entry(key).or_insert((axis, offset, vec![]));
                let mut interval = e.map(|p| p[0] * group.0[0] + p[1] * group.0[1]);
                interval.sort_by(f64::total_cmp);
                group.2.push(interval);
            }
            let mut lines = vec![];
            for (_, (axis, offset, mut intervals)) in groups {
                intervals.sort_by(|a, b| a[0].total_cmp(&b[0]));
                let mut merged = Vec::<[f64; 2]>::new();
                for v in intervals {
                    if let Some(last) = merged.last_mut().filter(|last| v[0] <= last[1] + 1e-5) {
                        last[1] = last[1].max(v[1]);
                    } else {
                        merged.push(v);
                    }
                }
                for v in merged {
                    lines.push(v.map(|t| {
                        [
                            axis[0] * t - axis[1] * offset,
                            axis[1] * t + axis[0] * offset,
                        ]
                    }));
                }
            }
            for (p, ids) in &self.intersections.graph.points {
                if ids.iter().any(|id| visible.contains(id)) {
                    points
                        .entry([p[0], p[1]].map(|v| (v * 1e5).round() as i64))
                        .or_insert([p[0], p[1]]);
                }
            }
            cache.lines = lines;
            cache.points = points.into_values().collect();
            cache.world_key = world_key;
            cache.screen_key.clear();
        }
        let ppp = painter.ctx().pixels_per_point();
        let mut screen_key = vec![
            camera.center[0].to_bits(),
            camera.center[1].to_bits(),
            camera.zoom.to_bits(),
            camera.angle.to_bits(),
            rect.min.x.to_bits() as u64,
            rect.min.y.to_bits() as u64,
            rect.max.x.to_bits() as u64,
            rect.max.y.to_bits() as u64,
            ppp.to_bits() as u64,
        ];
        if let Some(plane) = camera.plane {
            screen_key.extend(plane.iter().flatten().map(|v| v.to_bits()));
        }
        if cache.screen_key != screen_key {
            let mut mesh = egui::Mesh::default();
            let ink = Color32::from_rgb(25, 110, 255);
            let dot = |mesh: &mut egui::Mesh, p: Pos2| {
                if !rect.expand(3.).contains(p) {
                    return;
                }
                let center = mesh.vertices.len() as u32;
                mesh.colored_vertex(p, ink);
                for i in 0..12 {
                    let a = i as f32 * std::f32::consts::TAU / 12.;
                    mesh.colored_vertex(p + egui::vec2(a.cos(), a.sin()) * 3., ink);
                }
                for i in 0..12 {
                    mesh.add_triangle(center, center + 1 + i, center + 1 + (i + 1) % 12);
                }
            };
            for e in &cache.lines {
                if let Some(e) = clip_line(
                    camera.screen64(e[0], rect),
                    camera.screen64(e[1], rect),
                    rect.expand(3.),
                ) {
                    if e[0].distance(e[1]) < 1. {
                        dot(&mut mesh, e[0]);
                    } else {
                        crate::plan_trace::render_cache::line(&mut mesh, e[0], e[1], 5., ink, ppp);
                    }
                }
            }
            for &p in &cache.points {
                dot(&mut mesh, camera.screen(p, rect));
            }
            cache.mesh = std::sync::Arc::new(mesh);
            cache.screen_key = screen_key;
        }
        crate::plan_trace::render_cache::paint(painter, rect, cache.mesh.clone());
    }
    pub(super) fn spatial_intersections(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
    ) {
        if self.fe_isolated() || !self.intersections.show || !self.intersections.current {
            return;
        }
        let mut key = vec![
            self.intersections.revision,
            self.view_visibility.revision as u64,
            self.spatial.source_revision as u64,
            self.building.active,
        ];
        for p in [
            [0., 0., 0.],
            [1000., 0., 0.],
            [0., 1000., 0.],
            [0., 0., 1000.],
        ] {
            let q = project(p);
            key.extend([
                q.0.x.to_bits() as u64,
                q.0.y.to_bits() as u64,
                q.1.to_bits(),
            ]);
        }
        key.extend([
            rect.left().to_bits() as u64,
            rect.top().to_bits() as u64,
            rect.width().to_bits() as u64,
            rect.height().to_bits() as u64,
        ]);
        for (i, p) in self.sections.active_planes() {
            key.extend(p.normal(i).map(f64::to_bits));
            key.extend([p.offset(i).to_bits(), p.positive as u64]);
        }
        if let Some((_, vertices)) = self
            .intersections
            .spatial
            .borrow()
            .as_ref()
            .filter(|(old, _)| old == &key)
        {
            spatial_depth::paint_layer(
                painter,
                rect,
                vertices.clone(),
                spatial_depth::Layer::Intersections,
            );
            return;
        }
        let visible = self
            .contours
            .iter()
            .filter(|c| !self.object_hidden(c) && self.on_view_floor(c))
            .map(|c| c.id)
            .collect::<BTreeSet<_>>();
        let mut b = spatial_depth::Builder::new(rect, projected);
        for line in &self.intersections.graph.lines {
            if line.owners.iter().any(|id| visible.contains(id)) {
                let endpoints = line.ends.map(project);
                b.line(endpoints, 5., Color32::from_rgb(25, 110, 255));
                b.anchor_recent(
                    6,
                    &[
                        endpoints[0],
                        endpoints[1],
                        endpoints[1],
                        endpoints[0],
                        endpoints[1],
                        endpoints[0],
                    ],
                    0.0015,
                );
            }
        }
        for (p, ids) in &self.intersections.graph.points {
            if ids.iter().any(|id| visible.contains(id)) {
                let p = project(*p);
                let corners = [
                    Vec2::new(-2., -2.),
                    Vec2::new(2., -2.),
                    Vec2::new(2., 2.),
                    Vec2::new(-2., 2.),
                ]
                .map(|d| (p.0 + d, p.1));
                for i in [[0, 1, 2], [0, 2, 3]] {
                    b.triangle(i.map(|i| corners[i]), Color32::from_rgb(25, 110, 255), 0.);
                }
            }
        }
        let vertices = std::sync::Arc::new(b.vertices);
        *self.intersections.spatial.borrow_mut() = Some((key, vertices.clone()));
        spatial_depth::paint_layer(painter, rect, vertices, spatial_depth::Layer::Intersections);
    }
    pub(super) fn wall_connection_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal_wrapped(|ui| {
            ui.label("Порог, мм");
            ui.add(
                egui::DragValue::new(&mut self.intersections.tolerance)
                    .update_while_editing(false)
                    .range(0.001..=10000.),
            );
            let ready = !self.input_locked
                && !self.is_kind_locked(Kind::Support)
                && self.mode.is_none()
                && self.drag.is_none();
            for (trim, label) in [(false, "Дотянуть"), (true, "Подрезать")] {
                if ui.add_enabled(ready, egui::Button::new(label)).clicked() {
                    if let Err(e) = self.connect_wall_ends(trim) {
                        self.status = e;
                    }
                }
            }
            let beams_ready=!self.input_locked && !self.is_kind_locked(Kind::Beam) && self.mode.is_none() && self.drag.is_none();
            if ui.add_enabled(beams_ready,egui::Button::new("Балки к плитам")).on_hover_text("Привязать вершины балок к ближайшей границе плиты на той же отметке в пределах порога. Выделите также смежные балки, чтобы сохранить общие вершины. Ctrl+Z — отмена.").clicked() {
                if let Err(e)=self.snap_beams_to_slabs(){self.status=e;}
            }
        });
    }
    fn snap_beams_to_slabs(&mut self) -> Result<usize, String> {
        if self.input_locked || self.is_kind_locked(Kind::Beam) {
            return Err("Снимите блокировку балок и исходных данных".into());
        }
        let tolerance = self.intersections.tolerance;
        if !tolerance.is_finite() || tolerance <= 0. {
            return Err("Порог должен быть положительным".into());
        }
        let mut boundaries = vec![];
        for c in self.contours.iter().filter(|c| c.kind == Kind::Slab) {
            let mut path = c.mesh_boundary(self.mesh_size)?;
            if path.len() < 3 {
                continue;
            }
            path.push(path[0]);
            let z = self.source_heights(c)[0];
            for e in path.windows(2) {
                boundaries.push((z, [e[0], e[1]]));
            }
        }
        let mut next = self.contours.clone();
        let mut changed = BTreeSet::new();
        let mut count = 0;
        let movement_locked = self.movement_locked_contours();
        for (i, c) in self.contours.iter().enumerate().filter(|(_, c)| {
            c.kind == Kind::Beam
                && !c.is_free_beam()
                && self.operation_target(c.id)
                && !movement_locked.contains(&c.id)
                && c.curves.is_empty()
        }) {
            let z = self.source_heights(c)[0];
            for (j, &p) in c.points.iter().enumerate() {
                let nearest = boundaries
                    .iter()
                    .filter(|(h, _)| (*h - z).abs() < 1e-5)
                    .filter_map(|(_, [a, b])| {
                        let d = [b[0] - a[0], b[1] - a[1]];
                        let l = d[0] * d[0] + d[1] * d[1];
                        if l <= 1e-12 {
                            return None;
                        }
                        let t = (((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l).clamp(0., 1.);
                        let q = [a[0] + t * d[0], a[1] + t * d[1]];
                        let distance = (p[0] - q[0]).hypot(p[1] - q[1]);
                        (distance <= tolerance).then_some((distance, q))
                    })
                    .min_by(|a, b| a.0.total_cmp(&b.0));
                let Some((distance, q)) = nearest else {
                    continue;
                };
                if distance <= 1e-5 {
                    continue;
                }
                if let Some(other) = self.contours.iter().find(|b| {
                    b.id != c.id
                        && b.kind == Kind::Beam
                        && !b.is_free_beam()
                        && (!self.operation_target(b.id) || movement_locked.contains(&b.id))
                        && (self.source_heights(b)[0] - z).abs() < 1e-5
                        && b.points
                            .iter()
                            .any(|r| (p[0] - r[0]).hypot(p[1] - r[1]) < 1e-5)
                }) {
                    if movement_locked.contains(&other.id) {
                        continue;
                    }
                    return Err(format!(
                        "Для сохранения общей вершины выделите также балку {}",
                        other.id
                    ));
                }
                next[i].points[j] = q;
                changed.insert(i);
                count += 1;
            }
        }
        for &i in &changed {
            model::validate_candidate(&next, &next[i])
                .map_err(|e| format!("Балка {}: {e}", next[i].id))?;
        }
        if count > 0 {
            self.history.push(self.contours.clone());
            self.contours = next;
            self.clear_mesh();
            self.spatial.cache_reset();
            self.rebuild_index();
            self.intersections.current = false;
        }
        self.status = format!("Привязано вершин балок к плитам: {count}. Ctrl+Z — отменить");
        Ok(count)
    }
    pub(super) fn connect_wall_ends(&mut self, trim: bool) -> Result<usize, String> {
        if self.input_locked || self.is_kind_locked(Kind::Support) {
            return Err("Снимите блокировку стен и исходных данных".into());
        }
        let h = self.intersections.tolerance;
        if !h.is_finite() || h <= 0. {
            return Err("Порог должен быть положительным".into());
        }
        let layout = self.building_layout();
        let legacy = self.support_height.extent();
        let original = &self.contours;
        let movement_locked = self.movement_locked_contours();
        let movable = |id| self.operation_target(id) && !movement_locked.contains(&id);
        let mut candidates = vec![];
        for (i, c) in original.iter().enumerate().filter(|(_, c)| {
            movable(c.id)
                && c.kind == Kind::Support
                && !c.closed()
                && c.curves.is_empty()
                && c.points.len() >= 2
        }) {
            let hc = intersections::height(c, layout.as_ref(), legacy);
            for end in [0, c.points.len() - 1] {
                let near = if end == 0 { 1 } else { end - 1 };
                let p = c.points[end];
                let a = c.points[near];
                let d = [p[0] - a[0], p[1] - a[1]];
                let len = d[0].hypot(d[1]);
                if len < 1e-5 {
                    continue;
                }
                for (j, other) in original
                    .iter()
                    .enumerate()
                    .filter(|(j, v)| *j != i && v.kind == Kind::Support)
                {
                    let ho = intersections::height(other, layout.as_ref(), legacy);
                    if hc[1].min(ho[1]) - hc[0].max(ho[0]) < 1e-5 {
                        continue;
                    }
                    let mut path = other.points.clone();
                    if other.closed() {
                        path.push(path[0]);
                    }
                    for (k, e) in path.windows(2).enumerate() {
                        let v = [e[1][0] - e[0][0], e[1][1] - e[0][1]];
                        let det = d[0] * v[1] - d[1] * v[0];
                        if det.abs() < 1e-12 * len * v[0].hypot(v[1]) {
                            continue;
                        }
                        let r = [e[0][0] - p[0], e[0][1] - p[1]];
                        let t = (r[0] * v[1] - r[1] * v[0]) / det;
                        let u = (r[0] * d[1] - r[1] * d[0]) / det;
                        let distance = t * len;
                        if (if trim {
                            distance >= -1e-5 || distance < -h || -distance >= len - 1e-5
                        } else {
                            distance <= 1e-5 || distance > h
                        }) {
                            continue;
                        }
                        let point = [p[0] + t * d[0], p[1] + t * d[1]];
                        let mut partner = None;
                        if !(-1e-9..=1. + 1e-9).contains(&u) {
                            if trim || other.closed() || !other.curves.is_empty() {
                                continue;
                            }
                            let endpoint = if u < 0. && k == 0 {
                                0
                            } else if u > 1. && k + 1 == other.points.len() - 1 {
                                other.points.len() - 1
                            } else {
                                continue;
                            };
                            let q = other.points[endpoint];
                            let gap = (q[0] - point[0]).hypot(q[1] - point[1]);
                            if gap > h {
                                continue;
                            }
                            partner = Some((j, endpoint));
                        }
                        candidates.push((distance.abs(), i, end, point, partner));
                    }
                }
            }
        }
        candidates.sort_by(|a, b| {
            a.0.total_cmp(&b.0)
                .then_with(|| a.1.cmp(&b.1))
                .then_with(|| a.2.cmp(&b.2))
        });
        let mut edits = BTreeMap::new();
        for (_, i, end, p, partner) in candidates {
            if edits.contains_key(&(i, end)) || partner.is_some_and(|v| edits.contains_key(&v)) {
                continue;
            }
            if partner.is_some_and(|(j, _)| !movable(original[j].id)) {
                continue;
            }
            edits.insert((i, end), p);
            if let Some(v) = partner {
                edits.insert(v, p);
            }
        }
        if edits.is_empty() {
            self.status = "Подходящих концов стен в пределах порога не найдено".into();
            return Ok(0);
        }
        let mut next = self.contours.clone();
        let mut changed = std::collections::BTreeSet::new();
        for (&(i, end), &p) in &edits {
            next[i].points[end] = p;
            changed.insert(i);
        }
        for &i in &changed {
            let c = &next[i];
            if c.points.len() == 2 {
                let a = &original[i].points;
                if (c.points[1][0] - c.points[0][0]) * (a[1][0] - a[0][0])
                    + (c.points[1][1] - c.points[0][1]) * (a[1][1] - a[0][1])
                    <= 0.
                {
                    return Err(format!(
                        "Стена {}: подрезка удаляет весь участок; уменьшите порог",
                        c.id
                    ));
                }
            }
            model::validate_candidate(&next, c).map_err(|e| format!("Стена {}: {e}", c.id))?;
        }
        self.history.push(self.contours.clone());
        self.contours = next;
        self.clear_mesh();
        self.spatial.cache_reset();
        self.rebuild_index();
        self.intersections.current = false;
        self.status = format!(
            "{} концов стен: {}. Ctrl+Z — отменить",
            if trim {
                "Подрезано"
            } else {
                "Дотянуто"
            },
            edits.len()
        );
        Ok(edits.len())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn snapping_beam_to_plate_boundary_is_explicit_and_undoable() {
        let mut a = App::default();
        let c = |id, kind, p| {
            serde_json::from_value(serde_json::json!({"id":id,"kind":kind,"points":p,"support_open":kind=="Beam","curves":[]})).unwrap()
        };
        a.contours = vec![
            c(
                1,
                "Slab",
                vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
            ),
            c(2, "Beam", vec![[100., -2.], [900., -2.]]),
        ];
        a.rebuild_index();
        a.select_only(Some(2));
        a.intersections.tolerance = 20.;
        let before = a.contours.clone();
        assert_eq!(a.snap_beams_to_slabs().unwrap(), 2);
        assert_eq!(a.contours[1].points, vec![[100., 0.], [900., 0.]]);
        assert_eq!(a.contours[0], before[0]);
        a.undo();
        assert_eq!(a.contours, before);
    }
    fn wall(a: &mut App, p: &[Point]) -> u64 {
        a.begin(Kind::Support);
        for &p in p {
            a.sketch_click(p, false);
        }
        a.finish();
        a.cancel();
        a.contours.last().unwrap().id
    }
    #[test]
    fn extend_repairs_two_gapped_ends_atomically_and_trimming_has_one_undo() {
        let mut a = App::default();
        let x = wall(&mut a, &[[0., 0.], [990., 0.]]);
        let y = wall(&mut a, &[[1000., 10.], [1000., 1000.]]);
        let original = a.contours.clone();
        a.select_only(None);
        a.intersections.tolerance = 20.;
        assert_eq!(a.connect_wall_ends(false).unwrap(), 2);
        assert_eq!(
            a.contours.iter().find(|c| c.id == x).unwrap().points[1],
            [1000., 0.]
        );
        assert_eq!(
            a.contours.iter().find(|c| c.id == y).unwrap().points[0],
            [1000., 0.]
        );
        a.undo();
        assert_eq!(a.contours, original);
        let mut a = App::default();
        let x = wall(&mut a, &[[0., 0.], [1010., 0.]]);
        wall(&mut a, &[[1000., -500.], [1000., 500.]]);
        let original = a.contours.clone();
        a.select_only(None);
        a.intersections.tolerance = 20.;
        assert_eq!(a.connect_wall_ends(true).unwrap(), 1);
        assert_eq!(
            a.contours.iter().find(|c| c.id == x).unwrap().points[1],
            [1000., 0.]
        );
        a.undo();
        assert_eq!(a.contours, original);
    }

    #[test]
    fn wall_extension_can_meet_locked_wall_but_cannot_move_it() {
        for (start, expected) in [(-10., 1), (10., 0)] {
            let mut app = App::default();
            wall(&mut app, &[[0., 0.], [990., 0.]]);
            wall(&mut app, &[[1000., start], [1000., 1000.]]);
            app.contours[1].placement.movement_locked = true;
            app.intersections.tolerance = 20.;
            let before = app.contours.clone();
            assert_eq!(app.connect_wall_ends(false).unwrap(), expected);
            assert_eq!(app.contours[1], before[1]);
            assert_eq!(
                app.contours[0].points[1],
                [990. + 10. * expected as f64, 0.]
            );
        }
    }
    #[test]
    fn intersection_graph_clips_wall_trace_by_its_slabs_hole() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([1000., 1000.], false);
        a.cancel();
        a.select_only(Some(a.contours[0].id));
        a.begin(Kind::Hole);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([400., 400.], false);
        a.sketch_click([600., 600.], false);
        a.cancel();
        assert_eq!(
            a.contours.iter().filter(|c| c.kind == Kind::Hole).count(),
            1
        );
        wall(&mut a, &[[500., 0.], [500., 1000.]]);
        let graph = intersections::build(&a.contours, None, [-3000., 0.]).unwrap();
        let length: f64 = graph
            .lines
            .iter()
            .filter(|l| l.ends.iter().all(|p| p[2].abs() < 1e-8))
            .map(|l| (l.ends[1][0] - l.ends[0][0]).hypot(l.ends[1][1] - l.ends[0][1]))
            .sum();
        assert!((length - 800.).abs() < 1e-6, "{length}");
    }
}

#[cfg(test)]
mod plan_cache_regression {
    use super::*;
    #[test]
    fn sampled_intersections_are_merged_and_reused_between_frames() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([1000., 1000.], false);
        app.cancel();
        let id = app.contours[0].id;
        app.intersections.show = true;
        app.intersections.current = true;
        app.intersections.graph.lines = (0..10000)
            .map(|i| intersections::Segment {
                ends: [[i as f64, 0., 0.], [(i + 1) as f64, 0., 0.]],
                owners: vec![id],
            })
            .collect();
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(600., 400.));
        for _ in 0..2 {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(rect),
                    ..Default::default()
                },
                |ctx| {
                    let painter = ctx.layer_painter(egui::LayerId::background());
                    app.paint_intersections(&painter, rect, app.camera);
                },
            );
        }
        assert_eq!(app.intersections.plan.borrow().lines.len(), 1);
        let mesh = app.intersections.plan.borrow().mesh.clone();
        assert!(!mesh.vertices.is_empty());
        let _ = ctx.run(
            egui::RawInput {
                screen_rect: Some(rect),
                ..Default::default()
            },
            |ctx| {
                app.paint_intersections(
                    &ctx.layer_painter(egui::LayerId::background()),
                    rect,
                    app.camera,
                );
            },
        );
        assert!(std::sync::Arc::ptr_eq(
            &mesh,
            &app.intersections.plan.borrow().mesh
        ));
    }
}
