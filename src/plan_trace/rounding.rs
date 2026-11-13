use super::*;
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct Options {
    pub cluster: bool,
    pub radius_mm: f64,
    pub by_floor: bool,
}
impl Default for Options {
    fn default() -> Self {
        Self {
            cluster: false,
            radius_mm: 50.,
            by_floor: true,
        }
    }
}
fn clustered(points: Vec<(u64, Point)>, radius: f64) -> BTreeMap<(u64, [u64; 2]), Point> {
    use rstar::{primitives::GeomWithData, RTree};
    let unique = points
        .into_iter()
        .map(|(s, p)| ((s, p.map(f64::to_bits)), p))
        .collect::<BTreeMap<_, _>>();
    let mut groups = BTreeMap::<u64, Vec<Point>>::new();
    for ((s, _), p) in unique {
        groups.entry(s).or_default().push(p);
    }
    let mut out = BTreeMap::new();
    for (scope, p) in groups {
        let tree = RTree::bulk_load(
            p.iter()
                .enumerate()
                .map(|(i, &p)| GeomWithData::new(p, i))
                .collect(),
        );
        let mut assigned = vec![false; p.len()];
        for (i, point) in p.iter().enumerate() {
            if assigned[i] {
                continue;
            }
            let members = tree
                .locate_within_distance(*point, radius * radius)
                .filter(|v| !assigned[v.data])
                .map(|v| v.data)
                .collect::<Vec<_>>();
            let center = std::array::from_fn(|a| {
                members
                    .iter()
                    .map(|&i| p[i][a] / members.len() as f64)
                    .sum()
            });
            for i in members {
                assigned[i] = true;
                out.insert((scope, p[i].map(f64::to_bits)), center);
            }
        }
    }
    out
}

fn snap_to_fixed_vertices(
    points: &mut Vec<(u64, Point)>,
    fixed: Vec<(u64, Point)>,
    radius: f64,
) -> BTreeMap<(u64, [u64; 2]), Point> {
    use rstar::RTree;
    let mut by_scope = BTreeMap::<u64, Vec<Point>>::new();
    for (scope, point) in fixed {
        by_scope.entry(scope).or_default().push(point);
    }
    let indices = by_scope
        .into_iter()
        .map(|(scope, points)| (scope, RTree::bulk_load(points)))
        .collect::<BTreeMap<_, _>>();
    let mut snapped = BTreeMap::new();
    points.retain(|&(scope, point)| {
        let Some(tree) = indices.get(&scope) else {
            return true;
        };
        let distance = |p: &Point| (p[0] - point[0]).powi(2) + (p[1] - point[1]).powi(2);
        let closest = tree
            .locate_within_distance(point, radius * radius)
            .min_by(|a, b| {
                distance(a)
                    .total_cmp(&distance(b))
                    .then_with(|| a[0].total_cmp(&b[0]))
                    .then_with(|| a[1].total_cmp(&b[1]))
            });
        if let Some(&anchor) = closest {
            snapped.insert((scope, point.map(f64::to_bits)), anchor);
        }
        closest.is_none()
    });
    snapped
}
pub(super) fn default_step() -> f64 {
    10.
}
pub(super) fn default_tolerance() -> f64 {
    1.
}
fn straighten_axes(contours: &mut [Contour], scopes: &[Option<u64>], degrees: f64) -> usize {
    fn root(parents: &mut [usize], mut i: usize) -> usize {
        while parents[i] != i {
            parents[i] = parents[parents[i]];
            i = parents[i];
        }
        i
    }
    let mut ids = BTreeMap::new();
    let mut positions = vec![];
    let mut paths = Vec::with_capacity(contours.len());
    for (c, scope) in contours.iter().zip(scopes) {
        let path = scope
            .filter(|_| {
                c.placement.cut_source.is_none()
                    && c.grid.is_none()
                    && c.curves.is_empty()
                    && c.points.len() >= 2
            })
            .map(|scope| {
                c.points
                    .iter()
                    .map(|&p| {
                        *ids.entry((scope, p.map(|v| if v == 0. { 0 } else { v.to_bits() })))
                            .or_insert_with(|| {
                                positions.push(p);
                                positions.len() - 1
                            })
                    })
                    .collect::<Vec<_>>()
            });
        paths.push(path);
    }
    let mut parents = [
        (0..positions.len()).collect::<Vec<_>>(),
        (0..positions.len()).collect(),
    ];
    let tangent = degrees.to_radians().tan();
    for (c, path) in contours.iter().zip(&paths) {
        let Some(path) = path else { continue };
        for (&a, &b) in path
            .iter()
            .zip(path.iter().cycle().skip(1))
            .take(path.len() - usize::from(!c.closed()))
        {
            let delta = [0, 1].map(|i| (positions[a][i] - positions[b][i]).abs());
            if delta[0].max(delta[1]) <= 1e-9 {
                continue;
            }
            let axis = usize::from(delta[0] > delta[1]);
            if delta[axis] <= tangent * delta[1 - axis] {
                let p = &mut parents[axis];
                let (a, b) = (root(p, a), root(p, b));
                p[b] = a;
            }
        }
    }
    let mut points = positions.clone();
    for axis in 0..2 {
        let mut values = vec![(0., 0_u32); positions.len()];
        for (i, p) in positions.iter().enumerate() {
            let reference = root(&mut parents[axis], i);
            let (sum, n) = &mut values[reference];
            *sum += p[axis] - positions[reference][axis];
            *n += 1;
        }
        for (i, p) in points.iter_mut().enumerate() {
            let reference = root(&mut parents[axis], i);
            let (sum, n) = values[reference];
            if n > 1 {
                p[axis] = positions[reference][axis] + sum / f64::from(n);
            }
        }
    }
    let mut changed = 0;
    for (c, path) in contours.iter_mut().zip(paths) {
        if let Some(path) = path {
            let corrected = path.iter().map(|&i| points[i]).collect::<Vec<_>>();
            if c.points != corrected {
                c.points = corrected;
                changed += 1;
            }
        }
    }
    changed
}
impl App {
    fn rounding_object_description(&self, id: u64) -> String {
        let Some(c) = self.contours.iter().find(|c| c.id == id) else {
            return format!("Объект {id}");
        };
        let floor = self.building.floor(self.building.owner(id));
        let mut text = format!(
            "Объект {id} ({}, {}), этаж «{}», Z={} мм",
            c.kind.label(),
            c.name,
            floor.name,
            floor.elevation_mm
        );
        if let (Some(a), Some(b)) = (c.points.first(), c.points.last()) {
            text.push_str(&format!(
                "; от X={}, Y={} до X={}, Y={} мм",
                a[0], a[1], b[0], b[1]
            ));
        }
        text
    }

    pub(super) fn rounding_ready(&self) -> bool {
        !self.input_locked
            && !self.contours.is_empty()
            && !self
                .contours
                .iter()
                .any(|c| self.operation_target(c.id) && self.is_kind_locked(c.kind))
            && self.mode.is_none()
            && self.drag.is_none()
            && self.arc_placement.is_none()
            && self.clipboard_op.is_none()
            && !self.measurements.active
    }
    pub(super) fn rounding_controls(&mut self, ui: &mut egui::Ui) {
        ui.separator();
        ui.horizontal(|ui| {
            ui.label("Округление, мм").on_hover_text("Округление X/Y всех исходных объектов и нагрузок до ближайшего кратного шага. У кривых округляются управляющие точки");
            ui.add(egui::DragValue::new(&mut self.rounding_step).speed(1.).range(0.001..=1_000_000.));
            if ui.add_enabled(self.rounding_ready(),egui::Button::new("Применить"))
                .on_disabled_hover_text("Снимите блокировку исходных данных и типов объектов; завершите текущую операцию")
                .clicked(){if let Err(e)=self.round_coordinates(){self.status=e;}}
        });
        ui.horizontal_wrapped(|ui|{
            ui.checkbox(&mut self.rounding_options.cluster,"По радиусу")
                .on_hover_text("Привязывать к ближайшей заблокированной вершине в радиусе. Остальные близкие вершины объединяются и округляются.");
            ui.add_enabled(self.rounding_options.cluster,egui::DragValue::new(&mut self.rounding_options.radius_mm).range(0.001..=1e6).suffix(" мм"));
            egui::ComboBox::from_id_salt("round_scope").selected_text(if self.rounding_options.by_floor{"По этажам"}else{"По всей модели"}).show_ui(ui,|ui|{ui.selectable_value(&mut self.rounding_options.by_floor,true,"По этажам");ui.selectable_value(&mut self.rounding_options.by_floor,false,"По всей модели");});
        }).response.on_hover_text("Близкие вершины объединяются вокруг опорной точки без цепного слияния удалённых точек. Центр группы округляется один раз. По всей модели выравнивает XY также между разными этажами. Учитывается выделение.");
        self.wall_connection_controls(ui);
        ui.horizontal_wrapped(|ui| {
            ui.label("Отклонение, мм").on_hover_text("Максимальное расстояние исходных вершин до нового прямого отрезка. 0 — только строго прямолинейные участки");
            ui.add(egui::DragValue::new(&mut self.simplify_tolerance).speed(0.1).range(0. ..=1000.));
            let enabled = self.rounding_ready();
            let button = ui.add_enabled(enabled, egui::Button::new("").min_size(egui::vec2(28., 22.)));
            button.widget_info(|| egui::WidgetInfo::labeled(
                egui::WidgetType::Button, enabled, "Удалить промежуточные вершины",
            ));
            let center = button.rect.center();
            let at = |x, y| center + egui::vec2(x, y);
            let muted = ui.visuals().widgets.noninteractive.fg_stroke.color;
            let blue = if enabled { egui::Color32::from_rgb(70, 160, 245) } else { muted };
            let red = if enabled { egui::Color32::from_rgb(240, 75, 75) } else { muted };
            let line = egui::Stroke::new(1.5_f32, blue);
            let painter = ui.painter();
            painter.line_segment([at(-9., 0.), at(9., 0.)], line);
            for x in [-9., 0., 9.] {
                painter.rect_stroke(egui::Rect::from_center_size(at(x, 0.), egui::Vec2::splat(5.)),
                    0., line, egui::StrokeKind::Inside);
            }
            let cross = egui::Stroke::new(1.7_f32, red);
            painter.line_segment([at(-4., -5.), at(4., 5.)], cross);
            painter.line_segment([at(-4., 5.), at(4., -5.)], cross);
            if button.on_hover_text("Удалить промежуточные вершины. Дуги, сплайны и параметрические сетки сохраняются. Ctrl+Z — отмена")
                .on_disabled_hover_text("Удалить промежуточные вершины. Снимите блокировки исходных данных и объектов; завершите текущую операцию")
                .clicked(){if let Err(error)=self.simplify_vertices(){self.status=error;}}
        });
        self.contour_merge_controls(ui);
        if ui.add_enabled(self.vertical_merge_ready(),egui::Button::new("Объединить вертикальные конструкции"))
            .on_hover_text("Объединить совпадающие стены/колонны соседних этажей с одинаковыми жёсткостями, если на стыке нет плиты, примыкающей стены, балки или явной опоры. Ctrl+Z — отменить")
            .clicked(){if let Err(e)=self.merge_vertical_members(){self.status=e;}}
        ui.horizontal_wrapped(|ui| {
            ui.add(egui::DragValue::new(&mut self.rectify_angle_deg)
                .range(0. ..=15.).speed(0.1).suffix("°"))
                .on_hover_text("Отклонение ребра от X или Y");
            if ui.add_enabled(self.rounding_ready(), egui::Button::new("Исправить углы"))
                .on_hover_text("Выпрямить рёбра в пределах допуска к осям X/Y. Учитывается выделение. Ctrl+Z — отмена.")
                .clicked() {
                if let Err(error) = self.rectify_angles() { self.status = error; }
            }
        });
        self.vertical_conversion_controls(ui);
    }
    fn rectify_angles(&mut self) -> Result<(), String> {
        if !self.rounding_ready() {
            return Err("Снимите блокировку исходных данных".into());
        }
        let movement_locked = self.movement_locked_contours();
        let mut contours = self.contours.clone();
        let scopes = contours
            .iter()
            .map(|c| {
                (self.operation_target(c.id) && !movement_locked.contains(&c.id)).then(|| {
                    if self.building.enabled {
                        self.building.owner(c.id)
                    } else {
                        0
                    }
                })
            })
            .collect::<Vec<_>>();
        let changed = straighten_axes(&mut contours, &scopes, self.rectify_angle_deg);
        if changed > 0 {
            let changed_ids = self
                .contours
                .iter()
                .zip(&contours)
                .filter(|(old, new)| old.points != new.points)
                .map(|(_, new)| new.id)
                .collect::<std::collections::BTreeSet<_>>();
            for new in contours.iter().filter(|c| {
                changed_ids.contains(&c.id) || c.parent.is_some_and(|id| changed_ids.contains(&id))
            }) {
                model::validate_candidate(&contours, new)
                    .map_err(|e| format!("Объект {}: {e}. Исправление углов отменено", new.id))?;
            }
            self.history.push(self.contours.clone());
            self.contours = contours;
            self.clear_mesh();
            self.slab_cache = None;
            self.spatial.cache_reset();
            self.rebuild_index();
        }
        self.status = format!("Исправлено объектов: {changed}");
        Ok(())
    }
    fn simplify_vertices(&mut self) -> Result<usize, String> {
        if !self.rounding_ready() {
            return Err("Завершите операцию и снимите блокировки объектов/исходных данных".into());
        }
        let tolerance = self.simplify_tolerance;
        if !tolerance.is_finite() || tolerance < 0. {
            return Err("Введите неотрицательный допуск в мм".into());
        }
        let movement_locked = self.movement_locked_contours();
        let mut contours = self.contours.clone();
        let mut removed = 0;
        for c in &mut contours {
            if !self.operation_target(c.id) || movement_locked.contains(&c.id) {
                continue;
            }
            if c.grid.is_some() || !c.curves.is_empty() {
                continue;
            }
            let original = c.points.clone();
            let spatial = c.placement.beam_z_mm.as_ref();
            let coordinates =
                |i: usize| [original[i][0], original[i][1], spatial.map_or(0., |z| z[i])];
            let mut kept = (0..original.len()).collect::<Vec<_>>();
            let closed = c.closed();
            let minimum = if closed { 3 } else { 2 };
            let flags = c
                .refinement
                .as_ref()
                .filter(|_| c.kind == Kind::Punching)
                .map(|r| {
                    r.inactive_edges
                        .iter()
                        .copied()
                        .collect::<std::collections::BTreeSet<_>>()
                });
            loop {
                let before = kept.len();
                let mut i = if closed { 0 } else { 1 };
                while kept.len() > minimum && i < kept.len() - if closed { 0 } else { 1 } {
                    let n = kept.len();
                    let prev = (i + n - 1) % n;
                    let next = (i + 1) % n;
                    let (a, b, d) = (
                        coordinates(kept[prev]),
                        coordinates(kept[i]),
                        coordinates(kept[next]),
                    );
                    let delta = std::array::from_fn::<_, 3, _>(|i| d[i] - a[i]);
                    let dot = |v: [f64; 3], w: [f64; 3]| {
                        v.into_iter().zip(w).map(|(v, w)| v * w).sum::<f64>()
                    };
                    let len2 = dot(delta, delta);
                    let forward = dot(
                        std::array::from_fn(|i| b[i] - a[i]),
                        std::array::from_fn(|i| d[i] - b[i]),
                    );
                    // Never merge enabled/disabled summation edges across a flag boundary.
                    let same_flags = flags
                        .as_ref()
                        .is_none_or(|f| f.contains(&kept[prev]) == f.contains(&kept[i]));
                    let same_c1 = c.foundation.as_ref().is_none_or(|data| {
                        if len2 <= 1e-16 {
                            return false;
                        }
                        let t = ((b[0] - a[0]) * delta[0] + (b[1] - a[1]) * delta[1]) / len2;
                        let value = |i| data.value(i).in_units(self.load_units, self.gravity);
                        (value(kept[i]) - (value(kept[prev]) * (1. - t) + value(kept[next]) * t))
                            .abs()
                            < 1e-8
                    });
                    let mut within = len2 > 1e-16 && forward >= 0. && same_flags && same_c1;
                    let mut j = (kept[prev] + 1) % original.len();
                    while within && j != kept[next] {
                        let p = coordinates(j);
                        let v = std::array::from_fn(|i| p[i] - a[i]);
                        let along = dot(v, delta);
                        let t = along / len2;
                        let normal = std::array::from_fn(|i| v[i] - t * delta[i]);
                        within =
                            (0. ..=1.).contains(&t) && dot(normal, normal) <= tolerance * tolerance;
                        j = (j + 1) % original.len();
                    }
                    if within {
                        kept.remove(i);
                    } else {
                        i += 1;
                    }
                }
                if kept.len() == before {
                    break;
                }
            }
            removed += original.len() - kept.len();
            if let Some(data) = &mut c.foundation {
                let values = kept.iter().map(|&i| data.value(i)).collect();
                data.values = values;
            }
            c.points = kept.iter().map(|i| original[*i]).collect();
            if let Some(z) = spatial {
                c.placement.beam_z_mm = Some(kept.iter().map(|&i| z[i]).collect());
            }
            if let (Some(flags), Some(refinement)) = (flags, c.refinement.as_mut()) {
                refinement.inactive_edges = kept
                    .iter()
                    .enumerate()
                    .filter_map(|(i, k)| flags.contains(k).then_some(i))
                    .collect();
            }
            c.rebuild()
                .map_err(|e| format!("Удаление не применено: объект {}: {e}", c.id))?;
        }
        for c in &contours {
            model::validate_candidate(&contours, c).map_err(|e| {
                format!(
                    "Удаление не применено: объект {}: {e}. Уменьшите допуск",
                    c.id
                )
            })?;
        }
        if removed == 0 {
            self.status = "Промежуточных вершин в пределах допуска не найдено".into();
            return Ok(0);
        }
        self.history.push(self.contours.clone());
        self.contours = contours;
        self.clear_mesh();
        self.slab_cache = None;
        self.spatial.cache_reset();
        self.rebuild_index();
        self.status = format!(
            "Удалено промежуточных вершин: {removed}. Допуск {tolerance} мм. Ctrl+Z — отменить"
        );
        Ok(removed)
    }
    fn round_coordinates(&mut self) -> Result<usize, String> {
        let step = self.rounding_step;
        if !step.is_finite() || step <= 0. {
            return Err("Введите положительный шаг округления в мм".into());
        }
        if !self.rounding_ready() {
            return Err("Округление недоступно: завершите операцию и снимите блокировки объектов/исходных данных".into());
        }
        let radius = self.rounding_options.radius_mm;
        if self.rounding_options.cluster && (!radius.is_finite() || radius <= 0.) {
            return Err("Введите положительный радиус группировки".into());
        }
        let movement_locked = self.movement_locked_contours();
        let movable = |id| self.operation_target(id) && !movement_locked.contains(&id);
        if !self.contours.iter().any(|c| movable(c.id)) {
            self.status = "Нет объектов с разрешённым перемещением".into();
            return Ok(0);
        }
        let mut rounded = self.contours.clone();
        let scope = |id| {
            if self.rounding_options.by_floor && self.building.enabled {
                self.building.owner(id)
            } else {
                0
            }
        };
        let mut points = vec![];
        let mut fixed = vec![];
        if self.rounding_options.cluster {
            for c in &self.contours {
                let points = if movement_locked.contains(&c.id) {
                    &mut fixed
                } else if movable(c.id) {
                    &mut points
                } else {
                    continue;
                };
                let s = scope(c.id);
                if let Some(g) = &c.grid {
                    points.push((s, g.origin));
                } else if c.curves.is_empty() {
                    points.extend(c.points.iter().map(|&p| (s, p)));
                } else {
                    for mut curve in c.curves.clone() {
                        curve.map(|p| points.push((s, *p)));
                    }
                }
                if let Some(data) = &c.foundation {
                    points.extend(data.interior.iter().map(|p| (s, p.xy)));
                    points.extend(data.exclusions.iter().flatten().map(|&p| (s, p)));
                }
            }
        }
        let fixed_targets = snap_to_fixed_vertices(&mut points, fixed, radius);
        let clusters = clustered(points, radius);
        for c in &mut rounded {
            if !movable(c.id) {
                continue;
            }
            let round = |p: &mut Point| {
                let key = (scope(c.id), p.map(f64::to_bits));
                if let Some(&anchor) = fixed_targets.get(&key) {
                    *p = anchor;
                    return;
                }
                if let Some(&center) = clusters.get(&key) {
                    *p = center;
                }
                for value in p {
                    *value = (*value / step).round() * step;
                }
            };
            if let Some(grid) = &mut c.grid {
                round(&mut grid.origin);
                for p in [&mut grid.ex, &mut grid.ey] {
                    for v in p {
                        *v = (*v / step).round() * step;
                    }
                }
            } else if c.curves.is_empty() {
                for p in &mut c.points {
                    round(p);
                }
                if self.rounding_options.cluster && !punching::is_punching(c) {
                    let mut keep = vec![];
                    for i in 0..c.points.len() {
                        if keep.last().is_none_or(|&j| c.points[j] != c.points[i]) {
                            keep.push(i);
                        }
                    }
                    if c.closed()
                        && keep.len() > 1
                        && c.points[*keep.first().unwrap()] == c.points[*keep.last().unwrap()]
                    {
                        keep.pop();
                    }
                    if let Some(data) = &mut c.foundation {
                        data.values = keep.iter().map(|&i| data.value(i)).collect();
                    }
                    c.points = keep.iter().map(|&i| c.points[i]).collect();
                }
            } else {
                for curve in &mut c.curves {
                    curve.map(round);
                }
            }
            if let Some(data) = &mut c.foundation {
                data.map_xy(round);
            }
            c.rebuild().map_err(|e| {
                format!(
                    "Округление не применено. {}: {e}. Уменьшите шаг округления или радиус объединения вершин",
                    self.rounding_object_description(c.id)
                )
            })?;
            if c.points.iter().flatten().any(|v| !v.is_finite()) {
                return Err("Округление не применено: слишком большие координаты".into());
            }
        }
        for c in &rounded {
            model::validate_candidate(&rounded, c).map_err(|e| {
                format!(
                    "Округление не применено. {}: {e}. Уменьшите шаг округления или радиус объединения вершин",
                    self.rounding_object_description(c.id)
                )
            })?;
        }
        let changed = self
            .contours
            .iter()
            .zip(&rounded)
            .filter(|(a, b)| a != b)
            .count();
        if changed == 0 {
            self.status = "Координаты уже соответствуют шагу округления".into();
            return Ok(0);
        }
        let geometry = self
            .contours
            .iter()
            .zip(&rounded)
            .any(|(a, b)| a != b && a.is_mesh_input());
        self.history.push(self.contours.clone());
        self.contours = rounded;
        if geometry {
            self.fe_mesh = None;
            self.discard_mesh_job();
            self.slab_cache = None;
        }
        if self
            .history
            .last()
            .unwrap()
            .iter()
            .zip(&self.contours)
            .any(|(a, b)| a != b && a.kind.affects_calculation())
        {
            self.analysis = None;
            self.analysis_input = None;
            self.analysis_job = None;
            self.combination_cache = None;
        }
        self.invalidate_scene_buffers();
        self.status = format!(
            "Координаты округлены до {step} мм. Изменено объектов: {changed}. Ctrl+Z — отменить"
        );
        Ok(changed)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn straighten_keeps_shared_corner_and_leaves_diagonal_edge_outside_tolerance() {
        let mut app = App::default();
        for [a, b] in [
            [[0., 0.], [1000., 10.]],
            [[1000., 10.], [1010., 1010.]],
            [[3000., 0.], [4000., 500.]],
        ] {
            app.begin(Kind::Support);
            app.sketch_click(a, false);
            app.sketch_click(b, false);
            app.finish();
            app.cancel();
        }
        app.select_only(None);
        let original = app.contours.clone();
        app.rectify_angles().unwrap();
        let [a, b, c] = app.contours.as_slice() else {
            panic!()
        };
        assert_eq!(a.points[0][1], a.points[1][1]);
        assert_eq!(b.points[0][0], b.points[1][0]);
        assert_eq!(a.points[1], b.points[0]);
        assert_eq!(*c, original[2]);
        app.undo();
        assert_eq!(app.contours, original);
    }
    #[test]
    fn radius_groups_are_rounded_once_and_floor_scope_is_respected() {
        let points = vec![(1, [2., 0.]), (1, [7., 0.]), (2, [9., 0.])];
        let m = clustered(points, 50.);
        assert_eq!(m[&(1, [2., 0.].map(f64::to_bits))], [4.5, 0.]);
        assert_eq!(m[&(1, [7., 0.].map(f64::to_bits))], [4.5, 0.]);
        assert_eq!(m[&(2, [9., 0.].map(f64::to_bits))], [9., 0.]);
        let m = clustered(vec![(0, [2., 0.]), (0, [7., 0.]), (0, [9., 0.])], 50.);
        assert!((m[&(0, [2., 0.].map(f64::to_bits))][0] - 6.).abs() < 1e-10);
        let m = clustered(vec![(0, [0., 0.]), (0, [40., 0.]), (0, [80., 0.])], 50.);
        assert_ne!(
            m[&(0, [0., 0.].map(f64::to_bits))],
            m[&(0, [80., 0.].map(f64::to_bits))]
        );
    }
    #[test]
    fn rounding_snaps_selected_contour_to_unselected_locked_vertex() {
        let mut app = App::default();
        for [a, b] in [[[39., 3.], [39., 1003.]], [[14., 3.], [-986., 3.]]] {
            app.begin(Kind::Support);
            app.sketch_click(a, false);
            app.sketch_click(b, false);
            app.finish();
            app.cancel();
        }
        app.contours[0].placement.movement_locked = true;
        app.rounding_options.cluster = true;
        let before = app.contours.clone();
        app.select_only(Some(before[1].id));
        assert_eq!(app.round_coordinates().unwrap(), 1);
        assert_eq!(app.contours[0], before[0]);
        assert_eq!(app.contours[1].points, [[39., 3.], [-990., 0.]]);
        app.undo();
        assert_eq!(app.contours, before);
        app.select_only(Some(before[0].id));
        let history = app.history.len();
        assert_eq!(app.round_coordinates().unwrap(), 0);
        assert_eq!(app.history.len(), history);
        assert_eq!(app.contours, before);
    }
    #[test]
    fn radius_uses_the_nearest_fixed_vertex_in_scope() {
        let mut points = vec![(1, [4., 0.]), (1, [5., 0.]), (1, [6., 0.]), (2, [4., 0.])];
        let snapped = snap_to_fixed_vertices(&mut points, vec![(1, [10., 0.]), (1, [0., 0.])], 5.);
        assert_eq!(snapped[&(1, [4., 0.].map(f64::to_bits))], [0., 0.]);
        assert_eq!(snapped[&(1, [5., 0.].map(f64::to_bits))], [0., 0.]);
        assert_eq!(snapped[&(1, [6., 0.].map(f64::to_bits))], [10., 0.]);
        assert_eq!(points, [(2, [4., 0.])]);
    }
    #[test]
    fn all_contour_types_and_loads_round_atomically_and_undo_as_one_operation() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([2., 3.], false);
        a.sketch_click([6002., 4003.], false);
        a.begin(Kind::Hole);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([1004., 1004.], false);
        a.sketch_click([2004., 2004.], false);
        a.begin(Kind::MeshRegion);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([3004., 1004.], false);
        a.sketch_click([4004., 2004.], false);
        a.begin(Kind::Support);
        a.sketch_click([3., 504.], false);
        a.sketch_click([5997., 504.], false);
        a.finish();
        a.begin(Kind::LoadPoint);
        a.sketch_click([2504., 2503.], false);
        a.begin(Kind::LoadLine);
        a.sketch_click([1504., 2504.], false);
        a.sketch_click([3504., 3504.], false);
        a.finish();
        a.begin(Kind::LoadArea);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([4204., 504.], false);
        a.sketch_click([5204., 1504.], false);
        assert_eq!(a.contours.len(), 7);
        a.select_only(None);
        let before = a.contours.clone();
        let history = a.history.len();
        assert_eq!(a.round_coordinates().unwrap(), 7);
        assert_eq!(a.history.len(), history + 1);
        for (old, new) in before.iter().zip(&a.contours) {
            assert_eq!(old.id, new.id);
            assert_eq!(old.parent, new.parent);
            assert_eq!(old.load, new.load);
            assert_eq!(old.refinement, new.refinement);
            assert!(new
                .points
                .iter()
                .flatten()
                .all(|v| (*v / 10. - (*v / 10.).round()).abs() < 1e-10));
        }
        assert_eq!(a.round_coordinates().unwrap(), 0);
        assert_eq!(a.history.len(), history + 1);
        a.undo();
        assert_eq!(a.contours, before);
    }
    #[test]
    fn collapsed_geometry_or_locks_do_not_partially_modify_any_contour() {
        let mut a = App::default();
        a.begin(Kind::Support);
        a.sketch_click([1., 1.], false);
        a.sketch_click([4., 4.], false);
        a.finish();
        let before = a.contours.clone();
        let history = a.history.len();
        assert!(a.round_coordinates().is_err());
        assert_eq!(a.contours, before);
        assert_eq!(a.history.len(), history);
        a.rounding_step = 1.;
        a.contour_locks[2] = true;
        assert!(a.round_coordinates().is_err());
        assert_eq!(a.contours, before);
        a.contour_locks[2] = false;
        a.rounding_step = f64::NAN;
        assert!(a.round_coordinates().is_err());
    }
    #[test]
    fn analytic_arc_remains_an_arc_and_only_its_controls_are_rounded() {
        let mut a = App::default();
        a.begin(Kind::Support);
        a.choose_tool(Tool::Arc);
        for p in [[1003., 1003.], [1504., 2004.], [2503., 1003.]] {
            a.sketch_click(p, false);
        }
        a.finish();
        assert!(a.contours[0]
            .curves
            .iter()
            .any(|c| matches!(c, Curve::Arc(_))));
        a.round_coordinates().unwrap();
        assert!(a.contours[0]
            .curves
            .iter()
            .any(|c| matches!(c, Curve::Arc(_))));
        assert!(a.contours[0]
            .handles()
            .iter()
            .flatten()
            .all(|v| (*v / 10. - (*v / 10.).round()).abs() < 1e-10));
        assert!(a.contours[0]
            .points
            .iter()
            .flatten()
            .any(|v| (*v / 10. - (*v / 10.).round()).abs() > 1e-4));
    }
}

#[cfg(test)]
mod selected_scope_regression {
    use super::*;
    #[test]
    fn rounding_changes_only_selected_source_and_undo_keeps_other_object() {
        let mut a = App::default();
        for x in [2., 2002.] {
            a.begin(Kind::Slab);
            a.choose_tool(Tool::Rectangle);
            a.sketch_click([x, 3.], false);
            a.sketch_click([x + 1000., 1003.], false);
            a.cancel();
        }
        let before = a.contours.clone();
        a.select_only(Some(before[0].id));
        assert_eq!(a.round_coordinates().unwrap(), 1);
        assert_eq!(a.contours[1], before[1]);
        a.undo();
        assert_eq!(a.contours, before);
    }
}
