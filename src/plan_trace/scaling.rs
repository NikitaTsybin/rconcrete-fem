use super::*;
use crate::plan_trace::underlay::{Data as UnderlayData, Transform as UnderlayTransform, Underlay};
#[derive(Clone, Copy, PartialEq)]
pub(super) enum Mode {
    Factor,
    Reference,
    Move,
    Rotate,
}
#[derive(Clone, Copy)]
pub(super) enum SpatialPreview {
    Move(Point),
    Rotate { center: Point, angle: f64 },
}
impl SpatialPreview {
    fn point(&self, point: Point) -> Point {
        match *self {
            Self::Move(delta) => [point[0] + delta[0], point[1] + delta[1]],
            Self::Rotate { center, angle } => rotate_point(point, center, angle),
        }
    }
}
pub(super) struct Operation {
    mode: Mode,
    center: Option<Point>,
    reference: Option<Point>,
    factor: f64,
    ids: std::collections::BTreeSet<u64>,
    underlays: std::collections::BTreeSet<u64>,
    dimensions: std::collections::BTreeSet<u64>,
    dialog_at: Pos2,
}

#[cfg(test)]
mod move_tests {
    use super::*;
    #[test]
    fn move_keeps_ids_and_base_offset_moves_children_and_is_one_undo() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([4000., 3000.], false);
        a.begin(Kind::Hole);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([500., 500.], false);
        a.sketch_click([1000., 1000.], false);
        a.measurements.items = vec![measurements::Measurement {
            spatial: None,
            id: 1,
            a: [0., 0.],
            b: [1000., 0.],
            offset: Some(200.),
        }];
        a.underlays.push(Underlay {
            id: 1,
            name: "page.pdf".into(),
            visible: true,
            transparency: 0.25,
            origin: [50., 60.],
            scale: 2.,
            scale_y: Some(3.),
            angle_rad: 0.25,
            data: UnderlayData::Pdf {
                width: 1,
                height: 1,
                rgba: vec![255; 4],
                size_mm: [210., 297.],
                page: 1,
            },
        });
        a.select_all_scene([1], [1], [1], egui::Modifiers::default());
        let original = a.contours.clone();
        let depth = a.history.len();
        let next_id = a.next_id;
        a.begin_scaling(Mode::Move);
        a.scaling_op.as_mut().unwrap().center = Some([100., 200.]);
        assert_eq!(a.contours, original);
        a.commit_move([1100., -800.]).unwrap();
        assert_eq!(a.history.len(), depth + 1);
        assert_eq!(a.next_id, next_id);
        for (old, new) in original.iter().zip(&a.contours) {
            assert_eq!(old.id, new.id);
            assert_eq!(old.parent, new.parent);
            for (p, q) in old.points.iter().zip(&new.points) {
                assert!((q[0] - p[0] - 1000.).abs() < 1e-8);
                assert!((q[1] - p[1] + 1000.).abs() < 1e-8);
            }
        }
        assert_eq!(a.underlays[0].origin, [1050., -940.]);
        assert_eq!(a.underlays[0].scale, 2.);
        assert_eq!(a.underlays[0].transform().scale, [2., 3.]);
        assert_eq!(a.underlays[0].transform().angle, 0.25);
        assert_eq!(a.underlays[0].transparency, 0.25);
        assert_eq!(a.measurements.items[0].a, [1000., -1000.]);
        assert_eq!(a.measurements.items[0].offset, Some(200.));
        a.undo();
        assert_eq!(a.contours, original);
        assert_eq!(a.history.len(), depth);
        assert_eq!(a.underlays[0].origin, [50., 60.]);
        assert_eq!(a.measurements.items[0].a, [0., 0.]);
        a.select_only(Some(1));
        a.begin_scaling(Mode::Move);
        a.scaling_op.as_mut().unwrap().center = Some([300., 400.]);
        a.cancel();
        assert_eq!(a.contours, original);
        assert_eq!(a.history.len(), depth);
        a.select_only(Some(2));
        a.begin_scaling(Mode::Move);
        a.scaling_op.as_mut().unwrap().center = Some([0., 0.]);
        assert!(a.commit_move([100000., 0.]).is_err());
        assert_eq!(a.contours, original);
        assert_eq!(a.history.len(), depth);
    }
    #[test]
    fn spatial_move_follows_cursor_at_both_wall_elevations() {
        use std::cell::RefCell;

        let mut app = App::new_project();
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.change_floor(2);
        app.begin(Kind::Support);
        app.sketch_click([100., 100.], false);
        app.sketch_click([200., 100.], false);
        app.finish();
        app.sync_building_objects();
        assert_eq!(app.source_heights(&app.contours[0]), [3000., 6000.]);
        app.change_floor(1);
        app.spatial.enabled = true;
        app.selection_all_floors = true;
        app.select_only(Some(app.contours[0].id));
        app.begin_scaling(Mode::Move);
        app.scaling_op.as_mut().unwrap().center = Some([10., 20.]);
        let source = app.contours.clone();
        let depth = app.history.len();
        let projected = RefCell::new(Vec::new());
        let project = |p: [f64; 3]| {
            projected.borrow_mut().push(p);
            (Pos2::new((p[0] + p[2] / 100.) as f32, p[1] as f32), p[2])
        };
        let ctx = egui::Context::default();
        for _ in 0..2 {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.))),
                    events: vec![egui::Event::PointerMoved(Pos2::new(150., 200.))],
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| {
                        let (rect, response) =
                            ui.allocate_exact_size(ui.available_size(), egui::Sense::click());
                        app.spatial_source_interaction(
                            ui,
                            &response,
                            rect,
                            &project,
                            0.,
                            &[],
                            None,
                        );
                    });
                },
            );
        }
        assert_eq!(app.contours, source);
        assert_eq!(app.history.len(), depth);
        for z in [3000., 6000.] {
            assert!(projected.borrow().iter().any(|p| {
                (p[0] - 240.).abs() < 0.01 && (p[1] - 280.).abs() < 0.01 && p[2] == z
            }));
        }
        app.commit_move([150., 200.]).unwrap();
        assert_eq!(app.contours[0].points[0], [240., 280.]);
    }
    #[test]
    fn foundation_exclusion_follows_move_scale_and_rotation() {
        let mut app = App::default();
        app.begin(Kind::Foundation);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        let contour = &mut app.contours[0];
        contour
            .foundation
            .get_or_insert_with(Default::default)
            .exclusions = vec![vec![[400., 400.], [600., 400.], [600., 600.], [400., 600.]]];

        let moved = transform(contour, [0., 0.], 1., [10., 20.]).unwrap();
        let scaled = transform(&moved, [10., 20.], 2., [0., 0.]).unwrap();
        let rotated = rotate_contour(&scaled, [10., 20.], std::f64::consts::FRAC_PI_2).unwrap();
        let data = rotated.foundation.as_ref().unwrap();
        let first = data.exclusions[0][0];
        assert!((first[0] + 790.).abs() < 1e-9);
        assert!((first[1] - 820.).abs() < 1e-9);
        assert!(!data.covers(&rotated.points, [-990., 1020.]));
        assert!(data.covers(&rotated.points, [-1500., 2000.]));
    }
}
pub(super) struct Undo {
    depth: usize,
    underlays: Vec<(u64, UnderlayTransform)>,
    dimensions: Vec<measurements::Measurement>,
    geometry: bool,
    calculation: bool,
}
fn transform(c: &Contour, center: Point, factor: f64, delta: Point) -> Result<Contour, String> {
    let mut c = c.clone();
    if factor == 1. {
        c.translate(delta)?;
        return Ok(c);
    }
    let map = |p: &mut Point| {
        for axis in 0..2 {
            p[axis] = center[axis] + (p[axis] - center[axis]) * factor + delta[axis];
        }
    };
    if let Some(grid) = &mut c.grid {
        map(&mut grid.origin);
        for value in grid.ex.iter_mut().chain(&mut grid.ey) {
            *value *= factor;
        }
    } else if c.curves.is_empty() {
        for p in &mut c.points {
            map(p);
        }
    } else {
        for curve in &mut c.curves {
            curve.map(map);
        }
    }
    if let Some(data) = &mut c.foundation {
        data.map_xy(map);
    }
    c.rebuild()?;
    if c.points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("Слишком большие координаты".into());
    }
    Ok(c)
}
fn rotation_angle(center: Point, reference: Point, target: Point) -> Option<f64> {
    let a = [reference[0] - center[0], reference[1] - center[1]];
    let b = [target[0] - center[0], target[1] - center[1]];
    if a[0].hypot(a[1]) <= 1e-6 || b[0].hypot(b[1]) <= 1e-6 {
        return None;
    }
    Some((a[0] * b[1] - a[1] * b[0]).atan2(a[0] * b[0] + a[1] * b[1]))
}
fn rotate_point(p: Point, center: Point, angle: f64) -> Point {
    let (s, c) = angle.sin_cos();
    let x = p[0] - center[0];
    let y = p[1] - center[1];
    [center[0] + c * x - s * y, center[1] + s * x + c * y]
}
fn rotate_underlay(
    mut transform: UnderlayTransform,
    center: Point,
    angle: f64,
) -> UnderlayTransform {
    transform.origin = rotate_point(transform.origin, center, angle);
    transform.angle = (transform.angle + angle).rem_euclid(std::f64::consts::TAU);
    transform
}
fn rotate_contour(c: &Contour, center: Point, angle: f64) -> Result<Contour, String> {
    let mut c = c.clone();
    if c.kind == Kind::Column {
        c.column_angle = (c.column_angle + angle).rem_euclid(std::f64::consts::TAU);
    }
    if let Some(grid) = &mut c.grid {
        grid.origin = rotate_point(grid.origin, center, angle);
        grid.ex = rotate_point(grid.ex, [0., 0.], angle);
        grid.ey = rotate_point(grid.ey, [0., 0.], angle);
    } else if c.curves.is_empty() {
        for p in &mut c.points {
            *p = rotate_point(*p, center, angle);
        }
    } else {
        for curve in &mut c.curves {
            curve.map(|p| *p = rotate_point(*p, center, angle));
        }
    }
    if let Some(data) = &mut c.foundation {
        data.map_xy(|p| *p = rotate_point(*p, center, angle));
    }
    c.rebuild()?;
    if c.points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("Некорректные координаты поворота".into());
    }
    Ok(c)
}
impl App {
    fn draw_move_preview(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        target: Point,
        delta: Point,
        time: f64,
    ) {
        let Some(op) = &self.scaling_op else {
            return;
        };
        let alpha = if (time * 3.).floor() as i64 % 2 == 0 {
            0.85
        } else {
            0.35
        };
        let mut camera = self.camera;
        for axis in 0..2 {
            camera.center[axis] -= delta[axis];
        }
        if camera.center.iter().any(|v| !v.is_finite()) {
            return;
        }
        for c in self.contours.iter().filter(|c| op.ids.contains(&c.id)) {
            let ink = self.object_ink(c.id, c.kind).gamma_multiply(alpha);
            if c.kind == Kind::LoadPoint {
                let p = camera.screen(c.points[0], rect);
                if rect.contains(p) {
                    painter.circle_stroke(p, 7., Stroke::new(2_f32, ink));
                    for d in [egui::vec2(5., 5.), egui::vec2(5., -5.)] {
                        painter.line_segment([p - d, p + d], Stroke::new(1.5_f32, ink));
                    }
                }
            } else {
                let stroke = Stroke::new(2.5_f32, ink);
                draw_path(painter, &c.points, c.closed(), camera, rect, stroke);
                foundation_ui::draw_foundation_exclusions(painter, c, camera, rect, stroke);
            }
        }
        for m in self
            .measurements
            .items
            .iter()
            .filter(|m| op.dimensions.contains(&m.id))
        {
            let mut m = m.clone();
            for p in [&mut m.a, &mut m.b] {
                for axis in 0..2 {
                    p[axis] += delta[axis];
                }
            }
            self.draw_dimension_alpha(painter, rect, &m, alpha);
        }
        for u in self
            .underlays
            .iter()
            .filter(|u| op.underlays.contains(&u.id))
        {
            self.draw_underlay_transform_preview(painter, rect, camera, u, u.transform(), alpha);
        }
        painter.circle_stroke(
            self.camera.screen(target, rect),
            5.,
            Stroke::new(1.5_f32, Color32::LIGHT_BLUE),
        );
    }
    fn draw_underlay_transform_preview(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        camera: Camera,
        u: &Underlay,
        transform: UnderlayTransform,
        alpha: f32,
    ) {
        let opacity = alpha * (1. - u.transparency);
        let ex = transform.vector([1., 0.]);
        let ey = transform.vector([0., 1.]);
        let point = |p: Point| {
            std::array::from_fn(|axis| transform.origin[axis] + p[0] * ex[axis] + p[1] * ey[axis])
        };
        match &u.data {
            UnderlayData::Pdf { size_mm, .. } => {
                let Some(texture) = self.pdf_textures.get(&u.id) else {
                    return;
                };
                let corners = [[0., 0.], [size_mm[0], 0.], *size_mm, [0., size_mm[1]]]
                    .map(|p| camera.screen(point(p), rect));
                if corners.iter().any(|p| !p.x.is_finite() || !p.y.is_finite())
                    || !Rect::from_points(&corners).intersects(rect)
                {
                    return;
                }
                let mut mesh = egui::Mesh::with_texture(texture.id());
                for (pos, uv) in corners
                    .into_iter()
                    .zip([[0., 1.], [1., 1.], [1., 0.], [0., 0.]])
                {
                    mesh.vertices.push(egui::epaint::Vertex {
                        pos,
                        uv: egui::pos2(uv[0], uv[1]),
                        color: Color32::WHITE.gamma_multiply(opacity),
                    });
                }
                mesh.add_triangle(0, 1, 2);
                mesh.add_triangle(0, 2, 3);
                painter.add(egui::Shape::mesh(mesh));
            }
            UnderlayData::Dxf(d) => {
                let style = |layer: &str| self.layers.get(&format!("#{} · {layer}", u.id));
                let mut fills = egui::Mesh::default();
                for (triangles, color, layer) in &d.fills {
                    let s = style(layer);
                    if s.is_some_and(|s| !s.visible) {
                        continue;
                    }
                    let c = color.rgb();
                    let ink = s
                        .map_or(Color32::from_rgb(c[0], c[1], c[2]), |s| s.ink(c))
                        .gamma_multiply(opacity);
                    for &t in triangles {
                        append_triangle(&mut fills, t.map(point), camera, rect, ink);
                    }
                }
                if !fills.is_empty() {
                    painter.add(egui::Shape::mesh(fills));
                }
                for path in &d.paths {
                    let s = style(&path.layer);
                    if s.is_some_and(|s| !s.visible) {
                        continue;
                    }
                    let c = path.color.rgb();
                    let ink = s
                        .map_or(Color32::from_rgb(c[0], c[1], c[2]), |s| s.ink(c))
                        .gamma_multiply(opacity);
                    let stroke = Stroke::new(s.map_or(1., |s| s.width), ink);
                    let points = path
                        .points
                        .iter()
                        .copied()
                        .map(|p| camera.screen64(point(p), rect));
                    let n = path.points.len().saturating_sub(usize::from(!path.closed));
                    for (a, b) in points.clone().zip(points.cycle().skip(1)).take(n) {
                        if let Some(segment) = clip_line(a, b, rect.expand(stroke.width + 1.)) {
                            painter.line_segment(segment, stroke);
                        }
                    }
                }
                if self.show_text {
                    for label in &d.labels {
                        let s = style(&label.layer);
                        if s.is_some_and(|s| !s.visible) {
                            continue;
                        }
                        let c = label.color.rgb();
                        let mut ink = s.map_or(Color32::from_rgb(c[0], c[1], c[2]), |s| s.ink(c));
                        if self.contrast_text
                            && s.is_none_or(|s| s.color.is_none())
                            && (ink.r() as u16 + ink.g() as u16 + ink.b() as u16) < 140
                        {
                            ink = Color32::from_gray(225);
                        }
                        ink = ink.gamma_multiply(opacity);
                        let mut label = label.clone();
                        transform.transform_label(&mut label);
                        let galley = painter.layout_no_wrap(
                            label.text,
                            egui::FontId::proportional(
                                (label.height * camera.zoom).clamp(7., 120.) as f32,
                            ),
                            ink,
                        );
                        let offset = egui::vec2(
                            galley.size().x * label.anchor[0],
                            galley.size().y * label.anchor[1],
                        );
                        let angle = -(label.angle + camera.angle) as f32;
                        let rotated = egui::vec2(
                            angle.cos() * offset.x - angle.sin() * offset.y,
                            angle.sin() * offset.x + angle.cos() * offset.y,
                        );
                        let at = camera.screen(label.point, rect) - rotated;
                        if at.x.is_finite() && at.y.is_finite() {
                            painter.add(
                                egui::epaint::TextShape::new(at, galley, ink).with_angle(angle),
                            );
                        }
                    }
                }
            }
        }
    }
    pub(super) fn scaling_menu(&mut self, ui: &mut egui::Ui) {
        if self.mode.is_some()
            || self.clipboard_op.is_some()
            || self.measurements.active
            || self.arc_placement.is_some()
        {
            return;
        }
        if self.selected.is_none()
            && self.selection_ids.is_empty()
            && self.selected_underlay.is_none()
            && self.underlay_selection.is_empty()
            && self.measurements.selected.is_empty()
        {
            return;
        }
        if ui.button("Переместить").clicked() {
            self.begin_scaling(Mode::Move);
            ui.close_menu();
            return;
        }
        if ui.button("Повернуть").clicked() {
            self.begin_scaling(Mode::Rotate);
            ui.close_menu();
            return;
        }
        ui.menu_button("Масштабирование", |ui| {
            for (mode, text) in [
                (Mode::Factor, "Относительно точки"),
                (Mode::Reference, "По опорному отрезку"),
            ] {
                if ui.button(text).clicked() {
                    self.begin_scaling(mode);
                    ui.close_menu();
                }
            }
        });
    }
    fn begin_scaling(&mut self, mode: Mode) {
        self.prune_selection();
        let mut ids = self.selection_ids.clone();
        ids.extend(self.selected);
        let movement_locked = self.movement_locked_contours();
        for link in self
            .contours
            .iter()
            .filter(|c| c.placement.rigid_link_targets.is_some())
        {
            ids.remove(&link.id);
        }
        // Parents retain their associated openings and refinement zones.
        loop {
            let n = ids.len();
            let children = self
                .contours
                .iter()
                .filter(|c| c.parent.is_some_and(|id| ids.contains(&id)))
                .map(|c| c.id)
                .collect::<Vec<_>>();
            ids.extend(children);
            if ids.len() == n {
                break;
            }
        }
        if ids.iter().any(|id| movement_locked.contains(id))
            || self
                .contours
                .iter()
                .any(|c| ids.contains(&c.id) && self.is_kind_locked(c.kind))
        {
            self.status =
                "Для преобразования снимите блокировку выбранных объектов и их дочерних объектов"
                    .into();
            return;
        }
        let mut underlays = self.underlay_selection.clone();
        underlays.extend(self.selected_underlay);
        if (self.input_locked || self.underlays_locked) && !underlays.is_empty() {
            self.status = "Подложки PDF / DXF заблокированы".into();
            return;
        }
        let dimensions = self.measurements.selected.clone();
        if ids.is_empty() && underlays.is_empty() && dimensions.is_empty() {
            return;
        }
        self.scaling_op = Some(Operation {
            mode,
            center: None,
            reference: None,
            factor: 1.,
            ids,
            underlays,
            dimensions,
            dialog_at: Pos2::ZERO,
        });
        self.selection_start = None;
        self.status = if mode == Mode::Rotate {
            "Поворот: укажите центр поворота. Esc — отмена"
        } else if mode == Mode::Move {
            "Перемещение: укажите базовую точку. Esc — отмена"
        } else {
            "Масштабирование: укажите базовую точку. Esc — отмена"
        }
        .into();
    }
    fn commit_scaling(&mut self, factor: f64) -> Result<(), String> {
        self.commit_transform(factor, [0., 0.])
    }
    fn commit_move(&mut self, target: Point) -> Result<(), String> {
        let base = self
            .scaling_op
            .as_ref()
            .and_then(|o| o.center)
            .ok_or("Укажите базовую точку")?;
        self.commit_transform(1., [target[0] - base[0], target[1] - base[1]])
    }
    fn commit_transform(&mut self, factor: f64, delta: Point) -> Result<(), String> {
        if delta.iter().any(|v| !v.is_finite()) {
            return Err("Некорректное смещение".into());
        }
        let op = self
            .scaling_op
            .as_ref()
            .ok_or("Масштабирование не начато")?;
        let center = op.center.ok_or("Укажите базовую точку")?;
        let rotating = op.mode == Mode::Rotate;
        let angle = op.factor;
        if rotating && !angle.is_finite() {
            return Err("Некорректные параметры поворота".into());
        }
        if (self.input_locked || self.underlays_locked) && !op.underlays.is_empty() {
            return Err("Подложки PDF / DXF заблокированы".into());
        }
        let movement_locked = self.movement_locked_contours();
        if op.ids.iter().any(|id| movement_locked.contains(id)) {
            return Err("У выбранного объекта заблокировано перемещение.".into());
        }
        if !factor.is_finite() || factor <= 0. || !(1e-9..=1e9).contains(&factor) {
            return Err("Введите положительный коэффициент масштаба".into());
        }
        let mut next = (!op.ids.is_empty()).then(|| self.contours.clone());
        let mut geometry = false;
        let mut calculation = false;
        if let Some(contours) = &mut next {
            for c in contours.iter_mut().filter(|c| op.ids.contains(&c.id)) {
                if self.is_kind_locked(c.kind) {
                    return Err("Объект заблокирован".into());
                }
                geometry |= c.is_mesh_input();
                calculation |= c.kind.affects_calculation();
                *c = if rotating {
                    rotate_contour(c, center, angle)?
                } else {
                    transform(c, center, factor, delta)?
                };
            }
            for c in contours.iter() {
                model::validate_candidate(contours, c)?;
            }
        }
        let mut changes = Vec::new();
        let mut updates = Vec::new();
        for id in &op.underlays {
            let u = self
                .underlays
                .iter()
                .find(|u| u.id == *id)
                .ok_or("Подложка удалена")?;
            let previous = u.transform();
            let updated = if rotating {
                rotate_underlay(previous, center, angle)
            } else {
                UnderlayTransform {
                    origin: std::array::from_fn(|axis| {
                        let value = previous.origin[axis];
                        if factor == 1. {
                            value + delta[axis]
                        } else {
                            center[axis] + (value - center[axis]) * factor + delta[axis]
                        }
                    }),
                    scale: previous.scale.map(|scale| scale * factor),
                    angle: previous.angle,
                }
            };
            updated.validate()?;
            if !rotating
                && updated
                    .scale
                    .iter()
                    .any(|scale| !(1e-9..=1e9).contains(scale))
            {
                return Err("Некорректный масштаб подложки".into());
            }
            changes.push((*id, previous));
            updates.push((*id, updated));
        }
        let before_dimensions = self
            .measurements
            .items
            .iter()
            .filter(|m| op.dimensions.contains(&m.id))
            .cloned()
            .collect::<Vec<_>>();
        let mut dimensions = before_dimensions.clone();
        for m in &mut dimensions {
            for p in [&mut m.a, &mut m.b] {
                if rotating {
                    *p = rotate_point(*p, center, angle);
                    continue;
                }
                for axis in 0..2 {
                    p[axis] = if factor == 1. {
                        p[axis] + delta[axis]
                    } else {
                        center[axis] + (p[axis] - center[axis]) * factor + delta[axis]
                    };
                }
            }
            if let Some(offset) = &mut m.offset {
                *offset *= factor;
            }
            m.validate()?;
        }
        let moving = op.mode == Mode::Move;
        if factor == 1. && delta == [0., 0.] && (!rotating || angle.abs() < 1e-12) {
            self.scaling_op = None;
            return Ok(());
        }
        let has_underlays = !op.underlays.is_empty();
        self.history.push(self.contours.clone());
        self.scaling_history.push(Undo {
            depth: self.history.len(),
            underlays: changes,
            dimensions: before_dimensions,
            geometry,
            calculation,
        });
        if let Some(next) = next {
            self.contours = next;
        }
        for next in dimensions {
            if let Some(m) = self.measurements.items.iter_mut().find(|m| m.id == next.id) {
                *m = next;
            }
        }
        for (id, transform) in updates {
            self.set_underlay_transform(id, transform);
        }
        if has_underlays {
            self.sync_underlays();
        }
        self.scaling_op = None;
        self.transform_snap = None;
        if geometry {
            self.fe_mesh = None;
            self.discard_mesh_job();
            self.slab_cache = None;
        }
        if calculation {
            self.analysis = None;
            self.analysis_input = None;
            self.analysis_job = None;
            self.combination_cache = None;
        }
        self.invalidate_scene_buffers();
        self.status = if rotating {
            format!(
                "Поворот: {:.2}°. Ctrl+Z — отменить всю группу",
                angle.to_degrees()
            )
        } else if moving {
            "Перемещение выполнено. Ctrl+Z — отменить всю группу".into()
        } else {
            format!("Масштаб: {factor}. Ctrl+Z — отменить всю операцию")
        };
        Ok(())
    }
    pub(super) fn spatial_transform_preview(
        &self,
        painter: &egui::Painter,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        preview: SpatialPreview,
    ) {
        let Some(op) = &self.scaling_op else {
            return;
        };
        let screen = |p, z| {
            let [x, y] = preview.point(p);
            project([x, y, z]).0
        };
        for c in self.contours.iter().filter(|c| op.ids.contains(&c.id)) {
            let section = self
                .support_height
                .columns
                .get(&c.id)
                .filter(|_| c.kind == Kind::Column)
                .map(|section| crate::plan_trace::columns::section_corners(c, section));
            let path = section
                .as_ref()
                .map_or(c.points.as_slice(), |p| p.as_slice());
            let edge_count = if c.closed() || section.is_some() {
                path.len()
            } else {
                path.len().saturating_sub(1)
            };
            let ink = match preview {
                SpatialPreview::Move(_) => self.object_ink(c.id, c.kind).gamma_multiply(0.85),
                SpatialPreview::Rotate { .. } => Color32::WHITE,
            };
            let stroke = Stroke::new(2_f32, ink);
            let mut heights = self.source_heights(c);
            if matches!(c.kind, Kind::LoadArea | Kind::LoadLine) {
                heights.push(heights[0] + spatial_loads::height(c.kind));
            }
            for &z in &heights {
                if path.len() == 1 {
                    painter.circle_stroke(screen(path[0], z), 6., stroke);
                }
                for (&a, &b) in path
                    .iter()
                    .zip(path.iter().cycle().skip(1))
                    .take(edge_count)
                {
                    painter.line_segment([screen(a, z), screen(b, z)], stroke);
                }
                if let Some(data) = &c.foundation {
                    for ring in &data.exclusions {
                        for (&a, &b) in ring.iter().zip(ring.iter().cycle().skip(1)) {
                            painter.line_segment([screen(a, z), screen(b, z)], stroke);
                        }
                    }
                }
            }
            if heights.len() > 1 {
                for &p in path {
                    painter.line_segment(
                        [screen(p, heights[0]), screen(p, *heights.last().unwrap())],
                        stroke,
                    );
                }
            }
        }
        let stroke = Stroke::new(2_f32, Color32::WHITE);
        for &id in &op.underlays {
            let Some(corners) = self.underlay_corners(id) else {
                continue;
            };
            let z = if self.building.enabled {
                let floor = self
                    .building
                    .underlay_floors
                    .get(&id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id);
                self.building.floor(floor).elevation_mm
            } else {
                0.
            };
            for (&a, &b) in corners.iter().zip(corners.iter().cycle().skip(1)) {
                painter.line_segment([screen(a, z), screen(b, z)], stroke);
            }
        }
        let floor_z = if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        };
        for m in self
            .measurements
            .items
            .iter()
            .filter(|m| op.dimensions.contains(&m.id))
        {
            let [a, b] = m.xyz();
            let [sx, sy, sz] = m.spatial.as_ref().map_or([0.; 3], |s| s.shift);
            let ends = [a, b].map(|[x, y, z]| {
                let z = if m.spatial.is_some() { z } else { floor_z };
                let [x, y] = preview.point([x, y]);
                [x, y, z]
            });
            let shifted = ends.map(|[x, y, z]| project([x + sx, y + sy, z + sz]).0);
            painter.line_segment(shifted, stroke);
            for (end, offset) in ends.into_iter().zip(shifted) {
                painter.line_segment([project(end).0, offset], stroke);
            }
        }
    }
    pub(super) fn record_measurement_edit(&mut self, before: measurements::Measurement) {
        self.history.push(self.contours.clone());
        self.scaling_history.push(Undo {
            depth: self.history.len(),
            underlays: vec![],
            dimensions: vec![before],
            geometry: false,
            calculation: false,
        });
    }
    pub(super) fn undo_scaling(&mut self) -> bool {
        let Some(last) = self.scaling_history.last() else {
            return false;
        };
        if last.depth != self.history.len()
            || self
                .underlay_history
                .last()
                .is_some_and(|u| u.history >= last.depth)
        {
            return false;
        }
        if self
            .history
            .last()
            .is_some_and(|before| self.undo_changes_locked(before))
        {
            self.status = "Отмена преобразования затрагивает заблокированные объекты".into();
            return true;
        }
        if !last.underlays.is_empty() && (self.input_locked || self.underlays_locked) {
            self.status = "Снимите блокировку подложек для отмены".into();
            return true;
        }
        let undo = self.scaling_history.pop().unwrap();
        self.contours = self.history.pop().unwrap();
        for (id, transform) in undo.underlays {
            self.set_underlay_transform(id, transform);
        }
        for before in undo.dimensions {
            if let Some(m) = self
                .measurements
                .items
                .iter_mut()
                .find(|m| m.id == before.id)
            {
                *m = before;
            }
        }
        self.sync_underlays();
        if undo.geometry {
            self.slab_cache = None;
            self.fe_mesh = None;
            self.discard_mesh_job();
        }
        if undo.calculation {
            self.analysis = None;
            self.analysis_input = None;
            self.analysis_job = None;
            self.combination_cache = None;
        }
        self.select_only(None);
        self.transform_snap = None;
        self.invalidate_scene_buffers();
        self.status = "Преобразование отменено".into();
        true
    }
    pub(super) fn scaling_dialog(&mut self, ctx: &egui::Context) {
        let Some(op) = self
            .scaling_op
            .as_mut()
            .filter(|o| o.mode == Mode::Factor && o.center.is_some())
        else {
            return;
        };
        let mut accept = false;
        let mut cancel = false;
        egui::Window::new("Масштабирование")
            .id(egui::Id::new("scaling_factor"))
            .default_pos(op.dialog_at)
            .collapsible(false)
            .resizable(false)
            .show(ctx, |ui| {
                ui.label("Коэффициент");
                ui.add(
                    egui::DragValue::new(&mut op.factor)
                        .range(1e-9..=1e9)
                        .speed(0.01),
                );
                ui.horizontal(|ui| {
                    accept = ui.button("Применить").clicked();
                    cancel = ui.button("Отмена").clicked();
                });
                accept |= ui.input(|i| i.key_pressed(egui::Key::Enter));
                cancel |= ui.input(|i| i.key_pressed(egui::Key::Escape));
            });
        let factor = op.factor;
        if cancel {
            self.cancel();
        } else if accept {
            if let Err(e) = self.commit_scaling(factor) {
                self.status = e;
            }
        }
    }
    pub(super) fn transform_snap_anchor(&self) -> Option<Point> {
        self.scaling_op.as_ref().and_then(|op| op.center)
    }
    pub(super) fn scaling_canvas(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> Option<SpatialPreview> {
        let op = self.scaling_op.as_ref()?;
        let pointer = hovered?;
        let mut spatial_preview = None;
        let raw = self.camera.world(pointer, rect);
        let found = self
            .transform_snap
            .or_else(|| self.snap(raw, None))
            .or_else(|| self.polar_snap(raw));
        let target = found.map_or(raw, |s| s.point);
        ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
        let painter = ui.painter().with_clip_rect(rect);
        self.draw_polar_guides(
            &painter,
            rect,
            raw,
            found.is_some_and(|s| s.kind == snap::Kind::Angle),
        );
        let hint = match (op.mode, op.center, op.reference) {
            (Mode::Move, None, _) => "Базовая точка",
            (Mode::Move, Some(_), _) => "Точка размещения",
            (Mode::Rotate, None, _) => "Центр поворота",
            (Mode::Rotate, Some(_), None) => "Начальная точка направления",
            (Mode::Rotate, Some(_), Some(_)) => "Конечная точка направления",
            (_, None, _) => "Центр масштабирования",
            (Mode::Factor, Some(_), _) => "Введите коэффициент масштабирования",
            (Mode::Reference, Some(_), None) => "Конец исходного опорного отрезка",
            (Mode::Reference, Some(_), Some(_)) => "Новая конечная точка опорного отрезка",
        };
        painter.text(
            pointer + egui::vec2(14., 14.),
            egui::Align2::LEFT_TOP,
            hint,
            egui::FontId::proportional(13.),
            Color32::LIGHT_BLUE,
        );
        if op.mode == Mode::Rotate {
            if let Some(center) = op.center {
                let a = self.camera.screen(center, rect);
                let b = self.camera.screen(target, rect);
                painter.line_segment([a, b], Stroke::new(1_f32, Color32::LIGHT_BLUE));
                painter.circle_stroke(a, 4., Stroke::new(1.5_f32, Color32::LIGHT_BLUE));
                if let Some(reference) = op.reference {
                    painter.line_segment(
                        [a, self.camera.screen(reference, rect)],
                        Stroke::new(1_f32, Color32::GRAY),
                    );
                    if let Some(angle) = rotation_angle(center, reference, target) {
                        if self.spatial.enabled {
                            spatial_preview = Some(SpatialPreview::Rotate { center, angle });
                        }
                        painter.text(
                            pointer + egui::vec2(14., 32.),
                            egui::Align2::LEFT_TOP,
                            format!("{:.2}°", angle.to_degrees()),
                            egui::FontId::proportional(13.),
                            Color32::LIGHT_BLUE,
                        );
                        for c in self
                            .contours
                            .iter()
                            .filter(|c| !self.spatial.enabled && op.ids.contains(&c.id))
                        {
                            if let Ok(c) = rotate_contour(c, center, angle) {
                                if c.kind == Kind::Column {
                                    if let Some(section) = self.support_height.columns.get(&c.id) {
                                        let q = crate::plan_trace::columns::section_corners(
                                            &c, section,
                                        );
                                        draw_path(
                                            &painter,
                                            &q,
                                            true,
                                            self.camera,
                                            rect,
                                            Stroke::new(2_f32, Color32::WHITE),
                                        );
                                        painter.circle_filled(
                                            self.camera.screen(c.points[0], rect),
                                            3.,
                                            Color32::WHITE,
                                        );
                                    }
                                } else if c.kind == Kind::LoadPoint {
                                    if let Some(p) = c.points.first() {
                                        painter.circle_stroke(
                                            self.camera.screen(*p, rect),
                                            6.,
                                            Stroke::new(2_f32, Color32::WHITE),
                                        );
                                    }
                                } else {
                                    let stroke = Stroke::new(2_f32, Color32::WHITE);
                                    draw_path(
                                        &painter,
                                        &c.points,
                                        c.closed(),
                                        self.camera,
                                        rect,
                                        stroke,
                                    );
                                    foundation_ui::draw_foundation_exclusions(
                                        &painter,
                                        &c,
                                        self.camera,
                                        rect,
                                        stroke,
                                    );
                                }
                            }
                        }
                        if !self.spatial.enabled {
                            for u in self
                                .underlays
                                .iter()
                                .filter(|u| op.underlays.contains(&u.id))
                            {
                                self.draw_underlay_transform_preview(
                                    &painter,
                                    rect,
                                    self.camera,
                                    u,
                                    rotate_underlay(u.transform(), center, angle),
                                    0.8,
                                );
                            }
                        }
                        for m in self
                            .measurements
                            .items
                            .iter()
                            .filter(|m| !self.spatial.enabled && op.dimensions.contains(&m.id))
                        {
                            let mut m = m.clone();
                            m.a = rotate_point(m.a, center, angle);
                            m.b = rotate_point(m.b, center, angle);
                            self.draw_dimension_alpha(&painter, rect, &m, 0.8);
                        }
                    }
                }
            }
        } else if let Some(center) = op.center.filter(|_| op.mode == Mode::Move) {
            let delta = [target[0] - center[0], target[1] - center[1]];
            if self.spatial.enabled {
                spatial_preview = Some(SpatialPreview::Move(delta));
            } else {
                self.draw_move_preview(&painter, rect, target, delta, ui.input(|i| i.time));
                ui.ctx().request_repaint_after(Duration::from_millis(150));
            }
        } else if let Some(center) = op.center {
            let factor = if op.mode == Mode::Factor {
                op.factor
            } else if let Some(reference) = op.reference {
                (target[0] - center[0]).hypot(target[1] - center[1])
                    / (reference[0] - center[0]).hypot(reference[1] - center[1])
            } else {
                1.
            };
            if factor.is_finite() && factor > 0. {
                for c in self.contours.iter().filter(|c| op.ids.contains(&c.id)) {
                    if let Ok(c) = transform(c, center, factor, [0., 0.]) {
                        let stroke = Stroke::new(2_f32, Color32::WHITE);
                        draw_path(&painter, &c.points, c.closed(), self.camera, rect, stroke);
                        foundation_ui::draw_foundation_exclusions(
                            &painter,
                            &c,
                            self.camera,
                            rect,
                            stroke,
                        );
                    }
                }
                for m in self
                    .measurements
                    .items
                    .iter()
                    .filter(|m| op.dimensions.contains(&m.id))
                {
                    let mut m = m.clone();
                    for p in [&mut m.a, &mut m.b] {
                        for axis in 0..2 {
                            p[axis] = center[axis] + (p[axis] - center[axis]) * factor;
                        }
                    }
                    if let Some(offset) = &mut m.offset {
                        *offset *= factor;
                    }
                    self.draw_dimension_alpha(&painter, rect, &m, 0.8);
                }
                for id in &op.underlays {
                    if let Some(q) = self.underlay_corners(*id) {
                        let q = q.map(|p| {
                            [
                                center[0] + (p[0] - center[0]) * factor,
                                center[1] + (p[1] - center[1]) * factor,
                            ]
                        });
                        draw_path(
                            &painter,
                            &q,
                            true,
                            self.camera,
                            rect,
                            Stroke::new(2_f32, Color32::WHITE),
                        );
                    }
                }
            }
            painter.line_segment(
                [
                    self.camera.screen(center, rect),
                    self.camera.screen(target, rect),
                ],
                Stroke::new(1_f32, Color32::LIGHT_BLUE),
            );
        }
        // Draw after previews so the snap marker stays visible over selected objects.
        if let Some(found) = found {
            let at = self.camera.screen(found.point, rect);
            let ink = Color32::from_rgb(70, 235, 225);
            if found.kind == snap::Kind::Node {
                painter.circle_stroke(at, 5., Stroke::new(1.7_f32, ink));
            } else {
                painter.rect_stroke(
                    Rect::from_center_size(at, Vec2::splat(9.)),
                    0.,
                    Stroke::new(1.7_f32, ink),
                    egui::StrokeKind::Middle,
                );
            }
        }
        if response.clicked_by(egui::PointerButton::Primary) {
            // Give keyboard shortcuts back to the canvas. A sidebar TextEdit
            // can otherwise retain focus and consume Ctrl+Z after rotation.
            response.request_focus();
            let op = self.scaling_op.as_mut().unwrap();
            if op.center.is_none() {
                op.center = Some(target);
                op.dialog_at = pointer + egui::vec2(12., 12.);
                self.status = if op.mode == Mode::Rotate {
                    "Поворот: укажите начальную точку направления"
                } else if op.mode == Mode::Move {
                    "Перемещение: укажите точку размещения"
                } else {
                    "Укажите конец опорного отрезка или коэффициент масштаба"
                }
                .into();
            } else if op.mode == Mode::Move {
                if let Err(e) = self.commit_move(target) {
                    self.status = e;
                }
            } else if op.mode == Mode::Rotate {
                let center = op.center.unwrap();
                if (target[0] - center[0]).hypot(target[1] - center[1]) <= 1e-6 {
                    self.status = "Точка направления не должна совпадать с центром поворота".into();
                    return None;
                }
                if let Some(reference) = op.reference {
                    if let Some(angle) = rotation_angle(center, reference, target) {
                        op.factor = angle;
                        if let Err(e) = self.commit_transform(1., [0., 0.]) {
                            self.status = e;
                        }
                    }
                } else {
                    op.reference = Some(target);
                    self.status = "Поворот: укажите конечную точку направления".into();
                }
            } else if op.mode == Mode::Reference {
                let center = op.center.unwrap();
                let length = (target[0] - center[0]).hypot(target[1] - center[1]);
                if length <= 1e-6 {
                    self.status = "Опорный отрезок не может быть нулевым".into();
                    return None;
                }
                if let Some(reference) = op.reference {
                    let factor =
                        length / (reference[0] - center[0]).hypot(reference[1] - center[1]);
                    if let Err(e) = self.commit_scaling(factor) {
                        self.status = e;
                    }
                } else {
                    op.reference = Some(target);
                    self.status = "Укажите новую конечную точку опорного отрезка".into();
                }
            }
        }
        self.scaling_op.as_ref().and(spatial_preview)
    }
}

#[cfg(test)]
mod rotation_regression {
    use super::*;
    #[test]
    fn three_point_rotation_commits_once_and_undo_restores_group() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([100., 0.], false);
        app.sketch_click([2100., 1000.], false);
        app.cancel();
        app.underlays = vec![
            Underlay {
                id: 1,
                name: "Страница".into(),
                visible: true,
                transparency: 0.,
                origin: [200., 300.],
                scale: 2.,
                scale_y: Some(3.),
                angle_rad: 0.1,
                data: UnderlayData::Pdf {
                    width: 1,
                    height: 1,
                    rgba: vec![255; 4],
                    size_mm: [210., 297.],
                    page: 1,
                },
            },
            Underlay {
                id: 2,
                name: "Чертёж".into(),
                visible: true,
                transparency: 0.,
                origin: [-150., 300.],
                scale: 1.5,
                scale_y: Some(0.75),
                angle_rad: 0.4,
                data: UnderlayData::Dxf(import::Drawing {
                    nodes: vec![[0., 0.], [100., 200.]],
                    ..Default::default()
                }),
            },
        ];
        app.select_scene([app.contours[0].id], [1, 2], egui::Modifiers::NONE);
        let before = app.contours.clone();
        let underlays_before = app
            .underlays
            .iter()
            .map(Underlay::transform)
            .collect::<Vec<_>>();
        let depth = app.history.len();
        app.begin_scaling(Mode::Rotate);
        let op = app.scaling_op.as_mut().unwrap();
        op.center = Some([0., 0.]);
        op.reference = Some([1000., 0.]);
        op.factor = rotation_angle([0., 0.], [1000., 0.], [0., 1000.]).unwrap();
        assert_eq!(app.contours, before, "preview cannot mutate source");
        app.commit_transform(1., [0., 0.]).unwrap();
        assert_eq!(app.history.len(), depth + 1);
        for (p, q) in before[0].points.iter().zip(&app.contours[0].points) {
            assert!((q[0] + p[1]).abs() < 1e-6);
            assert!((q[1] - p[0]).abs() < 1e-6);
        }
        for (u, previous) in app.underlays.iter().zip(&underlays_before) {
            let probe = [60., 100.];
            let p = previous.point(probe);
            let q = u.point(probe);
            assert!((q[0] + p[1]).abs() < 1e-6);
            assert!((q[1] - p[0]).abs() < 1e-6);
            assert_eq!(u.transform().scale, previous.scale);
        }
        app.undo();
        assert_eq!(app.contours, before);
        assert_eq!(
            app.underlays
                .iter()
                .map(Underlay::transform)
                .collect::<Vec<_>>(),
            underlays_before
        );
        assert!(rotation_angle([0., 0.], [0., 0.], [1., 0.]).is_none());

        app.contours[0].points.truncate(2);
        let invalid = app.contours.clone();
        assert!(model::validate_candidate(&invalid, &invalid[0]).is_err());
        app.select_scene([], [1], egui::Modifiers::NONE);
        app.begin_scaling(Mode::Rotate);
        let op = app.scaling_op.as_mut().unwrap();
        op.center = Some([0., 0.]);
        op.factor = std::f64::consts::FRAC_PI_2;
        app.commit_transform(1., [0., 0.]).unwrap();
        assert_eq!(app.contours, invalid);
        app.undo();
        assert_eq!(app.underlays[0].transform(), underlays_before[0]);
    }
}

#[cfg(test)]
mod column_rotation_regression {
    use super::*;
    #[test]
    fn rotating_column_about_own_center_changes_section_and_is_one_undo() {
        let mut app = App::default();
        app.begin(Kind::Column);
        app.sketch_click([1000., 2000.], false);
        app.sync_stiffness_sources();
        let before = app.contours.clone();
        let depth = app.history.len();
        app.begin_scaling(Mode::Rotate);
        let op = app.scaling_op.as_mut().unwrap();
        op.center = Some([1000., 2000.]);
        op.factor = std::f64::consts::FRAC_PI_2;
        app.commit_transform(1., [0., 0.]).unwrap();
        let c = &app.contours[0];
        assert_eq!(c.points, before[0].points);
        assert!((c.column_angle - std::f64::consts::FRAC_PI_2).abs() < 1e-12);
        let q = crate::plan_trace::columns::section_corners(c, &app.support_height.columns[&c.id]);
        let min_x = q.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min);
        let max_x = q.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max);
        assert!((max_x - min_x - 750.).abs() < 1e-8);
        assert_eq!(app.history.len(), depth + 1);
        app.undo();
        assert_eq!(app.contours, before);
    }
}
