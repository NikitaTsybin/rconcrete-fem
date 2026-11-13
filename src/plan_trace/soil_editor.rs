use super::*;
use mini_fem::soil::Layer;

pub(super) struct Editor {
    pub index: usize,
    draft: Ground,
    original: Ground,
    station: usize,
    camera: Camera,
    spatial: bool,
    placed: Vec<Borehole>,
    volumes: Vec<GroundFace>,
    prisms: Vec<GroundPrism>,
    cut: Option<(sections::State, Vec<Excavation>, Vec<GroundFace>)>,
    geometry_ready: bool,
    excavate: bool,
    error: String,
}

impl Editor {
    fn new(index: usize, ground: &Ground) -> Self {
        let mut draft = ground.clone();
        draft.pressures.clear();
        migrate_materials(&mut draft);
        Self {
            index,
            original: draft.clone(),
            draft,
            station: 0,
            camera: Camera::default(),
            spatial: true,
            placed: Vec::new(),
            volumes: Vec::new(),
            prisms: Vec::new(),
            cut: None,
            geometry_ready: false,
            excavate: false,
            error: String::new(),
        }
    }

    fn modified(&self) -> bool {
        self.draft.name != self.original.name
            || self.draft.materials != self.original.materials
            || self.draft.holes != self.original.holes
            || self.draft.ground != self.original.ground
            || self.draft.model != self.original.model
            || self.draft.angle != self.original.angle
    }

    fn validate(&self) -> Result<(), String> {
        if self.draft.name.trim().is_empty() {
            return Err("Укажите имя модели грунта.".into());
        }
        if self.draft.holes.is_empty() {
            return Err("Добавьте хотя бы одну скважину.".into());
        }
        if !self
            .draft
            .ground
            .into_iter()
            .chain(self.draft.model)
            .chain([self.draft.angle])
            .all(f64::is_finite)
        {
            return Err("Проверьте координаты привязки и угол поворота.".into());
        }
        for (&id, m) in &self.draft.materials {
            if !m.e.is_finite() || m.e <= 0. || !m.gamma.is_finite() || m.gamma <= 0. {
                return Err(format!("ИГЭ {id}. E и γ должны быть положительными."));
            }
        }
        let mut ids = std::collections::HashSet::new();
        let mut xy = std::collections::HashSet::new();
        for h in &self.draft.holes {
            if h.id == 0 || !ids.insert(h.id) {
                return Err(format!(
                    "Номер скважины {} должен быть положительным и уникальным.",
                    h.id
                ));
            }
            if !h.xy.into_iter().chain([h.mouth_z]).all(f64::is_finite) {
                return Err(format!("Скважина {}: проверьте координаты.", h.id));
            }
            if !xy.insert(h.xy.map(|v| if v == 0. { 0 } else { v.to_bits() })) {
                return Err(format!(
                    "Скважина {}: координаты X/Y совпадают с другой скважиной.",
                    h.id
                ));
            }
            if h.layers.is_empty() {
                return Err(format!("Скважина {}: добавьте хотя бы один слой.", h.id));
            }
            let mut top = h.mouth_z;
            for (i, l) in h.layers.iter().enumerate() {
                if !l.bottom_z.is_finite() || l.bottom_z >= top {
                    return Err(format!(
                        "Скважина {}, слой {}. Подошва должна быть ниже кровли.",
                        h.id,
                        i + 1
                    ));
                }
                if !l
                    .material_id
                    .is_some_and(|id| self.draft.materials.contains_key(&id))
                {
                    return Err(format!("Скважина {}, слой {}. Выберите ИГЭ.", h.id, i + 1));
                }
                top = l.bottom_z;
            }
        }
        Ok(())
    }

    fn editor_controls(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        ui.horizontal_wrapped(|ui| {
            ui.add(
                egui::TextEdit::singleline(&mut self.draft.name)
                    .desired_width(180.)
                    .hint_text("Название модели"),
            );
            #[cfg(not(target_arch = "wasm32"))]
            if ui
                .button("Импорт модели грунта")
                .on_hover_text("Загрузить скважины и ИГЭ из SLD или JSON в редактируемую модель.")
                .clicked()
            {
                changed |= self.import_ground();
            }
        });
        egui::CollapsingHeader::new("Инженерно-геологические элементы (ИГЭ)").show(ui, |ui| {
            changed |= self.material_controls(ui);
        });
        egui::CollapsingHeader::new("Скважины и слои")
            .default_open(true)
            .show(ui, |ui| {
                changed |= self.borehole_controls(ui);
            });
        if changed {
            self.geometry_ready = false;
            self.error.clear();
        }
        if !self.error.is_empty() {
            ui.colored_label(Color32::LIGHT_RED, &self.error);
        }
    }

    fn view_controls(&mut self, ui: &mut egui::Ui) {
        let section_yaw =
            two_borehole_yaw(&self.draft.holes).map(|yaw| yaw - self.draft.angle.to_radians());
        ui.horizontal_wrapped(|ui| {
            let plan = ui
                .selectable_value(&mut self.spatial, false, "2D")
                .changed();
            let spatial = ui.selectable_value(&mut self.spatial, true, "3D").changed();
            if plan || spatial {
                self.camera.fit();
            }
            if ui.button("Вписать").clicked() {
                self.camera.fit();
            }
            if let Some(yaw) = section_yaw {
                if ui
                    .button("Разрез")
                    .on_hover_text("Вид прямо на разрез между двумя скважинами")
                    .clicked()
                {
                    self.spatial = true;
                    self.camera.face_section(yaw);
                }
            }
            ui.label(if self.spatial {
                "ПКМ — вращение · СКМ — панорама · колесо — масштаб"
            } else {
                "СКМ — панорама · колесо — масштаб"
            });
        });
    }

    fn alignment_controls(&mut self, ui: &mut egui::Ui) {
        let mut changed = false;
        egui::CollapsingHeader::new("Привязка модели грунта")
            .default_open(true)
            .show(ui, |ui| {
                egui::Grid::new("ground_alignment").show(ui, |ui| {
                    ui.label("Ось");
                    ui.label("Грунт, м");
                    ui.label("Модель, м");
                    ui.end_row();
                    for (i, axis) in ["X", "Y", "Z"].into_iter().enumerate() {
                        ui.label(axis);
                        changed |= ui
                            .add(egui::DragValue::new(&mut self.draft.ground[i]).speed(0.1))
                            .changed();
                        changed |= ui
                            .add(egui::DragValue::new(&mut self.draft.model[i]).speed(0.1))
                            .changed();
                        ui.end_row();
                    }
                });
                changed |= ui
                    .add(
                        egui::DragValue::new(&mut self.draft.angle)
                            .prefix("α ")
                            .suffix("°"),
                    )
                    .changed();
            });
        if changed {
            self.geometry_ready = false;
            self.error.clear();
        }
    }

    #[cfg(not(target_arch = "wasm32"))]
    fn import_ground(&mut self) -> bool {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Модель грунта", &["sld", "json"])
            .pick_file()
        else {
            return false;
        };
        let holes = match soil_decode::read_ground(&path) {
            Ok(holes) if !holes.is_empty() => holes,
            Ok(_) => {
                self.error = "В модели грунта нет скважин.".into();
                return false;
            }
            Err(error) => {
                self.error = format!("Импорт грунта: {error}");
                return false;
            }
        };
        if self.original.holes.is_empty() && self.draft.holes.is_empty() {
            let first = &holes[0];
            self.draft.ground = [first.xy[0], first.xy[1], first.mouth_z];
            if self.draft.name == self.original.name {
                if let Some(name) = path.file_stem() {
                    self.draft.name = name.to_string_lossy().into_owned();
                }
            }
        }
        self.draft.holes = holes;
        self.draft.materials.clear();
        migrate_materials(&mut self.draft);
        self.station = 0;
        self.camera.fit();
        true
    }

    fn material_controls(&mut self, ui: &mut egui::Ui) -> bool {
        let used: std::collections::BTreeSet<_> = self
            .draft
            .holes
            .iter()
            .flat_map(|h| &h.layers)
            .filter_map(|l| l.material_id)
            .collect();
        let mut remove = None;
        let mut changed = false;
        egui::ScrollArea::vertical()
            .id_salt("ige_scroll")
            .max_height(150.)
            .show(ui, |ui| {
                egui::Grid::new("ige_properties")
                    .striped(true)
                    .show(ui, |ui| {
                        for label in ["ИГЭ", "E, МПа", "γ, тс/м³", ""] {
                            ui.label(label);
                        }
                        ui.end_row();
                        for (&id, m) in &mut self.draft.materials {
                            ui.label(id.to_string());
                            const MPA_PER_TF_M2: f64 = 0.00980665;
                            let mut e = m.e * MPA_PER_TF_M2;
                            if ui.add(egui::DragValue::new(&mut e).speed(0.1)).changed() {
                                m.e = e / MPA_PER_TF_M2;
                                changed = true;
                            }
                            changed |= ui
                                .add(egui::DragValue::new(&mut m.gamma).speed(0.01))
                                .changed();
                            if ui
                                .add_enabled(!used.contains(&id), egui::Button::new("−"))
                                .on_disabled_hover_text("ИГЭ назначен слою скважины")
                                .clicked()
                            {
                                remove = Some(id);
                            }
                            ui.end_row();
                        }
                    });
            });
        if let Some(id) = remove {
            self.draft.materials.remove(&id);
            changed = true;
        }
        if ui.button("+ ИГЭ").clicked() {
            let id = (1..)
                .find(|id| !self.draft.materials.contains_key(id))
                .unwrap();
            self.draft.materials.insert(
                id,
                SoilMaterial {
                    e: 2000.,
                    gamma: 2.,
                },
            );
            changed = true;
        }
        changed
    }

    fn borehole_controls(&mut self, ui: &mut egui::Ui) -> bool {
        let mut changed = false;
        ui.horizontal_wrapped(|ui| {
            egui::ComboBox::from_id_salt("ground_station")
                .selected_text(
                    self.draft
                        .holes
                        .get(self.station)
                        .map_or_else(|| "Нет скважин".into(), |h| format!("Скважина {}", h.id)),
                )
                .show_ui(ui, |ui| {
                    for (i, h) in self.draft.holes.iter().enumerate() {
                        ui.selectable_value(&mut self.station, i, format!("Скважина {}", h.id));
                    }
                });
            if ui.button("+ Скважина").clicked() {
                let current = self.draft.holes.get(self.station);
                let mut new = current.cloned().unwrap_or(Borehole {
                    id: 1,
                    xy: [0.; 2],
                    mouth_z: 0.,
                    layers: vec![Layer {
                        material_id: self.draft.materials.keys().next().copied(),
                        bottom_z: -1.,
                        e: 2000.,
                        gamma: 2.,
                    }],
                });
                new.id = (1..)
                    .find(|n| !self.draft.holes.iter().any(|h| h.id == *n))
                    .unwrap();
                if current.is_some() {
                    new.xy[0] += 1.;
                }
                self.station = self.draft.holes.len();
                self.draft.holes.push(new);
                changed = true;
            }
            if ui
                .add_enabled(
                    !self.draft.holes.is_empty(),
                    egui::Button::new("Удалить скважину"),
                )
                .clicked()
            {
                self.draft.holes.remove(self.station);
                self.station = self.station.min(self.draft.holes.len().saturating_sub(1));
                changed = true;
            }
        });
        let Some(h) = self.draft.holes.get_mut(self.station) else {
            return changed;
        };
        ui.horizontal_wrapped(|ui| {
            changed |= ui
                .add(
                    egui::DragValue::new(&mut h.id)
                        .range(1..=u32::MAX)
                        .prefix("№ "),
                )
                .changed();
            for (v, axis) in
                h.xy.iter_mut()
                    .chain([&mut h.mouth_z])
                    .zip(["X ", "Y ", "Устье Z "])
            {
                changed |= ui
                    .add(egui::DragValue::new(v).speed(0.1).prefix(axis).suffix(" м"))
                    .changed();
            }
        });
        let mut remove = None;
        egui::ScrollArea::vertical()
            .id_salt("soil_layers_scroll")
            .max_height(150.)
            .show(ui, |ui| {
                egui::Grid::new(("soil_layers", self.station))
                    .striped(true)
                    .min_col_width(0.)
                    .spacing([4., 4.])
                    .show(ui, |ui| {
                        for title in ["Слой", "ИГЭ", "Кровля, м", "Подошва, м", ""]
                        {
                            ui.label(title);
                        }
                        ui.end_row();
                        let mut top = h.mouth_z;
                        for (i, l) in h.layers.iter_mut().enumerate() {
                            ui.label((i + 1).to_string());
                            let before = l.material_id;
                            egui::ComboBox::from_id_salt(("layer_ige", i))
                                .width(56.)
                                .truncate()
                                .selected_text(
                                    before.map_or_else(|| "—".into(), |id| id.to_string()),
                                )
                                .show_ui(ui, |ui| {
                                    for &id in self.draft.materials.keys() {
                                        ui.selectable_value(
                                            &mut l.material_id,
                                            Some(id),
                                            id.to_string(),
                                        );
                                    }
                                })
                                .response
                                .on_hover_text("Выбрать ИГЭ");
                            changed |= l.material_id != before;
                            ui.label(format!("{top:.3}"));
                            changed |= ui
                                .add(egui::DragValue::new(&mut l.bottom_z).speed(0.1))
                                .changed();
                            top = l.bottom_z;
                            if ui.small_button("−").on_hover_text("Удалить слой").clicked()
                            {
                                remove = Some(i);
                            }
                            ui.end_row();
                        }
                    });
            });
        if let Some(i) = remove {
            h.layers.remove(i);
            changed = true;
        }
        if ui.button("+ Слой").clicked() {
            let l = h.layers.last();
            h.layers.push(Layer {
                material_id: l
                    .and_then(|l| l.material_id)
                    .or_else(|| self.draft.materials.keys().next().copied()),
                bottom_z: l.map_or(h.mouth_z, |l| l.bottom_z) - 1.,
                e: l.map_or(2000., |l| l.e),
                gamma: l.map_or(2., |l| l.gamma),
            });
            changed = true;
        }
        changed
    }

    fn canvas(
        &mut self,
        ui: &mut egui::Ui,
        slab: Option<(&[Point], f64)>,
        excavations: &[Excavation],
        opacity: f32,
        sections: &mut sections::State,
        pick: &mut Option<usize>,
    ) -> bool {
        if !self.geometry_ready {
            self.geometry_ready = true;
            let previous_section = two_borehole_yaw(&self.placed).is_some();
            self.placed = self.draft.placed();
            if !previous_section {
                if let Some(yaw) = two_borehole_yaw(&self.placed) {
                    self.camera.face_section(yaw);
                }
            }
            self.prisms.clear();
            self.volumes = volume_geometry(&self.placed, Some(&mut self.prisms));
            self.cut = None;
        }
        let (response, painter) =
            ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
        let mut low = [f64::INFINITY; 3];
        let mut high = [f64::NEG_INFINITY; 3];
        for h in &self.placed {
            for z in [h.mouth_z, h.layers.last().map_or(h.mouth_z, |l| l.bottom_z)] {
                for (i, value) in [h.xy[0], h.xy[1], z].into_iter().enumerate() {
                    low[i] = low[i].min(value);
                    high[i] = high[i].max(value);
                }
            }
        }
        if !low.into_iter().chain(high).all(f64::is_finite) {
            return false;
        }
        self.camera.input(ui, &response, self.spatial);
        let rect = response.rect;
        let center: [f64; 3] = std::array::from_fn(|i| (low[i] + high[i]) * 0.5);
        let size = (0..if self.spatial { 3 } else { 2 })
            .map(|i| (high[i] - low[i]).powi(2))
            .sum::<f64>()
            .sqrt()
            .max(1.);
        let scale =
            f64::from(rect.width().min(rect.height()).max(1.)) * 0.85 * self.camera.zoom / size;
        let project = |p: [f64; 3]| {
            let [x, y, z] = std::array::from_fn(|i| p[i] - center[i]);
            let (a, v, depth) = if self.spatial {
                let (sy, cy) = self.camera.yaw.sin_cos();
                let (sp, cp) = self.camera.pitch.sin_cos();
                let b = sy * x + cy * y;
                (cy * x - sy * y, cp * b - sp * z, sp * b + cp * z)
            } else {
                (x, y, 0.)
            };
            (
                rect.center()
                    + self.camera.pan
                    + egui::vec2((a * scale) as f32, (-v * scale) as f32),
                depth,
            )
        };
        let excavations = if self.excavate { excavations } else { &[] };
        if self.spatial {
            let faces = if sections.active() || !excavations.is_empty() {
                if self
                    .cut
                    .as_ref()
                    .is_none_or(|(state, pits, _)| state != sections || pits != excavations)
                {
                    let mut faces = self.volumes.clone();
                    faces.extend(self.prisms.iter().flat_map(|p| p.section_faces(sections)));
                    let mut clipped: Vec<_> = faces
                        .into_iter()
                        .flat_map(|(face, color)| {
                            App::excavate_ground_face(face, excavations)
                                .into_iter()
                                .flat_map(|face| clip_ground_face(face, sections))
                                .map(move |face| (face, color))
                        })
                        .collect();
                    clipped.extend(excavation_faces(&self.prisms, excavations, sections));
                    self.cut = Some((sections.clone(), excavations.to_vec(), clipped));
                }
                &self.cut.as_ref().unwrap().2
            } else {
                &self.volumes
            };
            if opacity >= 0.999 {
                let bounds = std::array::from_fn::<_, 8, _>(|mask| {
                    project(std::array::from_fn(|i| {
                        if mask & (1 << i) == 0 {
                            low[i]
                        } else {
                            high[i]
                        }
                    }))
                });
                let mut depth = spatial_depth::Builder::new(rect, &bounds);
                for &(face, color) in faces {
                    depth.triangle(face.map(|p| project(p.map(|v| v / 1000.))), color, 0.);
                }
                let _clips = spatial_depth::clip_guard();
                spatial_depth::paint_layer(
                    &painter,
                    rect,
                    std::sync::Arc::new(depth.vertices),
                    spatial_depth::Layer::GroundOpaque,
                );
            } else {
                let mut triangles: Vec<_> = faces
                    .iter()
                    .map(|&(face, color)| {
                        let p = face.map(|point| project(point.map(|v| v / 1000.)));
                        (
                            p.iter().map(|p| p.1).sum::<f64>(),
                            p.map(|p| p.0),
                            opacity_ink(color, opacity),
                        )
                    })
                    .collect();
                triangles.sort_by(|a, b| a.0.total_cmp(&b.0));
                let mut mesh = egui::Mesh::default();
                for (_, points, color) in triangles {
                    let start = mesh.vertices.len() as u32;
                    mesh.add_triangle(start, start + 1, start + 2);
                    for point in points {
                        mesh.colored_vertex(point, color);
                    }
                }
                if !mesh.is_empty() {
                    painter.add(egui::Shape::mesh(mesh));
                }
            }
        }

        let draw_outline = |points: &[[f64; 3]]| {
            for (i, &point) in points.iter().enumerate() {
                let end = points[(i + 1) % points.len()];
                painter.line_segment(
                    [point, end].map(|p| project(p.map(|v| v / 1000.)).0),
                    egui::Stroke::new(1.5_f32, Color32::YELLOW),
                );
            }
        };
        if self.excavate {
            for (z, polygon) in excavations {
                for ring in std::iter::once(polygon.exterior()).chain(polygon.interiors()) {
                    let outline: Vec<_> = ring.0.iter().map(|p| [p.x, p.y, *z]).collect();
                    draw_outline(&outline);
                }
            }
        } else if let Some((points, z)) = slab {
            let outline: Vec<_> = points.iter().map(|&[x, y]| [x, y, z]).collect();
            draw_outline(&outline);
        }
        if pick.is_some() {
            ui.ctx().set_cursor_icon(egui::CursorIcon::Crosshair);
            if ui.input(|i| i.key_pressed(egui::Key::Escape)) {
                *pick = None;
            }
        }
        let mut hovered = None;
        for (i, h) in self.placed.iter().enumerate() {
            let world = [h.xy[0], h.xy[1], h.mouth_z].map(|v| v * 1000.);
            if self.spatial && pick.is_none() && !sections.contains(world) {
                continue;
            }
            let at = |z| project([h.xy[0], h.xy[1], z]).0;
            let mouth = at(h.mouth_z);
            let active = i == self.station;
            let color = if active {
                Color32::YELLOW
            } else {
                Color32::LIGHT_BLUE
            };
            if self.spatial {
                painter.line_segment(
                    [mouth, at(h.layers.last().map_or(h.mouth_z, |l| l.bottom_z))],
                    egui::Stroke::new(
                        if active { 2.0_f32 } else { 1.0_f32 },
                        color.gamma_multiply(if active { 1. } else { 0.6 }),
                    ),
                );
                if active {
                    for l in &h.layers {
                        painter.circle_filled(at(l.bottom_z), 2., color);
                    }
                }
            }
            painter.circle_filled(mouth, if active { 5. } else { 3. }, color);
            if self.spatial && self.placed.len() == 2 && !self.volumes.is_empty() {
                painter.text(
                    mouth + egui::vec2(0., -8.),
                    egui::Align2::CENTER_BOTTOM,
                    format!("Скв. {}", h.id),
                    egui::FontId::proportional(12.),
                    color,
                );
            }
            if let Some(d) = response
                .hover_pos()
                .map(|p| p.distance(mouth))
                .filter(|d| *d <= 8.)
            {
                if hovered.is_none_or(|(_, best)| d < best) {
                    hovered = Some((i, d));
                }
            }
        }
        if let Some((i, _)) = hovered {
            let h = &self.placed[i];
            if response.clicked_by(egui::PointerButton::Primary) {
                if let Some(i) = pick.take() {
                    if let Some(plane) = sections.planes.get_mut(i) {
                        let world = [h.xy[0], h.xy[1], h.mouth_z];
                        plane.set_origin(i, world.map(|v| v * 1000.));
                        return true;
                    }
                }
                self.station = i;
            }
            response.on_hover_text(format!("Скважина {} · Z {:.3} м", h.id, h.mouth_z));
        }
        false
    }
}

struct Camera {
    yaw: f64,
    pitch: f64,
    pan: egui::Vec2,
    zoom: f64,
}

impl Default for Camera {
    fn default() -> Self {
        Self {
            yaw: -0.65,
            pitch: -1.1,
            pan: egui::Vec2::ZERO,
            zoom: 1.,
        }
    }
}

impl Camera {
    fn fit(&mut self) {
        self.pan = egui::Vec2::ZERO;
        self.zoom = 1.;
    }

    fn face_section(&mut self, yaw: f64) {
        self.yaw = yaw;
        self.pitch = -std::f64::consts::FRAC_PI_2;
        self.fit();
    }

    fn input(&mut self, ui: &egui::Ui, response: &egui::Response, spatial: bool) {
        let delta = ui.input(|i| i.pointer.delta());
        if spatial && response.dragged_by(egui::PointerButton::Secondary) {
            self.yaw += f64::from(delta.x) * 0.007;
            self.pitch =
                (self.pitch + f64::from(delta.y) * 0.007).rem_euclid(std::f64::consts::TAU);
        }
        if response.dragged_by(egui::PointerButton::Middle) {
            self.pan += delta;
        }
        if response.double_clicked_by(egui::PointerButton::Middle) {
            self.fit();
        }
        if let Some(at) = response.hover_pos() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            let zoom = (self.zoom * (f64::from(scroll) * 0.002).exp()).clamp(0.02, 100.);
            if zoom != self.zoom {
                let anchor = at - response.rect.center();
                self.pan = anchor - (anchor - self.pan) * (zoom / self.zoom) as f32;
                self.zoom = zoom;
            }
        }
    }
}

impl App {
    #[cfg(not(target_arch = "wasm32"))]
    pub(in crate::plan_trace) fn smoke_open_ground_editor(&mut self, ctx: &egui::Context) {
        assert!(
            !self.soil_options.grounds.is_empty(),
            "Smoke test needs a project with a ground model"
        );
        self.request_ground_editor_exit(EditorExit::Switch(0), ctx);
        self.soil_view.editor.as_mut().unwrap().excavate = true;
        self.drawing_visibility.ground_opacity = 1.;
    }

    pub(in crate::plan_trace) fn soil_editor_tab(&mut self, ui: &mut egui::Ui) {
        if self.soil_view.editor.is_some() {
            if ui.selectable_label(self.soil_view.open, "Грунт").clicked() {
                self.soil_view.open = true;
            }
            if ui
                .small_button("×")
                .on_hover_text("Закрыть вкладку «Грунт»")
                .clicked()
            {
                self.request_ground_editor_exit(EditorExit::Close, ui.ctx());
            }
        }
    }

    pub(in crate::plan_trace) fn soil_editor_sidebars(&mut self, ctx: &egui::Context) {
        egui::SidePanel::left("soil_editor_left")
            .default_width(325.)
            .min_width(260.)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    if ui
                        .add_enabled(
                            self.soil_editor_modified(),
                            egui::Button::new("Сохранить в проект"),
                        )
                        .clicked()
                    {
                        self.save_ground_editor();
                    }
                    if let Some(editor) = &mut self.soil_view.editor {
                        editor.editor_controls(ui);
                    }
                });
            });
        egui::SidePanel::right("soil_editor_right")
            .default_width(325.)
            .min_width(250.)
            .show(ctx, |ui| {
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let Some(editor) = &mut self.soil_view.editor else {
                        return;
                    };
                    editor.alignment_controls(ui);
                    ui.separator();
                    ui.checkbox(&mut editor.excavate, "Выдавливание фундаментом")
                        .on_hover_text("Показать выемки над плитами с учётом основания");
                    ui.label("Непрозрачность грунта");
                    ui.add(egui::Slider::new(
                        &mut self.drawing_visibility.ground_opacity,
                        0.05..=1.,
                    ));
                    let spatial = editor.spatial;
                    ui.separator();
                    self.section_controls_for_view(ui, spatial);
                });
            });
    }

    pub(in crate::plan_trace) fn soil_editor_canvas(&mut self, ui: &mut egui::Ui) {
        let excavations = self.ground_excavations(false);
        let outline = self
            .soil_outline()
            .map(|c| (c.id, self.source_heights(c)[0]));
        let slab = outline.and_then(|(id, z)| {
            self.contours
                .iter()
                .find(|c| c.id == id)
                .map(|c| (c.points.as_slice(), z))
        });
        if let Some(editor) = &mut self.soil_view.editor {
            editor.view_controls(ui);
            if editor.canvas(
                ui,
                slab,
                &excavations,
                self.drawing_visibility.ground_opacity,
                &mut self.sections,
                &mut self.section_pick_origin,
            ) {
                self.section_geometry_changed();
            }
        }
    }

    pub(super) fn request_ground_editor_exit(&mut self, action: EditorExit, ctx: &egui::Context) {
        if let EditorExit::Switch(index) = action {
            if self
                .soil_view
                .editor
                .as_ref()
                .is_some_and(|e| e.index == index)
            {
                self.soil_view.open = true;
                return;
            }
        }
        if self.soil_view.editor.as_ref().is_some_and(Editor::modified) {
            self.soil_view.editor_prompt = Some(action);
        } else {
            self.finish_ground_editor_exit(action, ctx);
        }
    }

    pub(in crate::plan_trace) fn soil_editor_modified(&self) -> bool {
        self.soil_view.editor.as_ref().is_some_and(Editor::modified)
    }

    pub(in crate::plan_trace) fn defer_close_for_soil_editor(
        &mut self,
        ctx: &egui::Context,
    ) -> bool {
        if ctx.input(|i| i.viewport().close_requested()) && self.soil_editor_modified() {
            ctx.send_viewport_cmd(egui::ViewportCommand::CancelClose);
            self.soil_view.editor_prompt = Some(EditorExit::Application);
            return true;
        }
        false
    }

    fn finish_ground_editor_exit(&mut self, action: EditorExit, ctx: &egui::Context) {
        self.soil_view.editor_prompt = None;
        self.soil_view.editor = match action {
            EditorExit::Close | EditorExit::Application => None,
            EditorExit::Create => {
                let i = self.soil_options.grounds.len();
                let number = (1..)
                    .find(|n| {
                        !self
                            .soil_options
                            .grounds
                            .iter()
                            .any(|g| g.name == format!("Модель грунта {n}"))
                    })
                    .unwrap();
                let draft = Ground {
                    name: format!("Модель грунта {number}"),
                    visible: true,
                    ..Default::default()
                };
                Some(Editor::new(i, &draft))
            }
            EditorExit::Switch(i) => self.soil_options.grounds.get(i).map(|g| Editor::new(i, g)),
        };
        self.soil_view.open = self.soil_view.editor.is_some();
        if matches!(action, EditorExit::Application) {
            ctx.send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }

    fn save_ground_editor(&mut self) -> bool {
        let Some(editor) = &mut self.soil_view.editor else {
            return false;
        };
        if let Err(error) = editor.validate() {
            editor.error = error;
            return false;
        }
        editor.error.clear();
        let mut ground = editor.draft.clone();
        // Keep compatibility with readers that use material properties stored per layer.
        for layer in ground.holes.iter_mut().flat_map(|h| &mut h.layers) {
            if let Some(m) = layer.material_id.and_then(|id| ground.materials.get(&id)) {
                layer.e = m.e;
                layer.gamma = m.gamma;
            }
        }
        if let Some(saved) = self.soil_options.grounds.get_mut(editor.index) {
            ground.visible = saved.visible;
            *saved = ground;
        } else {
            self.soil_options.selected = editor.index;
            self.soil_options.grounds.push(ground);
        }
        editor.original = editor.draft.clone();
        self.soil_view.ground_geometry.get_mut().take();
        self.soil_view.cut_geometry.get_mut().take();
        self.invalidate_scene_buffers();
        true
    }

    pub(in crate::plan_trace) fn soil_editor_prompt(&mut self, ctx: &egui::Context) {
        let Some(action) = self.soil_view.editor_prompt else {
            return;
        };
        egui::Modal::new(egui::Id::new("close_ground_editor")).show(ctx, |ui| {
            ui.heading("Сохранить изменения модели грунта?");
            ui.horizontal(|ui| {
                if ui.button("Сохранить в проект").clicked() && self.save_ground_editor()
                {
                    self.finish_ground_editor_exit(action, ctx);
                }
                if ui.button("Не сохранять").clicked() {
                    self.finish_ground_editor_exit(action, ctx);
                }
                if ui.button("Продолжить редактирование").clicked() {
                    self.soil_view.open = true;
                    self.soil_view.editor_prompt = None;
                }
            });
            if let Some(e) = &self.soil_view.editor {
                if !e.error.is_empty() {
                    ui.colored_label(Color32::LIGHT_RED, &e.error);
                }
            }
        });
    }
}

// Preserve distinct properties from older projects, including cases where a
// single IGE was edited independently in two boreholes.
fn migrate_materials(ground: &mut Ground) {
    let mut reserved: std::collections::BTreeSet<_> = ground
        .holes
        .iter()
        .flat_map(|h| &h.layers)
        .filter_map(|l| l.material_id)
        .collect();
    reserved.extend(ground.materials.keys().copied());
    let mut free = (1..).filter(|id| !reserved.contains(id));
    let mut groups = BTreeMap::new();
    let authoritative = !ground.materials.is_empty();
    for l in ground.holes.iter_mut().flat_map(|h| &mut h.layers) {
        let material = SoilMaterial {
            e: l.e,
            gamma: l.gamma,
        };
        if let Some(id) = l.material_id {
            if let Some(existing) = ground.materials.get(&id) {
                if authoritative || *existing == material {
                    continue;
                }
            } else {
                ground.materials.insert(id, material);
                continue;
            }
        }
        let key = (l.material_id, l.e.to_bits(), l.gamma.to_bits());
        let id = *groups.entry(key).or_insert_with(|| free.next().unwrap());
        ground.materials.insert(id, material);
        l.material_id = Some(id);
    }
}

pub(super) fn opacity_ink(color: Color32, opacity: f32) -> Color32 {
    let [r, g, b, _] = color.to_array();
    // Both screen pipelines expect premultiplied sRGB channels.
    let a = (opacity * 255.).round() as u8;
    let premultiply = |channel: u8| ((u16::from(channel) * u16::from(a) + 127) / 255) as u8;
    Color32::from_rgba_premultiplied(premultiply(r), premultiply(g), premultiply(b), a)
}

pub(super) struct GroundPrism {
    top: [[f64; 3]; 3],
    bottom: [[f64; 3]; 3],
    color: Color32,
}

impl GroundPrism {
    fn plane_polygon(&self, normal: [f64; 3], offset: f64) -> Vec<[f64; 3]> {
        const EPS: f64 = 1e-6;
        let normal = glam::DVec3::from_array(normal);
        let distance = |p| normal.dot(glam::DVec3::from_array(p)) - offset;
        let vertices = self.top.into_iter().chain(self.bottom);
        let (mut lo, mut hi) = (f64::INFINITY, f64::NEG_INFINITY);
        for p in vertices.clone() {
            let d = distance(p);
            lo = lo.min(d);
            hi = hi.max(d);
        }
        if lo > EPS || hi < -EPS || hi - lo <= EPS {
            return Vec::new();
        }
        let mut points = Vec::new();
        let mut insert = |p: [f64; 3]| {
            if !points
                .iter()
                .any(|&q| glam::DVec3::from_array(q).distance(glam::DVec3::from_array(p)) <= EPS)
            {
                points.push(p);
            }
        };
        for p in vertices.filter(|&p| distance(p).abs() <= EPS) {
            insert(p);
        }
        for j in 0..3 {
            let k = (j + 1) % 3;
            for (a, b) in [
                (self.top[j], self.top[k]),
                (self.bottom[j], self.bottom[k]),
                (self.top[j], self.bottom[j]),
            ] {
                let da = distance(a);
                let db = distance(b);
                if (da < -EPS && db > EPS) || (db < -EPS && da > EPS) {
                    let t = da / (da - db);
                    insert(std::array::from_fn(|d| a[d] + (b[d] - a[d]) * t));
                }
            }
        }
        if points.len() < 3 {
            return Vec::new();
        }
        let axis = if normal.z.abs() < 0.9 {
            glam::DVec3::Z
        } else {
            glam::DVec3::X
        };
        let u = normal.cross(axis);
        let v = normal.cross(u);
        let center = points
            .iter()
            .copied()
            .map(glam::DVec3::from_array)
            .sum::<glam::DVec3>()
            / points.len() as f64;
        let angle = |p| {
            let p = glam::DVec3::from_array(p) - center;
            p.dot(v).atan2(p.dot(u))
        };
        points.sort_by(|&a, &b| angle(a).total_cmp(&angle(b)));
        points
    }

    pub(super) fn section_faces(&self, sections: &sections::State) -> Vec<GroundFace> {
        let mut faces = Vec::new();
        for (i, plane) in sections.active_planes() {
            if !self
                .top
                .into_iter()
                .chain(self.bottom)
                .any(|p| plane.distance(i, p) > 1e-6)
            {
                continue;
            }
            let polygon = self.plane_polygon(plane.normal(i), plane.offset(i));
            // The caller clips these faces by all active planes.
            faces.extend(polygon_triangles(&polygon).map(|tri| (tri, self.color)));
        }
        faces
    }
}

fn polygon_triangles(points: &[[f64; 3]]) -> impl Iterator<Item = [[f64; 3]; 3]> + '_ {
    (1..points.len().saturating_sub(1)).map(|j| [points[0], points[j], points[j + 1]])
}

pub(super) fn excavation_faces(
    prisms: &[GroundPrism],
    excavations: &[Excavation],
    sections: &sections::State,
) -> Vec<GroundFace> {
    use mini_fem::geo::{BooleanOps, BoundingRect, Polygon};
    let mut faces = Vec::new();
    for (i, (z, slab)) in excavations.iter().enumerate() {
        let Some(bounds) = slab.bounding_rect() else {
            continue;
        };
        let other_pits: Vec<_> = excavations
            .iter()
            .enumerate()
            .filter(|(j, _)| *j != i)
            .map(|(_, pit)| pit.clone())
            .collect();
        for prism in prisms {
            let disjoint = prism.top.iter().all(|p| p[0] < bounds.min().x)
                || prism.top.iter().all(|p| p[0] > bounds.max().x)
                || prism.top.iter().all(|p| p[1] < bounds.min().y)
                || prism.top.iter().all(|p| p[1] > bounds.max().y);
            if disjoint || prism.top.iter().all(|p| p[2] <= *z) {
                continue;
            }
            let mut add = |tri| {
                faces.extend(
                    App::excavate_ground_face(tri, &other_pits)
                        .into_iter()
                        .flat_map(|tri| clip_ground_face(tri, sections))
                        .map(|tri| (tri, prism.color)),
                );
            };
            // The base exposes the layer immediately below the slab.
            if prism.bottom.iter().any(|p| p[2] < *z - 1e-6) {
                let base = prism.plane_polygon([0., 0., 1.], *z);
                if base.len() >= 3 {
                    let polygon = Polygon::new(
                        base.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>().into(),
                        vec![],
                    );
                    for fragment in slab.intersection(&polygon).0 {
                        let rings = std::iter::once(fragment.exterior())
                            .chain(fragment.interiors())
                            .map(|ring| ring.0.iter().map(|p| [p.x, p.y]).collect())
                            .collect::<Vec<_>>();
                        if let Ok(triangles) = geometry::triangulate(&rings) {
                            for tri in triangles {
                                add(tri.map(|[x, y]| [x, y, *z]));
                            }
                        }
                    }
                }
            }
            for ring in std::iter::once(slab.exterior()).chain(slab.interiors()) {
                for edge in ring.0.windows(2) {
                    let (a, b) = (edge[0], edge[1]);
                    let d = glam::DVec2::new(b.x - a.x, b.y - a.y);
                    let length = d.length();
                    if length <= 1e-6 {
                        continue;
                    }
                    let tangent = d / length;
                    let n = tangent.perp();
                    let origin = glam::DVec2::new(a.x, a.y);
                    let along = |p: [f64; 3]| tangent.dot(glam::DVec2::new(p[0], p[1]) - origin);
                    let side = prism.plane_polygon([n.x, n.y, 0.], n.dot(origin));
                    let side = clip_ground_polygon(&side, along);
                    let side = clip_ground_polygon(&side, |p| length - along(p));
                    let side = clip_ground_polygon(&side, |p| p[2] - z);
                    for tri in polygon_triangles(&side) {
                        add(tri);
                    }
                }
            }
        }
    }
    faces
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum SoilHorizon {
    Ige(u32),
    Legacy(u64, u64),
}

impl SoilHorizon {
    fn color(self, saturation: f32, value: f32) -> Color32 {
        let hue = match self {
            Self::Ige(id) => f64::from(id) * 0.618,
            Self::Legacy(e, _) => f64::from_bits(e).max(f64::EPSILON).ln() * 0.71,
        }
        .rem_euclid(1.) as f32;
        egui::ecolor::Hsva::new(hue, saturation, value, 1.).into()
    }
}

fn two_borehole_yaw(holes: &[Borehole]) -> Option<f64> {
    let [a, b] = holes else {
        return None;
    };
    let [dx, dy] = [b.xy[0] - a.xy[0], b.xy[1] - a.xy[1]];
    (dx.is_finite() && dy.is_finite() && dx.hypot(dy) > 1e-9).then(|| -dy.atan2(dx))
}

// Align all borehole sequences to one stratigraphic order. A missing interval
// has zero thickness at that well. The same interfaces then meet on shared
// Delaunay edges, including where an interval tapers out.
fn merge_horizons(merged: &mut Vec<SoilHorizon>, profile: &[SoilHorizon]) {
    let mut next = profile.iter().peekable();
    for horizon in merged.iter() {
        if next.peek() == Some(&horizon) {
            next.next();
        }
    }
    if next.peek().is_none() {
        return;
    }
    let columns = profile.len() + 1;
    let mut lcs = vec![0; (merged.len() + 1) * columns];
    for i in (0..merged.len()).rev() {
        for j in (0..profile.len()).rev() {
            lcs[i * columns + j] = if merged[i] == profile[j] {
                1 + lcs[(i + 1) * columns + j + 1]
            } else {
                lcs[(i + 1) * columns + j].max(lcs[i * columns + j + 1])
            };
        }
    }
    let mut result = Vec::with_capacity(merged.len() + profile.len());
    let (mut i, mut j) = (0, 0);
    while i < merged.len() && j < profile.len() {
        if merged[i] == profile[j] {
            result.push(merged[i]);
            i += 1;
            j += 1;
        } else if lcs[(i + 1) * columns + j] >= lcs[i * columns + j + 1] {
            result.push(merged[i]);
            i += 1;
        } else {
            result.push(profile[j]);
            j += 1;
        }
    }
    result.extend_from_slice(&merged[i..]);
    result.extend_from_slice(&profile[j..]);
    *merged = result;
}

pub(super) fn volume_geometry(
    holes: &[Borehole],
    mut prisms: Option<&mut Vec<GroundPrism>>,
) -> Vec<GroundFace> {
    use spade::{DelaunayTriangulation, HasPosition, Point2, Triangulation};

    struct Station {
        xy: [f64; 2],
        interfaces: Vec<f64>,
    }
    impl Station {
        fn interface(&self, i: usize) -> [f64; 3] {
            [self.xy[0], self.xy[1], self.interfaces[i]].map(|m| m * 1000.)
        }
    }
    impl HasPosition for Station {
        type Scalar = f64;
        fn position(&self) -> Point2<f64> {
            Point2::new(self.xy[0], self.xy[1])
        }
    }

    let mut horizons = Vec::new();
    let mut profiles = Vec::new();
    for h in holes
        .iter()
        .filter(|h| h.mouth_z.is_finite() && h.xy.into_iter().all(f64::is_finite))
    {
        let mut sequence = Vec::new();
        let mut bottoms: Vec<f64> = Vec::new();
        for l in &h.layers {
            if !l.bottom_z.is_finite() || l.bottom_z >= bottoms.last().copied().unwrap_or(h.mouth_z)
            {
                continue;
            }
            let material = l.material_id.map_or_else(
                || SoilHorizon::Legacy(l.e.to_bits(), l.gamma.to_bits()),
                SoilHorizon::Ige,
            );
            if sequence.last() == Some(&material) {
                *bottoms.last_mut().unwrap() = l.bottom_z;
            } else {
                sequence.push(material);
                bottoms.push(l.bottom_z);
            }
        }
        if !sequence.is_empty() {
            merge_horizons(&mut horizons, &sequence);
            profiles.push((h, sequence, bottoms));
        }
    }
    let stations: Vec<_> = profiles
        .into_iter()
        .map(|(h, sequence, bottoms)| {
            let mut interfaces = Vec::with_capacity(horizons.len() + 1);
            let mut z = h.mouth_z;
            interfaces.push(z);
            let mut j = 0;
            for horizon in &horizons {
                if sequence.get(j) == Some(horizon) {
                    z = bottoms[j];
                    j += 1;
                }
                interfaces.push(z);
            }
            Station {
                xy: h.xy,
                interfaces,
            }
        })
        .collect();
    if let [a, b] = stations.as_slice() {
        if two_borehole_yaw(holes).is_some() {
            let mut section = Vec::new();
            for (i, material) in horizons.iter().enumerate() {
                let [left, right] = [a.interface(i), b.interface(i)];
                let [bottom_left, bottom_right] = [a.interface(i + 1), b.interface(i + 1)];
                let color = material.color(0.57, 0.75);
                if right != bottom_right {
                    section.push(([left, right, bottom_right], color));
                }
                if left != bottom_left {
                    section.push(([left, bottom_right, bottom_left], color));
                }
            }
            return section;
        }
    }
    let Ok(mesh) = DelaunayTriangulation::<Station>::bulk_load(stations) else {
        return vec![];
    };
    let edge_key = |a: usize, b: usize| (a.min(b), a.max(b));
    let hull: std::collections::BTreeSet<_> = mesh
        .convex_hull()
        .map(|e| edge_key(e.from().index(), e.to().index()))
        .collect();
    let mut faces = Vec::new();
    for face in mesh.inner_faces() {
        let vertices = face.vertices();
        let ids = vertices.map(|v| v.index());
        let stations = [&vertices[0], &vertices[1], &vertices[2]].map(|v| v.data());
        let interface = |i| stations.map(|s| s.interface(i));
        let mut top = interface(0);
        let mut surface = true;
        for (i, material) in horizons.iter().enumerate() {
            let bottom = interface(i + 1);
            if top == bottom {
                continue;
            }
            let color = material.color(0.52, 0.90);
            if surface {
                faces.push((top, color));
                surface = false;
            }
            faces.push((bottom, color));
            let side_color = material.color(0.57, 0.75);
            if let Some(prisms) = &mut prisms {
                prisms.push(GroundPrism {
                    top,
                    bottom,
                    color: side_color,
                });
            }
            for i in 0..3 {
                let j = (i + 1) % 3;
                if hull.contains(&edge_key(ids[i], ids[j])) {
                    if top[j] != bottom[j] {
                        faces.push(([top[i], top[j], bottom[j]], side_color));
                    }
                    if top[i] != bottom[i] {
                        faces.push(([top[i], bottom[j], bottom[i]], side_color));
                    }
                }
            }
            top = bottom;
        }
    }
    faces
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn new_ground_stays_in_the_editor_until_saved() {
        let mut app = App::default();
        let ctx = egui::Context::default();
        app.request_ground_editor_exit(EditorExit::Create, &ctx);
        assert!(app.soil_options.grounds.is_empty());
        assert!(!app.save_ground_editor());
        let editor = app.soil_view.editor.as_mut().unwrap();
        editor.draft.name = "Новый грунт".into();
        editor.draft.holes.push(Borehole {
            id: 1,
            xy: [0.; 2],
            mouth_z: 0.,
            layers: vec![Layer {
                material_id: Some(1),
                bottom_z: -10.,
                e: 2500.,
                gamma: 2.,
            }],
        });
        migrate_materials(&mut editor.draft);
        assert!(app.soil_editor_modified());
        assert!(app.save_ground_editor());
        assert!(!app.soil_editor_modified());
        assert_eq!(app.soil_options.grounds[0].name, "Новый грунт");
        app.request_ground_editor_exit(EditorExit::Create, &ctx);
        app.request_ground_editor_exit(EditorExit::Close, &ctx);
        assert!(app.soil_view.editor.is_none());
        assert_eq!(app.soil_options.grounds.len(), 1);
    }

    #[test]
    fn ground_layers_share_interfaces_when_a_horizon_is_missing() {
        let profile = |id, xy, ids: &[u32]| Borehole {
            id,
            xy,
            mouth_z: 0.,
            layers: ids
                .iter()
                .enumerate()
                .map(|(i, &material_id)| Layer {
                    material_id: Some(material_id),
                    bottom_z: -(i as f64 + 1.),
                    e: 2000.,
                    gamma: 2.,
                })
                .collect(),
        };
        let wells = [
            profile(1, [0., 0.], &[1, 2, 3]),
            profile(2, [1., 0.], &[1, 3]),
            profile(3, [0., 1.], &[1, 2, 3]),
        ];
        let mut layers = Vec::new();
        volume_geometry(&wells, Some(&mut layers));
        assert_eq!(layers.len(), 3);
        for pair in layers.windows(2) {
            assert_eq!(pair[0].bottom, pair[1].top);
        }
        for (top, bottom) in layers[1].top.iter().zip(layers[1].bottom) {
            assert_eq!(top[2] - bottom[2], if top[0] == 1000. { 0. } else { 1000. });
        }
    }

    #[test]
    fn two_boreholes_make_a_section_with_a_tapering_layer() {
        let borehole = |id, xy, mouth_z, intervals: &[(u32, f64)]| Borehole {
            id,
            xy,
            mouth_z,
            layers: intervals
                .iter()
                .map(|&(material_id, bottom_z)| Layer {
                    material_id: Some(material_id),
                    bottom_z,
                    e: 2000.,
                    gamma: 2.,
                })
                .collect(),
        };
        let mut wells = [
            borehole(1, [0., 0.], 0., &[(1, -2.), (2, -3.), (3, -5.)]),
            borehole(2, [3., 4.], 1., &[(1, -1.), (3, -6.)]),
        ];
        let mut prisms = Vec::new();
        let faces = volume_geometry(&wells, Some(&mut prisms));
        assert!(prisms.is_empty());
        assert_eq!(faces.len(), 5);
        for point in faces.iter().flat_map(|(face, _)| face) {
            assert!([0., 0.] == point[..2] || [3000., 4000.] == point[..2]);
        }
        let taper_color = SoilHorizon::Ige(2).color(0.57, 0.75);
        let taper: Vec<_> = faces
            .iter()
            .filter(|(_, color)| *color == taper_color)
            .map(|(face, _)| *face)
            .collect();
        assert_eq!(
            taper,
            [[[0., 0., -2000.], [3000., 4000., -1000.], [0., 0., -3000.]]]
        );
        assert!(faces
            .iter()
            .any(|(face, _)| face.contains(&[3000., 4000., 1000.])));

        wells[1].xy = wells[0].xy;
        assert!(volume_geometry(&wells, None).is_empty());
    }

    #[test]
    fn foundation_excavation_has_base_and_walls_and_leaves_openings() {
        use mini_fem::geo::Polygon;
        let prism = GroundPrism {
            top: [[0., 0., 2.], [4., 0., 2.], [0., 4., 2.]],
            bottom: [[0., 0., 0.], [4., 0., 0.], [0., 4., 0.]],
            color: Color32::WHITE,
        };
        let square = |a, b| vec![(a, a), (b, a), (b, b), (a, b), (a, a)].into();
        let pit = Polygon::new(square(0.25, 1.25), vec![square(0.5, 1.)]);
        let faces = excavation_faces(&[prism], &[(1., pit)], &sections::State::default());
        let base_area: f64 = faces
            .iter()
            .filter(|(tri, _)| tri.iter().all(|p| (p[2] - 1.).abs() < 1e-6))
            .map(|(tri, _)| geometry::area(&tri.map(|p| [p[0], p[1]])).abs())
            .sum();
        assert!((base_area - 0.75).abs() < 1e-6, "{base_area}");
        assert!(faces.iter().any(|(tri, _)| tri.iter().any(|p| p[2] > 1.5)));
        assert!(faces
            .iter()
            .all(|(tri, _)| tri.iter().all(|p| p[2] >= 1. - 1e-6)));
    }

    #[test]
    fn common_ige_properties_update_every_borehole_and_legacy_conflicts_survive() {
        let borehole = |id, e| Borehole {
            id,
            xy: [f64::from(id), 0.],
            mouth_z: 0.,
            layers: vec![Layer {
                material_id: Some(5),
                bottom_z: -2.,
                e,
                gamma: 2.,
            }],
        };
        let ground = Ground {
            name: String::new(),
            visible: true,
            holes: vec![borehole(1, 2000.), borehole(2, 2000.), borehole(3, 2200.)],
            materials: BTreeMap::new(),
            ground: [0.; 3],
            model: [0.; 3],
            angle: 0.,
            pressures: Vec::new(),
            pressure_reference: String::new(),
            pressure_z: 0.,
            pressure_foundation: 0,
        };
        let mut editor = Editor::new(0, &ground);
        assert!(!editor.modified());
        assert_eq!(editor.draft.materials.len(), 2);
        editor.draft.materials.get_mut(&5).unwrap().e = 2500.;
        assert!(editor.modified());
        assert_eq!(
            editor
                .draft
                .placed()
                .iter()
                .map(|h| h.layers[0].e)
                .collect::<Vec<_>>(),
            [2500., 2500., 2200.]
        );
    }

    #[test]
    fn ground_section_cap_stays_within_all_active_planes() {
        let prism = GroundPrism {
            top: [[0., 0., 2.], [2., 0., 2.], [0., 2., 2.]],
            bottom: [[0., 0., 0.], [2., 0., 0.], [0., 2., 0.]],
            color: Color32::WHITE,
        };
        let mut state = sections::State::default();
        state.planes[0].enabled = true;
        state.planes[0].offset_mm = 1.;
        state.planes[2].enabled = true;
        state.planes[2].offset_mm = 1.;
        let caps: Vec<_> = prism
            .section_faces(&state)
            .into_iter()
            .flat_map(|(tri, color)| {
                clip_ground_face(tri, &state)
                    .into_iter()
                    .map(move |tri| (tri, color))
            })
            .collect();
        assert!(caps
            .iter()
            .all(|(tri, _)| tri.iter().all(|&p| state.contains(p))));
        assert!(caps.iter().any(|(tri, _)| tri.iter().all(|p| p[2] == 1.)));
        assert!(caps.iter().any(|(tri, _)| tri.iter().all(|p| p[0] == 1.)));
    }

    #[test]
    fn ground_volume_closes_outer_boundary_and_ignores_modulus_variation_within_ige() {
        let holes = [[0., 0.], [1., 0.], [1., 1.], [0., 1.]]
            .into_iter()
            .enumerate()
            .map(|(i, xy)| Borehole {
                id: i as u32 + 1,
                xy,
                mouth_z: 0.,
                layers: vec![Layer {
                    material_id: Some(7),
                    bottom_z: -2.,
                    e: 2500. + i as f64 * 20.,
                    gamma: 2.,
                }],
            })
            .collect::<Vec<_>>();
        let faces = volume_geometry(&holes, None);
        assert_eq!(faces.len(), 12);
        let mut edges = std::collections::BTreeMap::new();
        for (tri, _) in faces {
            let nodes = tri.map(|p| p.map(|v| v as i64));
            for (a, b) in nodes.into_iter().zip(nodes.into_iter().cycle().skip(1)) {
                *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        assert!(edges.values().all(|&uses| uses == 2));
    }
}
