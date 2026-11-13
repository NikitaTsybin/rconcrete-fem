use super::*;
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
use crate::plan_trace::underlay::{Data, Underlay};
pub(super) struct PdfChoice {
    bytes: Vec<u8>,
    name: String,
    pages: usize,
    selected: usize,
}
pub(super) enum PdfImport {
    Choose(PdfChoice),
    Ready(Vec<Underlay>),
}
pub(super) fn prepare_pdf(bytes: Vec<u8>, name: String) -> Result<PdfImport, String> {
    let pages = crate::plan_trace::pdf_import::page_count(&bytes)?;
    if pages == 1 {
        Ok(PdfImport::Ready(
            crate::plan_trace::underlay::from_pdf_bytes(bytes, &name)?,
        ))
    } else {
        Ok(PdfImport::Choose(PdfChoice {
            bytes,
            name,
            pages,
            selected: 0,
        }))
    }
}
pub(super) fn read_pdf_input(path: &std::path::Path) -> Result<PdfImport, String> {
    let input = crate::plan_trace::platform::read(path).map_err(|e| e.to_string())?;
    let length = input.len() as u64;
    if length > 64 * 1024 * 1024 {
        return Err("PDF превышает 64 МБ".into());
    }
    prepare_pdf(
        input,
        path.file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .into_owned(),
    )
}
impl App {
    pub(super) fn pdf_busy(&self) -> bool {
        self.pdf_load.is_some() || self.pdf_choice.is_some()
    }
    pub(super) fn accept_pdf_import(&mut self, result: PdfImport) {
        match result {
            PdfImport::Ready(pages) => self.append_pdf_pages(pages),
            PdfImport::Choose(choice) => {
                self.status = format!("PDF: выберите страницу из {}", choice.pages);
                self.pdf_choice = Some(choice);
            }
        }
    }
    fn render_pdf_choice(&mut self, ctx: &egui::Context) {
        let Some(choice) = self.pdf_choice.take() else {
            return;
        };
        let (tx, rx) = mpsc::channel();
        self.pdf_load = Some(rx);
        self.status = format!("PDF: чтение страницы {}…", choice.selected + 1);
        let ctx = ctx.clone();
        crate::plan_trace::platform::spawn(move || {
            let result = crate::plan_trace::underlay::from_pdf_page(
                choice.bytes,
                &choice.name,
                choice.selected,
            )
            .map(|p| Some(PdfImport::Ready(vec![p])));
            let _ = tx.send(result);
            ctx.request_repaint();
        });
    }
    pub(super) fn pdf_page_dialog(&mut self, ctx: &egui::Context) {
        let Some(choice) = self.pdf_choice.as_mut() else {
            return;
        };
        let mut add = false;
        let mut cancel = false;
        let response = egui::Modal::new(egui::Id::new("pdf_page_choice")).show(ctx, |ui| {
            ui.set_width(340_f32.min((ctx.screen_rect().width() - 40.).max(180.)));
            ui.heading("Выбор страницы PDF");
            ui.add(egui::Label::new(&choice.name).wrap());
            ui.label(format!("Страниц в документе: {}", choice.pages));
            egui::ComboBox::from_label("Страница")
                .selected_text(format!("{}", choice.selected + 1))
                .height(220.)
                .show_ui(ui, |ui| {
                    for index in 0..choice.pages {
                        ui.selectable_value(&mut choice.selected, index, format!("{}", index + 1));
                    }
                });
            ui.horizontal(|ui| {
                add = ui.button("Добавить").clicked();
                cancel = ui.button("Отмена").clicked();
            });
        });
        if cancel || response.should_close() {
            self.pdf_choice = None;
            self.status = "Выбор страницы PDF отменён".into();
        } else if add {
            self.render_pdf_choice(ctx);
        }
    }

    pub(super) fn append_dxf(&mut self, drawing: Drawing) {
        self.migrate_underlay();
        let id = self.next_underlay_id();
        self.underlays.push(Underlay {
            transparency: 0.,
            id,
            name: drawing.name.clone(),
            visible: true,
            scale: 1.,
            scale_y: None,
            angle_rad: 0.,
            origin: [0., 0.],
            data: Data::Dxf(drawing),
        });
        self.sync_underlays();
        self.fit = true;
    }
    fn next_underlay_id(&self) -> u64 {
        self.underlays.iter().map(|u| u.id).max().unwrap_or(0) + 1
    }
    pub(super) fn migrate_underlay(&mut self) {
        if self.underlays.is_empty() {
            if let Some(d) = self.drawing.take() {
                let old_styles = std::mem::take(&mut self.layers);
                self.layers = old_styles
                    .into_iter()
                    .map(|(name, style)| (format!("#1 · {name}"), style))
                    .collect();
                self.underlays.push(Underlay {
                    transparency: 0.,
                    id: 1,
                    name: d.name.clone(),
                    visible: true,
                    scale: 1.,
                    scale_y: None,
                    angle_rad: 0.,
                    origin: [0., 0.],
                    data: Data::Dxf(d),
                });
            }
        }
    }
    pub(super) fn sync_underlays(&mut self) {
        if self.underlays.is_empty() {
            self.drawing = None;
            self.layers.clear();
            self.rebuild_index();
            return;
        }
        let mut d = Drawing {
            unit_mm: 1.,
            name: "Подложки".into(),
            ..Default::default()
        };
        for object in &self.underlays {
            if let Some(source) = object.drawing() {
                d.paths.extend(source.paths);
                d.fills.extend(source.fills);
                d.labels.extend(source.labels);
                d.nodes.extend(source.nodes);
                d.node_layers.extend(source.node_layers);
                d.layers.extend(source.layers);
                for (key, value) in source.skipped {
                    *d.skipped.entry(key).or_default() += value;
                }
            }
        }
        let valid = self
            .underlays
            .iter()
            .filter_map(|u| {
                if let Data::Dxf(d) = &u.data {
                    Some((u.id, d))
                } else {
                    None
                }
            })
            .flat_map(|(id, d)| {
                d.layers
                    .iter()
                    .map(move |l| (format!("#{id} · {}", l.name), l))
            })
            .collect::<Vec<_>>();
        self.layers
            .retain(|name, _| valid.iter().any(|(key, _)| key == name));
        for (name, l) in valid {
            self.layers.entry(name).or_insert(LayerStyle {
                visible: l.visible,
                color: None,
                original: l.color.rgb(),
                width: 1.,
            });
        }
        for l in &d.layers {
            self.layers.entry(l.name.clone()).or_insert(LayerStyle {
                visible: l.visible,
                color: None,
                original: l.color.rgb(),
                width: 1.,
            });
        }
        self.drawing = Some(d);
        self.rebuild_index();
    }
    pub(super) fn underlay_controls(&mut self, ui: &mut egui::Ui) {
        let busy = self.pdf_busy() || self.load.is_some();
        if ui
            .add_enabled(!busy, egui::Button::new("Импорт *.PDF"))
            .clicked()
        {
            let (tx, rx) = mpsc::channel();
            self.pdf_load = Some(rx);
            let ctx = ui.ctx().clone();
            crate::plan_trace::platform::spawn(move || {
                let result = if let Some(path) = rfd::FileDialog::new()
                    .add_filter("PDF", &["pdf"])
                    .pick_file()
                {
                    read_pdf_input(&path).map(Some)
                } else {
                    Ok(None)
                };
                let _ = tx.send(result);
                ctx.request_repaint();
            });
        }
        if self.pdf_load.is_some() {
            ui.spinner();
        }
    }
    pub(super) fn underlay_list_controls(&mut self, ui: &mut egui::Ui) {
        if self.underlays.is_empty() && self.drawing.is_some() {
            self.migrate_underlay();
            self.sync_underlays();
        }
        let mut changed = false;
        let mut layers_changed = false;
        let mut remove = None;
        let mut select = None;
        let locked = self.underlays_locked || self.input_locked;
        let selected = self.selected_underlay;
        let selected_ids = self.underlay_selection.clone();
        let visible = self.show_dxf;
        let floor_list = self.floor_list;
        let building = self.building.clone();
        for is_dxf in [true, false] {
            let label = if is_dxf { "DXF" } else { "PDF" };
            let count = self
                .underlays
                .iter()
                .filter(|u| {
                    matches!(u.data, Data::Dxf(_)) == is_dxf
                        && floor_list.is_none_or(|id| {
                            building
                                .underlay_floors
                                .get(&u.id)
                                .copied()
                                .unwrap_or(building.floors[0].id)
                                == id
                        })
                })
                .count();
            if count == 0 {
                continue;
            }
            egui::CollapsingHeader::new(format!("{label} ({count})"))
                .id_salt(("underlay_category", is_dxf))
                .default_open(false)
                .show(ui, |ui| {
                    for object in self.underlays.iter_mut().filter(|u| {
                        matches!(u.data, Data::Dxf(_)) == is_dxf
                            && floor_list.is_none_or(|id| {
                                building
                                    .underlay_floors
                                    .get(&u.id)
                                    .copied()
                                    .unwrap_or(building.floors[0].id)
                                    == id
                            })
                    }) {
                        ui.push_id(object.id, |ui| {
                            egui::CollapsingHeader::new(&object.name)
                                .id_salt("underlay_object")
                                .default_open(false)
                                .show(ui, |ui| {
                                    ui.horizontal(|ui| {
                                        if ui
                                            .add_enabled(
                                                visible
                                                    && object.visible
                                                    && object.transparency < 1.,
                                                egui::Button::new("Выделить").selected(
                                                    selected == Some(object.id)
                                                        || selected_ids.contains(&object.id),
                                                ),
                                            )
                                            .clicked()
                                        {
                                            select = Some(object.id);
                                        }
                                        changed |= ui
                                            .checkbox(&mut object.visible, "Показывать")
                                            .changed();
                                        if ui
                                            .add_enabled(!locked, egui::Button::new("−").small())
                                            .on_hover_text("Удалить подложку")
                                            .clicked()
                                        {
                                            remove = Some(object.id);
                                        }
                                    });
                                    let was_hidden = object.transparency >= 1.;
                                    let mut percent = object.transparency * 100.;
                                    if ui
                                        .add(
                                            egui::Slider::new(&mut percent, 0. ..=100.)
                                                .text("Прозрачность, %"),
                                        )
                                        .changed()
                                    {
                                        object.transparency = percent / 100.;
                                        changed |= was_hidden != (object.transparency >= 1.);
                                    }
                                    ui.add_enabled_ui(!locked, |ui| {
                                        egui::Grid::new("underlay_transform").num_columns(3).show(
                                            ui,
                                            |ui| {
                                                ui.label("Масштаб").on_hover_text("Пропорционально изменяет оба масштаба. Независимые масштабы и угол доступны в свойствах подложки.");
                                                ui.label("X, мм");
                                                ui.label("Y, мм");
                                                ui.end_row();
                                                let mut scale = object.scale;
                                                if ui
                                                    .add_sized(
                                                        [70., 20.],
                                                        egui::DragValue::new(&mut scale)
                                                            .speed(0.01)
                                                            .range(1e-9..=1e9),
                                                    )
                                                    .changed()
                                                {
                                                    let mut transform = object.transform();
                                                    let ratio = scale / transform.scale[0];
                                                    transform.scale = [scale, transform.scale[1] * ratio];
                                                    if transform.validate().is_ok() {
                                                        object.set_transform(transform);
                                                        changed = true;
                                                    }
                                                }
                                                changed |= ui
                                                    .add_sized(
                                                        [70., 20.],
                                                        egui::DragValue::new(&mut object.origin[0])
                                                            .speed(10.),
                                                    )
                                                    .changed();
                                                changed |= ui
                                                    .add_sized(
                                                        [70., 20.],
                                                        egui::DragValue::new(&mut object.origin[1])
                                                            .speed(10.),
                                                    )
                                                    .changed();
                                                ui.end_row();
                                            },
                                        );
                                    });
                                    if is_dxf {
                                        egui::CollapsingHeader::new("Слои DXF")
                                            .default_open(false)
                                            .show(ui, |ui| {
                                                layers_changed |=
                                                    layer_controls(ui, &mut self.layers, object.id);
                                            });
                                    }
                                });
                        });
                    }
                });
        }
        if let Some(id) = select {
            if self.building.enabled {
                self.change_floor(
                    self.building
                        .underlay_floors
                        .get(&id)
                        .copied()
                        .unwrap_or(self.building.floors[0].id),
                );
            }
            self.select_underlay(id);
        }
        if let Some(id) = remove {
            self.remove_underlays(&std::collections::BTreeSet::from([id]), false);
        }
        if changed {
            self.sync_underlays();
        } else if layers_changed {
            self.rebuild_index();
        }
    }
    pub(super) fn append_pdf_pages(&mut self, mut pages: Vec<Underlay>) {
        self.migrate_underlay();
        let next = self.next_underlay_id();
        for (i, p) in pages.iter_mut().enumerate() {
            p.id = next + i as u64;
        }
        self.status = format!("PDF: импортировано страниц {}", pages.len());
        self.underlays.extend(pages);
        self.sync_underlays();
        self.fit = true;
    }
    pub(super) fn poll_pdf(&mut self) {
        let result = self.pdf_load.as_ref().map(|r| r.try_recv());
        match result {
            Some(Ok(result)) => {
                self.pdf_load = None;
                match result {
                    Ok(Some(result)) => self.accept_pdf_import(result),
                    Ok(None) => self.status = "Выбор PDF отменён".into(),
                    Err(e) => self.status = e,
                }
            }
            Some(Err(mpsc::TryRecvError::Disconnected)) => {
                self.pdf_load = None;
                self.status = "Импорт PDF прерван".into();
            }
            _ => {}
        }
    }
    pub(super) fn underlay_ink(&self, layer: &str, ink: Color32) -> Color32 {
        let opacity = layer
            .strip_prefix('#')
            .and_then(|s| s.split_once(" · "))
            .and_then(|(id, _)| id.parse::<u64>().ok())
            .and_then(|id| self.underlays.iter().find(|u| u.id == id))
            .map_or(1., |u| 1. - u.transparency);
        ink.gamma_multiply(opacity.clamp(0., 1.))
    }
    pub(super) fn draw_pdf(&mut self, painter: &egui::Painter, rect: Rect) {
        if !self.show_dxf {
            return;
        }
        let building = &self.building;
        let hidden = self
            .underlays
            .iter()
            .filter(|u| self.underlay_hidden(u.id))
            .map(|u| u.id)
            .collect::<std::collections::BTreeSet<_>>();
        for u in self.underlays.iter().filter(|u| {
            (!building.enabled
                || building
                    .underlay_floors
                    .get(&u.id)
                    .copied()
                    .unwrap_or(building.floors[0].id)
                    == building.active)
                && !hidden.contains(&u.id)
                && u.visible
                && u.transparency < 1.
        }) {
            let Data::Pdf {
                width,
                height,
                rgba,
                size_mm,
                ..
            } = &u.data
            else {
                continue;
            };
            let corners = [[0., 0.], [size_mm[0], 0.], *size_mm, [0., size_mm[1]]]
                .map(|p| self.camera.screen(u.point(p), rect));
            if !Rect::from_points(&corners).intersects(rect) {
                continue;
            }
            let texture = self.pdf_textures.entry(u.id).or_insert_with(|| {
                painter.ctx().load_texture(
                    format!("pdf_{}", u.id),
                    egui::ColorImage::from_rgba_unmultiplied(
                        [*width as usize, *height as usize],
                        rgba,
                    ),
                    egui::TextureOptions::LINEAR,
                )
            });
            let mut mesh = egui::Mesh::with_texture(texture.id());
            for (pos, uv) in corners
                .into_iter()
                .zip([[0., 1.], [1., 1.], [1., 0.], [0., 0.]])
            {
                mesh.vertices.push(egui::epaint::Vertex {
                    pos,
                    uv: egui::pos2(uv[0], uv[1]),
                    color: Color32::WHITE.gamma_multiply(1. - u.transparency),
                });
            }
            mesh.add_triangle(0, 1, 2);
            mesh.add_triangle(0, 2, 3);
            painter.add(egui::Shape::mesh(mesh));
        }
    }
}

// Namespaced keys keep identically named layers in different files independent.
fn layer_controls(ui: &mut egui::Ui, layers: &mut BTreeMap<String, LayerStyle>, id: u64) -> bool {
    let prefix = format!("#{id} · ");
    let mut changed = false;
    egui::ScrollArea::vertical()
        .id_salt("object_layers")
        .max_height(260.)
        .show(ui, |ui| {
            egui::Grid::new("object_layer_table")
                .num_columns(5)
                .spacing([4., 4.])
                .striped(true)
                .show(ui, |ui| {
                    ui.label("Вкл.");
                    ui.label("Слой");
                    ui.label("Цвет");
                    ui.label("Толщ., px");
                    ui.label("");
                    ui.end_row();
                    for (name, style) in layers
                        .iter_mut()
                        .filter(|(name, _)| name.starts_with(&prefix))
                    {
                        changed |= ui.checkbox(&mut style.visible, "").changed();
                        let label = name.strip_prefix(&prefix).unwrap();
                        ui.add_sized([80., 20.], egui::Label::new(label).truncate())
                            .on_hover_text(label);
                        let mut color = style.color.unwrap_or(style.original);
                        if crate::plan_trace::color_picker::button(ui, &mut color).changed() {
                            style.color = Some(color);
                        }
                        ui.add(
                            egui::DragValue::new(&mut style.width)
                                .speed(0.1)
                                .range(0.1..=12.),
                        );
                        if ui
                            .small_button("↺")
                            .on_hover_text("Исходные цвета и толщина")
                            .clicked()
                        {
                            style.color = None;
                            style.width = 1.;
                        }
                        ui.end_row();
                    }
                });
        });
    changed
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn multipage_pdf_waits_for_choice_and_imports_only_the_chosen_original_page_number() {
        let bytes = crate::plan_trace::pdf_import::test_pdf(20);
        let mut a = App::default();
        a.accept_pdf_import(prepare_pdf(bytes, "План.pdf".into()).unwrap());
        assert!(a.pdf_busy() && a.pdf_load.is_none() && a.underlays.is_empty());
        let choice = a.pdf_choice.as_mut().unwrap();
        assert_eq!(choice.pages, 20);
        choice.selected = 19;
        a.render_pdf_choice(&egui::Context::default());
        assert!(a.pdf_choice.is_none() && a.pdf_load.is_some() && a.underlays.is_empty());
        let result = a
            .pdf_load
            .take()
            .unwrap()
            .recv_timeout(Duration::from_secs(30))
            .unwrap()
            .unwrap()
            .unwrap();
        a.accept_pdf_import(result);
        assert!(!a.pdf_busy());
        assert_eq!(a.underlays.len(), 1);
        assert_eq!(a.underlays[0].name, "План.pdf · стр. 20");
        let Data::Pdf {
            width,
            height,
            page,
            rgba,
            size_mm,
            ..
        } = &a.underlays[0].data
        else {
            panic!("PDF")
        };
        assert_eq!((*width, *height, *page), (1024, 2048, 20));
        assert_eq!(&rgba[..4], &[0, 255, 0, 255]);
        assert!((size_mm[0] - 100. * 25.4 / 72.).abs() < 1e-5);
    }
    #[test]
    fn pdf_page_choice_cancel_and_file_cancel_clear_busy_state_without_changing_scene() {
        let ctx = egui::Context::default();
        let mut a = App::default();
        a.accept_pdf_import(
            prepare_pdf(crate::plan_trace::pdf_import::test_pdf(2), "two.pdf".into()).unwrap(),
        );
        let _ = ctx.run(egui::RawInput::default(), |ctx| a.pdf_page_dialog(ctx));
        let _ = ctx.run(
            egui::RawInput {
                events: vec![egui::Event::Key {
                    key: egui::Key::Escape,
                    physical_key: None,
                    pressed: true,
                    repeat: false,
                    modifiers: egui::Modifiers::NONE,
                }],
                ..Default::default()
            },
            |ctx| a.pdf_page_dialog(ctx),
        );
        assert!(!a.pdf_busy() && a.underlays.is_empty());
        let (tx, rx) = mpsc::channel();
        a.pdf_load = Some(rx);
        tx.send(Ok(None)).unwrap();
        a.poll_pdf();
        assert!(!a.pdf_busy() && a.underlays.is_empty());
        let (tx, rx) = mpsc::channel();
        a.pdf_load = Some(rx);
        drop(tx);
        a.poll_pdf();
        assert!(!a.pdf_busy());
    }
    #[test]
    fn single_page_pdf_keeps_immediate_import_behavior() {
        let mut a = App::default();
        let input =
            prepare_pdf(crate::plan_trace::pdf_import::test_pdf(1), "one.pdf".into()).unwrap();
        assert!(matches!(&input,PdfImport::Ready(pages) if pages.len()==1));
        a.accept_pdf_import(input);
        assert_eq!(a.underlays.len(), 1);
        assert!(!a.pdf_busy());
        let Data::Pdf { page, .. } = a.underlays[0].data else {
            panic!("PDF")
        };
        assert_eq!(page, 1);
    }
    #[test]
    fn dxf_objects_scale_independently_and_keep_namespaced_layers_and_source_geometry() {
        let d=import::from_bytes(b"0\nSECTION\n2\nENTITIES\n0\nLINE\n8\nPlan\n10\n0\n20\n0\n11\n1000\n21\n0\n0\nENDSEC\n0\nEOF\n","one.dxf").unwrap();
        let mut a = App::default();
        a.loaded(d.clone());
        a.layers.get_mut("Plan").unwrap().color = Some([1, 2, 3]);
        a.append_dxf(d);
        assert_eq!(a.underlays.len(), 2);
        assert_eq!(a.layers["#1 · Plan"].color, Some([1, 2, 3]));
        a.underlays[0].scale = 2.;
        a.underlays[0].origin = [100., 200.];
        a.sync_underlays();
        let proxy = a.drawing.as_ref().unwrap();
        assert_eq!(proxy.paths[0].points, vec![[100., 200.], [2100., 200.]]);
        assert_eq!(proxy.paths[1].points, vec![[0., 0.], [1000., 0.]]);
        let Data::Dxf(source) = &a.underlays[0].data else {
            panic!()
        };
        assert_eq!(source.paths[0].points, vec![[0., 0.], [1000., 0.]]);
        assert!(a.index.query([100., 200.], 1., &[], &[]).is_some());
        a.underlays[0].visible = false;
        a.sync_underlays();
        assert!(a.index.query([100., 200.], 1., &[], &[]).is_none());
        assert_eq!(a.layers["#1 · Plan"].color, Some([1, 2, 3]));
    }
}
