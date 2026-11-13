use super::*;
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
pub(super) struct Job {
    rx: mpsc::Receiver<Result<String, String>>,
    stage: std::sync::Arc<std::sync::Mutex<String>>,
    cancel: crate::plan_trace::fem::progress::CancelToken,
    start: Instant,
}

#[cfg(all(test, not(target_arch = "wasm32")))]
#[test]
#[ignore = "requires MINI_FEM_LIRA_WALL_PROJECT pointing to a local slab project"]
fn local_project_lira_wall_export() {
    use crate::plan_trace::{columns, fem, lira_export};

    let mut app = App::default();
    let path = std::env::var_os("MINI_FEM_LIRA_WALL_PROJECT").expect("fixture path");
    app.load_project_path(path.into()).unwrap();
    let ctx = egui::Context::default();
    app.start_mesh(&ctx);
    while app.mesh_job.is_some() {
        std::thread::sleep(std::time::Duration::from_millis(10));
        app.poll_mesh();
    }
    let mesh = app
        .fe_mesh
        .as_ref()
        .unwrap_or_else(|| panic!("{}", app.status));
    let (scene, loads) = fem::lira_model(
        mesh,
        &app.contours,
        &app.materials,
        &app.load_cases,
        app.gravity,
        app.mesh_built_size,
        &app.support_height,
    )
    .unwrap();
    if std::env::var_os("MINI_FEM_REPORT_MESH_QUALITY").is_some() {
        let length = |v: [f64; 3]| v.into_iter().map(|x| x * x).sum::<f64>().sqrt();
        let subtract = |a: [f64; 3], b: [f64; 3]| std::array::from_fn(|i| a[i] - b[i]);
        let mut worst = scene
            .surfaces
            .iter()
            .enumerate()
            .filter_map(|(i, s)| {
                let xyz = s.nodes.iter().map(|&n| scene.xyz[n]).collect::<Vec<_>>();
                let mut height = f64::INFINITY;
                let mut edge = f64::INFINITY;
                for j in 0..xyz.len() {
                    let a = xyz[j];
                    let b = xyz[(j + 1) % xyz.len()];
                    let c = xyz[(j + 2) % xyz.len()];
                    let u = subtract(b, a);
                    let v = subtract(c, a);
                    let twice_area = length(std::array::from_fn(|k| {
                        let m = (k + 1) % 3;
                        let n = (k + 2) % 3;
                        u[m] * v[n] - u[n] * v[m]
                    }));
                    height = height
                        .min(twice_area / length(u).max(length(v)).max(length(subtract(c, b))));
                    edge = edge.min(length(u));
                }
                (height < 1. || edge < 1.).then_some((height, edge, i + 1, s.support, xyz))
            })
            .collect::<Vec<_>>();
        worst.sort_by(|a, b| a.0.total_cmp(&b.0));
        for threshold in [0.0001, 0.001, 0.01, 0.1, 1.] {
            eprintln!(
                "Quality below {threshold} mm: all={}, slabs={}, walls={}",
                worst.iter().filter(|s| s.0.min(s.1) < threshold).count(),
                worst
                    .iter()
                    .filter(|s| s.0.min(s.1) < threshold && s.3.is_none())
                    .count(),
                worst
                    .iter()
                    .filter(|s| s.0.min(s.1) < threshold && s.3.is_some())
                    .count()
            );
        }
        for row in worst.iter().take(16) {
            eprintln!("{row:?}");
        }
    }
    for index in [185745, 188579] {
        if let Some(s) = scene.surfaces.get(index - 1) {
            eprintln!(
                "КЭ {index}: {:?} {:?}",
                s.support,
                s.nodes.iter().map(|&n| scene.xyz[n]).collect::<Vec<_>>()
            );
            if let Some(c) = app.contours.iter().find(|c| Some(c.id) == s.support) {
                eprintln!("Объект {}: {:?} {:?}", c.id, c.points, c.placement);
            }
        }
    }
    let rigid = columns::append_contour_rigid_links(
        columns::rigid(&app.contours, &app.support_height, mesh, &scene),
        mesh,
        &scene,
    )
    .unwrap();
    let (c1, horizontal) = fem::lira_foundation(mesh, &app.contours, app.gravity).unwrap();
    let mut export = lira_export::encode_with_rigid_and_springs(
        &scene,
        &loads,
        &app.load_cases,
        app.gravity,
        "Проверка",
        &rigid,
        &horizontal,
    )
    .unwrap();
    lira_export::append_foundation(&mut export, &scene, &c1, app.gravity).unwrap();
    let (_, _, errors) = encoding_rs::WINDOWS_1251.encode(&export.text);
    assert!(!errors);
    let library = app.stiffnesses.for_variant(app.stiffness_variant);
    let mut metadata =
        lira_export::material_metadata(&export, &library, &app.contours, "Проверка", app.gravity);
    lira_export::append_object_blocks(
        &mut metadata,
        &scene,
        mesh,
        &app.contours,
        app.mesh_built_size,
    )
    .unwrap();
    eprintln!(
        "Экспорт: {} узлов, {} КЭ, {} АЖТ, {} загружений",
        export.nodes,
        export.elements,
        export.rigid_bodies,
        app.load_cases.len()
    );
}

impl App {
    pub(super) fn export_lira_txt(&mut self, ctx: &egui::Context) {
        if self.lira_export_job.is_some() {
            return;
        }
        let Some(mesh) = self.fe_mesh.as_ref() else {
            self.status = "Сначала выполните триангуляцию".into();
            return;
        };
        if !model::same_mesh_input(&self.mesh_input, &self.contours)
            || self.mesh_built_size != self.mesh_size
        {
            self.status = "Перестройте сетку перед экспортом".into();
            return;
        }
        let Some(path) = rfd::FileDialog::new()
            .add_filter("ЛИРА TXT", &["txt"])
            .set_file_name("Модель ЛИРА.txt")
            .save_file()
        else {
            return;
        };
        let mesh = mesh.clone();
        let contours = self.contours.clone();
        let materials = self.materials.clone();
        let library = self.stiffnesses.for_variant(self.stiffness_variant);
        let cases = self.load_cases.clone();
        let settings = self.support_height.clone();
        let gravity = self.gravity;
        let step = self.mesh_built_size;
        let title = self
            .project_path
            .as_ref()
            .and_then(|p| p.file_stem())
            .and_then(|s| s.to_str())
            .unwrap_or("Rconcrete FEM")
            .to_owned();
        let (tx, rx) = mpsc::channel();
        let stage = std::sync::Arc::new(std::sync::Mutex::new("Подготовка модели".to_owned()));
        let cancel = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.lira_export_job = Some(Job {
            rx,
            stage: stage.clone(),
            cancel: cancel.clone(),
            start: Instant::now(),
        });
        let ctx = ctx.clone();
        crate::plan_trace::platform::spawn(move || {
            crate::plan_trace::fem::progress::install_cancel(cancel);
            let repaint = ctx.clone();
            crate::plan_trace::fem::progress::install(move |label, _| {
                *stage.lock().unwrap() = label;
                repaint.request_repaint();
            });
            let result = (|| -> Result<String, String> {
                crate::plan_trace::fem::progress::stage("Экспорт ЛИРА: сборка модели и нагрузок");
                let (mut scene, loads) = crate::plan_trace::fem::lira_model(
                    &mesh, &contours, &materials, &cases, gravity, step, &settings,
                )?;
                if mesh.building.is_none() && !settings.enabled {
                    for &node in mesh
                        .support_boundary
                        .iter()
                        .filter(|(id, _)| !settings.columns.contains_key(id))
                        .flat_map(|(_, nodes)| nodes)
                    {
                        scene.fixed.extend([3 * node, 3 * node + 1, 3 * node + 2]);
                    }
                }
                crate::plan_trace::fem::progress::check_cancel()?;
                crate::plan_trace::fem::progress::stage("Экспорт ЛИРА: подготовка АЖТ");
                let rigid = crate::plan_trace::columns::rigid(&contours, &settings, &mesh, &scene);
                let expected = if settings.rigid_columns {
                    crate::plan_trace::columns::joints(
                        &contours,
                        &settings.columns,
                        mesh.building.as_ref().map(|d| &d.layout),
                    )
                    .len()
                } else {
                    0
                };
                if rigid.len() != expected {
                    return Err("Не найдены ведущие узлы АЖТ. Перестройте сетку".into());
                }
                let rigid =
                    crate::plan_trace::columns::append_contour_rigid_links(rigid, &mesh, &scene)?;
                crate::plan_trace::fem::progress::stage("Экспорт ЛИРА: формирование TXT");
                let (c1, horizontal) =
                    crate::plan_trace::fem::lira_foundation(&mesh, &contours, gravity)?;
                let mut export = crate::plan_trace::lira_export::encode_with_rigid_and_springs(
                    &scene,
                    &loads,
                    &cases,
                    gravity,
                    &title,
                    &rigid,
                    &horizontal,
                )?;
                crate::plan_trace::lira_export::append_foundation(
                    &mut export,
                    &scene,
                    &c1,
                    gravity,
                )?;
                crate::plan_trace::fem::progress::check_cancel()?;
                crate::plan_trace::fem::progress::stage("Экспорт ЛИРА: кодирование TXT");
                let (bytes, _, errors) = encoding_rs::WINDOWS_1251.encode(&export.text);
                if errors {
                    return Err(
                        "Имена содержат символы вне Windows-1251. Переименуйте загружения/проект"
                            .into(),
                    );
                }
                crate::plan_trace::fem::progress::check_cancel()?;
                crate::plan_trace::fem::progress::stage("Экспорт ЛИРА: запись файла");
                let mut metadata = crate::plan_trace::lira_export::material_metadata(
                    &export, &library, &contours, &title, gravity,
                );
                crate::plan_trace::lira_export::append_object_blocks(
                    &mut metadata,
                    &scene,
                    &mesh,
                    &contours,
                    step,
                )?;
                crate::plan_trace::platform::write(
                    &path.with_extension("ald"),
                    metadata.as_bytes(),
                )
                .map_err(|e| format!("Не удалось сохранить комментарии жёсткостей ALD: {e}"))?;
                crate::plan_trace::platform::write(&path, bytes)
                    .map_err(|e| format!("Не удалось сохранить TXT: {e}"))?;
                Ok(format!(
                    "Экспорт ЛИРА TXT + ALD: {} узлов, {} КЭ, {} АЖТ, {} загружений · {}",
                    export.nodes,
                    export.elements,
                    export.rigid_bodies,
                    cases.len(),
                    path.display()
                ))
            })();
            #[cfg(target_arch = "wasm32")]
            {
                crate::plan_trace::fem::progress::install_cancel(std::sync::Arc::new(
                    std::sync::atomic::AtomicBool::new(false),
                ));
                crate::plan_trace::fem::progress::install(|_, _| {});
            }
            let _ = tx.send(result);
            ctx.request_repaint();
        });
    }
    pub(super) fn poll_lira_export(&mut self) {
        let Some(job) = &self.lira_export_job else {
            return;
        };
        let result = match job.rx.try_recv() {
            Ok(result) => Some(result),
            Err(mpsc::TryRecvError::Empty) => None,
            Err(mpsc::TryRecvError::Disconnected) => {
                Some(Err("Фоновый экспорт завершился без результата".into()))
            }
        };
        if let Some(result) = result {
            let elapsed = job.start.elapsed().as_secs_f64();
            self.lira_export_job = None;
            self.status = match result {
                Ok(message) => format!("{message} · {elapsed:.1} с"),
                Err(error) => format!("Экспорт ЛИРА: {error}"),
            };
        }
    }
    pub(super) fn lira_export_progress(&self, ui: &mut egui::Ui) {
        if let Some(job) = &self.lira_export_job {
            let stopping = job.cancel.load(std::sync::atomic::Ordering::Relaxed);
            let label = if stopping {
                "Остановка экспорта…".into()
            } else {
                job.stage.lock().unwrap().clone()
            };
            ui.add(egui::ProgressBar::new(0.).animate(false).text(format!(
                "{label} · {:.1} с",
                job.start.elapsed().as_secs_f64()
            )));
            if ui
                .add_enabled(!stopping, egui::Button::new("Остановить экспорт"))
                .clicked()
            {
                job.cancel.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            ui.ctx()
                .request_repaint_after(std::time::Duration::from_millis(200));
        }
    }
}
