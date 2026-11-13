use super::*;
use crate::plan_trace::reinforcement as rc;
use std::sync::{
    atomic::{AtomicBool, Ordering},
    Arc,
};

pub(super) struct State {
    #[cfg(target_arch = "wasm32")]
    worker: Option<crate::plan_trace::web_worker::Handle>,
    pub result: Option<Arc<rc::Output>>,
    key: Option<Vec<u8>>,
    job: Option<mpsc::Receiver<Result<rc::Output, String>>>,
    progress: Arc<rc::Progress>,
    cancel: Arc<AtomicBool>,
    total: usize,
    error: String,
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn building_wall_shell_results_feed_ndm_and_its_spatial_mosaic() {
        let mut app = App::default();
        app.building.enabled = true;
        app.support_height.automatic_building_bottom = true;
        app.support_height.automatic_building_top = true;
        app.load_cases[0].kind = crate::plan_trace::loads::LoadType::SelfWeight;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        app.begin(Kind::Support);
        app.sketch_click([0., 1000.], false);
        app.sketch_click([2000., 1000.], false);
        app.finish();
        app.cancel();
        let wall = app
            .contours
            .iter()
            .find(|c| c.kind == Kind::Support)
            .unwrap()
            .id;
        app.sync_stiffness_sources();
        let section = app.stiffnesses.assignments[&wall];
        let material = app.stiffnesses.material_id(wall).unwrap();
        app.stiffnesses.elastic.get_mut(&material).unwrap().concrete = Some(Default::default());
        let design = app.stiffnesses.design_assignments[&section];
        let settings = &mut app.stiffnesses.designs.get_mut(&design).unwrap().settings;
        settings.wall_height = true;
        settings.wall_length_factor = 0.9;
        settings.wall_length_from_analysis = true;
        app.sync_stiffness_sources();
        app.mesh_built_size = 1000.;
        let mesh = crate::plan_trace::building_mesh::generate(
            &app.contours,
            app.mesh_built_size,
            app.building_layout().unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let mut analysis = crate::plan_trace::fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_built_size,
            app.element_type,
            &app.support_height,
        )
        .unwrap();
        analysis.lengths = Some(Ok([0., 1.5]
            .into_iter()
            .map(
                |bottom_m| crate::plan_trace::fem::shell::effective_length::Length {
                    id: wall,
                    axis: "Из плоскости".into(),
                    segment: Some([[0., 1000.], [2000., 1000.]]),
                    length_m: 1.5,
                    bottom_m,
                    effective_m: Some(0.9),
                    critical_kn: None,
                    diagnostic: None,
                },
            )
            .collect()));
        app.fe_mesh = Some(mesh);
        app.analysis = Some(analysis);
        app.analysis_input = Some(app.analysis_key());
        let input = app.reinforcement_input().unwrap();
        let wall_groups = input
            .groups
            .iter()
            .filter_map(|g| g.wall)
            .collect::<Vec<_>>();
        assert!(!wall_groups.is_empty());
        assert!(wall_groups
            .iter()
            .all(|wall| (wall.effective_m - 0.9).abs() < 1e-9 && wall.height_m == 1.5));
        let first = &input.cases[&app.load_cases[0].id];
        let plate_count = app.fe_mesh.as_ref().unwrap().elements.len();
        assert!(first[plate_count..].iter().any(|f| f.n[1] < 0.));
        let output = rc::run(
            input,
            Arc::new(rc::Progress::default()),
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        assert_eq!(output.wall_displays.len(), wall_groups.len());
        assert!(
            output
                .wall_displays
                .values()
                .all(|&i| output.cells[i].verified && output.cells[i].governing_angle.is_some()),
            "{:?}",
            output.errors
        );
        let mesh = app.fe_mesh.as_ref().unwrap();
        let scene = crate::plan_trace::fem::shell::scene(
            mesh,
            &vec![crate::plan_trace::fem::element::Material::default(); plate_count],
            &app.support_height,
            app.mesh_built_size,
            &app.contours,
        )
        .unwrap();
        app.schema_analysis.mode = Some(schema_analysis::Mode::Floors);
        app.length_view = 1;
        let (send, receive) = mpsc::channel();
        send.send(Ok(output)).unwrap();
        app.reinforcement.job = Some(receive);
        app.poll_reinforcement(&egui::Context::default());
        assert!(app.show_results && app.result_field == 26);
        assert!(app.schema_analysis.mode.is_none());
        assert_eq!(app.length_view, 0);
        app.result_field = 28;
        for wall in &wall_groups {
            assert!(app
                .spatial_surface_values(wall.surface, &scene.surfaces[wall.surface])
                .unwrap()
                .iter()
                .all(|v| v.is_finite() && *v > 0.));
        }
        app.reinforcement_directions = true;
        let world = spatial_world::geometry(&app, &scene, [0.; 3], 2000., None, vec![1]);
        assert_eq!(
            world
                .direction_labels
                .iter()
                .filter(|(p, _)| p[2] != 0.)
                .count(),
            wall_groups.len()
        );
        assert_eq!(
            world
                .edges
                .iter()
                .filter(|edge| edge[0].color == [1., 1., 0., 1.])
                .count(),
            world.direction_labels.len()
        );
        let reused = spatial_world::geometry(&app, &scene, [0.; 3], 2000., None, vec![1]);
        assert!(Arc::ptr_eq(&world.edges, &reused.edges));

        app.support_height.length_method = crate::plan_trace::fem::shell::LengthMethod::Spatial;
        app.contours
            .iter_mut()
            .find(|c| c.id == wall)
            .unwrap()
            .calculate_effective_length = true;
        app.analysis.as_mut().unwrap().lengths = None;
        app.analysis_input = Some(app.analysis_key());
        let fallback = app.reinforcement_input().unwrap();
        let fallback_walls = fallback
            .groups
            .iter()
            .filter_map(|group| group.wall)
            .collect::<Vec<_>>();
        assert_eq!(fallback_walls.len(), wall_groups.len());
        assert!(fallback_walls
            .iter()
            .all(|wall| (wall.effective_m - 2.7).abs() < 1e-9 && wall.height_m == 3.));
    }

    #[test]
    fn changing_rc_properties_invalidates_design_without_clearing_fem_results() {
        let mut app = App::default();
        app.analysis = Some(crate::plan_trace::fem::Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![],
            free_dofs: 1,
        });
        app.reinforcement.key = app.reinforcement_key();
        app.reinforcement.result = Some(Arc::new(rc::Output {
            cells: vec![],
            physical_count: 0,
            errors: vec![],
            wall_displays: Default::default(),
        }));
        app.result_field = 26;
        app.result_generation += 1;
        app.combination_variant = crate::plan_trace::load_combinations::Variant::Normative;
        let ctx = egui::Context::default();
        app.poll_reinforcement(&ctx);
        assert!(app.reinforcement.result.is_some());
        app.combinations[0].name = "Переименовано".into();
        app.poll_reinforcement(&ctx);
        assert!(app.reinforcement.result.is_some());
        app.stiffnesses
            .entries
            .get_mut(&1)
            .unwrap()
            .reinforcement
            .asbx += 1.;
        app.poll_reinforcement(&ctx);
        assert!(app.analysis.is_some());
        assert!(app.reinforcement.result.is_none());
    }
    #[test]
    fn rc_settings_survive_property_roundtrip_and_old_projects_get_defaults() {
        let mut property = crate::plan_trace::stiffness::Library::default().entries[&1].clone();
        property.reinforcement.asby = 42.;
        property.reinforcement.concrete = 3;
        let mut json = serde_json::to_value(&property).unwrap();
        let restored: crate::plan_trace::stiffness::Entry =
            serde_json::from_value(json.clone()).unwrap();
        assert_eq!(restored.reinforcement, property.reinforcement);
        json.as_object_mut().unwrap().remove("reinforcement");
        let restored: crate::plan_trace::stiffness::Entry = serde_json::from_value(json).unwrap();
        assert_eq!(restored.reinforcement, rc::Settings::default());
    }
    #[test]
    #[ignore = "requires MINI_FEM_PROJECT_FIXTURE pointing to a local slab project"]
    fn local_project_pilot_reinforcement_and_crack_report() {
        let mut app = App::default();
        let path =
            PathBuf::from(std::env::var_os("MINI_FEM_PROJECT_FIXTURE").expect("fixture path"));
        app.load_project_path(path).unwrap();
        let ctx = egui::Context::default();
        app.start_mesh(&ctx);
        while app.mesh_job.is_some() {
            std::thread::sleep(Duration::from_millis(10));
            app.poll_mesh();
        }
        let mesh = app
            .fe_mesh
            .take()
            .unwrap_or_else(|| panic!("{}", app.status));
        app.analysis = Some(
            crate::plan_trace::fem::solve_with_modal(
                &mesh,
                &app.contours,
                &app.materials,
                &app.load_cases,
                app.gravity,
                app.mesh_size,
                app.element_type,
                &app.support_height,
                None,
                app.calculate_effective_lengths,
                None,
            )
            .unwrap(),
        );
        app.fe_mesh = Some(mesh);
        app.mesh_generation += 1;
        app.mesh_built_size = app.mesh_size;
        app.analysis_input = Some(app.analysis_key());
        // Real-project interchange check: physical topology and applied resultant.
        let mesh = app.fe_mesh.as_ref().unwrap();
        if mesh.building.is_none() {
            let (mut scene, loads) = crate::plan_trace::fem::lira_model(
                mesh,
                &app.contours,
                &app.materials,
                &app.load_cases,
                app.gravity,
                app.mesh_built_size,
                &app.support_height,
            )
            .unwrap();
            if mesh.building.is_none() && !app.support_height.enabled {
                for &n in mesh
                    .support_boundary
                    .iter()
                    .filter(|(id, _)| !app.support_height.columns.contains_key(id))
                    .flat_map(|(_, nodes)| nodes)
                {
                    scene.fixed.extend([3 * n, 3 * n + 1, 3 * n + 2]);
                }
            }
            for (case, forces) in app.analysis.as_ref().unwrap().cases.iter().zip(&loads) {
                let total = -forces.resultant(&scene)[0];
                assert!(
                    (total - case.applied_kn).abs() < 1e-8 * total.abs().max(1.),
                    "export load balance: {}",
                    case.id
                );
            }
            let rigid =
                crate::plan_trace::columns::rigid(&app.contours, &app.support_height, mesh, &scene);
            let export = crate::plan_trace::lira_export::encode_with_rigid(
                &scene,
                &loads,
                &app.load_cases,
                app.gravity,
                "Project fixture",
                &rigid,
            )
            .unwrap();
            let parsed = crate::plan_trace::lira_txt_reader::parse(&export.text).unwrap();
            assert_eq!(parsed.nodes.len(), scene.xyz.len());
            assert_eq!(
                parsed.elements.len(),
                scene.surfaces.len() + scene.beams.len()
            );
            let (bytes, _, errors) = encoding_rs::WINDOWS_1251.encode(&export.text);
            assert!(!errors);
            std::fs::write("target/slab_lira_project_export.txt", bytes).unwrap();
            println!(
                "LIRA export: {} nodes, {} physical elements, {} cases",
                export.nodes,
                export.elements,
                loads.len()
            );
        }
        let input = app.reinforcement_input().unwrap();
        if std::env::var_os("MINI_FEM_BENCH_CROSS").is_some() {
            let text = serde_json::to_string(&input).unwrap();
            std::fs::write("target/rc_cross_input.json", &text).unwrap();
            let mut rows = vec![];
            for mode in 0..3 {
                let normalized: rc::Input = serde_json::from_str(&text).unwrap();
                let row = rc::benchmark_strength_model(normalized, mode, |stage, index| {
                    println!("CROSS_PROGRESS mode={mode} stage={stage} element={index}")
                })
                .unwrap();
                println!("CROSS_RESULT {row}");
                rows.push(row);
                std::fs::write(
                    "target/rc_cross_native.json",
                    serde_json::to_vec_pretty(&rows).unwrap(),
                )
                .unwrap();
            }
            return;
        }
        if std::env::var_os("MINI_FEM_BENCH_RC").is_some() {
            let weights = app
                .fe_mesh
                .as_ref()
                .unwrap()
                .elements
                .iter()
                .map(|e| {
                    let [a, b, c] = e.nodes.map(|n| app.fe_mesh.as_ref().unwrap().nodes[n].xy);
                    ((b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])).abs() * 0.5e-6
                })
                .collect::<Vec<_>>();
            if std::env::var_os("MINI_FEM_BENCH_BATCH").is_some() {
                rc::benchmark_batch(input, &weights);
            } else {
                rc::benchmark_ml_vs_mechanics(input, &weights);
            }
            return;
        }
        let start = web_time::Instant::now();
        let progress = Arc::new(rc::Progress::default());
        let out = rc::run(
            input.clone(),
            progress.clone(),
            Arc::new(AtomicBool::new(false)),
        )
        .unwrap();
        let verified = out.cells.iter().filter(|c| c.verified).count();
        let service = out.cells.iter().filter(|c| c.service_verified).count();
        assert!(verified > 0);
        assert!(out
            .cells
            .iter()
            .filter(|c| !c.verified || !c.service_verified)
            .all(|c| !c.diagnostic.is_empty()));
        assert!(out
            .cells
            .iter()
            .filter(|c| c.verified)
            .all(|c| c.areas.iter().all(|a| a.is_finite() && *a > 0.)));
        let by_id = input
            .groups
            .iter()
            .map(|group| (group.id, group))
            .collect::<BTreeMap<_, _>>();
        let shell = app
            .analysis
            .as_ref()
            .unwrap()
            .cases
            .first()
            .and_then(|c| c.shell.as_ref());
        let details = out.errors.iter().filter_map(|(id, _)| {
            let group = by_id.get(id)?;
            let display = group.displays[0];
            let wall = group.wall.and_then(|w| shell?.surface_forces[w.surface].support);
            Some(serde_json::json!({"id": id, "wall": wall, "group": group, "cases": input.cases.iter().map(|(id, forces)| (id, forces[display])).collect::<Vec<_>>() }))
        }).collect::<Vec<_>>();
        let report = serde_json::json!({"diagnostic_inputs":details,"ndm_stats":progress.stage_stats.iter().map(|s|*s.lock().unwrap()).collect::<Vec<_>>(),"unchanged":progress.unchanged.load(Ordering::Relaxed),"corrected":progress.corrected.load(Ordering::Relaxed),"stage_ms":progress.elapsed_ms.iter().map(|v|v.load(Ordering::Relaxed)).collect::<Vec<_>>(),"physical_elements":out.physical_count,"display_cells":out.cells.len(),"strength_verified":verified,
            "service_verified":service,"elapsed_s":start.elapsed().as_secs_f64(),"diagnostics":out.errors,
            "as_max":(0..4).map(|i|out.cells.iter().filter(|c|c.verified).map(|c|c.areas[i]).fold(0.,f64::max)).collect::<Vec<_>>(),
            "crack_short_max_mm":out.cells.iter().filter(|c|c.service_verified).map(|c|c.crack_short).fold(0.,f64::max)});
        std::fs::write(
            "target/slab_reinforcement_project_report.json",
            serde_json::to_vec_pretty(&report).unwrap(),
        )
        .unwrap();
        println!("{report}");
    }
}
impl State {
    fn stop_worker(&mut self) {
        #[cfg(target_arch = "wasm32")]
        {
            self.worker = None;
        }
    }
}
impl Default for State {
    fn default() -> Self {
        Self {
            #[cfg(target_arch = "wasm32")]
            worker: None,
            result: None,
            key: None,
            job: None,
            progress: Arc::new(rc::Progress::default()),
            cancel: Arc::new(AtomicBool::new(false)),
            total: 0,
            error: String::new(),
        }
    }
}
impl Drop for State {
    fn drop(&mut self) {
        self.cancel.store(true, Ordering::Relaxed);
    }
}

pub(super) fn property_controls(ui: &mut egui::Ui, id: u64, settings: &mut rc::Settings, h: f64) {
    ui.push_id(("plate_reinforcement_settings", id), |ui| {
        ui.separator();
        ui.horizontal_wrapped(|ui| {
            ui.vertical(|ui| {
                ui.label("Бетон");
                egui::ComboBox::from_id_salt("concrete_class")
                    .selected_text(rc::CONCRETE.get(settings.concrete).map_or("—", |v| v.0))
                    .show_ui(ui, |ui| {
                        for (index, row) in rc::CONCRETE.iter().enumerate() {
                            ui.selectable_value(&mut settings.concrete, index, row.0);
                        }
                    });
            });
            ui.vertical(|ui| {
                ui.label("Арматура");
                egui::ComboBox::from_id_salt("steel_class")
                    .selected_text(rc::STEEL.get(settings.steel).map_or("—", |v| v.0))
                    .show_ui(ui, |ui| {
                        for (index, row) in rc::STEEL.iter().enumerate() {
                            ui.selectable_value(&mut settings.steel, index, row.0);
                        }
                    });
            });
            ui.vertical(|ui| {
                ui.label("min, %");
                ui.add_sized([58.,20.],egui::DragValue::new(&mut settings.minimum_percent).range(0.01..=10.).speed(0.01));
            });
            ui.vertical(|ui| {
                ui.label("max, %").on_hover_text(
                    "Предел на каждое направление в каждом слое, от площади сечения полосы. 0 — предел 200 см²/м.",
                );
                let mut maximum = settings.maximum_percent.unwrap_or(0.);
                if ui.add_sized(
                    [58., 20.],
                    egui::DragValue::new(&mut maximum)
                        .range(0. ..=100.)
                        .speed(0.01)
                        .update_while_editing(false),
                ).changed() {
                    settings.maximum_percent = (maximum > 0.).then_some(maximum);
                }
            });
        });
        ui.horizontal(|ui| {
            for (label, value, hint) in [
                ("asbx", &mut settings.asbx, "Нижняя X"),
                ("asby", &mut settings.asby, "Нижняя Y"),
                ("astx", &mut settings.astx, "Верхняя X"),
                ("asty", &mut settings.asty, "Верхняя Y"),
            ] {
                ui.vertical(|ui| {
                    ui.label(label)
                        .on_hover_text(format!("{hint}: расстояние от грани до оси арматуры, мм"));
                    ui.add_sized(
                        [58., 20.],
                        egui::DragValue::new(value).range(1. ..=5000.).speed(1.),
                    );
                });
            }
        });
        ui.horizontal(|ui| {
            ui.vertical(|ui| {
                ui.label("d, мм").on_hover_text("Принятый диаметр периодической арматуры для расчёта расстояния между трещинами");
                ui.add_sized([70.,20.],egui::DragValue::new(&mut settings.diameter_mm).range(6. ..=40.));
            });
            for (label,value,hint) in [
                ("acrcl, мм",&mut settings.acrcl,"Допустимое продолжительное раскрытие"),
                ("acrcs, мм",&mut settings.acrcsh,"Допустимое непродолжительное раскрытие"),
            ] {
                ui.vertical(|ui| {
                    ui.label(label).on_hover_text(hint);
                    ui.add_sized([70.,20.],egui::DragValue::new(value).range(0.001..=5.).speed(0.01));
                });
            }
        });
        ui.checkbox(&mut settings.wall_height, "Учитывать высоту стены")
            .on_hover_text("Для стен и пилонов учитывает случайный эксцентриситет и продольный изгиб при проверке прочности.");
        if settings.wall_height {
            ui.horizontal_wrapped(|ui| {
                ui.label("Коэф. длины μ");
                ui.add(egui::DragValue::new(&mut settings.wall_length_factor).range(0.01..=10.).speed(0.01).update_while_editing(false));
                ui.checkbox(&mut settings.wall_length_from_analysis, "Брать из расчёта")
                    .on_hover_text("Использует μ из расчёта длин из плоскости стены, если он найден. Иначе используется введённое значение.");
            });
            ui.horizontal_wrapped(|ui| {
                let mut manual = settings.wall_height_mm.is_some();
                if ui.checkbox(&mut manual, "Задать высоту, мм").changed() {
                    settings.wall_height_mm = manual.then_some(3000.);
                }
                if let Some(height) = &mut settings.wall_height_mm {
                    ui.add(egui::DragValue::new(height).range(100. ..=100000.).speed(10.).update_while_editing(false));
                } else {
                    ui.weak("По геометрии стены");
                }
            });
        }
        ui.small("Привязки — мм. Eb для НДМ по классу бетона; E выше — для МКЭ.");
        if let Err(error) = settings.validate_source().and_then(|_| settings.validate(h)) {
            ui.colored_label(Color32::LIGHT_RED, error);
        }
    });
}
impl App {
    pub(super) fn reinforcement_busy(&self) -> bool {
        self.reinforcement.job.is_some()
    }

    fn reinforcement_key(&self) -> Option<Vec<u8>> {
        let analysis = self.analysis.as_ref()?;
        let mut library = self.stiffnesses.snapshot(&self.contours);
        library.material_colors.clear();
        serde_json::to_vec(&(
            self.mesh_generation,
            self.reinforcement_method,
            analysis.cases.as_ptr() as usize,
            &self.load_cases,
            self.combinations
                .iter()
                .map(|c| (c.id, &c.coefficients))
                .collect::<Vec<_>>(),
            library,
        ))
        .ok()
    }
    pub(super) fn poll_reinforcement(&mut self, ctx: &egui::Context) {
        let current = self.reinforcement_key();
        if self.reinforcement.key.is_some() && self.reinforcement.key != current {
            self.reinforcement.cancel.store(true, Ordering::Relaxed);
            self.reinforcement.stop_worker();
            self.reinforcement.job = None;
            self.reinforcement.result = None;
            self.reinforcement.key = None;
            self.reinforcement.error.clear();
            self.result_generation = self.result_generation.wrapping_add(1);
            self.invalidate_scene_buffers();
        }
        let message = self.reinforcement.job.as_ref().map(|job| job.try_recv());
        match message {
            Some(Ok(result)) => {
                self.reinforcement.stop_worker();
                self.reinforcement.job = None;
                match result {
                    Ok(output) => {
                        let walls = output.wall_displays.len();
                        let walls_verified = output
                            .wall_displays
                            .values()
                            .filter(|&&display| output.cells[display].verified)
                            .count();
                        self.status = format!(
                            "Армирование: {} физических КЭ, замечаний {}",
                            output.physical_count,
                            output.errors.len()
                        );
                        if walls > 0 {
                            self.status.push_str(&format!(
                                " · КЭ стен с подтверждённой прочностью {walls_verified}/{walls}"
                            ));
                        }
                        self.reinforcement.result = Some(Arc::new(output));
                        self.reinforcement.error.clear();
                        self.set_result_view(ctx, 26, true);
                        self.result_generation = self.result_generation.wrapping_add(1);
                        self.variant_scale_pending[26] = true;
                        self.invalidate_scene_buffers();
                    }
                    Err(error) => self.reinforcement.error = error,
                }
            }
            Some(Err(mpsc::TryRecvError::Disconnected)) => {
                self.reinforcement.stop_worker();
                self.reinforcement.job = None;
                self.reinforcement.error = "Поток подбора завершился без результата".into();
            }
            _ => {}
        }
        if self.reinforcement.result.is_none() && (26..=32).contains(&self.result_field) {
            self.result_field = 0;
        }
    }
    fn reinforcement_input(&self) -> Result<rc::Input, String> {
        if self
            .analysis_input
            .as_ref()
            .is_none_or(|key| *key != self.analysis_key())
        {
            return Err("Исходные данные МКЭ изменены — повторите расчёт всех загружений".into());
        }

        let mesh = self.fe_mesh.as_ref().ok_or("Постройте КЭ-сетку")?;
        let analysis = self
            .analysis
            .as_ref()
            .ok_or("Сначала рассчитайте все загружения")?;
        let indices = mesh
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect::<BTreeMap<_, _>>();
        let mut owned = std::collections::BTreeSet::new();
        let mut groups = vec![];
        let settings_at = |index: usize| -> Result<(f64, rc::Settings), String> {
            if let Some(building) = &mesh.building {
                let id = building.element_slabs[index];
                let property = self
                    .stiffnesses
                    .resolved_entry_for(id, self.stiffness_variant)
                    .filter(|entry| entry.family.is_plate())
                    .ok_or("Плите не назначена жёсткость")?;
                let material = self
                    .materials
                    .get(&id)
                    .ok_or("Нет активной жёсткости плиты")?;
                return Ok((material.thickness_mm, property.reinforcement));
            }
            let triangle = mesh.elements[index].nodes.map(|n| mesh.nodes[n].xy);
            let center =
                std::array::from_fn(|axis| triangle.iter().map(|p| p[axis]).sum::<f64>() / 3.);
            let mut owners = vec![];
            for contour in self.contours.iter().filter(|c| {
                c.kind == Kind::Slab
                    && mesh.contour_floor(c.id) == mesh.element_floor(index)
                    && crate::plan_trace::geometry::inside(center, &c.points)
            }) {
                if self.contours.iter().any(|hole| {
                    hole.active_hole()
                        && hole.parent == Some(contour.id)
                        && crate::plan_trace::geometry::inside(center, &hole.points)
                }) {
                    continue;
                }
                let property = self
                    .stiffnesses
                    .resolved_entry_for(contour.id, self.stiffness_variant)
                    .ok_or("Плите не назначена жёсткость")?;
                if !property.family.is_plate() {
                    return Err("Неверный тип жёсткости плиты".into());
                }
                owners.push((
                    self.materials
                        .get(&contour.id)
                        .ok_or("Нет активной жёсткости плиты")?
                        .thickness_mm,
                    property.reinforcement,
                ));
            }
            let first = owners
                .first()
                .copied()
                .ok_or_else(|| format!("КЭ {} вне плит", mesh.elements[index].id))?;
            if owners.iter().any(|owner| *owner != first) {
                return Err(format!(
                    "КЭ {}: пересечение плит с разными настройками армирования",
                    mesh.elements[index].id
                ));
            }
            Ok(first)
        };
        for quad in &mesh.quads {
            let displays = quad
                .display_triangles
                .iter()
                .map(|id| indices.get(id).copied().ok_or("Не найдена тесселяция Q4"))
                .collect::<Result<Vec<_>, _>>()?;
            let (h, settings) = settings_at(displays[0])?;
            for &display in &displays {
                if settings_at(display)? != (h, settings) {
                    return Err("Q4 пересекает границу параметров армирования".into());
                }
                owned.insert(display);
            }
            groups.push(rc::Group {
                id: quad.id,
                displays,
                h,
                settings,
                wall: None,
            });
        }
        for (index, element) in mesh
            .elements
            .iter()
            .enumerate()
            .filter(|(i, _)| !owned.contains(i))
        {
            let (h, settings) = settings_at(index)?;
            groups.push(rc::Group {
                id: element.id,
                displays: vec![index],
                h,
                settings,
                wall: None,
            });
        }
        let shell = analysis.cases.first().and_then(|case| case.shell.as_ref());
        let walls = shell.map_or_else(Vec::new, |shell| {
            shell
                .surface_forces
                .iter()
                .enumerate()
                .filter(|(_, force)| force.support.is_some())
                .collect::<Vec<_>>()
        });
        let design_count = mesh.elements.len();
        let display_count = design_count + walls.len();
        if !walls.is_empty() {
            let sources = self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Support)
                .map(|c| (c.id, c))
                .collect::<BTreeMap<_, _>>();
            let lengths = analysis
                .lengths
                .as_ref()
                .and_then(|result| result.as_ref().ok());
            let by_wall = lengths
                .into_iter()
                .flatten()
                .filter(|row| row.axis == "Из плоскости")
                .fold(BTreeMap::<u64, Vec<_>>::new(), |mut rows, row| {
                    rows.entry(row.id).or_default().push(row);
                    rows
                });
            let scene = if !by_wall.is_empty() {
                Some(crate::plan_trace::fem::shell::scene(
                    mesh,
                    &vec![crate::plan_trace::fem::element::Material::default(); design_count],
                    &self.support_height,
                    self.mesh_built_size,
                    &self.contours,
                )?)
            } else {
                None
            };
            let max_plate_id = mesh.elements.iter().map(|e| e.id).max().unwrap_or(0);
            for (position, &(surface, force)) in walls.iter().enumerate() {
                let wall_id = force.support.unwrap();
                let contour = sources
                    .get(&wall_id)
                    .ok_or("КЭ стены без исходного объекта")?;
                let entry = self
                    .stiffnesses
                    .resolved_entry_for(wall_id, self.stiffness_variant)
                    .filter(|entry| entry.family.is_plate())
                    .ok_or_else(|| format!("Стене {wall_id} не назначена жёсткость"))?;
                let settings = entry.reinforcement;
                let extent = self.source_heights(contour);
                let manual_height_m = settings.wall_height_mm.map(|height| height / 1000.);
                let calculated = if settings.wall_height && settings.wall_length_from_analysis {
                    scene
                        .as_ref()
                        .zip(by_wall.get(&wall_id))
                        .and_then(|(scene, rows)| {
                            let p: [f64; 3] = std::array::from_fn(|a| {
                                force.nodes.iter().map(|&n| scene.xyz[n][a]).sum::<f64>()
                                    / force.nodes.len() as f64
                            });
                            rows.iter()
                                .filter(|row| {
                                    let z = p[2] / 1000.;
                                    z >= row.bottom_m - 1e-6
                                        && z <= row.bottom_m + row.length_m + 1e-6
                                        && row.segment.is_none_or(|segment| {
                                            crate::plan_trace::meshing::on_segment(
                                                [p[0], p[1]],
                                                segment,
                                            )
                                        })
                                })
                                .filter_map(|row| {
                                    let effective = row.effective_m?;
                                    let ratio = effective / row.length_m;
                                    let height = manual_height_m.unwrap_or(row.length_m);
                                    let length = manual_height_m.map_or(effective, |h| h * ratio);
                                    (height.is_finite()
                                        && height > 0.
                                        && length.is_finite()
                                        && length > 0.)
                                        .then_some((height, length))
                                })
                                .max_by(|a, b| a.1.total_cmp(&b.1))
                        })
                } else {
                    None
                };
                let (height_m, effective_m) = calculated.unwrap_or_else(|| {
                    let height = manual_height_m.unwrap_or((extent[1] - extent[0]) / 1000.);
                    (height, height * settings.wall_length_factor)
                });
                groups.push(rc::Group {
                    id: max_plate_id + surface as u64 + 1,
                    displays: vec![design_count + position],
                    h: force.material.thickness_mm,
                    settings,
                    wall: Some(rc::Wall {
                        surface,
                        height_m,
                        effective_m,
                    }),
                });
            }
        }
        let forces_for =
            |case: &crate::plan_trace::fem::CaseResult| -> Result<Vec<rc::Forces>, String> {
                let mut values = Vec::with_capacity(display_count);
                for (i, &m) in case.moments.iter().enumerate() {
                    values.push(rc::Forces {
                        m,
                        n: case.shell.as_ref().map_or([0.; 3], |s| s.membrane[i]),
                    });
                }
                for &(index, expected) in &walls {
                    let force = case
                        .shell
                        .as_ref()
                        .and_then(|s| s.surface_forces.get(index))
                        .filter(|force| {
                            force.support == expected.support && force.nodes == expected.nodes
                        })
                        .ok_or("Результаты стен относятся к разным сеткам")?;
                    values.push(rc::Forces {
                        m: force.moments,
                        n: force.membrane,
                    });
                }
                Ok(values)
            };
        let cases = analysis
            .cases
            .iter()
            .map(|case| Ok((case.id, forces_for(case)?)))
            .collect::<Result<_, String>>()?;
        let nonlinear_offset = analysis
            .nonlinear
            .as_ref()
            .map(|n| forces_for(&n.offset))
            .transpose()?;
        Ok(rc::Input {
            method: self.reinforcement_method,
            groups,
            cases,
            loads: self.load_cases.clone(),
            combinations: self.combinations.clone(),
            display_count,
            nonlinear_offset,
        })
    }
    pub(super) fn reinforcement_method_controls(&mut self, ui: &mut egui::Ui) {
        ui.horizontal(|ui| {
            ui.label("Подбор арматуры")
                .on_hover_text("Метод подбора арматуры плит. Для стен и пилонов используется НДМ.");
            ui.add_enabled_ui(self.reinforcement.job.is_none(), |ui| {
                egui::ComboBox::from_id_salt("reinforcement_method")
                    .width(75.)
                    .selected_text(if self.reinforcement_method == rc::Method::Ml {
                        "ML"
                    } else {
                        "НДМ"
                    })
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut self.reinforcement_method, rc::Method::Ml, "ML")
                            .on_hover_text("Прогноз ML, проверка и коррекция НДМ");
                        ui.selectable_value(&mut self.reinforcement_method, rc::Method::Ndm, "НДМ")
                            .on_hover_text("Прямой подбор НДМ с бисекцией, без ML");
                    })
                    .response
                    .on_hover_text(
                        "НДМ — прямой подбор. ML — прогноз с проверкой и коррекцией НДМ.",
                    );
            });
        });
    }
    pub(super) fn reinforcement_controls(&mut self, ui: &mut egui::Ui) {
        let enabled = self.analysis.is_some()
            && self.analysis_job.is_none()
            && self.mesh_job.is_none()
            && self.reinforcement.job.is_none();
        if ui.add_enabled(enabled,egui::Button::new("Расчет армирования")).on_hover_text("ML с проверкой по НДМ, Капра–Мори через 10°. Расчётные и расчётные длительные (Rb × 0.9); трещины по нормативным и нормативным длительным").clicked() {
            match self.reinforcement_input() {
                Ok(input)=>{
                    self.reinforcement.cancel.store(true,Ordering::Relaxed);
                    let cancel=Arc::new(AtomicBool::new(false));let progress=Arc::new(rc::Progress::default());
                    self.reinforcement.total=input.groups.len();self.reinforcement.progress=progress.clone();self.reinforcement.cancel=cancel.clone();
                    self.reinforcement.result=None;self.reinforcement.error.clear();self.reinforcement.key=self.reinforcement_key();
                    let(tx,rx)=mpsc::channel();self.reinforcement.job=Some(rx);let ctx=ui.ctx().clone();
                    #[cfg(not(target_arch="wasm32"))]
                    crate::plan_trace::platform::spawn(move||{
                        let result=std::panic::catch_unwind(std::panic::AssertUnwindSafe(||rc::run(input,progress,cancel))).unwrap_or_else(|_|Err("Ошибка ядра подбора".into()));
                        let _=tx.send(result);ctx.request_repaint();
                    });
                    #[cfg(target_arch="wasm32")]
                    match crate::plan_trace::web_worker::Handle::start(input,progress,tx,ctx){
                        Ok(worker)=>self.reinforcement.worker=Some(worker),
                        Err(error)=>{self.reinforcement.job=None;self.reinforcement.error=error;}
                    }

                }
                Err(error)=>self.reinforcement.error=error,
            }
        }
    }
    pub(super) fn reinforcement_progress(&mut self, ui: &mut egui::Ui) {
        if self.reinforcement.job.is_some()
            || (self.reinforcement.result.is_some() && self.reinforcement.total > 0)
        {
            let total = self.reinforcement.total.max(1);
            let progress = &self.reinforcement.progress;
            let active = progress.stage.load(Ordering::Relaxed);
            for stage in 0..3 {
                if self.reinforcement_method == rc::Method::Ndm && stage == 1 {
                    continue;
                }
                let done = progress.done[stage].load(Ordering::Relaxed);
                let seconds = progress.elapsed_ms[stage].load(Ordering::Relaxed) as f64 / 1000.;
                let suffix = if active < stage {
                    " · ожидание"
                } else if active > stage {
                    " · готово"
                } else {
                    ""
                };
                ui.label(if self.reinforcement_method == rc::Method::Ndm {
                    if stage == 0 {
                        "НДМ: подбор площадей"
                    } else if stage == 1 {
                        "Прочность подтверждена при подборе"
                    } else {
                        rc::STAGES[stage]
                    }
                } else {
                    if stage == 2 {
                        "ML: трещины · контроль НДМ"
                    } else {
                        rc::STAGES[stage]
                    }
                });
                if stage == 2 && active == 2 {
                    ui.small(match progress.crack_phase.load(Ordering::Relaxed) {
                        2 => "Вычисление ширины раскрытия трещин",
                        3 => "Корректировка армирования",
                        _ => "Проверка образования трещин",
                    });
                }
                if stage == 1 {
                    ui.small(format!(
                        "Без коррекции: {} · скорректировано: {}",
                        progress.unchanged.load(Ordering::Relaxed),
                        progress.corrected.load(Ordering::Relaxed)
                    ));
                }
                ui.add(
                    egui::ProgressBar::new(done as f32 / total as f32)
                        .text(format!("{done}/{total} КЭ · {seconds:.1} с{suffix}")),
                );
            }
        }
        if self.reinforcement.job.is_some() {
            if ui.button("Отменить подбор").clicked() {
                self.reinforcement.cancel.store(true, Ordering::Relaxed);
                #[cfg(target_arch = "wasm32")]
                {
                    self.reinforcement.stop_worker();
                    self.reinforcement.job = None;
                    self.reinforcement.error = "Подбор отменён".into();
                }
            }
            ui.ctx().request_repaint_after(Duration::from_millis(100));
        }
        if !self.reinforcement.error.is_empty() {
            ui.colored_label(Color32::LIGHT_RED, &self.reinforcement.error);
        }
    }
    pub(super) fn prepare_reinforcement_smoke(&mut self) {
        let input = self
            .reinforcement_input()
            .expect("reinforcement smoke input");
        let total = input.groups.len();
        let progress = Arc::new(rc::Progress::default());
        let output = rc::run(input, progress.clone(), Arc::new(AtomicBool::new(false)))
            .expect("reinforcement smoke design");
        let timing = serde_json::json!({"ndm_stats":progress.stage_stats.iter().map(|s|*s.lock().unwrap()).collect::<Vec<_>>(),"unchanged":progress.unchanged.load(Ordering::Relaxed),"corrected":progress.corrected.load(Ordering::Relaxed),"physical_elements":total,"stage_ms":progress.elapsed_ms.iter().map(|v|v.load(Ordering::Relaxed)).collect::<Vec<_>>(),"diagnostics":output.errors.len()});
        std::fs::write(
            "target/slab_reinforcement_smoke_timing.json",
            serde_json::to_vec_pretty(&timing).unwrap(),
        )
        .expect("write smoke timing");
        self.reinforcement.total = total;
        self.reinforcement.progress = progress;
        if self.spatial.enabled {
            assert!(output
                .wall_displays
                .values()
                .any(|&i| output.cells[i].verified));
            assert!(output
                .errors
                .iter()
                .all(|(_, diagnostic)| !diagnostic.is_empty()));
        } else {
            assert!(
                output.errors.is_empty(),
                "reinforcement smoke: {:?}",
                output.errors
            );
        }
        self.reinforcement.key = self.reinforcement_key();
        self.reinforcement.result = Some(Arc::new(output));
        self.reinforcement_directions = true;
    }
    pub(super) fn reinforcement_results(&mut self, ui: &mut egui::Ui) {
        let Some(output) = self.reinforcement.result.clone() else {
            return;
        };
        egui::CollapsingHeader::new("Армирование")
            .default_open(false)
            .show(ui, |ui| {
                ui.horizontal_wrapped(|ui| {
                    for (field, label) in [(26, "As1"), (27, "As2"), (28, "As3"), (29, "As4")] {
                        self.result_button(ui, label, field, None);
                    }
                    let previous=self.reinforcement_area_view;
                    egui::ComboBox::from_id_salt("reinforcement_area_view").width(90.).selected_text(self.reinforcement_area_view.label()).show_ui(ui,|ui| {
                        for mode in [rc::AreaView::Total,rc::AreaView::Strength,rc::AreaView::CrackIncrement] {ui.selectable_value(&mut self.reinforcement_area_view,mode,mode.label());}
                    }).response.on_hover_text("Полная — итог; Прочность — до проверки трещин; Трещин — добавленная площадь");
                    if previous!=self.reinforcement_area_view {self.result_generation=self.result_generation.wrapping_add(1);self.invalidate_scene_buffers();}
                });
                ui.checkbox(&mut self.reinforcement_directions, "Определяющее направление");
                ui.small("As1 — нижняя X; As2 — верхняя X; As3 — нижняя Y; As4 — верхняя Y. см²/м");
                ui.horizontal_wrapped(|ui| {
                    self.result_button(ui, "a crc кратк.", 30, None);
                    self.result_button(ui, "a crc длит.", 31, None);
                    self.result_button(ui, "Образование трещин", 32, None);
                });
                ui.small("Огибающая всех сочетаний. Диаметр для трещин задан в параметрах конструирования.");
                if !output.wall_displays.is_empty() {
                    let verified = output.wall_displays.values().filter(|&&i| output.cells[i].verified).count();
                    ui.small(format!("КЭ стен с подтверждённой прочностью {verified}/{}", output.wall_displays.len()));
                    if !self.spatial.enabled && ui.button("Показать армирование стен в 3D").clicked() {
                        self.spatial.enabled = true;
                        let field = if (26..=32).contains(&self.result_field) { self.result_field } else { 26 };
                        self.set_result_view(ui.ctx(), field, true);
                    }
                    ui.small("X вдоль стены, Y вверх. Слои As по сторонам −Z и +Z.");
                }
                if !output.errors.is_empty() {
                    ui.colored_label(
                        Color32::LIGHT_RED,
                        format!("Непроверенных участков: {}", output.errors.len()),
                    );
                    egui::CollapsingHeader::new("Замечания подбора").show(ui, |ui| {
                        egui::ScrollArea::vertical()
                            .id_salt("reinforcement_diagnostics")
                            .max_height(200.)
                            .show(ui, |ui| {
                                for (id, message) in &output.errors {
                                    ui.add(egui::Label::new(format!("КЭ {id}: {message}")).wrap());
                                }
                            });
                    });
                }
            });
    }
    pub(super) fn paint_reinforcement_errors(&self, painter: &egui::Painter, rect: Rect) {
        if !(26..=32).contains(&self.result_field) || !self.show_results {
            return;
        }
        let (Some(mesh), Some(output)) = (&self.fe_mesh, &self.reinforcement.result) else {
            return;
        };
        for (element, cell) in mesh.elements.iter().zip(&output.cells) {
            let invalid = if self.result_field <= 29 {
                !cell.verified
            } else {
                !cell.service_verified
            };
            if invalid {
                let points = element
                    .nodes
                    .map(|n| self.camera.screen(mesh.nodes[n].xy, rect));
                painter.add(egui::Shape::convex_polygon(
                    points.to_vec(),
                    Color32::from_rgba_unmultiplied(240, 40, 40, 130),
                    Stroke::NONE,
                ));
            }
        }
    }
}
impl App {
    pub(super) fn paint_reinforcement_directions(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: impl Fn([f64; 3]) -> Pos2,
    ) {
        let (Some(mesh), Some(result)) = (&self.fe_mesh, &self.reinforcement.result) else {
            return;
        };
        let mut quad_secondary = std::collections::BTreeSet::new();
        let mut quad_primary = BTreeMap::new();
        for q in &mesh.quads {
            quad_secondary.insert(q.display_triangles[1]);
            quad_primary.insert(q.display_triangles[0], q.nodes.as_slice());
        }
        let mut labels = std::collections::BTreeSet::new();
        let mut paint_direction =
            |center: [f64; 3], direction: [f64; 3], radius: f64, angle: f64| {
                let at = project(center);
                if !rect.contains(at) {
                    return;
                }
                let segment = [-1., 1.].map(|sign| {
                    project(std::array::from_fn(|i| {
                        center[i] + sign * radius * direction[i]
                    }))
                });
                painter.line_segment(segment, egui::Stroke::new(2.0_f32, Color32::YELLOW));
                if self.show_result_values
                    && labels.insert(((at.x / 55.) as i32, (at.y / 20.) as i32))
                {
                    painter.text(
                        at,
                        egui::Align2::LEFT_BOTTOM,
                        format!("{:.1}°", angle.to_degrees().rem_euclid(180.)),
                        egui::FontId::proportional(12.),
                        Color32::YELLOW,
                    );
                }
            };
        for (i, (element, cell)) in mesh.elements.iter().zip(&result.cells).enumerate() {
            if !cell.verified
                || quad_secondary.contains(&element.id)
                || !self.mesh_element_on_floor(i)
            {
                continue;
            }
            let Some(angle) = cell.governing_angle else {
                continue;
            };
            if let Some(b) = &mesh.building {
                let Some(slab) = self.contours.iter().find(|c| c.id == b.element_slabs[i]) else {
                    continue;
                };
                if !self.contour_visible(slab) {
                    continue;
                }
            }
            let nodes = quad_primary
                .get(&element.id)
                .copied()
                .unwrap_or(&element.nodes);
            let mut center = [0.; 3];
            for &node in nodes {
                let [x, y] = mesh.nodes[node].xy;
                let z = mesh.building.as_ref().map_or(0., |b| b.z[node]);
                for (v, a) in center.iter_mut().zip([x, y, z]) {
                    *v += a / nodes.len() as f64;
                }
            }
            let radius = nodes
                .iter()
                .map(|&n| {
                    let p = mesh.nodes[n].xy;
                    (p[0] - center[0]).hypot(p[1] - center[1])
                })
                .fold(f64::INFINITY, f64::min)
                * 0.6;
            let (sin, cos) = angle.sin_cos();
            paint_direction(center, [cos, sin, 0.], radius, angle);
        }
    }
}
