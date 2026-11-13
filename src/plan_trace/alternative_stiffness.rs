//! Independent source/material variants sharing the same geometric mesh.
use super::*;
#[derive(Default)]
pub(super) struct Cache {
    pub analysis: Option<super::super::fem::Results>,
    pub input: Option<fem_ui::Input>,
    pub reinforcement: reinforcement_ui::State,
}
impl App {
    pub(super) fn switch_stiffness_variant(&mut self, next: usize) {
        if next == self.stiffness_variant {
            return;
        }
        std::mem::swap(&mut self.analysis, &mut self.alternative_cache.analysis);
        std::mem::swap(&mut self.analysis_input, &mut self.alternative_cache.input);
        std::mem::swap(
            &mut self.reinforcement,
            &mut self.alternative_cache.reinforcement,
        );
        self.stiffness_variant = next;
        self.sync_stiffness_sources();
        if self
            .analysis_input
            .as_ref()
            .is_some_and(|key| *key != self.analysis_key())
        {
            self.analysis = None;
            self.analysis_input = None;
            self.reinforcement = Default::default();
        }
        if self.analysis.is_none() {
            self.input_locked = false;
        }
        self.combination_cache = None;
        self.result_generation = self.result_generation.wrapping_add(1);
        self.invalidate_scene_buffers();
    }
    pub(super) fn stiffness_variant_controls(&mut self, ui: &mut egui::Ui) {
        if !self.stiffnesses.alternative_enabled
            || (self.fe_mesh.is_none() && self.analysis.is_none())
        {
            return;
        }
        bottom_selector_wrap(ui, 135.);
        let mut next = self.stiffness_variant;
        ui.add_enabled_ui(
            self.analysis_job.is_none()
                && self.alternative_job.is_none()
                && !self.reinforcement_busy(),
            |ui| {
                egui::ComboBox::from_id_salt("active_stiffness_variant")
                    .width(115.)
                    .selected_text(format!("Материалы {}", next + 1))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut next, 0, "Материалы 1");
                        ui.selectable_value(&mut next, 1, "Материалы 2");
                    });
            },
        );
        self.switch_stiffness_variant(next);
    }
    pub(super) fn start_variant_analyses(&mut self, ctx: &egui::Context) {
        if self.analysis_job.is_some() || self.alternative_job.is_some() {
            return;
        }
        if self.building.enabled
            && !crate::plan_trace::foundation::fields(&self.contours, self.gravity)
                .is_ok_and(|f| !f.is_empty() && self.contours.iter().any(|c| c.use_foundation))
            && !self
                .contours
                .iter()
                .any(|c| c.kind.is_restraint() && c.restraint_mask != 0)
            && !(self.support_height.automatic_building_bottom
                && self.support_height.building_bottom_mask != 0)
            && !(self.support_height.automatic_building_top
                && self.support_height.building_top_mask != 0)
        {
            ctx.data_mut(|d| d.insert_temp(egui::Id::new("automatic_restraint_prompt"), [0_u8; 3]));
            return;
        }
        if let Err(error) = self.nonlinear_request() {
            self.status = error;
            return;
        }
        if let Err(error) = self.buckling_request() {
            self.status = error;
            return;
        }
        let cancel = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        self.solver_cancel = Some(cancel.clone());
        self.switch_stiffness_variant(0);
        self.sync_stiffness_sources();
        let (modal_weights, modal_source) = match self.modal_weights_for_solve() {
            Ok(Some((weights, source))) => (Some(weights), Some(source)),
            Ok(None) => (None, None),
            Err(error) => {
                self.status = error;
                self.solver_cancel = None;
                return;
            }
        };
        let mut first = self.analysis_key();
        let mut second = if self.stiffnesses.alternative_enabled {
            self.stiffness_variant = 1;
            self.sync_stiffness_sources();
            let key = self.analysis_key();
            self.stiffness_variant = 0;
            self.sync_stiffness_sources();
            Some(key)
        } else {
            None
        };
        self.alternative_cache = Cache::default();
        self.reinforcement = Default::default();
        self.alternative_cache.input = second.clone();
        let mesh = self.fe_mesh.as_ref().unwrap().clone();
        let cases = self.load_cases.clone();
        let (tx, rx) = mpsc::channel();
        self.analysis_job = Some(rx);
        self.analysis = None;
        self.modal_mass_source = modal_source;
        self.combination_cache = None;
        self.analysis_input = Some(first.clone());
        first.modal_weights = modal_weights.clone();
        first.effective_lengths = self.calculate_effective_lengths;
        if let Some(second) = &mut second {
            second.modal_weights = modal_weights;
            second.effective_lengths = self.calculate_effective_lengths;
        }
        ctx.data_mut(|d| d.remove::<fem_ui::ModalView>(egui::Id::new("modal_view")));
        let alternative_tx = if second.is_some() {
            let (tx, rx) = mpsc::channel();
            self.alternative_job = Some(rx);
            Some(tx)
        } else {
            self.alternative_job = None;
            None
        };
        self.status = if second.is_some() {
            "Расчёт: жёсткости 1 и 2…"
        } else {
            "Расчёт…"
        }
        .into();
        self.solver_progress.lock().unwrap().clear();
        self.solver_progress
            .lock()
            .unwrap()
            .push(("Запуск расчёта".into(), 0.));
        #[cfg(target_arch = "wasm32")]
        {
            self.solver_worker = None;
            match solver_job::Handle::start(
                mesh,
                first,
                second,
                cases,
                self.solver_progress.clone(),
                tx.clone(),
                alternative_tx.clone(),
                ctx.clone(),
            ) {
                Ok(worker) => self.solver_worker = Some(worker),
                Err(error) => {
                    let _ = tx.send(Err(error.clone()));
                    if let Some(tx) = alternative_tx {
                        let _ = tx.send(Err(error));
                    }
                }
            }
            return;
        }
        #[cfg(not(target_arch = "wasm32"))]
        let ctx = ctx.clone();
        #[cfg(not(target_arch = "wasm32"))]
        let progress = self.solver_progress.clone();
        #[cfg(not(target_arch = "wasm32"))]
        crate::plan_trace::platform::spawn(move || {
            let repaint = ctx.clone();
            crate::plan_trace::fem::progress::install(move |label, seconds| {
                progress.lock().unwrap().push((label, seconds));
                repaint.request_repaint_after(Duration::from_millis(100));
            });
            crate::plan_trace::fem::progress::install_cancel(cancel.clone());
            let solve = |key: &fem_ui::Input| {
                std::panic::catch_unwind(|| fem_ui::solve_snapshot(&mesh, key, &cases))
                    .unwrap_or_else(|_| Err("Ошибка расчётного ядра".into()))
            };
            crate::plan_trace::fem::progress::stage("Материалы 1: подготовка модели");
            let _ = tx.send(solve(&first));
            ctx.request_repaint();
            if let (Some(key), Some(tx)) = (second, alternative_tx) {
                crate::plan_trace::fem::progress::stage("Материалы 2: подготовка модели");
                let _ = tx.send(solve(&key));
                ctx.request_repaint();
            }
            crate::plan_trace::fem::progress::stage(
                if cancel.load(std::sync::atomic::Ordering::Relaxed) {
                    "Расчёт остановлен"
                } else {
                    "Расчёт завершён"
                },
            );
            crate::plan_trace::fem::progress::clear();
        });
    }
    pub(super) fn poll_alternative_analysis(&mut self) {
        if self.fe_mesh.is_none() {
            self.alternative_job = None;
            self.alternative_cache = Default::default();
            return;
        }
        let Some(rx) = &self.alternative_job else {
            return;
        };
        match rx.try_recv() {
            Ok(result) => {
                self.alternative_job = None;
                match result {
                    Ok(r) => {
                        self.alternative_cache.analysis = Some(r);
                        self.status = "Расчёт жёсткостей 1 и 2 завершён".into();
                    }
                    Err(e) => {
                        self.alternative_cache.analysis = None;
                        self.status = format!("Материалы 2: {e}");
                    }
                }
            }
            Err(mpsc::TryRecvError::Disconnected) => {
                self.alternative_job = None;
                self.status = "Расчёт жёсткостей 2 прерван".into();
            }
            Err(mpsc::TryRecvError::Empty) => {}
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn alternate_sources_solve_independently_and_switch_without_recalculation() {
        use crate::plan_trace::loads::{ForceUnits, LoadCase, LoadData, LoadType, Magnitude};
        let mut app = App::default();
        let points = vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]];
        let slab = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
            id: 1,
            parent: None,
            kind: Kind::Slab,
            points: points.clone(),
            curves: vec![],
        };
        let mut support = slab.clone();
        support.id = 2;
        support.kind = Kind::Support;
        let mut area = slab.clone();
        area.id = 3;
        area.kind = Kind::LoadArea;
        area.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: Magnitude {
                value: 1.,
                units: ForceUnits::KiloNewton,
            },
            closed: true,
        });
        app.contours = vec![slab, support, area];
        let mut weight = LoadCase::new(2);
        weight.kind = LoadType::SelfWeight;
        app.load_cases = vec![LoadCase::new(1), weight];
        app.stiffnesses.alternative_enabled = true;
        app.sync_stiffness_sources();
        let id = app.stiffnesses.material_id(1).unwrap();
        let alt = &mut app.stiffnesses.elastic.get_mut(&id).unwrap().alternative;
        alt.e_mpa *= 0.5;
        alt.rho.value *= 2.;
        app.mesh_size = 500.;
        app.mesh_built_size = 500.;
        let mesh = crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap();
        app.fe_mesh = Some(mesh.clone());
        let first = app.analysis_key();
        let r1 = fem_ui::solve_snapshot(&mesh, &first, &app.load_cases).unwrap();
        app.stiffness_variant = 1;
        app.sync_stiffness_sources();
        let second = app.analysis_key();
        let r2 = fem_ui::solve_snapshot(&mesh, &second, &app.load_cases).unwrap();
        let max = |r: &crate::plan_trace::fem::Results| {
            r.cases[0].w_mm.iter().copied().fold(0., f64::max)
        };
        assert!((max(&r2) / max(&r1) - 2.).abs() < 1e-5);
        assert!((r2.cases[1].applied_kn / r1.cases[1].applied_kn - 2.).abs() < 1e-6);
        app.stiffness_variant = 0;
        app.sync_stiffness_sources();
        let v1 = max(&r1);
        let v2 = max(&r2);
        app.analysis = Some(r1);
        app.analysis_input = Some(first);
        app.alternative_cache.analysis = Some(r2);
        app.alternative_cache.input = Some(second);
        app.switch_stiffness_variant(1);
        assert_eq!(max(app.analysis.as_ref().unwrap()), v2);
        app.switch_stiffness_variant(0);
        assert_eq!(max(app.analysis.as_ref().unwrap()), v1);
        let bytes = serde_json::to_vec(&app.stiffnesses).unwrap();
        let restored: crate::plan_trace::stiffness::Library =
            serde_json::from_slice(&bytes).unwrap();
        assert_eq!(restored, app.stiffnesses);
    }
}
