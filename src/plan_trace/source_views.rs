//! Source/geometric views. No stiffness assembly, factorization or displacement solve.
use super::*;
pub(super) struct Cached {
    key: ViewKey,
    field: usize,
    generation: u64,
    pub(super) view: crate::plan_trace::fem::CaseResult,
}

#[derive(PartialEq)]
enum ViewKey {
    Geometry(Vec<u8>),
    Pressure(u64, Vec<(PressureCase, f64)>),
}

/// Only self weight changes a projection besides the case's ID and contours.
type PressureCase = (u64, bool);

fn pressure_case(case: &crate::plan_trace::loads::LoadCase) -> PressureCase {
    (
        case.id,
        case.kind == crate::plan_trace::loads::LoadType::SelfWeight,
    )
}

enum CasePressure {
    Dense(Vec<f64>),
    Sparse(Vec<(usize, f64)>),
}
impl CasePressure {
    fn new(values: Vec<f64>) -> Self {
        let nonzero = values.iter().filter(|&&value| value != 0.).count();
        if nonzero <= values.len() / 2 {
            let mut sparse = Vec::with_capacity(nonzero);
            sparse.extend(values.into_iter().enumerate().filter(|(_, v)| *v != 0.));
            Self::Sparse(sparse)
        } else {
            Self::Dense(values)
        }
    }

    fn add_to(&self, sum: &mut [f64], factor: f64) {
        match self {
            Self::Dense(values) => {
                for (sum, q) in sum.iter_mut().zip(values) {
                    *sum += q * factor;
                }
            }
            Self::Sparse(values) => {
                for &(i, q) in values {
                    sum[i] += q * factor;
                }
            }
        }
    }
}

pub(super) struct PressureCache {
    generation: u64,
    size: f64,
    gravity: f64,
    inputs: Vec<Contour>,
    materials: BTreeMap<u64, crate::plan_trace::fem::element::Material>,
    cases: BTreeMap<PressureCase, CasePressure>,
}
impl PressureCache {
    fn inputs(all: &[Contour]) -> impl Iterator<Item = &Contour> {
        all.iter().filter(|c| {
            matches!(
                c.kind,
                Kind::Slab | Kind::Hole | Kind::LoadArea | Kind::LoadHole
            )
        })
    }

    fn new(app: &App) -> Self {
        Self {
            generation: app.mesh_generation,
            size: app.mesh_built_size,
            gravity: app.gravity,
            inputs: Self::inputs(&app.contours).cloned().collect(),
            materials: app.materials.clone(),
            cases: BTreeMap::new(),
        }
    }

    fn matches(&self, app: &App) -> bool {
        self.generation == app.mesh_generation
            && self.size == app.mesh_built_size
            && self.gravity == app.gravity
            && self.materials == app.materials
            && self.inputs.iter().eq(Self::inputs(&app.contours))
    }
}
pub(super) struct FoundationView {
    inputs: Vec<Contour>,
    gravity: f64,
    result: Result<crate::plan_trace::fem::CaseResult, String>,
}
impl FoundationView {
    fn inputs(all: &[Contour]) -> impl Iterator<Item = &Contour> {
        all.iter()
            .filter(|c| matches!(c.kind, Kind::Slab | Kind::Foundation))
    }
    pub(super) fn prepare(
        mesh: &crate::plan_trace::meshing::Mesh,
        all: &[Contour],
        gravity: f64,
    ) -> Option<Self> {
        (all.iter().any(|c| c.kind == Kind::Foundation)
            && all.iter().any(|c| c.kind == Kind::Slab && c.use_foundation))
        .then(|| Self::new(mesh, all, gravity))
    }
    fn new(mesh: &crate::plan_trace::meshing::Mesh, all: &[Contour], gravity: f64) -> Self {
        let inputs = Self::inputs(all).cloned().collect::<Vec<_>>();
        let result = if inputs.iter().any(|c| c.kind == Kind::Foundation)
            && inputs
                .iter()
                .any(|c| c.kind == Kind::Slab && c.use_foundation)
        {
            crate::plan_trace::fem::foundation_coefficients(mesh, &inputs, gravity)
        } else {
            Ok(vec![f64::NAN; mesh.elements.len()])
        };
        Self {
            inputs,
            gravity,
            result: result
                .map(|values| geometric_case(0, mesh.nodes.len(), mesh.elements.len(), values)),
        }
    }
    fn matches(&self, all: &[Contour], gravity: f64) -> bool {
        self.gravity == gravity && self.inputs.iter().eq(Self::inputs(all))
    }
}

// Adapter for the shared drawing API. Structural quantities are not calculated here.
fn geometric_case(
    id: u64,
    nodes: usize,
    elements: usize,
    pressure: Vec<f64>,
) -> crate::plan_trace::fem::CaseResult {
    crate::plan_trace::fem::CaseResult {
        id,
        shell: None,
        punching: Default::default(),
        w_mm: vec![f64::NAN; nodes],
        rotations_rad: vec![],
        moments: vec![[f64::NAN; 3]; elements],
        shears: vec![],
        reaction_kn: vec![],
        loads_kn_m2: pressure,
        foundation_pressure_kn_m2: vec![],
        applied_kn: f64::NAN,
        balance_kn: f64::NAN,
        balance_m_knm: [f64::NAN; 2],
        relative_residual: f64::NAN,
    }
}
impl App {
    pub(super) fn is_source_field(&self) -> bool {
        matches!(self.result_field, 7..=9 | 15 | 16 | 33)
    }
    pub(super) fn source_view_result(&self) -> Option<&crate::plan_trace::fem::CaseResult> {
        if self.fe_mesh.is_none() || !self.is_source_field() {
            return None;
        }
        if self.result_field == 33 {
            return self
                .foundation_view
                .as_ref()
                .filter(|(generation, _)| self.show_results && *generation == self.mesh_generation)
                .and_then(|(_, view)| view.result.as_ref().ok());
        }
        if self.analysis.is_some() {
            return None;
        }
        self.source_view
            .as_ref()
            .filter(|c| c.field == self.result_field && c.generation == self.mesh_generation)
            .map(|c| &c.view)
    }
    pub(super) fn sync_source_view(&mut self) {
        if self.fe_mesh.is_none() || self.analysis.is_some() {
            self.source_pressure = None;
        }
        if self.result_field == 33 && self.show_results {
            self.source_view = None;
            let Some(mesh) = self.fe_mesh.as_ref() else {
                return;
            };
            if self
                .foundation_view
                .as_ref()
                .is_none_or(|(generation, view)| {
                    *generation != self.mesh_generation
                        || !view.matches(&self.contours, self.gravity)
                })
            {
                self.foundation_view = Some((
                    self.mesh_generation,
                    FoundationView::new(mesh, &self.contours, self.gravity),
                ));
                self.result_generation = self.result_generation.wrapping_add(1);
                self.invalidate_scene_buffers();
            }
            if let Some(Err(error)) = self.foundation_view.as_ref().map(|(_, view)| &view.result) {
                self.status = format!("Коэффициент постели: {error}");
            }
            return;
        }
        if (self.analysis.is_some() && self.result_field != 33) || self.fe_mesh.is_none() {
            self.source_view = None;
            return;
        }
        if !self.is_source_field() {
            self.show_results = false;
            self.result_field = 7;
        }
        if !self.show_results {
            self.source_view = None;
            return;
        }
        if self.result_field == 7 {
            self.sync_source_pressure();
            return;
        }
        let mut key = serde_json::to_vec(&(
            self.mesh_generation,
            &self.contours,
            &self.materials,
            &self.support_height,
            &self.load_cases,
            &self.combinations,
            self.active_load,
            self.active_combination,
            self.result_by_combinations,
            self.combination_variant,
            self.gravity,
        ))
        .unwrap_or_default();
        key.extend_from_slice(&(self.result_field as u64).to_le_bytes());
        let key = ViewKey::Geometry(key);
        if self.source_view.as_ref().is_some_and(|c| c.key == key) {
            return;
        }
        let mesh = self.fe_mesh.as_ref().unwrap();
        let n = mesh.nodes.len();
        let e = mesh.elements.len();
        let view = geometric_case(
            if self.result_by_combinations {
                self.active_combination
            } else {
                self.active_load
            },
            n,
            if self.result_field == 16 { e } else { 0 },
            vec![],
        );
        self.publish_source_view(key, view);
    }

    fn sync_source_pressure(&mut self) {
        let weights = self
            .load_cases
            .iter()
            .filter_map(|case| {
                self.displayed_load_factor(case.id)
                    .map(|weight| (pressure_case(case), weight))
            })
            .collect::<Vec<_>>();
        let id = if self.result_by_combinations {
            self.active_combination
        } else {
            self.active_load
        };
        let inputs_match = self
            .source_pressure
            .as_ref()
            .is_some_and(|cache| cache.matches(self));
        let key = ViewKey::Pressure(id, weights);
        if inputs_match
            && self
                .source_view
                .as_ref()
                .is_some_and(|view| view.key == key)
        {
            return;
        }
        if !inputs_match {
            self.source_pressure = Some(PressureCache::new(self));
        }
        let mesh = self.fe_mesh.as_ref().unwrap();
        let cache = self.source_pressure.as_mut().unwrap();
        let ViewKey::Pressure(_, weights) = &key else {
            unreachable!()
        };
        let missing = self
            .load_cases
            .iter()
            .filter(|case| {
                let case = pressure_case(case);
                !cache.cases.contains_key(&case) && weights.iter().any(|&(id, _)| id == case)
            })
            .cloned()
            .collect::<Vec<_>>();
        if !inputs_match || !missing.is_empty() {
            if let Err(error) = crate::plan_trace::fem::source_area_projection_cases(
                mesh,
                &cache.inputs,
                &cache.materials,
                &missing,
                cache.gravity,
                cache.size,
                |case, values| {
                    cache
                        .cases
                        .insert(pressure_case(case), CasePressure::new(values));
                },
            ) {
                self.source_view = None;
                self.source_pressure = None;
                self.status = format!("Проекция нагрузок: {error}");
                return;
            }
        }
        let mut pressure = vec![0.; mesh.elements.len()];
        for &(case, weight) in weights {
            cache.cases[&case].add_to(&mut pressure, weight);
        }
        let view = geometric_case(id, 0, 0, pressure);
        self.publish_source_view(key, view);
    }

    fn publish_source_view(&mut self, key: ViewKey, view: crate::plan_trace::fem::CaseResult) {
        self.source_view = Some(Cached {
            field: self.result_field,
            generation: self.mesh_generation,
            key,
            view,
        });
        self.result_generation = self.result_generation.wrapping_add(1);
        // qA rendering, legend, labels and load visibility already use view_key.
        // A changed pressure value does not change the mesh or its static buffers.
        if self.result_field != 7 {
            self.invalidate_scene_buffers();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn mesh_job_prepares_c1_and_view_switches_reuse_it_until_inputs_change() {
        let mut app = App::default();
        for kind in [Kind::Slab, Kind::Foundation] {
            app.begin(kind);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([0., 0.], false);
            app.sketch_click([2000., 1000.], false);
        }
        app.contours[0].use_foundation = true;
        app.selected = None;
        app.selection_ids.clear();
        app.start_mesh(&egui::Context::default());
        let deadline = std::time::Instant::now() + std::time::Duration::from_secs(10);
        while app.mesh_job.is_some() {
            assert!(std::time::Instant::now() < deadline, "{}", app.status);
            app.poll_mesh();
            std::thread::sleep(std::time::Duration::from_millis(1));
        }
        let prepared = app
            .foundation_view
            .as_ref()
            .unwrap()
            .1
            .result
            .as_ref()
            .unwrap();
        assert!(!prepared.loads_kn_m2.is_empty());
        let original = prepared.loads_kn_m2[0];
        assert!(original.is_finite());
        let allocation = prepared.loads_kn_m2.as_ptr();
        app.result_field = 33;
        for show in [true, false, true] {
            app.show_results = show;
            app.sync_source_view();
            assert_eq!(
                app.selected_result().map(|r| r.loads_kn_m2.as_ptr()),
                show.then_some(allocation)
            );
        }
        let mut doubled = crate::plan_trace::foundation::default_c1();
        doubled.value *= 2.;
        app.contours[1]
            .foundation
            .get_or_insert_with(Default::default)
            .values = vec![doubled; 4];
        app.sync_source_view();
        assert!((app.selected_result().unwrap().loads_kn_m2[0] - 2. * original).abs() < 1e-8);
    }

    #[test]
    fn load_projection_and_geometric_views_do_not_create_analysis() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        app.begin(Kind::Support);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 0.], false);
        app.sketch_click([2000., 1000.], false);
        app.sketch_click([0., 1000.], false);
        app.draft_line_closed = true;
        app.finish();
        app.new_load = crate::plan_trace::loads::Magnitude {
            value: 2.,
            units: crate::plan_trace::loads::ForceUnits::KiloNewton,
        };
        app.begin(Kind::LoadArea);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        app.cancel();
        app.sync_stiffness_sources();
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap());
        app.mesh_built_size = 500.;
        app.mesh_generation += 1;
        app.result_field = 7;
        app.show_results = true;
        app.sync_source_view();
        let before = app.selected_result().unwrap().loads_kn_m2.clone();
        assert!(before.iter().all(|q| (*q - 2.).abs() < 1e-8));
        assert!(app.analysis.is_none() && app.analysis_job.is_none());
        let cached_allocation =
            |app: &App| match &app.source_pressure.as_ref().unwrap().cases[&(1, false)] {
                CasePressure::Dense(values) => values.as_ptr(),
                CasePressure::Sparse(_) => panic!("full-area pressure should be dense"),
            };
        let projected = cached_allocation(&app);
        app.result_field = 8;
        app.sync_source_view();
        assert!(app
            .source_view
            .as_ref()
            .unwrap()
            .view
            .loads_kn_m2
            .is_empty());
        assert_eq!(cached_allocation(&app), projected);
        app.result_field = 7;
        app.sync_source_view();
        assert_eq!(app.selected_result().unwrap().loads_kn_m2, before);
        assert_eq!(cached_allocation(&app), projected);

        let calculated = crate::plan_trace::fem::solve_with_support_height(
            app.fe_mesh.as_ref().unwrap(),
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            500.,
            crate::plan_trace::fem::ElementType::Dkt,
            &app.support_height,
        )
        .unwrap();
        assert_eq!(before, calculated.cases[0].loads_kn_m2);
        app.result_by_combinations = true;
        for coefficient in [3., -2., 0., 1.] {
            app.combinations[0].coefficients.insert(1, coefficient);
            app.sync_source_view();
            assert!(app
                .selected_result()
                .unwrap()
                .loads_kn_m2
                .iter()
                .all(|v| (*v - 2. * coefficient).abs() < 1e-8));
            assert_eq!(cached_allocation(&app), projected);
        }
        app.contours
            .iter_mut()
            .find(|c| c.kind == Kind::LoadArea)
            .unwrap()
            .load
            .as_mut()
            .unwrap()
            .magnitude
            .value = 5.;
        app.sync_source_view();
        assert!(app
            .selected_result()
            .unwrap()
            .loads_kn_m2
            .iter()
            .all(|v| (*v - 5.).abs() < 1e-8));
        app.result_field = 15;
        app.sync_source_view();
        let distances = app.support_distance_data().unwrap();
        assert!(distances.distances.iter().any(|v| *v > 0.));
        app.result_field = 16;
        app.sync_source_view();
        let data = app.support_distance_data().unwrap();
        assert!((data.areas.values().sum::<f64>() - 2.).abs() < 1e-6);
        assert!(app.analysis.is_none());
        app.result_field = 0;
        assert!(app.selected_result().is_none());
    }
    #[test]
    fn switching_source_field_rejects_old_adapter_before_next_frame_sync() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 1000.], false);
        app.cancel();
        app.sync_stiffness_sources();
        app.mesh_built_size = 500.;
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap());
        app.mesh_generation += 1;
        app.show_results = true;
        for field in [33, 7, 16, 8, 7, 16, 33] {
            app.result_field = field;
            // A field switch happens in the sidebar before the renderer in the same frame.
            if field != 33 {
                assert!(app.selected_result().is_none());
            }
            app.sync_source_view();
            let result = app.selected_result().unwrap_or_else(|| {
                panic!("fresh source adapter for field {field}: {}", app.status)
            });
            if field == 7 {
                assert_eq!(
                    result.loads_kn_m2.len(),
                    app.fe_mesh.as_ref().unwrap().elements.len()
                );
            }
            assert!(app.analysis.is_none());
        }
        app.mesh_generation += 1;
        assert!(app.selected_result().is_none());
    }
}
