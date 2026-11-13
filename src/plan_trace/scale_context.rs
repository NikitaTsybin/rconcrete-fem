//! Applied numeric bounds are scoped by the selected result context.
use super::*;
#[derive(Clone, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) struct Key {
    #[serde(default)]
    membrane_area: bool,
    #[serde(default)]
    floor: u64,
    #[serde(default)]
    stiffness: usize,
    #[serde(default)]
    reinforcement_area: u8,
    #[serde(default)]
    summation_method: u8,
    #[serde(default)]
    displacement_axes: u8,
    field: usize,
    combination: bool,
    id: u64,
    variant: u8,
    sum: bool,
    spatial: bool,
    iso: bool,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub(super) struct Saved {
    key: Key,
    bounds: crate::plan_trace::result_scale::Bounds,
}
#[derive(Default)]
pub(super) struct Store {
    pub saved: Vec<Saved>,
    pub current: Option<Key>,
    pub(super) view: Option<[u64; 8]>,
}
impl App {
    pub(super) fn scale_context_key(&self) -> Key {
        Key {
            membrane_area: self.membrane_area && matches!(self.result_field, 12..=14),
            floor: if self.building.enabled && (!self.spatial.enabled || self.active_floor_only()) {
                self.building.active
            } else {
                0
            },
            stiffness: self.stiffness_variant,
            summation_method: if self.punching_view.is_some() {
                self.summation_method as u8
            } else {
                0
            },
            reinforcement_area: if (26..=29).contains(&self.result_field) {
                self.reinforcement_area_view as u8
            } else {
                0
            },
            field: self.result_field,
            displacement_axes: if self.result_field
                == crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD
            {
                self.displacement_axes
            } else {
                0
            },
            combination: self.result_by_combinations,
            id: if self.result_by_combinations {
                self.active_combination
            } else {
                self.active_load
            },
            variant: self.combination_variant as u8,
            sum: self.sum_reactions,
            spatial: self.spatial.enabled,
            iso: if crate::plan_trace::fem::displacement_field(self.result_field) {
                self.iso_displacements
            } else {
                self.iso_efforts
            },
        }
    }
    pub(super) fn scale_is_saved(&self) -> bool {
        let key = self.scale_context_key();
        self.scale_store.saved.iter().any(|s| s.key == key)
    }
    pub(super) fn sync_saved_scale(&mut self) {
        if self.selected_result().is_none() {
            return;
        }
        let key = self.scale_context_key();
        let view = self.view_key();
        let field = self.result_field;
        let changed = self.scale_store.current.as_ref() != Some(&key);
        let pending = self.variant_scale_pending[field] || self.scale_store.view != Some(view);
        if !changed && !pending {
            return;
        }
        self.result_scales[field].set_reinforcement_mode((26..=29).contains(&field));
        if let Some(saved) = self.scale_store.saved.iter().find(|s| s.key == key) {
            if changed {
                let _ = self.result_scales[field].restore_bounds(&saved.bounds);
            }
        } else {
            if matches!(field, 7..=9) {
                self.load_scale_key = None;
                self.sync_load_scale();
            } else {
                let range = if self.spatial.enabled {
                    self.spatial_result_range()
                } else {
                    self.result_range_cached()
                };
                let Some(range) = range else {
                    return;
                };
                let _ = self.result_scales[field].reset_values(range);
            }
        }
        self.variant_scale_pending[field] = false;
        self.scale_store.current = Some(key);
        self.scale_store.view = Some(view);
    }
    pub(super) fn record_scale_action(&mut self) {
        let field = self.result_field;
        if let Some(action) = self.result_scales[field].take_action() {
            self.store_scale_action(action);
        }
    }
    fn store_scale_action(&mut self, action: crate::plan_trace::result_scale::Action) {
        use crate::plan_trace::result_scale::Action;
        let key = self.scale_context_key();
        self.scale_store.saved.retain(|s| s.key != key);
        if let Action::Apply = action {
            if let Some(bounds) = self.result_scales[self.result_field].bounds() {
                self.scale_store.saved.push(Saved { key, bounds });
            }
        }
        self.scale_store.current = None;
        self.sync_saved_scale();
    }
    pub(super) fn rescale_saved_bounds(&mut self, factor: f64) {
        for entry in &mut self.scale_store.saved {
            let i = entry.key.field;
            if (i != 0 && i != 10 && i != 11 && (i < 15 || (17..=22).contains(&i)) && i < 26)
                || matches!(i, 33 | 34)
            {
                if let Some(values) = &mut entry.bounds.discrete {
                    for v in values {
                        *v *= factor;
                    }
                }
                for value in &mut entry.bounds.edges {
                    *value *= factor;
                }
                for label in &mut entry.bounds.labels {
                    if let Ok(v) = label.replace(',', ".").parse::<f64>() {
                        *label = (v * factor).to_string();
                    }
                }
            }
        }
    }
}

impl Saved {
    pub(super) fn validate(&self) -> Result<(), String> {
        if self.key.field >= crate::plan_trace::fem::FIELD_COUNT
            || self.key.variant > 3
            || self.key.displacement_axes > 0b111
        {
            return Err("Некорректный вид сохранённой шкалы".into());
        }
        crate::plan_trace::result_scale::Scale::default().restore_bounds(&self.bounds)
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn moment_view(second_floor: bool) -> App {
        use crate::plan_trace::{fem, fem::shell};
        let mut app = App::default();
        app.building.enabled = true;
        app.building.floors.push(super::super::building::Floor {
            id: 2,
            name: "Второй".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        let mut upper = app.contours[0].clone();
        upper.id = app.next_id;
        if second_floor {
            app.building.contour_floors.insert(upper.id, 2);
        } else {
            for p in &mut upper.points {
                p[0] += 3000.;
            }
        }
        let upper_id = upper.id;
        app.contours.push(upper);
        app.spatial.enabled = true;
        app.result_field = 1;
        app.iso_efforts = false;
        app.load_units = crate::plan_trace::loads::ForceUnits::KiloNewton;
        let mesh = crate::plan_trace::building_mesh::generate(
            &app.contours,
            1000.,
            app.building_layout().unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let scene = shell::scene(
            &mesh,
            &vec![fem::element::Material::default(); mesh.elements.len()],
            &app.support_height,
            1000.,
            &app.contours,
        )
        .unwrap();
        let moments = (0..mesh.elements.len())
            .map(|i| {
                let base = if mesh.building.as_ref().unwrap().element_slabs[i] == upper_id {
                    1000.
                } else {
                    1.
                };
                [base + (i % 2) as f64, 0., 0.]
            })
            .collect();
        app.analysis = Some(fem::Results {
            nonlinear: None,
            modal: None,
            buckling: None,
            lengths: None,
            free_dofs: 1,
            cases: vec![fem::CaseResult {
                punching: Default::default(),
                shell: None,
                id: 1,
                moments,
                w_mm: vec![0.; mesh.nodes.len()],
                rotations_rad: vec![],
                shears: vec![],
                reaction_kn: vec![],
                loads_kn_m2: vec![],
                foundation_pressure_kn_m2: vec![],
                applied_kn: 0.,
                balance_kn: 0.,
                balance_m_knm: [0.; 2],
                relative_residual: 0.,
            }],
        });
        app.fe_mesh = Some(mesh);
        app.spatial.cache = Some((
            app.mesh_generation,
            app.support_height.clone(),
            app.contours.clone(),
            true,
            scene,
        ));
        app
    }

    #[test]
    fn scale_uses_visible_3d_floors_for_shell_values() {
        let mut app = moment_view(true);
        let all_key = app.scale_context_key();
        assert_eq!(app.spatial_result_extrema(), Some([1., 1001.]));
        app.sync_saved_scale();

        app.toggle_active_floor_only();
        assert!(app.scale_context_key() != all_key);
        assert_eq!(app.spatial_result_extrema(), Some([1., 2.]));
        app.sync_saved_scale();
        assert!(app.result_scales[1]
            .bounds()
            .unwrap()
            .edges
            .iter()
            .all(|value| *value <= 2.));

        app.set_floor_visibility_from_menu(1, false);
        app.set_floor_visibility_from_menu(2, true);
        assert_eq!(app.spatial_result_extrema(), Some([1000., 1001.]));
        app.sync_saved_scale();
        assert!(app.result_scales[1]
            .bounds()
            .unwrap()
            .edges
            .iter()
            .all(|value| *value >= 1000.));
    }

    #[test]
    fn scale_switch_and_reset_use_active_case_and_visible_2d_contours() {
        use crate::plan_trace::result_scale::Action;
        let mut app = moment_view(false);
        app.spatial.enabled = false;
        app.show_results = true;
        app.result_by_combinations = true;
        app.combinations[0].coefficients.insert(1, 5.);
        let bounds = |app: &App| {
            let edges = app.result_scales[1].bounds().unwrap().edges;
            [edges[0], *edges.last().unwrap()]
        };
        app.sync_combination_result();
        app.sync_saved_scale();
        assert_eq!(bounds(&app), [5., 5005.]);

        app.result_by_combinations = false;
        app.sync_combination_result();
        app.sync_saved_scale();
        assert_eq!(bounds(&app), [1., 1001.]);

        let visible_slab = app.contours[0].id;
        let hidden_slab = app.contours[1].id;
        app.view_visibility.contours.insert(hidden_slab);
        app.view_visibility_changed();
        assert_eq!(app.result_range_cached(), Some([1., 2.]));
        app.sync_saved_scale();
        assert_eq!(bounds(&app), [1., 2.]);

        app.result_scales[1].reset_values([-100., 100.]).unwrap();
        app.store_scale_action(Action::Apply);
        assert_eq!(bounds(&app), [-100., 100.]);
        app.store_scale_action(Action::Reset);
        assert_eq!(bounds(&app), [1., 2.]);

        app.view_visibility.contours.remove(&hidden_slab);
        app.view_visibility.contours.insert(visible_slab);
        app.view_visibility_changed();
        app.sync_saved_scale();
        assert_eq!(bounds(&app), [1000., 1001.]);
    }

    #[test]
    fn view_scale_keys_separate_case_variant_result_and_projection() {
        let mut app = App::default();
        let a = app.scale_context_key();
        app.active_load += 1;
        assert!(a != app.scale_context_key());
        app.active_load -= 1;
        app.result_field += 1;
        assert!(a != app.scale_context_key());
        app.result_field -= 1;
        app.combination_variant = crate::plan_trace::load_combinations::Variant::Normative;
        assert!(a != app.scale_context_key());
    }
}
