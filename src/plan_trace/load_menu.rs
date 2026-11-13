use super::*;

impl App {
    pub(super) fn load_context_hit(&self, p: Point, zoom: f64) -> Option<u64> {
        self.contours
            .iter()
            .rev()
            .find(|c| c.kind.is_load() && self.hit_contour(c, p, zoom))
            .map(|c| c.id)
    }
    pub(super) fn move_load_to_case(&mut self, id: u64, target: u64) {
        if !self.contour_editable(id) {
            return;
        }
        let Some(destination) = self
            .load_cases
            .iter()
            .find(|c| c.id == target)
            .map(|c| c.display_name())
        else {
            return;
        };
        let Some(index) = self
            .contours
            .iter()
            .position(|c| c.id == id && c.kind.is_load())
        else {
            return;
        };
        let Some(load) = self.contours[index].load.as_ref() else {
            return;
        };
        if load.case_id == target {
            return;
        }
        self.history.push(self.contours.clone());
        self.contours[index].load.as_mut().unwrap().case_id = target;
        self.analysis = None;
        self.analysis_input = None;
        self.analysis_job = None;
        self.combination_cache = None;
        self.load_menu_target = None;
        if !self.contour_visible(&self.contours[index]) {
            self.selected = None;
        }
        self.status =
            format!("Нагрузка {id} перенесена в «{destination}». Выполните расчёт заново.");
    }
    pub(super) fn load_transfer_menu(&mut self, ui: &mut egui::Ui) {
        let Some(id) = self.load_menu_target else {
            return;
        };
        let Some(source) = self
            .contours
            .iter()
            .find(|c| c.id == id && self.contour_visible(c))
            .and_then(|c| c.load.as_ref())
            .map(|l| l.case_id)
        else {
            return;
        };
        let destinations = self
            .load_cases
            .iter()
            .filter(|c| c.id != source)
            .map(|c| (c.id, c.display_name()))
            .collect::<Vec<_>>();
        if destinations.is_empty() {
            return;
        }
        let enabled = self.contour_editable(id);
        ui.add_enabled_ui(enabled, |ui| {
            ui.menu_button("Перенести в", |ui| {
                for (target, name) in destinations {
                    if ui.button(name).clicked() {
                        self.move_load_to_case(id, target);
                        ui.close_menu();
                    }
                }
            });
        })
        .response
        .on_disabled_hover_text("Снимите блокировку исходных данных и нагрузок");
        ui.separator();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn sample() -> App {
        let mut a = App::default();
        a.load_cases
            .push(crate::plan_trace::loads::LoadCase::new(2));
        a.begin(Kind::LoadPoint);
        a.sketch_click([1000., 1000.], false);
        a.begin(Kind::LoadLine);
        a.sketch_click([0., 0.], false);
        a.sketch_click([1000., 0.], false);
        a.finish();
        a.begin(Kind::LoadArea);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([2000., 0.], false);
        a.sketch_click([3000., 1000.], false);
        a
    }
    #[test]
    fn transfer_changes_only_case_is_one_undo_and_keeps_mesh() {
        for id in [1, 2, 3] {
            let mut a = sample();
            let before = a.contours.clone();
            let history = a.history.len();
            a.fe_mesh = Some(crate::plan_trace::meshing::Mesh {
                building: None,
                quads: vec![],
                constrained_nodes: Default::default(),
                constrained_edges: Default::default(),
                nodes: vec![],
                elements: vec![],
                edges: vec![],
                support_boundary: BTreeMap::new(),
            });
            a.analysis = Some(crate::plan_trace::fem::Results {
                nonlinear: None,
                lengths: None,
                modal: None,
                buckling: None,
                cases: vec![],
                free_dofs: 0,
            });
            a.move_load_to_case(id, 2);
            let mut expected = before.clone();
            expected
                .iter_mut()
                .find(|c| c.id == id)
                .unwrap()
                .load
                .as_mut()
                .unwrap()
                .case_id = 2;
            assert_eq!(a.contours, expected);
            assert_eq!(a.history.len(), history + 1);
            assert!(a.fe_mesh.is_some());
            assert!(a.analysis.is_none());
            assert_eq!(a.active_load, 1);
            a.undo();
            assert_eq!(a.contours, before);
        }
    }
    #[test]
    fn transfer_respects_locks_and_context_hits_only_visible_loads() {
        let mut a = sample();
        let before = a.contours.clone();
        let history = a.history.len();
        a.input_locked = true;
        a.move_load_to_case(1, 2);
        assert_eq!(a.contours, before);
        a.input_locked = false;
        a.loads_locked = true;
        a.move_load_to_case(1, 2);
        assert_eq!(a.contours, before);
        a.loads_locked = false;
        a.move_load_to_case(1, 99);
        a.move_load_to_case(1, 1);
        assert_eq!(a.history.len(), history);
        assert_eq!(a.load_context_hit([1000., 1000.], 1.), Some(1));
        assert_eq!(a.load_context_hit([500., 0.], 1.), Some(2));
        assert_eq!(a.load_context_hit([2500., 500.], 1.), Some(3));
        for (index, position) in [(4, [1000., 1000.]), (5, [500., 0.]), (6, [2500., 500.])] {
            a.selection_filter[index] = false;
            assert!(a.load_context_hit(position, 1.).is_none());
            a.selection_filter[index] = true;
        }
        a.active_load = 2;
        assert!(a.load_context_hit([2500., 500.], 1.).is_none());
        a.active_load = 1;
        a.show_loads = false;
        assert!(a.load_context_hit([1000., 1000.], 1.).is_none());
    }
}
