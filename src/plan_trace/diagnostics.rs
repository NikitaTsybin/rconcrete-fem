//! Locate a reported source contour / FE without changing the calculation model.
use super::*;
#[path = "scene_diagnostics.rs"]
mod scan;
pub(super) struct State {
    open: bool,
    issues: Vec<scan::Issue>,
    rx: Option<mpsc::Receiver<Result<Vec<scan::Issue>, String>>>,
    input: Vec<Contour>,
    heights: BTreeMap<u64, Vec<f64>>,
    message: String,
    gap_mm: f64,
}
impl Default for State {
    fn default() -> Self {
        Self {
            open: false,
            issues: vec![],
            rx: None,
            input: vec![],
            heights: BTreeMap::new(),
            message: String::new(),
            gap_mm: 20.,
        }
    }
}
fn number_after(text: &str, marker: &str) -> Option<u64> {
    text.match_indices(marker).find_map(|(i, _)| {
        text[i + marker.len()..]
            .chars()
            .take_while(|c| c.is_ascii_digit())
            .collect::<String>()
            .parse()
            .ok()
    })
}
fn coordinates(text: &str, z: f64) -> Vec<[f64; 3]> {
    text.split("X=")
        .skip(1)
        .filter_map(|part| {
            let (x, rest) = part.split_once(',')?;
            let rest = rest.split_once("Y=")?.1;
            let y = rest
                .chars()
                .take_while(|c| c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E'))
                .collect::<String>();
            let actual_z = rest
                .split_once("Z=")
                .and_then(|(_, value)| {
                    value
                        .chars()
                        .take_while(|c| {
                            c.is_ascii_digit() || matches!(c, '.' | '-' | '+' | 'e' | 'E')
                        })
                        .collect::<String>()
                        .parse::<f64>()
                        .ok()
                })
                .unwrap_or(z);
            Some([x.trim().parse().ok()?, y.parse().ok()?, actual_z])
        })
        .collect()
}
impl App {
    fn warning_target(&self) -> Option<(Vec<[f64; 3]>, Option<u64>)> {
        if self.status.contains("Совпадающие стержни здания:") {
            let points = coordinates(&self.status, 0.);
            if !points.is_empty() {
                let floor = number_after(&self.status, "объект ")
                    .or_else(|| number_after(&self.status, "контур "))
                    .filter(|id| self.contours.iter().any(|c| c.id == *id))
                    .map(|id| self.building.owner(id));
                return Some((points, floor));
            }
        }
        if let Some(id) = ["Объект ", "объект ", "АЖТ ", "Контур ", "контур "]
            .iter()
            .find_map(|prefix| number_after(&self.status, prefix))
        {
            if let Some(c) = self.contours.iter().find(|c| c.id == id) {
                let z = self.source_heights(c)[0];
                return Some((
                    c.points.iter().map(|p| [p[0], p[1], z]).collect(),
                    Some(self.building.owner(id)),
                ));
            }
        }
        if let Some(id) = number_after(&self.status, "КЭ ") {
            if let Some(mesh) = &self.fe_mesh {
                if let Some((i, e)) = mesh.elements.iter().enumerate().find(|(_, e)| e.id == id) {
                    let points = e
                        .nodes
                        .iter()
                        .map(|&n| [mesh.nodes[n].xy[0], mesh.nodes[n].xy[1], mesh.node_z(n)])
                        .collect();
                    let floor = mesh
                        .building
                        .as_ref()
                        .and_then(|d| d.element_slabs.get(i))
                        .map(|id| self.building.owner(*id));
                    return Some((points, floor));
                }
            }
        }
        let z = if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        };
        let points = coordinates(&self.status, z);
        (!points.is_empty()).then_some((points, None))
    }
    pub(super) fn open_diagnostics(&mut self, ctx: &egui::Context) {
        self.diagnostics.open = true;
        self.start_diagnostics(ctx);
    }
    fn start_diagnostics(&mut self, ctx: &egui::Context) {
        if self.diagnostics.rx.is_some() {
            return;
        }
        let input = self.contours.clone();
        let heights: BTreeMap<u64, Vec<f64>> = input
            .iter()
            .map(|c| (c.id, self.source_heights(c)))
            .collect();
        let step = self.mesh_size;
        let gap = self.diagnostics.gap_mm;
        let (tx, rx) = mpsc::channel();
        let ctx = ctx.clone();
        self.diagnostics.input = input.clone();
        self.diagnostics.heights = heights.clone();
        self.diagnostics.rx = Some(rx);
        self.diagnostics.message = "Проверка всей схемы…".into();
        crate::plan_trace::platform::spawn(move || {
            let result = std::panic::catch_unwind(|| {
                let mut issues = scan::find(&input, &heights, step)?;
                issues.extend(scan::nearby(&input, &heights, step, gap)?);
                issues.truncate(1000);
                Ok(issues)
            })
            .unwrap_or_else(|_| Err("Ошибка проверки геометрии объектов".into()));
            let _ = tx.send(result);
            ctx.request_repaint();
        });
    }
    pub(super) fn diagnostics_window(&mut self, ctx: &egui::Context) {
        let done = self
            .diagnostics
            .rx
            .as_ref()
            .and_then(|rx| match rx.try_recv() {
                Ok(result) => Some(result),
                Err(mpsc::TryRecvError::Disconnected) => Some(Err("Проверка прервана".into())),
                Err(mpsc::TryRecvError::Empty) => None,
            });
        if let Some(result) = done {
            self.diagnostics.rx = None;
            match result {
                Ok(issues) => {
                    self.diagnostics.message = format!(
                        "Найдено: {}. ПКМ по сообщению — выделить в схеме",
                        issues.len()
                    );
                    self.diagnostics.issues = issues;
                }
                Err(e) => self.diagnostics.message = e,
            }
        }
        if !self.diagnostics.open {
            return;
        }
        let mut open = true;
        let mut refresh = false;
        let mut chosen = None;
        egui::Window::new("Диагностика").open(&mut open).default_size(egui::vec2(680.,420.)).show(ctx,|ui|{
            ui.add_enabled_ui(self.diagnostics.rx.is_none(), |ui| {
                ui.horizontal(|ui| {
                    ui.label("Зазор меньше, мм");
                    ui.add(egui::DragValue::new(&mut self.diagnostics.gap_mm).range(0.0..=10000.).speed(1.));
                });
            });
            ui.horizontal(|ui|{refresh=ui.add_enabled(self.diagnostics.rx.is_none(),egui::Button::new("Проверить заново")).clicked();if self.diagnostics.rx.is_some(){ui.spinner();ctx.request_repaint_after(Duration::from_millis(100));}});
            ui.label(&self.diagnostics.message);
            ui.small("Проверка снимка всей схемы. Намеренные наложения тоже отображаются. Автоматических изменений нет.");
            egui::ScrollArea::vertical().max_height(560.).show(ui,|ui|{
                for issue in &self.diagnostics.issues{
                    let response=ui.add(egui::Label::new(&issue.message).wrap().sense(egui::Sense::click()));
                    response.context_menu(|ui|{
                        if ui.button("Выделить в схеме").clicked(){chosen=Some(issue.clone());ui.close_menu();}
                    });ui.separator();
                }
            });
            if self.diagnostics.issues.len()>=1000{ui.label("Показаны первые 1000 проблем.");}
        });
        self.diagnostics.open = open;
        if refresh {
            self.start_diagnostics(ctx);
        }
        if let Some(issue) = chosen {
            if self.contours != self.diagnostics.input
                || issue.ids.iter().any(|id| {
                    self.contours.iter().find(|c| c.id == *id).is_none_or(|c| {
                        self.diagnostics.heights.get(id) != Some(&self.source_heights(c))
                    })
                })
            {
                self.diagnostics.message = "Схема изменилась. Нажмите «Проверить заново»".into();
                return;
            }
            if self.building.enabled
                && issue
                    .ids
                    .iter()
                    .map(|&id| self.building.owner(id))
                    .collect::<std::collections::BTreeSet<_>>()
                    .len()
                    > 1
            {
                self.spatial.enabled = true;
            }
            if self.active_floor_only() {
                self.toggle_active_floor_only();
            }
            if let Some(&id) = issue.ids.first() {
                if self.building.enabled {
                    self.change_floor(self.building.owner(id));
                }
            }
            for &id in &issue.ids {
                self.view_visibility.contours.remove(&id);
                self.building.hidden_floors.remove(&self.building.owner(id));
                if let Some(c) = self.contours.iter().find(|c| c.id == id) {
                    match c.kind {
                        Kind::Slab => {
                            self.drawing_visibility.contours[0] = true;
                            self.selection_filter[0] = true;
                        }
                        Kind::Support => {
                            self.drawing_visibility.contours[2] = true;
                            self.selection_filter[2] = true;
                            self.drawing_visibility.support_storeys = 2;
                        }
                        Kind::Beam => {
                            self.drawing_visibility.beam = true;
                            self.selection_filter[9] = true;
                        }
                        Kind::Column => {
                            self.drawing_visibility.columns = true;
                            self.columns_selectable = true;
                        }
                        _ => {}
                    }
                }
            }
            self.view_visibility_changed();
            self.selection_all_floors = true;
            self.select_ids(issue.ids, egui::Modifiers::NONE);
            if !issue.points.is_empty() {
                let lo: [f64; 3] = std::array::from_fn(|a| {
                    issue
                        .points
                        .iter()
                        .map(|p| p[a])
                        .fold(f64::INFINITY, f64::min)
                });
                let hi: [f64; 3] = std::array::from_fn(|a| {
                    issue
                        .points
                        .iter()
                        .map(|p| p[a])
                        .fold(f64::NEG_INFINITY, f64::max)
                });
                let center = std::array::from_fn(|a| (lo[a] + hi[a]) * 0.5);
                self.camera.center = [center[0], center[1]];
                self.camera.zoom =
                    (500. / (hi[0] - lo[0]).max(hi[1] - lo[1]).max(500.)).clamp(1e-8, 1000.);
                self.fit = false;
                self.spatial.focus_warning(
                    center,
                    (hi[0] - lo[0])
                        .hypot(hi[1] - lo[1])
                        .max(hi[2] - lo[2])
                        .max(700.),
                );
                self.diagnostic_points = issue.points;
            }
            ctx.request_repaint();
        }
    }
    pub(super) fn status_message(&mut self, ui: &mut egui::Ui) {
        let response = ui.add(egui::Label::new(&self.status).wrap());
        response.context_menu(|ui| {
            let target = self.warning_target();
            if ui
                .add_enabled(target.is_some(), egui::Button::new("Найти в схеме"))
                .clicked()
            {
                if let Some((points, floor)) = target {
                    if let Some(floor) = floor {
                        if self.building.enabled {
                            self.change_floor(floor);
                        }
                    }
                    let lo: [f64; 3] = std::array::from_fn(|a| {
                        points.iter().map(|p| p[a]).fold(f64::INFINITY, f64::min)
                    });
                    let hi: [f64; 3] = std::array::from_fn(|a| {
                        points
                            .iter()
                            .map(|p| p[a])
                            .fold(f64::NEG_INFINITY, f64::max)
                    });
                    let center = std::array::from_fn(|a| 0.5 * (lo[a] + hi[a]));
                    self.camera.center = [center[0], center[1]];
                    self.camera.zoom =
                        (500. / (hi[0] - lo[0]).max(hi[1] - lo[1]).max(500.)).clamp(1e-8, 1000.);
                    self.fit = false;
                    self.spatial.focus_warning(
                        center,
                        (hi[0] - lo[0])
                            .hypot(hi[1] - lo[1])
                            .max(hi[2] - lo[2])
                            .max(700.),
                    );
                    self.diagnostic_points = points;
                    ui.ctx().request_repaint();
                }
                ui.close_menu();
            }
        });
    }
    pub(super) fn paint_warning(
        &self,
        painter: &egui::Painter,
        project: impl Fn([f64; 3]) -> Pos2,
    ) {
        if self.diagnostic_points.is_empty() {
            return;
        }
        let p = self
            .diagnostic_points
            .iter()
            .copied()
            .map(project)
            .collect::<Vec<_>>();
        for edge in p.windows(2) {
            painter.line_segment([edge[0], edge[1]], Stroke::new(3., Color32::RED));
        }
        if p.len() > 2 {
            painter.line_segment([p[p.len() - 1], p[0]], Stroke::new(3., Color32::RED));
        }
        for at in p {
            painter.circle_stroke(at, 7., Stroke::new(2., Color32::YELLOW));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn duplicate_beam_focus_uses_exact_xyz_segment_instead_of_the_whole_contour() {
        let mut a = App::default();
        a.status="Совпадающие стержни здания: балка 4015 (объект 4015, этаж ID 31) и балка 4019 (объект 4019, этаж ID 31). Общий участок: X=77880.000000, Y=60680.000000, Z=38600.000000; X=77880.000000, Y=60670.000000, Z=38600.000000 мм".into();
        let (points, _) = a.warning_target().unwrap();
        assert_eq!(
            points,
            vec![[77880., 60680., 38600.], [77880., 60670., 38600.]]
        );
    }
    #[test]
    fn repeated_node_degenerate_element_remains_locatable_from_message() {
        let mut a = App::default();
        a.status="Проекция нагрузок: Вырожденный КЭ. КЭ 37. Узлы: 28 (X=9350.000000341, Y=16619.999997641); 29 (X=9540.898965423, Y=16619.999999986); 28 (X=9350.000000341, Y=16619.999997641)".into();
        let (p, _) = a.warning_target().unwrap();
        assert_eq!(p.len(), 3);
        assert_eq!(p[0], p[2]);
        assert!((p[1][0] - 9540.898965423).abs() < 1e-8);
    }
}
