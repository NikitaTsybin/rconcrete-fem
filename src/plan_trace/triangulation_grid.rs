//! One contour stores the entire rectilinear triangulation network.
use super::*;

pub(super) struct Grid {
    pub active: bool,
    dx: f64,
    nx: usize,
    dy: f64,
    ny: usize,
}
impl Default for Grid {
    fn default() -> Self {
        Self {
            active: false,
            dx: 1000.,
            nx: 5,
            dy: 1000.,
            ny: 5,
        }
    }
}
impl Grid {
    pub(super) fn spec(&self, origin: Point) -> model::GridSpec {
        model::GridSpec {
            origin,
            ex: [self.dx, 0.],
            ey: [0., self.dy],
            nx: self.nx,
            ny: self.ny,
        }
    }
    pub fn points(&self, origin: Point) -> Result<Vec<Point>, String> {
        self.spec(origin).points()
    }
}
impl App {
    pub(super) fn triangulation_grid_controls(&mut self, ui: &mut egui::Ui) {
        let before = self.triangulation_grid.active;
        ui.horizontal(|ui| {
            ui.selectable_value(&mut self.triangulation_grid.active, false, "Линии");
            ui.selectable_value(&mut self.triangulation_grid.active, true, "Сетка");
        });
        if before != self.triangulation_grid.active {
            self.draft.clear();
            self.draft_segments.clear();
            self.curve_controls.clear();
            self.sketch_history.clear();
            self.draft_line_closed = false;
            self.tool = Tool::Line;
            self.reset_dynamic();
        }
        if self.triangulation_grid.active {
            ui.horizontal(|ui| {
                ui.vertical(|ui| {
                    ui.label("dx, мм");
                    ui.add(
                        egui::DragValue::new(&mut self.triangulation_grid.dx)
                            .range(1.0..=1_000_000.)
                            .speed(10.),
                    );
                });
                ui.vertical(|ui| {
                    ui.label("nx");
                    ui.add(egui::DragValue::new(&mut self.triangulation_grid.nx).range(1..=500));
                });
                ui.vertical(|ui| {
                    ui.label("dy, мм");
                    ui.add(
                        egui::DragValue::new(&mut self.triangulation_grid.dy)
                            .range(1.0..=1_000_000.)
                            .speed(10.),
                    );
                });
                ui.vertical(|ui| {
                    ui.label("ny");
                    ui.add(egui::DragValue::new(&mut self.triangulation_grid.ny).range(1..=500));
                });
            });
            ui.small("nx, ny — число ячеек. Укажите нижний левый угол сетки; Esc — отмена.");
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn grid_is_one_saved_selected_contour_and_one_undo() {
        let mut app = App::default();
        app.begin(Kind::Triangulation);
        app.triangulation_grid.active = true;
        app.sketch_click([100., 200.], false);
        assert_eq!(app.contours.len(), 1);
        let contour = app.contours[0].clone();
        assert_eq!(app.selected, Some(contour.id));
        assert_eq!(contour.kind, Kind::Triangulation);
        let json = serde_json::to_vec(&contour).unwrap();
        assert_eq!(serde_json::from_slice::<Contour>(&json).unwrap(), contour);
        app.undo();
        assert!(app.contours.is_empty());
    }
    #[test]
    fn single_grid_path_contains_only_axis_lines_with_correct_extents() {
        for nx in [1, 2, 3] {
            for ny in [1, 2, 3] {
                let grid = Grid {
                    active: true,
                    dx: 1200.,
                    nx,
                    dy: 800.,
                    ny,
                };
                let p = grid.points([50., 70.]).unwrap();
                assert_eq!(p[0], [50., 70.]);
                assert!(p
                    .windows(2)
                    .all(|p| p[0] != p[1] && (p[0][0] == p[1][0] || p[0][1] == p[1][1])));
                assert_eq!(
                    p.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
                    50. + nx as f64 * 1200.
                );
                assert_eq!(
                    p.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
                    70. + ny as f64 * 800.
                );
            }
        }
    }
}

pub(super) struct Editor {
    id: u64,
    grid: model::GridSpec,
    values: [String; 4],
    dirty: bool,
}
impl Editor {
    fn new(id: u64, grid: model::GridSpec) -> Self {
        let values = [
            grid.dx().to_string(),
            grid.dy().to_string(),
            grid.nx.to_string(),
            grid.ny.to_string(),
        ];
        Self {
            id,
            grid,
            values,
            dirty: false,
        }
    }
    fn apply(&self) -> Result<model::GridSpec, String> {
        let parse = |i: usize| {
            self.values[i]
                .trim()
                .replace(',', ".")
                .parse::<f64>()
                .map_err(|_| "Введите положительное число".to_owned())
        };
        let dx = parse(0)?;
        let dy = parse(1)?;
        let nx = parse(2)?;
        let ny = parse(3)?;
        if dx <= 0.
            || dy <= 0.
            || !dx.is_finite()
            || !dy.is_finite()
            || nx.fract() != 0.
            || ny.fract() != 0.
            || nx < 1.
            || ny < 1.
            || nx > 500.
            || ny > 500.
        {
            return Err("dx/dy должны быть положительными; nx/ny — целые от 1 до 500".into());
        }
        let mut grid = self.grid.clone();
        for a in 0..2 {
            grid.ex[a] *= dx / self.grid.dx();
            grid.ey[a] *= dy / self.grid.dy();
        }
        grid.nx = nx as usize;
        grid.ny = ny as usize;
        grid.points()?;
        Ok(grid)
    }
}
impl App {
    pub(super) fn triangulation_grid_editor(&mut self, ui: &mut egui::Ui, rect: Rect) {
        if self.mode.is_some() || self.drag.is_some() || self.scaling_op.is_some() {
            self.grid_editor = None;
            return;
        }
        let Some(contour) = self
            .selected
            .and_then(|id| {
                self.contours
                    .iter()
                    .find(|c| c.id == id && c.kind == Kind::Triangulation && self.selectable(c))
            })
            .cloned()
        else {
            self.grid_editor = None;
            return;
        };
        let Some(grid) = contour
            .grid
            .clone()
            .or_else(|| model::GridSpec::infer(&contour.points))
        else {
            self.grid_editor = None;
            return;
        };
        let mut editor = self
            .grid_editor
            .take()
            .filter(|e| e.id == contour.id && e.grid == grid)
            .unwrap_or_else(|| Editor::new(contour.id, grid.clone()));
        let screen = |x: f64, y: f64| self.camera.screen(grid.at(x, y), rect);
        let origins = [
            screen(0.5, 0.) + egui::vec2(-40., 10.),
            screen(0., 0.5) + egui::vec2(-95., -12.),
            screen(grid.nx as f64, 0.) + egui::vec2(-38., 42.),
            screen(grid.nx as f64, grid.ny as f64) + egui::vec2(12., -12.),
        ];
        let painter = ui.painter().with_clip_rect(rect);
        for ends in [
            [screen(0., 0.), screen(1., 0.)],
            [screen(0., 0.), screen(0., 1.)],
        ] {
            painter.line_segment(
                ends,
                Stroke::new(1_f32, self.contour_color(Kind::Triangulation)),
            );
            for end in ends {
                painter.circle_filled(end, 2.5, self.contour_color(Kind::Triangulation));
            }
        }
        let enabled = !self.is_kind_locked(Kind::Triangulation);
        let mut commit = false;
        let mut cancel = false;
        for (i, label) in ["dx, мм", "dy, мм", "nx", "ny"].into_iter().enumerate() {
            let size = egui::vec2(110., 32.);
            let at = Pos2::new(
                origins[i]
                    .x
                    .clamp(rect.left(), (rect.right() - size.x).max(rect.left())),
                origins[i]
                    .y
                    .clamp(rect.top(), (rect.bottom() - size.y).max(rect.top())),
            );
            egui::Area::new(egui::Id::new(("triangulation_grid_field", contour.id, i)))
                .order(egui::Order::Foreground)
                .fixed_pos(at)
                .show(ui.ctx(), |ui| {
                    egui::Frame::popup(ui.style()).show(ui, |ui| {
                        ui.horizontal(|ui| {
                            ui.label(label);
                            let response = ui.add_enabled(
                                enabled,
                                egui::TextEdit::singleline(&mut editor.values[i])
                                    .desired_width(58.),
                            );
                            if response.changed() {
                                editor.dirty = true;
                            }
                            if response.has_focus() || response.lost_focus() {
                                if ui.input(|input| input.key_pressed(egui::Key::Escape)) {
                                    cancel = true;
                                } else if editor.dirty
                                    && (response.lost_focus()
                                        || ui.input(|input| input.key_pressed(egui::Key::Enter)))
                                {
                                    commit = true;
                                }
                            }
                        });
                    });
                });
        }
        if cancel {
            self.grid_editor = Some(Editor::new(contour.id, grid));
            return;
        }
        if commit && enabled {
            match editor.apply() {
                Ok(grid) => {
                    let mut changed = contour.clone();
                    changed.grid = Some(grid.clone());
                    match changed
                        .rebuild()
                        .and_then(|_| model::validate_candidate(&self.contours, &changed))
                    {
                        Ok(()) => {
                            if changed != contour {
                                self.history.push(self.contours.clone());
                                *self
                                    .contours
                                    .iter_mut()
                                    .find(|c| c.id == contour.id)
                                    .unwrap() = changed;
                                self.clear_mesh();
                                self.slab_cache = None;
                                self.status="Параметры сетки изменены. Выполните триангуляцию. Ctrl+Z — отменить.".into();
                            }
                            editor = Editor::new(contour.id, grid);
                        }
                        Err(error) => self.status = error,
                    }
                }
                Err(error) => self.status = error,
            }
        }
        self.grid_editor = Some(editor);
    }
}

#[cfg(test)]
mod node_snap_tests {
    use super::*;
    #[test]
    fn internal_grid_crossing_is_a_node_before_meshing_and_hidden_grid_is_not_snapped() {
        let mut app = App::default();
        app.camera.zoom = 1.;
        app.construction_grid.enabled = false;
        app.begin(Kind::Triangulation);
        app.triangulation_grid.active = true;
        app.sketch_click([0., 0.], false);
        assert!(app.fe_mesh.is_none());
        let found = app.snap([1002., 997.], None).unwrap();
        assert_eq!(found.kind, snap::Kind::Node);
        assert_eq!(found.point, [1000., 1000.]);
        app.drawing_visibility.triangulation = false;
        assert!(app.snap([1002., 997.], None).is_none());
    }
}
