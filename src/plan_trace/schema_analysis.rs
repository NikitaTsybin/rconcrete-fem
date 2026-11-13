//! View-only source diagnostics, available before the structural solve.
use super::*;
#[derive(Clone, Copy, PartialEq, Eq)]
pub(super) enum Mode {
    Thickness,
    Modulus,
    Stiffness,
    Lengths,
    MovementLocks,
    MeshSize,
    Floors,
}
impl Mode {
    fn scale_index(self) -> Option<usize> {
        match self {
            Self::Thickness => Some(0),
            Self::Modulus => Some(1),
            Self::Floors => Some(2),
            Self::MeshSize => Some(3),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Thickness => "Мозаика толщин, мм",
            Self::Modulus => "Мозаика модулей упругости, МПа",
            Self::Stiffness => "Мозаика жёсткостей",
            Self::Lengths => "Вычисление расчетных длин",
            Self::MovementLocks => "Блокировка перемещения",
            Self::MeshSize => "Шаг триангуляции, мм",
            Self::Floors => "Мозаика этажей",
        }
    }
}
#[derive(Default)]
pub(super) struct State {
    pub mode: Option<Mode>,
    values: BTreeMap<u64, f64>,
    range: [f64; 2],
    scales: [crate::plan_trace::result_scale::Scale; 4],
    inks: BTreeMap<u64, Color32>,
    legend: Vec<(String, Color32)>,
    revision: u64,
    mesh: std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<egui::Mesh>)>>,
}
impl App {
    pub(super) fn schema_ink(&self, id: u64) -> Option<Color32> {
        let mode = self.schema_analysis.mode?;
        if let Some(i) = mode.scale_index() {
            return self
                .schema_analysis
                .values
                .get(&id)
                .and_then(|v| self.schema_analysis.scales[i].color(*v, self.schema_range()));
        }
        self.schema_analysis.inks.get(&id).copied()
    }
    fn schema_range(&self) -> [f64; 2] {
        self.schema_analysis.range
    }
    pub(super) fn schema_hover(&self, id: u64) -> Option<String> {
        let mode = self.schema_analysis.mode?;
        let &value = self.schema_analysis.values.get(&id)?;
        if mode == Mode::MovementLocks {
            return Some(format!("Перемещение: {}", movement_status(value != 0.)));
        }
        Some(format!(
            "{}: {}",
            mode.label(),
            self.display_text.number(value)
        ))
    }
    pub(super) fn schema_legend(&self, p: &egui::Painter, r: Rect) {
        if let Some(i) = self.schema_analysis.mode.and_then(Mode::scale_index) {
            self.schema_analysis.scales[i].legend(p, r, self.schema_range(), self.display_text);
        }
    }
    pub(super) fn schema_legend_selection(&self, rect: Rect, pointer: Pos2) -> Option<Vec<u64>> {
        let scale = &self.schema_analysis.scales[self.schema_analysis.mode?.scale_index()?];
        let range = self.schema_range();
        let band = scale.legend_band_at(rect, pointer, range, self.display_text.size)?;
        Some(if scale.band_enabled(band) {
            self.contours
                .iter()
                .filter(|c| self.selectable(c))
                .filter(|c| {
                    self.schema_analysis
                        .values
                        .get(&c.id)
                        .and_then(|&v| scale.value_band(v, range))
                        == Some(band)
                })
                .map(|c| c.id)
                .collect()
        } else {
            vec![]
        })
    }
    pub(super) fn schema_scale_controls(&mut self, ui: &mut egui::Ui) -> bool {
        let Some(i) = self.schema_analysis.mode.and_then(Mode::scale_index) else {
            return false;
        };
        let before = serde_json::to_vec(&self.schema_analysis.scales[i].state()).ok();
        let range = self.schema_range();
        egui::CollapsingHeader::new("Шкала").show(ui, |ui| {
            if self.schema_analysis.mode == Some(Mode::MeshSize) && range[0] == 0. {
                ui.label(format!("0 — проектный размер ({:.0} мм)", self.mesh_size));
            }
            self.schema_analysis.scales[i].ui(ui, range, self.display_text);
        });
        if before != serde_json::to_vec(&self.schema_analysis.scales[i].state()).ok() {
            self.schema_analysis.revision = self.schema_analysis.revision.wrapping_add(1);
            self.schema_analysis.mesh.borrow_mut().take();
            self.slab_cache = None;
            self.invalidate_scene_buffers();
            self.spatial.cache_reset();
        }
        true
    }

    pub(super) fn schema_refresh(&mut self) {
        let Some(mode) = self.schema_analysis.mode else {
            return;
        };
        let mut values = BTreeMap::new();
        let mut range = [f64::INFINITY, f64::NEG_INFINITY];
        let scale_index = mode.scale_index();
        let mut visible_values = Vec::new();
        let active = self.stiffnesses.for_variant(self.stiffness_variant);
        for c in &self.contours {
            if !matches!(mode, Mode::Floors | Mode::MovementLocks)
                && !matches!(
                    c.kind,
                    Kind::Slab | Kind::Support | Kind::Column | Kind::Beam
                )
            {
                continue;
            }
            let material = || active.resolved_entry(c.id).map(|e| e.material);
            let value = match mode {
                Mode::Floors => self
                    .building
                    .floors
                    .iter()
                    .position(|f| f.id == self.building.owner(c.id))
                    .map(|i| (i + 1) as f64),
                Mode::Thickness if matches!(c.kind, Kind::Slab | Kind::Support) => {
                    material().map(|m| m.thickness_mm)
                }
                Mode::Modulus => material().map(|m| m.e_mpa),
                Mode::Stiffness => self.stiffnesses.assignments.get(&c.id).map(|id| *id as f64),
                Mode::Lengths if c.kind == Kind::Support => {
                    Some(c.calculate_effective_length as u8 as f64)
                }
                Mode::Lengths if c.kind == Kind::Column => {
                    Some(c.calculate_effective_length as u8 as f64)
                }
                Mode::MovementLocks => Some(u8::from(c.placement.movement_locked) as f64),
                Mode::MeshSize if matches!(c.kind, Kind::Slab | Kind::Support) => {
                    Some(c.mesh_size_mm)
                }
                _ => None,
            };
            if let Some(v) = value.filter(|v| v.is_finite()) {
                values.insert(c.id, v);
                if self.contour_visible(c) {
                    range[0] = range[0].min(v);
                    range[1] = range[1].max(v);
                    if scale_index.is_some() {
                        visible_values.push(v);
                    }
                }
            }
        }
        self.schema_analysis.range = if range[0].is_finite() {
            range
        } else {
            [0., 1.]
        };
        let scale_changed = scale_index
            .is_some_and(|i| self.schema_analysis.scales[i].sync_discrete_values(visible_values));
        let mut unique = if scale_index.is_none() {
            values.values().copied().collect::<Vec<_>>()
        } else {
            Vec::new()
        };
        unique.sort_by(f64::total_cmp);
        unique.dedup();
        let palette = [
            [80, 160, 245],
            [245, 185, 50],
            [95, 200, 125],
            [195, 110, 230],
            [245, 100, 90],
            [65, 210, 215],
            [170, 180, 75],
            [215, 135, 185],
            [130, 150, 225],
            [230, 160, 95],
            [90, 185, 170],
            [180, 170, 145],
        ];
        let mut legend = vec![];
        let colors = unique
            .iter()
            .enumerate()
            .map(|(i, &v)| {
                let rgb = if matches!(mode, Mode::Lengths | Mode::MovementLocks) {
                    if v == 0. {
                        [110, 115, 125]
                    } else {
                        [255, 160, 35]
                    }
                } else {
                    palette[i % palette.len()]
                };
                let ink = Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
                let label = match mode {
                    Mode::Stiffness => self
                        .stiffnesses
                        .entries
                        .get(&(v as u64))
                        .map(|e| e.name.clone())
                        .unwrap_or_else(|| format!("Жёсткость {v}")),
                    Mode::Lengths => {
                        if v == 0. {
                            "Выключено".into()
                        } else {
                            "Включено".into()
                        }
                    }
                    Mode::MovementLocks => movement_status(v != 0.).into(),
                    _ => self.display_text.number(v),
                };
                legend.push((label, ink));
                (v.to_bits(), ink)
            })
            .collect::<BTreeMap<_, _>>();
        let inks = values
            .iter()
            .filter_map(|(&id, v)| colors.get(&v.to_bits()).map(|&ink| (id, ink)))
            .collect::<BTreeMap<_, _>>();
        if scale_changed
            || self.schema_analysis.values != values
            || self.schema_analysis.inks != inks
            || self.schema_analysis.legend != legend
        {
            self.schema_analysis.inks = inks;
            self.schema_analysis.legend = legend;
            self.schema_analysis.values = values;
            self.schema_analysis.revision = self.schema_analysis.revision.wrapping_add(1);
            self.schema_analysis.mesh.borrow_mut().take();
            self.slab_cache = None;
            self.invalidate_scene_buffers();
            self.spatial.cache_reset();
        }
    }
    pub(super) fn schema_analysis_controls(&mut self, ui: &mut egui::Ui) {
        let previous = self.schema_analysis.mode;
        if self.show_results {
            self.schema_analysis.mode = None;
        }
        ui.collapsing("Анализ схемы", |ui| {
            if ui.button("Диагностика").clicked() {
                self.open_diagnostics(ui.ctx());
            }
            for mode in [
                Mode::Thickness,
                Mode::Modulus,
                Mode::Stiffness,
                Mode::Lengths,
                Mode::MovementLocks,
                Mode::MeshSize,
                Mode::Floors,
            ] {
                if ui
                    .add(
                        egui::Button::new(mode.label())
                            .selected(self.schema_analysis.mode == Some(mode)),
                    )
                    .clicked()
                {
                    self.schema_analysis.mode = if self.schema_analysis.mode == Some(mode) {
                        None
                    } else {
                        Some(mode)
                    };
                }
            }
            if self
                .schema_analysis
                .mode
                .is_some_and(|m| m.scale_index().is_none())
            {
                egui::ScrollArea::vertical()
                    .id_salt("schema_legend")
                    .max_height(180.)
                    .show(ui, |ui| {
                        for (label, ink) in &self.schema_analysis.legend {
                            ui.horizontal(|ui| {
                                let (rect, _) = ui.allocate_exact_size(
                                    egui::vec2(15., 12.),
                                    egui::Sense::hover(),
                                );
                                ui.painter().rect_filled(rect, 1., *ink);
                                ui.label(label);
                            });
                        }
                    });
            }
        });
        if previous != self.schema_analysis.mode {
            self.schema_analysis.revision = self.schema_analysis.revision.wrapping_add(1);
            if self.schema_analysis.mode.is_some() {
                self.show_results = false;
                self.punching_view = None;
                self.length_view = 0;
            }
            self.schema_analysis.mesh.borrow_mut().take();
            self.slab_cache = None;
            self.invalidate_scene_buffers();
            self.spatial.cache_reset();
        }
    }
    pub(super) fn schema_surface_ink(
        &self,
        scene: &crate::plan_trace::fem::shell::Scene,
        s: &crate::plan_trace::fem::shell::Surface,
    ) -> Option<Color32> {
        self.schema_analysis.mode?;
        let owner = s.support.or_else(|| {
            s.display.first().and_then(|i| {
                self.fe_mesh
                    .as_ref()?
                    .building
                    .as_ref()?
                    .element_slabs
                    .get(*i)
                    .copied()
            })
        });
        if let Some(id) = owner {
            return Some(self.schema_ink(id).unwrap_or(Color32::GRAY));
        }
        let p: [f64; 3] = std::array::from_fn(|a| {
            s.nodes.iter().map(|n| scene.xyz[*n][a]).sum::<f64>() / s.nodes.len().max(1) as f64
        });
        self.contours
            .iter()
            .filter(|c| {
                c.kind == Kind::Slab
                    && (self.source_heights(c)[0] - p[2]).abs() < 0.05
                    && geometry::inside([p[0], p[1]], &c.points)
            })
            .min_by(|a, b| {
                polygon_area(&a.points)
                    .abs()
                    .total_cmp(&polygon_area(&b.points).abs())
            })
            .and_then(|c| self.schema_ink(c.id))
            .or(Some(Color32::GRAY))
    }
    pub(super) fn paint_schema_mesh(&self, painter: &egui::Painter, rect: Rect) {
        if self
            .schema_analysis
            .mode
            .is_some_and(|m| m.scale_index().is_some())
        {
            return;
        }
        let Some(mesh) = self.fe_mesh.as_ref() else {
            return;
        };
        let key = vec![
            self.schema_analysis.revision,
            self.mesh_generation,
            self.building.active,
            self.camera.center[0].to_bits(),
            self.camera.center[1].to_bits(),
            self.camera.zoom.to_bits(),
            self.camera.angle.to_bits(),
            rect.left().to_bits() as u64,
            rect.top().to_bits() as u64,
            rect.width().to_bits() as u64,
            rect.height().to_bits() as u64,
            self.spatial.source_revision as u64,
        ];
        let rebuild = self
            .schema_analysis
            .mesh
            .borrow()
            .as_ref()
            .is_none_or(|(old, _)| old != &key);
        if rebuild {
            let mut output = egui::Mesh::default();
            for (i, e) in mesh.elements.iter().enumerate() {
                if !self.mesh_element_on_floor(i) || self.mesh_element_hidden(i) {
                    continue;
                }
                let points = e.nodes.map(|n| mesh.nodes[n].xy);
                let owner = mesh
                    .building
                    .as_ref()
                    .and_then(|b| b.element_slabs.get(i).copied())
                    .or_else(|| {
                        let center = [
                            points.iter().map(|p| p[0]).sum::<f64>() / 3.,
                            points.iter().map(|p| p[1]).sum::<f64>() / 3.,
                        ];
                        self.contours
                            .iter()
                            .filter(|c| c.kind == Kind::Slab && geometry::inside(center, &c.points))
                            .min_by(|a, b| {
                                polygon_area(&a.points)
                                    .abs()
                                    .total_cmp(&polygon_area(&b.points).abs())
                            })
                            .map(|c| c.id)
                    });
                let ink = owner
                    .and_then(|id| self.schema_ink(id))
                    .unwrap_or(Color32::GRAY);
                append_triangle(&mut output, points, self.camera, rect, ink);
            }
            *self.schema_analysis.mesh.borrow_mut() = Some((key, std::sync::Arc::new(output)));
        }
        if let Some((_, mesh)) = &*self.schema_analysis.mesh.borrow() {
            painter.add(egui::Shape::Mesh(mesh.clone()));
        }
    }
}

fn movement_status(locked: bool) -> &'static str {
    if locked {
        "Заблокировано"
    } else {
        "Разрешено"
    }
}

fn polygon_area(p: &[Point]) -> f64 {
    p.iter()
        .zip(p.iter().cycle().skip(1))
        .take(p.len())
        .map(|(a, b)| a[0] * b[1] - a[1] * b[0])
        .sum::<f64>()
        * 0.5
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn movement_analysis_colors_explicit_locks_including_openings() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        let slab = app.contours[0].id;
        let hole = app.next_id;
        app.contours.push(Contour {
            id: hole,
            parent: Some(slab),
            kind: Kind::Hole,
            points: vec![[500., 500.], [1500., 500.], [1500., 1500.], [500., 1500.]],
            ..app.contours[0].clone()
        });
        app.contours[0].placement.movement_locked = true;
        app.schema_analysis.mode = Some(Mode::MovementLocks);
        app.schema_refresh();

        let blocked = Color32::from_rgb(255, 160, 35);
        let allowed = Color32::from_rgb(110, 115, 125);
        assert_eq!(app.object_ink(slab, Kind::Slab), blocked);
        assert_eq!(app.object_ink(hole, Kind::Hole), allowed);
        assert_eq!(
            app.schema_hover(slab).as_deref(),
            Some("Перемещение: Заблокировано")
        );
        assert_eq!(
            app.schema_hover(hole).as_deref(),
            Some("Перемещение: Разрешено")
        );

        app.contours[1].placement.movement_locked = true;
        app.schema_refresh();
        assert_eq!(app.object_ink(hole, Kind::Hole), blocked);
    }

    #[test]
    fn thickness_scale_contains_only_distinct_visible_thicknesses() {
        use crate::plan_trace::{fem::element::Material, stiffness::Family};
        let mut app = App::default();
        for (i, thickness_mm) in [180., 200., 600., 180.].into_iter().enumerate() {
            let x = 2000. * i as f64;
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([x, 0.], false);
            app.sketch_click([x + 1000., 1000.], false);
            app.cancel();
            let material = Material {
                thickness_mm,
                ..Default::default()
            };
            let entry = app.stiffnesses.add(Family::Slab, material);
            app.stiffnesses
                .assignments
                .insert(app.contours[i].id, entry);
        }
        app.schema_analysis.mode = Some(Mode::Thickness);
        app.schema_refresh();
        assert_eq!(
            app.schema_analysis.scales[0]
                .bounds()
                .unwrap()
                .discrete
                .unwrap(),
            [180., 200., 600.]
        );
        app.view_visibility.contours.insert(app.contours[2].id);
        app.schema_refresh();
        assert_eq!(
            app.schema_analysis.scales[0]
                .bounds()
                .unwrap()
                .discrete
                .unwrap(),
            [180., 200.]
        );
    }

    #[test]
    fn modulus_floors_and_mesh_size_share_discrete_visible_values() {
        use crate::plan_trace::{fem::element::Material, stiffness::Family};
        let mut app = App::default();
        for (i, (e_mpa, mesh_size_mm)) in [(23500., 0.), (31850., 350.), (50000., 200.)]
            .into_iter()
            .enumerate()
        {
            let x = 2000. * i as f64;
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([x, 0.], false);
            app.sketch_click([x + 1000., 1000.], false);
            app.cancel();
            let c = &mut app.contours[i];
            c.mesh_size_mm = mesh_size_mm;
            let entry = app.stiffnesses.add(
                Family::Slab,
                Material {
                    e_mpa,
                    ..Default::default()
                },
            );
            app.stiffnesses.assignments.insert(c.id, entry);
            let floor_id = (i + 1) as u64;
            app.building.contour_floors.insert(c.id, floor_id);
            if i != 0 {
                app.building.floors.push(building::Floor {
                    id: floor_id,
                    name: format!("Этаж {floor_id}"),
                    elevation_mm: 3000. * i as f64,
                    height_mm: 3000.,
                });
            }
        }
        app.building.enabled = true;
        app.spatial.enabled = true;
        app.building.hidden_floors.insert(3);
        for (mode, expected) in [
            (Mode::Modulus, [23500., 31850.]),
            (Mode::Floors, [1., 2.]),
            (Mode::MeshSize, [0., 350.]),
        ] {
            app.schema_analysis.mode = Some(mode);
            app.schema_refresh();
            assert_eq!(
                app.schema_analysis.scales[mode.scale_index().unwrap()]
                    .bounds()
                    .unwrap()
                    .discrete
                    .unwrap(),
                expected
            );
        }
    }
}
