use super::*;
use std::collections::BTreeSet;

pub(super) const KINDS: [Kind; 14] = [
    Kind::Slab,
    Kind::Hole,
    Kind::Support,
    Kind::MeshRegion,
    Kind::LoadPoint,
    Kind::LoadLine,
    Kind::LoadArea,
    Kind::Auxiliary,
    Kind::LoadHole,
    Kind::Beam,
    Kind::Triangulation,
    Kind::Punching,
    Kind::Restraint,
    Kind::RigidLink,
];
pub(super) fn all_enabled() -> [bool; 14] {
    [true; 14]
}

impl App {
    pub(super) fn selectable(&self, c: &Contour) -> bool {
        self.contour_visible(c)
            && (!self.spatial.enabled || self.selection_all_floors || self.on_active_floor(c))
            && if c.kind == Kind::Foundation {
                self.foundation_selectable
            } else if c.kind == Kind::Column {
                self.columns_selectable
            } else {
                let kind = if c.kind == Kind::PointRestraint {
                    Kind::Restraint
                } else {
                    c.kind
                };
                self.selection_filter[KINDS.iter().position(|k| *k == kind).unwrap_or(7)]
            }
    }
    pub(super) fn is_selected(&self, id: u64) -> bool {
        self.selected == Some(id) || self.selection_ids.contains(&id)
    }
    pub(super) fn select_only(&mut self, id: Option<u64>) {
        let id = id.filter(|id| {
            self.contours
                .iter()
                .any(|c| c.id == *id && self.selectable(c))
        });
        self.selected_underlay = None;
        self.underlay_selection.clear();
        self.measurements.selected.clear();
        self.selected = id;
        self.selection_ids.clear();
        self.selection_ids.extend(id);
        self.sync_tree_load_selection();
    }
    pub(super) fn select_ids(
        &mut self,
        ids: impl IntoIterator<Item = u64>,
        modifiers: egui::Modifiers,
    ) {
        self.select_scene(ids, [], modifiers);
    }
    pub(super) fn select_scene(
        &mut self,
        ids: impl IntoIterator<Item = u64>,
        underlays: impl IntoIterator<Item = u64>,
        modifiers: egui::Modifiers,
    ) {
        self.select_all_scene(ids, underlays, [], modifiers);
    }
    pub(super) fn select_all_scene(
        &mut self,
        ids: impl IntoIterator<Item = u64>,
        underlays: impl IntoIterator<Item = u64>,
        measurements: impl IntoIterator<Item = u64>,
        modifiers: egui::Modifiers,
    ) {
        let dimensions = measurements.into_iter().collect::<BTreeSet<_>>();
        let ids = ids.into_iter().collect::<BTreeSet<_>>();
        let underlays = underlays.into_iter().collect::<BTreeSet<_>>();
        if !modifiers.shift && !modifiers.ctrl {
            self.selection_ids.clear();
            self.underlay_selection.clear();
            self.measurements.selected.clear();
        } else {
            self.selection_ids.extend(self.selected);
            self.underlay_selection.extend(self.selected_underlay);
        }
        if modifiers.ctrl {
            self.selection_ids.retain(|id| !ids.contains(id));
            self.underlay_selection.retain(|id| !underlays.contains(id));
            self.measurements
                .selected
                .retain(|id| !dimensions.contains(id));
        } else {
            self.selection_ids.extend(ids);
            self.underlay_selection.extend(underlays);
            self.measurements.selected.extend(dimensions);
        }
        self.selected = self.selection_ids.last().copied();
        self.selected_underlay = self.underlay_selection.last().copied();
        self.sync_tree_load_selection();
    }
    fn sync_tree_load_selection(&mut self) {
        if self.tree_load_selection.is_empty() {
            return;
        }
        let count = self.tree_load_selection.len();
        self.tree_load_selection
            .retain(|id| self.selection_ids.contains(id));
        if self.tree_load_selection.len() != count {
            self.view_visibility.revision = self.view_visibility.revision.wrapping_add(1);
        }
    }
    pub(super) fn select_load_tree(&mut self, ids: Vec<u64>) {
        let Some(first) = ids.first() else {
            return;
        };
        if self.building.enabled {
            self.change_floor(self.building.owner(*first));
        }
        self.cancel();
        let count = ids.len();
        self.tree_load_selection.extend(ids);
        let selected = self
            .contours
            .iter()
            .filter(|c| self.tree_load_selection.contains(&c.id) && self.selectable(c))
            .map(|c| c.id)
            .collect::<Vec<_>>();
        self.status = format!("Выделено нагрузок: {} из {count}", selected.len());
        self.select_ids(selected, egui::Modifiers::NONE);
        self.view_visibility.revision = self.view_visibility.revision.wrapping_add(1);
    }
    pub(super) fn prune_selection(&mut self) {
        if !self.measurements.selected.is_empty() {
            if self.measurements.show && self.measurements.selectable {
                let allowed = self
                    .measurements
                    .items
                    .iter()
                    .filter(|m| {
                        self.measurements.selected.contains(&m.id) && !self.dimension_hidden(m.id)
                    })
                    .map(|m| m.id)
                    .collect::<BTreeSet<_>>();
                self.measurements.selected = allowed;
            } else {
                self.measurements.selected.clear();
            }
        }
        if self.selected_underlay.is_some() || !self.underlay_selection.is_empty() {
            let allowed = self
                .underlays
                .iter()
                .filter(|u| {
                    (self.selected_underlay == Some(u.id)
                        || self.underlay_selection.contains(&u.id))
                        && self.underlay_selectable(u)
                })
                .map(|u| u.id)
                .collect::<BTreeSet<_>>();
            self.underlay_selection.retain(|id| allowed.contains(id));
            if self
                .selected_underlay
                .is_some_and(|id| !allowed.contains(&id))
            {
                self.selected_underlay = self.underlay_selection.last().copied();
            }
        }
        if self.selected.is_some() || !self.selection_ids.is_empty() {
            let mut ids = std::mem::take(&mut self.selection_ids);
            let primary_is_allowed = self
                .selected
                .is_none_or(|id| self.view_contour(id).is_some_and(|c| self.selectable(c)));
            if ids.len() > 1
                && ids
                    .iter()
                    .any(|&id| self.indexed_view_contour(id).is_none())
            {
                // During edits or in 2D the render index can be stale or absent.
                // One scan avoids a separate linear search for each selected ID.
                let allowed = self
                    .contours
                    .iter()
                    .filter(|c| ids.contains(&c.id) && self.selectable(c))
                    .map(|c| c.id)
                    .collect::<BTreeSet<_>>();
                ids.retain(|id| allowed.contains(id));
            } else {
                ids.retain(|&id| {
                    if self.selected == Some(id) {
                        primary_is_allowed
                    } else {
                        self.view_contour(id).is_some_and(|c| self.selectable(c))
                    }
                });
            }
            self.selection_ids = ids;
            if !primary_is_allowed {
                self.selected = self.selection_ids.last().copied();
            }
        }
        self.sync_tree_load_selection();
    }
    pub(super) fn underlay_selectable(&self, u: &crate::plan_trace::underlay::Underlay) -> bool {
        if self.underlay_hidden(u.id)
            || !(self.floor_underlay_visible(u.id)
                || (self.spatial.enabled && self.selection_all_floors))
            || !u.visible
            || u.transparency >= 1.
            || !self.show_dxf
        {
            return false;
        }
        match &u.data {
            crate::plan_trace::underlay::Data::Pdf { .. } => self.underlay_filter[1],
            crate::plan_trace::underlay::Data::Dxf(d) => {
                self.underlay_filter[0]
                    && (d.layers.is_empty()
                        || d.layers.iter().any(|l| {
                            self.layers
                                .get(&format!("#{} · {}", u.id, l.name))
                                .is_none_or(|s| s.visible)
                        }))
            }
        }
    }
    pub(super) fn frame_underlay_hits(&self, frame: Rect, viewport: Rect) -> Vec<u64> {
        self.underlays
            .iter()
            .filter(|u| self.underlay_selectable(u))
            .filter_map(|u| {
                let corners = self.underlay_corners(u.id)?;
                let points = corners.map(|p| self.camera.screen(p, viewport));
                let hit = points.iter().any(|p| frame.contains(*p))
                    || (0..4).any(|i| segment_hits(frame, points[i], points[(i + 1) % 4]))
                    || geometry::inside(self.camera.world(frame.center(), viewport), &corners);
                hit.then_some(u.id)
            })
            .collect()
    }
    pub(super) fn selection_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Фильтр выделения")
            .default_open(false)
            .show(ui, |ui| {
                ui.add_enabled_ui(self.spatial.enabled, |ui| {
                    ui.horizontal(|ui| {
                        ui.label("Этажи (3D)");
                        egui::ComboBox::from_id_salt("selection_floor_scope")
                            .width(170.)
                            .selected_text(if self.selection_all_floors {
                                "Все этажи"
                            } else {
                                "На активном этаже"
                            })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(
                                    &mut self.selection_all_floors,
                                    false,
                                    "На активном этаже",
                                );
                                ui.selectable_value(
                                    &mut self.selection_all_floors,
                                    true,
                                    "Все этажи",
                                );
                            });
                    });
                })
                .response
                .on_disabled_hover_text("Область выделения этажей используется в 3D");
                ui.horizontal(|ui| {
                    let all = ui.button("Все").clicked();
                    let none = ui.button("Ни один").clicked();
                    let value = if all {
                        Some(true)
                    } else if none {
                        Some(false)
                    } else {
                        None
                    };
                    if let Some(value) = value {
                        self.columns_selectable = value;
                        self.foundation_selectable = value;
                        self.selection_filter.fill(value);
                        self.fe_selection.enabled.fill(value);
                        self.underlay_filter.fill(value);
                        self.measurements.selectable = value;
                    }
                });
                for (i, kind) in KINDS.iter().enumerate() {
                    if *kind == Kind::RigidLink && !self.building.enabled {
                        continue;
                    }
                    ui.checkbox(
                        &mut self.selection_filter[i],
                        if *kind == Kind::Restraint {
                            "Опоры"
                        } else {
                            kind.label()
                        },
                    );
                }
                ui.checkbox(&mut self.columns_selectable, "Колонны");
                ui.checkbox(&mut self.foundation_selectable, "Основания");
                ui.checkbox(&mut self.underlay_filter[0], "Подложки DXF");
                ui.checkbox(&mut self.underlay_filter[1], "Подложки PDF");
                if self.fe_mesh.is_some() {
                    ui.checkbox(&mut self.fe_selection.enabled[0], "КЭ оболочек");
                    ui.checkbox(&mut self.fe_selection.enabled[1], "КЭ стержней");
                }
                ui.checkbox(&mut self.measurements.selectable, "Размеры");
            });
    }
    pub(super) fn frame_hits(&self, frame: Rect, viewport: Rect) -> Vec<u64> {
        self.contours
            .iter()
            .filter(|c| self.selectable(c))
            .filter(|c| {
                let p = c
                    .points
                    .iter()
                    .map(|&p| self.camera.screen(p, viewport))
                    .collect::<Vec<_>>();
                if p.iter().any(|p| frame.contains(*p)) {
                    return true;
                }
                let count = if c.closed() {
                    p.len()
                } else {
                    p.len().saturating_sub(1)
                };
                if (0..count).any(|i| segment_hits(frame, p[i], p[(i + 1) % p.len()])) {
                    return true;
                }
                c.closed()
                    && geometry::inside(self.camera.world(frame.center(), viewport), &c.points)
            })
            .map(|c| c.id)
            .collect()
    }
    pub(super) fn frame_measurement_hits(
        &self,
        painter: &egui::Painter,
        frame: Rect,
        viewport: Rect,
    ) -> Vec<u64> {
        if !self.measurements.show || !self.measurements.selectable {
            return vec![];
        }
        self.measurements
            .items
            .iter()
            .filter(|m| !self.dimension_hidden(m.id))
            .filter_map(|m| {
                let l = self.dimension_layout(m, viewport)?;
                let hit = std::iter::once(l.ends)
                    .chain(l.extensions)
                    .filter_map(|s| clip_line(s[0], s[1], viewport))
                    .any(|[a, b]| segment_hits(frame, a, b));
                let label = self.measurement_label_box(painter, m, &l);
                let text_hit = label.iter().any(|p| frame.contains(*p))
                    || (0..4).any(|i| segment_hits(frame, label[i], label[(i + 1) % 4]))
                    || geometry::inside(
                        [frame.center().x as f64, frame.center().y as f64],
                        &label.map(|p| [p.x as f64, p.y as f64]),
                    );
                (hit || text_hit).then_some(m.id)
            })
            .collect()
    }
    pub(super) fn selection_frame(
        &mut self,
        ui: &egui::Ui,
        response: &egui::Response,
        rect: Rect,
        hovered: Option<Pos2>,
    ) -> bool {
        if self.scaling_op.is_some()
            || self.mode.is_some()
            || self.arc_placement.is_some()
            || self.clipboard_op.is_some()
            || self.measurements.active
            || self.measurements.drag.is_some()
            || self.underlay_drag.is_some()
        {
            self.selection_start = None;
            return false;
        }
        if ui.input(|i| i.pointer.primary_pressed()) && self.drag.is_none() {
            self.selection_start = hovered;
            self.selection_modifiers = ui.input(|i| i.modifiers);
        }
        let mut completed = false;
        if let (Some(start), Some(end)) =
            (self.selection_start, ui.input(|i| i.pointer.interact_pos()))
        {
            if response.dragged_by(egui::PointerButton::Primary)
                || response.drag_stopped_by(egui::PointerButton::Primary)
            {
                let end = rect.clamp(end);
                let window = start.x <= end.x;
                let frame = Rect::from_two_pos(start, end);
                let ink = if window {
                    Color32::from_rgb(80, 160, 255)
                } else {
                    Color32::from_rgb(80, 210, 140)
                };
                ui.painter().with_clip_rect(rect).rect(
                    frame,
                    0.,
                    Color32::from_rgba_unmultiplied(ink.r(), ink.g(), ink.b(), 28),
                    Stroke::new(1_f32, ink),
                    egui::StrokeKind::Inside,
                );
                if response.drag_stopped_by(egui::PointerButton::Primary) {
                    self.fe_frame_2d(frame, rect, window, self.selection_modifiers);
                    let mut ids = self.frame_hits(frame, rect);
                    let mut underlays = self.frame_underlay_hits(frame, rect);
                    let mut dimensions = self.frame_measurement_hits(ui.painter(), frame, rect);
                    if window {
                        ids.retain(|id| {
                            self.contours.iter().find(|c| c.id == *id).is_some_and(|c| {
                                !c.points.is_empty()
                                    && c.points
                                        .iter()
                                        .all(|&p| frame.contains(self.camera.screen(p, rect)))
                            })
                        });
                        underlays.retain(|id| {
                            self.underlay_corners(*id).is_some_and(|corners| {
                                corners
                                    .iter()
                                    .all(|&p| frame.contains(self.camera.screen(p, rect)))
                            })
                        });
                        dimensions.retain(|id| {
                            self.measurements
                                .items
                                .iter()
                                .find(|m| m.id == *id)
                                .is_some_and(|m| {
                                    self.dimension_layout(m, rect).is_some_and(|l| {
                                        l.ends
                                            .iter()
                                            .chain(l.extensions.iter().flatten())
                                            .all(|&p| frame.contains(self.camera.screen(p, rect)))
                                            && self
                                                .measurement_label_box(ui.painter(), m, &l)
                                                .iter()
                                                .all(|&p| frame.contains(p))
                                    })
                                })
                        });
                    }
                    self.select_all_scene(ids, underlays, dimensions, self.selection_modifiers);
                    completed = true;
                }
            }
        }
        if ui.input(|i| i.pointer.primary_released()) {
            self.selection_start = None;
        }
        completed
    }
}

// Slab clipping detects a long edge crossing the frame even if both ends are outside.
fn segment_hits(r: Rect, a: Pos2, b: Pos2) -> bool {
    let mut lo: f32 = 0.;
    let mut hi: f32 = 1.;
    for (a, b, min, max) in [(a.x, b.x, r.min.x, r.max.x), (a.y, b.y, r.min.y, r.max.y)] {
        let d = b - a;
        if d.abs() < f32::EPSILON {
            if a < min || a > max {
                return false;
            }
        } else {
            let x = (min - a) / d;
            let y = (max - a) / d;
            lo = lo.max(x.min(y));
            hi = hi.min(x.max(y));
            if lo > hi {
                return false;
            }
        }
    }
    true
}

pub(super) fn read_filter<'de, D: serde::Deserializer<'de>>(d: D) -> Result<[bool; 14], D::Error> {
    let values = <Vec<bool> as serde::Deserialize>::deserialize(d)?;
    if !(7..=14).contains(&values.len()) {
        return Err(serde::de::Error::custom("Некорректный фильтр выделения"));
    }
    let mut result = all_enabled();
    result[..values.len()].copy_from_slice(&values);
    if let Some(&triangulation) = values.get(10).filter(|_| values.len() < 13) {
        result[12] = triangulation;
    }
    Ok(result)
}

#[derive(Default)]
pub(super) struct FeSelection {
    pub enabled: [bool; 2],
    pub items: std::collections::BTreeSet<(u8, usize)>,
    generation: u64,
}
impl FeSelection {
    pub fn refresh(&mut self, generation: u64) {
        if generation != self.generation {
            self.items.clear();
            self.generation = generation;
        }
        self.items
            .retain(|(kind, _)| self.enabled[usize::from(*kind == 1)]);
    }
    pub fn pick(&mut self, hit: Option<(u8, usize)>, modifiers: egui::Modifiers) -> bool {
        let Some(hit) = hit else {
            return false;
        };
        if modifiers.ctrl {
            self.items.remove(&hit);
        } else {
            if !modifiers.shift {
                self.items.clear();
            }
            self.items.insert(hit);
        }
        true
    }
    fn select_many(&mut self, items: Vec<(u8, usize)>, modifiers: egui::Modifiers) {
        if items.is_empty() {
            return;
        }
        if modifiers.ctrl {
            for item in items {
                self.items.remove(&item);
            }
        } else {
            if !modifiers.shift {
                self.items.clear();
            }
            self.items.extend(items);
        }
    }
}
fn fe_segment_distance(p: Pos2, a: Pos2, b: Pos2) -> f32 {
    let ab = b - a;
    if ab.length_sq() < 1e-8 {
        return p.distance(a);
    }
    p.distance(a + ab * ((p - a).dot(ab) / ab.length_sq()).clamp(0., 1.))
}
impl App {
    pub(super) fn pick_legend_band(
        &mut self,
        rect: Rect,
        response: &egui::Response,
        modifiers: egui::Modifiers,
    ) -> bool {
        if !response.clicked_by(egui::PointerButton::Primary)
            || self.mode.is_some()
            || self.drag.is_some()
            || self.clipboard_op.is_some()
        {
            return false;
        }
        let Some(pointer) = response.interact_pointer_pos() else {
            return false;
        };
        if let Some(ids) = self.schema_legend_selection(rect, pointer) {
            if !ids.is_empty() {
                self.select_ids(ids, modifiers);
            }
            return true;
        }
        if let Some(ids) = self.punching_legend_selection(rect, pointer) {
            if !ids.is_empty() {
                self.select_ids(ids, modifiers);
            }
            return true;
        }
        if !self.show_results || matches!(self.result_field, 8 | 9) {
            return false;
        }
        let Some(range) = (if self.spatial.enabled {
            self.spatial_result_range()
        } else {
            self.result_range_cached()
        }) else {
            return false;
        };
        let scale = &self.result_scales[self.result_field];
        let Some(band) = scale.legend_band_at(rect, pointer, range, self.display_text.size) else {
            return false;
        };
        if scale.band_enabled(band) {
            if !self.spatial.enabled && self.result_field == 6 && self.sum_reactions {
                let ids = self
                    .support_reaction_totals()
                    .iter()
                    .filter(|t| {
                        scale.value_band(t.value, range) == Some(band)
                            && self.view_contour(t.id).is_some_and(|c| self.selectable(c))
                    })
                    .map(|t| t.id)
                    .collect::<Vec<_>>();
                if !ids.is_empty() {
                    self.select_ids(ids, modifiers);
                }
                return true;
            }
            let bar_field = (17..=22).contains(&self.result_field);
            if self.fe_selection.enabled[usize::from(bar_field)] {
                let items = self.legend_fe_items(scale, band, range);
                self.fe_selection.refresh(self.mesh_generation);
                self.fe_selection.select_many(items, modifiers);
            } else {
                self.status = if bar_field {
                    "Включите «КЭ стержней» в фильтре выделения".into()
                } else {
                    "Включите «КЭ оболочек» в фильтре выделения".into()
                };
            }
        }
        true
    }
    fn legend_fe_items(
        &self,
        scale: &crate::plan_trace::result_scale::Scale,
        band: usize,
        range: [f64; 2],
    ) -> Vec<(u8, usize)> {
        let (Some(case), Some(mesh)) = (self.selected_result(), self.fe_mesh.as_ref()) else {
            return vec![];
        };
        let field = self.result_field;
        let matches = |values: &[f64], continuous: bool| {
            let mut lo = usize::MAX;
            let mut hi = 0;
            for value in values.iter().filter_map(|&v| scale.value_band(v, range)) {
                if value == band {
                    return true;
                }
                lo = lo.min(value);
                hi = hi.max(value);
            }
            continuous && lo <= band && band <= hi
        };
        let bar_field = (17..=22).contains(&field);
        let bar_matches = |index: usize| {
            let Some(force) = case.shell.as_ref().and_then(|s| s.beam_forces.get(index)) else {
                return false;
            };
            let c = field - 17;
            let mut values = force
                .sections()
                .map(|(_, f)| f[c] * self.force_result_factor())
                .to_vec();
            if let Some(shear) = match c {
                2 => Some(5),
                3 => Some(4),
                _ => None,
            } {
                let [a, b] = force.ends.map(|end| end[shear]);
                let t = -a / (b - a);
                if t > 0. && t < 1. {
                    values.push(force.at(t)[c] * self.force_result_factor());
                }
            }
            matches(&values, true)
        };
        if self.spatial.enabled {
            let Some((_, _, _, _, scene)) = &self.spatial.cache else {
                return vec![];
            };
            let draw = self.spatial_draw_list(scene);
            let in_filter = |id| {
                self.view_contour(id)
                    .is_none_or(|c| self.selection_all_floors || self.on_active_floor(c))
            };
            if bar_field {
                return draw
                    .beams
                    .iter()
                    .copied()
                    .filter(|&i| in_filter(scene.beams[i].id) && bar_matches(i))
                    .map(|i| (1, i))
                    .collect();
            }
            let nodal = if field == 6 || self.point_displacements() {
                self.spatial_visible_point_results(scene)
                    .iter()
                    .copied()
                    .filter(|(n, _)| self.sections.contains(scene.xyz[*n]))
                    .collect::<BTreeMap<_, _>>()
            } else {
                BTreeMap::new()
            };
            return draw
                .surfaces
                .iter()
                .copied()
                .filter_map(|i| {
                    let surface = &scene.surfaces[i];
                    let owner = surface.support.or_else(|| {
                        Some(
                            *mesh
                                .building
                                .as_ref()?
                                .element_slabs
                                .get(*surface.display.first()?)?,
                        )
                    });
                    if owner.is_some_and(|id| !in_filter(id)) {
                        return None;
                    }
                    let hit = if field == 6 || self.point_displacements() {
                        surface.nodes.iter().any(|node| {
                            nodal
                                .get(node)
                                .is_some_and(|&v| scale.value_band(v, range) == Some(band))
                        })
                    } else {
                        self.spatial_surface_values(i, surface)
                            .is_some_and(|v| matches(&v, true))
                    };
                    hit.then(|| surface.display.first().map_or((2, i), |&d| (0, d)))
                })
                .collect();
        }
        if bar_field {
            return self
                .planar_fe_bars()
                .iter()
                .enumerate()
                .filter(|(i, (id, _))| self.fe_bar_visible_2d(*i, *id) && bar_matches(*i))
                .map(|(i, _)| (1, i))
                .collect();
        }
        let smoothed = self.smoothed_efforts();
        let support_nodes = (field == 6).then(|| self.support_node_indices());
        mesh.elements
            .iter()
            .enumerate()
            .filter_map(|(i, el)| {
                if !self.fe_slab_visible(i) {
                    return None;
                }
                let hit = if let Some(nodes) = &support_nodes {
                    el.nodes.iter().any(|&node| {
                        nodes.contains(&node)
                            && case.reaction_kn.get(node).is_some_and(|v| {
                                scale.value_band(*v * self.result_display_factor(), range)
                                    == Some(band)
                            })
                    })
                } else if self.result_has_element_field() || matches!(field, 7 | 16 | 26..=34) {
                    smoothed.as_ref().and_then(|v| v.get(i)).map_or_else(
                        || {
                            matches(
                                &[self.result_element_value(case, i)
                                    * self.result_display_factor()],
                                false,
                            )
                        },
                        |values| matches(values, true),
                    )
                } else {
                    matches(
                        &el.nodes.map(|n| self.result_node_value(case, n)),
                        !self.point_displacements(),
                    )
                };
                hit.then_some((0, i))
            })
            .collect()
    }
    fn fe_slab_visible(&self, i: usize) -> bool {
        self.mesh_element_on_floor(i) && !self.mesh_element_hidden(i)
    }

    fn fe_frame_2d(&mut self, frame: Rect, rect: Rect, window: bool, modifiers: egui::Modifiers) {
        let Some(mesh) = &self.fe_mesh else {
            return;
        };
        let mut hits = Vec::new();
        let hit_face = |nodes: &[usize]| {
            let points = nodes
                .iter()
                .map(|&n| self.camera.screen(mesh.nodes[n].xy, rect))
                .collect::<Vec<_>>();
            building::face_frame(&points, frame, window)
        };
        if self.fe_selection.enabled[0] {
            let indices: BTreeMap<_, _> = mesh
                .elements
                .iter()
                .enumerate()
                .map(|(i, e)| (e.id, i))
                .collect();
            let mut in_quads = BTreeSet::new();
            for quad in &mesh.quads {
                in_quads.extend(quad.display_triangles);
                if let Some(&i) = indices.get(&quad.display_triangles[0]) {
                    if self.fe_slab_visible(i) && hit_face(&quad.nodes) {
                        hits.push((0, i));
                    }
                }
            }
            for (i, element) in mesh.elements.iter().enumerate() {
                if !in_quads.contains(&element.id)
                    && self.fe_slab_visible(i)
                    && hit_face(&element.nodes)
                {
                    hits.push((0, i));
                }
            }
        }
        if self.fe_selection.enabled[1] {
            for (i, (id, points)) in self.planar_fe_bars().into_iter().enumerate() {
                if self.fe_bar_visible_2d(i, id)
                    && building::face_frame(
                        &points.map(|p| self.camera.screen(p, rect)),
                        frame,
                        window,
                    )
                {
                    hits.push((1, i));
                }
            }
        }
        self.fe_selection.refresh(self.mesh_generation);
        self.fe_selection.select_many(hits, modifiers);
    }

    fn fe_surface_on_selection_floor(
        &self,
        surface: &crate::plan_trace::fem::shell::Surface,
    ) -> bool {
        if !self.building.enabled || self.selection_all_floors {
            return true;
        }
        if let Some(id) = surface.support {
            return self.building.owner(id) == self.building.active;
        }
        self.fe_mesh
            .as_ref()
            .zip(surface.display.first())
            .is_some_and(|(mesh, &i)| mesh.element_floor(i) == self.building.active)
    }

    fn fe_bar_on_selection_floor(&self, id: u64) -> bool {
        !self.building.enabled
            || self.selection_all_floors
            || self.building.owner(id) == self.building.active
    }

    pub(super) fn fe_frame_3d(
        &mut self,
        frame: Rect,
        window: bool,
        projected: &[(Pos2, f64)],
        modifiers: egui::Modifiers,
    ) {
        let Some((_, _, _, true, scene)) = &self.spatial.cache else {
            return;
        };
        if !self.fe_selection.enabled.iter().any(|&enabled| enabled) {
            return;
        }
        let draw = self.spatial_draw_list(scene);
        let hit = |nodes: &[usize]| {
            let mut shape = nodes.iter().map(|&n| projected[n]).collect::<Vec<_>>();
            if self.sections.active() {
                shape = spatial_depth::clip_shape(shape);
            }
            building::face_frame(
                &shape.iter().map(|p| p.0).collect::<Vec<_>>(),
                frame,
                window,
            )
        };
        let mut hits = Vec::new();
        if self.fe_selection.enabled[0] {
            for &i in &draw.surfaces {
                let surface = &scene.surfaces[i];
                if self.fe_surface_on_selection_floor(surface) && hit(&surface.nodes) {
                    hits.push(surface.display.first().map_or((2, i), |&d| (0, d)));
                }
            }
        }
        if self.fe_selection.enabled[1] {
            for &i in &draw.beams {
                let bar = &scene.beams[i];
                if self.fe_bar_on_selection_floor(bar.id) && hit(&bar.nodes) {
                    hits.push((1, i));
                }
            }
        }
        self.fe_selection.select_many(hits, modifiers);
    }
    fn planar_fe_bars(&self) -> Vec<(u64, [Point; 2])> {
        let Some(mesh) = &self.fe_mesh else {
            return vec![];
        };
        if let Some(data) = &mesh.building {
            return data
                .beams
                .iter()
                .map(|b| (b.id, b.nodes.map(|n| mesh.nodes[n].xy)))
                .collect();
        }
        if self.support_height.beams.is_empty() && self.support_height.columns.is_empty() {
            return vec![];
        }
        let materials =
            vec![crate::plan_trace::fem::element::Material::default(); mesh.elements.len()];
        crate::plan_trace::fem::shell::scene(
            mesh,
            &materials,
            &self.support_height,
            self.mesh_size,
            &self.contours,
        )
        .map(|scene| {
            scene
                .beams
                .iter()
                .map(|b| (b.id, b.nodes.map(|n| [scene.xyz[n][0], scene.xyz[n][1]])))
                .collect()
        })
        .unwrap_or_default()
    }
    fn fe_bar_visible_2d(&self, index: usize, id: u64) -> bool {
        self.fe_item_visible((1, index)) && self.beam_result_visible(id)
    }

    pub(super) fn fe_fragment_bounds_2d(&self) -> Option<[f64; 4]> {
        let mesh = self.fe_mesh.as_ref()?;
        let bars = self.planar_fe_bars();
        let mut points = mesh
            .nodes
            .iter()
            .enumerate()
            .filter_map(|(i, n)| self.mesh_node_on_floor(i).then_some(n.xy))
            .chain(
                bars.into_iter()
                    .enumerate()
                    .filter(|(i, (id, _))| self.fe_bar_visible_2d(*i, *id))
                    .flat_map(|(_, (_, p))| p),
            );
        let [x, y] = points.next()?;
        Some(points.fold([x, y, x, y], |b, [x, y]| {
            [b[0].min(x), b[1].min(y), b[2].max(x), b[3].max(y)]
        }))
    }

    pub(super) fn paint_fe_fragment_bars_2d(&self, painter: &egui::Painter, rect: Rect) {
        for (i, (id, points)) in self.planar_fe_bars().into_iter().enumerate() {
            if self.fe_bar_visible_2d(i, id) {
                let kind = if self.support_height.columns.contains_key(&id) {
                    Kind::Column
                } else {
                    Kind::Beam
                };
                let stroke = egui::Stroke::new(2., self.object_ink(id, kind));
                let [a, b] = points.map(|p| self.camera.screen(p, rect));
                if a.distance(b) < 1. {
                    painter.circle_filled(a, 3., stroke.color);
                } else {
                    painter.line_segment([a, b], stroke);
                }
            }
        }
    }
    pub(super) fn fe_pick_2d(
        &mut self,
        pointer: Option<Pos2>,
        camera: Camera,
        rect: Rect,
        modifiers: egui::Modifiers,
    ) -> bool {
        self.fe_selection.refresh(self.mesh_generation);
        let (Some(pointer), Some(mesh)) = (pointer, self.fe_mesh.as_ref()) else {
            return false;
        };
        let mut hit = None;
        if self.fe_selection.enabled[1] {
            hit = self
                .planar_fe_bars()
                .iter()
                .enumerate()
                .find(|(i, (id, [a, b]))| {
                    self.fe_bar_visible_2d(*i, *id)
                        && fe_segment_distance(
                            pointer,
                            camera.screen(*a, rect),
                            camera.screen(*b, rect),
                        ) <= 6.
                })
                .map(|(i, _)| (1, i));
        }
        if hit.is_none() && self.fe_selection.enabled[0] {
            let p = camera.world(pointer, rect);
            hit = mesh
                .elements
                .iter()
                .enumerate()
                .find(|(i, e)| {
                    self.fe_slab_visible(*i)
                        && geometry::inside(p, &e.nodes.map(|n| mesh.nodes[n].xy))
                })
                .map(|(i, _)| (0, i));
        }
        self.fe_selection.pick(hit, modifiers)
    }
    pub(super) fn paint_fe_selection_2d(
        &mut self,
        painter: &egui::Painter,
        camera: Camera,
        rect: Rect,
    ) {
        self.fe_selection.refresh(self.mesh_generation);
        let Some(mesh) = &self.fe_mesh else {
            return;
        };
        let bars = if self.fe_selection.items.iter().any(|(kind, _)| *kind == 1) {
            self.planar_fe_bars()
        } else {
            vec![]
        };
        let selected_faces = self
            .fe_selection
            .items
            .iter()
            .filter(|(kind, _)| *kind == 0)
            .count();
        let quads_by_triangle = (selected_faces > 1).then(|| {
            mesh.quads
                .iter()
                .flat_map(|q| q.display_triangles.map(|id| (id, q)))
                .collect::<BTreeMap<_, _>>()
        });
        let mut drawn_quads = BTreeSet::new();
        for &(kind, i) in &self.fe_selection.items {
            if kind == 0 && self.fe_slab_visible(i) {
                let e = &mesh.elements[i];
                let quad = match &quads_by_triangle {
                    Some(map) => map.get(&e.id).copied(),
                    None => mesh
                        .quads
                        .iter()
                        .find(|q| q.display_triangles.contains(&e.id)),
                };
                if quad.is_some_and(|q| !drawn_quads.insert(q.id)) {
                    continue;
                }
                let nodes = quad.map_or(e.nodes.as_slice(), |q| q.nodes.as_slice());
                let points = nodes
                    .iter()
                    .map(|&n| camera.screen(mesh.nodes[n].xy, rect))
                    .collect();
                painter.add(egui::Shape::closed_line(
                    points,
                    egui::Stroke::new(2.0_f32, Color32::YELLOW),
                ));
            } else if kind == 1 {
                if let Some((id, points)) = bars.get(i) {
                    if self.fe_bar_visible_2d(i, *id) {
                        painter.line_segment(
                            points.map(|p| camera.screen(p, rect)),
                            egui::Stroke::new(3.0_f32, Color32::YELLOW),
                        );
                    }
                }
            }
        }
    }
    pub(super) fn fe_pick_3d(
        &self,
        pointer: Pos2,
        scene: &crate::plan_trace::fem::shell::Scene,
        surfaces: &[usize],
        beams: &[usize],
        projected: &[(Pos2, f64)],
    ) -> Option<(u8, usize)> {
        let mut hit = None;
        let mut depth = f64::NEG_INFINITY;
        for &i in surfaces {
            let surface = &scene.surfaces[i];
            let nodes = &surface.nodes;
            for j in 1..nodes.len() - 1 {
                let tri = [nodes[0], nodes[j], nodes[j + 1]].map(|n| projected[n]);
                if let Some((weights, z)) = spatial_results::pick_triangle(pointer, tri) {
                    let world = std::array::from_fn(|axis| {
                        [nodes[0], nodes[j], nodes[j + 1]]
                            .iter()
                            .zip(weights)
                            .map(|(&n, w)| scene.xyz[n][axis] * w)
                            .sum()
                    });
                    if z > depth && self.sections.contains(world) {
                        depth = z;
                        hit = (self.fe_selection.enabled[0]
                            && self.fe_surface_on_selection_floor(surface))
                        .then(|| surface.display.first().map_or((2, i), |&d| (0, d)));
                    }
                }
            }
        }
        if self.fe_selection.enabled[1] {
            for &i in beams {
                let bar = &scene.beams[i];
                if !self.fe_bar_on_selection_floor(bar.id) {
                    continue;
                }
                let [a, b] = bar.nodes.map(|n| projected[n]);
                let ab = b.0 - a.0;
                let t = if ab.length_sq() > 1e-8 {
                    ((pointer - a.0).dot(ab) / ab.length_sq()).clamp(0., 1.) as f64
                } else {
                    0.
                };
                let z = a.1 + (b.1 - a.1) * t;
                let world = std::array::from_fn(|axis| {
                    scene.xyz[bar.nodes[0]][axis] * (1. - t) + scene.xyz[bar.nodes[1]][axis] * t
                });
                if z >= depth - 1e-6
                    && self.sections.contains(world)
                    && fe_segment_distance(pointer, a.0, b.0) <= 6.
                {
                    depth = z;
                    hit = Some((1, i));
                }
            }
        }
        hit
    }
}

#[cfg(test)]
mod legend_tests {
    use super::*;

    #[test]
    fn selected_contours_are_pruned_with_a_stale_view_index() {
        let mut app = App::default();
        for x in [0., 4000.] {
            app.begin(Kind::Slab);
            app.choose_tool(Tool::Rectangle);
            app.sketch_click([x, 0.], false);
            app.sketch_click([x + 2000., 2000.], false);
            app.cancel();
        }
        let first = app.contours[0].id;
        let second = app.contours[1].id;
        let removed = u64::MAX;
        app.spatial.enabled = true;
        app.spatial.source_index = BTreeMap::from([(first, 1), (second, 0)]);
        app.selection_ids.extend([first, second, removed]);
        app.selected = Some(first);
        app.view_visibility.contours.insert(first);

        app.prune_selection();
        assert_eq!(app.selection_ids, BTreeSet::from([second]));
        assert_eq!(app.selected, Some(second));

        app.selection_filter[0] = false;
        app.prune_selection();
        assert!(app.selection_ids.is_empty());
        assert_eq!(app.selected, None);
    }

    #[test]
    fn shell_frame_selection_respects_the_filter_in_2d_and_3d() {
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 800.).unwrap());
        let mesh = app.fe_mesh.as_ref().unwrap();
        let scene = crate::plan_trace::fem::shell::scene(
            mesh,
            &vec![Default::default(); mesh.elements.len()],
            &app.support_height,
            800.,
            &app.contours,
        )
        .unwrap();
        let projected = scene
            .xyz
            .iter()
            .map(|p| (Pos2::new(p[0] as f32, p[1] as f32), p[2]))
            .collect::<Vec<_>>();
        let total = mesh.elements.len();
        let rect = Rect::from_min_max(Pos2::new(-10., -10.), Pos2::new(2010., 2010.));
        app.spatial.cache = Some((
            0,
            app.support_height.clone(),
            app.contours.clone(),
            true,
            scene,
        ));
        app.fe_selection.enabled = [false, false];
        app.fe_frame_3d(rect, true, &projected, egui::Modifiers::NONE);
        assert!(app.fe_selection.items.is_empty());
        app.fe_selection.enabled[0] = true;
        app.fe_frame_3d(rect, true, &projected, egui::Modifiers::NONE);
        assert_eq!(app.fe_selection.items.len(), total);
        app.fe_frame_3d(rect, true, &projected, egui::Modifiers::CTRL);
        assert!(app.fe_selection.items.is_empty());
        let viewport = Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.));
        app.camera.center = [1000., 1000.];
        app.camera.zoom = 0.1;
        app.fe_frame_2d(viewport, viewport, true, egui::Modifiers::NONE);
        assert_eq!(app.fe_selection.items.len(), total);
    }

    #[test]
    fn c1_legend_band_selects_only_visible_foundation_cells() {
        let mut app = App::default();
        for x in [0., 4000.] {
            for kind in [Kind::Slab, Kind::Foundation] {
                app.begin(kind);
                app.choose_tool(Tool::Rectangle);
                app.sketch_click([x, 0.], false);
                app.sketch_click([x + 2000., 2000.], false);
                app.cancel();
            }
        }
        for c in &mut app.contours {
            if c.kind == Kind::Slab {
                c.use_foundation = true;
            }
        }
        let slab = app.contours[0].id;
        let mut doubled = crate::plan_trace::foundation::default_c1();
        doubled.value *= 2.;
        app.contours[3]
            .foundation
            .get_or_insert_with(Default::default)
            .values = vec![doubled; 4];
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap());
        app.result_field = 33;
        app.show_results = true;
        app.sync_source_view();
        let scale = &app.result_scales[33];
        let range = app.result_range_cached().unwrap();
        let expected = scale.value_band(doubled.value * 0.5, range).unwrap();
        let selected = app.legend_fe_items(scale, expected, range);
        assert!(!selected.is_empty());
        let mesh = app.fe_mesh.as_ref().unwrap();
        assert!(selected.iter().all(|&(_, i)| mesh.elements[i]
            .nodes
            .iter()
            .all(|&n| mesh.nodes[n].xy[0] <= 2000. + 1e-5)));
        app.view_visibility.contours.insert(slab);
        app.view_visibility_changed();
        assert!(app
            .legend_fe_items(&app.result_scales[33], expected, range)
            .is_empty());
    }
}
