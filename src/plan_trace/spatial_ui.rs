//! Spatial source selection and viewer; the authoritative 2D source geometry is unchanged.
#[path = "projection_toolbar.rs"]
mod projection_toolbar;
use super::*;
use crate::plan_trace::fem::{
    element::Material,
    shell::{self, Direction},
};
pub(super) struct CachedDepth {
    key: Vec<u64>,
    vertices: std::sync::Arc<Vec<spatial_depth::Vertex>>,
    zoom: f64,
    pan: egui::Vec2,
}
pub(super) struct View {
    pub(super) edit_camera_bounds: Option<([f64; 3], f64)>,
    pub(super) edit_hit: bool,
    pub(super) structure_edge_cache:
        std::cell::RefCell<Option<(Vec<u64>, spatial_world::StructureEdges)>>,
    pub(super) source_index: BTreeMap<u64, usize>,
    pub(super) source_revision: usize,
    pub(super) source_surface_owners: BTreeMap<usize, Vec<u64>>,
    projected: Vec<(Pos2, f64)>,
    projected_owner: Option<std::sync::Arc<object_visibility::DrawList>>,
    pub(super) hover_surface: Option<(Rect, egui::LayerId)>,
    pub(super) hover_cycle: isize,
    pub(super) hover_ids: Vec<u64>,
    pub(super) hover_index: usize,
    pub(super) underlay_hover: Option<u64>,
    pub(super) restraint_hover: Option<u64>,
    pub(super) edit_plane_z: f64,
    camera_busy_until: f64,
    pub(super) load_sources: std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<Vec<Contour>>)>>,
    pub(super) point_geometry:
        std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<Vec<spatial_points::Instance>>)>>,
    pub(super) visible_point_values: std::cell::RefCell<
        Option<(
            [u64; 8],
            std::sync::Arc<object_visibility::DrawList>,
            std::sync::Arc<Vec<(usize, f64)>>,
        )>,
    >,
    pub(super) point_values:
        std::cell::RefCell<Option<([u64; 8], std::sync::Arc<Vec<(usize, f64)>>)>>,
    pub(super) pdf_sources:
        std::cell::RefCell<std::collections::BTreeMap<u64, (usize, std::sync::Arc<Vec<u8>>)>>,
    pub(super) frame_start: Option<Pos2>,
    building_revision: u64,
    stiffness_revision: u64,
    bounds: Option<([f64; 3], f64)>,
    pub(super) range_cache: std::cell::RefCell<Option<([u64; 8], [f64; 2])>>,
    pub enabled: bool,
    pub(super) wall_cache: std::cell::RefCell<Option<([u64; 8], std::sync::Arc<Vec<Vec<f64>>>)>>,
    depth_cache: std::cell::RefCell<Option<CachedDepth>>,
    pub(super) world_cache:
        std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<spatial_world::Geometry>)>>,
    deformation_cache:
        std::cell::RefCell<Option<(Vec<u64>, std::sync::Arc<Vec<spatial_depth::Vertex>>)>>,
    yaw: f64,
    pub(super) pitch: f64,
    zoom: f64,
    pan: egui::Vec2,
    pub(super) cache: Option<(u64, shell::Settings, Vec<Contour>, bool, shell::Scene)>,
    scene_error: Option<(bool, String)>,
}
impl Default for View {
    fn default() -> Self {
        Self {
            edit_hit: false,
            source_index: BTreeMap::new(),
            structure_edge_cache: Default::default(),
            source_revision: 0,
            source_surface_owners: Default::default(),
            projected: vec![],
            projected_owner: None,
            hover_surface: None,
            hover_cycle: 0,
            hover_ids: vec![],
            hover_index: 0,
            underlay_hover: None,
            restraint_hover: None,
            edit_plane_z: 0.,
            camera_busy_until: 0.,
            load_sources: Default::default(),
            point_geometry: Default::default(),
            point_values: Default::default(),
            visible_point_values: Default::default(),
            pdf_sources: Default::default(),
            frame_start: None,
            building_revision: 0,
            stiffness_revision: 0,
            enabled: false,
            bounds: None,
            range_cache: Default::default(),
            wall_cache: Default::default(),
            depth_cache: Default::default(),
            world_cache: Default::default(),
            deformation_cache: Default::default(),
            yaw: -0.65,
            pitch: -0.65,
            edit_camera_bounds: None,
            zoom: 1.,
            pan: egui::Vec2::ZERO,
            cache: None,
            scene_error: None,
        }
    }
}
impl View {
    pub(super) fn lock_edit_camera(&mut self) {
        if self.edit_camera_bounds.is_none() {
            self.edit_camera_bounds = self.bounds;
        }
    }
    pub(super) fn focus_warning(&mut self, center: [f64; 3], extent: f64) {
        self.bounds = Some((center, extent));
        self.zoom = 0.75;
        self.pan = egui::Vec2::ZERO;
        self.world_cache.borrow_mut().take();
        self.depth_cache.borrow_mut().take();
    }

    pub(super) fn invalidate_load_geometry(&self) {
        self.load_sources.borrow_mut().take();
        self.world_cache.borrow_mut().take();
    }
    pub(super) fn fit_view(&mut self) {
        self.zoom = 1.;
        self.pan = egui::Vec2::ZERO;
        self.edit_camera_bounds = None;
        self.bounds = None;
    }

    pub(super) fn cache_reset(&mut self) {
        self.source_index.clear();
        self.structure_edge_cache.get_mut().take();
        self.source_surface_owners.clear();
        self.load_sources.borrow_mut().take();
        self.projected.clear();
        self.projected_owner = None;
        self.source_revision = self.source_revision.wrapping_add(1);
        self.point_geometry.borrow_mut().take();
        self.point_values.borrow_mut().take();
        self.visible_point_values.borrow_mut().take();
        self.cache = None;
        self.scene_error = None;
        self.bounds = self.edit_camera_bounds;
        self.world_cache.borrow_mut().take();
        self.depth_cache.borrow_mut().take();
        self.deformation_cache.borrow_mut().take();
    }

    pub(super) fn visibility_reset(&mut self) {
        self.load_sources.borrow_mut().take();
        self.point_geometry.borrow_mut().take();
        self.point_values.borrow_mut().take();
        self.visible_point_values.borrow_mut().take();
        self.world_cache.borrow_mut().take();
        self.depth_cache.borrow_mut().take();
        self.deformation_cache.borrow_mut().take();
    }

    fn zoom_to(&mut self, rect: Rect, at: Pos2, next: f64) {
        let old = self.zoom;
        let next = next.clamp(0.02, 100.);
        self.pan = (at - rect.center()) - ((at - rect.center()) - self.pan) * (next / old) as f32;
        self.zoom = next;
    }
}
fn extend_shape_depth(points: &mut Vec<(Pos2, f64)>, margin: f64) {
    if margin > 0. && margin.is_finite() {
        let [far, near] = points
            .iter()
            .map(|p| p.1)
            .filter(|z| z.is_finite())
            .fold([f64::INFINITY, f64::NEG_INFINITY], |[far, near], z| {
                [far.min(z), near.max(z)]
            });
        if far.is_finite() {
            // Mode vectors have unit peak displacement. Reserve room for the entire animation.
            points.extend([(Pos2::ZERO, far - margin), (Pos2::ZERO, near + margin)]);
        }
    }
}

fn scalar_ink(base: Color32, scalar: Option<Color32>) -> Color32 {
    scalar.map_or(Color32::TRANSPARENT, |ink| {
        Color32::from_rgba_unmultiplied(ink.r(), ink.g(), ink.b(), base.a())
    })
}

fn shape_hover_pointer(ui: &egui::Ui, rect: Rect) -> Option<Pos2> {
    ui.input(|i| {
        (!i.pointer.any_down())
            .then(|| i.pointer.hover_pos())
            .flatten()
    })
    .filter(|p| rect.contains(*p))
}

impl App {
    pub(super) fn support_height_controls(&mut self, ui: &mut egui::Ui) {
        ui.add_enabled_ui(!self.input_locked,|ui|{
            ui.checkbox(&mut self.support_height.enabled,"Учитывать высоту опор");
            if self.support_height.enabled {
                ui.horizontal(|ui|{
                    egui::ComboBox::from_id_salt("support_height_direction").selected_text(self.support_height.direction.label()).show_ui(ui,|ui|{
                        for d in [Direction::Up,Direction::Down,Direction::Both]{ui.selectable_value(&mut self.support_height.direction,d,d.label());}
                    });
                    ui.label("Высота, мм");ui.add(egui::DragValue::new(&mut self.support_height.height_mm).range(100.0..=1_000_000.).speed(100.));
                });
                ui.small("Вверх+вниз: указанная высота в каждую сторону. Низ: X,Y,Z; верх: X,Y. Повороты свободны.");

            }
        });
        ui.separator();
    }
    fn spatial_style_key(&self) -> u64 {
        use std::hash::{Hash, Hasher};
        let mut hash = std::collections::hash_map::DefaultHasher::new();
        let drawing_flags::Visibility {
            dark_theme,
            canvas_background,
            load_opacity,
            ground_opacity,
            fill,
            stiffness_colors,
            material_colors,
            support_extents,
            structure_edges,
            support_storeys,
            restraints,
            punching,
            foundation,
            columns,
            rigid_bodies,
            column_color,
            rigid_color,
            beam,
            triangulation,
            triangulation_color,
            beam_color,
            contours,
            loads,
            load_colors,
        } = self.drawing_visibility;
        [
            dark_theme,
            fill,
            stiffness_colors,
            material_colors,
            support_extents,
            structure_edges,
            restraints,
            punching,
            foundation,
            columns,
            rigid_bodies,
            beam,
            triangulation,
        ]
        .hash(&mut hash);
        [load_opacity.to_bits(), ground_opacity.to_bits()].hash(&mut hash);
        support_storeys.hash(&mut hash);
        contours.hash(&mut hash);
        loads.hash(&mut hash);
        [
            canvas_background,
            column_color,
            rigid_color,
            triangulation_color,
            beam_color,
        ]
        .hash(&mut hash);
        load_colors.hash(&mut hash);
        self.contour_colors.hash(&mut hash);
        self.slab_color.hash(&mut hash);
        if stiffness_colors || material_colors {
            self.stiffness_sources.revision.hash(&mut hash);
        }
        hash.finish()
    }

    fn paint_deformation_view(
        &self,
        ui: &egui::Ui,
        painter: &egui::Painter,
        rect: Rect,
        scene: &shell::Scene,
        projected: &[(Pos2, f64)],
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        center: [f64; 3],
        extent: f64,
        scale: f64,
    ) -> bool {
        let (show_deformation, deformation_scale) = ui.ctx().data(|data| {
            data.get_temp::<(bool, f64)>(egui::Id::new("static_deformation"))
                .unwrap_or((false, 100.))
        });
        let mut modal = ui
            .ctx()
            .data(|d| d.get_temp::<super::fem_ui::ModalView>(egui::Id::new("modal_view")))
            .filter(|view| {
                view.mesh_generation == self.mesh_generation
                    && view.stiffness_variant == self.stiffness_variant
                    && view.result_generation == self.result_generation
                    && view.shape.len() == scene.xyz.len()
                    && self.fe_mesh.is_some()
            });
        let show_deformation = show_deformation
            && modal.is_none()
            && self.selected_result().is_some()
            && self.fe_mesh.is_some();
        let now = ui.input(|i| i.time);
        let active = ui.input(|i| {
            i.viewport().focused.unwrap_or(true) && !i.viewport().minimized.unwrap_or(false)
        });
        let modal_animation = modal.as_mut().map(|view| {
            let result = view.animation(now, active);
            ui.ctx()
                .data_mut(|d| d.insert_temp(egui::Id::new("modal_view"), view.clone()));
            result
        });
        if show_deformation || modal.is_some() {
            let vibration = modal.as_ref().and_then(|view| view.vibration_values);
            let sensitivity = modal.as_ref().and_then(|view| view.active_sensitivity());
            let mut key = self.view_key().to_vec();
            key.extend([
                self.stiffness_variant as u64,
                modal.as_ref().map_or(0, |v| v.number as u64),
                modal.as_ref().map_or(0, |v| v.source.cache_id()),
                self.spatial_style_key(),
                self.slab_opacity.to_bits() as u64,
                self.show_mesh as u64,
            ]);
            if let Some((quantity, range)) = vibration {
                use super::fem_ui::VibrationQuantity;
                let id = match quantity {
                    VibrationQuantity::Axis(axis) => 1 + axis as u64,
                    VibrationQuantity::Magnitude(axes) => 8 + u64::from(axes),
                };
                key.extend([id, range[0].to_bits(), range[1].to_bits()]);
                key.push(self.shape_iso as u64);
                key.extend(self.shape_scale.signature());
            }
            if let Some(field) = sensitivity {
                key.push(32);
                key.extend(field.range.map(f64::to_bits));
                key.push(self.buckling_iso as u64);
                key.extend(self.buckling_scale.signature());
            }
            key.extend(center.map(f64::to_bits));
            key.push(extent.to_bits());
            key.extend(self.view_visibility.contours.iter().copied());
            key.extend(
                self.selected
                    .iter()
                    .chain(&self.selection_ids)
                    .copied()
                    .filter(|id| self.support_height.columns.contains_key(id)),
            );
            let cached = self
                .spatial
                .deformation_cache
                .borrow()
                .as_ref()
                .filter(|(old, _)| *old == key)
                .map(|(_, v)| v.clone());
            let vertices = cached.unwrap_or_else(|| {
                let result = modal
                    .is_none()
                    .then(|| self.selected_full_result())
                    .flatten();
                let displacement = |n: usize| -> Option<[f64; 3]> {
                    if let Some(view) = &modal {
                        return view.shape.get(n).copied();
                    }
                    let result = result?;
                    if let Some(shell) = &result.shell {
                        shell.translations_mm.get(n).copied()
                    } else {
                        result.w_mm.get(*scene.plan.get(n)?).map(|w| [0., 0., -*w])
                    }
                };
                let node_values = vibration.zip(modal.as_ref()).map(|((quantity, _), view)| {
                    view.shape
                        .iter()
                        .map(|&xyz| quantity.value(xyz))
                        .collect::<Vec<_>>()
                });
                let mut edges = BTreeMap::new();
                let draw = self.spatial_draw_list(scene);
                let mut surface_samples = std::collections::HashMap::<_, (f64, usize)>::new();
                let mut beam_samples = std::collections::HashMap::<_, (f64, usize)>::new();
                if let Some(field) = sensitivity.filter(|_| self.buckling_iso) {
                    for &index in &draw.surfaces {
                        let surface = &scene.surfaces[index];
                        if let Some(&value) =
                            field.data.surfaces.get(index).filter(|v| v.is_finite())
                        {
                            for &node in &surface.nodes {
                                let (sum, count) =
                                    surface_samples.entry((surface.support, node)).or_default();
                                *sum += f64::from(value);
                                *count += 1;
                            }
                        }
                    }
                    for &index in &draw.beams {
                        if let Some(&value) = field.data.beams.get(index).filter(|v| v.is_finite())
                        {
                            let beam = &scene.beams[index];
                            for node in beam.nodes {
                                let (sum, count) = beam_samples.entry((beam.id, node)).or_default();
                                *sum += f64::from(value);
                                *count += 1;
                            }
                        }
                    }
                }
                for &index in draw.surfaces.iter().filter(|_| self.show_mesh) {
                    let surface = &scene.surfaces[index];
                    for (&a, &b) in surface
                        .nodes
                        .iter()
                        .zip(surface.nodes.iter().cycle().skip(1))
                    {
                        edges.insert((a.min(b), a.max(b)), [Color32::from_rgb(36, 58, 74); 2]);
                    }
                }
                let mut out = spatial_world::deformation_faces(
                    scene,
                    &draw.surfaces,
                    displacement,
                    center,
                    extent,
                    |index, surface| {
                        let mut base = spatial_world::surface_fill_ink(self, surface, &scene.xyz)?;
                        if let Some(field) = sensitivity.filter(|_| !self.buckling_iso) {
                            let value = f64::from(*field.data.surfaces.get(index)?);
                            base = scalar_ink(base, self.buckling_scale.color(value, field.range));
                        } else if let Some((values, (_, range))) = node_values
                            .as_ref()
                            .zip(vibration)
                            .filter(|_| !self.shape_iso)
                        {
                            let mean = surface.nodes.iter().map(|&n| values[n]).sum::<f64>()
                                / surface.nodes.len() as f64;
                            base = scalar_ink(base, self.shape_scale.color(mean, range));
                        }
                        Some((
                            base,
                            spatial_world::wall_half_offset(self, surface, &scene.xyz),
                        ))
                    },
                    sensitivity
                        .filter(|_| self.buckling_iso)
                        .map(|field| (&self.buckling_scale, field.range))
                        .or_else(|| {
                            vibration
                                .filter(|_| self.shape_iso)
                                .map(|(_, range)| (&self.shape_scale, range))
                        }),
                    |index, node| {
                        if sensitivity.is_some() {
                            surface_samples
                                .get(&(scene.surfaces[index].support, node))
                                .map(|&(sum, count)| sum / count as f64)
                        } else {
                            node_values
                                .as_ref()
                                .and_then(|values| values.get(node).copied())
                        }
                    },
                );
                for &index in &draw.beams {
                    let beam = &scene.beams[index];
                    let [a, b] = beam.nodes;
                    let kind = if self.support_height.columns.contains_key(&beam.id) {
                        Kind::Column
                    } else {
                        Kind::Beam
                    };
                    let ink = self.object_ink(beam.id, kind);
                    let beam_ink = |node, base| {
                        if let Some(field) = sensitivity {
                            let value = if self.buckling_iso {
                                beam_samples
                                    .get(&(beam.id, node))
                                    .map(|&(sum, count)| sum / count as f64)
                            } else {
                                field.data.beams.get(index).copied().map(f64::from)
                            };
                            scalar_ink(
                                base,
                                value.and_then(|value| {
                                    self.buckling_scale.color(value, field.range)
                                }),
                            )
                        } else if let Some((values, (_, range))) =
                            node_values.as_ref().zip(vibration)
                        {
                            let value = if self.shape_iso {
                                values[node]
                            } else {
                                (values[a] + values[b]) * 0.5
                            };
                            scalar_ink(base, self.shape_scale.color(value, range))
                        } else {
                            base
                        }
                    };
                    if kind == Kind::Column
                        && self.drawing_visibility.support_extents
                        && spatial_world::deformation_column(
                            self,
                            scene,
                            beam,
                            displacement,
                            center,
                            extent,
                            &mut out,
                            beam_ink,
                        )
                    {
                        continue;
                    }
                    edges.insert(
                        (a.min(b), a.max(b)),
                        [beam_ink(a.min(b), ink), beam_ink(a.max(b), ink)],
                    );
                }
                out.reserve(edges.len() * 6);
                for ((a, b), [ink_a, ink_b]) in edges {
                    let (Some(da), Some(db)) = (displacement(a), displacement(b)) else {
                        continue;
                    };
                    if !da.into_iter().chain(db).all(f64::is_finite) {
                        continue;
                    }
                    let p = scene.xyz[a];
                    let q = scene.xyz[b];
                    let d: [f64; 3] = std::array::from_fn(|i| q[i] - p[i]);
                    let side = if d[0].hypot(d[1]) > d[2].abs() {
                        [-d[1], d[0], 0.]
                    } else {
                        [d[2], 0., -d[0]]
                    };
                    let norm = side.iter().map(|v| v * v).sum::<f64>().sqrt();
                    if norm <= 1e-12 {
                        continue;
                    }
                    let offset = (extent * 0.0006).min(norm * 0.025);
                    for (point, delta, sign, ink) in [
                        (p, da, 1., ink_a),
                        (q, db, 1., ink_b),
                        (q, db, -1., ink_b),
                        (p, da, 1., ink_a),
                        (q, db, -1., ink_b),
                        (p, da, -1., ink_a),
                    ] {
                        let point =
                            std::array::from_fn(|i| point[i] + sign * side[i] * offset / norm);
                        let mut vertex = spatial_world::vertex(point, ink, center, extent);
                        vertex.offset = delta.map(|v| (v / extent) as f32);
                        out.push(vertex);
                    }
                }
                let vertices = std::sync::Arc::new(out);
                *self.spatial.deformation_cache.borrow_mut() = Some((key, vertices.clone()));
                vertices
            });
            let amplitude = modal_animation.map_or(deformation_scale, |(amplitude, repaint)| {
                if repaint {
                    ui.ctx().request_repaint_after(Duration::from_millis(16));
                }
                amplitude
            });
            let matrix = spatial_world::camera(
                rect,
                projected,
                extent,
                scale,
                self.spatial.yaw,
                self.spatial.pitch,
                self.spatial.pan,
            );
            spatial_depth::set_clips(self.sections.screen_planes(matrix, center, extent));
            spatial_depth::set_clip_depth(rect, projected);
            spatial_depth::paint_deformation(painter, rect, vertices, matrix, amplitude as f32);
            if let Some((view, field)) =
                modal.as_ref().filter(|view| !view.playing).zip(sensitivity)
            {
                if let Some(pointer) = shape_hover_pointer(ui, rect) {
                    if let Some(text) =
                        self.sensitivity_hover(pointer, scene, project, view, field, amplitude)
                    {
                        fem_ui::result_label(
                            painter,
                            pointer + egui::vec2(16., 18.),
                            text,
                            rect,
                            self.display_text,
                        );
                    }
                }
            }
            if let Some((quantity, range)) = vibration {
                if let Some(view) = modal.as_ref().filter(|view| !view.playing) {
                    if let Some(pointer) = shape_hover_pointer(ui, rect) {
                        let draw = self.spatial_draw_list(scene);
                        let hit = draw
                            .nodes
                            .iter()
                            .filter_map(|&n| {
                                let xyz = view.shape[n];
                                let p =
                                    std::array::from_fn(|i| scene.xyz[n][i] + amplitude * xyz[i]);
                                let distance = project(p).0.distance_sq(pointer);
                                if distance > 64. || !self.sections.contains(p) {
                                    return None;
                                }
                                let value = quantity.value(xyz);
                                self.shape_scale
                                    .color(value, range)
                                    .map(|_| (distance, n, value))
                            })
                            .min_by(|a, b| a.0.total_cmp(&b.0));
                        if let Some((_, node, value)) = hit {
                            fem_ui::result_label(
                                painter,
                                pointer + egui::vec2(16., 18.),
                                format!(
                                    "Узел {} · {} отн.",
                                    node + 1,
                                    self.display_text.number(value)
                                ),
                                rect,
                                self.display_text,
                            );
                        }
                    }
                }
            }
        } else {
            self.spatial.deformation_cache.borrow_mut().take();
        }
        if let Some(view) = modal.as_ref().filter(|view| view.source.is_buckling()) {
            painter.text(
                rect.left_bottom() + egui::vec2(12., -12.),
                egui::Align2::LEFT_BOTTOM,
                view.source.caption(view.number),
                egui::FontId::proportional(14.),
                Color32::from_rgb(255, 160, 40),
            );
        }
        show_deformation || modal.is_some()
    }

    fn shape_legend(&self, painter: &egui::Painter, rect: Rect) -> bool {
        if let Some(range) = self
            .displayed_shape(painter.ctx(), true)
            .and_then(|view| view.active_sensitivity().map(|field| field.range))
        {
            self.buckling_scale
                .legend(painter, rect, range, self.display_text);
            return true;
        }
        if let Some((_, range)) = self
            .displayed_shape(painter.ctx(), false)
            .and_then(|view| view.vibration_values)
        {
            self.shape_scale
                .legend(painter, rect, range, self.display_text);
            return true;
        }
        false
    }

    fn sensitivity_hover(
        &self,
        pointer: Pos2,
        scene: &shell::Scene,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        view: &super::fem_ui::ModalView,
        field: &super::fem_ui::BucklingColors,
        amplitude: f64,
    ) -> Option<String> {
        let draw = self.spatial_draw_list(scene);
        let moved = |node: usize| {
            std::array::from_fn(|axis| scene.xyz[node][axis] + amplitude * view.shape[node][axis])
        };
        let mut front = f64::NEG_INFINITY;
        let mut hit = None;
        for &index in &draw.surfaces {
            let surface = &scene.surfaces[index];
            let Some(&value) = field.data.surfaces.get(index) else {
                continue;
            };
            let Some(&first) = surface.nodes.first() else {
                continue;
            };
            for edge in surface.nodes[1..].windows(2) {
                let world = [first, edge[0], edge[1]].map(moved);
                let Some((weights, depth)) =
                    spatial_results::pick_triangle(pointer, world.map(project))
                else {
                    continue;
                };
                if depth <= front {
                    continue;
                }
                let point =
                    std::array::from_fn(|axis| (0..3).map(|i| weights[i] * world[i][axis]).sum());
                if self.sections.contains(point)
                    && spatial_world::surface_fill_ink(self, surface, &scene.xyz).is_some()
                    && self
                        .buckling_scale
                        .color(f64::from(value), field.range)
                        .is_some()
                {
                    front = depth;
                    hit = Some((false, index, value));
                }
            }
        }
        for &index in &draw.beams {
            let beam = &scene.beams[index];
            let Some(&value) = field.data.beams.get(index) else {
                continue;
            };
            let world = beam.nodes.map(moved);
            let [(a, za), (b, zb)] = world.map(project);
            let delta = b - a;
            if delta.length_sq() <= 1e-6 {
                continue;
            }
            let t = ((pointer - a).dot(delta) / delta.length_sq()).clamp(0., 1.);
            let depth = za + f64::from(t) * (zb - za);
            if depth < front || (a + t * delta).distance_sq(pointer) > 36. {
                continue;
            }
            let point =
                std::array::from_fn(|i| world[0][i] + f64::from(t) * (world[1][i] - world[0][i]));
            if self.sections.contains(point)
                && self
                    .buckling_scale
                    .color(f64::from(value), field.range)
                    .is_some()
            {
                front = depth;
                hit = Some((true, index, value));
            }
        }
        let (beam, index, value) = hit?;
        let object = if beam {
            format!(
                "КЭ стержня {} · объект {}",
                index + 1,
                scene.beams[index].id
            )
        } else if let Some(wall) = scene.surfaces[index].support {
            format!("КЭ оболочки {} · стена {wall}", index + 1)
        } else {
            let id = scene.surfaces[index]
                .display
                .first()
                .and_then(|&display| {
                    let mesh = self.fe_mesh.as_ref()?;
                    let triangle = mesh.elements.get(display)?.id;
                    Some(
                        mesh.quads
                            .iter()
                            .find(|q| q.display_triangles.contains(&triangle))
                            .map_or(triangle, |q| q.id),
                    )
                })
                .unwrap_or(index as u64 + 1);
            format!("КЭ плиты {id}")
        };
        Some(format!(
            "{object} · {} %",
            self.display_text.number(f64::from(value))
        ))
    }

    fn spatial_construction_bounds(&self) -> ([f64; 3], f64) {
        let z = self.spatial_construction_z();
        let grid = self.construction_grid;
        if grid.enabled && grid.parameters_valid() {
            (
                [grid.center[0], grid.center[1], z],
                grid.width_mm.hypot(grid.height_mm).max(1.),
            )
        } else {
            ([0., 0., z], 10000_f64.hypot(10000.))
        }
    }

    pub(super) fn spatial_canvas(&mut self, ui: &mut egui::Ui) {
        let _clip_guard = spatial_depth::clip_guard();
        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        let painter = ui.painter().with_clip_rect(rect);
        painter.rect_filled(rect, 0., self.canvas_background());
        let projection_panel = self.projection_toolbar(ui.ctx(), rect);
        let legend_clicked = self.pick_legend_band(rect, &response, ui.input(|i| i.modifiers));
        self.prune_selection();
        if self.spatial.building_revision != self.building_sync.revision {
            self.spatial.cache_reset();
            self.spatial.building_revision = self.building_sync.revision;
        }
        let mesh = self.fe_mesh.as_ref();
        let stiffness_revision = self.stiffness_sources.revision;
        let stiffness_changed = self.spatial.stiffness_revision != stiffness_revision;
        let stale = self.spatial.cache.as_ref().is_none_or(
            |(generation, settings, contours, meshed, _)| {
                let constraints_changed = settings.enabled != self.support_height.enabled
                    || settings.direction != self.support_height.direction
                    || settings.height_mm != self.support_height.height_mm
                    || settings.automatic_building_bottom
                        != self.support_height.automatic_building_bottom
                    || settings.automatic_building_top
                        != self.support_height.automatic_building_top
                    || settings.building_bottom_mask != self.support_height.building_bottom_mask
                    || settings.building_top_mask != self.support_height.building_top_mask;
                constraints_changed
                    || *generation != self.mesh_generation
                    || self
                        .spatial
                        .scene_error
                        .as_ref()
                        .map_or(*meshed, |(requested, _)| *requested)
                        != mesh.is_some()
                    || if mesh.is_some() {
                        stiffness_changed
                            || settings.rigid_columns != self.support_height.rigid_columns
                            || settings.length_method != self.support_height.length_method
                            || contours != &self.contours
                    } else {
                        !same_source_geometry(contours, &self.contours)
                    }
            },
        );
        if !stale {
            let mut attributes_changed = false;
            if let Some((_, settings, contours, meshed, scene)) = &mut self.spatial.cache {
                if !*meshed
                    && (stiffness_changed
                        || settings.rigid_columns != self.support_height.rigid_columns
                        || settings.length_method != self.support_height.length_method)
                {
                    for beam in &mut scene.beams {
                        if let Some(section) = self
                            .support_height
                            .beams
                            .get(&beam.id)
                            .or_else(|| self.support_height.columns.get(&beam.id))
                        {
                            beam.section = section.clone();
                        }
                    }
                    *settings = self.support_height.clone();
                    self.spatial.stiffness_revision = stiffness_revision;
                    attributes_changed = true;
                }
                if contours.len() != self.contours.len()
                    || !contours
                        .iter()
                        .map(|c| c.id)
                        .eq(self.contours.iter().map(|c| c.id))
                {
                    *contours = self.contours.clone();
                    self.spatial.source_index = self
                        .contours
                        .iter()
                        .enumerate()
                        .map(|(i, c)| (c.id, i))
                        .collect();
                    attributes_changed = true;
                }
            }
            if attributes_changed {
                self.spatial.source_revision = self.spatial.source_revision.wrapping_add(1);
                self.spatial.visibility_reset();
            }
        }
        if stale {
            self.spatial.source_revision = self.spatial.source_revision.wrapping_add(1);
            self.spatial.source_index = self
                .contours
                .iter()
                .enumerate()
                .map(|(i, c)| (c.id, i))
                .collect();
            self.spatial.point_values.borrow_mut().take();
            self.spatial.point_geometry.borrow_mut().take();
            self.spatial.bounds = self.spatial.edit_camera_bounds;
            self.spatial.depth_cache.borrow_mut().take();
            self.spatial.world_cache.borrow_mut().take();
            self.spatial.deformation_cache.borrow_mut().take();
            let source = || {
                if self.building.enabled {
                    building_source_scene(&self.contours, &self.support_height, &self.building)
                } else {
                    source_scene(&self.contours, &self.support_height)
                }
            };
            let scene = if let Some(mesh) = mesh {
                shell::scene(
                    mesh,
                    &vec![Material::default(); mesh.elements.len()],
                    &self.support_height,
                    self.mesh_built_size,
                    &self.contours,
                )
            } else {
                source()
            };
            self.spatial.scene_error = None;
            let (mut scene, meshed) = match scene {
                Ok(scene) => (scene, mesh.is_some()),
                Err(error) => {
                    let fallback = if mesh.is_some() {
                        source().unwrap_or_default()
                    } else {
                        shell::Scene::default()
                    };
                    self.status.clone_from(&error);
                    self.spatial.scene_error = Some((mesh.is_some(), error));
                    (fallback, false)
                }
            };
            if scene.xyz.is_empty() {
                let z = if self.building.enabled {
                    self.building.floor(self.building.active).elevation_mm
                } else {
                    0.
                };
                let bounds = self.bounds().unwrap_or([0., 0., 10000., 10000.]);
                scene
                    .xyz
                    .extend([[bounds[0], bounds[1], z], [bounds[2], bounds[3], z]]);
            }
            self.spatial.source_surface_owners = if meshed {
                BTreeMap::new()
            } else {
                self.source_surface_owner_index(&scene)
            };
            self.spatial.cache = Some((
                self.mesh_generation,
                self.support_height.clone(),
                self.contours.clone(),
                meshed,
                scene,
            ));
            self.spatial.stiffness_revision = stiffness_revision;
            ui.ctx().request_repaint();
        }
        if self.spatial.scene_error.is_some() {
            painter.text(
                rect.center_bottom() - egui::vec2(0., 12.),
                egui::Align2::CENTER_BOTTOM,
                "Ошибка построения 3D. Подробности в левой панели.",
                egui::FontId::proportional(14.),
                Color32::LIGHT_RED,
            );
        }
        let mesh = mesh.filter(|_| self.spatial.cache.as_ref().is_some_and(|cache| cache.3));
        let camera_locked = self.drag.is_some() || self.arc_placement.is_some();
        if !camera_locked {
            // Consume the last geometry update before releasing the fit lock.
            self.spatial.edit_camera_bounds = None;
        }
        if !camera_locked && response.hovered() {
            let scroll = ui.input(|i| i.smooth_scroll_delta.y);
            let old = self.spatial.zoom;
            let next = (old * (scroll as f64 * 0.002).exp()).clamp(0.02, 100.);
            if let Some(at) = response.hover_pos() {
                self.spatial.zoom_to(rect, at, next);
            }
        }
        let delta = ui.input(|i| i.pointer.delta());
        if !camera_locked && response.dragged_by(egui::PointerButton::Secondary) {
            self.spatial.yaw += delta.x as f64 * 0.007;
            self.spatial.pitch =
                (self.spatial.pitch + delta.y as f64 * 0.007).rem_euclid(std::f64::consts::TAU);
        }
        if !camera_locked && response.dragged_by(egui::PointerButton::Middle) {
            self.spatial.pan += delta;
        }
        if !camera_locked && response.double_clicked_by(egui::PointerButton::Middle) {
            self.spatial.fit_view();
        }
        let now = ui.input(|i| i.time);
        if !camera_locked
            && (response.dragged_by(egui::PointerButton::Secondary)
                || response.dragged_by(egui::PointerButton::Middle)
                || (response.hovered() && ui.input(|i| i.smooth_scroll_delta.y.abs() > 0.)))
        {
            self.spatial.camera_busy_until = now + 0.15;
            ui.ctx().request_repaint_after(Duration::from_millis(160));
        }
        let camera_busy = now < self.spatial.camera_busy_until;
        self.fe_selection.refresh(self.mesh_generation);
        let scene = &self.spatial.cache.as_ref().unwrap().4;
        if scene.xyz.is_empty() {
            painter.text(
                rect.center(),
                egui::Align2::CENTER_CENTER,
                "Добавьте объекты для просмотра в 3D",
                egui::FontId::proportional(16.),
                Color32::LIGHT_GRAY,
            );
            return;
        }
        let construction_only = mesh.is_none()
            && self.contours.is_empty()
            && self.drawing.is_none()
            && self.underlays.is_empty();
        if construction_only {
            self.spatial.bounds = Some(self.spatial_construction_bounds());
        } else if self.spatial.bounds.is_none() {
            // The source scene intentionally retains hidden geometry. Fit the draw
            // list, not its complete XYZ buffer (isolation/storeys do not rebuild it).
            let draw = self.spatial_draw_list(scene);
            let mut lo = [f64::INFINITY; 3];
            let mut hi = [f64::NEG_INFINITY; 3];
            let mut include = |p: [f64; 3]| {
                if p.iter().all(|v| v.is_finite()) {
                    for a in 0..3 {
                        lo[a] = lo[a].min(p[a]);
                        hi[a] = hi[a].max(p[a]);
                    }
                }
            };
            for &n in &draw.nodes {
                include(scene.xyz[n]);
            }
            // Source-only overlays (including foundations) have no shell cells.
            for c in self.contours.iter().filter(|c| self.contour_visible(c)) {
                if c.kind == Kind::Beam {
                    for edge in self.source_beam_path(c).windows(2) {
                        let [a, b] = [edge[0], edge[1]];
                        if let Some(interval) = self.sections.segment_interval([a, b]) {
                            for t in interval {
                                include(std::array::from_fn(|i| a[i] + (b[i] - a[i]) * t));
                            }
                        }
                    }
                    continue;
                }
                for z in self.source_heights(c) {
                    for p in &c.points {
                        include([p[0], p[1], z]);
                    }
                }
            }
            if self.intersections.show && self.intersections.current {
                for line in &self.intersections.graph.lines {
                    if line
                        .owners
                        .iter()
                        .any(|&id| self.intersection_owner_visible(id))
                    {
                        for p in line.ends {
                            include(p);
                        }
                    }
                }
                for (p, ids) in &self.intersections.graph.points {
                    if ids.iter().any(|&id| self.intersection_owner_visible(id)) {
                        include(*p);
                    }
                }
            }
            let bounds = if lo[0].is_finite() {
                let center = std::array::from_fn(|a| lo[a] + (hi[a] - lo[a]) * 0.5);
                let extent = (0..3)
                    .map(|a| (hi[a] - lo[a]).powi(2))
                    .sum::<f64>()
                    .sqrt()
                    .max(1.);
                (center, extent)
            } else {
                self.spatial_construction_bounds()
            };
            self.spatial.bounds = Some(bounds);
        }
        let (center, extent) = self.spatial.bounds.unwrap();
        let scale = rect.width().min(rect.height()) as f64 * 0.85 * self.spatial.zoom / extent;
        let (sy, cy) = self.spatial.yaw.sin_cos();
        let (sp, cp) = self.spatial.pitch.sin_cos();
        let origin = rect.center() + self.spatial.pan;
        let project = |p: [f64; 3]| {
            let x = p[0] - center[0];
            let y = p[1] - center[1];
            let z = p[2] - center[2];
            let a = cy * x - sy * y;
            let b = sy * x + cy * y;
            let v = cp * b - sp * z;
            let depth = sp * b + cp * z;
            (
                origin + egui::vec2((a * scale) as f32, (-v * scale) as f32),
                depth,
            )
        };
        let draw = self.spatial_draw_list(scene);
        let mut projected = std::mem::take(&mut self.spatial.projected);
        projected.resize(scene.xyz.len(), (Pos2::new(f32::NAN, f32::NAN), f64::NAN));
        projected.truncate(scene.xyz.len());
        if self
            .spatial
            .projected_owner
            .as_ref()
            .is_none_or(|old| !std::sync::Arc::ptr_eq(old, &draw))
        {
            projected.fill((Pos2::new(f32::NAN, f32::NAN), f64::NAN));
            self.spatial.projected_owner = Some(draw.clone());
        }
        for &n in &draw.nodes {
            projected[n] = project(scene.xyz[n]);
        }
        if draw.nodes.is_empty() {
            projected[0] = project(scene.xyz[0]);
        }
        let deformation_only = ui
            .ctx()
            .data(|d| d.get_temp::<bool>(egui::Id::new("deformation_only")))
            .unwrap_or(false);
        let shape_depth_margin = self
            .displayed_shape(ui.ctx(), true)
            .or_else(|| self.displayed_shape(ui.ctx(), false))
            .filter(|view| mesh.is_some() && view.shape.len() == scene.xyz.len())
            .map_or(0., |view| view.amplitude.abs());
        if deformation_only {
            extend_shape_depth(&mut projected, shape_depth_margin);
        }
        if deformation_only
            && self.paint_deformation_view(
                ui, &painter, rect, scene, &projected, &project, center, extent, scale,
            )
        {
            self.shape_legend(&painter, rect);
            projected.truncate(scene.xyz.len());
            self.spatial.projected = projected;
            return;
        }
        projected.truncate(scene.xyz.len());
        // Rigid offsets can move the rendered beam axis outside its FE endpoints.
        for beam in draw.beams.iter().map(|&i| &scene.beams[i]) {
            if beam.rigid_offset_mm != [0.; 2] {
                let arm = beam.rigid_arm(&scene.xyz).unwrap_or([0.; 3]);
                projected.extend(
                    beam.nodes
                        .map(|n| project(std::array::from_fn(|a| scene.xyz[n][a] + arm[a]))),
                );
            }
        }
        // Include load glyphs and extrusions in the shared depth range.
        for c in self.spatial_load_sources().iter() {
            let delta = spatial_loads::offset(self, c);
            let tail = |p: [f64; 3]| std::array::from_fn(|axis| p[axis] + delta[axis]);
            if c.kind == Kind::LoadLine {
                for p in self.spatial_line_load_path(c) {
                    projected.extend([project(p), project(tail(p))]);
                }
                continue;
            }
            let z = self.source_heights(c)[0];
            for &[x, y] in &c.points {
                let p = [x, y, z];
                projected.extend([project(p), project(tail(p))]);
                if c.kind == Kind::LoadPoint {
                    let radius = if c.load.as_ref().is_some_and(|l| l.moment) {
                        235.
                    } else {
                        50.
                    };
                    for dx in [-radius, radius] {
                        for dy in [-radius, radius] {
                            for dz in [-radius, radius] {
                                projected.push(project([x + dx, y + dy, z + dz]));
                            }
                        }
                    }
                }
            }
        }
        for c in self
            .contours
            .iter()
            .filter(|c| matches!(c.kind, Kind::Column | Kind::RigidLink) && self.contour_visible(c))
        {
            if c.kind == Kind::RigidLink {
                if let Some(face) = spatial_world::rigid_link_face(self, c) {
                    projected.extend(face.map(project));
                }
            } else if let (Some(_), Some(section)) =
                (c.points.first(), self.support_height.columns.get(&c.id))
            {
                let q = crate::plan_trace::columns::section_corners(c, section);
                for z in self.source_heights(c) {
                    projected.extend(q.map(|p| project([p[0], p[1], z])));
                }
            }
        }
        // Ground and structure must use the same depth range, even when the
        // ground extends far beyond the structural model.
        if let Some(corners) = self.ground_depth_corners() {
            projected.extend(corners.map(project));
        }
        let visible_depth_nodes = if draw.nodes.is_empty() {
            &[0]
        } else {
            draw.nodes.as_slice()
        };
        let mut depth_points = visible_depth_nodes
            .iter()
            .map(|&n| projected[n])
            .chain(projected[scene.xyz.len()..].iter().copied())
            .collect::<Vec<_>>();
        extend_shape_depth(&mut depth_points, shape_depth_margin);
        let cut_matrix = spatial_world::camera(
            rect,
            &depth_points,
            extent,
            scale,
            self.spatial.yaw,
            self.spatial.pitch,
            self.spatial.pan,
        );
        spatial_depth::set_clips(self.sections.screen_planes(cut_matrix, center, extent));
        spatial_depth::set_clip_depth(rect, &depth_points);
        self.paint_spatial_construction_grid(&painter, rect, &project, &projected);
        let result_range = mesh
            .filter(|_| self.punching_view.is_none())
            .and_then(|_| self.selected_result())
            .and_then(|_| self.spatial_result_range());
        let result_scale = &self.result_scales[self.result_field];
        let pointer = response.hover_pos();
        let mut result_hit: Option<spatial_results::SpatialHit> = None;
        let mut occupied_labels = std::collections::BTreeSet::new();
        let mut result_labels = Vec::new();
        let faces = draw
            .surfaces
            .iter()
            .map(|&i| (i, &scene.surfaces[i]))
            .filter(|_| {
                !camera_busy
                    && self.show_results
                    && self.punching_view.is_none()
                    && !self.point_displacements()
                    && self.result_field != 6
                    && (self.show_result_values || pointer.is_some())
            })
            .filter(|(_, s)| {
                self.show_result_values
                    || pointer.is_some_and(|p| {
                        let mut bounds = Rect::NOTHING;
                        for &n in &s.nodes {
                            let at = projected[n].0;
                            bounds = bounds.union(Rect::from_min_max(at, at));
                        }
                        bounds.contains(p)
                    })
            })
            .map(|(index, s)| {
                let depth =
                    s.nodes.iter().map(|&n| projected[n].1).sum::<f64>() / s.nodes.len() as f64;
                (depth, index, s)
            })
            .collect::<Vec<_>>();

        for (depth, surface_index, s) in faces {
            if !self.show_results {
                continue;
            }
            let points = s.nodes.iter().map(|&n| projected[n].0).collect::<Vec<_>>();
            if points.iter().any(|p| !p.x.is_finite() || !p.y.is_finite()) {
                continue;
            }
            let wants_values = self.show_results
                && (self.show_result_values
                    || pointer.is_some_and(|p| Rect::from_points(&points).contains(p)));
            let values = if wants_values {
                result_range.and_then(|_| self.spatial_surface_values(surface_index, s))
            } else {
                None
            };
            if let (Some(values), Some(_range)) = (&values, result_range) {
                if self.show_result_values {
                    let at = points
                        .iter()
                        .fold(egui::Vec2::ZERO, |sum, p| sum + p.to_vec2())
                        / points.len() as f32;
                    result_labels.push((
                        depth,
                        Pos2::new(at.x, at.y),
                        values.iter().sum::<f64>() / values.len() as f64,
                        egui::Vec2::ZERO,
                    ));
                }
            }
            if let Some(pointer) = pointer {
                let bounds = points
                    .iter()
                    .fold(Rect::NOTHING, |r, &p| r.union(Rect::from_min_max(p, p)));
                if bounds.contains(pointer) {
                    let projected_face = s.nodes.iter().map(|&n| projected[n]).collect::<Vec<_>>();
                    let pick_values = values.clone().unwrap_or_else(|| vec![0.; s.nodes.len()]);
                    for (triangle, corner_values) in
                        spatial_results::surface_triangles(&projected_face, &pick_values)
                    {
                        if let Some((weights, hit_depth)) =
                            spatial_results::pick_triangle(pointer, triangle)
                        {
                            if result_hit.as_ref().is_none_or(|hit| hit_depth > hit.depth) {
                                let text = values.as_ref().map(|_| {
                                    let value = (0..3).map(|j| weights[j] * corner_values[j]).sum();
                                    let owner = if let Some(support) = s.support {
                                        format!("Стена {support} (X вдоль, Y вверх)")
                                    } else {
                                        format!(
                                            "КЭ {}",
                                            s.display
                                                .first()
                                                .and_then(|&i| mesh.and_then(|m| m.elements.get(i)))
                                                .map_or(0, |e| e.id)
                                        )
                                    };
                                    self.spatial_result_text(&owner, value)
                                });
                                result_hit = Some(spatial_results::SpatialHit {
                                    depth: hit_depth,
                                    text,
                                });
                            }
                        }
                    }
                }
            }
        }
        let wants_beam_results = self.show_results
            && self.punching_view.is_none()
            && (17..=22).contains(&self.result_field);
        let beam_results = self
            .selected_result()
            .filter(|_| mesh.is_some() && wants_beam_results)
            .and_then(|r| r.shell.as_ref())
            .map(|s| s.beam_forces.as_slice())
            .unwrap_or_default();
        // The solver recovers one force record per scene beam, in scene order.
        let beam_force = |index: usize| {
            beam_results
                .get(index)
                .filter(|force| force.id == scene.beams[index].id)
        };
        for &index in draw
            .beams
            .iter()
            .filter(|_| wants_beam_results && (self.show_result_values || pointer.is_some()))
        {
            let beam = &scene.beams[index];
            let points = beam_screen_points(beam, &scene, &projected, &project);
            if points
                .iter()
                .any(|p| !p.0.x.is_finite() || !p.0.y.is_finite())
            {
                continue;
            }
            let recovered = beam_force(index);
            let component = self.result_field.checked_sub(17).filter(|c| *c < 6);
            if let Some(value) = recovered
                .zip(component)
                .map(|(f, c)| f.at(0.5)[c] * self.force_result_factor())
            {
                if self.show_result_values {
                    for (t, section) in recovered.unwrap().sections() {
                        let at = points[0].0 + (points[1].0 - points[0].0) * t as f32;
                        let depth = points[0].1 + (points[1].1 - points[0].1) * t;
                        result_labels.push((
                            depth,
                            at,
                            section[component.unwrap()] * self.force_result_factor(),
                            // Separate the two section values at a shared joint:
                            // move each end label towards its own member centre.
                            if t == 0. {
                                {
                                    let d = points[1].0 - points[0].0;
                                    d * (22. / d.length().max(1.))
                                }
                            } else if t == 1. {
                                {
                                    let d = points[0].0 - points[1].0;
                                    d * (22. / d.length().max(1.))
                                }
                            } else {
                                egui::Vec2::ZERO
                            },
                        ));
                    }
                }
                if let Some(pointer) = pointer {
                    let d = points[1].0 - points[0].0;
                    let t =
                        ((pointer - points[0].0).dot(d) / d.length_sq().max(1e-10)).clamp(0., 1.);
                    let at = points[0].0 + d * t;
                    let depth = points[0].1 + (points[1].1 - points[0].1) * t as f64;
                    if pointer.distance(at) <= 7.
                        && result_hit.as_ref().is_none_or(|h| depth >= h.depth - 1e-6)
                    {
                        let value = recovered.unwrap().at(t as f64)[component.unwrap()]
                            * self.force_result_factor();
                        result_hit = Some(spatial_results::SpatialHit {
                            depth,
                            text: Some(self.spatial_result_text(
                                &format!(
                                    "{} {}",
                                    if self.support_height.columns.contains_key(&beam.id) {
                                        "Колонна"
                                    } else {
                                        "Балка"
                                    },
                                    beam.id
                                ),
                                value,
                            )),
                        });
                    }
                }
            }
        }
        let mut point_occlusion = None;
        if let Some(range) = result_range.filter(|_| self.show_results) {
            if !camera_busy && (self.show_result_values || pointer.is_some()) {
                for (node, value) in self
                    .spatial_visible_point_results(scene)
                    .iter()
                    .copied()
                    .filter(|(n, _)| self.spatial_node_view_visible(*n, scene))
                {
                    let (point, depth) = projected[node];
                    if !rect.contains(point) {
                        continue;
                    }
                    if self.show_result_values {
                        result_labels.push((depth, point, value, egui::Vec2::ZERO));
                    }
                    if pointer.is_some_and(|p| p.distance(point) <= self.point_size.max(9.))
                        && result_hit.as_ref().is_none_or(|h| depth >= h.depth - 1e-6)
                        && point_occlusion
                            .get_or_insert_with(|| {
                                spatial_results::Occlusion::new_visible(
                                    rect, scene, &projected, self,
                                )
                            })
                            .visible(point, depth)
                    {
                        result_hit = Some(spatial_results::SpatialHit {
                            depth,
                            text: Some(
                                self.spatial_result_text(&format!("Узел {}", node + 1), value),
                            ),
                        });
                    }
                }
            }
            if matches!(self.result_field, 8 | 9) {
                for shape in self.load_mosaic_shapes().iter() {
                    if result_scale.color(shape.value, range).is_none() {
                        continue;
                    }
                    let (at, depth, line) = match &shape.geometry {
                        load_mosaics::Geometry::Point(p) => {
                            let (at, depth) = project([p[0], p[1], shape.z]);
                            (at, depth, None)
                        }
                        load_mosaics::Geometry::Line([a, b]) => {
                            let a = project([a[0], a[1], shape.z]);
                            let b = project([b[0], b[1], shape.end_z.unwrap_or(shape.z)]);
                            (a.0 + (b.0 - a.0) * 0.5, (a.1 + b.1) * 0.5, Some([a.0, b.0]))
                        }
                    };
                    if self.show_result_values {
                        result_labels.push((depth, at, shape.value, egui::Vec2::ZERO));
                    }
                    if let Some(pointer) = pointer {
                        let distance = if let Some([a, b]) = line {
                            let d = b - a;
                            let t = ((pointer - a).dot(d) / d.length_sq().max(1e-10)).clamp(0., 1.);
                            pointer.distance(a + d * t)
                        } else {
                            pointer.distance(at)
                        };
                        if distance <= 9.
                            && result_hit.as_ref().is_none_or(|h| depth >= h.depth - 1e-6)
                        {
                            result_hit = Some(spatial_results::SpatialHit {
                                depth,
                                text: Some(self.spatial_result_text("Нагрузка", shape.value)),
                            });
                        }
                    }
                }
            }
        }
        let explicit_restraints = self
            .contours
            .iter()
            .any(|c| c.kind.is_restraint() && self.contour_visible(c));
        let occlusion = (explicit_restraints
            || !result_labels.is_empty()
            || (self.length_view != 0 && self.show_result_values && !self.show_results))
            .then(|| spatial_results::Occlusion::new_visible(rect, scene, &projected, self));
        let mut scene_labels = Vec::new();
        result_labels.sort_by(|a, b| b.0.total_cmp(&a.0));
        for (depth, point, value, offset) in result_labels {
            if !rect.contains(point) || !value.is_finite() {
                continue;
            }
            let bin = (
                ((point.x + offset.x) / 64.).floor() as i32,
                ((point.y + offset.y) / 24.).floor() as i32,
            );
            if (offset == egui::Vec2::ZERO && occupied_labels.contains(&bin))
                || occlusion.as_ref().is_some_and(|o| !o.visible(point, depth))
            {
                continue;
            }
            occupied_labels.insert(bin);
            let galley = painter.layout_no_wrap(
                self.display_text.number(value),
                egui::FontId::proportional(self.display_text.size),
                self.display_text.ink(),
            );
            let at = point + offset + egui::vec2(4., -galley.size().y - 3.);
            let area = Rect::from_min_size(at, galley.size());
            scene_labels.push((
                depth,
                [
                    egui::Shape::rect_filled(area.expand(2.), 2., Color32::from_rgb(20, 24, 30)),
                    egui::Shape::galley(at, galley, self.display_text.ink()),
                ],
            ));
        }
        {
            let mut key = vec![
                self.spatial.yaw.to_bits(),
                self.spatial.pitch.to_bits(),
                rect.left().to_bits() as u64,
                rect.top().to_bits() as u64,
                rect.width().to_bits() as u64,
                rect.height().to_bits() as u64,
                self.spatial_style_key(),
            ];
            key.extend(self.view_key());
            key.extend(result_scale.signature());
            key.extend([
                self.show_results as u64,
                self.show_loads as u64,
                self.show_mesh as u64,
                self.slab_opacity.to_bits() as u64,
                self.point_size.to_bits() as u64,
                self.length_view as u64,
                self.length_axis_y as u64,
                self.stiffness_variant as u64,
            ]);
            key.extend(center.map(f64::to_bits));
            key.push(extent.to_bits());
            let mut world_key = key[6..].to_vec();
            world_key.push(self.reinforcement_directions as u64);
            if let Some(index) = self.length_scale_index() {
                world_key.extend(self.length_scales[index].signature());
            }
            world_key.extend(
                self.selected
                    .iter()
                    .chain(
                        self.selection_ids
                            .iter()
                            .filter(|&&id| Some(id) != self.selected),
                    )
                    .copied()
                    .filter(|&id| {
                        self.view_contour(id)
                            .is_some_and(|c| matches!(c.kind, Kind::LoadPoint | Kind::Column))
                    }),
            );
            if let Some(range) = result_range {
                world_key.extend(range.map(f64::to_bits));
            }
            let world =
                spatial_world::geometry(self, scene, center, extent, result_range, world_key);
            // The shader expands edge tangents in physical screen pixels.
            spatial_depth::paint_world(&painter, rect, world, cut_matrix);
            if self.show_results {
                if let Some(range) = result_range {
                    spatial_points::paint(
                        self, &painter, rect, scene, center, extent, range, cut_matrix,
                    );
                }
            }
            self.spatial_underlays(&painter, rect, &project, &projected, scene);
            key.extend(cut_matrix.map(|column| u64::from(column[2].to_bits())));
            // Selection changes overlay ink, not the cached world surfaces.
            key.push(self.selected.unwrap_or(0));
            key.push(self.spatial.restraint_hover.unwrap_or(0));
            key.extend(self.selection_ids.iter().copied());
            if explicit_restraints {
                key.extend([
                    self.spatial.zoom.to_bits(),
                    self.spatial.pan.x.to_bits() as u64,
                    self.spatial.pan.y.to_bits() as u64,
                ]);
            }
            let cached = self
                .spatial
                .depth_cache
                .borrow()
                .as_ref()
                .filter(|old| old.key == key)
                .map(|old| (old.vertices.clone(), old.zoom, old.pan));
            let (vertices, base_zoom, base_pan) = if let Some(vertices) = cached {
                vertices
            } else {
                let mut builder = spatial_depth::Builder::new(rect, &depth_points);

                for contour in self.contours.iter().filter(|c| {
                    (mesh.is_some() || !spatial_world::source_outline_kind(c.kind))
                        && (matches!(
                            c.kind,
                            Kind::Slab
                                | Kind::Hole
                                | Kind::Support
                                | Kind::Triangulation
                                | Kind::Restraint
                                | Kind::PointRestraint
                                | Kind::RigidLink
                        ) || c.kind.is_load())
                        && self.contour_visible(c)
                        && (c.kind != Kind::Slab || self.drawing_visibility.contours[0])
                }) {
                    let mut points = contour.points.clone();
                    if contour.closed() && !points.is_empty() {
                        points.push(points[0]);
                    }
                    let heights = self.source_heights(contour);
                    if contour.kind == Kind::PointRestraint {
                        let [x, y] = points[0];
                        let world = [x, y, heights[0]];
                        if self.sections.contains(world) {
                            let (p, d) = project(world);
                            let color = if self.is_selected(contour.id)
                                || self.spatial.restraint_hover == Some(contour.id)
                            {
                                Color32::YELLOW
                            } else {
                                contour.kind.color()
                            };
                            let tip = (p, d);
                            let left = (p + egui::vec2(-5., 7.), d);
                            let right = (p + egui::vec2(5., 7.), d);
                            for edge in [[tip, left], [left, right], [right, tip]] {
                                builder.line(edge, 1.7, color);
                            }
                        }
                        continue;
                    }
                    if matches!(contour.kind, Kind::Support | Kind::RigidLink)
                        && self.drawing_visibility.border(contour.kind)
                    {
                        if let (Some(&bottom), Some(&top)) = (heights.first(), heights.last()) {
                            if (top - bottom).abs() > 1e-6 {
                                for p in &contour.points {
                                    builder.line(
                                        [project([p[0], p[1], bottom]), project([p[0], p[1], top])],
                                        1.5,
                                        self.object_ink(contour.id, contour.kind),
                                    );
                                }
                            }
                        }
                    }
                    for z in heights {
                        if contour.kind == Kind::Triangulation {
                            if let Some(grid) = &contour.grid {
                                let origin = project([grid.origin[0], grid.origin[1], z]).0;
                                let x = grid.at(1., 0.);
                                let y = grid.at(0., 1.);
                                let x = project([x[0], x[1], z]).0 - origin;
                                let y = project([y[0], y[1], z]).0 - origin;
                                if (x.x * y.y - x.y * y.x).abs() >= 36. {
                                    if let Some(camera) = spatial_edit::plane_camera(&project, z) {
                                        let corners = [
                                            rect.left_top(),
                                            rect.right_top(),
                                            rect.right_bottom(),
                                            rect.left_bottom(),
                                        ]
                                        .map(|p| grid.coordinates(camera.world(p, rect)));
                                        let bounds = |axis: usize, max: usize| {
                                            let lo = corners
                                                .iter()
                                                .map(|p| p[axis])
                                                .fold(f64::INFINITY, f64::min)
                                                .floor()
                                                .clamp(0., max as f64)
                                                as usize;
                                            let hi = corners
                                                .iter()
                                                .map(|p| p[axis])
                                                .fold(f64::NEG_INFINITY, f64::max)
                                                .ceil()
                                                .clamp(0., max as f64)
                                                as usize;
                                            (lo, hi)
                                        };
                                        let (x0, x1) = bounds(0, grid.nx);
                                        let (y0, y1) = bounds(1, grid.ny);
                                        let ink = if self.is_selected(contour.id) {
                                            Color32::YELLOW
                                        } else {
                                            self.object_ink(contour.id, contour.kind)
                                        };
                                        for i in x0..=x1 {
                                            for j in y0..=y1 {
                                                let p = grid.at(i as f64, j as f64);
                                                let q = project([p[0], p[1], z]);
                                                if !rect.contains(q.0) {
                                                    continue;
                                                }
                                                let at =
                                                    |x: f32, y: f32| (q.0 + egui::vec2(x, y), q.1);
                                                builder.triangle(
                                                    [at(-1.5, -1.5), at(1.5, -1.5), at(1.5, 1.5)],
                                                    ink,
                                                    0.00002,
                                                );
                                                builder.triangle(
                                                    [at(-1.5, -1.5), at(1.5, 1.5), at(-1.5, 1.5)],
                                                    ink,
                                                    0.00002,
                                                );
                                                builder.anchor_recent(6, &[q], 0.00002);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                        if contour.kind == Kind::LoadArea {
                            let triangles = self.cached_load_fill(contour);
                            let ink = if self.is_selected(contour.id) {
                                Color32::YELLOW
                            } else {
                                self.object_ink(contour.id, contour.kind)
                            };
                            for &edge in self.cached_load_boundary(contour.id, &triangles).iter() {
                                let base = edge.map(|p| project([p[0], p[1], z]));
                                let top = edge.map(|p| project([p[0], p[1], z + 250.]));
                                builder.line(base, 1.3, ink);
                                builder.line(top, 1.3, ink);
                                builder.line([base[0], top[0]], 1., ink);
                            }
                            continue;
                        }
                        for (index, edge) in points.windows(2).enumerate() {
                            let ends = [
                                project([edge[0][0], edge[0][1], z]),
                                project([edge[1][0], edge[1][1], z]),
                            ];
                            if matches!(
                                contour.kind,
                                Kind::LoadArea | Kind::LoadHole | Kind::LoadLine
                            ) {
                                let [dx, dy, dz] = spatial_loads::offset(self, contour);
                                let top = [edge[0], edge[1]]
                                    .map(|p| project([p[0] + dx, p[1] + dy, z + dz]));
                                let color = if self.is_selected(contour.id) {
                                    Color32::YELLOW
                                } else {
                                    self.object_ink(contour.id, contour.kind)
                                };
                                builder.line(top, 1.3, color);
                                builder.line([ends[0], top[0]], 1., color);
                                builder.line([ends[1], top[1]], 1., color);
                            }
                            let is_sum = punching::is_punching(contour);
                            let ink = if contour.kind == Kind::Restraint
                                && (self.is_selected(contour.id)
                                    || self.spatial.restraint_hover == Some(contour.id))
                            {
                                Color32::YELLOW
                            } else if (is_sum
                                || contour.kind.is_load()
                                || contour.kind == Kind::Triangulation)
                                && self.is_selected(contour.id)
                            {
                                Color32::YELLOW
                            } else {
                                self.object_ink(contour.id, contour.kind)
                            };
                            let inactive = is_sum
                                && contour
                                    .refinement
                                    .as_ref()
                                    .is_some_and(|r| r.inactive_edges.contains(&index));
                            if inactive {
                                let length = ends[0].0.distance(ends[1].0);
                                let at = |t: f32| {
                                    (
                                        ends[0].0 + (ends[1].0 - ends[0].0) * t,
                                        ends[0].1 + (ends[1].1 - ends[0].1) * t as f64,
                                    )
                                };
                                let mut x = 0_f32;
                                while x < length {
                                    builder.line(
                                        [at(x / length), at((x + 5.).min(length) / length)],
                                        1.5,
                                        ink,
                                    );
                                    x += 9.;
                                }
                            } else {
                                if contour.kind == Kind::Restraint {
                                    let [a, b] = ends;
                                    let steps =
                                        (a.0.distance(b.0) / 2.).ceil().clamp(1., 4096.) as usize;
                                    let at =
                                        |t: f32| (a.0.lerp(b.0, t), a.1 + (b.1 - a.1) * t as f64);
                                    let mut start = None;
                                    for i in 0..steps {
                                        let p = at((i as f32 + 0.5) / steps as f32);
                                        let shown = rect.expand(2.).contains(p.0)
                                            && occlusion
                                                .as_ref()
                                                .is_none_or(|o| o.visible(p.0, p.1));
                                        if shown {
                                            start.get_or_insert(i);
                                        } else if let Some(lo) = start.take() {
                                            builder.line(
                                                [
                                                    at(lo as f32 / steps as f32),
                                                    at(i as f32 / steps as f32),
                                                ],
                                                1.5,
                                                ink,
                                            );
                                        }
                                    }
                                    if let Some(lo) = start {
                                        builder.line([at(lo as f32 / steps as f32), b], 1.5, ink);
                                    }
                                } else if contour.kind == Kind::Triangulation {
                                    let a = edge[0];
                                    let plane = [
                                        [a[0], a[1], z],
                                        [a[0] + 1000., a[1], z],
                                        [a[0], a[1] + 1000., z],
                                    ]
                                    .map(project);
                                    builder.line_on_face(ends, 1.5, ink, plane);
                                } else {
                                    builder.line(ends, 1.5, ink);
                                }
                            }
                        }
                    }
                }
                if self.drawing_visibility.rigid_bodies {
                    if let Some(data) = self.fe_mesh.as_ref().and_then(|m| m.building.as_ref()) {
                        if !data.rigid_links.is_empty() {
                            let visible = self
                                .contours
                                .iter()
                                .filter(|c| !self.object_hidden_without_sections(c))
                                .map(|c| c.id)
                                .collect::<std::collections::BTreeSet<_>>();
                            for station in &data.rigid_links {
                                if !visible.contains(&station.contour)
                                    || !visible.contains(&station.master.owner)
                                {
                                    continue;
                                }
                                let ink = if self.is_selected(station.contour) {
                                    Color32::YELLOW
                                } else {
                                    self.contour_color(Kind::RigidLink)
                                };
                                for slave in &station.slaves {
                                    if visible.contains(&slave.owner) {
                                        builder.line(
                                            [project(station.master.xyz), project(slave.xyz)],
                                            2.,
                                            ink,
                                        );
                                    }
                                }
                            }
                        }
                    }
                }
                if self.drawing_visibility.beam || self.drawing_visibility.columns {
                    for &index in &draw.beams {
                        let beam = &scene.beams[index];
                        if mesh.is_none() && beam.rigid_offset_mm == [0.; 2] {
                            continue;
                        }
                        let force = beam_force(index);
                        let component = self.result_field.checked_sub(17).filter(|c| *c < 6);
                        let endpoints = beam_screen_points(beam, &scene, &projected, &project);
                        if beam.rigid_offset_mm != [0.; 2] {
                            let a = scene.xyz[beam.nodes[0]];
                            let b = scene.xyz[beam.nodes[1]];
                            let length = (0..3).map(|i| (b[i] - a[i]).powi(2)).sum::<f64>().sqrt();
                            let count = (length / 500.).ceil().max(1.) as usize;
                            for i in 0..=count {
                                let t = i as f64 / count as f64;
                                let mix = |p: [(Pos2, f64); 2]| {
                                    (
                                        p[0].0 + (p[1].0 - p[0].0) * t as f32,
                                        p[0].1 + (p[1].1 - p[0].1) * t,
                                    )
                                };
                                builder.line(
                                    [mix(beam.nodes.map(|n| projected[n])), mix(endpoints)],
                                    1.5,
                                    Color32::from_rgb(255, 160, 30),
                                );
                            }
                        }
                        if mesh.is_none() {
                            continue;
                        }
                        let active = wants_beam_results && force.is_some();
                        let steps = if active { 16 } else { 1 };
                        let kind = if self.support_height.columns.contains_key(&beam.id) {
                            Kind::Column
                        } else {
                            Kind::Beam
                        };
                        let base_ink = self.object_ink(beam.id, kind);
                        for i in 0..steps {
                            let t0 = i as f64 / steps as f64;
                            let t1 = (i + 1) as f64 / steps as f64;
                            let at = |t: f64| {
                                (
                                    endpoints[0].0 + (endpoints[1].0 - endpoints[0].0) * t as f32,
                                    endpoints[0].1 + (endpoints[1].1 - endpoints[0].1) * t,
                                )
                            };
                            let ink = if self.show_results {
                                force.zip(component).zip(result_range).and_then(
                                    |((f, c), range)| {
                                        result_scale.color(
                                            f.at((t0 + t1) * 0.5)[c] * self.force_result_factor(),
                                            range,
                                        )
                                    },
                                )
                            } else {
                                None
                            };
                            // A masked result interval stays hidden; never paint
                            // the ordinary source colour over the result path.
                            if active && ink.is_none() {
                                continue;
                            }
                            let axis = [at(t0), at(t1)];
                            if self.drawing_visibility.structure_edges {
                                builder.line(axis, 6., spatial_world::STRUCTURE_EDGE_INK);
                            }
                            builder.line(axis, 4., ink.unwrap_or(base_ink));
                        }
                    }
                }
                if let Some(range) = result_range {
                    if self.show_results {
                        for shape in self.load_mosaic_shapes().iter() {
                            if let Some(ink) = result_scale.color(shape.value, range) {
                                match &shape.geometry {
                                    load_mosaics::Geometry::Line([a, b]) => builder.line(
                                        [
                                            project([a[0], a[1], shape.z]),
                                            project([b[0], b[1], shape.end_z.unwrap_or(shape.z)]),
                                        ],
                                        5.,
                                        ink,
                                    ),
                                    load_mosaics::Geometry::Point(p) => {
                                        let point = project([p[0], p[1], shape.z]);
                                        for i in 0..16 {
                                            let at = |j: usize| {
                                                let a = j as f32 * std::f32::consts::TAU / 16.;
                                                (
                                                    point.0
                                                        + egui::vec2(a.cos(), a.sin())
                                                            * self.point_size
                                                            * 0.5,
                                                    point.1,
                                                )
                                            };
                                            builder.triangle(
                                                [point, at(i), at(i + 1)],
                                                ink,
                                                0.00003,
                                            );
                                            builder.anchor_recent(3, &[point], 0.00003);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
                for &(node, mask) in &draw.fixed_nodes {
                    if !self.spatial_node_view_visible(node, scene) {
                        continue;
                    }
                    if let Some(&point) = projected.get(node) {
                        if occlusion
                            .as_ref()
                            .is_none_or(|o| o.visible(point.0, point.1))
                        {
                            builder.marker(point, mask & 3 == 3 && mask & 4 == 0);
                        }
                    }
                }
                let vertices = std::sync::Arc::new(builder.vertices);
                *self.spatial.depth_cache.borrow_mut() = Some(CachedDepth {
                    key,
                    vertices: vertices.clone(),
                    zoom: self.spatial.zoom,
                    pan: self.spatial.pan,
                });
                (vertices, self.spatial.zoom, self.spatial.pan)
            };
            let ratio = (self.spatial.zoom / base_zoom) as f32;
            let pan = self.spatial.pan - base_pan * ratio;
            spatial_depth::paint_transformed(
                &painter,
                rect,
                vertices,
                [
                    ratio,
                    ratio,
                    pan.x * 2. / rect.width(),
                    -pan.y * 2. / rect.height(),
                ],
            );
        }
        scene_labels.sort_by(|a, b| a.0.total_cmp(&b.0));
        painter.extend(scene_labels.into_iter().flat_map(|(_, parts)| parts));
        if result_range.is_some() {
            if let (Some(pointer), Some(text)) = (pointer, result_hit.and_then(|hit| hit.text)) {
                fem_ui::result_label(
                    &painter,
                    pointer + egui::vec2(16., 18.),
                    text,
                    rect,
                    self.display_text,
                );
            }
        }
        let length_legend = self.length_mosaic(
            &painter,
            rect,
            Some(&projected),
            occlusion.as_ref(),
            &project,
        );
        self.paint_deformation_view(
            ui,
            &painter,
            rect,
            scene,
            &depth_points,
            &project,
            center,
            extent,
            scale,
        );
        let help_y = if length_legend.is_some()
            || self
                .displayed_shape(ui.ctx(), true)
                .is_some_and(|v| v.active_sensitivity().is_some())
            || self
                .displayed_shape(ui.ctx(), false)
                .is_some_and(|v| v.vibration_values.is_some())
            || (self.show_results
                && (result_range.is_some() || self.punching_view.is_some_and(|f| f != 3)))
        {
            self.display_text.size + 58.
        } else {
            10.
        };
        painter.text(
            rect.left_top() + egui::vec2(10., help_y),
            egui::Align2::LEFT_TOP,
            "ПКМ — вращение · колесо — масштаб · СКМ — панорама",
            egui::FontId::proportional(12.),
            Color32::WHITE,
        );
        if !self.building.enabled {
            painter.text(
                rect.left_top() + egui::vec2(10., help_y + 18.),
                egui::Align2::LEFT_TOP,
                "Стены: X вдоль, Y вверх · закрепления: голубой XY, оранжевый XYZ",
                egui::FontId::proportional(12.),
                Color32::WHITE,
            );
        }
        let mut origin_axes = spatial_depth::Builder::new(rect, &depth_points);
        self.spatial_origin_axes(
            &painter,
            rect,
            &project,
            &projected,
            scene,
            scale,
            &mut origin_axes,
        );
        spatial_depth::paint_layer(
            &painter,
            rect,
            std::sync::Arc::new(origin_axes.vertices),
            spatial_depth::Layer::Axes,
        );
        self.column_overlay(rect, &project, &depth_points, &painter);
        if mesh.is_some() {
            spatial_world::paint_structure_edges(
                self,
                scene,
                &draw.surfaces,
                &painter,
                rect,
                &project,
                &depth_points,
            );
        }
        if !self.fe_isolated() {
            self.spatial_ground(&painter, rect, &project, &depth_points);
        }
        self.spatial_foundations(&painter, rect, &project, &projected, scene);
        if self.reinforcement_directions && self.show_result_values {
            if let Some((_, world)) = &*self.spatial.world_cache.borrow() {
                let mut occupied = std::collections::BTreeSet::new();
                let clipped = self.sections.active();
                for &(point, angle) in &world.direction_labels {
                    let (at, depth) = project(point);
                    if rect.contains(at)
                        && (!clipped || spatial_depth::point_visible(at, depth))
                        && occupied.insert(((at.x / 55.) as i32, (at.y / 20.) as i32))
                    {
                        painter.text(
                            at,
                            egui::Align2::LEFT_BOTTOM,
                            format!("{:.1}°", angle.to_degrees().rem_euclid(180.)),
                            egui::FontId::proportional(12.),
                            Color32::YELLOW,
                        );
                    }
                }
            }
        }
        self.spatial_intersections(&painter, rect, &project, &depth_points);
        self.paint_warning(&painter, |p| project(p).0);
        let axis_origin = rect.left_bottom() + egui::vec2(65., -65.);
        for (a, label, color) in [
            (0, "X", Color32::LIGHT_RED),
            (1, "Y", Color32::LIGHT_GREEN),
            (2, "Z", Color32::LIGHT_BLUE),
        ] {
            let mut p = center;
            p[a] += extent * 0.08;
            let vector = (project(p).0 - origin) / self.spatial.zoom as f32;
            painter.arrow(axis_origin, vector, egui::Stroke::new(2_f32, color));
            painter.text(
                axis_origin + vector,
                egui::Align2::CENTER_CENTER,
                label,
                egui::FontId::proportional(14.),
                color,
            );
        }
        let summation_legend = self.spatial_summation(&painter, rect, &project, &projected, scene);
        let load_label_hit = self.spatial_load_labels(&painter, rect, &project, &projected, scene);
        let foundation_occlusion = (occlusion.is_none()
            && self.mode.is_none()
            && self.contours.iter().any(|c| {
                c.kind == Kind::Foundation && self.is_selected(c.id) && self.contour_visible(c)
            }))
        .then(|| spatial_results::Occlusion::new_visible(rect, scene, &projected, self));
        let over_projection_panel = ui
            .input(|i| i.pointer.hover_pos())
            .is_some_and(|p| projection_panel.contains(p));
        if mesh.is_some() && !self.fe_selection.items.is_empty() {
            let mut selected = spatial_depth::Builder::new(rect, &depth_points);
            for &i in &draw.surfaces {
                let surface = &scene.surfaces[i];
                let key = surface.display.first().map_or((2, i), |&d| (0, d));
                let selected_quad = surface
                    .display
                    .iter()
                    .any(|&d| self.fe_selection.items.contains(&(0, d)));
                if self.fe_selection.items.contains(&key) || selected_quad {
                    let offset =
                        spatial_world::wall_half_offset(self, surface, &scene.xyz).map(|d| {
                            if project(d).1 < project([0.; 3]).1 {
                                d.map(|v| -v)
                            } else {
                                d
                            }
                        });
                    let at = |n: usize| {
                        offset.map_or(projected[n], |d| {
                            project(std::array::from_fn(|a| scene.xyz[n][a] + d[a]))
                        })
                    };
                    for j in 0..surface.nodes.len() {
                        let [a, b] = [
                            surface.nodes[j],
                            surface.nodes[(j + 1) % surface.nodes.len()],
                        ]
                        .map(at);
                        selected.line([a, b], 2.0_f32, Color32::YELLOW);
                    }
                }
            }
            for &i in &draw.beams {
                if self.fe_selection.items.contains(&(1, i)) {
                    selected.line(
                        scene.beams[i].nodes.map(|n| projected[n]),
                        3.0_f32,
                        Color32::YELLOW,
                    );
                }
            }
            spatial_depth::paint_layer(
                &painter,
                rect,
                std::sync::Arc::new(selected.vertices),
                spatial_depth::Layer::SelectedElements,
            );
        }
        let fe_hit = if !legend_clicked
            && mesh.is_some()
            && self.fe_selection.enabled.iter().any(|x| *x)
            && !over_projection_panel
            && !camera_busy
            && self.mode.is_none()
            && self.drag.is_none()
            && self.clipboard_op.is_none()
            && response.clicked_by(egui::PointerButton::Primary)
        {
            response
                .interact_pointer_pos()
                .and_then(|p| self.fe_pick_3d(p, scene, &draw.surfaces, &draw.beams, &projected))
        } else {
            None
        };
        let measuring = !legend_clicked
            && self.spatial_measurements(
                ui,
                &response,
                rect,
                &project,
                &projected,
                occlusion.as_ref(),
                !over_projection_panel && !camera_busy,
            );
        let editing_foundation = !legend_clicked
            && !measuring
            && !over_projection_panel
            && !camera_busy
            && self.foundation_spatial_input(
                ui,
                &painter,
                rect,
                &project,
                occlusion.as_ref().or(foundation_occlusion.as_ref()),
            );
        let editing_load = editing_foundation
            || (!legend_clicked
                && !measuring
                && !over_projection_panel
                && self.load_value_spatial(ui, load_label_hit));
        let fe_clicked = !editing_load
            && !measuring
            && self.fe_selection.pick(fe_hit, ui.input(|i| i.modifiers));
        if !fe_clicked
            && !legend_clicked
            && !over_projection_panel
            && !camera_busy
            && !editing_load
            && !measuring
            && !self.spatial_construction(ui, &response, rect, &project, &projected)
        {
            if !self.spatial_edit(ui, &response, rect, &project, &projected) {
                self.spatial_source_interaction(
                    ui,
                    &response,
                    rect,
                    &project,
                    center[2],
                    &projected,
                    occlusion.as_ref(),
                );
            }
        }
        self.restraint_buttons(painter.ctx(), rect, |p, z| {
            let (at, depth) = project([p[0], p[1], z]);
            if occlusion.as_ref().is_none_or(|o| o.visible(at, depth)) {
                at
            } else {
                Pos2::new(f32::INFINITY, f32::INFINITY)
            }
        });
        // Legends cover every scene pass, including late GPU overlays such as intersections.
        if self.shape_legend(&painter, rect) {
        } else if let Some((index, range)) = summation_legend {
            self.punching_scales[index].legend(&painter, rect, range, self.display_text);
        } else if let Some((index, range)) = length_legend {
            self.length_scales[index].legend(&painter, rect, range, self.display_text);
        } else if let Some(range) = result_range.filter(|_| self.show_results) {
            self.result_scales[self.result_field].legend(&painter, rect, range, self.display_text);
        } else {
            self.schema_legend(&painter, rect);
        }
        self.spatial.projected = projected;
    }
}

impl App {
    fn spatial_origin_axes(
        &self,
        _painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        _projected: &[(Pos2, f64)],
        _scene: &shell::Scene,
        scale: f64,
        builder: &mut spatial_depth::Builder,
    ) {
        let z = if self.building.enabled {
            self.building.floor(self.building.active).elevation_mm
        } else {
            0.
        };
        let origin = project([0., 0., z]);
        if !origin.0.x.is_finite()
            || !origin.0.y.is_finite()
            || !rect.expand(80.).contains(origin.0)
        {
            return;
        }
        for (axis, label, color) in [
            (0, "X", Color32::from_rgb(245, 95, 85)),
            (1, "Y", Color32::from_rgb(100, 220, 130)),
        ] {
            let mut world = [0., 0., z];
            world[axis] = 58. / scale.max(1e-12);
            let tip = project(world);
            let delta = tip.0 - origin.0;
            if delta.length() < 3. {
                continue;
            }
            let direction = delta.normalized();
            let normal = egui::vec2(-direction.y, direction.x);
            builder.line([origin, tip], 2., color);
            builder.triangle(
                [
                    tip,
                    (tip.0 - direction * 9. + normal * 4., tip.1),
                    (tip.0 - direction * 9. - normal * 4., tip.1),
                ],
                color,
                0.00002,
            );
            let at = tip.0 + direction * 10.;
            let background = Rect::from_center_size(at, egui::vec2(12., 14.));
            let corners = [
                background.left_top(),
                background.right_top(),
                background.right_bottom(),
                background.left_bottom(),
            ]
            .map(|p| (p, tip.1));
            let ink = Color32::from_rgba_unmultiplied(20, 24, 30, 225);
            // Keep the background behind the glyph but in the same depth layer,
            // so both remain occluded by foreground model geometry.
            builder.triangle([corners[0], corners[1], corners[2]], ink, 0.);
            builder.triangle([corners[0], corners[2], corners[3]], ink, 0.);
            let lines = if label == "X" {
                vec![([-3., -5.], [3., 5.]), ([-3., 5.], [3., -5.])]
            } else {
                vec![
                    ([-3., -5.], [0., 0.]),
                    ([3., -5.], [0., 0.]),
                    ([0., 0.], [0., 5.]),
                ]
            };
            for (a, b) in lines {
                builder.line(
                    [
                        (at + egui::vec2(a[0], a[1]), tip.1),
                        (at + egui::vec2(b[0], b[1]), tip.1),
                    ],
                    1.5,
                    color,
                );
            }
        }
        for i in 0..12 {
            let a = i as f32 * std::f32::consts::TAU / 12.;
            let b = (i + 1) as f32 * std::f32::consts::TAU / 12.;
            builder.triangle(
                [
                    origin,
                    (origin.0 + egui::vec2(a.cos(), a.sin()) * 3., origin.1),
                    (origin.0 + egui::vec2(b.cos(), b.sin()) * 3., origin.1),
                ],
                Color32::WHITE,
                0.00002,
            );
        }
    }
    fn spatial_load_labels(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        scene: &shell::Scene,
    ) -> Option<u64> {
        if !self.load_values_visible() {
            return None;
        }
        let mut labels = Vec::new();
        let mut add = |p: Point,
                       z,
                       value,
                       kind: Kind,
                       id,
                       load: Option<&crate::plan_trace::loads::LoadData>| {
            let (at, depth) = project([p[0], p[1], z]);
            if rect.contains(at) {
                let axis = load
                    .filter(|l| l.moment || l.direction.index() != 2)
                    .map_or("", |l| l.direction.label(l.moment));
                let dimension = if load.is_some_and(|l| l.moment) {
                    3
                } else {
                    kind.load_dimension()
                };
                labels.push((at, depth, value, kind, id, axis, dimension));
            }
        };
        for c in self
            .contours
            .iter()
            .filter(|c| c.kind.is_load() && c.kind != Kind::LoadHole && self.contour_visible(c))
        {
            let Some(value) = self.displayed_load_value(c) else {
                continue;
            };
            let moment = c.load.as_ref().is_some_and(|l| l.moment);
            let offset = if moment {
                [0., 0., 0.]
            } else {
                spatial_loads::offset(self, c)
            };
            let z = self.source_heights(c)[0] + offset[2];
            let mut points = c.load_label_positions();
            if c.kind == Kind::LoadArea {
                let triangles = self.cached_load_fill(c);
                if triangles.is_empty() {
                    continue;
                }
                if points
                    .first()
                    .is_none_or(|p| !triangles.iter().any(|t| geometry::inside(*p, t)))
                {
                    if let Some(t) = triangles
                        .iter()
                        .max_by(|a, b| geometry::area(*a).total_cmp(&geometry::area(*b)))
                    {
                        points = vec![[
                            (t[0][0] + t[1][0] + t[2][0]) / 3.,
                            (t[0][1] + t[1][1] + t[2][1]) / 3.,
                        ]];
                    }
                }
            }
            for p in points {
                add(
                    [p[0] + offset[0], p[1] + offset[1]],
                    z,
                    value,
                    c.kind,
                    Some(c.id),
                    c.load.as_ref(),
                );
            }
        }
        let factor: f64 = self
            .load_cases
            .iter()
            .filter(|c| c.kind == crate::plan_trace::loads::LoadType::SelfWeight)
            .filter_map(|c| self.displayed_load_factor(c.id))
            .sum();
        if factor.is_finite() && factor != 0. {
            for c in self.contours.iter().filter(|c| {
                matches!(c.kind, Kind::Slab | Kind::Support | Kind::Beam) && !self.object_hidden(c)
            }) {
                let value = match c.kind {
                    Kind::Slab if self.drawing_visibility.loads[0] => {
                        self.materials
                            .get(&c.id)
                            .copied()
                            .unwrap_or_default()
                            .self_weight_kn_m2(self.gravity)
                            * self.force_result_factor()
                            * factor
                    }
                    Kind::Support
                        if self.drawing_visibility.loads[1]
                            && (self.building.enabled || self.support_height.enabled) =>
                    {
                        let height = if self.building.enabled {
                            let e = self.building.extent(self.building.owner(c.id));
                            e[1] - e[0]
                        } else {
                            let e = self.support_height.extent();
                            e[1] - e[0]
                        };
                        self.support_height
                            .materials
                            .get(&c.id)
                            .copied()
                            .unwrap_or_else(Material::wall_default)
                            .self_weight_kn_m2(self.gravity)
                            * height
                            / 1000.
                            * self.force_result_factor()
                            * factor
                    }
                    Kind::Beam if self.drawing_visibility.loads[1] => {
                        self.support_height.beams.get(&c.id).map_or(0., |b| {
                            b.material.weight.in_units(self.load_units, self.gravity)
                                * b.area()
                                * factor
                        })
                    }
                    _ => 0.,
                };
                if !value.is_finite() || value == 0. {
                    continue;
                }
                if c.is_free_beam() {
                    let h = spatial_loads::height(Kind::LoadLine);
                    for edge in self.source_beam_path(c).windows(2) {
                        let [x, y, z] = std::array::from_fn(|i| (edge[0][i] + edge[1][i]) * 0.5);
                        add([x, y], z + h, value, Kind::LoadLine, None, None);
                    }
                    continue;
                }
                let z = self.source_heights(c)[0];
                let mut source = c.clone();
                source.kind = if c.kind == Kind::Slab {
                    Kind::LoadArea
                } else {
                    Kind::LoadLine
                };
                source.load = None;
                let mut points = source.load_label_positions();
                if c.kind == Kind::Slab {
                    let triangles = self.cached_load_fill(&source);
                    if triangles.is_empty() {
                        continue;
                    }
                    if points
                        .first()
                        .is_none_or(|p| !triangles.iter().any(|t| geometry::inside(*p, t)))
                    {
                        if let Some(t) = triangles
                            .iter()
                            .max_by(|a, b| geometry::area(*a).total_cmp(&geometry::area(*b)))
                        {
                            points = vec![[
                                (t[0][0] + t[1][0] + t[2][0]) / 3.,
                                (t[0][1] + t[1][1] + t[2][1]) / 3.,
                            ]];
                        }
                    }
                }
                for p in points {
                    add(
                        p,
                        z + spatial_loads::height(source.kind),
                        value,
                        source.kind,
                        None,
                        None,
                    );
                }
            }
        }
        if labels.is_empty() {
            return None;
        }
        // At building scale many projected values share the same screen space.
        // Prefer selected loads, then the frontmost visible value in each cell.
        labels.sort_unstable_by(|a, b| {
            let selected = |id: Option<u64>| id.is_some_and(|id| self.is_selected(id));
            selected(b.4)
                .cmp(&selected(a.4))
                .then_with(|| b.1.total_cmp(&a.1))
        });
        let occlusion = spatial_results::Occlusion::new_visible(rect, scene, projected, self);
        let pointer = painter.ctx().pointer_hover_pos();
        let mut hit = None::<(u64, f64)>;
        let mut occupied = std::collections::HashSet::new();
        for (at, depth, value, kind, id, axis, dimension) in labels {
            let center = at
                + if kind == Kind::LoadPoint {
                    egui::vec2(35., -14.)
                } else {
                    egui::Vec2::ZERO
                };
            let cell = (
                ((center.x - rect.min.x) / 72.).floor() as i32,
                ((center.y - rect.min.y) / 18.).floor() as i32,
            );
            if occupied.contains(&cell) || !occlusion.visible(at, depth) {
                continue;
            }
            occupied.insert(cell);
            let caption = self.load_caption(
                painter,
                center,
                format!(
                    "{} {} {}",
                    axis,
                    self.display_text.number(value),
                    self.load_units.intensity_label(dimension)
                ),
            );
            if let Some(id) = id {
                if pointer.is_some_and(|p| caption.contains(p)) && hit.is_none_or(|h| depth >= h.1)
                {
                    hit = Some((id, depth));
                }
            }
        }
        hit.map(|h| h.0)
    }
}

pub(super) fn building_source_scene(
    contours: &[Contour],
    settings: &shell::Settings,
    building: &building::Building,
) -> Result<shell::Scene, String> {
    building.validate()?;
    let mut output = shell::Scene {
        xyz: vec![],
        plan: vec![],
        surfaces: vec![],
        fixed: Default::default(),
        support_nodes: Default::default(),
        fixed_rotations: Default::default(),
        beams: vec![],
    };
    let bottom = building
        .floors
        .iter()
        .map(|f| f.elevation_mm)
        .min_by(f64::total_cmp)
        .unwrap();
    let top = building
        .floors
        .iter()
        .map(|f| building.extent(f.id)[1])
        .max_by(f64::total_cmp)
        .unwrap();
    let mut contours_by_floor = BTreeMap::<u64, Vec<Contour>>::new();
    let mut vertical_contours = BTreeMap::new();
    for contour in contours.iter().filter(|c| source_geometry_kind(c.kind)) {
        let floor = building.owner(contour.id);
        contours_by_floor
            .entry(floor)
            .or_default()
            .push(contour.clone());
        if matches!(contour.kind, Kind::Support | Kind::Column) {
            vertical_contours.entry(contour.id).or_insert(contour);
        }
    }
    let mut options = settings.clone();
    options.enabled = true;
    options.direction = Direction::Up;
    for floor in &building.floors {
        let [lo, hi] = building.extent(floor.id);
        let all = contours_by_floor
            .get(&floor.id)
            .map_or(&[][..], Vec::as_slice);
        options.height_mm = hi - lo;
        let local = source_scene_at(all, &options, [0., hi - lo], lo)?;
        let offset = output.xyz.len();
        for p in local.xyz {
            output.xyz.push([p[0], p[1], p[2] + lo]);
            output.plan.push(usize::MAX);
        }
        for mut surface in local.surfaces {
            surface.nodes.iter_mut().for_each(|n| *n += offset);
            output.surfaces.push(surface);
        }
        for mut beam in local.beams {
            beam.nodes = beam.nodes.map(|n| n + offset);
            output.beams.push(beam);
        }
        for (id, nodes) in local.support_nodes {
            let nodes = nodes.into_iter().map(|n| n + offset).collect::<Vec<_>>();
            output.support_nodes.insert(id, nodes);
        }
    }
    output.apply_object_restraints(settings, |id| {
        let c = vertical_contours.get(&id)?;
        let extent = building.extent(building.owner(id));
        let physical = c.placement.wall(extent);
        Some((
            (extent[0] - bottom).abs() < 1e-5,
            (extent[1] - top).abs() < 1e-5 || (physical[1] - top).abs() < 1e-5,
            physical,
        ))
    });
    Ok(output)
}

fn source_geometry_kind(kind: Kind) -> bool {
    matches!(
        kind,
        Kind::Slab | Kind::Hole | Kind::Support | Kind::Column | Kind::Beam
    )
}

/// Calculation parameters and overlays do not alter source shell tessellation.
fn same_source_geometry(a: &[Contour], b: &[Contour]) -> bool {
    let physical = |c: &&Contour| source_geometry_kind(c.kind);
    let mut a = a.iter().filter(physical);
    let mut b = b.iter().filter(physical);
    loop {
        match (a.next(), b.next()) {
            (None, None) => return true,
            (Some(a), Some(b)) => {
                if a.id != b.id
                    || a.kind != b.kind
                    || a.parent != b.parent
                    || a.points != b.points
                    || a.curves != b.curves
                    || !a.placement.same_except_movement_lock(&b.placement)
                    || (matches!(a.kind, Kind::Beam | Kind::Column)
                        && a.column_angle != b.column_angle)
                    || a.support_open != b.support_open
                {
                    return false;
                }
            }
            _ => return false,
        }
    }
}

/// Graphics-only tessellation. Does not create or mutate FE nodes, elements or IDs.
fn beam_screen_points(
    beam: &shell::BeamGeometry,
    scene: &shell::Scene,
    projected: &[(Pos2, f64)],
    project: &impl Fn([f64; 3]) -> (Pos2, f64),
) -> [(Pos2, f64); 2] {
    let arm = beam.rigid_arm(&scene.xyz).unwrap_or([0.; 3]);
    beam.nodes.map(|n| {
        if arm == [0.; 3] {
            return projected[n];
        }
        let base = project(scene.xyz[n]);
        let offset = project(std::array::from_fn(|a| scene.xyz[n][a] + arm[a]));
        (
            offset.0 + (projected[n].0 - base.0),
            offset.1 + projected[n].1 - base.1,
        )
    })
}
fn source_scene(contours: &[Contour], settings: &shell::Settings) -> Result<shell::Scene, String> {
    source_scene_at(contours, settings, [0., 3000.], 0.)
}
fn source_scene_at(
    contours: &[Contour],
    settings: &shell::Settings,
    extent: [f64; 2],
    origin_z: f64,
) -> Result<shell::Scene, String> {
    settings.validate()?;
    let mut scene = shell::Scene {
        xyz: vec![],
        plan: vec![],
        surfaces: vec![],
        fixed: Default::default(),
        support_nodes: Default::default(),
        fixed_rotations: Default::default(),
        beams: vec![],
    };
    let node = |point: [f64; 3], scene: &mut shell::Scene| {
        let index = scene.xyz.len();
        scene.xyz.push(point);
        scene.plan.push(index);
        index
    };
    let mut planes = contours
        .iter()
        .filter(|c| c.kind == Kind::Slab)
        .map(|c| c.placement.slab.z(extent))
        .collect::<Vec<_>>();
    planes.sort_by(f64::total_cmp);
    planes.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
    for z in planes {
        let slabs = contours
            .iter()
            .filter(|c| c.kind == Kind::Slab && (c.placement.slab.z(extent) - z).abs() < 1e-5)
            .map(|c| c.id)
            .collect::<std::collections::BTreeSet<_>>();
        let selected = contours.iter().filter(|c| {
            slabs.contains(&c.id)
                || c.active_hole() && c.parent.is_some_and(|id| slabs.contains(&id))
        });
        for triangle in model::slab_triangles(selected) {
            let nodes = triangle.map(|p| node([p[0], p[1], z], &mut scene));
            scene.surfaces.push(shell::Surface {
                rigid_z_mm: 0.,
                nodes: nodes.to_vec(),
                material: Material::default(),
                display: vec![],
                support: None,
            });
        }
    }
    for c in contours.iter().filter(|c| c.kind == Kind::Column) {
        if let Some(p) = c.points.first() {
            let mut support_nodes = Vec::new();
            let heights = if c.placement.levels_default() {
                settings.extent()
            } else {
                c.placement.wall(extent)
            };
            for (end, z) in heights.into_iter().enumerate() {
                let n = node([p[0], p[1], z], &mut scene);
                support_nodes.push(n);
                if end == 0 {
                    scene.fixed.extend([3 * n, 3 * n + 1, 3 * n + 2]);
                } else {
                    scene.fixed.extend([3 * n, 3 * n + 1]);
                }
            }
            scene.support_nodes.insert(c.id, support_nodes);
        }
    }
    for contour in contours.iter().filter(|c| c.kind == Kind::Support) {
        let mut path = contour.points.clone();
        if contour.closed() && !path.is_empty() {
            path.push(path[0]);
        }
        let [lo, hi] = if contour.placement.levels_default() {
            settings.extent()
        } else {
            contour.placement.wall(extent)
        };
        if hi - lo < 100. {
            return Err(format!("Стена {}: высота менее 100 мм", contour.id));
        }
        let mut support_nodes = Vec::new();
        for edge in path.windows(2) {
            let nodes = [
                node([edge[0][0], edge[0][1], lo], &mut scene),
                node([edge[1][0], edge[1][1], lo], &mut scene),
                node([edge[1][0], edge[1][1], hi], &mut scene),
                node([edge[0][0], edge[0][1], hi], &mut scene),
            ];
            for &n in &nodes[..2] {
                scene.fixed.extend([3 * n, 3 * n + 1, 3 * n + 2]);
            }
            for &n in &nodes[2..] {
                scene.fixed.extend([3 * n, 3 * n + 1]);
            }
            support_nodes.extend(nodes);
            scene.surfaces.push(shell::Surface {
                rigid_z_mm: 0.,
                nodes: nodes.to_vec(),
                material: Material::default(),
                display: vec![],
                support: Some(contour.id),
            });
        }
        scene.support_nodes.insert(contour.id, support_nodes);
    }
    for contour in contours.iter().filter(|c| c.kind == Kind::Beam) {
        let mut path = contour.beam_path(extent.map(|z| z + origin_z));
        for p in &mut path {
            p[2] -= origin_z;
        }
        if contour.closed() && !path.is_empty() {
            path.push(path[0]);
        }
        for edge in path.windows(2) {
            let nodes = [node(edge[0], &mut scene), node(edge[1], &mut scene)];
            let section = settings.beams.get(&contour.id).cloned().unwrap_or(
                crate::plan_trace::fem::beam::Section {
                    material: Material::default(),
                    width_mm: 200.,
                },
            );
            scene.beams.push(shell::BeamGeometry {
                rigid_offset_mm: [0.; 2],
                roll_angle_rad: contour.column_angle,
                id: contour.id,
                nodes,
                section,
            });
        }
    }
    scene.apply_beam_offsets(contours)?;
    Ok(scene)
}

#[cfg(test)]
mod source_geometry_tests {
    use super::*;
    #[test]
    fn geometry_updates_keep_locked_edit_camera_until_manual_fit() {
        let mut view = View::default();
        let bounds = ([123., 456., 789.], 2400.);
        view.bounds = Some(bounds);
        view.zoom = 2.3;
        view.pan = egui::vec2(40., -15.);
        let pose = (view.yaw, view.pitch, view.zoom, view.pan);
        view.lock_edit_camera();
        for _ in 0..3 {
            view.cache_reset();
            assert_eq!(view.bounds, Some(bounds));
            assert_eq!((view.yaw, view.pitch, view.zoom, view.pan), pose);
        }
        view.edit_camera_bounds = None; // post-release scene rebuild has consumed the saved bounds
        assert_eq!(view.bounds, Some(bounds));
        view.fit_view();
        assert_eq!(view.bounds, None);
        assert_eq!(view.zoom, 1.);
        assert_eq!(view.pan, egui::Vec2::ZERO);
    }
    fn contour(id: u64, kind: Kind, parent: Option<u64>, points: Vec<Point>) -> Contour {
        Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id,
            kind,
            parent,
            points,
            curves: vec![],
            load: None,
            refinement: None,
            grid: None,
            support_open: kind == Kind::Support,
        }
    }
    #[test]
    fn source_scene_preserves_free_beam_elevations_on_upper_storey() {
        let mut building = building::Building::default();
        building.enabled = true;
        building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        building.contour_floors.insert(1, 2);
        building.contour_floors.insert(2, 2);
        let mut free = contour(1, Kind::Beam, None, vec![[0., 0.], [1000., 0.]]);
        free.support_open = true;
        free.placement.beam_z_mm = Some(vec![2700., 5100.]);
        let mut linked = contour(2, Kind::Beam, None, vec![[0., 100.], [1000., 100.]]);
        linked.support_open = true;
        let scene =
            building_source_scene(&[free, linked], &shell::Settings::default(), &building).unwrap();
        assert_eq!(scene.beams.len(), 2);
        assert_eq!(
            scene.beams[0].nodes.map(|n| scene.xyz[n][2]),
            [2700., 5100.]
        );
        assert_eq!(
            scene.beams[1].nodes.map(|n| scene.xyz[n][2]),
            [3000., 3000.]
        );
    }
    #[test]
    fn opaque_3d_legend_is_painted_after_the_intersection_overlay() {
        let mut app = App::default();
        app.spatial.enabled = true;
        app.show_results = false;
        app.contours.push(contour(
            1,
            Kind::Slab,
            None,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        ));
        app.contours.push(contour(
            2,
            Kind::Slab,
            None,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        ));
        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors.insert(2, 2);
        app.schema_analysis.mode = Some(schema_analysis::Mode::Floors);
        app.intersections.show = true;
        app.intersections.current = true;
        app.intersections
            .graph
            .lines
            .push(crate::plan_trace::intersections::Segment {
                ends: [[0., 0., 0.], [2000., 0., 0.]],
                owners: vec![1],
            });
        let frame = egui::Context::default().run(
            egui::RawInput {
                screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.))),
                ..Default::default()
            },
            |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    app.schema_refresh();
                    app.spatial_canvas(ui);
                });
            },
        );
        let last_geometry = frame
            .shapes
            .iter()
            .rposition(|s| matches!(s.shape, egui::Shape::Callback(_)))
            .unwrap();
        let background = frame
            .shapes
            .iter()
            .position(|s| {
                matches!(&s.shape, egui::Shape::Rect(r) if r.fill == Color32::from_rgb(20, 24, 30) && r.rect.width() > 700.)
            })
            .unwrap();
        assert!(background > last_geometry);
    }
    #[test]
    #[ignore = "requires MINI_FEM_BEAM_PROJECT; CPU frame profiling, no GPU timings"]
    fn beam_source_project_visible_hidden_frame_profile() {
        let path = PathBuf::from(std::env::var_os("MINI_FEM_BEAM_PROJECT").expect("beam project"));
        let mut app = App::default();
        app.load_project_path(path).unwrap();
        app.spatial.enabled = true;
        app.show_results = false;
        app.show_result_values = false;
        let ctx = egui::Context::default();
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(1200., 800.));
        let frame = |app: &mut App| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(rect),
                    events: vec![egui::Event::PointerMoved(rect.center())],
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| app.spatial_canvas(ui));
                },
            );
        };
        let mut vertex_counts = vec![];
        for visible in [true, false] {
            app.drawing_visibility.beam = visible;
            frame(&mut app);
            vertex_counts.push(
                app.spatial
                    .depth_cache
                    .borrow()
                    .as_ref()
                    .unwrap()
                    .vertices
                    .len(),
            );
            for orbit in [false, true] {
                let start = web_time::Instant::now();
                for _ in 0..12 {
                    if orbit {
                        app.spatial.pitch += 0.01;
                    }
                    frame(&mut app);
                }
                println!(
                    "BEAM_FRAME visible={visible} orbit={orbit} cpu_mean_ms={:.3}",
                    start.elapsed().as_secs_f64() * 1000. / 12.
                );
            }
        }
        println!(
            "BEAM_VERTICES visible={} hidden={}",
            vertex_counts[0], vertex_counts[1]
        );
        assert!(
            vertex_counts[1] < vertex_counts[0],
            "hidden beam geometry must leave depth buffer"
        );
    }
    #[test]
    fn automatic_column_preview_restraints_follow_masks_at_building_ends() {
        let mut building = building::Building::default();
        building.enabled = true;
        let all = vec![contour(10, Kind::Column, None, vec![[100., 200.]])];
        building.contour_floors.insert(10, building.floors[0].id);
        let mut settings = shell::Settings::default();
        settings.automatic_building_bottom = true;
        settings.automatic_building_top = true;
        settings.building_bottom_mask = 7 | (1 << 5);
        settings.building_top_mask = 1;
        let scene = building_source_scene(&all, &settings, &building).unwrap();
        let ns = &scene.support_nodes[&10];
        assert_eq!(ns.len(), 2);
        assert_eq!(scene.plan.len(), scene.xyz.len());
        for axis in 0..3 {
            assert!(scene.fixed.contains(&(3 * ns[0] + axis)));
        }
        assert!(scene.fixed_rotations.contains(&(3 * ns[0] + 2)));
        assert!(scene.fixed.contains(&(3 * ns[1])));
        assert!(!scene.fixed.contains(&(3 * ns[1] + 1)));
        assert!(!scene.fixed.contains(&(3 * ns[1] + 2)));
        settings.automatic_building_bottom = false;
        settings.automatic_building_top = false;
        let scene = building_source_scene(&all, &settings, &building).unwrap();
        assert!(scene.fixed.is_empty());
        assert!(scene.fixed_rotations.is_empty());
    }
    #[test]
    fn building_preview_has_distinct_slabs_and_only_extreme_wall_restraints() {
        let mut building = building::Building::default();
        building.enabled = true;
        building.floors.push(building::Floor {
            id: 2,
            name: "Этаж 2".into(),
            elevation_mm: 3000.,
            height_mm: 2500.,
        });
        let all = vec![
            contour(
                1,
                Kind::Slab,
                None,
                vec![[0., 0.], [2000., 0.], [2000., 1500.], [0., 1500.]],
            ),
            contour(2, Kind::Support, None, vec![[0., 0.], [2000., 0.]]),
            contour(
                3,
                Kind::Slab,
                None,
                vec![[0., 0.], [2000., 0.], [2000., 1500.], [0., 1500.]],
            ),
            contour(4, Kind::Support, None, vec![[0., 0.], [2000., 0.]]),
        ];
        building.contour_floors = BTreeMap::from([(1, 1), (2, 1), (3, 2), (4, 2)]);
        let scene = building_source_scene(&all, &shell::Settings::default(), &building).unwrap();
        let slabs = scene
            .surfaces
            .iter()
            .filter(|s| s.support.is_none())
            .collect::<Vec<_>>();
        assert!(slabs
            .iter()
            .any(|s| s.nodes.iter().all(|&n| scene.xyz[n][2] == 0.)));
        assert!(slabs
            .iter()
            .any(|s| s.nodes.iter().all(|&n| scene.xyz[n][2] == 3000.)));
        for &d in &scene.fixed {
            let z = scene.xyz[d / 3][2];
            assert!(z == 0. || z == 5500.);
            if z == 5500. {
                assert_ne!(d % 3, 2);
            }
        }
        assert!(scene.fixed.iter().any(|d| *d % 3 == 2));
        assert!(scene.fixed.iter().any(|d| scene.xyz[*d / 3][2] == 5500.));
    }
    #[test]
    fn empty_spatial_view_uses_grid_bounds_and_accepts_wheel_zoom() {
        let mut app = App::new_project();
        app.spatial.enabled = true;
        app.construction_grid.enabled = true;
        app.construction_grid.center = [4000., 2500.];
        let ctx = egui::Context::default();
        let frame = |app: &mut App, events| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(800., 600.))),
                    events,
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| app.spatial_canvas(ui));
                },
            );
        };
        let point = Pos2::new(400., 300.);
        frame(&mut app, vec![]);
        let bounds = ([4000., 2500., 0.], 20000_f64.hypot(20000.));
        assert_eq!(app.spatial.bounds, Some(bounds));
        frame(&mut app, vec![egui::Event::PointerMoved(point)]);
        let before = app.spatial.zoom;
        frame(
            &mut app,
            vec![egui::Event::MouseWheel {
                unit: egui::MouseWheelUnit::Point,
                delta: egui::vec2(0., -800.),
                modifiers: egui::Modifiers::NONE,
            }],
        );
        assert!(app.spatial.zoom < before);
        assert_eq!(app.spatial.bounds, Some(bounds));

        app.construction_grid.width_mm = 12000.;
        frame(&mut app, vec![]);
        assert_eq!(app.spatial.bounds.unwrap().1, 12000_f64.hypot(20000.));
    }

    #[test]
    fn cursor_zoom_preserves_anchor_and_pan_zoom_reuses_depth_geometry() {
        let rect = Rect::from_min_size(Pos2::new(10., 20.), egui::vec2(800., 600.));
        let mut view = View::default();
        view.pan = egui::vec2(30., -40.);
        let at = Pos2::new(260., 180.);
        let world = (at - rect.center() - view.pan) / view.zoom as f32;
        view.zoom_to(rect, at, 2.5);
        assert!((rect.center() + view.pan + world * view.zoom as f32 - at).length() < 0.001);
        let mut app = App::default();
        app.support_height.enabled = true;
        app.contours = vec![contour(
            1,
            Kind::Slab,
            None,
            vec![[0., 0.], [6000., 0.], [6000., 4000.], [0., 4000.]],
        )];
        app.fe_mesh = Some(crate::plan_trace::meshing::generate(&app.contours, 500.).unwrap());
        app.show_mesh = true;
        let ctx = egui::Context::default();
        let mut frame = |app: &mut App| {
            let _ = ctx.run(
                egui::RawInput {
                    screen_rect: Some(rect),
                    ..Default::default()
                },
                |ctx| {
                    egui::CentralPanel::default().show(ctx, |ui| app.spatial_canvas(ui));
                },
            );
        };
        frame(&mut app);
        let first = app
            .spatial
            .depth_cache
            .borrow()
            .as_ref()
            .unwrap()
            .vertices
            .clone();
        let world = app.spatial.world_cache.borrow().as_ref().unwrap().1.clone();
        assert!(
            !world.edges.is_empty(),
            "FE edges must be cached in GPU world geometry"
        );
        app.spatial.zoom = 1.4;
        app.spatial.pan = egui::vec2(20., -30.);
        frame(&mut app);
        assert!(std::sync::Arc::ptr_eq(
            &first,
            &app.spatial.depth_cache.borrow().as_ref().unwrap().vertices
        ));
        app.spatial.yaw += 0.2;
        frame(&mut app);
        assert!(
            std::sync::Arc::ptr_eq(
                &world,
                &app.spatial.world_cache.borrow().as_ref().unwrap().1
            ),
            "orbit must reuse FE edge geometry"
        );
        assert!(!std::sync::Arc::ptr_eq(
            &first,
            &app.spatial.depth_cache.borrow().as_ref().unwrap().vertices
        ));
    }
    #[test]
    fn labels_behind_surfaces_are_hidden_but_holes_remain_open() {
        let contours = vec![
            contour(
                1,
                Kind::Slab,
                None,
                vec![[0., 0.], [600., 0.], [600., 400.], [0., 400.]],
            ),
            contour(
                2,
                Kind::Hole,
                Some(1),
                vec![[100., 100.], [200., 100.], [200., 200.], [100., 200.]],
            ),
        ];
        let scene = source_scene(&contours, &shell::Settings::default()).unwrap();
        let projected = scene
            .xyz
            .iter()
            .map(|p| (Pos2::new(p[0] as f32, p[1] as f32), 0.))
            .collect::<Vec<_>>();
        let occlusion = spatial_results::Occlusion::new(
            Rect::from_min_max(Pos2::ZERO, Pos2::new(600., 400.)),
            &scene,
            &projected,
        );
        assert!(!occlusion.visible(Pos2::new(300., 250.), -10.));
        assert!(occlusion.visible(Pos2::new(300., 250.), 0.));
        assert!(occlusion.visible(Pos2::new(150., 150.), -10.));
    }

    #[test]
    fn unfilled_slabs_do_not_occlude_selected_opening_handles_in_top_view() {
        let _clip = spatial_depth::clip_guard();
        let mut upper = contour(
            3,
            Kind::Slab,
            None,
            vec![[0., 0.], [600., 0.], [600., 400.], [0., 400.]],
        );
        upper.placement.slab.offset_mm = 3000.;
        let mut app = App::default();
        app.spatial.enabled = true;
        app.contours = vec![
            contour(
                1,
                Kind::Slab,
                None,
                vec![[0., 0.], [600., 0.], [600., 400.], [0., 400.]],
            ),
            contour(
                2,
                Kind::Hole,
                Some(1),
                vec![[100., 100.], [200., 100.], [200., 200.], [100., 200.]],
            ),
            upper,
        ];
        let scene = source_scene(&app.contours, &app.support_height).unwrap();
        let projected = scene
            .xyz
            .iter()
            .map(|p| (Pos2::new(p[0] as f32, p[1] as f32), p[2]))
            .collect::<Vec<_>>();
        let rect = Rect::from_min_max(Pos2::ZERO, Pos2::new(600., 400.));
        app.drawing_visibility.contours[0] = false;
        app.select_only(Some(2));
        let point = app.contours[1].handles()[0];
        let at = Pos2::new(point[0] as f32, point[1] as f32);
        assert_eq!(app.selected, Some(2));
        assert!(
            !spatial_results::Occlusion::new_visible(rect, &scene, &projected, &app)
                .visible(at, 0.)
        );

        app.drawing_visibility.fill = false;
        assert!(app.contour_visible(&app.contours[1]));
        assert!(
            spatial_results::Occlusion::new_visible(rect, &scene, &projected, &app).visible(at, 0.)
        );
    }

    #[test]
    fn source_preview_subtracts_hole_and_extrudes_support_without_fe_mesh() {
        let contours = vec![
            contour(
                1,
                Kind::Slab,
                None,
                vec![[0., 0.], [6000., 0.], [6000., 4000.], [0., 4000.]],
            ),
            contour(
                2,
                Kind::Hole,
                Some(1),
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
            contour(3, Kind::Support, None, vec![[0., 0.], [6000., 0.]]),
        ];
        let original = contours.clone();
        let settings = shell::Settings {
            enabled: true,
            direction: Direction::Both,
            height_mm: 3000.,
            ..Default::default()
        };
        let scene = source_scene(&contours, &settings).unwrap();
        let area: f64 = scene
            .surfaces
            .iter()
            .filter(|s| s.support.is_none())
            .map(|s| {
                let points = s
                    .nodes
                    .iter()
                    .map(|&n| [scene.xyz[n][0], scene.xyz[n][1]])
                    .collect::<Vec<_>>();
                crate::plan_trace::geometry::area(&points)
            })
            .sum();
        assert!((area - 23_000_000.).abs() < 1.);
        assert!(scene.surfaces.iter().all(|s| s.display.is_empty()));
        assert!(scene.xyz.iter().any(|p| p[2] == -3000.));
        assert!(scene.xyz.iter().any(|p| p[2] == 3000.));
        assert_eq!(contours, original);
        assert!(source_scene(&[], &settings).unwrap().xyz.is_empty());
    }
}
#[cfg(test)]
mod source_cache_partition_tests {
    use super::*;
    #[test]
    fn source_geometry_ignores_names_soil_and_design_parameters_but_tracks_shape() {
        let mut a = App::default();
        a.begin(Kind::Slab);
        a.choose_tool(Tool::Rectangle);
        a.sketch_click([0., 0.], false);
        a.sketch_click([1000., 1000.], false);
        a.cancel();
        let old = a.contours.clone();
        a.contours[0].name = "Renamed".into();
        a.contours[0].use_foundation = true;
        assert!(same_source_geometry(&old, &a.contours));
        let mut overlay = a.contours[0].clone();
        overlay.id = 100;
        overlay.kind = Kind::Foundation;
        a.contours.push(overlay);
        assert!(same_source_geometry(&old, &a.contours));
        a.contours[0].points[1][0] += 1.;
        assert!(!same_source_geometry(&old, &a.contours));
    }
}
