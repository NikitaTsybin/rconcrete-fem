//! Temporary view state, never a change to the calculation model.
use super::*;
type FeKeys = std::collections::BTreeSet<(u8, usize)>;

#[derive(Default)]
struct FeFragment {
    generation: u64,
    hidden: FeKeys,
    isolated: Option<FeKeys>,
}
pub(super) struct DrawList {
    pub nodes: Vec<usize>,
    pub surfaces: Vec<usize>,
    pub beams: Vec<usize>,
    pub fixed_nodes: Vec<(usize, u8)>,
    node_mask: Vec<bool>,
}
impl DrawList {
    pub(super) fn contains_node(&self, n: usize) -> bool {
        self.node_mask.get(n) == Some(&true)
    }
}
struct FloorSnapshot {
    contours: std::collections::BTreeSet<u64>,
    underlays: std::collections::BTreeSet<u64>,
    dimensions: std::collections::BTreeSet<u64>,
    floors: std::collections::BTreeSet<u64>,
}
#[derive(Default)]
pub(super) struct State {
    fe: FeFragment,
    active_floor: Option<FloorSnapshot>,
    pub(super) contours: std::collections::BTreeSet<u64>,
    draw_list: std::cell::RefCell<Option<([usize; 8], std::sync::Arc<DrawList>)>>,
    pub(super) revision: usize,
    mesh_mask: std::cell::RefCell<Option<((u64, bool, u64), Vec<bool>, Vec<bool>)>>,
    underlays: std::collections::BTreeSet<u64>,
    dimensions: std::collections::BTreeSet<u64>,
}
impl App {
    pub(super) fn fe_fragment_active(&self) -> bool {
        self.fe_mesh.is_some()
            && self.view_visibility.fe.generation == self.mesh_generation
            && (!self.view_visibility.fe.hidden.is_empty() || self.fe_isolated())
    }

    pub(super) fn fe_isolated(&self) -> bool {
        self.fe_mesh.is_some()
            && self.view_visibility.fe.generation == self.mesh_generation
            && self.view_visibility.fe.isolated.is_some()
    }

    pub(super) fn fe_item_visible(&self, item: (u8, usize)) -> bool {
        let fe = &self.view_visibility.fe;
        !self.fe_fragment_active()
            || (!fe.hidden.contains(&item)
                && fe
                    .isolated
                    .as_ref()
                    .is_none_or(|items| items.contains(&item)))
    }

    fn fragment_selected_fe(&mut self, isolate: bool) {
        let Some(mesh) = &self.fe_mesh else { return };
        let mut items = self.fe_selection.items.clone();
        if items.iter().any(|&(kind, _)| kind == 0) && !mesh.quads.is_empty() {
            let by_id = mesh
                .elements
                .iter()
                .enumerate()
                .map(|(i, e)| (e.id, i))
                .collect::<BTreeMap<_, _>>();
            for q in &mesh.quads {
                let pair = q.display_triangles.map(|id| (0, by_id[&id]));
                if pair.iter().any(|i| items.contains(i)) {
                    items.extend(pair);
                }
            }
        }
        let fe = &mut self.view_visibility.fe;
        if fe.generation != self.mesh_generation {
            *fe = FeFragment {
                generation: self.mesh_generation,
                ..Default::default()
            };
        }
        if isolate {
            fe.isolated = Some(items);
        } else {
            fe.hidden.extend(items);
            self.fe_selection.items.clear();
        }
        self.view_visibility_changed();
    }

    /// Source tessellation has stable node IDs. Resolve slab ownership once
    /// when geometry changes, never by polygon searches on every visibility edit.
    pub(super) fn source_surface_owner_index(
        &self,
        scene: &crate::plan_trace::fem::shell::Scene,
    ) -> BTreeMap<usize, Vec<u64>> {
        use rstar::{
            primitives::{GeomWithData, Rectangle},
            Envelope, RTree, AABB,
        };
        const LEVEL_TOLERANCE_MM: f64 = 1e-5;
        let mut holes = BTreeMap::<u64, Vec<(AABB<Point>, &Contour)>>::new();
        let mut slabs = Vec::new();
        for (order, c) in self.contours.iter().enumerate() {
            if c.kind != Kind::Slab && !c.active_hole() || c.points.len() < 3 {
                continue;
            }
            let (lo, hi) = c
                .points
                .iter()
                .fold((c.points[0], c.points[0]), |(lo, hi), p| {
                    (
                        std::array::from_fn(|i| lo[i].min(p[i])),
                        std::array::from_fn(|i| hi[i].max(p[i])),
                    )
                });
            if c.kind == Kind::Slab {
                let z = self.source_heights(c)[0];
                let bounds = AABB::from_corners([lo[0], lo[1], z], [hi[0], hi[1], z]);
                slabs.push(GeomWithData::new(
                    Rectangle::from_aabb(bounds),
                    (order, c, z),
                ));
            } else if let Some(parent) = c.parent {
                holes
                    .entry(parent)
                    .or_default()
                    .push((AABB::from_corners(lo, hi), c));
            }
        }
        let slabs = RTree::bulk_load(slabs);
        let mut owners = BTreeMap::new();
        for s in scene
            .surfaces
            .iter()
            .filter(|s| s.support.is_none() && s.display.is_empty() && !s.nodes.is_empty())
        {
            let p: [f64; 3] = std::array::from_fn(|a| {
                s.nodes.iter().map(|&n| scene.xyz[n][a]).sum::<f64>() / s.nodes.len() as f64
            });
            let xy = [p[0], p[1]];
            let search = AABB::from_corners(
                [p[0], p[1], p[2] - LEVEL_TOLERANCE_MM],
                [p[0], p[1], p[2] + LEVEL_TOLERANCE_MM],
            );
            let mut ids = Vec::new();
            for slab in slabs.locate_in_envelope_intersecting(&search) {
                let (order, c, z) = slab.data;
                if (z - p[2]).abs() < LEVEL_TOLERANCE_MM
                    && geometry::inside(xy, &c.points)
                    && !holes
                        .get(&c.id)
                        .into_iter()
                        .flatten()
                        .any(|(bounds, hole)| {
                            bounds.contains_point(&xy) && geometry::inside(xy, &hole.points)
                        })
                {
                    ids.push(((z / LEVEL_TOLERANCE_MM).round() as i64, order, c.id));
                }
            }
            // Preserve fallback handling for graphics not originating in a slab.
            if !ids.is_empty() {
                ids.sort_unstable();
                owners.insert(s.nodes[0], ids.into_iter().map(|(_, _, id)| id).collect());
            }
        }
        owners
    }
    pub(super) fn active_floor_only(&self) -> bool {
        self.spatial.enabled && self.building.enabled && self.view_visibility.active_floor.is_some()
    }
    pub(super) fn toggle_active_floor_only(&mut self) {
        if let Some(saved) = self.view_visibility.active_floor.take() {
            self.view_visibility.contours = saved.contours;
            self.view_visibility.underlays = saved.underlays;
            self.view_visibility.dimensions = saved.dimensions;
            self.building.hidden_floors = saved.floors;
        } else {
            self.view_visibility.active_floor = Some(FloorSnapshot {
                contours: self.view_visibility.contours.clone(),
                underlays: self.view_visibility.underlays.clone(),
                dimensions: self.view_visibility.dimensions.clone(),
                floors: self.building.hidden_floors.clone(),
            });
        }
        self.view_visibility_changed();
    }
    pub(super) fn set_floor_visibility_from_menu(&mut self, id: u64, show: bool) {
        if !self.building.floors.iter().any(|f| f.id == id) {
            return;
        }
        let active_only = self.active_floor_only();
        // Unlike toggling the button off, this command keeps the CURRENT view,
        // not its earlier snapshot. Add/remove just the requested storey.
        self.view_visibility.active_floor.take();
        if active_only {
            self.building.hidden_floors.extend(
                self.building
                    .floors
                    .iter()
                    .filter(|f| f.id != self.building.active)
                    .map(|f| f.id),
            );
        }
        if show {
            self.building.hidden_floors.remove(&id);
            let owners = &self.building.contour_floors;
            let default = self.building.floors[0].id;
            self.view_visibility
                .contours
                .retain(|c| owners.get(c).copied().unwrap_or(default) != id);
            let owners = &self.building.underlay_floors;
            self.view_visibility
                .underlays
                .retain(|u| owners.get(u).copied().unwrap_or(default) != id);
        } else {
            self.building.hidden_floors.insert(id);
        }
        self.view_visibility_changed();
    }
    pub(super) fn object_hidden(&self, c: &Contour) -> bool {
        self.object_hidden_without_sections(c)
            || (self.spatial.enabled
                && self.sections.active()
                && !self.sections.intersects(
                    self.source_heights(c)
                        .into_iter()
                        .flat_map(|z| c.points.iter().map(move |p| [p[0], p[1], z])),
                ))
    }
    pub(super) fn object_hidden_without_sections(&self, c: &Contour) -> bool {
        if self.view_visibility.contours.contains(&c.id)
            || c.parent
                .is_some_and(|id| self.view_visibility.contours.contains(&id))
        {
            return true;
        }
        let active_only = self.active_floor_only();
        if !self.building.enabled || (!active_only && self.building.hidden_floors.is_empty()) {
            return false;
        }
        let floor = self.building.owner(c.id);
        (active_only && floor != self.building.active) || !self.building.floor_visible(floor)
    }
    pub(super) fn underlay_hidden(&self, id: u64) -> bool {
        self.fe_isolated()
            || (self.active_floor_only()
                && self
                    .building
                    .underlay_floors
                    .get(&id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id)
                    != self.building.active)
            || (self.building.enabled
                && !self.building.floor_visible(
                    self.building
                        .underlay_floors
                        .get(&id)
                        .copied()
                        .unwrap_or(self.building.floors[0].id),
                ))
            || self.view_visibility.underlays.contains(&id)
    }
    pub(super) fn dimension_hidden(&self, id: u64) -> bool {
        self.fe_isolated() || self.view_visibility.dimensions.contains(&id)
    }
    fn ensure_mesh_view_mask(&self) {
        if self
            .view_visibility
            .mesh_mask
            .borrow()
            .as_ref()
            .is_some_and(|c| {
                c.0 == (
                    self.mesh_generation,
                    self.active_floor_only(),
                    self.building.active,
                )
            })
        {
            return;
        }
        let Some(m) = &self.fe_mesh else {
            return;
        };
        let mut nodes = vec![false; m.nodes.len()];
        let elements = m
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| {
                let p = e.nodes.map(|n| m.nodes[n].xy);
                let center = [
                    p.iter().map(|p| p[0]).sum::<f64>() / 3.,
                    p.iter().map(|p| p[1]).sum::<f64>() / 3.,
                ];
                let hidden = if let Some(data) = m.building.as_ref() {
                    self.view_contour(data.element_slabs[i])
                        .is_some_and(|c| self.object_hidden(c))
                } else {
                    self.slab_point_hidden(center, None, None)
                };
                let hidden = hidden || !self.fe_item_visible((0, i));
                if !hidden {
                    for &n in &e.nodes {
                        nodes[n] = true;
                    }
                }
                hidden
            })
            .collect();
        *self.view_visibility.mesh_mask.borrow_mut() = Some((
            (
                self.mesh_generation,
                self.active_floor_only(),
                self.building.active,
            ),
            elements,
            nodes,
        ));
    }
    pub(super) fn mesh_element_hidden(&self, i: usize) -> bool {
        if !self.fe_item_visible((0, i)) {
            return true;
        }
        if !self.active_floor_only()
            && self.view_visibility.contours.is_empty()
            && (!self.building.enabled || self.building.hidden_floors.is_empty())
        {
            return false;
        }
        self.ensure_mesh_view_mask();
        self.view_visibility
            .mesh_mask
            .borrow()
            .as_ref()
            .is_some_and(|c| c.1.get(i) == Some(&true))
    }
    pub(super) fn mesh_node_view_visible(&self, n: usize) -> bool {
        if !self.fe_fragment_active()
            && !self.active_floor_only()
            && self.view_visibility.contours.is_empty()
            && (!self.building.enabled || self.building.hidden_floors.is_empty())
        {
            return true;
        }
        self.ensure_mesh_view_mask();
        self.view_visibility
            .mesh_mask
            .borrow()
            .as_ref()
            .is_none_or(|c| c.2.get(n) == Some(&true))
    }
    pub(super) fn spatial_draw_list(
        &self,
        scene: &crate::plan_trace::fem::shell::Scene,
    ) -> std::sync::Arc<DrawList> {
        let key = [
            scene.xyz.as_ptr() as usize,
            scene.surfaces.as_ptr() as usize,
            scene.beams.as_ptr() as usize,
            scene.xyz.len(),
            self.mesh_generation as usize,
            self.spatial.source_revision,
            self.view_visibility.revision,
            self.drawing_visibility.beam as usize
                | ((self.drawing_visibility.columns as usize) << 1),
        ];
        if let Some((old, list)) = &*self.view_visibility.draw_list.borrow() {
            if *old == key {
                return list.clone();
            }
        }
        let mut list = DrawList {
            nodes: vec![],
            surfaces: vec![],
            beams: vec![],
            fixed_nodes: vec![],
            node_mask: vec![false; scene.xyz.len()],
        };
        for (i, s) in scene.surfaces.iter().enumerate().filter(|(i, s)| {
            self.fe_item_visible(s.display.first().map_or((2, *i), |&d| (0, d)))
                && self.surface_view_visible(s, scene)
        }) {
            list.surfaces.push(i);
            for &n in &s.nodes {
                list.node_mask[n] = true;
            }
        }
        for (i, b) in scene
            .beams
            .iter()
            .enumerate()
            .filter(|(i, b)| self.fe_item_visible((1, *i)) && self.beam_result_visible(b.id))
        {
            list.beams.push(i);
            for &n in &b.nodes {
                list.node_mask[n] = true;
            }
        }
        list.nodes = list
            .node_mask
            .iter()
            .enumerate()
            .filter_map(|(n, &visible)| visible.then_some(n))
            .collect();
        let mut fixed = BTreeMap::<usize, u8>::new();
        for &dof in &scene.fixed {
            if list.node_mask[dof / 3] {
                *fixed.entry(dof / 3).or_default() |= 1 << (dof % 3);
            }
        }
        list.fixed_nodes = fixed.into_iter().collect();
        let list = std::sync::Arc::new(list);
        *self.view_visibility.draw_list.borrow_mut() = Some((key, list.clone()));
        list
    }
    pub(super) fn spatial_node_view_visible(
        &self,
        n: usize,
        scene: &crate::plan_trace::fem::shell::Scene,
    ) -> bool {
        scene.xyz.get(n).is_some_and(|p| self.sections.contains(*p))
            && self.spatial_draw_list(scene).node_mask.get(n) == Some(&true)
    }
    fn slab_point_hidden(&self, p: Point, floor: Option<u64>, z: Option<f64>) -> bool {
        let mut found = false;
        for c in self.contours.iter().filter(|c| {
            c.kind == Kind::Slab
                && floor.is_none_or(|f| self.building.owner(c.id) == f)
                && z.is_none_or(|z| (self.source_heights(c)[0] - z).abs() < 1e-5)
                && geometry::inside(p, &c.points)
        }) {
            found = true;
            if !self.object_hidden(c) {
                return false;
            }
        }
        found
    }
    pub(super) fn surface_view_visible(
        &self,
        s: &crate::plan_trace::fem::shell::Surface,
        scene: &crate::plan_trace::fem::shell::Scene,
    ) -> bool {
        if self.spatial.enabled
            && self.sections.active()
            && !self
                .sections
                .intersects(s.nodes.iter().map(|&n| scene.xyz[n]))
        {
            return false;
        }
        if !self.active_floor_only()
            && self.view_visibility.contours.is_empty()
            && (!self.building.enabled || self.building.hidden_floors.is_empty())
        {
            return true;
        }
        if let Some(id) = s.support {
            return self.view_contour(id).is_none_or(|c| !self.object_hidden(c));
        }
        if let Some(&i) = s.display.first() {
            return !self.mesh_element_hidden(i);
        }
        if let Some(owners) = s
            .nodes
            .first()
            .and_then(|n| self.spatial.source_surface_owners.get(n))
        {
            return owners.iter().any(|&id| {
                self.view_contour(id)
                    .is_some_and(|c| !self.object_hidden(c))
            });
        }
        if s.nodes.is_empty() {
            return false;
        }
        let p = [0, 1, 2]
            .map(|a| s.nodes.iter().map(|&n| scene.xyz[n][a]).sum::<f64>() / s.nodes.len() as f64);
        let floor = self.building.enabled.then(|| {
            self.contours
                .iter()
                .find(|c| {
                    c.kind == Kind::Slab
                        && (self.source_heights(c)[0] - p[2]).abs() < 1e-5
                        && geometry::inside([p[0], p[1]], &c.points)
                })
                .map(|c| self.building.owner(c.id))
                .unwrap_or(self.building.active)
        });
        !self.slab_point_hidden([p[0], p[1]], floor, Some(p[2]))
    }
    /// The 3D source cache rebuilds this index together with its geometry.
    /// Check ID before using an index: source edits may precede a viewer rebuild.
    pub(super) fn indexed_view_contour(&self, id: u64) -> Option<&Contour> {
        self.spatial
            .source_index
            .get(&id)
            .and_then(|&i| self.contours.get(i))
            .filter(|c| c.id == id)
    }
    pub(super) fn view_contour(&self, id: u64) -> Option<&Contour> {
        self.indexed_view_contour(id)
            .or_else(|| self.contours.iter().find(|c| c.id == id))
    }
    pub(super) fn beam_view_visible(&self, id: u64) -> bool {
        (!self.active_floor_only() || self.building.owner(id) == self.building.active)
            && !self.view_visibility.contours.contains(&id)
            && self.building.floor_visible(self.building.owner(id))
            && self.view_contour(id).is_none_or(|c| match c.kind {
                Kind::Column => self.drawing_visibility.columns,
                Kind::Beam => self.drawing_visibility.beam,
                _ => true,
            })
    }
    pub(super) fn section_view_changed(&mut self) {
        self.view_visibility.draw_list.borrow_mut().take();
        self.view_visibility.mesh_mask.borrow_mut().take();
        self.view_visibility.revision = self.view_visibility.revision.wrapping_add(1);
        self.spatial.visibility_reset();
        self.perf.load_shapes.borrow_mut().take();
        self.load_scale_key = None;
    }
    pub(super) fn view_visibility_changed(&mut self) {
        self.view_visibility.mesh_mask.borrow_mut().take();
        self.view_visibility.draw_list.borrow_mut().take();
        self.view_visibility.revision = self.view_visibility.revision.wrapping_add(1);
        self.perf.load_shapes.borrow_mut().take();
        self.load_scale_key = None;
        self.prune_selection();
        self.slab_cache = None;
        self.spatial.visibility_reset();
        self.invalidate_scene_buffers();
        self.rebuild_index();
    }
    pub(super) fn visibility_menu(&mut self, ui: &mut egui::Ui) {
        self.fe_selection.refresh(self.mesh_generation);
        let selected_fe = self.fe_mesh.is_some() && !self.fe_selection.items.is_empty();
        let mut selected = self.selection_ids.clone();
        selected.extend(self.selected);
        let mut underlays = self.underlay_selection.clone();
        underlays.extend(self.selected_underlay);
        let dimensions = self.measurements.selected.clone();
        if selected_fe || !selected.is_empty() || !underlays.is_empty() || !dimensions.is_empty() {
            if ui.button("Изолировать выделенное").clicked() {
                if selected_fe {
                    self.fragment_selected_fe(true);
                    ui.close_menu();
                    return;
                }
                self.view_visibility.contours.extend(
                    self.contours
                        .iter()
                        .filter(|c| {
                            !selected.contains(&c.id)
                                && !c.parent.is_some_and(|id| selected.contains(&id))
                        })
                        .map(|c| c.id),
                );
                self.view_visibility.underlays.extend(
                    self.underlays
                        .iter()
                        .filter(|u| !underlays.contains(&u.id))
                        .map(|u| u.id),
                );
                self.view_visibility.dimensions.extend(
                    self.measurements
                        .items
                        .iter()
                        .filter(|m| !dimensions.contains(&m.id))
                        .map(|m| m.id),
                );
                // A selected child contour must keep its owner surface visible.
                for c in &self.contours {
                    if selected.contains(&c.id) {
                        if let Some(id) = c.parent {
                            self.view_visibility.contours.remove(&id);
                        }
                    }
                }
                self.view_visibility_changed();
                ui.close_menu();
                return;
            }
            if ui.button("Скрыть выделенное").clicked() {
                if selected_fe {
                    self.fragment_selected_fe(false);
                    ui.close_menu();
                    return;
                }
                self.view_visibility.contours.extend(selected);
                self.view_visibility.underlays.extend(underlays);
                self.view_visibility.dimensions.extend(dimensions);
                self.view_visibility_changed();
                ui.close_menu();
                return;
            }
        }
        if self.fe_fragment_active()
            || !self.view_visibility.contours.is_empty()
            || !self.view_visibility.underlays.is_empty()
            || !self.view_visibility.dimensions.is_empty()
        {
            if ui.button("Показать все").clicked() {
                self.view_visibility = State {
                    active_floor: self.view_visibility.active_floor.take(),
                    revision: self.view_visibility.revision,
                    ..State::default()
                };
                self.view_visibility_changed();
                ui.close_menu();
            }
        }
        ui.separator();
    }
}

#[cfg(test)]
mod beam_visibility_index_tests {
    use super::*;
    #[test]
    fn fe_fragment_keeps_quad_halves_together_and_expires_on_new_mesh() {
        use crate::plan_trace::{fem::shell, meshing, quad_mesh};
        let mut app = App::default();
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        let mesh = quad_mesh::process(
            meshing::generate(&app.contours, 800.).unwrap(),
            quad_mesh::Options {
                merge: true,
                ..Default::default()
            },
        )
        .unwrap();
        let scene = shell::scene(
            &mesh,
            &vec![Default::default(); mesh.elements.len()],
            &app.support_height,
            800.,
            &app.contours,
        )
        .unwrap();
        let quad = scene
            .surfaces
            .iter()
            .find(|s| s.display.len() == 2)
            .unwrap();
        let [first, second] = [quad.display[0], quad.display[1]];
        app.fe_mesh = Some(mesh);
        app.fe_selection.items.insert((0, second));
        app.fragment_selected_fe(true);
        assert!(!app.mesh_element_hidden(first));
        assert!(!app.mesh_element_hidden(second));
        assert_eq!(app.spatial_draw_list(&scene).surfaces.len(), 1);
        app.fragment_selected_fe(false);
        assert!(app.mesh_element_hidden(first));
        assert!(app.mesh_element_hidden(second));
        assert!(app.spatial_draw_list(&scene).surfaces.is_empty());
        app.mesh_generation += 1;
        assert!(!app.mesh_element_hidden(first));
        assert_eq!(
            app.spatial_draw_list(&scene).surfaces.len(),
            scene.surfaces.len()
        );
    }
    #[test]
    fn isolation_compacts_draw_ids_without_renumbering_model_nodes() {
        use crate::plan_trace::fem::{element::Material, shell};
        let mut app = App::default();
        app.building.enabled = true;
        app.spatial.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Second".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        let c = |id| Contour {
            id,
            kind: Kind::Slab,
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            points: vec![[0., 0.], [1000., 0.], [0., 1000.]],
            parent: None,
            curves: vec![],
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
        };
        app.contours = vec![c(10), c(20)];
        app.building.contour_floors = BTreeMap::from([(10, 1), (20, 2)]);
        let scene = shell::Scene {
            xyz: vec![
                [0., 0., 0.],
                [1000., 0., 0.],
                [0., 1000., 0.],
                [0., 0., 3000.],
                [1000., 0., 3000.],
                [0., 1000., 3000.],
            ],
            plan: (0..6).collect(),
            surfaces: vec![
                shell::Surface {
                    rigid_z_mm: 0.,
                    nodes: vec![0, 1, 2],
                    material: Material::default(),
                    display: vec![],
                    support: None,
                },
                shell::Surface {
                    rigid_z_mm: 0.,
                    nodes: vec![3, 4, 5],
                    material: Material::default(),
                    display: vec![],
                    support: None,
                },
            ],
            fixed: std::collections::BTreeSet::from([0, 1, 2, 9, 10, 11]),
            support_nodes: BTreeMap::new(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let all = app.spatial_draw_list(&scene);
        assert_eq!(all.nodes.len(), 6);
        app.view_visibility.contours.insert(10);
        app.view_visibility_changed();
        let isolated = app.spatial_draw_list(&scene);
        assert_eq!(isolated.surfaces, vec![1]);
        assert_eq!(isolated.nodes, vec![3, 4, 5]);
        assert_eq!(isolated.fixed_nodes, vec![(3, 7)]);
        assert!(std::sync::Arc::ptr_eq(
            &isolated,
            &app.spatial_draw_list(&scene)
        ));
        assert!(!app.spatial_node_view_visible(0, &scene));
        assert!(app.spatial_node_view_visible(3, &scene));
        assert_eq!(scene.xyz.len(), 6);
        assert_eq!(scene.surfaces[1].nodes, vec![3, 4, 5]);
        app.view_visibility.contours.clear();
        app.view_visibility_changed();
        assert_eq!(app.spatial_draw_list(&scene).nodes.len(), 6);
    }
    #[test]
    fn indexed_beams_respect_flags_and_stale_indices_after_reordering() {
        let c = |id, kind| Contour {
            id,
            kind,
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            points: vec![[0., 0.], [1000., 0.]],
            parent: None,
            curves: vec![],
            grid: None,
            support_open: true,
            load: None,
            refinement: None,
        };
        let mut app = App::default();
        app.contours = vec![c(10, Kind::Beam), c(20, Kind::Column), c(30, Kind::Support)];
        app.spatial.source_index = BTreeMap::from([(10, 0), (20, 1), (30, 2)]);
        app.drawing_visibility.beam = false;
        app.drawing_visibility.columns = true;
        assert!(!app.beam_view_visible(10));
        assert!(app.beam_view_visible(20));
        assert!(app.beam_view_visible(30));
        app.contours.swap(0, 1);
        assert_eq!(app.view_contour(10).unwrap().kind, Kind::Beam);
        assert!(!app.beam_view_visible(10));
        assert!(app.beam_view_visible(20));
        app.view_visibility.contours.insert(20);
        assert!(!app.beam_view_visible(20));
    }
}
#[cfg(test)]
mod source_visibility_cache_tests {
    use super::*;
    #[test]
    fn changing_floor_visibility_preserves_source_geometry_and_owner_index() {
        let mut a = App::default();
        a.building.enabled = true;
        a.spatial.enabled = true;
        a.building.floors.push(building::Floor {
            id: 2,
            name: "Second".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        let scene = crate::plan_trace::fem::shell::Scene {
            xyz: vec![[0., 0., 0.]],
            plan: vec![0],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        a.spatial.cache = Some((1, a.support_height.clone(), vec![], false, scene));
        a.spatial.source_surface_owners.insert(0, vec![42]);
        let pointer = a.spatial.cache.as_ref().unwrap().4.xyz.as_ptr();
        let generation = a.spatial.source_revision;
        a.set_floor_visibility_from_menu(2, false);
        assert_eq!(a.spatial.cache.as_ref().unwrap().4.xyz.as_ptr(), pointer);
        assert_eq!(a.spatial.source_revision, generation);
        assert_eq!(a.spatial.source_surface_owners[&0], vec![42]);
        a.toggle_active_floor_only();
        a.toggle_active_floor_only();
        assert_eq!(a.spatial.cache.as_ref().unwrap().4.xyz.as_ptr(), pointer);
    }
}

#[cfg(test)]
mod source_hole_owner_tests {
    use super::*;
    fn c(id: u64, kind: Kind, parent: Option<u64>, x: f64, y: f64, w: f64, h: f64) -> Contour {
        Contour {
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
            curves: vec![],
            id,
            kind,
            parent,
            points: vec![[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
        }
    }

    #[test]
    fn insert_in_parent_hole_has_only_its_own_surface_owner() {
        let mut app = App::default();
        app.contours = vec![
            c(30, Kind::Slab, None, 0., 0., 100., 100.),
            c(31, Kind::Hole, Some(30), 20., 20., 20., 20.),
            c(34, Kind::Slab, None, 20., 20., 20., 20.),
        ];
        let mut ignored = c(35, Kind::Hole, Some(34), 20., 20., 20., 20.);
        ignored.placement.ignore_opening = true;
        app.contours.push(ignored);
        let mut scene = crate::plan_trace::fem::shell::Scene {
            xyz: vec![],
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let mut expected = Vec::new();
        for (id, t) in model::slab_triangles_with_owner(&app.contours) {
            let n = scene.xyz.len();
            scene.xyz.extend(t.map(|p| [p[0], p[1], 0.]));
            scene.surfaces.push(crate::plan_trace::fem::shell::Surface {
                rigid_z_mm: 0.,
                nodes: vec![n, n + 1, n + 2],
                material: Default::default(),
                display: vec![],
                support: None,
            });
            expected.push((n, id));
        }
        let owners = app.source_surface_owner_index(&scene);
        assert!(expected.iter().any(|(_, id)| *id == 34));
        for &(n, id) in &expected {
            assert_eq!(owners[&n], vec![id]);
        }

        // Overlaps retain source order for choosing a stiffness or material colour.
        app.contours
            .push(c(12, Kind::Slab, None, 20., 20., 20., 20.));
        let owners = app.source_surface_owner_index(&scene);
        for &(n, id) in expected.iter().filter(|(_, id)| *id == 34) {
            assert_eq!(owners[&n], vec![id, 12]);
        }

        app.building.enabled = true;
        app.building.floors.push(building::Floor {
            id: 2,
            name: "Second".into(),
            elevation_mm: 3000.,
            height_mm: 3000.,
        });
        app.building.contour_floors.insert(12, 2);
        let owners = app.source_surface_owner_index(&scene);
        for (n, id) in expected {
            assert_eq!(owners[&n], vec![id]);
        }
    }
}
