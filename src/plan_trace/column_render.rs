use super::*;
#[derive(Default)]
pub(super) struct MeshSections {
    rigid: bool,
    sections: Vec<(u64, f64, f64)>,
}

impl App {
    fn column_mesh_sections(&self) -> impl Iterator<Item = (u64, f64, f64)> + '_ {
        self.support_height.columns.iter().map(|(&id, section)| {
            if self.support_height.rigid_columns {
                (id, section.width_mm, section.material.thickness_mm)
            } else {
                (id, 0., 0.)
            }
        })
    }

    pub(super) fn column_signature(&self) -> MeshSections {
        MeshSections {
            rigid: self.support_height.rigid_columns,
            sections: self.column_mesh_sections().collect(),
        }
    }

    pub(super) fn column_signature_current(&self) -> bool {
        let built = &self.mesh_column_signature;
        (built.sections.is_empty() || built.rigid == self.support_height.rigid_columns)
            && built
                .sections
                .iter()
                .copied()
                .eq(self.column_mesh_sections())
    }

    pub(super) fn column_spiders(&self) -> Vec<(Point, Vec<Point>, f64)> {
        if !self.support_height.rigid_columns || !self.drawing_visibility.rigid_bodies {
            return vec![];
        }
        let layout = self.building_layout();
        let sources = self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column)
            .map(|c| (c, self.object_hidden(c)))
            .collect::<Vec<_>>();
        let key = serde_json::to_vec(&(
            self.mesh_generation,
            self.fe_mesh.is_some(),
            self.building.active,
            self.spatial.enabled,
            self.active_floor_only(),
            self.view_visibility.revision,
            &layout,
            &self.support_height.columns,
            &sources,
        ))
        .unwrap_or_default();
        if let Some((old, groups)) = &*self.column_spider_cache.borrow() {
            if *old == key {
                return groups.clone();
            }
        }
        let geometry_key =
            serde_json::to_vec(&(&self.contours, &self.support_height.columns, &layout))
                .unwrap_or_default();
        let cached = self.column_joint_cache.borrow();
        let joints = cached
            .as_ref()
            .filter(|(key, _)| *key == geometry_key)
            .map(|(_, joints)| joints.clone());
        drop(cached);
        let joints = joints.unwrap_or_else(|| {
            let joints = crate::plan_trace::columns::joints(
                &self.contours,
                &self.support_height.columns,
                layout.as_ref(),
            );
            *self.column_joint_cache.borrow_mut() = Some((geometry_key, joints.clone()));
            joints
        });
        let mut nodes_by_z = BTreeMap::<i64, Vec<Point>>::new();
        if let Some(mesh) = &self.fe_mesh {
            for (i, n) in mesh.nodes.iter().enumerate() {
                nodes_by_z
                    .entry((mesh.node_z(i) * 1e6).round() as i64)
                    .or_default()
                    .push(n.xy);
            }
        }
        let groups = joints
            .into_iter()
            .filter(|j| {
                (!self.building.enabled
                    || ((!self.active_floor_only() && self.spatial.enabled)
                        || j.floor == self.building.active)
                        && self.building.floor_visible(j.floor))
                    && self.contours.iter().any(|c| {
                        c.kind == Kind::Column
                            && !self.object_hidden(c)
                            && layout.as_ref().is_none_or(|l| {
                                let h = l.extent(c);
                                j.z >= h[0] - 1e-6 && j.z <= h[1] + 1e-6
                            })
                            && c.points.first().is_some_and(|p| {
                                p[0] >= j.bounds[0]
                                    && p[0] <= j.bounds[2]
                                    && p[1] >= j.bounds[1]
                                    && p[1] <= j.bounds[3]
                            })
                    })
            })
            .map(|j| {
                let nodes = if self.fe_mesh.is_some() {
                    nodes_by_z
                        .get(&((j.z * 1e6).round() as i64))
                        .into_iter()
                        .flatten()
                        .filter(|n| {
                            n[0] >= j.bounds[0] - 1e-6
                                && n[0] <= j.bounds[2] + 1e-6
                                && n[1] >= j.bounds[1] - 1e-6
                                && n[1] <= j.bounds[3] + 1e-6
                                && crate::plan_trace::columns::contains(&j, **n)
                        })
                        .copied()
                        .collect()
                } else {
                    j.outline.clone()
                };
                (j.master, nodes, j.z)
            })
            .collect::<Vec<_>>();
        *self.column_spider_cache.borrow_mut() = Some((key, groups.clone()));
        groups
    }
    pub(super) fn paint_columns(&self, painter: &egui::Painter, rect: Rect) {
        let start = self.draw_order.start(painter);
        for c in self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column && self.contour_visible(c))
        {
            if let Some(&p) = c.points.first() {
                let at = self.camera.screen(p, rect);
                let ink = if self.is_selected(c.id) {
                    Color32::YELLOW
                } else {
                    self.object_ink(c.id, c.kind)
                };
                painter.rect_filled(Rect::from_center_size(at, egui::vec2(9., 9.)), 0., ink);
            }
        }
        let ink = self.drawing_visibility.rigid_color;
        let ink = Color32::from_rgb(ink[0], ink[1], ink[2]);
        for (master, nodes, _) in self.column_spiders() {
            for p in nodes {
                draw_path(
                    painter,
                    &[master, p],
                    false,
                    self.camera,
                    rect,
                    Stroke::new(1_f32, ink),
                );
            }
        }
        self.draw_order.finish(painter, 19, start);
    }
    pub(super) fn column_world(
        &self,
        center: [f64; 3],
        extent: f64,
        lengths: &BTreeMap<u64, Vec<(f64, f64, Color32, Option<[Point; 2]>)>>,
        out: &mut Vec<spatial_depth::Vertex>,
    ) {
        if !self.drawing_visibility.support_extents {
            return;
        }
        for c in self
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Column && self.contour_visible(c))
        {
            let (Some(&p), Some(section)) =
                (c.points.first(), self.support_height.columns.get(&c.id))
            else {
                continue;
            };
            let q = crate::plan_trace::columns::section_corners(c, section);
            let z = self.source_heights(c);
            let lo = z[0];
            let hi = *z.last().unwrap();
            let ink = if self.is_selected(c.id) {
                Color32::YELLOW
            } else {
                self.object_ink(c.id, c.kind)
            };
            let fallback = [(lo, hi, ink, None)];
            let spans = lengths
                .get(&c.id)
                .map_or(fallback.as_slice(), Vec::as_slice);
            for &(lo, hi, color, _) in spans {
                let vertex = |p| spatial_world::vertex(p, color, center, extent);
                for i in 0..4 {
                    let a = q[i];
                    let b = q[(i + 1) % 4];
                    let face = [
                        [a[0], a[1], lo],
                        [b[0], b[1], lo],
                        [b[0], b[1], hi],
                        [a[0], a[1], hi],
                    ];
                    for ids in [[0, 1, 2], [0, 2, 3]] {
                        out.extend(ids.map(|i| vertex(face[i])));
                    }
                }
                for z in [lo, hi] {
                    for ids in [[0, 1, 2], [0, 2, 3]] {
                        out.extend(ids.map(|i| vertex([q[i][0], q[i][1], z])));
                    }
                }
            }
        }
    }
    pub(super) fn column_overlay(
        &self,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        painter: &egui::Painter,
    ) {
        let mut b = spatial_depth::Builder::new(rect, projected);
        let color = self.drawing_visibility.rigid_color;
        let ink = Color32::from_rgb(color[0], color[1], color[2]);
        for (master, nodes, z) in self.column_spiders() {
            for p in nodes {
                b.line(
                    [project([master[0], master[1], z]), project([p[0], p[1], z])],
                    1.,
                    ink,
                );
            }
        }
        // Without a mesh, the cached world ribbons already contain the axes.
        for c in self
            .contours
            .iter()
            .filter(|c| self.fe_mesh.is_some() && c.kind == Kind::Column && self.contour_visible(c))
        {
            let Some(&p) = c.points.first() else {
                continue;
            };
            let heights = self.source_heights(c);
            let lo = heights[0];
            let hi = *heights.last().unwrap();
            let has_force_result = self.show_results
                && self.punching_view.is_none()
                && (17..=22).contains(&self.result_field)
                && self
                    .selected_result()
                    .and_then(|r| r.shell.as_ref())
                    .is_some_and(|s| s.beam_forces.iter().any(|beam| beam.id == c.id));
            if !self.drawing_visibility.support_extents && !has_force_result {
                let axis = [project([p[0], p[1], lo]), project([p[0], p[1], hi])];
                if self.drawing_visibility.structure_edges {
                    b.line(axis, 5., super::spatial_world::STRUCTURE_EDGE_INK);
                }
                b.line(axis, 3., self.object_ink(c.id, Kind::Column));
            }
        }
        if self.drawing_visibility.support_extents && !self.drawing_visibility.structure_edges {
            for c in self
                .contours
                .iter()
                .filter(|c| c.kind == Kind::Column && self.contour_visible(c))
            {
                let ink = self.object_ink(c.id, Kind::Column);
                let (Some(&p), Some(section)) =
                    (c.points.first(), self.support_height.columns.get(&c.id))
                else {
                    continue;
                };
                let q = crate::plan_trace::columns::section_corners(c, section);
                for z in self.source_heights(c) {
                    for i in 0..4 {
                        let a = project([q[i][0], q[i][1], z]);
                        let c = project([q[(i + 1) % 4][0], q[(i + 1) % 4][1], z]);
                        let len = a.0.distance(c.0);
                        let mut x = 0.;
                        while x < len {
                            let at = |d: f32| {
                                (
                                    a.0 + (c.0 - a.0) * (d / len),
                                    a.1 + (c.1 - a.1) * (d / len) as f64,
                                )
                            };
                            b.line([at(x), at((x + 6.).min(len))], 1., ink);
                            x += 10.;
                        }
                    }
                }
            }
        }
        spatial_depth::paint_layer(
            painter,
            rect,
            std::sync::Arc::new(b.vertices),
            spatial_depth::Layer::Columns,
        );
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn column_tool_defaults_selection_and_locks() {
        let mut app = App::default();
        app.begin(Kind::Column);
        app.sketch_click([1000., 2000.], false);
        assert_eq!(app.contours.len(), 1);
        assert_eq!(app.contours[0].kind, Kind::Column);
        assert_eq!(app.contours[0].points, vec![[1000., 2000.]]);
        app.sync_stiffness_sources();
        let section = &app.support_height.columns[&app.contours[0].id];
        assert_eq!(section.width_mm, 250.);
        assert_eq!(section.material.thickness_mm, 750.);
        let c = app.contours[0].clone();
        assert!(app.selectable(&c));
        app.columns_selectable = false;
        assert!(!app.selectable(&c));
        app.columns_selectable = true;
        app.set_contour_lock(true);
        assert!(app.is_kind_locked(Kind::Column));
        app.set_contour_lock(false);
        assert!(!app.is_kind_locked(Kind::Column));
        app.stiffnesses.validate(&app.contours).unwrap();
    }
}
