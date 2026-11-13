//! Presentation of existing result values in the spatial scene; no solver changes.
use super::*;
use crate::plan_trace::fem::shell;

pub(super) struct SpatialHit {
    pub depth: f64,
    pub text: Option<String>,
}

pub(super) fn pick_triangle(point: Pos2, triangle: [(Pos2, f64); 3]) -> Option<([f64; 3], f64)> {
    let [a, b, c] = triangle.map(|p| p.0);
    let det = (b.x - a.x) as f64 * (c.y - a.y) as f64 - (b.y - a.y) as f64 * (c.x - a.x) as f64;
    if det.abs() < 1e-8 {
        return None;
    }
    let u = ((point.x - a.x) as f64 * (c.y - a.y) as f64
        - (point.y - a.y) as f64 * (c.x - a.x) as f64)
        / det;
    let v = ((b.x - a.x) as f64 * (point.y - a.y) as f64
        - (b.y - a.y) as f64 * (point.x - a.x) as f64)
        / det;
    let weights = [1. - u - v, u, v];
    weights.iter().all(|&x| x >= -1e-6).then(|| {
        let depth = (0..3).map(|i| weights[i] * triangle[i].1).sum();
        (weights, depth)
    })
}

/// Screen-space buckets avoid testing every face against every label.
pub(super) struct Occlusion {
    triangles: Vec<[(Pos2, f64); 3]>,
    bins: std::collections::BTreeMap<(i32, i32), Vec<usize>>,
    epsilon: f64,
}
impl Occlusion {
    pub fn new(rect: Rect, scene: &shell::Scene, projected: &[(Pos2, f64)]) -> Self {
        Self::with_visibility(rect, scene, projected, None)
    }
    pub fn new_visible(
        rect: Rect,
        scene: &shell::Scene,
        projected: &[(Pos2, f64)],
        app: &App,
    ) -> Self {
        Self::with_visibility(rect, scene, projected, Some(app))
    }
    fn with_visibility(
        rect: Rect,
        scene: &shell::Scene,
        projected: &[(Pos2, f64)],
        app: Option<&App>,
    ) -> Self {
        let draw = app.map(|a| a.spatial_draw_list(scene));
        let depths = draw
            .as_ref()
            .map(|v| v.nodes.iter().map(|&n| projected[n].1).collect::<Vec<_>>())
            .unwrap_or_else(|| projected.iter().map(|p| p.1).collect());
        let lo = depths.iter().copied().fold(f64::INFINITY, f64::min);
        let hi = depths.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        let mut result = Self {
            triangles: vec![],
            bins: Default::default(),
            epsilon: (hi - lo).abs().max(1.) * 1e-7,
        };
        let ids: Box<dyn Iterator<Item = usize> + '_> = if let Some(list) = &draw {
            Box::new(list.surfaces.iter().copied())
        } else {
            Box::new(0..scene.surfaces.len())
        };
        // The draw list also supplies wireframes and FE selection. Their slabs
        // have no filled depth unless the fill or a surface mosaic is drawn.
        let fill_disabled =
            app.filter(|app| !app.drawing_visibility.fill && app.schema_analysis.mode.is_none());
        let slab_result_fill = fill_disabled.is_some_and(|app| {
            app.show_results
                && app.punching_view.is_none()
                && app.fe_mesh.is_some()
                && app.spatial_result_range().is_some()
        });
        for index in ids {
            let face = &scene.surfaces[index];
            if face.support.is_none()
                && fill_disabled.is_some_and(|app| {
                    !slab_result_fill || app.spatial_surface_values(index, face).is_none()
                })
            {
                continue;
            }
            for i in 1..face.nodes.len() - 1 {
                let t = [
                    projected[face.nodes[0]],
                    projected[face.nodes[i]],
                    projected[face.nodes[i + 1]],
                ];
                let bounds = Rect::from_points(&t.map(|p| p.0)).intersect(rect);
                if !bounds.is_positive() {
                    continue;
                }
                let id = result.triangles.len();
                result.triangles.push(t);
                for x in
                    (bounds.left() / 64.).floor() as i32..=(bounds.right() / 64.).floor() as i32
                {
                    for y in
                        (bounds.top() / 64.).floor() as i32..=(bounds.bottom() / 64.).floor() as i32
                    {
                        result.bins.entry((x, y)).or_default().push(id);
                    }
                }
            }
        }
        result
    }
    pub fn visible(&self, p: Pos2, depth: f64) -> bool {
        if !spatial_depth::point_visible(p, depth) {
            return false;
        }
        !self
            .bins
            .get(&((p.x / 64.).floor() as i32, (p.y / 64.).floor() as i32))
            .into_iter()
            .flatten()
            .any(|&i| {
                pick_triangle(p, self.triangles[i]).is_some_and(|(_, front)| {
                    front > depth + self.epsilon && spatial_depth::point_visible(p, front)
                })
            })
    }
}
impl App {
    fn membrane_surface_factor(&self, thickness_mm: f64) -> f64 {
        if self.membrane_area && matches!(self.result_field, 12..=14) {
            if thickness_mm.is_finite() && thickness_mm > 0. {
                1000. / thickness_mm
            } else {
                f64::NAN
            }
        } else {
            1.
        }
    }
    pub(super) fn spatial_surface_values(
        &self,
        surface_index: usize,
        surface: &shell::Surface,
    ) -> Option<Vec<f64>> {
        let case = self.selected_result()?;
        let field = self.result_field;
        if (field == 15 || crate::plan_trace::fem::displacement_field(field))
            && !self.point_displacements()
        {
            return surface
                .nodes
                .iter()
                .map(|&node| {
                    if field == crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD {
                        let value = self.result_node_value(case, node);
                        return value.is_finite().then_some(value);
                    }
                    if let Some(shell) = case.shell.as_ref() {
                        if matches!(field, 0 | 10 | 11) {
                            let xyz = shell.translations_mm.get(node)?;
                            return Some(match field {
                                0 => -xyz[2],
                                10 => xyz[0],
                                _ => xyz[1],
                            });
                        }
                    }
                    if matches!(field, 23..=25) {
                        if let Some(shell) = &case.shell {
                            return Some(shell.rotations_rad.get(node)?[field - 23]);
                        }
                    }
                    if node >= case.w_mm.len() {
                        return None;
                    }
                    Some(self.result_node_value(case, node))
                })
                .collect();
        }
        if !matches!(field,1..=5|7|12..=14|16|26..=34) {
            return None;
        }
        if surface.support.is_some() {
            if (26..=32).contains(&field) {
                let result = self.reinforcement.result.as_ref()?;
                let value =
                    result.wall_value_for(field, surface_index, self.reinforcement_area_view)?;
                return Some(vec![value; surface.nodes.len()]);
            }
            let force = case.shell.as_ref()?.surface_forces.get(surface_index)?;
            let value = force.scalar(field)?
                * self.result_display_factor()
                * self.membrane_surface_factor(force.material.thickness_mm);
            if self.iso_efforts {
                return self.spatial_wall_corners().get(surface_index).cloned();
            }
            return Some(vec![value; surface.nodes.len()]);
        }
        let &display = surface.display.first()?;
        let mesh = self.fe_mesh.as_ref()?;
        mesh.elements.get(display)?;
        if let Some(recovered) = self.smoothed_efforts() {
            // Average contributions of both display triangles for a physical Q4 corner.
            return surface
                .nodes
                .iter()
                .map(|node| {
                    let mut sum = 0.;
                    let mut count = 0.;
                    for &index in &surface.display {
                        let e = mesh.elements.get(index)?;
                        if let Some(local) = e.nodes.iter().position(|n| n == node) {
                            sum += recovered.get(index)?[local];
                            count += 1.;
                        }
                    }
                    (count > 0.).then_some(sum / count)
                })
                .collect();
        }
        let value = self.result_element_value(case, display) * self.result_display_factor();
        if matches!(field, 33 | 34) && !value.is_finite() {
            return None;
        }
        Some(vec![value; surface.nodes.len()])
    }

    pub(super) fn spatial_result_text(&self, owner: &str, value: f64) -> String {
        if !value.is_finite() {
            return format!("{owner} · НДМ не подтверждено (см. замечания подбора)");
        }

        format!(
            "{owner} · {}: {}",
            self.result_field_label(self.result_field),
            self.display_text.number(value)
        )
    }

    pub(super) fn spatial_point_results(
        &self,
        scene: &shell::Scene,
    ) -> std::sync::Arc<Vec<(usize, f64)>> {
        let key = self.view_key();
        if let Some((old, values)) = self.spatial.point_values.borrow().as_ref() {
            if *old == key {
                return values.clone();
            }
        }
        let values = std::sync::Arc::new(self.spatial_point_results_uncached(scene));
        *self.spatial.point_values.borrow_mut() = Some((key, values.clone()));
        values
    }
    pub(super) fn spatial_visible_point_results(
        &self,
        scene: &shell::Scene,
    ) -> std::sync::Arc<Vec<(usize, f64)>> {
        let key = self.view_key();
        let draw = self.spatial_draw_list(scene);
        if let Some((old, owner, values)) = &*self.spatial.visible_point_values.borrow() {
            if *old == key && std::sync::Arc::ptr_eq(owner, &draw) {
                return values.clone();
            }
        }
        let values = std::sync::Arc::new(
            self.spatial_point_results(scene)
                .iter()
                .copied()
                .filter(|(n, _)| draw.contains_node(*n))
                .collect::<Vec<_>>(),
        );
        *self.spatial.visible_point_values.borrow_mut() = Some((key, draw, values.clone()));
        values
    }
    fn spatial_point_results_uncached(&self, scene: &shell::Scene) -> Vec<(usize, f64)> {
        let Some(case) = self.selected_result() else {
            return vec![];
        };
        if self.point_displacements() {
            return (0..scene.xyz.len())
                .filter_map(|node| {
                    let value =
                        if self.result_field == crate::plan_trace::fem::DISPLACEMENT_NORM_FIELD {
                            self.result_node_value(case, node)
                        } else if matches!(self.result_field, 0 | 10 | 11) {
                            let xyz = case.shell.as_ref()?.translations_mm.get(node)?;
                            match self.result_field {
                                0 => -xyz[2],
                                10 => xyz[0],
                                _ => xyz[1],
                            }
                        } else {
                            case.shell.as_ref()?.rotations_rad.get(node)?[self.result_field - 23]
                        };
                    value.is_finite().then_some((node, value))
                })
                .collect();
        }
        if self.result_field != 6 {
            return vec![];
        }
        let mesh = match self.fe_mesh.as_ref() {
            Some(m) => m,
            None => return vec![],
        };
        if let Some(shell) = case
            .shell
            .as_ref()
            .filter(|s| s.restrained_reactions_kn.len() == scene.xyz.len())
        {
            let factor = self.result_display_factor();
            let mut groups = std::collections::BTreeMap::<
                (u64, i64, bool),
                std::collections::BTreeSet<usize>,
            >::new();
            let explicit_restraints = self
                .contours
                .iter()
                .filter(|c| c.kind.is_restraint())
                .map(|c| c.id)
                .collect::<std::collections::BTreeSet<_>>();
            let mut restraint_groups = Vec::new();
            let mut supported = vec![false; scene.xyz.len()];
            for (&id, nodes) in &scene.support_nodes {
                if !self.beam_view_visible(id) {
                    continue;
                }
                if explicit_restraints.contains(&id) {
                    restraint_groups.push((id, nodes));
                    continue;
                }
                for &n in nodes {
                    if scene.fixed.contains(&(3 * n + 2)) {
                        supported[n] = true;
                        groups
                            .entry((id, (scene.xyz[n][2] * 1e6).round() as i64, true))
                            .or_default()
                            .insert(n);
                    }
                }
            }
            for (id, nodes) in restraint_groups {
                for &n in nodes {
                    if !supported[n] && scene.fixed.contains(&(3 * n + 2)) {
                        groups
                            .entry((id, (scene.xyz[n][2] * 1e6).round() as i64, true))
                            .or_default()
                            .insert(n);
                    }
                }
            }
            if shell.plate_transfer_kn.len() == scene.xyz.len() {
                for (&id, nodes) in &mesh.support_boundary {
                    if !self.beam_view_visible(id) {
                        continue;
                    }
                    for (n, p) in scene.xyz.iter().enumerate() {
                        if scene.fixed.contains(&(3 * n + 2)) {
                            continue;
                        }
                        let Some(&plan) = scene.plan.get(n) else {
                            continue;
                        };
                        if !nodes.contains(&plan) || plan >= mesh.nodes.len() {
                            continue;
                        }
                        let xy = mesh.nodes[plan].xy;
                        if (p[0] - xy[0]).hypot(p[1] - xy[1]) > 1e-6
                            || (p[2] - mesh.node_z(plan)).abs() > 1e-6
                        {
                            continue;
                        }
                        groups
                            .entry((id, (p[2] * 1e6).round() as i64, false))
                            .or_default()
                            .insert(n);
                    }
                }
            }
            let value = |n: usize, foundation: bool| {
                if foundation {
                    -shell.restrained_reactions_kn[n][2] * factor
                } else {
                    shell.plate_transfer_kn[n] * factor
                }
            };
            if !self.sum_reactions {
                let mut unique = std::collections::BTreeMap::new();
                for ((_, _, foundation), nodes) in &groups {
                    for &n in nodes {
                        unique.insert(n, value(n, *foundation));
                    }
                }
                return unique.into_iter().collect();
            }
            let mut out = vec![];
            for ((_, z, foundation), nodes) in &groups {
                let total = nodes
                    .iter()
                    .map(|&n| {
                        let count = groups
                            .iter()
                            .filter(|((_, level, base), ns)| {
                                level == z && base == foundation && ns.contains(&n)
                            })
                            .count()
                            .max(1);
                        value(n, *foundation) / count as f64
                    })
                    .sum::<f64>();
                out.extend(nodes.iter().map(|&n| (n, total)));
            }
            return out;
        }
        let totals = self.support_reaction_totals();
        // Legacy/flat results without physical-node reactions.
        let mut out = Vec::new();
        for (&id, nodes) in &mesh.support_boundary {
            for &node in nodes {
                let plan = node;
                let value = if self.sum_reactions {
                    let Some(total) = totals.iter().find(|t| t.id == id) else {
                        continue;
                    };
                    total.value
                } else {
                    let membership = mesh
                        .support_boundary
                        .values()
                        .filter(|v| v.contains(&plan))
                        .count()
                        .max(1);
                    case.reaction_kn.get(plan).copied().unwrap_or(0.) * self.result_display_factor()
                        / membership as f64
                };
                out.push((node, value));
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn projected_triangle_interpolates_values_and_front_depth() {
        let triangle = [
            (Pos2::new(0., 0.), 2.),
            (Pos2::new(100., 0.), 4.),
            (Pos2::new(0., 100.), 6.),
        ];
        let (weights, depth) = pick_triangle(Pos2::new(25., 25.), triangle).unwrap();
        assert!((weights[0] - 0.5).abs() < 1e-12);
        assert!((weights[1] - 0.25).abs() < 1e-12);
        assert!((weights[2] - 0.25).abs() < 1e-12);
        assert!((depth - 3.5).abs() < 1e-12);
        assert!(pick_triangle(Pos2::new(80., 80.), triangle).is_none());
        let front = triangle.map(|(p, z)| (p, z + 10.));
        assert!(pick_triangle(Pos2::new(25., 25.), front).unwrap().1 > depth);
    }
    #[test]
    fn edge_on_face_does_not_generate_nan_hover() {
        let triangle = [
            (Pos2::new(0., 0.), 0.),
            (Pos2::new(50., 0.), 1.),
            (Pos2::new(100., 0.), 2.),
        ];
        assert!(pick_triangle(Pos2::new(25., 0.), triangle).is_none());
    }

    #[test]
    fn spatial_wall_reaction_does_not_split_with_its_explicit_restraint() {
        use crate::plan_trace::{fem, loads, model::LevelAnchor};

        let mut app = App::default();
        if let Some(path) = std::env::var_os("MINI_FEM_REACTION_PROJECT") {
            app.load_project_path(path.into()).unwrap();
        } else {
            let contour = |id, kind, points: Vec<Point>| -> Contour {
                serde_json::from_value(serde_json::json!({
                    "id": id, "kind": kind, "points": points, "curves": [],
                    "support_open": kind == "Support" || kind == "Restraint",
                    "restraint_mask": 7,
                }))
                .unwrap()
            };
            app.building.enabled = true;
            app.support_height.automatic_building_bottom = false;
            app.support_height.automatic_building_top = false;
            let mut slab = contour(
                1,
                "Slab",
                vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
            );
            slab.placement.slab.anchor = LevelAnchor::Top;
            let mut load = contour(4, "LoadArea", slab.points.clone());
            load.placement.slab.anchor = LevelAnchor::Top;
            load.load = Some(loads::LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                case_id: 1,
                name: String::new(),
                magnitude: loads::Magnitude {
                    value: 1.,
                    units: loads::ForceUnits::TonForce,
                },
                mesh_constraint: false,
                closed: true,
            });
            app.contours = vec![
                slab,
                contour(2, "Support", vec![[0., 0.], [0., 2000.]]),
                contour(3, "Restraint", vec![[0., 0.], [0., 2000.]]),
                load,
                contour(5, "Support", vec![[2000., 0.], [4000., 0.]]),
                contour(6, "Restraint", vec![[2000., 0.], [4000., 0.]]),
            ];
        }
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let mesh = crate::plan_trace::building_mesh::generate(
            &app.contours,
            app.mesh_size,
            app.building_layout().unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let analysis = fem::solve_with_support_height(
            &mesh,
            &app.contours,
            &app.materials,
            &app.load_cases,
            app.gravity,
            app.mesh_size,
            app.element_type,
            &app.support_height,
        )
        .unwrap();
        let scene = shell::scene(
            &mesh,
            &vec![fem::element::Material::default(); mesh.elements.len()],
            &app.support_height,
            app.mesh_size,
            &app.contours,
        )
        .unwrap();
        app.fe_mesh = Some(mesh);
        app.analysis = Some(analysis);
        app.spatial.enabled = true;
        app.active_load = 1;
        app.result_by_combinations = false;
        app.combination_variant = crate::plan_trace::load_combinations::Variant::Design;
        app.load_units = loads::ForceUnits::TonForce;
        app.result_field = 6;
        app.sum_reactions = true;
        let case = app.selected_result().unwrap();
        let shell = case.shell.as_ref().unwrap();
        let values = app.spatial_point_results_uncached(&scene);
        let original_labels = values.len();
        let factor = app.result_display_factor();
        assert!(case.balance_kn.abs() < 1e-6 * case.applied_kn.abs().max(1.));

        let walls = app
            .contours
            .iter()
            .filter(|c| c.kind == Kind::Support)
            .collect::<Vec<_>>();
        assert!(!walls.is_empty());
        for wall in walls {
            let base = scene.support_nodes[&wall.id]
                .iter()
                .copied()
                .filter(|&n| scene.fixed.contains(&(3 * n + 2)))
                .collect::<std::collections::BTreeSet<_>>();
            let expected = -factor
                * base
                    .iter()
                    .map(|&n| shell.restrained_reactions_kn[n][2])
                    .sum::<f64>();
            let upper_transfer = factor
                * app.fe_mesh.as_ref().unwrap().support_boundary[&wall.id]
                    .iter()
                    .map(|&n| shell.plate_transfer_kn[n])
                    .sum::<f64>();
            let displayed = values
                .iter()
                .filter_map(|(n, v)| base.contains(n).then_some(*v))
                .collect::<Vec<_>>();
            println!(
                "wall {}: transfer={upper_transfer:.6}, base={expected:.6}, first label={:?}",
                wall.id,
                displayed.first(),
            );
            assert!(!base.is_empty());
            assert!((upper_transfer - expected).abs() < 1e-6);
            assert_eq!(displayed.len(), base.len());
            assert!(displayed.iter().all(|v| (v - expected).abs() < 1e-8));
        }

        // Give one restraint an additional node outside the wall base. Its
        // independent reaction remains available after the common nodes are omitted.
        let restraint = app
            .contours
            .iter()
            .find(|c| c.kind == Kind::Restraint)
            .unwrap()
            .id;
        let mut scene = scene;
        let standalone = scene.xyz.len();
        scene.xyz.push([10000., 10000., 0.]);
        scene.plan.push(usize::MAX);
        scene
            .support_nodes
            .get_mut(&restraint)
            .unwrap()
            .push(standalone);
        scene.fixed.insert(3 * standalone + 2);
        let shell = app
            .analysis
            .as_mut()
            .unwrap()
            .cases
            .iter_mut()
            .find(|case| case.id == 1)
            .and_then(|case| case.shell.as_mut())
            .unwrap();
        shell.restrained_reactions_kn.push([0., 0., 20.]);
        shell.plate_transfer_kn.push(0.);
        let values = app.spatial_point_results_uncached(&scene);
        assert_eq!(values.len(), original_labels + 1);
        assert!(values.contains(&(standalone, -20. * factor)));
    }
}

/// Area-weighted recovery along shared, coplanar wall edges only.
fn recover_wall_corners(
    forces: &[crate::plan_trace::fem::shell::SurfaceForces],
    field: usize,
    factor: f64,
) -> Vec<Vec<f64>> {
    use std::collections::BTreeMap;
    let mut offsets = vec![0];
    for face in forces {
        offsets.push(offsets.last().unwrap() + face.nodes.len());
    }
    let mut parents = (0..*offsets.last().unwrap()).collect::<Vec<_>>();
    fn root(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    let mut edges = BTreeMap::<[usize; 2], Vec<(usize, [usize; 2])>>::new();
    for (index, face) in forces
        .iter()
        .enumerate()
        .filter(|(_, f)| f.support.is_some())
    {
        for corner in 0..face.nodes.len() {
            let next = (corner + 1) % face.nodes.len();
            let (a, b) = (face.nodes[corner], face.nodes[next]);
            let (key, ids) = if a < b {
                ([a, b], [offsets[index] + corner, offsets[index] + next])
            } else {
                ([b, a], [offsets[index] + next, offsets[index] + corner])
            };
            for &(other, previous) in edges.get(&key).into_iter().flatten() {
                let adjacent = &forces[other];
                if adjacent.support != face.support || adjacent.material != face.material {
                    continue;
                }
                if (0..3).any(|axis| {
                    (0..3)
                        .map(|j| face.axes[axis][j] * adjacent.axes[axis][j])
                        .sum::<f64>()
                        < 1. - 1e-8
                }) {
                    continue;
                }
                for j in 0..2 {
                    let a = root(&mut parents, ids[j]);
                    let b = root(&mut parents, previous[j]);
                    parents[b] = a;
                }
            }
            edges.entry(key).or_default().push((index, ids));
        }
    }
    let mut sums = vec![[0.; 2]; parents.len()];
    for (index, face) in forces
        .iter()
        .enumerate()
        .filter(|(_, f)| f.support.is_some())
    {
        let Some(value) = face.scalar(field) else {
            continue;
        };
        for corner in 0..face.nodes.len() {
            let id = root(&mut parents, offsets[index] + corner);
            sums[id][0] += value * face.area;
            sums[id][1] += face.area;
        }
    }
    forces
        .iter()
        .enumerate()
        .map(|(index, face)| {
            if face.support.is_none() {
                return vec![];
            }
            (0..face.nodes.len())
                .map(|corner| {
                    let [sum, weight] = sums[root(&mut parents, offsets[index] + corner)];
                    if weight > 0. {
                        sum / weight * factor
                    } else {
                        face.scalar(field).unwrap_or(0.) * factor
                    }
                })
                .collect()
        })
        .collect()
}
impl App {
    pub(super) fn spatial_wall_corners(&self) -> std::sync::Arc<Vec<Vec<f64>>> {
        let key = self.view_key();
        if let Some((old, corners)) = &*self.spatial.wall_cache.borrow() {
            if *old == key {
                return corners.clone();
            }
        }
        let corners = std::sync::Arc::new(
            self.selected_result()
                .and_then(|r| r.shell.as_ref())
                .map_or_else(Vec::new, |shell| {
                    let mut corners = recover_wall_corners(
                        &shell.surface_forces,
                        self.result_field,
                        self.result_display_factor(),
                    );
                    for (values, face) in corners.iter_mut().zip(&shell.surface_forces) {
                        let factor = self.membrane_surface_factor(face.material.thickness_mm);
                        for value in values {
                            *value *= factor;
                        }
                    }
                    corners
                }),
        );
        *self.spatial.wall_cache.borrow_mut() = Some((key, corners.clone()));
        corners
    }
    pub(super) fn spatial_result_range(&self) -> Option<[f64; 2]> {
        Some(crate::plan_trace::result_scale::range(
            self.spatial_result_extrema()?.into_iter(),
        ))
    }

    pub(super) fn spatial_mesh_scene(&self) -> Option<&shell::Scene> {
        self.spatial
            .cache
            .as_ref()
            .filter(|(generation, _, _, meshed, _)| *meshed && *generation == self.mesh_generation)
            .map(|(_, _, _, _, scene)| scene)
    }

    pub(super) fn spatial_result_extrema(&self) -> Option<[f64; 2]> {
        if matches!(self.result_field, 7..=9 | 17..=22) {
            return self.result_extrema_cached();
        }
        self.selected_result()?;
        let scene = self.spatial_mesh_scene()?;
        let key = self.view_key();
        if let Some((cached, extrema)) = &*self.spatial.range_cache.borrow() {
            if *cached == key {
                return Some(*extrema);
            }
        }
        let draw = self.spatial_draw_list(scene);
        let mut extrema = [f64::INFINITY, f64::NEG_INFINITY];
        let mut include = |v: f64| {
            if v.is_finite() {
                extrema[0] = extrema[0].min(v);
                extrema[1] = extrema[1].max(v);
            }
        };
        if self.result_field == 6 || self.point_displacements() {
            for &(node, value) in self.spatial_visible_point_results(scene).iter() {
                if self.sections.contains(scene.xyz[node]) {
                    include(value);
                }
            }
        } else {
            let clipped = self.sections.active();
            for &index in &draw.surfaces {
                let surface = &scene.surfaces[index];
                let Some(values) = self.spatial_surface_values(index, surface) else {
                    continue;
                };
                if !clipped {
                    values.into_iter().for_each(&mut include);
                    continue;
                }
                let mut points = [[0.; 3]; 4];
                let Some(points) = points.get_mut(..surface.nodes.len()) else {
                    continue;
                };
                for (point, &node) in points.iter_mut().zip(&surface.nodes) {
                    *point = scene.xyz[node];
                }
                include_section_surface_values(&self.sections, points, &values, &mut include);
            }
        }
        if !extrema[0].is_finite() {
            extrema = [0., 0.];
        }
        *self.spatial.range_cache.borrow_mut() = Some((key, extrema));
        Some(extrema)
    }
}

fn include_section_surface_values(
    sections: &sections::State,
    points: &[[f64; 3]],
    values: &[f64],
    mut include: impl FnMut(f64),
) {
    if points.iter().all(|&point| sections.contains(point)) {
        values.iter().copied().for_each(include);
        return;
    }
    match (points, values) {
        (&[a, b, c], &[va, vb, vc]) => {
            sections.clip_triangle_values([a, b, c], [va, vb, vc], include);
        }
        (&[a, b, c, d], &[va, vb, vc, vd]) if va == vb && va == vc && va == vd => {
            sections.clip_triangle_values([a, b, c], [va; 3], &mut include);
            sections.clip_triangle_values([a, c, d], [va; 3], include);
        }
        (&[a, b, c, d], &[_, _, _, _]) => {
            // Use the same Q4 interpolation as the displayed isofield.
            let uv = [
                (Pos2::new(0., 0.), 0.),
                (Pos2::new(1., 0.), 0.),
                (Pos2::new(1., 1.), 0.),
                (Pos2::new(0., 1.), 0.),
            ];
            for (triangle, values) in surface_triangles(&uv, values) {
                let world = triangle.map(|(p, _)| {
                    let (u, v) = (f64::from(p.x), f64::from(p.y));
                    std::array::from_fn(|i| {
                        (1. - u) * (1. - v) * a[i]
                            + u * (1. - v) * b[i]
                            + u * v * c[i]
                            + (1. - u) * v * d[i]
                    })
                });
                sections.clip_triangle_values(world, values, &mut include);
            }
        }
        _ => {}
    }
}

/// Bilinear Q4 field/geometry sampling, linear T3; no change to physical FE topology.
pub(super) fn surface_triangles(
    points: &[(Pos2, f64)],
    values: &[f64],
) -> Vec<([(Pos2, f64); 3], [f64; 3])> {
    surface_triangles_subdivided(points, values, 4)
}

/// Linear/near-linear Q4 fields need no fixed 4x4 rendering grid.
/// For a bilinear field the maximum diagonal interpolation error is |mixed|/4.
pub(super) fn surface_world_triangles(
    points: &[(Pos2, f64)],
    values: &[f64],
    range: [f64; 2],
) -> Vec<([(Pos2, f64); 3], [f64; 3])> {
    let mixed = if values.len() == 4 {
        (values[0] - values[1] + values[2] - values[3]).abs()
    } else {
        0.
    };
    let tolerance = (range[1] - range[0]).abs().max(1e-12) * 1e-4;
    let subdivisions = if mixed / 4. <= tolerance {
        1
    } else if mixed / 16. <= tolerance {
        2
    } else {
        4
    };
    surface_triangles_subdivided(points, values, subdivisions)
}
fn surface_triangles_subdivided(
    points: &[(Pos2, f64)],
    values: &[f64],
    subdivisions: usize,
) -> Vec<([(Pos2, f64); 3], [f64; 3])> {
    if values.len() != points.len() {
        return vec![];
    }
    if points.len() == 3 {
        return vec![(
            [points[0], points[1], points[2]],
            [values[0], values[1], values[2]],
        )];
    }
    if points.len() != 4 || values.len() != 4 {
        return vec![];
    }
    if values.iter().all(|&value| value == values[0]) {
        return [[0, 1, 2], [0, 2, 3]]
            .into_iter()
            .map(|ids| (ids.map(|i| points[i]), ids.map(|i| values[i])))
            .collect();
    }
    let at = |x: f64, y: f64| {
        let w = [(1. - x) * (1. - y), x * (1. - y), x * y, (1. - x) * y];
        let screen = Pos2::new(
            (0..4).map(|i| points[i].0.x as f64 * w[i]).sum::<f64>() as f32,
            (0..4).map(|i| points[i].0.y as f64 * w[i]).sum::<f64>() as f32,
        );
        (
            (screen, (0..4).map(|i| points[i].1 * w[i]).sum()),
            (0..4).map(|i| values[i] * w[i]).sum(),
        )
    };
    let mut triangles = Vec::with_capacity(2 * subdivisions * subdivisions);
    let n = subdivisions as f64;
    for y in 0..subdivisions {
        for x in 0..subdivisions {
            let p = [
                at(x as f64 / n, y as f64 / n),
                at((x + 1) as f64 / n, y as f64 / n),
                at((x + 1) as f64 / n, (y + 1) as f64 / n),
                at(x as f64 / n, (y + 1) as f64 / n),
            ];
            for ids in [[0, 1, 2], [0, 2, 3]] {
                triangles.push((ids.map(|i| p[i].0), ids.map(|i| p[i].1)));
            }
        }
    }
    triangles
}

#[cfg(test)]
mod wall_field_tests {
    use super::*;
    use crate::plan_trace::fem::{element::Material, shell::SurfaceForces};
    #[test]
    fn section_surface_range_uses_interpolated_values_and_excludes_clipped_surfaces() {
        let mut sections = sections::State::default();
        sections.planes[2].enabled = true;
        sections.planes[2].offset_mm = 5.;
        let triangle = [[0., 0., 0.], [10., 0., 0.], [10., 10., 0.]];
        let quad = [triangle[0], triangle[1], triangle[2], [0., 10., 0.]];
        let range = |points: &[[f64; 3]], values: &[f64], sections: &sections::State| {
            let mut extrema = None::<[f64; 2]>;
            include_section_surface_values(sections, points, values, |value| {
                let [min, max] = extrema.unwrap_or([value; 2]);
                extrema = Some([min.min(value), max.max(value)]);
            });
            extrema
        };
        assert_eq!(
            range(&triangle, &[100., 0., 0.], &sections),
            Some([0., 50.])
        );
        assert_eq!(
            range(&quad, &[100., 0., 0., 50.], &sections),
            Some([0., 50.])
        );

        sections.planes.push(sections::Plane {
            enabled: true,
            offset_mm: 4.,
            positive: false,
            kind: Some(sections::Kind::Yoz),
            ..Default::default()
        });
        assert!(sections.intersects(quad));
        assert_eq!(range(&quad, &[100.; 4], &sections), None);
        sections.planes[3].offset_mm = 7.5;
        assert!(!quad.into_iter().any(|point| sections.contains(point)));
        assert_eq!(range(&quad, &[100.; 4], &sections), Some([100.; 2]));
    }

    #[test]
    fn adaptive_q4_keeps_linear_fields_small_and_retains_curved_detail() {
        let points = [
            (Pos2::new(0., 0.), 0.),
            (Pos2::new(1., 0.), 0.),
            (Pos2::new(1., 1.), 0.),
            (Pos2::new(0., 1.), 0.),
        ];
        assert_eq!(
            surface_world_triangles(&points, &[0., 1., 3., 2.], [0., 3.]).len(),
            2
        );
        assert_eq!(
            surface_world_triangles(&points, &[0., 0., 1., 0.], [0., 1.]).len(),
            32
        );
        let values = [0., 1., 3.0003, 2.];
        let triangles = surface_world_triangles(&points, &values, [0., 3.]);
        assert_eq!(triangles.len(), 2);
        for iy in 0..=20 {
            for ix in 0..=20 {
                let u = ix as f64 / 20.;
                let v = iy as f64 / 20.;
                let exact = (1. - u) * (1. - v) * values[0]
                    + u * (1. - v) * values[1]
                    + u * v * values[2]
                    + (1. - u) * v * values[3];
                let p = Pos2::new(u as f32, v as f32);
                let interpolated = triangles
                    .iter()
                    .find_map(|(t, q)| {
                        pick_triangle(p, *t).map(|(w, _)| (0..3).map(|i| w[i] * q[i]).sum::<f64>())
                    })
                    .unwrap();
                assert!((exact - interpolated).abs() < 3e-4);
            }
        }
    }
    #[test]
    fn wall_recovery_converts_units_once_and_does_not_cross_wall_corner() {
        let face = |nodes: Vec<usize>, value: f64, axes: [[f64; 3]; 3]| SurfaceForces {
            nodes,
            support: Some(42),
            axes,
            material: Material::default(),
            area: 1.,
            membrane: [value, 0., 0.],
            moments: [0.; 3],
            shears: [0.; 2],
        };
        let axes = [[1., 0., 0.], [0., 0., 1.], [0., -1., 0.]];
        let forces = vec![
            face(vec![0, 1, 2], 2., axes),
            face(vec![1, 3, 2], 4., axes),
            face(
                vec![1, 4, 3],
                100.,
                [[0., 1., 0.], [0., 0., 1.], [1., 0., 0.]],
            ),
        ];
        let values = recover_wall_corners(&forces, 12, 0.1);
        assert!((values[0][0] - 0.2).abs() < 1e-12);
        assert!((values[0][1] - 0.3).abs() < 1e-12);
        assert!((values[1][0] - 0.3).abs() < 1e-12);
        assert!(values[2].iter().all(|v| (*v - 10.).abs() < 1e-12));
    }
    #[test]
    fn quad_isofield_preserves_bilinear_center_not_display_diagonal() {
        let points = [
            (Pos2::new(0., 0.), 1.),
            (Pos2::new(100., 0.), 2.),
            (Pos2::new(100., 100.), 3.),
            (Pos2::new(0., 100.), 2.),
        ];
        let triangles = surface_triangles(&points, &[0., 1., 0., 1.]);
        let centers = triangles
            .iter()
            .flat_map(|(p, v)| {
                (0..3).filter_map(move |i| {
                    (p[i].0.distance(Pos2::new(50., 50.)) < 1e-5).then_some((p[i].1, v[i]))
                })
            })
            .collect::<Vec<_>>();
        assert!(!centers.is_empty());
        assert!(centers
            .iter()
            .all(|&(depth, value)| (depth - 2.).abs() < 1e-12 && (value - 0.5).abs() < 1e-12));
    }
}
