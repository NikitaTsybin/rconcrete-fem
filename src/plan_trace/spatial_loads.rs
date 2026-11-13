//! Visual load volumes in model millimetres; never changes FEM loads.
use super::*;
pub(super) fn height(kind: Kind) -> f64 {
    match kind {
        Kind::LoadArea | Kind::LoadHole => 250.,
        Kind::LoadLine => 350.,
        Kind::LoadPoint => 450.,
        _ => 0.,
    }
}
impl App {
    pub(super) fn spatial_line_load_path(&self, source: &Contour) -> Vec<[f64; 3]> {
        let mut path = if source.load.is_none() && source.placement.beam_z_mm.is_some() {
            self.source_beam_path(source)
        } else {
            let z = self.source_heights(source)[0];
            source.points.iter().map(|&[x, y]| [x, y, z]).collect()
        };
        if source.closed() && !path.is_empty() {
            path.push(path[0]);
        }
        path
    }

    pub(super) fn spatial_load_sources(&self) -> std::sync::Arc<Vec<Contour>> {
        if !self.show_loads && self.tree_load_selection.is_empty() {
            return std::sync::Arc::new(vec![]);
        }
        let mut key = self.view_key().to_vec();
        key.extend([
            self.spatial.source_revision as u64,
            self.show_loads as u64,
            self.drawing_visibility.loads[0] as u64,
            self.drawing_visibility.loads[1] as u64,
            self.drawing_visibility.loads[2] as u64,
        ]);
        if let Some((old, loads)) = &*self.spatial.load_sources.borrow() {
            if *old == key {
                return loads.clone();
            }
        }
        let mut out = self
            .contours
            .iter()
            .filter(|c| {
                matches!(c.kind, Kind::LoadArea | Kind::LoadLine | Kind::LoadPoint)
                    && self.contour_visible(c)
            })
            .cloned()
            .collect::<Vec<_>>();
        let weight = self
            .load_cases
            .iter()
            .filter(|c| c.kind == crate::plan_trace::loads::LoadType::SelfWeight)
            .filter_map(|c| self.displayed_load_factor(c.id))
            .sum::<f64>();
        if self.show_loads && weight.is_finite() && weight != 0. {
            for c in self.contours.iter().filter(|c| !self.object_hidden(c)) {
                let kind = match c.kind {
                    Kind::Slab
                        if self.drawing_visibility.loads[0]
                            && self
                                .materials
                                .get(&c.id)
                                .copied()
                                .unwrap_or_default()
                                .self_weight_kn_m2(self.gravity)
                                != 0. =>
                    {
                        Kind::LoadArea
                    }
                    Kind::Beam
                        if self.drawing_visibility.loads[1]
                            && self.support_height.beams.get(&c.id).is_some_and(|b| {
                                b.material.weight.in_units(self.load_units, self.gravity) != 0.
                            }) =>
                    {
                        Kind::LoadLine
                    }
                    Kind::Support
                        if self.drawing_visibility.loads[1]
                            && (self.building.enabled || self.support_height.enabled)
                            && self
                                .support_height
                                .materials
                                .get(&c.id)
                                .copied()
                                .unwrap_or_else(
                                    crate::plan_trace::fem::element::Material::wall_default,
                                )
                                .self_weight_kn_m2(self.gravity)
                                != 0. =>
                    {
                        Kind::LoadLine
                    }
                    _ => continue,
                };
                let mut source = c.clone();
                source.kind = kind;
                source.load = None;
                out.push(source);
            }
        }
        let out = std::sync::Arc::new(out);
        *self.spatial.load_sources.borrow_mut() = Some((key, out.clone()));
        out
    }
}
pub(super) fn boundary(triangles: &[[Point; 3]]) -> Vec<[Point; 2]> {
    let mut edges = BTreeMap::<[[u64; 2]; 2], (usize, [Point; 2])>::new();
    for t in triangles {
        for i in 0..3 {
            let edge = [t[i], t[(i + 1) % 3]];
            let mut key = edge.map(|p| p.map(|v| if v == 0. { 0 } else { v.to_bits() }));
            key.sort();
            let entry = edges.entry(key).or_insert((0, edge));
            entry.0 += 1;
        }
    }
    edges
        .into_values()
        .filter(|e| e.0 == 1)
        .map(|e| e.1)
        .collect()
}
pub(super) fn offset(app: &App, c: &Contour) -> [f64; 3] {
    let direction = c
        .load
        .as_ref()
        .map_or(crate::plan_trace::loads::Direction::Z, |load| {
            load.direction
        });
    let sign = if app.displayed_load_value(c).is_some_and(|v| v < 0.) {
        -1.
    } else {
        1.
    };
    direction.force(-sign * height(c.kind))
}

pub(super) fn arrow(
    anchor: [f64; 3],
    axis: usize,
    tail: f64,
    moment: bool,
    mut triangle: impl FnMut([[f64; 3]; 3]),
) {
    let at = |radius: f64, angle: f64, axial: f64| {
        let mut point = anchor;
        point[axis] += axial;
        point[(axis + 1) % 3] += radius * angle.cos();
        point[(axis + 2) % 3] += radius * angle.sin();
        point
    };
    if moment {
        let step = -tail.signum() * 0.20;
        for i in 0..24 {
            let a = f64::from(i) * step;
            let b = a + step;
            let quad = [
                at(162., a, 0.),
                at(198., a, 0.),
                at(198., b, 0.),
                at(162., b, 0.),
            ];
            triangle([quad[0], quad[1], quad[2]]);
            triangle([quad[0], quad[2], quad[3]]);
        }
        let end = 24. * step;
        triangle([
            at(128., end, 0.),
            at(232., end, 0.),
            at(180., end + 3. * step, 0.),
        ]);
        return;
    }
    let neck = 0.23 * tail;
    for i in 0..12 {
        let a = f64::from(i) * std::f64::consts::TAU / 12.;
        let b = f64::from(i + 1) * std::f64::consts::TAU / 12.;
        let quad = [
            at(9., a, neck),
            at(9., b, neck),
            at(9., b, tail),
            at(9., a, tail),
        ];
        triangle([quad[0], quad[1], quad[2]]);
        triangle([quad[0], quad[2], quad[3]]);
        triangle([anchor, at(48., a, neck), at(48., b, neck)]);
        triangle([at(0., 0., neck), at(48., b, neck), at(48., a, neck)]);
    }
}

pub(super) fn geometry(
    app: &App,
    center: [f64; 3],
    extent: f64,
    out: &mut Vec<spatial_depth::Vertex>,
) {
    let vertex = |p, ink| spatial_world::vertex(p, ink, center, extent);
    for c in app.spatial_load_sources().iter() {
        let z = app.source_heights(&c)[0];
        let delta = offset(app, c);
        let raised = |p: [f64; 3]| std::array::from_fn(|i| p[i] + delta[i]);
        let axis = c.load.as_ref().map_or(2, |load| load.direction.index());
        let moment = c.load.as_ref().is_some_and(|load| load.moment);
        let ink = if c.kind == Kind::LoadPoint && app.is_selected(c.id) {
            Color32::YELLOW
        } else {
            app.object_ink(c.id, c.kind)
        };
        let cap = ink.gamma_multiply(app.drawing_visibility.load_opacity);
        let side = cap;
        let mut quad = |q: [[f64; 3]; 4], color| {
            for ids in [[0, 1, 2], [0, 2, 3]] {
                out.extend(ids.map(|i| vertex(q[i], color)));
            }
        };
        match c.kind {
            Kind::LoadArea => {
                let triangles = app.cached_load_fill(&c);
                // Render only clipped boundary walls, never interior tessellation edges.
                for &[a, b] in app.cached_load_boundary(c.id, &triangles).iter() {
                    quad(
                        [
                            [a[0], a[1], z],
                            [b[0], b[1], z],
                            raised([b[0], b[1], z]),
                            raised([a[0], a[1], z]),
                        ],
                        side,
                    );
                }
                for t in triangles.iter() {
                    out.extend(t.map(|p| vertex(raised([p[0], p[1], z]), cap)));
                }
            }
            Kind::LoadLine => {
                for e in app.spatial_line_load_path(c).windows(2) {
                    quad([e[0], e[1], raised(e[1]), raised(e[0])], side);
                }
            }
            _ => {}
        }
        if c.kind == Kind::LoadPoint || axis != 2 || delta[2] < 0. {
            let point = if c.kind == Kind::LoadPoint {
                c.points.first().copied()
            } else {
                c.load_label_positions().into_iter().next()
            };
            if let Some([x, y]) = point {
                arrow(
                    [x, y, z],
                    axis,
                    if moment && axis == 2 {
                        -delta[axis]
                    } else {
                        delta[axis]
                    },
                    moment,
                    |t| {
                        out.extend(t.map(|p| vertex(p, cap)));
                    },
                );
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn glyphs_follow_each_global_axis_and_moment_sign() {
        for axis in 0..3 {
            let mut force = Vec::new();
            arrow([0.; 3], axis, -450., false, |t| force.extend(t));
            assert!(force.iter().all(|p| (-450.0..=0.0).contains(&p[axis])));
            assert!(force.iter().any(|p| p[axis] == -450.));
            for tail in [-450_f64, 450.] {
                let mut moment = Vec::new();
                arrow([0.; 3], axis, tail, true, |t| moment.push(t));
                assert!(moment.iter().flatten().all(|p| p[axis] == 0.));
                assert_eq!(moment[0][2][(axis + 2) % 3].signum(), -tail.signum());
            }
        }
    }
}
