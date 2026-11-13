//! Camera-independent colored surfaces. Only the small matrix changes on orbit.
use super::*;
use crate::plan_trace::fem::shell;
use spatial_depth::Vertex;
use std::sync::Arc;

pub(super) const STRUCTURE_EDGE_INK: Color32 = Color32::from_rgb(25, 27, 31);

pub(super) fn camera(
    rect: Rect,
    projected: &[(Pos2, f64)],
    extent: f64,
    scale: f64,
    yaw: f64,
    pitch: f64,
    pan: egui::Vec2,
) -> [[f32; 4]; 4] {
    let near = projected
        .iter()
        .map(|p| p.1)
        .fold(f64::NEG_INFINITY, f64::max);
    let far = projected.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    let span = (near - far).max(1.);
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let sx = extent * scale * 2. / rect.width() as f64;
    let sy_screen = extent * scale * 2. / rect.height() as f64;
    let dz = -extent * 0.98 / span;
    [
        [cy * sx, cp * sy * sy_screen, sp * sy * dz, 0.],
        [-sy * sx, cp * cy * sy_screen, sp * cy * dz, 0.],
        [0., -sp * sy_screen, cp * dz, 0.],
        [
            2. * pan.x as f64 / rect.width() as f64,
            -2. * pan.y as f64 / rect.height() as f64,
            near * 0.98 / span + 0.01,
            1.,
        ],
    ]
    .map(|column| column.map(|v| v as f32))
}
pub(super) fn vertex(p: [f64; 3], ink: Color32, center: [f64; 3], extent: f64) -> Vertex {
    Vertex {
        position: std::array::from_fn(|a| ((p[a] - center[a]) / extent) as f32),
        color: ink.to_array().map(|c| c as f32 / 255.),
        offset: [0.; 3],
    }
}
pub(super) struct Geometry {
    pub faces: Arc<Vec<Vertex>>,
    pub edges: Arc<Vec<[Vertex; 4]>>,
    pub direction_labels: Vec<([f64; 3], f64)>,
}

pub(super) fn rigid_link_face(app: &App, contour: &Contour) -> Option<[[f64; 3]; 4]> {
    let [a, b] = contour.points.as_slice() else {
        return None;
    };
    let heights = app.source_heights(contour);
    let [bottom, top] = heights.as_slice() else {
        return None;
    };
    (a != b && (top - bottom).abs() > 1e-6).then_some([
        [a[0], a[1], *bottom],
        [b[0], b[1], *bottom],
        [b[0], b[1], *top],
        [a[0], a[1], *top],
    ])
}

pub(super) fn wall_half_offset(
    app: &App,
    surface: &shell::Surface,
    xyz: &[[f64; 3]],
) -> Option<[f64; 3]> {
    if !app.drawing_visibility.support_extents {
        return None;
    }
    let material = app.support_height.materials.get(&surface.support?)?;
    let nodes: &[usize; 3] = surface.nodes.get(..3)?.try_into().ok()?;
    let [a, b, c] = nodes.map(|n| xyz[n]);
    let u = std::array::from_fn::<_, 3, _>(|i| b[i] - a[i]);
    let v = std::array::from_fn::<_, 3, _>(|i| c[i] - a[i]);
    let n = [
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    ];
    let length = n.iter().map(|x| x * x).sum::<f64>().sqrt();
    (length > 1e-12).then(|| n.map(|x| x / length * material.thickness_mm * 0.5))
}

pub(super) fn edge_ribbon(
    p: [f64; 3],
    q: [f64; 3],
    center: [f64; 3],
    extent: f64,
) -> Option<[Vertex; 4]> {
    ribbon([p, q], Color32::from_rgb(36, 58, 74), 1., center, extent)
}

fn ribbon(
    [p, q]: [[f64; 3]; 2],
    ink: Color32,
    width: f32,
    center: [f64; 3],
    extent: f64,
) -> Option<[Vertex; 4]> {
    let d = std::array::from_fn::<_, 3, _>(|k| q[k] - p[k]);
    let norm = d.iter().map(|v| v * v).sum::<f64>().sqrt();
    if norm <= 1e-12 {
        return None;
    }
    let tangent = d.map(|v| (v / norm) as f32 * width);
    Some([(p, 1.), (q, 1.), (q, -1.), (p, -1.)].map(|(point, sign)| {
        let mut v = vertex(point, ink, center, extent);
        v.offset = tangent.map(|x| x * sign);
        v
    }))
}

pub(super) fn source_outline_kind(kind: Kind) -> bool {
    matches!(kind, Kind::Slab | Kind::Hole | Kind::Support)
}

fn append_source_ribbons(
    app: &App,
    scene: &shell::Scene,
    draw: &object_visibility::DrawList,
    center: [f64; 3],
    extent: f64,
    edges: &mut Vec<[Vertex; 4]>,
) {
    let mut push = |line, width, ink| edges.extend(ribbon(line, ink, width, center, extent));
    let style = app.drawing_visibility;
    for c in app.contours.iter().filter(|c| app.contour_visible(c)) {
        if source_outline_kind(c.kind) && style.border(c.kind) {
            let heights = app.source_heights(c);
            let ink = app.object_ink(c.id, c.kind);
            for &z in &heights {
                let count = c.points.len().saturating_sub(usize::from(!c.closed()));
                for ends in ring_edges(&c.points).take(count) {
                    push(ends.map(|[x, y]| [x, y, z]), 1.5, ink);
                }
            }
            if let (Kind::Support, &[lo, hi]) = (c.kind, heights.as_slice()) {
                if (hi - lo).abs() > 1e-6 {
                    for &[x, y] in &c.points {
                        push([[x, y, lo], [x, y, hi]], 1.5, ink);
                    }
                }
            }
        } else if c.kind == Kind::Column && !style.support_extents {
            let Some(&[x, y]) = c.points.first() else {
                continue;
            };
            let heights = app.source_heights(c);
            let axis = [[x, y, heights[0]], [x, y, *heights.last().unwrap()]];
            if style.structure_edges {
                push(axis, 5., STRUCTURE_EDGE_INK);
            }
            push(axis, 3., app.object_ink(c.id, Kind::Column));
        }
    }
    for beam in draw.beams.iter().map(|&i| &scene.beams[i]) {
        let arm = beam.rigid_arm(&scene.xyz).unwrap_or([0.; 3]);
        let axis = beam
            .nodes
            .map(|n| std::array::from_fn(|k| scene.xyz[n][k] + arm[k]));
        if style.structure_edges {
            push(axis, 6., STRUCTURE_EDGE_INK);
        }
        push(axis, 4., app.object_ink(beam.id, Kind::Beam));
    }
    if style.structure_edges {
        for edge in structure_edges(app, scene, &draw.surfaces) {
            push(edge, 1.6, STRUCTURE_EDGE_INK);
        }
    }
}

fn wall_fill_ink(app: &App, surface: &shell::Surface) -> Color32 {
    surface
        .support
        .and_then(|id| app.property_ink(id, Kind::Support))
        .unwrap_or(Color32::from_rgb(80, 135, 168))
}

pub(super) fn surface_fill_ink(
    app: &App,
    surface: &shell::Surface,
    xyz: &[[f64; 3]],
) -> Option<Color32> {
    if surface.support.is_some() {
        return Some(wall_fill_ink(app, surface));
    }
    if !app.drawing_visibility.fill {
        return None;
    }
    let owner = if app.drawing_visibility.stiffness_colors || app.drawing_visibility.material_colors
    {
        if let Some(&i) = surface.display.first() {
            if let Some(data) = app.fe_mesh.as_ref().and_then(|mesh| mesh.building.as_ref()) {
                data.element_slabs.get(i).copied()
            } else {
                let center = [0, 1].map(|a| {
                    surface.nodes.iter().map(|&n| xyz[n][a]).sum::<f64>()
                        / surface.nodes.len() as f64
                });
                app.slab_property_owner(center)
            }
        } else {
            surface
                .nodes
                .first()
                .and_then(|n| app.spatial.source_surface_owners.get(n))
                .and_then(|owners| {
                    owners
                        .iter()
                        .copied()
                        .find(|&id| app.view_contour(id).is_some_and(|c| !app.object_hidden(c)))
                })
        }
    } else {
        None
    };
    let [r, g, b] = app.slab_color;
    Some(
        owner
            .and_then(|id| app.property_ink(id, Kind::Slab))
            .unwrap_or(Color32::from_rgb(r, g, b))
            .gamma_multiply(app.slab_opacity),
    )
}

pub(super) fn geometry(
    app: &App,
    scene: &shell::Scene,
    center: [f64; 3],
    extent: f64,
    range: Option<[f64; 2]>,
    key: Vec<u64>,
) -> Arc<Geometry> {
    if let Some((old, vertices)) = &*app.spatial.world_cache.borrow() {
        if *old == key {
            return vertices.clone();
        }
    }
    let mut out = vec![];
    let lengths = app.length_world_colors();
    let draw = app.spatial_draw_list(scene);
    let distributed_loads = range
        .filter(|_| app.show_results && app.result_field == 7)
        .and_then(|range| {
            let case = app.selected_result()?;
            let mesh = app.fe_mesh.as_ref()?;
            Some((
                case.loads_kn_m2.as_slice(),
                mesh.elements.len(),
                range,
                app.result_display_factor(),
            ))
        });
    for &index in &draw.surfaces {
        let surface = &scene.surfaces[index];
        let schema_ink = app.schema_surface_ink(scene, surface);
        if let (None, None, Some((pressures, element_count, range, factor))) =
            (schema_ink, surface.support, distributed_loads)
        {
            if let Some(&display) = surface.display.first().filter(|&&i| i < element_count) {
                let value = pressures.get(display).copied().unwrap_or(f64::NAN) * factor;
                let ink = if value.is_finite() {
                    app.result_scales[7].color(value, range)
                } else {
                    Some(Color32::from_rgb(230, 45, 45))
                };
                if let Some(ink) = ink {
                    let nodes = &surface.nodes;
                    for i in 1..nodes.len().saturating_sub(1) {
                        out.extend(
                            [nodes[0], nodes[i], nodes[i + 1]]
                                .map(|node| vertex(scene.xyz[node], ink, center, extent)),
                        );
                    }
                }
                continue;
            }
        }
        let surface_start = out.len();
        let xyz = surface
            .nodes
            .iter()
            .map(|&n| scene.xyz[n])
            .collect::<Vec<_>>();
        let wall_ink = || {
            surface
                .support
                .and_then(|id| {
                    let [x, y, z]: [f64; 3] = std::array::from_fn(|i| {
                        xyz.iter().map(|p| p[i]).sum::<f64>() / xyz.len() as f64
                    });
                    lengths
                        .get(&id)
                        .and_then(|rows| {
                            rows.iter().find(|(lo, hi, _, edge)| {
                                z >= *lo
                                    && z <= *hi
                                    && edge.is_none_or(|[a, b]| {
                                        let d = [b[0] - a[0], b[1] - a[1]];
                                        let v = [x - a[0], y - a[1]];
                                        let length = d[0].hypot(d[1]);
                                        let along = (v[0] * d[0] + v[1] * d[1]) / length;
                                        (v[0] * d[1] - v[1] * d[0]).abs() / length < 1e-4
                                            && along >= -1e-4
                                            && along <= length + 1e-4
                                    })
                            })
                        })
                        .map(|&(_, _, ink, _)| ink)
                })
                .unwrap_or_else(|| wall_fill_ink(app, surface))
        };
        let values = range.and_then(|_| app.spatial_surface_values(index, surface));
        if let Some(ink) = schema_ink {
            for i in 1..xyz.len() - 1 {
                out.extend([xyz[0], xyz[i], xyz[i + 1]].map(|p| vertex(p, ink, center, extent)));
            }
        } else if let (true, Some(values), Some(range)) = (app.show_results, values, range) {
            if let Some(&value) = values
                .first()
                .filter(|v| v.is_finite())
                .filter(|&&v| values.iter().all(|x| *x == v))
            {
                if let Some(ink) = app.result_scales[app.result_field].color(value, range) {
                    for i in 1..xyz.len() - 1 {
                        out.extend(
                            [xyz[0], xyz[i], xyz[i + 1]].map(|p| vertex(p, ink, center, extent)),
                        );
                    }
                }
            } else {
                let uv = if xyz.len() == 4 {
                    vec![[0., 0.], [1., 0.], [1., 1.], [0., 1.]]
                } else {
                    vec![[0., 0.], [1., 0.], [0., 1.]]
                };
                let points = uv
                    .iter()
                    .map(|p| (Pos2::new(p[0], p[1]), 0.))
                    .collect::<Vec<_>>();
                let at = |p: Pos2| -> [f64; 3] {
                    let (u, v) = (p.x as f64, p.y as f64);
                    let w = if xyz.len() == 4 {
                        vec![(1. - u) * (1. - v), u * (1. - v), u * v, (1. - u) * v]
                    } else {
                        vec![1. - u - v, u, v]
                    };
                    std::array::from_fn(|a| w.iter().zip(&xyz).map(|(w, p)| w * p[a]).sum())
                };
                for (triangle, values) in
                    spatial_results::surface_world_triangles(&points, &values, range)
                {
                    let world = triangle.map(|p| at(p.0));
                    if values.iter().any(|v| !v.is_finite()) {
                        out.extend(
                            world
                                .map(|p| vertex(p, Color32::from_rgb(230, 45, 45), center, extent)),
                        );
                        continue;
                    }
                    for colored in app.result_scales[app.result_field].triangles(
                        triangle.map(|p| [p.0.x as f64, p.0.y as f64]),
                        values,
                        range,
                    ) {
                        let vertices = colored.map(|(p, ink)| {
                            spatial_results::pick_triangle(
                                Pos2::new(p[0] as f32, p[1] as f32),
                                triangle,
                            )
                            .map(|(w, _)| {
                                let p = std::array::from_fn(|a| {
                                    (0..3).map(|i| w[i] * world[i][a]).sum()
                                });
                                vertex(p, ink, center, extent)
                            })
                        });
                        if vertices.iter().all(Option::is_some) {
                            out.extend(vertices.into_iter().flatten());
                        }
                    }
                }
            }
        } else {
            let ink = if surface.support.is_some() {
                wall_ink()
            } else if let Some(ink) = surface_fill_ink(app, surface, &scene.xyz) {
                ink
            } else {
                continue;
            };
            for i in 1..xyz.len() - 1 {
                out.extend([xyz[0], xyz[i], xyz[i + 1]].map(|p| vertex(p, ink, center, extent)));
            }
        }
        // Give the wall its physical thickness only when support extents are shown.
        // Keep the analysis midsurface and its IDs unchanged.
        if let Some(d) = wall_half_offset(app, surface, &scene.xyz) {
            let d = d.map(|x| x / extent);
            let front = out[surface_start..].to_vec();
            for vertex in &mut out[surface_start..] {
                for i in 0..3 {
                    vertex.position[i] += d[i] as f32;
                }
            }
            for mut vertex in front {
                for i in 0..3 {
                    vertex.position[i] -= d[i] as f32;
                }
                out.push(vertex);
            }
            let ink = wall_ink();
            for i in 0..xyz.len() {
                let a = xyz[i];
                let b = xyz[(i + 1) % xyz.len()];
                let face = [(a, 1.), (b, 1.), (b, -1.), (a, 1.), (b, -1.), (a, -1.)];
                for (p, sign) in face {
                    let mut vertex = vertex(p, ink, center, extent);
                    for i in 0..3 {
                        vertex.position[i] += (d[i] * sign) as f32;
                    }
                    out.push(vertex);
                }
            }
        }
    }
    let mut edges = Vec::new();
    if app.fe_mesh.is_some() && app.show_mesh {
        let mut seen =
            std::collections::HashSet::with_capacity(draw.surfaces.len().saturating_mul(2));
        for &index in &draw.surfaces {
            let nodes = &scene.surfaces[index].nodes;
            for (&a, &b) in nodes.iter().zip(nodes.iter().cycle().skip(1)) {
                if seen.insert((a.min(b), a.max(b))) {
                    if let Some(edge) = edge_ribbon(scene.xyz[a], scene.xyz[b], center, extent) {
                        edges.push(edge);
                    }
                }
            }
        }
    }
    if app.fe_mesh.is_none() {
        append_source_ribbons(app, scene, &draw, center, extent, &mut edges);
    }
    let mut direction_labels = Vec::new();
    if app.reinforcement_directions {
        if let Some(result) = &app.reinforcement.result {
            let shell = app
                .analysis
                .as_ref()
                .and_then(|analysis| analysis.cases.first())
                .and_then(|case| case.shell.as_ref());
            for &index in &draw.surfaces {
                let surface = &scene.surfaces[index];
                let (display, axes) = if surface.support.is_some() {
                    let (Some(&display), Some(force)) = (
                        result.wall_displays.get(&index),
                        shell.and_then(|s| s.surface_forces.get(index)),
                    ) else {
                        continue;
                    };
                    (display, [force.axes[0], force.axes[1]])
                } else {
                    let Some(&display) = surface.display.first() else {
                        continue;
                    };
                    (display, [[1., 0., 0.], [0., 1., 0.]])
                };
                let Some(angle) = result
                    .cells
                    .get(display)
                    .filter(|cell| cell.verified)
                    .and_then(|cell| cell.governing_angle)
                else {
                    continue;
                };
                let points = surface.nodes.iter().map(|&node| scene.xyz[node]);
                let at: [f64; 3] = std::array::from_fn(|axis| {
                    points.clone().map(|p| p[axis]).sum::<f64>() / surface.nodes.len() as f64
                });
                let radius = points
                    .map(|p| (p[0] - at[0]).hypot(p[1] - at[1]).hypot(p[2] - at[2]))
                    .fold(f64::INFINITY, f64::min)
                    * 0.6;
                let (sin, cos) = angle.sin_cos();
                let [a, b] = [-1., 1.].map(|sign| {
                    std::array::from_fn(|i| {
                        at[i] + sign * radius * (cos * axes[0][i] + sin * axes[1][i])
                    })
                });
                if let Some(mut edge) = edge_ribbon(a, b, center, extent) {
                    for vertex in &mut edge {
                        vertex.color = [1., 1., 0., 1.];
                    }
                    edges.push(edge);
                    direction_labels.push((at, angle));
                }
            }
        }
    }
    app.column_world(center, extent, &lengths, &mut out);
    spatial_loads::geometry(app, center, extent, &mut out);
    if app.drawing_visibility.rigid_bodies {
        let ink = app.contour_color(Kind::RigidLink);
        let fill = ink.gamma_multiply(0.28);
        let rgba = ink.to_array().map(|value| f32::from(value) / 255.);
        for contour in app
            .contours
            .iter()
            .filter(|c| c.kind == Kind::RigidLink && app.contour_visible(c))
        {
            let Some(face) = rigid_link_face(app, contour) else {
                continue;
            };
            for triangle in [[0, 1, 2], [0, 2, 3]] {
                out.extend(triangle.map(|i| vertex(face[i], fill, center, extent)));
            }
            for [a, b] in [[0, 2], [1, 3]] {
                if let Some(mut edge) = edge_ribbon(face[a], face[b], center, extent) {
                    for vertex in &mut edge {
                        vertex.color = rgba;
                    }
                    edges.push(edge);
                }
            }
        }
    }
    #[cfg(not(target_arch = "wasm32"))]
    if std::env::var_os("MINI_FEM_RENDER_DIAGNOSTICS").is_some() {
        eprintln!(
            "3D world: surfaces={} vertices={} bytes={}",
            scene.surfaces.len(),
            out.len(),
            out.len() * std::mem::size_of::<Vertex>()
        );
    }
    let out = Arc::new(Geometry {
        faces: Arc::new(out),
        edges: Arc::new(edges),
        direction_labels,
    });
    *app.spatial.world_cache.borrow_mut() = Some((key, out.clone()));
    out
}
pub(super) type StructureEdges = Arc<Vec<[[f64; 3]; 2]>>;

pub(super) fn paint_structure_edges(
    app: &App,
    scene: &shell::Scene,
    surfaces: &[usize],
    painter: &egui::Painter,
    rect: Rect,
    project: &impl Fn([f64; 3]) -> (Pos2, f64),
    depth_points: &[(Pos2, f64)],
) {
    let style = app.drawing_visibility;
    if !style.structure_edges {
        return;
    }
    let mut key = app.view_key().to_vec();
    key.extend([
        app.spatial.source_revision as u64,
        style.fill as u64,
        style.support_extents as u64,
        style.columns as u64,
    ]);
    key.extend(style.contours.map(|v| v as u64));
    key.extend(app.sections.active_planes().flat_map(|(i, p)| {
        let [x, y, z] = p.normal(i).map(f64::to_bits);
        [x, y, z, p.offset(i).to_bits(), p.positive as u64]
    }));
    let cached = app
        .spatial
        .structure_edge_cache
        .borrow()
        .as_ref()
        .filter(|(old, _)| *old == key)
        .map(|(_, edges)| edges.clone());
    let edges = cached.unwrap_or_else(|| {
        let edges = Arc::new(structure_edges(app, scene, surfaces));
        *app.spatial.structure_edge_cache.borrow_mut() = Some((key, edges.clone()));
        edges
    });
    let mut builder = spatial_depth::Builder::new(rect, depth_points);
    for &edge in edges.iter() {
        builder.line(edge.map(project), 1.6, STRUCTURE_EDGE_INK);
    }
    spatial_depth::paint_layer(
        painter,
        rect,
        Arc::new(builder.vertices),
        spatial_depth::Layer::StructureEdges,
    );
}

fn structure_edges(app: &App, scene: &shell::Scene, surfaces: &[usize]) -> Vec<[[f64; 3]; 2]> {
    let mut lines = Vec::new();
    let planes = &app.sections;
    for c in app.contours.iter().filter(|_| !app.fe_isolated()) {
        let parent_slab_visible = c.active_hole()
            && c.parent
                .and_then(|id| app.view_contour(id))
                .is_some_and(|slab| app.contour_visible(slab))
            && !app.object_hidden_without_sections(c);
        let visible = if c.kind == Kind::Support {
            !app.object_hidden_without_sections(c) && app.on_view_floor(c)
        } else {
            parent_slab_visible || app.contour_visible(c)
        };
        if !visible
            || !matches!(
                c.kind,
                Kind::Slab | Kind::Hole | Kind::Support | Kind::Column
            )
        {
            continue;
        }
        let heights = app.source_heights(c);
        let at = |[x, y]: [f64; 2], z: f64| [x, y, z];
        if matches!(c.kind, Kind::Slab | Kind::Hole) {
            lines
                .extend(ring_edges(&c.points).map(|[a, b]| [at(a, heights[0]), at(b, heights[0])]));
            continue;
        }
        let (lo, hi) = (heights[0], *heights.last().unwrap());
        if c.kind == Kind::Column {
            if app.drawing_visibility.support_extents {
                if let Some(section) = app.support_height.columns.get(&c.id) {
                    let ring = crate::plan_trace::columns::section_corners(c, section);
                    prism_edges(&ring, lo, hi, planes, &mut lines);
                }
            }
            continue;
        }
        let thickness = app
            .drawing_visibility
            .support_extents
            .then(|| app.support_height.materials.get(&c.id))
            .flatten()
            .map_or(0., |m| m.thickness_mm);
        let spans =
            ring_edges(&c.points).take(c.points.len().saturating_sub(usize::from(!c.closed())));
        for [a, b] in spans {
            let [dx, dy] = [b[0] - a[0], b[1] - a[1]];
            let length = dx.hypot(dy);
            if length <= 1e-9 {
                continue;
            }
            if thickness > 0. {
                let [nx, ny] = [
                    -dy / length * thickness * 0.5,
                    dx / length * thickness * 0.5,
                ];
                let ring = [
                    [a[0] + nx, a[1] + ny],
                    [b[0] + nx, b[1] + ny],
                    [b[0] - nx, b[1] - ny],
                    [a[0] - nx, a[1] - ny],
                ];
                prism_edges(&ring, lo, hi, planes, &mut lines);
            } else {
                let face = [at(a, lo), at(b, lo), at(b, hi), at(a, hi)];
                lines.extend(ring_edges(&face));
                lines.extend(section_edges(&face, planes));
            }
        }
    }
    if planes.active() {
        // Use the actual slab faces, so section lines respect holes and cutouts.
        for &index in surfaces {
            let face = &scene.surfaces[index];
            if face.support.is_none() {
                let xyz: Vec<_> = face.nodes.iter().map(|&n| scene.xyz[n]).collect();
                lines.extend(section_edges(&xyz, planes));
            }
        }
    }
    lines
}

fn ring_edges<T: Copy>(ring: &[T]) -> impl Iterator<Item = [T; 2]> + '_ {
    ring.iter()
        .copied()
        .zip(ring.iter().copied().cycle().skip(1))
        .map(|(a, b)| [a, b])
}

fn prism_edges(
    ring: &[[f64; 2]; 4],
    lo: f64,
    hi: f64,
    planes: &sections::State,
    lines: &mut Vec<[[f64; 3]; 2]>,
) {
    let bottom = ring.map(|[x, y]| [x, y, lo]);
    let top = ring.map(|[x, y]| [x, y, hi]);
    lines.extend(ring_edges(&bottom));
    lines.extend(ring_edges(&top));
    for ([a, b], [c, d]) in ring_edges(&bottom).zip(ring_edges(&top)) {
        lines.push([a, c]);
        lines.extend(section_edges(&[a, b, d, c], planes));
    }
    lines.extend(section_edges(&bottom, planes));
    lines.extend(section_edges(&top, planes));
}

fn section_edges(face: &[[f64; 3]], planes: &sections::State) -> Vec<[[f64; 3]; 2]> {
    const EPS: f64 = 1e-5;
    let touches_cut = planes.active_planes().any(|(i, p)| {
        let mut front = false;
        let mut behind = false;
        let mut on = 0;
        for &vertex in face {
            let d = p.distance(i, vertex);
            front |= d > EPS;
            behind |= d < -EPS;
            on += usize::from(d.abs() <= EPS);
        }
        front && (behind || on >= 2)
    });
    if !touches_cut {
        return Vec::new();
    }
    let mut clipped = face.to_vec();
    for (i, plane) in planes.active_planes() {
        let mut next = Vec::new();
        for [a, b] in ring_edges(&clipped) {
            let da = plane.distance(i, a);
            let db = plane.distance(i, b);
            if da >= 0. {
                next.push(a);
            }
            if (da < 0.) != (db < 0.) {
                let t = da / (da - db);
                next.push(std::array::from_fn(|j| a[j] + (b[j] - a[j]) * t));
            }
        }
        if next.len() < 3 {
            return Vec::new();
        }
        clipped = next;
    }
    ring_edges(&clipped)
        .filter(|edge| {
            planes.active_planes().any(|(i, p)| {
                edge.iter().all(|&v| p.distance(i, v).abs() <= EPS)
                    && face.iter().any(|&v| p.distance(i, v) > EPS)
            })
        })
        .collect()
}

/// Filled deformed faces retain source coordinates and a separate displacement.
/// The GPU applies the amplitude without rebuilding or re-uploading the mesh.
pub(super) fn deformation_faces(
    scene: &shell::Scene,
    surfaces: &[usize],
    displacement: impl Fn(usize) -> Option<[f64; 3]>,
    center: [f64; 3],
    extent: f64,
    appearance: impl Fn(usize, &shell::Surface) -> Option<(Color32, Option<[f64; 3]>)>,
    field: Option<(&crate::plan_trace::result_scale::Scale, [f64; 2])>,
    value: impl Fn(usize, usize) -> Option<f64>,
) -> Vec<Vertex> {
    let mut out = Vec::new();
    for &i in surfaces {
        let Some(surface) = scene.surfaces.get(i) else {
            continue;
        };
        let Some((ink, thickness)) = appearance(i, surface) else {
            continue;
        };
        if surface.nodes.len() < 3 || ink.a() == 0 {
            continue;
        }
        let vertices = surface
            .nodes
            .iter()
            .map(|&n| {
                let delta = displacement(n)?;
                let point = *scene.xyz.get(n)?;
                if !point.into_iter().chain(delta).all(f64::is_finite) {
                    return None;
                }
                let mut v = vertex(point, ink, center, extent);
                v.offset = delta.map(|x| (x / extent) as f32);
                Some(v)
            })
            .collect::<Option<Vec<_>>>();
        let Some(vertices) = vertices else {
            continue;
        };
        let shift = |mut vertex: Vertex, sign: f64| {
            if let Some(offset) = thickness {
                for (value, delta) in vertex.position.iter_mut().zip(offset) {
                    *value += (delta * sign / extent) as f32;
                }
            }
            vertex
        };
        let mut emit = |face: [Vertex; 3], corners: [usize; 3]| {
            if let Some((scale, range)) = field {
                let values = corners.map(|k| value(i, surface.nodes[k]));
                if let [Some(a), Some(b), Some(c)] = values {
                    for triangle in
                        scale.triangles([[0., 0.], [1., 0.], [0., 1.]], [a, b, c], range)
                    {
                        out.extend(triangle.map(|([x, y], color)| {
                            let weights = [(1. - x - y) as f32, x as f32, y as f32];
                            let interpolate = |get: fn(Vertex) -> [f32; 3]| {
                                std::array::from_fn(|axis| {
                                    face.into_iter()
                                        .zip(weights)
                                        .map(|(v, w)| w * get(v)[axis])
                                        .sum()
                                })
                            };
                            let mut v = vertex(
                                [0.; 3],
                                color.gamma_multiply(f32::from(ink.a()) / 255.),
                                center,
                                extent,
                            );
                            v.position = interpolate(|v| v.position);
                            v.offset = interpolate(|v| v.offset);
                            v
                        }));
                    }
                    return;
                }
            }
            out.extend(face);
        };
        for j in 1..vertices.len() - 1 {
            let corners = [0, j, j + 1];
            let face = corners.map(|k| vertices[k]);
            emit(face.map(|v| shift(v, 1.)), corners);
            if thickness.is_some() {
                emit(face.map(|v| shift(v, -1.)), corners);
            }
        }
        if thickness.is_some() {
            for (a, &va) in vertices.iter().enumerate() {
                let b = (a + 1) % vertices.len();
                let vb = vertices[b];
                let [a0, b0, a1, b1] =
                    [shift(va, -1.), shift(vb, -1.), shift(va, 1.), shift(vb, 1.)];
                emit([a0, b0, b1], [a, b, b]);
                emit([a0, b1, a1], [a, b, a]);
            }
        }
    }
    out
}

pub(super) fn deformation_column(
    app: &App,
    scene: &shell::Scene,
    beam: &shell::BeamGeometry,
    displacement: impl Fn(usize) -> Option<[f64; 3]>,
    center: [f64; 3],
    extent: f64,
    out: &mut Vec<Vertex>,
    vertex_ink: impl Fn(usize, Color32) -> Color32,
) -> bool {
    let Some(contour) = app.view_contour(beam.id) else {
        return false;
    };
    let Some(&origin) = contour.points.first() else {
        return false;
    };
    let [a, b] = beam.nodes;
    let (Some(da), Some(db)) = (displacement(a), displacement(b)) else {
        return false;
    };
    if !da.into_iter().chain(db).all(f64::is_finite) {
        return false;
    }
    let corners = crate::plan_trace::columns::section_corners(contour, &beam.section);
    let ink = if app.is_selected(beam.id) {
        Color32::YELLOW
    } else {
        app.object_ink(beam.id, Kind::Column)
    };
    let rings = [(a, da), (b, db)].map(|(node, delta)| {
        let [x, y, z] = scene.xyz[node];
        corners.map(|p| {
            let mut v = vertex(
                [x + p[0] - origin[0], y + p[1] - origin[1], z],
                vertex_ink(node, ink),
                center,
                extent,
            );
            v.offset = delta.map(|value| (value / extent) as f32);
            v
        })
    });
    let [lower, upper] = rings;
    for i in 0..4 {
        let j = (i + 1) % 4;
        out.extend([lower[i], lower[j], upper[j], lower[i], upper[j], upper[i]]);
    }
    for ring in rings {
        out.extend([ring[0], ring[1], ring[2], ring[0], ring[2], ring[3]]);
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn source_ribbons_keep_wall_verticals_and_follow_visibility_flags() {
        let mut app = App::new_project();
        app.spatial.enabled = true;
        app.drawing_visibility.support_extents = false;
        app.drawing_visibility.structure_edges = false;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([4000., 4000.], false);
        for (kind, a, b) in [
            (Kind::Support, [0., 0.], [4000., 0.]),
            (Kind::Beam, [0., 2000.], [4000., 2000.]),
        ] {
            app.begin(kind);
            app.choose_tool(Tool::Line);
            app.sketch_click(a, false);
            app.sketch_click(b, false);
            app.finish();
        }
        app.begin(Kind::Column);
        app.sketch_click([4000., 4000.], false);
        app.cancel();
        assert_eq!(app.contours.len(), 4, "{}", app.status);
        app.sync_building_objects();
        app.sync_stiffness_sources();
        let scene =
            spatial_ui::building_source_scene(&app.contours, &app.support_height, &app.building)
                .unwrap();
        let shown = geometry(&app, &scene, [0.; 3], 4000., None, vec![0]);
        assert_eq!(shown.edges.len(), 10); // four plate, four wall, beam and column
        let wall = vertex([0.; 3], app.contour_color(Kind::Support), [0.; 3], 4000.).color;
        assert_eq!(
            shown
                .edges
                .iter()
                .filter(|e| e[0].color == wall && e[0].offset[2].abs() == 1.5)
                .count(),
            2
        );
        assert!(Arc::ptr_eq(
            &shown,
            &geometry(&app, &scene, [0.; 3], 4000., None, vec![0])
        ));

        app.drawing_visibility.contours[2] = false;
        app.drawing_visibility.beam = false;
        app.view_visibility.contours.insert(app.contours[3].id);
        app.view_visibility_changed();
        let hidden = geometry(&app, &scene, [0.; 3], 4000., None, vec![1]);
        assert_eq!(hidden.edges.len(), 4);
        app.drawing_visibility.structure_edges = true;
        let outlined = geometry(&app, &scene, [0.; 3], 4000., None, vec![2]);
        let black = vertex([0.; 3], STRUCTURE_EDGE_INK, [0.; 3], 4000.).color;
        assert_eq!(
            outlined
                .edges
                .iter()
                .filter(|e| e[0].color == black)
                .count(),
            8
        );
        assert!(outlined.edges.iter().all(|e| e[0].color != wall));
    }

    #[test]
    fn rigid_link_uses_translucent_fill_and_two_diagonals_with_shared_visibility() {
        let mut app = App::new_project();
        app.spatial.enabled = true;
        app.begin(Kind::RigidLink);
        app.sketch_click([0., 0.], false);
        app.sketch_click([1000., 0.], false);
        app.drawing_visibility.rigid_bodies = true;
        app.drawing_visibility.rigid_color = [216, 101, 30];
        let contour = &app.contours[0];
        let id = contour.id;
        let face = rigid_link_face(&app, contour).unwrap();
        let scene = shell::Scene {
            xyz: vec![],
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let shown = geometry(&app, &scene, [0.; 3], 1000., None, vec![0]);
        let ink = app.contour_color(Kind::RigidLink);
        let fill = vertex(face[0], ink.gamma_multiply(0.28), [0.; 3], 1000.);
        assert_eq!(shown.faces.len(), 6);
        assert_eq!(shown.edges.len(), 2);
        assert!(shown.faces.iter().all(|v| v.color == fill.color));
        for (edge, [a, b]) in shown.edges.iter().zip([[0, 2], [1, 3]]) {
            for (v, i) in edge.iter().zip([a, b, b, a]) {
                let expected = vertex(face[i], ink, [0.; 3], 1000.);
                assert_eq!(v.position, expected.position);
                assert_eq!(v.color, expected.color);
            }
        }
        app.drawing_visibility.rigid_bodies = false;
        let hidden = geometry(&app, &scene, [0.; 3], 1000., None, vec![1]);
        assert!(hidden.faces.is_empty() && hidden.edges.is_empty());
        app.drawing_visibility.rigid_bodies = true;
        app.view_visibility.contours.insert(id);
        let hidden = geometry(&app, &scene, [0.; 3], 1000., None, vec![2]);
        assert!(hidden.faces.is_empty() && hidden.edges.is_empty());
    }

    #[test]
    fn structure_cut_edges_respect_other_planes_and_skip_coplanar_faces() {
        let face = [[0., 0., 0.], [4., 0., 0.], [4., 4., 0.], [0., 4., 0.]];
        let mut sections = sections::State::default();
        sections.planes[0].enabled = true;
        assert!(section_edges(&face, &sections).is_empty());
        sections.planes[1].enabled = true;
        sections.planes[1].offset_mm = 2.;
        sections.planes[2].enabled = true;
        sections.planes[2].offset_mm = 1.;
        let edges = section_edges(&face, &sections);
        assert_eq!(edges.len(), 2);
        assert!(edges.iter().flatten().all(|&p| sections.contains(p)));
        assert!(edges.iter().any(|edge| edge.iter().all(|p| p[1] == 2.)));
        assert!(edges.iter().any(|edge| edge.iter().all(|p| p[0] == 1.)));
    }

    #[test]
    fn deformed_quad_is_filled_and_keeps_displacement_separate_from_source() {
        let scene = shell::Scene {
            xyz: vec![
                [0., 0., 0.],
                [1000., 0., 0.],
                [1000., 1000., 0.],
                [0., 1000., 0.],
            ],
            plan: vec![0, 1, 2, 3],
            surfaces: vec![shell::Surface {
                rigid_z_mm: 0.,
                nodes: vec![0, 1, 2, 3],
                material: Default::default(),
                display: vec![0, 1],
                support: None,
            }],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let before = scene.xyz.clone();
        let mut app = App::default();
        app.drawing_visibility.fill = true;
        app.slab_color = [60, 120, 180];
        app.slab_opacity = 0.55;
        let faces = deformation_faces(
            &scene,
            &[0],
            |n| Some([n as f64, 0., -10.]),
            [0.; 3],
            1000.,
            |_, surface| surface_fill_ink(&app, surface, &scene.xyz).map(|ink| (ink, None)),
            None,
            |_, _| None,
        );
        assert_eq!(faces.len(), 6);
        let ink = Color32::from_rgb(60, 120, 180).gamma_multiply(0.55);
        assert!(faces
            .iter()
            .all(|v| v.color == vertex([0.; 3], ink, [0.; 3], 1000.).color));
        assert!(faces.iter().all(|v| (v.offset[2] + 0.01).abs() < 1e-7));
        assert_eq!(faces[0].position, [0.; 3]);
        assert_eq!(scene.xyz, before);
        let scale = crate::plan_trace::result_scale::Scale::default();
        let bands = deformation_faces(
            &scene,
            &[0],
            |n| Some([n as f64, 0., -10.]),
            [0.; 3],
            1000.,
            |_, _| Some((ink, None)),
            Some((&scale, [0., 3.])),
            |_, node| Some(node as f64),
        );
        assert!(bands.len() > faces.len());
        assert!(bands.iter().all(|v| (v.offset[2] + 0.01).abs() < 1e-7));
        assert!(bands.iter().any(|v| v.color != bands[0].color));
        app.drawing_visibility.fill = false;
        assert!(deformation_faces(
            &scene,
            &[0],
            |_| Some([0.; 3]),
            [0.; 3],
            1000.,
            |_, surface| surface_fill_ink(&app, surface, &scene.xyz).map(|ink| (ink, None)),
            None,
            |_, _| None,
        )
        .is_empty());
        assert!(deformation_faces(
            &scene,
            &[],
            |_| Some([0.; 3]),
            [0.; 3],
            1000.,
            |_, _| Some((Color32::YELLOW, None)),
            None,
            |_, _| None
        )
        .is_empty());
        assert!(deformation_faces(
            &scene,
            &[0],
            |_| Some([f64::NAN; 3]),
            [0.; 3],
            1000.,
            |_, _| Some((Color32::YELLOW, None)),
            None,
            |_, _| None
        )
        .is_empty());
    }

    #[test]
    fn gpu_world_matrix_matches_cpu_projection_for_orbit_zoom_pan_and_depth() {
        let rect = Rect::from_min_size(Pos2::new(35., 70.), egui::vec2(930., 610.));
        let extent = 20000.;
        let points = [[0.1, -0.3, 0.05], [-0.4, 0.2, -0.15], [0.25, 0.4, 0.2]];
        for (yaw, pitch, zoom) in [(0_f64, 0_f64, 1.), (0.8, 0.65, 2.5), (-1.3, -0.4, 0.3)] {
            let scale = rect.width().min(rect.height()) as f64 * 0.85 * zoom / extent;
            let pan = egui::vec2(41., -27.);
            let (sy, cy) = yaw.sin_cos();
            let (sp, cp) = pitch.sin_cos();
            let projected = points.map(|p| {
                let [x, y, z] = p.map(|v| v * extent);
                let a = cy * x - sy * y;
                let b = sy * x + cy * y;
                (
                    rect.center()
                        + pan
                        + egui::vec2((a * scale) as f32, (-(cp * b - sp * z) * scale) as f32),
                    sp * b + cp * z,
                )
            });
            let matrix = camera(rect, &projected, extent, scale, yaw, pitch, pan);
            let near = projected
                .iter()
                .map(|p| p.1)
                .fold(f64::NEG_INFINITY, f64::max);
            let far = projected.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
            for (p, cpu) in points.into_iter().zip(projected) {
                let p = [p[0] as f32, p[1] as f32, p[2] as f32, 1.];
                let out: [f32; 4] =
                    std::array::from_fn(|r| (0..4).map(|c| matrix[c][r] * p[c]).sum());
                let expected = [
                    (cpu.0.x - rect.left()) / rect.width() * 2. - 1.,
                    1. - (cpu.0.y - rect.top()) / rect.height() * 2.,
                    ((near - cpu.1) / (near - far).max(1.) * 0.98 + 0.01) as f32,
                ];
                for a in 0..3 {
                    assert!(
                        (out[a] - expected[a]).abs() < 1e-6,
                        "axis={a}: {out:?} {expected:?}"
                    );
                }
            }
        }
    }
}
