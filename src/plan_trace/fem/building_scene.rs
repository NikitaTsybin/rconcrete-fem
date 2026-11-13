#[path = "wall_mesh.rs"]
mod wall_mesh;
// Monolithic shell geometry. Adjacent storeys share physical joint IDs.
use super::{shell::*, *};
pub(super) fn scene(
    mesh: &Mesh,
    materials: &[Material],
    settings: &shell::Settings,
    step: f64,
) -> Result<shell::Scene, String> {
    let data = mesh
        .building
        .as_ref()
        .ok_or("Отсутствует пространственная топология")?;
    data.layout.validate()?;
    if data.z.len() != mesh.nodes.len()
        || data.node_floors.len() != mesh.nodes.len()
        || data.element_floors.len() != mesh.elements.len()
        || materials.len() != mesh.elements.len()
    {
        return Err("Несогласованные размеры пространственной сетки".into());
    }
    let mut weld = crate::plan_trace::building_mesh::Weld::new();
    data.validate(mesh)?;
    data.validate_beam_edges(mesh, |id| settings.columns.contains_key(&id))?;
    for (i, (n, z)) in mesh.nodes.iter().zip(&data.z).enumerate() {
        if weld.add([n.xy[0], n.xy[1], *z]) != i {
            return Err("Повторный узел пространственной сетки".into());
        }
    }
    let mut sc = shell::Scene {
        xyz: vec![],
        plan: (0..mesh.nodes.len()).collect(),
        surfaces: vec![],
        fixed: BTreeSet::new(),
        support_nodes: BTreeMap::new(),
        fixed_rotations: Default::default(),
        beams: vec![],
    };
    let indices = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let mut owned = BTreeSet::new();
    for quad in &mesh.quads {
        let display = quad
            .display_triangles
            .iter()
            .map(|id| {
                indices
                    .get(id)
                    .copied()
                    .ok_or("Нет тесселяции четырёхугольника здания")
            })
            .collect::<Result<Vec<_>, _>>()?;
        if display[0] == display[1]
            || display.iter().any(|i| owned.contains(i))
            || materials[display[0]] != materials[display[1]]
            || data.element_floors[display[0]] != data.element_floors[display[1]]
        {
            return Err("Четырёхугольник здания пересекает границу этажей или жёсткостей".into());
        }
        owned.extend(display.iter().copied());
        sc.surfaces.push(shell::Surface {
            rigid_z_mm: 0.,
            nodes: quad.nodes.to_vec(),
            material: materials[display[0]],
            display,
            support: None,
        });
    }
    for (i, e) in mesh
        .elements
        .iter()
        .enumerate()
        .filter(|(i, _)| !owned.contains(i))
    {
        sc.surfaces.push(shell::Surface {
            rigid_z_mm: 0.,
            nodes: e.nodes.to_vec(),
            material: materials[i],
            display: vec![i],
            support: None,
        });
    }
    let computed;
    let (wall_triangles, wall_quads) = match data
        .wall_mesh
        .as_ref()
        .filter(|_| data.wall_mesh_step == step)
    {
        Some(triangles) => (triangles.as_slice(), data.wall_quads.as_slice()),
        None if data.wall_quad_options.backend
            == crate::plan_trace::mesh_backend::Backend::Gmsh =>
        {
            return Err("Сетка стен Gmsh отсутствует или устарела. Перестройте сетку: построение выполняется в фоне при триангуляции".into());
        }
        None => {
            computed = wall_triangles(mesh, step)?;
            (computed.triangles.as_slice(), computed.quads.as_slice())
        }
    };
    use crate::plan_trace::mesh_pack::{quad_split, valid_quad, valid_triangle};
    let mut used = BTreeMap::<Vec<usize>, (Material, u64)>::new();
    let mut split_quads = Vec::new();
    for &(id, points) in wall_quads {
        let ns = match weld.add_face(points, valid_quad) {
            Ok(nodes) => nodes,
            Err(welded) => {
                let split = quad_split(welded).ok_or_else(|| {
                    format!("Стена {id}: четырёхугольник пересекает сам себя после сварки узлов")
                })?;
                split_quads.extend(split.map(|triangle| (id, triangle.map(|i| welded[i]))));
                continue;
            }
        };
        let material = settings
            .materials
            .get(&id)
            .copied()
            .unwrap_or_else(Material::wall_default);
        material.validate()?;
        while sc.plan.len() < weld.xyz.len() {
            sc.plan.push(usize::MAX);
        }
        let mut key = ns.to_vec();
        key.sort();
        if let Some((old, old_id)) = used.get(&key) {
            if *old != material {
                return Err(format!("Совпадают КЭ стен {old_id} и {id}"));
            }
            continue;
        }
        used.insert(key, (material, id));
        sc.support_nodes.entry(id).or_default().extend(ns);
        sc.surfaces.push(shell::Surface {
            rigid_z_mm: 0.,
            nodes: ns.to_vec(),
            material,
            display: vec![],
            support: Some(id),
        });
    }
    for &(id, points) in wall_triangles.iter().chain(&split_quads) {
        let Ok(ns) = weld.add_face(points, valid_triangle) else {
            continue;
        };
        let material = settings
            .materials
            .get(&id)
            .copied()
            .unwrap_or_else(Material::wall_default);
        material.validate()?;
        while sc.plan.len() < weld.xyz.len() {
            sc.plan.push(usize::MAX);
        }
        let mut key = ns.to_vec();
        key.sort();
        if let Some((old, old_id)) = used.get(&key) {
            if *old != material {
                return Err(format!("Совпадают КЭ стен {old_id} и {id} с разными жёсткостями; узлы {ns:?}; координаты {points:?}"));
            }
            continue;
        }
        used.insert(key, (material, id));
        sc.support_nodes.entry(id).or_default().extend(ns);
        sc.surfaces.push(shell::Surface {
            rigid_z_mm: 0.,
            nodes: ns.to_vec(),
            material,
            display: vec![],
            support: Some(id),
        });
    }

    for beam in &data.beams {
        let section = settings
            .beams
            .get(&beam.id)
            .ok_or_else(|| format!("Балка {}: не назначена жёсткость", beam.id))?
            .clone();
        section.validate()?;
        sc.beams.push(shell::BeamGeometry {
            rigid_offset_mm: [0.; 2],
            roll_angle_rad: beam.roll_angle_rad,
            id: beam.id,
            nodes: beam.nodes,
            section,
        });
    }
    for nodes in sc.support_nodes.values_mut() {
        nodes.sort_unstable();
        nodes.dedup();
    }
    let rigid_pins = data
        .rigid_links
        .iter()
        .flat_map(|s| std::iter::once(&s.master).chain(&s.slaves))
        .filter_map(|a| weld.find(a.xyz))
        .collect();
    sc.xyz = weld.xyz;
    merge_wall_surfaces(
        &mut sc,
        mesh.nodes.len(),
        data.wall_quad_options,
        &rigid_pins,
    )?;
    crate::plan_trace::limits::check(
        sc.xyz.len(),
        sc.surfaces.len().saturating_add(sc.beams.len()),
    )?;
    Ok(sc)
}

fn wall_triangles(mesh: &Mesh, step: f64) -> Result<wall_mesh::Faces, String> {
    let data = mesh.building.as_ref().ok_or("Нет пространственной сетки")?;
    let mut plane_levels = data.z.clone();
    plane_levels.sort_by(f64::total_cmp);
    plane_levels.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
    let layers = compatible_wall_levels(data, mesh, step, &plane_levels)?;
    wall_mesh::triangulate(data, mesh, step, &layers)
}
pub(super) fn precompute_walls(mesh: &mut Mesh, step: f64) -> Result<(), String> {
    let triangles = wall_triangles(mesh, step)?;
    let data = mesh.building.as_mut().ok_or("Нет пространственной сетки")?;
    data.wall_mesh = Some(triangles.triangles);
    data.wall_quads = triangles.quads;
    data.wall_mesh_step = step;
    Ok(())
}

/// Pair and optimise physical wall cells in their own UV plane. Shared contact
/// nodes and plate nodes are immutable; the result is a real four-node shell.
fn merge_wall_surfaces(
    sc: &mut shell::Scene,
    plate_nodes: usize,
    options: crate::plan_trace::quad_mesh::Options,
    rigid_pins: &BTreeSet<usize>,
) -> Result<(), String> {
    if !options.merge || options.backend == crate::plan_trace::meshing::Backend::Gmsh {
        return Ok(());
    }
    struct Group {
        id: u64,
        origin: [f64; 3],
        axis: [f64; 2],
        faces: Vec<usize>,
    }
    let mut groups: Vec<Group> = Vec::new();
    let mut source_groups = BTreeMap::<u64, Vec<usize>>::new();
    for (i, s) in sc.surfaces.iter().enumerate() {
        let Some(id) = s.support else { continue };
        if s.nodes.len() != 3 {
            continue;
        }
        let q = s.nodes.iter().map(|&n| sc.xyz[n]).collect::<Vec<_>>();
        let edge = (0..3)
            .map(|k| [q[(k + 1) % 3][0] - q[k][0], q[(k + 1) % 3][1] - q[k][1]])
            .max_by(|a, b| a[0].hypot(a[1]).total_cmp(&b[0].hypot(b[1])))
            .unwrap();
        let len = edge[0].hypot(edge[1]);
        if len < 1e-8 {
            continue;
        }
        let mut axis = edge.map(|v| v / len);
        if axis[0] < -1e-10 || axis[0].abs() < 1e-10 && axis[1] < 0. {
            axis = axis.map(|v| -v);
        }
        let found = source_groups
            .get(&id)
            .into_iter()
            .flatten()
            .copied()
            .find(|&index| {
                let g = &groups[index];
                g.id == id
                    && (g.axis[0] * axis[1] - g.axis[1] * axis[0]).abs() < 1e-10
                    && ((q[0][0] - g.origin[0]) * g.axis[1] - (q[0][1] - g.origin[1]) * g.axis[0])
                        .abs()
                        < 1e-5
            });
        if let Some(g) = found {
            groups[g].faces.push(i);
        } else {
            source_groups.entry(id).or_default().push(groups.len());
            groups.push(Group {
                id,
                origin: q[0],
                axis,
                faces: vec![i],
            });
        }
    }
    // Regular wall quads bypass triangle pairing, but their junctions still
    // constrain the adjacent triangle mesh just like those of slabs.
    let regular_wall_nodes = sc
        .surfaces
        .iter()
        .filter(|s| s.support.is_some() && s.nodes.len() == 4)
        .flat_map(|s| s.nodes.iter().copied())
        .collect::<BTreeSet<_>>();
    let retained_edges = sc
        .surfaces
        .iter()
        .filter(|s| s.support.is_none() || s.nodes.len() == 4)
        .flat_map(|s| {
            (0..s.nodes.len()).map(move |i| {
                let mut edge = [s.nodes[i], s.nodes[(i + 1) % s.nodes.len()]];
                edge.sort();
                edge
            })
        })
        .collect::<BTreeSet<_>>();
    let mut edge_groups = BTreeMap::<[usize; 2], BTreeSet<usize>>::new();
    for (g, group) in groups.iter().enumerate() {
        for &f in &group.faces {
            let ns = &sc.surfaces[f].nodes;
            for i in 0..ns.len() {
                let mut edge = [ns[i], ns[(i + 1) % ns.len()]];
                edge.sort();
                edge_groups.entry(edge).or_default().insert(g);
            }
        }
    }
    let mut memberships = BTreeMap::<usize, BTreeSet<usize>>::new();
    for (g, group) in groups.iter().enumerate() {
        for &face in &group.faces {
            for &n in &sc.surfaces[face].nodes {
                memberships.entry(n).or_default().insert(g);
            }
        }
    }
    let mut replacement = BTreeMap::<usize, shell::Surface>::new();
    let mut removed = BTreeSet::new();
    for g in groups {
        let global = g
            .faces
            .iter()
            .flat_map(|&f| sc.surfaces[f].nodes.iter().copied())
            .collect::<BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>();
        let ids = global
            .iter()
            .enumerate()
            .map(|(i, &n)| (n, i))
            .collect::<BTreeMap<_, _>>();
        let mut local = Mesh {
            building: None,
            nodes: global
                .iter()
                .enumerate()
                .map(|(i, &n)| crate::plan_trace::meshing::Node {
                    id: i as u64 + 1,
                    xy: [
                        (sc.xyz[n][0] - g.origin[0]) * g.axis[0]
                            + (sc.xyz[n][1] - g.origin[1]) * g.axis[1],
                        sc.xyz[n][2],
                    ],
                })
                .collect(),
            elements: Vec::new(),
            quads: Vec::new(),
            edges: Vec::new(),
            constrained_edges: BTreeSet::new(),
            constrained_nodes: BTreeSet::new(),
            support_boundary: BTreeMap::new(),
        };
        let mut adjacency = BTreeMap::<[usize; 2], usize>::new();
        for (i, &f) in g.faces.iter().enumerate() {
            let ns = sc.surfaces[f]
                .nodes
                .iter()
                .map(|n| ids[n])
                .collect::<Vec<_>>();
            let nodes = [ns[0], ns[1], ns[2]];
            local.elements.push(crate::plan_trace::meshing::Element {
                id: i as u64 + 1,
                nodes,
                supports: vec![],
            });
            for k in 0..3 {
                let mut e = [nodes[k], nodes[(k + 1) % 3]];
                e.sort();
                *adjacency.entry(e).or_default() += 1;
            }
        }
        local.edges = adjacency.keys().copied().collect();
        for (i, &n) in global.iter().enumerate() {
            if n < plate_nodes
                || regular_wall_nodes.contains(&n)
                || rigid_pins.contains(&n)
                || memberships[&n].len() > 1
            {
                local.constrained_nodes.insert(i);
            }
        }
        for (&e, &count) in &adjacency {
            let mut physical = e.map(|n| global[n]);
            physical.sort();
            if count == 1
                || retained_edges.contains(&physical)
                || edge_groups.get(&physical).is_some_and(|g| g.len() > 1)
            {
                local.constrained_edges.insert(e);
                local.constrained_nodes.extend(e);
            }
        }
        let local = crate::plan_trace::quad_mesh::process(local, options)?;
        for (i, &n) in global.iter().enumerate() {
            if local.constrained_nodes.contains(&i) {
                continue;
            }
            sc.xyz[n] = [
                g.origin[0] + g.axis[0] * local.nodes[i].xy[0],
                g.origin[1] + g.axis[1] * local.nodes[i].xy[0],
                local.nodes[i].xy[1],
            ];
        }
        for q in &local.quads {
            let face = g.faces[q.display_triangles[0] as usize - 1];
            let other = g.faces[q.display_triangles[1] as usize - 1];
            let original = &local.elements[q.display_triangles[0] as usize - 1];
            let p = original.nodes.map(|n| local.nodes[n].xy);
            let negative = (p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0])
                < 0.;
            let mut ns = q.nodes.map(|n| global[n]).to_vec();
            if negative {
                ns.reverse();
            }
            let mut cell = sc.surfaces[face].clone();
            cell.nodes = ns;
            replacement.insert(face, cell);
            removed.insert(other);
        }
    }
    sc.surfaces = std::mem::take(&mut sc.surfaces)
        .into_iter()
        .enumerate()
        .filter_map(|(i, s)| {
            if removed.contains(&i) {
                None
            } else {
                Some(replacement.remove(&i).unwrap_or(s))
            }
        })
        .collect();
    Ok(())
}

/// Partition common height intervals identically before constructing wall faces.
/// Endpoints and plate planes are global anchors; per-wall uniform divisions
/// would otherwise introduce hanging nodes on adjoining vertical edges.
fn wall_levels(z0: f64, z1: f64, planes: &[f64], step: f64) -> Result<Vec<f64>, String> {
    if !z0.is_finite() || !z1.is_finite() || z1 - z0 < 100. || !step.is_finite() || step <= 0. {
        return Err("Некорректная высота стены здания или размер КЭ".into());
    }
    let mut anchors = vec![z0];
    anchors.extend(
        planes
            .iter()
            .copied()
            .filter(|&z| z > z0 + 1e-5 && z < z1 - 1e-5),
    );
    anchors.push(z1);
    anchors.sort_by(f64::total_cmp);
    anchors.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
    let mut levels = vec![z0];
    for pair in anchors.windows(2) {
        let count = ((pair[1] - pair[0]) / step).ceil().max(1.) as usize;
        if count > 10000 || levels.len().saturating_add(count) > 10001 {
            return Err("Превышен лимит разбиения стены здания по высоте".into());
        }
        for k in 1..=count {
            levels.push(if k == count {
                pair[1]
            } else {
                pair[0] + (pair[1] - pair[0]) * k as f64 / count as f64
            });
        }
    }
    Ok(levels)
}
#[cfg(test)]
mod level_tests {
    use super::*;
    #[test]
    fn fine_short_wall_refines_only_its_shared_height_band() {
        use crate::plan_trace::{
            building_mesh::{Data, Layout, Wall},
            meshing::Node,
        };
        let xy = [
            [0., 0.],
            [1000., 0.],
            [0., 0.],
            [1000., 0.],
            [0., 0.],
            [0., 1000.],
            [0., 0.],
            [0., 1000.],
        ];
        let mesh = Mesh {
            building: None,
            nodes: xy
                .into_iter()
                .enumerate()
                .map(|(i, xy)| Node {
                    id: i as u64 + 1,
                    xy,
                })
                .collect(),
            quads: vec![],
            elements: vec![],
            edges: vec![],
            constrained_edges: Default::default(),
            constrained_nodes: Default::default(),
            support_boundary: Default::default(),
        };
        let data = Data {
            wall_quad_options: Default::default(),
            wall_mesh_step: 0.,
            wall_mesh: None,
            wall_quads: vec![],
            contacts: vec![],
            rigid_links: vec![],
            layout: Layout::default(),
            z: vec![0., 0., 3000., 3000., 1200., 1200., 1600., 1600.],
            node_floors: vec![1; 8],
            element_floors: vec![],
            element_slabs: vec![],
            beams: vec![],
            walls: vec![
                Wall {
                    id: 1,
                    mesh_size_mm: 400.,
                    bottom: [0, 1],
                    top: [2, 3],
                },
                Wall {
                    id: 2,
                    mesh_size_mm: 100.,
                    bottom: [4, 5],
                    top: [6, 7],
                },
            ],
        };
        let layers =
            compatible_wall_levels(&data, &mesh, 500., &[0., 1200., 1600., 3000.]).unwrap();
        let common0 = layers[0][0]
            .iter()
            .copied()
            .filter(|&z| z >= 1200. && z <= 1600.)
            .collect::<Vec<_>>();
        assert_eq!(common0, layers[1][0]);
        assert!(layers[1][0].contains(&1300.) && layers[1][0].contains(&1400.));
        let independent = wall_levels(0., 3000., &[], 400.).unwrap();
        assert_eq!(
            layers[0][1], independent,
            "the other side of the coarse wall must not inherit fine levels"
        );
        assert_eq!(
            layers[0][0],
            vec![0., 400., 800., 1200., 1300., 1400., 1500., 1600., 1950., 2300., 2650., 3000.],
            "one partition per contact band, not a union of independent uniform grids"
        );
        let mut detached = mesh.clone();
        for n in &mut detached.nodes[4..] {
            n.xy[0] += 10000.;
        }
        let remote =
            compatible_wall_levels(&data, &detached, 500., &[0., 1200., 1600., 3000.]).unwrap();
        let coarse = wall_levels(0., 3000., &[], 400.).unwrap();
        assert_eq!(
            remote[0],
            [coarse.clone(), coarse],
            "disconnected doorway levels must not cut this wall"
        );
        let mut actual = mesh.clone();
        let mut data = data.clone();
        data.layout = crate::plan_trace::building_mesh::Layout {
            levels: vec![crate::plan_trace::building_mesh::Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1)]),
            legacy_wall_extent: None,
        };
        actual.building = Some(Box::new(data));
        let sc = scene(&actual, &[], &shell::Settings::default(), 500.).unwrap();
        assert!(
            sc.surfaces.iter().any(|s| s.nodes.len() == 3),
            "edge transitions require real triangles"
        );
        let edge = |id| {
            sc.support_nodes[&id]
                .iter()
                .copied()
                .filter(|&n| {
                    sc.xyz[n][0].abs() < 1e-6
                        && sc.xyz[n][1].abs() < 1e-6
                        && sc.xyz[n][2] >= 1200.
                        && sc.xyz[n][2] <= 1600.
                })
                .collect::<BTreeSet<_>>()
        };
        assert_eq!(
            edge(1),
            edge(2),
            "the common vertical edge must have identical welded node IDs"
        );
    }
    #[test]
    fn adjoining_unequal_and_shifted_walls_have_identical_common_edge_partition() {
        let planes = [-500., 0., 175., 2500., 3000.];
        let a = wall_levels(0., 3000., &planes, 500.).unwrap();
        let b = wall_levels(0., 2500., &planes, 500.).unwrap();
        let c = wall_levels(-500., 2500., &planes, 500.).unwrap();
        assert_eq!(
            a.iter()
                .copied()
                .filter(|&z| z <= 2500.)
                .collect::<Vec<_>>(),
            b
        );
        assert_eq!(
            c.iter().copied().filter(|&z| z >= 0.).collect::<Vec<_>>(),
            b
        );
        assert!(a.contains(&175.));
        assert!(a
            .windows(2)
            .all(|w| w[1] > w[0] && w[1] - w[0] <= 500. + 1e-8));
        assert_eq!(
            wall_levels(0., 3000., &[0., 2500., 3000.], 500.).unwrap(),
            vec![0., 500., 1000., 1500., 2000., 2500., 3000.]
        );
    }
}

fn compatible_wall_levels(
    data: &crate::plan_trace::building_mesh::Data,
    mesh: &Mesh,
    step: f64,
    _planes: &[f64],
) -> Result<Vec<[Vec<f64>; 2]>, String> {
    let xy_key = |p: [f64; 2]| p.map(|v| (v * 1e5).round() as i64);
    // Only actual slab contact nodes impose plate levels. Wall jambs / lintel
    // elevations elsewhere in the building are not infinite cutting planes.
    let mut plate_nodes = BTreeMap::<[i64; 2], Vec<f64>>::new();
    for n in mesh
        .elements
        .iter()
        .flat_map(|e| e.nodes)
        .collect::<BTreeSet<_>>()
    {
        plate_nodes
            .entry(xy_key(mesh.nodes[n].xy))
            .or_default()
            .push(data.z[n]);
    }
    let mut anchors = Vec::new();
    let mut junctions = BTreeMap::<[i64; 2], Vec<(usize, usize)>>::new();
    for (i, w) in data.walls.iter().enumerate() {
        let (lo, hi) = (data.z[w.bottom[0]], data.z[w.top[0]]);
        let mut planes = Vec::new();
        for side in 0..2 {
            let key = xy_key(mesh.nodes[w.bottom[side]].xy);
            planes.extend(plate_nodes.get(&key).into_iter().flatten().copied());
            junctions.entry(key).or_default().push((i, side));
        }
        let size = if w.mesh_size_mm > 0. {
            w.mesh_size_mm
        } else {
            step
        };
        anchors.push(wall_levels(lo, hi, &planes, size)?);
    }
    let mut out = anchors
        .iter()
        .map(|z| [z.clone(), z.clone()])
        .collect::<Vec<_>>();
    let mut contact_levels = BTreeMap::<[i64; 2], Vec<f64>>::new();
    for p in data.contacts.iter().flatten() {
        contact_levels
            .entry(xy_key([p[0], p[1]]))
            .or_default()
            .push(p[2]);
    }
    for (xy, incident) in &junctions {
        // Partition the physical common edge ONCE. Merging independently spaced
        // per-wall grids here creates short alternating edges (e.g. 375/400 mm).
        let mut breaks = incident
            .iter()
            .flat_map(|&(i, _)| [anchors[i][0], *anchors[i].last().unwrap()])
            .collect::<Vec<_>>();
        breaks.extend(plate_nodes.get(xy).into_iter().flatten().copied());
        breaks.extend(contact_levels.get(xy).into_iter().flatten().copied());
        breaks.sort_by(f64::total_cmp);
        breaks.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        let mut shared = Vec::new();
        for interval in breaks.windows(2) {
            let middle = (interval[0] + interval[1]) * 0.5;
            let h = incident
                .iter()
                .filter_map(|&(i, _)| {
                    if middle < anchors[i][0] - 1e-5 || middle > *anchors[i].last().unwrap() + 1e-5
                    {
                        return None;
                    }
                    let value = data.walls[i].mesh_size_mm;
                    Some(if value > 0. { value } else { step })
                })
                .fold(f64::INFINITY, f64::min);
            if !h.is_finite() {
                continue;
            }
            let count = ((interval[1] - interval[0]) / h).ceil().max(1.) as usize;
            if count > 10000 || shared.len().saturating_add(count) > 20002 {
                return Err("Превышен лимит узлов общего ребра стены".into());
            }
            for n in 0..=count {
                shared.push(if n == count {
                    interval[1]
                } else {
                    interval[0] + (interval[1] - interval[0]) * n as f64 / count as f64
                });
            }
        }
        shared.sort_by(f64::total_cmp);
        shared.dedup_by(|a, b| (*a - *b).abs() < 1e-5);
        for &(i, side) in incident {
            let lo = anchors[i][0];
            let hi = *anchors[i].last().unwrap();
            out[i][side] = shared
                .iter()
                .copied()
                .filter(|&z| z >= lo - 1e-5 && z <= hi + 1e-5)
                .collect();
            if out[i][side].len() > 10001 {
                return Err("Превышен лимит узлов общего ребра стены".into());
            }
        }
    }
    Ok(out)
}

#[cfg(test)]
mod wall_quad_regression {
    use super::*;

    #[test]
    fn triangle_optimisation_preserves_an_interior_junction_with_a_regular_quad() {
        let xyz = vec![
            [0., 0., 0.],
            [1000., 0., 0.],
            [1000., 0., 1000.],
            [0., 0., 1000.],
            [100., 0., 500.],
            [100., 300., 500.],
            [100., 300., 800.],
            [100., 0., 800.],
        ];
        let wall = |id, nodes| shell::Surface {
            rigid_z_mm: 0.,
            nodes,
            material: Material::wall_default(),
            display: vec![],
            support: Some(id),
        };
        let mut sc = shell::Scene {
            xyz: xyz.clone(),
            plan: vec![usize::MAX; xyz.len()],
            surfaces: (0..4).map(|n| wall(1, vec![n, (n + 1) % 4, 4])).collect(),
            fixed: BTreeSet::new(),
            support_nodes: BTreeMap::new(),
            fixed_rotations: BTreeSet::new(),
            beams: vec![],
        };
        let options = crate::plan_trace::quad_mesh::Options {
            backend: Default::default(),
            merge: true,
            optimise: true,
        };
        let mut alone = sc.clone();
        merge_wall_surfaces(&mut alone, 0, options, &BTreeSet::new()).unwrap();
        assert!(alone.xyz[4][0] > xyz[4][0]);

        sc.surfaces.push(wall(2, vec![4, 5, 6, 7]));
        merge_wall_surfaces(&mut sc, 0, options, &BTreeSet::new()).unwrap();
        assert_eq!(sc.xyz[4], xyz[4]);
    }

    #[test]
    fn vertical_wall_pair_is_a_real_quad_and_preserves_boundary_nodes() {
        let original = vec![
            [0., 0., 0.],
            [2000., 0., 0.],
            [2000., 0., 3000.],
            [0., 0., 3000.],
        ];
        let mut sc = shell::Scene {
            xyz: original.clone(),
            plan: vec![usize::MAX; 4],
            surfaces: vec![
                shell::Surface {
                    rigid_z_mm: 0.,
                    nodes: vec![0, 1, 2],
                    material: Material::wall_default(),
                    display: vec![],
                    support: Some(1),
                },
                shell::Surface {
                    rigid_z_mm: 0.,
                    nodes: vec![0, 2, 3],
                    material: Material::wall_default(),
                    display: vec![],
                    support: Some(1),
                },
            ],
            fixed: BTreeSet::new(),
            support_nodes: BTreeMap::new(),
            fixed_rotations: BTreeSet::new(),
            beams: vec![],
        };
        let mut frozen = sc.clone();
        frozen.surfaces[0].nodes = vec![0, 1, 3];
        frozen.surfaces[1].nodes = vec![1, 2, 3];
        merge_wall_surfaces(
            &mut frozen,
            4,
            crate::plan_trace::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
            &BTreeSet::new(),
        )
        .unwrap();
        assert_eq!(
            frozen.surfaces.len(),
            1,
            "a fixed endpoint is not a constrained diagonal"
        );
        assert_eq!(frozen.xyz, original);
        merge_wall_surfaces(
            &mut sc,
            0,
            crate::plan_trace::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: true,
            },
            &BTreeSet::new(),
        )
        .unwrap();
        assert_eq!(sc.surfaces.len(), 1);
        assert_eq!(sc.surfaces[0].nodes.len(), 4);
        assert_eq!(sc.surfaces[0].support, Some(1));
        assert_eq!(sc.xyz, original);
        let p = sc.surfaces[0]
            .nodes
            .iter()
            .map(|&n| [sc.xyz[n][0] / 1000., sc.xyz[n][2] / 1000.])
            .collect::<Vec<_>>();
        super::super::mitc4::Quad4::new(p.try_into().unwrap()).unwrap();
    }
}
