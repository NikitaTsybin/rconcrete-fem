//! Column joints and exact rigid-body footprints. Coordinates in mm.
use super::{
    building_mesh::Layout,
    fem::{beam::Section, shell},
    geometry::Point,
    model::{Contour, Kind},
};
use std::collections::BTreeMap;
#[derive(Clone, Debug)]
pub struct Joint {
    pub z: f64,
    pub floor: u64,
    pub master: Point,
    pub bounds: [f64; 4],
    pub outline: Vec<Point>,
    pub lower: f64,
}
pub fn corners(b: [f64; 4]) -> [Point; 4] {
    [[b[0], b[1]], [b[2], b[1]], [b[2], b[3]], [b[0], b[3]]]
}
pub fn section_corners(c: &Contour, s: &Section) -> [Point; 4] {
    let p = c.points[0];
    let (sin, cos) = c.column_angle.sin_cos();
    corners(bounds([0., 0.], s)).map(|q| {
        [
            p[0] + cos * q[0] - sin * q[1],
            p[1] + sin * q[0] + cos * q[1],
        ]
    })
}
fn envelope(q: &[Point]) -> [f64; 4] {
    [
        q.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min),
        q.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min),
        q.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max),
        q.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max),
    ]
}
pub fn contains(j: &Joint, p: Point) -> bool {
    super::geometry::inside(p, &j.outline)
        || (0..j.outline.len()).any(|i| {
            let a = j.outline[i];
            let b = j.outline[(i + 1) % j.outline.len()];
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            let t = if l > 0. {
                ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l
            } else {
                0.
            };
            let t = t.clamp(0., 1.);
            (p[0] - a[0] - t * d[0]).hypot(p[1] - a[1] - t * d[1]) < 1e-6
        })
}
pub fn bounds(p: Point, s: &Section) -> [f64; 4] {
    [
        p[0] - s.width_mm / 2.,
        p[1] - s.material.thickness_mm / 2.,
        p[0] + s.width_mm / 2.,
        p[1] + s.material.thickness_mm / 2.,
    ]
}
fn outlines_intersect(a: &[Point], b: &[Point]) -> bool {
    let polygon = |p: &[Point]| {
        mini_fem::geo::Polygon::new(
            mini_fem::geo::LineString::from(p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
            vec![],
        )
    };
    mini_fem::geo::Intersects::intersects(&polygon(a), &polygon(b))
}
/// True source contact, including holes; a remote slab's elevation is not a joint.
fn plate_contact(column: &Contour, section: &Section, slab: &Contour, all: &[Contour]) -> bool {
    use mini_fem::geo::{Intersects, LineString, Polygon};
    let ring = |p: &[Point]| {
        let mut p = p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
        if !p.is_empty() {
            p.push(p[0]);
        }
        LineString::from(p)
    };
    let footprint = Polygon::new(ring(&section_corners(column, section)), vec![]);
    let holes = all
        .iter()
        .filter(|c| c.active_hole() && c.parent == Some(slab.id))
        .map(|h| ring(&h.points))
        .collect();
    footprint.intersects(&Polygon::new(ring(&slab.points), holes))
}
fn end_contact(
    column: &Contour,
    section: &Section,
    z: f64,
    all: &[Contour],
    sections: &BTreeMap<u64, Section>,
    layout: &Layout,
) -> bool {
    use mini_fem::geo::{Intersects, LineString, Polygon};
    let ring = |p: &[Point]| {
        let mut p = p.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
        if !p.is_empty() {
            p.push(p[0]);
        }
        LineString::from(p)
    };
    let footprint = Polygon::new(ring(&section_corners(column, section)), vec![]);
    all.iter().filter(|c| c.id != column.id).any(|c| {
        let h = layout.extent(c);
        match c.kind {
            Kind::Slab if (h[0] - z).abs() < 1e-5 => plate_contact(column, section, c, all),
            Kind::Column if (h[0] - z).abs() < 1e-5 || (h[1] - z).abs() < 1e-5 => {
                sections.get(&c.id).is_some_and(|s| {
                    footprint.intersects(&Polygon::new(ring(&section_corners(c, s)), vec![]))
                })
            }
            Kind::Support if z >= h[0] - 1e-5 && z <= h[1] + 1e-5 => {
                let mut path = c.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                footprint.intersects(&LineString::from(path))
            }
            Kind::Beam if (h[0] - z).abs() < 1e-5 => footprint.intersects(&LineString::from(
                c.points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>(),
            )),
            _ => false,
        }
    })
}
pub fn joints(
    all: &[Contour],
    sections: &BTreeMap<u64, Section>,
    layout: Option<&Layout>,
) -> Vec<Joint> {
    let mut groups = vec![];
    for c in all.iter().filter(|c| c.kind == Kind::Column) {
        let (Some(&p), Some(section)) = (c.points.first(), sections.get(&c.id)) else {
            continue;
        };
        let floor = layout.map_or(0, |l| l.owner(c.id));
        let extent = layout.map_or([0., 0.], |l| l.extent(c));
        let mut levels = BTreeMap::<i64, u64>::new();
        if let Some(layout) = layout {
            for slab in all.iter().filter(|c| c.kind == Kind::Slab) {
                let at = layout.extent(slab)[0];
                if at >= extent[0] - 1e-6
                    && at <= extent[1] + 1e-6
                    && plate_contact(c, section, slab, all)
                {
                    levels.insert((at * 1e6).round() as i64, layout.owner(slab.id));
                }
            }
            for z in extent {
                if end_contact(c, section, z, all, sections, layout) {
                    let owner = layout
                        .levels
                        .iter()
                        .find(|l| (l.z - z).abs() < 1e-6)
                        .map_or(floor, |l| l.id);
                    levels.entry((z * 1e6).round() as i64).or_insert(owner);
                }
            }
        } else {
            levels.insert(0, floor);
        }
        for (at, floor) in levels {
            groups.push(Joint {
                z: at as f64 / 1e6,
                floor,
                master: p,
                bounds: envelope(&section_corners(c, section)),
                outline: section_corners(c, section).to_vec(),
                lower: extent[0],
            });
        }
    }

    // Build connected components once. Restarting the pairwise search after
    // every merge made large column libraries cubic in the number of joints.
    let mut parents = (0..groups.len()).collect::<Vec<_>>();
    fn root(parents: &mut [usize], mut i: usize) -> usize {
        while parents[i] != i {
            parents[i] = parents[parents[i]];
            i = parents[i];
        }
        i
    }
    let mut planes = BTreeMap::<(u64, i64), Vec<usize>>::new();
    for (i, j) in groups.iter().enumerate() {
        planes
            .entry((j.floor, (j.z * 1e6).round() as i64))
            .or_default()
            .push(i);
    }
    for ids in planes.values_mut() {
        ids.sort_by(|&a, &b| groups[a].bounds[0].total_cmp(&groups[b].bounds[0]));
        for (at, &i) in ids.iter().enumerate() {
            for &j in &ids[at + 1..] {
                let (a, b) = (&groups[i], &groups[j]);
                if b.bounds[0] > a.bounds[2] + 1e-6 {
                    break;
                }
                if a.bounds[1] > b.bounds[3] + 1e-6 || b.bounds[1] > a.bounds[3] + 1e-6 {
                    continue;
                }
                if outlines_intersect(&a.outline, &b.outline) {
                    let (a, b) = (root(&mut parents, i), root(&mut parents, j));
                    parents[b] = a;
                }
            }
        }
    }
    let mut merged = BTreeMap::<usize, Joint>::new();
    for (i, b) in groups.into_iter().enumerate() {
        let key = root(&mut parents, i);
        if let Some(a) = merged.get_mut(&key) {
            if b.lower < a.lower {
                a.master = b.master;
                a.lower = b.lower;
            }
            let same_outline = a.outline == b.outline;
            a.bounds = [
                a.bounds[0].min(b.bounds[0]),
                a.bounds[1].min(b.bounds[1]),
                a.bounds[2].max(b.bounds[2]),
                a.bounds[3].max(b.bounds[3]),
            ];
            if !same_outline {
                a.outline = corners(a.bounds).to_vec();
            }
        } else {
            merged.insert(key, b);
        }
    }
    let groups = merged.into_values().collect();
    groups
}
pub fn meshing_sources(
    all: &[Contour],
    settings: &shell::Settings,
    layout: &mut Option<Layout>,
) -> Vec<Contour> {
    let keep_column_axes = layout.is_some() && all.iter().any(|c| c.kind == Kind::RigidLink);
    let mut output = all
        .iter()
        .filter(|c| c.kind != Kind::Column || keep_column_axes)
        .cloned()
        .collect::<Vec<_>>();
    let mut id = all.iter().map(|c| c.id).max().unwrap_or(0) + 1;

    let js = joints(all, &settings.columns, layout.as_ref());
    let mut additions = vec![];
    for c in all.iter().filter(|c| c.kind == Kind::Column) {
        if let Some(&p) = c.points.first() {
            if let Some(l) = layout.as_ref() {
                if l.levels.iter().any(|v| v.id == l.owner(c.id)) {
                    let extent = l.extent(c);
                    for slab in all.iter().filter(|c| c.kind == Kind::Slab) {
                        let z = l.extent(slab)[0];
                        if z >= extent[0] - 1e-5
                            && z <= extent[1] + 1e-5
                            && settings
                                .columns
                                .get(&c.id)
                                .is_some_and(|s| plate_contact(c, s, slab, all))
                        {
                            additions.push((Kind::LoadPoint, vec![p], l.owner(slab.id), z));
                        }
                    }
                }
            } else {
                additions.push((Kind::LoadPoint, vec![p], 0, 0.));
            }
        }
    }
    if settings.rigid_columns {
        for j in js {
            additions.push((Kind::LoadPoint, vec![j.master], j.floor, j.z));
            additions.push((Kind::Triangulation, j.outline.clone(), j.floor, j.z));
        }
    }
    for (kind, points, floor, z) in additions {
        let mut placement = super::model::Placement::default();
        if let Some(l) = layout.as_ref() {
            if let Some(level) = l.levels.iter().find(|v| v.id == floor) {
                placement.slab.offset_mm = z - level.z;
            }
        }
        let load = (kind == Kind::LoadPoint).then(|| super::loads::LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: true,
            case_id: u64::MAX,
            magnitude: super::loads::Magnitude {
                value: 0.,
                units: super::loads::ForceUnits::KiloNewton,
            },
            closed: false,
        });
        output.push(Contour {
            name: String::new(),
            placement,
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id,
            kind,
            points,
            parent: None,
            curves: vec![],
            grid: None,
            support_open: false,
            refinement: None,
            load,
        });
        if let Some(l) = layout.as_mut() {
            l.owners.insert(id, floor);
        }
        id += 1;
    }

    output
}
pub fn attach(
    scene: &mut shell::Scene,
    mesh: &super::meshing::Mesh,
    all: &[Contour],
    settings: &shell::Settings,
) {
    let layout = mesh.building.as_ref().map(|d| &d.layout);
    let free_ids = all
        .iter()
        .filter(|c| c.is_free_beam())
        .map(|c| c.id)
        .collect::<std::collections::BTreeSet<_>>();
    let free_beam_ids = &free_ids;
    let beam_nodes: rstar::RTree<[f64; 3]> = rstar::RTree::bulk_load(
        mesh.building
            .iter()
            .flat_map(|data| {
                data.beams
                    .iter()
                    .filter(move |beam| free_beam_ids.contains(&beam.id))
                    .flat_map(move |beam| {
                        beam.nodes.map(|n| {
                            let [x, y] = mesh.nodes[n].xy;
                            [x, y, data.z[n]]
                        })
                    })
            })
            .collect(),
    );
    let load_pins = rstar::RTree::bulk_load(layout.map_or_else(Vec::new, |layout| {
        all.iter()
            .filter(|c| {
                c.kind == Kind::LoadPoint
                    && c.load
                        .as_ref()
                        .is_some_and(|l| l.moment || l.mesh_constraint || l.direction.index() != 2)
            })
            .filter_map(|c| {
                c.points
                    .first()
                    .map(|&[x, y]| [x, y, layout.source_z(c, all)])
            })
            .collect()
    }));
    let tolerance = if free_ids.is_empty() {
        1e-6
    } else {
        super::building_mesh::WELD_TOLERANCE_MM
    };
    for c in all.iter().filter(|c| c.kind == Kind::Column) {
        let (Some(&p), Some(section)) = (c.points.first(), settings.columns.get(&c.id)) else {
            continue;
        };
        let extent = layout
            .map(|l| l.extent(c))
            .unwrap_or_else(|| settings.extent());
        let zs = if layout.is_some() {
            let mut zs = vec![extent[0], extent[1]];
            zs.extend(
                all.iter()
                    .filter(|slab| slab.kind == Kind::Slab && plate_contact(c, section, slab, all))
                    .map(|slab| layout.unwrap().extent(slab)[0])
                    .filter(|&z| z > extent[0] + 1e-6 && z < extent[1] - 1e-6),
            );
            if let Some(data) = &mesh.building {
                zs.extend(
                    data.rigid_links
                        .iter()
                        .filter_map(|s| (s.master.owner == c.id).then_some(s.master.xyz[2])),
                );
                let query = rstar::AABB::from_corners(
                    [p[0] - tolerance, p[1] - tolerance, extent[0]],
                    [p[0] + tolerance, p[1] + tolerance, extent[1]],
                );
                zs.extend(
                    beam_nodes
                        .locate_in_envelope_intersecting(&query)
                        .chain(load_pins.locate_in_envelope_intersecting(&query))
                        .filter(|q| {
                            (q[0] - p[0]).abs() < tolerance && (q[1] - p[1]).abs() < tolerance
                        })
                        .map(|p| p[2]),
                );
            }
            zs.sort_by(f64::total_cmp);
            zs.dedup_by(|a, b| (*a - *b).abs() < tolerance);
            zs
        } else if settings.enabled {
            let mut z = vec![extent[0], 0., extent[1]];
            z.sort_by(f64::total_cmp);
            z.dedup();
            z
        } else {
            vec![0.]
        };
        let mut nodes = vec![];
        for z in zs {
            let xyz = [p[0], p[1], z];
            let n = scene
                .xyz
                .iter()
                .position(|q| (0..3).all(|a| (q[a] - xyz[a]).abs() < tolerance))
                .unwrap_or_else(|| {
                    scene.xyz.push(xyz);
                    scene.xyz.len() - 1
                });
            while scene.plan.len() < scene.xyz.len() {
                let q = scene.xyz[scene.plan.len()];
                let closest = mesh
                    .nodes
                    .iter()
                    .enumerate()
                    .min_by(|(i, a), (j, b)| {
                        let distance = |i: usize, p: Point| {
                            (p[0] - q[0]).powi(2)
                                + (p[1] - q[1]).powi(2)
                                + (mesh.node_z(i) - q[2]).powi(2)
                        };
                        distance(*i, a.xy).total_cmp(&distance(*j, b.xy))
                    })
                    .map_or(usize::MAX, |(i, _)| i);
                scene.plan.push(closest);
            }
            nodes.push(n);
            if (layout.is_none() && !settings.enabled)
                || layout.is_none() && (z - extent[0]).abs() < 1e-6
            {
                scene.fixed.extend([3 * n, 3 * n + 1, 3 * n + 2]);
            }
            if layout.is_none() && settings.enabled && (z - extent[1]).abs() < 1e-6 {
                scene.fixed.extend([3 * n, 3 * n + 1]);
            }
        }
        scene.support_nodes.insert(c.id, nodes.clone());
        for pair in nodes.windows(2) {
            if pair[0] != pair[1] {
                scene.beams.push(shell::BeamGeometry {
                    rigid_offset_mm: [0.; 2],
                    roll_angle_rad: c.column_angle,
                    id: c.id,
                    nodes: [pair[0], pair[1]],
                    section: section.clone(),
                });
            }
        }
    }
}
#[derive(Clone)]
pub struct Rigid {
    pub master: usize,
    pub nodes: Vec<usize>,
}
pub fn rigid(
    all: &[Contour],
    settings: &shell::Settings,
    mesh: &super::meshing::Mesh,
    scene: &shell::Scene,
) -> Vec<Rigid> {
    if !settings.rigid_columns {
        return vec![];
    }
    joints(
        all,
        &settings.columns,
        mesh.building.as_ref().map(|d| &d.layout),
    )
    .iter()
    .filter_map(|j| {
        let master = scene.xyz.iter().position(|p| {
            (p[0] - j.master[0]).abs() < 1e-6
                && (p[1] - j.master[1]).abs() < 1e-6
                && (p[2] - j.z).abs() < 1e-6
        })?;
        let nodes = scene
            .xyz
            .iter()
            .enumerate()
            .filter(|(_, p)| {
                (p[2] - j.z).abs() < 1e-6
                    && p[0] >= j.bounds[0] - 1e-6
                    && p[0] <= j.bounds[2] + 1e-6
                    && p[1] >= j.bounds[1] - 1e-6
                    && p[1] <= j.bounds[3] + 1e-6
                    && contains(j, [p[0], p[1]])
            })
            .map(|(n, _)| n)
            .collect();
        Some(Rigid { master, nodes })
    })
    .collect()
}

pub(super) fn append_contour_rigid_links(
    mut groups: Vec<Rigid>,
    mesh: &super::meshing::Mesh,
    scene: &shell::Scene,
) -> Result<Vec<Rigid>, String> {
    use super::{building_mesh::WELD_TOLERANCE_MM, rigid_links::Attachment};
    use rstar::{primitives::GeomWithData, RTree, AABB};
    use std::collections::BTreeSet;
    let Some(data) = mesh.building.as_ref().filter(|d| !d.rigid_links.is_empty()) else {
        return Ok(groups);
    };
    let mut membership = BTreeSet::new();
    let wanted = data
        .rigid_links
        .iter()
        .flat_map(|s| std::iter::once(s.master.owner).chain(s.slaves.iter().map(|a| a.owner)))
        .collect::<BTreeSet<_>>();
    for surface in &scene.surfaces {
        let owner = surface
            .support
            .or_else(|| surface.display.first().map(|&i| data.element_slabs[i]));
        if let Some(owner) = owner.filter(|id| wanted.contains(id)) {
            membership.extend(surface.nodes.iter().map(|&n| (owner, n)));
        }
    }
    for beam in scene.beams.iter().filter(|b| wanted.contains(&b.id)) {
        membership.extend(beam.nodes.map(|n| (beam.id, n)));
    }
    let index = RTree::bulk_load(
        membership
            .into_iter()
            .map(|(id, n)| GeomWithData::new(scene.xyz[n], (id, n)))
            .collect(),
    );
    let mut owners = BTreeMap::new();
    let mut master_group = BTreeMap::new();
    for (i, group) in groups.iter().enumerate() {
        master_group.insert(group.master, i);
        for node in std::iter::once(group.master).chain(group.nodes.iter().copied()) {
            owners.insert(node, group.master);
        }
    }
    fn representative(parents: &mut BTreeMap<usize, usize>, mut n: usize) -> usize {
        while parents[&n] != n {
            let parent = parents[&n];
            let grandparent = parents[&parent];
            parents.insert(n, grandparent);
            n = grandparent;
        }
        n
    }
    let mut explicit_parents = BTreeMap::new();
    let mut explicit_masters = BTreeSet::new();
    let mut explicit_slaves = BTreeSet::new();
    for station in &data.rigid_links {
        let contour = station.contour;
        let resolve = |a: Attachment| {
            let radius = WELD_TOLERANCE_MM;
            let bounds = AABB::from_corners(a.xyz.map(|v| v - radius), a.xyz.map(|v| v + radius));
            let distance = |p: &[f64; 3]| {
                p.iter()
                    .zip(a.xyz)
                    .map(|(x, y)| (x - y).powi(2))
                    .sum::<f64>()
            };
            index.locate_in_envelope_intersecting(&bounds)
                .filter(|entry| entry.data.0 == a.owner)
                .min_by(|p, q| distance(p.geom()).total_cmp(&distance(q.geom())).then(p.data.1.cmp(&q.data.1)))
                .map(|entry| entry.data.1)
                .ok_or_else(|| format!(
                    "АЖТ {contour}: объект {} не содержит узел ({:.3}, {:.3}, {:.3}) мм. Проверьте сетку и пересечение.",
                    a.owner, a.xyz[0], a.xyz[1], a.xyz[2]
                ))
        };
        let master = resolve(station.master)?;
        let nodes = station
            .slaves
            .iter()
            .copied()
            .map(resolve)
            .collect::<Result<Vec<_>, _>>()?;
        for &n in std::iter::once(&master).chain(&nodes) {
            if owners.get(&n).is_some_and(|&m| m != master) {
                return Err(format!(
                    "АЖТ {contour}: узел {} уже входит в АЖТ с другим ведущим узлом",
                    n + 1
                ));
            }
            explicit_parents.entry(n).or_insert(n);
        }
        explicit_masters.insert(master);
        explicit_slaves.extend(nodes.iter().copied());
        for n in nodes {
            let a = representative(&mut explicit_parents, master);
            let b = representative(&mut explicit_parents, n);
            if a != b {
                explicit_parents.insert(a.max(b), a.min(b));
            }
        }
    }
    let mut components = BTreeMap::<usize, Vec<usize>>::new();
    let nodes = explicit_parents.keys().copied().collect::<Vec<_>>();
    for n in nodes {
        let root = representative(&mut explicit_parents, n);
        components.entry(root).or_default().push(n);
    }
    for nodes in components.into_values() {
        let mut original = nodes
            .iter()
            .filter(|n| master_group.contains_key(n))
            .copied();
        let fixed_master = original.next();
        if let Some(other) = original.next() {
            return Err(format!(
                "АЖТ объединяют два автоматических АЖТ с ведущими узлами {} и {}",
                fixed_master.unwrap() + 1,
                other + 1,
            ));
        }
        // Two explicit stations sharing a physical node form one rigid body.
        // Prefer a root of the explicit relations, then its stable mesh node ID.
        let master = fixed_master.unwrap_or_else(|| {
            nodes
                .iter()
                .copied()
                .filter(|n| explicit_masters.contains(n))
                .min_by_key(|n| (explicit_slaves.contains(n), *n))
                .expect("every explicit component contains a master")
        });
        let i = *master_group.entry(master).or_insert_with(|| {
            groups.push(Rigid {
                master,
                nodes: vec![master],
            });
            groups.len() - 1
        });
        groups[i].nodes.extend(nodes);
        groups[i].nodes.sort_unstable();
        groups[i].nodes.dedup();
    }
    Ok(groups)
}

pub fn tag_mesh(mesh: &mut super::meshing::Mesh, all: &[Contour], settings: &shell::Settings) {
    let js = joints(
        all,
        &settings.columns,
        mesh.building.as_ref().map(|d| &d.layout),
    );
    for c in all.iter().filter(|c| c.kind == Kind::Column) {
        let Some(&p) = c.points.first() else {
            continue;
        };
        let floor = mesh.contour_floor(c.id);
        let extent = mesh
            .building
            .as_ref()
            .map_or([0., 0.], |d| d.layout.extent(c));
        let z = extent[0];
        let top = extent[1];
        let nodes = mesh
            .nodes
            .iter()
            .enumerate()
            .filter(|(i, n)| {
                let at = mesh.node_z(*i);
                if at < z - 1e-6 || at > top + 1e-6 {
                    return false;
                }
                if settings.rigid_columns {
                    js.iter()
                        .find(|j| {
                            (j.z - at).abs() < 1e-6
                                && p[0] >= j.bounds[0]
                                && p[0] <= j.bounds[2]
                                && p[1] >= j.bounds[1]
                                && p[1] <= j.bounds[3]
                        })
                        .is_some_and(|j| {
                            n.xy[0] >= j.bounds[0] - 1e-6
                                && n.xy[0] <= j.bounds[2] + 1e-6
                                && n.xy[1] >= j.bounds[1] - 1e-6
                                && n.xy[1] <= j.bounds[3] + 1e-6
                                && contains(j, n.xy)
                        })
                } else {
                    (n.xy[0] - p[0]).hypot(n.xy[1] - p[1]) < 1e-6
                }
            })
            .map(|(i, _)| i)
            .collect();
        mesh.support_boundary.insert(c.id, nodes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        building_mesh::{generate, Level},
        fem,
        loads::{ForceUnits, LoadCase, LoadData, Magnitude},
        quad_mesh,
    };
    fn c(id: u64, kind: Kind, points: Vec<Point>) -> Contour {
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
            points,
            parent: None,
            curves: vec![],
            grid: None,
            support_open: false,
            refinement: None,
            load: None,
        }
    }
    #[test]
    fn eccentric_columns_choose_lower_master_and_bounding_box() {
        let all = vec![
            c(1, Kind::Column, vec![[0., 0.]]),
            c(2, Kind::Column, vec![[100., 50.]]),
        ];
        let section = Section {
            width_mm: 250.,
            material: crate::plan_trace::stiffness::column_material(),
        };
        let layout = Layout {
            legacy_wall_extent: None,
            levels: vec![
                Level {
                    id: 1,
                    z: 0.,
                    top: 3000.,
                },
                Level {
                    id: 2,
                    z: 3000.,
                    top: 6000.,
                },
            ],
            owners: BTreeMap::from([(1, 1), (2, 2)]),
        };
        let groups = joints(
            &all,
            &BTreeMap::from([(1, section.clone()), (2, section)]),
            Some(&layout),
        );
        let g = groups.iter().find(|g| g.z == 3000.).unwrap();
        assert_eq!(g.master, [0., 0.]);
        assert_eq!(g.bounds, [-125., -375., 225., 425.]);
    }
    #[test]
    fn building_columns_rigid_joints_balance_and_follow_exact_kinematics() {
        let mut all = vec![];
        let mut layout = Layout::default();
        let mut materials = BTreeMap::new();
        let mut settings = shell::Settings::default();
        settings.rigid_columns = true;
        for floor in 1..=2 {
            layout.levels.push(Level {
                id: floor,
                z: (floor - 1) as f64 * 3000.,
                top: floor as f64 * 3000.,
            });
            let id = floor * 10;
            all.push(c(
                id,
                Kind::Slab,
                vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
            ));
            layout.owners.insert(id, floor);
            materials.insert(id, fem::element::Material::default());
            for (j, p) in [[500., 500.], [1500., 500.], [1500., 1500.], [500., 1500.]]
                .into_iter()
                .enumerate()
            {
                let id = floor * 100 + j as u64;
                let p = if floor == 2 { [p[0] + 40., p[1]] } else { p };
                all.push(c(id, Kind::Column, vec![p]));
                layout.owners.insert(id, floor);
                settings.columns.insert(
                    id,
                    Section {
                        width_mm: 250.,
                        material: crate::plan_trace::stiffness::column_material(),
                    },
                );
            }
        }
        let mut load = c(
            1000,
            Kind::LoadArea,
            vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]],
        );
        load.load = Some(LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            case_id: 1,
            mesh_constraint: false,
            closed: true,
            magnitude: Magnitude {
                value: 2.,
                units: ForceUnits::KiloNewton,
            },
        });
        all.push(load);
        layout.owners.insert(1000, 2);
        let mut layout = Some(layout);
        let prepared = meshing_sources(&all, &settings, &mut layout);
        let mut mesh = generate(
            &prepared,
            500.,
            layout.unwrap(),
            quad_mesh::Options {
                backend: Default::default(),
                merge: false,
                optimise: false,
            },
        )
        .unwrap();
        tag_mesh(&mut mesh, &all, &settings);
        let request = serde_json::json!({"mesh":&mesh,"first":{"building":mesh.building.as_ref().map(|d|&d.layout),"contours":&all,"materials":&materials,"cases":[[1,false]],"g":10.,"step":500.,"element_type":"Dkt","support_height":&settings},"second":null,"cases":[LoadCase::new(1)]});
        std::fs::write(
            "target/columns_solver_request.json",
            serde_json::to_vec(&request).unwrap(),
        )
        .unwrap();
        let geometry = fem::prepare_cells(&mesh, &all, &materials, 500., 10.).unwrap();
        let scene = shell::scene(
            &mesh,
            &geometry.iter().map(|c| c.material).collect::<Vec<_>>(),
            &settings,
            500.,
            &all,
        )
        .unwrap();
        let result = fem::solve_with_support_height(
            &mesh,
            &all,
            &materials,
            &[LoadCase::new(1)],
            10.,
            500.,
            fem::ElementType::Dkt,
            &settings,
        )
        .unwrap();
        let case = &result.cases[0];
        assert!((case.applied_kn - 8.).abs() < 1e-6);
        assert!(case.balance_kn.abs() < 1e-6);
        assert!(case.relative_residual < 1e-6);
        let solved = case.shell.as_ref().unwrap();
        for group in rigid(&all, &settings, &mesh, &scene) {
            let m = group.master;
            let u = solved.translations_mm[m];
            let t = solved.rotations_rad[m];
            for n in group.nodes {
                let d = std::array::from_fn::<_, 3, _>(|a| scene.xyz[n][a] - scene.xyz[m][a]);
                let cross = [
                    t[1] * d[2] - t[2] * d[1],
                    t[2] * d[0] - t[0] * d[2],
                    t[0] * d[1] - t[1] * d[0],
                ];
                for a in 0..3 {
                    assert!((solved.translations_mm[n][a] - u[a] - cross[a]).abs() < 1e-7);
                }
            }
        }
    }
}
