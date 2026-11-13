//! Expanded LIRA-SAPR TXT. Coordinates m, forces tf, sections cm.
use super::{
    fem::{element::Material, shell::Scene},
    loads::{ForceUnits, LoadCase},
};
use std::{collections::BTreeMap, fmt::Write};
/// Positive values act downward along global Z (LIRA load convention).
/// No nodal moments: area and beam gravity remain element loads.
// Match the physical width axis, rather than copying an angle between
// different zero-angle conventions. LIRA vertical Y1 points against global Y.
fn beam_export_angle(scene: &Scene, beam: &super::fem::shell::BeamGeometry) -> Result<f64, String> {
    let [a, b] = beam.nodes.map(|n| scene.xyz[n]);
    let kernel = super::fem::beam::Element::spatial(a, b, beam.section.clone())?;
    let [x, y, z] = kernel.basis;
    let (sin, cos) = beam.roll_angle_rad.sin_cos();
    let desired = std::array::from_fn::<_, 3, _>(|i| cos * y[i] + sin * z[i]);
    let horizontal = x[0].hypot(x[1]);
    let base_y = if horizontal < 1e-10 {
        [0., -1., 0.]
    } else {
        [-x[1] / horizontal, x[0] / horizontal, 0.]
    };
    let base_z = [
        x[1] * base_y[2] - x[2] * base_y[1],
        x[2] * base_y[0] - x[0] * base_y[2],
        x[0] * base_y[1] - x[1] * base_y[0],
    ];
    let dot = |a: [f64; 3], b: [f64; 3]| (0..3).map(|i| a[i] * b[i]).sum::<f64>();
    Ok(dot(desired, base_z)
        .atan2(dot(desired, base_y))
        .to_degrees())
}

pub struct CaseLoads {
    pub nodal: Vec<f64>,
    pub surface: Vec<f64>,
    pub beam: Vec<f64>,
}
impl CaseLoads {
    /// Mechanical resultant [Fz, Mx, My], kN and kN m; for interchange checks.
    pub fn resultant(&self, scene: &Scene) -> [f64; 3] {
        let mut result = [0.; 3];
        let mut add = |p: [f64; 3], down: f64| {
            result[0] -= down;
            result[1] -= down * p[1] / 1000.;
            result[2] += down * p[0] / 1000.;
        };
        for (&value, &p) in self.nodal.iter().zip(&scene.xyz) {
            add(p, value);
        }
        for (&q, surface) in self.surface.iter().zip(&scene.surfaces) {
            let a = scene.xyz[surface.nodes[0]];
            for i in 1..surface.nodes.len() - 1 {
                let b = scene.xyz[surface.nodes[i]];
                let c = scene.xyz[surface.nodes[i + 1]];
                let u = std::array::from_fn::<_, 3, _>(|j| b[j] - a[j]);
                let v = std::array::from_fn::<_, 3, _>(|j| c[j] - a[j]);
                let cross = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                let area = cross.iter().map(|x| x * x).sum::<f64>().sqrt() / 2e6;
                add(std::array::from_fn(|j| (a[j] + b[j] + c[j]) / 3.), q * area);
            }
        }
        for (&q, beam) in self.beam.iter().zip(&scene.beams) {
            let [a, b] = beam.nodes.map(|n| scene.xyz[n]);
            let length = (0..3).map(|j| (b[j] - a[j]).powi(2)).sum::<f64>().sqrt() / 1000.;
            add(std::array::from_fn(|j| (a[j] + b[j]) / 2.), q * length);
        }
        result
    }
}
pub struct Export {
    pub text: String,
    pub nodes: usize,
    pub elements: usize,
    pub rigid_bodies: usize,
    materials: Vec<(Material, Option<f64>)>,
}
fn safe_name(s: &str) -> String {
    s.chars()
        .map(|c| {
            if c.is_control() || "();/:".contains(c) {
                ' '
            } else {
                c
            }
        })
        .take(160)
        .collect()
}
fn vector_sub(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    std::array::from_fn(|i| a[i] - b[i])
}
fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    (0..3).map(|i| a[i] * b[i]).sum()
}
fn cross(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}
fn unit(v: [f64; 3]) -> Result<[f64; 3], String> {
    let length = dot(v, v).sqrt();
    if !length.is_finite() || length < 1e-12 {
        return Err("Нельзя определить оси вырожденного КЭ".into());
    }
    Ok(v.map(|x| x / length))
}

fn wall_plan_deviation_mm(xyz: &[[f64; 3]], nodes: &[usize]) -> Result<f64, String> {
    // A vertical plane projects to a line. Use the longest plan baseline so
    // a short FE edge cannot amplify coordinate noise from node welding.
    let mut baseline = [0., 0., 0.];
    let mut origin = xyz[nodes[0]];
    let mut length_squared = 0.;
    for (i, &a) in nodes.iter().enumerate() {
        for &b in &nodes[i + 1..] {
            let delta = [xyz[b][0] - xyz[a][0], xyz[b][1] - xyz[a][1], 0.];
            let length = dot(delta, delta);
            if length > length_squared {
                baseline = delta;
                origin = xyz[a];
                length_squared = length;
            }
        }
    }
    let direction = unit(baseline)?;
    Ok(nodes
        .iter()
        .map(|&n| cross(direction, vector_sub(xyz[n], origin))[2].abs())
        .fold(0., f64::max))
}

/// Returns perimeter order plus the signed CCW angle from local X1 to agreed X1*.
fn surface_axes(
    scene: &Scene,
    surface: &super::fem::shell::Surface,
) -> Result<(Vec<usize>, f64), String> {
    let mut nodes = surface.nodes.clone();
    if !matches!(nodes.len(), 3 | 4) || nodes.iter().any(|n| *n >= scene.xyz.len()) {
        return Err("Некорректные узлы пластины".into());
    }
    let valid = match nodes.as_slice() {
        [a, b, c] => super::mesh_pack::valid_triangle([a, b, c].map(|&n| scene.xyz[n])),
        [a, b, c, d] => super::mesh_pack::valid_quad([a, b, c, d].map(|&n| scene.xyz[n])),
        _ => unreachable!(),
    };
    if !valid {
        return Err("Некорректная геометрия пластины после упаковки. Перестройте сетку".into());
    }
    let basis = |ns: &[usize]| -> Result<([f64; 3], [f64; 3]), String> {
        let a = scene.xyz[ns[0]];
        let x = unit(vector_sub(scene.xyz[ns[1]], a))?;
        let n = unit(cross(
            x,
            unit(vector_sub(scene.xyz[*ns.last().unwrap()], a))?,
        ))?;
        Ok((x, n))
    };
    let (_, normal) = basis(&nodes)?;
    if surface.support.is_none() && normal[2] < 0. {
        nodes[1..].reverse();
    }
    let (local_x, normal) = basis(&nodes)?;
    let target_x = if let Some(wall_id) = surface.support {
        let deviation = wall_plan_deviation_mm(&scene.xyz, &nodes)?;
        // Welding allows a coordinate change on each horizontal axis at each
        // end of the baseline and at the vertex being checked.
        if deviation > 4. * super::building_mesh::WELD_TOLERANCE_MM {
            return Err(format!(
                "Стена {wall_id} отклоняется от вертикальной плоскости на {deviation:.6} мм: согласование осей невозможно"
            ));
        }
        // Y* follows the projection of global up onto the actual FE plane.
        // Scene wall winding follows the source support contour (shell::scene),
        // so preserve its normal: exported stresses agree with native recovery.
        unit(cross([0., 0., 1.], normal))?
    } else {
        // The current slab model is XY. Reject accidental tilted surfaces
        // rather than silently exporting another result coordinate system.
        if normal[2] < 1. - 1e-8 {
            return Err("Плита вне плоскости XY: согласование осей невозможно".into());
        }
        [1., 0., 0.]
    };
    let angle = dot(normal, cross(local_x, target_x))
        .atan2(dot(local_x, target_x))
        .to_degrees();
    Ok((nodes, if angle.abs() < 1e-10 { 0. } else { angle }))
}
pub fn encode(
    scene: &Scene,
    loads: &[CaseLoads],
    cases: &[LoadCase],
    g: f64,
    title: &str,
) -> Result<Export, String> {
    encode_with_rigid(scene, loads, cases, g, title, &[])
}
/// Document25: one full spatial rigid body per record, leading node first.
pub fn encode_with_rigid(
    scene: &Scene,
    loads: &[CaseLoads],
    cases: &[LoadCase],
    g: f64,
    title: &str,
    rigid: &[super::columns::Rigid],
) -> Result<Export, String> {
    encode_with_rigid_and_springs(scene, loads, cases, g, title, rigid, &BTreeMap::new())
}

pub(super) fn encode_with_rigid_and_springs(
    scene: &Scene,
    loads: &[CaseLoads],
    cases: &[LoadCase],
    g: f64,
    title: &str,
    rigid: &[super::columns::Rigid],
    horizontal: &BTreeMap<usize, f64>,
) -> Result<Export, String> {
    if scene.xyz.is_empty() || scene.surfaces.is_empty() {
        return Err("Нет КЭ для экспорта".into());
    }
    if !g.is_finite() || g <= 0. || loads.len() != cases.len() {
        return Err("Некорректные данные экспорта".into());
    }
    let mut rigid_records = Vec::new();
    let mut used = std::collections::BTreeSet::new();
    for group in rigid {
        if group.master >= scene.xyz.len() || group.nodes.iter().any(|&n| n >= scene.xyz.len()) {
            return Err("АЖТ ссылается на отсутствующий узел".into());
        }
        let members = group
            .nodes
            .iter()
            .copied()
            .filter(|n| *n != group.master)
            .collect::<std::collections::BTreeSet<_>>();
        if members.is_empty() {
            continue;
        }
        for node in std::iter::once(group.master).chain(members.iter().copied()) {
            if !used.insert(node) {
                return Err("Узел включён в несколько АЖТ при экспорте".into());
            }
        }
        rigid_records.push((group.master, members));
    }
    let mut t = String::new();
    writeln!(
        t,
        "(0/1; {}/2; 5/33;M 1 CM 100 T 1 C 1/39;",
        safe_name(title)
    )
    .unwrap();
    for (i, case) in cases.iter().enumerate() {
        writeln!(t, "{}: {};", i + 1, safe_name(&case.name)).unwrap();
    }
    t.push_str("/)\n(4/\n");
    for (node_index, xyz) in scene.xyz.iter().enumerate() {
        if node_index % 4096 == 0 {
            super::fem::progress::check_cancel()?;
        }
        if xyz.iter().any(|v| !v.is_finite()) {
            return Err("Нечисловая координата узла".into());
        }
        writeln!(
            t,
            "{:.12} {:.12} {:.12}/",
            xyz[0] / 1000.,
            xyz[1] / 1000.,
            xyz[2] / 1000.
        )
        .unwrap();
    }
    t.push_str(")\n(1/\n");
    let mut materials: Vec<(Material, Option<f64>)> = vec![];
    let mut material_id = |m: Material, b: Option<f64>| -> Result<usize, String> {
        m.validate()?;
        if b.is_some_and(|v| !v.is_finite() || v <= 0.) {
            return Err("Некорректное сечение балки".into());
        }
        if let Some(i) = materials.iter().position(|x| *x == (m, b)) {
            Ok(i + 1)
        } else {
            materials.push((m, b));
            Ok(materials.len())
        }
    };
    let mut agreed_axes = Vec::with_capacity(scene.surfaces.len());
    for (index, surface) in scene.surfaces.iter().enumerate() {
        if index % 4096 == 0 {
            super::fem::progress::check_cancel()?;
        }
        let id = material_id(surface.material, None)?;
        let (perimeter, angle) =
            surface_axes(scene, surface).map_err(|e| format!("КЭ {}: {e}", index + 1))?;
        agreed_axes.push(angle);
        let nodes = match perimeter.as_slice() {
            [a, b, c] => vec![*a, *b, *c],
            [a, b, c, d] => vec![*a, *b, *d, *c], // Tensor ordering, not perimeter.
            _ => return Err("Неподдерживаемый КЭ для ЛИРА".into()),
        };
        write!(t, "{} {id}", if nodes.len() == 3 { 42 } else { 44 }).unwrap();
        for n in nodes {
            if n >= scene.xyz.len() {
                return Err("Нет узла КЭ".into());
            }
            write!(t, " {}", n + 1).unwrap();
        }
        t.push_str("/\n");
    }
    for beam in &scene.beams {
        let id = material_id(beam.section.material, Some(beam.section.width_mm))?;
        if beam.nodes.iter().any(|n| *n >= scene.xyz.len()) {
            return Err("Нет узла балки".into());
        }
        writeln!(t, "10 {id} {} {}/", beam.nodes[0] + 1, beam.nodes[1] + 1).unwrap();
    }
    let mut spring_ids = BTreeMap::new();
    let mut spring_stiffnesses = Vec::new();
    for (index, (&node, &kn_m)) in horizontal.iter().enumerate() {
        if index % 4096 == 0 {
            super::fem::progress::check_cancel()?;
        }
        let stiffness = kn_m / g;
        if node >= scene.xyz.len() || !stiffness.is_finite() || stiffness <= 0. {
            return Err(format!(
                "Некорректная пружина XY основания в узле {}",
                node + 1
            ));
        }
        let id = spring_ids.entry(stiffness.to_bits()).or_insert_with(|| {
            spring_stiffnesses.push(stiffness);
            materials.len() + spring_stiffnesses.len()
        });
        // LIRA type 56, one node and independent global translational/rotational stiffnesses.
        writeln!(t, "56 {id} {}/", node + 1).unwrap();
    }
    t.push_str(")\n(9/\n");
    for (index, angle) in agreed_axes.iter().enumerate() {
        writeln!(t, "{} {:.12}/", index + 1, angle).unwrap();
    }
    t.push_str(")\n");
    if !scene.beams.is_empty() {
        t.push_str("(17/\n");
        for index in 0..scene.beams.len() {
            writeln!(
                t,
                "{} {:.12}/",
                scene.surfaces.len() + index + 1,
                beam_export_angle(scene, &scene.beams[index])?
            )
            .unwrap();
        }
        t.push_str(")\n");
    }
    if scene.surfaces.iter().any(|s| s.rigid_z_mm != 0.)
        || scene.beams.iter().any(|b| b.rigid_offset_mm != [0.; 2])
    {
        t.push_str("(14/\n");
        for (i, plate) in scene
            .surfaces
            .iter()
            .enumerate()
            .filter(|(_, s)| s.rigid_z_mm != 0.)
        {
            if plate.support.is_some()
                || !plate.rigid_z_mm.is_finite()
                || plate.rigid_z_mm.abs() > 1e6
            {
                return Err(format!(
                    "Пластина {}: некорректная жёсткая вставка Z",
                    i + 1
                ));
            }
            // The plate winding exported in document 1 has normal +Z.
            // LIRA document 14 stores a single normal arm in metres for plates.
            writeln!(t, "{} 0 {:.12}/", i + 1, plate.rigid_z_mm / 1000.).unwrap();
        }
        for (i, beam) in scene
            .beams
            .iter()
            .enumerate()
            .filter(|(_, b)| b.rigid_offset_mm != [0.; 2])
        {
            // Global axes (3) avoid LIRA's unrolled local-axis convention.
            let [x, y, z] = beam.rigid_arm(&scene.xyz)?.map(|v| v / 1000.);
            writeln!(
                t,
                "{} 3 {x:.12} {x:.12} {y:.12} {y:.12} {z:.12} {z:.12}/",
                scene.surfaces.len() + i + 1
            )
            .unwrap();
        }
        t.push_str(")\n");
    }
    t.push_str("(3/\n");
    for (i, (m, width)) in materials.iter().enumerate() {
        let e = m.e_mpa * 1000. / g;
        let rho = m.weight.in_units(ForceUnits::TonForce, g);
        if let Some(b) = width {
            writeln!(
                t,
                "{} S0 {:.12e} {:.12} {:.12}/\n0 RO {}/\n0 Mu {:.12}/",
                i + 1,
                e,
                b / 10.,
                m.thickness_mm / 10.,
                // S0 RO is weight per metre, not volumetric weight.
                rho * b * m.thickness_mm / 1e6,
                m.nu
            )
            .unwrap();
        } else {
            writeln!(
                t,
                "{} GEI {:.12e} {:.12} {:.12} RO {}/",
                i + 1,
                e,
                m.nu,
                m.thickness_mm / 1000.,
                rho
            )
            .unwrap();
        }
    }
    for (i, k) in spring_stiffnesses.iter().enumerate() {
        writeln!(t, "{} {k} {k} 0 0 0 0/", materials.len() + i + 1).unwrap();
    }
    t.push_str(")\n(5/\n");
    let mut fixed = BTreeMap::<usize, Vec<usize>>::new();
    for &d in &scene.fixed {
        if d / 3 >= scene.xyz.len() {
            return Err("Неверный узел закрепления".into());
        }
        fixed.entry(d / 3).or_default().push(d % 3 + 1);
    }
    for &d in &scene.fixed_rotations {
        if d / 3 >= scene.xyz.len() {
            return Err("Неверный узел закрепления поворота".into());
        }
        fixed.entry(d / 3).or_default().push(d % 3 + 4);
    }
    for (node, axes) in fixed {
        write!(t, "{}", node + 1).unwrap();
        for axis in axes {
            write!(t, " {axis}").unwrap();
        }
        t.push_str("/\n");
    }
    t.push_str(")\n");
    if !rigid_records.is_empty() {
        t.push_str("(25/\n");
        for (master, members) in &rigid_records {
            write!(t, "{}", master + 1).unwrap();
            for node in members {
                write!(t, " {}", node + 1).unwrap();
            }
            t.push_str("/\n");
        }
        t.push_str(")\n");
    }
    let (mut doc6, mut doc7) = (String::new(), String::new());
    let mut value_id = 0;
    for (case, load) in loads.iter().enumerate() {
        super::fem::progress::check_cancel()?;
        if load.nodal.len() != scene.xyz.len()
            || load.surface.len() != scene.surfaces.len()
            || load.beam.len() != scene.beams.len()
        {
            return Err("Нагрузки относятся к другой сетке".into());
        }
        // Type 0: a nodal vertical force; type 16: uniform element load.
        // LIRA positive load is opposite the positive displacement axis:
        // positive Z load acts DOWN. Do not serialize mechanical Fz directly.
        let records = load
            .nodal
            .iter()
            .enumerate()
            .map(|(i, &v)| (i + 1, 0, v))
            .chain(
                load.surface
                    .iter()
                    .enumerate()
                    .map(|(i, &v)| (i + 1, 16, v)),
            )
            .chain(
                load.beam
                    .iter()
                    .enumerate()
                    .map(|(i, &v)| (scene.surfaces.len() + i + 1, 16, v)),
            );
        for (object, kind, value) in records {
            if !value.is_finite() {
                return Err("Нечисловая нагрузка".into());
            }
            if value.abs() < 1e-14 {
                continue;
            }
            value_id += 1;
            writeln!(doc6, "{object} {kind} 3 {value_id} {}/", case + 1).unwrap();
            if kind == 0 {
                writeln!(doc7, "{value_id} {:.12e} 0/", value / g).unwrap();
            } else {
                writeln!(doc7, "{value_id} {:.12e} 0 0 0/", value / g).unwrap();
            }
        }
    }
    if value_id > 0 {
        write!(t, "(6/\n{doc6})\n(7/\n{doc7})\n").unwrap();
    }
    Ok(Export {
        text: t,
        nodes: scene.xyz.len(),
        elements: scene.surfaces.len() + scene.beams.len() + horizontal.len(),
        rigid_bodies: rigid_records.len(),
        materials,
    })
}

/// Native LIRA sidecar: TXT itself does not preserve stiffness comments.
pub fn material_metadata(
    export: &Export,
    library: &super::stiffness::Library,
    contours: &[super::model::Contour],
    title: &str,
    g: f64,
) -> String {
    fn xml(s: &str) -> String {
        s.replace('&', "&amp;")
            .replace('<', "&lt;")
            .replace('>', "&gt;")
            .replace('"', "&quot;")
            .replace('\r', "&#13;")
            .replace('\n', "&#10;")
    }
    let mut keys = std::collections::BTreeSet::new();
    let used = contours
        .iter()
        .filter(|c| {
            library
                .assignments
                .get(&c.id)
                .is_some_and(|id| keys.insert((*id, library.material_id(c.id))))
        })
        .filter_map(|c| library.resolved_entry(c.id))
        .collect::<Vec<_>>();
    let mut output = format!("<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"yes\"?>\n<LIRA_Project Title=\"{}\" Code=\"{}\" Description=\"\">\n  <RigidArray>\n", xml(title), xml(title));
    for (i, (material, width)) in export.materials.iter().enumerate() {
        let names = used
            .iter()
            .filter(|e| {
                let w = matches!(
                    e.family,
                    super::stiffness::Family::Beam | super::stiffness::Family::Column
                )
                .then_some(e.width_mm);
                e.material == *material && w == *width
            })
            .map(|e| e.name.as_str())
            .collect::<std::collections::BTreeSet<_>>()
            .into_iter()
            .collect::<Vec<_>>()
            .join(" | ");
        // ALD Ro is volumetric even for S0 (TXT S0 RO is per metre).
        let rho = material.weight.in_units(ForceUnits::TonForce, g);
        writeln!(
            output,
            "    <Rigid Num=\"{}\" Ro=\"{}\" Comment=\"{}\" />",
            i + 1,
            rho,
            xml(&names)
        )
        .unwrap();
    }
    output.push_str("  </RigidArray>\n</LIRA_Project>\n");
    output
}

/// Group actual exported FE IDs by source object; a Q4 contributes one record.
pub fn append_object_blocks(
    metadata: &mut String,
    scene: &Scene,
    mesh: &super::meshing::Mesh,
    contours: &[super::model::Contour],
    step: f64,
) -> Result<usize, String> {
    use super::model::Kind;
    let boundaries = if mesh.building.is_none() {
        contours
            .iter()
            .filter(|c| matches!(c.kind, Kind::Slab | Kind::Hole))
            .map(|c| {
                let mut copy = c.clone();
                copy.points = c.mesh_boundary(step)?;
                Ok(copy)
            })
            .collect::<Result<Vec<_>, String>>()?
    } else {
        vec![]
    };
    let mut groups = BTreeMap::<u64, Vec<usize>>::new();
    for (i, surface) in scene.surfaces.iter().enumerate() {
        let owner = if let Some(id) = surface.support {
            Some(id)
        } else if let Some(building) = &mesh.building {
            let mut ids = surface
                .display
                .iter()
                .filter_map(|&n| building.element_slabs.get(n))
                .copied();
            let first = ids.next();
            if ids.any(|id| Some(id) != first) {
                return Err(format!(
                    "КЭ {} относится к нескольким плитам при группировке",
                    i + 1
                ));
            }
            first
        } else {
            let center = [0, 1].map(|a| {
                surface.nodes.iter().map(|&n| scene.xyz[n][a]).sum::<f64>()
                    / surface.nodes.len().max(1) as f64
            });
            boundaries
                .iter()
                .filter(|c| {
                    c.kind == Kind::Slab
                        && super::geometry::inside(center, &c.points)
                        && !boundaries.iter().any(|h| {
                            h.kind == Kind::Hole
                                && h.parent == Some(c.id)
                                && super::geometry::inside(center, &h.points)
                        })
                })
                .min_by(|a, b| {
                    let area = |c: &super::model::Contour| {
                        c.points
                            .iter()
                            .zip(c.points.iter().cycle().skip(1))
                            .take(c.points.len())
                            .map(|(a, b)| a[0] * b[1] - b[0] * a[1])
                            .sum::<f64>()
                            .abs()
                    };
                    area(a).total_cmp(&area(b)).then_with(|| b.id.cmp(&a.id))
                })
                .map(|c| c.id)
        }
        .ok_or_else(|| format!("Не найдена исходная плита для блока КЭ {}", i + 1))?;
        groups.entry(owner).or_default().push(i + 1);
    }
    for (i, beam) in scene.beams.iter().enumerate() {
        groups
            .entry(beam.id)
            .or_default()
            .push(scene.surfaces.len() + i + 1);
    }
    let closing = metadata
        .rfind("</LIRA_Project>")
        .ok_or("Некорректные метаданные ALD")?;
    let mut records = String::from("  <ElemBlockArray>\n");
    for (block, elements) in groups.values().enumerate() {
        for id in elements {
            writeln!(
                records,
                "    <ElemBlock NumFE=\"{}\" NumBlk=\"{}\" />",
                id,
                block + 1
            )
            .unwrap();
        }
    }
    records.push_str("  </ElemBlockArray>\n");
    metadata.insert_str(closing, &records);
    Ok(groups.len())
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn agreed_plate_axes_match_global_xy_for_rotated_and_reversed_elements() {
        for degrees in [0_f64, 37., 137., -83.] {
            let (s, c) = degrees.to_radians().sin_cos();
            let points = vec![[0., 0., 0.], [c, s, 0.], [c - s, s + c, 0.], [-s, c, 0.]];
            for nodes in [
                vec![0, 1, 3],
                vec![0, 3, 1],
                vec![0, 1, 2, 3],
                vec![0, 3, 2, 1],
                vec![2, 3, 0, 1],
            ] {
                let scene = Scene {
                    xyz: points.clone(),
                    plan: vec![],
                    surfaces: vec![],
                    fixed: Default::default(),
                    support_nodes: Default::default(),
                    fixed_rotations: Default::default(),
                    beams: vec![],
                };
                let surface = super::super::fem::shell::Surface {
                    rigid_z_mm: 0.,
                    nodes,
                    material: Material::default(),
                    display: vec![],
                    support: None,
                };
                let (nodes, angle) = surface_axes(&scene, &surface).unwrap();
                let local_x = unit(vector_sub(points[nodes[1]], points[nodes[0]])).unwrap();
                let normal = unit(cross(
                    local_x,
                    vector_sub(points[*nodes.last().unwrap()], points[nodes[0]]),
                ))
                .unwrap();
                let local_y = cross(normal, local_x);
                let (s, c) = angle.to_radians().sin_cos();
                let x = std::array::from_fn::<_, 3, _>(|i| c * local_x[i] + s * local_y[i]);
                let y = cross(normal, x);
                assert!(dot(x, [1., 0., 0.]) > 1. - 1e-12);
                assert!(dot(y, [0., 1., 0.]) > 1. - 1e-12);
                assert!(normal[2] > 1. - 1e-12);
            }
        }
    }
    #[test]
    fn agreed_wall_y_is_up_even_when_first_edge_is_vertical() {
        let points = vec![[0., 0., -1.], [3., 4., -1.], [3., 4., 1.], [0., 0., 1.]];
        let scene = Scene {
            xyz: points.clone(),
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        for nodes in [vec![0, 1, 2, 3], vec![1, 2, 3, 0], vec![0, 3, 2, 1]] {
            let surface = super::super::fem::shell::Surface {
                rigid_z_mm: 0.,
                nodes,
                material: Material::default(),
                display: vec![],
                support: Some(1),
            };
            let (nodes, angle) = surface_axes(&scene, &surface).unwrap();
            let x = unit(vector_sub(points[nodes[1]], points[nodes[0]])).unwrap();
            let n = unit(cross(
                x,
                vector_sub(points[*nodes.last().unwrap()], points[nodes[0]]),
            ))
            .unwrap();
            let y = cross(n, x);
            let (s, c) = angle.to_radians().sin_cos();
            let agreed_x = std::array::from_fn::<_, 3, _>(|i| c * x[i] + s * y[i]);
            assert!(dot(cross(n, agreed_x), [0., 0., 1.]) > 1. - 1e-12);
        }
    }
    #[test]
    fn wall_axes_accept_weld_noise_but_report_real_tilt() {
        let mut scene = Scene {
            xyz: vec![
                [0., 0., 0.],
                [500., 0., 0.],
                [500., 0., 40.],
                [0., 0.000_008, 40.],
            ],
            plan: vec![],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let surface = super::super::fem::shell::Surface {
            rigid_z_mm: 0.,
            nodes: vec![0, 3, 2, 1],
            material: Material::default(),
            display: vec![],
            support: Some(123),
        };
        let (nodes, angle) = surface_axes(&scene, &surface).unwrap();
        assert_eq!(nodes, surface.nodes);
        assert!((angle + 90.).abs() < 1e-9);

        scene.xyz[2][1] = 2.;
        scene.xyz[3][1] = 2.;
        assert!(surface_axes(&scene, &surface)
            .unwrap_err()
            .contains("Стена 123"));
    }

    #[test]
    fn exported_self_weight_of_slab_wall_and_beam_preserves_force_and_moment() {
        use crate::plan_trace::{
            fem,
            loads::{LoadType, Magnitude},
            meshing,
            model::{Contour, Kind},
        };
        let contour = |id, kind, points| Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: matches!(kind, Kind::Support | Kind::Beam),
            load: None,
            refinement: None,
            id,
            parent: None,
            kind,
            points,
            curves: vec![],
        };
        let all = vec![
            contour(
                1,
                Kind::Slab,
                vec![[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]],
            ),
            contour(2, Kind::Support, vec![[0., 0.], [1000., 0.]]),
            contour(3, Kind::Beam, vec![[0., 0.], [1000., 0.]]),
        ];
        let mesh = meshing::generate(&all, 500.).unwrap();
        let mut settings = fem::shell::Settings::default();
        settings.enabled = true;
        settings.height_mm = 1000.;
        let mut wall = Material::default();
        wall.weight = Magnitude {
            value: 1.,
            units: ForceUnits::TonForce,
        };
        settings.materials.insert(2, wall);
        let mut beam = wall;
        beam.thickness_mm = 500.;
        settings.beams.insert(
            3,
            fem::beam::Section {
                material: beam,
                width_mm: 200.,
            },
        );
        let mut case = LoadCase::new(1);
        case.kind = LoadType::SelfWeight;
        let (scene, loads) = fem::lira_model(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[case.clone()],
            10.,
            500.,
            &settings,
        )
        .unwrap();
        let [fz, mx, my] = loads[0].resultant(&scene);
        assert!((fz + 8.5).abs() < 1e-9);
        assert!((mx + 2.75).abs() < 1e-9, "Mx={mx}");
        assert!((my - 4.25).abs() < 1e-9, "My={my}");
        assert!(loads[0].nodal.iter().all(|v| *v == 0.));
        assert!(loads[0].beam.iter().all(|q| (*q - 1.).abs() < 1e-12));
        let export = encode(&scene, &loads, &[case], 10., "SW").unwrap();
        // Keep RO in the compact notation emitted by LIRA itself.
        assert!(export.text.contains(" RO 2.75/"));
        assert!(export.text.contains("0 RO 0.1/"));
        let mut library = super::super::stiffness::Library::default();
        let slab_id = library.add(super::super::stiffness::Family::Slab, Material::default());
        library.entries.get_mut(&slab_id).unwrap().name = "Плита & \"LIR 31\"".into();
        library.assignments.insert(1, slab_id);
        let beam_id = library.add(super::super::stiffness::Family::Beam, beam);
        library.entries.get_mut(&beam_id).unwrap().name = "Балка тест".into();
        library.entries.get_mut(&beam_id).unwrap().width_mm = 200.;
        library.assignments.insert(3, beam_id);
        let mut metadata = material_metadata(&export, &library, &all, "Test", 10.);
        assert_eq!(
            append_object_blocks(&mut metadata, &scene, &mesh, &all, 500.).unwrap(),
            3
        );
        assert_eq!(
            metadata.matches("<ElemBlock NumFE=").count(),
            export.elements
        );
        for (i, surface) in scene.surfaces.iter().enumerate() {
            let block = if surface.support == Some(2) { 2 } else { 1 };
            assert!(metadata.contains(&format!("NumFE=\"{}\" NumBlk=\"{}\"", i + 1, block)));
        }
        for i in 0..scene.beams.len() {
            assert!(metadata.contains(&format!(
                "NumFE=\"{}\" NumBlk=\"3\"",
                scene.surfaces.len() + i + 1
            )));
        }

        assert!(
            metadata.contains("Num=\"1\" Ro=\"2.75\" Comment=\"Плита &amp; &quot;LIR 31&quot;\"")
        );
        assert!(metadata.contains("Ro=\"1\" Comment=\"Балка тест\""));
        assert!(!metadata.contains("Comment=\"Плита 1\""));
        let doc6 = export
            .text
            .split("(6/\n")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        let records = doc6
            .split('/')
            .filter(|s| !s.trim().is_empty())
            .map(|s| s.split_whitespace().collect::<Vec<_>>())
            .collect::<Vec<_>>();
        assert!(records.iter().all(|r| r[1] == "16" && r[2] == "3"));
        assert!(records
            .iter()
            .any(|r| r[0].parse::<usize>().unwrap() > scene.surfaces.len()));
        let doc7 = export
            .text
            .split("(7/\n")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        assert!(doc7.split('/').filter(|s| !s.trim().is_empty()).all(|s| s
            .split_whitespace()
            .nth(1)
            .unwrap()
            .parse::<f64>()
            .unwrap()
            > 0.));
        let parsed = crate::plan_trace::lira_txt_reader::parse(&export.text).unwrap();
        assert!(parsed.elements.iter().any(|e| e.fe_type == 10));
        assert!(parsed.elements.iter().any(|e| e.fe_type == 44));
        assert_eq!(parsed.nodes.len(), scene.xyz.len());
    }
    #[test]
    fn physical_quad_roundtrips_without_duplicate_display_triangles() {
        let scene = Scene {
            xyz: vec![
                [0., 0., 0.],
                [1000., 0., 0.],
                [1000., 1000., 0.],
                [0., 1000., 0.],
            ],
            plan: vec![0, 1, 2, 3],
            surfaces: vec![super::super::fem::shell::Surface {
                rigid_z_mm: 0.,
                nodes: vec![0, 1, 2, 3],
                material: Material::default(),
                display: vec![0, 1],
                support: None,
            }],
            fixed: [0, 1, 2].into_iter().collect(),
            support_nodes: BTreeMap::new(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        let cases = vec![LoadCase::new(1)];
        let loads = vec![CaseLoads {
            nodal: vec![0., 0., 10., 0.],
            surface: vec![0.],
            beam: vec![],
        }];
        let exported = encode(&scene, &loads, &cases, 10., "Slab").unwrap();
        let parsed = crate::plan_trace::lira_txt_reader::parse(&exported.text).unwrap();
        assert_eq!(parsed.nodes.len(), 4);
        assert_eq!(parsed.nodes[2].xyz, [1., 1., 0.]);
        assert_eq!(parsed.elements.len(), 1);
        assert_eq!(parsed.elements[0].fe_type, 44);
        assert_eq!(parsed.elements[0].nodes, vec![1, 2, 4, 3]);
        assert!(exported.text.contains("1 1 2 3/"));
        assert!(exported.text.contains("3 0 3 1 1/"));
        assert!(exported.text.contains("1 1.000000000000e0 0/"));
        assert_eq!(exported.elements, 1);
        let upward = vec![CaseLoads {
            nodal: vec![0., 0., -10., 0.],
            surface: vec![0.],
            beam: vec![],
        }];
        let reversed = encode(&scene, &upward, &cases, 10., "Upward").unwrap();
        assert!(reversed.text.contains("1 -1.000000000000e0 0/"));
    }
}

#[cfg(test)]
mod rigid_export_tests {
    use super::*;
    #[test]
    fn document14_transverse_offsets_are_paired_and_follow_rolled_axes() {
        let (mut scene, _) = example();
        scene.xyz[0] = [0., 0., 0.];
        scene.xyz[1] = [3000., 0., 0.];
        scene.beams.push(super::super::fem::shell::BeamGeometry {
            id: 99,
            nodes: [0, 1],
            section: super::super::fem::beam::Section {
                material: Material::default(),
                width_mm: 200.,
            },
            rigid_offset_mm: [0., -300.],
            roll_angle_rad: std::f64::consts::FRAC_PI_2,
        });
        let out = encode(&scene, &[], &[], 10., "offset").unwrap();
        let doc = out
            .text
            .split("(14/\n")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        let fields = doc
            .trim()
            .trim_end_matches('/')
            .split_whitespace()
            .map(|v| v.parse::<f64>().unwrap())
            .collect::<Vec<_>>();
        assert_eq!(fields.len(), 8);
        assert_eq!(fields[0], scene.surfaces.len() as f64 + 1.);
        assert_eq!(fields[1], 3.);
        for i in [4, 5] {
            assert!((fields[i] - 0.3).abs() < 1e-12);
        } // rotated -Z is global +Y
        for i in [2, 3, 6, 7] {
            assert!(fields[i].abs() < 1e-12);
        }
    }
    #[test]
    fn document14_keeps_signed_plate_offsets_and_bar_records_in_one_document() {
        let (mut scene, _) = example();
        scene.surfaces[0].nodes[1..].reverse();
        scene.surfaces[0].rigid_z_mm = -111.;
        scene.surfaces.push(scene.surfaces[0].clone());
        scene.surfaces[1].rigid_z_mm = 0.;
        scene.surfaces.push(scene.surfaces[0].clone());
        scene.surfaces[2].rigid_z_mm = 30.;
        scene.beams.push(super::super::fem::shell::BeamGeometry {
            id: 100,
            nodes: [0, 1],
            section: super::super::fem::beam::Section {
                material: Material::default(),
                width_mm: 200.,
            },
            rigid_offset_mm: [0., -300.],
            roll_angle_rad: 0.,
        });
        let out = encode(&scene, &[], &[], 10., "offset").unwrap();
        assert_eq!(out.text.matches("(14/").count(), 1);
        let doc = out
            .text
            .split("(14/\n")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        let rows: Vec<Vec<f64>> = doc
            .lines()
            .map(|line| {
                line.trim_end_matches('/')
                    .split_whitespace()
                    .map(|v| v.parse().unwrap())
                    .collect()
            })
            .collect();
        assert_eq!(rows.len(), 3);
        assert_eq!(rows[0], [1., 0., -0.111]);
        assert_eq!(rows[1], [3., 0., 0.030]);
        assert_eq!(rows[2].len(), 8);
        assert_eq!(&rows[2][..2], &[4., 3.]);
    }
    fn example() -> (Scene, Vec<super::super::columns::Rigid>) {
        let text = include_str!("../../tests/fixtures/lira_rigid_body_example.txt");
        let doc4 = text
            .split("( 4/")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        let xyz = doc4
            .split('/')
            .filter_map(|row| {
                let values = row
                    .split_whitespace()
                    .map(|v| v.parse::<f64>().unwrap() * 1000.)
                    .collect::<Vec<_>>();
                (!values.is_empty()).then(|| values.try_into().unwrap())
            })
            .collect();
        let doc25 = text
            .split("( 25/")
            .nth(1)
            .unwrap()
            .split(')')
            .next()
            .unwrap();
        let groups = doc25
            .split('/')
            .filter_map(|row| {
                let ids = row
                    .split_whitespace()
                    .map(|n| n.parse::<usize>().unwrap() - 1)
                    .collect::<Vec<_>>();
                ids.first().map(|&master| super::super::columns::Rigid {
                    master,
                    nodes: ids.clone(),
                })
            })
            .collect();
        let scene = Scene {
            xyz,
            plan: vec![],
            surfaces: vec![super::super::fem::shell::Surface {
                rigid_z_mm: 0.,
                nodes: vec![0, 1, 11],
                material: Material::default(),
                display: vec![],
                support: None,
            }],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![],
        };
        (scene, groups)
    }
    #[test]
    fn document25_matches_user_example_and_keeps_coincident_member() {
        let (scene, groups) = example();
        assert_eq!(groups[0].master, 60);
        assert_eq!(scene.xyz[60], scene.xyz[121]);
        let exported = encode_with_rigid(&scene, &[], &[], 10., "example", &groups).unwrap();
        assert_eq!(exported.rigid_bodies, 1);
        assert!(exported
            .text
            .contains("(25/\n61 49 50 51 60 62 71 72 73 122/\n)"));
        let plain = encode(&scene, &[], &[], 10., "example").unwrap();
        assert!(!plain.text.contains("(25/"));
    }
    #[test]
    fn document25_rejects_missing_nodes_and_overlapping_bodies() {
        let (scene, groups) = example();
        let mut bad = groups.clone();
        bad[0].nodes.push(scene.xyz.len());
        assert!(encode_with_rigid(&scene, &[], &[], 10., "bad", &bad).is_err());
        let duplicate = vec![groups[0].clone(), groups[0].clone()];
        assert!(encode_with_rigid(&scene, &[], &[], 10., "bad", &duplicate).is_err());
    }
}

#[cfg(test)]
mod column_export_regression {
    use super::*;
    #[test]
    fn vertical_column_section_axes_and_linear_weight_match_footprint() {
        let section = super::super::fem::beam::Section {
            material: Material {
                thickness_mm: 750.,
                weight: super::super::loads::Magnitude {
                    value: 2.75,
                    units: ForceUnits::TonForce,
                },
                ..Material::default()
            },
            width_mm: 250.,
        };
        let mut scene = Scene {
            xyz: vec![[0., 0., 0.], [0., 0., 3000.]],
            plan: vec![0, 1],
            surfaces: vec![],
            fixed: Default::default(),
            support_nodes: Default::default(),
            fixed_rotations: Default::default(),
            beams: vec![super::super::fem::shell::BeamGeometry {
                id: 1,
                nodes: [0, 1],
                section,
                rigid_offset_mm: [0.; 2],
                roll_angle_rad: 0.,
            }],
        };
        for angle in [0_f64, 0.37, std::f64::consts::FRAC_PI_2] {
            scene.beams[0].roll_angle_rad = angle;
            let phi = beam_export_angle(&scene, &scene.beams[0])
                .unwrap()
                .to_radians();
            // LIRA default vertical axes Y=-globalY, Z=globalX.
            let width_axis = [phi.sin(), -phi.cos()];
            assert!((width_axis[0] - angle.cos()).abs() < 1e-12);
            assert!((width_axis[1] - angle.sin()).abs() < 1e-12);
        }
        scene.xyz.extend([[1000., 0., 0.], [0., 1000., 0.]]);
        scene.plan.extend([2, 3]);
        scene.surfaces.push(super::super::fem::shell::Surface {
            rigid_z_mm: 0.,
            nodes: vec![0, 2, 3],
            material: Material::default(),
            display: vec![0],
            support: None,
        });
        let exported = encode(&scene, &[], &[], 10., "Column").unwrap();
        assert!(exported.text.split("RO ").skip(1).any(|record| {
            record
                .split('/')
                .next()
                .unwrap()
                .trim()
                .parse::<f64>()
                .is_ok_and(|rho| (rho - 0.515625).abs() < 1e-12)
        }));
    }
}

/// Document 19 verified against the user's two-zone C1 TXT (400 records).
/// Only vertical Winkler C1 is exported; the remaining coefficients are zero.
pub fn append_foundation(
    export: &mut Export,
    scene: &Scene,
    c1: &[f64],
    g: f64,
) -> Result<(), String> {
    if !g.is_finite() || g <= 0. {
        return Err("Некорректное g для экспорта основания".into());
    }
    let mut body = String::new();
    for (i, s) in scene.surfaces.iter().enumerate() {
        if s.support.is_some() {
            continue;
        }
        let Some(&display) = s.display.first() else {
            continue;
        };
        let Some(&value) = c1.get(display) else {
            return Err("Несогласованное назначение C1 при экспорте".into());
        };
        if value.is_nan() {
            continue;
        }
        if !value.is_finite() || value < 0. {
            return Err("Некорректный C1 при экспорте".into());
        }
        writeln!(body, "{} {:.12e} 0 0 0 0/", i + 1, value / g).unwrap();
    }
    if !body.is_empty() {
        write!(export.text, "(19/\n{body})\n").unwrap();
    }
    Ok(())
}
