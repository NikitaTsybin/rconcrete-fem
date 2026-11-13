//! Mixed Reissner–Mindlin MITC3 triangles and twelve-DOF MITC4 quadrilaterals.
//! Triangles belonging to a quadrilateral are display/search tessellation only:
//! they NEVER contribute stiffness, load quadrature, or extra DOFs.
use super::mitc4::Quad4;
use super::*;

enum Physical<'a> {
    Tri {
        cell: &'a Cell,
        element: super::mitc3::Tri3,
        display: usize,
    },
    Quad {
        element: Quad4,
        material: Material,
        nodes: [usize; 4],
        display: [usize; 2],
    },
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{
        loads::{LoadCase, LoadData, Magnitude},
        meshing::{Element, Node, Quad},
    };
    fn fixture(alternate: bool) -> (Mesh, Vec<Contour>, BTreeMap<u64, Material>) {
        let nodes = (0..3)
            .flat_map(|row| {
                (0..3).map(move |col| Node {
                    id: (row * 3 + col + 1) as u64,
                    xy: [col as f64 * 1000., row as f64 * 1000.],
                })
            })
            .collect::<Vec<_>>();
        let mut elements = vec![];
        let mut quads = vec![];
        let mut edges = BTreeSet::new();
        for row in 0..2 {
            for col in 0..2 {
                let n = [
                    row * 3 + col,
                    row * 3 + col + 1,
                    (row + 1) * 3 + col + 1,
                    (row + 1) * 3 + col,
                ];
                let tessellation = if alternate {
                    [[n[0], n[1], n[3]], [n[1], n[2], n[3]]]
                } else {
                    [[n[0], n[1], n[2]], [n[0], n[2], n[3]]]
                };
                let id = elements.len() as u64 + 1;
                for nodes in tessellation {
                    for i in 0..3 {
                        let a = nodes[i];
                        let b = nodes[(i + 1) % 3];
                        edges.insert([a.min(b), a.max(b)]);
                    }
                    elements.push(Element {
                        id: elements.len() as u64 + 1,
                        nodes,
                        supports: vec![],
                    });
                }
                let diagonal = if alternate {
                    [n[1], n[3]]
                } else {
                    [n[0], n[2]]
                };
                quads.push(Quad {
                    id,
                    nodes: n,
                    display_triangles: [id, id + 1],
                    diagonal: [diagonal[0].min(diagonal[1]), diagonal[0].max(diagonal[1])],
                });
            }
        }
        let fixed = (0..9).filter(|n| *n != 4).collect::<Vec<_>>();
        let mesh = Mesh {
            building: None,
            nodes,
            elements,
            edges: edges.into_iter().collect(),
            quads,
            constrained_nodes: fixed.iter().copied().collect(),
            constrained_edges: BTreeSet::new(),
            support_boundary: BTreeMap::from([(2, fixed)]),
        };
        let outline = vec![[0., 0.], [2000., 0.], [2000., 2000.], [0., 2000.]];
        let slab = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 1,
            kind: Kind::Slab,
            parent: None,
            points: outline.clone(),
            curves: vec![],
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
        };
        let support = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 2,
            kind: Kind::Support,
            ..slab.clone()
        };
        let load = Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            id: 3,
            kind: Kind::LoadArea,
            load: Some(LoadData {
                direction: Default::default(),
                moment: false,
                additive: false,
                name: String::new(),
                mesh_constraint: false,
                case_id: 1,
                magnitude: Magnitude {
                    value: 10.,
                    units: ForceUnits::KiloNewton,
                },
                closed: true,
            }),
            ..slab.clone()
        };
        (
            mesh,
            vec![slab, support, load],
            BTreeMap::from([(1, Material::default())]),
        )
    }

    #[test]
    fn beam_line_export_removes_forest_only_for_complete_member_intervals() {
        let (mesh, mut all, materials) = fixture(false);
        let mut beam = all[1].clone();
        beam.id = 4;
        beam.kind = Kind::Beam;
        beam.support_open = true;
        beam.points = vec![[0., 1000.], [2000., 1000.]];
        all.push(beam);
        let mut line = all[2].clone();
        line.id = 5;
        line.kind = Kind::LoadLine;
        line.points = vec![[0., 1000.], [2000., 1000.]];
        line.load.as_mut().unwrap().closed = false;
        all.retain(|c| c.kind != Kind::LoadArea);
        all.push(line.clone());
        let settings = super::shell::Settings {
            beams: BTreeMap::from([(
                4,
                super::beam::Section {
                    material: crate::plan_trace::stiffness::beam_material(),
                    width_mm: 200.,
                },
            )]),
            ..Default::default()
        };
        let (scene, forces) = super::lira_model(
            &mesh,
            &all,
            &materials,
            &[LoadCase::new(1)],
            10.,
            1000.,
            &settings,
        )
        .unwrap();
        assert_eq!(scene.beams.len(), 2);
        assert!(forces[0].beam.iter().all(|q| (*q - 10.).abs() < 1e-10));
        assert!(forces[0].nodal.iter().all(|v| v.abs() < 1e-10));
        assert!((forces[0].resultant(&scene)[0] + 20.).abs() < 1e-8);
        line.points = vec![[250., 1000.], [750., 1000.]];
        let (remaining, q) = beam_line_export(&[line], &scene, 10.);
        assert!(q.is_empty());
        assert_eq!(remaining.len(), 1);
    }
    #[test]
    fn native_beam_slab_coupling_self_weight_and_combination_balance() {
        let (mesh, mut all, materials) = fixture(false);
        let mut beam = all[1].clone();
        beam.id = 4;
        beam.kind = Kind::Beam;
        beam.support_open = true;
        beam.points = vec![[0., 1000.], [2000., 1000.]];
        all.push(beam);
        let mut own = LoadCase::new(2);
        own.kind = crate::plan_trace::loads::LoadType::SelfWeight;
        let cases = [LoadCase::new(1), own];
        let section = super::super::beam::Section {
            material: crate::plan_trace::stiffness::beam_material(),
            width_mm: 200.,
        };
        let settings = super::super::shell::Settings {
            columns: Default::default(),
            rigid_columns: false,
            beams: BTreeMap::from([(4, section)]),
            ..Default::default()
        };
        let result = super::super::solve_with_support_height(
            &mesh,
            &all,
            &materials,
            &cases,
            10.,
            1000.,
            ElementType::Dkt,
            &settings,
        )
        .unwrap();
        assert_eq!(result.cases.len(), 2);
        for c in &result.cases {
            assert!(c.balance_kn.abs() < 1e-7);
            assert!(c.relative_residual < 1e-7);
            assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-7));
            let shell = c.shell.as_ref().unwrap();
            assert_eq!(shell.beam_forces.len(), 2);
            assert!(shell
                .beam_forces
                .iter()
                .flat_map(|b| b.ends.iter().flatten())
                .all(|v| v.is_finite()));
            assert!(shell
                .beam_forces
                .iter()
                .any(|b| b.ends.iter().any(|v| v[2].abs() > 1e-6)));
        }
        assert!((result.cases[1].applied_kn - 27.5).abs() < 1e-8);
        let combination =
            crate::plan_trace::load_combinations::combine(&result, 99, &[(1, 1.2), (2, -0.3)])
                .unwrap();
        let c = combination.shell.as_ref().unwrap();
        assert_eq!(
            c.surface_forces.len(),
            result.cases[0].shell.as_ref().unwrap().surface_forces.len()
        );
        for (index, force) in c.surface_forces.iter().enumerate() {
            let a = &result.cases[0].shell.as_ref().unwrap().surface_forces[index];
            let b = &result.cases[1].shell.as_ref().unwrap().surface_forces[index];
            for j in 0..3 {
                assert!(
                    (force.membrane[j] - (1.2 * a.membrane[j] - 0.3 * b.membrane[j])).abs() < 1e-8
                );
                assert!(
                    (force.moments[j] - (1.2 * a.moments[j] - 0.3 * b.moments[j])).abs() < 1e-8
                );
            }
        }
        for i in 0..2 {
            for end in 0..2 {
                for component in 0..6 {
                    let expected = 1.2
                        * result.cases[0].shell.as_ref().unwrap().beam_forces[i].ends[end]
                            [component]
                        - 0.3
                            * result.cases[1].shell.as_ref().unwrap().beam_forces[i].ends[end]
                                [component];
                    assert!((c.beam_forces[i].ends[end][component] - expected).abs() < 1e-9);
                }
            }
        }
    }
    #[test]
    fn genuine_quads_are_independent_of_display_diagonals_for_all_load_types() {
        let (a, mut all, materials) = fixture(false);
        let (b, _, _) = fixture(true);
        let cases = [LoadCase::new(1)];
        for kind in [Kind::LoadArea, Kind::LoadPoint, Kind::LoadLine] {
            all[2].kind = kind;
            all[2].points = match kind {
                Kind::LoadArea => vec![[100., 100.], [200., 100.], [200., 200.], [100., 200.]],
                Kind::LoadPoint => vec![[200., 400.]],
                _ => vec![[200., 100.], [200., 900.]],
            };
            all[2].load.as_mut().unwrap().closed = kind == Kind::LoadArea;
            let x = solve_load_cases(&a, &all, &materials, &cases, 10., 1000., ElementType::Dkt)
                .unwrap();
            let y = solve_load_cases(&b, &all, &materials, &cases, 10., 1000., ElementType::Dkt)
                .unwrap();
            let x = &x.cases[0];
            let y = &y.cases[0];
            let force = match kind {
                Kind::LoadArea => 10.,
                Kind::LoadPoint => 10.,
                _ => 8.,
            };
            assert!((x.applied_kn - force).abs() < 1e-9);
            assert!(x.balance_kn.abs() < 1e-7);
            assert!(x.balance_m_knm.iter().all(|v| v.abs() < 1e-7));
            assert!(x.w_mm[4] > 0.);
            for (u, v) in x.w_mm.iter().zip(&y.w_mm) {
                assert!((u - v).abs() < 1e-10);
            }
            assert!((x.applied_kn - y.applied_kn).abs() < 1e-10);
        }
        let incompatible = solve_load_cases(
            &a,
            &all,
            &materials,
            &cases,
            10.,
            1000.,
            ElementType::MindlinCr,
        );
        assert!(incompatible.is_err());
    }
    #[test]
    fn finite_support_height_uses_shared_nodes_and_balances_all_directions_and_triangle_types() {
        let (mesh, all, materials) = fixture(false);
        let mut mesh = mesh;
        mesh.constrained_edges = mesh
            .edges
            .iter()
            .copied()
            .filter(|e| {
                let p = e.map(|i| mesh.nodes[i].xy);
                (p[0][0] == p[1][0] && (p[0][0] == 0. || p[0][0] == 2000.))
                    || (p[0][1] == p[1][1] && (p[0][1] == 0. || p[0][1] == 2000.))
            })
            .collect();
        // Leave one genuine triangle pair among quads to exercise the interface.
        mesh.quads.pop();
        let cases = [LoadCase::new(1)];
        for direction in [
            super::super::shell::Direction::Up,
            super::super::shell::Direction::Down,
            super::super::shell::Direction::Both,
        ] {
            let settings = super::super::shell::Settings {
                columns: Default::default(),
                rigid_columns: false,
                enabled: true,
                beams: BTreeMap::new(),
                direction,
                height_mm: 2000.,
                materials: BTreeMap::from([(2, Material::default())]),
                ..Default::default()
            };
            for kind in [ElementType::Dkt] {
                let result = super::super::solve_with_support_height(
                    &mesh, &all, &materials, &cases, 10., 1000., kind, &settings,
                )
                .unwrap();
                let c = &result.cases[0];
                assert!((c.applied_kn - 40.).abs() < 1e-7);
                assert!(c.balance_kn.abs() < 1e-6);
                assert!(c.balance_m_knm.iter().all(|v| v.abs() < 1e-6));
                assert!(c.relative_residual < 1e-6);
                let shell = c.shell.as_ref().unwrap();
                assert_eq!(shell.membrane.len(), mesh.elements.len());
                assert!(shell
                    .translations_mm
                    .iter()
                    .flatten()
                    .all(|v| v.is_finite()));
                assert!(
                    (shell
                        .support_reactions_kn
                        .values()
                        .map(|r| r[2])
                        .sum::<f64>()
                        - 40.)
                        .abs()
                        < 1e-6
                );
            }
        }
    }
    #[test]
    fn wall_zero_weight_nonzero_weight_and_spatial_superposition_preserve_balance() {
        let (mut mesh, all, materials) = fixture(false);
        mesh.constrained_edges = mesh
            .edges
            .iter()
            .copied()
            .filter(|e| {
                let p = e.map(|i| mesh.nodes[i].xy);
                (p[0][0] == p[1][0] && (p[0][0] == 0. || p[0][0] == 2000.))
                    || (p[0][1] == p[1][1] && (p[0][1] == 0. || p[0][1] == 2000.))
            })
            .collect();
        let mut own = LoadCase::new(2);
        own.kind = crate::plan_trace::loads::LoadType::SelfWeight;
        let cases = [LoadCase::new(1), own];
        for (wall, expected) in [(Material::wall_default(), 22.), (Material::default(), 110.)] {
            let settings = super::super::shell::Settings {
                columns: Default::default(),
                rigid_columns: false,
                enabled: true,
                beams: BTreeMap::new(),
                direction: super::super::shell::Direction::Down,
                height_mm: 2000.,
                materials: BTreeMap::from([(2, wall)]),
                ..Default::default()
            };
            let r = super::super::solve_with_support_height(
                &mesh,
                &all,
                &materials,
                &cases,
                10.,
                1000.,
                ElementType::Dkt,
                &settings,
            )
            .unwrap();
            assert!((r.cases[1].applied_kn - expected).abs() < 1e-8);
            assert!(r.cases[1].balance_kn.abs() < 1e-6);
            let c = crate::plan_trace::load_combinations::combine(&r, 1, &[(1, 1.2), (2, -0.5)])
                .unwrap();
            assert!((c.applied_kn - (48. - 0.5 * expected)).abs() < 1e-8);
            assert!(c.balance_kn.abs() < 1e-6);
            for (i, p) in c.shell.as_ref().unwrap().translations_mm.iter().enumerate() {
                for a in 0..3 {
                    let value = 1.2 * r.cases[0].shell.as_ref().unwrap().translations_mm[i][a]
                        - 0.5 * r.cases[1].shell.as_ref().unwrap().translations_mm[i][a];
                    assert!((p[a] - value).abs() < 1e-10);
                }
            }
        }
    }
}
impl Physical<'_> {
    fn nodes(&self) -> &[usize] {
        match self {
            Self::Tri { cell, .. } => &cell.nodes,
            Self::Quad { nodes, .. } => nodes,
        }
    }
    fn display(&self) -> &[usize] {
        match self {
            Self::Tri { display, .. } => std::slice::from_ref(display),
            Self::Quad { display, .. } => display,
        }
    }
    fn material(&self) -> Material {
        match self {
            Self::Tri { cell, .. } => cell.material,
            Self::Quad { material, .. } => *material,
        }
    }
    fn dofs(&self) -> Vec<usize> {
        self.nodes()
            .iter()
            .flat_map(|&n| [3 * n, 3 * n + 1, 3 * n + 2])
            .collect()
    }
    fn area(&self) -> f64 {
        match self {
            Self::Tri { cell, .. } => cell.triangle.area,
            Self::Quad { element, .. } => element.area,
        }
    }
    fn polygon(&self) -> Polygon<f64> {
        match self {
            Self::Tri { cell, .. } => polygon(&cell.triangle.xy),
            Self::Quad { element, .. } => polygon(&element.xy),
        }
    }
    fn stiffness(&self) -> Vec<Vec<f64>> {
        match self {
            Self::Tri { cell, element, .. } => element
                .stiffness(cell.material)
                .into_iter()
                .map(|r| r.to_vec())
                .collect(),
            Self::Quad {
                element, material, ..
            } => element
                .stiffness(*material)
                .into_iter()
                .map(|r| r.to_vec())
                .collect(),
        }
    }
    fn point_weights(&self, p: Point) -> Option<Vec<f64>> {
        match self {
            Self::Tri { cell, .. } => {
                let l = cell.triangle.barycentric(p);
                l.iter()
                    .all(|v| (-1e-8..=1. + 1e-8).contains(v))
                    .then(|| l.to_vec())
            }
            Self::Quad { element, .. } => element
                .natural(p)
                .map(|p| super::mitc4::shape(p).0.to_vec()),
        }
    }
    fn pressure(&self, q: f64) -> Vec<f64> {
        match self {
            Self::Tri { cell, .. } => {
                let mut f = vec![0.; 9];
                for i in 0..3 {
                    f[3 * i] = q * cell.triangle.area / 3.;
                }
                f
            }
            Self::Quad { element, .. } => element.pressure_load(q).to_vec(),
        }
    }
    fn recover_at(&self, q: &[f64], p: Point) -> Option<([f64; 3], [f64; 2])> {
        let d = self.dofs();
        let p = p.map(|v| v / 1000.);
        match self {
            Self::Tri { cell, element, .. } => Some(element.recover_at(
                cell.material,
                std::array::from_fn(|i| q[d[i]]),
                cell.triangle.barycentric(p),
            )),
            Self::Quad {
                element, material, ..
            } => Some(element.recover(
                *material,
                std::array::from_fn(|i| q[d[i]]),
                element.natural(p)?,
            )),
        }
    }
    fn recover(&self, q: &[f64]) -> ([f64; 3], [f64; 2]) {
        let d = self.dofs();
        match self {
            Self::Tri { cell, element, .. } => {
                element.recover(cell.material, std::array::from_fn(|i| q[d[i]]))
            }
            Self::Quad {
                element, material, ..
            } => element.recover(*material, std::array::from_fn(|i| q[d[i]]), [0., 0.]),
        }
    }
}
fn physical_cells<'a>(
    mesh: &Mesh,
    cells: &'a [Cell],
    g: f64,
) -> Result<(Vec<Physical<'a>>, Vec<usize>), String> {
    let ids = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let mut owner = vec![usize::MAX; cells.len()];
    let mut physical = Vec::new();
    for quad in &mesh.quads {
        let indices = quad.display_triangles.map(|id| ids.get(&id).copied());
        let [Some(a), Some(b)] = indices else {
            return Err(format!(
                "MITC4 {}: отсутствует геометрическая тесселяция",
                quad.id
            ));
        };
        if a == b || owner[a] != usize::MAX || owner[b] != usize::MAX {
            return Err("Повторная принадлежность геометрии четырёхугольникам".into());
        }
        if quad.nodes.iter().any(|n| *n >= mesh.nodes.len()) {
            return Err("MITC4: неизвестный узел".into());
        }
        let material = cells[a].material;
        let other = cells[b].material;
        if material.e_mpa != other.e_mpa
            || material.nu != other.nu
            || material.thickness_mm != other.thickness_mm
            || (material.self_weight_kn_m2(g) - other.self_weight_kn_m2(g)).abs() > 1e-10
        {
            return Err(format!("MITC4 {} пересекает границу материалов", quad.id));
        }
        let xy = quad.nodes.map(|n| mesh.nodes[n].xy.map(|v| v / 1000.));
        let element = Quad4::new(xy).map_err(|e| {
            format!(
                "{e}. КЭ {}, узлы {:?}",
                quad.id,
                quad.nodes.map(|n| mesh.nodes[n].id)
            )
        })?;
        let display_area = cells[a].triangle.area + cells[b].triangle.area;
        if (element.area - display_area).abs() > display_area.max(1.) * 1e-9 {
            return Err(format!(
                "MITC4 {}: некорректная геометрическая топология",
                quad.id
            ));
        }
        owner[a] = physical.len();
        owner[b] = physical.len();
        physical.push(Physical::Quad {
            element,
            material,
            nodes: quad.nodes,
            display: [a, b],
        });
    }
    for (i, cell) in cells.iter().enumerate() {
        if owner[i] == usize::MAX {
            owner[i] = physical.len();
            physical.push(Physical::Tri {
                cell,
                element: super::mitc3::Tri3::new(cell.triangle.xy)?,
                display: i,
            });
        }
    }
    Ok((physical, owner))
}
fn candidates(index: &Index, owner: &[usize], bounds: [f64; 4]) -> BTreeSet<usize> {
    index
        .query(bounds, owner.len())
        .into_iter()
        .map(|i| owner[i])
        .collect()
}
fn point(
    cells: &[Physical<'_>],
    index: &Index,
    owner: &[usize],
    p: Point,
    value: f64,
    f: &mut [f64],
) -> Result<(), String> {
    point_optional(cells, index, owner, p, value, f).ok_or_else(|| {
        format!(
            "Нагрузка вне плиты / в отверстии: X={:.1}, Y={:.1} мм",
            p[0] * 1000.,
            p[1] * 1000.
        )
    })
}
fn point_optional(
    cells: &[Physical<'_>],
    index: &Index,
    owner: &[usize],
    p: Point,
    value: f64,
    f: &mut [f64],
) -> Option<()> {
    for i in candidates(
        index,
        owner,
        [p[0] - 1e-9, p[1] - 1e-9, p[0] + 1e-9, p[1] + 1e-9],
    ) {
        if let Some(n) = cells[i].point_weights(p) {
            super::cut_forces::active(i);
            for (&node, weight) in cells[i].nodes().iter().zip(n) {
                f[3 * node] += value * weight;
                super::cut_forces::load(3 * node, value * weight);
            }
            return Some(());
        }
    }
    None
}

fn loads(
    cells: &[Physical<'_>],
    index: &Index,
    owner: &[usize],
    all: &[Contour],
    case: u64,
    g: f64,
    step: f64,
    n: usize,
) -> Result<(Vec<f64>, Vec<f64>), String> {
    let mut f = vec![0.; n];
    let mut pressure = vec![0.; cells.len()];
    let mut area_pressure = vec![None; cells.len()];
    for load in all
        .iter()
        .filter(|c| c.load.as_ref().is_some_and(|l| l.case_id == case))
    {
        let value = load
            .load
            .as_ref()
            .unwrap()
            .magnitude
            .in_units(ForceUnits::KiloNewton, g);
        if !value.is_finite() {
            return Err("Некорректная нагрузка".into());
        }
        if value == 0. && load.kind != Kind::LoadArea {
            continue;
        }
        let points = load
            .points
            .iter()
            .map(|p| p.map(|x| x / 1000.))
            .collect::<Vec<_>>();
        match load.kind {
            Kind::LoadPoint => point(
                cells,
                index,
                owner,
                *points.first().ok_or("Пустая нагрузка")?,
                value,
                &mut f,
            )?,
            Kind::LoadArea => {
                let area = effective_load_area(all, load);
                for i in candidates(index, owner, bbox(&points)) {
                    let polygon = cells[i].polygon();
                    if !area.0.iter().any(|a| polygon_area_load_hits(&polygon, a)) {
                        continue;
                    }
                    // Any positive intersection activates FULL q on the physical Q4/T3.
                    let delta = area_pressure_delta_for(
                        &mut area_pressure[i],
                        value,
                        load.load.as_ref().unwrap().additive,
                    );
                    pressure[i] += delta;
                    super::cut_forces::active(i);
                    for (d, v) in cells[i].dofs().into_iter().zip(cells[i].pressure(delta)) {
                        f[d] += v;
                        super::cut_forces::load(d, v);
                    }
                }
            }
            Kind::LoadLine => {
                let segments = points.windows(2).map(|p| [p[0], p[1]]).chain(
                    if load.closed() && points.len() > 1 {
                        Some([*points.last().unwrap(), points[0]])
                    } else {
                        None
                    },
                );
                for [a, b] in segments {
                    let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                    let count = (length / step).ceil().max(1.) as usize;
                    if count > 1_000_000 {
                        return Err(format!("Линейная нагрузка {}: длина сегмента {:.3} м, шаг {:.6} м, требуется {count} точек (лимит 1000000). Проверьте координаты и единицы",load.id,length,step));
                    }
                    for k in 0..count {
                        if k % 4096 == 0 {
                            crate::plan_trace::fem::progress::check_cancel()?;
                        }
                        let t = (k as f64 + 0.5) / count as f64;
                        let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
                        let _ = point_optional(
                            cells,
                            index,
                            owner,
                            p,
                            value * length / count as f64,
                            &mut f,
                        );
                    }
                }
            }
            _ => {}
        }
    }
    Ok((f, pressure))
}
pub(super) fn solve(
    mesh: &Mesh,
    geometry: &[Cell],
    all: &[Contour],
    cases: &[u64],
    self_weight: &BTreeSet<u64>,
    g: f64,
    size: f64,
) -> Result<Results, String> {
    let (cells, owner) = physical_cells(mesh, geometry, g)?;
    let ndof = mesh.nodes.len() * 3;
    let fixed = mesh
        .support_boundary
        .values()
        .flatten()
        .copied()
        .collect::<BTreeSet<_>>();
    let springs = foundation::build(mesh, all, g, false, false)?;
    if springs.is_empty() {
        check_supports(mesh, &fixed)?;
    }
    let mut map = vec![usize::MAX; ndof];
    let mut free = 0;
    for i in 0..ndof {
        if i % 3 != 0 || !fixed.contains(&(i / 3)) {
            map[i] = free;
            free += 1;
        }
    }
    let mut diagonal = vec![0.; free];
    for cell in &cells {
        let k = cell.stiffness();
        let dofs = cell.dofs();
        for i in 0..dofs.len() {
            let d = map[dofs[i]];
            if d != usize::MAX {
                diagonal[d] += k[i][i];
            }
        }
    }
    foundation::flat_diagonal(&springs, &map, &mut diagonal);
    if diagonal.iter().any(|v| !v.is_finite() || *v <= 0.) {
        return Err("MITC4/DKT: некорректная жёсткость или свободный узел".into());
    }
    let scale = diagonal.iter().map(|v| 1. / v.sqrt()).collect::<Vec<_>>();
    crate::plan_trace::fem::progress::stage("Сборка матрицы жёсткости элементов");
    crate::plan_trace::fem::progress::check_cancel()?;
    let mut triplets = Vec::with_capacity(cells.len() * 78);
    for cell in &cells {
        let k = cell.stiffness();
        let dofs = cell.dofs();
        for i in 0..dofs.len() {
            let a = map[dofs[i]];
            if a == usize::MAX {
                continue;
            }
            for j in 0..dofs.len() {
                let b = map[dofs[j]];
                if b != usize::MAX && a <= b {
                    triplets.push(Triplet::new(a, b, k[i][j] * scale[a] * scale[b]));
                }
            }
        }
    }
    crate::plan_trace::fem::progress::stage("Сборка разреженной матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    foundation::flat_triplets(&springs, &map, &scale, &mut triplets);
    let matrix = SparseColMat::<usize, f64>::try_new_from_triplets(free, free, &triplets)
        .map_err(|e| format!("Сборка MITC4/DKT: {e:?}"))?;
    drop(triplets);
    crate::plan_trace::fem::progress::stage("Разложение матрицы жёсткости");
    crate::plan_trace::fem::progress::check_cancel()?;
    let factor = matrix
        .sp_cholesky(Side::Upper)
        .map_err(|e| format!("MITC4/DKT: проверьте закрепления и связность: {e:?}"))?;
    let index = Index::new(geometry, size / 1000.);
    let step = super::line_load_step_m(all, size);
    let results = super::parallel_cases::map(
        cases,
        free.max(mesh.nodes.len() * 3).max(cells.len() * 12),
        |&id| {
            crate::plan_trace::fem::progress::stage(&format!(
                "Сборка вектора нагрузок: загружение {id}"
            ));
            crate::plan_trace::fem::progress::check_cancel()?;
            super::cut_forces::begin(all, cells.len());
            let (mut f, mut pressure) = loads(&cells, &index, &owner, all, id, g, step, ndof)?;
            if self_weight.contains(&id) {
                for (i, cell) in cells.iter().enumerate() {
                    let q = cell.material().self_weight_kn_m2(g);
                    pressure[i] += q;
                    super::cut_forces::active(i);
                    for (d, v) in cell.dofs().into_iter().zip(cell.pressure(q)) {
                        f[d] += v;
                        super::cut_forces::load(d, v);
                    }
                }
            }
            #[cfg(test)]
            super::cut_forces::audit_rhs(&f);
            let mut rhs = Mat::<f64>::zeros(free, 1);
            for i in 0..ndof {
                if map[i] != usize::MAX {
                    rhs[(map[i], 0)] = f[i] * scale[map[i]];
                }
            }
            crate::plan_trace::fem::progress::stage("Вычисление неизвестных");
            crate::plan_trace::fem::progress::check_cancel()?;
            let x = factor.solve(rhs.as_ref());
            crate::plan_trace::fem::progress::stage("Восстановление перемещений, усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut q = vec![0.; ndof];
            for i in 0..ndof {
                if map[i] != usize::MAX {
                    q[i] = x[(map[i], 0)] * scale[map[i]];
                }
            }
            if q.iter().any(|v| !v.is_finite()) {
                return Err("MITC4/DKT: нечисловое решение".into());
            }
            let mut cuts = super::cut_forces::Collector::new(all);
            crate::plan_trace::fem::progress::stage("Восстановление усилий и реакций");
            crate::plan_trace::fem::progress::check_cancel()?;
            let mut residual = f.iter().map(|v| -v).collect::<Vec<_>>();
            let mut moments = vec![[0.; 3]; geometry.len()];
            let mut shears = vec![[0.; 2]; geometry.len()];
            let mut loads_kn_m2 = vec![0.; geometry.len()];
            for (cell_i, cell) in cells.iter().enumerate() {
                let k = cell.stiffness();
                let d = cell.dofs();
                for i in 0..d.len() {
                    for j in 0..d.len() {
                        residual[d[i]] += k[i][j] * q[d[j]];
                    }
                }
                if cuts.enabled() {
                    let forces: Vec<_> = (0..d.len())
                        .map(|i| {
                            super::cut_forces::net(
                                cell_i,
                                d[i],
                                (0..d.len()).map(|j| k[i][j] * q[d[j]]).sum(),
                            )
                        })
                        .collect();
                    let poly: Vec<_> = cell.nodes().iter().map(|&n| mesh.nodes[n].xy).collect();
                    cuts.add(
                        &poly,
                        cell.nodes().iter().enumerate().map(|(i, &n)| {
                            (
                                mesh.nodes[n].xy,
                                super::cut_forces::plate_node_action([
                                    forces[3 * i],
                                    forces[3 * i + 1],
                                    forces[3 * i + 2],
                                ]),
                            )
                        }),
                    );
                }
                let (m, s) = cell.recover(&q);
                if cuts.enabled() {
                    cuts.field(m, s);
                    cuts.direct_fields(|p| cell.recover_at(&q, p), false);
                }
                for &i in cell.display() {
                    moments[i] = m;
                    shears[i] = s;
                    loads_kn_m2[i] = pressure[cell_i];
                }
            }
            let ground = foundation::force(&springs, &q);
            for (r, f) in residual.iter_mut().zip(&ground) {
                *r += f;
            }
            let error = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (residual[i] * scale[map[i]]).powi(2))
                .sum::<f64>()
                .sqrt();
            let norm = (0..ndof)
                .filter(|&i| map[i] != usize::MAX)
                .map(|i| (f[i] * scale[map[i]]).powi(2))
                .sum::<f64>()
                .sqrt();
            let relative_residual = error / norm.max(1e-20);
            if relative_residual > 1e-6 && error > 1e-10 {
                return Err(format!("MITC4/DKT: невязка {relative_residual:.3e}"));
            }
            let applied_kn = f.iter().step_by(3).sum::<f64>();
            let reaction_kn = (0..mesh.nodes.len())
                .map(|n| {
                    if fixed.contains(&n) {
                        residual[3 * n]
                    } else {
                        -ground[3 * n]
                    }
                })
                .collect::<Vec<_>>();
            let balance_m_knm = std::array::from_fn(|a| {
                mesh.nodes
                    .iter()
                    .enumerate()
                    .map(|(i, n)| (f[3 * i] + reaction_kn[i]) * n.xy[a] / 1000.)
                    .sum()
            });
            Ok(CaseResult {
                punching: cuts.finish(),
                shell: None,
                id,
                w_mm: q.iter().step_by(3).map(|v| v * 1000.).collect(),
                rotations_rad: q.chunks_exact(3).map(|v| [-v[2], v[1], 0.]).collect(),
                moments,
                shears,
                loads_kn_m2,
                foundation_pressure_kn_m2: foundation::pressures(&springs, &q, mesh.elements.len()),
                applied_kn,
                balance_kn: applied_kn + reaction_kn.iter().sum::<f64>(),
                balance_m_knm,
                reaction_kn,
                relative_residual,
            })
        },
    )?;
    Ok(Results {
        nonlinear: None,
        lengths: None,
        modal: None,
        buckling: None,
        cases: results,
        free_dofs: free,
    })
}

// Keep pressures as element loads; only points and line samples form nodal forces.
/// Scalar area mosaic only: same physical Q4/T3 activation and signed maximum,
/// without constructing a shell scene or integrating a nodal force vector.
pub(super) fn source_area_projection_cases(
    mesh: &Mesh,
    geometry: &[Cell],
    all: &[Contour],
    cases: &[super::super::loads::LoadCase],
    g: f64,
    size: f64,
    mut consume: impl FnMut(&super::super::loads::LoadCase, Vec<f64>),
) -> Result<(), String> {
    let (cells, owner) = physical_cells(mesh, geometry, g)?;
    let index = Index::new(geometry, size / 1000.);
    for case in cases {
        let mut pressure = vec![None; cells.len()];
        let mut added = vec![0.; cells.len()];
        for load in all.iter().filter(|c| {
            c.kind == Kind::LoadArea && c.load.as_ref().is_some_and(|l| l.case_id == case.id)
        }) {
            let value = load
                .load
                .as_ref()
                .unwrap()
                .magnitude
                .in_units(ForceUnits::KiloNewton, g);
            if !value.is_finite() {
                return Err("Некорректная нагрузка".into());
            }
            let points = load
                .points
                .iter()
                .map(|p| p.map(|x| x / 1000.))
                .collect::<Vec<_>>();
            let area = effective_load_area(all, load);
            for i in candidates(&index, &owner, bbox(&points)) {
                if !mesh.load_matches_element(load, cells[i].display()[0]) {
                    continue;
                }
                let polygon = cells[i].polygon();
                if area.0.iter().any(|a| polygon_area_load_hits(&polygon, a)) {
                    if load.load.as_ref().unwrap().additive {
                        added[i] += value;
                    } else {
                        area_pressure_delta(&mut pressure[i], value);
                    }
                }
            }
        }
        let mut result = vec![0.; mesh.elements.len()];
        for ((cell, q), extra) in cells.iter().zip(pressure).zip(added) {
            let weight = if case.kind == super::super::loads::LoadType::SelfWeight {
                cell.material().self_weight_kn_m2(g)
            } else {
                0.
            };
            for &i in cell.display() {
                result[i] = q.unwrap_or(0.) + extra + weight;
            }
        }
        consume(case, result);
    }
    Ok(())
}

pub(super) fn export_loads(
    mesh: &Mesh,
    geometry: &[Cell],
    all: &[Contour],
    cases: &[super::super::loads::LoadCase],
    g: f64,
    size: f64,
    scene: &super::shell::Scene,
) -> Result<Vec<super::super::lira_export::CaseLoads>, String> {
    let (cells, owner) = physical_cells(mesh, geometry, g)?;
    let index = Index::new(geometry, size / 1000.);
    let step = super::line_load_step_m(all, size);
    let area_only = all
        .iter()
        .filter(|c| !matches!(c.kind, Kind::LoadPoint | Kind::LoadLine))
        .cloned()
        .collect::<Vec<_>>();
    let (discrete_source, beam_loads) = beam_line_export(all, scene, g);
    let discrete_only = discrete_source
        .iter()
        .filter(|c| c.kind != Kind::LoadArea)
        .cloned()
        .collect::<Vec<_>>();
    let mut out = vec![];
    for case in cases {
        crate::plan_trace::fem::progress::stage(&format!("Сборка вектора нагрузок: {}", case.name));
        crate::plan_trace::fem::progress::check_cancel()?;
        let (f, _) = loads(
            &cells,
            &index,
            &owner,
            &discrete_only,
            case.id,
            g,
            step,
            mesh.nodes.len() * 3,
        )?;
        let (_, mut pressure) = loads(
            &cells,
            &index,
            &owner,
            &area_only,
            case.id,
            g,
            step,
            mesh.nodes.len() * 3,
        )?;
        if case.kind == super::super::loads::LoadType::SelfWeight {
            for (q, cell) in pressure.iter_mut().zip(&cells) {
                *q += cell.material().self_weight_kn_m2(g);
            }
        }
        out.push(super::super::lira_export::CaseLoads {
            nodal: (0..mesh.nodes.len()).map(|i| f[3 * i]).collect(),
            surface: pressure,
            beam: beam_loads
                .get(&case.id)
                .cloned()
                .unwrap_or_else(|| vec![0.; scene.beams.len()]),
        });
    }
    Ok(out)
}

/// Export only complete physical beam intervals as uniform member loads.
/// Partial overlaps retain the existing forest-of-forces projection.
fn beam_line_export(
    all: &[Contour],
    scene: &super::shell::Scene,
    g: f64,
) -> (Vec<Contour>, std::collections::BTreeMap<u64, Vec<f64>>) {
    let mut output = vec![];
    let mut forces = std::collections::BTreeMap::new();
    for c in all {
        if c.kind != Kind::LoadLine || c.load.is_none() {
            output.push(c.clone());
            continue;
        }
        let data = c.load.as_ref().unwrap();
        let q = data.magnitude.in_units(ForceUnits::KiloNewton, g);
        let mut segments = c
            .points
            .windows(2)
            .map(|p| [p[0], p[1]])
            .collect::<Vec<_>>();
        if c.closed() && c.points.len() > 1 {
            segments.push([*c.points.last().unwrap(), c.points[0]]);
        }
        for [a, b] in segments {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l2 = d[0] * d[0] + d[1] * d[1];
            if l2 <= 1e-16 {
                continue;
            }
            let mut spans = vec![];
            for (i, beam) in scene.beams.iter().enumerate() {
                let p = beam.nodes.map(|n| scene.xyz[n]);
                if p.iter().any(|p| p[2].abs() > 1e-5) {
                    continue;
                }
                let t = p.map(|p| ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l2);
                let distance =
                    p.map(|p| ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / l2.sqrt());
                let lo = t[0].min(t[1]);
                let hi = t[0].max(t[1]);
                if distance.iter().all(|v| *v <= 1e-4)
                    && lo >= -1e-9
                    && hi <= 1. + 1e-9
                    && hi - lo > 1e-12
                {
                    spans.push([lo.max(0.), hi.min(1.)]);
                    forces
                        .entry(data.case_id)
                        .or_insert_with(|| vec![0.; scene.beams.len()])[i] += q;
                }
            }
            spans.sort_by(|a, b| a[0].total_cmp(&b[0]));
            let mut cursor = 0.;
            let mut gaps = vec![];
            for [lo, hi] in spans {
                if lo > cursor + 1e-10 {
                    gaps.push([cursor, lo]);
                }
                cursor = cursor.max(hi);
            }
            if cursor < 1. - 1e-10 {
                gaps.push([cursor, 1.]);
            }
            for [lo, hi] in gaps {
                let mut part = c.clone();
                part.points = vec![
                    [a[0] + lo * d[0], a[1] + lo * d[1]],
                    [a[0] + hi * d[0], a[1] + hi * d[1]],
                ];
                part.curves.clear();
                part.load.as_mut().unwrap().closed = false;
                output.push(part);
            }
        }
    }
    (output, forces)
}
