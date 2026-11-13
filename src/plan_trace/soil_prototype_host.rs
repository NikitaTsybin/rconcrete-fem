//! Research command-line host for the real building solver; never modifies the input.
use super::*;
use mini_fem::soil::{Borehole, Cell, Engine, Options};
use std::collections::BTreeSet;
pub(super) fn run(args: &[std::ffi::OsString]) -> Result<(), String> {
    let source = PathBuf::from(args.get(2).ok_or("Project path")?);
    let soil = PathBuf::from(args.get(3).ok_or("Decoded SLD JSON path")?);
    let output = PathBuf::from(args.get(4).ok_or("Output folder")?);
    std::fs::create_dir_all(&output).map_err(|e| e.to_string())?;
    let mut app = App::default();
    app.load_project_path(source.clone())?;
    app.sync_stiffness_sources();
    // Explicit prototype settings: 1m structural mesh, 2m ground pressure cells.
    app.mesh_size = args
        .get(5)
        .and_then(|s| s.to_str())
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(1000.)
        .clamp(300., 2000.);
    app.support_height.automatic_building_bottom = true;
    app.support_height.building_bottom_mask = 3;
    app.support_height.automatic_building_top = false;
    let bottom = app
        .building
        .floors
        .iter()
        .min_by(|a, b| a.elevation_mm.total_cmp(&b.elevation_mm))
        .ok_or("No floors")?
        .id;
    let slab = app
        .contours
        .iter()
        .find(|c| c.kind == Kind::Slab && app.building.owner(c.id) == bottom)
        .ok_or("No base slab")?
        .clone();
    let lo: [f64; 2] = std::array::from_fn(|a| {
        slab.points
            .iter()
            .map(|p| p[a])
            .fold(f64::INFINITY, f64::min)
    });
    let hi: [f64; 2] = std::array::from_fn(|a| {
        slab.points
            .iter()
            .map(|p| p[a])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let holes = soil_decode::read_ground(&soil)?;
    // Building local XY aligned to soil coordinates (no georeference supplied).
    // Original building coords retained: soil boreholes transformed into its local frame.
    let shift = [10. - (lo[0] + hi[0]) / 2000., -(lo[1] + hi[1]) / 2000.];
    let mut holes = holes;
    for h in &mut holes {
        h.xy[0] -= shift[0];
        h.xy[1] -= shift[1];
    }
    let mut options = Options {
        base_z: 144.5,
        max_depth: 16.,
        dz: 0.25,
        tolerance: 0.02,
        max_iterations: 80,
        relaxation: 0.8,
        ..Default::default()
    };
    if let Some(config) = args.get(8) {
        options = serde_json::from_slice(&std::fs::read(config).map_err(|e| e.to_string())?)
            .map_err(|e| e.to_string())?;
    }
    let step = args
        .get(6)
        .and_then(|s| s.to_str())
        .and_then(|s| s.parse().ok())
        .unwrap_or(2000.);
    let warm = args
        .get(7)
        .filter(|s| s.to_string_lossy() != "-")
        .map(|path| -> Result<serde_json::Value, String> {
            serde_json::from_slice(&std::fs::read(path).map_err(|e| e.to_string())?)
                .map_err(|e| e.to_string())
        })
        .transpose()?;
    app.soil_options.combination = 1;
    app.soil_options.variant = crate::plan_trace::load_combinations::Variant::NormativeLong;
    let mut report = calculate(
        &mut app,
        holes,
        options,
        step,
        Start::Prototype { slab, warm },
        |s| {
            eprintln!("{s}");
            Ok(())
        },
    )?;
    report["source"] = serde_json::json!(source);
    report["ground_shift_m"] = serde_json::json!(shift);
    app.write_soil_project(&output.join("building_on_soil.slab.zip"))?;
    std::fs::write(
        output.join("report.json"),
        serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())
}

pub(super) enum Start<'a> {
    ExistingMesh {
        mesh: &'a super::super::meshing::Mesh,
        step: f64,
        foundation: u64,
    },
    Prototype {
        slab: Contour,
        warm: Option<serde_json::Value>,
    },
}

fn regular_cells(footprint: &Contour, step: f64) -> Result<Vec<Cell>, String> {
    let lo: [f64; 2] = std::array::from_fn(|a| {
        footprint
            .points
            .iter()
            .map(|p| p[a])
            .fold(f64::INFINITY, f64::min)
    });
    let hi: [f64; 2] = std::array::from_fn(|a| {
        footprint
            .points
            .iter()
            .map(|p| p[a])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    if !step.is_finite()
        || step < 100.
        || (hi[0] - lo[0]) / step * ((hi[1] - lo[1]) / step) > 10000.
    {
        return Err("Увеличьте шаг сетки основания".into());
    }
    let mut cells = vec![];
    let nx = ((hi[0] - lo[0]) / step).ceil() as usize;
    let ny = ((hi[1] - lo[1]) / step).ceil() as usize;
    for y in 0..ny {
        for x in 0..nx {
            let b = [
                lo[0] + x as f64 * step,
                lo[1] + y as f64 * step,
                (lo[0] + (x + 1) as f64 * step).min(hi[0]),
                (lo[1] + (y + 1) as f64 * step).min(hi[1]),
            ];
            let center = [(b[0] + b[2]) / 2., (b[1] + b[3]) / 2.];
            if super::super::foundation::contains(footprint, center) {
                cells.push(Cell {
                    bounds: b.map(|v| v / 1000.),
                    coverage: 1.,
                    polygon: vec![],
                });
            }
        }
    }
    if cells.is_empty() {
        return Err("В основании нет ячеек. Уменьшите шаг основания.".into());
    }
    Ok(cells)
}

fn regular_cell_at(cells: &[Cell], p: Point) -> usize {
    cells
        .iter()
        .position(|cell| {
            p[0] >= cell.bounds[0]
                && p[0] <= cell.bounds[2]
                && p[1] >= cell.bounds[1]
                && p[1] <= cell.bounds[3]
        })
        .unwrap_or_else(|| nearest_cell(cells, p))
}

fn nearest_cell(cells: &[Cell], p: Point) -> usize {
    let distance = |c: &Cell| {
        let q = c.center();
        (p[0] - q[0]).hypot(p[1] - q[1])
    };
    cells
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| distance(a).total_cmp(&distance(b)))
        .map(|(i, _)| i)
        .expect("soil mesh is nonempty")
}

pub(super) fn calculate(
    app: &mut App,
    holes: Vec<Borehole>,
    mut options: Options,
    step: f64,
    start: Start<'_>,
    mut progress: impl FnMut(String) -> Result<(), String>,
) -> Result<serde_json::Value, String> {
    let footprint = match &start {
        Start::ExistingMesh { foundation, .. } => app
            .contours
            .iter()
            .find(|c| c.id == *foundation && c.kind == Kind::Foundation)
            .ok_or("Выберите исходное основание.")?
            .clone(),
        Start::Prototype { slab, .. } => slab.clone(),
    };
    let bottom = app.building.owner(footprint.id);
    let elevation = app.source_heights(&footprint)[0];
    let use_fe =
        app.soil_options.discretization == super::soil_settings::Discretization::FiniteElements;
    let generated;
    let (mesh, mesh_step) = match &start {
        Start::ExistingMesh { mesh, step, .. } => {
            progress("Подготовка C1 по текущей МКЭ-сетке".into())?;
            (*mesh, *step)
        }
        Start::Prototype { .. } => {
            let mut layout = app.building_layout();
            let sources = crate::plan_trace::columns::meshing_sources(
                &app.contours,
                &app.support_height,
                &mut layout,
            );
            progress("Построение сетки здания".into())?;
            let mut mesh = crate::plan_trace::building_mesh::generate(
                &sources,
                app.mesh_size,
                layout.ok_or("layout")?,
                app.mesh_options,
            )?;
            crate::plan_trace::columns::tag_mesh(&mut mesh, &app.contours, &app.support_height);
            crate::plan_trace::fem::precompute_wall_mesh(&mut mesh, app.mesh_size)?;
            generated = mesh;
            (&generated, app.mesh_size)
        }
    };
    let mut fe_centers = mesh
        .elements
        .iter()
        .map(|e| {
            std::array::from_fn::<_, 2, _>(|a| {
                e.nodes.iter().map(|n| mesh.nodes[*n].xy[a]).sum::<f64>() / 3.
            })
        })
        .collect::<Vec<_>>();
    let display_ids = mesh
        .elements
        .iter()
        .enumerate()
        .map(|(i, e)| (e.id, i))
        .collect::<BTreeMap<_, _>>();
    let building = mesh.building.as_ref().ok_or("Требуется модель здания")?;
    if building.element_slabs.len() != mesh.elements.len() {
        return Err("Нет принадлежности КЭ плитам".into());
    }
    for q in &mesh.quads {
        let center = std::array::from_fn::<_, 2, _>(|a| {
            q.nodes.iter().map(|n| mesh.nodes[*n].xy[a]).sum::<f64>() / 4.
        });
        for id in q.display_triangles {
            fe_centers[display_ids[&id]] = center;
        }
    }
    let mut owners = BTreeSet::new();
    let mut selected = vec![false; fe_centers.len()];
    for (i, (&slab_id, center)) in building.element_slabs.iter().zip(&fe_centers).enumerate() {
        let z = mesh.node_z(mesh.elements[i].nodes[0]);
        let included = match &start {
            Start::ExistingMesh { .. } => {
                super::super::foundation::same_elevation(elevation, z)
                    && super::super::foundation::contains(&footprint, *center)
            }
            Start::Prototype { slab, .. } => slab_id == slab.id,
        };
        if !included {
            continue;
        }
        selected[i] = true;
        owners.insert(slab_id);
    }
    if owners.is_empty() {
        return Err(format!(
            "В основании нет КЭ плит на отметке Z={elevation:.3} мм."
        ));
    }
    if matches!(&start, Start::ExistingMesh { .. }) {
        options.base_z = elevation / 1000.;
    }
    let (foundation_id, initial_field) = match &start {
        Start::ExistingMesh { .. } => (
            footprint.id,
            Some(crate::plan_trace::foundation::Field::new(
                &footprint,
                app.gravity,
            )?),
        ),
        Start::Prototype { slab, .. } => {
            let mut f = slab.clone();
            f.parent = None;
            let existing = app.contours.iter().position(|c| {
                c.kind == Kind::Foundation
                    && super::super::foundation::same_elevation(app.source_heights(c)[0], elevation)
                    && c.points == slab.points
            });
            f.id = existing.map_or_else(|| app.next_id, |i| app.contours[i].id);
            f.kind = Kind::Foundation;
            f.name = "Основание · расчёт C1".into();
            f.use_foundation = false;
            f.foundation = Some(Default::default());

            let id = f.id;
            app.building.contour_floors.insert(f.id, bottom);
            if let Some(i) = existing {
                f.foundation = app.contours[i].foundation.clone();
                app.contours[i] = f;
            } else {
                app.next_id += 1;
                app.contours.push(f);
            }
            (id, None)
        }
    };
    // The prototype and existing-mesh callers can create a foundation after
    // triangulation. Only its source ownership needs to be added to the mesh.
    let updated_mesh = (building.layout.owner(foundation_id) != bottom).then(|| {
        let mut updated = mesh.clone();
        updated
            .building
            .as_mut()
            .unwrap()
            .layout
            .owners
            .insert(foundation_id, bottom);
        updated
    });
    let mesh = updated_mesh.as_ref().unwrap_or(mesh);
    for c in &mut app.contours {
        if owners.contains(&c.id) {
            c.use_foundation = true;
        }
    }
    let competing = app
        .contours
        .iter()
        .filter(|c| {
            c.kind == Kind::Foundation
                && c.id != foundation_id
                && super::super::foundation::same_elevation(app.source_heights(c)[0], elevation)
        })
        .collect::<Vec<_>>();
    if fe_centers.iter().enumerate().any(|(i, p)| {
        selected[i]
            && competing
                .iter()
                .any(|c| super::super::foundation::contains(c, *p))
    }) {
        return Err("КЭ выбранного основания также попадают в другое основание.".into());
    }
    let mut cell_by_display = vec![None; mesh.elements.len()];
    let mut cells = if use_fe {
        Vec::new()
    } else {
        regular_cells(&footprint, step)?
    };
    if use_fe {
        for q in &mesh.quads {
            let displays = q.display_triangles.map(|id| display_ids[&id]);
            if !selected[displays[0]] {
                continue;
            }
            let points = q.nodes.map(|n| mesh.nodes[n].xy.map(|v| v / 1000.));
            let cell = Cell::from_polygon(points.to_vec()).ok_or_else(|| {
                format!(
                    "Вырожденный четырёхугольный КЭ {} для расчёта основания",
                    q.id
                )
            })?;
            for i in displays {
                cell_by_display[i] = Some(cells.len());
            }
            cells.push(cell);
        }
    }
    let mut display_area = vec![0.; mesh.elements.len()];
    let mut display_initial = vec![0.; mesh.elements.len()];
    for (i, e) in mesh.elements.iter().enumerate() {
        if !selected[i] {
            continue;
        }
        let ps = e.nodes.map(|n| mesh.nodes[n].xy);
        display_area[i] = geometry::area(&ps).abs() / 1e6;
        if use_fe && cell_by_display[i].is_none() {
            let cell =
                Cell::from_polygon(ps.map(|p| p.map(|v| v / 1000.)).to_vec()).ok_or_else(|| {
                    format!("Вырожденный треугольный КЭ {} для расчёта основания", e.id)
                })?;
            cell_by_display[i] = Some(cells.len());
            cells.push(cell);
        } else if !use_fe {
            cell_by_display[i] = Some(regular_cell_at(&cells, fe_centers[i].map(|v| v / 1000.)));
        }
        if let Some(field) = &initial_field {
            display_initial[i] = field
                .at(fe_centers[i])
                .filter(|c| c.is_finite() && *c > 0.)
                .ok_or_else(|| {
                    format!(
                        "КЭ {}. В центре элемента требуется положительное исходное C1.",
                        e.id
                    )
                })?
                / 9.80665;
        }
    }
    if use_fe && cells.len() > 10_000 {
        return Err(format!("В основании {} КЭ. Для расчёта осадок по сетке КЭ допустимо до 10 000. Увеличьте размер КЭ или выберите регулярную сетку.", cells.len()));
    }
    let mut occupied = vec![0.; cells.len()];
    let mut integrated_c1 = vec![0.; cells.len()];
    for (i, j) in cell_by_display.iter().enumerate() {
        if let Some(j) = *j {
            occupied[j] += display_area[i];
            integrated_c1[j] += display_area[i] * display_initial[i];
        }
    }
    let mut initial = Vec::with_capacity(cells.len());
    let mut retained = Vec::with_capacity(cells.len());
    let mut remap = vec![None; cells.len()];
    for (i, mut cell) in cells.into_iter().enumerate() {
        let area = occupied[i];
        if !use_fe {
            cell.coverage = area / cell.area();
        }
        if cell.coverage <= 1e-8 {
            continue;
        }
        initial.push(if initial_field.is_some() {
            integrated_c1[i] / area
        } else {
            1000.
        });
        remap[i] = Some(retained.len());
        retained.push(cell);
    }
    if retained.is_empty() {
        return Err("На выбранной плите нет КЭ".into());
    }
    for index in &mut cell_by_display {
        *index = index.and_then(|j| remap[j]);
    }
    let engine = Engine {
        cells: retained,
        holes,
        options,
    };
    (progress)(format!(
        "Расчёт основания · {} {}",
        engine.cells.len(),
        if use_fe {
            "КЭ плит"
        } else {
            "ячеек"
        }
    ))?;
    let weights = app
        .combinations
        .iter()
        .find(|c| c.id == app.soil_options.combination)
        .ok_or("Выберите сочетание для расчёта основания")?
        .weights(&app.load_cases, app.soil_options.variant)?;
    let gravity = app.gravity;
    let boundary_cells = crate::plan_trace::foundation::vertices(&footprint)
        .into_iter()
        .map(|p| {
            let p = p.map(|v| v / 1000.);
            if use_fe {
                nearest_cell(&engine.cells, p)
            } else {
                regular_cell_at(&engine.cells, p)
            }
        })
        .collect::<Vec<_>>();
    let samples = fe_centers
        .iter()
        .zip(&cell_by_display)
        .filter_map(|(&p, &j)| j.map(|j| (p.map(f64::to_bits), (p, j))))
        .collect::<BTreeMap<_, _>>();
    let update = |all: &mut Vec<Contour>, c: &[f64]| {
        let f = all.iter_mut().find(|f| f.id == foundation_id).unwrap();
        let value = |index: usize| crate::plan_trace::loads::Magnitude {
            value: c[index] * 9.80665 / gravity,
            units: crate::plan_trace::loads::ForceUnits::TonForce,
        };
        f.foundation = Some(crate::plan_trace::foundation::Data {
            values: boundary_cells.iter().map(|&j| value(j)).collect(),
            interior: samples
                .values()
                .map(|&(xy, j)| crate::plan_trace::foundation::Sample { xy, c1: value(j) })
                .collect(),
            ..f.foundation.clone().unwrap_or_default()
        });
    };
    let mut iteration = 0;
    let progress = std::cell::RefCell::new(&mut progress);
    let mut last_fe_settlement = vec![0.; engine.cells.len()];
    let mut last_force_balance = 0.;
    let mut transferred_pressure = None;
    let mut previous_pressure = None;
    let mut last_fe_nodal = vec![0.; engine.cells.len()];
    let started = std::time::Instant::now();
    if let Start::Prototype {
        warm: Some(report), ..
    } = start
    {
        let old: Vec<Cell> =
            serde_json::from_value(report["cells"].clone()).map_err(|e| e.to_string())?;
        let c: Vec<f64> =
            serde_json::from_value(report["result"]["c1"].clone()).map_err(|e| e.to_string())?;
        if old.len() != c.len() {
            return Err("Warm report size mismatch".into());
        }
        for (i, cell) in engine.cells.iter().enumerate() {
            let p = cell.center();
            if let Some((j, _)) = old.iter().enumerate().min_by(|(_, a), (_, b)| {
                let d = |c: &Cell| {
                    let q = c.center();
                    (p[0] - q[0]).hypot(p[1] - q[1])
                };
                d(a).total_cmp(&d(b))
            }) {
                initial[i] = c[j];
            }
        }
    }
    let cancel = crate::plan_trace::fem::progress::current_cancel();
    let result = engine.iterate_with_progress_and_cancel(
        initial,
        |c| {
            iteration += 1;
            if iteration > 1 || initial_field.is_none() {
                update(&mut app.contours, c);
            }
            let phase = if iteration > engine.options.max_iterations {
                "Контрольный расчёт после последнего обновления C1".to_owned()
            } else {
                format!("Итерация {iteration}/{}", engine.options.max_iterations)
            };
            (progress.borrow_mut())(format!("{phase} · решение МКЭ"))?;
            let r = crate::plan_trace::fem::solve_with_modal(
                mesh,
                &app.contours,
                &app.materials,
                &app.load_cases,
                app.gravity,
                mesh_step,
                app.element_type,
                &app.support_height,
                None,
                false,
                None,
            )?;
            let combined = crate::plan_trace::load_combinations::combine(&r, 1, &weights)?;
            last_force_balance = combined.balance_kn;
            let mut forces = vec![0.; engine.cells.len()];
            let mut displacements = vec![0.; engine.cells.len()];
            let mut areas = vec![0.; engine.cells.len()];
            let mut nodal = vec![0.; engine.cells.len()];
            for (i, e) in mesh.elements.iter().enumerate() {
                let Some(j) = cell_by_display[i] else {
                    continue;
                };
                let area = display_area[i];
                let q = -combined.foundation_pressure_kn_m2[i] / 9.80665;
                if !q.is_finite() {
                    return Err(format!("No foundation pressure for FE {}", e.id));
                }
                if q < -1e-5 {
                    return Err(format!("Unilateral contact required: pressure {q}tf/m2"));
                }
                forces[j] += q.max(0.) * area;
                let c1 = if iteration == 1 && initial_field.is_some() {
                    display_initial[i]
                } else {
                    c[j]
                };
                displacements[j] += q / c1 * area;
                nodal[j] += e
                    .nodes
                    .iter()
                    .map(|n| combined.w_mm[*n] / 1000.)
                    .sum::<f64>()
                    / 3.
                    * area;
                areas[j] += area;
            }
            last_fe_nodal = nodal
                .iter()
                .zip(&areas)
                .map(|(w, a)| if *a > 0. { w / a } else { 0. })
                .collect();
            last_fe_settlement = displacements
                .iter()
                .zip(&areas)
                .map(|(w, a)| if *a > 0. { w / a } else { 0. })
                .collect();
            let q = forces
                .iter()
                .zip(&engine.cells)
                .map(|(f, c)| f / c.area())
                .collect::<Vec<_>>();
            let reaction = forces.iter().sum::<f64>();
            if initial_field.is_some() && reaction <= 1e-10 {
                return Err(
                    "В основании нет реакции сжатия. Проверьте сочетание и опирание.".into(),
                );
            }
            (progress.borrow_mut())(format!(
                "{phase} · реакция {reaction:.3} тс · расчёт осадок"
            ))?;
            transferred_pressure = previous_pressure.replace(q.clone());
            Ok(q)
        },
        |i, residual| {
            (progress.borrow_mut())(format!(
                "Итерация {i}/{} · невязка {:.2}%",
                engine.options.max_iterations,
                residual * 100.
            ))
        },
        cancel.as_deref(),
    )?;
    if result.iterations > 1 || initial_field.is_none() || !result.converged {
        update(&mut app.contours, &result.c1);
    }
    let residual = result
        .pressure
        .iter()
        .zip(&result.settlement.metres)
        .zip(&result.c1)
        .filter(|((q, s), _)| **q > 1e-10 && **s > 1e-12)
        .map(|((&q, &s), &c1)| {
            let target = q / s;
            (target - c1).abs() / target.max(c1)
        })
        .fold(0., f64::max);
    Ok(
        serde_json::json!({"residual":residual,"transferred_pressure_tf_m2":transferred_pressure,"foundation_id":foundation_id,"reference":format!("Сочетание {} · {}",app.soil_options.combination,app.soil_options.variant.label()),"fe_settlement_m":last_fe_settlement,"fe_nodal_proxy_m":last_fe_nodal,"fe_force_balance_kn":last_force_balance,"ground_base_z":engine.options.base_z,"elapsed_seconds":started.elapsed().as_secs_f64(),"options":engine.options,"cells":engine.cells,"result":result}),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plan_trace::{building_mesh, loads};

    #[test]
    fn project_c1_uses_the_supplied_mesh_and_initial_foundation() {
        let mut app = App::default();
        app.building.enabled = true;
        app.support_height.automatic_building_bottom = true;
        app.support_height.building_bottom_mask = 3;
        app.load_cases[0].kind = loads::LoadType::SelfWeight;
        app.begin(Kind::Slab);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([0., 0.], false);
        app.sketch_click([2000., 2000.], false);
        app.cancel();
        app.sync_stiffness_sources();
        let mesh = building_mesh::generate(
            &app.contours,
            1000.,
            app.building_layout().unwrap(),
            app.mesh_options,
        )
        .unwrap();
        let holes = vec![Borehole {
            id: 1,
            xy: [1., 1.],
            mouth_z: 0.,
            layers: vec![mini_fem::soil::Layer {
                material_id: None,
                bottom_z: -10.,
                e: 2500.,
                gamma: 2.,
            }],
        }];
        let options = Options {
            base_z: 0.,
            max_depth: 2.,
            tolerance: 1.,
            ..Default::default()
        };
        let mut stages = Vec::new();
        let missing = calculate(
            &mut app,
            holes.clone(),
            options.clone(),
            1000.,
            Start::ExistingMesh {
                mesh: &mesh,
                step: 1000.,
                foundation: 999,
            },
            |s| {
                stages.push(s);
                Ok(())
            },
        )
        .unwrap_err();
        assert!(missing.contains("исходное основание"), "{missing}");
        assert_eq!(app.contours.len(), 1);

        app.begin(Kind::Foundation);
        app.choose_tool(Tool::Rectangle);
        app.sketch_click([-100., -100.], false);
        app.sketch_click([2100., 2100.], false);
        app.cancel();
        let id = app.contours.last().unwrap().id;
        assert!(app.contours.last().unwrap().foundation.is_none());
        let default_started = calculate(
            &mut app,
            holes.clone(),
            options.clone(),
            1000.,
            Start::ExistingMesh {
                mesh: &mesh,
                step: 1000.,
                foundation: id,
            },
            |_| Err("started".into()),
        )
        .unwrap_err();
        assert_eq!(default_started, "started");
        let base = app.contours.last_mut().unwrap();
        let data = base.foundation.get_or_insert_with(Default::default);
        // A standalone slab has no walls on which to impose the automatic XY
        // restraints. Give this fixture horizontal springs through its foundation.
        data.horizontal_enabled = true;
        data.horizontal.value = 1000.;
        data.values = vec![
            loads::Magnitude {
                value: 320.,
                units: loads::ForceUnits::TonForce,
            };
            4
        ];
        let report = calculate(
            &mut app,
            holes.clone(),
            options.clone(),
            1000.,
            Start::ExistingMesh {
                mesh: &mesh,
                step: 1000.,
                foundation: id,
            },
            |s| {
                stages.push(s);
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(report["result"]["converged"], true);
        assert_eq!(report["result"]["iterations"], 1);
        assert_eq!(report["foundation_id"], id);
        assert!(report["result"]["c1"]
            .as_array()
            .unwrap()
            .iter()
            .all(|v| (v.as_f64().unwrap() - 320. * app.gravity / 9.80665).abs() < 1e-8));
        assert!(report["result"]["pressure"]
            .as_array()
            .unwrap()
            .iter()
            .any(|v| v.as_f64().unwrap() > 0.));
        let prior = app.contours.last().unwrap().foundation.clone();
        let limited = calculate(
            &mut app,
            holes.clone(),
            Options {
                max_iterations: 1,
                tolerance: 1e-9,
                ..options.clone()
            },
            1000.,
            Start::ExistingMesh {
                mesh: &mesh,
                step: 1000.,
                foundation: id,
            },
            |s| {
                stages.push(s);
                Ok(())
            },
        )
        .unwrap();
        assert_eq!(limited["result"]["converged"], false);
        assert_ne!(app.contours.last().unwrap().foundation, prior);
        let resumed = calculate(
            &mut app,
            holes.clone(),
            options.clone(),
            1000.,
            Start::ExistingMesh {
                mesh: &mesh,
                step: 1000.,
                foundation: id,
            },
            |s| {
                stages.push(s);
                Ok(())
            },
        )
        .unwrap();
        let saved = limited["result"]["c1"].as_array().unwrap();
        let reused = resumed["result"]["c1"].as_array().unwrap();
        assert_eq!(saved.len(), reused.len());
        assert!(saved
            .iter()
            .zip(reused)
            .all(|(a, b)| (a.as_f64().unwrap() - b.as_f64().unwrap()).abs() < 1e-6));
        assert!(stages.iter().all(|s| !s.contains("Построение сетки")));

        app.soil_options.discretization =
            super::super::soil_settings::Discretization::FiniteElements;
        let quad_mesh = building_mesh::generate(
            &app.contours,
            1000.,
            app.building_layout().unwrap(),
            crate::plan_trace::quad_mesh::Options {
                merge: true,
                ..Default::default()
            },
        )
        .unwrap();
        assert!(!quad_mesh.quads.is_empty());
        let on_mesh = calculate(
            &mut app,
            holes,
            options,
            0.,
            Start::ExistingMesh {
                mesh: &quad_mesh,
                step: 1000.,
                foundation: id,
            },
            |_| Ok(()),
        )
        .unwrap();
        let cells: Vec<Cell> = serde_json::from_value(on_mesh["cells"].clone()).unwrap();
        assert_eq!(
            cells.len(),
            quad_mesh.elements.len() - quad_mesh.quads.len()
        );
        assert_eq!(
            cells.iter().filter(|c| c.polygon.len() == 4).count(),
            quad_mesh.quads.len()
        );
        assert!((cells.iter().map(Cell::area).sum::<f64>() - 4.).abs() < 1e-9);
        let force = |report: &serde_json::Value| {
            let cells: Vec<Cell> = serde_json::from_value(report["cells"].clone()).unwrap();
            cells
                .iter()
                .zip(report["result"]["pressure"].as_array().unwrap())
                .map(|(c, q)| c.area() * q.as_f64().unwrap())
                .sum::<f64>()
        };
        assert!((force(&resumed) - force(&on_mesh)).abs() < 1e-7);
    }
}
