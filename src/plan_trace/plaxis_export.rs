//! Append-only PLAXIS Input commands; latest-object references avoid generated names.
#[path = "plaxis_python.rs"]
mod python;
use super::*;
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
use crate::plan_trace::{loads, stiffness};
use mini_fem::geo::{Area, BooleanOps, LineString, MultiPolygon, Polygon};
use std::fmt::Write;
struct Export {
    text: String,
    surfaces: usize,
    lines: usize,
    loads: usize,
}
fn polygon(points: &[Point]) -> Polygon<f64> {
    let mut ring = points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>();
    if let Some(first) = ring.first().copied() {
        if ring.last() != Some(&first) {
            ring.push(first);
        }
    }
    Polygon::new(LineString::from(ring), vec![])
}
fn points(p: &Polygon<f64>) -> Vec<Point> {
    let mut out = p
        .exterior()
        .0
        .iter()
        .map(|p| [p.x, p.y])
        .collect::<Vec<_>>();
    if out.len() > 1 && out.first() == out.last() {
        out.pop();
    }
    out
}
fn shared_edge(a: &Polygon<f64>, b: &Polygon<f64>) -> bool {
    a.exterior().0.windows(2).any(|p| {
        b.exterior()
            .0
            .windows(2)
            .any(|q| p[0] == q[1] && p[1] == q[0])
    })
}
fn slab_parts(slab: &Contour, all: &[Contour]) -> Result<Vec<Vec<Point>>, String> {
    let mut area = MultiPolygon(vec![polygon(&slab.points)]);
    for hole in all
        .iter()
        .filter(|c| c.active_hole() && c.parent == Some(slab.id))
    {
        model::validate_candidate(all, hole).map_err(|e| format!("Отверстие {}: {e}", hole.id))?;
        area = area.difference(&polygon(&hole.points));
    }
    partition(area, slab.id)
}
fn partition(area: MultiPolygon<f64>, id: u64) -> Result<Vec<Vec<Point>>, String> {
    if area.0.is_empty() {
        return Ok(vec![]);
    }
    let expected = area.unsigned_area();
    let mut result = vec![];
    for poly in area.0 {
        if poly.interiors().is_empty() {
            result.push(points(&poly));
            continue;
        }
        let rings = std::iter::once(poly.exterior())
            .chain(poly.interiors())
            .map(|r| r.0.iter().map(|p| [p.x, p.y]).collect::<Vec<_>>())
            .collect::<Vec<_>>();
        let triangles = geometry::triangulate(&rings)?;
        let mut parts = triangles.iter().map(|p| polygon(p)).collect::<Vec<_>>();
        // Join adjacent geometric pieces while keeping every hole empty. These
        // are contour partitions, independent of the user's finite-element mesh.
        loop {
            let mut joined = false;
            'pairs: for i in 0..parts.len() {
                for j in i + 1..parts.len() {
                    if !shared_edge(&parts[i], &parts[j]) {
                        continue;
                    }
                    let merged = parts[i].union(&parts[j]);
                    if merged.0.len() == 1
                        && merged.0[0].interiors().is_empty()
                        && merged.0[0].exterior().0.len() <= 256
                    {
                        parts[i] = merged.0.into_iter().next().unwrap();
                        parts.remove(j);
                        joined = true;
                        break 'pairs;
                    }
                }
            }
            if !joined {
                break;
            }
        }
        result.extend(parts.iter().map(points));
    }
    let actual = result
        .iter()
        .map(|p| polygon(p).unsigned_area())
        .sum::<f64>();
    if expected <= 0. || (actual - expected).abs() > expected.abs().max(1.) * 1e-8 {
        return Err(format!(
            "Плита {}: не удалось сохранить площадь и отверстия при разбиении",
            id
        ));
    }
    Ok(result)
}
fn command(text: &mut String, name: &str, p: &[[f64; 3]]) -> Result<(), String> {
    if p.iter().flatten().any(|v| !v.is_finite()) {
        return Err("Нечисловые координаты экспортируемого объекта".into());
    }
    let mut vertices = p.to_vec();
    if name == "_surface" {
        let start = (0..vertices.len())
            .find(|&i| {
                let [a, b, c] = [
                    vertices[i],
                    vertices[(i + 1) % vertices.len()],
                    vertices[(i + 2) % vertices.len()],
                ];
                let u = [b[0] - a[0], b[1] - a[1], b[2] - a[2]];
                let v = [c[0] - a[0], c[1] - a[1], c[2] - a[2]];
                let cross = [
                    u[1] * v[2] - u[2] * v[1],
                    u[2] * v[0] - u[0] * v[2],
                    u[0] * v[1] - u[1] * v[0],
                ];
                cross.iter().map(|x| x * x).sum::<f64>() > 1e-12
            })
            .ok_or("Вырожденная поверхность для Plaxis")?;
        vertices.rotate_left(start);
    }
    text.push_str(name);
    for p in &vertices {
        for v in p {
            write!(text, " {:.9}", v / 1000.).unwrap();
        }
    }
    text.push('\n');
    Ok(())
}
fn material_commands(
    text: &mut String,
    entry: &stiffness::Entry,
    id: u64,
    gamma: f64,
) -> Result<(), String> {
    entry.material.validate()?;
    let m = entry.material;
    writeln!(
        text,
        "## FEM stiffness: {}",
        entry.name.replace(['\n', '\r'], " ")
    )
    .unwrap();
    if matches!(
        entry.family,
        stiffness::Family::Slab | stiffness::Family::Wall
    ) {
        writeln!(text, "_platemat \"Identification\" \"RCF_{id}\" \"MaterialType\" \"Elastic\" \"Gamma\" {gamma:.9} \"E1\" {:.9} \"StructNu12\" {:.9} \"d\" {:.9}", m.e_mpa * 1000., m.nu, m.thickness_mm / 1000.).unwrap();
    } else {
        if !entry.width_mm.is_finite() || entry.width_mm <= 0. {
            return Err(format!("Стержень {id}: некорректная ширина"));
        }
        writeln!(text, "_beammat \"Identification\" \"RCF_{id}\" \"MaterialType\" \"Elastic\" \"Gamma\" {gamma:.9} \"PredefinedCrossSectionType\" \"Solid rectangular beam\" \"Height\" {:.9} \"Width\" {:.9} \"E\" {:.9}", m.thickness_mm / 1000., entry.width_mm / 1000., m.e_mpa * 1000.).unwrap();
        writeln!(text, "## CHECK beam local axes / section rotation in PLAXIS; source nu={:.9} (beam nu command not verified).", m.nu).unwrap();
    }
    Ok(())
}
fn assign_latest(text: &mut String, surface: bool) {
    let (feature, objects) = if surface {
        ("plate", "Surfaces")
    } else {
        ("beam", "Lines")
    };
    writeln!(
        text,
        "_{feature} {objects}[-1]\n_set {objects}[-1].{}.Material Materials[-1]",
        if surface { "Plate" } else { "Beam" }
    )
    .unwrap();
}
struct LoadExport {
    factors: BTreeMap<u64, f64>,
    gravity: f64,
    weight_factor: f64,
    title: String,
}
fn export_loads(
    text: &mut String,
    all: &[Contour],
    heights: &BTreeMap<u64, Vec<f64>>,
    loads: &LoadExport,
) -> Result<usize, String> {
    writeln!(
        text,
        "## Loads: {}. Positive FEM down -> negative PLAXIS global Z.",
        loads.title.replace(['\n', '\r'], " ")
    )
    .unwrap();
    if all.iter().any(|c| {
        c.load.as_ref().is_some_and(|l| {
            (l.moment || l.direction != loads::Direction::Z)
                && loads.factors.get(&l.case_id).is_some_and(|f| *f != 0.)
        })
    }) {
        return Err(
            "Экспорт PLAXIS пока поддерживает только нагрузки по Z без сосредоточенных моментов"
                .into(),
        );
    }
    let mut count = 0;
    let mut areas = all
        .iter()
        .filter(|c| {
            c.kind == Kind::LoadArea
                && c.load
                    .as_ref()
                    .is_some_and(|l| loads.factors.get(&l.case_id).is_some_and(|f| *f != 0.))
        })
        .collect::<Vec<_>>();
    // FEM uses the largest overlapping area intensity within each load case.
    // Partition in source intensity order, then apply the view coefficient.
    areas.sort_by(|a, b| {
        b.load
            .as_ref()
            .unwrap()
            .magnitude
            .in_units(loads::ForceUnits::KiloNewton, loads.gravity)
            .total_cmp(
                &a.load
                    .as_ref()
                    .unwrap()
                    .magnitude
                    .in_units(loads::ForceUnits::KiloNewton, loads.gravity),
            )
    });
    let mut covered = BTreeMap::<(u64, i64), MultiPolygon<f64>>::new();
    for c in areas {
        let l = c.load.as_ref().unwrap();
        let z = heights[&c.id][0];
        let plane = all
            .iter()
            .filter(|p| {
                if p.kind == Kind::Slab {
                    return (heights[&p.id][0] - z).abs() < 0.05;
                }
                if p.kind == Kind::Hole {
                    return p
                        .parent
                        .and_then(|id| heights.get(&id))
                        .is_some_and(|h| (h[0] - z).abs() < 0.05);
                }
                p.id == c.id || (p.kind == Kind::LoadHole && p.parent == Some(c.id))
            })
            .cloned()
            .collect::<Vec<_>>();
        let area = model::visible_load_area(&plane, c);
        let prior = covered
            .entry((l.case_id, (z * 100.).round() as i64))
            .or_insert_with(|| MultiPolygon(vec![]));
        let selected = area.difference(prior);
        *prior = prior.union(&area);
        let q = -l
            .magnitude
            .in_units(loads::ForceUnits::KiloNewton, loads.gravity)
            * loads.factors[&l.case_id];
        if !q.is_finite() {
            return Err(format!("Нагрузка {}: некорректная интенсивность", c.id));
        }
        for part in partition(selected, c.id)? {
            command(
                text,
                "_surface",
                &part.iter().map(|p| [p[0], p[1], z]).collect::<Vec<_>>(),
            )?;
            writeln!(
                text,
                "_surfload Surfaces[-1]\n_set Surfaces[-1].SurfaceLoad.sigz {q:.9}"
            )
            .unwrap();
            count += 1;
        }
    }
    for c in all
        .iter()
        .filter(|c| matches!(c.kind, Kind::LoadLine | Kind::LoadPoint))
    {
        let Some(l) = &c.load else {
            continue;
        };
        let Some(&factor) = loads.factors.get(&l.case_id).filter(|f| **f != 0.) else {
            continue;
        };
        model::validate_candidate(all, c).map_err(|e| format!("Нагрузка {}: {e}", c.id))?;
        let q = -l
            .magnitude
            .in_units(loads::ForceUnits::KiloNewton, loads.gravity)
            * factor;
        if !q.is_finite() {
            return Err(format!("Нагрузка {}: некорректная интенсивность", c.id));
        }
        let z = heights[&c.id][0];
        writeln!(text, "## FEM load {} case {}", c.id, l.case_id).unwrap();
        if c.kind == Kind::LoadPoint {
            let p = c.points[0];
            command(text, "_point", &[[p[0], p[1], z]])?;
            writeln!(
                text,
                "_pointload Points[-1]\n_set Points[-1].PointLoad.Fz {q:.9}"
            )
            .unwrap();
            count += 1;
        } else {
            let mut path = c.points.clone();
            if c.closed() {
                path.push(path[0]);
            }
            for edge in path.windows(2).filter(|e| e[0] != e[1]) {
                command(
                    text,
                    "_line",
                    &edge.iter().map(|p| [p[0], p[1], z]).collect::<Vec<_>>(),
                )?;
                writeln!(text,"_lineload Lines[-1]\n_set Lines[-1].LineLoad.qz_start {q:.9}\n_set Lines[-1].LineLoad.qz_end {q:.9}").unwrap();
                count += 1;
            }
        }
    }
    Ok(count)
}
fn encode(
    all: &[Contour],
    heights: &BTreeMap<u64, Vec<f64>>,
    materials: Option<&BTreeMap<u64, stiffness::Entry>>,
    loads: Option<&LoadExport>,
) -> Result<Export, String> {
    let mut text = String::from(
        "## Rconcrete FEM -> PLAXIS 3D Input
## Coordinates in METRES, force in kN. Use a COPY with UnitLength=m and UnitForce=kN.
## Slabs with openings may consist of several adjoining surfaces.
## Elastic properties when supplied; weight uses selected self-weight case coefficient.
## Check rectangular beam local axes and shear/torsion properties before calculation.
## No mesh, new, delete or save commands.
_gotostructures
",
    );
    let mut surfaces = 0;
    let mut lines = 0;
    for c in all.iter().filter(|c| {
        matches!(
            c.kind,
            Kind::Slab | Kind::Support | Kind::Beam | Kind::Column
        )
    }) {
        model::validate_candidate(all, c).map_err(|e| format!("Объект {}: {e}", c.id))?;
        let z = heights
            .get(&c.id)
            .ok_or_else(|| format!("Объект {}: не заданы отметки", c.id))?;
        let low = *z.first().ok_or("Отсутствует отметка объекта")?;
        writeln!(text, "## FEM {:?} ID {}", c.kind, c.id).unwrap();
        if let Some(materials) = materials {
            let entry = materials
                .get(&c.id)
                .ok_or_else(|| format!("Объект {}: нет назначенной жёсткости", c.id))?;
            let gamma = loads.map_or(0., |l| {
                entry
                    .material
                    .weight
                    .in_units(loads::ForceUnits::KiloNewton, l.gravity)
                    * l.weight_factor
            });
            if !gamma.is_finite() || gamma < 0. {
                return Err(
                    "Plaxis: отрицательный/нечисловой коэффициент собственного веса не поддержан"
                        .into(),
                );
            }
            material_commands(&mut text, entry, c.id, gamma)?;
            if matches!(c.kind, Kind::Beam | Kind::Column) {
                writeln!(
                    text,
                    "## Source section rotation: {:.9} radians (verify local axes).",
                    c.column_angle
                )
                .unwrap();
            }
        }
        match c.kind {
            Kind::Slab => {
                for part in slab_parts(c, all)? {
                    let p = part.iter().map(|p| [p[0], p[1], low]).collect::<Vec<_>>();
                    command(&mut text, "_surface", &p)?;
                    if materials.is_some() {
                        assign_latest(&mut text, true);
                    }
                    surfaces += 1;
                }
            }
            Kind::Beam => {
                let mut path = c.beam_path([low, low]);
                if !c.is_free_beam() {
                    path.iter_mut().for_each(|p| p[2] = low);
                }
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                for e in path.windows(2).filter(|e| e[0] != e[1]) {
                    command(&mut text, "_line", e)?;
                    if materials.is_some() {
                        assign_latest(&mut text, false);
                    }
                    lines += 1;
                }
            }
            Kind::Support => {
                let mut path = c.points.clone();
                if c.closed() && !path.is_empty() {
                    path.push(path[0]);
                }
                for edge in path.windows(2) {
                    if edge[0] == edge[1] {
                        continue;
                    }
                    let [a, b] = [edge[0], edge[1]];
                    let top = *z.last().unwrap();
                    if top - low <= 0. {
                        return Err(format!("Стена {}: некорректная высота", c.id));
                    }
                    command(
                        &mut text,
                        "_surface",
                        &[
                            [a[0], a[1], low],
                            [b[0], b[1], low],
                            [b[0], b[1], top],
                            [a[0], a[1], top],
                        ],
                    )?;
                    if materials.is_some() {
                        assign_latest(&mut text, true);
                    }
                    surfaces += 1;
                }
            }
            Kind::Column => {
                let p = c.points[0];
                let top = *z.last().unwrap();
                if top - low <= 0. {
                    return Err(format!("Колонна {}: некорректная высота", c.id));
                }
                command(&mut text, "_line", &[[p[0], p[1], low], [p[0], p[1], top]])?;
                if materials.is_some() {
                    assign_latest(&mut text, false);
                }
                lines += 1;
            }
            _ => {}
        }
    }
    if surfaces + lines == 0 {
        return Err("Нет геометрии для экспорта в Plaxis".into());
    }
    let mut load_count = 0;
    if let Some(loads) = loads {
        let n = export_loads(&mut text, all, heights, loads)?;
        load_count = n;
        writeln!(text, "## Exported load features: {n}").unwrap();
    }
    Ok(Export {
        text,
        surfaces,
        lines,
        loads: load_count,
    })
}
impl App {
    pub(super) fn export_plaxis_txt(&mut self) {
        self.export_plaxis_script(false);
    }
    pub(super) fn export_plaxis_python(&mut self) {
        self.export_plaxis_script(true);
    }
    fn export_plaxis_script(&mut self, python: bool) {
        let heights = self
            .contours
            .iter()
            .map(|c| (c.id, self.source_heights(c)))
            .collect();
        let result = (|| -> Result<Option<String>, String> {
            let library = self.stiffnesses.for_variant(self.stiffness_variant);
            let materials = library
                .assignments
                .keys()
                .filter_map(|&contour| {
                    library
                        .resolved_entry(contour)
                        .map(|entry| (contour, entry))
                })
                .collect();
            let factors = self
                .load_cases
                .iter()
                .filter_map(|c| self.displayed_load_factor(c.id).map(|f| (c.id, f)))
                .collect::<BTreeMap<_, _>>();
            let weight_factor = self
                .load_cases
                .iter()
                .filter(|c| c.kind == loads::LoadType::SelfWeight)
                .map(|c| factors.get(&c.id).copied().unwrap_or(0.))
                .sum();
            let title = if self.result_by_combinations {
                self.combinations
                    .iter()
                    .find(|c| c.id == self.active_combination)
                    .map(|c| c.name.clone())
                    .unwrap_or_default()
            } else {
                self.load_cases
                    .iter()
                    .find(|c| c.id == self.active_load)
                    .map(|c| c.name.clone())
                    .unwrap_or_default()
            };
            let loads = LoadExport {
                factors,
                gravity: self.gravity,
                weight_factor,
                title: format!("{} / {}", title, self.combination_variant.label()),
            };
            let export = if python {
                python::encode(&self.contours, &heights, Some(&materials), Some(&loads))?
            } else {
                encode(&self.contours, &heights, Some(&materials), Some(&loads))?
            };
            let extension = if python { "py" } else { "txt" };
            let Some(path) = rfd::FileDialog::new()
                .add_filter(
                    if python {
                        "Plaxis Python"
                    } else {
                        "Plaxis commands"
                    },
                    &[extension],
                )
                .set_file_name(format!("Rconcrete_Plaxis3D.{extension}"))
                .save_file()
            else {
                return Ok(None);
            };
            crate::plan_trace::platform::write(&path, export.text.as_bytes())
                .map_err(|e| format!("Не удалось сохранить {extension}: {e}"))?;
            if python {
                crate::plan_trace::platform::write(&path.with_extension("cmd"), python::launcher())
                    .map_err(|e| format!("Не удалось сохранить CMD запуска: {e}"))?;
                return Ok(Some(format!(
                    "Python-скрипт Plaxis и CMD запуска сохранены: {}. Откройте CMD. Запустите сервер Remote Scripting в Plaxis Input; единицы м, кН. При неизвестном API геометрии скрипт остановится с диагностикой.", path.display()
                )));
            }
            Ok(Some(format!(
                "Экспорт Plaxis: {} поверхностей, {} линий, {} нагрузок · {}. Единицы Plaxis: м, кН. Жёсткости и нагрузки текущего вида переданы; оси и сдвиг стержней требуют проверки",
                export.surfaces,
                export.lines,
                export.loads,
                path.display()
            )))
        })();
        match result {
            Ok(Some(message)) => self.status = message,
            Ok(None) => {}
            Err(e) => self.status = format!("Экспорт Plaxis: {e}"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn slab_opening_remains_empty_and_commands_are_append_only_metres() {
        let c = |id, kind: &str, parent, points: Vec<Point>| -> Contour {
            serde_json::from_value(
                serde_json::json!({"id":id,"kind":kind,"parent":parent,"points":points,
                "support_open":false,"load":null,"refinement":null,"curves":[]}),
            )
            .unwrap()
        };
        let slab = c(
            1,
            "Slab",
            None::<u64>,
            vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
        );
        let hole = c(
            2,
            "Hole",
            Some(1),
            vec![
                [1000., 1000.],
                [3000., 1000.],
                [3000., 3000.],
                [1000., 3000.],
            ],
        );
        let all = vec![slab, hole];
        let parts = slab_parts(&all[0], &all).unwrap();
        assert!(
            (parts
                .iter()
                .map(|p| polygon(p).unsigned_area())
                .sum::<f64>()
                - 12e6)
                .abs()
                < 1e-5
        );
        for part in &parts {
            assert!(
                polygon(part)
                    .intersection(&polygon(&all[1].points))
                    .unsigned_area()
                    < 1e-5
            );
        }
        let export = encode(&all, &BTreeMap::from([(1, vec![3500.])]), None, None).unwrap();
        assert!(export
            .text
            .lines()
            .filter(|l| !l.starts_with("##"))
            .all(|l| l == "_gotostructures" || l.starts_with("_surface ")));
        assert!(export.text.contains("3.500000000"));
        assert!(!export.text.contains("3500.000000000"));
    }
    #[test]
    fn plaxis_loads_preserve_downward_sign_units_case_factor_and_opening() {
        let make = |id, kind: &str, p: Vec<Point>| -> Contour {
            serde_json::from_value(serde_json::json!({"id":id,"kind":kind,"points":p,"parent":null,"support_open":false,"load":null,"refinement":null,"curves":[]})).unwrap()
        };
        let slab = make(
            1,
            "Slab",
            vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
        );
        let mut area = slab.clone();
        area.id = 2;
        area.kind = Kind::LoadArea;
        area.load = Some(loads::LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: "q".into(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: loads::Magnitude {
                value: 1.5,
                units: loads::ForceUnits::TonForce,
            },
            closed: true,
        });
        let mut hole = make(
            3,
            "Hole",
            vec![
                [1000., 1000.],
                [3000., 1000.],
                [3000., 3000.],
                [1000., 3000.],
            ],
        );
        hole.parent = Some(1);
        let mut point = make(4, "LoadPoint", vec![[500., 500.]]);
        point.load = area.load.clone();
        point.load.as_mut().unwrap().magnitude.value = 2.;
        let mut line = make(5, "LoadLine", vec![[0., 0.], [4000., 0.]]);
        line.support_open = true;
        line.load = point.load.clone();
        line.load.as_mut().unwrap().closed = false;
        let all = vec![slab, area, hole, point, line];
        let heights = all.iter().map(|c| (c.id, vec![3000.])).collect();
        let loads = LoadExport {
            factors: BTreeMap::from([(1, 2.)]),
            gravity: 10.,
            weight_factor: 0.,
            title: "test".into(),
        };
        let mut text = String::new();
        assert!(export_loads(&mut text, &all, &heights, &loads).unwrap() >= 3);
        assert!(text.contains("SurfaceLoad.sigz -30.000000000"));
        assert!(text.contains("PointLoad.Fz -40.000000000"));
        assert!(text.contains("LineLoad.qz_start -40.000000000"));
        let area_sum = text
            .lines()
            .filter(|s| s.starts_with("_surface "))
            .map(|s| {
                let v = s
                    .split_whitespace()
                    .skip(1)
                    .map(|s| s.parse::<f64>().unwrap())
                    .collect::<Vec<_>>();
                let p = v.chunks_exact(3).map(|p| [p[0], p[1]]).collect::<Vec<_>>();
                assert!(
                    polygon(&p)
                        .intersection(&polygon(
                            &all[2]
                                .points
                                .iter()
                                .map(|p| [p[0] / 1000., p[1] / 1000.])
                                .collect::<Vec<_>>()
                        ))
                        .unsigned_area()
                        < 1e-7
                );
                polygon(&p).unsigned_area()
            })
            .sum::<f64>();
        assert!((area_sum - 12.).abs() < 1e-7);
        let mut skipped = String::new();
        let other = LoadExport {
            factors: BTreeMap::from([(2, 1.)]),
            ..loads
        };
        assert_eq!(
            export_loads(&mut skipped, &all, &heights, &other).unwrap(),
            0
        );
    }
}
