//! Common planar mesh exchange. Gmsh runs out of process; FE IDs remain native.
use super::geometry::Point;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use std::collections::{BTreeMap, BTreeSet};
thread_local! { static CONFIG:std::cell::RefCell<Option<(String,bool,bool)>>=const{std::cell::RefCell::new(None)}; }
pub fn saved_path() -> String {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(dir) = std::env::var_os("APPDATA") {
        return std::fs::read_to_string(
            std::path::PathBuf::from(dir).join("RconcreteFEM/gmsh_path.txt"),
        )
        .unwrap_or_default();
    }
    String::new()
}
pub fn save_path(path: &str) {
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(dir) = std::env::var_os("APPDATA") {
        let dir = std::path::PathBuf::from(dir).join("RconcreteFEM");
        if std::fs::create_dir_all(&dir).is_ok() {
            let _ = std::fs::write(dir.join("gmsh_path.txt"), path);
        }
    }
}
pub fn configured<T>(path: String, quads: bool, optimise: bool, run: impl FnOnce() -> T) -> T {
    struct Reset(Option<(String, bool, bool)>);
    impl Drop for Reset {
        fn drop(&mut self) {
            CONFIG.with(|c| *c.borrow_mut() = self.0.take());
        }
    }
    let _reset = Reset(CONFIG.with(|c| c.replace(Some((path, quads, optimise)))));
    run()
}
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Backend {
    #[default]
    Spade,
    Gmsh,
}
impl Backend {
    pub fn label(self) -> &'static str {
        match self {
            Self::Spade => "Spade",
            Self::Gmsh => "Gmsh",
        }
    }
}
#[cfg(test)]
static GMSH_RUNS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
pub struct Raw {
    pub quads: Vec<[usize; 4]>,
    pub points: BTreeMap<usize, Point>,
    pub triangles: Vec<[usize; 3]>,
    pub constraints: Vec<[usize; 2]>,
    pub source_nodes: BTreeMap<usize, usize>,
}
pub fn spade(cdt: &ConstrainedDelaunayTriangulation<Point2<f64>>) -> Raw {
    Raw {
        quads: vec![],
        source_nodes: BTreeMap::new(),
        points: cdt
            .vertices()
            .map(|v| (v.fix().index(), [v.position().x, v.position().y]))
            .collect(),
        triangles: cdt
            .inner_faces()
            .map(|f| f.vertices().map(|v| v.fix().index()))
            .collect(),
        constraints: cdt
            .undirected_edges()
            .filter(|e| e.is_constraint_edge())
            .map(|e| e.vertices().map(|v| v.fix().index()))
            .collect(),
    }
}
/// A common acceptance check keeps local mesh-size requirements backend-independent.
pub fn gmsh_refined(
    cdt: &mut ConstrainedDelaunayTriangulation<Point2<f64>>,
    size_at: impl Fn(Point) -> f64,
    max_size: f64,
    target_at: impl Fn(Point) -> Option<f64>,
) -> Result<Raw, String> {
    for pass in 0..12 {
        let raw = gmsh(cdt, |p| size_at(p) * 0.8, max_size * 0.8)?;
        let mut extra = Vec::new();
        for t in &raw.triangles {
            let p = t.map(|n| raw.points[&n]);
            let center = [
                (p[0][0] + p[1][0] + p[2][0]) / 3.,
                (p[0][1] + p[1][1] + p[2][1]) / 3.,
            ];
            let Some(target) = target_at(center) else {
                continue;
            };
            let area = ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                .abs()
                * 0.5;
            if area > target * 1.000001 {
                extra.push(center);
            }
        }
        if extra.is_empty() {
            return Ok(raw);
        }
        if pass == 11 || cdt.num_vertices() + extra.len() > crate::plan_trace::limits::MAX_NODES {
            return Err("Gmsh: не достигнут заданный размер КЭ; сетка отклонена".into());
        }
        for p in extra {
            cdt.insert(Point2::new(p[0], p[1]))
                .map_err(|e| format!("Уточнение Gmsh: {e:?}"))?;
        }
    }
    Err("Gmsh не завершил построение".into())
}
#[cfg(not(target_arch = "wasm32"))]
fn executable() -> Result<std::path::PathBuf, String> {
    use std::path::PathBuf;
    if let Some((path, _, _)) = CONFIG.with(|c| c.borrow().clone()) {
        let p = PathBuf::from(path);
        return p
            .is_file()
            .then_some(p)
            .ok_or("Укажите существующий gmsh.exe в настройках МКЭ-сетки".into());
    }
    #[cfg(not(test))]
    {
        let path = PathBuf::from(saved_path());
        return path
            .is_file()
            .then_some(path)
            .ok_or("Укажите путь к gmsh.exe в настройках МКЭ-сетки".into());
    }
    #[cfg(test)]
    {
        if let Some(p) = std::env::var_os("GMSH_EXE") {
            let p = PathBuf::from(p);
            return p
                .is_file()
                .then_some(p)
                .ok_or("Не найден файл из GMSH_EXE".into());
        }
        let mut candidates = Vec::new();
        if let Ok(p) = std::env::current_exe() {
            if let Some(p) = p.parent() {
                candidates.push(p.join("gmsh/gmsh.exe"));
                candidates.push(p.join("gmsh.exe"));
            }
        }
        candidates.push(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("dist/gmsh/gmsh.exe"));
        if let Some(path) = std::env::var_os("PATH") {
            candidates.extend(
                std::env::split_paths(&path)
                    .map(|p| p.join(if cfg!(windows) { "gmsh.exe" } else { "gmsh" })),
            );
        }
        candidates.into_iter().find(|p| p.is_file()).ok_or(
        "Gmsh не найден. Поместите gmsh.exe в папку gmsh рядом с приложением или задайте GMSH_EXE"
            .into(),
    )
    }
}
pub fn available() -> bool {
    !cfg!(target_arch = "wasm32")
}
#[cfg(target_arch = "wasm32")]
pub fn gmsh(
    _: &ConstrainedDelaunayTriangulation<Point2<f64>>,
    _: impl Fn(Point) -> f64,
    _: f64,
) -> Result<Raw, String> {
    Err("Gmsh доступен только в EXE; в web используйте Spade".into())
}
#[cfg(not(target_arch = "wasm32"))]
pub fn gmsh(
    cdt: &ConstrainedDelaunayTriangulation<Point2<f64>>,
    size_at: impl Fn(Point) -> f64,
    max_size: f64,
) -> Result<Raw, String> {
    use std::{
        fmt::Write,
        fs,
        process::{Command, Stdio},
        sync::atomic::{AtomicU64, Ordering},
        time::{Duration, Instant},
    };
    static NEXT: AtomicU64 = AtomicU64::new(0);
    use crate::plan_trace::fem::progress;
    progress::check_cancel()?;
    let exe = executable()?;
    #[cfg(test)]
    GMSH_RUNS.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    let seed = spade(cdt);
    let hull = cdt
        .convex_hull()
        .map(|e| [e.from().fix().index(), e.to().fix().index()])
        .collect::<Vec<_>>();
    if hull.len() < 3 {
        return Err("Gmsh: вырожденная область".into());
    }
    let mut edges = BTreeMap::<[usize; 2], (usize, [usize; 2])>::new();
    for e in seed.constraints.iter().chain(hull.iter()) {
        let mut key = *e;
        key.sort();
        let n = edges.len() + 1;
        edges.entry(key).or_insert((n, *e));
    }
    let mut geo=format!("Mesh.MshFileVersion=2.2; Mesh.Binary=0; Mesh.ElementOrder=1; Mesh.Algorithm=6; Mesh.RecombineAll=0; Mesh.MeshSizeFromCurvature=0; Mesh.MeshSizeExtendFromBoundary=0; Mesh.MeshSizeMax={max_size:.15}; Geometry.Tolerance=1e-9;\n");
    if let Some((_, quads, optimise)) = CONFIG.with(|c| c.borrow().clone()) {
        if quads {
            geo.push_str("Mesh.Algorithm=8; Mesh.RecombineAll=1; Mesh.RecombinationAlgorithm=1;\n");
        }
        if optimise {
            geo.push_str("Mesh.Smoothing=10;\n");
        }
    }
    for (&id, &p) in &seed.points {
        let h = size_at(p);
        if !h.is_finite() || h <= 0. {
            return Err("Gmsh: неверный шаг".into());
        }
        writeln!(
            geo,
            "Point({})={{{:.15},{:.15},0,{:.15}}};",
            id + 1,
            p[0],
            p[1],
            h
        )
        .unwrap();
    }
    for &(id, [a, b]) in edges.values() {
        writeln!(
            geo,
            "Line({id})={{{},{}}}; Transfinite Curve {{{id}}}=2;",
            a + 1,
            b + 1
        )
        .unwrap();
    }
    let loop_ids = hull
        .iter()
        .map(|e| {
            let mut key = *e;
            key.sort();
            let (id, dir) = edges[&key];
            if dir == *e {
                id.to_string()
            } else {
                format!("-{id}")
            }
        })
        .collect::<Vec<_>>()
        .join(",");
    writeln!(geo, "Curve Loop(1)={{{loop_ids}}}; Plane Surface(1)={{1}};").unwrap();
    let hull_keys = hull
        .iter()
        .map(|e| {
            let mut k = *e;
            k.sort();
            k
        })
        .collect::<BTreeSet<_>>();
    for (key, (id, _)) in &edges {
        if !hull_keys.contains(key) {
            writeln!(geo, "Curve {{{id}}} In Surface {{1}};").unwrap();
        }
    }
    let line_points = edges.keys().flatten().copied().collect::<BTreeSet<_>>();
    for id in seed.points.keys().filter(|id| !line_points.contains(id)) {
        writeln!(geo, "Point {{{}}} In Surface {{1}};", id + 1).unwrap();
    }
    let folder = std::env::temp_dir().join(format!(
        "rconcrete-gmsh-{}-{}",
        std::process::id(),
        NEXT.fetch_add(1, Ordering::Relaxed)
    ));
    fs::create_dir(&folder).map_err(|e| format!("Gmsh: временная папка: {e}"))?;
    struct Cleanup(std::path::PathBuf);
    impl Drop for Cleanup {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }
    let _cleanup = Cleanup(folder.clone());
    let input = folder.join("mesh.geo");
    let output = folder.join("mesh.msh");
    let log = folder.join("gmsh.log");
    fs::write(&input, geo).map_err(|e| e.to_string())?;
    let log_file = fs::File::create(&log).map_err(|e| e.to_string())?;
    let mut command = Command::new(exe);
    command
        .args(["-2", "-format", "msh2", "-v", "2", "-nt", "1"])
        .arg(&input)
        .arg("-o")
        .arg(&output)
        .stdout(Stdio::from(
            log_file.try_clone().map_err(|e| e.to_string())?,
        ))
        .stderr(Stdio::from(log_file));
    #[cfg(windows)]
    {
        use std::os::windows::process::CommandExt;
        command.creation_flags(0x08000000);
    }
    progress::check_cancel()?;
    let mut child = command.spawn().map_err(|e| format!("Запуск Gmsh: {e}"))?;
    let start = Instant::now();
    let result = (|| loop {
        progress::check_cancel()?;
        if let Some(status) = child
            .try_wait()
            .map_err(|e| format!("Ожидание Gmsh: {e}"))?
        {
            break Ok(status);
        }
        if start.elapsed() > Duration::from_secs(120) {
            break Err("Gmsh: превышено время 120 с на область".into());
        }
        std::thread::sleep(Duration::from_millis(20));
    })();
    let status = match result {
        Ok(status) => status,
        Err(error) => {
            let _ = child.kill();
            let _ = child.wait();
            return Err(error);
        }
    };
    if !status.success() {
        let log = fs::read_to_string(&log).unwrap_or_default();
        return Err(format!(
            "Gmsh завершился со статусом {status}: {}",
            log.chars().take(2400).collect::<String>()
        ));
    }
    let mut raw = parse_msh2(
        &fs::read_to_string(&output).map_err(|e| format!("Gmsh не создал сетку: {e}"))?,
    )?;
    // Every pre-sampled contact must stay one mesh edge, not just retain endpoints.
    let output_edges = raw
        .triangles
        .iter()
        .flat_map(|t| {
            (0..3).map(move |i| {
                let mut e = [t[i], t[(i + 1) % 3]];
                e.sort();
                e
            })
        })
        .collect::<BTreeSet<_>>();
    let mut located = BTreeMap::new();
    let mut buckets = BTreeMap::<(i64, i64), Vec<(usize, Point)>>::new();
    for (&id, &p) in &raw.points {
        buckets
            .entry(((p[0] / 1e-6).floor() as i64, (p[1] / 1e-6).floor() as i64))
            .or_default()
            .push((id, p));
    }
    for (&id, &p) in &seed.points {
        let key = ((p[0] / 1e-6).floor() as i64, (p[1] / 1e-6).floor() as i64);
        let found = (-1..=1)
            .flat_map(|dx| (-1..=1).map(move |dy| (key.0 + dx, key.1 + dy)))
            .filter_map(|k| buckets.get(&k))
            .flatten()
            .find(|(_, q)| (q[0] - p[0]).hypot(q[1] - p[1]) < 1e-6)
            .map(|(id, _)| *id);
        located.insert(id, found.ok_or("Gmsh потерял обязательный узел")?);
    }
    for [a, b] in &seed.constraints {
        let mut e = [located[a], located[b]];
        e.sort();
        if !output_edges.contains(&e) {
            return Err("Gmsh изменил фиксированное разбиение контакта; сетка отклонена".into());
        }
    }
    raw.source_nodes = located;
    Ok(raw)
}
fn parse_msh2(text: &str) -> Result<Raw, String> {
    let mut lines = text.lines();
    let mut points = BTreeMap::new();
    let mut triangles = vec![];
    let mut quads = vec![];
    let mut constraints = vec![];
    while let Some(line) = lines.next() {
        match line.trim() {
            "$Nodes" => {
                let n: usize = lines
                    .next()
                    .ok_or("Нет числа узлов")?
                    .trim()
                    .parse()
                    .map_err(|_| "Неверное число узлов")?;
                if n > crate::plan_trace::limits::MAX_NODES {
                    return Err("Gmsh: превышен лимит узлов".into());
                }
                for _ in 0..n {
                    let fields = lines
                        .next()
                        .ok_or("Оборвана запись узла")?
                        .split_whitespace()
                        .collect::<Vec<_>>();
                    if fields.len() != 4 {
                        return Err("Неверный узел Gmsh".into());
                    }
                    let id = fields[0].parse::<usize>().map_err(|_| "Неверный ID узла")?;
                    let p = [fields[1], fields[2]].map(|v| v.parse::<f64>().unwrap_or(f64::NAN));
                    if !p.into_iter().all(f64::is_finite) || points.insert(id, p).is_some() {
                        return Err("Неверные координаты/ID Gmsh".into());
                    }
                }
            }
            "$Elements" => {
                let n: usize = lines
                    .next()
                    .ok_or("Нет числа КЭ")?
                    .trim()
                    .parse()
                    .map_err(|_| "Неверное число КЭ")?;
                for _ in 0..n {
                    let v = lines
                        .next()
                        .ok_or("Оборван КЭ")?
                        .split_whitespace()
                        .map(str::parse::<usize>)
                        .collect::<Result<Vec<_>, _>>()
                        .map_err(|_| "Неверный КЭ Gmsh")?;
                    if v.len() < 3 || v.len() < 3 + v[2] {
                        return Err("Неверные теги КЭ".into());
                    }
                    let ns = &v[3 + v[2]..];
                    match (v[1], ns) {
                        (1, [a, b]) => constraints.push([*a, *b]),
                        (2, [a, b, c]) => triangles.push([*a, *b, *c]),
                        (3, [a, b, c, d]) => {
                            quads.push([*a, *b, *c, *d]);
                            triangles.extend([[*a, *b, *c], [*a, *c, *d]]);
                        }
                        (15, [_]) => {}
                        _ => return Err("Gmsh вернул неподдержанный тип КЭ".into()),
                    }
                    if triangles.len().saturating_sub(quads.len())
                        > crate::plan_trace::limits::MAX_ELEMENTS
                    {
                        return Err("Gmsh: превышен лимит КЭ".into());
                    }
                }
            }
            _ => {}
        }
    }
    if triangles.is_empty() || triangles.iter().flatten().any(|n| !points.contains_key(n)) {
        return Err("Gmsh: пустая или повреждённая сетка".into());
    }
    for q in &mut quads {
        let signed = (0..4)
            .map(|i| {
                let a = points[&q[i]];
                let b = points[&q[(i + 1) % 4]];
                a[0] * b[1] - b[0] * a[1]
            })
            .sum::<f64>();
        if signed < 0. {
            q.swap(1, 3);
        }
        for i in 0..4 {
            let a = points[&q[i]];
            let b = points[&q[(i + 1) % 4]];
            let c = points[&q[(i + 2) % 4]];
            if (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]) <= 1e-10 {
                return Err("Gmsh: вырожденный или невыпуклый четырёхугольник".into());
            }
        }
    }
    Ok(Raw {
        quads,
        points,
        triangles,
        constraints,
        source_nodes: BTreeMap::new(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn msh_quad_is_preserved_as_a_physical_cell_with_display_triangles() {
        let raw=parse_msh2("$Nodes\n4\n1 0 0 0\n2 1 0 0\n3 1 1 0\n4 0 1 0\n$EndNodes\n$Elements\n1\n1 3 0 1 2 3 4\n$EndElements\n").unwrap();
        assert_eq!(raw.quads, vec![[1, 2, 3, 4]]);
        assert_eq!(raw.triangles, vec![[1, 2, 3], [1, 3, 4]]);
    }
    #[test]
    #[ignore = "requires native gmsh.exe"]
    fn gmsh_native_quads_survive_plate_wall_scene_pipeline() {
        let path = executable().unwrap().to_string_lossy().into_owned();
        configured(path, true, true, || {
            let all = vec![
                c(
                    1,
                    "Slab",
                    None,
                    vec![[0., 0.], [3000., 0.], [3000., 3000.], [0., 3000.]],
                ),
                c(2, "Support", None, vec![[1500., 0.], [1500., 3000.]]),
            ];
            let layout = crate::plan_trace::building_mesh::Layout {
                levels: vec![crate::plan_trace::building_mesh::Level {
                    id: 1,
                    z: 0.,
                    top: 3000.,
                }],
                owners: BTreeMap::from([(1, 1), (2, 1)]),
                legacy_wall_extent: None,
            };
            let mesh = crate::plan_trace::building_mesh::generate(
                &all,
                500.,
                layout,
                crate::plan_trace::quad_mesh::Options {
                    backend: Backend::Gmsh,
                    merge: true,
                    optimise: true,
                },
            )
            .unwrap();
            assert!(!mesh.quads.is_empty());
            assert!(!mesh.building.as_ref().unwrap().wall_quads.is_empty());
            let runs = GMSH_RUNS.load(std::sync::atomic::Ordering::Relaxed);
            let (scene, _) = crate::plan_trace::fem::lira_model(
                &mesh,
                &all,
                &BTreeMap::from([(1, crate::plan_trace::fem::element::Material::default())]),
                &[],
                10.,
                500.,
                &Default::default(),
            )
            .unwrap();
            assert!(scene
                .surfaces
                .iter()
                .any(|s| s.support == Some(2) && s.nodes.len() == 4));
            assert_eq!(GMSH_RUNS.load(std::sync::atomic::Ordering::Relaxed), runs);
        });
    }
    fn c(
        id: u64,
        kind: &str,
        parent: Option<u64>,
        points: Vec<Point>,
    ) -> crate::plan_trace::model::Contour {
        serde_json::from_value(serde_json::json!({"id":id,"kind":kind,"parent":parent,"points":points,"support_open":kind=="Support","load":null,"refinement":null,"curves":[]})).unwrap()
    }
    #[test]
    fn msh_parser_rejects_dangling_nodes_and_preserves_tags() {
        let text="$MeshFormat\n2.2 0 8\n$EndMeshFormat\n$Nodes\n3\n10 0 0 0\n20 1 0 0\n30 0 1 0\n$EndNodes\n$Elements\n2\n1 1 2 0 1 10 20\n2 2 2 0 1 10 20 30\n$EndElements\n";
        let raw = parse_msh2(text).unwrap();
        assert_eq!(raw.triangles, vec![[10, 20, 30]]);
        assert_eq!(raw.constraints, vec![[10, 20]]);
        assert!(parse_msh2(&text.replace("10 20 30", "10 20 99")).is_err());
        assert_eq!(
            serde_json::from_str::<crate::plan_trace::quad_mesh::Options>(
                "{\"merge\":true,\"optimise\":false}"
            )
            .unwrap()
            .backend,
            Backend::Spade
        );
    }
    #[test]
    #[ignore = "requires the official native Gmsh executable"]
    fn gmsh_real_hole_and_contact_keep_area_and_shared_nodes() {
        let all = vec![
            c(
                1,
                "Slab",
                None,
                vec![[0., 0.], [6000., 0.], [6000., 4000.], [0., 4000.]],
            ),
            c(
                2,
                "Hole",
                Some(1),
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
            c(3, "Support", None, vec![[3000., 0.], [3000., 4000.]]),
        ];
        let mesh = crate::plan_trace::meshing::generate_backend(&all, 500., Backend::Gmsh).unwrap();
        let area: f64 = mesh
            .elements
            .iter()
            .map(|e| {
                let p = e.nodes.map(|n| mesh.nodes[n].xy);
                ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                    .abs()
                    / 2.
            })
            .sum();
        assert!((area - 23_000_000.).abs() < 0.1, "area={area}");
        assert!(mesh.support_boundary[&3].len() >= 9);
        let max_area = mesh
            .elements
            .iter()
            .map(|e| {
                let p = e.nodes.map(|n| mesh.nodes[n].xy);
                ((p[1][0] - p[0][0]) * (p[2][1] - p[0][1])
                    - (p[1][1] - p[0][1]) * (p[2][0] - p[0][0]))
                    .abs()
                    / 2.
            })
            .fold(0., f64::max);
        assert!(max_area <= 3_f64.sqrt() * 0.25 * 500. * 500. * 1.000001);
        println!(
            "GMSH hole: {} nodes {} triangles max_area={max_area}",
            mesh.nodes.len(),
            mesh.elements.len()
        );
        let layout = crate::plan_trace::building_mesh::Layout {
            levels: vec![crate::plan_trace::building_mesh::Level {
                id: 1,
                z: 0.,
                top: 3000.,
            }],
            owners: BTreeMap::from([(1, 1), (2, 1), (3, 1)]),
            legacy_wall_extent: None,
        };
        let mesh = crate::plan_trace::building_mesh::generate(
            &all,
            500.,
            layout,
            crate::plan_trace::quad_mesh::Options {
                backend: Backend::Gmsh,
                merge: true,
                optimise: true,
            },
        )
        .unwrap();
        let materials = BTreeMap::from([(1, crate::plan_trace::fem::element::Material::default())]);
        assert!(mesh.building.as_ref().unwrap().wall_mesh.is_some());
        let runs = GMSH_RUNS.load(std::sync::atomic::Ordering::Relaxed);
        let (scene, _) = crate::plan_trace::fem::lira_model(
            &mesh,
            &all,
            &materials,
            &[],
            10.,
            500.,
            &Default::default(),
        )
        .unwrap();
        assert!(scene.surfaces.iter().any(|s| s.support == Some(3)));
        assert_eq!(
            GMSH_RUNS.load(std::sync::atomic::Ordering::Relaxed),
            runs,
            "Scene preparation must not relaunch Gmsh"
        );
        println!(
            "GMSH building: {} surfaces, {} wall surfaces",
            scene.surfaces.len(),
            scene
                .surfaces
                .iter()
                .filter(|s| s.support.is_some())
                .count()
        );
    }
}
