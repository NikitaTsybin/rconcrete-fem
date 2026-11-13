use super::*;
use std::collections::{BTreeMap, BTreeSet};
pub(super) struct Dialog {
    id: u64,
    layers: BTreeMap<String, Option<Kind>>,
    tolerance: f64,
    message: String,
}
/// Split at branches; never guess a closed boundary through a T-junction.
fn chains(paths: &[Vec<Point>], tolerance: f64) -> Vec<(Vec<Point>, bool)> {
    let mut points: Vec<Point> = vec![];
    let mut buckets = BTreeMap::<(i64, i64), Vec<usize>>::new();
    let mut edges = BTreeSet::new();
    let mut node = |p: Point| {
        let key = (
            (p[0] / tolerance).floor() as i64,
            (p[1] / tolerance).floor() as i64,
        );
        for dx in -1..=1 {
            for dy in -1..=1 {
                if let Some(ids) = buckets.get(&(key.0 + dx, key.1 + dy)) {
                    for &i in ids {
                        if (points[i][0] - p[0]).hypot(points[i][1] - p[1]) <= tolerance {
                            return i;
                        }
                    }
                }
            }
        }
        let id = points.len();
        points.push(p);
        buckets.entry(key).or_default().push(id);
        id
    };
    for path in paths {
        for p in path.windows(2) {
            let a = node(p[0]);
            let b = node(p[1]);
            if a != b {
                edges.insert([a.min(b), a.max(b)]);
            }
        }
    }
    let mut adjacency = vec![vec![]; points.len()];
    for &[a, b] in &edges {
        adjacency[a].push(b);
        adjacency[b].push(a);
    }
    let mut out = vec![];
    while !edges.is_empty() {
        let start = edges.first().unwrap()[0];
        let mut at = start;
        let mut ids = vec![start];
        let mut closed = false;
        loop {
            let Some(next) = adjacency[at]
                .iter()
                .copied()
                .find(|&n| edges.contains(&[at.min(n), at.max(n)]))
            else {
                break;
            };
            edges.remove(&[at.min(next), at.max(next)]);
            at = next;
            if at == start {
                closed = true;
                break;
            }
            ids.push(at);
            if adjacency[at].len() != 2 {
                break;
            }
        }
        out.push((ids.into_iter().map(|i| points[i]).collect(), closed));
    }
    out
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn transformed_dxf_creation_assigns_hole_and_is_one_undo() {
        let d=import::from_bytes(b"0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\nslab\n70\n1\n10\n0\n20\n0\n10\n1000\n20\n0\n10\n1000\n20\n1000\n10\n0\n20\n1000\n0\nLWPOLYLINE\n8\nhole\n70\n1\n10\n100\n20\n100\n10\n200\n20\n100\n10\n200\n20\n200\n10\n100\n20\n200\n0\nENDSEC\n0\nEOF\n","fixture").unwrap();
        let mut a = App::default();
        a.underlays.push(crate::plan_trace::underlay::Underlay {
            id: 1,
            name: "fixture".into(),
            visible: true,
            transparency: 0.,
            scale: 2.,
            scale_y: Some(3.),
            angle_rad: std::f64::consts::FRAC_PI_2,
            origin: [500., 800.],
            data: crate::plan_trace::underlay::Data::Dxf(d),
        });
        let dialog = Dialog {
            id: 1,
            layers: BTreeMap::from([
                ("slab".into(), Some(Kind::Slab)),
                ("hole".into(), Some(Kind::Hole)),
            ]),
            tolerance: 0.1,
            message: String::new(),
        };
        a.create_dxf_contours(&dialog).unwrap();
        assert_eq!(a.contours.len(), 2);
        assert_eq!(a.contours[1].parent, Some(a.contours[0].id));
        assert!(a.contours[0].points.contains(&[500., 800.]));
        assert!((geometry::area(&a.contours[0].points).abs() - 6_000_000.).abs() < 1e-6);
        assert_eq!(a.history.len(), 1);
        a.undo();
        assert!(a.contours.is_empty());
        assert_eq!(a.underlays.len(), 1);
    }
    #[test]
    fn load_fill_excludes_slab_holes_and_outside_but_preserves_source() {
        use mini_fem::geo::Area;
        let mut a = App::default();
        for (kind, p, q) in [
            (Kind::Slab, [0., 0.], [1000., 1000.]),
            (Kind::Hole, [100., 100.], [200., 200.]),
            (Kind::LoadArea, [-100., -100.], [1100., 1100.]),
        ] {
            a.begin(kind);
            a.choose_tool(Tool::Rectangle);
            a.sketch_click(p, false);
            a.sketch_click(q, false);
        }
        let source = a.contours.clone();
        let load = a
            .contours
            .iter()
            .find(|c| c.kind == Kind::LoadArea)
            .unwrap();
        assert!(
            (model::visible_load_area(&a.contours, load).unsigned_area() - 990000.).abs() < 1e-5
        );
        assert_eq!(source, a.contours);
    }
    #[test]
    fn unordered_reversed_duplicate_lines_make_one_ring_and_gaps_remain_open() {
        let p = vec![
            vec![[10., 10.], [0., 10.]],
            vec![[0., 0.], [10., 0.]],
            vec![[10., 10.], [10., 0.]],
            vec![[0., 10.], [0., 0.]],
            vec![[10., 0.], [0., 0.]],
        ];
        let c = chains(&p, 0.1);
        assert_eq!(c.len(), 1);
        assert!(c[0].1);
        assert_eq!(c[0].0.len(), 4);
        assert!(chains(&p[..3], 0.1).iter().all(|(_, closed)| !closed));
    }
}
impl App {
    pub(super) fn dxf_contours_menu(&mut self, ui: &mut egui::Ui) {
        let Some(id) = self.dxf_menu_target.or(self.selected_underlay) else {
            return;
        };
        let Some(u) = self.underlays.iter().find(|u| u.id == id) else {
            return;
        };
        let crate::plan_trace::underlay::Data::Dxf(d) = &u.data else {
            return;
        };
        if ui
            .add_enabled(
                !self.input_locked && self.mode.is_none() && self.mesh_job.is_none(),
                egui::Button::new("Создать объекты на основе примитивов…"),
            )
            .clicked()
        {
            self.dxf_contours = Some(Dialog {
                id,
                layers: d.paths.iter().map(|p| (p.layer.clone(), None)).collect(),
                tolerance: 0.1,
                message: String::new(),
            });
            ui.close_menu();
        }
    }
    pub(super) fn dxf_contours_dialog(&mut self, ctx: &egui::Context) {
        let Some(mut dialog) = self.dxf_contours.take() else {
            return;
        };
        let mut open = true;
        let mut create = false;
        egui::Window::new("Объекты из DXF").open(&mut open).default_width(500.).resizable(true).show(ctx,|ui|{
            ui.horizontal(|ui|{ui.label("Допуск соединения, мм");ui.add(egui::DragValue::new(&mut dialog.tolerance).range(0.000001..=100.).speed(0.1));});
            egui::ScrollArea::vertical().max_height(420.).show(ui,|ui|{egui::Grid::new("dxf_layer_roles").striped(true).show(ui,|ui|{
                ui.strong("Слой");ui.strong("Использовать как");ui.end_row();
                for(name,role)in &mut dialog.layers{ui.label(name);egui::ComboBox::from_id_salt(("dxf_role",name)).selected_text(role.map_or("—",Kind::label)).show_ui(ui,|ui|{ui.selectable_value(role,None,"—");for kind in [Kind::Slab,Kind::Hole,Kind::Support,Kind::Beam,Kind::Triangulation,Kind::Auxiliary,Kind::LoadLine,Kind::LoadArea]{ui.selectable_value(role,Some(kind),kind.label());}});ui.end_row();}
            });});
            ui.small("Незамкнутые и разветвлённые цепочки не становятся плитами или отверстиями. Опоры могут быть открытыми. Нагрузки — в активном загружении с текущей интенсивностью.");
            ui.label(&dialog.message);create=ui.add_enabled(!self.input_locked&&dialog.layers.values().any(Option::is_some),egui::Button::new("Создать")).clicked();
        });
        if create {
            match self.create_dxf_contours(&dialog) {
                Ok(message) => {
                    self.status = message;
                    open = false;
                }
                Err(e) => dialog.message = e,
            }
        }
        if open {
            self.dxf_contours = Some(dialog);
        }
    }
    fn create_dxf_contours(&mut self, dialog: &Dialog) -> Result<String, String> {
        if self.input_locked {
            return Err("Исходные данные заблокированы".into());
        }
        let u = self
            .underlays
            .iter()
            .find(|u| u.id == dialog.id)
            .ok_or("Подложка удалена")?;
        let crate::plan_trace::underlay::Data::Dxf(d) = &u.data else {
            return Err("Требуется DXF".into());
        };
        let mut pending = vec![];
        let mut skipped = 0;
        for (layer, role) in &dialog.layers {
            let Some(kind) = *role else {
                continue;
            };
            if self.is_kind_locked(kind) {
                return Err(format!("{} заблокирован", kind.label()));
            }
            let paths = d
                .paths
                .iter()
                .filter(|p| p.layer == *layer)
                .map(|p| {
                    let mut v = p.points.iter().map(|&p| u.point(p)).collect::<Vec<_>>();
                    if p.closed && v.len() > 1 {
                        v.push(v[0]);
                    }
                    v
                })
                .collect::<Vec<_>>();
            for (points, closed) in chains(&paths, dialog.tolerance) {
                if matches!(kind, Kind::Slab | Kind::Hole | Kind::LoadArea) && !closed {
                    skipped += 1;
                    continue;
                }
                pending.push((kind, points, closed));
            }
        }
        pending.sort_by_key(|(k, _, _)| match k {
            Kind::Slab => 0,
            Kind::Hole => 2,
            _ => 1,
        });
        let mut all = self.contours.clone();
        let start = all.len();
        let mut next = self
            .next_id
            .max(all.iter().map(|c| c.id).max().unwrap_or(0) + 1);
        for (kind, points, closed) in pending {
            let mut c = Contour {
                name: String::new(),
                placement: Default::default(),
                mesh_size_mm: 0.,
                column_angle: 0.,
                calculate_effective_length: false,
                restraint_mask: 4,
                use_foundation: false,
                foundation: None,
                id: next,
                kind,
                parent: None,
                points,
                curves: vec![],
                grid: None,
                support_open: matches!(
                    kind,
                    Kind::Support | Kind::Auxiliary | Kind::Beam | Kind::Triangulation
                ) && !closed,
                refinement: None,
                load: kind
                    .is_load()
                    .then_some(crate::plan_trace::loads::LoadData {
                        direction: Default::default(),
                        moment: false,
                        additive: false,
                        name: String::new(),
                        mesh_constraint: false,
                        case_id: self.active_load,
                        magnitude: self.new_load,
                        closed,
                    }),
            };
            if c.rebuild().is_err() {
                skipped += 1;
                continue;
            }
            if kind == Kind::Hole {
                let parent = all
                    .iter()
                    .filter(|p| p.kind == Kind::Slab)
                    .filter_map(|p| {
                        let mut probe = c.clone();
                        probe.parent = Some(p.id);
                        model::validate_candidate(&all, &probe)
                            .is_ok()
                            .then_some((p.id, geometry::area(&p.points)))
                    })
                    .min_by(|a, b| a.1.total_cmp(&b.1))
                    .map(|p| p.0);
                let Some(parent) = parent else {
                    skipped += 1;
                    continue;
                };
                c.parent = Some(parent);
            }
            if model::validate_candidate(&all, &c).is_err() {
                skipped += 1;
                continue;
            }
            all.push(c);
            next = next.checked_add(1).ok_or("Исчерпаны ID")?;
        }
        let count = all.len() - start;
        if count == 0 {
            return Err(format!("Объекты не созданы. Не замкнуты или некорректны: {skipped}. Проверьте назначения слоёв и допуск."));
        }
        self.history.push(self.contours.clone());
        self.contours = all;
        self.next_id = next;
        self.clear_mesh();
        self.slab_cache = None;
        self.sync_stiffness_sources();
        Ok(format!(
            "Создано объектов: {count}. Пропущено цепочек: {skipped}. Ctrl+Z — отменить создание."
        ))
    }
}
