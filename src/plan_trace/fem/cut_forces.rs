//! Element-side cut actions; coordinates mm, force kN, couples kN m.
use super::*;
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub enum Method {
    #[default]
    NodalEquilibrium,
    ElementEdges,
}
impl Method {
    pub fn label(self) -> &'static str {
        match self {
            Self::NodalEquilibrium => "Равновесие узлов",
            Self::ElementEdges => "Согласованные усилия граней",
        }
    }
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct EdgeForce {
    pub index: usize,
    pub ends: [Point; 2],
    pub force: f64,
    /// Moments about the fixed global origin, kN m.
    pub mx_origin: f64,
    pub my_origin: f64,
    pub active: bool,
    pub diagnostic: String,
}
#[derive(Clone, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct CutResult {
    pub method: Method,
    pub patch_edges: Vec<EdgeForce>,
    pub direct_edges: Vec<EdgeForce>,
    pub base_diagnostic: String,
    pub edge_forces: Vec<EdgeForce>,
    #[cfg(test)]
    pub closing_force: f64,
    #[cfg(test)]
    pub inner_force: f64,
    #[cfg(test)]
    pub full_net_force: f64,
    pub center: Point,
    pub force: f64,
    pub mx: f64,
    pub my: f64,
    pub nodes: usize,
    pub elements: usize,
    pub diagnostic: String,
}
impl CutResult {
    pub fn select_method(&mut self, method: Method) -> bool {
        if self.method == method {
            return false;
        }
        self.method = method;
        if self.patch_edges.is_empty() && self.direct_edges.is_empty() {
            return true;
        }
        self.edge_forces = match method {
            Method::NodalEquilibrium => self.patch_edges.clone(),
            Method::ElementEdges => self.direct_edges.clone(),
        };
        self.sum_active_edges();
        true
    }
    pub fn select_edges(&mut self, inactive: &[usize]) -> bool {
        let changed = self
            .edge_forces
            .iter()
            .any(|e| e.active == inactive.contains(&e.index));
        if changed {
            for e in &mut self.edge_forces {
                e.active = !inactive.contains(&e.index);
            }
            self.sum_active_edges();
        }
        changed
    }
    pub fn sum_active_edges(&mut self) {
        if self.edge_forces.is_empty() {
            return;
        }
        self.center = [0.; 2];
        self.force = 0.;
        self.mx = 0.;
        self.my = 0.;
        self.diagnostic = self.base_diagnostic.clone();
        let mut length = 0.;
        for e in self.edge_forces.iter().filter(|e| e.active) {
            let [a, b] = e.ends;
            let l = (b[0] - a[0]).hypot(b[1] - a[1]);
            length += l;
            for d in 0..2 {
                self.center[d] += (a[d] + b[d]) * 0.5 * l;
            }
            self.force += e.force;
            self.mx += e.mx_origin;
            self.my += e.my_origin;
            if !e.diagnostic.is_empty() {
                self.diagnostic = e.diagnostic.clone();
            }
        }
        if length > 0. {
            for v in &mut self.center {
                *v /= length;
            }
        }
        self.mx += self.center[1] * self.force / 1000.;
        self.my -= self.center[0] * self.force / 1000.;
    }
}
#[derive(Default)]
struct Trace {
    enabled: bool,
    active: usize,
    loads: Vec<BTreeMap<usize, f64>>,
}
thread_local! {static TRACE:std::cell::RefCell<Trace>=std::cell::RefCell::new(Trace::default());}
pub(super) fn begin(all: &[Contour], count: usize) {
    TRACE.with(|t| {
        *t.borrow_mut() = Trace {
            enabled: all.iter().any(is_cut),
            active: 0,
            loads: vec![BTreeMap::new(); count],
        }
    });
}
pub(super) fn active(i: usize) {
    TRACE.with(|t| t.borrow_mut().active = i);
}
pub(super) fn record_interface_loads() {
    TRACE.with(|t| t.borrow_mut().enabled = true);
}
pub(super) fn load(d: usize, v: f64) {
    TRACE.with(|t| {
        let mut t = t.borrow_mut();
        let i = t.active;
        if t.enabled && i < t.loads.len() {
            *t.loads[i].entry(d).or_default() += v;
        }
    });
}
pub(super) fn net(i: usize, d: usize, ku: f64) -> f64 {
    TRACE.with(|t| {
        ku - t
            .borrow()
            .loads
            .get(i)
            .and_then(|l| l.get(&d))
            .copied()
            .unwrap_or(0.)
    })
}
#[cfg(test)]
pub(super) fn audit_rhs(rhs: &[f64]) {
    TRACE.with(|t| {
        let t = t.borrow();
        if !t.enabled {
            return;
        }
        let mut sum = vec![0.; rhs.len()];
        for cell in &t.loads {
            for (&d, &v) in cell {
                sum[d] += v;
            }
        }
        let scale = rhs.iter().fold(1.0_f64, |a, b| a.max(b.abs()));
        for (d, (&a, &b)) in sum.iter().zip(rhs).enumerate() {
            assert!(
                (a - b).abs() < 1e-10 * scale,
                "element load scatter mismatch DOF {d}: recorded={a}, assembled={b}"
            );
        }
    });
}
pub(super) fn plate_node_action(r: [f64; 3]) -> [f64; 3] {
    [r[0], -r[2], r[1]]
}
pub(crate) fn is_cut(c: &Contour) -> bool {
    c.kind == Kind::Punching
        || c.kind == Kind::Auxiliary
            && c.refinement
                .as_ref()
                .is_some_and(|r| r.name.starts_with("Продавливание: торец"))
}
// AD closes the FE selection domain but is not a punching control segment.
// Recover AD geometrically from its crossing with the originating finite wall axis;
// polygon clipping may reorder exterior vertices, so last->first is not reliable.
pub(crate) fn control_edges(c: &Contour, all: &[Contour]) -> Vec<[Point; 2]> {
    let wall_id = c
        .refinement
        .as_ref()
        .and_then(|r| r.name.split_whitespace().nth(3))
        .and_then(|v| v.parse::<u64>().ok());
    let wall = wall_id.and_then(|id| all.iter().find(|w| w.id == id && w.kind == Kind::Support));
    c.points
        .iter()
        .copied()
        .zip(c.points.iter().copied().cycle().skip(1))
        .take(c.points.len())
        .enumerate()
        .filter_map(|(index, (a, b))| {
            if c.kind == Kind::Punching {
                return (!c
                    .refinement
                    .as_ref()
                    .is_some_and(|r| r.inactive_edges.contains(&index)))
                .then_some([a, b]);
            }
            let closing = wall.is_some_and(|wall| {
                wall.points.windows(2).any(|w| {
                    let d = [b[0] - a[0], b[1] - a[1]];
                    let e = [w[1][0] - w[0][0], w[1][1] - w[0][1]];
                    let cross = |a: Point, b: Point| a[0] * b[1] - a[1] * b[0];
                    let den = cross(d, e);
                    if den.abs() < 1e-12 {
                        return false;
                    }
                    let q = [w[0][0] - a[0], w[0][1] - a[1]];
                    let t = cross(q, e) / den;
                    let u = cross(q, d) / den;
                    t > 1e-8 && t < 1. - 1e-8 && (-1e-8..=1. + 1e-8).contains(&u)
                })
            });
            (!closing).then_some([a, b])
        })
        .collect()
}
fn boundary(p: Point, poly: &[Point]) -> bool {
    poly.iter()
        .copied()
        .zip(poly.iter().copied().cycle().skip(1))
        .take(poly.len())
        .any(|(a, b)| {
            let d = [b[0] - a[0], b[1] - a[1]];
            let l = d[0] * d[0] + d[1] * d[1];
            if l < 1e-16 {
                return false;
            }
            let t = ((p[0] - a[0]) * d[0] + (p[1] - a[1]) * d[1]) / l;
            (-1e-8..=1. + 1e-8).contains(&t)
                && ((p[0] - a[0]) * d[1] - (p[1] - a[1]) * d[0]).abs() / l.sqrt() < 1e-4
        })
}
pub(super) struct Collector {
    cells: Vec<super::edge_actions::Cell>,
    direct: Vec<Option<Result<Vec<[f64; 3]>, String>>>,
    face_reference: Vec<Vec<[[f64; 3]; 2]>>,
    edge_regions: BTreeSet<u64>,
    regions: Vec<(
        u64,
        Vec<Point>,
        Vec<[Point; 2]>,
        CutResult,
        BTreeSet<(i64, i64)>,
    )>,
}
impl Collector {
    pub fn new(all: &[Contour]) -> Self {
        Self {
            cells: vec![],
            direct: vec![],
            face_reference: vec![],
            edge_regions: all
                .iter()
                .filter(|c| c.kind == Kind::Punching)
                .map(|c| c.id)
                .collect(),
            regions: all
                .iter()
                .filter(|c| is_cut(c))
                .map(|c| {
                    let mut center = [0.; 2];
                    let mut length = 0.;
                    let edges = control_edges(c, all);
                    for [a, b] in &edges {
                        let l = (b[0] - a[0]).hypot(b[1] - a[1]);
                        length += l;
                        for j in 0..2 {
                            center[j] += (a[j] + b[j]) * 0.5 * l;
                        }
                    }
                    if length > 0. {
                        for v in &mut center {
                            *v /= length;
                        }
                    }
                    (
                        c.id,
                        c.points.clone(),
                        edges.clone(),
                        CutResult {
                            edge_forces: c
                                .points
                                .iter()
                                .copied()
                                .zip(c.points.iter().copied().cycle().skip(1))
                                .take(c.points.len())
                                .enumerate()
                                .filter_map(|(index, (a, b))| {
                                    (c.kind == Kind::Punching).then_some(EdgeForce {
                                        index,
                                        ends: [a, b],
                                        force: 0.,
                                        mx_origin: 0.,
                                        my_origin: 0.,
                                        active: edges.contains(&[a, b]),
                                        diagnostic: String::new(),
                                    })
                                })
                                .collect(),
                            center,
                            diagnostic: if c.closed() {
                                String::new()
                            } else {
                                "Граница не замкнута. Повторите генерацию".into()
                            },
                            ..Default::default()
                        },
                        BTreeSet::new(),
                    )
                })
                .collect(),
        }
    }
    pub fn add(&mut self, poly: &[Point], actions: impl IntoIterator<Item = (Point, [f64; 3])>) {
        let actions: Vec<_> = actions.into_iter().collect();
        self.direct.push(None);
        self.face_reference.push(vec![]);
        self.cells.push(super::edge_actions::Cell {
            poly: poly.to_vec(),
            actions: actions.clone(),
            field: [0.; 5],
            kirchhoff: false,
            quadratic: false,
        });
        for (_, region, edges, r, nodes) in &mut self.regions {
            if !r.diagnostic.is_empty() {
                continue;
            }
            let center = [
                poly.iter().map(|p| p[0]).sum::<f64>() / poly.len() as f64,
                poly.iter().map(|p| p[1]).sum::<f64>() / poly.len() as f64,
            ];
            if !super::super::geometry::inside(center, region) {
                continue;
            }
            if poly
                .iter()
                .any(|p| !boundary(*p, region) && !super::super::geometry::inside(*p, region))
            {
                r.diagnostic = "КЭ пересекает границу. Перестройте сетку".into();
                continue;
            }
            r.elements += 1;
            #[cfg(test)]
            {
                r.full_net_force += actions.iter().map(|(_, v)| v[0]).sum::<f64>();
            }
            for (p, v) in &actions {
                if edges.iter().any(|[a, b]| boundary(*p, &[*a, *b])) {
                    r.force += v[0];
                    r.mx += v[1] - (p[1] - r.center[1]) * v[0] / 1000.;
                    r.my += v[2] + (p[0] - r.center[0]) * v[0] / 1000.;
                    nodes.insert(((p[0] * 1e5).round() as i64, (p[1] * 1e5).round() as i64));
                } else {
                    #[cfg(test)]
                    {
                        if boundary(*p, region) {
                            r.closing_force += v[0];
                        } else {
                            r.inner_force += v[0];
                        }
                    }
                }
            }
        }
    }
    pub fn direct_fields(
        &mut self,
        mut sample: impl FnMut(Point) -> Option<([f64; 3], [f64; 2])>,
        kirchhoff: bool,
    ) {
        if let (Some(cell), Some(out)) = (self.cells.last(), self.direct.last_mut()) {
            let touches = self.regions.iter().any(|(_, _, _, r, _)| {
                r.edge_forces
                    .iter()
                    .any(|edge| cell.poly.iter().any(|&p| boundary(p, &edge.ends)))
            });
            if !touches {
                return;
            }
            if let Ok(reference) = super::line_forces::nodal_reference(
                &cell.poly,
                &mut sample,
                kirchhoff,
                cell.quadratic,
            ) {
                if let Some(out) = self.face_reference.last_mut() {
                    *out = reference;
                }
            }
            *out = Some(super::line_forces::integrate(&cell.poly, sample, kirchhoff));
        }
    }
    pub fn quadratic(&mut self) {
        if let Some(c) = self.cells.last_mut() {
            c.quadratic = true;
        }
    }
    pub fn kirchhoff(&mut self) {
        if let Some(c) = self.cells.last_mut() {
            c.kirchhoff = true;
        }
    }
    pub fn field(&mut self, moments: [f64; 3], shear: [f64; 2]) {
        if let Some(c) = self.cells.last_mut() {
            c.field = [moments[0], moments[1], moments[2], shear[0], shear[1]];
        }
    }
    pub fn enabled(&self) -> bool {
        !self.regions.is_empty()
    }
    pub fn finish(mut self) -> BTreeMap<u64, CutResult> {
        if !self.edge_regions.is_empty() {
            let recovery = super::edge_actions::Recovery::nodal_sources(&self.cells);
            let faces = super::edge_actions::Recovery::faces(&self.cells, &self.face_reference);
            for (id, region, edges, r, nodes) in &mut self.regions {
                r.base_diagnostic = r.diagnostic.clone();
                if !self.edge_regions.contains(id) || !r.diagnostic.is_empty() {
                    continue;
                }
                r.force = 0.;
                for edge in &mut r.edge_forces {
                    edge.force = 0.;
                    edge.mx_origin = 0.;
                    edge.my_origin = 0.;
                    edge.diagnostic.clear();
                }
                r.mx = 0.;
                r.my = 0.;
                nodes.clear();
                r.direct_edges = r.edge_forces.clone();
                for edge in &mut r.direct_edges {
                    edge.diagnostic.clear();
                }
                for (ci, cell) in self.cells.iter().enumerate() {
                    let center = [
                        cell.poly.iter().map(|p| p[0]).sum::<f64>() / cell.poly.len() as f64,
                        cell.poly.iter().map(|p| p[1]).sum::<f64>() / cell.poly.len() as f64,
                    ];
                    if !super::super::geometry::inside(center, region) {
                        continue;
                    }
                    for ei in 0..cell.poly.len() {
                        let a = cell.poly[ei];
                        let b = cell.poly[(ei + 1) % cell.poly.len()];
                        let Some(edge_index) = r
                            .edge_forces
                            .iter()
                            .position(|edge| boundary(a, &edge.ends) && boundary(b, &edge.ends))
                        else {
                            continue;
                        };
                        if let Some(Err(error)) = self.direct.get(ci).and_then(|v| v.as_ref()) {
                            r.direct_edges[edge_index].diagnostic = error.clone();
                        }
                        if let Some(&values) = faces.edges.get(&(ci, ei)) {
                            for p in [a, b] {
                                if let Some(error) = faces.error(p) {
                                    r.direct_edges[edge_index].diagnostic = error.into();
                                }
                            }
                            let area = (0..cell.poly.len())
                                .map(|i| {
                                    let a = cell.poly[i];
                                    let b = cell.poly[(i + 1) % cell.poly.len()];
                                    a[0] * b[1] - a[1] * b[0]
                                })
                                .sum::<f64>();
                            let length = (b[0] - a[0]).hypot(b[1] - a[1]);
                            let normal = [
                                (b[1] - a[1]) / length * area.signum(),
                                -(b[0] - a[0]) / length * area.signum(),
                            ];
                            let midpoint =
                                faces.midpoints.get(&(ci, ei)).copied().unwrap_or([0.; 3]);
                            let trace = super::face_trace::Trace::new(
                                a,
                                b,
                                normal,
                                values,
                                midpoint,
                                cell.kirchhoff,
                                cell.quadratic,
                            );
                            let value = trace.integral(a, b);
                            let edge = &mut r.direct_edges[edge_index];
                            edge.force += value[0];
                            edge.mx_origin += value[1];
                            edge.my_origin += value[2];
                        } else {
                            r.direct_edges[edge_index].diagnostic =
                                "Не восстановлена грань КЭ".into();
                        }
                        let actions = recovery
                            .edges
                            .get(&(ci, ei))
                            .copied()
                            .unwrap_or([[0.; 3]; 2]);
                        for (p, v) in [a, b].into_iter().zip(actions) {
                            if let Some(error) = recovery.error(p) {
                                r.edge_forces[edge_index].diagnostic = format!(
                                    "Ребро {}: {error}",
                                    r.edge_forces[edge_index].index + 1
                                );
                            }
                            r.force += v[0];
                            r.edge_forces[edge_index].force += v[0];
                            r.edge_forces[edge_index].mx_origin += v[1] - p[1] * v[0] / 1000.;
                            r.edge_forces[edge_index].my_origin += v[2] + p[0] * v[0] / 1000.;
                            r.mx += v[1] - (p[1] - r.center[1]) * v[0] / 1000.;
                            r.my += v[2] + (p[0] - r.center[0]) * v[0] / 1000.;
                            nodes
                                .insert(((p[0] * 1e5).round() as i64, (p[1] * 1e5).round() as i64));
                        }
                        // P2/CR edge DOFs already belong to one unique edge.
                        for &(p, v) in &cell.actions {
                            if cell.poly.contains(&p) || !boundary(p, &[a, b]) {
                                continue;
                            }
                            r.force += v[0];
                            r.edge_forces[edge_index].force += v[0];
                            r.edge_forces[edge_index].mx_origin += v[1] - p[1] * v[0] / 1000.;
                            r.edge_forces[edge_index].my_origin += v[2] + p[0] * v[0] / 1000.;
                            r.mx += v[1] - (p[1] - r.center[1]) * v[0] / 1000.;
                            r.my += v[2] + (p[0] - r.center[0]) * v[0] / 1000.;
                        }
                    }
                }
                r.patch_edges = r.edge_forces.clone();
                r.sum_active_edges();
            }
        }
        self.regions
            .into_iter()
            .map(|(id, _, _, mut r, n)| {
                r.nodes = n.len();
                if r.elements == 0 && r.diagnostic.is_empty() {
                    r.diagnostic = "Нет внутренних КЭ. Перестройте сетку".into();
                    r.base_diagnostic = r.diagnostic.clone();
                }
                (id, r)
            })
            .collect()
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    fn contour(id: u64, kind: Kind, points: Vec<Point>) -> Contour {
        Contour {
            name: String::new(),
            placement: Default::default(),
            mesh_size_mm: 0.,
            column_angle: 0.,
            calculate_effective_length: false,
            restraint_mask: 4,
            use_foundation: false,
            foundation: None,
            grid: None,
            support_open: false,
            load: None,
            refinement: None,
            id,
            parent: None,
            kind,
            points,
            curves: vec![],
        }
    }
    fn fixture() -> Vec<Contour> {
        let p = vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]];
        let mut cut = contour(
            4,
            Kind::Auxiliary,
            vec![
                [1000., 1000.],
                [3000., 1000.],
                [3000., 3000.],
                [1000., 3000.],
            ],
        );
        cut.refinement = Some(super::super::super::model::Refinement {
            inactive_edges: vec![],

            name: "Продавливание: торец тест".into(),
            size: 200.,
        });
        cut.parent = Some(1);
        let mut load = contour(3, Kind::LoadPoint, vec![[2500., 2200.]]);
        load.load = Some(super::super::super::loads::LoadData {
            direction: Default::default(),
            moment: false,
            additive: false,
            name: String::new(),
            mesh_constraint: false,
            case_id: 1,
            magnitude: super::super::super::loads::Magnitude {
                value: 100.,
                units: ForceUnits::KiloNewton,
            },
            closed: false,
        });
        vec![
            contour(1, Kind::Slab, p.clone()),
            contour(2, Kind::Support, p),
            load,
            cut,
        ]
    }
    #[test]
    fn algorithm_switch_uses_both_cached_edge_sets_and_keeps_masks() {
        let edge = EdgeForce {
            index: 0,
            ends: [[0., 0.], [1000., 0.]],
            force: 2.,
            mx_origin: 3.,
            my_origin: 4.,
            active: true,
            diagnostic: "patch cannot balance source node".into(),
        };
        let mut direct = edge.clone();
        direct.force = 7.;
        direct.diagnostic.clear();
        let mut r = CutResult {
            edge_forces: vec![edge.clone()],
            patch_edges: vec![edge],
            direct_edges: vec![direct],
            ..Default::default()
        };
        r.sum_active_edges();
        assert!(!r.diagnostic.is_empty());
        assert!(r.select_method(Method::ElementEdges));
        assert_eq!(r.force, 7.);
        assert!(r.diagnostic.is_empty());
        r.select_edges(&[0]);
        assert_eq!(r.force, 0.);
        r.select_method(Method::NodalEquilibrium);
        r.select_edges(&[0]);
        assert_eq!(r.force, 0.);
        assert!(r.diagnostic.is_empty());
        r.select_edges(&[]);
        assert_eq!(r.force, 2.);
        assert!(!r.diagnostic.is_empty());
    }
    #[test]
    fn cached_edges_reselect_without_solution_and_shift_moment_reference() {
        let mut r = CutResult {
            edge_forces: vec![
                EdgeForce {
                    index: 0,
                    ends: [[0., 0.], [1000., 0.]],
                    force: 10.,
                    mx_origin: 2.,
                    my_origin: 4.,
                    active: true,
                    diagnostic: String::new(),
                },
                EdgeForce {
                    index: 1,
                    ends: [[1000., 0.], [1000., 1000.]],
                    force: 20.,
                    mx_origin: 3.,
                    my_origin: 5.,
                    active: true,
                    diagnostic: String::new(),
                },
            ],
            ..Default::default()
        };
        r.sum_active_edges();
        assert_eq!(r.force, 30.);
        assert_eq!(r.center, [750., 250.]);
        assert!((r.mx - 12.5).abs() < 1e-12 && (r.my + 13.5).abs() < 1e-12);
        assert!(r.select_edges(&[1]));
        assert_eq!(r.force, 10.);
        assert_eq!(r.center, [500., 0.]);
        assert!((r.mx - 2.).abs() < 1e-12 && (r.my + 1.).abs() < 1e-12);
        assert_eq!(
            r.edge_forces[1].force, 20.,
            "inactive side must retain its cache"
        );
        assert!(
            !r.select_edges(&[1]),
            "unchanged flags do not invalidate results"
        );
        assert!(r.select_edges(&[]));
        assert_eq!(r.force, 30.);
        assert!((r.mx - 12.5).abs() < 1e-12 && (r.my + 13.5).abs() < 1e-12);
        r.edge_forces[1].diagnostic = "test unsupported edge".into();
        r.sum_active_edges();
        assert!(!r.diagnostic.is_empty());
        r.select_edges(&[1]);
        assert!(r.diagnostic.is_empty());
        r.select_edges(&[0, 1]);
        assert_eq!((r.force, r.mx, r.my), (0., 0., 0.));
    }
    #[test]
    fn summation_edge_metadata_excludes_disabled_sides_and_keeps_source_indices() {
        let mut all = fixture();
        let cut = all.last_mut().unwrap();
        cut.kind = Kind::Punching;
        cut.refinement.as_mut().unwrap().inactive_edges = vec![1, 3];
        let collector = Collector::new(&all);
        let r = &collector.regions[0].3;
        assert_eq!(
            r.edge_forces
                .iter()
                .filter(|e| e.active)
                .map(|e| e.index)
                .collect::<Vec<_>>(),
            vec![0, 2]
        );
        assert_eq!(r.edge_forces[0].ends, [all[3].points[0], all[3].points[1]]);
    }
    #[test]
    fn equilibrated_closed_edges_preserve_eccentric_point_balance() {
        let mut all = fixture();
        all.last_mut().unwrap().kind = Kind::Punching;
        let mesh = super::super::super::meshing::generate(&all, 500.).unwrap();
        for kind in [ElementType::Dkt, ElementType::MindlinCr] {
            let result = solve_with_type(
                &mesh,
                &all,
                &BTreeMap::from([(1, Material::default())]),
                &[1],
                10.,
                500.,
                kind,
            )
            .unwrap();
            let r = &result.cases[0].punching[&4];
            assert!(r.diagnostic.is_empty(), "{kind:?}: {r:?}");
            let mut face_result = r.clone();
            face_result.select_method(Method::ElementEdges);
            assert!(face_result.diagnostic.is_empty(), "{face_result:?}");
            assert!((face_result.force + 100.).abs() < 1e-5, "{face_result:?}");
            assert!(
                (face_result.mx - 20.).abs() < 1e-4 && (face_result.my + 50.).abs() < 1e-4,
                "{face_result:?}"
            );
            assert_eq!(r.edge_forces.len(), 4);
            assert!((r.edge_forces.iter().map(|e| e.force).sum::<f64>() - r.force).abs() < 1e-10);
            let combined =
                super::super::super::load_combinations::combine(&result, 2, &[(1, -2.)]).unwrap();
            for (a, b) in combined.punching[&4].edge_forces.iter().zip(&r.edge_forces) {
                assert_eq!(a.index, b.index);
                assert_eq!(a.ends, b.ends);
                assert!((a.force + 2. * b.force).abs() < 1e-10);
            }
            assert!((r.force + 100.).abs() < 1e-5, "{kind:?}: {r:?}");
            assert!(
                (r.mx - 20.).abs() < 1e-4 && (r.my + 50.).abs() < 1e-4,
                "{kind:?}: {r:?}"
            );
        }
    }
    #[test]
    fn cut_recovers_point_resultant_and_global_moments_for_both_triangle_types() {
        let all = fixture();
        let mesh = super::super::super::meshing::generate(&all, 500.).unwrap();
        for kind in [ElementType::Dkt, ElementType::MindlinCr] {
            let result = solve_with_type(
                &mesh,
                &all,
                &BTreeMap::from([(1, Material::default())]),
                &[1],
                10.,
                500.,
                kind,
            )
            .unwrap();
            let r = &result.cases[0].punching[&4];
            assert!(r.diagnostic.is_empty(), "{:?}", r);
            assert!(r.nodes >= 4);
            assert!((r.force + 100.).abs() < 1e-5, "{:?} {:?}", kind, r);
            assert!((r.mx - 20.).abs() < 1e-4, "{:?} {:?}", kind, r);
            assert!((r.my + 50.).abs() < 1e-4, "{:?} {:?}", kind, r);
        }
    }
    #[test]
    fn closed_cut_inserts_e_in_wall_and_preserves_quad_boundary() {
        let mut all = fixture();
        let mut wall = contour(5, Kind::Support, vec![[0., 2000.], [2000., 2000.]]);
        wall.support_open = true;
        all.push(wall);
        let mesh = super::super::super::meshing::generate(&all, 500.).unwrap();
        let e = mesh
            .nodes
            .iter()
            .position(|n| (n.xy[0] - 1000.).abs() < 1e-5 && (n.xy[1] - 2000.).abs() < 1e-5)
            .expect("E at AD/wall crossing");
        assert!(mesh.support_boundary[&5].contains(&e));
        assert!(mesh.constrained_nodes.contains(&e));
        let mesh = super::super::super::quad_mesh::process(
            mesh,
            super::super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: false,
            },
        )
        .unwrap();
        assert!(mesh.support_boundary[&5].contains(&e));
    }
    #[test]
    fn mixed_and_shell_cuts_recover_same_eccentric_resultant() {
        let mut all = fixture();
        all.last_mut().unwrap().kind = Kind::Punching;
        let mesh = super::super::super::meshing::generate(&all, 500.).unwrap();
        let mesh = super::super::super::quad_mesh::process(
            mesh,
            super::super::super::quad_mesh::Options {
                backend: Default::default(),
                merge: true,
                optimise: false,
            },
        )
        .unwrap();
        assert!(!mesh.quads.is_empty());
        let plain = solve_with_type(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[1],
            10.,
            500.,
            ElementType::Dkt,
        )
        .unwrap();
        let settings = shell::Settings {
            enabled: true,
            height_mm: 1000.,
            ..Default::default()
        };
        let shells = solve_with_support_height(
            &mesh,
            &all,
            &BTreeMap::from([(1, Material::default())]),
            &[super::super::super::loads::LoadCase::new(1)],
            10.,
            500.,
            ElementType::Dkt,
            &settings,
        )
        .unwrap();
        for result in [plain, shells] {
            let r = &result.cases[0].punching[&4];
            assert!(r.diagnostic.is_empty(), "{:?}", r);
            assert!((r.force + 100.).abs() < 1e-4, "{:?}", r);
            assert!((r.mx - 20.).abs() < 1e-3, "{:?}", r);
            assert!((r.my + 50.).abs() < 1e-3, "{:?}", r);
        }
    }
    #[test]
    fn cut_subtracts_consistent_area_loads_and_combines_linearly() {
        let mut all = fixture();
        all[2].kind = Kind::LoadArea;
        all[2].points = all[0].points.clone();
        all[2].load.as_mut().unwrap().magnitude.value = 1.;
        for kind in [ElementType::Dkt, ElementType::MindlinCr] {
            let mesh = super::super::super::meshing::generate(&all, 500.).unwrap();
            let result = solve_with_type(
                &mesh,
                &all,
                &BTreeMap::from([(1, Material::default())]),
                &[1],
                10.,
                500.,
                kind,
            )
            .unwrap();
            let r = &result.cases[0].punching[&4];
            assert!(r.diagnostic.is_empty(), "{:?}", r);
            assert!((r.force + 4.).abs() < 1e-5, "{:?} {:?}", kind, r);
            assert!(r.mx.abs() < 1e-4 && r.my.abs() < 1e-4, "{:?}", r);
            let combined =
                super::super::super::load_combinations::combine(&result, 2, &[(1, 2.)]).unwrap();
            assert!((combined.punching[&4].force + 8.).abs() < 1e-5);
        }
    }
    #[test]
    fn ad_closes_selection_but_e_does_not_contribute_to_punching() {
        let mut all = fixture();
        all[3].refinement.as_mut().unwrap().name = "Продавливание: торец стены 5".into();
        let mut wall = contour(
            5,
            Kind::Support,
            vec![[0., 2000.], [1000., 2000.], [2000., 2000.]],
        );
        wall.support_open = true;
        all.push(wall);
        let edges = control_edges(&all[3], &all);
        assert_eq!(edges.len(), 3);
        let poly = all[3].points.clone();
        let mut collector = Collector::new(&all);
        collector.add(
            &poly,
            vec![
                (poly[0], [1., 0., 0.]),
                (poly[1], [2., 0., 0.]),
                (poly[2], [3., 0., 0.]),
                (poly[3], [4., 0., 0.]),
                ([1000., 2000.], [100., 0., 0.]),
            ],
        );
        let result = collector.finish();
        let r = &result[&4];
        assert!((r.force - 10.).abs() < 1e-9);
        assert_eq!(r.nodes, 4);
        assert!((r.center[0] - 2333.3333333333335).abs() < 1e-6);
        assert!((r.center[1] - 2000.).abs() < 1e-6);
    }
}
#[cfg(test)]
mod flat_nodal_action_audit {
    use super::*;
    #[test]
    fn flat_dkt_mitc3_mitc4_node_actions_match_physical_virtual_work() {
        let m = Material::default();
        let xy = [[0., 0.], [1.7, 0.2], [2.1, 1.5]];
        let kernels = vec![
            Triangle::new(xy)
                .unwrap()
                .stiffness(m)
                .iter()
                .map(|r| r.to_vec())
                .collect::<Vec<_>>(),
            super::mitc3::Tri3::new(xy)
                .unwrap()
                .stiffness(m)
                .iter()
                .map(|r| r.to_vec())
                .collect(),
            super::mitc4::Quad4::new([[0., 0.], [1.7, 0.2], [2.1, 1.5], [0.1, 1.3]])
                .unwrap()
                .stiffness(m)
                .iter()
                .map(|r| r.to_vec())
                .collect(),
        ];
        for (kid, k) in kernels.iter().enumerate() {
            let n = k.len();
            let q = (0..n)
                .map(|i| ((i + 1) as f64).sin() * 1e-4)
                .collect::<Vec<_>>();
            let f = (0..n)
                .map(|i| ((i + 3) as f64).cos() * 0.2)
                .collect::<Vec<_>>();
            let r = (0..n)
                .map(|i| (0..n).map(|j| k[i][j] * q[j]).sum::<f64>() - f[i])
                .collect::<Vec<_>>();
            let energy = |x: &[f64]| {
                0.5 * (0..n)
                    .map(|i| x[i] * (0..n).map(|j| k[i][j] * x[j]).sum::<f64>())
                    .sum::<f64>()
                    - x.iter().zip(&f).map(|(x, f)| x * f).sum::<f64>()
            };
            for node in 0..n / 3 {
                let v = plate_node_action([r[3 * node], r[3 * node + 1], r[3 * node + 2]]);
                for axis in 0..3 {
                    let (mut plus, mut minus) = (q.clone(), q.clone());
                    let (i, sign) = match axis {
                        0 => (3 * node, 1.),
                        1 => (3 * node + 2, -1.),
                        _ => (3 * node + 1, 1.),
                    };
                    let eps = 1e-7;
                    plus[i] += eps * sign;
                    minus[i] -= eps * sign;
                    let derivative = (energy(&plus) - energy(&minus)) / (2. * eps);
                    assert!(
                        (derivative - v[axis]).abs() < 1e-6 * (1. + v[axis].abs()),
                        "kernel={kid} node={node} axis={axis}"
                    );
                }
            }
        }
    }
}
