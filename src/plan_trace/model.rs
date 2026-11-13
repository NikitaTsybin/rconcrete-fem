#[path = "grid_spec.rs"]
mod grid_spec;
use super::geometry::{self, Point};
pub use grid_spec::GridSpec;
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Debug)]
pub enum Kind {
    Slab,
    Hole,
    Support,
    MeshRegion,
    LoadPoint,
    LoadLine,
    LoadArea,
    Auxiliary,
    LoadHole,
    Beam,
    Triangulation,
    Punching,
    Column,
    Restraint,
    Foundation,
    RigidLink,
    PointRestraint,
}
impl Kind {
    pub fn is_restraint(self) -> bool {
        matches!(self, Self::Restraint | Self::PointRestraint)
    }
    pub fn is_load(self) -> bool {
        matches!(
            self,
            Self::LoadPoint | Self::LoadLine | Self::LoadArea | Self::LoadHole
        )
    }
    pub fn is_mesh_input(self) -> bool {
        matches!(
            self,
            Self::Slab
                | Self::Hole
                | Self::Support
                | Self::MeshRegion
                | Self::Beam
                | Self::Triangulation
                | Self::Punching
                | Self::Column
                | Self::Restraint
                | Self::PointRestraint
                | Self::RigidLink
        )
    }
    pub fn affects_calculation(self) -> bool {
        self != Self::Auxiliary
    }
    pub fn load_dimension(self) -> u8 {
        match self {
            Self::LoadPoint => 0,
            Self::LoadLine => 1,
            _ => 2,
        }
    }
    pub const ALL: [Self; 9] = [
        Self::Slab,
        Self::Hole,
        Self::Support,
        Self::Beam,
        Self::Column,
        Self::Auxiliary,
        Self::Triangulation,
        Self::Punching,
        Self::Foundation,
    ];
    pub fn label(self) -> &'static str {
        match self {
            Self::Slab => "Плита",
            Self::Hole => "Отверстие",
            Self::Support => "Стена",
            Self::MeshRegion => "Область КЭ",
            Self::LoadPoint => "Сосредоточенная",
            Self::LoadLine => "Линейная",
            Self::LoadArea => "Распределенная",
            Self::Auxiliary => "Вспомогательная линия",
            Self::Beam => "Балка",
            Self::Column => "Колонна",
            Self::Restraint => "Линейная опора",
            Self::PointRestraint => "Точечная опора",
            Self::Foundation => "Основание",
            Self::RigidLink => "АЖТ",
            Self::Triangulation => "Ограничение триангуляции",
            Self::LoadHole => "Отверстие нагрузки",
            Self::Punching => "Область суммирования нагрузок",
        }
    }
    pub fn color(self) -> eframe::egui::Color32 {
        match self {
            Self::Slab => eframe::egui::Color32::from_rgb(80, 210, 140),
            Self::Hole => eframe::egui::Color32::from_rgb(255, 145, 75),
            Self::Support => eframe::egui::Color32::from_rgb(90, 185, 255),
            Self::MeshRegion => eframe::egui::Color32::from_rgb(195, 140, 255),
            Self::Punching => eframe::egui::Color32::from_rgb(210, 185, 100),
            Self::Auxiliary => eframe::egui::Color32::from_rgb(185, 185, 195),
            Self::Column => eframe::egui::Color32::from_rgb(190, 160, 240),
            Self::Foundation => eframe::egui::Color32::from_rgb(160, 180, 90),
            Self::Restraint | Self::PointRestraint => eframe::egui::Color32::from_rgb(240, 170, 75),
            Self::RigidLink => eframe::egui::Color32::from_rgb(255, 156, 55),
            Self::Beam => eframe::egui::Color32::from_rgb(255, 210, 80),
            Self::Triangulation => eframe::egui::Color32::from_rgb(225, 120, 230),
            Self::LoadHole => eframe::egui::Color32::from_rgb(255, 150, 90),
            Self::LoadPoint | Self::LoadLine | Self::LoadArea => {
                eframe::egui::Color32::from_rgb(245, 75, 75)
            }
        }
    }
}
fn default_restraint_mask() -> u8 {
    4
}
fn is_default_restraint_mask(mask: &u8) -> bool {
    *mask == 4
}
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum LevelAnchor {
    #[default]
    Bottom,
    Top,
}
impl LevelAnchor {
    pub fn label(self) -> &'static str {
        match self {
            Self::Bottom => "От низа этажа",
            Self::Top => "От верха этажа",
        }
    }
}
#[derive(serde::Serialize, serde::Deserialize, Clone, Copy, PartialEq, Debug)]
pub struct LevelRef {
    pub anchor: LevelAnchor,
    pub offset_mm: f64,
}
impl Default for LevelRef {
    fn default() -> Self {
        Self {
            anchor: LevelAnchor::Bottom,
            offset_mm: 0.,
        }
    }
}
impl LevelRef {
    pub fn z(self, extent: [f64; 2]) -> f64 {
        extent[if self.anchor == LevelAnchor::Bottom {
            0
        } else {
            1
        }] + self.offset_mm
    }
}
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
pub struct Placement {
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub beam_z_mm: Option<Vec<f64>>,
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub movement_locked: bool,
    #[serde(default)]
    pub cut_other_slabs: bool,
    #[serde(default)]
    pub cut_source: Option<u64>,
    #[serde(default)]
    pub cut_part: usize,
    /// Transverse rigid arms at both ends, local Y/Z, millimetres.
    #[serde(default)]
    pub rigid_yz_mm: [f64; 2],
    #[serde(default)]
    pub slab_rigid_z_mm: f64,
    #[serde(default)]
    pub ignore_opening: bool,
    /// Automatic contour АЖТ endpoints, identified by column and wall IDs.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub rigid_link_targets: Option<[u64; 2]>,
    pub slab: LevelRef,
    pub bottom: LevelRef,
    pub top: LevelRef,
}
impl Default for Placement {
    fn default() -> Self {
        Self {
            beam_z_mm: None,
            movement_locked: false,
            cut_other_slabs: false,
            cut_source: None,
            cut_part: 0,
            rigid_yz_mm: [0.; 2],
            slab_rigid_z_mm: 0.,
            ignore_opening: false,
            rigid_link_targets: None,
            slab: Default::default(),
            bottom: Default::default(),
            top: LevelRef {
                anchor: LevelAnchor::Top,
                offset_mm: 0.,
            },
        }
    }
}
impl Placement {
    pub fn same_except_movement_lock(&self, other: &Self) -> bool {
        if self.movement_locked == other.movement_locked {
            return self == other;
        }
        Self {
            movement_locked: other.movement_locked,
            ..self.clone()
        } == *other
    }

    pub fn levels_default(&self) -> bool {
        self.slab == Self::default().slab
            && self.bottom == Self::default().bottom
            && self.top == Self::default().top
    }
    pub fn is_default(&self) -> bool {
        *self == Self::default()
    }
    pub fn validate(&self) -> Result<(), String> {
        if self
            .rigid_yz_mm
            .iter()
            .chain([&self.slab_rigid_z_mm])
            .any(|v| !v.is_finite() || v.abs() > 1e6)
        {
            return Err("Жёсткие вставки: допустимо смещение от -1000000 до 1000000 мм".into());
        }
        if [self.slab, self.bottom, self.top]
            .iter()
            .any(|v| !v.offset_mm.is_finite() || v.offset_mm.abs() > 1e6)
        {
            return Err("Некорректное смещение уровня объекта".into());
        }
        Ok(())
    }
    pub fn wall(&self, extent: [f64; 2]) -> [f64; 2] {
        [self.bottom.z(extent), self.top.z(extent)]
    }
}
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
pub struct Contour {
    #[serde(default, skip_serializing_if = "String::is_empty")]
    pub name: String,
    #[serde(default, skip_serializing_if = "Placement::is_default")]
    pub placement: Placement,
    #[serde(default)]
    pub mesh_size_mm: f64,
    #[serde(default)]
    pub column_angle: f64,
    #[serde(default)]
    pub calculate_effective_length: bool,
    #[serde(
        default = "default_restraint_mask",
        skip_serializing_if = "is_default_restraint_mask"
    )]
    pub restraint_mask: u8,
    #[serde(default)]
    pub use_foundation: bool,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub foundation: Option<super::foundation::Data>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub grid: Option<GridSpec>,
    pub support_open: bool,
    pub load: Option<super::loads::LoadData>,
    pub refinement: Option<Refinement>,
    pub id: u64,
    pub parent: Option<u64>,
    pub kind: Kind,
    pub points: Vec<Point>,
    /// Authoritative analytic geometry. `points` is a derived sampling cache.
    /// Empty for ordinary polygon contours, whose vertices remain authoritative.
    pub curves: Vec<Curve>,
}
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
pub struct Refinement {
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    pub inactive_edges: Vec<usize>,
    pub name: String,
    pub size: f64,
}
pub fn mesh_sources(all: &[Contour]) -> impl Iterator<Item = &Contour> {
    let parents = all
        .iter()
        .filter(|c| c.kind == Kind::LoadArea && c.load.as_ref().is_some_and(|l| l.mesh_constraint))
        .map(|c| c.id)
        .collect::<std::collections::BTreeSet<_>>();
    all.iter().filter(move |c| {
        c.is_mesh_input()
            || (c.kind == Kind::LoadPoint && c.load.as_ref().is_some_and(|l| l.moment))
            || (c.kind == Kind::LoadHole && c.parent.is_some_and(|id| parents.contains(&id)))
    })
}
pub fn same_mesh_input(a: &[Contour], b: &[Contour]) -> bool {
    let mut b = mesh_sources(b);
    mesh_sources(a).all(|a| {
        b.next().is_some_and(|b| {
            a.id == b.id
                && a.parent == b.parent
                && a.kind == b.kind
                && a.support_open == b.support_open
                && a.mesh_size_mm == b.mesh_size_mm
                && a.placement.same_except_movement_lock(&b.placement)
                && a.closed() == b.closed()
                && a.points == b.points
                && (a.kind != Kind::Column || a.column_angle == b.column_angle)
                && a.curves == b.curves
                && a.grid == b.grid
                && a.refinement.as_ref().map(|r| r.size) == b.refinement.as_ref().map(|r| r.size)
        })
    }) && b.next().is_none()
}
#[derive(serde::Serialize, serde::Deserialize, Clone, PartialEq, Debug)]
pub enum Curve {
    Line(Point, Point),
    Arc([Point; 3]),
    Spline { controls: Vec<Point>, closed: bool },
    Ellipse([Point; 3]),
}
pub struct ArcDimensions {
    pub midpoint: Point,
    pub through: Point,
    pub ends: [Point; 2],
    pub radius: f64,
}
impl Curve {
    pub fn arc_dimensions(&self) -> Option<ArcDimensions> {
        let Self::Arc([a, b, c]) = self else {
            return None;
        };
        let u = [b[0] - a[0], b[1] - a[1]];
        let v = [c[0] - a[0], c[1] - a[1]];
        let determinant = 2. * (u[0] * v[1] - u[1] * v[0]);
        if determinant.abs() < 1e-12 {
            return None;
        }
        let uu = u[0] * u[0] + u[1] * u[1];
        let vv = v[0] * v[0] + v[1] * v[1];
        let center = [
            a[0] + (uu * v[1] - vv * u[1]) / determinant,
            a[1] + (u[0] * vv - v[0] * uu) / determinant,
        ];
        let radius = (a[0] - center[0]).hypot(a[1] - center[1]);
        if !radius.is_finite() || radius < 1e-6 {
            return None;
        }
        let angle = |p: Point| (p[1] - center[1]).atan2(p[0] - center[0]);
        let start = angle(*a);
        let end = (angle(*c) - start).rem_euclid(std::f64::consts::TAU);
        let sweep = if (angle(*b) - start).rem_euclid(std::f64::consts::TAU) <= end {
            end
        } else {
            end - std::f64::consts::TAU
        };
        let mid = start + sweep * 0.5;
        Some(ArcDimensions {
            midpoint: [
                center[0] + radius * mid.cos(),
                center[1] + radius * mid.sin(),
            ],
            through: *b,
            ends: [*a, *c],
            radius,
        })
    }
    pub fn endpoints(&self) -> (Point, Point) {
        match self {
            Self::Line(a, b) => (*a, *b),
            Self::Arc(p) => (p[0], p[2]),
            Self::Spline { controls, closed } => (
                controls[0],
                if *closed {
                    controls[0]
                } else {
                    *controls.last().unwrap()
                },
            ),
            Self::Ellipse(p) => (p[1], p[1]),
        }
    }
    pub fn controls(&self) -> Vec<Point> {
        match self {
            Self::Line(a, b) => vec![*a, *b],
            Self::Arc(p) | Self::Ellipse(p) => p.to_vec(),
            Self::Spline { controls, .. } => controls.clone(),
        }
    }
    pub fn map(&mut self, mut f: impl FnMut(&mut Point)) {
        match self {
            Self::Line(a, b) => {
                f(a);
                f(b);
            }
            Self::Arc(p) | Self::Ellipse(p) => p.iter_mut().for_each(f),
            Self::Spline { controls, .. } => controls.iter_mut().for_each(f),
        }
    }
    pub fn sample(&self) -> Result<Vec<Point>, String> {
        use super::sketch;
        match self {
            Self::Line(a, b) => Ok(vec![*a, *b]),
            Self::Arc(p) => sketch::arc(p[0], p[1], p[2]),
            Self::Spline { controls, closed } => sketch::spline(controls, *closed),
            Self::Ellipse(p) => sketch::ellipse(p[0], p[1], p[2]),
        }
    }
}
impl Contour {
    pub(super) fn load_dimension(&self) -> u8 {
        if self.load.as_ref().is_some_and(|load| load.moment) {
            3
        } else {
            self.kind.load_dimension()
        }
    }

    pub fn is_free_beam(&self) -> bool {
        self.kind == Kind::Beam && self.placement.beam_z_mm.is_some()
    }

    pub fn beam_path(&self, extent: [f64; 2]) -> Vec<[f64; 3]> {
        let linked_z = self.placement.slab.z(extent);
        self.points
            .iter()
            .enumerate()
            .map(|(i, &[x, y])| {
                let z = self
                    .placement
                    .beam_z_mm
                    .as_ref()
                    .and_then(|zs| zs.get(i))
                    .copied()
                    .unwrap_or(linked_z);
                [x, y, z]
            })
            .collect()
    }

    fn validate_beam_path(&self) -> Result<(), String> {
        let Some(z) = &self.placement.beam_z_mm else {
            return Ok(());
        };
        if self.kind != Kind::Beam
            || self.closed()
            || !self.curves.is_empty()
            || z.len() != self.points.len()
            || z.len() < 2
            || z.iter()
                .chain(self.points.iter().flatten())
                .any(|v| !v.is_finite())
        {
            return Err(
                "Свободная балка задаётся открытой ломаной с отметкой Z каждой вершины".into(),
            );
        }
        if self.points.windows(2).zip(z.windows(2)).any(|(p, z)| {
            (p[0][0] - p[1][0])
                .hypot(p[0][1] - p[1][1])
                .hypot(z[0] - z[1])
                < 1e-6
        }) {
            return Err("Совпадающие соседние вершины балки".into());
        }
        Ok(())
    }
    pub fn active_hole(&self) -> bool {
        self.kind == Kind::Hole && !self.placement.ignore_opening
    }

    pub fn display_name(&self) -> String {
        if !self.name.trim().is_empty() {
            return self.name.clone();
        }
        if let Some(load) = &self.load {
            if !load.name.trim().is_empty() {
                return load.name.clone();
            }
        }
        if let Some(r) = &self.refinement {
            if !r.name.trim().is_empty() {
                return r.name.clone();
            }
        }
        format!("{} {}", self.kind.label(), self.id)
    }

    pub fn is_mesh_input(&self) -> bool {
        super::fem::cut_forces::is_cut(self)
            || self.kind.is_mesh_input()
            || (self.kind.is_load() && self.load.as_ref().is_some_and(|l| l.mesh_constraint))
    }

    pub fn translate(&mut self, delta: Point) -> Result<(), String> {
        if let Some(data) = &mut self.foundation {
            data.map_xy(|p| {
                p[0] += delta[0];
                p[1] += delta[1];
            });
        }
        if let Some(grid) = &mut self.grid {
            for a in 0..2 {
                grid.origin[a] += delta[a];
            }
            return self.rebuild();
        }
        for p in &mut self.points {
            p[0] += delta[0];
            p[1] += delta[1];
        }
        for curve in &mut self.curves {
            curve.map(|p| {
                p[0] += delta[0];
                p[1] += delta[1];
            });
        }
        self.rebuild()
    }
    pub fn load_label_positions(&self) -> Vec<Point> {
        if self.kind == Kind::LoadPoint {
            return self.points.first().copied().into_iter().collect();
        }
        if self.kind == Kind::LoadArea {
            use mini_fem::geo::Centroid;
            return polygon(&self.points)
                .centroid()
                .map(|p| [p.x(), p.y()])
                .into_iter()
                .collect();
        }
        let midpoint =
            |a: Point, b: Point| [a[0] + (b[0] - a[0]) * 0.5, a[1] + (b[1] - a[1]) * 0.5];
        if self.curves.is_empty() {
            return self
                .straight_edges()
                .into_iter()
                .map(|(_, a, b)| midpoint(a, b))
                .collect();
        }
        let mut out = vec![];
        for curve in &self.curves {
            match curve {
                Curve::Line(a, b) => out.push(midpoint(*a, *b)),
                Curve::Arc(_) | Curve::Ellipse(_) => {
                    if let Ok(p) = curve.sample() {
                        if !p.is_empty() {
                            out.push(p[p.len() / 2]);
                        }
                    }
                }
                Curve::Spline { controls, closed } => {
                    let n = controls.len();
                    if n < 2 {
                        continue;
                    }
                    for i in 0..if *closed { n } else { n - 1 } {
                        let a = controls[i];
                        let b = controls[(i + 1) % n];
                        let prev = if i > 0 {
                            controls[i - 1]
                        } else if *closed {
                            controls[n - 1]
                        } else {
                            [2. * a[0] - b[0], 2. * a[1] - b[1]]
                        };
                        let next = if i + 2 < n {
                            controls[i + 2]
                        } else if *closed {
                            controls[(i + 2) % n]
                        } else {
                            [2. * b[0] - a[0], 2. * b[1] - a[1]]
                        };
                        out.push([
                            (-prev[0] + 9. * a[0] + 9. * b[0] - next[0]) / 16.,
                            (-prev[1] + 9. * a[1] + 9. * b[1] - next[1]) / 16.,
                        ]);
                    }
                }
            }
        }
        for (i, a, b) in self.straight_edges() {
            if i == self.curves.len() {
                out.push(midpoint(a, b));
            }
        }
        out
    }
    pub fn closed(&self) -> bool {
        if matches!(
            self.kind,
            Kind::Support | Kind::Auxiliary | Kind::Beam | Kind::Triangulation | Kind::Restraint
        ) {
            !self.support_open
        } else if matches!(
            self.kind,
            Kind::LoadPoint | Kind::Column | Kind::PointRestraint | Kind::RigidLink
        ) {
            false
        } else if self.kind == Kind::LoadLine {
            self.load.as_ref().is_some_and(|l| l.closed)
        } else {
            true
        }
    }
    pub fn straight_edges(&self) -> Vec<(usize, Point, Point)> {
        if self.curves.is_empty() {
            let count = if self.closed() {
                self.points.len()
            } else {
                self.points.len().saturating_sub(1)
            };
            return (0..count)
                .map(|i| (i, self.points[i], self.points[(i + 1) % self.points.len()]))
                .collect();
        }
        let mut out = self
            .curves
            .iter()
            .enumerate()
            .filter_map(|(i, c)| {
                if let Curve::Line(a, b) = c {
                    Some((i, *a, *b))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        let first = self.curves[0].endpoints().0;
        let last = self.curves.last().unwrap().endpoints().1;
        if self.closed() && last != first {
            out.push((self.curves.len(), last, first));
        }
        out
    }
    pub fn bend_edge(&mut self, edge: usize, through: Point) -> Result<(), String> {
        if self.kind == Kind::RigidLink {
            return Err("АЖТ задаётся прямым отрезком".into());
        }
        if self.grid.is_some() {
            return Err("Для сетки используйте поля dx,dy,nx,ny".into());
        }
        let (_, a, b) = self
            .straight_edges()
            .into_iter()
            .find(|(i, _, _)| *i == edge)
            .ok_or("Выберите прямолинейное ребро")?;
        if self.curves.is_empty() {
            let count = if self.closed() {
                self.points.len()
            } else {
                self.points.len().saturating_sub(1)
            };
            self.curves = (0..count)
                .map(|i| Curve::Line(self.points[i], self.points[(i + 1) % self.points.len()]))
                .collect();
        }
        let arc = Curve::Arc([a, through, b]);
        if edge == self.curves.len() {
            self.curves.push(arc);
        } else {
            self.curves[edge] = arc;
        }
        self.rebuild()
    }
    /// FE boundary discretization is independent of the fine display cache.
    /// Original corners and joins are retained; smooth spans use arc-length spacing.
    pub fn mesh_step(&self, project: f64) -> f64 {
        if self.mesh_size_mm > 0. {
            self.mesh_size_mm
        } else {
            project
        }
    }
    pub fn mesh_boundary(&self, step: f64) -> Result<Vec<Point>, String> {
        if self.curves.is_empty() {
            return Ok(self.points.clone());
        }
        let mut out = Vec::new();
        for curve in &self.curves {
            let sampled = curve.sample()?;
            let closed = matches!(
                curve,
                Curve::Ellipse(_) | Curve::Spline { closed: true, .. }
            );
            let points = if matches!(curve, Curve::Line(..)) || sampled.len() <= 2 {
                sampled
            } else {
                spaced_boundary(&sampled, closed, step)?
            };
            let skip = usize::from(out.last() == points.first());
            out.extend(points.into_iter().skip(skip));
        }
        if out.len() > 1 && out.first() == out.last() {
            out.pop();
        }
        if out.len() > 4096 {
            return Err("Слишком много точек границы МКЭ".into());
        }
        if self.closed() {
            valid(&out)?;
        } else {
            valid_open_path(&out)?;
        }
        Ok(out)
    }
    pub fn handles(&self) -> Vec<Point> {
        if let Some(grid) = &self.grid {
            return vec![grid.origin];
        }
        if self.curves.is_empty() {
            return self.points.clone();
        }
        let mut out = vec![];
        for c in &self.curves {
            for p in c.controls() {
                if !out.contains(&p) {
                    out.push(p);
                }
            }
        }
        out
    }
    pub fn rebuild(&mut self) -> Result<(), String> {
        self.placement.validate()?;
        self.validate_beam_path()?;
        if !self.mesh_size_mm.is_finite()
            || self.mesh_size_mm != 0. && !(100. ..=1e6).contains(&self.mesh_size_mm)
        {
            return Err("Размер КЭ объекта должен быть 0 или от 100 до 1000000 мм".into());
        }
        if self.name.len() > 1024 {
            return Err("Слишком длинное имя объекта".into());
        }
        if let Some(grid) = &self.grid {
            if self.kind != Kind::Triangulation || !self.curves.is_empty() {
                return Err("Параметры сетки допустимы только для ограничения триангуляции".into());
            }
            self.points = grid.points()?;
            self.support_open = true;
            return Ok(());
        }
        if self.curves.is_empty() {
            return Ok(());
        }
        let mut points = vec![];
        for c in &self.curves {
            let p = c.sample()?;
            let skip = usize::from(points.last() == p.first());
            points.extend(p.into_iter().skip(skip));
        }
        if points.len() > 1 && points.first() == points.last() {
            points.pop();
        }
        if points.len() > 4096 {
            return Err("Объект превышает лимит аппроксимации 4096 вершин".into());
        }
        if !self.closed() {
            valid_open_path(&points)?;
        } else {
            valid(&points)?;
        }
        self.points = points;
        Ok(())
    }
    pub fn move_edge(&mut self, index: usize, delta: Point) -> Result<(), String> {
        let (_, a, b) = self
            .straight_edges()
            .into_iter()
            .find(|e| e.0 == index)
            .ok_or("Не найден прямой отрезок")?;
        let shift = |p: &mut Point| {
            if *p == a || *p == b {
                p[0] += delta[0];
                p[1] += delta[1];
            }
        };
        if self.curves.is_empty() {
            for p in &mut self.points {
                shift(p);
            }
            Ok(())
        } else {
            for curve in &mut self.curves {
                curve.map(shift);
            }
            self.rebuild()
        }
    }
    pub fn move_handle(&mut self, index: usize, p: Point) -> Result<(), String> {
        if let Some(grid) = &mut self.grid {
            if index != 0 {
                return Err("У сетки редактируются dx,dy,nx,ny".into());
            }
            grid.origin = p;
            return self.rebuild();
        }
        if self.curves.is_empty() {
            self.points[index] = p;
            return Ok(());
        }
        let old = *self
            .handles()
            .get(index)
            .ok_or("Не найдена управляющая точка")?;
        for c in &mut self.curves {
            if let Curve::Ellipse(points) = c {
                if points[0] == old {
                    let delta = [p[0] - old[0], p[1] - old[1]];
                    for q in points {
                        q[0] += delta[0];
                        q[1] += delta[1];
                    }
                    continue;
                }
            }
            c.map(|v| {
                if *v == old {
                    *v = p;
                }
            });
        }
        self.rebuild()
    }
    pub fn can_insert(&self, index: usize) -> bool {
        if self.grid.is_some() {
            return false;
        }
        if self.curves.is_empty() {
            return !matches!(
                self.kind,
                Kind::LoadPoint | Kind::Column | Kind::PointRestraint | Kind::RigidLink
            ) && (self.closed() || index + 1 < self.points.len());
        }
        let Some(p) = self.handles().get(index).copied() else {
            return false;
        };
        self.curves.iter().any(|c| match c {
            Curve::Line(a, _) => *a == p,
            Curve::Spline { controls, closed } => controls
                .iter()
                .position(|q| *q == p)
                .is_some_and(|i| *closed || i + 1 < controls.len()),
            _ => false,
        })
    }
    pub fn can_remove(&self, index: usize) -> bool {
        if self.grid.is_some() {
            return false;
        }
        if self.curves.is_empty() {
            return self.points.len() > if !self.closed() { 2 } else { 3 };
        }
        let Some(p) = self.handles().get(index).copied() else {
            return false;
        };
        let count = self
            .curves
            .iter()
            .filter(|c| c.controls().contains(&p))
            .count();
        count == 1
            && self.curves.iter().any(|c| match c {
                Curve::Spline { controls, closed } => {
                    controls.contains(&p) && controls.len() > if *closed { 3 } else { 2 }
                }
                _ => false,
            })
    }
    pub fn change_handle(&mut self, index: usize, add: bool) -> Result<(), String> {
        if self.curves.is_empty() {
            if self.kind == Kind::Punching {
                if let Some(r) = &mut self.refinement {
                    let n = self.points.len();
                    let mut flags: Vec<_> = (0..n).map(|i| r.inactive_edges.contains(&i)).collect();
                    if add {
                        flags.insert(index + 1, flags[index]);
                    } else {
                        let previous = (index + n - 1) % n;
                        flags[previous] = flags[previous] || flags[index];
                        flags.remove(index);
                    }
                    r.inactive_edges = flags
                        .iter()
                        .enumerate()
                        .filter_map(|(i, &off)| off.then_some(i))
                        .collect();
                }
            }
            if add {
                let a = self.points[index];
                let b = self.points[(index + 1) % self.points.len()];
                if let Some(z) = &mut self.placement.beam_z_mm {
                    z.insert(index + 1, (z[index] + z[index + 1]) / 2.);
                }
                self.points
                    .insert(index + 1, [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.]);
            } else {
                if let Some(z) = &mut self.placement.beam_z_mm {
                    z.remove(index);
                }
                self.points.remove(index);
            }
            return Ok(());
        }
        if !(if add {
            self.can_insert(index)
        } else {
            self.can_remove(index)
        }) {
            return Err("Для дуги и эллипса изменяйте управляющие точки; для сплайна можно добавлять и удалять точки".into());
        }
        let p = self.handles()[index];
        for i in 0..self.curves.len() {
            match &mut self.curves[i] {
                Curve::Spline { controls, closed } => {
                    if let Some(j) = controls.iter().position(|q| *q == p) {
                        if add && (*closed || j + 1 < controls.len()) {
                            let b = controls[(j + 1) % controls.len()];
                            controls.insert(j + 1, [(p[0] + b[0]) / 2., (p[1] + b[1]) / 2.]);
                            break;
                        }
                        if !add {
                            controls.remove(j);
                            break;
                        }
                    }
                }
                Curve::Line(a, b) if add && *a == p => {
                    let a = *a;
                    let b = *b;
                    let m = [(a[0] + b[0]) / 2., (a[1] + b[1]) / 2.];
                    self.curves[i] = Curve::Line(a, m);
                    self.curves.insert(i + 1, Curve::Line(m, b));
                    break;
                }
                _ => {}
            }
        }
        self.rebuild()
    }
}
fn spaced_boundary(points: &[Point], closed: bool, step: f64) -> Result<Vec<Point>, String> {
    if !step.is_finite() || step <= 0. {
        return Err("Некорректный шаг границы МКЭ".into());
    }
    let mut path = points.to_vec();
    if closed && path.first() != path.last() {
        path.push(path[0]);
    }
    let mut lengths = vec![0.0_f64];
    for pair in path.windows(2) {
        lengths.push(
            lengths.last().copied().unwrap()
                + (pair[1][0] - pair[0][0]).hypot(pair[1][1] - pair[0][1]),
        );
    }
    let total = *lengths.last().unwrap();
    if total < 1e-6 {
        return Err("Вырожденный участок границы МКЭ".into());
    }
    let count = ((total / step).round() as usize).max(if closed { 8 } else { 2 });
    if count > 4096 {
        return Err("Слишком много точек границы МКЭ".into());
    }
    let mut out = Vec::new();
    let mut segment = 0;
    for i in 0..=count {
        if closed && i == count {
            break;
        }
        if i == count {
            out.push(*path.last().unwrap());
            break;
        }
        let distance = total * i as f64 / count as f64;
        while segment + 2 < lengths.len() && lengths[segment + 1] < distance {
            segment += 1;
        }
        let span = lengths[segment + 1] - lengths[segment];
        let t = if span > 1e-12 {
            (distance - lengths[segment]) / span
        } else {
            0.
        };
        let a = path[segment];
        let b = path[segment + 1];
        out.push([a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]);
    }
    Ok(out)
}
fn valid_open_path(points: &[Point]) -> Result<(), String> {
    if points.len() < 2 {
        return Err("Укажите не менее двух точек линии".into());
    }
    if points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("Некорректные координаты".into());
    }
    if points
        .windows(2)
        .any(|p| (p[0][0] - p[1][0]).hypot(p[0][1] - p[1][1]) < 1e-6)
    {
        return Err("Совпадающие соседние точки".into());
    }
    Ok(())
}
/// Remove only exactly collinear backtracking tips (zero swept area).
/// Unlike ordinary simplification this does not smooth any real corner.
pub fn without_collinear_spurs(mut points: Vec<Point>) -> Vec<Point> {
    loop {
        let before = points.len();
        let mut i = 0;
        while points.len() > 3 && i < points.len() {
            let n = points.len();
            let (a, b, c) = (points[(i + n - 1) % n], points[i], points[(i + 1) % n]);
            let u = [b[0] - a[0], b[1] - a[1]];
            let v = [c[0] - b[0], c[1] - b[1]];
            let cross = u[0] * v[1] - u[1] * v[0];
            if u[0] * v[0] + u[1] * v[1] < 0. && cross.abs() <= 1e-9 && a != c {
                points.remove(i);
            } else {
                i += 1;
            }
        }
        if points.len() == before {
            break;
        }
    }
    points
}
pub fn self_contacts(points: &[Point]) -> usize {
    let mut count = 0;
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        for j in i + 1..points.len() {
            if j == i + 1 || (i == 0 && j == points.len() - 1) {
                continue;
            }
            let c = points[j];
            let d = points[(j + 1) % points.len()];
            let overlap = a[0].min(b[0]) <= c[0].max(d[0]) + 1e-9
                && c[0].min(d[0]) <= a[0].max(b[0]) + 1e-9
                && a[1].min(b[1]) <= c[1].max(d[1]) + 1e-9
                && c[1].min(d[1]) <= a[1].max(b[1]) + 1e-9;
            if overlap
                && geometry::cross(a, b, c) * geometry::cross(a, b, d) <= 1e-12
                && geometry::cross(c, d, a) * geometry::cross(c, d, b) <= 1e-12
            {
                count += 1;
            }
        }
    }
    count
}
pub fn valid(points: &[Point]) -> Result<(), String> {
    if points.len() < 3 {
        return Err("Нужно не менее трёх вершин".into());
    }
    if points.iter().flatten().any(|v| !v.is_finite()) {
        return Err("Некорректные координаты".into());
    }
    for i in 0..points.len() {
        let a = points[i];
        let b = points[(i + 1) % points.len()];
        if (a[0] - b[0]).hypot(a[1] - b[1]) < 1e-6 {
            return Err("Совпадающие соседние вершины".into());
        }
        for j in i + 1..points.len() {
            if j == i + 1 || (i == 0 && j == points.len() - 1) {
                continue;
            }
            let c = points[j];
            let d = points[(j + 1) % points.len()];
            let c1 = geometry::cross(a, b, c);
            let c2 = geometry::cross(a, b, d);
            let c3 = geometry::cross(c, d, a);
            let c4 = geometry::cross(c, d, b);
            let overlap = a[0].min(b[0]) <= c[0].max(d[0]) + 1e-9
                && c[0].min(d[0]) <= a[0].max(b[0]) + 1e-9
                && a[1].min(b[1]) <= c[1].max(d[1]) + 1e-9
                && c[1].min(d[1]) <= a[1].max(b[1]) + 1e-9;
            if overlap && c1 * c2 <= 1e-12 && c3 * c4 <= 1e-12 {
                return Err("Граница пересекает или касается сама себя".into());
            }
        }
    }
    if geometry::area(points) < 1e-6 {
        return Err("Площадь равна нулю".into());
    }
    Ok(())
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn arc_hover_dimensions_use_true_radius_and_angular_midpoint() {
        let arc = Curve::Arc([[-1000., 0.], [600., 800.], [1000., 0.]]);
        let d = arc.arc_dimensions().unwrap();
        assert!((d.radius - 1000.).abs() < 1e-6);
        assert!(d.midpoint[0].abs() < 1e-6 && (d.midpoint[1] - 1000.).abs() < 1e-6);
        assert_eq!(d.through, [600., 800.]);
        assert_eq!(d.ends, [[-1000., 0.], [1000., 0.]]);
        assert!(Curve::Line([0., 0.], [1., 0.]).arc_dimensions().is_none());
    }
    #[test]
    fn fe_curve_boundary_uses_requested_spacing_without_changing_source_or_display() {
        for curve in [
            Curve::Ellipse([[0., 0.], [1000., 0.], [0., 1000.]]),
            Curve::Arc([[-1000., 0.], [0., 1000.], [1000., 0.]]),
            Curve::Spline {
                closed: true,
                controls: vec![[0., 0.], [3000., 0.], [3000., 2000.], [0., 2000.]],
            },
        ] {
            let mut c = Contour {
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
                id: 1,
                parent: None,
                kind: Kind::Slab,
                points: vec![],
                curves: vec![curve],
            };
            c.rebuild().unwrap();
            let source = c.curves.clone();
            let display = c.points.clone();
            let coarse = c.mesh_boundary(500.).unwrap();
            let fine = c.mesh_boundary(250.).unwrap();
            assert!(coarse.len() < fine.len());
            assert!(coarse.len() < display.len());
            assert_eq!(c.curves, source);
            assert_eq!(c.points, display);
            if matches!(source[0], Curve::Ellipse(_)) {
                assert!((10..=16).contains(&coarse.len()));
                for i in 0..coarse.len() {
                    let a = coarse[i];
                    let b = coarse[(i + 1) % coarse.len()];
                    let length = (a[0] - b[0]).hypot(a[1] - b[1]);
                    assert!((375.0..=625.0).contains(&length));
                }
            }
        }
    }
    #[test]
    fn ellipse_keeps_three_controls_and_center_drag_translates_axes() {
        let mut c = Contour {
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
            id: 1,
            parent: None,
            kind: Kind::Slab,
            points: vec![],
            curves: vec![Curve::Ellipse([[0., 0.], [1000., 0.], [0., 500.]])],
        };
        c.rebuild().unwrap();
        assert_eq!(c.handles().len(), 3);
        assert!(c.points.len() > 30);
        c.move_handle(0, [100., 200.]).unwrap();
        assert_eq!(
            c.curves,
            vec![Curve::Ellipse([[100., 200.], [1100., 200.], [100., 700.]])]
        );
        c.move_handle(2, [100., 800.]).unwrap();
        assert_eq!(c.handles().len(), 3);
        assert!(matches!(c.curves[0], Curve::Ellipse(_)));
        assert!(!c.can_remove(0));
    }
    #[test]
    fn closed_contours_reject_self_intersections_and_degeneracy() {
        assert!(valid(&[[0., 0.], [10., 0.], [10., 10.], [0., 10.]]).is_ok());
        assert!(valid(&[[0., 0.], [10., 10.], [0., 10.], [10., 0.]]).is_err());
        assert!(valid(&[[0., 0.], [0., 0.], [1., 1.]]).is_err());
    }
}

fn polygon(points: &[Point]) -> mini_fem::geo::Polygon<f64> {
    mini_fem::geo::Polygon::new(
        mini_fem::geo::LineString::from(points.iter().map(|p| (p[0], p[1])).collect::<Vec<_>>()),
        vec![],
    )
}
pub fn validate_candidate(all: &[Contour], candidate: &Contour) -> Result<(), String> {
    if candidate.is_free_beam() {
        return candidate.validate_beam_path();
    }
    if candidate.kind == Kind::RigidLink {
        if candidate.points.len() != 2 || !candidate.curves.is_empty() {
            return Err("АЖТ задаётся двумя вершинами в плане".into());
        }
        return valid_open_path(&candidate.points);
    }
    if candidate.kind == Kind::Foundation {
        candidate
            .foundation
            .clone()
            .unwrap_or_default()
            .validate(candidate)?;
    }
    if candidate.kind.is_restraint() && candidate.restraint_mask > 63 {
        return Err("Некорректные закрепления опоры".into());
    }
    if candidate.kind == Kind::PointRestraint {
        return if candidate.points.len() == 1
            && candidate.points[0].iter().all(|v| v.is_finite())
            && candidate.curves.is_empty()
        {
            Ok(())
        } else {
            Err("Укажите одну точку опоры".into())
        };
    }
    if candidate.kind == Kind::Column {
        return if candidate.column_angle.is_finite()
            && candidate.points.len() == 1
            && candidate.points[0].iter().all(|v| v.is_finite())
        {
            Ok(())
        } else {
            Err("Укажите одну точку колонны".into())
        };
    }

    use mini_fem::geo::{Area, BooleanOps};
    if candidate.kind == Kind::LoadHole {
        let area = all
            .iter()
            .find(|c| Some(c.id) == candidate.parent && c.kind == Kind::LoadArea)
            .ok_or("Сначала выберите площадную нагрузку")?;
        if candidate.load.as_ref().map(|l| l.case_id) != area.load.as_ref().map(|l| l.case_id) {
            return Err("Отверстие относится к другому загружению".into());
        }
        if polygon(&candidate.points)
            .difference(&polygon(&area.points))
            .unsigned_area()
            > 1e-6
        {
            return Err("Отверстие нагрузки должно быть внутри её границ".into());
        }
    }
    if candidate.kind == Kind::LoadArea
        && all
            .iter()
            .filter(|c| c.kind == Kind::LoadHole && c.parent == Some(candidate.id))
            .any(|h| {
                polygon(&h.points)
                    .difference(&polygon(&candidate.points))
                    .unsigned_area()
                    > 1e-6
            })
    {
        return Err("Отверстие выходит за границу нагрузки".into());
    }
    if candidate.kind.is_load() {
        let load = candidate
            .load
            .as_ref()
            .ok_or("Не задано загружение нагрузки")?;
        if load.case_id == 0 || !load.magnitude.value.is_finite() {
            return Err("Некорректные параметры нагрузки".into());
        }
        return match candidate.kind {
            Kind::LoadPoint => {
                if candidate.points.len() == 1 && candidate.points[0].iter().all(|v| v.is_finite())
                {
                    Ok(())
                } else {
                    Err("Укажите одну точку нагрузки".into())
                }
            }
            Kind::LoadLine => {
                if load.closed && candidate.points.len() < 3 {
                    Err("Для замкнутой линии нужно не менее трёх точек".into())
                } else {
                    valid_open_path(&candidate.points)
                }
            }
            _ => valid(&candidate.points),
        };
    }
    if matches!(
        candidate.kind,
        Kind::Support | Kind::Auxiliary | Kind::Beam | Kind::Triangulation | Kind::Restraint
    ) && !candidate.closed()
    {
        return valid_open_path(&candidate.points);
    }
    valid(&candidate.points)?;
    let contains = |slab: &[Point], hole: &[Point]| {
        polygon(hole).difference(&polygon(slab)).unsigned_area() < 1e-6
    };
    if matches!(candidate.kind, Kind::Hole | Kind::MeshRegion) {
        let slab = all
            .iter()
            .find(|c| Some(c.id) == candidate.parent && c.kind == Kind::Slab)
            .ok_or("Сначала выберите плиту для вложенного объекта")?;
        if !contains(&slab.points, &candidate.points) {
            return Err("Вложенный объект должен находиться внутри выбранной плиты".into());
        }
    }
    if candidate.kind == Kind::MeshRegion {
        let settings = candidate
            .refinement
            .as_ref()
            .ok_or("У области КЭ не задан размер")?;
        if !settings.size.is_finite() || settings.size < 100. {
            return Err("Размер КЭ области должен быть не менее 100 мм".into());
        }
    }
    if candidate.kind == Kind::Slab
        && all
            .iter()
            .filter(|c| {
                matches!(c.kind, Kind::Hole | Kind::MeshRegion)
                    && c.parent == Some(candidate.id)
                    && c.placement.cut_source.is_none()
            })
            .any(|h| !contains(&candidate.points, &h.points))
    {
        return Err("Изменение плиты оставляет отверстие или область КЭ за её границей".into());
    }
    Ok(())
}
pub fn slab_triangles<'a>(all: impl IntoIterator<Item = &'a Contour>) -> Vec<[Point; 3]> {
    slab_triangles_with_owner(all)
        .into_iter()
        .map(|(_, triangle)| triangle)
        .collect()
}
pub(super) fn slab_material<'a>(
    slab: &Contour,
    holes: impl IntoIterator<Item = &'a Contour>,
) -> mini_fem::geo::MultiPolygon<f64> {
    use mini_fem::geo::{
        algorithm::{orient::Direction, unary_union},
        BooleanOps, MultiPolygon, Orient,
    };
    let area = MultiPolygon(vec![polygon(&slab.points)]);
    let mut holes = holes.into_iter();
    let Some(first) = holes.next() else {
        return area;
    };
    let first = polygon(&first.points);
    let Some(second) = holes.next() else {
        return area.difference(&first);
    };
    let holes = std::iter::once(first)
        .chain(std::iter::once(polygon(&second.points)))
        .chain(holes.map(|hole| polygon(&hole.points)))
        .map(|hole| hole.orient(Direction::Default))
        .collect::<Vec<_>>();
    area.difference(&unary_union(&holes))
}

/// Boolean output already assigns each interior ring to its exterior.
pub(super) fn polygon_triangles(polygon: &mini_fem::geo::Polygon<f64>) -> Vec<[Point; 3]> {
    let mut coordinates = Vec::new();
    let mut holes = Vec::with_capacity(polygon.interiors().len());
    for (index, ring) in std::iter::once(polygon.exterior())
        .chain(polygon.interiors())
        .enumerate()
    {
        if index != 0 {
            holes.push(coordinates.len() / 2);
        }
        coordinates.extend(ring.0.iter().flat_map(|p| [p.x, p.y]));
    }
    let Ok(indices) = earcutr::earcut(&coordinates, &holes, 2) else {
        return vec![];
    };
    indices
        .chunks_exact(3)
        .map(|t| std::array::from_fn(|i| [coordinates[2 * t[i]], coordinates[2 * t[i] + 1]]))
        .collect()
}

pub fn slab_triangles_with_owner<'a>(
    all: impl IntoIterator<Item = &'a Contour>,
) -> Vec<(u64, [Point; 3])> {
    let mut slabs = Vec::new();
    let mut holes = std::collections::BTreeMap::<u64, Vec<&Contour>>::new();
    for contour in all {
        if contour.kind == Kind::Slab {
            slabs.push(contour);
        } else if contour.active_hole() {
            if let Some(parent) = contour.parent {
                holes.entry(parent).or_default().push(contour);
            }
        }
    }
    let mut triangles = Vec::new();
    for slab in slabs {
        let area = slab_material(slab, holes.get(&slab.id).into_iter().flatten().copied());
        for p in &area.0 {
            triangles.extend(polygon_triangles(p).into_iter().map(|t| (slab.id, t)));
        }
    }
    triangles
}
#[cfg(test)]
mod slab_tests {
    use super::*;
    #[test]
    fn ignored_opening_is_retained_but_restores_plate_area() {
        let slab = c(1, Kind::Slab, None, 0., 0., 2000., 2000.);
        let mut opening = c(2, Kind::Hole, Some(1), 500., 500., 1000., 1000.);
        let area = |hole: Contour| {
            let mesh = super::super::meshing::generate(&[slab.clone(), hole], 400.).unwrap();
            mesh.elements
                .iter()
                .map(|e| super::super::geometry::area(&e.nodes.map(|n| mesh.nodes[n].xy)).abs())
                .sum::<f64>()
        };
        assert!((area(opening.clone()) - 3e6).abs() < 1e-4);
        opening.placement.ignore_opening = true;
        assert!((area(opening) - 4e6).abs() < 1e-4);
    }
    fn c(id: u64, kind: Kind, parent: Option<u64>, x: f64, y: f64, w: f64, h: f64) -> Contour {
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
            curves: vec![],
            id,
            kind,
            parent,
            points: vec![[x, y], [x + w, y], [x + w, y + h], [x, y + h]],
        }
    }
    #[test]
    fn associated_overlapping_holes_subtract_only_from_their_own_slab() {
        let mut all = vec![
            c(1, Kind::Slab, None, 0., 0., 100., 100.),
            c(2, Kind::Hole, Some(1), 10., 10., 20., 20.),
            c(3, Kind::Hole, Some(1), 20., 10., 20., 20.),
            c(4, Kind::Slab, None, 200., 0., 100., 100.),
        ];
        all[2].points.reverse();
        assert!(validate_candidate(&all, &all[1]).is_ok());
        assert!(
            (slab_triangles(&all)
                .iter()
                .map(|t| geometry::area(t))
                .sum::<f64>()
                - 19400.)
                .abs()
                < 1e-6
        );
        let bad = c(5, Kind::Hole, Some(1), 90., 90., 20., 20.);
        assert!(validate_candidate(&all, &bad).is_err());
        let bad_slab = c(1, Kind::Slab, None, 50., 50., 50., 50.);
        assert!(validate_candidate(&all, &bad_slab).is_err());
    }
}

pub fn load_area_material(all: &[Contour], area: &Contour) -> mini_fem::geo::MultiPolygon<f64> {
    use mini_fem::geo::{BooleanOps, MultiPolygon};
    let mut material = MultiPolygon(vec![polygon(&area.points)]);
    for h in all
        .iter()
        .filter(|c| c.kind == Kind::LoadHole && c.parent == Some(area.id))
    {
        material = material.difference(&polygon(&h.points));
    }
    material
}

/// Display clipping only; source loads and FE load activation are unchanged.
pub fn visible_load_area(all: &[Contour], area: &Contour) -> mini_fem::geo::MultiPolygon<f64> {
    use mini_fem::geo::BooleanOps;
    load_area_material(all, area).intersection(&visible_load_domain(all))
}

fn load_domain_slabs(
    all: &[Contour],
) -> impl Iterator<Item = (u64, mini_fem::geo::MultiPolygon<f64>)> + '_ {
    let mut holes = std::collections::BTreeMap::<u64, Vec<&Contour>>::new();
    for hole in all.iter().filter(|c| c.active_hole()) {
        if let Some(parent) = hole.parent {
            holes.entry(parent).or_default().push(hole);
        }
    }
    all.iter()
        .filter(|c| c.kind == Kind::Slab)
        .map(move |slab| {
            (
                slab.id,
                slab_material(slab, holes.remove(&slab.id).into_iter().flatten()),
            )
        })
}

pub(super) fn visible_load_domain(all: &[Contour]) -> mini_fem::geo::MultiPolygon<f64> {
    use mini_fem::geo::{algorithm::orient::Direction, unary_union, Orient};
    let slabs = load_domain_slabs(all)
        .map(|(_, slab)| slab.orient(Direction::Default))
        .collect::<Vec<_>>();
    unary_union(&slabs)
}

/// Display-only ownership of slab self-weight, excluding holes and duplicate overlaps.
pub(super) fn visible_self_weight_areas(
    all: &[Contour],
) -> std::collections::BTreeMap<u64, mini_fem::geo::MultiPolygon<f64>> {
    use mini_fem::geo::{BooleanOps, MultiPolygon};
    let mut occupied = MultiPolygon(vec![]);
    load_domain_slabs(all)
        .map(|(id, area)| {
            let part = area.difference(&occupied);
            occupied = occupied.union(&area);
            (id, part)
        })
        .collect()
}
