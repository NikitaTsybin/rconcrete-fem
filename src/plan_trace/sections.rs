//! Named view-only clipping half-spaces in model coordinates.
use super::*;

pub(super) const MAX_PLANES: usize = 12;
pub(super) type ClipPlanes = [[f32; 4]; MAX_PLANES];
pub(super) const NO_CLIPS: ClipPlanes = [[0., 0., 0., 1.]; MAX_PLANES];

#[derive(Clone, Copy, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub(super) enum Kind {
    Xoy,
    Xoz,
    Yoz,
    VerticalAngle,
}
impl Kind {
    const ALL: [Self; 4] = [Self::Xoy, Self::Xoz, Self::Yoz, Self::VerticalAngle];
    fn label(self) -> &'static str {
        match self {
            Self::Xoy => "XOY",
            Self::Xoz => "XOZ",
            Self::Yoz => "YOZ",
            Self::VerticalAngle => "Z-α",
        }
    }
}
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct Plane {
    pub enabled: bool,
    pub offset_mm: f64,
    pub positive: bool,
    pub name: String,
    pub kind: Option<Kind>,
    pub alpha_deg: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub base_xy: Option<Point>,
}
impl Default for Plane {
    fn default() -> Self {
        Self {
            enabled: false,
            offset_mm: 0.,
            positive: true,
            name: String::new(),
            kind: None,
            alpha_deg: 0.,
            base_xy: None,
        }
    }
}
impl Plane {
    pub fn kind(&self, index: usize) -> Kind {
        self.kind.unwrap_or(Kind::ALL[index.min(2)])
    }
    pub fn normal(&self, index: usize) -> [f64; 3] {
        match self.kind(index) {
            Kind::Xoy => [0., 0., 1.],
            Kind::Xoz => [0., 1., 0.],
            Kind::Yoz => [1., 0., 0.],
            Kind::VerticalAngle => {
                let (sin, cos) = self.alpha_deg.to_radians().sin_cos();
                [-sin, cos, 0.]
            }
        }
    }
    pub fn offset(&self, index: usize) -> f64 {
        if self.kind(index) == Kind::VerticalAngle {
            let [nx, ny, _] = self.normal(index);
            let [x, y] = self.base_xy.unwrap_or([0.; 2]);
            nx * x + ny * y
        } else {
            self.offset_mm
        }
    }
    pub fn set_origin(&mut self, index: usize, [x, y, z]: [f64; 3]) {
        match self.kind(index) {
            Kind::Xoy => self.offset_mm = z,
            Kind::Xoz => self.offset_mm = y,
            Kind::Yoz => self.offset_mm = x,
            Kind::VerticalAngle => self.base_xy = Some([x, y]),
        }
    }
    pub fn distance(&self, index: usize, p: [f64; 3]) -> f64 {
        let distance = self
            .normal(index)
            .into_iter()
            .zip(p)
            .map(|(a, b)| a * b)
            .sum::<f64>()
            - self.offset(index);
        if self.positive {
            distance
        } else {
            -distance
        }
    }
}
#[derive(Clone, PartialEq, serde::Serialize, serde::Deserialize)]
#[serde(default)]
pub(super) struct State {
    #[serde(deserialize_with = "read_planes")]
    pub planes: Vec<Plane>,
}
impl Default for State {
    fn default() -> Self {
        Self {
            planes: Kind::ALL[..3]
                .iter()
                .map(|&kind| Plane {
                    name: kind.label().into(),
                    kind: Some(kind),
                    ..Default::default()
                })
                .collect(),
        }
    }
}
fn read_planes<'de, D: serde::Deserializer<'de>>(d: D) -> Result<Vec<Plane>, D::Error> {
    let mut planes = <Vec<Plane> as serde::Deserialize>::deserialize(d)?;
    if planes.len() > MAX_PLANES {
        return Err(serde::de::Error::custom("Слишком много секущих плоскостей"));
    }
    for (i, p) in planes.iter_mut().enumerate() {
        let kind = p.kind(i);
        p.kind = Some(kind);
        if p.name.is_empty() {
            p.name = kind.label().into();
        }
    }
    Ok(planes)
}
impl State {
    pub fn active_planes(&self) -> impl Iterator<Item = (usize, &Plane)> {
        self.planes.iter().enumerate().filter(|(_, p)| p.enabled)
    }
    pub fn contains(&self, p: [f64; 3]) -> bool {
        self.active_planes().all(|(i, s)| s.distance(i, p) >= -1e-5)
    }
    pub fn intersects(&self, p: impl IntoIterator<Item = [f64; 3]>) -> bool {
        if !self.active() {
            return true;
        }
        let mut mask = 0_u16;
        let enabled = self
            .active_planes()
            .fold(0_u16, |mask, (i, _)| mask | (1 << i));
        for point in p {
            for (i, plane) in self.active_planes() {
                let bit = 1 << i;
                if mask & bit == 0 && plane.distance(i, point) >= -1e-5 {
                    mask |= bit;
                }
            }
            if mask == enabled {
                return true;
            }
        }
        false
    }
    pub fn segment_interval(&self, [a, b]: [[f64; 3]; 2]) -> Option<[f64; 2]> {
        let [mut lo, mut hi] = [0_f64, 1_f64];
        for (i, plane) in self.active_planes() {
            let da = plane.distance(i, a);
            let db = plane.distance(i, b);
            match (da >= 0., db >= 0.) {
                (false, false) => return None,
                (false, true) => lo = lo.max(da / (da - db)),
                (true, false) => hi = hi.min(da / (da - db)),
                (true, true) => {}
            }
        }
        (lo < hi).then_some([lo, hi])
    }
    pub fn clip_triangle_values(
        &self,
        points: [[f64; 3]; 3],
        values: [f64; 3],
        mut include: impl FnMut(f64),
    ) {
        // Every half-space can add at most one vertex to a convex polygon.
        const CAPACITY: usize = 3 + MAX_PLANES;
        let mut vertices = [([0.; 3], 0.); CAPACITY];
        let mut clipped = vertices;
        let mut count = 3;
        for (i, (point, value)) in points.into_iter().zip(values).enumerate() {
            vertices[i] = (point, value);
        }
        for (i, plane) in self.active_planes() {
            let (mut a, mut va) = vertices[count - 1];
            let mut da = plane.distance(i, a);
            let mut retained = 0;
            for &(b, vb) in &vertices[..count] {
                let db = plane.distance(i, b);
                if (da < 0. && db > 0.) || (da > 0. && db < 0.) {
                    let t = da / (da - db);
                    let point = std::array::from_fn(|k| a[k] + t * (b[k] - a[k]));
                    clipped[retained] = (point, va + t * (vb - va));
                    retained += 1;
                }
                if db >= 0. {
                    clipped[retained] = (b, vb);
                    retained += 1;
                }
                (a, va, da) = (b, vb, db);
            }
            if retained < 3 {
                return;
            }
            count = retained;
            std::mem::swap(&mut vertices, &mut clipped);
        }
        let origin = vertices[0].0;
        let has_area = vertices[1..count].windows(2).any(|pair| {
            let a = std::array::from_fn::<_, 3, _>(|k| pair[0].0[k] - origin[k]);
            let b = std::array::from_fn::<_, 3, _>(|k| pair[1].0[k] - origin[k]);
            (0..3).any(|i| {
                let j = (i + 1) % 3;
                let k = (i + 2) % 3;
                (a[j] * b[k] - a[k] * b[j]).abs() > 1e-12
            })
        });
        if has_area {
            for &(_, value) in &vertices[..count] {
                include(value);
            }
        }
    }
    pub fn active(&self) -> bool {
        self.active_planes().next().is_some()
    }
    pub fn screen_planes(&self, m: [[f32; 4]; 4], center: [f64; 3], extent: f64) -> ClipPlanes {
        let mut clips = NO_CLIPS;
        if !self.active() {
            return clips;
        }
        let inv = glam::Mat4::from_cols_array_2d(&m).inverse().transpose();
        for (dst, (i, p)) in clips.iter_mut().zip(self.active_planes()) {
            let sign = if p.positive { 1. } else { -1. };
            let n = p.normal(i);
            let w =
                (n.into_iter().zip(center).map(|(a, b)| a * b).sum::<f64>() - p.offset(i)) / extent;
            *dst = (inv
                * glam::Vec4::new(
                    (n[0] * sign) as f32,
                    (n[1] * sign) as f32,
                    (n[2] * sign) as f32,
                    (w * sign) as f32,
                ))
            .to_array();
        }
        clips
    }
}
impl App {
    pub(super) fn section_geometry_changed(&mut self) {
        // Clipping does not change the source scene, snapping geometry or FE mesh.
        self.section_view_changed();
    }
    pub(super) fn section_controls(&mut self, ui: &mut egui::Ui) {
        self.section_controls_for_view(ui, self.spatial.enabled);
    }
    pub(super) fn section_controls_for_view(&mut self, ui: &mut egui::Ui, can_pick: bool) {
        let before = self.sections.clone();
        let mut remove = None;
        ui.collapsing("Секущие плоскости", |ui| {
            for (i, p) in self.sections.planes.iter_mut().enumerate() {
                ui.push_id(i, |ui| {
                    ui.horizontal_wrapped(|ui| {
                        ui.checkbox(&mut p.enabled, "");
                        ui.add(
                            egui::TextEdit::singleline(&mut p.name)
                                .desired_width(75.)
                                .char_limit(64)
                                .hint_text("Имя"),
                        );
                        let mut kind = p.kind(i);
                        egui::ComboBox::from_id_salt("kind")
                            .width(48.)
                            .selected_text(kind.label())
                            .show_ui(ui, |ui| {
                                for k in Kind::ALL {
                                    ui.selectable_value(&mut kind, k, k.label());
                                }
                            });
                        p.kind = Some(kind);
                        if kind == Kind::VerticalAngle {
                            ui.add(
                                egui::DragValue::new(&mut p.alpha_deg)
                                    .prefix("α ")
                                    .suffix("°")
                                    .range(-360. ..=360.)
                                    .update_while_editing(false),
                            );
                        } else {
                            ui.add(
                                egui::DragValue::new(&mut p.offset_mm)
                                    .suffix(" мм")
                                    .range(-1e9..=1e9)
                                    .speed(100.)
                                    .update_while_editing(false),
                            );
                        }
                        egui::ComboBox::from_id_salt("side")
                            .width(30.)
                            .selected_text(if p.positive { "+" } else { "−" })
                            .show_ui(ui, |ui| {
                                ui.selectable_value(&mut p.positive, true, "+");
                                ui.selectable_value(&mut p.positive, false, "−");
                            });
                        if ui
                            .add_enabled(can_pick, egui::Button::new("Точка"))
                            .on_hover_text("Указать базовую точку")
                            .clicked()
                        {
                            self.section_pick_origin = Some(i);
                        }
                        if ui.button("−").on_hover_text("Удалить плоскость").clicked()
                        {
                            remove = Some(i);
                        }
                    })
                });
            }
            if ui
                .add_enabled(
                    self.sections.planes.len() < MAX_PLANES,
                    egui::Button::new("+ Плоскость"),
                )
                .clicked()
            {
                let i = self.sections.planes.len() + 1;
                self.sections.planes.push(Plane {
                    name: format!("Плоскость {i}"),
                    kind: Some(Kind::VerticalAngle),
                    ..Default::default()
                });
            }
            ui.small(format!(
                "До {MAX_PLANES} плоскостей. Z-α проходит через базовую точку параллельно оси Z."
            ));
        });
        if let Some(i) = remove {
            self.sections.planes.remove(i);
            self.section_pick_origin = None;
        }
        let signature = |s: &State| {
            s.active_planes()
                .map(|(i, p)| (p.normal(i), p.offset(i), p.positive))
                .collect::<Vec<_>>()
        };
        if signature(&before) != signature(&self.sections) {
            self.section_geometry_changed();
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn legacy_planes_migrate_and_named_z_alpha_matches_gpu_clip() {
        let mut s: State = serde_json::from_str(
            r#"{"planes":[{"enabled":true,"offset_mm":100,"positive":true}]}"#,
        )
        .unwrap();
        assert_eq!(s.planes[0].name, "XOY");
        s.planes.push(Plane {
            name: "Диагональ".into(),
            kind: Some(Kind::VerticalAngle),
            alpha_deg: 45.,
            enabled: true,
            ..Default::default()
        });
        s.planes[1].set_origin(1, [10., 30., 500.]);
        assert!(s.planes[1].distance(1, [10., 30., 200.]).abs() < 1e-8);
        s.planes[1].alpha_deg = 35.;
        assert!(s.planes[1].distance(1, [10., 30., 200.]).abs() < 1e-8);
        let m =
            glam::Mat4::from_rotation_x(0.7) * glam::Mat4::from_scale(glam::Vec3::new(2., 3., 0.4));
        let center = [40., 50., 60.];
        let planes = s.screen_planes(m.to_cols_array_2d(), center, 1000.);
        for p in [[0., 100., 200.], [100., 0., 200.], [0., 100., 0.]] {
            let [x, y, z] = std::array::from_fn(|i| ((p[i] - center[i]) / 1000.) as f32);
            let clip = m * glam::Vec4::new(x, y, z, 1.);
            assert_eq!(
                s.contains(p),
                planes
                    .iter()
                    .all(|&c| glam::Vec4::from_array(c).dot(clip) >= -1e-6)
            );
        }
        assert!(s.contains([0., 100., 200.]));
        assert!(!s.contains([100., 0., 200.]));
        assert_eq!(
            serde_json::from_value::<State>(serde_json::to_value(&s).unwrap())
                .unwrap()
                .planes[1]
                .name,
            "Диагональ"
        );
    }
}
