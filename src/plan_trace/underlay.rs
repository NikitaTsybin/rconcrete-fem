use super::{
    geometry::Point,
    import::{Drawing, Label},
};
use serde::{Deserialize, Serialize};

#[derive(Clone, Serialize, Deserialize)]
pub struct Underlay {
    pub id: u64,
    pub name: String,
    pub visible: bool,
    #[serde(default)]
    pub transparency: f32,
    pub scale: f64,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub scale_y: Option<f64>,
    #[serde(default, skip_serializing_if = "zero_angle")]
    pub angle_rad: f64,
    pub origin: Point,
    pub data: Data,
}
#[derive(Clone, Serialize, Deserialize)]
pub enum Data {
    Dxf(Drawing),
    Pdf {
        width: u32,
        height: u32,
        rgba: Vec<u8>,
        size_mm: Point,
        page: u32,
    },
}
fn zero_angle(angle: &f64) -> bool {
    *angle == 0.
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Transform {
    pub origin: Point,
    pub scale: Point,
    pub angle: f64,
}
impl Default for Transform {
    fn default() -> Self {
        Self {
            origin: [0.; 2],
            scale: [1.; 2],
            angle: 0.,
        }
    }
}
impl Transform {
    pub fn vector(self, p: Point) -> Point {
        let [x, y] = [p[0] * self.scale[0], p[1] * self.scale[1]];
        if self.angle == 0. {
            return [x, y];
        }
        let (sin, cos) = self.angle.sin_cos();
        [cos * x - sin * y, sin * x + cos * y]
    }
    pub fn point(self, p: Point) -> Point {
        let [x, y] = self.vector(p);
        [self.origin[0] + x, self.origin[1] + y]
    }
    pub fn inverse_vector(self, p: Point) -> Point {
        if self.angle == 0. {
            return [p[0] / self.scale[0], p[1] / self.scale[1]];
        }
        let (sin, cos) = self.angle.sin_cos();
        [
            (cos * p[0] + sin * p[1]) / self.scale[0],
            (-sin * p[0] + cos * p[1]) / self.scale[1],
        ]
    }
    pub fn inverse(self, p: Point) -> Point {
        self.inverse_vector([p[0] - self.origin[0], p[1] - self.origin[1]])
    }
    pub fn validate(self) -> Result<(), String> {
        if !self.angle.is_finite()
            || self.origin.iter().any(|v| !v.is_finite())
            || self.scale.iter().any(|v| !v.is_finite() || *v <= 0.)
        {
            return Err("Некорректный масштаб, угол или положение подложки".into());
        }
        Ok(())
    }
    pub fn transform_label(self, label: &mut Label) {
        map_label(label, |p| self.point(p), |p| self.vector(p));
    }
}
pub(super) fn map_label(
    label: &mut Label,
    point: impl Fn(Point) -> Point,
    vector: impl Fn(Point) -> Point,
) {
    let (sin, cos) = label.angle.sin_cos();
    let [x, y] = vector([cos, sin]);
    let [u, v] = vector([-sin, cos]);
    let width = x.hypot(y);
    let height = ((x / width) * v - (y / width) * u).abs();
    label.point = point(label.point);
    label.angle = y.atan2(x);
    label.height *= height;
    label.width_factor *= width / height;
}

impl Underlay {
    pub fn transform(&self) -> Transform {
        Transform {
            origin: self.origin,
            scale: [self.scale, self.scale_y.unwrap_or(self.scale)],
            angle: self.angle_rad,
        }
    }
    pub fn set_transform(&mut self, transform: Transform) {
        self.origin = transform.origin;
        self.scale = transform.scale[0];
        self.scale_y = (transform.scale[0] != transform.scale[1]).then_some(transform.scale[1]);
        self.angle_rad = transform.angle;
    }
    pub fn point(&self, p: Point) -> Point {
        self.transform().point(p)
    }
    pub fn local_bounds(&self) -> Option<[f64; 4]> {
        match &self.data {
            Data::Dxf(drawing) => drawing.bounds(),
            Data::Pdf { size_mm, .. } => Some([0., 0., size_mm[0], size_mm[1]]),
        }
    }
    pub fn corners(&self) -> Option<[Point; 4]> {
        let [x0, y0, x1, y1] = self.local_bounds()?;
        Some([[x0, y0], [x1, y0], [x1, y1], [x0, y1]].map(|p| self.point(p)))
    }
    pub fn contains(&self, point: Point) -> bool {
        self.local_bounds().is_some_and(|[x0, y0, x1, y1]| {
            let [x, y] = self.transform().inverse(point);
            (x0 - 1e-8..=x1 + 1e-8).contains(&x) && (y0 - 1e-8..=y1 + 1e-8).contains(&y)
        })
    }
    pub fn validate(&self) -> Result<(), String> {
        self.transform().validate()?;
        if !self.transparency.is_finite() || !(0. ..=1.).contains(&self.transparency) {
            return Err("Некорректная прозрачность подложки".into());
        }
        if let Data::Pdf {
            width,
            height,
            rgba,
            size_mm,
            ..
        } = &self.data
        {
            let pixels = (*width as u64) * (*height as u64);
            if *width == 0
                || *height == 0
                || pixels > 16_777_216
                || pixels * 4 != rgba.len() as u64
                || size_mm.iter().any(|v| !v.is_finite() || *v <= 0.)
            {
                return Err("Некорректная страница PDF".into());
            }
        }
        Ok(())
    }
    pub fn drawing(&self) -> Option<Drawing> {
        let Data::Dxf(source) = &self.data else {
            return None;
        };
        if !self.visible || self.transparency >= 1. {
            return None;
        }
        let mut d = source.clone();
        let layer = |name: &str| format!("#{} · {name}", self.id);
        for path in &mut d.paths {
            path.layer = layer(&path.layer);
            for p in &mut path.points {
                *p = self.point(*p);
            }
        }
        for (triangles, _, name) in &mut d.fills {
            *name = layer(name);
            for p in triangles.iter_mut().flatten() {
                *p = self.point(*p);
            }
        }
        for l in &mut d.labels {
            l.layer = layer(&l.layer);
            self.transform().transform_label(l);
        }
        for p in &mut d.nodes {
            *p = self.point(*p);
        }
        for l in &mut d.node_layers {
            *l = layer(l);
        }
        for l in &mut d.layers {
            l.name = layer(&l.name);
        }
        d.unit_mm = 1.;
        Some(d)
    }
}

pub fn from_pdf_bytes(bytes: Vec<u8>, name: &str) -> Result<Vec<Underlay>, String> {
    let pages = super::pdf_import::decode(bytes)?;
    let mut offset = 0.;
    Ok(pages
        .into_iter()
        .enumerate()
        .map(|(i, p)| {
            let origin = [offset, 0.];
            offset += p.size_mm[0] + 20.;
            Underlay {
                transparency: 0.,
                id: 0,
                name: format!("{name} · стр. {}", i + 1),
                visible: true,
                scale: 1.,
                scale_y: None,
                angle_rad: 0.,
                origin,
                data: Data::Pdf {
                    width: p.width,
                    height: p.height,
                    rgba: p.rgba,
                    size_mm: p.size_mm,
                    page: i as u32 + 1,
                },
            }
        })
        .collect())
}

/// Import exactly the selected page and retain its original one-based number.
pub fn from_pdf_page(bytes: Vec<u8>, name: &str, index: usize) -> Result<Underlay, String> {
    let p = super::pdf_import::decode_page(bytes, index)?;
    Ok(Underlay {
        transparency: 0.,
        id: 0,
        name: format!("{name} · стр. {}", index + 1),
        visible: true,
        scale: 1.,
        scale_y: None,
        angle_rad: 0.,
        origin: [0., 0.],
        data: Data::Pdf {
            width: p.width,
            height: p.height,
            rgba: p.rgba,
            size_mm: p.size_mm,
            page: index as u32 + 1,
        },
    })
}

#[cfg(test)]
mod opacity_tests {
    use super::*;
    #[test]
    fn opacity_is_independent_persistent_and_defaults_to_opaque_for_old_projects() {
        let mut a = Underlay {
            id: 1,
            name: "page.pdf".into(),
            visible: true,
            transparency: 0.3,
            scale: 1.,
            scale_y: None,
            angle_rad: 0.,
            origin: [0., 0.],
            data: Data::Pdf {
                width: 1,
                height: 1,
                rgba: vec![10, 20, 30, 255],
                size_mm: [210., 297.],
                page: 1,
            },
        };
        let mut b = a.clone();
        b.id = 2;
        b.transparency = 0.7;
        assert_eq!(a.transparency, 0.3);
        assert_eq!(b.transparency, 0.7);
        let value = serde_json::to_value(&a).unwrap();
        let restored: Underlay = serde_json::from_value(value.clone()).unwrap();
        assert_eq!(restored.transparency, 0.3);
        let mut legacy = value;
        legacy.as_object_mut().unwrap().remove("transparency");
        let restored: Underlay = serde_json::from_value(legacy).unwrap();
        assert_eq!(restored.transparency, 0.);
        for invalid in [-0.1, 1.1, f32::NAN] {
            a.transparency = invalid;
            assert!(a.validate().is_err());
        }
        let Data::Pdf { rgba, .. } = &b.data else {
            panic!("PDF");
        };
        assert_eq!(rgba, &vec![10, 20, 30, 255]);
    }
}
