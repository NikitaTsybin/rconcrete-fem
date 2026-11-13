use super::{geometry::Point, source_dxf::Color};
use glam::{DMat4, DVec3};
use std::collections::{BTreeMap, HashMap};
const LIMIT: usize = 500_000;
#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Path2 {
    pub layer: String,
    pub points: Vec<Point>,
    pub color: Color,
    pub closed: bool,
}
#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Label {
    pub layer: String,
    pub anchor: [f32; 2],
    pub width_factor: f64,
    pub point: Point,
    pub text: String,
    pub height: f64,
    pub angle: f64,
    pub color: Color,
}
#[derive(serde::Serialize, serde::Deserialize, Clone)]
pub struct Layer {
    pub name: String,
    pub color: Color,
    pub visible: bool,
}
#[derive(serde::Serialize, serde::Deserialize, Clone, Default)]
pub struct Drawing {
    pub paths: Vec<Path2>,
    pub fills: Vec<(Vec<[Point; 3]>, Color, String)>,
    pub layers: Vec<Layer>,
    pub node_layers: Vec<String>,
    pub labels: Vec<Label>,
    pub nodes: Vec<Point>,
    pub skipped: BTreeMap<String, usize>,
    pub unit_mm: f64,
    pub name: String,
}
impl Drawing {
    pub fn bounds(&self) -> Option<[f64; 4]> {
        let mut b = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for p in self
            .paths
            .iter()
            .flat_map(|r| r.points.iter())
            .chain(self.labels.iter().map(|t| &t.point))
            .chain(&self.nodes)
        {
            b[0] = b[0].min(p[0]);
            b[1] = b[1].min(p[1]);
            b[2] = b[2].max(p[0]);
            b[3] = b[3].max(p[1]);
        }
        b[0].is_finite().then_some(b)
    }
}
#[derive(Clone)]
struct Record {
    kind: String,
    tags: Vec<(i32, String)>,
}
impl Record {
    fn s(&self, c: i32) -> &str {
        self.tags
            .iter()
            .find(|p| p.0 == c)
            .map_or("", |p| p.1.as_str())
    }
    fn n(&self, c: i32, d: f64) -> f64 {
        self.s(c)
            .parse::<f64>()
            .ok()
            .filter(|n| n.is_finite())
            .unwrap_or(d)
    }
    fn point(&self, c: i32) -> DVec3 {
        DVec3::new(self.n(c, 0.), self.n(c + 10, 0.), self.n(c + 20, 0.))
    }
}
struct Block {
    base: DVec3,
    records: Vec<Record>,
}
struct Reader {
    current_layer: String,
    blocks: HashMap<String, Block>,
    layers: HashMap<String, (Color, bool)>,
    drawing: Drawing,
    count: usize,
}
fn ocs(r: &Record) -> DMat4 {
    let n = DVec3::new(r.n(210, 0.), r.n(220, 0.), r.n(230, 1.)).normalize_or_zero();
    if n.length_squared() < 0.5 {
        return DMat4::IDENTITY;
    }
    let x = if n.x.abs() < 1. / 64. && n.y.abs() < 1. / 64. {
        DVec3::Y.cross(n)
    } else {
        DVec3::Z.cross(n)
    }
    .normalize();
    let y = n.cross(x);
    DMat4::from_cols(
        x.extend(0.),
        y.extend(0.),
        n.extend(0.),
        DVec3::ZERO.extend(1.),
    )
}
fn decode_text(text: &str) -> String {
    let mut out = String::new();
    let mut chars = text.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '{' || c == '}' {
            continue;
        }
        if c != '\\' {
            out.push(c);
            continue;
        }
        match chars.next() {
            Some('P') => out.push('\n'),
            Some('~') => out.push(' '),
            Some('\\') => out.push('\\'),
            Some('U') => {
                if chars.next() == Some('+') {
                    let hex = chars.by_ref().take(4).collect::<String>();
                    if let Ok(v) = u32::from_str_radix(&hex, 16) {
                        if let Some(c) = char::from_u32(v) {
                            out.push(c);
                        }
                    }
                }
            }
            Some('L' | 'l' | 'O' | 'o') => {}
            Some(_) => {
                while chars.peek().is_some_and(|c| *c != ';') {
                    chars.next();
                }
                if chars.peek() == Some(&';') {
                    chars.next();
                }
            }
            None => {}
        }
    }
    out
}
impl Reader {
    fn skip(&mut self, kind: &str) {
        *self.drawing.skipped.entry(kind.into()).or_default() += 1;
    }
    fn color(&self, r: &Record, layer: &str, parent: Color) -> Color {
        if !r.s(420).is_empty() {
            return Color::Rgb(r.n(420, 0.) as u32 & 0xffffff);
        }
        match r.n(62, 256.) as i32 {
            0 => parent,
            256 => self.layers.get(layer).map_or(Color::Aci(7), |v| v.0),
            n => Color::Aci(n.unsigned_abs().min(255) as u16),
        }
    }
    fn add(
        &mut self,
        points: Vec<DVec3>,
        matrix: DMat4,
        color: Color,
        closed: bool,
        nodes: bool,
    ) -> Result<(), String> {
        if points.len() < 2 {
            return Ok(());
        }
        self.count += points.len();
        if self.count > LIMIT {
            return Err("DXF слишком большой: более 500000 точек представления".into());
        }
        let p = points
            .into_iter()
            .map(|p| matrix.transform_point3(p))
            .map(|p| [p.x * self.drawing.unit_mm, p.y * self.drawing.unit_mm])
            .collect::<Vec<_>>();
        if p.iter().flatten().any(|v| !v.is_finite() || v.abs() > 1e15) {
            return Err("Некорректные координаты DXF".into());
        }
        if nodes {
            self.drawing.nodes.extend(&p);
            self.drawing
                .node_layers
                .extend(std::iter::repeat_n(self.current_layer.clone(), p.len()));
        }
        self.drawing.paths.push(Path2 {
            layer: self.current_layer.clone(),
            points: p,
            color,
            closed,
        });
        Ok(())
    }
    fn node(&mut self, p: DVec3, m: DMat4) {
        let p = m.transform_point3(p);
        self.drawing
            .nodes
            .push([p.x * self.drawing.unit_mm, p.y * self.drawing.unit_mm]);
        self.drawing.node_layers.push(self.current_layer.clone());
    }
    fn arc(
        &mut self,
        center: DVec3,
        rx: f64,
        ry: f64,
        start: f64,
        sweep: f64,
        m: DMat4,
        color: Color,
        closed: bool,
    ) -> Result<(), String> {
        let radius = m
            .transform_vector3(DVec3::X * rx)
            .length()
            .max(m.transform_vector3(DVec3::Y * ry).length())
            * self.drawing.unit_mm;
        let max_angle = if radius > 0.25 {
            2. * (1. - 0.25 / radius).clamp(-1., 1.).acos()
        } else {
            0.2
        };
        let n = (sweep.abs() / max_angle.max(0.002)).ceil().clamp(8., 4096.) as usize;
        let points = (0..=n)
            .map(|i| {
                let a = start + sweep * i as f64 / n as f64;
                center + DVec3::new(rx * a.cos(), ry * a.sin(), 0.)
            })
            .collect::<Vec<_>>();
        self.node(points[0], m);
        if !closed {
            self.node(points[n], m);
        }
        self.node(center, m);
        self.add(
            if closed { points[..n].to_vec() } else { points },
            m,
            color,
            closed,
            false,
        )
    }
    fn poly(
        &mut self,
        vertices: &[(DVec3, f64)],
        closed: bool,
        m: DMat4,
        color: Color,
    ) -> Result<(), String> {
        for (p, _) in vertices {
            self.node(*p, m);
        }
        let mut points = Vec::new();
        let count = if closed {
            vertices.len()
        } else {
            vertices.len().saturating_sub(1)
        };
        for i in 0..count {
            let (a, bulge) = vertices[i];
            let b = vertices[(i + 1) % vertices.len()].0;
            points.push(a);
            let chord = (b - a).truncate().length();
            if bulge.abs() > 1e-10 && chord > 1e-10 {
                let theta = 4. * bulge.atan();
                let dir = (b - a) / chord;
                let center = (a + b) * 0.5
                    + DVec3::new(-dir.y, dir.x, 0.) * (chord * (1. - bulge * bulge) / (4. * bulge));
                let radius = (a - center).truncate().length();
                let start = (a.y - center.y).atan2(a.x - center.x);
                let n = (theta.abs() / 0.04).ceil().clamp(2., 2048.) as usize;
                for j in 1..n {
                    let t = start + theta * j as f64 / n as f64;
                    points.push(DVec3::new(
                        center.x + radius * t.cos(),
                        center.y + radius * t.sin(),
                        a.z + (b.z - a.z) * j as f64 / n as f64,
                    ));
                }
            }
        }
        if !closed {
            if let Some(v) = vertices.last() {
                points.push(v.0);
            }
        }
        self.add(points, m, color, closed, false)
    }
    fn expand(
        &mut self,
        records: &[Record],
        matrix: DMat4,
        parent_layer: &str,
        parent_color: Color,
        depth: usize,
    ) -> Result<(), String> {
        if depth > 16 {
            return Err("Слишком глубокая вложенность блоков DXF".into());
        }
        let mut i = 0;
        while i < records.len() {
            let r = &records[i];
            i += 1;
            self.count += 1;
            if self.count > LIMIT {
                return Err("Превышен лимит раскрытия объектов DXF".into());
            }
            let layer = if r.s(8).is_empty() || r.s(8) == "0" {
                parent_layer
            } else {
                r.s(8)
            };
            if r.n(60, 0.) == 1. {
                continue;
            }
            self.current_layer = layer.to_owned();
            let color = self.color(r, layer, parent_color);
            match r.kind.as_str() {
                "INSERT" => {
                    let Some(block) = self.blocks.get(r.s(2)) else {
                        self.skip("INSERT: нет блока");
                        continue;
                    };
                    let base = block.base;
                    let children = block.records.clone();
                    let rows = r.n(71, 1.).clamp(1., 1000.) as usize;
                    let cols = r.n(70, 1.).clamp(1., 1000.) as usize;
                    if rows * cols > 10000 {
                        return Err("Слишком большой массив блоков DXF".into());
                    }
                    for row in 0..rows {
                        for col in 0..cols {
                            let m = matrix
                                * ocs(r)
                                * DMat4::from_translation(r.point(10))
                                * DMat4::from_rotation_z(r.n(50, 0.).to_radians())
                                * DMat4::from_translation(DVec3::new(
                                    col as f64 * r.n(44, 0.),
                                    row as f64 * r.n(45, 0.),
                                    0.,
                                ))
                                * DMat4::from_scale(DVec3::new(
                                    r.n(41, 1.),
                                    r.n(42, 1.),
                                    r.n(43, 1.),
                                ))
                                * DMat4::from_translation(-base);
                            self.expand(&children, m, layer, color, depth + 1)?;
                        }
                    }
                }
                "LINE" => self.add(vec![r.point(10), r.point(11)], matrix, color, false, true)?,
                "POINT" => self.node(r.point(10), matrix),
                "LWPOLYLINE" => {
                    let mut vertices = Vec::new();
                    for (c, v) in &r.tags {
                        match c {
                            10 => vertices.push((
                                DVec3::new(v.parse().unwrap_or(f64::NAN), 0., r.n(38, 0.)),
                                0.,
                            )),
                            20 => {
                                if let Some((p, _)) = vertices.last_mut() {
                                    p.y = v.parse().unwrap_or(f64::NAN);
                                }
                            }
                            42 => {
                                if let Some((_, b)) = vertices.last_mut() {
                                    *b = v.parse().unwrap_or(0.);
                                }
                            }
                            _ => {}
                        }
                    }
                    self.poly(
                        &vertices,
                        r.n(70, 0.) as i32 & 1 != 0,
                        matrix * ocs(r),
                        color,
                    )?;
                }
                "POLYLINE" => {
                    let mut vertices = Vec::new();
                    let flags = r.n(70, 0.) as i32;
                    while i < records.len() && records[i].kind == "VERTEX" {
                        let v = &records[i];
                        i += 1;
                        vertices.push((v.point(10), v.n(42, 0.)));
                    }
                    if i < records.len() && records[i].kind == "SEQEND" {
                        i += 1;
                    }
                    if flags & (16 | 64) != 0 {
                        self.skip("POLYLINE: mesh/polyface");
                        continue;
                    }
                    let m = if flags & 8 != 0 {
                        matrix
                    } else {
                        matrix * ocs(r) * DMat4::from_translation(DVec3::new(0., 0., r.n(30, 0.)))
                    };
                    self.poly(&vertices, flags & 1 != 0, m, color)?;
                }
                "CIRCLE" | "ARC" => {
                    let closed = r.kind == "CIRCLE";
                    let start = if closed { 0. } else { r.n(50, 0.).to_radians() };
                    let sweep = if closed {
                        std::f64::consts::TAU
                    } else {
                        (r.n(51, 0.).to_radians() - start).rem_euclid(std::f64::consts::TAU)
                    };
                    self.arc(
                        r.point(10),
                        r.n(40, 0.),
                        r.n(40, 0.),
                        start,
                        sweep,
                        matrix * ocs(r),
                        color,
                        closed,
                    )?;
                }
                "ELLIPSE" => {
                    let major = r.point(11);
                    let rx = major.length();
                    let normal =
                        DVec3::new(r.n(210, 0.), r.n(220, 0.), r.n(230, 1.)).normalize_or_zero();
                    let ex = major.normalize_or_zero();
                    let ey = normal.cross(ex).normalize_or_zero();
                    let basis = DMat4::from_cols(
                        ex.extend(0.),
                        ey.extend(0.),
                        normal.extend(0.),
                        DVec3::ZERO.extend(1.),
                    );
                    let m = matrix * DMat4::from_translation(r.point(10)) * basis;
                    let start = r.n(41, 0.);
                    let end = r.n(42, std::f64::consts::TAU);
                    let sweep = end - start;
                    self.arc(
                        DVec3::ZERO,
                        rx,
                        rx * r.n(40, 1.),
                        start,
                        sweep,
                        m,
                        color,
                        (sweep.abs() - std::f64::consts::TAU).abs() < 1e-8,
                    )?;
                }
                "TEXT" | "MTEXT" | "ATTRIB" => {
                    let raw = if r.kind == "MTEXT" {
                        r.tags
                            .iter()
                            .filter(|p| p.0 == 1 || p.0 == 3)
                            .map(|p| p.1.as_str())
                            .collect::<String>()
                    } else {
                        r.s(1).into()
                    };
                    let m = matrix * ocs(r);
                    let is_mtext = r.kind == "MTEXT";
                    let horizontal = r.n(72, 0.) as i32;
                    let vertical = r.n(73, 0.) as i32;
                    let p = m.transform_point3(
                        if !is_mtext && (horizontal != 0 || vertical != 0) && !r.s(11).is_empty() {
                            r.point(11)
                        } else {
                            r.point(10)
                        },
                    );
                    let direction = if is_mtext && !r.s(11).is_empty() {
                        m.transform_vector3(r.point(11))
                    } else {
                        m.transform_vector3(DVec3::new(
                            r.n(50, 0.).to_radians().cos(),
                            r.n(50, 0.).to_radians().sin(),
                            0.,
                        ))
                    };
                    let anchor = if is_mtext {
                        let n = (r.n(71, 1.) as usize).clamp(1, 9) - 1;
                        [(n % 3) as f32 * 0.5, (n / 3) as f32 * 0.5]
                    } else {
                        [
                            match horizontal {
                                1 | 4 => 0.5,
                                2 => 1.,
                                _ => 0.,
                            },
                            match vertical {
                                2 => 0.5,
                                3 => 0.,
                                _ => 1.,
                            },
                        ]
                    };
                    self.drawing.labels.push(Label {
                        layer: self.current_layer.clone(),
                        anchor,
                        width_factor: if is_mtext { 1. } else { r.n(41, 1.).max(0.01) },
                        point: [p.x * self.drawing.unit_mm, p.y * self.drawing.unit_mm],
                        text: decode_text(&raw),
                        height: r.n(40, 2.5) * direction.length() * self.drawing.unit_mm,
                        angle: direction.y.atan2(direction.x),
                        color,
                    });
                }
                "3DFACE" | "SOLID" | "HATCH" => {
                    match super::source_dxf::drawing_rings(&r.kind, &r.tags) {
                        Ok(rings) => {
                            if r.kind != "HATCH" || r.n(70, 1.) == 1. {
                                let world = rings
                                    .iter()
                                    .map(|ring| {
                                        ring.iter()
                                            .map(|p| {
                                                let p =
                                                    matrix.transform_point3(DVec3::from_array(*p));
                                                [
                                                    p.x * self.drawing.unit_mm,
                                                    p.y * self.drawing.unit_mm,
                                                ]
                                            })
                                            .collect::<Vec<_>>()
                                    })
                                    .collect::<Vec<_>>();
                                if let Ok(triangles) = super::geometry::triangulate(&world) {
                                    self.drawing.fills.push((
                                        triangles,
                                        color,
                                        self.current_layer.clone(),
                                    ));
                                }
                            } else {
                                self.skip("Штриховка показана границами");
                            }
                            for ring in rings {
                                self.add(
                                    ring.into_iter().map(DVec3::from_array).collect(),
                                    matrix,
                                    color,
                                    true,
                                    true,
                                )?;
                            }
                        }
                        Err(e) => self.skip(&format!("{}: {}", r.kind, e)),
                    }
                }
                "SEQEND" | "VERTEX" => {}
                _ => self.skip(&r.kind),
            }
            if self.drawing.paths.len() + self.drawing.labels.len() + self.drawing.nodes.len()
                > LIMIT
            {
                return Err("Слишком много объектов DXF".into());
            }
        }
        Ok(())
    }
}
pub fn read(path: &std::path::Path) -> Result<Drawing, String> {
    let b = std::fs::read(path).map_err(|e| e.to_string())?;
    from_bytes(&b, &path.file_name().unwrap_or_default().to_string_lossy())
}
pub fn from_bytes(bytes: &[u8], name: &str) -> Result<Drawing, String> {
    if bytes.starts_with(b"AutoCAD Binary DXF") {
        return Err("Нужен текстовый DXF; бинарный DXF пока не поддержан".into());
    }
    let text = match std::str::from_utf8(bytes) {
        Ok(s) => s.trim_start_matches('\u{feff}').to_owned(),
        Err(_) => encoding_rs::WINDOWS_1251.decode(bytes).0.into_owned(),
    };
    let mut pairs = Vec::new();
    let mut lines = text.lines();
    while let Some(code) = lines.next() {
        let value = lines.next().ok_or("Нечётное число строк DXF")?;
        pairs.push((
            code.trim()
                .parse::<i32>()
                .map_err(|_| "Некорректный код DXF")?,
            value.trim().to_owned(),
        ));
    }
    let unit = pairs
        .windows(2)
        .find(|p| p[0].0 == 9 && p[0].1 == "$INSUNITS")
        .and_then(|p| p[1].1.parse::<i32>().ok());
    let unit_mm = match unit {
        Some(1) => 25.4,
        Some(2) => 304.8,
        Some(3) => 1609344.,
        Some(4) => 1.,
        Some(5) => 10.,
        Some(6) => 1000.,
        Some(7) => 1e6,
        Some(8) => 0.0000254,
        Some(9) => 0.0254,
        Some(10) => 914.4,
        Some(11) => 1e-7,
        Some(12) => 1e-6,
        Some(13) => 0.001,
        Some(14) => 100.,
        Some(15) => 1e4,
        Some(16) => 1e5,
        Some(17) => 1e12,
        None | Some(0) => 1.,
        _ => return Err("Неподдерживаемые единицы DXF".into()),
    };
    let mut records: Vec<Record> = Vec::new();
    for p in pairs {
        if p.0 == 0 {
            records.push(Record {
                kind: p.1.to_uppercase(),
                tags: vec![],
            });
        } else if let Some(r) = records.last_mut() {
            r.tags.push(p);
        }
    }
    let mut reader = Reader {
        current_layer: "0".into(),
        blocks: HashMap::new(),
        layers: HashMap::new(),
        drawing: Drawing {
            unit_mm,
            name: name.into(),
            ..Default::default()
        },
        count: 0,
    };
    let mut section = String::new();
    let mut model = Vec::new();
    let mut i = 0;
    while i < records.len() {
        let r = &records[i];
        i += 1;
        if r.kind == "SECTION" {
            section = r.s(2).to_uppercase();
            continue;
        }
        if r.kind == "ENDSEC" {
            section.clear();
            continue;
        }
        match (section.as_str(), r.kind.as_str()) {
            ("TABLES", "LAYER") => {
                let c = if !r.s(420).is_empty() {
                    Color::Rgb(r.n(420, 0.) as u32)
                } else {
                    Color::Aci(r.n(62, 7.).abs() as u16)
                };
                reader.layers.insert(
                    r.s(2).into(),
                    (c, r.n(62, 7.) >= 0. && r.n(70, 0.) as i32 & 1 == 0),
                );
            }
            ("BLOCKS", "BLOCK") => {
                let name = r.s(2).to_owned();
                let base = r.point(10);
                let mut children = Vec::new();
                while i < records.len() && records[i].kind != "ENDBLK" {
                    children.push(records[i].clone());
                    i += 1;
                }
                if i < records.len() {
                    i += 1;
                }
                reader.blocks.insert(
                    name,
                    Block {
                        base,
                        records: children,
                    },
                );
            }
            ("ENTITIES", _) => model.push(r.clone()),
            _ => {}
        }
    }
    if unit.is_none() || unit == Some(0) {
        reader.skip("Единицы DXF не заданы: принято 1 ед. = 1 мм");
    }
    reader.expand(&model, DMat4::IDENTITY, "0", Color::Aci(7), 0)?;
    if reader.drawing.bounds().is_none() {
        return Err("В DXF нет поддерживаемой геометрии для отображения".into());
    }
    let mut names = reader.layers.keys().cloned().collect::<Vec<_>>();
    names.extend(reader.drawing.paths.iter().map(|p| p.layer.clone()));
    names.extend(reader.drawing.labels.iter().map(|p| p.layer.clone()));
    names.extend(reader.drawing.node_layers.iter().cloned());
    names.sort();
    names.dedup();
    reader.drawing.layers = names
        .into_iter()
        .map(|name| {
            let (color, visible) = reader
                .layers
                .get(&name)
                .copied()
                .unwrap_or((Color::Aci(7), true));
            Layer {
                name,
                color,
                visible,
            }
        })
        .collect();
    Ok(reader.drawing)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn lines_at_different_z_flatten_and_keep_truecolor_and_layer_color() {
        let d=from_bytes(b"0\nSECTION\n2\nTABLES\n0\nLAYER\n2\na\n62\n3\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nLINE\n8\na\n10\n1\n20\n2\n30\n999\n11\n3\n21\n4\n31\n-900\n0\nLINE\n420\n16711680\n10\n5\n20\n6\n11\n7\n21\n8\n0\nENDSEC\n0\nEOF\n","test").unwrap();
        assert_eq!(d.paths[0].points, vec![[1., 2.], [3., 4.]]);
        assert_eq!(d.paths[0].color, Color::Aci(3));
        assert_eq!(d.paths[1].color, Color::Rgb(0xff0000));
    }
    #[test]
    fn blocks_and_bulges_have_transformed_nodes_and_curved_edges() {
        let d=from_bytes(b"0\nSECTION\n2\nBLOCKS\n0\nBLOCK\n2\nb\n10\n10\n20\n0\n0\nLWPOLYLINE\n62\n0\n70\n0\n10\n10\n20\n0\n42\n1\n10\n12\n20\n0\n0\nENDBLK\n0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nINSERT\n2\nb\n62\n5\n10\n100\n20\n200\n30\n500\n50\n90\n0\nENDSEC\n0\nEOF\n","test").unwrap();
        assert_eq!(d.paths[0].color, Color::Aci(5));
        assert!(d.paths[0].points.len() > 10);
        assert!((d.nodes[0][0] - 100.).abs() < 1e-9);
        assert!((d.nodes[1][1] - 202.).abs() < 1e-9);
    }
    #[test]
    fn centered_text_and_mtext_keep_layer_alignment_and_disabled_geometry() {
        let d = from_bytes(
            b"0
SECTION
2
TABLES
0
LAYER
2
off
62
-3
0
ENDSEC
0
SECTION
2
ENTITIES
0
TEXT
8
off
10
0
20
0
11
100
21
200
40
70
72
1
73
2
1
12/250
0
MTEXT
8
notes
10
300
20
400
40
100
71
5
11
0
21
1
1
Test\\Psecond
0
LINE
8
off
10
0
20
0
11
100
21
100
0
ENDSEC
0
EOF
",
            "text",
        )
        .unwrap();
        assert_eq!(d.labels[0].point, [100., 200.]);
        assert_eq!(d.labels[0].anchor, [0.5, 0.5]);
        assert_eq!(d.labels[0].layer, "off");
        assert_eq!(
            d.labels[1].text,
            "Test
second"
        );
        assert_eq!(d.labels[1].anchor, [0.5, 0.5]);
        assert!((d.labels[1].angle - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
        assert!(d.layers.iter().any(|l| l.name == "off" && !l.visible));
        assert_eq!(d.paths[0].layer, "off");
        assert_eq!(d.node_layers, vec!["off", "off"]);
    }
    #[test]
    fn invalid_and_empty_inputs_return_errors() {
        assert!(from_bytes(b"0\nLINE\n10", "x").is_err());
        assert!(from_bytes(b"AutoCAD Binary DXF", "x").is_err());
        assert!(from_bytes(b"0\nEOF\n", "x").is_err());
    }
}
