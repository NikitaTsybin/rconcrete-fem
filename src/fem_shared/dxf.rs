use super::geometry::{self, Point};
use std::collections::{BTreeMap, HashMap};
#[cfg(not(target_arch = "wasm32"))]
use std::path::Path;

#[derive(
    Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, Debug, serde::Serialize, serde::Deserialize,
)]
pub enum Color {
    Aci(u16),
    Rgb(u32),
}
impl Color {
    pub fn label(self) -> String {
        match self {
            Self::Aci(n) => format!("ACI {n}"),
            Self::Rgb(n) => format!("#{n:06X}"),
        }
    }
    pub fn rgb(self) -> [u8; 3] {
        match self {
            Self::Rgb(n) => [(n >> 16) as u8, (n >> 8) as u8, n as u8],
            Self::Aci(n) => {
                let basic = [
                    [220, 220, 220],
                    [255, 0, 0],
                    [255, 255, 0],
                    [0, 255, 0],
                    [0, 255, 255],
                    [0, 0, 255],
                    [255, 0, 255],
                    [235, 235, 235],
                    [128, 128, 128],
                    [192, 192, 192],
                ];
                if n < 10 {
                    return basic[n as usize];
                }
                if n >= 250 {
                    let g = [51, 80, 105, 130, 190, 255][(n.min(255) - 250) as usize];
                    return [g, g, g];
                }
                let h = ((n - 10) / 10) as f64 * 15.;
                let shade = ((n - 10) % 10) as usize;
                let v = [1., 1., 0.65, 0.65, 0.5, 0.5, 0.3, 0.3, 0.15, 0.15][shade];
                let s = if shade % 2 == 0 { 1. } else { 0.5 };
                let c = v * s;
                let x = c * (1. - ((h / 60.) % 2. - 1.).abs());
                let m = v - c;
                let q = match (h / 60.) as usize {
                    0 => [c, x, 0.],
                    1 => [x, c, 0.],
                    2 => [0., c, x],
                    3 => [0., x, c],
                    4 => [x, 0., c],
                    _ => [c, 0., x],
                };
                q.map(|v| ((v + m) * 255.).round() as u8)
            }
        }
    }
}
#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct DxfReinforcement {
    pub component: usize,
    pub expression: String,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Class {
    pub color: Color,
    pub value: f64,
    pub range: Option<[f64; 2]>,
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub reinforcement: Option<DxfReinforcement>,
}
/// Source identity and result axes, independent of 2D render triangles.
#[derive(Clone, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "camelCase")]
pub struct SourceElement {
    pub node_ids: Vec<usize>,
    pub xyz: Vec<[f64; 3]>,
    pub direction: [f64; 3],
    pub raw_values: [f64; 4],
    pub definition_index: u16,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Face {
    pub id: String,
    pub layer: String,
    pub rings: Vec<Vec<Point>>,
    #[serde(skip_serializing, default)]
    pub triangles: Vec<[Point; 3]>,
    pub color: usize,
    pub z: f64,
    pub level: usize,
    #[serde(skip_serializing, default)]
    pub bbox: [f64; 4],
    pub source: Option<SourceElement>,
}
impl Face {
    pub fn center(&self) -> Point {
        let mut sum = [0.; 2];
        let mut weight = 0.;
        for t in &self.triangles {
            let a = geometry::area(t);
            weight += a;
            for k in 0..2 {
                sum[k] += a * (t[0][k] + t[1][k] + t[2][k]) / 3.;
            }
        }
        if weight > 0. {
            [sum[0] / weight, sum[1] / weight]
        } else {
            [
                (self.bbox[0] + self.bbox[2]) * 0.5,
                (self.bbox[1] + self.bbox[3]) * 0.5,
            ]
        }
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub struct Dataset {
    pub faces: Vec<Face>,
    pub classes: Vec<Class>,
    pub layers: Vec<String>,
    pub levels: Vec<f64>,
    pub preferred_layer: usize,
    pub unit_mm: f64,
    pub warnings: Vec<String>,
    pub name: String,
    pub value_label: Option<String>,
}
impl Dataset {
    pub fn rescale(&mut self, factor: f64) -> Result<(), String> {
        if !factor.is_finite() || factor <= 0. {
            return Err("Масштаб единиц должен быть положительным".into());
        }
        let ratio = factor / self.unit_mm;
        for f in &mut self.faces {
            for p in f.rings.iter_mut().flatten() {
                p[0] *= ratio;
                p[1] *= ratio;
            }
            for p in f.triangles.iter_mut().flatten() {
                p[0] *= ratio;
                p[1] *= ratio;
            }
            f.z *= ratio;
            f.bbox = geometry::bounds(&f.rings);
        }
        for z in &mut self.levels {
            *z *= ratio;
        }
        self.unit_mm = factor;
        Ok(())
    }
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct Record {
    kind: String,
    p: Vec<(i32, String)>,
}
impl Record {
    fn get(&self, code: i32) -> Option<&str> {
        self.p
            .iter()
            .find(|(c, _)| *c == code)
            .map(|(_, v)| v.as_str())
    }
    fn n(&self, code: i32, default: f64) -> f64 {
        self.get(code)
            .and_then(|v| v.parse().ok())
            .unwrap_or(default)
    }
    fn required(&self, code: i32) -> Result<f64, String> {
        self.get(code)
            .ok_or_else(|| format!("Отсутствует DXF-код {code}"))?
            .parse::<f64>()
            .map_err(|_| format!("Неверное число DXF {code}"))
    }
}
#[derive(Clone, Copy)]
struct Transform {
    a: f64,
    b: f64,
    c: f64,
    d: f64,
    tx: f64,
    ty: f64,
    sz: f64,
    tz: f64,
}
impl Transform {
    fn identity() -> Self {
        Self {
            a: 1.,
            b: 0.,
            c: 0.,
            d: 1.,
            tx: 0.,
            ty: 0.,
            sz: 1.,
            tz: 0.,
        }
    }
    fn point(self, p: [f64; 3]) -> [f64; 3] {
        [
            self.a * p[0] + self.c * p[1] + self.tx,
            self.b * p[0] + self.d * p[1] + self.ty,
            self.sz * p[2] + self.tz,
        ]
    }
    fn compose(self, q: Self) -> Self {
        Self {
            a: self.a * q.a + self.c * q.b,
            b: self.b * q.a + self.d * q.b,
            c: self.a * q.c + self.c * q.d,
            d: self.b * q.c + self.d * q.d,
            tx: self.a * q.tx + self.c * q.ty + self.tx,
            ty: self.b * q.tx + self.d * q.ty + self.ty,
            sz: self.sz * q.sz,
            tz: self.sz * q.tz + self.tz,
        }
    }
}
struct Block {
    origin: [f64; 3],
    records: Vec<Record>,
}
struct RawFace {
    id: String,
    layer: String,
    rings: Vec<Vec<[f64; 3]>>,
    color: Color,
    kind: String,
}
fn clean(mut p: Vec<[f64; 3]>) -> Vec<[f64; 3]> {
    p.dedup_by(|a, b| a == b);
    if p.len() > 1 && p.first() == p.last() {
        p.pop();
    }
    p
}
fn hatch(r: &Record) -> Result<Vec<Vec<[f64; 3]>>, String> {
    if r.n(210, 0.).abs() > 1e-8 || r.n(220, 0.).abs() > 1e-8 || (r.n(230, 1.) - 1.).abs() > 1e-8 {
        return Err("HATCH с наклонной/обратной OCS не поддержан".into());
    }
    let starts: Vec<_> =
        r.p.iter()
            .enumerate()
            .filter(|(_, p)| p.0 == 92)
            .map(|(i, _)| i)
            .collect();
    let mut rings = Vec::new();
    let z = r.n(30, 0.);
    for (j, start) in starts.iter().enumerate() {
        let p = &r.p[*start..starts.get(j + 1).copied().unwrap_or(r.p.len())];
        let flags = p[0]
            .1
            .parse::<i32>()
            .map_err(|_| "Неверный флаг контура HATCH")?;
        let k = p
            .iter()
            .position(|(c, _)| *c == 93)
            .ok_or("Нет числа вершин/рёбер HATCH")?;
        let count = p[k]
            .1
            .parse::<usize>()
            .map_err(|_| "Неверное число вершин HATCH")?;
        if count > 100_000 {
            return Err("Слишком большой контур HATCH".into());
        }
        let mut points = Vec::new();
        if flags & 2 != 0 {
            if p[k + 1..]
                .iter()
                .take_while(|(c, _)| *c != 97)
                .any(|(c, v)| *c == 42 && v.parse::<f64>().unwrap_or(1.).abs() > 1e-12)
            {
                return Err("Дуги bulge в HATCH пока не поддержаны".into());
            }
            let mut i = k + 1;
            while i < p.len() && points.len() < count {
                if p[i].0 == 10 {
                    let x = p[i]
                        .1
                        .parse::<f64>()
                        .map_err(|_| "Неверная вершина HATCH")?;
                    if p.get(i + 1).is_none_or(|(c, _)| *c != 20) {
                        return Err("Нет Y вершины HATCH".into());
                    }
                    let y = p[i + 1]
                        .1
                        .parse::<f64>()
                        .map_err(|_| "Неверная вершина HATCH")?;
                    points.push([x, y, z]);
                    i += 1;
                }
                i += 1;
            }
        } else {
            let edges: Vec<_> = p[k + 1..]
                .iter()
                .enumerate()
                .filter(|(_, p)| p.0 == 72)
                .take(count)
                .map(|(i, _)| k + 1 + i)
                .collect();
            if edges.len() != count {
                return Err("Не хватает рёбер HATCH".into());
            }
            for (edge, i) in edges.iter().enumerate() {
                if p[*i].1 != "1" {
                    return Err("Криволинейный контур HATCH пока не поддержан".into());
                }
                let values = &p[*i..edges.get(edge + 1).copied().unwrap_or(p.len())];
                let get = |code| -> Result<f64, String> {
                    values
                        .iter()
                        .find(|(c, _)| *c == code)
                        .ok_or_else(|| format!("Нет кода {code} ребра HATCH"))?
                        .1
                        .parse()
                        .map_err(|_| "Неверная координата ребра".into())
                };
                points.push([get(10)?, get(20)?, z]);
                let end = [get(11)?, get(21)?, z];
                if edge + 1 == count {
                    points.push(end);
                }
            }
        }
        let points = clean(points);
        if points.len() < 3 {
            return Err("Вырожденный контур HATCH".into());
        }
        rings.push(points);
    }
    if rings.is_empty() {
        return Err("В HATCH нет контуров".into());
    }
    Ok(rings)
}
fn geometry(r: &Record) -> Result<Vec<Vec<[f64; 3]>>, String> {
    match r.kind.as_str() {
        "3DFACE" | "SOLID" => {
            let mut points = Vec::new();
            for i in 0..4 {
                if i == 3 && r.get(13).is_none() {
                    points.push(points[2]);
                    continue;
                }
                points.push([r.required(10 + i)?, r.required(20 + i)?, r.n(30 + i, 0.)]);
            }
            if r.kind == "SOLID" {
                points.swap(2, 3);
            }
            Ok(vec![clean(points)])
        }
        "HATCH" => hatch(r),
        _ => Err("Неподдержанный объект".into()),
    }
}
/// Shared raw outlines for the standalone planar DXF tracing application.
pub(super) fn drawing_rings(
    kind: &str,
    tags: &[(i32, String)],
) -> Result<Vec<Vec<[f64; 3]>>, String> {
    geometry(&Record {
        kind: kind.into(),
        p: tags.to_vec(),
    })
}
fn color(r: &Record, layer: &str, layers: &HashMap<String, Color>, parent: Color) -> Color {
    if let Some(value) = r.get(420).and_then(|v| v.parse::<u32>().ok()) {
        return Color::Rgb(value & 0xffffff);
    }
    match r.n(62, 256.) as i32 {
        0 => parent,
        256 => *layers.get(layer).unwrap_or(&Color::Aci(7)),
        n => Color::Aci(n.unsigned_abs().min(255) as u16),
    }
}
struct Reader {
    blocks: HashMap<String, Block>,
    layer_colors: HashMap<String, Color>,
    faces: Vec<RawFace>,
    skips: BTreeMap<String, usize>,
    expanded: usize,
}
impl Reader {
    fn expand(
        &mut self,
        records: &[Record],
        transform: Transform,
        parent_layer: &str,
        parent_color: Color,
        prefix: &str,
        depth: usize,
    ) {
        if depth > 16 {
            *self
                .skips
                .entry("Слишком глубокие/циклические блоки".into())
                .or_default() += 1;
            return;
        }
        for (number, r) in records.iter().enumerate() {
            if r.n(60, 0.) != 0. {
                continue;
            }
            let own = r.get(8).unwrap_or("0");
            let layer = if own == "0" && !parent_layer.is_empty() {
                parent_layer
            } else {
                own
            };
            let c = color(r, layer, &self.layer_colors, parent_color);
            let id = format!(
                "{prefix}{}",
                r.get(5)
                    .map(str::to_owned)
                    .unwrap_or_else(|| number.to_string())
            );
            if r.kind == "INSERT" {
                if r.n(210, 0.).abs() > 1e-8
                    || r.n(220, 0.).abs() > 1e-8
                    || (r.n(230, 1.) - 1.).abs() > 1e-8
                {
                    *self.skips.entry("Наклонные INSERT".into()).or_default() += 1;
                    continue;
                }
                let name = r.get(2).unwrap_or("");
                let Some(block) = self.blocks.get(name) else {
                    *self.skips.entry("Отсутствующие BLOCK".into()).or_default() += 1;
                    continue;
                };
                let origin = block.origin;
                let records = block.records.clone();
                let angle = r.n(50, 0.).to_radians();
                let a = angle.cos() * r.n(41, 1.);
                let b = angle.sin() * r.n(41, 1.);
                let cc = -angle.sin() * r.n(42, 1.);
                let d = angle.cos() * r.n(42, 1.);
                let rows = r.n(71, 1.).max(1.) as usize;
                let cols = r.n(70, 1.).max(1.) as usize;
                if rows.saturating_mul(cols) > 10_000 {
                    *self
                        .skips
                        .entry("Слишком большой массив INSERT".into())
                        .or_default() += 1;
                    continue;
                }
                for row in 0..rows {
                    for col in 0..cols {
                        let dx = col as f64 * r.n(44, 0.);
                        let dy = row as f64 * r.n(45, 0.);
                        let t = Transform {
                            a,
                            b,
                            c: cc,
                            d,
                            tx: r.n(10, 0.) - a * origin[0] - cc * origin[1] + angle.cos() * dx
                                - angle.sin() * dy,
                            ty: r.n(20, 0.) - b * origin[0] - d * origin[1]
                                + angle.sin() * dx
                                + angle.cos() * dy,
                            sz: r.n(43, 1.),
                            tz: r.n(30, 0.) - r.n(43, 1.) * origin[2],
                        };
                        self.expand(
                            &records,
                            transform.compose(t),
                            layer,
                            c,
                            &format!("{id}[{row},{col}]/"),
                            depth + 1,
                        );
                    }
                }
            } else if matches!(r.kind.as_str(), "3DFACE" | "SOLID" | "HATCH") {
                self.expanded += 1;
                if self.expanded > 1_000_000 {
                    *self
                        .skips
                        .entry("Лимит геометрии: 1 млн объектов".into())
                        .or_default() += 1;
                    return;
                }
                if layer.to_lowercase().contains("palette")
                    || layer.eq_ignore_ascii_case("COLORSCALE")
                {
                    continue;
                }
                match geometry(r) {
                    Ok(rings) => self.faces.push(RawFace {
                        id,
                        layer: layer.into(),
                        rings: rings
                            .into_iter()
                            .map(|p| p.into_iter().map(|p| transform.point(p)).collect())
                            .collect(),
                        color: c,
                        kind: r.kind.clone(),
                    }),
                    Err(message) => *self.skips.entry(message).or_default() += 1,
                }
            }
        }
    }
}
#[cfg(not(target_arch = "wasm32"))]
pub fn read(path: &Path) -> Result<Dataset, String> {
    let bytes = std::fs::read(path).map_err(|e| e.to_string())?;
    from_bytes(
        &bytes,
        path.file_name()
            .unwrap_or_default()
            .to_string_lossy()
            .as_ref(),
    )
}
pub fn from_bytes(bytes: &[u8], name: &str) -> Result<Dataset, String> {
    if bytes.starts_with(b"AutoCAD Binary DXF") {
        return Err("Нужен текстовый ASCII DXF, бинарный DXF не поддержан".into());
    }
    let text = match std::str::from_utf8(bytes) {
        Ok(s) => s.trim_start_matches('\u{feff}').to_owned(),
        Err(_) => encoding_rs::WINDOWS_1251.decode(bytes).0.into_owned(),
    };
    parse(&text, name)
}
pub fn parse(text: &str, name: &str) -> Result<Dataset, String> {
    let mut lines = text.lines();
    let mut pairs = Vec::new();
    let mut line = 0;
    loop {
        let Some(code) = lines.next() else {
            break;
        };
        line += 1;
        let value = lines.next().ok_or("Нечётное число строк DXF")?;
        line += 1;
        let code = code
            .trim()
            .parse::<i32>()
            .map_err(|_| format!("Строка {}: неверный код DXF", line - 1))?;
        pairs.push((code, value.trim().to_owned()));
    }
    let units = pairs
        .windows(2)
        .find(|p| p[0].0 == 9 && p[0].1 == "$INSUNITS")
        .and_then(|p| p[1].1.parse::<i32>().ok());
    let unit_mm = match units {
        Some(1) => 25.4,
        Some(2) => 304.8,
        Some(4) => 1.,
        Some(5) => 10.,
        Some(6) => 1000.,
        _ => 1000.,
    };
    let mut records = Vec::new();
    let mut record = Record {
        kind: String::new(),
        p: Vec::new(),
    };
    for (code, value) in pairs {
        if code == 0 {
            if !record.kind.is_empty() {
                records.push(record);
            }
            record = Record {
                kind: value,
                p: Vec::new(),
            };
        } else {
            record.p.push((code, value));
        }
    }
    if !record.kind.is_empty() {
        records.push(record);
    }
    let mut section = String::new();
    let mut current = None::<String>;
    let mut blocks = HashMap::new();
    let mut model = Vec::new();
    let mut layer_colors = HashMap::new();
    let mut palette = Vec::new();
    let mut boundary_labels = Vec::new();
    for r in &records {
        if r.kind == "SECTION" {
            section = r.get(2).unwrap_or("").into();
            continue;
        }
        if r.kind == "ENDSEC" {
            section.clear();
            continue;
        }
        if r.kind == "LAYER" {
            let layer = r.get(2).unwrap_or("0");
            layer_colors.insert(
                layer.into(),
                color(r, layer, &HashMap::new(), Color::Aci(7)),
            );
        }
        let layer = r.get(8).unwrap_or("");
        let palette_layer = layer.eq_ignore_ascii_case("COLORSCALE")
            || layer.to_lowercase().contains("result_palette");
        if palette_layer && matches!(r.kind.as_str(), "SOLID" | "3DFACE") {
            let c = color(r, layer, &layer_colors, Color::Aci(7));
            if !palette.contains(&c) {
                palette.push(c);
            }
        }
        if palette_layer && matches!(r.kind.as_str(), "ATTRIB" | "TEXT" | "MTEXT") {
            if let Some(value) = r
                .get(1)
                .and_then(|v| v.replace(',', ".").parse::<f64>().ok())
                .filter(|v| v.is_finite())
            {
                let order = r
                    .get(2)
                    .and_then(|t| t.trim_end_matches('S').parse::<u32>().ok())
                    .map(|v| v as f64)
                    .unwrap_or(r.n(10, 0.));
                boundary_labels.push((order, value));
            }
        }
        if section == "BLOCKS" {
            if r.kind == "BLOCK" {
                let n = r.get(2).unwrap_or("").to_owned();
                blocks.insert(
                    n.clone(),
                    Block {
                        origin: [r.n(10, 0.), r.n(20, 0.), r.n(30, 0.)],
                        records: Vec::new(),
                    },
                );
                current = Some(n);
            } else if r.kind == "ENDBLK" {
                current = None;
            } else if let Some(n) = &current {
                blocks.get_mut(n).unwrap().records.push(r.clone());
            }
        } else if section == "ENTITIES" {
            model.push(r.clone());
        }
    }
    let mut reader = Reader {
        blocks,
        layer_colors,
        faces: Vec::new(),
        skips: BTreeMap::new(),
        expanded: 0,
    };
    reader.expand(&model, Transform::identity(), "", Color::Aci(7), "", 0);
    let mut classes: Vec<Class> = palette
        .iter()
        .enumerate()
        .map(|(i, c)| Class {
            color: *c,
            value: i as f64,
            range: None,
            reinforcement: None,
        })
        .collect();
    boundary_labels.sort_by(|a, b| a.0.total_cmp(&b.0));
    if boundary_labels.len() == palette.len() + 1 && !palette.is_empty() {
        for (i, c) in classes.iter_mut().enumerate() {
            let range = [boundary_labels[i].1, boundary_labels[i + 1].1];
            c.range = Some(range);
        }
    }
    let mut warnings = Vec::new();
    if !matches!(units, Some(1 | 2 | 4 | 5 | 6)) {
        warnings.push(
            "В DXF не заданы единицы: принято 1 ед. DXF = 1000 мм. Проверьте масштаб единиц."
                .into(),
        );
    }
    if classes.iter().all(|c| c.range.is_none()) {
        warnings.push("Числовая шкала не восстановлена: значения — фиксированные условные классы цвета 0, 1, 2… .".into());
    } else {
        warnings.push("Цвета задают фиксированные классы 0, 1, 2… . Подписи интервалов DXF сохранены только как справка; точных числовых результатов нет.".into());
    }
    let mut faces = Vec::new();
    let mut levels = Vec::<f64>::new();
    for f in reader.faces {
        let minz = f
            .rings
            .iter()
            .flatten()
            .map(|p| p[2] * unit_mm)
            .fold(f64::INFINITY, f64::min);
        let maxz = f
            .rings
            .iter()
            .flatten()
            .map(|p| p[2] * unit_mm)
            .fold(f64::NEG_INFINITY, f64::max);
        if !minz.is_finite() || maxz - minz > 0.01 {
            *reader
                .skips
                .entry("Негоризонтальные/некорректные элементы".into())
                .or_default() += 1;
            continue;
        }
        let rings: Vec<Vec<Point>> = f
            .rings
            .iter()
            .map(|r| r.iter().map(|p| [p[0] * unit_mm, p[1] * unit_mm]).collect())
            .collect();
        if rings
            .iter()
            .any(|r| r.len() < 3 || geometry::area(r) < 1e-8)
            || rings.iter().flatten().flatten().any(|v| !v.is_finite())
        {
            *reader
                .skips
                .entry("Вырожденные контуры".into())
                .or_default() += 1;
            continue;
        }
        let color = match classes.iter().position(|c| c.color == f.color) {
            Some(i) => i,
            None => {
                classes.push(Class {
                    color: f.color,
                    value: classes.len() as f64,
                    range: None,
                    reinforcement: None,
                });
                classes.len() - 1
            }
        };
        let triangles = match geometry::triangulate(&rings) {
            Ok(t) if !t.is_empty() => t,
            _ => {
                *reader
                    .skips
                    .entry("Ошибка триангуляции".into())
                    .or_default() += 1;
                continue;
            }
        };
        let level = match levels.iter().position(|z| (*z - minz).abs() < 0.01) {
            Some(i) => i,
            None => {
                levels.push(minz);
                levels.len() - 1
            }
        };
        let bbox = geometry::bounds(&rings);
        let _kind = f.kind;
        faces.push(Face {
            source: None,
            id: f.id,
            layer: f.layer,
            rings,
            triangles,
            color,
            z: minz,
            level,
            bbox,
        });
    }
    if faces.is_empty() {
        return Err(format!(
            "Нет горизонтальных 3DFACE/SOLID/HATCH. Диагностика: {:?}",
            reader.skips
        ));
    }
    let mut layer_counts = BTreeMap::<String, usize>::new();
    for f in &faces {
        *layer_counts.entry(f.layer.clone()).or_default() += 1;
    }
    let layers: Vec<_> = layer_counts.keys().cloned().collect();
    let preferred_layer = layers
        .iter()
        .position(|l| l == "KLEENKA")
        .or_else(|| layers.iter().position(|l| l == "layer_elements"))
        .or_else(|| layers.iter().position(|l| l == "Результаты"))
        .unwrap_or_else(|| {
            layers
                .iter()
                .enumerate()
                .max_by_key(|(_, l)| layer_counts[*l])
                .unwrap()
                .0
        });
    for (message, count) in reader.skips {
        warnings.push(format!("Пропущено: {message}: {count}"));
    }
    Ok(Dataset {
        value_label: None,
        faces,
        classes,
        layers,
        levels,
        preferred_layer,
        unit_mm,
        warnings,
        name: name.into(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn unsupported_curves_do_not_become_fake_polygons() {
        let r = Record {
            kind: "HATCH".into(),
            p: vec![(92, "1".into()), (93, "1".into()), (72, "2".into())],
        };
        assert!(hatch(&r).is_err());
    }
}
