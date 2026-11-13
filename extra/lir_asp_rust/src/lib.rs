//! Independent LIR + ASP adapter. No UI, graphics or application dependency.
pub mod physical_loads;
mod sections;
pub mod stamp_loads;
pub use sections::Definition as SectionDefinition;
use serde::Serialize;
use std::{
    collections::{BTreeMap, BTreeSet},
    io::{Read, Seek, SeekFrom},
};
pub type Result<T> = std::result::Result<T, String>;
pub type Point = [f64; 3];
fn require(ok: bool, message: impl Into<String>) -> Result<()> {
    if ok {
        Ok(())
    } else {
        Err(message.into())
    }
}
struct Reader<'a>(&'a [u8]);
impl<'a> Reader<'a> {
    fn span(&self, o: usize, n: usize) -> Result<&'a [u8]> {
        self.0
            .get(o..o.checked_add(n).ok_or("Offset overflow")?)
            .ok_or_else(|| format!("Truncated file at byte {o}, need {n}"))
    }
    fn u32(&self, o: usize) -> Result<u32> {
        Ok(u32::from_le_bytes(self.span(o, 4)?.try_into().unwrap()))
    }
    fn u16(&self, o: usize) -> Result<u16> {
        Ok(u16::from_le_bytes(self.span(o, 2)?.try_into().unwrap()))
    }
    fn float(&self, o: usize, wide: bool) -> Result<f64> {
        let v = if wide {
            f64::from_le_bytes(self.span(o, 8)?.try_into().unwrap())
        } else {
            f32::from_le_bytes(self.span(o, 4)?.try_into().unwrap()) as f64
        };
        require(v.is_finite(), format!("Non-finite number at byte {o}"))?;
        Ok(v)
    }
    fn floats<const N: usize>(&self, o: usize, wide: bool) -> Result<[f64; N]> {
        let mut v = [0.; N];
        for (i, a) in v.iter_mut().enumerate() {
            *a = self.float(o + i * if wide { 8 } else { 4 }, wide)?;
        }
        Ok(v)
    }
    fn text(&self, o: usize, n: usize) -> Result<String> {
        let b = self.span(o, n)?;
        let b = &b[..b.iter().position(|&c| c == 0).unwrap_or(b.len())];
        Ok(encoding_rs::WINDOWS_1251.decode(b).0.into_owned())
    }
}
#[derive(Debug, Serialize)]
pub struct AspPlate {
    pub id: u32,
    pub points: Vec<Point>,
    pub direction: Point,
    pub raw_values: [f64; 4],
    pub thickness: f64,
    pub flags: u32,
}
#[derive(Debug, Serialize)]
pub struct Asp {
    pub plates: Vec<AspPlate>,
    pub bars: u32,
    pub rows: usize,
    pub signature: String,
    pub definition_size: usize,
    pub trailing_bytes: usize,
    pub code: String,
    pub result_kind: String,
    pub design_option: String,
}
pub fn parse_asp(bytes: &[u8]) -> Result<Asp> {
    let r = Reader(bytes);
    r.span(0, 272)?;
    let signature = r.text(0, 32)?;
    let definition_size = match signature.as_str() {
        "ARM-SAPFIR_5 binary v.5.0 2015" => 88,
        "ARM-SAPFIR_6 binary v.6.0 2022" => 120,
        _ => return Err(format!("Unsupported ASP signature: {signature}")),
    };
    require(
        r.u32(100)? == 156 && r.u32(244)? == definition_size as u32 && r.u32(256)? == 160,
        "Unknown ASP record sizes",
    )?;
    let count = r.u32(96)? as usize;
    let definitions = r.u32(240)? as usize;
    let bars = r.u32(252)?;
    require(
        count <= 5_000_000 && definitions <= 1_000_000 && bars <= 5_000_000,
        "Invalid ASP counts",
    )?;
    let mut offset = 272 + count * 156 + definitions * definition_size;
    r.span(272, offset - 272)?;
    let mut plates = Vec::with_capacity(count);
    let mut seen = BTreeSet::new();
    for i in 0..count {
        let o = 272 + i * 156;
        let id = r.u32(o)?.checked_add(1).ok_or("ASP element ID overflow")?;
        let flags = r.u32(o + 152)?;
        let n = (flags & 15) as usize;
        require(
            n == 3 || n == 4,
            format!("Unsupported ASP plate flags {id}: {flags}"),
        )?;
        require(seen.insert(id), format!("Duplicate ASP element {id}"))?;
        let points = (0..n)
            .map(|j| r.floats(o + 8 + j * 12, false))
            .collect::<Result<_>>()?;
        plates.push(AspPlate {
            id,
            points,
            direction: r.floats(o + 56, false)?,
            raw_values: r.floats(o + 132, false)?,
            thickness: r.float(o + 148, false)?,
            flags,
        });
    }
    let mut rows = 0;
    for i in 0..bars {
        r.span(offset, 160)?;
        let n = r.u32(offset + 156)? as usize;
        require(n <= 100_000, "Too many ASP bar rows")?;
        r.span(offset + 160, n * 44)?;
        r.floats::<10>(offset + 72, false)?;
        let summary = r.floats::<11>(offset + 112, false)?;
        let mut maxima = [f64::NEG_INFINITY; 11];
        for j in 0..n {
            for (k, max) in maxima.iter_mut().enumerate() {
                *max = max.max(r.float(offset + 160 + j * 44 + k * 4, false)?);
            }
        }
        require(
            n == 0
                || summary
                    .iter()
                    .zip(maxima)
                    .all(|(v, m)| (v - m).abs() <= 1e-4 * v.abs().max(1.)),
            format!("ASP bar {} summary differs from row maxima", i + 1),
        )?;
        offset += 160 + n * 44;
        rows += n;
    }
    Ok(Asp {
        plates,
        bars,
        rows,
        signature,
        definition_size,
        trailing_bytes: bytes.len() - offset,
        code: r.text(32, 64)?,
        result_kind: r.text(104, 64)?,
        design_option: r.text(168, 72)?,
    })
}
#[derive(Debug, Serialize)]
pub struct LirElement {
    pub node_ids: Vec<u32>,
    pub points: Vec<Point>,
    pub definition_index: u16,
}
#[derive(Debug, Serialize)]
pub struct LirSubset {
    pub nodes: u32,
    pub elements: u32,
    pub retained_nodes: usize,
    pub plates: BTreeMap<u32, LirElement>,
}
fn read_at<R: Read + Seek>(input: &mut R, offset: u64, n: usize) -> Result<Vec<u8>> {
    input
        .seek(SeekFrom::Start(offset))
        .map_err(|e| e.to_string())?;
    let mut b = vec![0; n];
    input
        .read_exact(&mut b)
        .map_err(|e| format!("Read at {offset}: {e}"))?;
    Ok(b)
}
fn layout(bytes: &[u8]) -> Result<(u32, u32, usize)> {
    let r = Reader(bytes);
    let len = *r.span(0, 1)?.first().unwrap() as usize;
    require((36..255).contains(&len), "Unsupported LIR header length")?;
    let b = r.span(1, len)?;
    require(
        b.iter().all(|&c| c == 0 || (32..=126).contains(&c)),
        "Unsupported LIR header encoding",
    )?;
    let text = std::str::from_utf8(b).map_err(|e| e.to_string())?;
    let mut fields: Vec<_> = text.split('\0').collect();
    while fields.len() > 1 && fields.last() == Some(&"") {
        fields.pop();
    }
    require(
        fields[0] == "LIRA-SAPR 2013 software ver.13.0.0.a",
        format!("Unsupported LIR signature: {}", fields[0]),
    )?;
    let mut seen = BTreeSet::new();
    for f in fields.iter().skip(1) {
        let (name, value) = f
            .split_once('=')
            .ok_or_else(|| format!("Unknown LIR header field: {f}"))?;
        require(seen.insert(name), "Duplicate LIR header field")?;
        let valid = if name == "FileAnsiCP" {
            value.split_once('/').is_some_and(|(a, b)| {
                [a.trim(), b.trim()]
                    .iter()
                    .all(|s| !s.is_empty() && s.bytes().all(|c| c.is_ascii_digit()))
            })
        } else if name == "Preview"
            || name
                .strip_prefix("MetaData")
                .is_some_and(|s| !s.is_empty() && s.bytes().all(|c| c.is_ascii_digit()))
        {
            value.len() == 16 && value.bytes().all(|c| c.is_ascii_hexdigit())
        } else {
            false
        };
        require(valid, format!("Unsupported LIR header field: {f}"))?;
    }
    let base = 1 + len;
    r.span(base, 52)?;
    let nodes = r.u32(base + 12)?;
    let elements = r.u32(base + 20)?;
    require(
        nodes > 0 && nodes <= 5_000_000 && elements > 0 && elements <= 5_000_000,
        "Invalid LIR counts",
    )?;
    Ok((nodes, elements, base + 52))
}
// Coalesce sparse reads: gap <=4 KiB, each buffer <=1 MiB. Do not retain full geometry.
fn records<R: Read + Seek>(
    input: &mut R,
    ids: &BTreeSet<u32>,
    base: usize,
    stride: usize,
    mut consume: impl FnMut(u32, &Reader<'_>, usize) -> Result<()>,
) -> Result<()> {
    let ids: Vec<_> = ids.iter().copied().collect();
    let mut i = 0;
    while i < ids.len() {
        let begin = ids[i] as usize;
        let mut j = i + 1;
        while j < ids.len()
            && (ids[j] - ids[j - 1]) as usize * stride <= 4096
            && (ids[j] as usize - begin + 1) * stride <= 1_048_576
        {
            j += 1;
        }
        let b = read_at(
            input,
            (base + begin * stride) as u64,
            (ids[j - 1] as usize - begin + 1) * stride,
        )?;
        let r = Reader(&b);
        for &id in &ids[i..j] {
            consume(id, &r, (id as usize - begin) * stride)?;
        }
        i = j;
    }
    Ok(())
}
pub fn read_lir_subset<R: Read + Seek>(input: &mut R, asp: &Asp) -> Result<LirSubset> {
    let size = input.seek(SeekFrom::End(0)).map_err(|e| e.to_string())?;
    let header = read_at(input, 0, size.min(1024) as usize)?;
    let (nodes, elements, node_start) = layout(&header)?;
    let start = node_start + nodes as usize * 28;
    require(
        (start + elements as usize * 38) as u64 <= size,
        "Truncated LIR geometry",
    )?;
    let mut sizes = BTreeMap::new();
    for p in &asp.plates {
        require(p.id >= 1 && p.id <= elements, "ASP element absent from LIR")?;
        sizes.insert(p.id - 1, p.points.len());
    }
    let mut connectivity = BTreeMap::new();
    let mut needed = BTreeSet::new();
    records(
        input,
        &sizes.keys().copied().collect(),
        start,
        38,
        |index, r, o| {
            require(
                r.u32(o + 34)? == 1,
                format!("Unsupported LIR element flag {}", index + 1),
            )?;
            let ids = (0..sizes[&index])
                .map(|i| r.u32(o + i * 4))
                .collect::<Result<Vec<_>>>()?;
            require(
                ids.iter().all(|&i| i < nodes),
                "LIR node reference out of range",
            )?;
            require(
                ids.iter().copied().collect::<BTreeSet<_>>().len() == ids.len(),
                "Repeated LIR node IDs",
            )?;
            needed.extend(ids.iter().copied());
            connectivity.insert(index + 1, (ids, r.u16(o + 32)?));
            Ok(())
        },
    )?;
    let mut points = BTreeMap::new();
    records(input, &needed, node_start, 28, |index, r, o| {
        require(
            r.u32(o + 24)? == 1,
            format!("Unsupported LIR node flag {}", index + 1),
        )?;
        points.insert(index, r.floats::<3>(o, true)?);
        Ok(())
    })?;
    let plates = connectivity
        .into_iter()
        .map(|(id, (indices, definition_index))| {
            (
                id,
                LirElement {
                    node_ids: indices.iter().map(|i| i + 1).collect(),
                    points: indices.iter().map(|i| points[i]).collect(),
                    definition_index,
                },
            )
        })
        .collect();
    Ok(LirSubset {
        nodes,
        elements,
        retained_nodes: points.len(),
        plates,
    })
}
#[derive(Debug, Serialize)]
pub struct MatchedPlate {
    pub id: u32,
    pub node_ids: Vec<u32>,
    pub xyz: Vec<Point>,
    pub perimeter: Vec<Point>,
    pub direction: Point,
    pub raw_values: [f64; 4],
    pub definition_index: u16,
}
fn point_key(p: Point) -> [u32; 3] {
    p.map(|v| {
        let x = v as f32;
        if x == 0. {
            0
        } else {
            x.to_bits()
        }
    })
}
pub fn match_plates(lir: &LirSubset, asp: &Asp) -> Result<Vec<MatchedPlate>> {
    let mut definitions = BTreeMap::new();
    let mut result = Vec::with_capacity(asp.plates.len());
    for p in &asp.plates {
        let src = lir
            .plates
            .get(&p.id)
            .ok_or("Selected plate absent in LIR subset")?;
        require(
            src.points.len() == p.points.len(),
            "Inconsistent plate vertex count",
        )?;
        let expected: BTreeMap<_, _> = src
            .points
            .iter()
            .enumerate()
            .map(|(i, &p)| (point_key(p), i))
            .collect();
        require(
            expected.len() == p.points.len(),
            "LIR coincident plate vertices at f32 precision",
        )?;
        let order = p
            .points
            .iter()
            .map(|&v| {
                expected
                    .get(&point_key(v))
                    .copied()
                    .ok_or_else(|| format!("ASP/LIR geometry mismatch: plate {}", p.id))
            })
            .collect::<Result<Vec<_>>>()?;
        require(
            order.iter().copied().collect::<BTreeSet<_>>().len() == order.len(),
            "ASP repeated plate vertices",
        )?;
        if let Some(n) = definitions.insert(src.definition_index, p.points.len()) {
            require(n == p.points.len(), "Conflicting LIR definition topology")?;
        }
        result.push(MatchedPlate {
            id: p.id,
            node_ids: src.node_ids.clone(),
            xyz: src.points.clone(),
            perimeter: order.iter().map(|&i| src.points[i]).collect(),
            direction: p.direction,
            raw_values: p.raw_values,
            definition_index: src.definition_index,
        });
    }
    Ok(result)
}
#[derive(Debug, Clone, Copy)]
pub enum Unit {
    Meters,
    Centimeters,
    Millimeters,
}
impl Unit {
    fn scale(self) -> f64 {
        match self {
            Self::Meters => 1000.,
            Self::Centimeters => 10.,
            Self::Millimeters => 1.,
        }
    }
}
#[derive(Debug, Serialize)]
pub struct SelectedPlate {
    pub id: u32,
    pub z_mm: f64,
    pub xy_mm: Vec<[f64; 2]>,
    pub source: MatchedPlate,
}
#[derive(Debug, Serialize)]
pub struct Selection {
    pub plates: Vec<SelectedPlate>,
    pub skipped_missing: usize,
    pub skipped_non_horizontal: usize,
}
/// Retain any plate with at least one valid As. Values remain in original ASP axes.
pub fn select_all(plates: Vec<MatchedPlate>, unit: Unit) -> Result<Selection> {
    require(
        !plates.is_empty(),
        "ASP contains no reinforcement plate records",
    )?;
    let mut out = Selection {
        plates: Vec::new(),
        skipped_missing: 0,
        skipped_non_horizontal: 0,
    };
    let scale = unit.scale();
    for p in plates {
        if p.raw_values.iter().all(|&v| v < 0.) {
            out.skipped_missing += 1;
            continue;
        }
        let min = p
            .perimeter
            .iter()
            .map(|v| v[2] * scale)
            .fold(f64::INFINITY, f64::min);
        let max = p
            .perimeter
            .iter()
            .map(|v| v[2] * scale)
            .fold(f64::NEG_INFINITY, f64::max);
        if max - min > 0.01 {
            out.skipped_non_horizontal += 1;
            continue;
        }
        let z_mm = p.perimeter.iter().map(|v| v[2] * scale).sum::<f64>() / p.perimeter.len() as f64;
        let xy_mm = p
            .perimeter
            .iter()
            .map(|v| [v[0] * scale, v[1] * scale])
            .collect();
        out.plates.push(SelectedPlate {
            id: p.id,
            z_mm,
            xy_mm,
            source: p,
        });
    }
    require(
        !out.plates.is_empty(),
        "No horizontal plates with reinforcement results",
    )?;
    Ok(out)
}
pub fn import_pair<R: Read + Seek>(lir: &mut R, asp_bytes: &[u8], unit: Unit) -> Result<Selection> {
    let asp = parse_asp(asp_bytes)?;
    let subset = read_lir_subset(lir, &asp)?;
    select_all(match_plates(&subset, &asp)?, unit)
}

/// Whole geometry for reconstruction without ASP. IDs remain 1-based.
/// Types are cross-checked against redundant per-element records, never assigned
/// from a universal definition-index table.
#[derive(Debug, Serialize)]
pub struct ModelElement {
    pub id: u32,
    pub definition: u16,
    pub kind: u16,
    pub nodes: Vec<u32>,
}
#[derive(Debug, Serialize, Clone)]
pub struct ShellProperty {
    pub id: u16,
    pub weight: f32,
    pub e: f32,
    pub nu: f32,
    pub thickness_cm: f32,
}
#[derive(Debug, Serialize)]
pub struct FoundationAssignment {
    pub c1: f64,
    pub c2: f64,
    pub rz_text: String,
}
#[derive(Debug, Serialize)]
pub struct Model {
    pub nodes: Vec<Point>,
    pub elements: Vec<ModelElement>,
    pub properties: BTreeMap<u16, ShellProperty>,
    pub sections: BTreeMap<u16, SectionDefinition>,
    pub section_assignments: Vec<u16>,
    pub section_notes: Vec<String>,
    pub bar_angles_deg: BTreeMap<u32, f64>,
    pub foundations: BTreeMap<u32, FoundationAssignment>,
    pub inactive_elements: usize,
    pub inferred_types: usize,
    pub skipped_unknown: BTreeMap<u16, usize>,
    pub skipped_invalid: usize,
}
pub fn read_model(bytes: &[u8]) -> Result<Model> {
    let (nn, ne, ns) = layout(bytes)?;
    let es = ns + nn as usize * 28;
    let end = es + ne as usize * 38;
    let r = Reader(bytes);
    r.span(ns, end - ns)?;
    let mut nodes = Vec::with_capacity(nn as usize);
    let mut live_nodes = Vec::with_capacity(nn as usize);
    for i in 0..nn as usize {
        let o = ns + i * 28;
        let flag = r.u32(o + 24)?;
        require(flag <= 1, format!("Unknown node flag {}", i + 1))?;
        nodes.push(r.floats::<3>(o, true)?);
        live_nodes.push(flag == 1);
    }
    let mut active = Vec::with_capacity(ne as usize);
    for i in 0..ne as usize {
        let o = es + i * 38;
        let flag = r.u32(o + 34)?;
        require(flag <= 1, format!("Unknown element flag {}", i + 1))?;
        active.push(flag == 1);
    }
    let mut types = BTreeMap::new();
    let mut bar_angles_deg = BTreeMap::new();
    let mut definitions = BTreeMap::<u16, BTreeSet<u16>>::new();
    let mut headers = BTreeMap::<u32, Vec<(usize, u16, usize)>>::new();
    for o in end.saturating_add(10)..bytes.len().saturating_sub(6) {
        let kind = r.u16(o)?;
        let n = match kind {
            10 => 2,
            42 => 3,
            41 | 44 => 4,
            56 => 1,
            _ => continue,
        };
        if r.u16(o + 2)? as usize != n {
            continue;
        }
        let key = r.u32(o - 8)?;
        if key >= ne || (r.u32(o - 4)? & 1 != 0) != active[key as usize] {
            continue;
        }
        let raw = es + key as usize * 38;
        if r.u16(o - 10)? != r.u16(raw + 32)? {
            continue;
        }
        headers.entry(key).or_default().push((o, kind, n));
    }
    for (&key, candidates) in &headers {
        if !active[key as usize] {
            continue;
        }
        let raw = es + key as usize * 38;
        for &(o, kind, n) in candidates {
            // The node list preceding a header belongs to the PREVIOUS object.
            // Verify this object's own list just before the following header.
            let verified = headers.get(&(key + 1)).is_some_and(|next| {
                next.iter().any(|&(next, _, _)| {
                    next > o
                        && next >= 10 + n * 4
                        && bytes.get(next - 10 - n * 4..next - 10) == bytes.get(raw..raw + n * 4)
                })
            });
            // The final header is anchored by the preceding object's verified
            // end-node list. Its own tail is not needed to read kind/count;
            // load-tail parsing must still validate that boundary separately.
            let terminal = key + 1 == ne
                && candidates.len() == 1
                && key > 0
                && headers.get(&(key - 1)).is_some_and(|previous| {
                    previous.iter().any(|&(before, _, n)| {
                        let previous_raw = es + (key as usize - 1) * 38;
                        before < o
                            && o >= 10 + n * 4
                            && bytes.get(o - 10 - n * 4..o - 10)
                                == bytes.get(previous_raw..previous_raw + n * 4)
                    })
                });
            if !verified && !terminal {
                continue;
            }

            // Observed type-10 orientation tail: f32 degrees, u16 tag2/tag5,
            // 32 zero service bytes, followed by this object's ordered nodes.
            // Anchor both boundaries; never search for a plausible angle alone.
            if kind == 10 && verified {
                if let Some(next) = headers.get(&(key + 1)).filter(|v| v.len() == 1) {
                    let bound = next[0].0.saturating_sub(10 + n * 4);
                    if bound >= o + 42
                        && matches!(r.u16(bound - 34).ok(), Some(2 | 5))
                        && bytes
                            .get(bound - 32..bound)
                            .is_some_and(|b| b.iter().all(|v| *v == 0))
                        && bytes.get(bound..bound + n * 4) == bytes.get(raw..raw + n * 4)
                    {
                        if let Ok(angle) = r.float(bound - 38, false) {
                            if angle.abs() <= 3600. {
                                bar_angles_deg.insert(key + 1, angle);
                            }
                        }
                    }
                }
            }
            if let Some(old) = types.insert(key + 1, kind) {
                require(old == kind, "Conflicting verified LIR element metadata")?;
            }
            definitions
                .entry(r.u16(raw + 32)?)
                .or_default()
                .insert(kind);
        }
    }
    let mut elements = Vec::new();
    let mut inferred_types = 0;
    let mut skipped_invalid = 0;
    let mut skipped_unknown = BTreeMap::<u16, usize>::new();
    'records: for (i, live) in active.iter().enumerate() {
        if !live {
            continue;
        }
        let id = i as u32 + 1;
        let o = es + i * 38;
        let definition = r.u16(o + 32)?;
        let kind = if let Some(&kind) = types.get(&id) {
            kind
        } else {
            let Some(candidates) = definitions.get(&definition).filter(|c| c.len() == 1) else {
                *skipped_unknown.entry(definition).or_default() += 1;
                continue;
            };
            inferred_types += 1;
            *candidates.first().unwrap()
        };
        let n = match kind {
            10 => 2,
            42 => 3,
            41 | 44 => 4,
            56 => 1,
            _ => unreachable!(),
        };
        let mut ids = Vec::new();
        for j in 0..n {
            let node = r.u32(o + j * 4)?;
            if node >= nn || !live_nodes[node as usize] {
                skipped_invalid += 1;
                continue 'records;
            }
            ids.push(node + 1);
        }
        if ids.iter().copied().collect::<BTreeSet<_>>().len() != ids.len() {
            skipped_invalid += 1;
            continue;
        }
        elements.push(ModelElement {
            id,
            definition,
            kind,
            nodes: ids,
        });
    }
    let (sections, section_assignments, section_notes) = sections::read(
        bytes,
        end,
        &active,
        &elements.iter().map(|e| (e.id, e.kind)).collect(),
    );
    let properties = sections
        .iter()
        .filter(|(_, p)| p.kind == 36)
        .map(|(&id, p)| {
            (
                id,
                ShellProperty {
                    id,
                    weight: p.parameters[0],
                    e: p.parameters[1],
                    nu: p.parameters[2],
                    thickness_cm: p.parameters[3],
                },
            )
        })
        .collect();
    let mut foundations = BTreeMap::new();
    for e in elements.iter().filter(|e| matches!(e.kind, 41 | 42 | 44)) {
        let key = e.id - 1;
        let Some(records) = headers.get(&key) else {
            continue;
        };
        for &(start, _, n) in records {
            // Some versions serialize C1/C2 in the object's CString property
            // rather than the zero-tail layout below. Anchor both the header
            // and this object's ordered topology before trusting that property.
            let topology_verified = headers.get(&(key + 1)).is_some_and(|next| {
                next.iter().any(|&(end, _, _)| {
                    let raw = es + key as usize * 38;
                    end > start
                        && end >= 10 + n * 4
                        && bytes.get(end - 10 - n * 4..end - 10) == bytes.get(raw..raw + n * 4)
                })
            });
            if topology_verified {
                if let Ok((text, _)) = sections::text(bytes, start + 8) {
                    if let Some(values) = text.strip_prefix("C1\t") {
                        let mut fields = values.split(';');
                        if let Some((a, b)) = fields.next().and_then(|v| v.split_once(',')) {
                            if let (Ok(c1), Ok(c2)) = (a.parse::<f64>(), b.parse::<f64>()) {
                                if c1.is_finite() && c2.is_finite() && c1 >= 0. && c2 >= 0. {
                                    foundations.insert(
                                        e.id,
                                        FoundationAssignment {
                                            c1,
                                            c2,
                                            rz_text: fields.next().unwrap_or("").to_owned(),
                                        },
                                    );
                                }
                            }
                        }
                    }
                }
            }
            let stop = headers
                .range(key + 1..)
                .next()
                .and_then(|(_, r)| r.iter().map(|r| r.0).min())
                .unwrap_or(bytes.len())
                .min(start + 10000);
            for o in start + 12..stop.saturating_sub(28 + n * 4) {
                if bytes.get(o - 12..o) != Some(&[1, 0, 254, 255, 0, 0, 0, 0, 0, 0, 0, 0][..]) {
                    continue;
                }
                let raw = es + key as usize * 38;
                if bytes.get(o + 28..o + 28 + n * 4) != bytes.get(raw..raw + n * 4) {
                    continue;
                }
                if !bytes[o + 8..o + 28].iter().all(|&b| b == 0) {
                    continue;
                }
                let (Ok(c1), Ok(c2)) = (r.float(o, false), r.float(o + 4, false)) else {
                    continue;
                };
                if c1 < 0. || c2 < 0. {
                    continue;
                }
                let text_bytes = &bytes[start..o - 12];
                let marker = [67, 0, 49, 0, 9, 0];
                let Some(pos) = text_bytes.windows(6).position(|v| v == marker) else {
                    continue;
                };
                let text = String::from_utf16_lossy(
                    &text_bytes[pos..]
                        .chunks_exact(2)
                        .map(|b| u16::from_le_bytes([b[0], b[1]]))
                        .collect::<Vec<_>>(),
                );
                let Some(pair) = text.strip_prefix("C1\t").and_then(|v| v.split(';').next()) else {
                    continue;
                };
                let Some((a, b)) = pair.split_once(',') else {
                    continue;
                };
                let (Ok(a), Ok(b)) = (a.parse::<f64>(), b.parse::<f64>()) else {
                    continue;
                };
                if (a - c1 as f64).abs() > 1e-5 || (b - c2 as f64).abs() > 1e-5 {
                    continue;
                }
                let rz_text = text
                    .split("Rz\t")
                    .nth(1)
                    .unwrap_or("")
                    .chars()
                    .take_while(|c| {
                        c.is_ascii_digit() || matches!(c, '.' | ',' | '-' | '+' | 'e' | 'E')
                    })
                    .collect();
                foundations.insert(
                    e.id,
                    FoundationAssignment {
                        c1: a,
                        c2: b,
                        rz_text,
                    },
                );
            }
        }
    }
    Ok(Model {
        foundations,
        nodes,
        elements,
        properties,
        sections,
        section_assignments,
        section_notes,
        bar_angles_deg,
        inactive_elements: active.iter().filter(|live| !**live).count(),
        inferred_types,
        skipped_unknown,
        skipped_invalid,
    })
}

#[cfg(test)]
mod model_tests {
    use super::*;
    fn fixture() -> Vec<u8> {
        let mut header = b"LIRA-SAPR 2013 software ver.13.0.0.a".to_vec();
        header.resize(36, 0);
        let mut b = vec![header.len() as u8];
        b.extend(header);
        let base = b.len();
        b.resize(base + 52, 0);
        b[base + 12..base + 16].copy_from_slice(&4u32.to_le_bytes());
        b[base + 20..base + 24].copy_from_slice(&2u32.to_le_bytes());
        for p in [[0., 0., 3.], [1., 0., 3.], [0., 1., 3.], [1., 1., 3.]] {
            for v in p {
                b.extend(f64::to_le_bytes(v));
            }
            b.extend(1u32.to_le_bytes());
        }
        for (ids, flag) in [
            ([0u32, 1, 2, 0, 0, 0, 0, 0], 1u32),
            ([1u32, 3, 2, 0, 0, 0, 0, 0], 0u32),
        ] {
            for id in ids {
                b.extend(id.to_le_bytes());
            }
            b.extend(7u16.to_le_bytes());
            b.extend(flag.to_le_bytes());
        }
        b.extend(7u16.to_le_bytes());
        b.extend(0u32.to_le_bytes());
        b.extend(1u32.to_le_bytes());
        b.extend(42u16.to_le_bytes());
        b.extend(3u16.to_le_bytes());
        b.extend(0u32.to_le_bytes());
        for id in [0u32, 1, 2] {
            b.extend(id.to_le_bytes());
        }
        b.extend(7u16.to_le_bytes());
        b.extend(1u32.to_le_bytes());
        b.extend(0u32.to_le_bytes());
        b.extend(42u16.to_le_bytes());
        b.extend(3u16.to_le_bytes());
        b.extend(0u32.to_le_bytes());
        for id in [1u32, 3, 2] {
            b.extend(id.to_le_bytes());
        }
        b.extend([0; 64]);
        b
    }
    #[test]
    fn whole_model_accepts_grid_header_variants_without_fixed_offsets() {
        let base = fixture();
        let tail = &base[1 + base[0] as usize..];
        for header in [
            "LIRA-SAPR 2013 software ver.13.0.0.a\0\0\0",
            "LIRA-SAPR 2013 software ver.13.0.0.a\0FileAnsiCP=1251 / 866\0Preview=0000000005610E0B\0\0",
            "LIRA-SAPR 2013 software ver.13.0.0.a\0FileAnsiCP=1251 / 866\0Preview=0000000004770717\0MetaData1=0000000004E9AAD0",
        ] {
            let mut bytes=vec![header.len() as u8];bytes.extend(header.as_bytes());bytes.extend(tail);
            let model=read_model(&bytes).unwrap();
            assert_eq!(model.nodes,vec![[0.,0.,3.],[1.,0.,3.],[0.,1.,3.],[1.,1.,3.]]);
            assert_eq!(model.elements.len(),1);assert_eq!(model.inactive_elements,1);
            assert_eq!((model.elements[0].id,model.elements[0].kind,model.elements[0].definition),(1,42,7));
            assert_eq!(model.elements[0].nodes,vec![1,2,3]);assert_eq!(model.inferred_types,0);
        }
    }
    #[test]
    fn metadata_node_count_is_u16_and_next_field_can_be_nonzero() {
        let mut bytes = fixture();
        let pos = bytes
            .windows(6)
            .rposition(|w| w == [42, 0, 3, 0, 0, 0])
            .unwrap();
        bytes[pos + 4..pos + 6].copy_from_slice(&5u16.to_le_bytes());
        let model = read_model(&bytes).unwrap();
        assert_eq!(model.elements.len(), 1);
        assert_eq!(model.elements[0].kind, 42);
        assert_eq!(model.elements[0].nodes, vec![1, 2, 3]);
        assert_eq!(model.inferred_types, 0);
    }
    #[test]
    fn model_uses_file_metadata_and_skips_inactive_elements_without_renumbering() {
        let model = read_model(&fixture()).unwrap();
        assert_eq!(model.inactive_elements, 1);
        assert_eq!(model.elements.len(), 1);
        let e = &model.elements[0];
        assert_eq!((e.id, e.definition, e.kind), (1, 7, 42));
        assert_eq!(e.nodes, vec![1, 2, 3]);
        assert!(read_model(&fixture()[..100]).is_err());
    }
}
