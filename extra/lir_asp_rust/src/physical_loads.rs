//! Physical LIR load records. No direction/unit conversion and no FEM application.
//! Entry offsets must come from independently verified object navigation.
use super::{Reader, Result};
#[derive(Debug, Clone, serde::Serialize)]
pub struct Record {
    pub offset: usize,
    pub tag: u16,
    pub code: u16,
    pub case_id: u16,
    pub group_raw: u16,
    pub payload: Vec<f32>,
}
/// Decode a single supported element load; unknown tags never advance the cursor.
pub fn element_record(bytes: &[u8], offset: usize) -> Result<(Record, usize)> {
    let r = Reader(bytes);
    let tag = r.u16(offset)?;
    let size = match tag {
        2 | 10 => 20,
        12 => 16,
        14 | 28 => 24,
        _ => {
            return Err(format!(
                "Неподдержанный тип записи нагрузки {tag}, позиция {offset}"
            ))
        }
    };
    r.span(offset, size)?;
    let code = r.u16(offset + 2)?;
    let case_id = r.u16(offset + 4)?;
    if case_id == 0 {
        return Err("Некорректный нулевой номер загружения LIR".into());
    }
    let compatible = matches!(
        (tag, code / 10),
        (2, 16) | (10, 15) | (12, 6) | (12, 16) | (14, 7) | (28, 19)
    );
    if !compatible {
        return Err(format!("Неподдержанные тип/код нагрузки LIR {tag}/{code}"));
    }
    let mut payload = vec![];
    for i in (8..size).step_by(4) {
        payload.push(r.float(offset + i, false)? as f32);
    }
    Ok((
        Record {
            offset,
            tag,
            code,
            case_id,
            group_raw: r.u16(offset + 6)?,
            payload,
        },
        offset + size,
    ))
}
pub fn nodal_record(bytes: &[u8], offset: usize) -> Result<(Record, usize)> {
    let r = Reader(bytes);
    r.span(offset, 10)?;
    let code = r.u16(offset)?;
    let case_id = r.u16(offset + 2)?;
    if !(101..=106).contains(&code) || case_id == 0 {
        return Err("Неподдержанная запись узловой нагрузки".into());
    }
    Ok((
        Record {
            offset,
            tag: 0,
            code,
            case_id,
            group_raw: r.u16(offset + 8)?,
            payload: vec![r.float(offset + 4, false)? as f32],
        },
        offset + 10,
    ))
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn preserves_payload_and_does_not_apply_auxiliary_factor() {
        let mut b = vec![];
        for v in [2u16, 163, 1, 15001] {
            b.extend(v.to_le_bytes());
        }
        for v in [0.99f32, 0., 1.1] {
            b.extend(v.to_le_bytes());
        }
        let (r, end) = element_record(&b, 0).unwrap();
        assert_eq!(end, 20);
        assert_eq!(r.payload, vec![0.99, 0., 1.1]);
        b[0..2].copy_from_slice(&99u16.to_le_bytes());
        assert!(element_record(&b, 0).is_err());
        assert!(element_record(&b[..5], 0).is_err());
    }

    #[test]
    fn reads_utf16_case_table_instead_of_longer_unframed_strings() {
        let mut bytes = b"\x04Fake\x04Data\x04More\x04Text".to_vec();
        bytes.extend([5, 0, 2, 0, 0, 0, 1, 0, 7, 0, 0, 0, 2, 0, 0, 0]);
        for name in ["Собственный вес", "Покрытия\r\n", ""] {
            let units: Vec<_> = name.encode_utf16().collect();
            bytes.extend([255, 254, 255, units.len() as u8]);
            for unit in units {
                bytes.extend(unit.to_le_bytes());
            }
        }
        bytes.extend([0; 8]);
        assert_eq!(
            case_names(&bytes, 3).unwrap(),
            ["Собственный вес", "Покрытия", "Загружение 3"]
        );
        assert!(case_names(&bytes, 2).is_none());
    }

    #[test]
    fn reads_ansi_single_case_table() {
        let mut bytes = vec![5, 0, 2, 0, 255, 255, 0, 0, 5];
        bytes.extend([0xC2, 0xE5, 0xF2, 0xE5, 0xF0]);
        bytes.extend([0; 8]);
        assert_eq!(case_names(&bytes, 1).unwrap(), ["Ветер"]);
    }
}
#[derive(Debug, Default, serde::Serialize)]
pub struct Probe {
    pub nodal_objects_offset: Option<usize>,
    pub nodal_objects_end: Option<usize>,
    pub nodal_loads: usize,
    pub element_headers_verified: usize,
    pub element_prefixes_decoded: usize,
    pub element_loads: usize,
    pub first_element_offset: Option<usize>,
    pub diagnostics: Vec<String>,
    pub attachments: Vec<(u16, u16, u16, usize, f64)>,
}
/// Locate objects from topology. Element tails are bounded only by a separately
/// verified next header and current object's ordered end-node list.
#[derive(Debug, Clone, Copy, PartialEq, Eq, serde::Serialize)]
pub enum TargetKind {
    Node,
    Element,
}
#[derive(Debug, Clone, serde::Serialize)]
pub struct TargetRecord {
    pub target_kind: TargetKind,
    pub target_id: u32,
    pub record: Record,
}
#[derive(Debug, Clone, serde::Serialize)]
pub struct Case {
    pub id: u16,
    pub name: String,
}
#[derive(Debug, Default, serde::Serialize)]
pub struct Report {
    pub records: Vec<TargetRecord>,
    pub cases: Vec<Case>,
    pub unsupported_counts: std::collections::BTreeMap<u16, usize>,
    pub notes: Vec<String>,
}
impl Report {
    pub fn empty(note: String) -> Self {
        Self {
            notes: vec![note],
            ..Default::default()
        }
    }
}
pub fn probe_file(bytes: &[u8]) -> Result<Probe> {
    probe_impl(bytes, None)
}
pub fn read_uniform(bytes: &[u8]) -> Result<Report> {
    let mut report = Report::default();
    let probe = probe_impl(bytes, Some(&mut report))?;
    report.notes.extend(probe.diagnostics);
    let n = report.cases.iter().map(|c| c.id).max().unwrap_or(0);
    let count_offset = usize::from(bytes[0]) + 1 + 24;
    let declared = Reader(bytes).u32(count_offset).unwrap_or_default() as usize;
    let names = (declared >= usize::from(n))
        .then(|| case_names(bytes, declared))
        .flatten();
    if let Some(names) = names {
        for c in &mut report.cases {
            c.name = names[c.id as usize - 1].clone();
        }
        // Retain names of cases without supported physical records as well.
        for (i, name) in names.into_iter().enumerate() {
            let id = (i + 1) as u16;
            if !report.cases.iter().any(|c| c.id == id) {
                report.cases.push(Case { id, name });
            }
        }
        report.cases.sort_by_key(|c| c.id);
    } else if n != 0 || declared != 0 {
        report.notes.push(
            "Названия загружений не удалось однозначно прочитать; сохранены их номера".into(),
        );
    }
    Ok(report)
}
fn record_into(report: &mut Report, target_kind: TargetKind, target_id: u32, record: &Record) {
    if !report.cases.iter().any(|c| c.id == record.case_id) {
        report.cases.push(Case {
            id: record.case_id,
            name: format!("Загружение {}", record.case_id),
        });
    }
    if (target_kind == TargetKind::Element && matches!(record.code, 63 | 163))
        || (target_kind == TargetKind::Node && record.code == 103)
    {
        report.records.push(TargetRecord {
            target_kind,
            target_id,
            record: record.clone(),
        });
    } else {
        *report.unsupported_counts.entry(record.code).or_default() += 1;
    }
}
fn probe_impl(bytes: &[u8], mut report: Option<&mut Report>) -> Result<Probe> {
    let (nn, ne, ns) = super::layout(bytes)?;
    let es = ns + nn as usize * 28;
    let end = es + ne as usize * 38;
    let r = Reader(bytes);
    let mut out = Probe {
        nodal_objects_offset: None,
        nodal_objects_end: None,
        nodal_loads: 0,
        element_headers_verified: 0,
        element_prefixes_decoded: 0,
        element_loads: 0,
        first_element_offset: None,
        diagnostics: vec![],
        attachments: vec![],
    };
    // Full node-array verification prevents accepting an incidental coordinate match.
    let xyz = r.span(ns, 24)?;
    for p in end..bytes.len().saturating_sub(48) {
        if bytes[p..p + 4] != [0; 4] || r.u32(p + 4)? & 1 == 0 || r.span(p + 8, 24)? != xyz {
            continue;
        }
        let mut candidate_records = Vec::new();
        let parsed = (|| -> Result<(usize, usize)> {
            let mut o = p;
            let mut count = 0;
            for i in 0..nn as usize {
                if r.u32(o)? != i as u32
                    || r.u32(o + 4)? & 1 != r.u32(ns + i * 28 + 24)?
                    || r.span(o + 8, 24)? != r.span(ns + i * 28, 24)?
                {
                    return Err(format!("Не совпали данные узла {}, позиция {o}", i + 1));
                }
                let loads = r.u16(o + 38)? as usize;
                if loads > 10000 {
                    return Err("Превышено допустимое число нагрузок узла".into());
                }
                let mut at = o + 40;
                for _ in 0..loads {
                    let (record, next) = nodal_record(bytes, at)?;
                    at = next;
                    if report.is_some() {
                        candidate_records.push((i as u32 + 1, record));
                    }
                    count += 1;
                }
                o = at + 8;
                r.span(o, 0)?;
            }
            Ok((o, count))
        })();
        match parsed {
            Ok((o, n)) => {
                if let Some(report) = report.as_deref_mut() {
                    for (id, record) in candidate_records {
                        record_into(report, TargetKind::Node, id, &record);
                    }
                }
                out.nodal_objects_offset = Some(p);
                out.nodal_objects_end = Some(o);
                out.nodal_loads = n;
                break;
            }
            Err(e) => {
                if out.diagnostics.len() < 4 {
                    out.diagnostics.push(e);
                }
            }
        }
    }
    #[derive(Clone, Copy)]
    struct H {
        at: usize,
        n: usize,
    }
    let mut headers = vec![Vec::<H>::new(); ne as usize];
    for o in end + 10..bytes.len().saturating_sub(4) {
        let kind = r.u16(o)?;
        let n = match kind {
            10 => 2,
            41 => 4,
            42 => 3,
            44 => 4,
            56 => 1,
            _ => continue,
        };
        if r.u16(o + 2)? as usize != n {
            continue;
        }
        let id = r.u32(o - 8)? as usize;
        if id >= ne as usize
            || r.u32(o - 4)? & 1 != r.u32(es + id * 38 + 34)?
            || r.u16(o - 10)? != r.u16(es + id * 38 + 32)?
        {
            continue;
        }
        headers[id].push(H { at: o - 10, n });
    }
    let mut skipped_objects = 0usize;
    for i in 0..ne as usize {
        let raw = es + i * 38;
        if r.u32(raw + 34)? == 0 {
            continue;
        }
        let mut pairs = vec![];
        for h in &headers[i] {
            if i + 1 < headers.len() {
                for next in &headers[i + 1] {
                    if next.at > h.at + 18 + h.n * 4
                        && r.span(next.at - h.n * 4, h.n * 4)? == r.span(raw, h.n * 4)?
                    {
                        pairs.push((*h, next.at - h.n * 4));
                    }
                }
            }
        }
        // Observed terminal layout: own ordered node array, a zero/FFFF
        // service tail, 0x0002, then a one-entry type36 material table.
        // Validate fields, never a fixed absolute offset or EOF assumption.
        if i + 1 == headers.len() && headers[i].len() == 1 {
            let h = headers[i][0];
            let wanted = r.span(raw, h.n * 4)?;
            for end_nodes in h.at + 18..bytes.len().saturating_sub(h.n * 4 + 100) {
                if r.span(end_nodes, h.n * 4)? != wanted {
                    continue;
                }
                let tail = end_nodes + h.n * 4;
                for table in tail + 26..(tail + 130).min(bytes.len().saturating_sub(40)) {
                    let gap = &bytes[tail..table];
                    if gap.len() < 26
                        || gap[gap.len() - 2..] != [2, 0]
                        || gap[..gap.len() - 2].iter().any(|v| *v != 0 && *v != 255)
                        || r.u16(table)? != 1
                        || r.u16(table + 2)? != 36
                        || r.u16(table + 4)? != 1
                        || r.u32(table + 6)? != 100
                    {
                        continue;
                    }
                    let params = r.floats::<6>(table + 10, false)?;
                    if !(0. ..1000.).contains(&params[0])
                        || !(0. ..1e12).contains(&params[1])
                        || !(0. ..0.5).contains(&params[2])
                        || !(0.1..1000.).contains(&params[3])
                    {
                        continue;
                    }
                    pairs.push((h, end_nodes));
                }
            }
        }
        // Other object tables may contain the same numeric ID/type/header and
        // even be followed much later by the wanted node array. A real element
        // object additionally has a verified text + multiplier + load prefix.
        // Never resolve this ambiguity by choosing the first/nearest offset.
        pairs.retain(|(h, bound)| {
            let valid = (|| -> Result<()> {
                let (_, o) = super::sections::text(bytes, h.at + 18)?;
                if r.float(o, false)? != 1. {
                    return Err("Unknown load multiplier".into());
                }
                let count = r.u16(o + 4)? as usize;
                let mut at = o + 6;
                for _ in 0..count {
                    let (_, next) = element_record(bytes, at)?;
                    at = next;
                    if at > *bound {
                        return Err("Load prefix outside object".into());
                    }
                }
                if at > *bound {
                    return Err("Load prefix outside object".into());
                }
                Ok(())
            })();
            valid.is_ok()
        });
        if pairs.len() != 1 {
            skipped_objects += 1;
            if skipped_objects <= 5 {
                out.diagnostics.push(format!("КЭ {}: подтверждённых границ записи нагрузки — {}; нагрузки этого КЭ пропущены. Остальные КЭ читаются независимо.",i+1,pairs.len()));
            }
            continue;
        }
        let (h, bound) = pairs[0];
        out.element_headers_verified += 1;
        if out.first_element_offset.is_none() {
            out.first_element_offset = Some(h.at);
        }
        let parsed = (|| -> Result<(usize, Vec<Record>)> {
            let (_, o) = super::sections::text(bytes, h.at + 18)?;
            let multiplier = r.float(o, false)?;
            if multiplier != 1. {
                return Err(format!("Неподдержанный множитель нагрузки {multiplier}"));
            }
            let count = r.u16(o + 4)? as usize;
            let mut at = o + 6;
            let mut records = vec![];
            for _ in 0..count {
                let (record, next) = element_record(bytes, at)?;
                records.push(record);
                at = next;
                if at > bound {
                    return Err("Записи нагрузок выходят за границу объекта".into());
                }
            }
            Ok((at, records))
        })();
        match parsed {
            Ok((tail, records)) => {
                out.element_prefixes_decoded += 1;
                out.element_loads += records.len();
                for record in records {
                    if let Some(report) = report.as_deref_mut() {
                        record_into(report, TargetKind::Element, i as u32 + 1, &record);
                    }
                    if let Some(a) = out.attachments.iter_mut().find(|a| {
                        (a.0, a.1, a.2) == (record.group_raw, record.case_id, record.code)
                    }) {
                        a.3 += 1;
                        a.4 += record.payload[0] as f64;
                    } else {
                        out.attachments.push((
                            record.group_raw,
                            record.case_id,
                            record.code,
                            1,
                            record.payload[0] as f64,
                        ));
                    }
                }
                if out.element_prefixes_decoded == 1 {
                    out.diagnostics.push(format!("Справочно: начало служебной части записи КЭ — {tail}, начало подтверждённого списка узлов — {bound}"));
                }
            }
            Err(e) => {
                skipped_objects += 1;
                if skipped_objects <= 5 {
                    out.diagnostics.push(format!("КЭ {}: ошибка записи нагрузки в позиции {}: {e}; нагрузки этого КЭ пропущены",i+1,h.at));
                }
                continue;
            }
        }
    }
    if skipped_objects > 0 {
        out.diagnostics.push(format!("Неподтверждённые записи нагрузок пропущены у {skipped_objects} КЭ. Это не останавливает чтение остальных проверенных записей; их суммарные нагрузки могут быть неполными."));
    }
    Ok(out)
}

fn case_names(bytes: &[u8], count: usize) -> Option<Vec<String>> {
    if count == 0 || count > 1000 {
        return None;
    }
    let mut found = None;
    // The header declares the total number of cases, including cases without
    // physical records. The case metadata precedes the CString table.
    for (offset, prefix) in bytes.windows(4).enumerate() {
        if !matches!(prefix, [5, 0, 2 | 3, 0]) {
            continue;
        }
        let metadata = offset + prefix.len();
        let start = metadata + count * 4;
        let Some(fields) = bytes.get(metadata..start) else {
            continue;
        };
        let legacy = bytes.get(1..16) == Some(b"LIRA-SAPR 2013 ".as_slice())
            && prefix == [5, 0, 2, 0]
            && fields.starts_with(&[2, 0, 0, 0]);
        if !(legacy
            || fields.starts_with(&[0, 0, 1, 0])
            || count == 1 && fields == [255, 255, 0, 0])
            || !fields[4..]
                .chunks_exact(4)
                .all(|code| matches!(code, [0..=9, 0, 0 | 1, 0]))
        {
            continue;
        }
        let Some(names) = read_case_name_table(bytes, start, count) else {
            continue;
        };
        if found.as_ref().is_some_and(|previous| previous != &names) {
            return None;
        }
        found = Some(names);
    }
    found
}

fn read_case_name_table(bytes: &[u8], mut at: usize, count: usize) -> Option<Vec<String>> {
    let wide = bytes.get(at..)?.starts_with(&[255, 254, 255]);
    let mut names = Vec::with_capacity(count);
    for id in 1..=count {
        if bytes.get(at..)?.starts_with(&[255, 254, 255]) != wide {
            return None;
        }
        let (name, end) = super::sections::text(bytes, at).ok()?;
        if name.len() > 512
            || name
                .chars()
                .any(|c| c == '\u{fffd}' || c.is_control() && !matches!(c, '\r' | '\n' | '\t'))
        {
            return None;
        }
        let name = name.replace("\r\n", " ").replace(['\r', '\n', '\t'], " ");
        let name = name.trim();
        names.push(if name.is_empty() {
            format!("Загружение {id}")
        } else {
            name.to_owned()
        });
        at = end;
    }
    bytes.get(at..)?.starts_with(&[0; 8]).then_some(names)
}
