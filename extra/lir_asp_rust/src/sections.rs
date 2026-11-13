//! Verified repeated section tables and per-element assignments; no absolute offsets.
use super::{Reader, Result};
use std::collections::BTreeMap;
#[derive(Debug, Clone, PartialEq, serde::Serialize)]
pub struct Definition {
    pub id: u16,
    pub kind: u16,
    pub flag: u32,
    pub name: String,
    pub color: [u8; 3],
    pub parameters: [f32; 6],
}
pub(super) fn text(bytes: &[u8], mut o: usize) -> Result<(String, usize)> {
    fn length(bytes: &[u8], o: &mut usize) -> Result<u32> {
        let r = Reader(bytes);
        let n = *r.span(*o, 1)?.first().unwrap();
        *o += 1;
        if n < 255 {
            return Ok(n as u32);
        }
        let n = r.u16(*o)?;
        *o += 2;
        if n < 65535 {
            Ok(n as u32)
        } else {
            let n = r.u32(*o)?;
            *o += 4;
            Ok(n)
        }
    }
    let mut n = length(bytes, &mut o)?;
    let wide = n == 65534;
    if wide {
        n = length(bytes, &mut o)?;
    }
    if n > 4096 {
        return Err("Unsupported section name length".into());
    }
    let size = n as usize * if wide { 2 } else { 1 };
    let raw = Reader(bytes).span(o, size)?;
    let name = if wide {
        String::from_utf16(
            &raw.chunks_exact(2)
                .map(|v| u16::from_le_bytes([v[0], v[1]]))
                .collect::<Vec<_>>(),
        )
        .map_err(|_| "Invalid UTF16 section name")?
    } else {
        encoding_rs::WINDOWS_1251.decode(raw).0.into_owned()
    };
    Ok((name, o + size))
}
fn table(bytes: &[u8], start: usize) -> Result<(Vec<Definition>, usize)> {
    let r = Reader(bytes);
    let n = r.u16(start.checked_sub(2).ok_or("Section table boundary")?)? as usize;
    if n == 0 || n > 4096 {
        return Err("Unsupported section count".into());
    }
    let mut out = vec![];
    let mut o = start;
    for i in 1..=n {
        let kind = r.u16(o)?;
        let id = r.u16(o + 2)?;
        let flag = r.u32(o + 4)?;
        if id as usize != i || flag > 1 {
            return Err("Unsupported section record header".into());
        }
        let (name_offset, tail) = match kind {
            0 => (40, 22),
            36 => (34, 28),
            55 => (36, 4),
            _ => return Err("Unsupported section definition kind".into()),
        };
        if (kind == 36 && !matches!(r.span(o + 32, 2)?, [255, 255] | [0, 0]))
            || (kind == 0
                && r.span(o + 32, 8)? != [255, 255, 0, 0, 0, 0, 0, 0]
                && r.span(o + 32, 8)? != [0; 8])
        {
            return Err("Unsupported physical section layout".into());
        }
        let p = r.floats::<6>(o + 8, false)?.map(|v| v as f32);
        if matches!(kind, 0 | 36)
            && (!(0. ..1000.).contains(&p[0])
                || !(0. ..1e12).contains(&p[1])
                || !(0.1..=1000.).contains(&p[3])
                || (kind == 0 && !(0.1..=1000.).contains(&p[2]))
                // LIR can store the incompressible limit in a shell record.
                // The calculation model validates its own usable material range.
                || (kind == 36 && !(0. ..=0.5).contains(&p[2])))
        {
            return Err("Invalid section parameters".into());
        }
        let (name, end) = text(bytes, o + name_offset)?;
        if name.chars().any(|c| c == '\0' || c.is_control()) {
            return Err("Invalid section name".into());
        }
        let color = r.span(end, 3)?.try_into().unwrap();
        r.span(end, tail)?;
        out.push(Definition {
            id,
            kind,
            flag,
            name,
            color,
            parameters: p,
        });
        o = end + tail;
    }
    Ok((out, o))
}
fn same_physics(a: &[Definition], b: &[Definition]) -> bool {
    a.len() == b.len()
        && a.iter().zip(b).all(|(a, b)| {
            a.id == b.id
                && a.kind == b.kind
                && (!matches!(a.kind, 0 | 36)
                    || a.parameters == b.parameters && a.name == b.name && a.color == b.color)
        })
}
pub fn read(
    bytes: &[u8],
    start: usize,
    active: &[bool],
    element_kinds: &BTreeMap<u32, u16>,
) -> (BTreeMap<u16, Definition>, Vec<u16>, Vec<String>) {
    let r = Reader(bytes);
    let mut tables = vec![];
    for o in start.max(2)..bytes.len().saturating_sub(64) {
        if !matches!(bytes[o], 0 | 36 | 55)
            || bytes[o + 1] != 0
            || bytes[o + 2] != 1
            || bytes[o + 3] != 0
            || bytes[o + 4] > 1
            || bytes[o + 5..o + 8] != [0, 0, 0]
        {
            continue;
        }
        if let Ok((rows, end)) = table(bytes, o) {
            tables.push((o, end, rows));
        }
    }
    let mut found = None;
    let mut notes = vec![];
    let mut conflicting = false;
    for (start, end, rows) in &tables {
        // A following decoded table is another copy, never the assignment array.
        if tables.iter().any(|(s, _, _)| *s == *end + 2) {
            continue;
        }
        let previous = tables.iter().find(|(_, p, _)| *p + 2 == *start);
        if previous.is_some_and(|(_, _, a)| !same_physics(a, rows)) {
            conflicting = true;
            continue;
        }
        for shift in [2, 4] {
            let Ok(raw) = r.span(end + shift, active.len().saturating_mul(2)) else {
                continue;
            };
            let values = raw
                .chunks_exact(2)
                .map(|v| u16::from_le_bytes([v[0], v[1]]))
                .collect::<Vec<_>>();
            let valid = values.iter().enumerate().all(|(i, &v)| {
                if v == u16::MAX {
                    return !active[i];
                }
                let Some(section) = rows.get(v as usize) else {
                    return false;
                };
                if !active[i] {
                    return true;
                }
                match element_kinds.get(&(i as u32 + 1)) {
                    Some(10) => section.kind == 0,
                    Some(41 | 42 | 44) => section.kind == 36,
                    Some(56) => section.kind == 55,
                    _ => true,
                }
            });
            if !valid {
                continue;
            }
            let assignments = values
                .into_iter()
                .map(|v| if v == u16::MAX { 0 } else { v + 1 })
                .collect::<Vec<_>>();
            let definitions = rows
                .iter()
                .cloned()
                .map(|v| (v.id, v))
                .collect::<BTreeMap<_, _>>();
            if found
                .as_ref()
                .is_some_and(|old| old != &(definitions.clone(), assignments.clone()))
            {
                return(Default::default(),vec![],vec!["Найдены неоднозначные таблицы или смещения назначений; автоматическое назначение отключено".into()]);
            }
            if previous.is_some_and(|(_, _, a)| a != rows) {
                notes.push("Служебные данные копий жёсткостей различаются; физические параметры, имена и цвета поддержанных сечений совпали. Служебные флаги не интерпретируются, тип 55 не импортируется.".into());
            }
            if previous.is_none() {
                notes.push("Прочитана одна поддержанная таблица жёсткостей. Назначения проверены по типам КЭ и признакам отключённых элементов.".into());
            }
            found = Some((definitions, assignments));
        }
    }
    if let Some((d, a)) = found {
        (d, a, notes)
    } else {
        if conflicting {
            notes.push("Физические параметры копий жёсткостей различаются; автоматическое назначение отключено".into());
        }
        (Default::default(), vec![], notes)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn record(kind: u16, id: u16, wide: bool) -> Vec<u8> {
        let mut b = vec![];
        b.extend(kind.to_le_bytes());
        b.extend(id.to_le_bytes());
        b.extend(1u32.to_le_bytes());
        for v in [
            2.5f32,
            1988450.,
            if kind == 0 { 30. } else { 0.2 },
            if kind == 0 { 120. } else { 70. },
            0.,
            0.,
        ] {
            b.extend(v.to_le_bytes());
        }
        let (offset, tail) = if kind == 0 { (40, 22) } else { (34, 28) };
        b.resize(offset, 0);
        b[32..34].copy_from_slice(&[255, 255]);
        if wide {
            b.extend([255, 254, 255, 5]);
            for c in "Пилон".encode_utf16() {
                b.extend(c.to_le_bytes());
            }
        } else {
            b.push(6);
            b.extend(b"Column");
        }
        b.extend([10, 20, 30]);
        b.resize(b.len() + tail - 3, 0);
        b
    }
    #[test]
    fn single_table_four_byte_prefix_and_inactive_sentinel_are_verified() {
        let mut row = record(0, 1, true);
        row[32..40].fill(0);
        row[12..16].copy_from_slice(&0f32.to_le_bytes());
        let mut bytes = 1u16.to_le_bytes().to_vec();
        bytes.extend(row);
        bytes.extend([0, 0, 34, 0]);
        for v in [0u16, u16::MAX, 0] {
            bytes.extend(v.to_le_bytes());
        }
        let (d, a, _) = read(
            &bytes,
            0,
            &[true, false, true],
            &BTreeMap::from([(1, 10), (3, 10)]),
        );
        assert_eq!(d[&1].parameters[1], 0.);
        assert_eq!(a, vec![1, 0, 1]);
        let (d, _, _) = read(
            &bytes,
            0,
            &[true, true, true],
            &BTreeMap::from([(1, 10), (2, 10), (3, 10)]),
        );
        assert!(d.is_empty());
    }
    #[test]
    fn shell_poisson_limit_keeps_the_mixed_section_table_readable() {
        let mut shell = record(36, 1, true);
        shell[16..20].copy_from_slice(&0.5f32.to_le_bytes());
        let beam = record(0, 2, false);
        let mut bytes = 2u16.to_le_bytes().to_vec();
        bytes.extend(shell);
        bytes.extend(beam);
        bytes.extend(2u16.to_le_bytes());
        bytes.extend([0, 0, 1, 0]);
        let (sections, assignments, _) = read(
            &bytes,
            0,
            &[true, true],
            &BTreeMap::from([(1, 44), (2, 10)]),
        );
        assert_eq!(assignments, [1, 2]);
        assert_eq!(sections[&1].parameters[2], 0.5);
        assert_eq!(sections[&2].parameters[2..4], [30., 120.]);
    }
    #[test]
    fn copy_flags_may_differ_but_supported_physical_values_must_match() {
        let mut row = record(0, 1, true);
        row[32..40].fill(0);
        let mut first = row.clone();
        first[4..8].copy_from_slice(&0u32.to_le_bytes());
        let mut bytes = 1u16.to_le_bytes().to_vec();
        bytes.extend(first);
        bytes.extend(1u16.to_le_bytes());
        bytes.extend(row);
        bytes.extend(3u16.to_le_bytes());
        bytes.extend(0u16.to_le_bytes());
        let (d, a, n) = read(&bytes, 0, &[true], &BTreeMap::from([(1, 10)]));
        assert_eq!(d.len(), 1);
        assert_eq!(a, vec![1]);
        assert!(!n.is_empty());
    }
    #[test]
    fn repeated_tables_with_cyrillic_and_ansi_names_and_zero_based_assignments() {
        let mut ramp = record(36, 3, true);
        ramp[32..34].fill(0);
        let records = [record(36, 1, false), record(0, 2, true), ramp].concat();
        let mut b = 3u16.to_le_bytes().to_vec();
        b.extend(&records);
        b.extend(3u16.to_le_bytes());
        b.extend(&records);
        b.extend(3u16.to_le_bytes());
        b.extend(1u16.to_le_bytes());
        b.extend(0u16.to_le_bytes());
        b.extend(2u16.to_le_bytes());
        let kinds = BTreeMap::from([(1, 10), (2, 44), (3, 42)]);
        let (d, a, _) = read(&b, 0, &[true; 3], &kinds);
        assert_eq!(a, vec![2, 1, 3]);
        assert_eq!(d[&2].name, "Пилон");
        assert_eq!(d[&2].parameters[2..4], [30., 120.]);
        assert_eq!(d[&1].color, [10, 20, 30]);
        assert_eq!(d[&3].parameters[3], 70.);
        let second = 2 + records.len() + 2;
        b[second + 12..second + 16].copy_from_slice(&12345f32.to_le_bytes());
        let (d, a, n) = read(&b, 0, &[true; 3], &kinds);
        assert!(d.is_empty() && a.is_empty() && !n.is_empty());
    }
}
