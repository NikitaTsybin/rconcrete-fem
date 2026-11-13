//! Limited source stamp reader. Raw header semantics remain opaque; case/direction
//! are joined exclusively through topology-verified physical records.
use super::{Reader, Result};
#[derive(Clone, Debug, serde::Serialize)]
pub struct Stamp {
    pub id: u32,
    pub kind: u32,
    pub raw_kind: u32,
    pub source_group: u16,
    pub source_direction: [f64; 3],
    pub end_value: f64,
    pub physical_resultant: Option<f64>,
    pub attachment_case_id: Option<u16>,
    pub attachment_direction_code: Option<u16>,
    pub name: String,
    pub points: Vec<[f64; 3]>,
    pub contours: Vec<Vec<[f64; 3]>>,
    pub value: f64,
    pub case_id: Option<u16>,
    pub direction_code: Option<u16>,
}
#[derive(Debug, serde::Serialize)]
pub struct Report {
    pub stamps: Vec<Stamp>,
    pub notes: Vec<String>,
}
pub fn read(bytes: &[u8]) -> Result<Report> {
    let marker: Vec<u8> = vec![
        196, 97, 237, 237, 251, 101, 32, 225, 235, 111, 234, 111, 226, 32, 237, 97, 227, 240, 121,
        231, 111, 234, 32, 228, 235, 255, 32, 232, 245, 32, 112, 101, 228, 97, 234, 242, 232, 112,
        111, 226, 97, 237, 232, 255, 32, 226, 32, 67, 65, 207, 212, 200, 80,
    ];
    let positions = bytes
        .windows(marker.len())
        .enumerate()
        .filter_map(|(i, b)| (b == marker).then_some(i))
        .collect::<Vec<_>>();
    if positions.is_empty() {
        return Ok(Report {
            stamps: vec![],
            notes: vec!["В файле не найдена таблица исходных контуров нагрузок".into()],
        });
    }
    if positions.len() != 1 {
        return Err("В файле найдены неоднозначные таблицы исходных штампов".into());
    }
    let start = positions[0] + marker.len() + 1;
    let r = Reader(bytes);
    let mut offset = start;
    let mut stamps = vec![];
    let mut notes = vec![];
    // Only the observed header is accepted, never guessed from a plausible float.
    let header = [1, 10, 0, 2, 0, 2, 0, 0, 0];
    while offset + 105 < bytes.len() {
        if bytes[offset..offset + 9] != header {
            offset += 1;
            continue;
        }
        let parsed = (|| -> Result<(Stamp, usize)> {
            let value = r.float(offset + 9, true)?;
            let source_direction = [
                r.float(offset + 73, true)?,
                r.float(offset + 81, true)?,
                r.float(offset + 89, true)?,
            ];
            let count = r.u32(offset + 97)? as usize;
            if !(1..=1000).contains(&count) {
                return Err(format!("Некорректное количество контуров штампа: {count}"));
            }
            let mut end = offset + 101;
            let mut contours = Vec::with_capacity(count);
            for _ in 0..count {
                let n = r.u32(end)? as usize;
                end += 4;
                if !(2..=100000).contains(&n) {
                    return Err("Некорректное количество вершин штампа".into());
                }
                let mut points = Vec::with_capacity(n);
                for i in 0..n {
                    points.push([
                        r.float(end + i * 24, true)?,
                        r.float(end + 8 + i * 24, true)?,
                        r.float(end + 16 + i * 24, true)?,
                    ]);
                }
                end += n * 24;
                contours.push(points);
            }
            let points = contours[0].clone();
            let id = r.u32(end)?;
            let raw_kind = r.u32(end + 4)?;
            // Observed area772 / line770 shapes; high bits remain opaque.
            let kind = match raw_kind {
                772 => 4,
                770 => 2,
                _ => raw_kind,
            };
            if id == 0
                || !matches!(kind, 2 | 4)
                || kind == 4
                    && contours
                        .iter()
                        .any(|p| p.len() < 4 || p.first() != p.last())
            {
                return Err("Неподдержанная форма штампа".into());
            }
            let (name, end) = super::sections::text(bytes, end + 8)?;
            Ok((
                Stamp {
                    id,
                    kind,
                    raw_kind,
                    source_group: (stamps.len() + 1)
                        .try_into()
                        .map_err(|_| "Слишком много исходных штампов")?,
                    source_direction,
                    end_value: r.float(offset + 41, true)?,
                    physical_resultant: None,
                    attachment_case_id: None,
                    attachment_direction_code: None,
                    name,
                    points,
                    contours,
                    value,
                    case_id: None,
                    direction_code: None,
                },
                end,
            ))
        })();
        let (stamp, next) = match parsed {
            Ok(value) => value,
            Err(error) => {
                notes.push(format!("Штамп в позиции {offset}: {error}; пропущен"));
                offset += header.len();
                continue;
            }
        };
        // A source ID is not a global record identity. Keep every decoded
        // occurrence: attachment groups address the 1-based source sequence,
        // so dropping an occurrence also shifts all subsequent case bindings.
        stamps.push(stamp);
        offset = next;
    }
    let probe = super::physical_loads::probe_file(bytes)?;
    notes.extend(probe.diagnostics);
    let mut different_resultants = 0usize;
    for s in &mut stamps {
        // Even unsupported directions occupy a source-sequence slot. Do not
        // discard them while decoding, or later attachment IDs would shift.
        if s.source_direction != [0., 0., 1.] {
            notes.push(format!(
                "Штамп {} (запись {}): направление {:?} не поддержано; порядковый номер сохранён",
                s.id, s.source_group, s.source_direction
            ));
            continue;
        }
        let attached = probe
            .attachments
            .iter()
            .filter(|a| a.0 == s.source_group)
            .collect::<Vec<_>>();
        let flat = s
            .contours
            .iter()
            .flatten()
            .all(|p| (p[2] - s.points[0][2]).abs() <= 1e-8);
        let measure = if s.kind == 4 {
            s.contours
                .iter()
                .enumerate()
                .map(|(i, p)| {
                    let depth = s
                        .contours
                        .iter()
                        .enumerate()
                        .filter(|(j, r)| *j != i && ring_contains(p[0], r))
                        .count();
                    let area = p
                        .windows(2)
                        .map(|e| e[0][0] * e[1][1] - e[1][0] * e[0][1])
                        .sum::<f64>()
                        .abs()
                        * 0.5;
                    if depth % 2 == 0 {
                        area
                    } else {
                        -area
                    }
                })
                .sum::<f64>()
        } else {
            s.contours
                .iter()
                .map(|p| {
                    p.windows(2)
                        .map(|e| (e[1][0] - e[0][0]).hypot(e[1][1] - e[0][1]))
                        .sum::<f64>()
                })
                .sum()
        };
        let expected = measure * s.value;
        if attached.len() == 1 && attached[0].2 == 153 && flat && measure > 0. {
            s.physical_resultant = Some(attached[0].4);
            s.attachment_case_id = Some(attached[0].1);
            s.attachment_direction_code = Some(153);
        }
        if attached.len() == 1
            && attached[0].2 == 153
            && flat
            && measure > 0.
            && s.value == s.end_value
            && (attached[0].4 - expected).abs() <= expected.abs().max(1.) * 1e-5
        {
            s.case_id = Some(attached[0].1);
            s.direction_code = Some(153);
        } else {
            different_resultants += 1;
        }
    }
    if different_resultants > 0 {
        notes.push(format!("У {different_resultants} штампов интеграл исходного контура отличается от сохранённой КЭ-проекции. Это справочная информация, а не причина пропуска: проекция может быть обрезана отверстиями, иметь другую отметку или быть неполной. Исходная интенсивность не изменяется."));
    }
    notes.push("Загружение штампа определяется по его порядковому номеру в исходной таблице, а не по ID. Контур переносится независимо от старой КЭ-проекции.".into());
    Ok(Report { stamps, notes })
}

fn ring_contains(p: [f64; 3], ring: &[[f64; 3]]) -> bool {
    let mut inside = false;
    for e in ring.windows(2) {
        let [a, b] = [e[0], e[1]];
        if (a[1] > p[1]) != (b[1] > p[1])
            && p[0] < (b[0] - a[0]) * (p[1] - a[1]) / (b[1] - a[1]) + a[0]
        {
            inside = !inside;
        }
    }
    inside
}
