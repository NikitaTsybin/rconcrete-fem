//! Geometry subset of LIRA-SAPR's expanded processor input language.
use std::{collections::BTreeMap, path::Path};

#[derive(Clone, Debug)]
pub struct Node {
    pub id: usize,
    pub xyz: [f64; 3],
}
#[derive(Clone, Debug)]
pub struct Topology {
    pub id: usize,
    pub fe_type: u32,
    pub stiffness: usize,
    pub nodes: Vec<usize>,
}
#[derive(Debug)]
pub struct Geometry {
    pub nodes: Vec<Node>,
    pub elements: Vec<Topology>,
    pub skipped: BTreeMap<u32, usize>,
    pub documents: Vec<u32>,
}

pub fn read(path: &Path) -> Result<Geometry, String> {
    let bytes = std::fs::read(path).map_err(|e| format!("Не удалось прочитать файл: {e}"))?;
    let text = if bytes.starts_with(&[0xff, 0xfe]) {
        encoding_rs::UTF_16LE.decode(&bytes[2..]).0.into_owned()
    } else if bytes.starts_with(&[0xfe, 0xff]) {
        encoding_rs::UTF_16BE.decode(&bytes[2..]).0.into_owned()
    } else if let Ok(text) = std::str::from_utf8(&bytes) {
        text.to_owned()
    } else {
        encoding_rs::WINDOWS_1251.decode(&bytes).0.into_owned()
    };
    parse(&text)
}

fn documents(text: &str) -> Result<BTreeMap<u32, &str>, String> {
    let bytes = text.as_bytes();
    let mut result = BTreeMap::new();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] != b'(' {
            i += 1;
            continue;
        }
        let mut j = i + 1;
        while j < bytes.len() && bytes[j].is_ascii_whitespace() {
            j += 1;
        }
        let start = j;
        while j < bytes.len() && bytes[j].is_ascii_digit() {
            j += 1;
        }
        if start == j {
            i += 1;
            continue;
        }
        let number = text[start..j]
            .parse::<u32>()
            .map_err(|_| "Некорректный номер документа")?;
        while j < bytes.len() && bytes[j].is_ascii_whitespace() {
            j += 1;
        }
        if bytes.get(j) != Some(&b'/') {
            i += 1;
            continue;
        }
        let body = j + 1;
        let end = bytes[body..]
            .iter()
            .position(|b| *b == b')')
            .map(|p| p + body)
            .ok_or_else(|| format!("Документ {number}: отсутствует закрывающая скобка"))?;
        if result.insert(number, &text[body..end]).is_some() {
            return Err(format!(
                "Документ {number} повторяется; составные документы пока не поддерживаются"
            ));
        }
        i = end + 1;
    }
    Ok(result)
}
fn records(body: &str) -> Result<Vec<&str>, String> {
    let mut records: Vec<_> = body.split('/').collect();
    if records.last().is_some_and(|s| s.trim().is_empty()) {
        records.pop();
    } else {
        return Err("Последняя запись должна заканчиваться символом /".into());
    }
    if records.iter().any(|s| s.trim().is_empty()) {
        return Err(
            "Пустые записи не поддерживаются: нумерация не будет изменена автоматически".into(),
        );
    }
    Ok(records)
}
fn integer(value: &str, context: &str) -> Result<usize, String> {
    value.parse().map_err(|_| {
        format!(
            "{context}: ожидалось целое число, получено «{value}»; сжатый язык не поддерживается"
        )
    })
}
pub fn parse(text: &str) -> Result<Geometry, String> {
    let docs = documents(text)?;
    let coordinates = docs
        .get(&4)
        .ok_or("Не найден документ (4/ … ) с координатами узлов")?;
    let topology = docs
        .get(&1)
        .ok_or("Не найден документ (1/ … ) с элементами")?;
    let mut nodes = Vec::new();
    for (i, record) in records(coordinates)?.into_iter().enumerate() {
        let values: Vec<_> = record.split_whitespace().collect();
        if values.len() != 3 {
            return Err(format!(
                "Документ 4, узел {}: нужны три координаты X Y Z, получено {} полей",
                i + 1,
                values.len()
            ));
        }
        let mut xyz = [0.; 3];
        for (axis, value) in values.iter().enumerate() {
            xyz[axis] = value
                .replace(',', ".")
                .replace(['D', 'd'], "E")
                .parse::<f64>()
                .map_err(|_| format!("Узел {}: неверная координата «{value}»", i + 1))?;
            if !xyz[axis].is_finite() {
                return Err(format!("Узел {}: координаты должны быть конечными", i + 1));
            }
        }
        nodes.push(Node { id: i + 1, xyz });
    }
    if nodes.is_empty() {
        return Err("Документ 4 не содержит узлов".into());
    }
    let mut elements = Vec::new();
    let mut skipped = BTreeMap::new();
    for (i, record) in records(topology)?.into_iter().enumerate() {
        let fields: Vec<_> = record.split_whitespace().collect();
        let context = format!("Документ 1, элемент {}", i + 1);
        if fields.len() < 3 {
            return Err(format!("{context}: недостаточно полей"));
        }
        let fe_type = u32::try_from(integer(fields[0], &context)?)
            .map_err(|_| format!("{context}: неверный тип КЭ"))?;
        let stiffness = integer(fields[1], &context)?;
        let ids: Vec<_> = fields[2..]
            .iter()
            .map(|v| integer(v, &context))
            .collect::<Result<_, _>>()?;
        if let Some(id) = ids.iter().find(|id| **id == 0 || **id > nodes.len()) {
            return Err(format!("{context}: ссылка на отсутствующий узел {id}"));
        }
        let expected = match fe_type {
            1..=10 => 2,
            42 => 3,
            44 => 4,
            56 => 1,
            _ => {
                *skipped.entry(fe_type).or_insert(0) += 1;
                continue;
            }
        };
        if ids.len() != expected {
            return Err(format!(
                "{context}: для КЭ {fe_type} нужно {expected} узлов, получено {}",
                ids.len()
            ));
        }
        elements.push(Topology {
            id: i + 1,
            fe_type,
            stiffness,
            nodes: ids,
        });
    }
    if elements.is_empty() {
        return Err(format!(
            "Нет поддерживаемых элементов. Неподдержанные типы: {skipped:?}"
        ));
    }
    Ok(Geometry {
        nodes,
        elements,
        skipped,
        documents: docs.keys().copied().collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn records_are_not_lines_and_node_identity_is_preserved() {
        let g=parse("(0/1; Тест/)(1/10 7 1 2/42 2 1 2 3/44 3 1 2 3 4/56 8 5/)(4/0 0 0/1 0 0/0 1 0/1 1 0/0 0 0/)").unwrap();
        assert_eq!(g.nodes.len(), 5);
        assert_eq!(g.elements.len(), 4);
        assert_eq!(g.nodes[4].id, 5);
        assert_eq!(g.nodes[0].xyz, g.nodes[4].xyz);
        assert_eq!(g.elements[2].nodes, vec![1, 2, 3, 4]);
        assert_eq!(g.elements[0].stiffness, 7);
    }
    #[test]
    fn rejects_invalid_geometry_without_silent_renumbering() {
        assert!(parse("(4/0 0 0//1 0 0/)(1/10 1 1 3/)").is_err());
        assert!(parse("(4/0 0 0/)(1/10 1 1 2/)").is_err());
        assert!(parse("(4/NaN 0 0/)(1/56 1 1/)").is_err());
        assert!(parse("(4/0 0 0/)(1/42 1 1/)").is_err());
        assert!(parse("(4/0 0 0/)").is_err());
    }
    #[test]
    fn skipped_types_keep_original_element_numbers() {
        let g = parse("(4/1D0 2,5 -3e0/)(1/999 1 1/56 2 1/)").unwrap();
        assert_eq!(g.elements[0].id, 2);
        assert_eq!(g.skipped[&999], 1);
        assert_eq!(g.nodes[0].xyz, [1., 2.5, -3.]);
    }
}
