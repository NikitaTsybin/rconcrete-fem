use std::f64::consts::PI;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReinforcementResult {
    pub area_cm2: f64,
    pub ratio_percent: f64,
    pub equivalent_diameter_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpacingDesign {
    pub diameter_for_spacing_mm: f64,
    pub spacing_for_diameter_mm: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ReinforcementTerm {
    pub area_cm2: f64,
    pub diameter_mm: Option<f64>,
}

pub fn design_for_area(
    area_cm2: f64,
    width_mm: f64,
    target_spacing_mm: f64,
    target_diameter_mm: f64,
) -> Result<SpacingDesign, &'static str> {
    if !area_cm2.is_finite() || area_cm2 <= 0.0 {
        return Err("Площадь As должна быть больше нуля");
    }
    if !width_mm.is_finite()
        || !target_spacing_mm.is_finite()
        || !target_diameter_mm.is_finite()
        || width_mm <= 0.0
        || target_spacing_mm <= 0.0
        || target_diameter_mm <= 0.0
    {
        return Err("Ширина, шаг и диаметр должны быть больше нуля");
    }

    let diameter_for_spacing_mm =
        (area_cm2 * 4.0 * target_spacing_mm / (PI * width_mm)).sqrt() * 10.0;
    let diameter_cm = target_diameter_mm / 10.0;
    let spacing_for_diameter_mm = PI * diameter_cm.powi(2) / 4.0 * width_mm / area_cm2;

    Ok(SpacingDesign {
        diameter_for_spacing_mm,
        spacing_for_diameter_mm,
    })
}

pub fn calculate(
    expression: &str,
    width_mm: f64,
    height_mm: f64,
    equivalent_bar_count: u32,
) -> Result<ReinforcementResult, String> {
    if width_mm <= 0.0 || height_mm <= 0.0 {
        return Err("Размеры сечения должны быть больше нуля".to_owned());
    }
    if equivalent_bar_count == 0 {
        return Err("Число стержней должно быть больше нуля".to_owned());
    }

    let area_cm2 = parse_terms(expression, width_mm)?;
    let section_area_cm2 = width_mm * height_mm / 100.0;
    let ratio_percent = area_cm2 * 100.0 / section_area_cm2;
    let equivalent_diameter_mm = if area_cm2 >= 0.0 {
        (area_cm2 * 4.0 / (PI * equivalent_bar_count as f64)).sqrt() * 10.0
    } else {
        return Err("Суммарная площадь не может быть отрицательной".to_owned());
    };

    Ok(ReinforcementResult {
        area_cm2,
        ratio_percent,
        equivalent_diameter_mm,
    })
}

fn parse_terms(expression: &str, width_mm: f64) -> Result<f64, String> {
    Ok(parse_area_terms(expression, width_mm)?
        .into_iter()
        .map(|term| term.area_cm2)
        .sum())
}

pub fn parse_area_terms(expression: &str, width_mm: f64) -> Result<Vec<ReinforcementTerm>, String> {
    let normalized = normalize(expression);
    if normalized.is_empty() {
        return Ok(Vec::new());
    }

    let mut terms = Vec::new();
    let mut found = false;
    for raw_term in normalized.replace('-', "+-").split('+') {
        if raw_term.is_empty() {
            continue;
        }
        found = true;
        terms.push(
            parse_term(raw_term, width_mm).map_err(|message| format!("{message}: «{raw_term}»"))?,
        );
    }
    if !found {
        return Err("В строке нет расчётных элементов".to_owned());
    }
    Ok(terms)
}

/// Возвращает `true`, если запись арматуры сама задаёт диаметр стержней.
/// Помимо форм `5d12` учитывается краткая запись с шагом `10/200`.
pub fn expression_has_diameter(expression: &str) -> bool {
    let normalized = normalize(expression);
    if normalized.contains('d') {
        return true;
    }
    normalized.split(['+', '-']).any(|term| {
        term.split_once('s')
            .is_some_and(|(diameter, spacing)| !diameter.is_empty() && !spacing.is_empty())
    })
}

fn normalize(input: &str) -> String {
    input
        .chars()
        .filter(|character| !character.is_whitespace())
        .map(|character| match character {
            'д' | 'Д' | 'в' | 'В' | 'ф' | 'Ф' | 'D' | 'ø' | '⌀' => 'd',
            'ы' | 'Ы' | 'ш' | 'Ш' | 'S' | '/' => 's',
            ',' => '.',
            other => other,
        })
        .collect()
}

fn parse_term(raw_term: &str, width_mm: f64) -> Result<ReinforcementTerm, &'static str> {
    if let Ok(direct_area) = raw_term.parse::<f64>() {
        return Ok(ReinforcementTerm {
            area_cm2: direct_area,
            diameter_mm: None,
        });
    }

    let (sign, term) = raw_term
        .strip_prefix('-')
        .map_or((1.0, raw_term), |term| (-1.0, term));
    if term.is_empty() {
        return Err("Пустой элемент");
    }

    let (count, diameter_mm, spacing_mm) = if let Some((left, right)) = term.split_once('d') {
        let (diameter, spacing_after) = split_spacing(right)?;
        let diameter = number(diameter, "Не указан диаметр")?;

        if let Some(spacing) = left.strip_prefix('s') {
            (1.0, diameter, number(spacing, "Не указан шаг")?)
        } else {
            let count = if left.is_empty() {
                1.0
            } else {
                number(left, "Некорректное количество")?
            };
            let spacing = spacing_after
                .map(|value| number(value, "Не указан шаг"))
                .transpose()?
                .unwrap_or(width_mm);
            (count, diameter, spacing)
        }
    } else {
        let (diameter, spacing) = split_spacing(term)?;
        let spacing = spacing.ok_or("Ожидался формат d12 или 10/200")?;
        (
            1.0,
            number(diameter, "Не указан диаметр")?,
            number(spacing, "Не указан шаг")?,
        )
    };

    if count <= 0.0 || diameter_mm <= 0.0 || spacing_mm <= 0.0 {
        return Err("Количество, диаметр и шаг должны быть больше нуля");
    }

    let diameter_cm = diameter_mm / 10.0;
    Ok(ReinforcementTerm {
        area_cm2: sign * PI * diameter_cm.powi(2) / 4.0 * width_mm / spacing_mm * count,
        diameter_mm: Some(diameter_mm),
    })
}

fn split_spacing(value: &str) -> Result<(&str, Option<&str>), &'static str> {
    let mut parts = value.split('s');
    let first = parts.next().unwrap_or_default();
    let second = parts.next();
    if parts.next().is_some() {
        return Err("Слишком много разделителей шага");
    }
    Ok((first, second))
}

fn number(value: &str, message: &'static str) -> Result<f64, &'static str> {
    value.parse::<f64>().map_err(|_| message)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_mixed_reinforcement_terms() {
        let terms = parse_area_terms("10/200 + s200d12 + d8s200 + 2d16 + 3 - 2", 1000.0).unwrap();
        let expected = PI * 0.5_f64.powi(2) * 5.0
            + PI * 0.6_f64.powi(2) * 5.0
            + PI * 0.4_f64.powi(2) * 5.0
            + PI * 0.8_f64.powi(2) * 2.0
            + 1.0;
        let area = terms.iter().map(|term| term.area_cm2).sum::<f64>();
        assert!((area - expected).abs() < 0.0001);
    }

    #[test]
    fn accepts_keyboard_layout_aliases() {
        let latin = parse_area_terms("2d16+d8s200", 1000.0).unwrap();
        let aliases = parse_area_terms("2Д16+В8Ы200", 1000.0).unwrap();
        assert_eq!(latin, aliases);
    }
}
