//! Scale boundary expressions. Reinforcement uses a 1000 mm wide strip.
use super::{color_scale::Breaks, reinforcement::parse_area_terms};

pub fn value(text: &str, reinforcement: bool) -> Result<f64, String> {
    let text = text.trim();
    if text.is_empty() || text.len() > 1024 {
        return Err("Введите границу диапазона (до 1024 байт)".into());
    }
    let result = if reinforcement {
        let lower = text.to_lowercase();
        if lower.contains("nan") || lower.contains("inf") || lower.contains("e-") {
            return Err("NaN, inf и отрицательная степень e- не поддерживаются".into());
        }
        let terms = parse_area_terms(text, 1000.)?;
        if terms
            .iter()
            .any(|t| !t.area_cm2.is_finite() || t.diameter_mm.is_some_and(|d| !d.is_finite()))
        {
            return Err("Площадь и диаметр должны быть конечными".into());
        }
        terms.iter().map(|t| t.area_cm2).sum::<f64>()
    } else {
        text.replace(',', ".")
            .parse::<f64>()
            .map_err(|_| "Ожидалась числовая граница".to_string())?
    };
    if !result.is_finite() || (reinforcement && result < 0.) {
        return Err("Площадь должна быть конечной и неотрицательной".into());
    }
    Ok(result)
}

/// Deficit scales accept signed numbers as well as ordinary reinforcement expressions.
pub fn value_mode(text: &str, reinforcement: bool, deficit: bool) -> Result<f64, String> {
    if deficit {
        if let Ok(v) = text.trim().replace(',', ".").parse::<f64>() {
            return if v.is_finite() {
                Ok(v)
            } else {
                Err("Граница должна быть конечной".into())
            };
        }
    }
    value(text, reinforcement)
}
/// Each entered expression is the inclusive upper limit of its row.
pub fn breaks(ends: &[String], minimum: f64, reinforcement: bool) -> Result<Breaks, String> {
    breaks_mode(ends, minimum, reinforcement, false)
}
pub fn breaks_mode(
    ends: &[String],
    minimum: f64,
    reinforcement: bool,
    deficit: bool,
) -> Result<Breaks, String> {
    let mut values = ends
        .iter()
        .enumerate()
        .map(|(i, s)| {
            value_mode(s, reinforcement, deficit).map_err(|e| format!("Диапазон {}: {e}", i + 1))
        })
        .collect::<Result<Vec<_>, _>>()?;
    for i in 1..values.len() {
        if (values[i] - values[i - 1]).abs() <= 1e-10 * values[i].abs().max(1.) {
            values[i] = values[i - 1];
        }
    }
    let first = *values.first().ok_or("Нет диапазонов")?;
    let lower = if minimum < first {
        minimum
    } else {
        first - first.abs().max(1.)
    };
    let mut boundaries = vec![lower.to_string()];
    boundaries.extend(values.iter().map(ToString::to_string));
    Breaks::parse(&boundaries.join(";"))
}

pub fn sort_order(starts: &[String], reinforcement: bool) -> Result<Vec<usize>, String> {
    sort_order_mode(starts, reinforcement, false)
}
pub fn sort_order_mode(
    starts: &[String],
    reinforcement: bool,
    deficit: bool,
) -> Result<Vec<usize>, String> {
    let values = starts
        .iter()
        .map(|s| value_mode(s, reinforcement, deficit))
        .collect::<Result<Vec<_>, _>>()?;
    let mut order: Vec<_> = (0..values.len()).collect();
    order.sort_by(|&a, &b| values[a].total_cmp(&values[b]));
    Ok(order)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn expressions_use_metre_strip_and_validate_values() {
        let area = value("d12s200", true).unwrap();
        assert!((area - 5.65486677646).abs() < 1e-9);
        assert_eq!(value("3,5", true).unwrap(), 3.5);
        assert!((value("5d12 + 3 - 2", true).unwrap() - area - 1.).abs() < 1e-9);
        for s in ["", "NaN", "d8sinf", "1e-3", "-2", "hello"] {
            assert!(value(s, true).is_err(), "{s}");
        }
        assert_eq!(value("-2,5", false).unwrap(), -2.5);
    }
}
