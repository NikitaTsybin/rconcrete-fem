//! Inference for the two supplied NumPy archives. No Python/BLAS runtime.
use std::io::{Cursor, Read};
pub struct Network {
    mean: Vec<f32>,
    std: Vec<f32>,
    pub ymean: Vec<f32>,
    pub ystd: Vec<f32>,
    weights: Vec<Vec<f32>>,
    biases: Vec<Vec<f32>>,
    sizes: Vec<usize>,
}
fn array(
    zip: &mut zip::ZipArchive<Cursor<&[u8]>>,
    name: &str,
    shape: &[usize],
) -> Result<Vec<f32>, String> {
    let mut file = zip
        .by_name(&format!("{name}.npy"))
        .map_err(|e| e.to_string())?;
    if file.size() > 2_000_000 {
        return Err("Слишком большой массив модели".into());
    }
    let mut bytes = Vec::new();
    file.read_to_end(&mut bytes).map_err(|e| e.to_string())?;
    if bytes.len() < 10 || &bytes[..6] != b"\x93NUMPY" {
        return Err("Некорректный NPY".into());
    }
    let (start, length) = match bytes[6] {
        1 => (10, u16::from_le_bytes([bytes[8], bytes[9]]) as usize),
        2 | 3 if bytes.len() >= 12 => (
            12,
            u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize,
        ),
        _ => return Err("Версия NPY не поддержана".into()),
    };
    let end = start + length;
    let header = std::str::from_utf8(bytes.get(start..end).ok_or("Нет заголовка NPY")?)
        .map_err(|e| e.to_string())?;
    if !header.contains("'<f4'") || !header.contains("'fortran_order': False") {
        return Err("Ожидался float32 C-order".into());
    }
    let dims = header
        .split("'shape':")
        .nth(1)
        .and_then(|s| s.split('(').nth(1))
        .and_then(|s| s.split(')').next())
        .ok_or("Нет shape")?
        .split(',')
        .filter_map(|s| {
            let s = s.trim();
            if s.is_empty() {
                None
            } else {
                Some(s.parse::<usize>())
            }
        })
        .collect::<Result<Vec<_>, _>>()
        .map_err(|e| e.to_string())?;
    if dims != shape {
        return Err(format!("Неверная форма {name}: {dims:?}"));
    }
    let data = bytes.get(end..).ok_or("Нет данных NPY")?;
    if data.len() != shape.iter().product::<usize>() * 4 {
        return Err("Неверная длина NPY".into());
    }
    let values = data
        .chunks_exact(4)
        .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
        .collect::<Vec<_>>();
    if values.iter().any(|v| !v.is_finite()) {
        return Err("Нечисловые веса".into());
    }
    Ok(values)
}
impl Network {
    pub fn load(crack: bool) -> Result<Self, String> {
        let bytes: &[u8] = if crack {
            include_bytes!("../../../assets/ndm/crack.npz")
        } else {
            include_bytes!("../../../assets/ndm/strength.npz")
        };
        let sizes = if crack {
            vec![22, 188, 188, 188, 9]
        } else {
            vec![17, 192, 192, 192, 2]
        };
        let mut zip = zip::ZipArchive::new(Cursor::new(bytes)).map_err(|e| e.to_string())?;
        let mean = array(
            &mut zip,
            if crack { "x_mean" } else { "xmean" },
            &[sizes[0]],
        )?;
        let std = array(&mut zip, if crack { "x_std" } else { "xstd" }, &[sizes[0]])?;
        if std.iter().any(|v| *v <= 0.) {
            return Err("Неверная нормировка модели".into());
        }
        let ymean = array(&mut zip, if crack { "y_mean" } else { "ymean" }, &[2])?;
        let ystd = array(&mut zip, if crack { "y_std" } else { "ystd" }, &[2])?;
        let mut weights = vec![];
        let mut biases = vec![];
        for i in 0..4 {
            weights.push(array(
                &mut zip,
                &format!("W{i}"),
                &[sizes[i], sizes[i + 1]],
            )?);
            biases.push(array(&mut zip, &format!("b{i}"), &[sizes[i + 1]])?);
        }
        Ok(Self {
            mean,
            std,
            ymean,
            ystd,
            weights,
            biases,
            sizes,
        })
    }
    pub fn forward(&self, features: &[f32]) -> Vec<f32> {
        let mut x = features
            .iter()
            .enumerate()
            .map(|(i, v)| (*v - self.mean[i]) / self.std[i])
            .collect::<Vec<_>>();
        for layer in 0..4 {
            let mut y = self.biases[layer].clone();
            for (&value, weights) in x
                .iter()
                .zip(self.weights[layer].chunks_exact(self.sizes[layer + 1]))
            {
                for (out, &weight) in y.iter_mut().zip(weights) {
                    *out += value * weight;
                }
            }
            if layer < 3 {
                for value in &mut y {
                    *value /= 1. + (-value.clamp(-40., 40.)).exp();
                }
            }
            x = y;
        }
        x
    }
    pub fn forward_batch<const N: usize>(&self, features: &[[f32; N]]) -> faer::Mat<f32> {
        use faer::{Accum, Mat, MatRef, Par};
        assert_eq!(N, self.sizes[0]);
        let n = features.len();
        if n == 0 {
            return Mat::zeros(0, self.sizes[4]);
        }
        let mut x = Mat::<f32>::from_fn(n, N, |r, c| (features[r][c] - self.mean[c]) / self.std[c]);
        for layer in 0..4 {
            let width = self.sizes[layer + 1];
            let w = MatRef::from_column_major_slice(&self.weights[layer], width, self.sizes[layer]);
            let mut y = Mat::<f32>::zeros(n, width);
            faer::linalg::matmul::matmul(
                y.as_mut(),
                Accum::Replace,
                x.as_ref(),
                w.transpose(),
                1.,
                Par::Seq,
            );
            for c in 0..width {
                for r in 0..n {
                    let mut v = y[(r, c)] + self.biases[layer][c];
                    if layer < 3 {
                        v /= 1. + (-v.clamp(-40., 40.)).exp();
                    }
                    y[(r, c)] = v;
                }
            }
            x = y;
        }
        x
    }

    /// All angles/variants of a physical FE share four matrix multiplications.
    pub fn strength_batch(&self, rows: &[[f64; 9]]) -> Vec<[f64; 2]> {
        let features = rows
            .iter()
            .map(|r| strength_features(*r))
            .collect::<Vec<_>>();
        let out = self.forward_batch(&features);
        (0..rows.len())
            .map(|r| {
                std::array::from_fn(|i| {
                    ((out[(r, i)] * self.ystd[i] + self.ymean[i])
                        .clamp(-10., 10.)
                        .exp() as f64)
                        .clamp(0.5_f64.max(0.001 * rows[r][4]), 200_f64.min(rows[r][4]))
                })
            })
            .collect()
    }
    pub fn strength(&self, row: [f64; 9]) -> [f64; 2] {
        let out = self.forward(&strength_features(row));
        std::array::from_fn(|i| {
            ((out[i] * self.ystd[i] + self.ymean[i])
                .clamp(-10., 10.)
                .exp() as f64)
                .clamp(0.5_f64.max(0.001 * row[4]), 200_f64.min(row[4]))
        })
    }
}
pub fn strength_features(x: [f64; 9]) -> [f32; 17] {
    let [n, m, rb, eb, h, ab, at, rs, rsc] = x;
    let mut values = [0.; 17];
    values[..9].copy_from_slice(&x);
    values[9..].copy_from_slice(&[
        n / (h * rb),
        1000. * m / (h * h * rb),
        ab / h,
        at / h,
        rb / eb,
        1. - (ab + at) / h,
        rs / rb,
        rsc / rb,
    ]);
    values.map(|v| v as f32)
}
pub fn crack_features(x: [f64; 11], rbser: f64, rbtser: f64, rsser: f64) -> [f32; 22] {
    let [n, m, rb, eb, h, ab, at, rs, rsc, asb, ast] = x;
    let nn = n / (h * rb);
    let mm = m * 1000. / (h * h * rb);
    [
        nn,
        mm,
        nn.abs(),
        mm.abs(),
        nn * mm,
        rb / 35.,
        eb / 40000.,
        h / 300.,
        ab / h,
        at / h,
        rs / 500.,
        rsc / 500.,
        asb / (h / 10.),
        ast / (h / 10.),
        asb / 40.,
        ast / 40.,
        200000. / eb,
        rbser / rb,
        rbtser / rb,
        rsser / 500.,
        nn / (1. + mm.abs()),
        mm / (1. + nn.abs()),
    ]
    .map(|v| v as f32)
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn batched_strength_matches_scalar_and_numpy_for_partial_and_empty_batches() {
        let model = Network::load(false).unwrap();
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../../assets/ndm/reference.json")).unwrap();
        let rows = reference["strength"]
            .as_array()
            .unwrap()
            .iter()
            .map(|v| std::array::from_fn(|i| v["raw"][i].as_f64().unwrap()))
            .collect::<Vec<[f64; 9]>>();
        assert!(model.strength_batch(&[]).is_empty());
        for n in [1, 7, 36, 72] {
            let inputs = (0..n).map(|i| rows[i % rows.len()]).collect::<Vec<_>>();
            let out = model.strength_batch(&inputs);
            for (r, p) in inputs.iter().zip(out) {
                let scalar = model.strength(*r);
                for i in 0..2 {
                    assert!(
                        (p[i] - scalar[i]).abs() <= 2e-4 * scalar[i].max(1.),
                        "{p:?} != {scalar:?}"
                    );
                }
            }
        }
    }
    #[test]
    fn embedded_models_match_independent_numpy_vectors() {
        let reference: serde_json::Value =
            serde_json::from_str(include_str!("../../../assets/ndm/reference.json")).unwrap();
        let model = Network::load(false).unwrap();
        for row in reference["strength"].as_array().unwrap() {
            let raw = std::array::from_fn(|i| row["raw"][i].as_f64().unwrap());
            let predicted = model.strength(raw);
            for i in 0..2 {
                let expected = row["areas"][i].as_f64().unwrap();
                assert!(
                    (predicted[i] - expected).abs() < expected.max(1.) * 0.0002,
                    "{predicted:?} vs {expected}"
                );
            }
        }
        let model = Network::load(true).unwrap();
        for row in reference["crack"].as_array().unwrap() {
            let features = row["features"]
                .as_array()
                .unwrap()
                .iter()
                .map(|v| v.as_f64().unwrap() as f32)
                .collect::<Vec<_>>();
            let raw = std::array::from_fn(|i| row["raw"][i].as_f64().unwrap());
            let concrete = super::super::CONCRETE
                .iter()
                .find(|r| (r.1 - raw[2]).abs() < 1e-8)
                .unwrap();
            let rsser = if raw[7] == 435. { 500. } else { 400. };
            let engineered = crack_features(raw, concrete.3, concrete.4, rsser);
            for i in 0..22 {
                assert!((engineered[i] - features[i]).abs() < 1e-5);
            }
            let result = model.forward(&features);
            for (i, value) in result.iter().enumerate() {
                assert!((*value as f64 - row["network"][i].as_f64().unwrap()).abs() < 0.0002);
            }
        }
    }
}
