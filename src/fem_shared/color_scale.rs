use eframe::egui;
#[derive(Clone, Copy, PartialEq)]
pub enum Palette {
    Spectrum,
    Diverging,
    Gray,
    Engineering,
    Contrast,
    Traffic = 6,
}
impl Palette {
    // Existing project IDs 0..2 stay unchanged; append new palettes only.
    pub const ALL: [Self; 6] = [
        Self::Engineering,
        Self::Traffic,
        Self::Contrast,
        Self::Spectrum,
        Self::Diverging,
        Self::Gray,
    ];
    pub fn id(self) -> u8 {
        self as u8
    }
    pub fn from_id(id: u8) -> Option<Self> {
        match id {
            0 => Some(Self::Spectrum),
            1 => Some(Self::Diverging),
            2 => Some(Self::Gray),
            3 => Some(Self::Engineering),
            4 => Some(Self::Contrast),
            5 => Some(Self::Engineering),
            6 => Some(Self::Traffic),
            _ => None,
        }
    }
    pub fn label(self) -> &'static str {
        match self {
            Self::Spectrum => "Спектр",
            Self::Diverging => "Синий — белый — красный",
            Self::Gray => "Оттенки серого",
            Self::Engineering => "Дискретная · Инженерная",
            Self::Contrast => "Дискретная · Контрастная",
            Self::Traffic => "Дискретная · Светофор",
        }
    }
    pub(crate) fn category_color(self, index: usize, count: usize) -> egui::Color32 {
        if let Some(colors) = self.discrete_colors() {
            let size = colors.len();
            let (entry, cycle) = if count <= 1 {
                (size / 2, 0)
            } else if count <= size {
                (index * (size - 1) / (count - 1), 0)
            } else {
                (index % size, index / size)
            };
            let rgb = colors[entry];
            if cycle == 0 {
                return egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2]);
            }
            let target = if cycle % 2 == 1 { 255. } else { 0. };
            let blend = 1. - 0.76_f64.powf(cycle.div_ceil(2) as f64);
            let [r, g, b] =
                rgb.map(|v| (f64::from(v) + (target - f64::from(v)) * blend).round() as u8);
            return egui::Color32::from_rgb(r, g, b);
        }
        let scale = ColorScale {
            palette: self,
            bands: 0,
            ..Default::default()
        };
        let position = if count > 1 {
            index as f64 / (count - 1) as f64
        } else {
            0.5
        };
        scale.color(position, [0., 1.])
    }
    fn discrete_colors(self) -> Option<&'static [[u8; 3]]> {
        match self {
            Self::Engineering => Some(&[
                [160, 160, 160],
                [255, 255, 0],
                [145, 255, 0],
                [0, 190, 0],
                [0, 230, 210],
                [0, 150, 255],
                [0, 55, 220],
                [120, 0, 220],
                [235, 0, 215],
                [255, 145, 0],
                [245, 55, 30],
                [155, 0, 0],
            ]),
            Self::Contrast => Some(&[
                [0, 90, 170],
                [240, 125, 0],
                [0, 135, 95],
                [200, 45, 90],
                [125, 70, 175],
                [190, 155, 0],
                [0, 170, 195],
                [135, 75, 35],
                [225, 110, 175],
                [75, 75, 75],
                [125, 175, 50],
                [70, 120, 130],
            ]),
            Self::Traffic => Some(&[
                [40, 145, 75],
                [125, 200, 90],
                [195, 230, 110],
                [255, 235, 80],
                [255, 180, 50],
                [250, 120, 45],
                [225, 65, 45],
                [165, 25, 45],
                [120, 20, 70],
            ]),
            _ => None,
        }
    }
}
#[derive(Clone, Copy, Debug)]
pub struct Breaks {
    pub edges: [f64; 33],
    pub len: usize,
}
impl Breaks {
    pub fn parse(text: &str) -> Result<Self, String> {
        let values: Vec<f64> = text
            .split(|c: char| c == ';' || c.is_whitespace())
            .filter(|s| !s.is_empty())
            .map(|s| {
                s.replace(',', ".")
                    .parse::<f64>()
                    .map_err(|_| format!("Не число: {s}"))
            })
            .collect::<Result<_, _>>()?;
        if !(2..=33).contains(&values.len()) {
            return Err("Нужно от 2 до 33 границ (до 32 интервалов)".into());
        }
        let mut r = Self {
            edges: [0.; 33],
            len: values.len(),
        };
        r.edges[..r.len].copy_from_slice(&values);
        if !r.valid() {
            return Err("Границы должны быть конечными и не убывать".into());
        }
        Ok(r)
    }
    pub fn uniform(range: [f64; 2], n: usize) -> Self {
        let mut r = Self {
            edges: [0.; 33],
            len: n + 1,
        };
        for i in 0..=n {
            r.edges[i] = range[0] + (range[1] - range[0]) * i as f64 / n as f64;
        }
        r
    }
    pub fn valid(self) -> bool {
        (2..=33).contains(&self.len)
            && self.edges[..self.len].iter().all(|x| x.is_finite())
            && self.edges[..self.len].windows(2).all(|p| p[0] <= p[1])
            && self.edges[0] < self.edges[self.len - 1]
    }
    pub fn band(self, value: f64) -> usize {
        self.edges[1..self.len - 1].partition_point(|&edge| value > edge)
    }
}
#[derive(Clone, Copy)]
pub struct ColorScale {
    pub range: Option<[f64; 2]>,
    pub bands: usize,
    pub palette: Palette,
    pub breaks: Option<Breaks>,
    pub colors: [Option<egui::Color32>; 32],
}
impl Default for ColorScale {
    fn default() -> Self {
        Self {
            range: None,
            bands: 8,
            palette: Palette::Spectrum,
            breaks: None,
            colors: [None; 32],
        }
    }
}
impl ColorScale {
    pub fn valid(self) -> bool {
        self.breaks.is_none_or(Breaks::valid)
            && self
                .range
                .is_none_or(|r| r[0].is_finite() && r[1].is_finite() && r[0] < r[1])
    }
    pub fn bounds(self, auto: [f64; 2]) -> [f64; 2] {
        if self.valid() {
            self.breaks
                .map(|b| [b.edges[0], b.edges[b.len - 1]])
                .unwrap_or_else(|| self.range.unwrap_or(auto))
        } else {
            auto
        }
    }
    fn normalized(self, value: f64, auto: [f64; 2]) -> f64 {
        let [min, max] = self.bounds(auto);
        if max > min {
            ((value - min) / (max - min)).clamp(0., 1.)
        } else {
            0.5
        }
    }
    pub fn band(self, value: f64, auto: [f64; 2]) -> Option<usize> {
        if let Some(b) = self.breaks {
            return Some(b.band(value));
        }
        if self.bands < 2 {
            None
        } else if let Some(b) = self.intervals(auto) {
            Some(b.band(value))
        } else {
            Some(self.bands / 2)
        }
    }
    pub fn interval_count(self) -> usize {
        self.breaks.map(|b| b.len - 1).unwrap_or(self.bands)
    }
    pub fn intervals(self, auto: [f64; 2]) -> Option<Breaks> {
        if let Some(b) = self.breaks {
            Some(b)
        } else if self.bands >= 2 && self.bounds(auto)[0] < self.bounds(auto)[1] {
            Some(Breaks::uniform(self.bounds(auto), self.bands))
        } else {
            None
        }
    }
    pub fn color(self, value: f64, auto: [f64; 2]) -> egui::Color32 {
        if let Some(color) = self
            .band(value, auto)
            .and_then(|i| self.colors.get(i))
            .copied()
            .flatten()
        {
            return color;
        }
        let t = self
            .band(value, auto)
            .map(|i| {
                if self.interval_count() > 1 {
                    i as f64 / (self.interval_count() - 1) as f64
                } else {
                    0.5
                }
            })
            .unwrap_or_else(|| self.normalized(value, auto));
        if let Some(colors) = self.palette.discrete_colors() {
            let n = self.interval_count().max(1);
            let band = self
                .band(value, auto)
                .unwrap_or_else(|| (t * (n - 1) as f64).round() as usize)
                .min(n - 1);
            let i = if n == 1 {
                colors.len() / 2
            } else if n <= colors.len() {
                band * (colors.len() - 1) / (n - 1)
            } else {
                band % colors.len()
            };
            let [r, g, b] = colors[i];
            return egui::Color32::from_rgb(r, g, b);
        }
        let colors: &[[f64; 3]] = match self.palette {
            Palette::Spectrum => &[
                [45., 76., 190.],
                [38., 180., 213.],
                [91., 202., 128.],
                [249., 210., 72.],
                [227., 71., 57.],
            ],
            Palette::Diverging => &[[45., 76., 190.], [245., 245., 245.], [227., 71., 57.]],
            Palette::Gray => &[[40., 40., 40.], [245., 245., 245.]],
            _ => unreachable!("discrete palettes handled above"),
        };
        let x = t * (colors.len() - 1) as f64;
        let i = (x.floor() as usize).min(colors.len() - 2);
        let f = x - i as f64;
        let rgb: [u8; 3] =
            std::array::from_fn(|k| (colors[i][k] * (1. - f) + colors[i + 1][k] * f).round() as u8);
        egui::Color32::from_rgb(rgb[0], rgb[1], rgb[2])
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn discrete_palettes_use_only_fixed_swatches_and_keep_project_ids_stable() {
        assert!(Palette::from_id(0) == Some(Palette::Spectrum));
        assert!(Palette::from_id(1) == Some(Palette::Diverging));
        assert!(Palette::from_id(2) == Some(Palette::Gray));
        assert!(Palette::from_id(5) == Some(Palette::Engineering));
        assert!(Palette::ALL[0] == Palette::Engineering);
        assert!(Palette::ALL[1] == Palette::Traffic);
        for palette in [Palette::Engineering, Palette::Contrast, Palette::Traffic] {
            assert!(Palette::from_id(palette.id()) == Some(palette));
            let swatches = palette.discrete_colors().unwrap();
            for n in [1, 5, 12, 32] {
                let scale = ColorScale {
                    bands: n,
                    palette,
                    ..Default::default()
                };
                for i in 0..n {
                    let color = scale.color((i as f64 + 0.5) / n as f64, [0., 1.]);
                    assert!(swatches.contains(&[color.r(), color.g(), color.b()]));
                }
            }
            let mut custom = ColorScale {
                bands: 5,
                palette,
                ..Default::default()
            };
            custom.colors[0] = Some(egui::Color32::BLACK);
            assert_eq!(custom.color(-1., [0., 1.]), egui::Color32::BLACK);
        }
    }
    #[test]
    fn custom_colors_follow_intervals_including_clamped_values() {
        let mut scale = ColorScale {
            breaks: Some(Breaks::parse("0; 3; 10").unwrap()),
            ..Default::default()
        };
        let color = egui::Color32::from_rgb(123, 45, 67);
        scale.colors[1] = Some(color);
        assert_eq!(scale.color(3.01, [0., 10.]), color);
        assert_eq!(scale.color(100., [0., 10.]), color);
        assert_ne!(scale.color(2., [0., 10.]), color);
        assert_eq!(scale.band(3., [0., 10.]), Some(0));
        scale.colors = [None; 32];
        assert_ne!(scale.color(3.01, [0., 10.]), color);
    }
    #[test]
    fn editable_nonuniform_boundaries_and_validation() {
        let b = Breaks::parse("0; 3,5; 7; 20").unwrap();
        let s = ColorScale {
            breaks: Some(b),
            ..Default::default()
        };
        assert_eq!(s.bounds([-100., 100.]), [0., 20.]);
        for (v, band) in [
            (-1., 0),
            (0., 0),
            (3.49, 0),
            (3.5, 0),
            (6.9, 1),
            (7., 1),
            (20., 2),
            (100., 2),
        ] {
            assert_eq!(s.band(v, [0., 1.]), Some(band));
            assert_eq!(b.band(v), band);
        }
        for bad in ["", "0", "5;1", "0;NaN", "0;Infinity"] {
            assert!(Breaks::parse(bad).is_err());
        }
        assert_eq!(s.color(0., [0., 1.]), s.color(3., [0., 1.]));
        assert_ne!(s.color(3., [0., 1.]), s.color(3.51, [0., 1.]));
    }
    #[test]
    fn endpoints_clamping_bands_and_ranges() {
        let s = ColorScale {
            range: Some([-10., 10.]),
            bands: 4,
            palette: Palette::Gray,
            breaks: None,
            colors: [None; 32],
        };
        assert_eq!(s.band(-100., [0., 1.]), Some(0));
        assert_eq!(s.band(-5., [0., 1.]), Some(0));
        assert_eq!(s.band(100., [0., 1.]), Some(3));
        assert_eq!(s.color(-100., [0., 1.]), s.color(-10., [0., 1.]));
        assert_eq!(s.color(100., [0., 1.]), s.color(10., [0., 1.]));
        assert_eq!(s.color(-9., [0., 1.]), s.color(-6., [0., 1.]));
        assert_ne!(s.color(-9., [0., 1.]), s.color(0., [0., 1.]));
        assert_eq!(s.bounds([0., 1.]), [-10., 10.]);
        let invalid = ColorScale {
            range: Some([5., 1.]),
            ..s
        };
        assert!(!invalid.valid());
        assert_eq!(invalid.bounds([0., 3.]), [0., 3.]);
        assert_eq!(ColorScale::default().color(3., [3., 3.]).a(), 255);
    }
}
