//! Result presentation only; shares palettes, picker and portable presets with Grid.
use super::{
    color_scale::{Breaks, ColorScale, Palette},
    geometry::Point,
};
#[cfg(target_arch = "wasm32")]
use crate::plan_trace::platform::dialog as rfd;
use eframe::egui::{self, Color32};
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct Row {
    upper: String,
    color: [u8; 3],
    enabled: bool,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub(super) struct State {
    rows: Vec<Row>,
    #[serde(default)]
    discrete: Option<Vec<f64>>,
    palette: u8,
    active: Option<Applied>,
    enabled: Vec<bool>,
    smooth: bool,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
struct Applied {
    #[serde(default)]
    labels: Vec<String>,
    range: Option<[f64; 2]>,
    bands: usize,
    palette: u8,
    breaks: Option<Vec<f64>>,
    colors: Vec<Option<[u8; 4]>>,
}
#[derive(Clone, serde::Serialize, serde::Deserialize)]
pub(super) struct Bounds {
    #[serde(default)]
    pub discrete: Option<Vec<f64>>,
    pub labels: Vec<String>,
    pub edges: Vec<f64>,
}
#[derive(Clone, Copy)]
pub(super) enum Action {
    Apply,
    Reset,
}
pub struct Scale {
    rows: Vec<Row>,
    discrete: Option<Vec<f64>>,
    palette: Palette,
    active: Option<ColorScale>,
    enabled: Vec<bool>,
    pub smooth: bool,
    error: String,
    reinforcement: bool,
    action: Option<Action>,
    committed_labels: Vec<String>,
}
impl Default for Scale {
    fn default() -> Self {
        Self {
            rows: vec![],
            discrete: None,
            palette: Palette::Engineering,
            active: None,
            enabled: vec![true; 8],
            smooth: true,
            error: String::new(),
            reinforcement: false,
            action: None,
            committed_labels: Vec::new(),
        }
    }
}
pub fn range(values: impl Iterator<Item = f64>) -> [f64; 2] {
    let mut r = [f64::INFINITY, f64::NEG_INFINITY];
    for v in values.filter(|v| v.is_finite()) {
        r[0] = r[0].min(v);
        r[1] = r[1].max(v);
    }
    if !r[0].is_finite() {
        return [0., 1.];
    }
    let magnitude = r[0].abs().max(r[1].abs());
    if r[1] - r[0] <= 64. * f64::EPSILON * magnitude {
        let d = r[0].abs().max(1.) * 0.01;
        r = [r[0] - d, r[1] + d];
    }
    r
}
impl Scale {
    pub(super) fn take_action(&mut self) -> Option<Action> {
        self.action.take()
    }
    pub(super) fn bounds(&self) -> Option<Bounds> {
        if let Some(values) = &self.discrete {
            return Some(Bounds {
                discrete: Some(values.clone()),
                labels: self.rows.iter().map(|r| r.upper.clone()).collect(),
                edges: vec![],
            });
        }
        let b = self.active?.breaks?;
        Some(Bounds {
            discrete: None,
            labels: if self.committed_labels.is_empty() {
                self.rows.iter().map(|r| r.upper.clone()).collect()
            } else {
                self.committed_labels.clone()
            },
            edges: b.edges[..b.len].to_vec(),
        })
    }
    pub(super) fn restore_bounds(&mut self, b: &Bounds) -> Result<(), String> {
        if let Some(values) = &b.discrete {
            if values.len() > 100000
                || values.len() != b.labels.len()
                || values.iter().any(|v| !v.is_finite())
                || values.windows(2).any(|p| p[0] >= p[1])
            {
                return Err("Некорректная шкала нагрузок".into());
            }
            self.sync_discrete_values(values.clone());
            return Ok(());
        }
        let n = b.labels.len();
        if !(1..=32).contains(&n)
            || b.edges.len() != n + 1
            || b.labels.iter().any(|s| s.len() > 1024)
            || b.edges.iter().any(|v| !v.is_finite())
            || b.edges.windows(2).any(|p| p[0] >= p[1])
        {
            return Err("Некорректные сохранённые границы шкалы".into());
        }
        self.discrete = None;
        let palette = ColorScale {
            palette: self.palette,
            bands: n,
            ..Default::default()
        };
        self.rows.resize_with(n, || Row {
            upper: String::new(),
            color: [0; 3],
            enabled: true,
        });
        for (i, row) in self.rows.iter_mut().enumerate() {
            if row.color == [0; 3] {
                let c = palette.color((i as f64 + 0.5) / n as f64, [0., 1.]);
                row.color = [c.r(), c.g(), c.b()];
            }
            row.upper = b.labels[i].clone();
        }
        let mut breaks = Breaks {
            edges: [0.; 33],
            len: b.edges.len(),
        };
        breaks.edges[..breaks.len].copy_from_slice(&b.edges);
        let mut active = ColorScale {
            palette: self.palette,
            bands: n,
            breaks: Some(breaks),
            ..Default::default()
        };
        for (i, row) in self.rows.iter().enumerate() {
            active.colors[i] = Some(Color32::from_rgb(row.color[0], row.color[1], row.color[2]));
        }
        self.committed_labels = b.labels.clone();
        self.active = Some(active);
        self.enabled = self.rows.iter().map(|r| r.enabled).collect();
        Ok(())
    }

    pub fn set_reinforcement_mode(&mut self, enabled: bool) {
        self.reinforcement = enabled;
    }
    fn set_band_visibility(&mut self, index: usize, enabled: bool) {
        if let Some(row) = self.rows.get_mut(index) {
            row.enabled = enabled;
        }
        // Visibility is applied immediately, without parsing draft boundaries.
        // Newly added draft rows only acquire an applied band after Apply.
        if self.discrete.is_none() && index < self.engine().interval_count() {
            self.enabled.resize(self.engine().interval_count(), true);
            self.enabled[index] = enabled;
        }
    }

    fn restore_palette_colors(&mut self) {
        let n = self.rows.len();
        if n == 0 {
            return;
        }
        let scale = ColorScale {
            palette: self.palette,
            bands: n,
            ..Default::default()
        };
        for (i, row) in self.rows.iter_mut().enumerate() {
            let c = if self.discrete.is_some() {
                self.palette.category_color(i, n)
            } else {
                scale.color((i as f64 + 0.5) / n as f64, [0., 1.])
            };
            row.color = [c.r(), c.g(), c.b()];
        }
        // Update applied colors without applying any unfinished boundary edits.
        if let Some(active) = &mut self.active {
            active.palette = self.palette;
            let count = active.interval_count();
            for i in 0..count {
                let palette = ColorScale {
                    palette: self.palette,
                    bands: count,
                    ..Default::default()
                };
                active.colors[i] = Some(palette.color((i as f64 + 0.5) / count as f64, [0., 1.]));
            }
        }
    }
    pub fn rescale_values(&mut self, factor: f64) {
        if !factor.is_finite() || factor <= 0. || factor == 1. {
            return;
        }
        if let Some(values) = &mut self.discrete {
            for value in values {
                *value *= factor;
            }
        }
        for row in &mut self.rows {
            if let Ok(value) = row.upper.trim().replace(',', ".").parse::<f64>() {
                row.upper = (value * factor).to_string();
            }
        }
        if let Some(e) = &mut self.active {
            if let Some(range) = &mut e.range {
                for v in range {
                    *v *= factor;
                }
            }
            if let Some(b) = &mut e.breaks {
                for v in &mut b.edges[..b.len] {
                    *v *= factor;
                }
            }
        }
    }
    pub(super) fn state(&self) -> State {
        State {
            rows: self.rows.clone(),
            discrete: self.discrete.clone(),
            palette: self.palette.id(),
            enabled: self.enabled.clone(),
            smooth: self.smooth,
            active: self.active.map(|e| Applied {
                labels: self.committed_labels.clone(),
                range: e.range,
                bands: e.bands,
                palette: e.palette.id(),
                breaks: e.breaks.map(|b| b.edges[..b.len].to_vec()),
                colors: e.colors.map(|c| c.map(|c| c.to_array())).to_vec(),
            }),
        }
    }
    pub(super) fn from_state(s: State) -> Result<Self, String> {
        let palette = Palette::from_id(s.palette).ok_or("Неизвестная палитра")?;
        if let Some(values) = &s.discrete {
            if values.len() > 100_000
                || values.len() != s.rows.len()
                || s.rows.iter().any(|row| row.upper.len() > 1024)
                || values.iter().any(|v| !v.is_finite())
                || values.windows(2).any(|p| p[0] >= p[1])
            {
                return Err("Некорректная шкала нагрузок".into());
            }
            return Ok(Self {
                rows: s.rows,
                discrete: s.discrete,
                palette,
                active: None,
                enabled: vec![],
                smooth: false,
                error: String::new(),
                reinforcement: false,
                action: None,
                committed_labels: Vec::new(),
            });
        }
        if s.rows.len() > 32 || s.rows.iter().any(|r| r.upper.len() > 1024) || s.enabled.len() > 32
        {
            return Err("Некорректная шкала проекта".into());
        }
        let committed_labels = s
            .active
            .as_ref()
            .map(|a| a.labels.clone())
            .filter(|v| !v.is_empty())
            .unwrap_or_else(|| s.rows.iter().map(|r| r.upper.clone()).collect());
        if committed_labels.len() > 32 || committed_labels.iter().any(|s| s.len() > 1024) {
            return Err("Некорректные подписи шкалы".into());
        }
        let active = if let Some(a) = s.active {
            if !(1..=32).contains(&a.bands) || a.colors.len() != 32 {
                return Err("Некорректные цвета шкалы".into());
            }
            let breaks = if let Some(values) = a.breaks {
                if !(2..=33).contains(&values.len()) {
                    return Err("Некорректные границы шкалы".into());
                }
                let mut b = Breaks {
                    edges: [0.; 33],
                    len: values.len(),
                };
                b.edges[..b.len].copy_from_slice(&values);
                Some(b)
            } else {
                None
            };
            let e = ColorScale {
                range: a.range,
                bands: a.bands,
                palette: Palette::from_id(a.palette).ok_or("Неизвестная палитра")?,
                breaks,
                colors: std::array::from_fn(|i| {
                    a.colors[i].map(|c| Color32::from_rgba_premultiplied(c[0], c[1], c[2], c[3]))
                }),
            };
            if !e.valid()
                || e.intervals([0., 1.]).is_none()
                || s.enabled.len() != e.interval_count()
            {
                return Err("Некорректная примененная шкала".into());
            }
            Some(e)
        } else {
            if s.enabled.len() != 8 {
                return Err("Некорректная автоматическая шкала".into());
            }
            None
        };
        Ok(Self {
            committed_labels,
            rows: s.rows,
            discrete: None,
            palette,
            active,
            enabled: s.enabled,
            smooth: s.smooth,
            error: String::new(),
            reinforcement: false,
            action: None,
        })
    }
    pub fn signature(&self) -> Vec<u64> {
        if let Some(values) = &self.discrete {
            let mut key = vec![u64::MAX, values.len() as u64];
            key.extend(values.iter().map(|v| v.to_bits()));
            for row in &self.rows {
                key.push(u32::from_le_bytes([
                    row.color[0],
                    row.color[1],
                    row.color[2],
                    row.enabled as u8,
                ]) as u64);
            }
            return key;
        }
        let e = self.engine();
        let mut key = vec![self.smooth as u64, e.palette.id() as u64, e.bands as u64];
        if let Some(b) = e.breaks {
            key.push(b.len as u64);
            key.extend(b.edges[..b.len].iter().map(|v| v.to_bits()));
        } else {
            key.push(0);
        }
        if let Some(r) = e.range {
            key.push(1);
            key.extend(r.map(f64::to_bits));
        } else {
            key.push(0);
        }
        for c in e.colors {
            key.push(
                c.map(|c| u32::from_le_bytes(c.to_array()) as u64 + 1)
                    .unwrap_or(0),
            );
        }
        key.extend(self.enabled.iter().map(|v| *v as u64));
        key
    }
    fn legend_bounds(rect: egui::Rect, size: f32) -> egui::Rect {
        egui::Rect::from_min_size(
            rect.left_top() + egui::vec2(8., 8.),
            egui::vec2((rect.width() - 16.).max(1.), size + 36.),
        )
    }
    pub(super) fn legend_band_at(
        &self,
        rect: egui::Rect,
        pointer: egui::Pos2,
        auto: [f64; 2],
        size: f32,
    ) -> Option<usize> {
        let bounds = Self::legend_bounds(rect, size);
        let width = bounds.width() - 8.;
        let x = pointer.x - bounds.left() - 4.;
        let y = pointer.y - bounds.top();
        if x < 0. || x >= width || y < size + 8. || y > size + 23. {
            return None;
        }
        let count = match &self.discrete {
            Some(values) => values.len(),
            None => self.engine().intervals(auto)?.len - 1,
        };
        (count > 0).then(|| ((x / width * count as f32) as usize).min(count - 1))
    }
    pub(super) fn band_enabled(&self, band: usize) -> bool {
        if self.discrete.is_some() {
            self.rows.get(band).is_some_and(|r| r.enabled)
        } else {
            self.enabled.get(band).copied().unwrap_or(true)
        }
    }
    pub(super) fn value_band(&self, value: f64, auto: [f64; 2]) -> Option<usize> {
        if !value.is_finite() {
            return None;
        }
        if let Some(values) = &self.discrete {
            let last = values.len().checked_sub(1)?;
            let tolerance = value.abs().max(1.) * 1e-10;
            Some(values.partition_point(|v| *v < value - tolerance).min(last))
        } else {
            Some(self.engine().intervals(auto)?.band(value))
        }
    }
    pub fn legend(
        &self,
        painter: &egui::Painter,
        rect: egui::Rect,
        auto: [f64; 2],
        style: super::app::drawing_flags::TextStyle,
    ) {
        let edges = if let Some(values) = &self.discrete {
            let Some(first) = values.first() else {
                return;
            };
            std::iter::once(*first)
                .chain(values.iter().copied())
                .collect::<Vec<_>>()
        } else {
            let b = self.engine().intervals(auto).unwrap();
            b.edges[..b.len].to_vec()
        };
        let n = edges.len() - 1;
        let bounds = Self::legend_bounds(rect, style.size);
        painter.rect_filled(bounds, 3., Color32::from_rgb(20, 24, 30));
        let width = (bounds.width() - 8.) / n as f32;
        for i in 0..n {
            let x = bounds.left() + 4. + i as f32 * width;
            let area = egui::Rect::from_min_size(
                egui::pos2(x, bounds.top() + style.size + 8.),
                egui::vec2(width - 1., 15.),
            );
            let e = (edges[i + 1] - edges[i]).abs() * 1e-9;
            let mid = (edges[i] + edges[i + 1]) * 0.5;
            if let Some(color) = self.color(mid, auto) {
                let a = if self.discrete.is_some() {
                    color
                } else {
                    self.color(edges[i] + e, auto).unwrap_or(color)
                };
                let z = if self.discrete.is_some() {
                    color
                } else {
                    self.color(edges[i + 1] - e, auto).unwrap_or(color)
                };
                let mut mesh = egui::Mesh::default();
                for (p, c) in [
                    (area.left_top(), a),
                    (area.right_top(), z),
                    (area.right_bottom(), z),
                    (area.left_bottom(), a),
                ] {
                    mesh.colored_vertex(p, c);
                }
                mesh.add_triangle(0, 1, 2);
                mesh.add_triangle(0, 2, 3);
                painter.add(egui::Shape::mesh(mesh));
            } else {
                painter.rect_filled(area, 0., Color32::from_gray(45));
            }
        }
        for i in 0..=n {
            if self.discrete.is_some() && i == 0 {
                continue;
            }
            if width >= 35. || i == 0 || i == n {
                let x = bounds.left() + 4. + i as f32 * width;
                let align = if i == 0 {
                    egui::Align2::LEFT_CENTER
                } else if i == n {
                    egui::Align2::RIGHT_CENTER
                } else {
                    egui::Align2::CENTER_CENTER
                };
                painter.text(
                    egui::pos2(x, bounds.top() + style.size * 0.5 + 4.),
                    align,
                    if self.reinforcement && i > 0 {
                        self.committed_labels
                            .get(i - 1)
                            .filter(|s| !s.eq_ignore_ascii_case("max"))
                            .map(|s| style.numeric_text(s))
                            .unwrap_or_else(|| style.number(edges[i]))
                    } else {
                        style.number(edges[i])
                    },
                    egui::FontId::proportional(style.size),
                    style.ink(),
                );
            }
        }
    }
    /// One category per actual displayed value; no fixed 32-band limit.
    pub(super) fn sync_discrete_values(&mut self, mut values: Vec<f64>) -> bool {
        values.retain(|v| v.is_finite());
        values.sort_by(f64::total_cmp);
        values.dedup_by(|a, b| (*a - *b).abs() <= a.abs().max(b.abs()).max(1.) * 1e-10);
        if self.discrete.as_ref() == Some(&values) {
            return false;
        }
        let old = self
            .discrete
            .as_ref()
            .map(|v| {
                v.iter()
                    .copied()
                    .zip(self.rows.iter().cloned())
                    .collect::<Vec<_>>()
            })
            .unwrap_or_default();
        let n = values.len();
        self.rows = values
            .iter()
            .enumerate()
            .map(|(i, &value)| {
                let tolerance = value.abs().max(1.) * 1e-10;
                let old_index = old.partition_point(|(v, _)| *v < value - tolerance);
                if let Some((old_value, row)) = old.get(old_index) {
                    if (*old_value - value).abs() <= tolerance {
                        let mut row = row.clone();
                        row.upper = value.to_string();
                        return row;
                    }
                }
                let color = self.palette.category_color(i, n);
                Row {
                    upper: value.to_string(),
                    color: [color.r(), color.g(), color.b()],
                    enabled: true,
                }
            })
            .collect();
        self.discrete = Some(values);
        self.active = None;
        self.enabled.clear();
        self.smooth = false;
        self.error.clear();
        true
    }
    fn discrete_ui(&mut self, ui: &mut egui::Ui, style: super::app::drawing_flags::TextStyle) {
        ui.label(format!("Значений: {}", self.rows.len()));
        egui::ScrollArea::vertical()
            .id_salt("load_scale_values")
            .max_height(300.)
            .show(ui, |ui| {
                egui::Grid::new("load_scale_rows")
                    .striped(true)
                    .show(ui, |ui| {
                        for text in ["Вкл.", "Цвет", "Значение"] {
                            ui.label(text);
                        }
                        ui.end_row();
                        for (i, row) in self.rows.iter_mut().enumerate() {
                            ui.checkbox(&mut row.enabled, "");
                            ui.push_id(i, |ui| {
                                super::color_picker::button(ui, &mut row.color);
                            });
                            ui.label(style.numeric_text(&row.upper))
                                .on_hover_text(&row.upper);
                            ui.end_row();
                        }
                    });
            });
        ui.horizontal(|ui| {
            if ui.button("Применить").clicked() {
                self.action = Some(Action::Apply);
            }
            if ui.button("Сбросить").clicked() {
                self.action = Some(Action::Reset);
            }
        });
        let previous_palette = self.palette;
        egui::ComboBox::from_id_salt("load_scale_palette")
            .selected_text(self.palette.label())
            .show_ui(ui, |ui| {
                for palette in Palette::ALL {
                    ui.selectable_value(&mut self.palette, palette, palette.label());
                }
            });
        if previous_palette != self.palette || ui.button("Восстановить цвета палитры").clicked()
        {
            self.restore_palette_colors();
        }
        ui.horizontal(|ui| {
            if ui.button("Сохранить…").clicked() {
                let result = (|| -> Result<(), String> {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Шкала нагрузок", &["json"])
                        .set_file_name("Шкала нагрузок.json")
                        .save_file()
                    {
                        crate::plan_trace::platform::write(
                            path,
                            serde_json::to_vec_pretty(&self.state()).map_err(|e| e.to_string())?,
                        )
                        .map_err(|e| e.to_string())?;
                    }
                    Ok(())
                })();
                self.error = result.err().unwrap_or_default();
            }
            if ui.button("Открыть…").clicked() {
                let result = (|| -> Result<(), String> {
                    let Some(path) = rfd::FileDialog::new()
                        .add_filter("Шкала нагрузок", &["json"])
                        .pick_file()
                    else {
                        return Ok(());
                    };
                    let state: State = serde_json::from_slice(
                        &crate::plan_trace::platform::read(path).map_err(|e| e.to_string())?,
                    )
                    .map_err(|e| e.to_string())?;
                    let imported = Self::from_state(state)?;
                    let Some(values) = imported.discrete else {
                        return Err("Нужна шкала значений нагрузок".into());
                    };
                    for row in &mut self.rows {
                        if let Some(i) = values.iter().position(|v| v.to_string() == row.upper) {
                            row.color = imported.rows[i].color;
                            row.enabled = imported.rows[i].enabled;
                        }
                    }
                    self.palette = imported.palette;
                    Ok(())
                })();
                self.error = result.err().unwrap_or_default();
            }
        });
        if !self.error.is_empty() {
            ui.colored_label(Color32::from_rgb(200, 130, 240), &self.error);
        }
    }
    fn engine(&self) -> ColorScale {
        self.active.unwrap_or(ColorScale {
            palette: self.palette,
            ..Default::default()
        })
    }
    fn initialise(&mut self, auto: [f64; 2]) {
        if !self.rows.is_empty() {
            return;
        }
        let e = self.engine();
        let b = e.intervals(auto).unwrap();
        self.rows = (0..e.interval_count())
            .map(|i| {
                let c = e.color((b.edges[i] + b.edges[i + 1]) * 0.5, auto);
                Row {
                    upper: if i + 1 == e.interval_count() {
                        "max".into()
                    } else {
                        format!("{:.6}", b.edges[i + 1])
                    },
                    color: [c.r(), c.g(), c.b()],
                    enabled: true,
                }
            })
            .collect();
    }
    pub(super) fn reset_values(&mut self, auto: [f64; 2]) -> Result<(), String> {
        self.initialise(auto);
        let n = self.rows.len();
        for (i, row) in self.rows.iter_mut().enumerate() {
            row.upper = if i + 1 == n {
                "max".into()
            } else {
                (auto[0] + (auto[1] - auto[0]) * (i + 1) as f64 / n as f64).to_string()
            };
        }
        self.apply(auto)
    }
    fn apply(&mut self, auto: [f64; 2]) -> Result<(), String> {
        let n = self.rows.len();
        if !(1..=32).contains(&n) {
            return Err("Допустимо от 1 до 32 диапазонов".into());
        }
        let mut b = Breaks {
            edges: [0.; 33],
            len: n + 1,
        };
        for (i, row) in self.rows.iter().enumerate() {
            let v = if i + 1 == n && row.upper.trim().eq_ignore_ascii_case("max") {
                auto[1].max(if i > 0 {
                    b.edges[i] + b.edges[i].abs().max(1.) * 1e-9
                } else {
                    auto[0] + 1e-9
                })
            } else {
                super::scale_input::value(&row.upper, self.reinforcement)
                    .map_err(|error| format!("Диапазон {}: {error}", i + 1))?
            };
            if !v.is_finite() || (i > 0 && v <= b.edges[i]) {
                return Err("Границы должны быть конечными и строго возрастать".into());
            }
            b.edges[i + 1] = v;
        }
        b.edges[0] = auto[0].min(b.edges[1] - b.edges[1].abs().max(1.) * 1e-9);
        let mut e = ColorScale {
            palette: self.palette,
            bands: n,
            breaks: Some(b),
            ..Default::default()
        };
        for (i, row) in self.rows.iter().enumerate() {
            e.colors[i] = Some(Color32::from_rgb(row.color[0], row.color[1], row.color[2]));
        }
        self.committed_labels = self.rows.iter().map(|r| r.upper.clone()).collect();
        self.active = Some(e);
        self.enabled = self.rows.iter().map(|r| r.enabled).collect();
        Ok(())
    }
    pub fn color(&self, value: f64, auto: [f64; 2]) -> Option<Color32> {
        if !value.is_finite() {
            return None;
        }
        if self.discrete.is_some() {
            let i = self.value_band(value, auto)?;
            let row = self.rows.get(i)?;
            return row
                .enabled
                .then(|| Color32::from_rgb(row.color[0], row.color[1], row.color[2]));
        }
        let e = self.engine();
        let b = e.intervals(auto)?;
        let i = b.band(value);
        if !self.enabled.get(i).copied().unwrap_or(true) {
            return None;
        }
        let c = e.color((b.edges[i] + b.edges[i + 1]) * 0.5, auto);
        if !self.smooth || i == 0 {
            return Some(c);
        }
        let a = e.color((b.edges[i - 1] + b.edges[i]) * 0.5, auto);
        let t = ((value - b.edges[i]) / (b.edges[i + 1] - b.edges[i]).max(1e-30)).clamp(0., 1.);
        Some(Color32::from_rgb(
            (a.r() as f64 * (1. - t) + c.r() as f64 * t).round() as u8,
            (a.g() as f64 * (1. - t) + c.g() as f64 * t).round() as u8,
            (a.b() as f64 * (1. - t) + c.b() as f64 * t).round() as u8,
        ))
    }
    /// Clip the linear scalar field to bands, so disabled intervals really disappear.
    pub fn triangles(
        &self,
        xy: [Point; 3],
        v: [f64; 3],
        auto: [f64; 2],
    ) -> Vec<[(Point, Color32); 3]> {
        if v.iter().any(|value| !value.is_finite()) {
            return vec![];
        }
        if v.iter().all(|x| *x == v[0]) {
            return self
                .color(v[0], auto)
                .map(|c| vec![xy.map(|p| (p, c))])
                .unwrap_or_default();
        }
        let b = self.engine().intervals(auto).unwrap();
        let mut out = vec![];
        let lo = v.into_iter().fold(f64::INFINITY, f64::min);
        let hi = v.into_iter().fold(f64::NEG_INFINITY, f64::max);
        for i in 0..b.len - 1 {
            if !self.enabled.get(i).copied().unwrap_or(true) {
                continue;
            }
            if (i > 0 && hi <= b.edges[i]) || (i + 2 < b.len && lo >= b.edges[i + 1]) {
                continue;
            }
            let mut p = xy.into_iter().zip(v).collect::<Vec<_>>();
            if i > 0 {
                p = clip(p, b.edges[i], true);
            }
            if i + 2 < b.len {
                p = clip(p, b.edges[i + 1], false);
            }
            if p.len() < 3 {
                continue;
            }
            // At a shared band boundary use this band's one-sided color.
            let mid = (b.edges[i] + b.edges[i + 1]) * 0.5;
            let flat = self.engine().color(mid, auto);
            for j in 1..p.len() - 1 {
                out.push([p[0], p[j], p[j + 1]].map(|(xy, value)| {
                    (
                        xy,
                        if self.smooth {
                            self.color(
                                value.max(b.edges[i] + 1e-12).min(b.edges[i + 1] - 1e-12),
                                auto,
                            )
                            .unwrap_or(flat)
                        } else {
                            flat
                        },
                    )
                }));
            }
        }
        out
    }
    pub fn ui(
        &mut self,
        ui: &mut egui::Ui,
        auto: [f64; 2],
        style: super::app::drawing_flags::TextStyle,
    ) {
        self.ui_with_header(ui, auto, style, |_| {});
    }
    pub fn ui_with_header(
        &mut self,
        ui: &mut egui::Ui,
        auto: [f64; 2],
        style: super::app::drawing_flags::TextStyle,
        header: impl FnOnce(&mut egui::Ui),
    ) {
        if self.discrete.is_some() {
            header(ui);
            self.discrete_ui(ui, style);
            return;
        }
        self.initialise(auto);
        if self.reinforcement {
            ui.add(
                egui::Label::new("Площадь, см²/м: число, 12/250, s250d12 или сумма 12/250+16/250.")
                    .wrap(),
            );
        }
        let mut remove = None;
        ui.horizontal(|ui| {
            if ui
                .add_enabled(self.rows.len() < 32, egui::Button::new("Добавить"))
                .clicked()
            {
                if let Some(last) = self.rows.last_mut() {
                    if last.upper == "max" {
                        last.upper = auto[1].to_string();
                    }
                }
                let c = ColorScale {
                    palette: self.palette,
                    ..Default::default()
                }
                .color(auto[1], auto);
                self.rows.push(Row {
                    upper: "max".into(),
                    color: [c.r(), c.g(), c.b()],
                    enabled: true,
                });
            }
            header(ui);
        });
        egui::Grid::new("result_scale_rows")
            .spacing([5., 4.])
            .striped(true)
            .show(ui, |ui| {
                for h in ["Вкл.", "Цвет", "От", "До (включительно)", ""] {
                    ui.label(h);
                }
                ui.end_row();
                let last = self.rows.len() - 1;
                for i in 0..self.rows.len() {
                    if ui.checkbox(&mut self.rows[i].enabled, "").changed() {
                        self.set_band_visibility(i, self.rows[i].enabled);
                        ui.ctx().request_repaint();
                    }
                    ui.push_id(i, |ui| {
                        super::color_picker::button(ui, &mut self.rows[i].color);
                    });
                    ui.label(if i == 0 {
                        "−∞".into()
                    } else {
                        if self.reinforcement {
                            super::scale_input::value(&self.rows[i - 1].upper, true)
                                .map(|v| style.number(v))
                                .unwrap_or_else(|_| "Ошибка".into())
                        } else {
                            style.numeric_text(&self.rows[i - 1].upper)
                        }
                    });
                    let id = ui.id().with(("scale_upper", i));
                    let focused = ui.memory(|m| m.has_focus(id));
                    let mut text = if focused {
                        self.rows[i].upper.clone()
                    } else {
                        style.numeric_text(&self.rows[i].upper)
                    };
                    let response =
                        ui.add_sized([110., 22.], egui::TextEdit::singleline(&mut text).id(id));
                    if response.changed() {
                        self.rows[i].upper = text;
                    }
                    if response.gained_focus() {
                        ui.ctx().request_repaint();
                    }
                    if ui.add_enabled(last > 0, egui::Button::new("−")).clicked() {
                        remove = Some(i);
                    }
                    ui.end_row();
                }
            });
        if let Some(i) = remove {
            let last = i + 1 == self.rows.len();
            self.rows.remove(i);
            if last {
                self.rows.last_mut().unwrap().upper = "max".into();
            }
        }
        let previous_palette = self.palette;
        egui::ComboBox::from_id_salt("result_palette")
            .selected_text(self.palette.label())
            .show_ui(ui, |ui| {
                for p in Palette::ALL {
                    ui.selectable_value(&mut self.palette, p, p.label());
                }
            });
        if previous_palette != self.palette || ui.button("Восстановить цвета палитры").clicked()
        {
            self.restore_palette_colors();
        }
        ui.checkbox(&mut self.smooth, "Плавные переходы");
        ui.horizontal(|ui| {
            if ui.button("Применить").clicked() {
                self.error = self.apply(auto).err().unwrap_or_default();
                if self.error.is_empty() {
                    self.action = Some(Action::Apply);
                }
            }
            if ui.button("Сбросить").clicked() {
                self.error = self.reset_values(auto).err().unwrap_or_default();
                if self.error.is_empty() {
                    self.action = Some(Action::Reset);
                }
            }
        });
        ui.horizontal(|ui| {
            if ui.button("Сохранить…").clicked() {
                self.error = self.save(auto).err().unwrap_or_default();
            }
            if ui.button("Открыть…").clicked() {
                self.error = self.open().err().unwrap_or_default();
                if self.error.is_empty() {
                    self.action = Some(Action::Apply);
                }
            }
        });
        if !self.error.is_empty() {
            ui.colored_label(Color32::from_rgb(200, 130, 240), &self.error);
        }
    }
    fn save(&mut self, auto: [f64; 2]) -> Result<(), String> {
        self.initialise(auto);
        self.apply(auto)?;
        let e = self.engine();
        let b = e.intervals(auto).unwrap();
        let preset = super::scale_file::Preset {
            format: "mini-fem-scale".into(),
            version: 1,
            kind: "numeric".into(),
            edges: b.edges[..b.len].to_vec(),
            labels: self.rows.iter().map(|r| r.upper.clone()).collect(),
            colors: self.rows.iter().map(|r| r.color).collect(),
            enabled: self.enabled.clone(),
        };
        let text = preset.encode()?;
        if let Some(path) = rfd::FileDialog::new()
            .add_filter("Шкала", &["json"])
            .set_file_name("Шкала.json")
            .save_file()
        {
            crate::plan_trace::platform::write(path, text).map_err(|e| e.to_string())?;
        }
        Ok(())
    }
    fn open(&mut self) -> Result<(), String> {
        let Some(path) = rfd::FileDialog::new()
            .add_filter("Шкала", &["json"])
            .pick_file()
        else {
            return Ok(());
        };
        let p = super::scale_file::Preset::decode(
            &std::fs::read_to_string(path).map_err(|e| e.to_string())?,
        )?;
        self.load_preset(p)
    }
    #[cfg(target_arch = "wasm32")]
    pub(super) fn open_browser_preset(&mut self, bytes: &[u8]) -> Result<(), String> {
        if let Ok(state) = serde_json::from_slice::<State>(bytes) {
            *self = Self::from_state(state)?;
            return Ok(());
        }
        self.load_preset(super::scale_file::Preset::decode(
            std::str::from_utf8(bytes).map_err(|e| e.to_string())?,
        )?)
    }
    fn load_preset(&mut self, p: super::scale_file::Preset) -> Result<(), String> {
        if p.edges.len() != p.colors.len() + 1 || p.colors.len() > 32 {
            return Err("Нужна числовая шкала".into());
        }
        let mut b = Breaks {
            edges: [0.; 33],
            len: p.edges.len(),
        };
        b.edges[..b.len].copy_from_slice(&p.edges);
        let mut e = ColorScale {
            breaks: Some(b),
            bands: p.colors.len(),
            palette: self.palette,
            ..Default::default()
        };
        self.rows = p
            .colors
            .iter()
            .enumerate()
            .map(|(i, &color)| {
                e.colors[i] = Some(Color32::from_rgb(color[0], color[1], color[2]));
                Row {
                    upper: if super::scale_input::value(&p.labels[i], self.reinforcement).is_ok_and(
                        |value| (value - p.edges[i + 1]).abs() <= 1e-9 * value.abs().max(1.),
                    ) {
                        p.labels[i].clone()
                    } else {
                        p.edges[i + 1].to_string()
                    },
                    color,
                    enabled: p.enabled[i],
                }
            })
            .collect();
        self.committed_labels = self.rows.iter().map(|r| r.upper.clone()).collect();
        self.enabled = p.enabled;
        self.active = Some(e);
        Ok(())
    }
}
fn clip(p: Vec<(Point, f64)>, edge: f64, above: bool) -> Vec<(Point, f64)> {
    let mut out = vec![];
    if p.is_empty() {
        return out;
    }
    let inside = |v: f64| if above { v >= edge } else { v <= edge };
    let mut a = *p.last().unwrap();
    for b in p {
        if inside(a.1) != inside(b.1) {
            let t = (edge - a.1) / (b.1 - a.1);
            out.push((
                [
                    a.0[0] + t * (b.0[0] - a.0[0]),
                    a.0[1] + t * (b.0[1] - a.0[1]),
                ],
                edge,
            ));
        }
        if inside(b.1) {
            out.push(b);
        }
        a = b;
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn visibility_applies_without_committing_invalid_draft_boundaries() {
        let mut scale = Scale::default();
        scale.initialise([0., 8.]);
        scale.apply([0., 8.]).unwrap();
        let before = scale.signature();
        let boundaries = scale.engine().breaks.unwrap().edges;
        scale.rows[0].upper = "unfinished".into();
        scale.set_band_visibility(0, false);
        assert!(scale.color(0.5, [0., 8.]).is_none());
        assert_ne!(scale.signature(), before);
        assert_eq!(scale.engine().breaks.unwrap().edges, boundaries);
        let restored = Scale::from_state(scale.state()).unwrap();
        assert!(restored.color(0.5, [0., 8.]).is_none());
        scale.set_band_visibility(0, true);
        assert!(scale.color(0.5, [0., 8.]).is_some());
        assert_eq!(scale.rows[0].upper, "unfinished");
    }

    #[test]
    fn applied_text_bounds_restore_independently_of_unapplied_draft_and_colors() {
        let mut scale = Scale::default();
        scale.set_reinforcement_mode(true);
        scale.initialise([0., 20.]);
        scale.rows.truncate(2);
        scale.rows[0].upper = "10/250".into();
        scale.rows[1].upper = "16/250".into();
        scale.apply([0., 20.]).unwrap();
        let saved = scale.bounds().unwrap();
        scale.rows[0].upper = "unfinished".into();
        assert_eq!(scale.bounds().unwrap().labels[0], "10/250");
        scale.reset_values([0., 100.]).unwrap();
        scale.restore_bounds(&saved).unwrap();
        assert_eq!(scale.rows[0].upper, "10/250");
        assert_eq!(scale.bounds().unwrap().edges, saved.edges);
        let palette = scale.palette;
        let count = scale.rows.len();
        scale.reset_values([0., 200.]).unwrap();
        assert!(scale.palette == palette);
        assert_eq!(scale.rows.len(), count);
    }
    #[test]
    fn reinforcement_boundaries_use_grid_notation_and_keep_source_labels() {
        let mut scale = Scale::default();
        scale.set_reinforcement_mode(true);
        scale.initialise([0., 20.]);
        scale.rows.truncate(2);
        scale.rows[0].upper = "s250d12".into();
        scale.rows[1].upper = "12/250+16/250".into();
        scale.apply([0., 20.]).unwrap();
        let boundaries = scale.active.unwrap().breaks.unwrap();
        assert!((boundaries.edges[1] - 4.523893421169302).abs() < 1e-8);
        assert!((boundaries.edges[2] - 12.566370614359172).abs() < 1e-8);
        let restored = Scale::from_state(scale.state()).unwrap();
        assert_eq!(restored.rows[1].upper, "12/250+16/250");
        let before = scale.active;
        scale.rows[0].upper = "-1".into();
        assert!(scale.apply([0., 20.]).is_err());
        assert_eq!(
            scale.active.unwrap().breaks.unwrap().edges,
            before.unwrap().breaks.unwrap().edges
        );
    }
    #[test]
    fn automatic_load_scale_preserves_categories_colors_and_exact_values() {
        let mut s = Scale::default();
        s.sync_discrete_values(vec![5., 0., -2., 5. + 1e-12, f64::NAN]);
        assert_eq!(s.discrete.as_ref().unwrap().len(), 3);
        assert_eq!(s.rows.len(), 3);
        s.rows[2].color = [1, 2, 3];
        s.rows[1].enabled = false;
        assert_eq!(s.color(5., [-2., 5.]), Some(Color32::from_rgb(1, 2, 3)));
        assert!(s.color(0., [-2., 5.]).is_none());
        let before = serde_json::to_vec(&s.state()).unwrap();
        s.sync_discrete_values(vec![5. + 1e-12, 5., 0., -2.]);
        assert_eq!(serde_json::to_vec(&s.state()).unwrap(), before);
        s.sync_discrete_values(vec![-2., 0., 3., 5.]);
        assert_eq!(s.rows[3].color, [1, 2, 3]);
        assert!(!s.rows[1].enabled);
        s.rescale_values(0.1);
        assert_eq!(s.color(0.5, [-0.2, 0.5]), Some(Color32::from_rgb(1, 2, 3)));
        let restored = Scale::from_state(
            serde_json::from_slice(&serde_json::to_vec(&s.state()).unwrap()).unwrap(),
        )
        .unwrap();
        assert_eq!(restored.signature(), s.signature());
        s.sync_discrete_values((0..65).map(|i| i as f64).collect());
        assert_eq!(s.rows.len(), 65);
        assert!(s.color(64., [0., 64.]).is_some());
        assert!(Scale::from_state(s.state()).is_ok());
        s.sync_discrete_values(vec![2.3567]);
        assert_eq!(s.rows.len(), 1);
        assert_eq!(s.rows[0].upper, "2.3567");
        s.sync_discrete_values(vec![]);
        assert!(s.rows.is_empty());
        assert!(s.color(0., [0., 1.]).is_none());
    }
    #[test]
    fn categorical_palettes_do_not_repeat_every_third_floor() {
        for n in [8, 33] {
            for palette in [Palette::Engineering, Palette::Contrast, Palette::Traffic] {
                let mut scale = Scale {
                    palette,
                    ..Scale::default()
                };
                scale.sync_discrete_values((1..=n).map(|floor| floor as f64).collect());
                let colors = |scale: &Scale| {
                    scale
                        .rows
                        .iter()
                        .map(|row| row.color)
                        .collect::<std::collections::BTreeSet<_>>()
                };
                assert_eq!(colors(&scale).len(), n);
                scale.rows[0].color = scale.rows[1].color;
                scale.restore_palette_colors();
                assert_eq!(colors(&scale).len(), n);
            }
        }
    }
    #[test]
    fn legend_values_align_with_color_boundaries_and_include_both_ends() {
        let s = Scale::default();
        let ctx = egui::Context::default();
        let rect = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1000., 200.));
        let style = super::super::app::drawing_flags::TextStyle::default();
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            let painter = ctx.layer_painter(egui::LayerId::new(
                egui::Order::Middle,
                egui::Id::new("legend"),
            ));
            s.legend(&painter, rect, [0., 8.], style);
        });
        let labels = out
            .shapes
            .iter()
            .filter_map(|s| {
                if let egui::Shape::Text(t) = &s.shape {
                    Some(t)
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        assert_eq!(labels.len(), 9);
        assert_eq!(labels[0].galley.job.text, "0.00");
        assert_eq!(labels[8].galley.job.text, "8.00");
        for i in 1..8 {
            let center = labels[i].pos.x + labels[i].galley.size().x * 0.5;
            let boundary = 12. + i as f32 * (976. / 8.);
            assert!((center - boundary).abs() < 1.);
            assert_eq!(labels[i].galley.job.text, format!("{i}.00"));
        }
    }
    #[test]
    fn reinforcement_legend_rounds_numbers_but_preserves_bar_expressions() {
        let mut s = Scale::default();
        s.set_reinforcement_mode(true);
        s.initialise([0., 5.]);
        s.rows.truncate(3);
        s.rows[0].upper = "2.3283333333333333".into();
        s.rows[1].upper = "10/250".into();
        s.rows[2].upper = "max".into();
        s.apply([0., 5.]).unwrap();
        let before = serde_json::to_vec(&s.state()).unwrap();
        let ctx = egui::Context::default();
        let style = super::super::app::drawing_flags::TextStyle {
            decimals: 3,
            ..Default::default()
        };
        let out = ctx.run(egui::RawInput::default(), |ctx| {
            s.legend(
                &ctx.layer_painter(egui::LayerId::new(
                    egui::Order::Middle,
                    egui::Id::new("rc_legend"),
                )),
                egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(1000., 200.)),
                [0., 5.],
                style,
            )
        });
        let labels = out
            .shapes
            .iter()
            .filter_map(|s| {
                if let egui::Shape::Text(t) = &s.shape {
                    Some(t.galley.job.text.as_str())
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        assert!(labels.contains(&"2.328"));
        assert!(labels.contains(&"10/250"));
        assert!(labels.contains(&"5.000"));
        assert_eq!(before, serde_json::to_vec(&s.state()).unwrap());
    }
    #[test]
    fn display_rounding_does_not_modify_scale_thresholds_or_drafts() {
        let mut s = Scale::default();
        s.initialise([0., 10.]);
        s.rows.truncate(2);
        s.rows[0].upper = "2.3567".into();
        s.rows[1].upper = "max".into();
        s.apply([0., 10.]).unwrap();
        let before = serde_json::to_vec(&s.state()).unwrap();
        let signature = s.signature();
        let style = super::super::app::drawing_flags::TextStyle::default();
        assert_eq!(style.numeric_text(&s.rows[0].upper), "2.36");
        let ctx = egui::Context::default();
        for decimals in [0, 2, 8] {
            let style = super::super::app::drawing_flags::TextStyle { decimals, ..style };
            let _ = ctx.run(egui::RawInput::default(), |ctx| {
                egui::CentralPanel::default().show(ctx, |ui| {
                    s.ui(ui, [0., 10.], style);
                    s.legend(ui.painter(), ui.max_rect(), [0., 10.], style);
                });
            });
        }
        assert_eq!(serde_json::to_vec(&s.state()).unwrap(), before);
        assert_eq!(s.signature(), signature);
        assert_eq!(s.engine().intervals([0., 10.]).unwrap().edges[1], 2.3567);
        assert_eq!(style.number(-0.0001), "0.00");
    }
    #[test]
    fn reset_equalises_values_without_changing_count_colors_visibility_or_smoothing() {
        let mut s = Scale::default();
        s.initialise([-10., 20.]);
        s.rows.truncate(3);
        s.rows[0].color = [17, 31, 45];
        s.rows[1].enabled = false;
        s.smooth = false;
        let colors = s.rows.iter().map(|r| r.color).collect::<Vec<_>>();
        s.rows[0].upper = "unfinished".into();
        s.reset_values([-3., 9.]).unwrap();
        assert_eq!(s.rows.len(), 3);
        assert_eq!(s.rows.iter().map(|r| r.color).collect::<Vec<_>>(), colors);
        assert!(!s.smooth);
        assert_eq!(s.enabled, vec![true, false, true]);
        let b = s.engine().intervals([-3., 9.]).unwrap();
        assert_eq!(&b.edges[..b.len], &[-3., 1., 5., 9.]);
        assert_eq!(s.rows[2].upper, "max");
    }

    #[test]
    fn reset_replaces_bounds_when_current_values_are_almost_constant() {
        let mut scale = Scale::default();
        scale.reset_values([-100., 100.]).unwrap();
        let limits = range([20., 20_f64.next_up()].into_iter());
        scale.reset_values(limits).unwrap();
        let edges = scale.bounds().unwrap().edges;
        assert_eq!(edges.first(), Some(&limits[0]));
        assert_eq!(edges.last(), Some(&limits[1]));
        assert!(edges.windows(2).all(|p| p[0] < p[1]));
    }
    #[test]
    fn converting_force_units_preserves_band_colors_and_visibility() {
        let mut s = Scale::default();
        s.initialise([0., 10.]);
        s.rows[0].enabled = false;
        s.apply([0., 10.]).unwrap();
        let color = s.color(5., [0., 10.]);
        s.rescale_values(0.1);
        assert_eq!(s.color(0.5, [0., 1.]), color);
        assert!(s.color(0.01, [0., 1.]).is_none());
        s.rescale_values(10.);
        assert_eq!(s.color(5., [0., 10.]), color);
    }
    #[test]
    fn project_scale_state_preserves_applied_settings_and_unapplied_draft() {
        let mut s = Scale::default();
        s.initialise([-10., 20.]);
        s.rows[0].color = [10, 20, 30];
        s.rows[0].enabled = false;
        s.apply([-10., 20.]).unwrap();
        s.rows[1].upper = "unfinished".into();
        let state = serde_json::to_vec(&s.state()).unwrap();
        let copy = Scale::from_state(serde_json::from_slice(&state).unwrap()).unwrap();
        assert_eq!(s.signature(), copy.signature());
        assert_eq!(copy.rows[1].upper, "unfinished");
        assert_eq!(s.color(-9., [-10., 20.]), copy.color(-9., [-10., 20.]));
    }
    #[test]
    fn upper_bound_is_inclusive_and_disabled_band_clips_linear_field() {
        let mut s = Scale::default();
        s.rows = vec![
            Row {
                upper: "0.5".into(),
                color: [0, 0, 255],
                enabled: true,
            },
            Row {
                upper: "max".into(),
                color: [255, 0, 0],
                enabled: false,
            },
        ];
        s.apply([0., 1.]).unwrap();
        assert!(s.color(0.5, [0., 1.]).is_some());
        assert!(s.color(0.5001, [0., 1.]).is_none());
        let triangles = s.triangles([[0., 0.], [1., 0.], [0., 1.]], [0., 1., 0.], [0., 1.]);
        let mut area = 0.;
        for t in triangles {
            assert!(t.iter().all(|(p, _)| p[0] <= 0.5 + 1e-12));
            area += super::super::geometry::area(&t.map(|(p, _)| p));
        }
        assert!((area - 0.375).abs() < 1e-12);
    }
    #[test]
    fn palette_switch_updates_rows_and_applied_colors_without_applying_draft_boundaries() {
        let mut scale = Scale::default();
        scale.initialise([0., 10.]);
        scale.apply([0., 10.]).unwrap();
        let breaks = scale.engine().breaks.unwrap();
        scale.rows[0].upper = "unfinished".into();
        scale.palette = Palette::ALL[0];
        scale.restore_palette_colors();
        assert_eq!(scale.rows[0].upper, "unfinished");
        assert_eq!(scale.engine().breaks.unwrap().edges, breaks.edges);
        for (i, row) in scale.rows.iter().enumerate() {
            let color = scale.engine().colors[i].unwrap();
            assert_eq!(row.color, [color.r(), color.g(), color.b()]);
        }
    }
    #[test]
    fn bad_draft_does_not_replace_applied_scale_and_constants_are_finite() {
        let mut s = Scale::default();
        s.initialise([0., 1.]);
        s.apply([0., 1.]).unwrap();
        let old = s.color(0.2, [0., 1.]);
        s.rows[0].upper = "bad".into();
        assert!(s.apply([0., 1.]).is_err());
        assert_eq!(old, s.color(0.2, [0., 1.]));
        let r = range([5., 5.].into_iter());
        assert!(r[0] < 5. && r[1] > 5.);
        assert!(s.color(5., r).is_some());
    }
}
