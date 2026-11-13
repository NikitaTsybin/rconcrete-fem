use super::*;
use std::cell::{Cell, RefCell};
pub(super) const DEFAULT: [u8; 21] = [
    20, 19, 18, 13, 10, 9, 8, 7, 6, 5, 17, 16, 4, 3, 2, 11, 12, 1, 14, 15, 0,
];
pub(super) fn default_order() -> Vec<u8> {
    DEFAULT.to_vec()
}
pub(super) fn category(kind: Kind) -> u8 {
    match kind {
        Kind::Slab => 2,
        Kind::Hole => 3,
        Kind::Support => 4,
        Kind::Auxiliary => 5,
        Kind::Punching => 18,
        Kind::Foundation => 20,
        Kind::MeshRegion => 6,
        Kind::LoadArea => 7,
        Kind::LoadLine => 8,
        Kind::LoadPoint => 9,
        Kind::LoadHole => 10,
        Kind::Column => 19,
        Kind::Beam => 16,
        Kind::Triangulation | Kind::Restraint | Kind::PointRestraint | Kind::RigidLink => 17,
    }
}
fn label(kind: u8) -> &'static str {
    match kind {
        0 => "Подложки PDF",
        1 => "Подложки DXF",
        2 => "Границы плит",
        3 => "Отверстия",
        4 => "Стены",
        19 => "Колонны",
        5 => "Вспомогательные линии",
        6 => "Области КЭ",
        7 => "Распределённые нагрузки",
        8 => "Линейные нагрузки",
        9 => "Сосредоточенные нагрузки",
        10 => "Отверстия нагрузок",
        11 => "КЭ-сетка",
        12 => "Результаты",
        13 => "Размеры",
        14 => "Вспомогательная сетка",
        15 => "Заливка плит",
        16 => "Балки",
        18 => "Области суммирования нагрузок",
        20 => "Основания",
        _ => "Ограничения триангуляции",
    }
}
pub(super) struct Order {
    pub items: Vec<u8>,
    dragging: Option<u8>,
    recording: Cell<bool>,
    spans: RefCell<Vec<(u8, std::ops::Range<usize>)>>,
}
impl Default for Order {
    fn default() -> Self {
        Self {
            items: default_order(),
            dragging: None,
            recording: Cell::new(false),
            spans: RefCell::new(vec![]),
        }
    }
}
impl Order {
    pub fn restore(items: Vec<u8>) -> Self {
        let mut order = Self::default();
        let mut clean = vec![];
        for item in items.into_iter().chain(DEFAULT) {
            if item < 21 && !clean.contains(&item) {
                clean.push(item);
            }
        }
        order.items = clean;
        order
    }
    fn move_relative(&mut self, source: u8, target: u8, after: bool) {
        if source == target || !self.items.contains(&target) {
            return;
        }
        let Some(from) = self.items.iter().position(|&kind| kind == source) else {
            return;
        };
        self.items.remove(from);
        let target = self.items.iter().position(|&kind| kind == target).unwrap();
        self.items.insert(target + usize::from(after), source);
    }
    pub fn begin_frame(&self) {
        self.spans.borrow_mut().clear();
        self.recording.set(true);
    }
    pub fn start(&self, painter: &egui::Painter) -> usize {
        painter.ctx().graphics(|g| {
            g.get(painter.layer_id())
                .map_or(0, |list| list.next_idx().0)
        })
    }
    pub fn finish(&self, painter: &egui::Painter, category: u8, start: usize) {
        if !self.recording.get() {
            return;
        }
        let end = self.start(painter);
        if end > start {
            self.spans.borrow_mut().push((category, start..end));
        }
    }
    pub fn apply(&self, painter: &egui::Painter) {
        self.recording.set(false);
        let spans = self.spans.take();
        if spans.is_empty() {
            return;
        }
        painter.ctx().graphics_mut(|graphics| {
            let Some(list) = graphics.get_mut(painter.layer_id()) else {
                return;
            };
            let mut slots = vec![];
            let mut buckets: Vec<Vec<egui::epaint::ClippedShape>> =
                (0..DEFAULT.len()).map(|_| vec![]).collect();
            for (kind, range) in spans {
                for index in range {
                    slots.push(index);
                    list.mutate_shape(egui::layers::ShapeIdx(index), |shape| {
                        buckets[kind as usize].push(std::mem::replace(
                            shape,
                            egui::epaint::ClippedShape {
                                clip_rect: Rect::NOTHING,
                                shape: egui::Shape::Noop,
                            },
                        ));
                    });
                }
            }
            slots.sort_unstable();
            let mut slots = slots.into_iter();
            for &kind in self.items.iter().rev() {
                for shape in buckets[kind as usize].drain(..) {
                    let Some(index) = slots.next() else {
                        return;
                    };
                    list.mutate_shape(egui::layers::ShapeIdx(index), |destination| {
                        *destination = shape
                    });
                }
            }
        });
    }
}
impl App {
    pub(super) fn draw_order_controls(&mut self, ui: &mut egui::Ui) {
        egui::CollapsingHeader::new("Порядок отрисовки")
            .default_open(false)
            .show(ui, |ui| {
                ui.small("Выше в списке — поверх остальных.");
                let mut present = std::collections::BTreeSet::new();
                for c in &self.contours {
                    present.insert(category(c.kind));
                    if c.kind == Kind::Slab {
                        present.insert(15);
                    }
                }
                for u in &self.underlays {
                    present.insert(
                        if matches!(u.data, crate::plan_trace::underlay::Data::Pdf { .. }) {
                            0
                        } else {
                            1
                        },
                    );
                }
                if self.drawing.is_some() {
                    present.insert(1);
                }
                if self.fe_mesh.is_some() {
                    present.insert(11);
                }
                if self.analysis.is_some() {
                    present.insert(12);
                }
                if !self.measurements.items.is_empty() {
                    present.insert(13);
                }
                if self.construction_grid.enabled {
                    present.insert(14);
                }
                if self
                    .load_cases
                    .iter()
                    .any(|c| c.kind == crate::plan_trace::loads::LoadType::SelfWeight)
                {
                    present.insert(7);
                }
                let visible = self
                    .draw_order
                    .items
                    .iter()
                    .enumerate()
                    .filter(|(_, kind)| present.contains(*kind))
                    .map(|(i, &kind)| (i, kind))
                    .collect::<Vec<_>>();
                let mut swap = None;
                let mut rows = Vec::new();
                ui.vertical(|ui| {
                    for (row, &(index, kind)) in visible.iter().enumerate() {
                        ui.push_id(kind, |ui| {
                            let row_response = ui.horizontal(|ui| {
                                if order_arrow(ui, true, row > 0).clicked() {
                                    swap = Some((index, visible[row - 1].0));
                                }
                                if order_arrow(ui, false, row + 1 < visible.len()).clicked() {
                                    swap = Some((index, visible[row + 1].0));
                                }
                                let response = ui
                                    .add_sized(
                                        [ui.available_width(), 22.],
                                        egui::Label::new(label(kind))
                                            .selectable(false)
                                            .sense(egui::Sense::click_and_drag()),
                                    )
                                    .on_hover_text("Перетащите строку, чтобы изменить порядок");
                                if response.drag_started_by(egui::PointerButton::Primary) {
                                    self.draw_order.dragging = Some(kind);
                                }
                            });
                            rows.push((kind, row_response.response.rect));
                        });
                    }
                });
                if let Some(source) = self.draw_order.dragging {
                    let pointer = ui.input(|i| i.pointer.interact_pos());
                    let target = pointer.and_then(|p| {
                        rows.iter()
                            .find(|(_, rect)| rect.contains(p))
                            .map(|(kind, rect)| (*kind, p.y >= rect.center().y, *rect))
                    });
                    if let Some((_, _, rect)) = rows
                        .iter()
                        .find(|(kind, _)| *kind == source)
                        .map(|(kind, rect)| (*kind, false, *rect))
                    {
                        ui.painter().rect_stroke(
                            rect.expand(1.),
                            2.,
                            Stroke::new(1_f32, ui.visuals().selection.stroke.color),
                            egui::StrokeKind::Middle,
                        );
                    }
                    if let Some((target, after, rect)) = target {
                        if target != source {
                            let y = if after {
                                rect.bottom() + 1.
                            } else {
                                rect.top() - 1.
                            };
                            ui.painter().line_segment(
                                [Pos2::new(rect.left(), y), Pos2::new(rect.right(), y)],
                                Stroke::new(2_f32, ui.visuals().selection.stroke.color),
                            );
                        }
                        ui.ctx().set_cursor_icon(egui::CursorIcon::Grabbing);
                    }
                    let escape = ui.input(|i| i.key_pressed(egui::Key::Escape));
                    if ui.input(|i| i.pointer.any_released()) {
                        if !escape {
                            if let Some((target, after, _)) = target {
                                if target != source {
                                    self.draw_order.move_relative(source, target, after);
                                }
                            }
                        }
                        self.draw_order.dragging = None;
                    } else if escape || !ui.input(|i| i.pointer.primary_down()) {
                        self.draw_order.dragging = None;
                    }
                }
                if let Some((a, b)) = swap {
                    self.draw_order.items.swap(a, b);
                }
            });
    }
}

fn order_arrow(ui: &mut egui::Ui, up: bool, enabled: bool) -> egui::Response {
    let response = ui.add_enabled(
        enabled,
        egui::Button::new("").min_size(egui::vec2(24., 22.)),
    );
    let color = if enabled {
        ui.style().interact(&response).fg_stroke.color
    } else {
        ui.visuals().weak_text_color()
    };
    let direction = if up { -1. } else { 1. };
    let center = response.rect.center();
    ui.painter().arrow(
        center - egui::vec2(0., direction * 5.),
        egui::vec2(0., direction * 10.),
        Stroke::new(1.5_f32, color),
    );
    response.on_hover_text(if up { "Выше" } else { "Ниже" })
}

#[cfg(test)]
mod drag_tests {
    use super::*;
    #[test]
    fn reorder_moves_before_after_without_losing_hidden_categories() {
        let mut order = Order::default();
        let original = order.items.clone();
        let source = original[0];
        let target = original[5];
        order.move_relative(source, target, true);
        let index = order.items.iter().position(|&x| x == target).unwrap();
        assert_eq!(order.items[index + 1], source);
        order.move_relative(source, target, false);
        let index = order.items.iter().position(|&x| x == target).unwrap();
        assert_eq!(order.items[index - 1], source);
        assert_eq!(
            order
                .items
                .iter()
                .copied()
                .filter(|&x| x != source)
                .collect::<Vec<_>>(),
            original
                .into_iter()
                .filter(|&x| x != source)
                .collect::<Vec<_>>()
        );
        let unchanged = order.items.clone();
        order.move_relative(source, source, true);
        assert_eq!(order.items, unchanged);
    }
}
