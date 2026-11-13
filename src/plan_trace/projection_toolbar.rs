//! Camera-only presets and resolution-independent cube icons.
use super::*;
use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

#[derive(Clone, Copy)]
struct Projection {
    label: &'static str,
    yaw: f64,
    pitch: f64,
    face: Option<(usize, f64)>,
}
fn presets() -> [Projection; 10] {
    let iso = -(2_f64.sqrt()).atan();
    [
        Projection {
            label: "Сверху · XY",
            yaw: 0.,
            pitch: 0.,
            face: Some((2, 1.)),
        },
        Projection {
            label: "Снизу · XY",
            yaw: 0.,
            pitch: -PI,
            face: Some((2, -1.)),
        },
        Projection {
            label: "Спереди · XZ",
            yaw: 0.,
            pitch: -FRAC_PI_2,
            face: Some((1, -1.)),
        },
        Projection {
            label: "Сзади · XZ",
            yaw: PI,
            pitch: -FRAC_PI_2,
            face: Some((1, 1.)),
        },
        Projection {
            label: "Слева · YZ",
            yaw: FRAC_PI_2,
            pitch: -FRAC_PI_2,
            face: Some((0, -1.)),
        },
        Projection {
            label: "Справа · YZ",
            yaw: -FRAC_PI_2,
            pitch: -FRAC_PI_2,
            face: Some((0, 1.)),
        },
        Projection {
            label: "Изометрия · +X −Y +Z",
            yaw: -FRAC_PI_4,
            pitch: iso,
            face: None,
        },
        Projection {
            label: "Изометрия · −X −Y +Z",
            yaw: FRAC_PI_4,
            pitch: iso,
            face: None,
        },
        Projection {
            label: "Изометрия · −X +Y +Z",
            yaw: 3. * FRAC_PI_4,
            pitch: iso,
            face: None,
        },
        Projection {
            label: "Изометрия · +X +Y +Z",
            yaw: -3. * FRAC_PI_4,
            pitch: iso,
            face: None,
        },
    ]
}
fn same_angle(a: f64, b: f64) -> bool {
    ((a - b + PI).rem_euclid(2. * PI) - PI).abs() < 1e-6
}
impl App {
    pub(super) fn projection_toolbar(&mut self, ctx: &egui::Context, canvas: Rect) -> Rect {
        egui::Area::new(egui::Id::new("spatial_projection_toolbar"))
            .order(egui::Order::Middle)
            .fixed_pos(egui::pos2(canvas.right() - 44., canvas.center().y - 166.))
            .movable(false)
            .constrain_to(canvas.shrink(4.))
            .show(ctx, |ui| {
                egui::Frame::popup(ui.style())
                    .inner_margin(4.)
                    .show(ui, |ui| {
                        ui.spacing_mut().item_spacing = egui::vec2(0., 2.);
                        ui.vertical(|ui| {
                            for (i, preset) in presets().into_iter().enumerate() {
                                if i == 6 {
                                    ui.add_space(4.);
                                }
                                let selected = same_angle(self.spatial.yaw, preset.yaw)
                                    && same_angle(self.spatial.pitch, preset.pitch);
                                let (rect, response) = ui.allocate_exact_size(
                                    egui::vec2(28., 28.),
                                    egui::Sense::click(),
                                );
                                let visual = ui.style().interact_selectable(&response, selected);
                                ui.painter().rect(
                                    rect,
                                    3.,
                                    visual.bg_fill,
                                    visual.bg_stroke,
                                    egui::StrokeKind::Inside,
                                );
                                cube_icon(ui.painter(), rect.shrink(3.), preset);
                                if response.on_hover_text(preset.label).clicked() {
                                    self.spatial.yaw = preset.yaw;
                                    self.spatial.pitch = preset.pitch;
                                    self.spatial.camera_busy_until = 0.;
                                    ctx.request_repaint();
                                }
                            }
                        });
                    });
            })
            .response
            .rect
    }
}

fn cube_icon(painter: &egui::Painter, rect: Rect, preset: Projection) {
    // Tilt orthographic icons slightly so the selected face and adjacent edges remain legible.
    let (yaw, pitch) = if preset.face.is_some() {
        (
            preset.yaw - 0.28,
            preset.pitch + if preset.pitch == 0. { -0.32 } else { 0.32 },
        )
    } else {
        (preset.yaw, preset.pitch)
    };
    let (sy, cy) = yaw.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let project = |p: [f64; 3]| {
        let a = cy * p[0] - sy * p[1];
        let b = sy * p[0] + cy * p[1];
        (
            egui::vec2(a as f32, -(cp * b - sp * p[2]) as f32),
            sp * b + cp * p[2],
        )
    };
    let corners = (0..8)
        .map(|i| {
            project(std::array::from_fn(|a| {
                if i & (1 << a) == 0 {
                    -1.
                } else {
                    1.
                }
            }))
        })
        .collect::<Vec<_>>();
    let max = corners
        .iter()
        .map(|p| p.0.x.abs().max(p.0.y.abs()))
        .fold(0_f32, f32::max);
    let scale = rect.width().min(rect.height()) * 0.47 / max;
    let mut faces = Vec::new();
    for axis in 0..3 {
        for side in [0, 1] {
            let other = (0..3).filter(|&a| a != axis).collect::<Vec<_>>();
            let ids = [(0, 0), (1, 0), (1, 1), (0, 1)]
                .map(|(a, b)| (side << axis) | (a << other[0]) | (b << other[1]));
            let depth = ids.iter().map(|&i| corners[i].1).sum::<f64>() / 4.;
            if depth <= 1e-8 {
                continue;
            }
            let highlighted = preset
                .face
                .is_none_or(|(a, sign)| a == axis && (if side == 0 { -1. } else { 1. }) == sign);
            let fill = if highlighted {
                [
                    Color32::from_rgb(238, 149, 130),
                    Color32::from_rgb(129, 203, 159),
                    Color32::from_rgb(107, 184, 235),
                ][axis]
            } else {
                Color32::from_gray(206 - axis as u8 * 16)
            };
            faces.push((
                depth,
                ids.map(|i| rect.center() + corners[i].0 * scale).to_vec(),
                fill,
            ));
        }
    }
    faces.sort_by(|a, b| a.0.total_cmp(&b.0));
    for (_, points, fill) in faces {
        painter.add(egui::Shape::convex_polygon(
            points,
            fill,
            egui::Stroke::new(0.9, Color32::from_rgb(40, 63, 79)),
        ));
    }
}
