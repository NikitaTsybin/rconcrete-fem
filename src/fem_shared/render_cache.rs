//! Batched antialiased screen lines. Geometry/IDs remain in the calculation model.
use eframe::egui;
use eframe::egui_wgpu::{self, wgpu};
use std::sync::{Arc, Weak};
use wgpu::util::DeviceExt;

struct Buffers {
    owner: Weak<egui::Mesh>,
    viewport: egui::Rect,
    vertices: wgpu::Buffer,
    indices: wgpu::Buffer,
    count: u32,
}
struct Gpu {
    pipeline: wgpu::RenderPipeline,
    uploads: usize,
    buffers: std::collections::HashMap<usize, Buffers>,
}
struct Draw {
    mesh: Arc<egui::Mesh>,
    viewport: egui::Rect,
}

const SHADER: &str = r#"
struct Vertex { @location(0) position: vec2<f32>, @location(1) color: vec4<f32> };
struct Output { @builtin(position) position: vec4<f32>, @location(0) color: vec4<f32> };
@vertex fn vs_main(v: Vertex) -> Output {
    var o: Output; o.position=vec4<f32>(v.position,0.0,1.0); o.color=v.color; return o;
}
@fragment fn fs_gamma(v: Output) -> @location(0) vec4<f32> { return v.color; }
@fragment fn fs_linear(v: Output) -> @location(0) vec4<f32> {
    let low=v.color.rgb/12.92;
    let high=pow((v.color.rgb+vec3<f32>(0.055))/1.055,vec3<f32>(2.4));
    return vec4<f32>(select(high,low,v.color.rgb<vec3<f32>(0.04045)),v.color.a);
}
"#;

impl Gpu {
    fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        Self::with_depth(device, format, None)
    }
    fn with_depth(
        device: &wgpu::Device,
        format: wgpu::TextureFormat,
        depth: Option<wgpu::TextureFormat>,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("cached 2D geometry"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("cached 2D geometry"),
            layout: None,
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: 24,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x2,1=>Float32x4],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some(if format.is_srgb() {
                    "fs_linear"
                } else {
                    "fs_gamma"
                }),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: depth.map(|format| wgpu::DepthStencilState {
                format,
                depth_write_enabled: false,
                depth_compare: wgpu::CompareFunction::Always,
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        });
        Self {
            pipeline,
            uploads: 0,
            buffers: Default::default(),
        }
    }
}

/// Register once per renderer, not per project. Opening a project keeps resources.
pub fn install(cc: &eframe::CreationContext<'_>) {
    install_with_depth(cc, None);
}
pub fn install_with_depth(cc: &eframe::CreationContext<'_>, depth: Option<wgpu::TextureFormat>) {
    if let Some(state) = &cc.wgpu_render_state {
        state
            .renderer
            .write()
            .callback_resources
            .insert(Gpu::with_depth(&state.device, state.target_format, depth));
        cc.egui_ctx
            .data_mut(|d| d.insert_temp(egui::Id::new("matrix_cached_gpu"), true));
    }
}

pub fn paint(painter: &egui::Painter, viewport: egui::Rect, mesh: Arc<egui::Mesh>) {
    if mesh.is_empty() {
        return;
    }
    if painter.ctx().data(|d| {
        d.get_temp::<bool>(egui::Id::new("matrix_cached_gpu"))
            .unwrap_or(false)
    }) {
        painter.add(egui_wgpu::Callback::new_paint_callback(
            viewport,
            Draw { mesh, viewport },
        ));
    } else {
        // CPU-only tests and alternative backends retain a safe ordinary path.
        painter.add(egui::Shape::mesh(mesh));
    }
}

impl egui_wgpu::CallbackTrait for Draw {
    fn prepare(
        &self,
        device: &wgpu::Device,
        _: &wgpu::Queue,
        _: &egui_wgpu::ScreenDescriptor,
        _: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if self.mesh.is_empty() || self.viewport.width() <= 0. || self.viewport.height() <= 0. {
            return vec![];
        }
        let Some(gpu) = resources.get_mut::<Gpu>() else {
            return vec![];
        };
        gpu.buffers.retain(|_, b| b.owner.strong_count() > 0);
        let key = Arc::as_ptr(&self.mesh) as usize;
        if gpu
            .buffers
            .get(&key)
            .is_some_and(|b| b.viewport == self.viewport)
        {
            return vec![];
        }
        let mut vertices = Vec::with_capacity(self.mesh.vertices.len() * 24);
        for vertex in &self.mesh.vertices {
            let position = [
                (vertex.pos.x - self.viewport.left()) / self.viewport.width() * 2. - 1.,
                1. - (vertex.pos.y - self.viewport.top()) / self.viewport.height() * 2.,
            ];
            let [r, g, b, a] = vertex.color.to_array();
            for v in [
                position[0],
                position[1],
                r as f32 / 255.,
                g as f32 / 255.,
                b as f32 / 255.,
                a as f32 / 255.,
            ] {
                vertices.extend_from_slice(&v.to_ne_bytes());
            }
        }
        let mut indices = Vec::with_capacity(self.mesh.indices.len() * 4);
        for i in &self.mesh.indices {
            indices.extend_from_slice(&i.to_ne_bytes());
        }
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("retained scene vertices"),
            contents: &vertices,
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("retained scene indices"),
            contents: &indices,
            usage: wgpu::BufferUsages::INDEX,
        });
        gpu.uploads += 1;
        gpu.buffers.insert(
            key,
            Buffers {
                owner: Arc::downgrade(&self.mesh),
                viewport: self.viewport,
                vertices: vertex_buffer,
                indices: index_buffer,
                count: self.mesh.indices.len() as u32,
            },
        );
        vec![]
    }
    fn paint(
        &self,
        _: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        let Some(gpu) = resources.get::<Gpu>() else {
            return;
        };
        let Some(buffers) = gpu.buffers.get(&(Arc::as_ptr(&self.mesh) as usize)) else {
            return;
        };
        pass.set_pipeline(&gpu.pipeline);
        pass.set_vertex_buffer(0, buffers.vertices.slice(..));
        pass.set_index_buffer(buffers.indices.slice(..), wgpu::IndexFormat::Uint32);
        pass.draw_indexed(0..buffers.count, 0, 0..1);
    }
}

/// Append a thin line with a one-pixel alpha fringe, without per-line Shapes.
pub fn line(
    mesh: &mut egui::Mesh,
    a: egui::Pos2,
    b: egui::Pos2,
    width: f32,
    color: egui::Color32,
    pixels_per_point: f32,
) {
    let delta = b - a;
    let length = delta.length();
    if !length.is_finite() || length < 1e-3 {
        return;
    }
    let normal = egui::vec2(-delta.y, delta.x) / length;
    let fringe = 1. / pixels_per_point.max(0.1);
    let half = (width - fringe).max(0.) * 0.5;
    // Match egui's subpixel stroke coverage instead of making thin lines darker.
    let color = if width < fringe {
        color.gamma_multiply((width / fringe).max(0.))
    } else {
        color
    };
    let base = mesh.vertices.len() as u32;
    for (distance, ink) in [
        (-half - fringe, egui::Color32::TRANSPARENT),
        (-half, color),
        (half, color),
        (half + fringe, egui::Color32::TRANSPARENT),
    ] {
        for p in [a, b] {
            mesh.vertices.push(egui::epaint::Vertex {
                pos: p + normal * distance,
                uv: egui::epaint::WHITE_UV,
                color: ink,
            });
        }
    }
    for strip in 0..3 {
        let i = base + strip * 2;
        mesh.indices
            .extend_from_slice(&[i, i + 1, i + 2, i + 1, i + 3, i + 2]);
    }
}

/// Merge contiguous active horizontal and vertical cell edges. Each shared edge
/// appears once. Empty/hidden cells do not create lines; holes remain outlined.
pub fn grid_runs(
    width: usize,
    height: usize,
    visible: &[bool],
    mut emit: impl FnMut([usize; 2], [usize; 2]),
) {
    assert_eq!(visible.len(), width * height);
    for y in 0..=height {
        let mut start = None;
        for x in 0..=width {
            let edge = x < width
                && ((y > 0 && visible[(y - 1) * width + x])
                    || (y < height && visible[y * width + x]));
            if edge {
                if start.is_none() {
                    start = Some(x);
                }
            } else if let Some(begin) = start.take() {
                emit([begin, y], [x, y]);
            }
        }
    }
    for x in 0..=width {
        let mut start = None;
        for y in 0..=height {
            let edge = y < height
                && ((x > 0 && visible[y * width + x - 1]) || (x < width && visible[y * width + x]));
            if edge {
                if start.is_none() {
                    start = Some(y);
                }
            } else if let Some(begin) = start.take() {
                emit([x, begin], [x, y]);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires a headless wgpu adapter; renders and reads an offscreen texture"]
    fn retained_gpu_draw_matches_viewport_pixels_and_reuses_buffers() {
        use egui_wgpu::CallbackTrait;
        fn block_on<T>(future: impl std::future::Future<Output = T>) -> T {
            struct Wake(std::thread::Thread);
            impl std::task::Wake for Wake {
                fn wake(self: Arc<Self>) {
                    self.0.unpark();
                }
            }
            let waker = std::task::Waker::from(Arc::new(Wake(std::thread::current())));
            let mut cx = std::task::Context::from_waker(&waker);
            let mut future = std::pin::pin!(future);
            loop {
                match future.as_mut().poll(&mut cx) {
                    std::task::Poll::Ready(v) => return v,
                    std::task::Poll::Pending => std::thread::park(),
                }
            }
        }
        let instance = wgpu::Instance::default();
        let adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default()))
            .expect("No headless wgpu adapter");
        println!("adapter: {:?}", adapter.get_info());
        let (device, queue) =
            block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None)).unwrap();
        let format = wgpu::TextureFormat::Rgba8Unorm;
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("cache render test"),
            size: wgpu::Extent3d {
                width: 64,
                height: 64,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let view = texture.create_view(&Default::default());
        let viewport = egui::Rect::from_min_size(egui::pos2(10., 10.), egui::vec2(32., 32.));
        let mut mesh = egui::Mesh::default();
        mesh.add_colored_rect(viewport, egui::Color32::RED);
        let draw = Draw {
            mesh: Arc::new(mesh),
            viewport,
        };
        let mut resources = egui_wgpu::CallbackResources::default();
        resources.insert(Gpu::new(&device, format));
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [64, 64],
            pixels_per_point: 1.,
        };
        let mut encoder = device.create_command_encoder(&Default::default());
        draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        assert_eq!(resources.get::<Gpu>().unwrap().uploads, 1);
        {
            let mut pass = encoder
                .begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("cache test"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &view,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                })
                .forget_lifetime();
            pass.set_viewport(10., 10., 32., 32., 0., 1.);
            draw.paint(
                egui::PaintCallbackInfo {
                    viewport,
                    clip_rect: viewport,
                    pixels_per_point: 1.,
                    screen_size_px: [64, 64],
                },
                &mut pass,
                &resources,
            );
        }
        let readback = device.create_buffer(&wgpu::BufferDescriptor {
            label: None,
            size: 64 * 256,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &readback,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(256),
                    rows_per_image: Some(64),
                },
            },
            wgpu::Extent3d {
                width: 64,
                height: 64,
                depth_or_array_layers: 1,
            },
        );
        queue.submit([encoder.finish()]);
        let (tx, rx) = std::sync::mpsc::channel();
        readback
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
        device.poll(wgpu::Maintain::Wait);
        rx.recv().unwrap().unwrap();
        let data = readback.slice(..).get_mapped_range();
        assert_eq!(
            &data[20 * 256 + 20 * 4..20 * 256 + 20 * 4 + 4],
            &[255, 0, 0, 255]
        );
        assert_eq!(&data[..4], &[0, 0, 0, 255]);
        drop(data);
        readback.unmap();
        // Empty geometry must neither allocate buffers nor paint.
        let empty = Draw {
            mesh: Arc::new(egui::Mesh::default()),
            viewport,
        };
        let mut encoder = device.create_command_encoder(&Default::default());
        empty.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        assert_eq!(resources.get::<Gpu>().unwrap().uploads, 1);
    }
    #[test]
    fn batched_lines_are_finite_indexed_and_empty_safe() {
        let mut mesh = egui::Mesh::default();
        line(
            &mut mesh,
            egui::pos2(0., 0.),
            egui::pos2(0., 0.),
            0.7,
            egui::Color32::WHITE,
            1.,
        );
        assert!(mesh.is_empty());
        line(
            &mut mesh,
            egui::pos2(0., 0.),
            egui::pos2(100., 50.),
            0.7,
            egui::Color32::WHITE,
            2.,
        );
        assert_eq!(mesh.vertices.len(), 8);
        assert_eq!(mesh.indices.len(), 18);
        assert!(mesh.is_valid());
        assert!(mesh.vertices.iter().all(|v| v.pos.is_finite()));
    }
}
