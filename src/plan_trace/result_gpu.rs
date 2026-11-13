//! Retained world-space 2D result fill. Camera changes only update a uniform.
use super::*;
use eframe::egui_wgpu::{self, wgpu};
use std::sync::Arc;
use wgpu::util::DeviceExt;
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 4],
    color: [f32; 4],
}
pub(super) struct Geometry {
    origin: Point,
    vertices: Vec<Vertex>,
}
impl Geometry {
    pub fn new(triangles: &[[(Point, Color32); 3]]) -> Self {
        let mut bounds = [
            f64::INFINITY,
            f64::INFINITY,
            f64::NEG_INFINITY,
            f64::NEG_INFINITY,
        ];
        for triangle in triangles {
            for (p, _) in triangle {
                if p.iter().all(|v| v.is_finite()) {
                    for j in 0..2 {
                        bounds[j] = bounds[j].min(p[j]);
                        bounds[j + 2] = bounds[j + 2].max(p[j]);
                    }
                }
            }
        }
        let origin = if bounds[0].is_finite() {
            [(bounds[0] + bounds[2]) * 0.5, (bounds[1] + bounds[3]) * 0.5]
        } else {
            [0.; 2]
        };
        let mut vertices = Vec::with_capacity(triangles.len() * 3);
        for triangle in triangles {
            let v = triangle.map(|(p, c)| Vertex {
                position: split_point([p[0] - origin[0], p[1] - origin[1]]),
                color: c.to_array().map(|x| x as f32 / 255.),
            });
            if v.iter().all(|v| v.position.iter().all(|p| p.is_finite())) {
                vertices.extend(v);
            }
        }
        Self { origin, vertices }
    }
    fn transform(&self, camera: Camera, rect: Rect) -> [[f32; 4]; 3] {
        let (s, c) = camera.angle.sin_cos();
        let sx = 2. * camera.zoom / rect.width() as f64;
        let sy = 2. * camera.zoom / rect.height() as f64;
        [
            [(sx * c) as f32, (-sx * s) as f32, 0., 0.],
            [(sy * s) as f32, (sy * c) as f32, 0., 0.],
            split_point([
                camera.center[0] - self.origin[0],
                camera.center[1] - self.origin[1],
            ]),
        ]
    }
}
// Preserve sub-millimetre precision at remote origins and high zoom without
// changing vertex data when the camera moves.
fn split_point(p: Point) -> [f32; 4] {
    let x = p[0] as f32;
    let y = p[1] as f32;
    [x, y, (p[0] - x as f64) as f32, (p[1] - y as f64) as f32]
}
struct Gpu {
    pipeline: wgpu::RenderPipeline,
    uniform: wgpu::Buffer,
    bind: wgpu::BindGroup,
    buffer: Option<wgpu::Buffer>,
    owner: Option<Arc<Geometry>>,
    uploads: usize,
}
struct Draw {
    geometry: Arc<Geometry>,
    matrix: [[f32; 4]; 3],
}
const SHADER: &str = r#"
struct Camera {x:vec4<f32>,y:vec4<f32>,center:vec4<f32>};
@group(0) @binding(0) var<uniform> camera:Camera;
struct Output {@builtin(position) position:vec4<f32>,@location(0) color:vec4<f32>};
@vertex fn vs_main(@location(0) p:vec4<f32>,@location(1) color:vec4<f32>)->Output {
 var o:Output;let delta=(p.xy-camera.center.xy)+(p.zw-camera.center.zw);let v=vec4<f32>(delta,0.0,0.0);o.position=vec4<f32>(dot(camera.x,v),dot(camera.y,v),0.0,1.0);o.color=color;return o;
}
@fragment fn fs_gamma(v:Output)->@location(0) vec4<f32>{return v.color;}
@fragment fn fs_linear(v:Output)->@location(0) vec4<f32>{
 let low=v.color.rgb/12.92;let high=pow((v.color.rgb+vec3<f32>(0.055))/1.055,vec3<f32>(2.4));
 return vec4<f32>(select(high,low,v.color.rgb<vec3<f32>(0.04045)),v.color.a);
}
"#;
fn new_gpu(device: &wgpu::Device, format: wgpu::TextureFormat) -> Gpu {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("world 2D results"),
        source: wgpu::ShaderSource::Wgsl(SHADER.into()),
    });
    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("retained world 2D results"),
        layout: None,
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            compilation_options: Default::default(),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: 32,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0=>Float32x4,1=>Float32x4],
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
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: false,
            depth_compare: wgpu::CompareFunction::Always,
            stencil: Default::default(),
            bias: Default::default(),
        }),
        multisample: Default::default(),
        multiview: None,
        cache: None,
    });
    let uniform = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("2D result camera"),
        size: 48,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("2D result camera bind"),
        layout: &pipeline.get_bind_group_layout(0),
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform.as_entire_binding(),
        }],
    });
    Gpu {
        pipeline,
        uniform,
        bind,
        buffer: None,
        owner: None,
        uploads: 0,
    }
}
pub(super) fn install(cc: &eframe::CreationContext<'_>) {
    if let Some(state) = &cc.wgpu_render_state {
        state
            .renderer
            .write()
            .callback_resources
            .insert(new_gpu(&state.device, state.target_format));
        cc.egui_ctx
            .data_mut(|d| d.insert_temp(egui::Id::new("slab_world_results_gpu"), true));
    }
}
/// Returns false for a CPU-only backend, where the caller keeps its clipped path.
pub(super) fn paint(
    painter: &egui::Painter,
    rect: Rect,
    geometry: Arc<Geometry>,
    camera: Camera,
) -> bool {
    if !painter.ctx().data(|d| {
        d.get_temp::<bool>(egui::Id::new("slab_world_results_gpu"))
            .unwrap_or(false)
    }) {
        return false;
    }
    if geometry.vertices.is_empty() || rect.width() <= 0. || rect.height() <= 0. {
        return true;
    }
    let matrix = geometry.transform(camera, rect);
    if matrix.iter().flatten().any(|v| !v.is_finite()) {
        return false;
    }
    painter.add(egui_wgpu::Callback::new_paint_callback(
        rect,
        Draw { geometry, matrix },
    ));
    true
}
impl egui_wgpu::CallbackTrait for Draw {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _: &egui_wgpu::ScreenDescriptor,
        _: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let Some(gpu) = resources.get_mut::<Gpu>() else {
            return vec![];
        };
        if self.geometry.vertices.is_empty() {
            return vec![];
        }
        if gpu
            .owner
            .as_ref()
            .is_none_or(|g| !Arc::ptr_eq(g, &self.geometry))
        {
            gpu.buffer = Some(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("world result vertices"),
                    contents: bytemuck::cast_slice(&self.geometry.vertices),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
            );
            gpu.owner = Some(self.geometry.clone());
            gpu.uploads += 1;
        }
        queue.write_buffer(&gpu.uniform, 0, bytemuck::cast_slice(&self.matrix));
        vec![]
    }
    fn paint(
        &self,
        _: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        if self.geometry.vertices.is_empty() {
            return;
        }
        let Some(gpu) = resources.get::<Gpu>() else {
            return;
        };
        let Some(buffer) = &gpu.buffer else {
            return;
        };
        pass.set_pipeline(&gpu.pipeline);
        pass.set_bind_group(0, &gpu.bind, &[]);
        pass.set_vertex_buffer(0, buffer.slice(..));
        pass.draw(0..self.geometry.vertices.len() as u32, 0..1);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "requires headless wgpu adapter"]
    fn result_world_gpu_reuses_uploads_for_pan_zoom_and_empty_scene() {
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
        let adapter =
            block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).unwrap();
        println!("adapter: {:?}", adapter.get_info());
        let (device, queue) =
            block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None)).unwrap();
        let format = wgpu::TextureFormat::Rgba8Unorm;
        let texture = device.create_texture(&wgpu::TextureDescriptor {
            label: None,
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
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: None,
            size: wgpu::Extent3d {
                width: 64,
                height: 64,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth24Plus,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth.create_view(&Default::default());
        let points = [
            [1e9, 1e9],
            [1e9 + 16., 1e9],
            [1e9 + 16., 1e9 + 16.],
            [1e9, 1e9 + 16.],
        ];
        let geometry = Arc::new(Geometry::new(
            &[[0, 1, 2], [0, 2, 3]].map(|t| t.map(|i| (points[i], Color32::RED))),
        ));
        let viewport = Rect::from_min_size(Pos2::ZERO, egui::vec2(64., 64.));
        let screen = egui_wgpu::ScreenDescriptor {
            size_in_pixels: [64, 64],
            pixels_per_point: 1.,
        };
        let mut resources = egui_wgpu::CallbackResources::default();
        resources.insert(new_gpu(&device, format));
        for (center, zoom, pixel) in [
            ([1e9, 1e9], 1., Some([40, 24])),
            ([1e9 + 16., 1e9], 1., Some([24, 24])),
            ([1e9 + 8., 1e9 + 8.], 2., Some([20, 20])),
            ([1e9 + 1000., 1e9 + 1000.], 1., None),
        ] {
            let draw = Draw {
                geometry: geometry.clone(),
                matrix: geometry.transform(
                    Camera {
                        center,
                        zoom,
                        angle: 0.,
                        plane: None,
                    },
                    viewport,
                ),
            };
            let mut encoder = device.create_command_encoder(&Default::default());
            draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            assert_eq!(resources.get::<Gpu>().unwrap().uploads, 1);
            {
                let mut pass = encoder
                    .begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: None,
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                                store: wgpu::StoreOp::Store,
                            },
                        })],
                        depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                            view: &depth_view,
                            depth_ops: Some(wgpu::Operations {
                                load: wgpu::LoadOp::Clear(1.),
                                store: wgpu::StoreOp::Store,
                            }),
                            stencil_ops: None,
                        }),
                        timestamp_writes: None,
                        occlusion_query_set: None,
                    })
                    .forget_lifetime();
                pass.set_viewport(0., 0., 64., 64., 0., 1.);
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
            let buffer = device.create_buffer(&wgpu::BufferDescriptor {
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
                    buffer: &buffer,
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
            buffer
                .slice(..)
                .map_async(wgpu::MapMode::Read, move |r| tx.send(r).unwrap());
            device.poll(wgpu::Maintain::Wait);
            rx.recv().unwrap().unwrap();
            let data = buffer.slice(..).get_mapped_range();
            if let Some([x, y]) = pixel {
                let at = y * 256 + x * 4;
                assert_eq!(&data[at..at + 4], &[255, 0, 0, 255]);
            } else {
                assert!(data.chunks_exact(4).all(|p| p == [0, 0, 0, 255]));
            }
            drop(data);
            buffer.unmap();
        }
        let empty = Draw {
            geometry: Arc::new(Geometry::new(&[])),
            matrix: [[0.; 4]; 3],
        };
        let mut encoder = device.create_command_encoder(&Default::default());
        empty.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        assert_eq!(resources.get::<Gpu>().unwrap().uploads, 1);
        let replacement = Arc::new(Geometry::new(&[[
            [0, 1, 2].map(|i| (points[i], Color32::BLUE))
        ][0]]));
        let draw = Draw {
            geometry: replacement.clone(),
            matrix: replacement.transform(Camera::default(), viewport),
        };
        draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        assert_eq!(resources.get::<Gpu>().unwrap().uploads, 2);
    }
    #[test]
    fn retained_transform_matches_camera_with_remote_origin_zoom_and_pan() {
        let points = [
            [1e9 + 123.125, -2e9 + 80.0625],
            [1e9 + 17000., -2e9],
            [1e9, -2e9 + 18000.],
        ];
        let geometry = Geometry::new(&[points.map(|p| (p, Color32::WHITE))]);
        let rect = Rect::from_min_size(egui::pos2(315., 75.), egui::vec2(900., 650.));
        for (zoom, angle) in [(0.03, 0.), (1000., 0.), (0.4, 0.6)] {
            let camera = Camera {
                center: [points[0][0] + 0.037, points[0][1] - 0.073],
                zoom,
                angle,
                plane: None,
            };
            let matrix = geometry.transform(camera, rect);
            let p = geometry.vertices[0].position;
            let delta = [
                (p[0] - matrix[2][0]) + (p[2] - matrix[2][2]),
                (p[1] - matrix[2][1]) + (p[3] - matrix[2][3]),
            ];
            let ndc = matrix[..2]
                .iter()
                .map(|row| row[0] * delta[0] + row[1] * delta[1])
                .collect::<Vec<_>>();
            let screen = [
                rect.center().x as f64 + ndc[0] as f64 * rect.width() as f64 / 2.,
                rect.center().y as f64 - ndc[1] as f64 * rect.height() as f64 / 2.,
            ];
            let expected = camera.screen64(points[0], rect);
            assert!(
                (screen[0] - expected[0]).abs() < 0.002 && (screen[1] - expected[1]).abs() < 0.002
            );
        }
        assert!(Geometry::new(&[]).vertices.is_empty());
        assert!(Geometry::new(&[[([f64::NAN, 0.], Color32::WHITE); 3]])
            .vertices
            .is_empty());
    }
}
