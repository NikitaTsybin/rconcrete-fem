//! Raster/vector underlays on their storey plane, using the scene depth buffer.
use super::*;
use eframe::egui_wgpu::{self, wgpu};
use std::sync::Arc;
use wgpu::util::DeviceExt;
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    p: [f32; 3],
    uv: [f32; 2],
}
struct Page {
    image: Arc<Vec<u8>>,
    bind: wgpu::BindGroup,
    uniform: wgpu::Buffer,
    vertices: wgpu::Buffer,
}
struct Gpu {
    format: wgpu::TextureFormat,
    pipeline: Option<wgpu::RenderPipeline>,
    pages: BTreeMap<u64, Page>,
}
struct Draw {
    clips: super::sections::ClipPlanes,
    live: Arc<Vec<u64>>,
    id: u64,
    image: Arc<Vec<u8>>,
    size: [u32; 2],
    vertices: [Vertex; 6],
    opacity: f32,
}
const SHADER: &str = r#"
@group(0) @binding(0) var image:texture_2d<f32>;
@group(0) @binding(1) var filtering:sampler;
struct Params {alpha:vec4<f32>,planes:array<vec4<f32>,12>};
@group(0) @binding(2) var<uniform> params:Params;
struct Output { @builtin(position) p:vec4<f32>, @location(0) uv:vec2<f32>, @location(1) clip_position:vec4<f32> };
@vertex fn vs_main(@location(0) p:vec3<f32>,@location(1) uv:vec2<f32>)->Output {
 var o:Output;o.p=vec4<f32>(p,1.0);o.uv=uv;o.clip_position=o.p;return o;
}
fn clipped(p:vec4<f32>)->bool {
    for(var i=0u;i<12u;i++) {
        let plane=params.planes[i];
        if all(plane.xyz==vec3<f32>(0.0)) {break;}
        if dot(plane,p)<-0.000001 {return true;}
    }
    return false;
}
@fragment fn fs_gamma(v:Output)->@location(0) vec4<f32> {
 if clipped(v.clip_position) {discard;}
 let c=textureSample(image,filtering,v.uv);let a=c.a*params.alpha.x;
 if a<0.001 { discard; } return vec4<f32>(c.rgb*a,a);
}
@fragment fn fs_linear(v:Output)->@location(0) vec4<f32> {
 if clipped(v.clip_position) {discard;}
 let c=textureSample(image,filtering,v.uv);let a=c.a*params.alpha.x;
 if a<0.001 { discard; }
 let low=c.rgb/12.92;let high=pow((c.rgb+vec3<f32>(0.055))/1.055,vec3<f32>(2.4));
 return vec4<f32>(select(high,low,c.rgb<vec3<f32>(0.04045))*a,a);
}
"#;
#[cfg(test)]
pub(super) fn validate_shader(device: &wgpu::Device) {
    let _ = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("underlay clip shader test"),
        source: wgpu::ShaderSource::Wgsl(SHADER.into()),
    });
}
pub(super) fn install(cc: &eframe::CreationContext<'_>) {
    if let Some(s) = &cc.wgpu_render_state {
        s.renderer.write().callback_resources.insert(Gpu {
            format: s.target_format,
            pipeline: None,
            pages: BTreeMap::new(),
        });
    }
}
impl Draw {
    fn uniform_values(&self) -> [[f32; 4]; 1 + super::sections::MAX_PLANES] {
        let mut values = [[0.; 4]; 1 + super::sections::MAX_PLANES];
        values[0][0] = self.opacity;
        values[1..].copy_from_slice(&self.clips);
        values
    }
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
        gpu.pages.retain(|id, _| self.live.contains(id));
        let pipeline = gpu.pipeline.get_or_insert_with(|| {
            let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("3D PDF"),
                source: wgpu::ShaderSource::Wgsl(SHADER.into()),
            });
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("3D PDF depth"),
                layout: None,
                vertex: wgpu::VertexState {
                    module: &shader,
                    entry_point: Some("vs_main"),
                    compilation_options: Default::default(),
                    buffers: &[wgpu::VertexBufferLayout {
                        array_stride: 20,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x2],
                    }],
                },
                fragment: Some(wgpu::FragmentState {
                    module: &shader,
                    entry_point: Some(if gpu.format.is_srgb() {
                        "fs_linear"
                    } else {
                        "fs_gamma"
                    }),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState {
                        format: gpu.format,
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
                    depth_compare: wgpu::CompareFunction::LessEqual,
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                multiview: None,
                cache: None,
            })
        });
        if gpu
            .pages
            .get(&self.id)
            .is_none_or(|p| !Arc::ptr_eq(&p.image, &self.image))
        {
            // Keep GPU images only for pages used recently; independent page buffers avoid prepare/paint aliasing.
            let texture = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("3D PDF image"),
                size: wgpu::Extent3d {
                    width: self.size[0],
                    height: self.size[1],
                    depth_or_array_layers: 1,
                },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8Unorm,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            queue.write_texture(
                wgpu::TexelCopyTextureInfo {
                    texture: &texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                &self.image,
                wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(self.size[0] * 4),
                    rows_per_image: Some(self.size[1]),
                },
                wgpu::Extent3d {
                    width: self.size[0],
                    height: self.size[1],
                    depth_or_array_layers: 1,
                },
            );
            let view = texture.create_view(&Default::default());
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                ..Default::default()
            });
            let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("PDF opacity"),
                contents: bytemuck::cast_slice(&self.uniform_values()),
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            });
            let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("3D PDF bind"),
                layout: &pipeline.get_bind_group_layout(0),
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&sampler),
                    },
                    wgpu::BindGroupEntry {
                        binding: 2,
                        resource: uniform.as_entire_binding(),
                    },
                ],
            });
            let vertices = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("PDF quad"),
                size: 120,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            gpu.pages.insert(
                self.id,
                Page {
                    image: self.image.clone(),
                    bind,
                    uniform,
                    vertices,
                },
            );
        }
        if let Some(page) = gpu.pages.get(&self.id) {
            queue.write_buffer(
                &page.uniform,
                0,
                bytemuck::cast_slice(&self.uniform_values()),
            );
            queue.write_buffer(&page.vertices, 0, bytemuck::cast_slice(&self.vertices));
        }
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
        if let (Some(pipeline), Some(page)) = (&gpu.pipeline, gpu.pages.get(&self.id)) {
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &page.bind, &[]);
            pass.set_vertex_buffer(0, page.vertices.slice(..));
            pass.draw(0..6, 0..1);
        }
    }
}
impl App {
    pub(super) fn spatial_underlays(
        &self,
        painter: &egui::Painter,
        rect: Rect,
        project: &impl Fn([f64; 3]) -> (Pos2, f64),
        projected: &[(Pos2, f64)],
        scene: &crate::plan_trace::fem::shell::Scene,
    ) {
        if !self.show_dxf {
            return;
        }
        let near = projected
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max);
        let far = projected.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        let span = (near - far).max(1.);
        let live = Arc::new(self.underlays.iter().map(|u| u.id).collect::<Vec<_>>());
        self.spatial
            .pdf_sources
            .borrow_mut()
            .retain(|id, _| live.contains(id));
        let mut vectors = spatial_depth::Builder::new(rect, projected);
        let mut occlusion = None;
        for u in self
            .underlays
            .iter()
            .filter(|u| u.visible && u.transparency < 1. && !self.underlay_hidden(u.id))
        {
            let z = if self.building.enabled {
                let floor = self
                    .building
                    .underlay_floors
                    .get(&u.id)
                    .copied()
                    .unwrap_or(self.building.floors[0].id);
                self.building.floor(floor).elevation_mm
            } else {
                0.
            };
            let selected =
                self.underlay_selection.contains(&u.id) || self.selected_underlay == Some(u.id);
            let hover = self.spatial.underlay_hover == Some(u.id)
                && self.mode.is_none()
                && self.clipboard_op.is_none()
                && self.scaling_op.is_none();
            if self.underlay_selectable(u) && (selected || hover) {
                if let Some(q) = self.underlay_corners(u.id) {
                    let q = q.map(|p| project([p[0], p[1], z]));
                    for i in 0..4 {
                        vectors.line_on_face(
                            [q[i], q[(i + 1) % 4]],
                            1.5,
                            if selected {
                                Color32::YELLOW
                            } else {
                                Color32::LIGHT_BLUE
                            },
                            [q[0], q[1], q[2]],
                        );
                    }
                }
            }
            match &u.data {
                crate::plan_trace::underlay::Data::Pdf {
                    width,
                    height,
                    rgba,
                    size_mm,
                    ..
                } => {
                    let corners =
                        [[0., 0.], [size_mm[0], 0.], *size_mm, [0., size_mm[1]]].map(|p| {
                            let q = u.point(p);
                            project([q[0], q[1], z])
                        });
                    if !Rect::from_points(&corners.map(|p| p.0)).intersects(rect) {
                        continue;
                    }
                    let mut sources = self.spatial.pdf_sources.borrow_mut();
                    let token = rgba.as_ptr() as usize;
                    let cached = sources
                        .entry(u.id)
                        .or_insert_with(|| (token, Arc::new(rgba.clone())));
                    if cached.0 != token {
                        *cached = (token, Arc::new(rgba.clone()));
                    }
                    let image = cached.1.clone();
                    let uv = [[0., 1.], [1., 1.], [1., 0.], [0., 0.]];
                    let vertices = [0, 1, 2, 0, 2, 3].map(|i| {
                        let (p, d) = corners[i];
                        Vertex {
                            p: [
                                (p.x - rect.left()) / rect.width() * 2. - 1.,
                                1. - (p.y - rect.top()) / rect.height() * 2.,
                                (((near - d) / span) as f32 * 0.98 + 0.01 - 0.000005).clamp(0., 1.),
                            ],
                            uv: uv[i],
                        }
                    });
                    painter.add(egui_wgpu::Callback::new_paint_callback(
                        rect,
                        Draw {
                            clips: spatial_depth::clips(),
                            live: live.clone(),
                            id: u.id,
                            image,
                            size: [*width, *height],
                            vertices,
                            opacity: 1. - u.transparency,
                        },
                    ));
                }
                crate::plan_trace::underlay::Data::Dxf(_) => {
                    let Some(d) = u.drawing() else { continue };
                    let ink = |layer: &str, rgb: [u8; 3]| {
                        let style = self.layers.get(layer);
                        if style.is_some_and(|s| !s.visible) {
                            None
                        } else {
                            Some(
                                style
                                    .map_or(Color32::from_rgb(rgb[0], rgb[1], rgb[2]), |s| {
                                        s.ink(rgb)
                                    })
                                    .gamma_multiply(1. - u.transparency),
                            )
                        }
                    };
                    for path in &d.paths {
                        let Some(color) = ink(&path.layer, path.color.rgb()) else {
                            continue;
                        };
                        let width = self.layers.get(&path.layer).map_or(1., |s| s.width);
                        for (a, b) in path
                            .points
                            .iter()
                            .zip(path.points.iter().cycle().skip(1))
                            .take(if path.closed {
                                path.points.len()
                            } else {
                                path.points.len().saturating_sub(1)
                            })
                        {
                            vectors.line(
                                [project([a[0], a[1], z]), project([b[0], b[1], z])],
                                width,
                                color,
                            );
                        }
                    }
                    for (triangles, rgb, layer) in &d.fills {
                        if let Some(color) = ink(layer, rgb.rgb()) {
                            for t in triangles {
                                vectors.triangle(
                                    t.map(|p| project([p[0], p[1], z])),
                                    color,
                                    0.000005,
                                );
                            }
                        }
                    }
                    if self.show_text {
                        let occ = occlusion.get_or_insert_with(|| {
                            spatial_results::Occlusion::new_visible(rect, scene, projected, self)
                        });
                        for label in &d.labels {
                            let Some(color) = ink(&label.layer, label.color.rgb()) else {
                                continue;
                            };
                            let p = project([label.point[0], label.point[1], z]);
                            if !occ.visible(p.0, p.1) {
                                continue;
                            }
                            let a = label.angle;
                            let end = project([
                                label.point[0] + a.cos() * label.height,
                                label.point[1] + a.sin() * label.height,
                                z,
                            ])
                            .0;
                            let size = (end - p.0).length().clamp(1., 200.);
                            let galley = painter.layout_no_wrap(
                                label.text.clone(),
                                egui::FontId::proportional(size),
                                color,
                            );
                            let mut text = egui::epaint::TextShape::new(p.0, galley, color);
                            text.angle = (end - p.0).angle();
                            painter.add(text);
                        }
                    }
                }
            }
        }
        spatial_depth::paint_layer(
            painter,
            rect,
            Arc::new(vectors.vertices),
            spatial_depth::Layer::Underlay,
        );
    }
}
