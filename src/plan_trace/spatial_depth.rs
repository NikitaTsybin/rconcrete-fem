//! GPU depth-tested source geometry. Independent of the calculation mesh.
use super::sections::{ClipPlanes, MAX_PLANES, NO_CLIPS};
use super::*;
use eframe::egui_wgpu::{self, wgpu};
use std::sync::Arc;
use wgpu::util::DeviceExt;
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub(super) struct Vertex {
    pub(super) position: [f32; 3],
    pub(super) color: [f32; 4],
    pub(super) offset: [f32; 3],
}
struct Gpu {
    pipeline: wgpu::RenderPipeline,
    uniform: wgpu::Buffer,
    bind: wgpu::BindGroup,
    buffer: Option<wgpu::Buffer>,
    chunks: Vec<(wgpu::Buffer, u32)>,
    owner: Option<Arc<Vec<Vertex>>>,
    #[cfg(test)]
    uploads: usize,
}
/// Each layer has its own retained buffer. Egui prepares all callbacks
/// before painting any of them, so simultaneous layers cannot share a slot.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub(super) enum Layer {
    Main,
    Underlay,
    Construction,
    Editing,
    Summation,
    Axes,
    Columns,
    EffectiveLengths,
    Foundation,
    Intersections,
    Restraints,
    Ground,
    GroundOpaque,
    StructureEdges,
    SelectedElements,
    Measurements,
}
impl Layer {
    const OVERLAYS: [Self; 15] = [
        Self::Underlay,
        Self::Construction,
        Self::Editing,
        Self::Summation,
        Self::Axes,
        Self::Columns,
        Self::EffectiveLengths,
        Self::Foundation,
        Self::Intersections,
        Self::Restraints,
        Self::Ground,
        Self::GroundOpaque,
        Self::StructureEdges,
        Self::SelectedElements,
        Self::Measurements,
    ];
}
struct Draw {
    clips: ClipPlanes,
    lane: Layer,
    transform: [f32; 4],
    vertices: Arc<Vec<Vertex>>,
}
const SHADER: &str = r#"
struct ScreenCamera { transform:vec4<f32>, planes:array<vec4<f32>,12> };
@group(0) @binding(0) var<uniform> camera:ScreenCamera;
struct Output { @builtin(position) position:vec4<f32>, @location(0) color:vec4<f32>, @location(1) clip_position:vec4<f32> };
@vertex fn vs_main(@location(0) p:vec3<f32>,@location(1) c:vec4<f32>,@location(2) offset:vec3<f32>)->Output {
    var o:Output;o.position=vec4<f32>(p.xy*camera.transform.xy+camera.transform.zw+offset.xy,p.z+offset.z/camera.transform.x,1.0);o.color=c;o.clip_position=o.position;return o;
}
fn clipped(p:vec4<f32>)->bool {
    for(var i=0u;i<12u;i++) {
        let plane=camera.planes[i];
        if all(plane.xyz==vec3<f32>(0.0)) {break;}
        if dot(plane,p)<-0.000001 {return true;}
    }
    return false;
}
@fragment fn fs_gamma(v:Output)->@location(0) vec4<f32>{if clipped(v.clip_position) {discard;}return v.color;}
@fragment fn fs_linear(v:Output)->@location(0) vec4<f32>{
    if clipped(v.clip_position) {discard;}
    let alpha=v.color.a;
    let rgb=v.color.rgb/max(alpha,0.000001);
    let low=rgb/12.92;
    let high=pow((rgb+vec3<f32>(0.055))/1.055,vec3<f32>(2.4));
    return vec4<f32>(select(high,low,rgb<vec3<f32>(0.04045))*alpha,alpha);
}
"#;
thread_local! {static CLIPS:std::cell::Cell<ClipPlanes>=const{std::cell::Cell::new(NO_CLIPS)};}
pub(super) struct ClipGuard(ClipPlanes);
impl Drop for ClipGuard {
    fn drop(&mut self) {
        CLIPS.with(|c| c.set(self.0));
    }
}
pub(super) fn clip_guard() -> ClipGuard {
    ClipGuard(CLIPS.with(|c| c.replace(NO_CLIPS)))
}
pub(super) fn set_clips(v: ClipPlanes) {
    CLIPS.with(|c| c.set(v));
}
pub(super) fn clips() -> ClipPlanes {
    CLIPS.with(|c| c.get())
}
thread_local! {static CLIP_DEPTH:std::cell::Cell<Option<(Rect,f64,f64)>>=const{std::cell::Cell::new(None)};}
pub(super) fn set_clip_depth(rect: Rect, p: &[(Pos2, f64)]) {
    let near = p.iter().map(|p| p.1).fold(f64::NEG_INFINITY, f64::max);
    let far = p.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
    CLIP_DEPTH.with(|c| c.set(Some((rect, near, (near - far).max(1.)))));
}
pub(super) fn point_visible(p: Pos2, depth: f64) -> bool {
    let Some((r, near, span)) = CLIP_DEPTH.with(|c| c.get()) else {
        return true;
    };
    let p = [
        (p.x - r.left()) / r.width() * 2. - 1.,
        1. - (p.y - r.top()) / r.height() * 2.,
        ((near - depth) / span * 0.98 + 0.01) as f32,
        1.,
    ];
    clips()
        .iter()
        .all(|c| c.iter().zip(p).map(|(a, b)| a * b).sum::<f32>() >= -1e-6)
}
pub(super) fn clip_shape(mut shape: Vec<(Pos2, f64)>) -> Vec<(Pos2, f64)> {
    let Some((r, near, span)) = CLIP_DEPTH.with(|c| c.get()) else {
        return shape;
    };
    let closed = shape.len() > 2;
    if closed && shape.first() == shape.last() {
        shape.pop();
    }
    for plane in clips().into_iter().take_while(|p| p[..3] != [0.; 3]) {
        let distance = |p: (Pos2, f64)| {
            let p = [
                (p.0.x - r.left()) / r.width() * 2. - 1.,
                1. - (p.0.y - r.top()) / r.height() * 2.,
                ((near - p.1) / span * 0.98 + 0.01) as f32,
                1.,
            ];
            plane.iter().zip(p).map(|(a, b)| a * b).sum::<f32>()
        };
        if shape.is_empty() {
            break;
        }
        if shape.len() == 1 {
            if distance(shape[0]) < 0. {
                shape.clear();
            }
            continue;
        }
        if !closed {
            let [a, b] = [shape[0], shape[1]];
            let da = distance(a);
            let db = distance(b);
            if da < 0. && db < 0. {
                shape.clear();
            } else if (da < 0.) != (db < 0.) {
                let t = da / (da - db);
                let p = (a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t as f64);
                if da < 0. {
                    shape[0] = p;
                } else {
                    shape[1] = p;
                }
            }
            continue;
        }
        let mut out = vec![];
        let mut a = *shape.last().unwrap();
        let mut da = distance(a);
        for &b in &shape {
            let db = distance(b);
            if (da < 0.) != (db < 0.) {
                let t = da / (da - db);
                out.push((a.0 + (b.0 - a.0) * t, a.1 + (b.1 - a.1) * t as f64));
            }
            if db >= 0. {
                out.push(b);
            }
            a = b;
            da = db;
        }
        shape = out;
    }
    if closed && !shape.is_empty() {
        shape.push(shape[0]);
    }
    shape
}
pub(super) fn install(cc: &eframe::CreationContext<'_>) {
    spatial_underlays::install(cc);
    spatial_points::install(cc);
    let Some(state) = &cc.wgpu_render_state else {
        return;
    };
    #[cfg(not(target_arch = "wasm32"))]
    {
        let adapter = format!("{:?}", state.adapter.get_info());
        gpu_diagnostic(&format!(
            "start pid={} adapter={adapter}",
            std::process::id()
        ));
        state
            .device
            .set_device_lost_callback(move |reason, message| {
                gpu_diagnostic(&format!(
                    "device lost: {reason:?}: {message}; adapter={adapter}"
                ));
            });
    }
    state
        .renderer
        .write()
        .callback_resources
        .insert(new_gpu(&state.device, state.target_format));
    state.renderer.write().callback_resources.insert(ExtraGpu(
        Layer::OVERLAYS
            .into_iter()
            .map(|id| {
                (
                    id,
                    make_gpu(
                        &state.device,
                        state.target_format,
                        SHADER,
                        &[0.; (1 + MAX_PLANES) * 4],
                        id != Layer::GroundOpaque,
                    ),
                )
            })
            .collect(),
    ));
    state
        .renderer
        .write()
        .callback_resources
        .insert(new_world_gpu(&state.device, state.target_format));
    state
        .renderer
        .write()
        .callback_resources
        .insert(DeformationGpu(new_world_gpu(
            &state.device,
            state.target_format,
        )));
}
fn new_world_gpu(device: &wgpu::Device, format: wgpu::TextureFormat) -> WorldGpu {
    let shader=SHADER
        .replace("@group(0) @binding(0) var<uniform> camera:ScreenCamera;", "struct WorldCamera { matrix:mat4x4<f32>, edge:vec4<f32>,planes:array<vec4<f32>,12> }; @group(0) @binding(0) var<uniform> camera:WorldCamera;")
        .replace("vec4<f32>(p.xy*camera.transform.xy+camera.transform.zw+offset.xy,p.z+offset.z/camera.transform.x,1.0)","world_position(p,offset)");
    let shader = format!(
        r#"
fn world_position(p:vec3<f32>, offset:vec3<f32>)->vec4<f32> {{
    if camera.edge.w > 0.5 {{
        return camera.matrix*vec4<f32>(p+offset*camera.edge.x,1.0);
    }}
    var clip=camera.matrix*vec4<f32>(p,1.0);
    let width=length(offset);
    if width>0.0 {{
        let tangent=(camera.matrix*vec4<f32>(offset,0.0)).xy / max(camera.edge.yz,vec2<f32>(0.00000001));
        let length_px=length(tangent);
        // Unit tangents retain the original physical-pixel FE width.
        // Source ribbons encode a logical-pixel width in the tangent length.
        let source=width>1.25;
        let thickness=select(1.0,width*camera.edge.x/1.2,source);
        if length_px>0.000001 {{
            clip.x+=-tangent.y/length_px*camera.edge.y*thickness;
            clip.y+=tangent.x/length_px*camera.edge.z*thickness;
        }}
        clip.z-=select(0.00002,0.00001,source);
    }}
    return clip;
}}
{shader}
"#,
        shader = shader
    );
    let filtered = |condition: &str| {
        shader
            .replace(
                "return v.color;",
                &format!("if {condition} {{discard;}}return v.color;"),
            )
            .replace(
                "let alpha=v.color.a",
                &format!("if {condition} {{discard;}}\n let alpha=v.color.a"),
            )
    };
    let opaque = make_gpu(
        device,
        format,
        &filtered("v.color.a < 0.999"),
        &[0.; (5 + MAX_PLANES) * 4],
        false,
    );
    let transparent = make_gpu(
        device,
        format,
        &filtered("v.color.a >= 0.999 || v.color.a <= 0.0001"),
        &[0.; (5 + MAX_PLANES) * 4],
        true,
    );
    WorldGpu(opaque, transparent, EdgeBuffers::default())
}

fn new_gpu(device: &wgpu::Device, format: wgpu::TextureFormat) -> Gpu {
    make_gpu(device, format, SHADER, &[0.; (1 + MAX_PLANES) * 4], false)
}
fn make_gpu(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    shader: &str,
    camera: &[f32],
    translucent: bool,
) -> Gpu {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("3D depth shader"),
        source: wgpu::ShaderSource::Wgsl(shader.into()),
    });
    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("3D source depth"),
        layout: None,
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            compilation_options: Default::default(),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: 40,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x4,2=>Float32x3],
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
                format: format,
                blend: if translucent {
                    Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING)
                } else {
                    None
                },
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: !translucent,
            depth_compare: wgpu::CompareFunction::LessEqual,
            stencil: Default::default(),
            bias: Default::default(),
        }),
        multisample: Default::default(),
        multiview: None,
        cache: None,
    });
    let uniform = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("3D camera"),
        contents: bytemuck::cast_slice(camera),
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    });
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("3D camera bind"),
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
        chunks: vec![],
        owner: None,
        #[cfg(test)]
        uploads: 0,
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
        let gpu = if self.lane == Layer::Main {
            resources.get_mut::<Gpu>()
        } else {
            resources
                .get_mut::<ExtraGpu>()
                .and_then(|r| r.0.get_mut(&self.lane))
        };
        let Some(gpu) = gpu else {
            return vec![];
        };
        if self.vertices.is_empty() {
            return vec![];
        }
        let mut camera = [[0.; 4]; 1 + MAX_PLANES];
        camera[0] = self.transform;
        camera[1..].copy_from_slice(&self.clips);
        queue.write_buffer(&gpu.uniform, 0, bytemuck::cast_slice(&camera));
        if gpu
            .owner
            .as_ref()
            .is_none_or(|owner| !Arc::ptr_eq(owner, &self.vertices))
        {
            gpu.buffer = Some(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("3D depth vertices"),
                    contents: bytemuck::cast_slice(self.vertices.as_slice()),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
            );
            gpu.owner = Some(self.vertices.clone());
            #[cfg(test)]
            {
                gpu.uploads += 1;
            }
        }
        vec![]
    }
    fn paint(
        &self,
        _: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        if self.vertices.is_empty() {
            return;
        }
        let gpu = if self.lane == Layer::Main {
            resources.get::<Gpu>()
        } else {
            resources
                .get::<ExtraGpu>()
                .and_then(|r| r.0.get(&self.lane))
        };
        let Some(gpu) = gpu else {
            return;
        };
        // A duplicate submission must never draw using another callback's buffer.
        if !gpu
            .owner
            .as_ref()
            .is_some_and(|owner| Arc::ptr_eq(owner, &self.vertices))
        {
            return;
        }
        let Some(buffer) = &gpu.buffer else {
            return;
        };
        pass.set_pipeline(&gpu.pipeline);
        pass.set_bind_group(0, &gpu.bind, &[]);
        pass.set_vertex_buffer(0, buffer.slice(..));
        pass.draw(0..self.vertices.len() as u32, 0..1);
    }
}
pub(super) fn paint(painter: &egui::Painter, rect: Rect, vertices: Arc<Vec<Vertex>>) {
    paint_transformed(painter, rect, vertices, [1., 1., 0., 0.]);
}
pub(super) fn paint_transformed(
    painter: &egui::Painter,
    rect: Rect,
    vertices: Arc<Vec<Vertex>>,
    transform: [f32; 4],
) {
    if !vertices.is_empty() && rect.is_positive() {
        painter.add(egui_wgpu::Callback::new_paint_callback(
            rect,
            Draw {
                clips: clips(),
                lane: Layer::Main,
                vertices,
                transform,
            },
        ));
    }
}
pub(super) fn paint_layer(
    painter: &egui::Painter,
    rect: Rect,
    vertices: Arc<Vec<Vertex>>,
    lane: Layer,
) {
    if !vertices.is_empty() && rect.is_positive() {
        painter.add(egui_wgpu::Callback::new_paint_callback(
            rect,
            Draw {
                clips: clips(),
                lane,
                vertices,
                transform: [1., 1., 0., 0.],
            },
        ));
    }
}
// Independent buffer/uniforms: overlays and world surfaces prepare before either
// paint callback runs. Sharing their resources would overwrite one draw's data.
struct WorldGpu(Gpu, Gpu, EdgeBuffers);
#[derive(Default)]
struct EdgeBuffers {
    owner: Option<Arc<Vec<[Vertex; 4]>>>,
    vertices: Vec<(wgpu::Buffer, u32)>,
    indices: Option<wgpu::Buffer>,
}
impl EdgeBuffers {
    fn prepare(&mut self, device: &wgpu::Device, edges: &Arc<Vec<[Vertex; 4]>>) {
        if self
            .owner
            .as_ref()
            .is_some_and(|old| Arc::ptr_eq(old, edges))
        {
            return;
        }
        self.vertices.clear();
        let per_chunk = (device.limits().max_buffer_size as usize
            / std::mem::size_of::<[Vertex; 4]>())
        .min(16384);
        if let Some(count) = (!edges.is_empty()).then_some(edges.len().min(per_chunk)) {
            let indices: Vec<u16> = (0..count)
                .flat_map(|i| {
                    let a = (4 * i) as u16;
                    [a, a + 1, a + 2, a, a + 2, a + 3]
                })
                .collect();
            self.indices = Some(
                device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("FE ribbon indices"),
                    contents: bytemuck::cast_slice(&indices),
                    usage: wgpu::BufferUsages::INDEX,
                }),
            );
            for chunk in edges.chunks(per_chunk) {
                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("FE ribbon vertices"),
                    contents: bytemuck::cast_slice(chunk),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                self.vertices.push((buffer, (chunk.len() * 6) as u32));
            }
        } else {
            self.indices = None;
        }
        self.owner = Some(edges.clone());
    }
}

struct DeformationGpu(WorldGpu);
struct ExtraGpu(BTreeMap<Layer, Gpu>);
struct WorldDraw {
    clips: ClipPlanes,
    deformation: bool,
    edge_scale: f32,
    pixel: [f32; 2],
    vertices: Arc<Vec<Vertex>>,
    edges: Option<Arc<Vec<[Vertex; 4]>>>,
    matrix: [[f32; 4]; 4],
}
impl egui_wgpu::CallbackTrait for WorldDraw {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _: &egui_wgpu::ScreenDescriptor,
        _: &mut wgpu::CommandEncoder,
        resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        let gpu = if self.deformation {
            resources.get_mut::<DeformationGpu>().map(|r| &mut r.0)
        } else {
            resources.get_mut::<WorldGpu>()
        };
        let Some(WorldGpu(gpu, transparent, edges)) = gpu else {
            return vec![];
        };
        if let Some(geometry) = &self.edges {
            edges.prepare(device, geometry);
        }
        if self.vertices.is_empty() && self.edges.as_ref().is_none_or(|e| e.is_empty()) {
            return vec![];
        }
        let mut camera = [[0_f32; 4]; 5 + MAX_PLANES];
        camera[..4].copy_from_slice(&self.matrix);
        camera[4][0] = self.edge_scale;
        camera[4][1] = self.pixel[0];
        camera[4][2] = self.pixel[1];
        camera[4][3] = self.deformation as u8 as f32;
        camera[5..].copy_from_slice(&self.clips);
        queue.write_buffer(&gpu.uniform, 0, bytemuck::cast_slice(&camera));
        queue.write_buffer(&transparent.uniform, 0, bytemuck::cast_slice(&camera));
        if gpu
            .owner
            .as_ref()
            .is_none_or(|old| !Arc::ptr_eq(old, &self.vertices))
        {
            gpu.chunks.clear();
            // Never assume the adapter's max_buffer_size can hold the entire scene.
            for chunk in world_vertex_chunks(&self.vertices, device.limits().max_buffer_size) {
                let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("3D world isofields chunk"),
                    contents: bytemuck::cast_slice(chunk),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                gpu.chunks.push((buffer, chunk.len() as u32));
            }
            gpu.owner = Some(self.vertices.clone());
            #[cfg(test)]
            {
                gpu.uploads += 1;
            }
        }
        vec![]
    }
    fn paint(
        &self,
        _: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        if self.vertices.is_empty() && self.edges.as_ref().is_none_or(|e| e.is_empty()) {
            return;
        }
        let gpu = if self.deformation {
            resources.get::<DeformationGpu>().map(|r| &r.0)
        } else {
            resources.get::<WorldGpu>()
        };
        let Some(WorldGpu(gpu, transparent, edges)) = gpu else {
            return;
        };
        pass.set_pipeline(&gpu.pipeline);
        pass.set_bind_group(0, &gpu.bind, &[]);
        for (buffer, count) in &gpu.chunks {
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..*count, 0..1);
        }
        if let Some(indices) = edges.indices.as_ref().filter(|_| self.edges.is_some()) {
            pass.set_index_buffer(indices.slice(..), wgpu::IndexFormat::Uint16);
            for (buffer, count) in &edges.vertices {
                pass.set_vertex_buffer(0, buffer.slice(..));
                pass.draw_indexed(0..*count, 0, 0..1);
            }
        }
        pass.set_pipeline(&transparent.pipeline);
        pass.set_bind_group(0, &transparent.bind, &[]);
        for (buffer, count) in &gpu.chunks {
            pass.set_vertex_buffer(0, buffer.slice(..));
            pass.draw(0..*count, 0..1);
        }
    }
}
fn world_vertex_chunks(vertices: &[Vertex], max_bytes: u64) -> impl Iterator<Item = &[Vertex]> {
    let count =
        ((max_bytes.min(32 * 1024 * 1024) / std::mem::size_of::<Vertex>() as u64) / 3 * 3) as usize;
    assert!(count >= 3, "GPU must support at least one triangle");
    vertices.chunks(count)
}

#[test]
fn world_chunks_respect_buffer_limit_and_preserve_triangles() {
    let vertices = vec![bytemuck::Zeroable::zeroed(); 39];
    let chunks = world_vertex_chunks(&vertices, 400).collect::<Vec<_>>();
    assert_eq!(
        chunks.iter().map(|c| c.len()).sum::<usize>(),
        vertices.len()
    );
    assert!(chunks
        .iter()
        .all(|c| c.len() % 3 == 0 && std::mem::size_of_val(*c) <= 400));
    assert!(world_vertex_chunks(&[], 400).next().is_none());
}

pub(super) fn paint_world(
    painter: &egui::Painter,
    rect: Rect,
    geometry: Arc<spatial_world::Geometry>,
    matrix: [[f32; 4]; 4],
) {
    if (!geometry.faces.is_empty() || !geometry.edges.is_empty()) && rect.is_positive() {
        let pixels_per_point = painter.ctx().pixels_per_point();
        painter.add(egui_wgpu::Callback::new_paint_callback(
            rect,
            WorldDraw {
                clips: clips(),
                deformation: false,
                vertices: geometry.faces.clone(),
                edges: Some(geometry.edges.clone()),
                matrix,
                edge_scale: pixels_per_point,
                pixel: [
                    1.2 / (rect.width() * pixels_per_point),
                    1.2 / (rect.height() * pixels_per_point),
                ],
            },
        ));
    }
}
pub(super) fn paint_deformation(
    painter: &egui::Painter,
    rect: Rect,
    vertices: Arc<Vec<Vertex>>,
    matrix: [[f32; 4]; 4],
    amplitude: f32,
) {
    if !vertices.is_empty() && rect.is_positive() {
        painter.add(egui_wgpu::Callback::new_paint_callback(
            rect,
            WorldDraw {
                clips: clips(),
                deformation: true,
                vertices,
                edges: None,
                matrix,
                edge_scale: amplitude,
                pixel: [0.; 2],
            },
        ));
    }
}
pub(super) struct Builder {
    pub vertices: Vec<Vertex>,
    rect: Rect,
    near: f64,
    span: f64,
}
impl Builder {
    pub fn new(rect: Rect, projected: &[(Pos2, f64)]) -> Self {
        let near = projected
            .iter()
            .map(|p| p.1)
            .fold(f64::NEG_INFINITY, f64::max);
        let far = projected.iter().map(|p| p.1).fold(f64::INFINITY, f64::min);
        Self {
            vertices: vec![],
            rect,
            near,
            span: (near - far).max(1.),
        }
    }
    fn vertex(&self, point: (Pos2, f64), color: Color32, bias: f32) -> Vertex {
        let [r, g, b, a] = color.to_array();
        Vertex {
            offset: [0.; 3],
            position: [
                (point.0.x - self.rect.left()) / self.rect.width() * 2. - 1.,
                1. - (point.0.y - self.rect.top()) / self.rect.height() * 2.,
                (((self.near - point.1) / self.span) as f32 * 0.98 + 0.01 - bias).clamp(0., 1.),
            ],
            color: [
                r as f32 / 255.,
                g as f32 / 255.,
                b as f32 / 255.,
                a as f32 / 255.,
            ],
        }
    }
    pub fn anchor_recent(&mut self, count: usize, anchors: &[(Pos2, f64)], bias: f32) {
        let start = self.vertices.len() - count;
        for (v, anchor) in self.vertices[start..]
            .iter_mut()
            .zip(anchors.iter().cycle())
        {
            let p = [
                (anchor.0.x - self.rect.left()) / self.rect.width() * 2. - 1.,
                1. - (anchor.0.y - self.rect.top()) / self.rect.height() * 2.,
            ];
            let z =
                (((self.near - anchor.1) / self.span) as f32 * 0.98 + 0.01 - bias).clamp(0., 1.);
            v.offset = [
                v.position[0] + v.offset[0] - p[0],
                v.position[1] + v.offset[1] - p[1],
                0.,
            ];
            v.position[2] = z;
            v.position[0] = p[0];
            v.position[1] = p[1];
        }
    }
    pub fn triangle(&mut self, points: [(Pos2, f64); 3], ink: Color32, bias: f32) {
        if points
            .iter()
            .any(|p| !p.0.x.is_finite() || !p.0.y.is_finite() || !p.1.is_finite())
        {
            return;
        }
        let vertices = points.map(|p| self.vertex(p, ink, bias));
        self.vertices.extend(vertices);
    }
    pub fn colored_triangle(&mut self, points: [((Pos2, f64), Color32); 3]) {
        let vertices = points.map(|(point, ink)| self.vertex(point, ink, 0.));
        if vertices
            .iter()
            .any(|v| v.position.iter().any(|x| !x.is_finite()))
        {
            return;
        }
        self.vertices.extend(vertices);
    }
    pub fn result_triangle(
        &mut self,
        points: [(Pos2, f64); 3],
        values: [f64; 3],
        scale: &crate::plan_trace::result_scale::Scale,
        range: [f64; 2],
    ) {
        for triangle in scale.triangles(points.map(|p| [p.0.x as f64, p.0.y as f64]), values, range)
        {
            let vertices = triangle.map(|(p, ink)| {
                let at = Pos2::new(p[0] as f32, p[1] as f32);
                let depth = spatial_results::pick_triangle(at, points).map_or(
                    (points[0].1 + points[1].1 + points[2].1) / 3.,
                    |(_, depth)| depth,
                );
                ((at, depth), ink)
            });
            self.colored_triangle(vertices);
        }
    }
    pub fn line(&mut self, points: [(Pos2, f64); 2], width: f32, ink: Color32) {
        let [a, b] = points;
        let d = b.0 - a.0;
        let length = d.length();
        if length < 1e-5 {
            return;
        }
        let n = egui::vec2(-d.y, d.x) * (0.5 * width / length);
        let quad = [
            (a.0 + n, a.1),
            (b.0 + n, b.1),
            (b.0 - n, b.1),
            (a.0 - n, a.1),
        ];
        self.triangle([quad[0], quad[1], quad[2]], ink, 0.00001);
        self.triangle([quad[0], quad[2], quad[3]], ink, 0.00001);
        self.anchor_recent(6, &[a, b, b, a, b, a], 0.00001);
    }
    pub fn line_on_face(
        &mut self,
        points: [(Pos2, f64); 2],
        width: f32,
        ink: Color32,
        plane: [(Pos2, f64); 3],
    ) {
        let [a, b] = points;
        let d = b.0 - a.0;
        let length = d.length();
        if length < 1e-5 {
            return;
        }
        let n = egui::vec2(-d.y, d.x) * (0.5 * width / length);
        let [p, q, r] = plane;
        let u = q.0 - p.0;
        let v = r.0 - p.0;
        let det = u.x as f64 * v.y as f64 - u.y as f64 * v.x as f64;
        if det.abs() < 1e-8 {
            self.line(points, width, ink);
            return;
        }
        let z = |at: Pos2| {
            let w = at - p.0;
            let a = (w.x as f64 * v.y as f64 - w.y as f64 * v.x as f64) / det;
            let b = (u.x as f64 * w.y as f64 - u.y as f64 * w.x as f64) / det;
            (at, p.1 + a * (q.1 - p.1) + b * (r.1 - p.1))
        };
        let quad = [z(a.0 + n), z(b.0 + n), z(b.0 - n), z(a.0 - n)];
        self.triangle([quad[0], quad[1], quad[2]], ink, 0.00001);
        self.triangle([quad[0], quad[2], quad[3]], ink, 0.00001);
        self.anchor_recent(6, &[a, b, b, a, b, a], 0.00001);
    }
    pub fn marker(&mut self, point: (Pos2, f64), xy: bool) {
        let ink = if xy {
            Color32::from_rgb(90, 225, 255)
        } else {
            Color32::from_rgb(245, 165, 65)
        };
        let count = if xy { 4 } else { 16 };
        let angle = if xy { std::f32::consts::FRAC_PI_4 } else { 0. };
        for i in 0..count {
            let at = |j: usize| {
                let a = angle + j as f32 * std::f32::consts::TAU / count as f32;
                (point.0 + egui::vec2(a.cos(), a.sin()) * 4., point.1)
            };
            self.triangle([point, at(i), at(i + 1)], ink, 0.00002);
            self.anchor_recent(3, &[point], 0.00002);
        }
    }
}

#[cfg(test)]
mod gpu_tests {
    use super::*;
    #[test]
    fn intersection_reanchoring_preserves_line_width() {
        let rect = Rect::from_min_size(Pos2::ZERO, egui::vec2(100., 100.));
        let ends = [(egui::pos2(10., 50.), 0.), (egui::pos2(90., 50.), 0.)];
        let mut b = Builder::new(rect, &ends);
        b.line(ends, 5., Color32::BLUE);
        let before = b
            .vertices
            .iter()
            .map(|v| [v.position[0] + v.offset[0], v.position[1] + v.offset[1]])
            .collect::<Vec<_>>();
        b.anchor_recent(
            6,
            &[ends[0], ends[1], ends[1], ends[0], ends[1], ends[0]],
            0.00008,
        );
        for (v, p) in b.vertices.iter().zip(before) {
            assert!((v.position[0] + v.offset[0] - p[0]).abs() < 1e-6);
            assert!((v.position[1] + v.offset[1] - p[1]).abs() < 1e-6);
        }
        let ys = b
            .vertices
            .iter()
            .map(|v| v.position[1] + v.offset[1])
            .collect::<Vec<_>>();
        assert!(
            ys.iter().copied().fold(f32::NEG_INFINITY, f32::max)
                - ys.iter().copied().fold(f32::INFINITY, f32::min)
                > 0.09
        );
    }
    #[test]
    #[ignore = "requires headless wgpu adapter"]
    fn spatial_depth_hides_intersections_independent_of_face_order() {
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
        let viewport = egui::Rect::from_min_size(egui::Pos2::ZERO, egui::vec2(64., 64.));
        let depth = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("spatial test depth"),
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
        spatial_underlays::validate_shader(&device);
        for (reverse, cut) in [(false, false), (true, false), (false, true), (true, true)] {
            let mut test_clips = NO_CLIPS;
            if cut {
                test_clips[0] = [1., 0., 0., 0.];
            }
            let mut builder = Builder::new(
                viewport,
                &[(egui::Pos2::ZERO, 0.), (egui::pos2(64., 64.), 10.)],
            );
            let red = [
                (egui::pos2(0., 0.), 10.),
                (egui::pos2(64., 0.), 0.),
                (egui::pos2(64., 64.), 0.),
                (egui::pos2(0., 64.), 10.),
            ];
            let blue = red.map(|(p, _)| (p, 5.));
            let faces = if reverse {
                [(blue, egui::Color32::BLUE), (red, egui::Color32::RED)]
            } else {
                [(red, egui::Color32::RED), (blue, egui::Color32::BLUE)]
            };
            for (points, color) in faces {
                for ids in [[0, 1, 2], [0, 2, 3]] {
                    builder.triangle(ids.map(|i| points[i]), color, 0.);
                }
            }
            let draw = Draw {
                clips: test_clips,
                lane: Layer::Main,
                transform: [1., 1., 0., 0.],
                vertices: Arc::new(builder.vertices),
            };
            let mut resources = egui_wgpu::CallbackResources::default();
            resources.insert(new_gpu(&device, format));
            // Auxiliary contour lanes use the same expanded camera uniform.
            let _auxiliary = make_gpu(&device, format, SHADER, &[0.; (1 + MAX_PLANES) * 4], true);
            let screen = egui_wgpu::ScreenDescriptor {
                size_in_pixels: [64, 64],
                pixels_per_point: 1.,
            };
            resources.insert(new_world_gpu(&device, format));
            let identity = [
                [1., 0., 0., 0.],
                [0., 1., 0., 0.],
                [0., 0., 1., 0.],
                [0., 0., 0., 1.],
            ];
            let world = WorldDraw {
                edges: None,
                clips: test_clips,
                deformation: false,
                edge_scale: 1.,
                pixel: [1.2 / 64.; 2],
                vertices: draw.vertices.clone(),
                matrix: identity,
            };
            let mut encoder = device.create_command_encoder(&Default::default());
            world.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            let rotated = WorldDraw {
                edges: None,
                clips: test_clips,
                deformation: false,
                edge_scale: 1.,
                pixel: [1.2 / 64.; 2],
                vertices: world.vertices.clone(),
                matrix: [
                    [0., 1., 0., 0.],
                    [-1., 0., 0., 0.],
                    [0., 0., 1., 0.],
                    [0., 0., 0., 1.],
                ],
            };
            rotated.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            assert_eq!(resources.get::<WorldGpu>().unwrap().0.uploads, 1);
            world.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            let moved = Draw {
                clips: test_clips,
                lane: Layer::Main,
                vertices: draw.vertices.clone(),
                transform: [1.3, 1.3, 0.2, -0.1],
            };
            moved.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            assert_eq!(resources.get::<Gpu>().unwrap().uploads, 1);

            draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            assert!(Arc::ptr_eq(
                resources.get::<Gpu>().unwrap().owner.as_ref().unwrap(),
                &draw.vertices
            ));
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
                // Render only the world pipeline: a screen-space draw must not
                // mask missing pixels or a broken world shader in this check.
                world.paint(
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
                &data[32 * 256 + 16 * 4..32 * 256 + 16 * 4 + 4],
                if cut {
                    &[0, 0, 0, 255]
                } else {
                    &[255, 0, 0, 255]
                }
            );
            assert_eq!(
                &data[32 * 256 + 48 * 4..32 * 256 + 48 * 4 + 4],
                &[0, 0, 255, 255]
            );
            drop(data);
            readback.unmap();
            // Empty geometry must neither allocate buffers nor paint.
            let empty = Draw {
                clips: test_clips,
                lane: Layer::Main,
                transform: [1., 1., 0., 0.],
                vertices: Arc::new(vec![]),
            };
            let mut encoder = device.create_command_encoder(&Default::default());
            empty.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
            assert!(Arc::ptr_eq(
                resources.get::<Gpu>().unwrap().owner.as_ref().unwrap(),
                &draw.vertices
            ));
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
fn gpu_diagnostic(message: &str) {
    use std::io::Write;
    let Some(local) = std::env::var_os("LOCALAPPDATA") else {
        return;
    };
    let directory = std::path::PathBuf::from(local).join("RConcrete");
    if std::fs::create_dir_all(&directory).is_err() {
        return;
    }
    if let Ok(mut file) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(directory.join("fem-gpu.log"))
    {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map_or(0, |d| d.as_secs());
        let _ = writeln!(file, "{stamp} {message}");
    }
}
#[cfg(all(test, not(target_arch = "wasm32")))]
#[path = "render_bench.rs"]
mod render_bench;
