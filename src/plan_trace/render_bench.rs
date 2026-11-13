//! Reproducible comparison of the offscreen GPU paths, without GUI overhead or vsync.
use super::*;
use egui_wgpu::CallbackTrait;
use std::{future::Future, time::Instant};

fn block_on<T>(future: impl Future<Output = T>) -> T {
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

#[test]
#[ignore = "requires a local building project, egui_fem source and GPU"]
fn matched_building_gpu_paths() {
    let input =
        PathBuf::from(std::env::var_os("MINI_FEM_BENCH_PROJECT").expect("MINI_FEM_BENCH_PROJECT"));
    let shader_path = PathBuf::from(
        std::env::var_os("EGUI_FEM_BUILDING_SHADER").expect("EGUI_FEM_BUILDING_SHADER"),
    );
    let mut app = App::default();
    app.load_project_path(input.clone()).unwrap();
    app.sync_stiffness_sources();
    let step = std::env::var("MINI_FEM_BENCH_STEP")
        .ok()
        .and_then(|s| s.parse::<f64>().ok())
        .unwrap_or(500.);
    let mut layout = app.building_layout();
    let sources = crate::plan_trace::columns::meshing_sources(
        &app.contours,
        &app.support_height,
        &mut layout,
    );
    let mut mesh = crate::plan_trace::building_mesh::generate(
        &sources,
        step,
        layout.unwrap(),
        app.mesh_options,
    )
    .unwrap();
    crate::plan_trace::columns::tag_mesh(&mut mesh, &app.contours, &app.support_height);
    crate::plan_trace::fem::precompute_wall_mesh(&mut mesh, step).unwrap();
    let mats = vec![crate::plan_trace::fem::element::Material::default(); mesh.elements.len()];
    let scene = crate::plan_trace::fem::shell::scene(
        &mesh,
        &mats,
        &app.support_height,
        step,
        &app.contours,
    )
    .unwrap();
    let lo: [f64; 3] =
        std::array::from_fn(|i| scene.xyz.iter().map(|p| p[i]).fold(f64::INFINITY, f64::min));
    let hi: [f64; 3] = std::array::from_fn(|i| {
        scene
            .xyz
            .iter()
            .map(|p| p[i])
            .fold(f64::NEG_INFINITY, f64::max)
    });
    let center = std::array::from_fn(|i| (lo[i] + hi[i]) * 0.5);
    let extent = (0..3).map(|i| (hi[i] - lo[i]).powi(2)).sum::<f64>().sqrt();
    let mut tris = vec![];
    let mut edges = std::collections::BTreeSet::new();
    for face in &scene.surfaces {
        let n = &face.nodes;
        for i in 1..n.len() - 1 {
            tris.push([n[0], n[i], n[i + 1]]);
        }
        for (&a, &b) in n.iter().zip(n.iter().cycle().skip(1)) {
            edges.insert([a.min(b), a.max(b)]);
        }
    }
    let points: Vec<_> = scene
        .xyz
        .iter()
        .map(|&p| spatial_world::vertex(p, Color32::from_rgb(60, 120, 182), center, extent))
        .collect();
    let faces = Arc::new(
        tris.iter()
            .flat_map(|tri| tri.map(|n| points[n]))
            .collect::<Vec<_>>(),
    );
    let ribbons = Arc::new(
        edges
            .iter()
            .filter_map(|&[a, b]| {
                spatial_world::edge_ribbon(scene.xyz[a], scene.xyz[b], center, extent)
            })
            .collect::<Vec<_>>(),
    );
    let baseline = Arc::new(
        faces
            .iter()
            .copied()
            .chain(
                ribbons
                    .iter()
                    .flat_map(|r| [0, 1, 2, 0, 2, 3].map(|i| r[i])),
            )
            .collect::<Vec<_>>(),
    );

    let instance = wgpu::Instance::default();
    let adapter =
        block_on(instance.request_adapter(&wgpu::RequestAdapterOptions::default())).unwrap();
    let (device, queue) =
        block_on(adapter.request_device(&wgpu::DeviceDescriptor::default(), None)).unwrap();
    let format = wgpu::TextureFormat::Rgba8Unorm;
    let size = wgpu::Extent3d {
        width: 1400,
        height: 1000,
        depth_or_array_layers: 1,
    };
    let texture = |format| {
        device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("benchmark target"),
                size,
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            })
            .create_view(&Default::default())
    };
    let color = texture(format);
    let depth24 = texture(wgpu::TextureFormat::Depth24Plus);
    let depth32 = texture(wgpu::TextureFormat::Depth32Float);
    let mut resources = egui_wgpu::CallbackResources::default();
    resources.insert(new_world_gpu(&device, format));
    let screen = egui_wgpu::ScreenDescriptor {
        size_in_pixels: [size.width, size.height],
        pixels_per_point: 1.,
    };
    let rect = Rect::from_min_size(
        Pos2::ZERO,
        egui::vec2(size.width as f32, size.height as f32),
    );
    let info = || egui::PaintCallbackInfo {
        viewport: rect,
        clip_rect: rect,
        pixels_per_point: 1.,
        screen_size_px: [size.width, size.height],
    };

    // The other project's shader and pipeline state, with the same input topology.
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("egui_fem building.wgsl"),
        source: wgpu::ShaderSource::Wgsl(std::fs::read_to_string(shader_path).unwrap().into()),
    });
    let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: None,
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    let pl = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: None,
        bind_group_layouts: &[&layout],
        push_constant_ranges: &[],
    });
    let pipeline = |wire: bool| {
        device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("egui_fem benchmark pipeline"),
            layout: Some(&pl),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: 24,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &wgpu::vertex_attr_array![0=>Float32x3,1=>Float32x3],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some(if wire { "fs_wire" } else { "fs_main" }),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(if wire {
                        wgpu::BlendState::ALPHA_BLENDING
                    } else {
                        wgpu::BlendState::REPLACE
                    }),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: if wire {
                    wgpu::PrimitiveTopology::LineList
                } else {
                    wgpu::PrimitiveTopology::TriangleList
                },
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: !wire,
                depth_compare: if wire {
                    wgpu::CompareFunction::LessEqual
                } else {
                    wgpu::CompareFunction::Less
                },
                stencil: Default::default(),
                bias: if wire {
                    wgpu::DepthBiasState {
                        constant: -1,
                        slope_scale: -1.,
                        clamp: 0.,
                    }
                } else {
                    Default::default()
                },
            }),
            multisample: Default::default(),
            multiview: None,
            cache: None,
        })
    };
    let ref_pipelines = [pipeline(false), pipeline(true)];
    let buffer = |bytes: &[u8], usage| {
        device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: None,
            contents: bytes,
            usage,
        })
    };
    let ref_points: Vec<_> = points
        .iter()
        .map(|p| [p.position[0], p.position[1], p.position[2], 0., 0., 0.])
        .collect();
    let ref_vertex = buffer(
        bytemuck::cast_slice(&ref_points),
        wgpu::BufferUsages::VERTEX,
    );
    let ref_indices = [
        tris.iter()
            .flat_map(|t| t.map(|n| n as u32))
            .collect::<Vec<_>>(),
        edges.iter().flat_map(|e| e.map(|n| n as u32)).collect(),
    ];
    let ref_index = ref_indices
        .each_ref()
        .map(|i| buffer(bytemuck::cast_slice(i), wgpu::BufferUsages::INDEX));
    let uniform = buffer(
        &[0; 80],
        wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
    );
    let bind = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: None,
        layout: &layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: uniform.as_entire_binding(),
        }],
    });

    let camera = |frame: usize| {
        let yaw = 0.3 + frame as f32 * 0.031;
        (glam::Mat4::from_translation(glam::Vec3::new(0., 0., 0.5))
            * glam::Mat4::from_scale(glam::Vec3::new(1.24, 1.74, -0.80))
            * glam::Mat4::from_rotation_x(0.61)
            * glam::Mat4::from_rotation_z(yaw))
        .to_cols_array_2d()
    };
    let mut measurements = [vec![], vec![], vec![]];
    let mut frame = |mode: usize, i: usize| {
        let matrix = camera(i);
        let t = Instant::now();
        let mut encoder = device.create_command_encoder(&Default::default());
        let draw = WorldDraw {
            clips: NO_CLIPS,
            deformation: false,
            edge_scale: 0.,
            pixel: [1.2 / 1400., 1.2 / 1000.],
            vertices: if mode == 1 {
                baseline.clone()
            } else {
                faces.clone()
            },
            edges: (mode == 2).then(|| ribbons.clone()),
            matrix,
        };
        if mode == 0 {
            let mut u = [0_f32; 20];
            u[..16].copy_from_slice(matrix.as_flattened());
            u[17] = 1.;
            queue.write_buffer(&uniform, 0, bytemuck::cast_slice(&u));
        } else {
            draw.prepare(&device, &queue, &screen, &mut encoder, &mut resources);
        }
        {
            let mut pass = encoder
                .begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: None,
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &color,
                        resolve_target: None,
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                            store: wgpu::StoreOp::Store,
                        },
                    })],
                    depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                        view: if mode == 0 { &depth32 } else { &depth24 },
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
            if mode == 0 {
                for j in 0..2 {
                    pass.set_pipeline(&ref_pipelines[j]);
                    pass.set_bind_group(0, &bind, &[]);
                    pass.set_vertex_buffer(0, ref_vertex.slice(..));
                    pass.set_index_buffer(ref_index[j].slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..ref_indices[j].len() as u32, 0, 0..1);
                }
            } else {
                draw.paint(info(), &mut pass, &resources);
            }
        }
        queue.submit([encoder.finish()]);
        device.poll(wgpu::Maintain::Wait);
        t.elapsed().as_secs_f64() * 1000.
    };
    for order in [[0, 1, 2], [2, 0, 1], [1, 2, 0]] {
        for mode in order {
            for i in 0..12 {
                frame(mode, i);
            }
            for i in 0..60 {
                measurements[mode].push(frame(mode, i));
            }
        }
    }
    let metric = |mut samples: Vec<f64>| {
        samples.sort_by(f64::total_cmp);
        let median = samples[samples.len() / 2];
        let p95 = samples[(samples.len() * 95 / 100).min(samples.len() - 1)];
        serde_json::json!({"samples":samples.len(),"median_ms":median,"p95_ms":p95,"fps_at_median":1000./median})
    };
    let result = serde_json::json!({
        "project":input,"adapter":format!("{:?}",adapter.get_info()),"resolution":[size.width,size.height],"mesh_mm":step,
        "nodes":scene.xyz.len(),"shells":scene.surfaces.len(),"triangles":tris.len(),"unique_edges":edges.len(),
        "edge_vertex_bytes_before":ribbons.len()*6*40,"edge_vertex_bytes_after":ribbons.len()*4*40,
        "edge_shared_index_bytes":ribbons.len().min(16384)*6*2,
        "egui_fem_pipeline":metric(measurements[0].clone()),"mini_fem_before":metric(measurements[1].clone()),
        "mini_fem_indexed":metric(measurements[2].clone()),
    });
    std::fs::write(
        "target/indexed_mesh_fps.json",
        serde_json::to_vec_pretty(&result).unwrap(),
    )
    .unwrap();
    println!("{result}");
}
