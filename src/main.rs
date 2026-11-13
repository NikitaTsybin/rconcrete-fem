use eframe::{
    egui,
    egui_wgpu::{self, wgpu},
};
use glam::{Mat4, Vec3};
use std::collections::HashSet;
use std::sync::Arc;
use wgpu::util::DeviceExt;
mod lira;

#[derive(Clone, Copy, PartialEq, Debug)]
enum Kind {
    Slab,
    Wall,
    Column,
    Point,
}
#[derive(Clone)]
struct Element {
    id: usize,
    floor: usize,
    kind: Kind,
    points: Vec<Vec3>,
    source: Option<lira::Topology>,
}
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Vertex {
    position: [f32; 3],
    color: [f32; 4],
}
struct Gpu {
    triangles: wgpu::RenderPipeline,
    lines: wgpu::RenderPipeline,
}
#[derive(Clone)]
struct Draw {
    faces: Arc<Vec<Vertex>>,
    edges: Arc<Vec<Vertex>>,
    revision: u64,
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
        if resources
            .get::<Buffers>()
            .is_some_and(|buffers| buffers.revision == self.revision)
        {
            return vec![];
        }
        let faces = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("faces"),
            contents: bytemuck::cast_slice(self.faces.as_slice()),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let edges = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("edges"),
            contents: bytemuck::cast_slice(self.edges.as_slice()),
            usage: wgpu::BufferUsages::VERTEX,
        });
        resources.insert(Buffers {
            faces,
            edges,
            revision: self.revision,
        });
        vec![]
    }
    fn paint(
        &self,
        _: egui::PaintCallbackInfo,
        pass: &mut wgpu::RenderPass<'static>,
        resources: &egui_wgpu::CallbackResources,
    ) {
        let gpu = resources.get::<Gpu>().unwrap();
        let b = resources.get::<Buffers>().unwrap();
        if !self.faces.is_empty() {
            pass.set_pipeline(&gpu.triangles);
            pass.set_vertex_buffer(0, b.faces.slice(..));
            pass.draw(0..self.faces.len() as u32, 0..1);
        }
        if !self.edges.is_empty() {
            pass.set_pipeline(&gpu.lines);
            pass.set_vertex_buffer(0, b.edges.slice(..));
            pass.draw(0..self.edges.len() as u32, 0..1);
        }
    }
}
struct Buffers {
    faces: wgpu::Buffer,
    edges: wgpu::Buffer,
    revision: u64,
}
fn pipeline(
    device: &wgpu::Device,
    format: wgpu::TextureFormat,
    topology: wgpu::PrimitiveTopology,
) -> wgpu::RenderPipeline {
    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("FEM shader"),
        source: wgpu::ShaderSource::Wgsl(include_str!("shader.wgsl").into()),
    });
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("FEM pipeline"),
        layout: None,
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            compilation_options: Default::default(),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: 28,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x4],
            }],
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            topology,
            cull_mode: None,
            ..Default::default()
        },
        depth_stencil: Some(wgpu::DepthStencilState {
            format: wgpu::TextureFormat::Depth24Plus,
            depth_write_enabled: true,
            depth_compare: wgpu::CompareFunction::LessEqual,
            stencil: Default::default(),
            bias: Default::default(),
        }),
        multisample: Default::default(),
        multiview: None,
        cache: None,
    })
}
fn building() -> Vec<Element> {
    building_with_refinement(35)
}
fn building_with_refinement(refinement: usize) -> Vec<Element> {
    assert!(refinement > 0);
    let mut elements = Vec::with_capacity(410 * refinement * refinement + 100);
    let step = 2.0 / refinement as f32;
    let wall_height = 1.6 / refinement as f32;
    let mut add = |floor, kind, points| {
        elements.push(Element {
            id: elements.len() + 1,
            floor,
            kind,
            points,
            source: None,
        });
    };
    for floor in 1..=5 {
        let z = floor as f32 * 3.2;
        for x in 0..=4 {
            for y in 0..=3 {
                add(
                    floor,
                    Kind::Column,
                    vec![
                        Vec3::new(x as f32 * 4., y as f32 * 4., z - 3.2),
                        Vec3::new(x as f32 * 4., y as f32 * 4., z),
                    ],
                );
            }
        }
        for x in 0..8 * refinement {
            for y in 0..6 * refinement {
                let a = Vec3::new(x as f32 * step, y as f32 * step, z);
                let b = a + Vec3::X * step;
                let c = b + Vec3::Y * step;
                let d = a + Vec3::Y * step;
                if (x + y + floor) % 3 == 0 {
                    add(floor, Kind::Slab, vec![a, b, c]);
                    add(floor, Kind::Slab, vec![a, c, d]);
                } else {
                    add(floor, Kind::Slab, vec![a, b, c, d]);
                }
            }
        }
        // L-shaped shear walls, subdivided vertically and horizontally.
        for side in 0..2 {
            for i in 0..3 * refinement {
                for j in 0..2 * refinement {
                    let base = Vec3::new(4., 4., z - 3.2 + j as f32 * wall_height);
                    let axis = if side == 0 { Vec3::X } else { Vec3::Y };
                    let a = base + axis * i as f32 * step;
                    let b = a + axis * step;
                    let c = b + Vec3::Z * wall_height;
                    let d = a + Vec3::Z * wall_height;
                    if (i + j + floor) % 2 == 0 {
                        add(floor, Kind::Wall, vec![a, b, c]);
                        add(floor, Kind::Wall, vec![a, c, d]);
                    } else {
                        add(floor, Kind::Wall, vec![a, b, c, d]);
                    }
                }
            }
        }
    }
    elements
}
#[derive(Clone, Copy, PartialEq, Debug)]
enum Projection {
    Isometric,
    Dimetric,
    Perspective,
}
impl Projection {
    fn name(self) -> &'static str {
        match self {
            Self::Isometric => "Изометрическая",
            Self::Dimetric => "Диметрическая",
            Self::Perspective => "Перспективная",
        }
    }
    fn angles(self) -> (f32, f32) {
        match self {
            Self::Isometric => (
                std::f32::consts::FRAC_PI_4,
                (1.0_f32 / 3.0_f32.sqrt()).asin(),
            ),
            Self::Dimetric => (20.7_f32.to_radians(), 19.47_f32.to_radians()),
            Self::Perspective => (0.75, 0.55),
        }
    }
}
struct DisplayOptions {
    open: bool,
    tab: usize,
    projection: Projection,
    nodes: bool,
    node_numbers: bool,
    node_size: f32,
    element_numbers: bool,
    outlines: bool,
    background: egui::Color32,
}
impl Default for DisplayOptions {
    fn default() -> Self {
        Self {
            open: false,
            tab: 0,
            projection: Projection::Isometric,
            nodes: false,
            node_numbers: false,
            node_size: 3.,
            element_numbers: false,
            outlines: true,
            background: egui::Color32::from_rgb(19, 25, 35),
        }
    }
}
// Display-only unique nodes. Quantization tolerates rounding at shared storey boundaries.
fn display_nodes(elements: &[Element]) -> Vec<Vec3> {
    let mut seen = HashSet::new();
    elements
        .iter()
        .flat_map(|e| e.points.iter().copied())
        .filter(|p| {
            seen.insert((
                (p.x * 1000.).round() as i32,
                (p.y * 1000.).round() as i32,
                (p.z * 1000.).round() as i32,
            ))
        })
        .collect()
}
struct Performance {
    continuous: bool,
    window: std::time::Instant,
    frames: usize,
    fps: f32,
    cpu_ms: f32,
    benchmark: Option<usize>,
    benchmark_start: Option<std::time::Instant>,
    benchmark_orbit: bool,
}
impl Default for Performance {
    fn default() -> Self {
        Self {
            continuous: true,
            window: std::time::Instant::now(),
            frames: 0,
            fps: 0.,
            cpu_ms: 0.,
            benchmark: None,
            benchmark_start: None,
            benchmark_orbit: false,
        }
    }
}
#[derive(Default)]
struct ViewCache {
    dirty: bool,
    visibility_key: Option<([bool; 4], usize)>,
    indices: Vec<usize>,
    visibility_revision: u64,
    selection_revision: u64,
    render_key: Option<RenderKey>,
    draw: Option<Draw>,
    surfaces: Arc<Vec<[Vec3; 3]>>,
    depth_index: Option<DepthIndex>,
    overlay_key: Option<OverlayKey>,
    overlay_shapes: Vec<egui::Shape>,
    revision: u64,
    all_nodes: Option<Vec<Vec3>>,
    node_indices: Option<Vec<usize>>,
}
#[derive(PartialEq)]
struct OverlayKey {
    revision: u64,
    rect: [f32; 4],
    pixels_per_point: f32,
    nodes: bool,
    node_numbers: bool,
    element_numbers: bool,
    node_size: f32,
}
const DEPTH_GRID: usize = 128;
struct DepthIndex {
    triangles: Arc<Vec<[Vec3; 3]>>,
    buckets: Vec<Vec<usize>>,
}
impl DepthIndex {
    fn cell(value: f32) -> usize {
        (((value + 1.) * 0.5 * DEPTH_GRID as f32).floor() as isize)
            .clamp(0, DEPTH_GRID as isize - 1) as usize
    }
    fn new(triangles: Arc<Vec<[Vec3; 3]>>) -> Self {
        if triangles.is_empty() {
            return Self {
                triangles,
                buckets: Vec::new(),
            };
        }
        let mut buckets = vec![Vec::new(); DEPTH_GRID * DEPTH_GRID];
        for (i, [a, b, c]) in triangles.iter().enumerate() {
            let min = a.min(*b).min(*c);
            let max = a.max(*b).max(*c);
            // Conservative margin includes the barycentric tolerance used by surface_depth.
            let pad = (max.x - min.x + max.y - min.y + 1.) * 0.00003;
            if max.x + pad < -1. || min.x - pad > 1. || max.y + pad < -1. || min.y - pad > 1. {
                continue;
            }
            for y in Self::cell(min.y - pad)..=Self::cell(max.y + pad) {
                for x in Self::cell(min.x - pad)..=Self::cell(max.x + pad) {
                    buckets[y * DEPTH_GRID + x].push(i);
                }
            }
        }
        Self { triangles, buckets }
    }
    fn visible(&self, point: Vec3) -> bool {
        self.buckets.is_empty()
            || !self.buckets[Self::cell(point.y) * DEPTH_GRID + Self::cell(point.x)]
                .iter()
                .any(|i| {
                    surface_depth(point, self.triangles[*i])
                        .is_some_and(|depth| depth < point.z - 0.0001)
                })
    }
}
fn number_shape(
    painter: &egui::Painter,
    position: egui::Pos2,
    align: egui::Align2,
    number: usize,
    color: egui::Color32,
) -> egui::Shape {
    let galley = painter.layout_no_wrap(number.to_string(), egui::FontId::monospace(11.), color);
    let origin = align.anchor_size(position, galley.size()).min;
    egui::Shape::galley(origin, galley, color)
}
#[derive(PartialEq)]
struct RenderKey {
    camera: [f32; 8],
    perspective: bool,
    wire: bool,
    outlines: bool,
    labels: bool,
    visibility: u64,
    selection: u64,
}
const NUMBER_LIMIT: usize = 5000;
const CAMERA_QUIET: std::time::Duration = std::time::Duration::from_millis(180);
#[derive(Default)]
struct CameraMotion {
    camera: Option<[f32; 5]>,
    moved_at: Option<std::time::Instant>,
}
impl CameraMotion {
    fn suspend_numbers(&mut self, count: usize, camera: [f32; 5], now: std::time::Instant) -> bool {
        if self.camera.is_some_and(|previous| previous != camera) {
            self.moved_at = Some(now);
        }
        self.camera = Some(camera);
        count > NUMBER_LIMIT
            && self
                .moved_at
                .is_some_and(|last| now.duration_since(last) < CAMERA_QUIET)
    }
}
fn number_count(node_numbers: bool, element_numbers: bool, nodes: usize, elements: usize) -> usize {
    (if node_numbers { nodes } else { 0 }) + (if element_numbers { elements } else { 0 })
}
struct ImportState {
    pending: Option<
        std::sync::mpsc::Receiver<Result<Option<(std::path::PathBuf, lira::Geometry)>, String>>,
    >,
    nodes: Vec<lira::Node>,
    indices: std::collections::HashMap<usize, usize>,
    origin: [f64; 3],
    loaded: bool,
    center: Vec3,
    distance: f32,
    default_zoom: f32,
    info: String,
    title: String,
}
impl Default for ImportState {
    fn default() -> Self {
        Self {
            pending: None,
            nodes: Vec::new(),
            indices: Default::default(),
            origin: [0.; 3],
            loaded: false,
            center: Vec3::new(8., 6., 8.),
            distance: 45.,
            default_zoom: 27.,
            info: String::new(),
            title: "Демонстрационная модель · 5 этажей".into(),
        }
    }
}
struct App {
    elements: Vec<Element>,
    selected: HashSet<usize>,
    previous_selected: Option<HashSet<usize>>,
    hidden: HashSet<usize>,
    show: [bool; 4],
    floor: usize,
    yaw: f32,
    pitch: f32,
    zoom: f32,
    pan: egui::Vec2,
    wire: bool,
    display: DisplayOptions,
    performance: Performance,
    cache: ViewCache,
    camera_motion: CameraMotion,
    import: ImportState,
    box_start: Option<egui::Pos2>,
    smoke_frame: Option<usize>,
}
impl App {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let rs = cc.wgpu_render_state.as_ref().unwrap();
        rs.renderer.write().callback_resources.insert(Gpu {
            triangles: pipeline(
                &rs.device,
                rs.target_format,
                wgpu::PrimitiveTopology::TriangleList,
            ),
            lines: pipeline(
                &rs.device,
                rs.target_format,
                wgpu::PrimitiveTopology::LineList,
            ),
        });
        let started = std::time::Instant::now();
        let smoke = std::env::args().any(|a| a == "--smoke-test");
        let elements = if smoke {
            building_with_refinement(1)
        } else {
            building()
        };
        eprintln!(
            "Generated {} elements in {:.1} ms",
            elements.len(),
            started.elapsed().as_secs_f64() * 1000.
        );
        let mut performance = Performance::default();
        if std::env::args().any(|a| a == "--benchmark" || a == "--benchmark-orbit") {
            performance.benchmark = Some(0);
        }
        performance.benchmark_orbit = std::env::args().any(|a| a == "--benchmark-orbit");
        let mut app = Self {
            elements,
            selected: HashSet::new(),
            previous_selected: None,
            hidden: HashSet::new(),
            show: [true; 4],
            floor: 0,
            yaw: Projection::Isometric.angles().0,
            pitch: Projection::Isometric.angles().1,
            zoom: 27.,
            pan: egui::Vec2::ZERO,
            wire: false,
            display: DisplayOptions::default(),
            performance,
            cache: ViewCache::default(),
            camera_motion: CameraMotion::default(),
            import: ImportState::default(),
            box_start: None,
            smoke_frame: smoke.then_some(0),
        };
        let fragment = if std::env::args().any(|a| a == "--fragment-13000") {
            Some(13000)
        } else if std::env::args().any(|a| a == "--fragment-3000") {
            Some(3000)
        } else {
            None
        };
        if let Some(count) = fragment {
            app.set_selection((1..=count.min(app.elements.len())).collect());
            app.fragment_selected();
        }
        if std::env::args().any(|a| a == "--labels") {
            app.display.nodes = true;
            app.display.node_numbers = true;
            app.display.element_numbers = true;
        }
        let args: Vec<_> = std::env::args_os().collect();
        if let Some(index) = args.iter().position(|a| a == "--lira") {
            if let Some(path) = args.get(index + 1) {
                let path = std::path::PathBuf::from(path);
                match lira::read(&path) {
                    Ok(geometry) => app.apply_import(path, geometry),
                    Err(error) => app.import.info = format!("Ошибка импорта: {error}"),
                }
            }
        }
        app
    }
    fn visible(&self, e: &Element) -> bool {
        self.show[match e.kind {
            Kind::Slab => 0,
            Kind::Wall => 1,
            Kind::Column => 2,
            Kind::Point => 3,
        }] && (self.floor == 0 || self.floor == e.floor)
            && !self.hidden.contains(&e.id)
    }
    fn refresh_visibility(&mut self) {
        let key = (self.show, self.floor);
        if self.cache.dirty || self.cache.visibility_key != Some(key) {
            self.cache.indices = self
                .elements
                .iter()
                .enumerate()
                .filter(|(_, e)| self.visible(e))
                .map(|(i, _)| i)
                .collect();
            self.cache.visibility_revision += 1;
            self.cache.visibility_key = Some(key);
            self.cache.node_indices = None;
            self.cache.dirty = false;
        }
    }
    fn restore_visibility(&mut self) {
        if !self.hidden.is_empty() {
            self.hidden.clear();
            self.cache.dirty = true;
        }
    }
    fn refresh_nodes(&mut self) {
        if self.cache.all_nodes.is_none() {
            self.cache.all_nodes = Some(display_nodes(&self.elements));
        }
        if self.cache.node_indices.is_none() {
            if self.import.loaded {
                let mut referenced = vec![false; self.import.nodes.len()];
                for index in &self.cache.indices {
                    if let Some(source) = &self.elements[*index].source {
                        for id in &source.nodes {
                            referenced[*id - 1] = true;
                        }
                    }
                }
                self.cache.node_indices = Some(
                    referenced
                        .iter()
                        .enumerate()
                        .filter(|(_, used)| **used)
                        .map(|(i, _)| i)
                        .collect(),
                );
                return;
            }
            let key = |p: Vec3| {
                (
                    (p.x * 1000.).round() as i32,
                    (p.y * 1000.).round() as i32,
                    (p.z * 1000.).round() as i32,
                )
            };
            let visible: HashSet<_> = self
                .cache
                .indices
                .iter()
                .flat_map(|i| self.elements[*i].points.iter().copied())
                .map(key)
                .collect();
            self.cache.node_indices = Some(
                self.cache
                    .all_nodes
                    .as_ref()
                    .unwrap()
                    .iter()
                    .enumerate()
                    .filter(|(_, p)| visible.contains(&key(**p)))
                    .map(|(i, _)| i)
                    .collect(),
            );
        }
    }
    fn set_selection(&mut self, next: HashSet<usize>) {
        if next != self.selected {
            self.cache.selection_revision += 1;
            self.previous_selected = Some(std::mem::replace(&mut self.selected, next));
        }
    }
    fn select(&mut self, ids: impl IntoIterator<Item = usize>, modifiers: egui::Modifiers) {
        let mut next = self.selected.clone();
        apply_selection(&mut next, ids, modifiers);
        self.set_selection(next);
    }
    fn hide_selected(&mut self) {
        self.cache.dirty = true;
        self.hidden.extend(self.selected.iter().copied());
        self.set_selection(HashSet::new());
    }
    fn fragment_selected(&mut self) {
        if !self.selected.is_empty() {
            self.cache.dirty = true;
            self.hidden = self
                .elements
                .iter()
                .filter(|e| !self.selected.contains(&e.id))
                .map(|e| e.id)
                .collect();
        }
    }
    fn restore_previous_selection(&mut self) {
        if let Some(previous) = self.previous_selected.take() {
            let visible = previous
                .into_iter()
                .filter(|id| self.element_by_id(*id).is_some_and(|e| self.visible(e)))
                .collect();
            self.previous_selected = Some(std::mem::replace(&mut self.selected, visible));
            self.cache.selection_revision += 1;
        }
    }
    fn element_by_id(&self, id: usize) -> Option<&Element> {
        if self.import.loaded {
            self.import
                .indices
                .get(&id)
                .and_then(|index| self.elements.get(*index))
        } else {
            self.elements.get(id.checked_sub(1)?)
        }
    }
    fn start_import(&mut self) {
        let (sender, receiver) = std::sync::mpsc::channel();
        self.import.pending = Some(receiver);
        self.import.info = "Выберите файл TXT ЛИРА…".into();
        std::thread::spawn(move || {
            let result = match rfd::FileDialog::new()
                .add_filter("Геометрия TXT ЛИРА", &["txt"])
                .set_directory("C:/Lira/Data")
                .pick_file()
            {
                Some(path) => lira::read(&path).map(|geometry| Some((path, geometry))),
                None => Ok(None),
            };
            let _ = sender.send(result);
        });
    }
    fn poll_import(&mut self, ctx: &egui::Context) {
        let result = self
            .import
            .pending
            .as_ref()
            .map(|receiver| receiver.try_recv());
        match result {
            Some(Ok(result)) => {
                self.import.pending = None;
                match result {
                    Ok(Some((path, geometry))) => self.apply_import(path, geometry),
                    Ok(None) => self.import.info = "Импорт отменён".into(),
                    Err(error) => self.import.info = format!("Ошибка импорта: {error}"),
                }
            }
            Some(Err(std::sync::mpsc::TryRecvError::Disconnected)) => {
                self.import.pending = None;
                self.import.info = "Поток импорта завершился без результата".into();
            }
            Some(Err(std::sync::mpsc::TryRecvError::Empty)) => {
                ctx.request_repaint_after(std::time::Duration::from_millis(50))
            }
            None => {}
        }
    }
    fn apply_import(&mut self, path: std::path::PathBuf, geometry: lira::Geometry) {
        let mut min = [f64::INFINITY; 3];
        let mut max = [f64::NEG_INFINITY; 3];
        for node in &geometry.nodes {
            for axis in 0..3 {
                min[axis] = min[axis].min(node.xyz[axis]);
                max[axis] = max[axis].max(node.xyz[axis]);
            }
        }
        let origin = std::array::from_fn(|axis| (min[axis] + max[axis]) * 0.5);
        let local: Vec<Vec3> = geometry
            .nodes
            .iter()
            .map(|n| {
                Vec3::new(
                    (n.xyz[0] - origin[0]) as f32,
                    (n.xyz[1] - origin[1]) as f32,
                    (n.xyz[2] - origin[2]) as f32,
                )
            })
            .collect();
        let radius = Vec3::new(
            (max[0] - min[0]) as f32,
            (max[1] - min[1]) as f32,
            (max[2] - min[2]) as f32,
        )
        .length()
        .max(0.01)
            * 0.5;
        self.elements = geometry
            .elements
            .into_iter()
            .map(|source| {
                // LIRA shell 44 uses a tensor-product ordering 1,2,3,4, not perimeter ordering.
                let order: Vec<_> = if source.fe_type == 44 {
                    vec![0, 1, 3, 2]
                } else {
                    (0..source.nodes.len()).collect()
                };
                let points: Vec<_> = order.iter().map(|i| local[source.nodes[*i] - 1]).collect();
                let kind = match points.len() {
                    1 => Kind::Point,
                    2 => Kind::Column,
                    _ => {
                        let normal = (points[1] - points[0])
                            .cross(points[2] - points[0])
                            .normalize_or_zero();
                        if normal.z.abs() > 0.7 {
                            Kind::Slab
                        } else {
                            Kind::Wall
                        }
                    }
                };
                Element {
                    id: source.id,
                    floor: 0,
                    kind,
                    points,
                    source: Some(source),
                }
            })
            .collect();
        let indices = self
            .elements
            .iter()
            .enumerate()
            .map(|(i, e)| (e.id, i))
            .collect();
        let info=format!("Узлов: {} · элементов: {}\nГабариты: {:.3} × {:.3} × {:.3} (ед. файла)\nПропущенные типы: {:?}\nДокументы: {:?}",geometry.nodes.len(),self.elements.len(),max[0]-min[0],max[1]-min[1],max[2]-min[2],geometry.skipped,geometry.documents);
        let cache_revision = self.cache.revision + 1;
        self.cache = ViewCache::default();
        self.cache.revision = cache_revision;
        self.cache.all_nodes = Some(local);
        self.import = ImportState {
            pending: None,
            nodes: geometry.nodes,
            indices,
            origin,
            loaded: true,
            center: Vec3::ZERO,
            distance: radius * 3.5,
            default_zoom: radius * 2.7,
            info,
            title: path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
        };
        self.hidden.clear();
        self.selected.clear();
        self.previous_selected = None;
        self.show = [true; 4];
        self.floor = 0;
        self.display.nodes = false;
        self.display.node_numbers = false;
        self.display.element_numbers = false;
        self.zoom = self.import.default_zoom;
        self.pan = egui::Vec2::ZERO;
        (self.yaw, self.pitch) = self.display.projection.angles();
        self.camera_motion = CameraMotion::default();
        self.box_start = None;
        eprintln!(
            "Imported {} nodes / {} elements from {}",
            self.import.nodes.len(),
            self.elements.len(),
            path.display()
        );
    }
    fn drawing_flags(&mut self, ctx: &egui::Context) {
        let mut open = self.display.open;
        egui::Window::new("Флаги рисования")
            .open(&mut open)
            .default_size([490., 300.])
            .collapsible(false)
            .show(ctx, |ui| {
                ui.horizontal_top(|ui| {
                    ui.vertical(|ui| {
                        ui.set_min_width(110.);
                        for (i, label) in ["Узлы", "Элементы", "Общее"].iter().enumerate()
                        {
                            ui.selectable_value(&mut self.display.tab, i, *label);
                        }
                    });
                    ui.separator();
                    ui.vertical(|ui| {
                        ui.set_min_width(300.);
                        match self.display.tab {
                            0 => {
                                ui.heading("Узлы");
                                ui.checkbox(&mut self.display.nodes, "Показывать узлы");
                                ui.checkbox(&mut self.display.node_numbers, "Номера узлов");
                                ui.add(
                                    egui::Slider::new(&mut self.display.node_size, 1.5..=7.)
                                        .text("Размер точки"),
                                );
                                ui.small(
                                    "Номера узлов пока относятся к демонстрационной геометрии.",
                                );
                            }
                            1 => {
                                ui.heading("Элементы");
                                for (i, label) in ["Плиты", "Стены", "Стержни", "Одноузловые КЭ"]
                                    .iter()
                                    .enumerate()
                                {
                                    ui.checkbox(&mut self.show[i], *label);
                                }
                                ui.checkbox(&mut self.wire, "Каркасный режим");
                                ui.checkbox(&mut self.display.outlines, "Границы оболочек");
                                ui.checkbox(&mut self.display.element_numbers, "Номера элементов");
                            }
                            _ => {
                                ui.heading("Общее");
                                let before = self.display.projection;
                                egui::ComboBox::from_label("Проекция")
                                    .selected_text(before.name())
                                    .show_ui(ui, |ui| {
                                        for p in [
                                            Projection::Isometric,
                                            Projection::Dimetric,
                                            Projection::Perspective,
                                        ] {
                                            ui.selectable_value(
                                                &mut self.display.projection,
                                                p,
                                                p.name(),
                                            );
                                        }
                                    });
                                if self.display.projection != before {
                                    (self.yaw, self.pitch) = self.display.projection.angles();
                                    self.pan = egui::Vec2::ZERO;
                                }
                                ui.horizontal(|ui| {
                                    ui.label("Цвет фона");
                                    ui.color_edit_button_srgba(&mut self.display.background);
                                });
                                ui.add(
                                    egui::Slider::new(
                                        &mut self.zoom,
                                        self.import.default_zoom * 0.02
                                            ..=self.import.default_zoom * 20.,
                                    )
                                    .text("Масштаб / охват"),
                                );
                                if ui.button("Исходный вид").clicked() {
                                    (self.yaw, self.pitch) = self.display.projection.angles();
                                    self.zoom = self.import.default_zoom;
                                    self.pan = egui::Vec2::ZERO;
                                }
                            }
                        }
                        ui.separator();
                        ui.small("Изменения применяются сразу.");
                    });
                });
            });
        self.display.open = open;
    }
}
fn distance(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2) -> f32 {
    let ab = b - a;
    let t = ((p - a).dot(ab) / ab.length_sq().max(0.001)).clamp(0., 1.);
    p.distance(a + ab * t)
}
fn inside(p: egui::Pos2, a: egui::Pos2, b: egui::Pos2, c: egui::Pos2) -> bool {
    let cross = |u: egui::Vec2, v: egui::Vec2| u.x * v.y - u.y * v.x;
    let s = [
        cross(b - a, p - a),
        cross(c - b, p - b),
        cross(a - c, p - c),
    ];
    s.iter().all(|x| *x >= 0.) || s.iter().all(|x| *x <= 0.)
}
// Liang-Barsky clipping handles segments crossing a box with both ends outside.
fn segment_crosses_rect(r: egui::Rect, a: egui::Pos2, b: egui::Pos2) -> bool {
    let d = b - a;
    let mut low: f32 = 0.;
    let mut high: f32 = 1.;
    for (p, q) in [
        (-d.x, a.x - r.min.x),
        (d.x, r.max.x - a.x),
        (-d.y, a.y - r.min.y),
        (d.y, r.max.y - a.y),
    ] {
        if p.abs() < 1e-6 {
            if q < 0. {
                return false;
            }
        } else {
            let t = q / p;
            if p < 0. {
                low = low.max(t);
            } else {
                high = high.min(t);
            }
            if low > high {
                return false;
            }
        }
    }
    true
}
fn box_contains_element(r: egui::Rect, points: &[egui::Pos2], crossing: bool) -> bool {
    if points.is_empty() {
        return false;
    }
    if !crossing {
        return points.iter().all(|p| r.contains(*p));
    }
    if points.iter().any(|p| r.contains(*p)) {
        return true;
    }
    let count = if points.len() == 2 { 1 } else { points.len() };
    if (0..count).any(|i| segment_crosses_rect(r, points[i], points[(i + 1) % points.len()])) {
        return true;
    }
    points.len() >= 3
        && (1..points.len() - 1).any(|i| inside(r.center(), points[0], points[i], points[i + 1]))
}
// NDC depth is affine over a rasterized triangle, including perspective projection.
fn surface_depth(point: Vec3, triangle: [Vec3; 3]) -> Option<f32> {
    let [a, b, c] = triangle;
    let ab = b.truncate() - a.truncate();
    let ac = c.truncate() - a.truncate();
    let ap = point.truncate() - a.truncate();
    let den = ab.perp_dot(ac);
    if den.abs() < 1e-10 {
        return None;
    }
    let u = ap.perp_dot(ac) / den;
    let v = ab.perp_dot(ap) / den;
    if u < -1e-5 || v < -1e-5 || u + v > 1.0 + 1e-5 {
        return None;
    }
    Some(a.z * (1.0 - u - v) + b.z * u + c.z * v)
}
#[cfg(test)]
fn label_visible(point: Vec3, surfaces: &[[Vec3; 3]]) -> bool {
    !surfaces.iter().any(|triangle| {
        surface_depth(point, *triangle).is_some_and(|depth| depth < point.z - 0.0001)
    })
}
fn apply_selection(
    selected: &mut HashSet<usize>,
    ids: impl IntoIterator<Item = usize>,
    modifiers: egui::Modifiers,
) {
    if !modifiers.shift && !modifiers.ctrl {
        selected.clear();
    }
    for id in ids {
        if modifiers.ctrl {
            selected.remove(&id);
        } else {
            selected.insert(id);
        }
    }
}
impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _: &mut eframe::Frame) {
        self.poll_import(ctx);
        let update_started = std::time::Instant::now();
        self.performance.frames += 1;
        let elapsed = self.performance.window.elapsed().as_secs_f32();
        if elapsed >= 1.0 {
            self.performance.fps = self.performance.frames as f32 / elapsed;
            self.performance.frames = 0;
            self.performance.window = update_started;
        }
        if let Some(frame) = self.performance.benchmark {
            if frame == 0 {
                self.performance.benchmark_start = Some(update_started);
            }
            if self.performance.benchmark_orbit {
                self.yaw += 0.02;
            }
            if frame >= 60
                && self
                    .performance
                    .benchmark_start
                    .unwrap()
                    .elapsed()
                    .as_secs_f32()
                    >= 3.0
            {
                eprintln!(
                    "Benchmark: {} total / {} visible, orbit={}, {:.1} FPS (last sample), {:.1} ms CPU preparation",
                    self.elements.len(),
                    self.cache.indices.len(),
                    self.performance.benchmark_orbit,
                    self.performance.fps,
                    self.performance.cpu_ms
                );
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
                self.performance.benchmark = None;
            } else {
                self.performance.benchmark = Some(frame + 1);
            }
        }
        let escape = ctx.input(|i| i.key_pressed(egui::Key::Escape));
        if escape {
            self.box_start = None;
            self.set_selection(HashSet::new());
        }
        if let Some(frame) = self.smoke_frame {
            self.cache.dirty = true;
            self.display.projection = [
                Projection::Isometric,
                Projection::Dimetric,
                Projection::Perspective,
            ][(frame / 12).min(2)];
            (self.yaw, self.pitch) = self.display.projection.angles();
            self.display.nodes = true;
            self.display.node_numbers = true;
            self.display.element_numbers = true;
            self.hidden.clear();
            self.show = [true; 4];
            self.wire = false;
            match (frame / 3) % 4 {
                0 => {
                    self.hidden = self
                        .elements
                        .iter()
                        .filter(|e| e.kind != Kind::Column)
                        .map(|e| e.id)
                        .collect();
                }
                1 => {
                    self.wire = true;
                }
                2 => {
                    self.hidden = self.elements.iter().map(|e| e.id).collect();
                }
                3 => {
                    self.show = [false; 4];
                }
                _ => {}
            }
            if frame >= 36 {
                ctx.send_viewport_cmd(egui::ViewportCommand::Close);
            }
            self.smoke_frame = Some(frame + 1);
            ctx.request_repaint();
        }
        self.refresh_visibility();
        egui::SidePanel::left("tools").resizable(false).default_width(240.).show(ctx,|ui| {
            ui.heading("MINI FEM"); ui.label(&self.import.title); ui.separator();
            if ui.add_enabled(self.import.pending.is_none(),egui::Button::new("Импорт геометрии TXT ЛИРА")).clicked() {self.start_import();}
            if self.import.pending.is_some() {ui.spinner();ui.label("Чтение геометрии…");}
            if !self.import.info.is_empty() {ui.small(&self.import.info);}
            if self.import.loaded {ui.small(format!("Начало отображения XYZ: {:.3}, {:.3}, {:.3}",self.import.origin[0],self.import.origin[1],self.import.origin[2]));}
            ui.checkbox(&mut self.performance.continuous,"Непрерывная отрисовка");
            ui.label(format!("FPS: {:.1} | CPU: {:.1} ms",self.performance.fps,self.performance.cpu_ms));
            if ui.button("Флаги рисования").clicked() {self.display.open=true;}
            for (i,label) in ["Плиты","Стены","Стержни","Одноузловые КЭ"].iter().enumerate() { ui.checkbox(&mut self.show[i],*label); }
            ui.checkbox(&mut self.wire,"Wireframe");
            ui.label("Box selection");
            ui.small("Box: left → right = contained; right → left = crossing. Hidden elements are excluded.");
            ui.add_enabled_ui(!self.import.loaded,|ui| {egui::ComboBox::from_label("Floor").selected_text(if self.floor==0 {"All floors".into()} else {format!("Floor {}",self.floor)}).show_ui(ui,|ui| {ui.selectable_value(&mut self.floor,0,"All floors"); for i in 1..=5 {ui.selectable_value(&mut self.floor,i,format!("Floor {i}"));}});});
            self.refresh_visibility();
            ui.separator(); ui.label(format!("Elements: {}",self.elements.len())); ui.label(format!("Visible: {}",self.cache.indices.len())); ui.label(format!("Selected: {}",self.selected.len()));
            if ui.button("Hide selected").clicked() {self.hide_selected();}
            if ui.button("Isolate selected").clicked() {self.fragment_selected();}
            if ui.button("Show all / clear selection").clicked() {self.restore_visibility();self.set_selection(HashSet::new());}
            if ui.button("Reset camera").clicked() {(self.yaw,self.pitch)=self.display.projection.angles();self.zoom=self.import.default_zoom;self.pan=egui::Vec2::ZERO;}
            ui.separator(); ui.label("Drag left: box select\nDrag right: orbit\nRight click: context menu\nDrag middle: pan\nWheel: zoom\nClick element / box: replace selection\nShift: add to selection\nCtrl: remove from selection\nEsc: clear selection / cancel box");
            if self.selected.len()==1 {let id=*self.selected.iter().next().unwrap();if let Some(e)=self.element_by_id(id) {ui.separator();ui.label(format!("Элемент #{}\n{:?}\nУзлов: {}",e.id,e.kind,e.points.len()));if let Some(source)=&e.source{ui.label(format!("КЭ: {} · жёсткость: {}\nУзлы ЛИРА: {:?}",source.fe_type,source.stiffness,source.nodes));}}}
            ui.separator();ui.small("Просмотр геометрии · без расчётного ядра");
        });
        if self.display.open {
            self.drawing_flags(ctx);
        }
        self.refresh_visibility();
        if self.display.nodes || self.display.node_numbers {
            self.refresh_nodes();
        }
        egui::CentralPanel::default()
            .frame(egui::Frame::NONE.fill(self.display.background))
            .show(ctx, |ui| {
                let (rect, response) =
                    ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
                let primary = egui::PointerButton::Primary;
                if response.drag_started_by(primary) && !escape {
                    self.box_start = ctx.input(|i| i.pointer.press_origin());
                }
                let box_end = ctx
                    .input(|i| i.pointer.interact_pos())
                    .map(|p| rect.clamp(p));
                let selection_box = self
                    .box_start
                    .zip(box_end)
                    .map(|(a, b)| (egui::Rect::from_two_pos(a, b), b.x < a.x));
                let finish_box =
                    self.box_start.is_some() && ctx.input(|i| i.pointer.button_released(primary));
                let mut box_hits = Vec::new();
                if response.dragged_by(egui::PointerButton::Secondary) {
                    let d = ctx.input(|i| i.pointer.delta());
                    self.yaw -= d.x * 0.008;
                    self.pitch = (self.pitch + d.y * 0.008).clamp(-1.4, 1.4);
                }
                if response.dragged_by(egui::PointerButton::Middle) {
                    self.pan += ctx.input(|i| i.pointer.delta());
                }
                if response.hovered() {
                    self.zoom = (self.zoom
                        * (-ctx.input(|i| i.smooth_scroll_delta.y) * 0.002).exp())
                    .clamp(
                        self.import.default_zoom * 0.02,
                        self.import.default_zoom * 20.,
                    );
                }
                let requested_numbers = number_count(
                    self.display.node_numbers,
                    self.display.element_numbers,
                    self.cache.node_indices.as_ref().map_or(0, Vec::len),
                    self.cache.indices.len(),
                );
                let now = std::time::Instant::now();
                let numbers_suspended = self.camera_motion.suspend_numbers(
                    requested_numbers,
                    [self.yaw, self.pitch, self.zoom, self.pan.x, self.pan.y],
                    now,
                );
                if numbers_suspended {
                    let remaining = CAMERA_QUIET
                        .saturating_sub(now.duration_since(self.camera_motion.moved_at.unwrap()));
                    ctx.request_repaint_after(remaining);
                }
                let node_numbers = self.display.node_numbers && !numbers_suspended;
                let element_numbers = self.display.element_numbers && !numbers_suspended;
                let center = self.import.center;
                let eye = center
                    + Vec3::new(
                        self.yaw.cos() * self.pitch.cos(),
                        self.yaw.sin() * self.pitch.cos(),
                        self.pitch.sin(),
                    ) * self.import.distance;
                let aspect = rect.width() / rect.height().max(1.);
                let projection = if self.display.projection == Projection::Perspective {
                    Mat4::perspective_rh(
                        2.0 * (self.zoom / (self.import.distance * 2.)).atan(),
                        aspect,
                        self.import.distance * 0.0001,
                        self.import.distance * 4.,
                    )
                } else {
                    Mat4::orthographic_rh(
                        -self.zoom * aspect / 2.,
                        self.zoom * aspect / 2.,
                        -self.zoom / 2.,
                        self.zoom / 2.,
                        self.import.distance * 0.0001,
                        self.import.distance * 4.,
                    )
                };
                let matrix = projection * Mat4::look_at_rh(eye, center, Vec3::Z);
                let pan = self.pan;
                let project = |p: Vec3| {
                    let mut q = matrix.project_point3(p);
                    q.x += pan.x * 2. / rect.width();
                    q.y -= pan.y * 2. / rect.height();
                    q
                };
                let screen = |p: Vec3| {
                    egui::pos2(
                        rect.center().x + p.x * rect.width() / 2.,
                        rect.center().y - p.y * rect.height() / 2.,
                    )
                };
                let mut faces = vec![];
                let mut edges = vec![];
                let mut label_surfaces = Vec::new();
                let labels_enabled = self.display.nodes || node_numbers || element_numbers;
                let mut hit: Option<(usize, f32)> = None;
                let pointer = if response.clicked_by(primary) && !escape {
                    response.interact_pointer_pos()
                } else {
                    None
                };
                let key = RenderKey {
                    camera: [
                        self.yaw,
                        self.pitch,
                        self.zoom,
                        self.pan.x,
                        self.pan.y,
                        rect.width(),
                        rect.height(),
                        0.,
                    ],
                    perspective: self.display.projection == Projection::Perspective,
                    wire: self.wire,
                    outlines: self.display.outlines,
                    labels: labels_enabled,
                    visibility: self.cache.visibility_revision,
                    selection: self.cache.selection_revision,
                };
                let rebuild = self.cache.render_key.as_ref() != Some(&key);
                if rebuild || pointer.is_some() || finish_box {
                    for index in &self.cache.indices {
                        let e = &self.elements[*index];
                        let mut projected_points = [Vec3::ZERO; 4];
                        for (out, point) in projected_points.iter_mut().zip(&e.points) {
                            *out = project(*point);
                        }
                        let ps = &projected_points[..e.points.len()];
                        if finish_box {
                            if let Some((r, crossing)) = selection_box {
                                let projected: Vec<_> = ps.iter().copied().map(screen).collect();
                                if box_contains_element(r, &projected, crossing) {
                                    box_hits.push(e.id);
                                }
                            }
                        }
                        let color = if self.selected.contains(&e.id) {
                            [1., 0.75, 0.75, 1.]
                        } else {
                            match e.kind {
                                Kind::Slab => [0.30, 0.57, 0.76, 1.],
                                Kind::Wall => [0.40, 0.72, 0.60, 1.],
                                Kind::Column => [0.9, 0.91, 0.95, 1.],
                                Kind::Point => [0.76, 0.50, 0.94, 1.],
                            }
                        };
                        if rebuild && e.points.len() >= 3 && !self.wire {
                            for i in 1..ps.len() - 1 {
                                if labels_enabled {
                                    label_surfaces.push([ps[0], ps[i], ps[i + 1]]);
                                }
                                for k in [0, i, i + 1] {
                                    faces.push(Vertex {
                                        position: ps[k].to_array(),
                                        color,
                                    });
                                }
                            }
                        }
                        let edge_color = if self.selected.contains(&e.id) {
                            [1., 0.12, 0.12, 1.]
                        } else if e.kind == Kind::Column || e.kind == Kind::Point {
                            color
                        } else {
                            [0.08, 0.15, 0.22, 1.]
                        };
                        let n = if ps.len() == 1 {
                            0
                        } else if ps.len() == 2 {
                            1
                        } else if self.display.outlines
                            || self.wire
                            || self.selected.contains(&e.id)
                        {
                            ps.len()
                        } else {
                            0
                        };
                        if rebuild {
                            if ps.len() == 1 {
                                let p = ps[0];
                                for offset in [
                                    Vec3::new(7. / rect.width(), 0., 0.),
                                    Vec3::new(0., 7. / rect.height(), 0.),
                                ] {
                                    for q in [p - offset, p + offset] {
                                        edges.push(Vertex {
                                            position: (q - Vec3::Z * 0.0001).to_array(),
                                            color: edge_color,
                                        });
                                    }
                                }
                            }
                            for i in 0..n {
                                for k in [i, (i + 1) % ps.len()] {
                                    let mut p = ps[k];
                                    p.z -= 0.0001;
                                    edges.push(Vertex {
                                        position: p.to_array(),
                                        color: edge_color,
                                    });
                                }
                            }
                        }
                        if let Some(p) = pointer {
                            let mut depth = None;
                            if ps.len() == 1 {
                                if p.distance(screen(ps[0])) < 6. {
                                    depth = Some(ps[0].z - 0.001);
                                }
                            } else if ps.len() == 2 {
                                if distance(p, screen(ps[0]), screen(ps[1])) < 6. {
                                    depth = Some((ps[0].z + ps[1].z) / 2. - 0.001);
                                }
                            } else {
                                for i in 1..ps.len() - 1 {
                                    let a = screen(ps[0]);
                                    let b = screen(ps[i]);
                                    let c = screen(ps[i + 1]);
                                    if inside(p, a, b, c) {
                                        let v0 = b - a;
                                        let v1 = c - a;
                                        let v2 = p - a;
                                        let den = v0.x * v1.y - v1.x * v0.y;
                                        if den.abs() > 0.0001 {
                                            let u = (v2.x * v1.y - v1.x * v2.y) / den;
                                            let v = (v0.x * v2.y - v2.x * v0.y) / den;
                                            depth = Some(
                                                ps[0].z * (1. - u - v)
                                                    + ps[i].z * u
                                                    + ps[i + 1].z * v,
                                            );
                                        }
                                    }
                                }
                            }
                            if let Some(d) = depth {
                                if hit.is_none_or(|(_, old)| d < old) {
                                    hit = Some((e.id, d));
                                }
                            }
                        }
                    }
                }
                if rebuild {
                    self.cache.revision += 1;
                    self.cache.draw = Some(Draw {
                        faces: Arc::new(faces),
                        edges: Arc::new(edges),
                        revision: self.cache.revision,
                    });
                    self.cache.surfaces = Arc::new(label_surfaces);
                    self.cache.depth_index = Some(DepthIndex::new(self.cache.surfaces.clone()));
                    self.cache.render_key = Some(key);
                }
                if finish_box {
                    self.select(box_hits, ctx.input(|i| i.modifiers));
                    self.box_start = None;
                    ctx.request_repaint();
                } else if let Some((id, _)) = hit {
                    self.select([id], ctx.input(|i| i.modifiers));
                    ctx.request_repaint();
                }
                ui.painter().add(egui_wgpu::Callback::new_paint_callback(
                    rect,
                    self.cache.draw.as_ref().unwrap().clone(),
                ));
                if !finish_box {
                    if let Some((r, crossing)) = selection_box {
                        let color = if crossing {
                            egui::Color32::LIGHT_GREEN
                        } else {
                            egui::Color32::LIGHT_BLUE
                        };
                        ui.painter().rect_filled(r, 0., color.gamma_multiply(0.12));
                        ui.painter().rect_stroke(
                            r,
                            0.,
                            egui::Stroke::new(1.5_f32, color),
                            egui::StrokeKind::Inside,
                        );
                    }
                }
                ui.painter().text(
                    rect.left_top() + egui::vec2(18., 18.),
                    egui::Align2::LEFT_TOP,
                    format!("STRUCTURAL MODEL / {}", self.display.projection.name()),
                    egui::FontId::monospace(13.),
                    egui::Color32::from_rgb(170, 190, 210),
                );
                if numbers_suspended {
                    ui.painter().text(
                        rect.left_top() + egui::vec2(18., 40.),
                        egui::Align2::LEFT_TOP,
                        "Номера временно скрыты при движении камеры",
                        egui::FontId::proportional(12.),
                        egui::Color32::LIGHT_GRAY,
                    );
                }
                let overlay_key = OverlayKey {
                    revision: self.cache.revision,
                    rect: [rect.min.x, rect.min.y, rect.max.x, rect.max.y],
                    pixels_per_point: ctx.pixels_per_point(),
                    nodes: self.display.nodes,
                    node_numbers,
                    element_numbers,
                    node_size: self.display.node_size,
                };
                if self.cache.overlay_key.as_ref() != Some(&overlay_key) {
                    let mut shapes = Vec::new();
                    let depth = self.cache.depth_index.as_ref().unwrap();
                    if self.display.nodes || node_numbers {
                        for i in self.cache.node_indices.as_ref().unwrap() {
                            let p = self.cache.all_nodes.as_ref().unwrap()[*i];
                            let q = project(p);
                            let pos = screen(q);
                            if !rect.contains(pos)
                                || !(0.0..=1.0).contains(&q.z)
                                || !depth.visible(q)
                            {
                                continue;
                            }
                            if self.display.nodes {
                                shapes.push(egui::Shape::circle_filled(
                                    pos,
                                    self.display.node_size,
                                    egui::Color32::from_rgb(240, 195, 110),
                                ));
                            }
                            if node_numbers {
                                shapes.push(number_shape(
                                    ui.painter(),
                                    pos + egui::vec2(4., -4.),
                                    egui::Align2::LEFT_BOTTOM,
                                    if self.import.loaded {
                                        self.import.nodes[*i].id
                                    } else {
                                        i + 1
                                    },
                                    egui::Color32::LIGHT_YELLOW,
                                ));
                            }
                        }
                    }
                    if element_numbers {
                        for index in &self.cache.indices {
                            let e = &self.elements[*index];
                            let p = e.points.iter().copied().sum::<Vec3>() / e.points.len() as f32;
                            let q = project(p);
                            let pos = screen(q);
                            if rect.contains(pos) && (0.0..=1.0).contains(&q.z) && depth.visible(q)
                            {
                                shapes.push(number_shape(
                                    ui.painter(),
                                    pos,
                                    egui::Align2::CENTER_CENTER,
                                    e.id,
                                    egui::Color32::WHITE,
                                ));
                            }
                        }
                    }
                    self.cache.overlay_shapes = shapes;
                    self.cache.overlay_key = Some(overlay_key);
                }
                ui.painter()
                    .extend(self.cache.overlay_shapes.iter().cloned());
                response.context_menu(|ui| {
                    if ui
                        .add_enabled(!self.hidden.is_empty(), egui::Button::new("Восстановить"))
                        .clicked()
                    {
                        self.restore_visibility();
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(
                            !self.selected.is_empty(),
                            egui::Button::new("Скрыть выделенное"),
                        )
                        .clicked()
                    {
                        self.hide_selected();
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(
                            !self.selected.is_empty(),
                            egui::Button::new("Фрагментировать выделенное"),
                        )
                        .clicked()
                    {
                        self.fragment_selected();
                        ui.close_menu();
                    }
                    if ui
                        .add_enabled(
                            self.previous_selected.is_some(),
                            egui::Button::new("Предыдущее выделение"),
                        )
                        .clicked()
                    {
                        self.restore_previous_selection();
                        ui.close_menu();
                    }
                });
            });
        self.performance.cpu_ms = update_started.elapsed().as_secs_f32() * 1000.;
        if self.performance.continuous || self.performance.benchmark.is_some() {
            ctx.request_repaint();
        }
    }
}
fn main() -> eframe::Result {
    let args: Vec<_> = std::env::args_os().collect();
    if let Some(index) = args.iter().position(|a| a == "--check-lira") {
        let Some(path) = args.get(index + 1) else {
            eprintln!("Укажите файл после --check-lira");
            std::process::exit(1);
        };
        match lira::read(std::path::Path::new(path)) {
            Ok(geometry) => {
                let mut types = std::collections::BTreeMap::new();
                for e in &geometry.elements {
                    *types.entry(e.fe_type).or_insert(0_usize) += 1;
                }
                println!(
                    "Nodes: {}\nElements: {}\nTypes: {:?}\nSkipped: {:?}\nDocuments: {:?}",
                    geometry.nodes.len(),
                    geometry.elements.len(),
                    types,
                    geometry.skipped,
                    geometry.documents
                );
                return Ok(());
            }
            Err(error) => {
                eprintln!("{error}");
                std::process::exit(1);
            }
        }
    }
    let options = eframe::NativeOptions {
        renderer: eframe::Renderer::Wgpu,
        depth_buffer: 24,
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280., 850.])
            .with_title("Mini FEM · Building viewer"),
        ..Default::default()
    };
    eframe::run_native(
        "Mini FEM",
        options,
        Box::new(|cc| Ok(Box::new(App::new(cc)))),
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    fn p(x: f32, y: f32) -> egui::Pos2 {
        egui::pos2(x, y)
    }
    #[test]
    fn imported_geometry_keeps_ids_topology_and_coincident_nodes() {
        let mut app = App {
            elements: building_with_refinement(1),
            selected: HashSet::from([1]),
            previous_selected: None,
            hidden: HashSet::from([2]),
            show: [true; 4],
            floor: 5,
            yaw: 0.,
            pitch: 0.,
            zoom: 27.,
            pan: egui::Vec2::ZERO,
            wire: false,
            display: DisplayOptions::default(),
            performance: Performance::default(),
            cache: ViewCache::default(),
            camera_motion: CameraMotion::default(),
            import: ImportState::default(),
            box_start: None,
            smoke_frame: None,
        };
        app.cache.revision = 42;
        let geometry=lira::parse("(4/1000000 2000000 3/1000001 2000000 3/1000000 2000001 3/1000001 2000001 3/1000000 2000000 3/)(1/999 1 1/44 2 1 2 3 4/56 3 5/)").unwrap();
        app.apply_import("test.txt".into(), geometry);
        assert_eq!(app.elements.len(), 2);
        assert_eq!(
            app.element_by_id(2).unwrap().source.as_ref().unwrap().nodes,
            vec![1, 2, 3, 4]
        );
        assert!(app.element_by_id(1).is_none());
        let points = &app.element_by_id(2).unwrap().points;
        assert_eq!(points[2], Vec3::new(0.5, 0.5, 0.));
        assert_eq!(app.import.nodes[0].xyz, [1000000., 2000000., 3.]);
        assert_eq!(app.import.nodes.len(), 5);
        assert!(app.selected.is_empty() && app.hidden.is_empty());
        assert_eq!(app.floor, 0);
        assert!(app.cache.revision > 42);
        assert!(app.zoom.is_finite() && app.zoom > 0.);
        app.refresh_visibility();
        app.refresh_nodes();
        assert_eq!(app.cache.node_indices.as_ref().unwrap().len(), 5);
        app.select([2], egui::Modifiers::NONE);
        app.set_selection(HashSet::new());
        app.restore_previous_selection();
        assert_eq!(app.selected, HashSet::from([2]));
        app.select([3], egui::Modifiers::NONE);
        app.hide_selected();
        app.refresh_visibility();
        app.refresh_nodes();
        assert_eq!(app.cache.node_indices.as_ref().unwrap(), &vec![0, 1, 2, 3]);
        assert_eq!(app.element_by_id(3).unwrap().kind, Kind::Point);
    }
    #[test]
    fn depth_grid_matches_full_triangle_search() {
        let mut triangles = vec![
            [
                Vec3::new(-2., -2., 0.2),
                Vec3::new(2., -2., 0.4),
                Vec3::new(0., 2., 0.3),
            ],
            [Vec3::ZERO; 3],
            [
                Vec3::new(-0.5, -0.5, 0.1),
                Vec3::new(-0.4, -0.5, 0.1),
                Vec3::new(-0.5, -0.4, 0.1),
            ],
        ];
        let mut seed = 42_u32;
        let mut random = || {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            seed as f32 / u32::MAX as f32
        };
        for _ in 0..100 {
            let p = Vec3::new(random() * 4. - 2., random() * 4. - 2., random());
            triangles.push([
                p,
                p + Vec3::new(random() * 0.4, 0., 0.1),
                p + Vec3::new(0., random() * 0.4, -0.1),
            ]);
        }
        let triangles = Arc::new(triangles);
        let index = DepthIndex::new(triangles.clone());
        for _ in 0..2000 {
            let p = Vec3::new(random() * 2. - 1., random() * 2. - 1., random());
            assert_eq!(index.visible(p), label_visible(p, &triangles), "{p:?}");
        }
        for p in [
            Vec3::new(-0.5, -0.5, 0.9),
            Vec3::new(-0.5000001, -0.5, 0.9),
            Vec3::new(1., 1., 0.9),
            Vec3::ZERO,
        ] {
            assert_eq!(index.visible(p), label_visible(p, &triangles));
        }
        assert!(DepthIndex::new(Arc::new(vec![])).visible(Vec3::ZERO));
    }
    #[test]
    fn dense_numbers_hide_only_during_camera_motion() {
        let mut motion = CameraMotion::default();
        let start = std::time::Instant::now();
        let camera = [0.; 5];
        assert!(!motion.suspend_numbers(5001, camera, start));
        let rotated = [1., 0., 0., 0., 0.];
        let moved = start + std::time::Duration::from_millis(10);
        assert!(motion.suspend_numbers(5001, rotated, moved));
        assert!(!motion.suspend_numbers(5000, rotated, moved));
        assert!(motion.suspend_numbers(
            5001,
            rotated,
            moved + std::time::Duration::from_millis(100)
        ));
        assert!(!motion.suspend_numbers(5001, rotated, moved + CAMERA_QUIET));
        let panned = [1., 0., 0., 2., 0.];
        assert!(motion.suspend_numbers(5001, panned, moved + CAMERA_QUIET));
        assert_eq!(number_count(true, true, 2501, 2500), 5001);
        assert_eq!(number_count(false, true, 9999, 3000), 3000);
        assert_eq!(number_count(true, false, 3000, 9999), 3000);
        assert_eq!(number_count(false, false, 9999, 9999), 0);
    }
    #[test]
    fn refined_building_has_half_million_elements() {
        let model = building();
        assert_eq!(model.len(), 502_350);
        assert_eq!(
            model.iter().filter(|e| e.kind == Kind::Slab).count(),
            392_000
        );
        assert_eq!(
            model.iter().filter(|e| e.kind == Kind::Wall).count(),
            110_250
        );
        assert_eq!(model.iter().filter(|e| e.kind == Kind::Column).count(), 100);
        assert_eq!(
            model.iter().filter(|e| e.points.len() == 3).count(),
            269_500
        );
        assert!(model
            .iter()
            .enumerate()
            .all(|(i, e)| e.id == i + 1 && e.points.iter().all(|p| p.is_finite())));
    }
    #[test]
    fn labels_respect_surface_depth() {
        let triangle = [
            Vec3::new(-1., -1., 0.2),
            Vec3::new(1., -1., 0.4),
            Vec3::new(0., 1., 0.3),
        ];
        assert!(!label_visible(Vec3::new(0., 0., 0.8), &[triangle]));
        assert!(label_visible(Vec3::new(0., 0., 0.1), &[triangle]));
        assert!(label_visible(Vec3::new(0., 0., 0.3), &[triangle]));
        assert!(!label_visible(Vec3::new(-1., -1., 0.8), &[triangle]));
        assert!(label_visible(Vec3::new(2., 0., 0.8), &[triangle]));
        assert!(label_visible(Vec3::new(0., 0., 0.8), &[]));
        assert!(surface_depth(Vec3::ZERO, [Vec3::ZERO; 3]).is_none());
        assert!((surface_depth(Vec3::ZERO, triangle).unwrap() - 0.3).abs() < 1e-6);
    }
    #[test]
    fn box_geometry() {
        let r = egui::Rect::from_min_max(p(0., 0.), p(10., 10.));
        assert!(box_contains_element(r, &[p(1., 1.), p(9., 9.)], false));
        assert!(!box_contains_element(r, &[p(-5., 5.), p(15., 5.)], false));
        assert!(box_contains_element(r, &[p(-5., 5.), p(15., 5.)], true));
        assert!(!box_contains_element(r, &[p(-5., 15.), p(15., 15.)], true));
        assert!(box_contains_element(
            r,
            &[p(-20., -20.), p(30., -20.), p(30., 30.), p(-20., 30.)],
            true
        ));
        assert!(!box_contains_element(
            r,
            &[p(-20., -20.), p(-10., -20.), p(-10., -10.)],
            true
        ));
        assert!(segment_crosses_rect(r, p(0., 0.), p(0., 0.)));
    }
    #[test]
    fn selection_modifiers() {
        let mut selected = HashSet::from([1, 2]);
        apply_selection(&mut selected, [3], egui::Modifiers::SHIFT);
        assert_eq!(selected, HashSet::from([1, 2, 3]));
        apply_selection(&mut selected, [2, 4], egui::Modifiers::CTRL);
        assert_eq!(selected, HashSet::from([1, 3]));
        apply_selection(&mut selected, [4], egui::Modifiers::NONE);
        assert_eq!(selected, HashSet::from([4]));
        apply_selection(&mut selected, [], egui::Modifiers::NONE);
        assert!(selected.is_empty());
    }
    #[test]
    fn selection_history_and_hidden_restore() {
        let mut app = App {
            elements: building_with_refinement(1),
            selected: HashSet::new(),
            previous_selected: None,
            hidden: HashSet::new(),
            show: [true; 4],
            floor: 0,
            yaw: 0.,
            pitch: 0.,
            zoom: 27.,
            pan: egui::Vec2::ZERO,
            wire: false,
            display: DisplayOptions::default(),
            performance: Performance::default(),
            cache: ViewCache::default(),
            camera_motion: CameraMotion::default(),
            import: ImportState::default(),
            box_start: None,
            smoke_frame: None,
        };
        app.select([1, 2], egui::Modifiers::NONE);
        app.select([3], egui::Modifiers::SHIFT);
        app.restore_previous_selection();
        assert_eq!(app.selected, HashSet::from([1, 2]));
        app.restore_previous_selection();
        assert_eq!(app.selected, HashSet::from([1, 2, 3]));
        // A no-op must preserve history (including a click on an already selected element).
        let previous = app.previous_selected.clone();
        app.select([1], egui::Modifiers::SHIFT);
        assert_eq!(app.previous_selected, previous);
        app.set_selection(HashSet::new());
        assert!(app.selected.is_empty());
        app.restore_previous_selection();
        assert_eq!(app.selected, HashSet::from([1, 2, 3]));
        app.hide_selected();
        app.restore_previous_selection();
        assert!(app.selected.is_empty());
        app.hidden.clear();
        app.select([1], egui::Modifiers::NONE);
        app.fragment_selected();
        assert!(app.visible(&app.elements[0]));
        assert!(!app.visible(&app.elements[1]));
    }
    #[test]
    fn hidden_and_filtered_elements_are_excluded() {
        // Visibility must be checked before projecting any selection candidates.
        let app = App {
            elements: building_with_refinement(1),
            selected: HashSet::new(),
            previous_selected: None,
            hidden: HashSet::from([1]),
            show: [true; 4],
            floor: 0,
            yaw: 0.,
            pitch: 0.,
            zoom: 27.,
            pan: egui::Vec2::ZERO,
            wire: false,
            display: DisplayOptions::default(),
            performance: Performance::default(),
            cache: ViewCache::default(),
            camera_motion: CameraMotion::default(),
            import: ImportState::default(),
            box_start: None,
            smoke_frame: None,
        };
        assert!(!app.visible(&app.elements[0]));
        assert!(app.visible(&app.elements[1]));
        let mut app = app;
        app.show[2] = false;
        assert!(!app.visible(&app.elements[1]));
        app.show[2] = true;
        app.floor = 5;
        assert!(!app.visible(&app.elements[1]));
        app.floor = 0;
        app.show = [true; 4];
        app.hidden.clear();
        app.cache.dirty = true;
        app.refresh_visibility();
        assert_eq!(app.cache.indices.len(), 510);
        let revision = app.cache.visibility_revision;
        app.refresh_visibility();
        assert_eq!(app.cache.visibility_revision, revision);
        app.set_selection(HashSet::from([1, 2]));
        app.fragment_selected();
        app.refresh_visibility();
        assert_eq!(app.cache.indices, vec![0, 1]);
        app.set_selection(HashSet::from([3, 4]));
        app.fragment_selected();
        app.refresh_visibility();
        assert_eq!(app.cache.indices, vec![2, 3]);
        app.refresh_nodes();
        assert!(!app.cache.node_indices.as_ref().unwrap().is_empty());
        app.show[2] = false;
        app.refresh_visibility();
        assert!(app.cache.indices.is_empty());
        app.refresh_nodes();
        assert!(app.cache.node_indices.as_ref().unwrap().is_empty());
        app.show = [true; 4];
        app.restore_visibility();
        app.refresh_visibility();
        assert_eq!(app.cache.indices.len(), 510);
    }
}
