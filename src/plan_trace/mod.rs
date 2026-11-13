mod limits;
pub use mini_fem::geometry;
mod app;
mod import;
mod snap;
#[allow(dead_code)]
#[path = "../fem_shared/dxf.rs"]
mod source_dxf;
#[cfg(not(target_arch = "wasm32"))]
pub use app::run;
#[cfg(target_arch = "wasm32")]
pub use app::{start_web, stop_web};
mod platform;

mod building_mesh;
mod grid;
mod loads;
mod mesh_backend;
mod mesh_pack;
mod meshing;
mod model;
mod quad_mesh;
mod sketch;

#[path = "../fem_shared/color_picker.rs"]
mod color_picker;

mod fem;

#[allow(dead_code)]
#[path = "../fem_shared/color_scale.rs"]
mod color_scale;
mod result_scale;
#[allow(dead_code)]
#[path = "../fem_shared/scale_file.rs"]
mod scale_file;

mod load_combinations;

#[allow(dead_code)]
#[path = "../fem_shared/render_cache.rs"]
mod render_cache;

mod isofields;

mod underlay;

mod pdf_import;

mod stiffness;

pub(crate) mod reinforcement;
#[cfg(target_arch = "wasm32")]
mod web_worker;

mod scale_input;

mod lira_export;

#[cfg(test)]
#[allow(dead_code)]
#[path = "../lira.rs"]
mod lira_txt_reader;

#[cfg(target_arch = "wasm32")]
pub(crate) use app::solver_worker_run;

#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
pub(crate) fn solver_parallel_configure(channel: String, start: f64, limit: usize) {
    fem::progress::configure(channel, start, limit);
}

#[cfg(target_arch = "wasm32")]
pub(crate) use app::solver_worker_run_bytes;

mod columns;
mod rigid_links;

pub(super) mod foundation;

pub(super) mod intersections;
