#![cfg_attr(target_os = "windows", windows_subsystem = "windows")]
#[path = "../plan_trace/mod.rs"]
mod plan_trace;
fn main() -> eframe::Result {
    plan_trace::run()
}
