//! Parse scale boundaries without coupling them to the NDM engine.
use super::color_scale;
#[allow(dead_code)]
#[path = "../fem_shared/scale_expression.rs"]
mod expressions;
#[allow(dead_code)]
#[path = "../fem_shared/reinforcement.rs"]
mod reinforcement;
pub use expressions::value;
