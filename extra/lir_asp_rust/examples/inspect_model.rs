use std::{env, fs};
fn main() {
    let a = env::args_os().collect::<Vec<_>>();
    let b = fs::read(&a[1]).unwrap();
    let m = lir_asp_parser::read_model(&b).unwrap();
    fs::write(&a[2], serde_json::to_vec(&m).unwrap()).unwrap();
}
