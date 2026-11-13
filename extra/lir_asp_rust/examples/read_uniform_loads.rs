use std::{env, fs};
fn main() {
    let a = env::args_os().collect::<Vec<_>>();
    let b = fs::read(&a[1]).unwrap();
    let r = lir_asp_parser::physical_loads::read_uniform(&b).unwrap();
    fs::write(&a[2], serde_json::to_vec(&r).unwrap()).unwrap();
}
