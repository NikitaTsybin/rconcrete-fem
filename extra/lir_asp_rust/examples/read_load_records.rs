use std::{env, fs};
fn main() {
    let a = env::args_os().collect::<Vec<_>>();
    let b = fs::read(&a[1]).unwrap();
    let requests: Vec<(bool, usize)> = serde_json::from_slice(&fs::read(&a[2]).unwrap()).unwrap();
    let out = requests
        .iter()
        .map(|&(node, o)| {
            if node {
                lir_asp_parser::physical_loads::nodal_record(&b, o)
            } else {
                lir_asp_parser::physical_loads::element_record(&b, o)
            }
            .map(|x| x.0)
        })
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    fs::write(&a[3], serde_json::to_vec(&out).unwrap()).unwrap();
}
