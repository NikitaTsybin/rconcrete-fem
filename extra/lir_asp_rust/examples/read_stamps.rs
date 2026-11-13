fn main() {
    let a = std::env::args_os().collect::<Vec<_>>();
    let b = std::fs::read(&a[1]).unwrap();
    let out = lir_asp_parser::stamp_loads::read(&b).unwrap();
    std::fs::write(&a[2], serde_json::to_vec_pretty(&out).unwrap()).unwrap();
}
