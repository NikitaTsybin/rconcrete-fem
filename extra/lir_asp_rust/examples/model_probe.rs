fn main() {
    let path = std::env::args_os().nth(1).expect("LIR path");
    let bytes = std::fs::read(path).unwrap();
    let model = lir_asp_parser::read_model(&bytes).unwrap();
    println!(
        "nodes={} elements={} inactive={} inferred={}",
        model.nodes.len(),
        model.elements.len(),
        model.inactive_elements,
        model.inferred_types
    );
    println!("properties={:?}", model.properties);
}
