use lir_asp_parser::{import_pair, Unit};
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
fn run() -> Result<(), String> {
    let args: Vec<_> = std::env::args_os().skip(1).collect();
    if args.len() < 2 {
        return Err("Usage: lir_asp_parser MODEL.lir RESULTS.asp [m|cm|mm] [repeat]\nJSON to stdout; parse timings to stderr. No app integration.".into());
    }
    let unit = match args.get(2).and_then(|s| s.to_str()).unwrap_or("m") {
        "m" => Unit::Meters,
        "cm" => Unit::Centimeters,
        "mm" => Unit::Millimeters,
        _ => return Err("Invalid unit".into()),
    };
    let repeat = args
        .get(3)
        .and_then(|s| s.to_str())
        .unwrap_or("1")
        .parse::<usize>()
        .map_err(|e| e.to_string())?;
    if !(1..=1000).contains(&repeat) {
        return Err("repeat must be 1..1000".into());
    }
    let bytes = std::fs::read(&args[1]).map_err(|e| e.to_string())?;
    let mut lir = std::fs::File::open(&args[0]).map_err(|e| e.to_string())?;
    let mut last = None;
    for pass in 0..repeat {
        let time = std::time::Instant::now();
        let result = import_pair(&mut lir, &bytes, unit)?;
        eprintln!(
            "pass {}: {:.3} ms, {} retained plates",
            pass + 1,
            time.elapsed().as_secs_f64() * 1000.,
            result.plates.len()
        );
        last = Some(result);
    }
    serde_json::to_writer(std::io::stdout().lock(), &last.unwrap()).map_err(|e| e.to_string())?;
    Ok(())
}
