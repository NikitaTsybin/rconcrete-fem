//! Offline host-adapter prototype on the previously decoded original SLD.
use mini_fem::soil::*;
use serde_json::{json, Value};
fn main() {
    if let Err(e) = run() {
        eprintln!("{e}");
        std::process::exit(1);
    }
}
fn run() -> Result<(), String> {
    let args = std::env::args().collect::<Vec<_>>();
    let input = args
        .get(1)
        .map(String::as_str)
        .unwrap_or("target/sld_ground_research/parsed_original_s1_s11.json");
    let output = args
        .get(2)
        .map(String::as_str)
        .unwrap_or("target/soil_prototype_report.json");
    let data: Value = serde_json::from_slice(&std::fs::read(input).map_err(|e| e.to_string())?)
        .map_err(|e| e.to_string())?;
    let mut soils = std::collections::BTreeMap::new();
    for s in data["soils"].as_array().ok_or("Missing soils")? {
        let p = &s["E_nu_gamma_c_phi_R0"];
        soils.insert(
            s["id"].as_u64().ok_or("Soil ID")?,
            (p[0].as_f64().ok_or("E")?, p[2].as_f64().ok_or("gamma")?),
        );
    }
    let mut holes = vec![];
    for h in data["drills"].as_array().ok_or("Missing drills")? {
        let a = &h["xy_mouth_depth"];
        let number = |i| a[i].as_f64().ok_or("Borehole number");
        let mut layers = vec![];
        for l in h["strata"].as_array().ok_or("Missing strata")? {
            let id = l[0].as_u64().ok_or("Stratum ID")?;
            let &(e, gamma) = soils.get(&id).ok_or("Unknown IGE")?;
            layers.push(Layer {
                material_id: None,
                bottom_z: l[1].as_f64().ok_or("Layer bottom")?,
                e,
                gamma,
            });
        }
        holes.push(Borehole {
            id: h["id"].as_u64().ok_or("Borehole ID")? as u32,
            xy: [number(0)?, number(1)?],
            mouth_z: number(2)?,
            layers,
        });
    }
    let n = args
        .get(3)
        .and_then(|s| s.parse::<usize>().ok())
        .unwrap_or(10);
    if !(2..=80).contains(&n) {
        return Err("Grid count must be 2..80".into());
    }
    let size = 20.;
    let step = size / n as f64;
    let mut cells = vec![];
    for y in 0..n {
        for x in 0..n {
            cells.push(Cell {
                polygon: vec![],
                coverage: 1.,
                bounds: [
                    x as f64 * step,
                    y as f64 * step,
                    (x + 1) as f64 * step,
                    (y + 1) as f64 * step,
                ],
            });
        }
    }
    let engine = Engine {
        cells,
        holes,
        options: Options {
            base_z: 144.5,
            max_depth: 12.,
            dz: 0.25,
            ..Default::default()
        },
    };
    // Rigid plate: 3 DOFs (translation and two rotations), free perimeter.
    // This callback can be replaced by the Rconcrete shell solve without changing soil code.
    let pressure = |c: &[f64], load: f64| -> Result<Vec<f64>, String> {
        let mut k = [[0.; 4]; 3];
        let mut force = 0.;
        for (cell, &c) in engine.cells.iter().zip(c) {
            let p = cell.center();
            let v = [1., p[0] - 10., p[1] - 10.];
            let a = cell.area();
            for i in 0..3 {
                for j in 0..3 {
                    k[i][j] += c * a * v[i] * v[j];
                }
            }
            force += load * a;
        }
        k[0][3] = force;
        for i in 0..3 {
            let pivot = (i..3)
                .max_by(|&a, &b| k[a][i].abs().total_cmp(&k[b][i].abs()))
                .unwrap();
            k.swap(i, pivot);
            let d = k[i][i];
            if d.abs() < 1e-14 {
                return Err("Rigid plate singular".into());
            }
            for j in i..4 {
                k[i][j] /= d;
            }
            for r in 0..3 {
                if r != i {
                    let m = k[r][i];
                    for j in i..4 {
                        k[r][j] -= m * k[i][j];
                    }
                }
            }
        }
        engine
            .cells
            .iter()
            .zip(c)
            .map(|(cell, c)| {
                let p = cell.center();
                let q = c * (k[0][3] + (p[0] - 10.) * k[1][3] + (p[1] - 10.) * k[2][3]);
                if q < 0. {
                    Err("Lift-off: unilateral adapter required".into())
                } else {
                    Ok(q)
                }
            })
            .collect()
    };
    let start = std::time::Instant::now();
    let result = engine.iterate(vec![1000.; engine.cells.len()], |c| pressure(c, 10.))?;
    let elapsed = start.elapsed().as_secs_f64();
    let frozen = pressure(&result.c1, 20.)?;
    let equilibrium = result
        .pressure
        .iter()
        .zip(&engine.cells)
        .map(|(p, c)| p * c.area())
        .sum::<f64>();
    let mut fine_options = engine.options.clone();
    fine_options.dz = 0.125;
    let fine = Engine {
        cells: engine.cells.clone(),
        holes: engine.holes.clone(),
        options: fine_options,
    }
    .settlements(&result.pressure)?;
    let refinement = result
        .settlement
        .metres
        .iter()
        .zip(&fine.metres)
        .map(|(a, b)| (a - b).abs() / b.max(1e-12))
        .fold(0., f64::max);
    let report = json!({"prototype":true,"source":input,"units":"m, tf, tf/m2, tf/m3","boreholes":engine.holes.len(),"cells":engine.cells,"options":engine.options,"host":"rigid plate 3DOF; replace callback with shell FEM","reference_pressure":10.,"force_expected_tf":4000.,"force_recovered_tf":equilibrium,"elapsed_seconds":elapsed,"depth_step_relative_difference_max":refinement,"frozen_c1_case_20tf_m2_pressure":frozen,"result":result});
    std::fs::write(
        output,
        serde_json::to_vec_pretty(&report).map_err(|e| e.to_string())?,
    )
    .map_err(|e| e.to_string())?;
    println!("{} cells, converged={}, iterations={}, time={:.3}s, force={:.6}tf, depth refinement={:.4}% -> {}",n*n,result.converged,result.iterations,elapsed,equilibrium,100.*refinement,output);
    Ok(())
}
