//! Explicit opt-in benchmark; does not alter the production inference path.
use super::*;
use std::{hint::black_box, ops::Range};

#[test]
#[ignore = "local real-project inference benchmark; run release explicitly"]
fn large_batches_on_same_project_rows() {
    let path = std::env::var("MINI_FEM_BATCH_INPUT")
        .unwrap_or_else(|_| "target/rc_cross_input.json".into());
    let bytes = std::fs::read(&path).expect("exported reinforcement Input JSON required");
    let input: Input = serde_json::from_slice(&bytes).unwrap();
    let combined = combinations(&input).unwrap();
    let mut rows = Vec::new();
    let mut groups = Vec::<Range<usize>>::new();
    for group in &input.groups {
        let start = rows.len();
        for angle in adaptive_angles(group, &combined) {
            for c in &combined {
                for (forces, rb) in [(&c.design, 1.), (&c.design_long, 0.9)] {
                    let load = forces[group.displays[0]].angle(angle);
                    let mut section = group.settings.section(
                        group.h,
                        group.settings.asbx.max(group.settings.asby),
                        group.settings.astx.max(group.settings.asty),
                        false,
                    );
                    section.rb *= rb;
                    if in_domain(section) && load.iter().any(|v| v.abs() >= 1e-10) {
                        rows.push(raw(load, section));
                    }
                }
            }
        }
        if rows.len() > start {
            groups.push(start..rows.len());
        }
    }
    assert!(!rows.is_empty());
    std::fs::create_dir_all("reports").unwrap();
    let model = ml::Network::load(false).unwrap();
    black_box(model.strength_batch(&rows[..rows.len().min(4096)]));
    let modes = [0_usize, 256, 1024, 4096, 16384];
    let mut reference: Option<Vec<[f64; 2]>> = None;
    let mut runs = Vec::new();
    for repeat in 0..3 {
        let order = if repeat % 2 == 0 {
            modes.to_vec()
        } else {
            modes.iter().copied().rev().collect()
        };
        for batch in order {
            let start = web_time::Instant::now();
            let mut output = Vec::with_capacity(rows.len());
            if batch == 0 {
                for range in &groups {
                    output.extend(model.strength_batch(black_box(&rows[range.clone()])));
                }
            } else {
                for chunk in rows.chunks(batch) {
                    output.extend(model.strength_batch(black_box(chunk)));
                }
            }
            black_box(&output);
            let seconds = start.elapsed().as_secs_f64();
            assert_eq!(output.len(), rows.len());
            let mut max_rel = 0_f64;
            if let Some(reference) = &reference {
                for (a, b) in output.iter().zip(reference) {
                    for k in 0..2 {
                        assert!(a[k].is_finite());
                        max_rel = max_rel.max((a[k] - b[k]).abs() / b[k].abs().max(1.));
                    }
                }
                assert!(max_rel < 2e-4, "prediction drift {max_rel}");
            } else {
                reference = Some(output);
            }
            let record = serde_json::json!({"repeat":repeat,"batch":batch,"seconds":seconds,"rows":rows.len(),"physical_elements":input.groups.len(),"nonempty_groups":groups.len(),"mean_rows_per_group":rows.len() as f64/groups.len() as f64,"max_relative_difference":max_rel});
            println!("LARGE_BATCH {record}");
            runs.push(record);
            std::fs::write("reports/rc_large_batches_native_2026-11-02.json",serde_json::to_vec_pretty(&serde_json::json!({"input":path,"input_bytes":bytes.len(),"input_crc32":crc32(&bytes),"scope":"strength network only, same real-project rows, one thread; no Capra assembly/NDM/cracks","runs":runs})).unwrap()).unwrap();
        }
    }
}
fn crc32(bytes: &[u8]) -> String {
    // Stable input fingerprint without another benchmark dependency.
    let mut crc = !0_u32;
    for &byte in bytes {
        crc ^= byte as u32;
        for _ in 0..8 {
            crc = (crc >> 1) ^ (0xedb88320_u32 & (0_u32.wrapping_sub(crc & 1)));
        }
    }
    format!("{:08x}", !crc)
}
