use lir_asp_parser::*;
use std::io::Cursor;
fn u32_at(b: &mut [u8], o: usize, v: u32) {
    b[o..o + 4].copy_from_slice(&v.to_le_bytes());
}
fn f32_at(b: &mut [u8], o: usize, v: f32) {
    u32_at(b, o, v.to_bits());
}
fn fixture(header: &str, v6: bool) -> (Vec<u8>, Vec<u8>) {
    let start = 1 + header.len() + 52;
    // Fifth node and second element intentionally invalid, unreferenced by ASP.
    let mut l = vec![0; start + 5 * 28 + 2 * 38];
    l[0] = header.len() as u8;
    l[1..1 + header.len()].copy_from_slice(header.as_bytes());
    let base = 1 + header.len();
    u32_at(&mut l, base + 12, 5);
    u32_at(&mut l, base + 20, 2);
    let xyz = [[0., 0., 0.], [1., 0., 0.], [0., 1., 0.], [1., 1., 0.]];
    for (i, p) in xyz.iter().enumerate() {
        for (k, &v) in p.iter().enumerate() {
            l[start + i * 28 + k * 8..start + i * 28 + k * 8 + 8]
                .copy_from_slice(&f64::to_le_bytes(v));
        }
        u32_at(&mut l, start + i * 28 + 24, 1);
    }
    let eo = start + 5 * 28;
    for i in 0..4 {
        u32_at(&mut l, eo + i * 4, i as u32);
    }
    l[eo + 32..eo + 34].copy_from_slice(&71u16.to_le_bytes());
    u32_at(&mut l, eo + 34, 1);
    let signature = if v6 {
        "ARM-SAPFIR_6 binary v.6.0 2022"
    } else {
        "ARM-SAPFIR_5 binary v.5.0 2015"
    };
    let def = if v6 { 120 } else { 88 };
    let mut a = vec![0; 272 + 156 + def + 9];
    a[..signature.len()].copy_from_slice(signature.as_bytes());
    for (o, v) in [
        (96, 1),
        (100, 156),
        (240, 1),
        (244, def as u32),
        (256, 160),
        (424, 324),
    ] {
        u32_at(&mut a, o, v);
    }
    for (i, &j) in [0, 1, 3, 2].iter().enumerate() {
        for (k, &v) in xyz[j].iter().enumerate() {
            f32_at(&mut a, 280 + i * 12 + k * 4, v as f32);
        }
    }
    for (i, &v) in [0., -200., 3., 4.].iter().enumerate() {
        f32_at(&mut a, 404 + i * 4, v);
    }
    (l, a)
}
const BASIC: &str = "LIRA-SAPR 2013 software ver.13.0.0.a\0\0";
#[test]
fn sparse_identity_perimeter_units_and_as() {
    let (l, a) = fixture(BASIC, false);
    let asp = parse_asp(&a).unwrap();
    assert_eq!(asp.trailing_bytes, 9);
    let subset = read_lir_subset(&mut Cursor::new(&l), &asp).unwrap();
    assert_eq!(subset.retained_nodes, 4);
    assert_eq!(subset.plates.len(), 1);
    assert_eq!(subset.elements, 2);
    let result = import_pair(&mut Cursor::new(&l), &a, Unit::Meters).unwrap();
    let p = &result.plates[0];
    assert_eq!(p.source.raw_values, [0., -200., 3., 4.]);
    assert_eq!(p.source.node_ids, [1, 2, 3, 4]);
    assert_eq!(
        p.xy_mm,
        [[0., 0.], [1000., 0.], [1000., 1000.], [0., 1000.]]
    );
}
#[test]
fn extended_headers_2024_2026_and_both_asp_versions() {
    for h in [BASIC,"LIRA-SAPR 2013 software ver.13.0.0.a\0FileAnsiCP=1251 / 866\0Preview=0000000005610E0B\0\0","LIRA-SAPR 2013 software ver.13.0.0.a\0FileAnsiCP=1251 / 866\0Preview=0000000004770717\0MetaData1=0000000004E9AAD0"]{
 for v6 in [false,true]{let(l,a)=fixture(h,v6);assert_eq!(import_pair(&mut Cursor::new(l),&a,Unit::Millimeters).unwrap().plates[0].xy_mm[1],[1.,0.]);}}
}
#[test]
fn malformed_inputs_are_errors_not_panics() {
    let (l, a) = fixture(BASIC, false);
    for n in 0..a.len() - 9 {
        assert!(parse_asp(&a[..n]).is_err());
    }
    assert!(import_pair(&mut Cursor::new(&l[..l.len() - 1]), &a, Unit::Meters).is_err());
    let mut bad = a.clone();
    f32_at(&mut bad, 280, 100.);
    assert!(import_pair(&mut Cursor::new(&l), &bad, Unit::Meters).is_err());
    f32_at(&mut bad, 404, f32::NAN);
    assert!(parse_asp(&bad).is_err());
    let mut bad = l.clone();
    u32_at(&mut bad, 1 + BASIC.len() + 12, u32::MAX);
    assert!(import_pair(&mut Cursor::new(&bad), &a, Unit::Meters).is_err());
}
#[test]
fn bars_summary_rows_and_tail() {
    let (l, a) = fixture(BASIC, true);
    let start = a.len() - 9;
    let mut a = a[..start].to_vec();
    a.resize(start + 160 + 88 + 7, 0);
    u32_at(&mut a, 252, 1);
    u32_at(&mut a, start + 156, 2);
    for k in 0..11 {
        for o in [start + 112, start + 160, start + 204] {
            f32_at(&mut a, o + k * 4, k as f32);
        }
    }
    let p = parse_asp(&a).unwrap();
    assert_eq!((p.bars, p.rows, p.trailing_bytes), (1, 2, 7));
    assert!(import_pair(&mut Cursor::new(l), &a, Unit::Meters).is_ok());
    f32_at(&mut a, start + 112, 999.);
    assert!(parse_asp(&a).is_err());
}
#[test]
fn duplicate_header_field_unknown_header_and_empty_results() {
    for h in [
        format!("{}\0Unknown=00", BASIC.trim_end_matches('\0')),
        format!(
            "{}\0Preview=0000000000000000\0Preview=0000000000000000",
            BASIC.trim_end_matches('\0')
        ),
    ] {
        let (l, a) = fixture(&h, false);
        assert!(import_pair(&mut Cursor::new(l), &a, Unit::Meters).is_err());
    }
    let (l, mut a) = fixture(BASIC, false);
    for i in 0..4 {
        f32_at(&mut a, 404 + i * 4, -1.);
    }
    assert!(import_pair(&mut Cursor::new(l), &a, Unit::Meters).is_err());
}
