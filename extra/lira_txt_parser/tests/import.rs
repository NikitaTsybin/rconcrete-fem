use lira_txt_parser::{parse, read};
use std::path::Path;

#[test]
fn sample_file() {
    let g = read(Path::new(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/tests/sample.txt"
    )))
    .unwrap();
    assert_eq!(g.nodes.len(), 4);
    assert_eq!(g.elements.len(), 4);
    assert_eq!(g.elements[2].nodes, [1, 2, 3, 4]);
    assert_eq!(g.documents, [0, 1, 4]);
}

#[test]
fn file_encodings() {
    let source = "(0/Тест/)(4/0 0 0/)(1/56 1 1/)";
    let dir = std::env::temp_dir().join(format!("lira_parser_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let mut versions = vec![
        source.as_bytes().to_vec(),
        encoding_rs::WINDOWS_1251.encode(source).0.into_owned(),
    ];
    versions.push(
        [
            vec![0xff, 0xfe],
            source.encode_utf16().flat_map(u16::to_le_bytes).collect(),
        ]
        .concat(),
    );
    versions.push(
        [
            vec![0xfe, 0xff],
            source.encode_utf16().flat_map(u16::to_be_bytes).collect(),
        ]
        .concat(),
    );
    for (i, bytes) in versions.iter().enumerate() {
        let path = dir.join(format!("{i}.txt"));
        std::fs::write(&path, bytes).unwrap();
        assert_eq!(read(&path).unwrap().elements[0].fe_type, 56);
        std::fs::remove_file(path).unwrap();
    }
    std::fs::remove_dir(dir).unwrap();
    assert_eq!(parse(source).unwrap().nodes.len(), 1);
}

#[test]
#[ignore = "Set LIRA_TXT_SAMPLE to the original project TXT path"]
fn real_project_geometry() {
    let path = std::env::var("LIRA_TXT_SAMPLE").expect("Set LIRA_TXT_SAMPLE");
    let g = read(Path::new(&path)).unwrap();
    assert_eq!(g.nodes.len(), 114309);
    assert_eq!(g.elements.len(), 161823);
    let mut types = std::collections::BTreeMap::new();
    for e in g.elements {
        *types.entry(e.fe_type).or_insert(0) += 1;
    }
    assert_eq!(
        types.into_iter().collect::<Vec<_>>(),
        [(10, 202), (42, 25794), (44, 99516), (56, 36311)]
    );
}
