//! Python Remote Scripting export. Never delete geometry by generated object number.
use super::*;
pub(super) fn launcher() -> &'static [u8] {
    include_bytes!("plaxis_python_launch.cmd")
}
pub(super) fn encode(
    all: &[Contour],
    heights: &BTreeMap<u64, Vec<f64>>,
    materials: Option<&BTreeMap<u64, stiffness::Entry>>,
    loads: Option<&LoadExport>,
) -> Result<Export, String> {
    let original = super::encode(all, heights, materials, loads)?;
    let mut text = String::from(include_str!("plaxis_python_runtime.py"));
    text.push_str("\n# Generated model commands\n");
    let mut skipping = false;
    for line in original.text.lines() {
        if line.starts_with("## FEM ")
            && !line.starts_with("## FEM stiffness:")
            && !line.starts_with("## FEM load ")
        {
            skipping = false;
            if let Some(id) = line
                .strip_prefix("## FEM Slab ID ")
                .and_then(|s| s.parse::<u64>().ok())
            {
                let slab = all
                    .iter()
                    .find(|c| c.id == id)
                    .ok_or("Missing source slab")?;
                let outer = slab
                    .points
                    .iter()
                    .map(|p| [p[0] / 1000., p[1] / 1000., heights[&id][0] / 1000.])
                    .collect::<Vec<_>>();
                let holes = all
                    .iter()
                    .filter(|h| h.active_hole() && h.parent == Some(id))
                    .map(|h| {
                        h.points
                            .iter()
                            .map(|p| [p[0] / 1000., p[1] / 1000., heights[&id][0] / 1000.])
                            .collect::<Vec<_>>()
                    })
                    .collect::<Vec<_>>();
                if let Some(lib) = materials {
                    let entry = &lib[&id];
                    let gamma = loads.map_or(0., |l| {
                        entry
                            .material
                            .weight
                            .in_units(loads::ForceUnits::KiloNewton, l.gravity)
                            * l.weight_factor
                    });
                    let mut commands = String::new();
                    material_commands(&mut commands, entry, id, gamma)?;
                    for cmd in commands.lines().filter(|l| !l.starts_with('#')) {
                        writeln!(text, "_exec({})", serde_json::to_string(cmd).unwrap()).unwrap();
                    }
                }
                writeln!(
                    text,
                    "_slab({}, {}, {}, {})",
                    id,
                    serde_json::to_string(&outer).unwrap(),
                    serde_json::to_string(&holes).unwrap(),
                    if materials.is_some() { "True" } else { "False" }
                )
                .unwrap();
                skipping = true;
            }
        }
        if line.starts_with("## Loads:") {
            skipping = false;
        }
        if !skipping && !line.starts_with('#') && !line.trim().is_empty() {
            writeln!(text, "_exec({})", serde_json::to_string(line).unwrap()).unwrap();
        }
    }
    text.push_str("\nprint('Rconcrete FEM import finished. Verify geometry, beam axes and phase activation before calculation.')\n");
    text.push_str("\n_pause()\n");
    Ok(Export { text, ..original })
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn python_export_keeps_outer_and_holes_as_input_to_intersection() {
        let contour = |id, kind, parent, points| {
            serde_json::from_value::<Contour>(serde_json::json!({
                "id":id,"kind":kind,"parent":parent,"points":points,"support_open":false,
                "load":null,"refinement":null,"curves":[]
            }))
            .unwrap()
        };
        let all = vec![
            contour(
                1,
                "Slab",
                None::<u64>,
                vec![[0., 0.], [4000., 0.], [4000., 4000.], [0., 4000.]],
            ),
            contour(
                2,
                "Hole",
                Some(1),
                vec![
                    [1000., 1000.],
                    [2000., 1000.],
                    [2000., 2000.],
                    [1000., 2000.],
                ],
            ),
        ];
        let heights = BTreeMap::from([(1, vec![3000.]), (2, vec![3000.])]);
        let out = encode(&all, &heights, None, None).unwrap();
        assert!(out.text.contains("_slab(1, [[0.0,0.0,3.0]"));
        assert!(out.text.contains("[[[1.0,1.0,3.0]"));
        assert!(!out.text.contains("_exec(\"_surface "));
        assert!(out.text.contains("result=g_i.intersect(*inputs)"));
        assert!(out.text.contains("for obj in interior: g_i.delete(obj)"));
        assert!(!out.text.contains("delete(Polygons[-1])"));
    }
}
