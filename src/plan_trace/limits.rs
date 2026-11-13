//! Shared calculation-model limits (display triangles are not separate FE).
pub const MAX_NODES: usize = 1_000_000;
pub const MAX_ELEMENTS: usize = 1_000_000;
pub const MAX_DISPLAY_TRIANGLES: usize = 2 * MAX_ELEMENTS;
pub fn check(nodes: usize, elements: usize) -> Result<(), String> {
    if nodes > MAX_NODES {
        return Err(format!(
            "Превышен лимит узлов: {nodes} (максимум {MAX_NODES})"
        ));
    }
    if elements > MAX_ELEMENTS {
        return Err(format!(
            "Превышен лимит КЭ: {elements} (максимум {MAX_ELEMENTS})"
        ));
    }
    Ok(())
}
