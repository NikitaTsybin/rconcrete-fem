use super::*;

#[test]
fn marquee_selects_crossing_edges_filters_and_preserves_geometry() {
    let ctx = egui::Context::default();
    let mut a = sample();
    a.select_only(None);
    a.selection_filter.fill(false);
    a.selection_filter[2] = true;
    let rect = frame(&ctx, &mut a, vec![]);
    frame(&ctx, &mut a, vec![]);
    let before = a.contours.clone();
    // Narrow frame crosses the open support; neither endpoint is in the frame.
    let start = a.camera.screen([2300., 2800.], rect);
    let end = a.camera.screen([1700., 2200.], rect);
    frame(
        &ctx,
        &mut a,
        vec![egui::Event::PointerMoved(start), button(start, true)],
    );
    frame(&ctx, &mut a, vec![egui::Event::PointerMoved(end)]);
    assert!(a.selected.is_none());
    frame(&ctx, &mut a, vec![button(end, false)]);
    assert_eq!(a.selection_ids, [3].into_iter().collect());
    assert_eq!(a.contours, before);
    let empty = a.camera.screen([7000., 5000.], rect);
    frame(
        &ctx,
        &mut a,
        vec![egui::Event::PointerMoved(empty), button(empty, true)],
    );
    frame(&ctx, &mut a, vec![button(empty, false)]);
    assert_eq!(a.selected, Some(3));
    a.selection_filter[2] = false;
    frame(&ctx, &mut a, vec![]);
    assert!(a.selected.is_none() && a.selection_ids.is_empty());
}

#[test]
fn selection_modifiers_delete_undo_and_escape_are_consistent() {
    let mut a = sample();
    a.select_ids([3], egui::Modifiers::NONE);
    a.select_ids([4], egui::Modifiers::SHIFT);
    assert!(a.is_selected(3) && a.is_selected(4));
    a.select_ids([3], egui::Modifiers::CTRL);
    assert!(!a.is_selected(3) && a.is_selected(4));
    a.select_ids([3], egui::Modifiers::SHIFT);
    let before = a.contours.clone();
    let history = a.history.len();
    a.remove();
    assert!(a.contours.iter().all(|c| !matches!(c.id, 3 | 4)));
    assert_eq!(a.history.len(), history + 1);
    a.undo();
    assert_eq!(a.contours, before);
    a.select_ids([3, 4], egui::Modifiers::NONE);
    a.contour_locks[2] = true;
    a.remove();
    assert_eq!(a.contours, before);
    a.selection_start = Some(Pos2::ZERO);
    a.cancel();
    assert!(a.selection_start.is_none() && a.selected.is_none() && a.selection_ids.is_empty());
}
fn sample() -> App {
    let mut a = App::default();
    a.begin(Kind::Slab);
    a.choose_tool(Tool::Rectangle);
    a.sketch_click([0., 0.], false);
    a.sketch_click([6000., 4000.], false);
    a.begin(Kind::Hole);
    a.choose_tool(Tool::Rectangle);
    a.sketch_click([2000., 1000.], false);
    a.sketch_click([2500., 1500.], false);
    a.begin(Kind::Support);
    a.sketch_click([1000., 2500.], false);
    a.sketch_click([3000., 2500.], false);
    a.finish();
    a.begin(Kind::Support);
    a.choose_tool(Tool::Rectangle);
    a.sketch_click([3500., 500.], false);
    a.sketch_click([4500., 1500.], false);
    a.fit = false;
    a.camera = Camera {
        center: [3000., 2000.],
        zoom: 0.1,
        angle: 0.,
        plane: None,
    };
    a
}
fn frame(ctx: &egui::Context, a: &mut App, events: Vec<egui::Event>) -> Rect {
    let mut rect = Rect::NOTHING;
    let _ = ctx.run(
        egui::RawInput {
            screen_rect: Some(Rect::from_min_size(Pos2::ZERO, egui::vec2(1000., 800.))),
            events,
            ..Default::default()
        },
        |ctx| {
            egui::CentralPanel::default().show(ctx, |ui| {
                rect = ui.available_rect_before_wrap();
                a.canvas(ui);
            });
        },
    );
    rect
}
fn button(p: Pos2, pressed: bool) -> egui::Event {
    egui::Event::PointerButton {
        pos: p,
        button: egui::PointerButton::Primary,
        pressed,
        modifiers: egui::Modifiers::NONE,
    }
}

#[test]
fn whole_slab_hole_open_and_closed_support_drag_commits_once_and_undoes() {
    for (id, anchor) in [
        (1, [5000., 3000.]),
        (1, [4000., 0.]),
        (2, [2250., 1250.]),
        (3, [2000., 2500.]),
        (4, [4000., 1000.]),
    ] {
        let ctx = egui::Context::default();
        let mut a = sample();
        a.selected = Some(id);
        let before = a.contours.clone();
        let history = a.history.len();
        let rect = frame(&ctx, &mut a, vec![]);
        frame(&ctx, &mut a, vec![]);
        let start = a.camera.screen(anchor, rect);
        let middle = start + egui::vec2(20., -10.);
        let end = start + egui::vec2(40., -20.);
        frame(
            &ctx,
            &mut a,
            vec![egui::Event::PointerMoved(start), button(start, true)],
        );
        assert_eq!(a.drag.as_ref().unwrap().vertex, None);
        frame(&ctx, &mut a, vec![egui::Event::PointerMoved(middle)]);
        frame(&ctx, &mut a, vec![egui::Event::PointerMoved(end)]);
        assert_eq!(a.history.len(), history);
        frame(&ctx, &mut a, vec![button(end, false)]);
        let mut expected = before.clone();
        expected
            .iter_mut()
            .find(|c| c.id == id)
            .unwrap()
            .translate([400., 200.])
            .unwrap();
        assert_eq!(a.contours, expected, "contour {id}");
        assert_eq!(a.history.len(), history + 1);
        assert!(a.drag.is_none());
        a.undo();
        assert_eq!(a.contours, before);
    }
}
#[test]
fn whole_drag_respects_locks_and_cancel_restores_original_without_history() {
    let ctx = egui::Context::default();
    let mut a = sample();
    a.selected = Some(3);
    let before = a.contours.clone();
    let history = a.history.len();
    let rect = frame(&ctx, &mut a, vec![]);
    frame(&ctx, &mut a, vec![]);
    let start = a.camera.screen([2000., 2500.], rect);
    let end = start + egui::vec2(30., -20.);
    a.input_locked = true;
    frame(
        &ctx,
        &mut a,
        vec![egui::Event::PointerMoved(start), button(start, true)],
    );
    frame(&ctx, &mut a, vec![egui::Event::PointerMoved(end)]);
    frame(&ctx, &mut a, vec![button(end, false)]);
    assert_eq!(a.contours, before);
    assert!(a.drag.is_none());
    a.input_locked = false;
    a.select_only(Some(3));
    frame(
        &ctx,
        &mut a,
        vec![egui::Event::PointerMoved(start), button(start, true)],
    );
    frame(&ctx, &mut a, vec![egui::Event::PointerMoved(end)]);
    assert_ne!(a.contours, before);
    a.cancel();
    frame(&ctx, &mut a, vec![button(end, false)]);
    assert_eq!(a.contours, before);
    assert_eq!(a.history.len(), history);
}

#[test]
fn clicking_a_hole_while_slab_selected_selects_hole_without_moving_slab() {
    let ctx = egui::Context::default();
    let mut a = sample();
    a.selected = Some(1);
    let before = a.contours.clone();
    let history = a.history.len();
    let rect = frame(&ctx, &mut a, vec![]);
    frame(&ctx, &mut a, vec![]);
    let p = a.camera.screen([2250., 1250.], rect);
    frame(
        &ctx,
        &mut a,
        vec![egui::Event::PointerMoved(p), button(p, true)],
    );
    frame(&ctx, &mut a, vec![button(p, false)]);
    assert_eq!(a.selected, Some(2));
    assert_eq!(a.contours, before);
    assert_eq!(a.history.len(), history);
}

#[test]
fn selected_vertex_on_another_contour_keeps_drag_priority_and_one_undo() {
    let ctx = egui::Context::default();
    let mut app = sample();
    let mut other = app.contours[2].clone();
    other.id = 99;
    other.points = vec![[6000., 4000.], [6500., 4500.]];
    other.kind = Kind::Auxiliary;
    other.parent = None;
    app.contours.push(other);
    app.select_only(Some(1));
    app.snap_settings.nearest = false;
    app.snap_settings.geometry_nodes = false;
    let rect = frame(&ctx, &mut app, vec![]);
    frame(&ctx, &mut app, vec![]);
    let before = app.contours.clone();
    let history = app.history.len();
    let start = app.camera.screen([6000., 4000.], rect);
    let end = app.camera.screen([5800., 3800.], rect);
    frame(
        &ctx,
        &mut app,
        vec![egui::Event::PointerMoved(start), button(start, true)],
    );
    assert_eq!(
        app.drag.as_ref().map(|d| (d.id, d.vertex)),
        Some((1, Some(2)))
    );
    frame(&ctx, &mut app, vec![egui::Event::PointerMoved(end)]);
    frame(&ctx, &mut app, vec![button(end, false)]);
    assert_eq!(app.history.len(), history + 1);
    assert_ne!(app.contours[0], before[0]);
    assert_eq!(app.contours.last(), before.last());
    app.undo();
    assert_eq!(app.contours, before);
}
