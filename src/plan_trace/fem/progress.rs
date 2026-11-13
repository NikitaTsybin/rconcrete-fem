use std::cell::RefCell;
pub(crate) type CancelToken = std::sync::Arc<std::sync::atomic::AtomicBool>;
thread_local! {static CANCEL:RefCell<Option<CancelToken>>=const {RefCell::new(None)};}
pub(crate) fn install_cancel(token: CancelToken) {
    CANCEL.with(|c| *c.borrow_mut() = Some(token));
}
pub(crate) fn current_cancel() -> Option<CancelToken> {
    CANCEL.with(|c| c.borrow().clone())
}
pub(crate) fn check_cancel() -> Result<(), String> {
    if CANCEL.with(|c| {
        c.borrow()
            .as_ref()
            .is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed))
    }) {
        Err("Расчёт остановлен пользователем".into())
    } else {
        Ok(())
    }
}
#[cfg(not(target_arch = "wasm32"))]
type Sink = std::sync::Arc<dyn Fn(String, f64) + Send + Sync>;
#[cfg(target_arch = "wasm32")]
type Sink = std::rc::Rc<dyn Fn(String, f64)>;
thread_local! {
    static SINK: RefCell<Option<Sink>> = RefCell::new(None);
    static START: RefCell<web_time::Instant> = RefCell::new(web_time::Instant::now());
    static PREFIX: RefCell<String> = const {RefCell::new(String::new())};
}
#[cfg(not(target_arch = "wasm32"))]
pub(crate) fn install(sink: impl Fn(String, f64) + Send + Sync + 'static) {
    START.with(|s| *s.borrow_mut() = web_time::Instant::now());
    SINK.with(|s| *s.borrow_mut() = Some(std::sync::Arc::new(sink)));
    PREFIX.with(|p| p.borrow_mut().clear());
}
#[cfg(target_arch = "wasm32")]
pub(crate) fn install(sink: impl Fn(String, f64) + 'static) {
    START.with(|s| *s.borrow_mut() = web_time::Instant::now());
    SINK.with(|s| *s.borrow_mut() = Some(std::rc::Rc::new(sink)));
    PREFIX.with(|p| p.borrow_mut().clear());
}
pub(super) fn case_prefix(index: usize) {
    PREFIX.with(|p| *p.borrow_mut() = format!("Загружение #{index} | "));
}
pub(super) fn clear_case_prefix() {
    PREFIX.with(|p| p.borrow_mut().clear());
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn capture() -> (Option<Sink>, web_time::Instant, Option<CancelToken>) {
    (
        SINK.with(|s| s.borrow().clone()),
        START.with(|s| *s.borrow()),
        current_cancel(),
    )
}
#[cfg(not(target_arch = "wasm32"))]
pub(super) fn attach(context: (Option<Sink>, web_time::Instant, Option<CancelToken>)) {
    SINK.with(|s| *s.borrow_mut() = context.0);
    START.with(|s| *s.borrow_mut() = context.1);
    CANCEL.with(|c| *c.borrow_mut() = context.2);
}
pub(crate) fn stage(label: &str) {
    let label = PREFIX.with(|p| format!("{}{label}", p.borrow()));
    #[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
    if let Some((_, start)) = &*CHANNEL.lock().unwrap() {
        HISTORY
            .lock()
            .unwrap()
            .push((label.clone(), (js_sys::Date::now() - *start) / 1000.));
    }
    let elapsed = START.with(|s| s.borrow().elapsed().as_secs_f64());
    SINK.with(|s| {
        if let Some(f) = &*s.borrow() {
            f(label.clone(), elapsed);
        } else {
            #[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
            broadcast_stage(&label);
        }
    });
}
pub(crate) fn clear() {
    CANCEL.with(|c| c.borrow_mut().take());
    SINK.with(|s| s.borrow_mut().take());
    PREFIX.with(|p| p.borrow_mut().clear());
}

#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
static CHANNEL: std::sync::Mutex<Option<(String, f64)>> = std::sync::Mutex::new(None);
#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
static HISTORY: std::sync::Mutex<Vec<(String, f64)>> = std::sync::Mutex::new(Vec::new());
#[cfg(target_arch = "wasm32")]
pub(crate) fn history() -> Vec<(String, f64)> {
    #[cfg(feature = "parallel-solver")]
    {
        let mut events = HISTORY.lock().unwrap().clone();
        events.sort_by(|a, b| a.1.total_cmp(&b.1));
        events
    }
    #[cfg(not(feature = "parallel-solver"))]
    {
        vec![]
    }
}
#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
pub(crate) fn configure(channel: String, start: f64, limit: usize) {
    super::parallel_cases::set_limit(limit);
    *CHANNEL.lock().unwrap() = Some((channel, start));
}
#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
fn broadcast_stage(label: &str) {
    use wasm_bindgen::prelude::*;
    #[wasm_bindgen(
        inline_js = "export function caseStage(channel,label,seconds){const c=new BroadcastChannel(channel);c.postMessage([label,seconds]);c.close();}"
    )]
    extern "C" {
        #[wasm_bindgen(js_name=caseStage)]
        fn post(channel: &str, label: &str, seconds: f64);
    }
    if let Some((channel, start)) = &*CHANNEL.lock().unwrap() {
        post(channel, label, (js_sys::Date::now() - *start) / 1000.);
    }
}

#[cfg(all(test, not(target_arch = "wasm32")))]
mod cancel_tests {
    use super::*;
    #[test]
    fn cancellation_is_per_job_and_propagates_to_case_threads() {
        clear();
        let token = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
        install_cancel(token.clone());
        assert!(check_cancel().is_ok());
        let context = capture();
        token.store(true, std::sync::atomic::Ordering::Relaxed);
        std::thread::spawn(move || {
            attach(context);
            assert!(check_cancel().is_err());
            clear();
            assert!(check_cancel().is_ok());
        })
        .join()
        .unwrap();
        assert!(check_cancel().is_err());
        clear();
        assert!(check_cancel().is_ok());
    }
}
