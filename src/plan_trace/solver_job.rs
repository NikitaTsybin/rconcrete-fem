use super::*;
use crate::plan_trace::{fem, loads, meshing};
#[cfg(target_arch = "wasm32")]
use wasm_bindgen::{prelude::*, JsCast};
#[derive(serde::Serialize, serde::Deserialize)]
struct Request {
    mesh: meshing::Mesh,
    first: fem_ui::Input,
    second: Option<fem_ui::Input>,
    cases: Vec<loads::LoadCase>,
}
#[derive(serde::Serialize, serde::Deserialize)]
struct Reply {
    first: Result<fem::Results, String>,
    second: Option<Result<fem::Results, String>>,
    #[serde(default)]
    events: Vec<(String, f64)>,
}
#[cfg(any(test, target_arch = "wasm32"))]
fn pack_reply(reply: &Reply) -> Result<Vec<u8>, String> {
    let encoder = flate2::write::ZlibEncoder::new(Vec::new(), flate2::Compression::fast());
    let mut writer = std::io::BufWriter::with_capacity(128 * 1024, encoder);
    serde_json::to_writer(&mut writer, reply).map_err(|e| e.to_string())?;
    writer
        .into_inner()
        .map_err(|e| e.to_string())?
        .finish()
        .map_err(|e| e.to_string())
}
#[cfg(any(test, target_arch = "wasm32"))]
fn unpack_reply(bytes: &[u8]) -> Result<Reply, String> {
    let decoder = flate2::read::ZlibDecoder::new(bytes);
    serde_json::from_reader(std::io::BufReader::with_capacity(128 * 1024, decoder))
        .map_err(|e| e.to_string())
}
#[cfg(target_arch = "wasm32")]
fn worker_reply(text: &str, callback: &js_sys::Function) -> Result<Reply, JsValue> {
    let request: Request =
        serde_json::from_str(text).map_err(|e| JsValue::from_str(&e.to_string()))?;
    let callback = callback.clone();
    fem::progress::install(move |label, seconds| {
        if let Ok(text) = serde_json::to_string(&(label, seconds)) {
            let _ = callback.call1(&JsValue::NULL, &JsValue::from_str(&text));
        }
    });
    fem::progress::stage("Жёсткости 1: подготовка модели");
    let first = fem_ui::solve_snapshot(&request.mesh, &request.first, &request.cases);
    let second = request.second.map(|key| {
        fem::progress::stage("Жёсткости 2: подготовка модели");
        fem_ui::solve_snapshot(&request.mesh, &key, &request.cases)
    });
    fem::progress::stage("Расчёт завершён");
    fem::progress::clear();
    Ok(Reply {
        first,
        second,
        events: fem::progress::history(),
    })
}
#[cfg(target_arch = "wasm32")]
pub(crate) fn worker_run(text: &str, callback: &js_sys::Function) -> Result<String, JsValue> {
    serde_json::to_string(&worker_reply(text, callback)?)
        .map_err(|e| JsValue::from_str(&e.to_string()))
}
#[cfg(target_arch = "wasm32")]
pub(crate) fn worker_run_bytes(
    text: &str,
    callback: &js_sys::Function,
) -> Result<Vec<u8>, JsValue> {
    let reply = worker_reply(text, callback)?;
    let seconds = reply.events.last().map_or(0., |p| p.1);
    let _ = callback.call1(
        &JsValue::NULL,
        &JsValue::from_str(&serde_json::to_string(&("Упаковка результатов", seconds)).unwrap()),
    );
    pack_reply(&reply).map_err(|e| JsValue::from_str(&e))
}
#[cfg(target_arch = "wasm32")]
#[wasm_bindgen(inline_js = r#"export function startSolver(text,callback){
 const channel=typeof BroadcastChannel==='function'?new BroadcastChannel('rconcrete-solver-'+Date.now()+'-'+Math.random()):null;
 if(channel)channel.onmessage=({data})=>callback('progress',JSON.stringify(data));
 const w=new Worker(new URL('../../../solver_worker.js',import.meta.url),{type:'module'});
 w.onerror=e=>{e.preventDefault();callback('error',e.message||'Solver Worker failed');};
 w.onmessageerror=()=>callback('error','Solver Worker message error');
 w.onmessage=({data})=>callback(data.kind,data.payload);w.postMessage({text,channel:channel?.name,binary:true});return {worker:w,channel};
}
export function stopSolver(job){const w=job.worker;job.channel?.close();w.onmessage=null;w.onerror=null;w.onmessageerror=null;w.terminate();}"#)]
extern "C" {
    #[wasm_bindgen(catch,js_name=startSolver)]
    fn start_solver(text: &str, callback: &js_sys::Function) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(js_name=stopSolver)]
    fn stop_solver(worker: &JsValue);
}
#[cfg(target_arch = "wasm32")]
pub(super) struct Handle {
    worker: JsValue,
    _callback: Closure<dyn FnMut(String, JsValue)>,
}
#[cfg(target_arch = "wasm32")]
impl Handle {
    pub(super) fn start(
        mesh: meshing::Mesh,
        first: fem_ui::Input,
        second: Option<fem_ui::Input>,
        cases: Vec<loads::LoadCase>,
        progress: std::sync::Arc<std::sync::Mutex<Vec<(String, f64)>>>,
        tx: mpsc::Sender<Result<fem::Results, String>>,
        mut alternative: Option<mpsc::Sender<Result<fem::Results, String>>>,
        ctx: egui::Context,
    ) -> Result<Self, String> {
        let text = serde_json::to_string(&Request {
            mesh,
            first,
            second,
            cases,
        })
        .map_err(|e| e.to_string())?;
        let callback =
            Closure::<dyn FnMut(String, JsValue)>::new(move |kind: String, payload: JsValue| {
                match kind.as_str() {
                    "progress" => {
                        if let Ok(event) = serde_json::from_str::<(String, f64)>(
                            &payload.as_string().unwrap_or_default(),
                        ) {
                            progress.lock().unwrap().push(event);
                            ctx.request_repaint_after(Duration::from_millis(100));
                        }
                        return;
                    }
                    "result" | "result_bytes" => match if kind == "result_bytes" {
                        unpack_reply(&js_sys::Uint8Array::new(&payload).to_vec())
                    } else {
                        serde_json::from_str::<Reply>(&payload.as_string().unwrap_or_default())
                            .map_err(|e| e.to_string())
                    } {
                        Ok(reply) => {
                            if !reply.events.is_empty() {
                                *progress.lock().unwrap() = reply.events;
                            }
                            let _ = tx.send(reply.first);
                            if let (Some(tx), Some(r)) = (alternative.take(), reply.second) {
                                let _ = tx.send(r);
                            }
                        }
                        Err(e) => {
                            let _ = tx.send(Err(format!("Результат Worker: {e}")));
                            if let Some(tx) = alternative.take() {
                                let _ = tx.send(Err(e.to_string()));
                            }
                        }
                    },
                    _ => {
                        let _ = tx.send(Err(payload
                            .as_string()
                            .unwrap_or_else(|| "Ошибка Solver Worker".into())));
                        if let Some(tx) = alternative.take() {
                            let _ = tx.send(Err(payload
                                .as_string()
                                .unwrap_or_else(|| "Ошибка Solver Worker".into())));
                        }
                    }
                }
                ctx.request_repaint();
            });
        let worker = start_solver(&text, callback.as_ref().unchecked_ref())
            .map_err(|e| format!("Worker МКЭ: {e:?}"))?;
        Ok(Self {
            worker,
            _callback: callback,
        })
    }
}
#[cfg(target_arch = "wasm32")]
impl Drop for Handle {
    fn drop(&mut self) {
        stop_solver(&self.worker);
    }
}
impl App {
    pub(super) fn solver_cancellation_requested(&self) -> bool {
        self.solver_cancel
            .as_ref()
            .is_some_and(|c| c.load(std::sync::atomic::Ordering::Relaxed))
    }
    pub(super) fn stop_solver(&mut self) {
        if self.analysis_job.is_none() && self.alternative_job.is_none() {
            return;
        }
        self.solver_cancel
            .get_or_insert_with(|| std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false)))
            .store(true, std::sync::atomic::Ordering::Relaxed);
        #[cfg(target_arch = "wasm32")]
        {
            // Dropping Handle terminates the Worker and disconnects its callbacks.
            self.solver_worker = None;
            self.analysis_job = None;
            self.alternative_job = None;
        }
        self.analysis = None;
        self.analysis_input = None;
        self.alternative_cache = Default::default();
        self.combination_cache = None;
        self.reinforcement = Default::default();
        self.input_locked = false;
        self.restore_manual_locks();
        self.show_results = false;
        self.punching_view = None;
        self.result_generation = self.result_generation.wrapping_add(1);
        self.invalidate_scene_buffers();
        self.status = "Остановка расчёта…".into();
        self.poll_solver_cancellation();
    }
    /// A cancelled job is drained but can never publish partial or late results.
    pub(super) fn poll_solver_cancellation(&mut self) -> bool {
        if !self.solver_cancellation_requested() {
            return false;
        }
        fn drain<T>(job: &mut Option<mpsc::Receiver<T>>) {
            if job
                .as_ref()
                .is_some_and(|rx| !matches!(rx.try_recv(), Err(mpsc::TryRecvError::Empty)))
            {
                *job = None;
            }
        }
        drain(&mut self.analysis_job);
        drain(&mut self.alternative_job);
        if self.analysis_job.is_none() && self.alternative_job.is_none() {
            self.solver_cancel = None;
            self.status = "Расчёт остановлен. КЭ-сетка сохранена.".into();
            let mut events = self.solver_progress.lock().unwrap();
            let seconds = events.last().map_or(0., |e| e.1);
            events.push(("Расчёт остановлен пользователем".into(), seconds));
        }
        true
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn cancel_discards_partial_variant_and_late_reply_but_keeps_mesh() {
        let mut app = App::default();
        let request: serde_json::Value =
            serde_json::from_str(include_str!("../../fem_web/building_solver_fixture.json"))
                .unwrap();
        app.fe_mesh = Some(serde_json::from_value(request["mesh"].clone()).unwrap());
        app.analysis = Some(fem::Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![],
            free_dofs: 12,
        });
        let (tx, rx) = mpsc::channel();
        app.alternative_job = Some(rx);
        app.solver_cancel = Some(std::sync::Arc::new(std::sync::atomic::AtomicBool::new(
            false,
        )));
        app.stop_solver();
        assert!(app.analysis.is_none());
        assert!(app.alternative_job.is_some());
        tx.send(Ok(fem::Results {
            nonlinear: None,
            lengths: None,
            modal: None,
            buckling: None,
            cases: vec![],
            free_dofs: 18,
        }))
        .unwrap();
        assert!(app.poll_solver_cancellation());
        assert!(app.analysis.is_none());
        assert!(app.alternative_cache.analysis.is_none());
        assert!(app.alternative_job.is_none());
        assert!(app.fe_mesh.is_some());
        assert!(!app.input_locked);
    }
    #[test]
    fn solver_wire_roundtrip_preserves_solution_and_reports_actual_stages() {
        let mut app = App::default();
        app.mesh_size = 500.;
        app.mesh_built_size = 500.;
        let points = [[0., 0.], [3000., 0.], [3000., 2000.], [0., 2000.]];
        for kind in [Kind::Slab, Kind::Support, Kind::LoadArea] {
            app.begin(kind);
            for point in points {
                app.sketch_click(point, false);
            }
            if kind == Kind::Support {
                app.draft_line_closed = true;
            }
            app.finish();
            app.cancel();
        }
        let original = app
            .contours
            .iter()
            .find(|c| c.kind == Kind::LoadArea)
            .unwrap()
            .clone();
        for id in [2, 3] {
            app.load_cases.push(loads::LoadCase::new(id));
            let mut load = original.clone();
            load.id = 10 + id;
            load.load.as_mut().unwrap().case_id = id;
            load.load.as_mut().unwrap().magnitude.value *= id as f64;
            app.contours.push(load);
        }
        app.sync_stiffness_sources();
        let mesh = crate::plan_trace::meshing::generate(&app.contours, app.mesh_size).unwrap();
        let request = Request {
            mesh,
            first: app.analysis_key(),
            second: None,
            cases: app.load_cases.clone(),
        };
        let text = serde_json::to_string(&request).unwrap();
        let restored: Request = serde_json::from_str(&text).unwrap();
        let events = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
        let sink = events.clone();
        fem::progress::install(move |stage, _| sink.lock().unwrap().push(stage));
        let result =
            fem_ui::solve_snapshot(&restored.mesh, &restored.first, &restored.cases).unwrap();
        fem::progress::clear();
        let wire = serde_json::to_string(&Reply {
            first: Ok(result),
            second: None,
            events: vec![],
        })
        .unwrap();
        let decoded: Reply = serde_json::from_str(&wire).unwrap();
        let packed = pack_reply(&decoded).unwrap();
        let reply = unpack_reply(&packed).unwrap();
        let mut plain = Vec::new();
        std::io::Read::read_to_end(
            &mut flate2::read::ZlibDecoder::new(packed.as_slice()),
            &mut plain,
        )
        .unwrap();
        assert_eq!(plain, serde_json::to_vec(&decoded).unwrap());
        let result = reply.first.unwrap();
        assert!(result.cases[0].balance_kn.abs() < 1e-7);
        assert!(events
            .lock()
            .unwrap()
            .iter()
            .any(|s| s.contains("Разложение")));
        assert!(events
            .lock()
            .unwrap()
            .iter()
            .any(|s| s.contains("Вычисление неизвестных")));
        let reference =
            fem_ui::solve_snapshot(&request.mesh, &request.first, &request.cases).unwrap();
        for (a, b) in result.cases[0].w_mm.iter().zip(&reference.cases[0].w_mm) {
            assert!((a - b).abs() < 1e-12);
        }
        std::fs::write("target/solver_worker_fixture.json", text).unwrap();
        std::fs::write("target/solver_worker_expected.json",serde_json::to_string(&serde_json::json!({"applied":result.cases[0].applied_kn,"max_w":result.cases[0].w_mm.iter().copied().fold(0_f64,f64::max)})).unwrap()).unwrap();
    }
}
