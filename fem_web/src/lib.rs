#[path="../../src/plan_trace/mod.rs"] mod plan_trace;
#[wasm_bindgen::prelude::wasm_bindgen]
pub async fn start_application(webgl2: bool) -> Result<(), wasm_bindgen::JsValue> {
    plan_trace::start_web(webgl2).await
}

#[wasm_bindgen::prelude::wasm_bindgen]
pub fn stop_application() {
    plan_trace::stop_web();
}
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn reinforcement_worker_run(input:&str,progress:&js_sys::Function)->Result<String,wasm_bindgen::JsValue>{plan_trace::reinforcement::worker_run(input,progress)}

#[cfg(feature="inference-bench")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn inference_benchmark(text:&str,repeats:usize)->Result<String,wasm_bindgen::JsValue>{
    let err=|e:String|wasm_bindgen::JsValue::from_str(&e);
    let batches:Vec<Vec<[f64;9]>>=serde_json::from_str(text).map_err(|e|err(e.to_string()))?;
    let model=plan_trace::reinforcement::ml::Network::load(false).map_err(err)?;
    let mut max_relative=0_f64;
    for batch in &batches{for (r,b) in batch.iter().zip(model.strength_batch(batch)){let a=model.strength(*r);for i in 0..2{max_relative=max_relative.max((a[i]-b[i]).abs()/a[i].abs().max(1.));}}}
    let mut timings=vec![];
    for repeat in 0..3{for batch_mode in if repeat%2==0{[false,true]}else{[true,false]}{
        let start=web_time::Instant::now();let mut checksum=0.;
        for _ in 0..repeats{for batch in &batches{
            if batch_mode{for a in model.strength_batch(batch){checksum+=a[0]+a[1];}}
            else{for r in batch{let a=model.strength(*r);checksum+=a[0]+a[1];}}
        }}
        timings.push(serde_json::json!({"batch":batch_mode,"repeat":repeat,"ms":start.elapsed().as_secs_f64()*1000.,"checksum":checksum}));
    }}
    Ok(serde_json::json!({"timings":timings,"max_relative":max_relative,"predictions_per_run":repeats*batches.iter().map(|r|r.len()).sum::<usize>()}).to_string())
}

#[cfg(feature="inference-bench")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn model_strength_benchmark(text:&str,mode:u8,progress:&js_sys::Function)->Result<String,wasm_bindgen::JsValue>{
    let err=|e:String|wasm_bindgen::JsValue::from_str(&e);
    let input=serde_json::from_str(text).map_err(|e|err(e.to_string()))?;
    let result=plan_trace::reinforcement::benchmark_strength_model(input,mode,|stage,index|{let _=progress.call2(&wasm_bindgen::JsValue::NULL,&wasm_bindgen::JsValue::from_f64(stage as f64),&wasm_bindgen::JsValue::from_f64(index as f64));}).map_err(err)?;
    Ok(result.to_string())
}

#[wasm_bindgen::prelude::wasm_bindgen]
pub fn solver_worker_run(input:&str,progress:&js_sys::Function)->Result<String,wasm_bindgen::JsValue>{plan_trace::solver_worker_run(input,progress)}

#[cfg(feature="parallel-solver")]
pub use wasm_bindgen_rayon::init_thread_pool;
#[cfg(feature="parallel-solver")]
#[wasm_bindgen::prelude::wasm_bindgen]
pub fn solver_parallel_configure(channel:String,start:f64,limit:usize){plan_trace::solver_parallel_configure(channel,start,limit);}

#[wasm_bindgen::prelude::wasm_bindgen]
pub fn solver_worker_run_bytes(input:&str,progress:&js_sys::Function)->Result<Vec<u8>,wasm_bindgen::JsValue>{plan_trace::solver_worker_run_bytes(input,progress)}
