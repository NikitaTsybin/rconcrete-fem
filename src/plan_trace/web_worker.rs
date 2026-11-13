//! Worker lifetime is tied to a single RC job; dropping it aborts stale work.
use super::reinforcement::{Input, Output, Progress, Snapshot};
use std::sync::{mpsc, Arc};
use wasm_bindgen::{prelude::*, JsCast};
#[wasm_bindgen(inline_js = r#"export function startWorker(text,callback){
 const input=JSON.parse(text), count=Math.min(4,Math.max(1,(navigator.hardwareConcurrency||2)-1),Math.max(1,input.groups.length));
 const workers=[], snapshots=Array.from({length:count},()=>({stage:0,done:[0,0,0],elapsed_ms:[0,0,0],unchanged:0,corrected:0}));
 const output={cells:Array(input.display_count),physical_count:input.groups.length,wall_displays:{},errors:[]}; let finished=0,failed=false;
 const fail=e=>{if(!failed){failed=true;workers.forEach(w=>w.terminate());callback('error',String(e));}};
 for(let k=0;k<count;k++){
  const groups=input.groups.filter((_,i)=>i%count===k), ids=groups.flatMap(g=>g.displays), lookup=new Map(ids.map((id,i)=>[id,i]));
  const chunk={...input,
   groups:groups.map(g=>({...g,displays:g.displays.map(id=>lookup.get(id))})),
   display_count:ids.length,
   cases:Object.fromEntries(Object.entries(input.cases).map(([key,values])=>[key,ids.map(id=>values[id])])),
   nonlinear_offset:input.nonlinear_offset==null?null:ids.map(id=>input.nonlinear_offset[id])};
  const worker=new Worker(new URL('../../../reinforcement_worker.js',import.meta.url),{type:'module'});workers.push(worker);
  worker.onerror=e=>{e.preventDefault();fail(e.message||'Worker failed');};worker.onmessageerror=()=>fail('Worker message could not be read');
  worker.onmessage=({data})=>{if(failed)return;try{
   if(data.kind==='progress'){snapshots[k]=JSON.parse(data.payload);const p={stage:Math.min(...snapshots.map(s=>s.stage)),done:[0,1,2].map(j=>snapshots.reduce((n,s)=>n+s.done[j],0)),elapsed_ms:[0,1,2].map(j=>Math.max(...snapshots.map(s=>s.elapsed_ms[j]))),unchanged:snapshots.reduce((n,s)=>n+s.unchanged,0),corrected:snapshots.reduce((n,s)=>n+s.corrected,0)};callback('progress',JSON.stringify(p));}
   else if(data.kind==='result'){
    const r=JSON.parse(data.payload);
    ids.forEach((id,i)=>output.cells[id]=r.cells[i]);
    for(const [surface,local] of Object.entries(r.wall_displays)) output.wall_displays[surface]=ids[local];
    output.errors=output.errors.concat(r.errors);
    worker.terminate();if(++finished===count)callback('result',JSON.stringify(output));
   }
   else fail(data.payload);
  }catch(e){fail(e);}};
  worker.postMessage(JSON.stringify(chunk));
 }
 return {workers};
}
export function stopWorker(pool){pool.workers.forEach(worker=>{worker.onmessage=null;worker.onerror=null;worker.onmessageerror=null;worker.terminate();});}"#)]
extern "C" {
    #[wasm_bindgen(catch,js_name=startWorker)]
    fn start_worker(input: &str, callback: &js_sys::Function) -> Result<JsValue, JsValue>;
    #[wasm_bindgen(js_name=stopWorker)]
    fn stop_worker(worker: &JsValue);
}
pub struct Handle {
    worker: JsValue,
    _callback: Closure<dyn FnMut(String, String)>,
}
impl Handle {
    pub fn start(
        input: Input,
        progress: Arc<Progress>,
        tx: mpsc::Sender<Result<Output, String>>,
        ctx: eframe::egui::Context,
    ) -> Result<Self, String> {
        let text = serde_json::to_string(&input).map_err(|e| e.to_string())?;
        let callback =
            Closure::<dyn FnMut(String, String)>::new(move |kind: String, payload: String| {
                match kind.as_str() {
                    "progress" => match serde_json::from_str::<Snapshot>(&payload) {
                        Ok(s) => s.apply(&progress),
                        Err(e) => {
                            let _ = tx.send(Err(format!("Прогресс worker: {e}")));
                        }
                    },
                    "result" => {
                        let output = serde_json::from_str::<Output>(&payload)
                            .map_err(|e| format!("Результат worker: {e}"));
                        let _ = tx.send(output);
                    }
                    _ => {
                        let _ = tx.send(Err(payload));
                    }
                }
                ctx.request_repaint();
            });
        let worker = start_worker(&text, callback.as_ref().unchecked_ref())
            .map_err(|e| format!("Web Worker: {e:?}"))?;
        Ok(Self {
            worker,
            _callback: callback,
        })
    }
}
impl Drop for Handle {
    fn drop(&mut self) {
        stop_worker(&self.worker);
    }
}
