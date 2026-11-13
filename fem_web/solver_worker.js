self.onmessage=async ({data})=>{
  try {
    const text=typeof data==='string'?data:data.text,channel=data.channel;
    const caseCount=JSON.parse(text).cases.length;
    let core;
    const build=await fetch('./solver_build.json',{cache:'no-cache'}).then(r=>r.json()).catch(()=>({parallel:false}));
    if(self.crossOriginIsolated && channel && build.parallel && caseCount>1){
      try {
        core=await import('./pkg_parallel/slab_contours_web.js');
        const wasm=await core.default();
        if(!(wasm.memory.buffer instanceof SharedArrayBuffer))throw new Error('Parallel WASM memory is not shared');
        await core.initThreadPool(Math.min(4,caseCount,Math.max(1,(navigator.hardwareConcurrency||2)-1)));
        if(channel)core.solver_parallel_configure(channel,Date.now(),data.threads||4);
        self.postMessage({kind:'progress',payload:JSON.stringify(['Параллельный пул готов',0])});
      } catch(error) {
        core=null;
        self.postMessage({kind:'progress',payload:JSON.stringify(['Последовательный режим: '+String(error),0])});
      }
    }
    if(!core){core=await import('./pkg/slab_contours_web.js');await core.default();}
    const progress=payload=>self.postMessage({kind:'progress',payload});
    if(data.binary && typeof core.solver_worker_run_bytes==='function'){
      const output=core.solver_worker_run_bytes(text,progress);
      self.postMessage({kind:'result_bytes',payload:output},output.buffer instanceof ArrayBuffer?[output.buffer]:[]);
    }else{
      const output=core.solver_worker_run(text,progress);
      self.postMessage({kind:'result',payload:output});
    }
  }catch(error){self.postMessage({kind:'error',payload:String(error?.stack||error)});}
};
