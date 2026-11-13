import init, {reinforcement_worker_run} from './pkg/slab_contours_web.js';
self.onmessage=async ({data})=>{
  try {
    await init();
    const output=reinforcement_worker_run(data, payload=>self.postMessage({kind:'progress',payload}));
    self.postMessage({kind:'result',payload:output});
  } catch(error) { self.postMessage({kind:'error',payload:String(error?.stack || error)}); }
};
