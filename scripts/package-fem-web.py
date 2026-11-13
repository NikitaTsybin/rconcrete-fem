from pathlib import Path
import zipfile
import sys
import json
root=Path(__file__).resolve().parents[1]
parallel='--no-parallel' not in sys.argv and (root/'fem_web/pkg_parallel/slab_contours_web_bg.wasm').exists()
manifest=root/'fem_web/solver_build.json'
manifest.write_text(json.dumps({'parallel':parallel}),encoding='utf-8')
files=[root/'fem_web/index.html',root/'fem_web/icon.png',root/'fem_web/reinforcement_worker.js',root/'fem_web/solver_worker.js',manifest]+list((root/'fem_web/pkg').rglob('*'))
if parallel: files+=list((root/'fem_web/pkg_parallel').rglob('*'))
files=[p for p in files if p.is_file() and p.suffix in ('.html','.png','.js','.wasm','.json')]
assert any(p.suffix=='.wasm' for p in files)
with zipfile.ZipFile(root/'dist/slab_contours_web.zip','w',zipfile.ZIP_DEFLATED) as z:
    for p in files:z.write(p,p.relative_to(root/'fem_web'))
print(root/'dist/slab_contours_web.zip')
