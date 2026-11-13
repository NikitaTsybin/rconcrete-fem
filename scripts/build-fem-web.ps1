param([switch]$Serve,[int]$Port=8766,[switch]$NoParallel)
$ErrorActionPreference='Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
cargo build --manifest-path fem_web/Cargo.toml --release --target wasm32-unknown-unknown
if ($LASTEXITCODE -ne 0) {throw 'FEM WASM build failed'}
wasm-bindgen fem_web/target/wasm32-unknown-unknown/release/slab_contours_web.wasm --target web --out-dir fem_web/pkg
if ($LASTEXITCODE -ne 0) {throw 'wasm-bindgen failed'}
Copy-Item -LiteralPath assets/rconcrete/rconcrete-icon.png -Destination fem_web/icon.png -Force
if($NoParallel){
    python scripts/package-fem-web.py --no-parallel
} else {
    & "$PSScriptRoot/build-fem-web-parallel.ps1"
}
if ($LASTEXITCODE -ne 0) {throw 'FEM web archive failed'}
if ($Serve) { python scripts/serve-fem-web.py --port $Port }
