param([string]$Toolchain='nightly-2026-11-02')
$ErrorActionPreference='Stop'
Set-Location (Split-Path $PSScriptRoot -Parent)
$previousFlags=$env:RUSTFLAGS
try {
    $env:RUSTFLAGS='-C target-feature=+atomics,+bulk-memory,+mutable-globals'
    cargo "+$Toolchain" rustc --manifest-path fem_web/Cargo.toml --target-dir fem_web/target_parallel --release --target wasm32-unknown-unknown --features parallel-solver -Z build-std=std,panic_abort -- -C link-arg=--shared-memory -C link-arg=--import-memory -C link-arg=--export-memory -C link-arg=--max-memory=4294967296 -C link-arg=--export=__wasm_init_tls -C link-arg=--export=__tls_size -C link-arg=--export=__tls_align -C link-arg=--export=__tls_base
    if($LASTEXITCODE -ne 0){throw 'Parallel FEM WASM build failed'}
    wasm-bindgen fem_web/target_parallel/wasm32-unknown-unknown/release/slab_contours_web.wasm --target web --out-dir fem_web/pkg_parallel
    if($LASTEXITCODE -ne 0){throw 'Parallel wasm-bindgen failed'}
    python scripts/package-fem-web.py
    if($LASTEXITCODE -ne 0){throw 'FEM archive failed'}
} finally {$env:RUSTFLAGS=$previousFlags}
