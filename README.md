# Rconcrete FEM

Приложение для построения и конечно-элементного расчёта моделей зданий. В редакторе 2D и 3D можно задавать этажи, плиты с отверстиями, стены, балки, колонны, опоры и нагрузки. Приложение строит КЭ-сетку и показывает перемещения, усилия, реакции и деформированную схему. Доступны расчёт собственных форм, общая устойчивость и подбор арматуры оболочек.

Проекты сохраняются в `.slab.zip`. Расчётная сетка и результаты строятся заново после открытия. Поддерживаются подложки DXF и PDF, импорт модели из ЛИРА (`.lir`), экспорт в ЛИРА TXT и PLAXIS 3D. В браузере проекты можно также сохранять в OPFS текущего сайта.

Сосредоточенные, линейные и площадные силы задаются по X, Y или Z. Положительная сила по Z направлена вниз. Сосредоточенные моменты Mx, My и Mz задаются отдельной командой. Горизонтальные силы и моменты поддерживаются в пространственном расчёте и 3D, их экспорт в ЛИРА и PLAXIS пока недоступен.

## Windows EXE

Для сборки нужны Rust с MSVC, Visual Studio Build Tools с инструментами C++ и Windows SDK. Команды выполняются из корня проекта.

```powershell
cargo build --locked --release --bin slab_contours
.\target\release\slab_contours.exe
```

Запуск из исходников в режиме разработки:

```powershell
cargo run --locked --bin slab_contours
```

## Веб

Для сборки нужны PowerShell, Python 3, Rust и `wasm-bindgen-cli` версии из `fem_web/Cargo.lock`. Для многопоточного расчётного модуля нужен nightly с компонентом `rust-src`.

```powershell
rustup target add wasm32-unknown-unknown
cargo install wasm-bindgen-cli --version 0.2.127 --locked
rustup toolchain install nightly-2026-11-02 --profile minimal --component rust-src
New-Item -ItemType Directory -Force dist
.\scripts\build-fem-web.ps1 -Serve
```

Откройте [локальный веб](http://127.0.0.1:8766/). Для сборки только обычного WASM используйте `-NoParallel`, тогда установка nightly не требуется. Чтобы запустить уже собранный веб, выполните `python scripts/serve-fem-web.py --port 8766`.

Для публикации распакуйте `dist/slab_contours_web.zip` целиком на HTTPS-сервере. Многопоточный расчёт требует заголовков `Cross-Origin-Opener-Policy: same-origin` и `Cross-Origin-Embedder-Policy: require-corp`. Веб сначала пробует WebGPU, затем WebGL2. Для принудительного запуска через WebGL2 добавьте к адресу `?graphics=webgl2`.
