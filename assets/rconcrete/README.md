# Иконка rconcrete.ru для Mini FEM

Создана 28.10.2026 встроенным ImageGen по предоставленному пользователем
фирменному знаку rconcrete.ru. Эмблема R с бетонным каркасом, без мелкого текста.

- `rconcrete-icon-source.png` — оригинальный результат генерации, RGBA.
- `rconcrete-icon.png` — 512×512 для окна приложения, с прозрачностью.
- `rconcrete-icon.ico` — Windows: 16, 20, 24, 32, 40, 48, 64, 128 и 256 px.
- `rconcrete-icon.rc` — ресурс EXE, группа иконок №1.
- `rconcrete-icon-preview.png` — просмотр на тёмной подложке в малых размерах.

PNG встроен в приложение через `include_bytes!`, отдельные файлы около EXE
не требуются. `build.rs` встраивает ICO только в `slab_contours.exe`:
MSVC использует `rc.exe` Windows SDK (можно задать `RC`), GNU — `windres`.
При цели WASM/Linux ресурсный компилятор Windows не запускается.

Изменения будут видны после следующей сборки и перезапуска приложения.
