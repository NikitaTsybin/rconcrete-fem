#[cfg(not(target_arch = "wasm32"))]
pub fn spawn(f: impl FnOnce() + Send + 'static) {
    std::thread::spawn(f);
}
#[cfg(target_arch = "wasm32")]
pub fn spawn(f: impl FnOnce() + 'static) {
    #[wasm_bindgen::prelude::wasm_bindgen(
        inline_js = "export function afterPaint(){return new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(()=>setTimeout(resolve,0))));}"
    )]
    extern "C" {
        #[wasm_bindgen::prelude::wasm_bindgen(js_name = afterPaint)]
        fn after_paint() -> js_sys::Promise;
    }
    wasm_bindgen_futures::spawn_local(async move {
        // A resolved Promise only yields to microtasks, before the browser can
        // present the busy frame. Two animation frames allow an actual paint.
        let _ = wasm_bindgen_futures::JsFuture::from(after_paint()).await;
        f();
    });
}

#[cfg(not(target_arch = "wasm32"))]
pub fn write(path: impl AsRef<std::path::Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
    std::fs::write(path, bytes)
}
#[cfg(not(target_arch = "wasm32"))]
pub fn read(path: impl AsRef<std::path::Path>) -> std::io::Result<Vec<u8>> {
    std::fs::read(path)
}
#[cfg(target_arch = "wasm32")]
mod browser {
    use std::{
        cell::RefCell,
        collections::BTreeMap,
        path::{Path, PathBuf},
    };
    thread_local! {static FILES:RefCell<BTreeMap<PathBuf,Vec<u8>>>=RefCell::new(BTreeMap::new());static READY:RefCell<Vec<PathBuf>>=const{RefCell::new(vec![])};}
    thread_local! {
        static REPAINT: RefCell<Option<eframe::egui::Context>> = const { RefCell::new(None) };
    }
    pub fn set_repaint_context(ctx: &eframe::egui::Context) {
        REPAINT.with(|slot| {
            if slot.borrow().is_none() {
                *slot.borrow_mut() = Some(ctx.clone());
            }
        });
    }
    #[wasm_bindgen::prelude::wasm_bindgen(
        inline_js = "export function downloadFile(name,bytes){const u=URL.createObjectURL(new Blob([bytes]));const a=document.createElement('a');a.href=u;a.download=name;a.click();setTimeout(()=>URL.revokeObjectURL(u),10000);}"
    )]
    extern "C" {
        #[wasm_bindgen::prelude::wasm_bindgen(js_name=downloadFile)]
        fn download(name: &str, bytes: &[u8]);
    }
    pub fn write(path: impl AsRef<Path>, bytes: impl AsRef<[u8]>) -> std::io::Result<()> {
        download(&path.as_ref().to_string_lossy(), bytes.as_ref());
        Ok(())
    }
    pub fn read(path: impl AsRef<Path>) -> std::io::Result<Vec<u8>> {
        FILES
            .with(|f| f.borrow().get(path.as_ref()).cloned())
            .ok_or_else(|| std::io::Error::new(std::io::ErrorKind::NotFound, "file not selected"))
    }
    pub fn ready() -> Vec<PathBuf> {
        READY.with(|q| std::mem::take(&mut *q.borrow_mut()))
    }
    pub mod dialog {
        use super::*;
        #[derive(Default)]
        pub struct FileDialog {
            name: String,
            ext: Vec<String>,
        }
        impl FileDialog {
            pub fn new() -> Self {
                Self::default()
            }
            pub fn add_filter(mut self, _name: &str, ext: &[&str]) -> Self {
                self.ext = ext.iter().map(|s| s.to_string()).collect();
                self
            }
            pub fn set_file_name(mut self, name: impl AsRef<str>) -> Self {
                self.name = name.as_ref().into();
                self
            }
            pub fn save_file(self) -> Option<PathBuf> {
                Some(PathBuf::from(if self.name.is_empty() {
                    "export.json".into()
                } else {
                    self.name
                }))
            }
            pub fn pick_file(self) -> Option<PathBuf> {
                wasm_bindgen_futures::spawn_local(async move {
                    let ext = self.ext.iter().map(String::as_str).collect::<Vec<_>>();
                    if let Some(f) = rfd::AsyncFileDialog::new()
                        .add_filter("Files", &ext)
                        .pick_file()
                        .await
                    {
                        let path = PathBuf::from(f.file_name());
                        let bytes = f.read().await;
                        FILES.with(|m| m.borrow_mut().insert(path.clone(), bytes));
                        READY.with(|q| q.borrow_mut().push(path));
                        REPAINT.with(|ctx| {
                            if let Some(ctx) = ctx.borrow().as_ref() {
                                ctx.request_repaint();
                            }
                        });
                    }
                });
                None
            }
        }
        pub enum MessageLevel {
            Warning,
        }
        pub enum MessageButtons {
            YesNo,
        }
        #[derive(PartialEq)]
        pub enum MessageDialogResult {
            Yes,
            No,
        }
        #[derive(Default)]
        pub struct MessageDialog {
            text: String,
        }
        impl MessageDialog {
            pub fn new() -> Self {
                Self::default()
            }
            pub fn set_title(self, _title: &str) -> Self {
                self
            }
            pub fn set_description(mut self, text: impl AsRef<str>) -> Self {
                self.text = text.as_ref().into();
                self
            }
            pub fn set_level(self, _: MessageLevel) -> Self {
                self
            }
            pub fn set_buttons(self, _: MessageButtons) -> Self {
                self
            }
            pub fn show(self) -> MessageDialogResult {
                if web_sys::window()
                    .unwrap()
                    .confirm_with_message(&self.text)
                    .unwrap_or(false)
                {
                    MessageDialogResult::Yes
                } else {
                    MessageDialogResult::No
                }
            }
        }
    }
}
#[cfg(target_arch = "wasm32")]
pub use browser::*;

#[cfg(target_arch = "wasm32")]
pub async fn browser_yield() {
    let promise = js_sys::Promise::new(&mut |resolve, _reject| {
        web_sys::window()
            .expect("browser window")
            .set_timeout_with_callback_and_timeout_and_arguments_0(&resolve, 0)
            .expect("browser timer");
    });
    let _ = wasm_bindgen_futures::JsFuture::from(promise).await;
}
