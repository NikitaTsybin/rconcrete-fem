use super::*;
use js_sys::{Array, Promise, Uint8Array};
use std::{future::Future, sync::mpsc};
use wasm_bindgen::{prelude::wasm_bindgen, JsCast, JsValue};
use wasm_bindgen_futures::{spawn_local, JsFuture};

#[wasm_bindgen(module = "/src/opfs.js")]
extern "C" {
    #[wasm_bindgen(js_name = listProjects)]
    fn list_projects() -> Promise;
    #[wasm_bindgen(js_name = writeProject)]
    fn write_project(
        name: &str,
        bytes: &Uint8Array,
        overwrite: bool,
        existing_only: bool,
    ) -> Promise;
    #[wasm_bindgen(js_name = readProject)]
    fn read_project(name: &str) -> Promise;
    #[wasm_bindgen(js_name = downloadProject)]
    fn download_project(name: &str) -> Promise;
    #[wasm_bindgen(js_name = deleteProject)]
    fn delete_project(name: &str) -> Promise;
}

#[derive(Default)]
pub(crate) struct State {
    projects: Option<Vec<ProjectFile>>,
    linked_name: Option<String>,
    error: Option<String>,
    pending: Option<mpsc::Receiver<Result<Reply, String>>>,
    dialog: Option<Dialog>,
    opening: bool,
}

struct ProjectFile {
    name: String,
    saved_at: String,
}

enum Dialog {
    Name(String),
    Replace(String),
    Open(String),
    Delete(String),
}

enum SaveMode {
    Create,
    Replace,
    Update,
}

enum Reply {
    Listed(Vec<ProjectFile>),
    Exists(String),
    Saved(String, Option<Vec<u8>>),
    Read(String, Vec<u8>),
    Downloaded(String),
    Deleted(String),
}

fn project_name(input: &str) -> Option<String> {
    let input = input.trim();
    if input.is_empty()
        || matches!(input, "." | "..")
        || input
            .chars()
            .any(|c| c.is_control() || "/\\:*?\"<>|".contains(c))
    {
        return None;
    }
    let lower = input.to_ascii_lowercase();
    let name = if lower.ends_with(".zip") {
        input.to_owned()
    } else if lower.ends_with(".slab") {
        format!("{input}.zip")
    } else {
        format!("{input}.slab.zip")
    };
    (5..=240).contains(&name.len()).then_some(name)
}

async fn browser(promise: Promise) -> Result<JsValue, String> {
    JsFuture::from(promise).await.map_err(|error| {
        js_sys::Reflect::get(&error, &JsValue::from_str("message"))
            .ok()
            .and_then(|v| v.as_string())
            .or_else(|| error.as_string())
            .unwrap_or_else(|| "Не удалось обратиться к хранилищу браузера".into())
    })
}

impl State {
    fn start(
        &mut self,
        ctx: &egui::Context,
        task: impl Future<Output = Result<Reply, String>> + 'static,
    ) {
        let (send, receive) = mpsc::channel();
        self.error = None;
        self.pending = Some(receive);
        let ctx = ctx.clone();
        spawn_local(async move {
            if send.send(task.await).is_ok() {
                ctx.request_repaint();
            }
        });
    }

    fn refresh(&mut self, ctx: &egui::Context) {
        self.start(ctx, async {
            let list = browser(list_projects()).await?;
            Ok(Reply::Listed(
                Array::from(&list)
                    .iter()
                    .filter_map(|value| {
                        let pair = value.dyn_into::<Array>().ok()?;
                        Some(ProjectFile {
                            name: pair.get(0).as_string()?,
                            saved_at: pair.get(1).as_string()?,
                        })
                    })
                    .collect(),
            ))
        });
    }
}

impl App {
    pub(crate) fn opfs_controls(&mut self, ui: &mut egui::Ui, ready: bool) {
        egui::CollapsingHeader::new("OPFS").show(ui, |ui| {
            if self.opfs.projects.is_none()
                && self.opfs.pending.is_none()
                && self.opfs.error.is_none()
            {
                self.opfs.refresh(ui.ctx());
            }
            let busy = self.opfs.pending.is_some();
            ui.horizontal_wrapped(|ui| {
                if ui
                    .add_enabled(ready && !busy, egui::Button::new("Сохранить"))
                    .clicked()
                {
                    if let Some(name) = self.opfs.linked_name.clone() {
                        self.save_opfs(ui.ctx(), name, SaveMode::Update);
                    } else {
                        self.name_opfs_project(ui.ctx());
                    }
                }
                if ui
                    .add_enabled(ready && !busy, egui::Button::new("Сохранить как…"))
                    .clicked()
                {
                    self.name_opfs_project(ui.ctx());
                }
                if ui
                    .add_enabled(!busy, egui::Button::new("Обновить"))
                    .clicked()
                {
                    self.opfs.refresh(ui.ctx());
                }
                if busy {
                    ui.spinner();
                }
            });
            if let Some(error) = &self.opfs.error {
                ui.label(error);
            }
            let mut open = None;
            let mut download = None;
            let mut delete = None;
            if let Some(projects) = &self.opfs.projects {
                if projects.is_empty() && !busy {
                    ui.label("Нет сохранённых проектов");
                }
                for ProjectFile { name, saved_at } in projects {
                    let time = ui.painter().layout_no_wrap(
                        saved_at.clone(),
                        egui::TextStyle::Body.resolve(ui.style()),
                        ui.visuals().text_color(),
                    );
                    let row = ui.horizontal(|ui| {
                        let width =
                            (ui.available_width() - time.size().x - ui.spacing().item_spacing.x)
                                .max(0.);
                        ui.allocate_ui_with_layout(
                            egui::vec2(width, time.size().y),
                            egui::Layout::left_to_right(egui::Align::Center),
                            |ui| {
                                ui.set_width(width);
                                ui.add(egui::Label::new(name).truncate().selectable(false));
                            },
                        );
                        ui.add(egui::Label::new(time).selectable(false));
                    });
                    ui.interact(
                        row.response.rect,
                        ui.id().with(("opfs_file", name)),
                        egui::Sense::click(),
                    )
                    .on_hover_text(format!("{name}\n{saved_at}"))
                    .context_menu(|ui| {
                        if ui
                            .add_enabled(
                                !busy && self.load.is_none() && !self.pdf_busy(),
                                egui::Button::new("Открыть"),
                            )
                            .clicked()
                        {
                            open = Some(name.clone());
                            ui.close_menu();
                        }
                        if ui
                            .add_enabled(!busy, egui::Button::new("Скачать"))
                            .clicked()
                        {
                            download = Some(name.clone());
                            ui.close_menu();
                        }
                        if ui
                            .add_enabled(!busy, egui::Button::new("Удалить"))
                            .clicked()
                        {
                            delete = Some(name.clone());
                            ui.close_menu();
                        }
                    });
                }
            }
            if let Some(name) = open {
                if self.project_state_bytes() != self.saved_project_state || self.mode.is_some() {
                    self.opfs.dialog = Some(Dialog::Open(name));
                } else {
                    self.open_opfs(ui.ctx(), name);
                }
            } else if let Some(name) = download {
                self.opfs.start(ui.ctx(), async move {
                    browser(download_project(&name)).await?;
                    Ok(Reply::Downloaded(name))
                });
            } else if let Some(name) = delete {
                self.opfs.dialog = Some(Dialog::Delete(name));
            }
        });
    }

    fn name_opfs_project(&mut self, ctx: &egui::Context) {
        let name = self
            .opfs
            .linked_name
            .clone()
            .or_else(|| {
                self.project_path
                    .as_deref()
                    .and_then(Path::file_name)
                    .and_then(|name| name.to_str())
                    .map(str::to_owned)
            })
            .unwrap_or_else(|| "Проект.slab.zip".into());
        self.opfs.dialog = Some(Dialog::Name(name));
        ctx.memory_mut(|memory| memory.request_focus(egui::Id::new("opfs_project_name")));
    }

    fn open_opfs(&mut self, ctx: &egui::Context, name: String) {
        self.opfs.opening = true;
        self.opfs.start(ctx, async move {
            let bytes = browser(read_project(&name)).await?;
            let bytes = bytes
                .dyn_into::<Uint8Array>()
                .map_err(|_| "Некорректный файл OPFS")?;
            Ok(Reply::Read(name, bytes.to_vec()))
        });
    }

    fn save_opfs(&mut self, ctx: &egui::Context, name: String, mode: SaveMode) {
        self.sync_slab_cuts();
        let mut project = self.project_snapshot();
        self.opfs.start(ctx, async move {
            crate::plan_trace::platform::browser_yield().await;
            let signature = project.state_bytes();
            let bytes = Uint8Array::from(project.write_archive()?.as_slice());
            let written = browser(write_project(
                &name,
                &bytes,
                !matches!(mode, SaveMode::Create),
                matches!(mode, SaveMode::Update),
            ))
            .await?;
            if written.as_bool() == Some(true) {
                Ok(Reply::Saved(name, signature))
            } else {
                Ok(Reply::Exists(name))
            }
        });
    }

    pub(crate) fn opfs_dialog(&mut self, ctx: &egui::Context) {
        if self.opfs.opening {
            egui::Modal::new(egui::Id::new("opfs_opening")).show(ctx, |ui| {
                ui.spinner();
                ui.label("Открытие проекта из OPFS…");
            });
            return;
        }
        let Some(mut dialog) = self.opfs.dialog.take() else {
            return;
        };
        let modal = egui::Modal::new(egui::Id::new("opfs_dialog")).show(ctx, |ui| {
            let action = match &mut dialog {
                Dialog::Name(name) => {
                    ui.heading("Сохранить в OPFS");
                    ui.label("Имя проекта");
                    let field = ui.add(
                        egui::TextEdit::singleline(name).id(egui::Id::new("opfs_project_name")),
                    );
                    let name = project_name(name);
                    let enter = field.lost_focus() && ui.input(|i| i.key_pressed(egui::Key::Enter));
                    let button = ui
                        .add_enabled(name.is_some(), egui::Button::new("Сохранить"))
                        .on_disabled_hover_text(
                            "Введите имя файла без пути и запрещённых символов",
                        );
                    if button.clicked() || enter {
                        name
                    } else {
                        None
                    }
                }
                Dialog::Replace(name) => {
                    ui.heading("Заменить проект в OPFS?");
                    ui.label(name.as_str());
                    ui.button("Заменить").clicked().then(|| name.clone())
                }
                Dialog::Open(name) => {
                    ui.heading("Открыть проект из OPFS?");
                    ui.label("Текущий проект содержит несохранённые изменения.");
                    ui.button("Открыть без сохранения")
                        .clicked()
                        .then(|| name.clone())
                }
                Dialog::Delete(name) => {
                    ui.heading("Удалить проект из OPFS?");
                    ui.label(name.as_str());
                    if self.opfs.linked_name.as_ref() == Some(name) {
                        ui.label("Проект останется открыт в редакторе.");
                    }
                    ui.button("Удалить").clicked().then(|| name.clone())
                }
            };
            (action, ui.button("Отмена").clicked())
        });
        if let Some(name) = modal.inner.0.as_ref() {
            match dialog {
                Dialog::Name(_)
                    if self
                        .opfs
                        .projects
                        .as_ref()
                        .is_some_and(|files| files.iter().any(|file| file.name == *name)) =>
                {
                    self.opfs.dialog = Some(Dialog::Replace(name.clone()));
                }
                Dialog::Name(_) => self.save_opfs(ctx, name.clone(), SaveMode::Create),
                Dialog::Replace(_) => self.save_opfs(ctx, name.clone(), SaveMode::Replace),
                Dialog::Open(_) => self.open_opfs(ctx, name.clone()),
                Dialog::Delete(_) => {
                    let name = name.clone();
                    self.opfs.start(ctx, async move {
                        browser(delete_project(&name)).await?;
                        Ok(Reply::Deleted(name))
                    });
                }
            }
        } else if !modal.inner.1 && !modal.should_close() {
            self.opfs.dialog = Some(dialog);
        }
    }

    pub(crate) fn poll_opfs(&mut self) {
        let Some(pending) = &self.opfs.pending else {
            return;
        };
        let result = match pending.try_recv() {
            Ok(result) => result,
            Err(mpsc::TryRecvError::Empty) => return,
            Err(mpsc::TryRecvError::Disconnected) => Err("Операция прервана".into()),
        };
        self.opfs.pending = None;
        self.opfs.opening = false;
        match result {
            Ok(Reply::Listed(files)) => {
                if self
                    .opfs
                    .linked_name
                    .as_ref()
                    .is_some_and(|name| !files.iter().any(|file| &file.name == name))
                {
                    self.opfs.linked_name = None;
                }
                self.opfs.projects = Some(files);
            }
            Ok(Reply::Exists(name)) => self.opfs.dialog = Some(Dialog::Replace(name)),
            Ok(Reply::Saved(name, signature)) => {
                self.opfs.linked_name = Some(name.clone());
                self.saved_project_state = signature;
                self.status = format!("Проект сохранён в OPFS · {name}");
                self.opfs.projects = None;
            }
            Ok(Reply::Read(name, bytes)) => {
                let projects = self.opfs.projects.take();
                let result = Project::read_archive(bytes)
                    .and_then(|project| self.apply_project(project, PathBuf::from(&name)));
                self.opfs.projects = projects;
                match result {
                    Ok(()) => {
                        self.opfs.linked_name = Some(name);
                        self.status = format!("OPFS · {}", self.status);
                    }
                    Err(error) => self.opfs_error(error),
                }
            }
            Ok(Reply::Downloaded(name)) => self.status = format!("Скачивание из OPFS · {name}"),
            Ok(Reply::Deleted(name)) => {
                if self.opfs.linked_name.as_ref() == Some(&name) {
                    self.opfs.linked_name = None;
                }
                if let Some(files) = &mut self.opfs.projects {
                    files.retain(|file| file.name != name);
                }
                self.status = format!("Проект удалён из OPFS · {name}");
            }
            Err(error) => self.opfs_error(error),
        }
    }

    fn opfs_error(&mut self, error: String) {
        self.status = format!("OPFS · {error}");
        self.opfs.error = Some(error);
    }
}
