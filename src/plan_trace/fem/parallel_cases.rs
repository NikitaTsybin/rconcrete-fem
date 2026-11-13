//! One immutable factorization, bounded independent RHS/recovery workspaces.
#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
static MAX_WORKERS: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(4);
#[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
pub(super) fn set_limit(limit: usize) {
    MAX_WORKERS.store(limit.clamp(1, 4), std::sync::atomic::Ordering::Relaxed);
}
#[cfg(test)]
thread_local! {static LIMIT:std::cell::Cell<Option<usize>>=const{std::cell::Cell::new(None)};}
#[cfg(test)]
pub(super) fn with_workers<R>(count: usize, f: impl FnOnce() -> R) -> R {
    struct Reset(Option<usize>);
    impl Drop for Reset {
        fn drop(&mut self) {
            LIMIT.with(|v| v.set(self.0));
        }
    }
    let _reset = Reset(LIMIT.with(|v| v.replace(Some(count))));
    f()
}
pub(super) fn workers(cases: usize, dofs: usize) -> usize {
    #[cfg(not(target_arch = "wasm32"))]
    let available = std::thread::available_parallelism().map_or(1, usize::from);
    #[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
    let available =
        rayon::current_num_threads().min(MAX_WORKERS.load(std::sync::atomic::Ordering::Relaxed));
    #[cfg(all(target_arch = "wasm32", not(feature = "parallel-solver")))]
    let available = 1;
    let memory_limit = (256_usize * 1024 * 1024 / dofs.max(1).saturating_mul(256)).max(1);
    #[cfg(test)]
    let available = LIMIT.with(|v| v.get()).unwrap_or(available);
    available.min(4).min(cases.max(1)).min(memory_limit)
}
pub(super) fn map<T: Sync, R: Send>(
    cases: &[T],
    dofs: usize,
    f: impl Fn(&T) -> Result<R, String> + Sync,
) -> Result<Vec<R>, String> {
    let count = workers(cases.len(), dofs);
    super::progress::stage(&format!("Решение загружений: {} поток(а)", count));
    if count == 1 {
        let result = cases
            .iter()
            .enumerate()
            .map(|(i, c)| {
                super::progress::case_prefix(i + 1);
                let result = f(c);
                super::progress::stage(if result.is_ok() {
                    "Загружение завершено"
                } else {
                    "Загружение: ошибка"
                });
                result
            })
            .collect();
        super::progress::clear_case_prefix();
        return result;
    }
    let chunk = cases.len().div_ceil(count);
    #[cfg(not(target_arch = "wasm32"))]
    let batches = std::thread::scope(|scope| {
        let context = super::progress::capture();
        let mut handles = vec![];
        for (batch, items) in cases.chunks(chunk).enumerate() {
            let context = context.clone();
            let f = &f;
            handles.push(scope.spawn(move || {
                super::progress::attach(context);
                items
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        super::progress::case_prefix(batch * chunk + i + 1);
                        let result = f(c);
                        super::progress::stage(if result.is_ok() {
                            "Загружение завершено"
                        } else {
                            "Загружение: ошибка"
                        });
                        result
                    })
                    .collect::<Vec<_>>()
            }));
        }
        handles
            .into_iter()
            .map(|h| h.join().map_err(|_| "Ошибка потока загружения".to_owned()))
            .collect::<Result<Vec<_>, _>>()
    })?;
    #[cfg(all(target_arch = "wasm32", feature = "parallel-solver"))]
    let batches = {
        use rayon::prelude::*;
        cases
            .par_chunks(chunk)
            .enumerate()
            .map(|(batch, items)| {
                items
                    .iter()
                    .enumerate()
                    .map(|(i, c)| {
                        super::progress::case_prefix(batch * chunk + i + 1);
                        let result = f(c);
                        super::progress::stage(if result.is_ok() {
                            "Загружение завершено"
                        } else {
                            "Загружение: ошибка"
                        });
                        result
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>()
    };
    #[cfg(all(target_arch = "wasm32", not(feature = "parallel-solver")))]
    let batches: Vec<Vec<Result<R, String>>> = vec![];
    super::progress::clear_case_prefix();
    batches.into_iter().flatten().collect()
}
