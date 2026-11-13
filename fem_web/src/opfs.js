const directoryName = "rconcrete-fem-projects";
const savedTime = new Intl.DateTimeFormat("ru-RU", {
    day: "2-digit",
    month: "2-digit",
    year: "2-digit",
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
    hourCycle: "h23",
});

function isProjectName(name) {
    return typeof name === "string"
        && name.length > 4
        && /\.zip$/i.test(name)
        && !/[\\/\u0000-\u001f\u007f]/.test(name);
}

function requireProjectName(name) {
    if (!isProjectName(name)) {
        throw new Error("Укажите имя ZIP-файла без пути.");
    }
}

async function withDirectory(operation) {
    try {
        const storage = navigator.storage;
        if (typeof storage?.getDirectory !== "function") {
            throw new Error("OPFS недоступно. Откройте приложение по HTTPS или на localhost в браузере с поддержкой OPFS.");
        }
        const root = await storage.getDirectory();
        return await operation(await root.getDirectoryHandle(directoryName, { create: true }));
    } catch (error) {
        switch (error.name) {
            case "QuotaExceededError":
                throw new Error("В хранилище браузера недостаточно места для проекта.");
            case "NotAllowedError":
            case "SecurityError":
                throw new Error("Браузер заблокировал доступ к OPFS для этого сайта.");
            case "NotFoundError":
                throw new Error("Проект в OPFS не найден.");
            case "NoModificationAllowedError":
                throw new Error("Файл OPFS занят. Повторите сохранение после завершения другой записи.");
            default:
                throw error;
        }
    }
}

export async function listProjects() {
    return withDirectory(async directory => {
        const projects = [];
        for await (const entry of directory.values()) {
            if (entry.kind !== "file" || !isProjectName(entry.name)) continue;
            try {
                const file = await entry.getFile();
                projects.push([entry.name, savedTime.format(file.lastModified)]);
            } catch (error) {
                if (error.name !== "NotFoundError") throw error;
            }
        }
        return projects.sort(([a], [b]) => a.localeCompare(b, "ru", { numeric: true }));
    });
}

function withProjectWriteLock(operation) {
    const locks = navigator.locks;
    return typeof locks?.request === "function"
        ? locks.request(directoryName, () => operation(true))
        : operation(false);
}

export async function writeProject(name, bytes, overwrite, existingOnly = false) {
    requireProjectName(name);
    return withProjectWriteLock(hasLock => withDirectory(async directory => {
        if (typeof globalThis.FileSystemFileHandle?.prototype?.createWritable !== "function") {
            throw new Error("Этот браузер не поддерживает запись проектов в OPFS.");
        }
        let existing;
        try {
            existing = await directory.getFileHandle(name);
        } catch (error) {
            if (error.name !== "NotFoundError" || existingOnly) throw error;
        }
        if (existing && !overwrite) return false;

        const handle = existing ?? await directory.getFileHandle(name, { create: true });
        let stream;
        try {
            stream = await handle.createWritable();
            await stream.write(bytes);
            await stream.close();
            return true;
        } catch (error) {
            if (stream) await stream.abort().catch(() => {});
            // Only remove an entry when other tabs cannot replace it during cleanup.
            if (!existing && hasLock) await directory.removeEntry(name).catch(() => {});
            throw error;
        }
    }));
}

export async function deleteProject(name) {
    requireProjectName(name);
    return withProjectWriteLock(() => withDirectory(async directory => {
        await directory.getFileHandle(name);
        await directory.removeEntry(name);
    }));
}

async function projectFile(name) {
    requireProjectName(name);
    return withDirectory(async directory => {
        const handle = await directory.getFileHandle(name);
        return handle.getFile();
    });
}

export async function readProject(name) {
    const file = await projectFile(name);
    return new Uint8Array(await file.arrayBuffer());
}

export async function downloadProject(name) {
    const file = await projectFile(name);
    const url = URL.createObjectURL(file);
    const link = document.createElement("a");
    link.href = url;
    link.download = name;
    document.body.append(link);
    try {
        link.click();
    } finally {
        link.remove();
        setTimeout(() => URL.revokeObjectURL(url), 10000);
    }
}
