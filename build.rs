use std::{env, fs, path::PathBuf, process::Command};

fn windows_rc() -> PathBuf {
    if let Some(rc) = env::var_os("RC") {
        return PathBuf::from(rc);
    }
    let host = match env::consts::ARCH {
        "aarch64" => "arm64",
        "x86" => "x86",
        _ => "x64",
    };
    let kits = env::var_os("WindowsSdkDir").map(PathBuf::from).or_else(|| {
        env::var_os("ProgramFiles(x86)").map(|p| PathBuf::from(p).join("Windows Kits").join("10"))
    });
    if let Some(kits) = kits {
        if let Ok(entries) = fs::read_dir(kits.join("bin")) {
            let mut versions = entries
                .filter_map(Result::ok)
                .map(|e| e.path())
                .collect::<Vec<_>>();
            versions.sort();
            for version in versions.into_iter().rev() {
                let candidate = version.join(host).join("rc.exe");
                if candidate.is_file() {
                    return candidate;
                }
            }
        }
    }
    PathBuf::from("rc.exe")
}

fn embed(name: &str, bin: &str) {
    for ext in ["rc", "ico", "png"] {
        println!("cargo:rerun-if-changed=assets/rconcrete/{name}.{ext}");
    }
    println!("cargo:rerun-if-env-changed=RC");
    println!("cargo:rerun-if-env-changed=WindowsSdkDir");
    // No Windows tool or platform dependency is used for a WASM/Linux target.
    if env::var("CARGO_CFG_TARGET_OS").ok().as_deref() != Some("windows") {
        return;
    }
    let root = PathBuf::from(env::var_os("CARGO_MANIFEST_DIR").expect("manifest directory"));
    let out = PathBuf::from(env::var_os("OUT_DIR").expect("build output directory"));
    let assets = root.join("assets/rconcrete");
    let target_env = env::var("CARGO_CFG_TARGET_ENV").unwrap_or_default();
    let (output, status) = if target_env == "msvc" {
        let output = out.join(format!("{name}.res"));
        let status = Command::new(windows_rc())
            .current_dir(&assets)
            .arg("/nologo")
            .arg("/fo")
            .arg(&output)
            .arg(format!("{name}.rc"))
            .status()
            .expect("Windows SDK rc.exe is required to embed the application icon (or set RC)");
        (output, status)
    } else {
        let output = out.join(format!("{name}.o"));
        let arch = env::var("CARGO_CFG_TARGET_ARCH").unwrap_or_default();
        let target = match arch.as_str() {
            "x86_64" => "pe-x86-64",
            "x86" => "pe-i386",
            _ => panic!("Set up a resource compiler for this Windows GNU architecture"),
        };
        let compiler = env::var_os("RC").unwrap_or_else(|| "windres".into());
        let status = Command::new(compiler)
            .current_dir(&assets)
            .args([
                "--target",
                target,
                "-O",
                "coff",
                "-i",
                &format!("{name}.rc"),
                "-o",
            ])
            .arg(&output)
            .status()
            .expect("windres is required to embed the Windows application icon");
        (output, status)
    };
    assert!(
        status.success(),
        "Failed to compile the rconcrete application icon"
    );
    // Only the Slab/Mini FEM executable receives this brand resource.
    println!("cargo:rustc-link-arg-bin={bin}={}", output.display());
}

fn main() {
    embed("rconcrete-icon", "slab_contours");
}
