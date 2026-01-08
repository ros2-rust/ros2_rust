use std::{env, fs, path::PathBuf};

use ament_rs::{search_paths::get_search_paths, AMENT_PREFIX_PATH_ENV_VAR};
use cargo_toml::Manifest;

const ROS_DISTRO: &str = "ROS_DISTRO";
const KNOWN_DISTROS: &[&str] = &["humble", "jazzy", "kilted", "rolling"];

fn get_ros_distro() -> String {
    env::var(ROS_DISTRO)
        .or_else(|_| {
            if env::var("CARGO_FEATURE_USE_ROS_SHIM").is_ok() {
                rustflags::from_env()
                    .find_map(|f| match f {
                        rustflags::Flag::Cfg { name, value } if name.as_str() == "ros_distro" => {
                            value
                        }
                        _ => None,
                    })
                    .ok_or_else(|| "Missing --cfg ros_distro in RUSTFLAGS".to_string())
            } else {
                Err(format!("Set {ROS_DISTRO} or use ROS shim"))
            }
        })
        .expect("Failed to determine ROS distro")
}

fn is_marked_for_reexport(path: &PathBuf) -> bool {
    fs::read_to_string(path)
        .map(|s| s.contains("[package.metadata.rclrs]") && s.contains("reexport = true"))
        .unwrap_or(false)
}

fn star_deps_to_use(manifest: &Manifest) -> String {
    // Find all dependencies for this crate that have a `*` version requirement.
    // We will assume that these are other exported dependencies that need symbols
    // exposed in their module.
    manifest
        .dependencies
        .iter()
        .filter(|(_, version)| version.req() == "*")
        .map(|(name, _)| format!("use crate::{name};\n"))
        .collect::<String>()
}

fn main() {
    println!(
        "cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))",
        KNOWN_DISTROS.join("\", \"")
    );
    println!("cargo:rustc-cfg=ros_distro=\"{}\"", get_ros_distro());
    println!("cargo:rerun-if-env-changed={ROS_DISTRO}");
    println!("cargo:rerun-if-env-changed={AMENT_PREFIX_PATH_ENV_VAR}");

    let ament_prefix_paths = get_search_paths().unwrap_or_default();

    for ament_prefix_path in &ament_prefix_paths {
        // Link the native libraries
        let library_path = PathBuf::from(ament_prefix_path).join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }

    // Re-export any generated interface crates that we find
    let export_crate_tomls: Vec<PathBuf> = ament_prefix_paths
        .iter()
        .map(PathBuf::from)
        .flat_map(|base_path| {
            // 1. Try to read share/ directory
            fs::read_dir(base_path.join("share")).into_iter().flatten()
        })
        .filter_map(|entry| entry.ok())
        .filter(|entry| entry.path().is_dir())
        .flat_map(|package_dir| {
            // 2. Try to read <package>/rust/ directory
            fs::read_dir(package_dir.path().join("rust"))
                .into_iter()
                .flatten()
        })
        .filter_map(|entry| entry.ok())
        .map(|entry| entry.path())
        .filter(|path| path.file_name() == Some(std::ffi::OsStr::new("Cargo.toml")))
        .filter(is_marked_for_reexport)
        .collect();

    let content: String = export_crate_tomls
        .iter()
        .filter_map(|path| path.parent().map(|p| p.to_path_buf()))
        .map(|package_dir| {
            let package = package_dir
                .parent()
                .unwrap()
                .file_name()
                .unwrap()
                .to_str()
                .unwrap();

            // Find all dependencies for this crate that have a `*` version requirement.
            // We will assume that these are other exported dependencies that need symbols
            // exposed in their module.
            let dependencies: String = Manifest::from_path(package_dir.clone().join("Cargo.toml"))
                .iter()
                .map(star_deps_to_use)
                .collect();

            let internal_mods: String = fs::read_dir(package_dir.join("src"))
                .into_iter()
                .flatten()
                .filter_map(|entry| entry.ok())
                .filter(|entry| entry.path().is_file())
                // Ignore lib.rs and any rmw.rs. lib.rs is only used if the crate is consumed
                // independently, and rmw.rs files need their top level module
                // (i.e., msg, srv, action) to exist to be re-exported.
                .filter(|entry| {
                    let name = entry.file_name();
                    name != "lib.rs" && name != "rmw.rs"
                })
                // Wrap the inclusion of each file in a module matching the file stem
                // so that the generated code can be imported like `rclrs::std_msgs::msgs::Bool`
                .filter_map(|e| {
                    e.path()
                        .file_stem()
                        .and_then(|stem| stem.to_str())
                        .map(|stem| {
                            let idiomatic_path = e.path().display().to_string();
                            let sep = std::path::MAIN_SEPARATOR;
                            let rmw_path = idiomatic_path
                                .rsplit_once(std::path::MAIN_SEPARATOR)
                                .map(|(dir, _)| format!("{dir}{sep}{stem}{sep}rmw.rs"))
                                .unwrap_or_else(|| "rmw.rs".to_string());

                            // TODO I would like to run rustfmt on this generated code, similar to how bindgen does it
                            format!("pub mod {stem} {{ {dependencies} include!(\"{idiomatic_path}\"); pub mod rmw {{ {dependencies} include!(\"{rmw_path}\"); }} }}")
                        })
                })
                .collect();

            format!("#[allow(unused_imports, missing_docs)]\npub mod {package} {{ {internal_mods} }}")
        })
        .collect();

    let out_path =
        PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR not set")).join("interfaces.rs");
    fs::write(out_path, content).expect("Failed to write interfaces.rs");

    [
        "rcl",
        "rcl_action",
        "rcl_yaml_param_parser",
        "rcutils",
        "rmw",
        "rmw_implementation",
    ]
    .iter()
    .for_each(|lib| println!("cargo:rustc-link-lib=dylib={lib}"));
}
