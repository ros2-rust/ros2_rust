use std::{env, fs, path::Path};
use std::path::PathBuf;

use cargo_toml::Manifest;

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";
const ROS_DISTRO: &str = "ROS_DISTRO";

fn get_env_or_shim<F>(var_name: &str, shim_logic: F) -> String
where
    F: FnOnce() -> Result<String, String>,
{
    let res = env::var(var_name).or_else(|_| {
        if env::var("CARGO_FEATURE_USE_ROS_SHIM").is_ok() {
            shim_logic()
        } else {
            Err(format!(
                "{var_name} environment variable not set - please source ROS 2 installation first."
            ))
        }
    }).expect(format!("Failed to get {var_name}").as_str());

    // Make sure this script will rerun if the environment variable changes.
    // If we used `env!` or `option_env!`, we would not need this.
    println!("cargo:rerun-if-env-changed={var_name}");

    res
}

fn marked_reexport(cargo_toml: String) -> bool {
    cargo_toml.contains("[package.metadata.rclrs]")
        && cargo_toml.contains("reexport = true")
}

fn star_deps_to_use(manifest: &Manifest) -> String {
    manifest.dependencies
        .iter()
        .filter(|(_, version)| version.req() == "*")
        .map(|(name, _)| format!("use crate::{name};\n"))
        .collect::<String>()
}

fn main() {
    println!(
        "cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))",
        vec!["humble", "jazzy", "kilted", "rolling"].join("\", \"")
    );
    let ros_distro = get_env_or_shim(ROS_DISTRO, || {
        rustflags::from_env()
            .find_map(|flag| match flag {
                rustflags::Flag::Cfg { name, value } if &name == "ros_distro" => value,
                _ => None,
            })
            .ok_or_else(|| "When using use_ros_shim, you must pass --cfg ros_distro=\"...\" via RUSTFLAGS".to_string())
    });

    println!("cargo:rustc-cfg=ros_distro=\"{ros_distro}\"");

    let ament_prefix_paths = get_env_or_shim(AMENT_PREFIX_PATH, || Ok(String::new()));

    for ament_prefix_path in ament_prefix_paths.split(':').map(Path::new) {
        // Link the native libraries
        let library_path = ament_prefix_path.join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }

    // Re-export any generated interface crates that we find
    let export_crate_tomls = ament_prefix_paths
        .split(':')
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
        .filter(|path| {
            fs::read_to_string(path)
                .map(marked_reexport)
                .unwrap_or(false)
        });

    let content: String = export_crate_tomls
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
                            format!("pub mod {stem} {{ {dependencies} include!(\"{idiomatic_path}\"); pub mod rmw {{ {dependencies} include!(\"{rmw_path}\");}} }}")
                        })
                })
                .collect();

            format!("pub mod {package} {{ {internal_mods} }}")
        })
        .collect();

    let out_dir = env::var("OUT_DIR").expect("OUT_DIR not set ");
    let dest_path = PathBuf::from(out_dir).join("interfaces.rs");

    // TODO I would like to run rustfmt on this generated code, similar to how bindgen does it
    fs::write(&dest_path, content.clone()).expect("Failed to write interfaces.rs");

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_action");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");
}
