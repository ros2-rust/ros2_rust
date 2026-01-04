use std::env;
use std::fs;
use std::path::{Path, PathBuf};

use cargo_toml::Manifest;

use ament_rs::search_paths::get_search_paths;
use ament_rs::AMENT_PREFIX_PATH_ENV_VAR;

const ROS_DISTRO: &str = "ROS_DISTRO";
const KNOWN_DISTROS: &[&str] = &["humble", "jazzy", "kilted", "rolling"];

fn main() {
    println!("cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))", KNOWN_DISTROS.join("\", \""));
    println!("cargo:rustc-cfg=ros_distro=\"{}\"", get_ros_distro());
    println!("cargo:rerun-if-env-changed={ROS_DISTRO}");
    println!("cargo:rerun-if-env-changed={AMENT_PREFIX_PATH_ENV_VAR}");

    let search_paths = get_search_paths();

    // Link search paths
    search_paths.iter()
        .for_each(|p| println!("cargo:rustc-link-search=native={}", p.join("lib")));

    // Generate interfaces.rs
    let content: String = search_paths.iter()
        .filter_map(|base| fs::read_dir(base.join("share")).ok())
        .flatten()
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| path.is_dir())
        .map(|pkg_dir| pkg_dir.join("rust"))
        .filter(|rust_dir| is_marked_for_reexport(&rust_dir.join("Cargo.toml")))
        .filter_map(|rust_dir| build_package_module(&rust_dir))
        .collect();

    let out_path = PathBuf::from(env::var_os("OUT_DIR").unwrap()).join("interfaces.rs");
    fs::write(out_path, content).expect("Failed to write interfaces.rs");

    // Static library links
    ["rcl", "rcl_action", "rcl_yaml_param_parser", "rcutils", "rmw", "rmw_implementation"]
        .iter()
        .for_each(|lib| println!("cargo:rustc-link-lib=dylib={lib}"));
}

fn get_ros_distro() -> String {
    env::var(ROS_DISTRO).or_else(|_| {
        if env::var("CARGO_FEATURE_USE_ROS_SHIM").is_ok() {
            rustflags::from_env().find_map(|f| match f {
                rustflags::Flag::Cfg { name, value } if name.as_str() == "ros_distro" => value,
                _ => None,
            }).ok_or_else(|| "Missing --cfg ros_distro in RUSTFLAGS".to_string())
        } else {
            Err(format!("Set {ROS_DISTRO} or use ROS shim"))
        }
    }).expect("Failed to determine ROS distro")
}

fn is_marked_for_reexport(path: &Path) -> bool {
    fs::read_to_string(path)
        .map(|s| s.contains("[package.metadata.rclrs]") && s.contains("reexport = true"))
        .unwrap_or(false)
}

fn build_package_module(rust_dir: &Path) -> Option<String> {
    let manifest = Manifest::from_path(rust_dir.join("Cargo.toml")).ok()?;
    let pkg_name = rust_dir.parent()?.file_name()?.to_str()?;

    let deps_code: String = manifest.dependencies.iter()
        .filter(|(_, details)| details.req() == "*")
        .map(|(name, _)| format!("use crate::{name};\n"))
        .collect();

    let internal_mods: String = fs::read_dir(rust_dir.join("src")).ok()?
        .flatten()
        .filter(|e| e.path().is_file())
        .filter_map(|e| {
            let stem = e.path().file_stem()?.to_str()?.to_string();
            if stem == "lib" || stem == "rmw" { return None; }

            let path = e.path().display().to_string();
            let rmw = e.path().with_file_name(format!("{stem}/rmw.rs")).display().to_string();

            Some(format!(
                "pub mod {stem} {{ {deps_code} include!(\"{path}\"); \
                 pub mod rmw {{ {deps_code} include!(\"{rmw}\"); }} }}"
            ))
        })
        .collect();

    Some(format!("pub mod {pkg_name} {{ {internal_mods} }}\n"))
}