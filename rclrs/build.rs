use ament_rs::{search_paths::get_search_paths, AMENT_PREFIX_PATH_ENV_VAR};
use std::{env, path::PathBuf};

const ROS_DISTRO: &str = "ROS_DISTRO";
const KNOWN_DISTROS: &[&str] = &["humble", "jazzy", "kilted", "lyrical", "rolling"];

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

fn main() {
    println!(
        "cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))",
        KNOWN_DISTROS.join("\", \"")
    );
    println!("cargo:rustc-cfg=ros_distro=\"{}\"", get_ros_distro());
    println!("cargo:rerun-if-env-changed={ROS_DISTRO}");

    let ament_prefix_paths = get_search_paths().unwrap_or_default();
    // The link search paths below come from this variable. Declaring it is
    // required, not optional: once a build script emits any rerun-if line,
    // Cargo reruns it only for the listed triggers.
    println!("cargo:rerun-if-env-changed={AMENT_PREFIX_PATH_ENV_VAR}");

    for ament_prefix_path in &ament_prefix_paths {
        // Link the native libraries
        let library_path = PathBuf::from(ament_prefix_path).join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }

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
