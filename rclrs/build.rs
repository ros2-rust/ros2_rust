use ament_rs::search_paths::get_search_paths;
use std::{env, path::PathBuf};

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

#[cfg(unix)]
fn link_search_paths(prefixes: &[String]) {
    use std::{collections::BTreeSet, fs, os::unix::fs::symlink};

    // One directory preserves AMENT precedence when Rustdoc sorts search paths.
    let directory =
        PathBuf::from(env::var_os("OUT_DIR").expect("OUT_DIR is set")).join("native-libraries");
    if directory.exists() {
        fs::remove_dir_all(&directory).expect("remove previous native link directory");
    }
    fs::create_dir_all(&directory).expect("create native link directory");
    let mut names = BTreeSet::new();
    for prefix in prefixes {
        let Ok(entries) = fs::read_dir(PathBuf::from(prefix).join("lib")) else {
            continue;
        };
        for entry in entries {
            let entry = entry.expect("read native library entry");
            let source = entry.path();
            if !source.is_file() || !names.insert(entry.file_name()) {
                continue;
            }
            symlink(
                fs::canonicalize(source).expect("resolve native library"),
                directory.join(entry.file_name()),
            )
            .expect("link native library");
        }
    }
    println!("cargo:rustc-link-search=native={}", directory.display());
}

#[cfg(not(unix))]
fn link_search_paths(prefixes: &[String]) {
    for prefix in prefixes {
        println!(
            "cargo:rustc-link-search=native={}",
            PathBuf::from(prefix).join("lib").display()
        );
    }
}

fn main() {
    println!(
        "cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))",
        KNOWN_DISTROS.join("\", \"")
    );
    println!("cargo:rustc-cfg=ros_distro=\"{}\"", get_ros_distro());
    println!("cargo:rerun-if-env-changed={ROS_DISTRO}");

    let ament_prefix_paths = get_search_paths().unwrap_or_default();

    link_search_paths(&ament_prefix_paths);
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-changed=build.rs");

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
