use std::{env, path::Path};
const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";
const ROS_DISTRO: &str = "ROS_DISTRO";

fn get_env_var_or_abort(env_var: &'static str) -> String {
    if let Ok(value) = env::var(env_var) {
        value
    } else {
        panic!(
            "{} environment variable not set - please source ROS 2 installation first.",
            env_var
        );
    }
}

fn main() {
    println!(
        "cargo:rustc-check-cfg=cfg(ros_distro, values(\"{}\"))",
        ["humble", "jazzy", "kilted", "rolling"].join("\", \"")
    );
    let ros_distro = if let Ok(value) = env::var(ROS_DISTRO) {
        value
    } else {
        cfg_if::cfg_if! {
            if #[cfg(feature="use_ros_shim")] {
                use rustflags;
                // // Look for --cfg ros_distro=<ros_distro>
                for flag in rustflags::from_env() {
                    if matches!(flag, rustflags::Flag::Cfg { ref name, value : _ } if name == "ros_distro") {
                        if let rustflags::Flag::Cfg {name:_, value: flag_value} = flag {
                            println!("cargo:rustc-cfg=ros_distro=\"{}\"", flag_value.unwrap());
                            return;
                        } else {
                            continue;
                        }
                    }
                }
                let error_msg =
                    "When using the use_ros_shim feature, you must pass the ROS distribution you are targeting as a compiler flag with --cfg ros_distro=\"<ros_distro>\"";
                panic!("{}", error_msg);
            } else {
                let error_msg =
                    "ROS_DISTRO environment variable not set - please source ROS 2 installation first.";
                panic!("{}", error_msg);
            }
        }
    };
    println!("cargo:rustc-cfg=ros_distro=\"{ros_distro}\"");

    let ament_prefix_paths = get_env_var_or_abort(AMENT_PREFIX_PATH);
    for ament_prefix_path in ament_prefix_paths.split(':').map(Path::new) {
        // Link the native libraries
        let library_path = ament_prefix_path.join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_action");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");
}
