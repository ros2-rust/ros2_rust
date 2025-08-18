use std::{
    env,
    fs::read_dir,
    path::{Path, PathBuf},
};

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";
const ROS_DISTRO: &str = "ROS_DISTRO";
const BINDGEN_WRAPPER: &str = "src/rcl_wrapper.h";

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
    let ros_distro = if let Ok(value) = env::var(ROS_DISTRO) {
        value
    } else {
        let error_msg =
            "ROS_DISTRO environment variable not set - please source ROS 2 installation first.";
        cfg_if::cfg_if! {
            if #[cfg(feature="use_ros_shim")] {
                println!("{}", error_msg);
                return;
            } else {
                panic!("{}", error_msg);
            }
        }
    };

    println!("cargo:rustc-check-cfg=cfg(ros_distro, values(\"humble\", \"jazzy\", \"rolling\"))");
    println!("cargo:rustc-cfg=ros_distro=\"{ros_distro}\"");

    let ament_prefix_paths = get_env_var_or_abort(AMENT_PREFIX_PATH);
    for ament_prefix_path in ament_prefix_paths.split(':').map(Path::new) {
        // Link the native libraries
        let library_path = ament_prefix_path.join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");
}
