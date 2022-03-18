use std::env;
use std::path::Path;

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";

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
    let ament_prefix_path_list = get_env_var_or_abort(AMENT_PREFIX_PATH);
    for ament_prefix_path in ament_prefix_path_list.split(':') {
        let library_path = Path::new(ament_prefix_path).join("lib");
        println!("cargo:rustc-link-search=native={}", library_path.display());
    }
}
