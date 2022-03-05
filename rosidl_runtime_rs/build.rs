use std::env;
use std::path::Path;

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";

fn main() {
    if let Ok(ament_prefix_path_list) = env::var(AMENT_PREFIX_PATH) {
        for ament_prefix_path in ament_prefix_path_list.split(':') {
            let library_path = Path::new(ament_prefix_path).join("lib");
            println!("cargo:rustc-link-search=native={}", library_path.display());
        }
    } else {
        panic!(
            "{} environment variable not set - please source ROS 2 installation first.",
            AMENT_PREFIX_PATH
        );
    }
}
