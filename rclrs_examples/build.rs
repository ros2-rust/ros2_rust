use std::env;
use std::path::Path;

fn main() {
    let dir = env::var("CARGO_MANIFEST_DIR").unwrap();
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/rclrs/lib/");
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/rcl/lib/");
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/rcutils/lib/");
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/rmw/lib/");
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/std_msgs/lib/");
    println!("cargo:rustc-link-search=native=/home/esteve/Projects/rust/ros2_rust_ws/install_isolated/builtin_interfaces/lib/");
}
