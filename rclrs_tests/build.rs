use std::env;

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
    let ros_distro = get_env_var_or_abort(ROS_DISTRO);
    println!("cargo:rustc-cfg=ros_distro=\"{ros_distro}\"");
}
