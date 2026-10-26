cfg_if::cfg_if! {
    if #[cfg(not(feature="use_ros_shim"))] {
        use std::env;

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

        /// Compiles the probe that reports the layout of the installed C
        /// sequence structs, so that the tests can check `Sequence<T>` against
        /// this ROS installation instead of against a Rust recreation of those
        /// structs.
        #[cfg(feature = "abi_check")]
        fn compile_sequence_abi_probe(ament_prefix_path_list: &str) {
            let mut probe = cc::Build::new();
            for ament_prefix_path in env::split_paths(ament_prefix_path_list) {
                // Iron and later nest the headers of a package one level
                // deeper, so offer both conventions and let the compiler pick.
                let include_path = ament_prefix_path.join("include");
                probe.include(include_path.join("rosidl_runtime_c"));
                probe.include(include_path.join("builtin_interfaces"));
                probe.include(include_path);
            }
            probe
                .file("src/sequence_abi.c")
                .compile("rosidl_rs_sequence_abi");

            println!("cargo:rustc-cfg=has_c_abi_probe");
        }
    }
}

// Gate the primitive-sequence layout on the ROS distro, like other distro
// differences in rclrs (starting with Lyrical, primitive sequences carry extra
// ABI flags; see `Sequence`/`BufferFlags`).
const ROS_DISTRO: &str = "ROS_DISTRO";
const KNOWN_DISTROS: &[&str] = &["humble", "jazzy", "kilted", "lyrical", "rolling"];

fn get_ros_distro() -> String {
    std::env::var(ROS_DISTRO)
        .or_else(|_| {
            if std::env::var("CARGO_FEATURE_USE_ROS_SHIM").is_ok() {
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
    println!("cargo:rustc-check-cfg=cfg(has_c_abi_probe)");

    #[cfg(not(feature = "use_ros_shim"))]
    {
        let ament_prefix_path_list = get_env_var_or_abort(AMENT_PREFIX_PATH);
        for ament_prefix_path in env::split_paths(&ament_prefix_path_list) {
            let library_path = ament_prefix_path.join("lib");
            println!("cargo:rustc-link-search=native={}", library_path.display());
        }

        #[cfg(feature = "abi_check")]
        compile_sequence_abi_probe(&ament_prefix_path_list);
    }

    // Invalidate the built crate whenever this script changes
    println!("cargo:rerun-if-changed=build.rs");
}
