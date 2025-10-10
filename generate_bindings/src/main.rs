use cargo_manifest::Manifest;
use std::path::Path;
use std::fs::read_dir;

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";
const ROS_DISTRO: &str = "ROS_DISTRO";
const BINDGEN_WRAPPER: &str = "rclrs/src/rcl_wrapper.h";

fn get_env_var_or_abort(env_var: &'static str) -> String {
    if let Ok(value) = std::env::var(env_var) {
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
    let ament_prefix_paths = get_env_var_or_abort(AMENT_PREFIX_PATH);

    let Ok(workspace) = Manifest::from_path("Cargo.toml")
        .map_err(|_| ())
        .and_then(|manifest| manifest.workspace.ok_or(())) else
    {
        panic!(" > ERROR: Run the generate_bindings script from the root Cargo workspace of ros2_rust");
    };

    let has_rclrs = workspace.members.iter().find(|member | *member == "rclrs").is_some();
    if !has_rclrs {
        panic!(
            " > ERROR: Run the generate_bindings script from the root Cargo workspace of ros2_rust. \
            Could not find rclrs in this workspace. Members include {:?}",
            workspace.members,
        );
    }

    let mut builder = bindgen::Builder::default()
        .header(BINDGEN_WRAPPER)
        .derive_copy(false)
        .allowlist_type("rcl_.*")
        .allowlist_type("rmw_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("rosidl_.*")
        .allowlist_function("rcl_.*")
        .allowlist_function("rmw_.*")
        .allowlist_function("rcutils_.*")
        .allowlist_function("rosidl_.*")
        .allowlist_var("rcl_.*")
        .allowlist_var("rmw_.*")
        .allowlist_var("rcutils_.*")
        .allowlist_var("rosidl_.*")
        .layout_tests(false)
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    for ament_prefix_path in ament_prefix_paths.split(':').map(Path::new) {
        // Locate the ament index
        let ament_index = ament_prefix_path.join("share/ament_index/resource_index/packages");
        if !ament_index.is_dir() {
            continue;
        }

        // Old-style include directory
        let include_dir = ament_prefix_path.join("include");

        // Including the old-style packages
        builder = builder.clang_arg(format!("-isystem{}", include_dir.display()));

        // Search for and include new-style-converted package paths
        for dir_entry in read_dir(&ament_index).unwrap().filter_map(|p| p.ok()) {
            let package = dir_entry.file_name();
            let package_include_dir = include_dir.join(&package);

            if package_include_dir.is_dir() {
                let new_style_include_dir = package_include_dir.join(&package);

                // CycloneDDS is a special case - it needs to be included as if it were a new-style path, but
                // doesn't actually have a secondary folder within it called "CycloneDDS"
                // TODO(jhdcs): if this changes in future, remove this check
                if package == "CycloneDDS" || new_style_include_dir.is_dir() {
                    builder =
                        builder.clang_arg(format!("-isystem{}", package_include_dir.display()));
                }
            }
        }
    }

    let bindings = builder.generate().expect("Unable to generate bindings");
    let bindings_path = format!("rclrs/src/rcl_bindings_generated_{ros_distro}.rs");
    bindings.write_to_file(bindings_path).expect("Failed to generate bindings");
}
