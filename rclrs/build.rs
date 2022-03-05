extern crate bindgen;

use std::env;
use std::fs::read_dir;
use std::path::{Path, PathBuf};

const AMENT_PREFIX_PATH: &str = "AMENT_PREFIX_PATH";

fn main() {
    let mut builder = bindgen::Builder::default()
        .header("src/rcl_wrapper.h")
        .use_core()
        .ctypes_prefix("cty")
        .derive_copy(false)
        .allowlist_recursively(true)
        .allowlist_type("rcl_.*")
        .allowlist_type("rmw_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_function("rcl_.*")
        .allowlist_function("rmw_.*")
        .allowlist_function("rcutils_.*")
        .allowlist_var("rcl_.*")
        .allowlist_var("rmw_.*")
        .allowlist_var("rcutils_.*")
        .size_t_is_usize(true)
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    // #############
    // # ALGORITHM #
    // #############
    //
    // For each prefix in ${AMENT_PREFIX_PATH}:
    // Search through ament index at ${prefix}/share/ament_index/resource_index/packages/ to find packages to include
    // The include root will be located at either:
    // - ${prefix}/include/ (old style)
    // - ${prefix}/include/${package_name} (new style)
    // - ${prefix}/include/CycloneDDS (special case, match for this)
    // End of loop
    // Compiled libraries are always at ${prefix}/lib
    //
    // See REP 122 for more details: https://www.ros.org/reps/rep-0122.html#filesystem-layout

    if let Ok(ament_prefix_paths) = env::var(AMENT_PREFIX_PATH) {
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

            // Link the native libraries
            let library_path = ament_prefix_path.join("lib");
            println!("cargo:rustc-link-search=native={}", library_path.display());
        }
    } else {
        panic!(
            "{} environment variable not set - please source ROS 2 installation first.",
            AMENT_PREFIX_PATH
        );
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");

    let bindings = builder.generate().expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("rcl_bindings.rs"))
        .expect("Couldn't write bindings!");
}
