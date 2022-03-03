extern crate bindgen;

use std::env;
use std::fs::read_dir;
use std::path::PathBuf;

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

    let ament_prefix_var_name = "AMENT_PREFIX_PATH";
    let ament_prefix_var = env::var(ament_prefix_var_name);

    // #############
    // # ALGORITHM #
    // #############
    //
    // For each prefix in ${AMENT_PREFIX_PATH}:
    // Search through ament index at ${prefix}/share/ament_index/resource_index/packages/ to find packages to include
    // The include root will be located at either:
    // - ${prefix}/include/ (old style)
    // - ${prefix}/include/package_name (new style)
    // - ${prefix}/include/CycloneDDS (special case, match for this)
    // End of loop
    // Compiled libraries are always at ${prefix}/lib
    //
    // See REP 122 for more details: https://www.ros.org/reps/rep-0122.html#filesystem-layout

    if let Ok(ament_prefix_paths) = ament_prefix_var {
        for ament_prefix_path in ament_prefix_paths.split(':').map(PathBuf::from) {
            // Locate the ament index
            let ament_index = ament_prefix_path.join("share/ament_index/resource_index/packages");
            if !ament_index.is_dir() {
                continue;
            }

            // Base directory all headers will be
            let include_dir = ament_prefix_path.join("include");

            // Including the old-style packages
            // todo(jhdcs): Remove this when old-style installation paths are fully removed
            builder = builder.clang_arg(format!("-isystem{}", include_dir.to_string_lossy()));

            // Search for and include new-style-converted package paths
            for dir_entry in read_dir(&ament_index).unwrap().filter_map(|p| p.ok()) {
                let package = dir_entry.file_name().to_string_lossy().to_string();
                let project_include_dir = include_dir.join(&package);

                if project_include_dir.is_dir() {
                    // If the project_include_dir has another directory within it with the same name as the project
                    // then this is a new-style installation path
                    // todo(jhdcs): remove this check when no more packages use the old style
                    let new_style_include_dir = project_include_dir.join(&package);

                    // CycloneDDS is a special case - it needs to be included as if it were a new-style path, but
                    // doesn't actually have a secondary folder within it called "CycloneDDS"
                    // todo(jhdcs): if this changes in future, remove this check
                    if package == "CycloneDDS" || new_style_include_dir.is_dir() {
                        builder = builder.clang_arg(format!(
                            "-isystem{}",
                            project_include_dir.to_string_lossy()
                        ));
                    }
                }
            }

            // Link the native libraries
            let library_path = ament_prefix_path.join("lib");
            println!(
                "cargo:rustc-link-search=native={}",
                library_path.to_string_lossy()
            );
        }
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
