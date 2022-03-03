extern crate bindgen;

use std::env;
use std::fs::read_dir;
use std::path::{Path, PathBuf};

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

    // # ALGORITHM #
    // Search through ament index at AMENT_PREFIX_PATH../share/ament_index/resource_index/packages/ to find packages to include
    // Packages will be located at (for each path in AMENT_PREFIX_PATHS):
    // > AMENT_PREFIX_PATH/include/ (For the header files) (OLD STYLE)
    //      -isystemAMENT_PREFIX_PATH/include
    // > AMENT_PREFIX_PATH/include/project_name (NEW STYLE)
    //      -isystemAMENT_PREFIX_PATH/include/project_name (if and only if it has a folder inside of it also named project_name)
    //      -isystemAMENT_PREFIX_PATH/include/CycloneDDS (special case, match for this)
    // -OR-
    // > AMENT_PREFIX_PATH/lib  (For the link libraries)
    //
    // See REP 122 for more details: https://www.ros.org/reps/rep-0122.html#filesystem-layout

    if let Ok(ament_prefix_paths) = ament_prefix_var {
        for ament_prefix_path in ament_prefix_paths.split(':').filter_map(|p| {
            let path = Path::new(p).to_path_buf();
            if path.is_dir() {
                Some(path)
            } else {
                None
            }
        }) {
            // Locate the ament index
            let mut ament_index = ament_prefix_path.clone();
            ament_index.push("share/ament_index/resource_index/packages");
            if !ament_index.is_dir() {
                continue;
            }

            // Including the old-style packages
            // todo<jhassold>: Remove this when old-style installation paths are fully removed
            builder = builder.clang_arg(format!(
                "-isystem{}/include",
                ament_prefix_path.to_string_lossy()
            ));

            // Search for and include new-style-converted packages
            for package_index_path in read_dir(&ament_index)
                .unwrap()
                .filter_map(|p| p.ok().map(|p| p.path()))
                .filter(|p| p.file_name().is_some())
            {
                let package = package_index_path.file_name().unwrap().to_string_lossy();

                let mut include_dir = ament_prefix_path.clone();
                include_dir.push(format!("include/{}", package.to_string()));

                if include_dir.is_dir() {
                    let mut new_style_include_dir = include_dir.clone();
                    new_style_include_dir.push(package.to_string());
                    if package == "CycloneDDS" || new_style_include_dir.is_dir() {
                        builder =
                            builder.clang_arg(format!("-isystem{}", include_dir.to_string_lossy()));
                    }
                }
            }

            // Link the native libraries
            println!(
                "cargo:rustc-link-search=native={}/lib",
                ament_prefix_path.to_string_lossy()
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
