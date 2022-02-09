extern crate bindgen;

use std::env;
use std::path::{Path, PathBuf};

fn generate_clang_include_arg(ament_prefix_path: &str) -> Option<Vec<String>> {
    let ament_prefix_path = Path::new(ament_prefix_path);
    if ament_prefix_path.exists() && ament_prefix_path.is_dir() {
        let include_dir = ament_prefix_path.join(Path::new("include"));
        if include_dir.exists() {
            // If the path exists, there should be a directory name. Otherwise, abort
            let project_name = ament_prefix_path.file_name().unwrap();

            // Always include top-level include dir
            let mut clang_flags = Vec::<String>::new();
            clang_flags.push(format!("-I{}", include_dir.to_str().unwrap()));
            
            // Generate potential include path
            let nested_project_include_path = include_dir.join(Path::new(project_name));
            println!(
                "\nNested project include path: {}",
                nested_project_include_path.to_str().unwrap()
            );

            if nested_project_include_path.exists() {
                println!("Nested path exists!");
                // The path was created from only `str` objects, this should be safe to do
                let generated_arg = format!("-I{}", nested_project_include_path.to_str().unwrap());
                println!("Generated clang_arg: {}\n", generated_arg);
                clang_flags.push(generated_arg)
            } 

            return Some(clang_flags);
        }
    }

    None
}

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

    if let Ok(ament_prefix_paths) = ament_prefix_var {
        for ament_prefix_path in ament_prefix_paths.split(':') {
            if let Some(clang_args) = generate_clang_include_arg(ament_prefix_path) {
                builder = builder.clang_args(clang_args);
            }
            println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
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
