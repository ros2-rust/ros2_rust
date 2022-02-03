extern crate bindgen;

use std::env;
use std::path::{PathBuf, Path};

fn generate_clang_include_arg(ament_prefix_path: &str) -> Option<String> {
    let ament_prefix_path = Path::new(ament_prefix_path);
    if ament_prefix_path.exists() && ament_prefix_path.is_dir() {
        let include_dir = ament_prefix_path.join(Path::new("include"));
        if include_dir.exists() {
            // If the path exists, there should be a directory name. Otherwise, abort
            let project_name = ament_prefix_path.file_name().unwrap();
            let nested_project_include_path = include_dir.join(Path::new(project_name));
            println!("\nNested project include path: {}", nested_project_include_path.to_str().unwrap());
            if nested_project_include_path.exists() {
                println!("Nested path exists!");
                // The path was created from only `str` objects, this should be safe to do
                let clang_arg = format!("-I{}", nested_project_include_path.to_str().unwrap());
                return Some(clang_arg.to_owned());
            } else {
                println!("Nested path does not exist...");
                // The path was created from only `str` objects, this should be safe to do
                let clang_arg = format!("-I{}", include_dir.to_str().unwrap());
                return Some(clang_arg.to_owned());
            }
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
            if let Some(clang_arg) = generate_clang_include_arg(ament_prefix_path) {
                println!("Generated clang_arg: {}\n", clang_arg);
                builder = builder.clang_arg(clang_arg);
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
