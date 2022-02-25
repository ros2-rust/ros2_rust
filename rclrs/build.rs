extern crate bindgen;

use std::env;
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

    if let Ok(ament_prefix_paths) = ament_prefix_var {
        for ament_prefix_path in ament_prefix_paths.split(':') {
            // Should not panic, since this was originally a valid UTF-8 String
            builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));

            // If this is RCL or rcutils, we need to add a special include
            match ament_prefix_path.split('/').last() {
                None => (),
                Some(project)
                    if project.eq("rcl")
                        || project.eq("rcutils")
                        || project.eq("rmw")
                        || project.eq("rcl_yaml_param_parser")
                        || project.eq("rosidl_runtime_c")
                        || project.eq("rosidl_typesupport_interface") =>
                {
                    builder =
                        builder.clang_arg(format!("-I{}/include/{}", ament_prefix_path, project))
                }
                Some(_) => (),
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
