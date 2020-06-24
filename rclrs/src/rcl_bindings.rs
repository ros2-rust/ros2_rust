#[allow(dead_code)]

/// namespace and entry point for the RCL FFI bindings
mod rcl_bindings {
    #![allow(non_upper_case_globals)]
    #![allow(non_camel_case_types)]
    #![allow(non_snake_case)]
    #![allow(clippy::all)]

    include!(concat!(env!("OUT_DIR"), "/rcl_bindings.rs"));
}

pub(crate) use self::rcl_bindings::*;
