use proc_macro::TokenStream;

/// Derive macro for declaring ROS 2 parameters from struct definitions.
///
/// See `rclrs::ParameterSet` for full documentation.
#[proc_macro_derive(ParameterSet, attributes(parameters, param))]
pub fn derive_parameter_set(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    match parameter_set::expand(input) {
        Ok(tokens) => tokens.into(),
        Err(e) => e.to_compile_error().into(),
    }
}

mod parameter_set;
