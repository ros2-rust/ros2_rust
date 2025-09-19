mod r#impl;
use proc_macro::TokenStream;
use syn::{parse_macro_input, DeriveInput};

#[proc_macro_derive(StructuredParameters, attributes(param))]
pub fn derive_struct_parameters(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    r#impl::derive_struct_parameters(input)
        .unwrap_or_else(|e| e.to_compile_error())
        .into()
}
