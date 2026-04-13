//! Implementation of the `#[derive(ParameterSet)]` macro.
//!
//! The macro generates a `ParameterSet` trait impl for a struct, expanding
//! each field into either a parameter builder chain (for leaf fields) or a
//! recursive `ParameterSet::declare` call (for nested structs).

mod codegen;
mod field_attrs;
mod field_info;
mod struct_attrs;

use proc_macro2::TokenStream;
use quote::quote;
use syn::{spanned::Spanned, DeriveInput};

use field_info::FieldInfo;
use struct_attrs::StructAttrs;

/// Converts a PascalCase string to snake_case.
///
/// Consecutive uppercase letters are each treated as separate words:
/// `"HTTPServer"` becomes `"h_t_t_p_server"`. This is acceptable for
/// typical ROS 2 PascalCase struct names.
fn to_snake_case(s: &str) -> String {
    let mut result = String::with_capacity(s.len() + 4);
    for (i, c) in s.chars().enumerate() {
        if c.is_uppercase() {
            if i > 0 {
                result.push('_');
            }
            result.push(c.to_lowercase().next().unwrap());
        } else {
            result.push(c);
        }
    }
    result
}

/// Entry point for the derive macro expansion.
pub(crate) fn expand(input: DeriveInput) -> syn::Result<TokenStream> {
    let struct_attrs = StructAttrs::parse(&input)?;
    let ident = &input.ident;

    // Only works on structs with named fields
    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            syn::Fields::Named(fields) => &fields.named,
            _ => {
                return Err(syn::Error::new(
                    data.fields.span(),
                    "ParameterSet can only be derived for structs with named fields",
                ))
            }
        },
        syn::Data::Enum(e) => {
            return Err(syn::Error::new(
                e.enum_token.span(),
                "ParameterSet cannot be derived for enums",
            ))
        }
        syn::Data::Union(u) => {
            return Err(syn::Error::new(
                u.union_token.span(),
                "ParameterSet cannot be derived for unions",
            ))
        }
    };

    // Classify each field and generate its initializer
    let field_initializers: Vec<TokenStream> = fields
        .iter()
        .map(|field| {
            let info = FieldInfo::from_field(field)?;
            Ok(codegen::generate_field(&info))
        })
        .collect::<syn::Result<Vec<_>>>()?;

    // Determine the default namespace
    let namespace = match &struct_attrs.namespace {
        Some(ns) => ns.clone(),
        None => to_snake_case(&ident.to_string()),
    };

    Ok(quote! {
        impl rclrs::ParameterSet for #ident {
            fn default_namespace() -> &'static str {
                #namespace
            }

            fn declare(
                node: &rclrs::NodeState,
                prefix: &str,
            ) -> ::core::result::Result<Self, rclrs::DeclarationError> {
                ::core::result::Result::Ok(Self {
                    #(#field_initializers,)*
                })
            }
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_snake_case() {
        assert_eq!(to_snake_case("DriveConfig"), "drive_config");
        assert_eq!(to_snake_case("Robot"), "robot");
        assert_eq!(to_snake_case("SensorConfig"), "sensor_config");
        assert_eq!(to_snake_case("A"), "a");
        assert_eq!(to_snake_case("AB"), "a_b");
    }
}
