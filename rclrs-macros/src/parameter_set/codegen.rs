//! Code generation for individual fields in a `ParameterSet` struct.

use proc_macro2::TokenStream;
use quote::quote;

use super::field_attrs::{FieldAttrs, ParameterType, RangeAttr};
use super::field_info::FieldInfo;

/// Generates the field initializer expression for one struct field.
pub(crate) fn generate_field(field: &FieldInfo) -> TokenStream {
    match field {
        FieldInfo::Leaf {
            ident,
            attrs,
            param_type,
        } => {
            let name_str = ident.to_string();
            let builder_chain = generate_builder_chain(attrs);
            let terminal_call = generate_terminal(*param_type);
            quote! {
                #ident: node
                    .declare_parameter(
                        rclrs::__private::param_name(prefix, #name_str)
                    )
                    #builder_chain
                    #terminal_call?
            }
        }
        FieldInfo::Nested { ident, flatten } => generate_nested(ident, *flatten),
    }
}

/// Generates the chained builder method calls from field attributes.
fn generate_builder_chain(attrs: &FieldAttrs) -> TokenStream {
    let mut calls = TokenStream::new();

    if let Some(default) = &attrs.default {
        calls.extend(quote! { .default(#default) });
    }
    if let Some(description) = &attrs.description {
        calls.extend(quote! { .description(#description) });
    }
    if let Some(constraints) = &attrs.constraints {
        calls.extend(quote! { .constraints(#constraints) });
    }
    if let Some(range) = &attrs.range {
        let range_expr = generate_range(range);
        calls.extend(quote! { .range(#range_expr) });
    }
    if attrs.ignore_override {
        calls.extend(quote! { .ignore_override() });
    }
    if attrs.discard_mismatching_prior_value {
        calls.extend(quote! { .discard_mismatching_prior_value() });
    }
    if let Some(discriminate) = &attrs.discriminate {
        calls.extend(quote! { .discriminate(#discriminate) });
    }

    calls
}

/// Generates the terminal builder call (`.mandatory()`, `.optional()`, `.read_only()`).
fn generate_terminal(param_type: ParameterType) -> TokenStream {
    match param_type {
        ParameterType::Mandatory => quote! { .mandatory() },
        ParameterType::Optional => quote! { .optional() },
        ParameterType::ReadOnly => quote! { .read_only() },
    }
}

/// Generates a `ParameterRange { ... }` expression from parsed range attributes.
fn generate_range(range: &RangeAttr) -> TokenStream {
    let lower = match &range.lower {
        Some(expr) => quote! { Some(#expr) },
        None => quote! { None },
    };
    let upper = match &range.upper {
        Some(expr) => quote! { Some(#expr) },
        None => quote! { None },
    };
    let step = match &range.step {
        Some(expr) => quote! { Some(#expr) },
        None => quote! { None },
    };
    quote! {
        rclrs::ParameterRange {
            lower: #lower,
            upper: #upper,
            step: #step,
        }
    }
}

/// Generates the field initializer for a nested `ParameterSet` field.
fn generate_nested(ident: &syn::Ident, flatten: bool) -> TokenStream {
    let name_str = ident.to_string();
    if flatten {
        quote! {
            #ident: rclrs::ParameterSet::declare(node, prefix)?
        }
    } else {
        quote! {
            #ident: rclrs::ParameterSet::declare(
                node,
                &rclrs::__private::param_name(prefix, #name_str),
            )?
        }
    }
}
