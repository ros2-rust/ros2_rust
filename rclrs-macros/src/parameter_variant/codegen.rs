//! Emitting the conversions and the `ParameterVariant` implementation.

use proc_macro2::TokenStream;
use quote::quote;
use syn::DeriveInput;

use super::Strategy;

pub(crate) fn generate(input: &DeriveInput, strategy: &Strategy) -> TokenStream {
    let ident = &input.ident;
    let body = match strategy {
        Strategy::Choice { variants } => choice(input, variants),
        Strategy::Transparent { inner, accessor } => transparent(input, inner, accessor),
        Strategy::FromStr => from_str(input),
    };

    quote! {
        #body

        // The trait implementations that let this type be a field of a `ParameterSet`.
        ::rclrs::declare_parameter_field!(#ident);
    }
}

/// An enum of plain variants, stored as a string.
fn choice(input: &DeriveInput, variants: &[super::Choice]) -> TokenStream {
    let ident = &input.ident;
    let type_name = ident.to_string();

    let to_str = variants.iter().map(|choice| {
        let variant = choice.ident;
        let name = &choice.name;
        quote!(#ident::#variant => #name)
    });
    let from_str = variants.iter().map(|choice| {
        let variant = choice.ident;
        let name = &choice.name;
        quote!(#name => ::core::result::Result::Ok(#ident::#variant))
    });

    // Both the rejection message and the descriptor's constraints list the valid values, so an
    // operator setting the parameter over the parameter services is told what they may be.
    let valid_values = variants
        .iter()
        .map(|choice| choice.name.clone())
        .collect::<Vec<_>>()
        .join(", ");
    let constraints = format!("one of: {valid_values}");
    let unknown = format!("unknown {type_name} '{{other}}', expected one of: {valid_values}");

    quote! {
        impl ::core::convert::From<#ident> for ::rclrs::ParameterValue {
            fn from(value: #ident) -> Self {
                ::rclrs::ParameterValue::String(
                    ::core::convert::Into::into(match value { #(#to_str,)* }),
                )
            }
        }

        impl ::core::convert::TryFrom<::rclrs::ParameterValue> for #ident {
            type Error = ::rclrs::ParameterValueError;

            fn try_from(
                value: ::rclrs::ParameterValue,
            ) -> ::core::result::Result<Self, Self::Error> {
                match value {
                    ::rclrs::ParameterValue::String(text) => match ::core::convert::AsRef::<str>::as_ref(&text) {
                        #(#from_str,)*
                        other => ::core::result::Result::Err(
                            ::rclrs::ParameterValueError::Invalid(::std::format!(#unknown)),
                        ),
                    },
                    _ => ::core::result::Result::Err(::rclrs::ParameterValueError::TypeMismatch),
                }
            }
        }

        impl ::rclrs::ParameterVariant for #ident {
            type Range = ();

            fn kind() -> ::rclrs::ParameterKind {
                ::rclrs::ParameterKind::String
            }

            fn type_constraints() -> ::core::option::Option<::std::sync::Arc<str>> {
                ::core::option::Option::Some(::core::convert::Into::into(#constraints))
            }
        }
    }
}

/// A newtype, stored as whatever the value inside it is stored as.
fn transparent(input: &DeriveInput, inner: &syn::Type, accessor: &TokenStream) -> TokenStream {
    let ident = &input.ident;
    // Rebuilding the wrapper needs the field's shape: `Self(v)` or `Self { name: v }`.
    let rebuild = match &input.data {
        syn::Data::Struct(data) if matches!(data.fields, syn::Fields::Named(_)) => {
            quote!(|value| #ident { #accessor: value })
        }
        _ => quote!(#ident),
    };

    quote! {
        impl ::core::convert::From<#ident> for ::rclrs::ParameterValue {
            fn from(value: #ident) -> Self {
                ::core::convert::Into::into(value.#accessor)
            }
        }

        impl ::core::convert::TryFrom<::rclrs::ParameterValue> for #ident {
            // Whatever the inner type reports, so nothing is lost by wrapping it.
            type Error = <#inner as ::core::convert::TryFrom<::rclrs::ParameterValue>>::Error;

            fn try_from(
                value: ::rclrs::ParameterValue,
            ) -> ::core::result::Result<Self, Self::Error> {
                ::core::result::Result::map(
                    <#inner as ::core::convert::TryFrom<::rclrs::ParameterValue>>::try_from(value),
                    #rebuild,
                )
            }
        }

        impl ::rclrs::ParameterVariant for #ident {
            // Ranges are expressed in the units of the wrapped type.
            type Range = <#inner as ::rclrs::ParameterVariant>::Range;

            fn kind() -> ::rclrs::ParameterKind {
                <#inner as ::rclrs::ParameterVariant>::kind()
            }

            fn type_constraints() -> ::core::option::Option<::std::sync::Arc<str>> {
                <#inner as ::rclrs::ParameterVariant>::type_constraints()
            }
        }
    }
}

/// A type with a `FromStr`, stored as a string.
fn from_str(input: &DeriveInput) -> TokenStream {
    let ident = &input.ident;

    quote! {
        impl ::core::convert::From<#ident> for ::rclrs::ParameterValue {
            fn from(value: #ident) -> Self {
                ::rclrs::ParameterValue::String(
                    ::core::convert::Into::into(::std::string::ToString::to_string(&value)),
                )
            }
        }

        impl ::core::convert::TryFrom<::rclrs::ParameterValue> for #ident {
            type Error = ::rclrs::ParameterValueError;

            fn try_from(
                value: ::rclrs::ParameterValue,
            ) -> ::core::result::Result<Self, Self::Error> {
                match value {
                    ::rclrs::ParameterValue::String(text) => {
                        <#ident as ::core::str::FromStr>::from_str(
                            ::core::convert::AsRef::<str>::as_ref(&text),
                        )
                        .map_err(|err| {
                            ::rclrs::ParameterValueError::Invalid(::std::format!("{}", err))
                        })
                    }
                    _ => ::core::result::Result::Err(::rclrs::ParameterValueError::TypeMismatch),
                }
            }
        }

        impl ::rclrs::ParameterVariant for #ident {
            type Range = ();

            fn kind() -> ::rclrs::ParameterKind {
                ::rclrs::ParameterKind::String
            }
        }
    }
}
