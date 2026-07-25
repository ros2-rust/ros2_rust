//! Implementation of `#[derive(ParameterVariant)]`.
//!
//! Where `#[derive(ParameterSet)]` describes a *group* of parameters, this describes a single
//! parameter *value*: a Rust type that can be stored in one of the nine types ROS 2 has.
//!
//! Three strategies, one of which is chosen by the shape of the type and by
//! `#[parameter(...)]`:
//!
//! * an enum whose variants carry no data becomes a string, one spelling per variant,
//! * `#[parameter(transparent)]` on a newtype takes on the representation of the type inside it,
//! * `#[parameter(from_str)]` uses [`FromStr`](std::str::FromStr) and [`Display`](std::fmt::Display).
//!
//! All three emit `ParameterVariant`, so `ParameterConversion::of_variant` can assemble a
//! conversion from them and a derived type is usable through the conversion-based API too. The
//! derive is for a type you own. A conversion is what a type from another crate needs, since a
//! derive cannot be applied to one.

pub(crate) mod attrs;
mod codegen;

use syn::{Data, DeriveInput, Fields};

use crate::errors::Errors;
use attrs::{RenameAll, VariantAttrs};

/// How a type is represented as a ROS 2 parameter.
pub(crate) enum Strategy<'a> {
    /// A string holding the name of one variant.
    Choice { variants: Vec<Choice<'a>> },
    /// Whatever the single field inside this newtype is represented as.
    Transparent {
        inner: &'a syn::Type,
        /// The field to go through: an index for a tuple struct, a name otherwise.
        accessor: proc_macro2::TokenStream,
    },
    /// A string, parsed with `FromStr` and written with `Display`.
    FromStr,
}

/// One variant of a string-valued enum.
pub(crate) struct Choice<'a> {
    pub ident: &'a syn::Ident,
    /// The string this variant is stored as.
    pub name: String,
}

pub(crate) fn expand(input: &DeriveInput) -> syn::Result<proc_macro2::TokenStream> {
    let mut errors = Errors::default();
    let attrs = attrs::TypeAttrs::parse(&input.attrs, &mut errors);

    if !input.generics.params.is_empty() {
        errors.at(
            &input.generics,
            "`ParameterVariant` cannot be derived for a generic type: how it is represented as a \
             parameter has to be known when the type is defined",
        );
    }

    let strategy = strategy_for(input, &attrs, &mut errors);
    errors.into_result()?;
    let strategy = strategy.expect("a strategy was produced when no error was recorded");

    Ok(codegen::generate(input, &strategy))
}

/// Works out how the type should be represented, reporting the shapes that cannot be.
fn strategy_for<'a>(
    input: &'a DeriveInput,
    attrs: &attrs::TypeAttrs,
    errors: &mut Errors,
) -> Option<Strategy<'a>> {
    if let (Some(_), Some(from_str)) = (&attrs.transparent, &attrs.from_str) {
        errors.at(
            from_str,
            "`transparent` and `from_str` are two different representations, use one or the other",
        );
        return None;
    }

    if let Some(from_str) = &attrs.from_str {
        if attrs.rename_all.is_some() {
            errors.at(
                from_str,
                "`rename_all` names the variants of an enum, so it has no meaning together with \
                 `from_str`",
            );
        }
        return Some(Strategy::FromStr);
    }

    match &input.data {
        Data::Struct(data) => {
            if attrs.transparent.is_none() {
                errors.at(
                    &input.ident,
                    "a struct has no representation as a single parameter value on its own. Add \
                     `#[parameter(transparent)]` if it wraps a single value, or \
                     `#[parameter(from_str)]` if it has a `FromStr` implementation. If it is \
                     really a group of parameters, derive `ParameterSet` instead",
                );
                return None;
            }
            transparent_strategy(&data.fields, errors)
        }
        Data::Enum(data) => {
            if let Some(transparent) = &attrs.transparent {
                errors.at(
                    transparent,
                    "`transparent` applies to a type that wraps a single value, not to an enum",
                );
                return None;
            }
            choice_strategy(data, attrs.rename_all, errors)
        }
        Data::Union(data) => {
            errors.at(
                data.union_token,
                "`ParameterVariant` cannot be derived for a union",
            );
            None
        }
    }
}

/// A newtype takes on the representation of the one value inside it.
fn transparent_strategy<'a>(fields: &'a Fields, errors: &mut Errors) -> Option<Strategy<'a>> {
    let mut iter = fields.iter();
    let (Some(field), None) = (iter.next(), iter.next()) else {
        errors.at(
            fields,
            "`transparent` needs exactly one field, since the type is represented as whatever \
             that field is represented as",
        );
        return None;
    };
    let accessor = match &field.ident {
        Some(name) => quote::quote!(#name),
        None => quote::quote!(0),
    };
    Some(Strategy::Transparent {
        inner: &field.ty,
        accessor,
    })
}

/// An enum of plain variants becomes a string.
fn choice_strategy<'a>(
    data: &'a syn::DataEnum,
    rename_all: Option<RenameAll>,
    errors: &mut Errors,
) -> Option<Strategy<'a>> {
    if data.variants.is_empty() {
        errors.at(
            &data.variants,
            "an enum with no variants has no value to store",
        );
        return None;
    }

    let mut variants = Vec::new();
    for variant in &data.variants {
        if !matches!(variant.fields, Fields::Unit) {
            errors.at(
                &variant.fields,
                "a variant that carries data is a group of parameters rather than a single \
                 value, so it cannot be part of a `ParameterVariant`. A group of parameters that \
                 is one of several shapes is not supported yet",
            );
            continue;
        }
        let attrs = VariantAttrs::parse(&variant.attrs, errors);
        let name = match &attrs.rename {
            Some(rename) => rename.value(),
            None => rename_all
                .unwrap_or(RenameAll::None)
                .apply(&variant.ident.to_string()),
        };
        variants.push(Choice {
            ident: &variant.ident,
            name,
        });
    }

    // Two variants stored as the same string would make the conversion back ambiguous.
    let mut seen = std::collections::HashMap::new();
    for choice in &variants {
        if let Some(previous) = seen.insert(choice.name.clone(), choice.ident) {
            errors.at(
                choice.ident,
                format!(
                    "`{}` is already the stored value of variant `{previous}`",
                    choice.name
                ),
            );
        }
    }

    if !errors.is_empty() {
        return None;
    }
    Some(Strategy::Choice { variants })
}

#[cfg(test)]
#[path = "expand_tests.rs"]
mod expand_tests;
