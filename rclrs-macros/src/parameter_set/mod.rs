//! Implementation of `#[derive(ParameterSet)]`.
//!
//! Most fields emit the same shape of code:
//!
//! ```ignore
//! field: <FieldType as DeclareField<Mode>>::declare(node, &name, spec)?
//! ```
//!
//! so the macro does not need to know what kind of parameter such a field is. Whether it is a
//! single parameter, an optional one, or a whole nested set of parameters is decided by which
//! `DeclareField` implementation applies to its type, which is what allows a nested set to be
//! written without any annotation and a user-defined type to be used as a parameter without the
//! macro knowing about it.
//!
//! A field with `#[param(convert = ...)]` is the exception, and calls `declare_converted` and
//! its siblings instead. A conversion is a value, so no implementation can be selected by one,
//! which means the macro has to name the handle type itself. That is the cost of letting a field
//! be a type whose crate could never implement a trait from rclrs.
//!
//! The macro does inspect field types, but only to produce better diagnostics. See
//! [`known_types`].

mod attrs;
mod codegen;
mod known_types;

use proc_macro2::TokenStream;
use syn::{spanned::Spanned, Data, DeriveInput, Fields, Ident};

use crate::errors::Errors;
use attrs::{FieldAttrs, SetAttrs};
use known_types::{shape_of, TypeShape};

/// One field of the struct, with everything needed to generate its declaration.
pub(crate) struct Field<'a> {
    pub ident: &'a Ident,
    pub ty: &'a syn::Type,
    /// The parameter name, which is the field name unless `rename` says otherwise.
    pub name: String,
    pub attrs: FieldAttrs,
}

pub(crate) fn expand(input: &DeriveInput) -> syn::Result<TokenStream> {
    let mut errors = Errors::default();

    let set_attrs = SetAttrs::parse(&input.attrs, &mut errors);

    if !input.generics.params.is_empty() {
        errors.at(
            &input.generics,
            "`ParameterSet` cannot be derived for a generic struct: the parameters to declare \
             have to be known when the struct is defined",
        );
    }

    // Only a struct with named fields can describe a set of parameters, since each field name is
    // a parameter name.
    let named_fields = match &input.data {
        Data::Struct(data) => match &data.fields {
            Fields::Named(fields) => Some(&fields.named),
            Fields::Unnamed(_) => {
                errors.at(
                    &data.fields,
                    "`ParameterSet` requires named fields, because each field's name is the name \
                     of the parameter it declares",
                );
                None
            }
            Fields::Unit => {
                errors.at(
                    &input.ident,
                    "`ParameterSet` requires a struct with at least one field",
                );
                None
            }
        },
        Data::Enum(data) => {
            errors.at(
                data.enum_token,
                "`ParameterSet` cannot yet be derived for an enum",
            );
            None
        }
        Data::Union(data) => {
            errors.at(
                data.union_token,
                "`ParameterSet` cannot be derived for a union",
            );
            None
        }
    };

    // Nothing further can be said about a struct whose shape is wrong, so report what is known.
    let Some(named_fields) = named_fields else {
        return Err(errors.into_result().expect_err("an error was recorded"));
    };

    // Parse each field's attributes, check them against its type, and settle the name of the
    // parameter it declares. Checking every field before giving up reports all the mistakes at
    // once rather than one per compile.
    let mut fields = Vec::new();
    for field in named_fields {
        let ident = field
            .ident
            .as_ref()
            .expect("fields of a named struct have idents");

        let attrs = FieldAttrs::parse(&field.attrs, &mut errors);

        check_field(field, &attrs, &mut errors);

        let name = match &attrs.rename {
            Some(rename) => rename.value(),
            None => ident.to_string(),
        };

        fields.push(Field {
            ident,
            ty: &field.ty,
            name,
            attrs,
        });
    }

    // Two fields declaring one parameter name would have the second declaration fail at runtime,
    // which is a poor way to find out about a typo in a `rename`.
    if let Some(duplicate) = duplicate_parameter_name(&fields) {
        errors.at(
            duplicate.ident,
            format!(
                "another field of this set already declares a parameter called `{}`",
                duplicate.name
            ),
        );
    }

    // Generating code from a struct that was rejected only produces a second round of errors
    // about the code that was generated from it. Return the errors already collected instead.
    errors.into_result()?;

    Ok(codegen::generate(input, &set_attrs, &fields))
}

/// The `T` of an `Option<T>`, by syntax alone.
fn option_inner(ty: &syn::Type) -> Option<&syn::Type> {
    let syn::Type::Path(path) = ty else {
        return None;
    };
    let segment = path.path.segments.last()?;
    if segment.ident != "Option" {
        return None;
    }
    let syn::PathArguments::AngleBracketed(args) = &segment.arguments else {
        return None;
    };
    match args.args.first()? {
        syn::GenericArgument::Type(inner) => Some(inner),
        _ => None,
    }
}

/// Reports the mistakes the macro can recognise from the field's type and attributes.
fn check_field(field: &syn::Field, attrs: &FieldAttrs, errors: &mut Errors) {
    let shape = shape_of(&field.ty);

    // Check the attributes on a skipped field. A skipped field is not a parameter, so any other
    // attribute on it is meaningless.
    if attrs.skip.is_some() {
        if let Some(conflict) = attrs.conflicts_with_skip() {
            errors.push(syn::Error::new(
                conflict,
                "this option has no effect on a `skip`ped field, which is not a parameter at all",
            ));
        }
        // A skipped field is not a parameter, so nothing else about it matters.
        return;
    }

    // Report a type the macro recognises as a mistake, and stop. The message names the type to
    // use instead if possible. A `convert`ed field is exempt, because what its type may be is
    // decided by the conversion rather than by anything the macro knows about the type.
    if let (TypeShape::Rejected(message), None) = (&shape, &attrs.convert) {
        errors.at(&field.ty, message);
        return;
    }

    // Check the options that contradict `read_only`
    if let Some(read_only) = &attrs.read_only {
        // A read-only parameter always has a value, so there is no absence for `Option` to mean.
        if shape.is_optional() {
            errors.at(
                read_only,
                "`read_only` cannot be combined with `Option`: a read-only parameter always has \
                 a value, so there is nothing for the absence of one to mean. Remove `read_only`, \
                 or drop the `Option`",
            );
        }

        // A read-only parameter never changes after the initial value is set, so nothing would
        // ever call `on_change`.
        if let Some(on_change) = &attrs.on_change {
            errors.at(
                on_change,
                "`on_change` cannot be used on a `read_only` parameter, which never changes",
            );
        }
    }

    if let Some(range) = &attrs.range {
        // Reject a range where the macro can see that it cannot mean anything e.g. a recognised
        // type that is not numeric. An unrecognised type is let through, and so is a `convert`ed
        // one: its range is in the units the conversion stores, which the field's type does not
        // describe.
        if attrs.convert.is_none() && !shape.accepts_range() {
            errors.at(range, shape.range_rejection());
        }
        // Only where the macro takes the range apart does its shape matter here. Otherwise the
        // builder's conversions decide, and an exclusive integer range is as good as any.
        if attrs.convert.is_some() {
            if let Err(error) = attrs::range_bounds(range) {
                errors.push(error);
            }
        }
    // `step` describes the values inside a range, so on its own it has nothing to describe.
    } else if let Some(step) = &attrs.step {
        errors.at(
            step,
            "`step` describes the values within a range, so it needs a `range` to go with it",
        );
    }

    // A conversion says how one value is represented, which is not a thing a nested set has.
    if let (Some(convert), Some(_)) = (&attrs.convert, &attrs.flatten) {
        errors.at(
            convert,
            "`convert` gives the representation of a single value, so it cannot be combined with \
             `flatten`, which declares the fields of a nested set",
        );
    }

    // A flattened set contributes no segment to the parameter names, leaving nothing to rename.
    if let (Some(_), Some(rename)) = (&attrs.flatten, &attrs.rename) {
        errors.at(
            rename,
            "a `flatten`ed set has no name of its own, so it cannot be renamed",
        );
    }
}

/// Two fields declaring the same parameter name, which `rename` makes possible.
fn duplicate_parameter_name<'a>(fields: &'a [Field<'a>]) -> Option<&'a Field<'a>> {
    let mut seen = std::collections::HashSet::new();
    fields
        .iter()
        .filter(|field| field.attrs.skip.is_none() && field.attrs.flatten.is_none())
        .find(|field| !seen.insert(field.name.clone()))
}

impl Field<'_> {
    /// The type a converted field's handle and value are in, which is the field's own type unless
    /// it is an `Option`, where the parameter is of the type inside.
    ///
    /// Only the syntax is inspected, so `Option` has to be written as `Option<..>` for a field to
    /// be optional. That is already true of every other field, since the macro chooses which
    /// `DeclareField` implementation applies by the type as written.
    pub fn converted_value_ty(&self) -> &syn::Type {
        option_inner(self.ty).unwrap_or(self.ty)
    }

    /// Whether the field is an `Option`, as written.
    pub fn is_optional(&self) -> bool {
        option_inner(self.ty).is_some()
    }

    /// Whether the declaration gives the representation instead of the type.
    pub fn is_converted(&self) -> bool {
        self.attrs.convert.is_some()
    }

    /// Whether this field declares a parameter at all.
    pub fn is_declared(&self) -> bool {
        self.attrs.skip.is_none()
    }

    /// The mode marker this field is declared in.
    pub fn mode(&self) -> TokenStream {
        if self.attrs.read_only.is_some() {
            quote::quote!(::rclrs::ReadOnly)
        } else {
            quote::quote!(::rclrs::Writable)
        }
    }

    pub fn span(&self) -> proc_macro2::Span {
        self.ty.span()
    }
}

#[cfg(test)]
mod expand_tests;
