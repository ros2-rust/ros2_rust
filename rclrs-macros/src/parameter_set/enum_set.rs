//! Parameter sets that are enums: a group of parameters whose shape depends on a value.
//!
//! ROS 2 parameters are declared statically, so an enum set is declared in two steps. A
//! read-only string parameter, the *tag*, says which variant is in use, and the parameters of
//! that variant are then declared alongside it. The tag has to be read-only: changing the
//! variant at runtime would mean undeclaring one set of parameters and declaring another,
//! invalidating handles the caller may be holding.

use syn::{DataEnum, Fields, Ident, Variant as SynVariant};

use super::{attrs::FieldAttrs, check_field, duplicate_parameter_name, parameter_name, Field};
use crate::{errors::Errors, parameter_variant::attrs::RenameAll};

/// The default name of the parameter that says which variant is in use.
pub(crate) const DEFAULT_TAG: &str = "type";

/// One variant of an enum parameter set.
pub(crate) struct Variant<'a> {
    pub ident: &'a Ident,
    /// The string stored in the tag parameter for this variant.
    pub tag: String,
    pub shape: VariantShape<'a>,
}

/// What a variant declares, beyond the tag.
pub(crate) enum VariantShape<'a> {
    /// Named fields, declared under the set's own namespace.
    Fields(Vec<Field<'a>>),
    /// Another parameter set, declared under the set's own namespace.
    Delegate(&'a syn::Type),
    /// Nothing.
    Unit,
}

/// Reads the variants of an enum parameter set, reporting the shapes that cannot be one.
pub(crate) fn variants<'a>(
    data: &'a DataEnum,
    ident: &Ident,
    rename_all: Option<RenameAll>,
    tag: &str,
    errors: &mut Errors,
) -> Vec<Variant<'a>> {
    if data.variants.is_empty() {
        errors.at(
            &data.variants,
            "an enum with no variants describes no parameters",
        );
        return Vec::new();
    }

    if data
        .variants
        .iter()
        .all(|variant| matches!(variant.fields, Fields::Unit))
    {
        errors.at(
            ident,
            "no variant of this enum carries any parameters, so it is a single value rather than \
             a group of them. Derive `ParameterVariant` instead, which represents it as one \
             string parameter",
        );
        return Vec::new();
    }

    let mut variants = Vec::new();
    for variant in &data.variants {
        let attrs = FieldAttrs::parse(&variant.attrs, errors);
        check_variant_attrs(variant, &attrs, errors);

        let tag_value = match &attrs.rename {
            Some(rename) => rename.value(),
            None => rename_all
                .unwrap_or(RenameAll::None)
                .apply(&parameter_name(&variant.ident)),
        };

        let shape = match &variant.fields {
            Fields::Unit => VariantShape::Unit,
            Fields::Named(fields) => {
                let mut variant_fields = Vec::new();

                for field in &fields.named {
                    let field_ident = field
                        .ident
                        .as_ref()
                        .expect("fields of a named variant have idents");

                    let field_attrs = FieldAttrs::parse(&field.attrs, errors);

                    check_field(field, &field_attrs, errors);

                    let name = match &field_attrs.rename {
                        Some(rename) => rename.value(),
                        None => parameter_name(field_ident),
                    };

                    if name == tag {
                        errors.at(
                            field_ident,
                            format!(
                                "this field would declare a parameter called `{name}`, which is \
                                 already the name of the parameter that says which variant is in \
                                 use. Rename the field, or choose another tag with \
                                 `#[parameters(tag = \"...\")]`"
                            ),
                        );
                    }

                    variant_fields.push(Field {
                        ident: field_ident,
                        ty: &field.ty,
                        name,
                        attrs: field_attrs,
                    });
                }

                if let Some(duplicate) = duplicate_parameter_name(&variant_fields) {
                    errors.at(
                        duplicate.ident,
                        format!(
                            "another field of this variant already declares a parameter called \
                             `{}`",
                            duplicate.name
                        ),
                    );
                }
                VariantShape::Fields(variant_fields)
            }
            Fields::Unnamed(fields) => {
                let mut iter = fields.unnamed.iter();

                let (Some(field), None) = (iter.next(), iter.next()) else {
                    errors.at(
                        fields,
                        "a variant of a parameter set holds either named fields, each of which \
                         is a parameter, or a single parameter set to delegate to",
                    );
                    continue;
                };

                // The delegated set's parameters are declared under this set's namespace, since
                // the variant is already identified by the tag. A single *value* has nowhere to
                // go: it would have to be named after the namespace it sits in.
                if super::shape_of(&field.ty).is_definitely_leaf() {
                    errors.at(
                        &field.ty,
                        "a variant holding a single value has no name to declare it under, since \
                         the variant itself is identified by the tag parameter. Use a struct \
                         variant so the value has a field name",
                    );
                    continue;
                }

                VariantShape::Delegate(&field.ty)
            }
        };

        variants.push(Variant {
            ident: &variant.ident,
            tag: tag_value,
            shape,
        });
    }

    // Two variants stored under the same tag value could not be told apart.
    let mut seen = std::collections::HashMap::new();
    for variant in &variants {
        if let Some(previous) = seen.insert(variant.tag.clone(), variant.ident) {
            errors.at(
                variant.ident,
                format!(
                    "`{}` is already the tag value of variant `{previous}`",
                    variant.tag
                ),
            );
        }
    }

    variants
}

/// Only `rename` means anything on a variant, since the rest describe a single parameter.
fn check_variant_attrs(variant: &SynVariant, attrs: &FieldAttrs, errors: &mut Errors) {
    if let Some(span) = attrs.conflicts_with_rename() {
        errors.push(syn::Error::new(
            span,
            format!(
                "this option describes a single parameter, so it has no meaning on the `{}` \
                 variant. Put it on one of the variant's fields",
                variant.ident
            ),
        ));
    }
}
