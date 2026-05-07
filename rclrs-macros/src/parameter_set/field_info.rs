//! Field classification: determines if a field is a leaf parameter or a nested
//! `ParameterSet` struct, and validates that attributes are used correctly.

use syn::spanned::Spanned;

use super::field_attrs::{FieldAttrs, ParameterType};

/// A classified struct field.
pub(crate) enum FieldInfo<'a> {
    /// A leaf parameter field — generates builder calls.
    Leaf {
        ident: &'a syn::Ident,
        attrs: FieldAttrs,
        param_type: ParameterType,
    },
    /// A nested `ParameterSet` field — generates a recursive `declare` call.
    Nested {
        ident: &'a syn::Ident,
        flatten: bool,
    },
}

impl<'a> FieldInfo<'a> {
    /// Classifies a field based on its type and attributes.
    pub fn from_field(field: &'a syn::Field) -> syn::Result<Self> {
        let ident = field
            .ident
            .as_ref()
            .ok_or_else(|| syn::Error::new(field.span(), "ParameterSet requires named fields"))?;

        let attrs = FieldAttrs::parse(field)?;
        let inferred = infer_param_type_from_type(&field.ty);

        // Validate contradictions between explicit attribute and inferred type
        if let (Some(explicit), Some(inferred)) = (attrs.param_type, inferred) {
            if explicit != inferred {
                let explicit_name = param_type_name(explicit);
                let inferred_name = param_type_name(inferred);
                return Err(syn::Error::new(
                    field.ty.span(),
                    format!(
                        "field type implies '{inferred_name}' but attribute specifies \
                         '{explicit_name}'"
                    ),
                ));
            }
        }

        // Determine if leaf or nested
        let param_type = attrs.param_type.or(inferred);
        match param_type {
            Some(param_type) => {
                // It's a leaf — validate no nested-only attrs
                if attrs.flatten {
                    return Err(syn::Error::new(
                        ident.span(),
                        "'flatten' can only be used on nested ParameterSet fields, \
                         not parameter fields",
                    ));
                }
                Ok(FieldInfo::Leaf {
                    ident,
                    attrs,
                    param_type,
                })
            }
            None => {
                // It's nested — validate no leaf-only attrs
                if attrs.has_leaf_attrs() {
                    return Err(syn::Error::new(
                        ident.span(),
                        "parameter attributes (default, description, range, etc.) \
                         can only be used on parameter fields, not nested ParameterSet fields",
                    ));
                }
                Ok(FieldInfo::Nested {
                    ident,
                    flatten: attrs.flatten,
                })
            }
        }
    }
}

/// Tries to infer the parameter type from the field's type name.
///
/// Returns `Some(param_type)` if the outermost type is `MandatoryParameter`,
/// `OptionalParameter`, or `ReadOnlyParameter`. Returns `None` for any other
/// type (assumed to be a nested `ParameterSet`).
fn infer_param_type_from_type(ty: &syn::Type) -> Option<ParameterType> {
    let syn::Type::Path(type_path) = ty else {
        return None;
    };
    let segment = type_path.path.segments.last()?;
    match segment.ident.to_string().as_str() {
        "MandatoryParameter" => Some(ParameterType::Mandatory),
        "OptionalParameter" => Some(ParameterType::Optional),
        "ReadOnlyParameter" => Some(ParameterType::ReadOnly),
        _ => None,
    }
}

/// Returns the attribute name for a parameter type (for error messages).
fn param_type_name(t: ParameterType) -> &'static str {
    match t {
        ParameterType::Mandatory => "mandatory",
        ParameterType::Optional => "optional",
        ParameterType::ReadOnly => "read_only",
    }
}
