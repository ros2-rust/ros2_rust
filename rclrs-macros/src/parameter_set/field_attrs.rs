//! Parsing of field-level `#[param(...)]` attributes.

use syn::{Expr, LitStr};

/// The kind of parameter: mandatory, optional, or read-only.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) enum ParameterType {
    Mandatory,
    Optional,
    ReadOnly,
}

/// Parsed range attribute: `range(lower = x, upper = y, step = z)`.
/// All fields are optional.
#[derive(Debug, Default)]
pub(crate) struct RangeAttr {
    pub lower: Option<Expr>,
    pub upper: Option<Expr>,
    pub step: Option<Expr>,
}

/// Parsed field-level configuration from `#[param(...)]`.
#[derive(Debug)]
pub(crate) struct FieldAttrs {
    /// Explicit parameter type (escape hatch for type aliases).
    pub param_type: Option<ParameterType>,
    /// Default value expression.
    pub default: Option<Expr>,
    /// Parameter description string.
    pub description: Option<LitStr>,
    /// Parameter constraints string.
    pub constraints: Option<LitStr>,
    /// Parameter range.
    pub range: Option<RangeAttr>,
    /// Whether to ignore parameter overrides from command line.
    pub ignore_override: bool,
    /// Whether to discard prior values with mismatching types.
    pub discard_mismatching_prior_value: bool,
    /// Path to a custom discriminator function.
    pub discriminate: Option<Expr>,
    /// Whether this nested field should be flattened (no extra namespace).
    pub flatten: bool,
}

impl FieldAttrs {
    /// Parses all `#[param(...)]` attributes on a field.
    pub fn parse(field: &syn::Field) -> syn::Result<Self> {
        let mut attrs = Self {
            param_type: None,
            default: None,
            description: None,
            constraints: None,
            range: None,
            ignore_override: false,
            discard_mismatching_prior_value: false,
            discriminate: None,
            flatten: false,
        };

        for attr in &field.attrs {
            if !attr.path().is_ident("param") {
                continue;
            }
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("mandatory") {
                    attrs.set_param_type(ParameterType::Mandatory, &meta)?;
                    Ok(())
                } else if meta.path.is_ident("optional") {
                    attrs.set_param_type(ParameterType::Optional, &meta)?;
                    Ok(())
                } else if meta.path.is_ident("read_only") {
                    attrs.set_param_type(ParameterType::ReadOnly, &meta)?;
                    Ok(())
                } else if meta.path.is_ident("default") {
                    attrs.default = Some(meta.value()?.parse()?);
                    Ok(())
                } else if meta.path.is_ident("description") {
                    attrs.description = Some(meta.value()?.parse()?);
                    Ok(())
                } else if meta.path.is_ident("constraints") {
                    attrs.constraints = Some(meta.value()?.parse()?);
                    Ok(())
                } else if meta.path.is_ident("range") {
                    attrs.range = Some(parse_range(&meta)?);
                    Ok(())
                } else if meta.path.is_ident("ignore_override") {
                    attrs.ignore_override = true;
                    Ok(())
                } else if meta.path.is_ident("discard_mismatching_prior_value") {
                    attrs.discard_mismatching_prior_value = true;
                    Ok(())
                } else if meta.path.is_ident("discriminate") {
                    attrs.discriminate = Some(meta.value()?.parse()?);
                    Ok(())
                } else if meta.path.is_ident("flatten") {
                    attrs.flatten = true;
                    Ok(())
                } else {
                    Err(meta.error(format!(
                        "unknown param attribute: '{}'",
                        meta.path.get_ident().map_or("?".into(), |i| i.to_string())
                    )))
                }
            })?;
        }

        Ok(attrs)
    }

    /// Sets the parameter type, erroring if one was already set.
    fn set_param_type(
        &mut self,
        param_type: ParameterType,
        meta: &syn::meta::ParseNestedMeta<'_>,
    ) -> syn::Result<()> {
        if self.param_type.is_some() {
            return Err(meta.error("only one of 'mandatory', 'optional', 'read_only' is allowed"));
        }
        self.param_type = Some(param_type);
        Ok(())
    }

    /// Returns true if any leaf-only attribute was set.
    pub fn has_leaf_attrs(&self) -> bool {
        self.default.is_some()
            || self.description.is_some()
            || self.constraints.is_some()
            || self.range.is_some()
            || self.ignore_override
            || self.discard_mismatching_prior_value
            || self.discriminate.is_some()
            || self.param_type.is_some()
    }
}

/// Parses `range(lower = x, upper = y, step = z)`.
fn parse_range(meta: &syn::meta::ParseNestedMeta<'_>) -> syn::Result<RangeAttr> {
    let mut range = RangeAttr::default();
    meta.parse_nested_meta(|nested| {
        if nested.path.is_ident("lower") {
            range.lower = Some(nested.value()?.parse()?);
            Ok(())
        } else if nested.path.is_ident("upper") {
            range.upper = Some(nested.value()?.parse()?);
            Ok(())
        } else if nested.path.is_ident("step") {
            range.step = Some(nested.value()?.parse()?);
            Ok(())
        } else {
            Err(nested.error("expected 'lower', 'upper', or 'step'"))
        }
    })?;
    Ok(range)
}
