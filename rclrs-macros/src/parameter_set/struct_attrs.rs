//! Parsing of struct-level `#[parameters(...)]` attributes.

use syn::{DeriveInput, LitStr};

/// Parsed struct-level configuration from `#[parameters(...)]`.
pub(crate) struct StructAttrs {
    /// The namespace for this parameter set.
    /// `None` means derive from struct name (snake_case).
    /// `Some("")` means flatten (struct name not used as namespace).
    /// `Some("custom")` means use this exact string.
    pub namespace: Option<String>,
}

impl StructAttrs {
    /// Parses `#[parameters(...)]` attributes from the derive input.
    pub fn parse(input: &DeriveInput) -> syn::Result<Self> {
        let mut namespace = None;

        for attr in &input.attrs {
            if !attr.path().is_ident("parameters") {
                continue;
            }
            attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("flatten") {
                    if namespace.is_some() {
                        return Err(meta.error("cannot combine 'flatten' with 'namespace'"));
                    }
                    namespace = Some(String::new());
                    Ok(())
                } else if meta.path.is_ident("namespace") {
                    if namespace.is_some() {
                        return Err(meta.error(
                            "cannot specify 'namespace' more than once, or combine with 'flatten'",
                        ));
                    }
                    let value: LitStr = meta.value()?.parse()?;
                    namespace = Some(value.value());
                    Ok(())
                } else {
                    Err(meta.error(format!(
                        "unknown parameters attribute: '{}'",
                        meta.path.get_ident().map_or("?".into(), |i| i.to_string())
                    )))
                }
            })?;
        }

        Ok(Self { namespace })
    }
}
