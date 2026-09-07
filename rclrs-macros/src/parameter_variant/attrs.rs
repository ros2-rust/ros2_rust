//! Parsing `#[parameter(...)]`.

use syn::{spanned::Spanned, Attribute, Ident, LitStr};

use crate::errors::Errors;

/// Type-level configuration from `#[parameter(...)]`.
#[derive(Default)]
pub(crate) struct TypeAttrs {
    /// Represent the type as whatever the single value it wraps is represented as.
    pub transparent: Option<Ident>,
    /// Represent the type as a string, via `FromStr` and `Display`.
    pub from_str: Option<Ident>,
    /// Naming convention for the stored value of each variant.
    pub rename_all: Option<RenameAll>,
}

impl TypeAttrs {
    pub fn parse(attrs: &[Attribute], errors: &mut Errors) -> Self {
        let mut parsed = Self::default();
        for attr in attrs {
            if !attr.path().is_ident("parameter") {
                continue;
            }
            let result = attr.parse_nested_meta(|meta| {
                let ident = || {
                    meta.path
                        .get_ident()
                        .cloned()
                        .unwrap_or_else(|| Ident::new("parameter", attr.path().span()))
                };
                if meta.path.is_ident("transparent") {
                    parsed.transparent = Some(ident());
                } else if meta.path.is_ident("from_str") {
                    parsed.from_str = Some(ident());
                } else if meta.path.is_ident("rename_all") {
                    let value: LitStr = meta.value()?.parse()?;
                    match RenameAll::parse(&value.value()) {
                        Some(convention) => parsed.rename_all = Some(convention),
                        None => {
                            return Err(syn::Error::new(
                                value.span(),
                                format!(
                                    "unknown naming convention {:?}; expected one of {}",
                                    value.value(),
                                    RenameAll::NAMES.join(", "),
                                ),
                            ))
                        }
                    }
                } else {
                    return Err(meta.error(
                        "unknown `parameter` option; expected one of `transparent`, `from_str`, \
                         `rename_all`",
                    ));
                }
                Ok(())
            });
            errors.handle(result);
        }
        parsed
    }
}

/// Variant-level configuration from `#[parameter(...)]`.
#[derive(Default)]
pub(crate) struct VariantAttrs {
    /// The exact string this variant is stored as.
    pub rename: Option<LitStr>,
}

impl VariantAttrs {
    pub fn parse(attrs: &[Attribute], errors: &mut Errors) -> Self {
        let mut parsed = Self::default();
        for attr in attrs {
            if !attr.path().is_ident("parameter") {
                continue;
            }
            let result = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("rename") {
                    parsed.rename = Some(meta.value()?.parse()?);
                    Ok(())
                } else {
                    Err(meta.error("unknown `parameter` option on a variant; expected `rename`"))
                }
            });
            errors.handle(result);
        }
        parsed
    }
}

/// How to turn a variant's name into the string it is stored as.
///
/// Shared with the `ParameterSet` derive, whose enum sets name their variants the same way.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum RenameAll {
    /// Exactly as written.
    None,
    SnakeCase,
    KebabCase,
    LowerCase,
    UpperCase,
    ScreamingSnakeCase,
}

impl RenameAll {
    pub const NAMES: &'static [&'static str] = &[
        "\"snake_case\"",
        "\"kebab-case\"",
        "\"lowercase\"",
        "\"UPPERCASE\"",
        "\"SCREAMING_SNAKE_CASE\"",
    ];

    pub fn parse(value: &str) -> Option<Self> {
        Some(match value {
            "snake_case" => Self::SnakeCase,
            "kebab-case" => Self::KebabCase,
            "lowercase" => Self::LowerCase,
            "UPPERCASE" => Self::UpperCase,
            "SCREAMING_SNAKE_CASE" => Self::ScreamingSnakeCase,
            _ => return None,
        })
    }

    /// Applies the convention to a variant name, which is assumed to be `PascalCase`.
    pub fn apply(self, name: &str) -> String {
        match self {
            Self::None => name.to_string(),
            Self::LowerCase => name.to_lowercase(),
            Self::UpperCase => name.to_uppercase(),
            Self::SnakeCase => split_words(name).join("_"),
            Self::KebabCase => split_words(name).join("-"),
            Self::ScreamingSnakeCase => split_words(name).join("_").to_uppercase(),
        }
    }
}

/// Splits a `PascalCase` name into lowercase words.
///
/// A run of capitals is one word, so `HTTPServer` becomes `http_server` rather than
/// `h_t_t_p_server`.
fn split_words(name: &str) -> Vec<String> {
    let chars: Vec<char> = name.chars().collect();
    let mut words = Vec::new();
    let mut current = String::new();
    for (index, &c) in chars.iter().enumerate() {
        let starts_word = c.is_uppercase()
            && index > 0
            && (
                // A lowercase letter or digit before a capital ends the previous word.
                !chars[index - 1].is_uppercase()
                    // The last capital of a run belongs to the next word: `HTTPServer`.
                    || chars.get(index + 1).is_some_and(|next| next.is_lowercase())
            );
        if starts_word && !current.is_empty() {
            words.push(std::mem::take(&mut current));
        }
        current.extend(c.to_lowercase());
    }
    if !current.is_empty() {
        words.push(current);
    }
    words
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_naming_conventions() {
        assert_eq!(RenameAll::None.apply("VelocityMode"), "VelocityMode");
        assert_eq!(RenameAll::SnakeCase.apply("VelocityMode"), "velocity_mode");
        assert_eq!(RenameAll::KebabCase.apply("VelocityMode"), "velocity-mode");
        assert_eq!(RenameAll::LowerCase.apply("VelocityMode"), "velocitymode");
        assert_eq!(RenameAll::UpperCase.apply("VelocityMode"), "VELOCITYMODE");
        assert_eq!(
            RenameAll::ScreamingSnakeCase.apply("VelocityMode"),
            "VELOCITY_MODE"
        );
    }

    #[test]
    fn test_runs_of_capitals_are_one_word() {
        assert_eq!(RenameAll::SnakeCase.apply("HTTPServer"), "http_server");
        assert_eq!(RenameAll::SnakeCase.apply("PID"), "pid");
        assert_eq!(
            RenameAll::KebabCase.apply("UseTCPNoDelay"),
            "use-tcp-no-delay"
        );
        assert_eq!(RenameAll::SnakeCase.apply("Velocity"), "velocity");
    }
}
