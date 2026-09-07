//! Recognising field types, so that mistakes can be reported by the macro rather than by trait
//! resolution.
//!
//! Which types may be parameters is decided by the `DeclareField` implementations in rclrs, not
//! here. A type this module does not recognise is passed through and works if an implementation
//! exists for it, which is what lets nested sets, type aliases and user-defined types be used
//! without the macro knowing anything about them.
//!
//! What this module adds is diagnostics. The macro can see the field's type as written, so for
//! the types an application is likely to reach for by mistake it can say what to use instead, at
//! the span of the field, instead of leaving the user with an unsatisfied trait bound.

use syn::{GenericArgument, PathArguments, Type};

/// What the macro was able to work out about a field's type.
pub(crate) enum TypeShape {
    /// A recognised parameter type. `numeric` records whether a range applies to it.
    Leaf { numeric: bool },
    /// `Option<T>` where `T` is a recognised parameter type.
    OptionalLeaf { numeric: bool },
    /// A recognised sequence parameter type, which a range cannot reach the elements of.
    Array,
    /// `Option<T>` where `T` is a recognised sequence parameter type.
    OptionalArray,
    /// Not recognised: a nested parameter set, a user-defined parameter type, a type alias, or
    /// something that is simply not a parameter type at all. Left to trait resolution.
    Unrecognised,
    /// A recognised mistake, with what to say about it.
    Rejected(String),
}

impl TypeShape {
    /// Whether a `range` may be given for a field of this type. Unrecognised types are allowed,
    /// since the macro cannot know better. A wrong guess becomes a type error on the range.
    pub fn accepts_range(&self) -> bool {
        match self {
            TypeShape::Leaf { numeric } | TypeShape::OptionalLeaf { numeric } => *numeric,
            TypeShape::Array | TypeShape::OptionalArray => false,
            TypeShape::Unrecognised | TypeShape::Rejected(_) => true,
        }
    }

    /// Why a range cannot be given for a field of this type.
    pub fn range_rejection(&self) -> &'static str {
        if matches!(self, TypeShape::Array | TypeShape::OptionalArray) {
            "a ROS 2 range constrains a single value, so it cannot be given for an array. The \
             elements are still checked against the range of their own type"
        } else {
            "a range applies only to numeric parameters, and this field's type is not one"
        }
    }

    /// Whether the field is an `Option`, as far as the macro can tell. Used to reject
    /// combinations such as `read_only` on an optional parameter, whose message is much better
    /// coming from here than from a missing trait implementation.
    pub fn is_optional(&self) -> bool {
        matches!(
            self,
            TypeShape::OptionalLeaf { .. } | TypeShape::OptionalArray
        )
    }
}

/// Renders a type as a comparable key, ignoring the path it was reached by, so that
/// `std::path::PathBuf` and `PathBuf` compare equal.
///
/// Returns `None` for types whose shape this module does not model, such as references,
/// tuples and slices, which are handled separately by their caller.
fn type_key(ty: &Type) -> Option<String> {
    let Type::Path(path) = ty else {
        return None;
    };

    let segment = path.path.segments.last()?;
    let ident = segment.ident.to_string();

    match &segment.arguments {
        PathArguments::None => Some(ident),
        // Rendered recursively, so that a nested type such as `Option<Vec<String>>` becomes a
        // single key. An argument that is not a type, a lifetime for instance, gives up on the
        // whole key rather than dropping it and producing a key that means something else.
        PathArguments::AngleBracketed(args) => {
            let arguments = args
                .args
                .iter()
                .map(|arg| match arg {
                    GenericArgument::Type(ty) => type_key(ty),
                    _ => None,
                })
                .collect::<Option<Vec<_>>>()?;
            Some(format!("{ident}<{}>", arguments.join(",")))
        }
        // `Fn(A) -> B` sugar, which is never a parameter type.
        PathArguments::Parenthesized(_) => None,
    }
}

/// Parameter types a range applies to.
const NUMERIC_LEAVES: &[&str] = &["i64", "f64", "f32", "i8", "i16", "i32", "u8", "u16", "u32"];

/// Parameter types a range does not apply to.
/// The array forms are not listed here: they are recognised by [`is_array`], so that every
/// sequence is classified as one and gets the message about a range constraining a single value.
const OTHER_LEAVES: &[&str] = &["bool", "String", "PathBuf", "ParameterValue"];

/// The item types that `Vec<_>` is a parameter type for, which is every scalar parameter type
/// except the dynamically typed one.
const VEC_ITEMS: &[&str] = &[
    "bool", "i64", "f64", "f32", "i8", "i16", "i32", "u8", "u16", "u32", "String", "PathBuf",
];

/// Whether `key` names one of the sequence parameter types.
fn is_array(key: &str) -> bool {
    key.strip_prefix("Vec<")
        .and_then(|k| k.strip_suffix('>'))
        .is_some_and(|item| VEC_ITEMS.contains(&item))
}

/// The owned type to reach for in place of an unsized one.
///
/// Shared by [`rejection`] and by the reference branch of [`shape_of`], which see `str` and
/// `&str` respectively. Those are two different mistakes, one unsized and one borrowed, but the
/// type to use instead is the same, and naming it once keeps the two pieces of advice together.
fn owned_alternative(key: &str) -> Option<&'static str> {
    match key {
        "str" => Some("String"),
        "Path" => Some("PathBuf"),
        _ => None,
    }
}

/// Explains why a type cannot be a parameter and what to use instead, for the types most likely
/// to be reached for by mistake.
fn rejection(key: &str) -> Option<String> {
    let message = match key {
        // Every width an i64 cannot hold, whether the value runs past i64::MAX or past i64::MIN.
        "u64" | "usize" | "i128" | "u128" => format!(
            "`{key}` cannot be a ROS 2 parameter: a parameter value is stored as an i64, and \
             every way of handling a value outside its range would silently change it. Use \
             `i64`, or `u32` if the value must be unsigned"
        ),
        "isize" => "`isize` cannot be a ROS 2 parameter because its width is \
                    platform-dependent. Use `i64`"
            .to_string(),
        "Duration" => "`Duration` is not a ROS 2 parameter type, because the unit it would be \
                       stored in would be left implicit. Say which unit with \
                       `#[param(convert = ...)]`"
            .to_string(),
        "char" => "`char` cannot be a ROS 2 parameter. Use `String`".to_string(),
        "Path" | "str" => {
            let owned = owned_alternative(key).expect("both of these keys have one");
            format!("`{key}` is unsized and cannot be a ROS 2 parameter. Use `{owned}`")
        }
        _ => return None,
    };
    Some(message)
}

/// Works out what the macro can say about a field's type.
pub(crate) fn shape_of(ty: &Type) -> TypeShape {
    // A parameter field owns its value, so a reference is always wrong. When it is a reference to
    // one of the unsized types, the owned type to use instead is worth naming.
    if let Type::Reference(reference) = ty {
        let suggestion = type_key(&reference.elem)
            .as_deref()
            .and_then(owned_alternative)
            .map(|owned| format!(". Use `{owned}`"))
            .unwrap_or_default();
        return TypeShape::Rejected(format!(
            "a parameter field owns its value, so it cannot be a reference{suggestion}"
        ));
    }

    // A type whose shape this module does not model, such as a tuple or a slice. Trait resolution
    // has the final say on it, exactly as it does for a nested set or a user-defined type.
    let Some(key) = type_key(ty) else {
        return TypeShape::Unrecognised;
    };

    // A type that is recognisably the wrong choice, reported with the right one named.
    if let Some(message) = rejection(&key) {
        return TypeShape::Rejected(message);
    }

    // The parameter types the macro knows by name. Which list a type is in decides whether a
    // `range` may be given for it.
    if NUMERIC_LEAVES.contains(&key.as_str()) {
        return TypeShape::Leaf { numeric: true };
    }

    if OTHER_LEAVES.contains(&key.as_str()) {
        return TypeShape::Leaf { numeric: false };
    }

    if is_array(&key) {
        return TypeShape::Array;
    }

    // `Vec<T>` is a parameter type only for the item types that are parameter types themselves.
    if let Some(item) = key.strip_prefix("Vec<").and_then(|k| k.strip_suffix('>')) {
        if !VEC_ITEMS.contains(&item) {
            return TypeShape::Rejected(format!(
                "`{key}` cannot be a ROS 2 parameter. A sequence is a parameter only when its \
                 items are, and a sequence of parameter sets has no ROS 2 representation. \
                 Supported item types: {}",
                VEC_ITEMS
                    .iter()
                    .map(|i| format!("`{i}`"))
                    .collect::<Vec<_>>()
                    .join(", ")
            ));
        }
    }

    // A map is a recognised mistake rather than an unrecognised type, since no `DeclareField`
    // implementation could ever make one work.
    if key.starts_with("HashMap<") || key.starts_with("BTreeMap<") {
        return TypeShape::Rejected(format!(
            "`{key}` cannot be a ROS 2 parameter: ROS 2 has no map parameter type"
        ));
    }

    // `Option<T>` is an optional parameter when `T` is a parameter type. `Option` of anything
    // the macro does not recognise is left alone: it may be an optional parameter of a
    // user-defined type, which is perfectly valid.
    if let Some(inner) = key
        .strip_prefix("Option<")
        .and_then(|k| k.strip_suffix('>'))
    {
        if inner.starts_with("Option<") {
            return TypeShape::Rejected(
                "a nested `Option` is not a parameter type: an optional parameter is either \
                 set or unset, so there is no second level of absence to represent"
                    .to_string(),
            );
        }
        if let Some(message) = rejection(inner) {
            return TypeShape::Rejected(message);
        }
        if NUMERIC_LEAVES.contains(&inner) {
            return TypeShape::OptionalLeaf { numeric: true };
        }
        if OTHER_LEAVES.contains(&inner) {
            return TypeShape::OptionalLeaf { numeric: false };
        }
        if is_array(inner) {
            return TypeShape::OptionalArray;
        }
    }

    // Everything else is left alone. A nested parameter set, a type alias and a user-defined
    // parameter type are all indistinguishable from a mistake here, and only one of those should
    // produce an error, so the decision belongs to trait resolution.
    TypeShape::Unrecognised
}
