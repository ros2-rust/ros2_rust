//! Procedural macros for [rclrs](https://docs.rs/rclrs).
//!
//! Everything here is re-exported from `rclrs`, so depend on that crate rather than on this one.

use proc_macro::TokenStream;

mod errors;
mod parameter_set;
mod parameter_variant;

/// Declares a struct's fields as a group of ROS 2 parameters.
///
/// See the `rclrs::ParameterSet` trait for the full description, and
/// `rclrs::NodeState::declare_parameters` for how to declare the result on a node.
///
/// # Struct attributes
///
/// * `#[parameters(namespace = "drive")]`: declare this set's parameters under `drive` when it
///   is declared at the top level. Defaults to the node's root, so that the struct mirrors the
///   shape of a parameter YAML file.
/// * `#[parameters(default = expr)]`: take the default value of every field from `expr`, which
///   must evaluate to `Self`, e.g. `Self::default()`. A field with its own
///   `#[param(default = ...)]` keeps that default.
/// * `#[parameters(handles = MyHandles)]`: name of the generated handles struct. Defaults to
///   the struct's own name with `Params` appended.
///
/// # Field attributes
///
/// * `#[param(default = expr)]`: the default value, in the field's own type. An array literal
///   is converted element by element, so `["a", "b"]` works for a `Vec<String>`.
/// * `#[param(description = "...")]`: descriptor description. Defaults to the doc comment.
/// * `#[param(constraints = "...")]`: descriptor constraints. Defaults to whatever the field's
///   type says about itself.
/// * `#[param(range = 0.0..=10.0, step = 0.5)]`: valid range, in the field's own units, or in
///   the units the conversion stores when `convert` is given. Any Rust range works.
/// * `#[param(convert = expr)]`: a `ParameterConversion<T>` saying how the field is represented,
///   for a type that does not implement `ParameterVariant` and cannot, such as one belonging to
///   another crate.
/// * `#[param(read_only)]`: declare as a read-only parameter.
/// * `#[param(validate = expr)]`: `fn(&T) -> Result<(), String>` run before a value is applied.
/// * `#[param(on_change = expr)]`: `fn(&T)`, or `fn(Option<&T>)` for an `Option` field, run
///   after a value has been applied.
/// * `#[param(discriminate = expr)]`: choose the initial value from those available.
/// * `#[param(ignore_override)]`, `#[param(discard_mismatching_prior_value)]`: as on the
///   parameter builder.
/// * `#[param(rename = "name")]`: the ROS 2 name of this parameter, if not the field name.
/// * `#[param(flatten)]`: for a nested set: declare its parameters directly under this set's
///   namespace, without one of its own.
/// * `#[param(skip)]`: not a parameter. Filled in with `Default::default()` when the set is
///   read back.
#[proc_macro_derive(ParameterSet, attributes(parameters, param))]
pub fn derive_parameter_set(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    match parameter_set::expand(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Represents a Rust type as a single ROS 2 parameter value.
///
/// See the `rclrs::ParameterVariant` trait for what this provides, and `rclrs::ParameterSet` for
/// declaring parameters that use it.
///
/// This is for a type you own, since a derive cannot be applied to one you do not. To use a type
/// from another crate, give the declaration a `rclrs::ParameterConversion` instead, with
/// `#[param(convert = ...)]` on a set field or `declare_parameter_with` on the builder. The
/// difference is where the representation is stated: a derive states it once for the type, a
/// conversion states it at each declaration.
///
/// The representation is chosen by the shape of the type and by `#[parameter(...)]`:
///
/// * An **enum whose variants carry no data** becomes a string holding the name of one variant.
///   `#[parameter(rename_all = "snake_case")]` sets the naming convention, one of `snake_case`,
///   `kebab-case`, `lowercase`, `UPPERCASE` or `SCREAMING_SNAKE_CASE`, and
///   `#[parameter(rename = "...")]` on a variant sets its stored value exactly. The valid values
///   appear in the parameter descriptor's constraints and in the message a rejected value gets.
/// * **`#[parameter(transparent)]`** on a type wrapping a single value gives it that value's
///   representation, including its range type. Useful for units: a `Meters(f64)` parameter
///   behaves exactly as an `f64` one, so its range is written in the units of the wrapped value,
///   `0.0..=10.0` rather than `Meters(0.0)..=Meters(10.0)`.
/// * **`#[parameter(from_str)]`** stores the type as a string, using its
///   [`FromStr`](std::str::FromStr) and [`Display`](std::fmt::Display) implementations. The
///   `FromStr` error is reported when a value is rejected, so it should say what was wrong.
///
/// The type also needs to be [`Clone`], which every parameter value must be.
///
/// A derived type works with the conversion-based API as well:
/// `rclrs::ParameterConversion::of_variant` assembles a conversion from what this derive emits,
/// so the kind and the constraints carry across and `ros2 param describe` reports them either
/// way.
#[proc_macro_derive(ParameterVariant, attributes(parameter))]
pub fn derive_parameter_variant(input: TokenStream) -> TokenStream {
    let input = syn::parse_macro_input!(input as syn::DeriveInput);
    match parameter_variant::expand(&input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}
