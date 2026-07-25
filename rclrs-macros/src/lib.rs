//! Procedural macros for [rclrs](https://docs.rs/rclrs).
//!
//! Everything here is re-exported from `rclrs`, so depend on that crate rather than on this one.

use proc_macro::TokenStream;

mod parameter_set;

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
/// * `#[param(range = 0.0..=10.0, step = 0.5)]`: valid range, in the field's own units.
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
