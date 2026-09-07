//! Tests for what the derive macro accepts and, more importantly, what it says about what it
//! rejects.
//!
//! These call `expand` directly rather than compiling generated code, so the messages asserted
//! here are exactly the ones the macro produces, independently of how any particular rustc
//! version chooses to render a failed trait bound. The generated code itself is exercised by the
//! tests in `rclrs`.

use super::expand;
use syn::DeriveInput;

/// The messages the macro reports for `input`, in the order it reports them.
fn errors(input: &str) -> Vec<String> {
    let parsed: DeriveInput = syn::parse_str(input).expect("test input should parse");
    match expand(&parsed) {
        Ok(_) => Vec::new(),
        Err(error) => error.into_iter().map(|e| e.to_string()).collect(),
    }
}

/// Asserts that `input` is rejected with a single message containing each of `expected`.
#[track_caller]
fn rejected_with(input: &str, expected: &[&str]) {
    let errors = errors(input);
    assert_eq!(
        errors.len(),
        1,
        "expected exactly one error, got {errors:#?}"
    );
    for fragment in expected {
        assert!(
            errors[0].contains(fragment),
            "message should mention {fragment:?}, but was: {}",
            errors[0]
        );
    }
}

#[track_caller]
fn accepted(input: &str) {
    let errors = errors(input);
    assert!(errors.is_empty(), "should have been accepted: {errors:#?}");
}

#[test]
fn test_accepts_a_well_formed_set() {
    accepted(
        r#"
        struct DriveConfig {
            /// Maximum speed.
            #[param(default = 1.5, range = 0.0..=10.0)]
            max_speed: f64,
            #[param(default = ["a", "b"])]
            wheels: Vec<String>,
            #[param(convert = seconds())]
            watchdog: Option<Duration>,
            #[param(read_only)]
            device: PathBuf,
            limits: Limits,
            #[param(flatten)]
            extra: Extra,
            #[param(skip)]
            cache: Cache,
        }
        "#,
    );
}

#[test]
fn test_generated_code_names_the_handles_struct() {
    let parsed: DeriveInput = syn::parse_str("struct DriveConfig { speed: f64 }").unwrap();
    let generated = expand(&parsed).unwrap().to_string();
    assert!(
        generated.contains("struct DriveConfigParams"),
        "{generated}"
    );

    let parsed: DeriveInput =
        syn::parse_str("#[parameters(handles = DriveHandles)] struct DriveConfig { speed: f64 }")
            .unwrap();
    let generated = expand(&parsed).unwrap().to_string();
    assert!(generated.contains("struct DriveHandles"), "{generated}");
}

// -------------------------------------------------------------------------------------------
// Types that cannot be parameters
// -------------------------------------------------------------------------------------------

#[test]
fn test_rejects_integers_wider_than_a_parameter() {
    rejected_with(
        "struct C { count: u64 }",
        &["`u64` cannot be a ROS 2 parameter", "Use `i64`"],
    );
    rejected_with("struct C { count: usize }", &["`usize`", "`u32`"]);
    rejected_with("struct C { count: isize }", &["platform-dependent"]);
    rejected_with("struct C { count: i128 }", &["`i128`"]);
    rejected_with("struct C { letter: char }", &["`char`", "`String`"]);
}

#[test]
fn test_rejects_duration_because_the_unit_would_be_implicit() {
    rejected_with("struct C { timeout: Duration }", &["unit", "convert"]);
    rejected_with("struct C { timeout: std::time::Duration }", &["convert"]);
}

#[test]
fn test_rejects_references() {
    rejected_with(
        "struct C { name: &'static str }",
        &["reference", "`String`"],
    );
    rejected_with("struct C { path: &'static Path }", &["`PathBuf`"]);
}

#[test]
fn test_rejects_sequences_of_things_ros_has_no_array_type_for() {
    rejected_with(
        "struct C { sensors: Vec<SensorConfig> }",
        &["Vec<SensorConfig>", "no ROS 2 representation"],
    );
}

#[test]
fn test_rejects_maps() {
    rejected_with(
        "struct C { extra: HashMap<String, String> }",
        &["no map parameter type"],
    );
}

#[test]
fn test_rejects_nested_option() {
    rejected_with(
        "struct C { speed: Option<Option<f64>> }",
        &["nested `Option`"],
    );
}

/// A type the macro does not recognise is passed through: it may be a nested set, a user-defined
/// parameter type, or a type alias, and only trait resolution can tell.
#[test]
fn test_accepts_types_it_does_not_recognise() {
    accepted("struct C { limits: Limits }");
    accepted("struct C { mode: ControlMode }");
    accepted("struct C { mode: Option<ControlMode> }");
    accepted("struct C { speed: Speed }");
    // The path a type is reached by does not matter, only its name.
    accepted("struct C { limits: my_crate::config::Limits }");
    accepted("struct C { name: Arc<str> }");
}

// -------------------------------------------------------------------------------------------
// Attribute combinations
// -------------------------------------------------------------------------------------------

#[test]
fn test_rejects_read_only_optional() {
    rejected_with(
        "struct C { #[param(read_only)] speed: Option<f64> }",
        &["`read_only` cannot be combined with `Option`"],
    );
}

#[test]
fn test_rejects_on_change_on_a_read_only_parameter() {
    rejected_with(
        "struct C { #[param(read_only, on_change = f)] speed: f64 }",
        &["`on_change`", "read_only", "never changes"],
    );
}

// A read-only parameter is declared once and never set again, but its one value can still come
// from a parameter file or the command line, so it is worth checking.
#[test]
fn test_accepts_validate_on_a_read_only_parameter() {
    accepted("struct C { #[param(read_only, validate = f)] speed: f64 }");
}

/// Whether an unrecognised type accepts a range is decided by its `ParameterVariant::Range` and
/// not here, so the macro passes it through. A newtype over a number is the case that matters.
#[test]
fn test_accepts_a_range_on_an_unrecognised_type() {
    accepted("struct C { #[param(range = 0.0..=10.0)] depth: Meters }");
}

#[test]
fn test_rejects_a_range_on_a_non_numeric_parameter() {
    rejected_with(
        r#"struct C { #[param(range = 0..=1)] name: String }"#,
        &["range applies only to numeric parameters"],
    );
}

/// A ROS 2 range constrains one value, so it cannot be given for an array. The elements are still
/// held to the range of their own type, which is what makes `Vec<u16>` worth having.
/// A `convert`ed field's range is in the units the conversion stores, which the field's type says
/// nothing about, so the macro has no grounds to judge it.
#[test]
fn test_accepts_a_range_on_a_converted_field_of_any_type() {
    accepted(r#"struct C { #[param(convert = c(), range = 0..=10)] name: String }"#);
    accepted("struct C { #[param(convert = c(), range = 0..=10)] raw: Vec<u8> }");
}

/// Every sequence gets the message about a range constraining one value, not the one about the
/// type not being numeric. `Vec<i64>` and `Vec<u16>` are recognised by different routes and used
/// to disagree.
#[test]
fn test_rejects_a_range_on_an_array() {
    for item in ["u16", "i64", "f64", "bool", "String", "u8"] {
        rejected_with(
            &format!("struct C {{ #[param(range = 0..=100)] v: Vec<{item}> }}"),
            &["constrains a single value", "cannot be given for an array"],
        );
    }
    rejected_with(
        "struct C { #[param(range = 0..=100)] ports: Option<Vec<u16>> }",
        &["cannot be given for an array"],
    );
}

/// Every scalar parameter type has a `Vec` form.
#[test]
fn test_accepts_arrays_of_every_scalar_type() {
    for item in [
        "bool", "i64", "f64", "f32", "i8", "i16", "i32", "u8", "u16", "u32", "String", "PathBuf",
    ] {
        accepted(&format!("struct C {{ field: Vec<{item}> }}"));
        accepted(&format!("struct C {{ field: Option<Vec<{item}>> }}"));
    }
}

/// A range the macro hands over as written is the builder's business, so what a range may be is
/// decided in one place. An exclusive integer range is usable; an exclusive floating point one is
/// refused by the absent conversion rather than here.
#[test]
fn test_a_range_is_passed_through() {
    accepted("struct C { #[param(range = 0..10)] count: i64 }");
    accepted("struct C { #[param(range = 0.0..10.0)] speed: f64 }");
    accepted("struct C { #[param(range = ..)] speed: f64 }");
}

/// A step is added after the conversion, so it puts no restriction on how the range is written.
#[test]
fn test_a_range_with_a_step_is_passed_through() {
    accepted("struct C { #[param(range = 0..10, step = 2)] count: i64 }");
    accepted("struct C { #[param(range = 0..=10, step = 2)] count: i64 }");
}

/// A converted field's spec holds the erased range, which no Rust range converts into, so there
/// the macro still reads the ends off and still says what it can take.
#[test]
fn test_rejects_an_exclusive_range_on_a_converted_field() {
    rejected_with(
        "struct C { #[param(convert = seconds(), range = 0.0..10.0)] timeout: Duration }",
        &["inclusive", "`..=`"],
    );
}

#[test]
fn test_rejects_a_step_without_a_range() {
    rejected_with(
        "struct C { #[param(step = 5)] speed: i64 }",
        &["`step` describes the values within a range, so it needs a `range`"],
    );
}

/// `flatten` on a single parameter is left to `DeclareFlattened`, whose `on_unimplemented` names
/// the field, labels it "not a parameter set" and explains what `flatten` is for.
#[test]
fn test_accepts_flatten_on_a_single_parameter() {
    accepted("struct C { #[param(flatten)] speed: f64 }");
}

#[test]
fn test_rejects_renaming_a_flattened_set() {
    rejected_with(
        r#"struct C { #[param(flatten, rename = "x")] limits: Limits }"#,
        &["no name of its own"],
    );
}

#[test]
fn test_rejects_options_on_a_skipped_field() {
    rejected_with(
        "struct C { #[param(skip, default = 1.0)] cache: Cache }",
        &["no effect on a `skip`ped field"],
    );
}

#[test]
fn test_rejects_two_fields_declaring_the_same_parameter() {
    rejected_with(
        r#"
        struct C {
            #[param(rename = "speed")]
            max_speed: f64,
            speed: f64,
        }
        "#,
        &["already declares a parameter called `speed`"],
    );
}

/// The attributes people could intuitively reach for. Reporting an error that explains what to do
/// instead is friendlier than reporting them as unknown.
#[test]
fn test_rejects_intuitive_attributes() {
    rejected_with(
        "struct C { #[param(mandatory)] speed: f64 }",
        &["not needed", "`Option`"],
    );
    rejected_with(
        "struct C { #[param(nested)] limits: Limits }",
        &["not needed", "declared as one automatically"],
    );
}

#[test]
fn test_reports_unknown_options() {
    rejected_with(
        "struct C { #[param(colour = \"red\")] speed: f64 }",
        &["unknown `param` option `colour`"],
    );
    rejected_with(
        "#[parameters(prefix = \"x\")] struct C { speed: f64 }",
        &["unknown `parameters` option `prefix`"],
    );
}

// -------------------------------------------------------------------------------------------
// Shapes that are not parameter sets
// -------------------------------------------------------------------------------------------

#[test]
fn test_rejects_shapes_that_cannot_describe_parameters() {
    rejected_with("struct C(f64);", &["requires named fields"]);
    rejected_with("struct C;", &["at least one field"]);
    rejected_with("enum C { A }", &["cannot yet be derived for an enum"]);
    rejected_with("union C { a: f64 }", &["cannot be derived for a union"]);
    rejected_with(
        "struct C<T> { speed: T }",
        &["generic", "known when the struct is defined"],
    );
}

/// Several mistakes are reported together, rather than one compile at a time.
#[test]
fn test_reports_every_problem_at_once() {
    let errors = errors(
        r#"
        struct C {
            count: u64,
            timeout: Duration,
            #[param(read_only)]
            speed: Option<f64>,
        }
        "#,
    );
    assert_eq!(errors.len(), 3, "{errors:#?}");
}
