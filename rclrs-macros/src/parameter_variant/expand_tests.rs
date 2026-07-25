//! Tests for what `#[derive(ParameterVariant)]` accepts and what it says about what it rejects.
//!
//! As in `parameter_set::expand_tests`, these assert on the messages the macro produces rather
//! than on how a particular rustc renders the generated code. The generated code itself is
//! exercised by the tests in `rclrs`.

use super::expand;
use syn::DeriveInput;

fn errors(input: &str) -> Vec<String> {
    let parsed: DeriveInput = syn::parse_str(input).expect("test input should parse");
    match expand(&parsed) {
        Ok(_) => Vec::new(),
        Err(error) => error.into_iter().map(|e| e.to_string()).collect(),
    }
}

#[track_caller]
fn rejected_with(input: &str, expected: &[&str]) {
    let errors = errors(input);
    assert!(!errors.is_empty(), "should have been rejected");
    let joined = errors.join("\n");
    for fragment in expected {
        assert!(
            joined.contains(fragment),
            "message should mention {fragment:?}, but was: {joined}"
        );
    }
}

#[track_caller]
fn accepted(input: &str) -> String {
    let parsed: DeriveInput = syn::parse_str(input).expect("test input should parse");
    match expand(&parsed) {
        Ok(tokens) => tokens.to_string(),
        Err(error) => panic!(
            "should have been accepted: {:#?}",
            error.into_iter().map(|e| e.to_string()).collect::<Vec<_>>()
        ),
    }
}

#[test]
fn test_a_plain_enum_becomes_a_string_of_variant_names() {
    let generated = accepted(
        r#"
        #[parameter(rename_all = "snake_case")]
        enum ControlMode { Velocity, Position, EffortLimited }
        "#,
    );
    assert!(generated.contains("\"velocity\""), "{generated}");
    assert!(generated.contains("\"effort_limited\""), "{generated}");
    // The valid values are reported both as descriptor constraints and in the rejection message.
    assert!(
        generated.contains("one of: velocity, position, effort_limited"),
        "{generated}"
    );
    assert!(generated.contains("unknown ControlMode"), "{generated}");
    // And the type is usable as a parameter set field.
    assert!(generated.contains("declare_parameter_field"), "{generated}");
}

#[test]
fn test_a_variant_can_set_its_own_stored_value() {
    let generated = accepted(
        r#"
        #[parameter(rename_all = "snake_case")]
        enum Mode { Velocity, #[parameter(rename = "pos")] Position }
        "#,
    );
    assert!(generated.contains("\"pos\""), "{generated}");
    assert!(!generated.contains("\"position\""), "{generated}");
}

#[test]
fn test_variant_names_are_kept_as_written_by_default() {
    let generated = accepted("enum Mode { Velocity, Position }");
    assert!(generated.contains("\"Velocity\""), "{generated}");
}

#[test]
fn test_transparent_takes_the_representation_of_what_it_wraps() {
    let generated = accepted("#[parameter(transparent)] struct Meters(f64);");
    assert!(generated.contains("f64"), "{generated}");
    // Including the range type, so a range on a `Meters` parameter is written in metres.
    assert!(
        generated.contains("ParameterVariant > :: Range"),
        "{generated}"
    );

    // A named field works too.
    let generated = accepted("#[parameter(transparent)] struct Meters { value: f64 }");
    assert!(generated.contains("value"), "{generated}");
}

#[test]
fn test_from_str_uses_from_str_and_display() {
    let generated = accepted("#[parameter(from_str)] struct Hostname(String);");
    assert!(generated.contains("FromStr"), "{generated}");
    assert!(generated.contains("to_string"), "{generated}");
}

// -------------------------------------------------------------------------------------------
// Rejections
// -------------------------------------------------------------------------------------------

#[test]
fn test_rejects_a_struct_with_no_representation_chosen() {
    rejected_with(
        "struct Gains { kp: f64, ki: f64 }",
        &[
            "no representation as a single parameter value",
            "transparent",
            "from_str",
            "derive `ParameterSet`",
        ],
    );
}

#[test]
fn test_rejects_transparent_on_more_than_one_field() {
    rejected_with(
        "#[parameter(transparent)] struct Gains { kp: f64, ki: f64 }",
        &["exactly one field"],
    );
    rejected_with(
        "#[parameter(transparent)] struct Nothing;",
        &["exactly one field"],
    );
}

#[test]
fn test_rejects_transparent_on_an_enum() {
    rejected_with(
        "#[parameter(transparent)] enum Mode { A, B }",
        &["wraps a single value, not to an enum"],
    );
}

#[test]
fn test_rejects_a_variant_that_carries_data() {
    rejected_with(
        "enum SensorConfig { Lidar { rate: i64 }, Disabled }",
        &["carries data", "not supported yet"],
    );
    rejected_with(
        "enum SensorConfig { Lidar(LidarConfig) }",
        &["carries data"],
    );
}

#[test]
fn test_rejects_two_variants_stored_as_the_same_value() {
    rejected_with(
        r#"
        enum Mode {
            Velocity,
            #[parameter(rename = "Velocity")]
            Speed,
        }
        "#,
        &["already the stored value of variant `Velocity`"],
    );
}

#[test]
fn test_rejects_an_empty_enum() {
    rejected_with("enum Nothing {}", &["no variants"]);
}

#[test]
fn test_rejects_contradictory_representations() {
    rejected_with(
        "#[parameter(transparent, from_str)] struct Hostname(String);",
        &["two different representations"],
    );
}

#[test]
fn test_rejects_rename_all_where_it_has_no_meaning() {
    rejected_with(
        r#"#[parameter(from_str, rename_all = "snake_case")] struct Hostname(String);"#,
        &["no meaning together with `from_str`"],
    );
}

#[test]
fn test_rejects_an_unknown_naming_convention() {
    rejected_with(
        r#"#[parameter(rename_all = "SpongeBobCase")] enum Mode { A }"#,
        &["unknown naming convention", "snake_case"],
    );
}

#[test]
fn test_rejects_generic_and_union_types() {
    rejected_with(
        "#[parameter(transparent)] struct Wrapper<T>(T);",
        &["generic"],
    );
    rejected_with("union U { a: f64 }", &["cannot be derived for a union"]);
}

#[test]
fn test_rejects_unknown_options() {
    rejected_with(
        "#[parameter(json)] struct Gains { kp: f64 }",
        &["unknown `parameter` option"],
    );
    rejected_with(
        r#"enum Mode { #[parameter(alias = "v")] Velocity }"#,
        &["unknown `parameter` option on a variant"],
    );
}
