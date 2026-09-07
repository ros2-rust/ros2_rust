//! Tests for `#[derive(ParameterVariant)]`: the code it generates, and how the parameters that
//! use it behave when a value arrives from outside the node.

use std::{fmt, str::FromStr, sync::Arc};

use crate::{parameter::test_support::parameter_descriptor, *};

fn node(name: &str) -> Node {
    Context::default()
        .create_basic_executor()
        .create_node(name)
        .unwrap()
}

// ---------------------------------------------------------------------------------------------
// An enum of plain variants
// ---------------------------------------------------------------------------------------------

/// Which quantity a controller closes the loop on.
#[derive(ParameterVariant, Clone, Copy, Debug, PartialEq)]
#[parameter(rename_all = "snake_case")]
enum ControlMode {
    Velocity,
    Position,
    /// Stored under a different name from the variant's.
    #[parameter(rename = "torque")]
    Effort,
}

#[test]
fn test_enum_round_trips_through_a_string() {
    let stored: ParameterValue = ControlMode::Velocity.into();
    assert_eq!(stored, ParameterValue::String("velocity".into()));
    assert_eq!(
        ControlMode::try_from(stored).unwrap(),
        ControlMode::Velocity
    );

    let stored: ParameterValue = ControlMode::Effort.into();
    assert_eq!(stored, ParameterValue::String("torque".into()));
    assert_eq!(ControlMode::try_from(stored).unwrap(), ControlMode::Effort);
}

#[test]
fn test_an_unknown_variant_is_rejected_with_the_valid_ones() {
    let err = ControlMode::try_from(ParameterValue::String("banana".into())).unwrap_err();
    let ParameterValueError::Invalid(reason) = &err else {
        panic!("expected Invalid, got {err:?}");
    };
    assert!(reason.contains("banana"), "{reason}");
    assert!(
        reason.contains("velocity, position, torque"),
        "the reason should list the valid values: {reason}"
    );

    // A value of the wrong ROS 2 type is a different kind of problem.
    assert!(matches!(
        ControlMode::try_from(ParameterValue::Integer(1)),
        Err(ParameterValueError::TypeMismatch)
    ));
}

#[test]
fn test_enum_parameters_can_be_declared_and_set() {
    let node = node("enum_param");
    let mode: MandatoryParameter<ControlMode> = node
        .declare_parameter("mode")
        .default(ControlMode::Velocity)
        .mandatory()
        .unwrap();

    assert_eq!(mode.get(), ControlMode::Velocity);
    mode.set(ControlMode::Position).unwrap();
    assert_eq!(mode.get(), ControlMode::Position);

    // On the ROS 2 side it is a string.
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<Arc<str>>("mode")
            .as_deref(),
        Some("position")
    );
}

/// The reason a string-backed enum needs the declared type to be enforced: without it, this set
/// would be accepted and the next `get()` would panic.
#[test]
fn test_an_invalid_value_from_outside_the_node_is_rejected() {
    let node = node("enum_param_rejects");
    let mode: MandatoryParameter<ControlMode> = node
        .declare_parameter("mode")
        .default(ControlMode::Velocity)
        .mandatory()
        .unwrap();

    let err = node
        .use_undeclared_parameters()
        .set::<Arc<str>>("mode", "banana".into())
        .unwrap_err();
    assert!(
        matches!(&err, ParameterValueError::Invalid(reason) if reason.contains("velocity")),
        "unexpected error: {err}"
    );
    assert_eq!(mode.get(), ControlMode::Velocity);
}

/// The valid values reach the parameter descriptor, so `ros2 param describe` can report them
/// without the declaration having to restate them.
#[test]
fn test_the_valid_values_appear_in_the_descriptor() {
    let node = node("enum_param_describe");
    let _mode: MandatoryParameter<ControlMode> = node
        .declare_parameter("mode")
        .default(ControlMode::Velocity)
        .mandatory()
        .unwrap();

    assert_eq!(
        parameter_descriptor(&node, "mode")
            .additional_constraints
            .to_string(),
        "one of: velocity, position, torque"
    );
}

// ---------------------------------------------------------------------------------------------
// A newtype over another parameter type
// ---------------------------------------------------------------------------------------------

/// A distance in metres.
#[derive(ParameterVariant, Clone, Copy, Debug, PartialEq, PartialOrd, Default)]
#[parameter(transparent)]
struct Meters(pub f64);

/// A TCP port.
#[derive(ParameterVariant, Clone, Copy, Debug, PartialEq)]
#[parameter(transparent)]
struct Port {
    number: u16,
}

#[test]
fn test_a_transparent_newtype_behaves_as_what_it_wraps() {
    let stored: ParameterValue = Meters(2.5).into();
    assert_eq!(stored, ParameterValue::Double(2.5));
    assert_eq!(Meters::try_from(stored).unwrap(), Meters(2.5));
    assert_eq!(Meters::kind(), ParameterKind::Double);

    // Including a named field.
    let stored: ParameterValue = Port { number: 8080 }.into();
    assert_eq!(stored, ParameterValue::Integer(8080));
    assert_eq!(Port::try_from(stored).unwrap(), Port { number: 8080 });
}

/// The wrapped type's validation is inherited, so a `Port` cannot hold a value a `u16` could not.
#[test]
fn test_a_transparent_newtype_inherits_validation() {
    assert!(Port::try_from(ParameterValue::Integer(70000)).is_err());

    let node = node("transparent_validation");
    let port: MandatoryParameter<Port> = node
        .declare_parameter("port")
        .default(Port { number: 8080 })
        .mandatory()
        .unwrap();

    let err = node
        .use_undeclared_parameters()
        .set::<i64>("port", 70000)
        .unwrap_err();
    assert!(matches!(err, ParameterValueError::Invalid(_)), "{err}");
    assert_eq!(port.get(), Port { number: 8080 });
}

/// Ranges are expressed in the units of the wrapped type, so a range on a `Meters` parameter is
/// written in metres.
#[test]
fn test_a_transparent_newtype_keeps_the_range_type() {
    let node = node("transparent_range");
    let distance: MandatoryParameter<Meters> = node
        .declare_parameter("distance")
        .default(Meters(1.0))
        .range(ParameterRange {
            lower: Some(0.0),
            upper: Some(5.0),
            step: None,
        })
        .mandatory()
        .unwrap();

    assert!(distance.set(Meters(3.0)).is_ok());
    assert!(distance.set(Meters(6.0)).is_err());
}

// ---------------------------------------------------------------------------------------------
// A type with a FromStr
// ---------------------------------------------------------------------------------------------

/// A hostname, which must not be empty.
#[derive(ParameterVariant, Clone, Debug, PartialEq)]
#[parameter(from_str)]
struct Hostname(String);

impl FromStr for Hostname {
    type Err = String;

    fn from_str(text: &str) -> Result<Self, Self::Err> {
        if text.is_empty() {
            Err("a hostname must not be empty".to_string())
        } else {
            Ok(Hostname(text.to_string()))
        }
    }
}

impl fmt::Display for Hostname {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}", self.0)
    }
}

#[test]
fn test_from_str_round_trips_and_reports_its_own_errors() {
    let stored: ParameterValue = Hostname("robot.local".to_string()).into();
    assert_eq!(stored, ParameterValue::String("robot.local".into()));
    assert_eq!(
        Hostname::try_from(stored).unwrap(),
        Hostname("robot.local".to_string())
    );

    // The FromStr error becomes the reason the value was rejected.
    let err = Hostname::try_from(ParameterValue::String("".into())).unwrap_err();
    assert!(
        matches!(&err, ParameterValueError::Invalid(reason) if reason.contains("must not be empty")),
        "unexpected error: {err}"
    );
}

// ---------------------------------------------------------------------------------------------
// In a parameter set
// ---------------------------------------------------------------------------------------------

/// Configuration using types of its own.
#[derive(ParameterSet, Debug, PartialEq)]
struct CustomTypesConfig {
    /// How the controller closes the loop.
    #[param(default = ControlMode::Velocity)]
    mode: ControlMode,

    /// Stopping distance.
    #[param(default = Meters(1.5), range = 0.0..=10.0)]
    stopping_distance: Meters,

    /// Where to reach the robot.
    #[param(default = Hostname("robot.local".to_string()))]
    host: Hostname,

    /// Unset unless configured.
    fallback: Option<ControlMode>,
}

#[test]
fn test_custom_types_as_parameter_set_fields() {
    let node = node("custom_types_set");
    let config: CustomTypesConfig = node.load_parameters().unwrap();

    assert_eq!(
        config,
        CustomTypesConfig {
            mode: ControlMode::Velocity,
            stopping_distance: Meters(1.5),
            host: Hostname("robot.local".to_string()),
            fallback: None,
        }
    );
}

/// The whole point of the enum representation: a parameter file names the variant.
#[test]
fn test_a_parameter_file_can_name_an_enum_variant() {
    use std::io::Write;

    let mut file = tempfile::NamedTempFile::new().unwrap();
    write!(
        file,
        r#"
/custom_types_yaml:
  ros__parameters:
    mode: torque
    stopping_distance: 2.5
    host: arm.local
    fallback: position
"#
    )
    .unwrap();
    let node = Context::default()
        .create_basic_executor()
        .create_node(NodeOptions::new("custom_types_yaml").arguments([
            "--ros-args",
            "--params-file",
            &file.path().display().to_string(),
        ]))
        .unwrap();

    let config: CustomTypesConfig = node.load_parameters().unwrap();
    assert_eq!(
        config,
        CustomTypesConfig {
            mode: ControlMode::Effort,
            stopping_distance: Meters(2.5),
            host: Hostname("arm.local".to_string()),
            fallback: Some(ControlMode::Position),
        }
    );
}

/// A value in the file that is not a valid variant fails the declaration, naming the parameter.
#[test]
fn test_an_invalid_value_in_a_parameter_file_fails_the_declaration() {
    use std::io::Write;

    let mut file = tempfile::NamedTempFile::new().unwrap();
    write!(
        file,
        r#"
/custom_types_bad_yaml:
  ros__parameters:
    mode: banana
"#
    )
    .unwrap();
    let node = Context::default()
        .create_basic_executor()
        .create_node(NodeOptions::new("custom_types_bad_yaml").arguments([
            "--ros-args",
            "--params-file",
            &file.path().display().to_string(),
        ]))
        .unwrap();

    let err = node
        .declare_parameters::<CustomTypesConfig>()
        .err()
        .unwrap();
    assert_eq!(err.name, "mode");
    assert_eq!(err.source, DeclarationError::OverrideValueTypeMismatch);
}
