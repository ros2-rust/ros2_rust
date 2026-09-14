//! Tests for `#[derive(ParameterSet)]` on an enum: a group of parameters whose shape depends on
//! which variant a parameter file selected.

use std::sync::Arc;

use crate::{
    parameter::test_support::{
        node_with_parameter_overrides as node_with_params, parameter_descriptor,
    },
    *,
};

/// Configuration for one sensor.
#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(rename_all = "snake_case")]
enum SensorConfig {
    /// A 2D scanning lidar.
    Lidar {
        /// Scan rate in Hz.
        #[param(default = 30, range = 1..=100)]
        rate: i64,
        /// Maximum usable range in m.
        #[param(default = 25.0)]
        range_m: f64,
    },
    /// A USB camera, configured by an existing parameter set.
    Camera(CameraConfig),
    /// Present but not configured.
    Disabled,
}

/// Camera settings.
#[derive(ParameterSet, Debug, PartialEq)]
struct CameraConfig {
    /// Frame width in pixels.
    #[param(default = 1920)]
    width: u32,
    /// Frame height in pixels.
    #[param(default = 1080)]
    height: u32,
}

#[test]
fn test_a_parameter_file_selects_the_variant() {
    let (node, _file) = node_with_params(
        "sensor",
        r#"
/sensor:
  ros__parameters:
    type: lidar
    rate: 40
    range_m: 30.0
"#,
    );

    let config: SensorConfig = node.load_parameters().unwrap();
    assert_eq!(
        config,
        SensorConfig::Lidar {
            rate: 40,
            range_m: 30.0
        }
    );

    // Only the selected variant's parameters are declared.
    let undeclared = node.use_undeclared_parameters();
    assert_eq!(undeclared.get::<i64>("rate"), Some(40));
    assert_eq!(undeclared.get::<u32>("width"), None);
}

#[test]
fn test_a_newtype_variant_delegates_to_its_parameter_set() {
    let (node, _file) = node_with_params(
        "sensor_camera",
        r#"
/sensor_camera:
  ros__parameters:
    type: camera
    width: 640
"#,
    );

    let config: SensorConfig = node.load_parameters().unwrap();
    assert_eq!(
        config,
        SensorConfig::Camera(CameraConfig {
            width: 640,
            // Not in the file, so the field's own default.
            height: 1080,
        })
    );

    // The delegated set's parameters are declared under this set's namespace, not under a
    // namespace of the variant's.
    assert_eq!(
        node.use_undeclared_parameters().get::<i64>("width"),
        Some(640)
    );
}

#[test]
fn test_a_unit_variant_declares_only_the_tag() {
    let (node, _file) = node_with_params(
        "sensor_disabled",
        r#"
/sensor_disabled:
  ros__parameters:
    type: disabled
"#,
    );

    let config: SensorConfig = node.load_parameters().unwrap();
    assert_eq!(config, SensorConfig::Disabled);
    assert_eq!(node.use_undeclared_parameters().get::<i64>("rate"), None);
}

#[test]
fn test_the_variant_handles_can_be_matched_on() {
    let (node, _file) = node_with_params(
        "sensor_handles",
        r#"
/sensor_handles:
  ros__parameters:
    type: lidar
"#,
    );

    let params = node.declare_parameters::<SensorConfig>().unwrap();
    assert_eq!(params.tag(), "lidar");

    match &params.variant {
        SensorConfigVariantParams::Lidar { rate, range_m } => {
            assert_eq!(rate.get(), 30);
            assert_eq!(range_m.get(), 25.0);
            // The variant's parameters are live, like any others.
            rate.set(50).unwrap();
            assert!(rate.set(200).is_err(), "the range still applies");
        }
        other => panic!("expected a lidar, got {:?}", std::mem::discriminant(other)),
    }

    assert_eq!(
        params.snapshot(),
        SensorConfig::Lidar {
            rate: 50,
            range_m: 25.0
        }
    );
}

/// The tag is read-only: the set of declared parameters depends on it, and changing which
/// parameters exist at runtime would invalidate handles the caller is holding.
#[test]
fn test_the_tag_cannot_be_changed() {
    let (node, _file) = node_with_params(
        "sensor_read_only_tag",
        r#"
/sensor_read_only_tag:
  ros__parameters:
    type: lidar
"#,
    );
    let _params = node.declare_parameters::<SensorConfig>().unwrap();

    let err = node
        .use_undeclared_parameters()
        .set::<Arc<str>>("type", "camera".into())
        .unwrap_err();
    assert!(matches!(err, ParameterValueError::ReadOnly), "{err}");
}

#[test]
fn test_an_unknown_variant_fails_the_declaration() {
    let (node, _file) = node_with_params(
        "sensor_unknown",
        r#"
/sensor_unknown:
  ros__parameters:
    type: banana
"#,
    );

    let err = node.declare_parameters::<SensorConfig>().err().unwrap();
    assert_eq!(err.name, "type");
    let DeclarationError::InitialValueRejected(reason) = &err.source else {
        panic!("expected InitialValueRejected, got {:?}", err.source);
    };
    assert!(reason.contains("banana"), "{reason}");
    assert!(
        reason.contains("lidar, camera, disabled"),
        "the reason should list the variants: {reason}"
    );
}

#[test]
fn test_a_missing_tag_fails_the_declaration() {
    let node = Context::default()
        .create_basic_executor()
        .create_node("sensor_no_tag")
        .unwrap();

    let err = node.declare_parameters::<SensorConfig>().err().unwrap();
    assert_eq!(err.name, "type");
    assert_eq!(err.source, DeclarationError::NoValueAvailable);
}

/// The valid variants reach the descriptor, so an operator can discover them.
#[test]
fn test_the_variants_appear_in_the_tag_descriptor() {
    let (node, _file) = node_with_params(
        "sensor_describe",
        r#"
/sensor_describe:
  ros__parameters:
    type: disabled
"#,
    );
    let _params = node.declare_parameters::<SensorConfig>().unwrap();

    let descriptor = parameter_descriptor(&node, "type");
    assert_eq!(
        descriptor.additional_constraints.to_string(),
        "one of: lidar, camera, disabled"
    );
    assert_eq!(
        descriptor.description.to_string(),
        "Configuration for one sensor."
    );
    assert!(descriptor.read_only);
}

// ---------------------------------------------------------------------------------------------
// Nesting and naming
// ---------------------------------------------------------------------------------------------

/// A hub with one sensor, to check that an enum set nests like any other.
#[derive(ParameterSet, Debug, PartialEq)]
struct SensorHub {
    /// How many times to retry a read.
    #[param(default = 3)]
    retries: u8,
    /// The sensor to use.
    sensor: SensorConfig,
}

#[test]
fn test_an_enum_set_nests_inside_a_struct_set() {
    let (node, _file) = node_with_params(
        "hub",
        r#"
/hub:
  ros__parameters:
    retries: 5
    sensor:
      type: lidar
      rate: 15
"#,
    );

    let config: SensorHub = node.load_parameters().unwrap();
    assert_eq!(
        config,
        SensorHub {
            retries: 5,
            sensor: SensorConfig::Lidar {
                rate: 15,
                range_m: 25.0
            },
        }
    );

    let undeclared = node.use_undeclared_parameters();
    assert_eq!(
        undeclared.get::<Arc<str>>("sensor.type").as_deref(),
        Some("lidar")
    );
    assert_eq!(undeclared.get::<i64>("sensor.rate"), Some(15));
}

#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(tag = "kind", rename_all = "kebab-case")]
enum Renamed {
    TwoDimensional { resolution: f64 },
    Disabled,
}

#[test]
fn test_the_tag_name_and_values_can_be_chosen() {
    let (node, _file) = node_with_params(
        "renamed",
        r#"
/renamed:
  ros__parameters:
    kind: two-dimensional
    resolution: 0.05
"#,
    );

    let config: Renamed = node.load_parameters().unwrap();
    assert_eq!(config, Renamed::TwoDimensional { resolution: 0.05 });
}

// ---------------------------------------------------------------------------------------------
// Defaults
// ---------------------------------------------------------------------------------------------

#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(default = Self::default(), rename_all = "snake_case")]
enum Defaulted {
    Fast { rate: i64 },
    Slow { rate: i64 },
}

impl Default for Defaulted {
    fn default() -> Self {
        Self::Slow { rate: 2 }
    }
}

/// A whole-value default supplies both which variant to use and that variant's values.
#[test]
fn test_a_default_supplies_the_variant_and_its_values() {
    let node = Context::default()
        .create_basic_executor()
        .create_node("defaulted_enum")
        .unwrap();

    let config: Defaulted = node.load_parameters().unwrap();
    assert_eq!(config, Defaulted::Slow { rate: 2 });
}

/// A parameter file that selects a different variant gets that variant's own defaults, since the
/// values in the default belong to the variant it names.
#[test]
fn test_selecting_another_variant_does_not_take_the_defaults_values() {
    let (node, _file) = node_with_params(
        "defaulted_enum_other",
        r#"
/defaulted_enum_other:
  ros__parameters:
    type: fast
    rate: 100
"#,
    );

    let config: Defaulted = node.load_parameters().unwrap();
    assert_eq!(config, Defaulted::Fast { rate: 100 });
}

#[test]
fn test_selecting_another_variant_without_defaults_and_no_overrides_errors() {
    let (node, _file) = node_with_params(
        "defaulted_enum_other",
        r#"
/defaulted_enum_other:
  ros__parameters:
    type: fast
"#,
    );

    let err = node.declare_parameters::<Defaulted>().err().unwrap();
    assert!(
        err.to_string()
            .contains("'rate': parameter was declared as non-optional but no value was available"),
        "Expected error message not found"
    );
}
