//! Tests for map fields: parameter sets whose entries are named by whoever configures the node.

use std::collections::{BTreeMap, HashMap};

use crate::{parameter::test_support::node_with_parameter_overrides as node_with_params, *};

/// Settings for one sensor.
#[derive(ParameterSet, Debug, PartialEq, Clone)]
struct SensorConfig {
    /// Publish rate in Hz.
    #[param(default = 30, range = 1..=100)]
    rate: i64,
    /// Frame this sensor reports in.
    #[param(default = "base_link")]
    frame_id: String,
}

/// A node that manages however many sensors it is configured with.
#[derive(ParameterSet, Debug, PartialEq)]
struct SensorHub {
    /// One entry per sensor, named in the parameter file.
    sensors: BTreeMap<String, SensorConfig>,
}

/// The names of the entries cannot be known when the node is written, and are not visible
/// anywhere until the parameters they name are declared. They come from the overrides.
#[test]
fn test_entry_names_come_from_the_parameter_file() {
    let (node, _file) = node_with_params(
        "sensor_hub",
        r#"
/sensor_hub:
  ros__parameters:
    sensors:
      front_lidar:
        rate: 40
      rear_lidar:
        rate: 10
        frame_id: rear_mount
"#,
    );

    let config: SensorHub = node.load_parameters().unwrap();
    assert_eq!(
        config.sensors.keys().collect::<Vec<_>>(),
        vec!["front_lidar", "rear_lidar"]
    );
    assert_eq!(
        config.sensors["front_lidar"],
        SensorConfig {
            rate: 40,
            // Not in the file, so the field's own default.
            frame_id: "base_link".to_string(),
        }
    );
    assert_eq!(
        config.sensors["rear_lidar"],
        SensorConfig {
            rate: 10,
            frame_id: "rear_mount".to_string(),
        }
    );
}

/// Every entry's fields are ordinary parameters: named, described, ranged and live.
#[test]
fn test_the_entries_are_ordinary_parameters() {
    let (node, _file) = node_with_params(
        "sensor_hub_params",
        r#"
/sensor_hub_params:
  ros__parameters:
    sensors:
      front_lidar:
        rate: 40
"#,
    );

    let params = node.declare_parameters::<SensorHub>().unwrap();
    let front = &params.sensors["front_lidar"];
    assert_eq!(front.rate.get(), 40);
    front.rate.set(50).unwrap();
    assert!(front.rate.set(500).is_err(), "the range still applies");

    assert_eq!(
        node.use_undeclared_parameters()
            .get::<i64>("sensors.front_lidar.rate"),
        Some(50)
    );
}

#[test]
fn test_no_entries_is_an_empty_map_rather_than_an_error() {
    let node = Context::default()
        .create_basic_executor()
        .create_node("sensor_hub_empty")
        .unwrap();

    let config: SensorHub = node.load_parameters().unwrap();
    assert!(config.sensors.is_empty());
}

/// A default supplies entries of its own, so a node can have built-in ones.
#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(default = Self::default())]
struct DefaultedHub {
    sensors: BTreeMap<String, SensorConfig>,
}

impl Default for DefaultedHub {
    fn default() -> Self {
        let mut sensors = BTreeMap::new();
        sensors.insert(
            "builtin".to_string(),
            SensorConfig {
                rate: 5,
                frame_id: "builtin_frame".to_string(),
            },
        );
        Self { sensors }
    }
}

#[test]
fn test_a_default_supplies_entries() {
    let node = Context::default()
        .create_basic_executor()
        .create_node("hub_defaulted")
        .unwrap();

    let config: DefaultedHub = node.load_parameters().unwrap();
    assert_eq!(config.sensors.len(), 1);
    assert_eq!(config.sensors["builtin"].rate, 5);
}

#[test]
fn test_a_parameter_file_adds_to_and_overrides_the_default_entries() {
    let (node, _file) = node_with_params(
        "hub_defaulted_yaml",
        r#"
/hub_defaulted_yaml:
  ros__parameters:
    sensors:
      builtin:
        rate: 50
      extra:
        rate: 20
"#,
    );

    let config: DefaultedHub = node.load_parameters().unwrap();
    assert_eq!(config.sensors.len(), 2);
    // The file wins for an entry that exists in both...
    assert_eq!(config.sensors["builtin"].rate, 50);
    // ...but the default still supplies the fields the file does not mention.
    assert_eq!(config.sensors["builtin"].frame_id, "builtin_frame");
    // And a new entry appears.
    assert_eq!(config.sensors["extra"].rate, 20);
    assert_eq!(config.sensors["extra"].frame_id, "base_link");
}

/// An error inside an entry names the entry, since that is what the caller has to go and look at.
#[derive(ParameterSet)]
struct RequiredHub {
    sensors: BTreeMap<String, RequiredSensor>,
}

#[derive(ParameterSet)]
struct RequiredSensor {
    /// No default: the file has to supply it.
    rate: i64,
}

#[test]
fn test_an_error_inside_an_entry_names_the_entry() {
    let (node, _file) = node_with_params(
        "hub_required",
        r#"
/hub_required:
  ros__parameters:
    sensors:
      front_lidar:
        frame_id: x
"#,
    );

    let err = node.declare_parameters::<RequiredHub>().err().unwrap();
    assert_eq!(err.name, "sensors.front_lidar.rate");
    assert_eq!(err.source, DeclarationError::NoValueAvailable);
}

/// A `HashMap` works the same way, and only the iteration order differs.
#[derive(ParameterSet, Debug, PartialEq)]
struct HashHub {
    sensors: HashMap<String, SensorConfig>,
}

#[test]
fn test_a_hash_map_field() {
    let (node, _file) = node_with_params(
        "hash_hub",
        "/hash_hub:\n  ros__parameters:\n    sensors:\n      a:\n        rate: 1\n",
    );

    let config: HashHub = node.load_parameters().unwrap();
    assert_eq!(config.sensors["a"].rate, 1);
}

/// The names of the entries are the first path segment under the map's own name. Anything that
/// merely starts with the same characters is a different parameter, and a name with nothing under
/// it is not an entry at all.
#[test]
fn test_which_override_names_count_as_entries() {
    let (node, _file) = node_with_params(
        "hub_names",
        r#"
/hub_names:
  ros__parameters:
    sensors:
      front_lidar:
        rate: 40
      rear_lidar:
        nested:
          deeper: 1
    sensors_extra:
      rate: 1
    unrelated: 2
"#,
    );

    let names = node
        .parameter_interface()
        .override_names_under("sensors")
        .into_iter()
        .collect::<Vec<_>>();
    // `sensors_extra` shares a prefix but is not under `sensors`, and a name is reported once
    // however deeply nested the parameters beneath it are.
    assert_eq!(names, vec!["front_lidar", "rear_lidar"]);

    // A name with no parameters under it has no entries of its own.
    assert!(node
        .parameter_interface()
        .override_names_under("unrelated")
        .is_empty());
    assert!(node
        .parameter_interface()
        .override_names_under("nonexistent")
        .is_empty());
}

// ---------------------------------------------------------------------------------------------
// Entries that are not all the same shape
// ---------------------------------------------------------------------------------------------

/// Configuration for one device, whatever kind it is.
#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(rename_all = "snake_case")]
enum DeviceConfig {
    /// A 2D scanning lidar.
    Lidar {
        /// Scan rate in Hz.
        #[param(default = 30)]
        rate: i64,
    },
    /// A camera.
    Camera {
        /// Frame width in pixels.
        #[param(default = 1920)]
        width: u32,
        /// Frame height in pixels.
        #[param(default = 1080)]
        height: u32,
    },
}

/// A node that manages however many devices it is configured with, of whatever kinds.
#[derive(ParameterSet, Debug, PartialEq)]
struct DeviceHub {
    /// One entry per device.
    devices: BTreeMap<String, DeviceConfig>,
}

/// Maps and enum sets compose: the entries are named by the parameter file *and* each one
/// declares only the parameters its own kind needs.
#[test]
fn test_entries_can_have_different_shapes() {
    let (node, _file) = node_with_params(
        "device_hub",
        r#"
/device_hub:
  ros__parameters:
    devices:
      front_lidar:
        type: lidar
        rate: 40
      main_camera:
        type: camera
        width: 640
"#,
    );

    let config: DeviceHub = node.load_parameters().unwrap();
    assert_eq!(
        config.devices["front_lidar"],
        DeviceConfig::Lidar { rate: 40 }
    );
    assert_eq!(
        config.devices["main_camera"],
        DeviceConfig::Camera {
            width: 640,
            height: 1080,
        }
    );

    // Each entry declared only what its own kind needs.
    let undeclared = node.use_undeclared_parameters();
    assert_eq!(undeclared.get::<i64>("devices.front_lidar.rate"), Some(40));
    assert_eq!(undeclared.get::<i64>("devices.front_lidar.width"), None);
    assert_eq!(
        undeclared.get::<i64>("devices.main_camera.width"),
        Some(640)
    );
    assert_eq!(undeclared.get::<i64>("devices.main_camera.rate"), None);
}
