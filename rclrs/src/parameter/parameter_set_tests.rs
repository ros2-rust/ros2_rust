//! Integration tests for the `ParameterSet` derive macro.

use crate as rclrs;
use crate::*;

#[derive(ParameterSet)]
struct BasicParams {
    #[param(default = 42)]
    count: MandatoryParameter<i64>,

    #[param(default = 3.14)]
    ratio: OptionalParameter<f64>,

    #[param(default = true)]
    enabled: ReadOnlyParameter<bool>,
}

#[test]
fn test_basic_parameter_set() {
    assert_eq!(BasicParams::default_namespace(), "basic_params");

    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: BasicParams = node.declare_parameter_set().unwrap();

    // Values are correct
    assert_eq!(params.count.get(), 42);
    assert_eq!(params.ratio.get(), Some(3.14));
    assert!(params.enabled.get());

    // Parameters are declared under default namespace "basic_params.*"
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<i64>("basic_params.count"),
        Some(42)
    );
}

#[derive(ParameterSet)]
struct SensorConfig {
    #[param(default = 30)]
    rate: MandatoryParameter<i64>,
}

#[derive(ParameterSet)]
struct RobotParams {
    #[param(default = 50.0)]
    speed: MandatoryParameter<f64>,

    sensors: SensorConfig,
}

#[test]
fn test_nested_parameter_set() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let robot: RobotParams = node.declare_parameter_set().unwrap();

    assert_eq!(robot.speed.get(), 50.0);
    assert_eq!(robot.sensors.rate.get(), 30);

    // Check namespacing: "robot_params.speed", "robot_params.sensors.rate"
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<f64>("robot_params.speed"),
        Some(50.0)
    );
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<i64>("robot_params.sensors.rate"),
        Some(30)
    );
}

#[test]
fn test_custom_prefix() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    // Prefix is additive: "bot" + "robot_params" -> "bot.robot_params.*"
    let _robot: RobotParams = node.declare_parameter_set_with_prefix("bot").unwrap();

    assert_eq!(
        node.use_undeclared_parameters()
            .get::<f64>("bot.robot_params.speed"),
        Some(50.0)
    );
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<i64>("bot.robot_params.sensors.rate"),
        Some(30)
    );
}

#[derive(ParameterSet)]
#[parameters(flatten)]
struct FlattenedParams {
    #[param(default = 100)]
    global_limit: MandatoryParameter<i64>,
}

#[test]
fn test_flattened_parameter_set() {
    assert_eq!(FlattenedParams::default_namespace(), "");

    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: FlattenedParams = node.declare_parameter_set().unwrap();

    assert_eq!(params.global_limit.get(), 100);
    assert_eq!(
        node.use_undeclared_parameters().get::<i64>("global_limit"),
        Some(100)
    );
}

#[derive(ParameterSet)]
#[parameters(namespace = "drive")]
struct DriveConfig {
    #[param(default = 50.0)]
    speed: MandatoryParameter<f64>,
}

#[test]
fn test_custom_namespace() {
    assert_eq!(DriveConfig::default_namespace(), "drive");

    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let config: DriveConfig = node.declare_parameter_set().unwrap();

    assert_eq!(config.speed.get(), 50.0);
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("drive.speed"),
        Some(50.0)
    );
}

#[derive(ParameterSet)]
struct Limits {
    #[param(default = 100.0)]
    max_force: MandatoryParameter<f64>,
}

#[derive(ParameterSet)]
struct FlattenTest {
    #[param(default = 1.0)]
    speed: MandatoryParameter<f64>,

    #[param(flatten)]
    limits: Limits,
}

#[test]
fn test_flatten() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: FlattenTest = node.declare_parameter_set().unwrap();

    assert_eq!(params.speed.get(), 1.0);
    assert_eq!(params.limits.max_force.get(), 100.0);

    // "limits" should NOT appear in the parameter name
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<f64>("flatten_test.max_force"),
        Some(100.0)
    );
    assert_eq!(
        node.use_undeclared_parameters()
            .get::<f64>("flatten_test.speed"),
        Some(1.0)
    );
}

#[derive(ParameterSet)]
#[parameters(flatten)]
struct FullOptions {
    #[param(
        default = 50,
        description = "Motor speed",
        constraints = "must be positive",
        range(lower = 0, upper = 100),
        ignore_override
    )]
    speed: MandatoryParameter<i64>,
}

#[test]
fn test_full_builder_options() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: FullOptions = node.declare_parameter_set().unwrap();

    assert_eq!(params.speed.get(), 50);
    // Range should be enforced
    assert!(params.speed.set(200).is_err());
    assert!(params.speed.set(75).is_ok());
    assert_eq!(params.speed.get(), 75);
}

fn always_max(_available: AvailableValues<i64>) -> Option<i64> {
    // Return the known upper bound of the range
    Some(100)
}

#[derive(ParameterSet)]
#[parameters(flatten)]
struct DiscriminateParams {
    #[param(
        default = 10,
        range(lower = 0, upper = 100),
        discriminate = always_max,
    )]
    value: MandatoryParameter<i64>,
}

#[test]
fn test_discriminate() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: DiscriminateParams = node.declare_parameter_set().unwrap();

    // Discriminator should pick upper bound (100), not default (10)
    assert_eq!(params.value.get(), 100);
}

type Speed = MandatoryParameter<f64>;

#[derive(ParameterSet)]
#[parameters(flatten)]
struct AliasedParams {
    #[param(mandatory, default = 25.0)]
    speed: Speed,
}

#[test]
fn test_explicit_parameter_type_with_type_alias() {
    let executor = Context::default().create_basic_executor();
    let node = executor
        .create_node(&format!("param_set_test_{}", line!()))
        .unwrap();

    let params: AliasedParams = node.declare_parameter_set().unwrap();
    assert_eq!(params.speed.get(), 25.0);
}
