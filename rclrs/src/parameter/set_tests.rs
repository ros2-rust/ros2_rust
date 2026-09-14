//! Tests for `#[derive(ParameterSet)]` and the node methods that declare a set.

use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
    time::Duration,
};

use crate::{
    parameter::test_support::{
        node_with_parameter_overrides as node_with_params, parameter_descriptor,
    },
    *,
};

fn node(name: &str) -> Node {
    Context::default()
        .create_basic_executor()
        .create_node(name)
        .unwrap()
}

// ---------------------------------------------------------------------------------------------
// The shape of a set
// ---------------------------------------------------------------------------------------------

/// Configuration for a differential drive controller.
#[derive(ParameterSet, Debug, PartialEq)]
struct DriveConfig {
    /// Maximum forward speed in m/s.
    #[param(default = 1.5, range = 0.0..=10.0)]
    max_speed: f64,

    /// Names of the wheel joints.
    #[param(default = ["left_wheel", "right_wheel"])]
    wheels: Vec<String>,

    /// Watchdog timeout. Unset disables the watchdog.
    #[param(convert = seconds())]
    watchdog: Option<Duration>,

    /// Serial device the motor controller is on.
    #[param(read_only, default = "/dev/ttyUSB0")]
    device: PathBuf,

    /// Safety limits. A nested set needs no annotation.
    limits: Limits,
}

/// Safety limits.
#[derive(ParameterSet, Debug, PartialEq)]
struct Limits {
    /// Maximum motor force in N.
    #[param(default = 100.0, range = 0.0..=250.0)]
    max_force: f64,
}

/// Every field is a parameter, readable through its handle or all at once through a snapshot.
/// The snapshot is a plain struct, so code that consumes a configuration needs no node to test.
#[test]
fn test_declares_every_field_as_a_parameter() {
    let node = node("drive_controller");
    let params = node.declare_parameters::<DriveConfig>().unwrap();

    assert_eq!(params.max_speed.get(), 1.5);
    assert_eq!(params.wheels.get(), vec!["left_wheel", "right_wheel"]);
    assert_eq!(params.watchdog.get(), None);
    assert_eq!(params.device.get(), PathBuf::from("/dev/ttyUSB0"));
    assert_eq!(params.limits.max_force.get(), 100.0);

    assert_eq!(
        params.snapshot(),
        DriveConfig {
            max_speed: 1.5,
            wheels: vec!["left_wheel".to_string(), "right_wheel".to_string()],
            watchdog: None,
            device: PathBuf::from("/dev/ttyUSB0"),
            limits: Limits { max_force: 100.0 },
        }
    );

    // A snapshot is taken when it is asked for, not when the set was declared.
    params.max_speed.set(2.5).unwrap();
    assert_eq!(params.snapshot().max_speed, 2.5);
}

/// A field name is the parameter name, and a nested set's field name is its namespace, so the
/// struct has the same shape as the parameter file it is configured from.
#[test]
fn test_parameter_names_mirror_the_struct() {
    let node = node("names");
    let _params = node.declare_parameters::<DriveConfig>().unwrap();
    let undeclared = node.use_undeclared_parameters();

    assert_eq!(undeclared.get::<f64>("max_speed"), Some(1.5));
    assert_eq!(undeclared.get::<f64>("limits.max_force"), Some(100.0));
    assert_eq!(
        undeclared.get::<Arc<str>>("device").as_deref(),
        Some("/dev/ttyUSB0")
    );
}

// ---------------------------------------------------------------------------------------------
// Nested YAML
// ---------------------------------------------------------------------------------------------

#[test]
fn test_nested_yaml_maps_onto_nested_structs() {
    let (node, _file) = node_with_params(
        "drive_controller",
        r#"
/drive_controller:
  ros__parameters:
    max_speed: 2.0
    wheels: ["fl", "fr"]
    watchdog: 0.5
    device: /dev/ttyACM0
    limits:
      max_force: 180.0
"#,
    );

    let config: DriveConfig = node.load_parameters().unwrap();
    assert_eq!(
        config,
        DriveConfig {
            max_speed: 2.0,
            wheels: vec!["fl".to_string(), "fr".to_string()],
            watchdog: Some(Duration::from_millis(500)),
            device: PathBuf::from("/dev/ttyACM0"),
            limits: Limits { max_force: 180.0 },
        }
    );
}

/// A parameter file only has to say what it wants to change.
#[test]
fn test_partial_yaml_leaves_the_rest_at_their_defaults() {
    let (node, _file) = node_with_params(
        "drive_controller",
        r#"
/drive_controller:
  ros__parameters:
    limits:
      max_force: 42.0
"#,
    );

    let config: DriveConfig = node.load_parameters().unwrap();
    assert_eq!(config.limits.max_force, 42.0);
    assert_eq!(config.max_speed, 1.5);
}

// ---------------------------------------------------------------------------------------------
// Namespacing
// ---------------------------------------------------------------------------------------------

#[derive(ParameterSet)]
#[parameters(namespace = "drive")]
struct NamespacedConfig {
    #[param(default = 1.0)]
    speed: f64,
}

/// A set declares at the node's root unless it says otherwise, a prefix is prepended to whatever
/// namespace it has, and a set with no namespace of its own takes the prefix alone.
#[test]
fn test_namespaces_and_prefixes() {
    assert_eq!(<NamespacedConfig as ParameterSet>::NAMESPACE, "drive");
    assert_eq!(<Limits as ParameterSet>::NAMESPACE, "");

    let node = node("namespaced");
    let _root = node.declare_parameters::<NamespacedConfig>().unwrap();
    let _prefixed = node
        .declare_parameters_with_prefix::<NamespacedConfig>("front")
        .unwrap();
    let _prefixed_root = node
        .declare_parameters_with_prefix::<Limits>("arm")
        .unwrap();

    let undeclared = node.use_undeclared_parameters();
    assert_eq!(undeclared.get::<f64>("drive.speed"), Some(1.0));
    assert_eq!(undeclared.get::<f64>("front.drive.speed"), Some(1.0));
    assert_eq!(undeclared.get::<f64>("arm.max_force"), Some(100.0));
}

#[derive(ParameterSet)]
struct FlattenedConfig {
    #[param(default = 1.0)]
    speed: f64,
    /// Declared without a namespace of its own.
    #[param(flatten)]
    limits: Limits,
}

#[test]
fn test_flatten_declares_a_nested_set_without_a_namespace() {
    let node = node("flattened");
    let params = node.declare_parameters::<FlattenedConfig>().unwrap();

    assert_eq!(params.limits.max_force.get(), 100.0);
    let undeclared = node.use_undeclared_parameters();
    assert_eq!(undeclared.get::<f64>("max_force"), Some(100.0));
    assert_eq!(undeclared.get::<f64>("limits.max_force"), None);
}

#[derive(ParameterSet)]
struct RenamedConfig {
    /// The ROS 2 name need not be the field name.
    #[param(rename = "max-speed", default = 1.0)]
    max_speed: f64,
    #[param(rename = "safety", default = 2.0)]
    limits_multiplier: f64,
}

#[test]
fn test_rename() {
    let node = node("renamed");
    let params = node.declare_parameters::<RenamedConfig>().unwrap();
    assert_eq!(params.max_speed.get(), 1.0);

    let undeclared = node.use_undeclared_parameters();
    assert_eq!(undeclared.get::<f64>("max-speed"), Some(1.0));
    assert_eq!(undeclared.get::<f64>("safety"), Some(2.0));
    assert_eq!(undeclared.get::<f64>("max_speed"), None);
}

// ---------------------------------------------------------------------------------------------
// Field options
// ---------------------------------------------------------------------------------------------

#[derive(Debug, Default, PartialEq)]
struct Cache {
    hits: u32,
}

#[derive(ParameterSet, Debug, PartialEq)]
struct WithSkipped {
    #[param(default = 1.0)]
    speed: f64,
    /// Not a parameter.
    #[param(skip)]
    cache: Cache,
}

#[test]
fn test_skip_leaves_a_field_out_of_the_parameters() {
    let node = node("skipped");
    let params = node.declare_parameters::<WithSkipped>().unwrap();

    assert_eq!(node.use_undeclared_parameters().get::<f64>("cache"), None);
    assert_eq!(
        params.snapshot(),
        WithSkipped {
            speed: 1.0,
            cache: Cache { hits: 0 },
        }
    );
}

#[derive(ParameterSet)]
struct RangedConfig {
    /// An exclusive range, which the builder's own conversions accept for an integer.
    #[param(default = 5, range = 0..10)]
    exclusive: i64,
    /// An exclusive range with a step, which the macro converts and then adds the step to.
    #[param(default = 50, range = 0..101, step = 5)]
    stepped: i64,
    #[param(default = 8080, range = 1024..)]
    open_ended: u16,
    #[param(default = 0.5, range = ..=1.0)]
    upper_bounded: f64,
}

#[test]
fn test_ranges() {
    let node = node("ranged");
    let params = node.declare_parameters::<RangedConfig>().unwrap();

    // Inclusive range with a step.
    assert!(params.stepped.set(55).is_ok());
    assert!(params.stepped.set(57).is_err(), "not on a step boundary");
    assert!(params.stepped.set(105).is_err(), "above the range");

    // Open upper bound.
    assert!(params.open_ended.set(1023u16).is_err());
    assert!(params.open_ended.set(60000u16).is_ok());

    // Open lower bound.
    assert!(params.upper_bounded.set(-100.0).is_ok());
    assert!(params.upper_bounded.set(1.5).is_err());

    // Exclusive of its end, so the last value it admits is one below.
    assert!(params.exclusive.set(9).is_ok());
    assert!(params.exclusive.set(10).is_err());
}

fn must_be_even(value: &i64) -> Result<(), String> {
    if value % 2 == 0 {
        Ok(())
    } else {
        Err(format!("{value} is not even"))
    }
}

#[derive(ParameterSet)]
struct ValidatedConfig {
    #[param(default = 2, validate = must_be_even)]
    even: i64,
}

#[test]
fn test_validate_rejects_bad_values() {
    let node = node("validated");
    let params = node.declare_parameters::<ValidatedConfig>().unwrap();

    assert!(params.even.set(4).is_ok());
    let err = params.even.set(5).unwrap_err();
    assert!(
        matches!(&err, ParameterValueError::ValidationFailed(reason) if reason.contains("not even")),
        "unexpected error: {err}"
    );
    assert_eq!(params.even.get(), 4);
}

/// A `validate` that rejects the initial value fails the declaration, naming the parameter.
#[derive(ParameterSet)]
struct BadlyValidatedConfig {
    #[param(default = 3, validate = must_be_even)]
    even: i64,
}

#[test]
fn test_validate_runs_on_the_initial_value() {
    let node = node("badly_validated");
    let err = node
        .declare_parameters::<BadlyValidatedConfig>()
        .err()
        .unwrap();
    assert_eq!(err.name, "even");
    assert!(matches!(
        err.source,
        DeclarationError::InitialValueRejected(_)
    ));
}

/// A `read_only` parameter takes its one value from an override, so `validate` still has
/// something to check and its reason is what the declaration fails with.
#[derive(ParameterSet)]
struct ValidatedReadOnly {
    #[param(read_only, default = 2, validate = must_be_even)]
    even: i64,
}

#[test]
fn test_validate_checks_a_read_only_override() {
    let (rejected, _file) = node_with_params(
        "read_only_rejected",
        r#"
/read_only_rejected:
  ros__parameters:
    even: 7
"#,
    );
    let err = rejected
        .declare_parameters::<ValidatedReadOnly>()
        .err()
        .unwrap();
    assert_eq!(err.name, "even");
    assert!(
        matches!(
            &err.source,
            DeclarationError::InitialValueRejected(reason) if reason.contains("not even")
        ),
        "unexpected error: {}",
        err.source
    );

    let (accepted, _file) = node_with_params(
        "read_only_accepted",
        r#"
/read_only_accepted:
  ros__parameters:
    even: 8
"#,
    );
    let params = accepted.declare_parameters::<ValidatedReadOnly>().unwrap();
    assert_eq!(params.even.get(), 8);
}

/// `constraints` is free text that nothing enforces but `ros2 param describe` reports, for
/// rules a range cannot express.
#[derive(ParameterSet)]
struct ConstrainedConfig {
    #[param(default = 1, constraints = "must divide the control period")]
    divisor: i64,
}

#[test]
fn test_constraints_reach_the_descriptor() {
    let node = node("constrained");
    let _params = node.declare_parameters::<ConstrainedConfig>().unwrap();

    assert_eq!(
        parameter_descriptor(&node, "divisor")
            .additional_constraints
            .to_string(),
        "must divide the control period"
    );
}

/// A prior value of the wrong type fails the declaration by default.
/// `discard_mismatching_prior_value` asks for the default instead.
#[derive(ParameterSet)]
struct DiscardingPrior {
    #[param(default = 1.0, discard_mismatching_prior_value)]
    speed: f64,
}

#[derive(ParameterSet)]
struct KeepingPrior {
    #[param(default = 1.0)]
    speed: f64,
}

#[test]
fn test_discard_mismatching_prior_value() {
    let node = node("discarding");
    // An integer where the set expects a double.
    node.use_undeclared_parameters().set("speed", 5i64).unwrap();

    let err = node.declare_parameters::<KeepingPrior>().err().unwrap();
    assert_eq!(err.name, "speed");
    assert_eq!(err.source, DeclarationError::PriorValueTypeMismatch);

    let params = node.declare_parameters::<DiscardingPrior>().unwrap();
    assert_eq!(params.speed.get(), 1.0);
}

// Don't use in any other test because tests run in parallel
static SEEN: Mutex<Vec<f64>> = Mutex::new(Vec::new());

fn record(value: &f64) {
    SEEN.lock().unwrap().push(*value);
}

#[derive(ParameterSet)]
struct WatchedConfig {
    #[param(default = 1.0, on_change = record)]
    speed: f64,
}

#[test]
fn test_on_change_runs_after_a_change() {
    let node = node("watched");
    let params = node.declare_parameters::<WatchedConfig>().unwrap();

    SEEN.lock().unwrap().clear();
    params.speed.set(2.0).unwrap();
    params.speed.set(3.0).unwrap();
    assert_eq!(*SEEN.lock().unwrap(), vec![2.0, 3.0]);
}

fn always_ten(_available: AvailableValues<i64>) -> Option<i64> {
    Some(10)
}

#[derive(ParameterSet)]
struct DiscriminatedConfig {
    #[param(default = 1, discriminate = always_ten)]
    chosen: i64,
}

#[test]
fn test_discriminate_chooses_the_initial_value() {
    let node = node("discriminated");
    let params = node.declare_parameters::<DiscriminatedConfig>().unwrap();
    assert_eq!(params.chosen.get(), 10);
}

#[derive(ParameterSet)]
struct IgnoringOverrides {
    #[param(default = 1.0, ignore_override)]
    speed: f64,
}

#[test]
fn test_ignore_override() {
    let (node, _file) = node_with_params(
        "ignoring",
        r#"
/ignoring:
  ros__parameters:
    speed: 99.0
"#,
    );
    let params = node.declare_parameters::<IgnoringOverrides>().unwrap();
    assert_eq!(params.speed.get(), 1.0);
}

/// A field's doc comment becomes the parameter's description, so the two cannot drift. An
/// explicit `description` wins where one is given.
#[test]
fn test_descriptions_come_from_doc_comments() {
    let node = node("described");
    let _params = node.declare_parameters::<DriveConfig>().unwrap();
    let _described = node.declare_parameters::<DescribedConfig>().unwrap();

    assert_eq!(
        parameter_descriptor(&node, "max_speed")
            .description
            .to_string(),
        "Maximum forward speed in m/s."
    );
    assert_eq!(
        parameter_descriptor(&node, "limits.max_force")
            .description
            .to_string(),
        "Maximum motor force in N."
    );
    assert_eq!(
        parameter_descriptor(&node, "speed").description.to_string(),
        "An explicit description."
    );
}

#[derive(ParameterSet)]
struct DescribedConfig {
    /// This doc comment is not the description.
    #[param(default = 1.0, description = "An explicit description.")]
    speed: f64,
}

// ---------------------------------------------------------------------------------------------
// Defaults from a whole-struct value
// ---------------------------------------------------------------------------------------------

#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(default = Self::default())]
struct DefaultedConfig {
    /// No `default` of its own: comes from `Default`.
    speed: f64,
    /// Also from `Default`.
    limits: DefaultedLimits,
    /// More specific than the set's own `Default`, so this wins over it.
    #[param(default = 7.0)]
    explicit: f64,
    /// `Default` gives `None`, which means no default.
    optional: Option<f64>,
}

impl Default for DefaultedConfig {
    fn default() -> Self {
        Self {
            speed: 3.0,
            limits: DefaultedLimits { max_force: 4.0 },
            explicit: 999.0,
            optional: None,
        }
    }
}

#[derive(ParameterSet, Debug, PartialEq)]
#[parameters(default = Self::default())]
struct DefaultedLimits {
    max_force: f64,
}

impl Default for DefaultedLimits {
    fn default() -> Self {
        Self { max_force: 5.0 }
    }
}

#[test]
fn test_defaults_can_come_from_a_default_impl() {
    let node = node("defaulted");
    let config: DefaultedConfig = node.load_parameters().unwrap();

    assert_eq!(config.speed, 3.0);
    // The parent's Default supplies the nested set's values, in preference to the nested
    // struct's own Default.
    assert_eq!(config.limits.max_force, 4.0);
    // Both the set's own `Default` and the field attribute describe the type in general, so the
    // more specific of the two wins. `Default` supplies 999.0 and the field attribute 7.0.
    assert_eq!(config.explicit, 7.0);
    assert_eq!(config.optional, None);
}

/// A parent supplying a default for a nested set overrides the nested fields' own defaults,
/// which is how it configures a set it did not write.
#[derive(ParameterSet, Debug, PartialEq)]
struct ParentSuppliedDefaults {
    #[param(default = Limits { max_force: 250.0 })]
    limits: Limits,
}

#[test]
fn test_a_parent_can_supply_a_nested_sets_defaults() {
    let node = node("parent_supplied");
    let config: ParentSuppliedDefaults = node.load_parameters().unwrap();
    assert_eq!(config.limits.max_force, 250.0);
}

#[test]
fn test_a_nested_set_falls_back_to_its_own_default() {
    let node = node("defaulted_alone");
    let limits: DefaultedLimits = node.load_parameters().unwrap();
    assert_eq!(limits.max_force, 5.0);
}

// ---------------------------------------------------------------------------------------------
// Declaring, retaining and loading
// ---------------------------------------------------------------------------------------------

#[test]
fn test_dropping_the_handles_undeclares_the_parameters() {
    let node = node("dropped");
    {
        let _params = node.declare_parameters::<Limits>().unwrap();
        assert_eq!(
            node.use_undeclared_parameters().get::<f64>("max_force"),
            Some(100.0)
        );
    }
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("max_force"),
        None
    );
    // And so the set can be declared again.
    let _params = node.declare_parameters::<Limits>().unwrap();
}

/// Retained parameters stay declared once the handles are gone, and until then the handles keep
/// reporting whatever the parameter currently holds.
#[test]
fn test_retain_parameters() {
    let node = node("retained");
    let params = node.retain_parameters::<Limits>().unwrap();
    assert_eq!(params.max_force.get(), 100.0);

    // A change made elsewhere is visible through a handle that is still held.
    node.use_undeclared_parameters()
        .set::<f64>("max_force", 42.0)
        .unwrap();
    assert_eq!(params.snapshot().max_force, 42.0);

    drop(params);
    // Unlike `declare_parameters`, dropping the handles leaves the declaration in place.
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("max_force"),
        Some(42.0)
    );
}

#[test]
fn test_load_parameters_keeps_the_declarations() {
    let node = node("loaded");
    let limits: Limits = node.load_parameters().unwrap();
    assert_eq!(limits.max_force, 100.0);
    // Still declared, so still visible to `ros2 param` and the parameter services.
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("max_force"),
        Some(100.0)
    );

    let prefixed: Limits = node.load_parameters_with_prefix("arm").unwrap();
    assert_eq!(prefixed.max_force, 100.0);
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("arm.max_force"),
        Some(100.0)
    );
}

// ---------------------------------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------------------------------

#[derive(ParameterSet)]
struct RequiredConfig {
    /// No default and no override: there is nothing to declare it with.
    required: f64,
    nested: RequiredNested,
}

#[derive(ParameterSet)]
struct RequiredNested {
    also_required: f64,
}

/// An error names the parameter that failed, by the full name it was declared under: on its own,
/// nested under its set, or under a prefix.
#[test]
fn test_declaration_error_names_the_parameter() {
    let plain = node("required");
    let err = plain.declare_parameters::<RequiredConfig>().err().unwrap();
    assert_eq!(err.name, "required");
    assert_eq!(err.source, DeclarationError::NoValueAvailable);
    assert!(
        err.to_string()
            .contains("failed to declare parameter 'required'"),
        "unexpected message: {err}"
    );

    // Nested, once the outer field has a value and the inner one is what fails.
    let (nested_node, _file) = node_with_params(
        "required_nested",
        r#"
/required_nested:
  ros__parameters:
    required: 1.0
"#,
    );
    let err = nested_node
        .declare_parameters::<RequiredConfig>()
        .err()
        .unwrap();
    assert_eq!(err.name, "nested.also_required");

    // And under a prefix.
    let prefixed_node = node("required_prefixed");
    let err = prefixed_node
        .declare_parameters_with_prefix::<RequiredConfig>("robot")
        .err()
        .unwrap();
    assert_eq!(err.name, "robot.required");
}

// ---------------------------------------------------------------------------------------------
// Ergonomic value types inside a set
// ---------------------------------------------------------------------------------------------

#[derive(ParameterSet, Debug, PartialEq)]
struct ManyTypes {
    #[param(default = true)]
    flag: bool,
    #[param(default = 5)]
    count: i64,
    #[param(default = 3)]
    small: u16,
    #[param(default = 1.5)]
    ratio: f32,
    #[param(default = "hello")]
    text: String,
    #[param(default = "/tmp")]
    path: PathBuf,
    #[param(default = [1, 2, 3])]
    numbers: Vec<i64>,
    /// An array of a narrower type. Its elements are checked against the range of a `u16`.
    #[param(default = [8080, 9090])]
    ports: Vec<u16>,
    #[param(default = ["a", "b"])]
    names: Vec<String>,
    #[param(convert = seconds(), default = Duration::from_millis(250))]
    timeout: Duration,
    maybe: Option<i64>,
}

#[test]
fn test_a_set_of_many_value_types() {
    let node = node("many_types");
    let values: ManyTypes = node.load_parameters().unwrap();

    assert_eq!(
        values,
        ManyTypes {
            flag: true,
            count: 5,
            small: 3,
            ratio: 1.5,
            text: "hello".to_string(),
            path: PathBuf::from("/tmp"),
            numbers: vec![1, 2, 3],
            ports: vec![8080, 9090],
            names: vec!["a".to_string(), "b".to_string()],
            timeout: Duration::from_millis(250),
            maybe: None,
        }
    );
}

/// A range works for any type whose `ParameterVariant::Range` is a real range, not just the ones
/// the macro recognises.
#[derive(Clone, Debug, PartialEq)]
struct Meters(f64);

impl From<Meters> for ParameterValue {
    fn from(value: Meters) -> Self {
        ParameterValue::Double(value.0)
    }
}

impl TryFrom<ParameterValue> for Meters {
    type Error = ParameterValueError;
    fn try_from(value: ParameterValue) -> Result<Self, Self::Error> {
        match value {
            ParameterValue::Double(v) => Ok(Meters(v)),
            _ => Err(ParameterValueError::TypeMismatch),
        }
    }
}

impl ParameterVariant for Meters {
    // Expressed in the units of the wrapped type, so the bounds are written as plain `f64`.
    type Range = ParameterRange<f64>;

    fn kind() -> ParameterKind {
        ParameterKind::Double
    }
}

declare_parameter_field!(Meters);

#[derive(ParameterSet)]
struct RangedNewtype {
    #[param(default = Meters(1.0), range = 0.0..=10.0)]
    depth: Meters,
}

#[test]
fn test_range_on_a_user_defined_type() {
    let node = node("ranged_newtype");
    let params = node.declare_parameters::<RangedNewtype>().unwrap();

    assert!(params.depth.set(Meters(9.0)).is_ok());
    assert!(params.depth.set(Meters(11.0)).is_err());
    assert_eq!(params.depth.get(), Meters(9.0));

    // The range reaches the descriptor, so `ros2 param describe` reports it.
    let descriptor = parameter_descriptor(&node, "depth");
    assert_eq!(descriptor.floating_point_range[0].from_value, 0.0);
    assert_eq!(descriptor.floating_point_range[0].to_value, 10.0);
}

/// A field whose type cannot implement `ParameterVariant`, because both the trait and the type
/// belong to other crates. Saying how it is represented is enough, and the handle and the
/// snapshot are in the field's own type.
fn seconds() -> ParameterConversion<Duration> {
    ParameterConversion::double(Duration::as_secs_f64, Duration::try_from_secs_f64)
}

#[derive(ParameterSet, Debug, PartialEq)]
struct ConvertedConfig {
    #[param(convert = seconds(), default = Duration::from_millis(500))]
    timeout: Duration,
    #[param(convert = seconds())]
    grace: Option<Duration>,
    #[param(convert = seconds(), read_only, default = Duration::from_secs(30))]
    lifetime: Duration,
}

#[test]
fn test_a_field_can_carry_its_own_conversion() {
    let node = node("converted");
    let params = node.declare_parameters::<ConvertedConfig>().unwrap();

    assert_eq!(params.timeout.get(), Duration::from_millis(500));
    assert_eq!(params.grace.get(), None);
    assert_eq!(params.lifetime.get(), Duration::from_secs(30));

    params.timeout.set(Duration::from_secs(2)).unwrap();
    assert_eq!(
        params.snapshot(),
        ConvertedConfig {
            timeout: Duration::from_secs(2),
            grace: None,
            lifetime: Duration::from_secs(30),
        }
    );

    // What is stored is the representation, so a parameter file writes seconds as a double.
    assert_eq!(
        node.use_undeclared_parameters().get::<f64>("timeout"),
        Some(2.0)
    );
}

/// A parameter file configures a converted field in the units the conversion stores, and a range
/// on such a field is in those units too.
#[derive(ParameterSet, Debug, PartialEq)]
struct BoundedConverted {
    #[param(convert = seconds(), default = Duration::from_secs(1), range = 0.0..=5.0)]
    timeout: Duration,
}

#[test]
fn test_a_converted_field_takes_a_range_and_an_override() {
    let (node, _file) = node_with_params(
        "converted_override",
        r#"
/converted_override:
  ros__parameters:
    timeout: 3.5
"#,
    );
    let params = node.declare_parameters::<BoundedConverted>().unwrap();
    assert_eq!(params.timeout.get(), Duration::from_millis(3500));

    assert!(params.timeout.set(Duration::from_secs(4)).is_ok());
    assert!(params.timeout.set(Duration::from_secs(6)).is_err());
}

/// A type alias hides the field's type from the macro, so trait resolution decides what the
/// field is.
type Speed = f64;

#[derive(ParameterSet)]
struct AliasedConfig {
    #[param(default = 1.0, range = 0.0..=2.0)]
    speed: Speed,
}

#[test]
fn test_type_aliases_work() {
    let node = node("aliased");
    let params = node.declare_parameters::<AliasedConfig>().unwrap();
    assert_eq!(params.speed.get(), 1.0);
    assert!(params.speed.set(3.0).is_err());
}

/// Visibility is inherited, so a set can be part of a crate's public API.
pub mod public {
    use super::*;

    /// A publicly visible set has publicly visible handles.
    #[derive(ParameterSet)]
    pub struct PublicConfig {
        #[param(default = 1.0)]
        pub speed: f64,
    }
}

#[derive(ParameterSet)]
#[parameters(handles = CustomHandles)]
struct CustomHandlesConfig {
    #[param(default = 1.0)]
    speed: f64,
}

/// The type annotations do the checking here: naming `public::PublicConfigParams` from outside
/// its module only compiles if visibility was inherited, and `CustomHandles` only if `handles`
/// was honoured.
#[test]
fn test_generated_handles_struct() {
    let node = node("handles");
    let params: public::PublicConfigParams =
        node.declare_parameters::<public::PublicConfig>().unwrap();
    assert_eq!(params.speed.get(), 1.0);

    let custom: CustomHandles = node
        .declare_parameters_with_prefix::<CustomHandlesConfig>("custom")
        .unwrap();
    assert_eq!(custom.speed.get(), 1.0);
}
