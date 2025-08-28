use crate::NodeState;

pub enum DefaultForbidden {}

pub struct ParameterOptions<'a, 'b, T> {
    node: &'a crate::NodeState,
    name: &'b str,
    default: Option<T>,
}
pub trait StructuredParametersMeta<T>: Sized {
    fn declare_structured_(
        options: ParameterOptions<T>,
    ) -> core::result::Result<Self, crate::DeclarationError>;
}

pub trait StructuredParameters: Sized {
    fn declare_structured<T>(
        options: ParameterOptions<T>,
    ) -> core::result::Result<Self, crate::DeclarationError>
    where
        Self: StructuredParametersMeta<T>,
    {
        Self::declare_structured_(options)
    }
}
impl<T: crate::ParameterVariant> StructuredParameters for crate::MandatoryParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::MandatoryParameter<T> {
    fn declare_structured_(
        options: ParameterOptions<T>,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = options.node.declare_parameter(options.name);
        let builder = match options.default {
            Some(default) => builder.default(default),
            None => builder,
        };
        builder.mandatory()
    }
}
impl<T: crate::ParameterVariant> StructuredParameters for crate::ReadOnlyParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::ReadOnlyParameter<T> {
    fn declare_structured_(
        options: ParameterOptions<T>,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = options.node.declare_parameter(options.name);
        let builder = match options.default {
            Some(default) => builder.default(default),
            None => builder,
        };
        builder.read_only()
    }
}
impl<T: crate::ParameterVariant> StructuredParameters for crate::OptionalParameter<T> {}
impl<T: crate::ParameterVariant> StructuredParametersMeta<T> for crate::OptionalParameter<T> {
    fn declare_structured_(
        options: ParameterOptions<T>,
    ) -> core::result::Result<Self, crate::DeclarationError> {
        let builder = options.node.declare_parameter(options.name);
        let builder = match options.default {
            Some(default) => builder.default(default),
            None => builder,
        };
        builder.optional()
    }
}

impl NodeState {
    fn declare_parameters<T, T0>(
        &self,
        name: &str,
    ) -> core::result::Result<T, crate::DeclarationError>
    where
        T: StructuredParameters + StructuredParametersMeta<T0>,
    {
        T::declare_structured(ParameterOptions {
            node: self,
            name: name,
            default: None,
        })
    }
}

mod tests {
    use std::sync::Arc;

    use crate as rclrs;
    use rclrs::parameter::structured::*;
    use rclrs::CreateBasicExecutor;
    use rclrs_proc_macros::StructuredParameters;

    #[derive(StructuredParameters, Debug)]
    struct SimpleStructuredParameters {
        _mandatory: rclrs::MandatoryParameter<f64>,
        _optional: rclrs::OptionalParameter<f64>,
        _readonly: rclrs::ReadOnlyParameter<f64>,
    }

    #[test]
    fn test_simple_structured_parameters() {
        let args: Vec<String> = [
            "test",
            "--ros-args",
            "-p",
            "_mandatory:=1.0",
            "-p",
            "_optional:=1.0",
            "-p",
            "_readonly:=1.0",
        ]
        .into_iter()
        .map(str::to_string)
        .collect();

        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: SimpleStructuredParameters = node.declare_parameters("").unwrap();
        println!("{:?}", _params);
    }

    #[derive(StructuredParameters, Debug)]
    struct NestedStructuredParameters {
        _simple: SimpleStructuredParameters,
        _mandatory: rclrs::MandatoryParameter<Arc<str>>,
    }

    #[test]
    fn test_nested_structured_parameters() {
        let args: Vec<String> = [
            "test",
            "--ros-args",
            "-p",
            "nested._simple._mandatory:=1.0",
            "-p",
            "nested._simple._optional:=1.0",
            "-p",
            "nested._simple._readonly:=1.0",
            "-p",
            "nested._mandatory:=foo",
        ]
        .into_iter()
        .map(str::to_string)
        .collect();

        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: NestedStructuredParameters = node.declare_parameters("nested").unwrap();
        println!("{:?}", _params);
    }

    #[derive(Debug, StructuredParameters)]
    struct SimpleStructuredParametersWithDefaults {
        #[param(default = 42.0)]
        _mandatory: rclrs::MandatoryParameter<f64>,
        #[param(default = 42.0)]
        _optional: rclrs::OptionalParameter<f64>,
        #[param(default = Arc::from("test"))]
        _readonly: rclrs::ReadOnlyParameter<Arc<str>>,
    }

    #[test]
    fn test_simple_structured_parameters_with_defaults() {
        let args: Vec<String> = ["test", "--ros-args"]
            .into_iter()
            .map(str::to_string)
            .collect();
        let context = crate::Context::new(args, rclrs::InitOptions::default()).unwrap();
        let exec = context.create_basic_executor();
        let node = exec.create_node(rclrs::NodeOptions::new("test")).unwrap();
        let _params: SimpleStructuredParametersWithDefaults = node.declare_parameters("").unwrap();
        println!("{:?}", _params);
    }
}
