use std::borrow::Cow;
use std::process::Command;
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use ros_env::{rcl_interfaces, sensor_msgs, std_msgs, test_msgs};
use rosidl_runtime_rs::{Action, BoundedSequence, BufferError, Message, RmwMessage, Service};

#[test]
fn cpu_fields_and_ros_identity_are_preserved() {
    type Cpu = sensor_msgs::msg::Image;
    type Portable = sensor_msgs::msg::buffer::Image;
    let mut cpu = Cpu {
        height: 1,
        width: 3,
        encoding: "mono8".into(),
        step: 3,
        data: vec![1, 2, 3],
        ..Default::default()
    };
    cpu.data[1] = 7;
    let copied = cpu.clone();
    let portable = Portable::from(cpu);
    assert_eq!(portable.data.backend_name().unwrap(), "cpu");
    assert_eq!(portable.clone().try_into_cpu().unwrap(), copied);
    assert_eq!(
        std::any::TypeId::of::<<Cpu as Message>::RmwMsg>(),
        std::any::TypeId::of::<<Portable as Message>::RmwMsg>()
    );
    assert_eq!(
        <Cpu as Message>::RmwMsg::get_type_support(),
        <Portable as Message>::RmwMsg::get_type_support()
    );
    let mut bounded = test_msgs::msg::BoundedSequences::default();
    let legacy: BoundedSequence<u8, 3> = vec![1, 2].try_into().unwrap();
    bounded.uint8_values = legacy;
    let portable = test_msgs::msg::buffer::BoundedSequences::from(bounded.clone());
    assert_eq!(portable.try_into_cpu().unwrap(), bounded);
}

#[test]
fn generated_service_and_action_representations_share_type_support() {
    assert_eq!(
        <test_msgs::srv::Arrays as Service>::get_type_support(),
        <test_msgs::srv::buffer::Arrays as Service>::get_type_support()
    );
    let request = test_msgs::srv::Arrays_Request::default();
    assert_eq!(
        test_msgs::srv::buffer::Arrays_Request::from(request.clone())
            .try_into_cpu()
            .unwrap(),
        request
    );
    assert_eq!(
        <test_msgs::action::Fibonacci as Action>::get_type_support(),
        <test_msgs::action::buffer::Fibonacci as Action>::get_type_support()
    );
    let result = test_msgs::action::Fibonacci_Result {
        sequence: vec![1, 2, 3],
    };
    let result = test_msgs::action::buffer::Fibonacci_Result::from(result);
    assert_eq!(result.sequence.to_vec().unwrap(), vec![1, 2, 3]);
    let response = <test_msgs::action::buffer::Fibonacci as Action>::create_result_response(
        4,
        test_msgs::action::buffer::Fibonacci_Result::into_rmw_message(Cow::Owned(result))
            .into_owned(),
    );
    let (status, native) =
        <test_msgs::action::buffer::Fibonacci as Action>::split_result_response(response);
    assert_eq!(status, 4);
    assert_eq!(
        test_msgs::action::Fibonacci_Result::try_from_rmw_message(native)
            .unwrap()
            .sequence,
        vec![1, 2, 3]
    );
}

#[cfg(feature = "serde")]
#[test]
fn cpu_and_buffer_json_have_the_same_schema() {
    let image = sensor_msgs::msg::Image {
        data: vec![1, 2, 3],
        ..Default::default()
    };
    let json = serde_json::to_string(&image).unwrap();
    let portable: sensor_msgs::msg::buffer::Image = serde_json::from_str(&json).unwrap();
    assert_eq!(serde_json::to_string(&portable).unwrap(), json);
}

fn publish_cpu_image_once() {
    let mut executor = Context::default().create_basic_executor();
    let node = executor.create_node("portable_image_test").unwrap();
    let topic = format!("portable_image_cpu_{}", std::process::id());
    let received = Arc::new(Mutex::new((None, None)));
    let cpu_received = Arc::clone(&received);
    let _cpu = node
        .create_subscription::<sensor_msgs::msg::Image, _>(
            &topic,
            move |image: sensor_msgs::msg::Image| {
                assert_eq!(image.width, 3);
                cpu_received.lock().unwrap().0 = Some(image.data);
            },
        )
        .unwrap();
    let buffer_received = Arc::clone(&received);
    let _portable = node
        .create_subscription::<sensor_msgs::msg::buffer::Image, _>(
            rclrs::SubscriptionOptions::new(&topic).acceptable_buffer_backends("cpu"),
            move |image: sensor_msgs::msg::buffer::Image| {
                let name = image.data.backend_name().unwrap();
                buffer_received.lock().unwrap().1 = Some((name, image.data.to_vec().unwrap()));
            },
        )
        .unwrap();
    let publisher = node
        .create_publisher::<sensor_msgs::msg::buffer::Image>(&topic)
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while publisher.get_subscription_count().unwrap() < 2 {
        assert!(Instant::now() < deadline, "subscriber discovery timed out");
        std::thread::sleep(Duration::from_millis(10));
    }
    let image = sensor_msgs::msg::buffer::Image {
        width: 3,
        data: vec![11, 22, 33].into(),
        ..Default::default()
    };
    publisher.publish(image).unwrap();
    loop {
        let errors = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));
        assert!(
            errors.iter().all(rclrs::RclrsError::is_timeout),
            "{errors:?}"
        );
        let results = received.lock().unwrap();
        if let (Some(cpu), Some((name, portable))) = &*results {
            assert_eq!(cpu, &[11, 22, 33]);
            assert_eq!(portable, cpu);
            assert_eq!(name, "cpu");
            break;
        }
        assert!(
            Instant::now() < deadline,
            "one publication did not reach both representations: {results:?}"
        );
    }
}

#[test]
fn one_cpu_publication_reaches_both_representations() {
    publish_cpu_image_once();
}

#[test]
fn cpu_delivery_without_a_visible_gpu() {
    const CHILD: &str = "ROSIDL_BUFFER_CPU_CHILD";
    if std::env::var_os(CHILD).is_some() {
        publish_cpu_image_once();
        return;
    }
    let mut child = Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "cpu_delivery_without_a_visible_gpu",
            "--nocapture",
        ])
        .env(CHILD, "1")
        .env("CUDA_VISIBLE_DEVICES", "-1")
        .spawn()
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(30);
    loop {
        if let Some(status) = child.try_wait().unwrap() {
            assert!(status.success());
            break;
        }
        if Instant::now() >= deadline {
            let _ = child.kill();
            let _ = child.wait();
            panic!("GPU-disabled subprocess timed out");
        }
        std::thread::sleep(Duration::from_millis(20));
    }
}

#[test]
fn conversion_failures_reach_the_executor_without_calling_the_callback() {
    #[derive(Clone, Debug, Default)]
    struct Reject;
    impl Message for Reject {
        type RmwMsg = std_msgs::msg::rmw::UInt8MultiArray;
        fn into_rmw_message(_: Cow<'_, Self>) -> Cow<'_, Self::RmwMsg> {
            Cow::Owned(Default::default())
        }
        fn from_rmw_message(_: Self::RmwMsg) -> Self {
            panic!("infallible conversion used")
        }
        fn try_from_rmw_message(_: Self::RmwMsg) -> Result<Self, BufferError> {
            Err(BufferError::Native {
                operation: "test conversion",
                code: -42,
            })
        }
    }
    let mut executor = Context::default().create_basic_executor();
    let node = executor.create_node("portable_conversion_failure").unwrap();
    let topic = format!("portable_conversion_failure_{}", std::process::id());
    let _subscription = node
        .create_subscription::<Reject, _>(&topic, |_: Reject| {
            panic!("failed conversion reached callback")
        })
        .unwrap();
    let publisher = node
        .create_publisher::<std_msgs::msg::UInt8MultiArray>(&topic)
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while publisher.get_subscription_count().unwrap() < 1 {
        assert!(Instant::now() < deadline);
        std::thread::sleep(Duration::from_millis(10));
    }
    publisher
        .publish(std_msgs::msg::UInt8MultiArray::default())
        .unwrap();
    loop {
        let errors = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));
        if let Some(error) = errors.iter().find(|error| !error.is_timeout()) {
            use std::error::Error;
            assert!(error
                .source()
                .unwrap()
                .to_string()
                .contains("test conversion"));
            return;
        }
        assert!(Instant::now() < deadline, "conversion error was lost");
    }
}

#[test]
fn cpu_client_receives_buffer_service_response() {
    use rcl_interfaces::{msg, srv};
    let mut executor = Context::default().create_basic_executor();
    let node = executor.create_node("portable_service_test").unwrap();
    let name = format!("portable_service_cpu_{}", std::process::id());
    let _service = node
        .create_service::<srv::buffer::GetParameters, _>(
            &name,
            move |request: srv::buffer::GetParameters_Request| {
                assert_eq!(request.names, vec!["pixels"]);
                srv::buffer::GetParameters_Response {
                    values: vec![msg::buffer::ParameterValue {
                        byte_array_value: vec![13, 17, 23].into(),
                        ..Default::default()
                    }],
                }
            },
        )
        .unwrap();
    let client = node.create_client::<srv::GetParameters>(&name).unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while !client.service_is_ready().unwrap() {
        assert!(Instant::now() < deadline, "service discovery timed out");
        std::thread::sleep(Duration::from_millis(10));
    }
    let received = Arc::new(Mutex::new(None));
    let output = Arc::clone(&received);
    let _call = client
        .call_then(
            srv::GetParameters_Request {
                names: vec!["pixels".into()],
            },
            move |response: srv::GetParameters_Response| {
                *output.lock().unwrap() = Some(response.values[0].byte_array_value.clone());
            },
        )
        .unwrap();
    loop {
        let errors = executor.spin(SpinOptions::spin_once().timeout(Duration::from_millis(20)));
        assert!(
            errors.iter().all(rclrs::RclrsError::is_timeout),
            "{errors:?}"
        );
        if let Some(values) = &*received.lock().unwrap() {
            assert_eq!(values, &[13, 17, 23]);
            break;
        }
        assert!(Instant::now() < deadline, "service response timed out");
    }
}
