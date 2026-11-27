// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

#![cfg(feature = "rosidl-buffer")]

use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use rclrs::{Context, CreateBasicExecutor, SpinOptions};
use ros_env::{rcl_interfaces, sensor_msgs};

#[test]
fn one_cpu_publication_reaches_both_representations() {
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

#[test]
fn cpu_and_buffer_clients_receive_buffer_action_feedback_and_results() {
    use rclrs::GoalStatusCode;
    use ros_env::example_interfaces::action;

    let mut executor = Context::default().create_basic_executor();
    let server_node = executor.create_node("buffer_action_server_test").unwrap();
    let client_node = executor.create_node("buffer_action_client_test").unwrap();
    let name = format!("buffer_action_cpu_{}", std::process::id());
    let _server = server_node
        .create_action_server::<action::buffer::Fibonacci, _>(&name, |requested| async move {
            assert_eq!(requested.goal().order, 5);
            let executing = requested.accept().execute();
            executing.publish_feedback(action::buffer::Fibonacci_Feedback {
                sequence: vec![1, 1, 2].into(),
            });
            executing.succeeded_with(action::buffer::Fibonacci_Result {
                sequence: vec![1, 1, 2, 3, 5].into(),
            })
        })
        .unwrap();
    let cpu_client = client_node
        .create_action_client::<action::Fibonacci>(&name)
        .unwrap();
    let buffer_client = client_node
        .create_action_client::<action::buffer::Fibonacci>(&name)
        .unwrap();
    let deadline = Instant::now() + Duration::from_secs(10);
    while !cpu_client.server_is_available().unwrap()
        || !buffer_client.server_is_available().unwrap()
    {
        assert!(Instant::now() < deadline, "action discovery timed out");
        std::thread::sleep(Duration::from_millis(10));
    }

    // Fibonacci's int32 sequences remain CPU-backed in both representations.
    let cpu_request = cpu_client.request_goal(action::Fibonacci_Goal { order: 5 });
    let buffer_request = buffer_client.request_goal(action::buffer::Fibonacci_Goal { order: 5 });
    let response = executor.commands().run(async move {
        let mut cpu_goal = cpu_request.await.expect("CPU goal was rejected");
        let mut buffer_goal = buffer_request.await.expect("buffer goal was rejected");
        let cpu_feedback = cpu_goal
            .feedback
            .recv()
            .await
            .expect("missing CPU feedback");
        let buffer_feedback = buffer_goal
            .feedback
            .recv()
            .await
            .expect("missing buffer feedback");
        (
            cpu_feedback,
            cpu_goal.result.await,
            buffer_feedback,
            buffer_goal.result.await,
        )
    });
    let (mut response, done) = executor.commands().create_notice(response);
    let errors = executor.spin(
        SpinOptions::default()
            .until_promise_resolved(done)
            .timeout(Duration::from_secs(10)),
    );
    assert!(
        errors.iter().all(rclrs::RclrsError::is_timeout),
        "{errors:?}"
    );
    let (cpu_feedback, (cpu_status, cpu_result), buffer_feedback, (buffer_status, buffer_result)) =
        response
            .try_recv()
            .expect("action task failed")
            .expect("action response timed out");
    assert_eq!(cpu_status, GoalStatusCode::Succeeded);
    assert_eq!(buffer_status, GoalStatusCode::Succeeded);
    assert_eq!(cpu_feedback.sequence, [1, 1, 2]);
    assert_eq!(cpu_result.sequence, [1, 1, 2, 3, 5]);
    assert_eq!(buffer_feedback.sequence, [1, 1, 2]);
    assert_eq!(buffer_result.sequence, [1, 1, 2, 3, 5]);
}
