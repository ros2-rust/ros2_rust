use rclrs::*;
use example_interfaces::srv::*;
use std::sync::{Arc, atomic::{AtomicBool, Ordering}};
use tokio::sync::mpsc;

#[tokio::test]
async fn test_immediate_service() {
    let context = Context::new([]).unwrap();
    let node = create_node(&context, "test_immediate_service").unwrap();
    let _service = node.create_service::<AddTwoInts, _>(
        "add",
        |_, request| AddTwoInts_Response { sum: request.a + request.b },
    ).unwrap();
    let client = node.create_client::<AddTwoInts>("add").unwrap();
    let request = AddTwoInts_Request { a: 41, b: 1 };
    let future = client.call_async(&request);
    let run = Arc::new(AtomicBool::new(true));
    let set_run = run.clone();
    let _spinning = tokio::task::spawn_blocking(
        move || {
            while run.load(Ordering::Relaxed) {
                if let Err(err) = spin_once(
                    node.clone(), Some(std::time::Duration::new(1, 0))
                ) {
                    match err {
                        RclrsError::RclError { code: RclReturnCode::Timeout, .. } => {
                            return;
                        }
                        _ => panic!("{err:?}"),
                    }
                }
            }
        });
    let response = future.await.unwrap();
    assert_eq!(response.sum, 42);
    set_run.store(false, Ordering::Relaxed);
}

#[tokio::test]
async fn test_deferred_service() {
    let context = Context::new([]).unwrap();
    let node = create_node(&context, "test_deferred_service").unwrap();
    let (sender, mut receiver) = mpsc::unbounded_channel::<Payload>();
    tokio::spawn(async move {
        while let Some(payload) = receiver.recv().await {
            let Payload { request, response_sender } = payload;
            let sum = request.a + request.b;
            response_sender.send(AddTwoInts_Response { sum }).unwrap();
        }
    });

    let _service = node.create_deferred_service::<AddTwoInts, _>(
        "add",
        move |request, response_sender| sender.send(Payload { request, response_sender }).unwrap(),
    ).unwrap();

    let client = node.create_client::<AddTwoInts>("add").unwrap();
    let request = AddTwoInts_Request { a: 41, b: 1 };
    let future = client.call_async(&request);
    let run = Arc::new(AtomicBool::new(true));
    let set_run = run.clone();
    let _spinning = tokio::task::spawn_blocking(
        move || {
            while run.load(Ordering::Relaxed) {
                if let Err(err) = spin_once(
                    node.clone(), Some(std::time::Duration::new(1, 0))
                ) {
                    match err {
                        RclrsError::RclError { code: RclReturnCode::Timeout, .. } => {
                            return;
                        }
                        _ => panic!("{err:?}"),
                    }
                }
            }
        });
    let response = future.await.unwrap();
    assert_eq!(response.sum, 42);
    set_run.store(false, Ordering::Relaxed);
}

struct Payload {
    request: AddTwoInts_Request,
    response_sender: ResponseSender<AddTwoInts_Response>,
}
