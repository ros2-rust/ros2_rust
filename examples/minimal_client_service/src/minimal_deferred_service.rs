use std::{env, sync::mpsc::{self, Receiver}};
use anyhow::{Error, Result};
use example_interfaces::srv::{AddTwoInts, AddTwoInts_Request, AddTwoInts_Response};

struct Payload {
    request: AddTwoInts_Request,
    response_sender: rclrs::ResponseSender<AddTwoInts_Response>,
}

fn worker_thread(receiver: Receiver<Payload>) {
    loop {
        let Ok(Payload { request, response_sender }) = receiver.recv() else {
            return;
        };
        let sum = request.a + request.b;
        if let Err(err) = response_sender.send(AddTwoInts_Response { sum }) {
            println!("Error while sending result: {err:?}");
        }
    }
}

fn main() -> Result<(), Error> {
    let context = rclrs::Context::new(env::args())?;

    let node = rclrs::create_node(&context, "minimal_deferred_service")?;
    let (sender, receiver) = mpsc::channel();
    let _worker = std::thread::spawn(move || worker_thread(receiver));

    let _server = node
        .create_deferred_service::<AddTwoInts, _>(
            "add_two_ints",
            move |request, response_sender| {
                sender.send(Payload { request, response_sender }).ok();
            }
        );
    rclrs::spin(node).map_err(|err| err.into())
}
