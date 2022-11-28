# Writing your first `rclrs` node

This tutorial is intended to point out some of the differences of the Rust client library with the other client libraries. It assumes knowledge of Rust, and is also not intended to be an introduction to ROS 2.

As a semi-realistic example, let's create a node that periodically republishes the last message it received. It's limited to only one specific message type – `std_msgs/msg/String` in this example.

## Create a package

In ROS 2, `ros2 pkg create` is the standard way of creating packages. However, the functionality to create Rust packages with this tool is not yet implemented, so they need to be created manually.

You can start by creating a package with `cargo` in the usual way:

```console
cargo new republisher_node && cd republisher_node
```

In the `Cargo.toml` file, add a dependency on `rclrs = "*"` and `std_msgs = "*"`.

Additionally, create a new `package.xml` if you want your node to be buildable with `colcon`. Make sure to change the build type to `ament_cargo` and to include the two packages mentioned above in the dependencies, as such:

```xml
<package format="3">
  <name>republisher_node</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclrs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```


## Writing the basic node structure

Since Rust doesn't have inheritance, it's not possible to inherit from `Node` as is common practice in `rclcpp` or `rclpy`.

Instead, you can store the node as a regular member. Let's add a struct that contains the node, a subscription, and a field for the last message that was received to `main.rs`:

```rust
use std::sync::Arc;
use std_msgs::msg::String as StringMsg;

struct RepublisherNode {
    node: rclrs::Node,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    data: Option<StringMsg>,
}

impl RepublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = None;
        let _subscription = node.create_subscription(
            "in_topic",
            rclrs::QOS_PROFILE_DEFAULT,
            |msg: StringMsg| { todo!("Assign msg to self.data") },
        )?;
        Ok(Self {
            node,
            _subscription,
            data,
        })
    }
}
```

Next, add a main function to launch it:

```rust
fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = RepublisherNode::new(&context)?;
    rclrs::spin(republisher.node)
}
```

You should now be able to run this node with `cargo run`. However, the subscription callback still has a `todo!` in it, so it will exit with an error when it receives a message.


## Storing received data in the struct

Let's do something about that `todo!`. The obvious thing for the subscription callback to do would be this:

```rust
|msg: StringMsg| {
    data = Some(msg);
},
```

This is a standard pattern in C++, but doesn't work in Rust. Why not?

Written like this, `data` is *borrowed* by the callback, but `data` is a local variable which only exists in its current form until the end of `RepublisherNode::new()`. The subscription callback is required to not borrow any variables because the subscription, and therefore the callback, could live indefinitely.

> 💡 As an aside, this requirement is expressed by the `'static` bound on the generic parameter `F` for the callback in `Node::create_subscription()`.

You might think "I don't want to borrow from the local variable `data` anyway, I want to borrow from the `data` field in `RepublisherNode`!" and you would be right. That variable lives considerably longer, but also not forever, so it wouldn't be enough. A secondary problem is that it would be a *self-referential struct*, which is not allowed in Rust.

The solution is _shared ownership_ of the data by the callback and the node. The `Arc` type provides shared ownership, but since it only gives out shared references to its data, we also need a `Mutex` or a `RefCell`. This `Arc<Mutex<T>>` type is a frequent pattern in Rust code.

So, to store the received data in the struct, the following things have to change:
1. Import `Mutex`
2. Adjust the type of the `data` field
3. Create two pointers to the same data (wrapped in a `Mutex`)
4. Make the closure `move`, and inside it, lock the `Mutex` and store the message  

```rust
use std::sync::{Arc, Mutex};  // (1)
use std_msgs::msg::String as StringMsg;

struct RepublisherNode {
    node: rclrs::Node,
    _subscription: Arc<rclrs::Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,  // (2)
}

impl RepublisherNode {
    fn new(context: &rclrs::Context) -> Result<Self, rclrs::RclrsError> {
        let node = rclrs::Node::new(context, "republisher")?;
        let data = Arc::new(Mutex::new(None));  // (3)
        let data_cb = Arc::clone(&data);
        let _subscription = {
            // Create a new shared pointer instance that will be owned by the closure
            node.create_subscription(
                "in_topic",
                rclrs::QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    // This subscription now owns the data_cb variable
                    *data_cb.lock().unwrap() = Some(msg);  // (4)
                },
            )?
        };
        Ok(Self {
            node,
            _subscription,
            data,
        })
    }
}
```

If that seems needlessly complicated – maybe it is, in the sense that `rclrs` could potentially introduce new abstractions to improve the ergonomics of this use case. This is to be discussed.

If you couldn't follow the explanation involving borrowing, closures etc. above, an explanation of these concepts is unfortunately out of scope of this tutorial. There are many good Rust books and tutorials that can help you understand these crucial features. The online book [*The Rust Programming Language*](https://doc.rust-lang.org/book/) is a good place to start for most topics.

## Periodically run a republishing function

The node still doesn't republish the received messages. First, let's add a publisher to the node:

```rust
// Add this new field to the RepublisherNode struct, after the subscription:
publisher: rclrs::Publisher<StringMsg>,

// Change the end of RepublisherNode::new() to this:
let publisher = node.create_publisher("out_topic", rclrs::QOS_PROFILE_DEFAULT)?;
Ok(Self {
    node,
    _subscription,
    publisher,
    data,
})
```

Then, let's add a `republish()` function to the `RepublisherNode` that publishes the latest message received, or does nothing if none was received:

```rust
fn republish(&self) -> Result<(), rclrs::RclrsError> {
    if let Some(s) = &*self.data.lock().unwrap() {
        self.publisher.publish(s)?;
    }
    Ok(())
}
```

What's left to do is to call this function every second. `rclrs` doesn't yet have ROS timers, which run a function at a fixed interval, but it's easy enough to achieve with a thread, a loop, and the sleep function. Change your main function to spawn a separate thread:

```rust
fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = RepublisherNode::new(&context)?;
    std::thread::spawn(|| -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher.republish()?;
        }
    });
    rclrs::spin(republisher.node)
}
```

But wait, this doesn't work – there is an error about the thread closure needing to outlive `'static`. That's again the same issue as above: Rust doesn't allow borrowing variables in this closure, because the function that the variable is coming from might return before the thread that borrows the variable ends.

> 💡 Of course, you could argue that this cannot really happen here, because returning from `main()` will also terminate the other threads, but Rust isn't that smart.

The solution is also the same as above: Shared ownership with `Arc`. Only this time, `Mutex` isn't needed since both the `rclcpp::spin()` and the `republish()` function only require a shared reference:

```rust
fn main() -> Result<(), rclrs::RclrsError> {
    let context = rclrs::Context::new(std::env::args())?;
    let republisher = Arc::new(RepublisherNode::new(&context)?);
    let republisher_other_thread = Arc::clone(&republisher);
    std::thread::spawn(move || -> Result<(), rclrs::RclrsError> {
        loop {
            use std::time::Duration;
            std::thread::sleep(Duration::from_millis(1000));
            republisher_other_thread.republish()?;
        }
    });
    rclrs::spin(republisher.node)
}
```


## Trying it out

In separate terminals, run `cargo run` and `ros2 topic echo /out_topic`. Nothing will be shown yet, since our node hasn't received any data yet.

In another terminal, publish a single message with `ros2 topic pub /in_topic std_msgs/msg/String '{data: "Bonjour"}' -1`. The terminal with `ros2 topic echo` should now receive a new `Bonjour` message every second.

Now publish another message, e.g. `ros2 topic pub /in_topic std_msgs/msg/String '{data: "Servus"}' -1` and observe the `ros2 topic echo` terminal receiving that message from that point forward.
