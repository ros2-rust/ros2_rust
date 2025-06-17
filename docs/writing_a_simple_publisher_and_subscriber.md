# Writing a simple publisher and subscriber (Rust)
* Goal: Create and run a publisher and subscriber node using Rust.
* Tutorial level: Beginner
* Time: 20 minutes
<details><summary>Background</summary>

In this tutorial you will create a pair of 
[nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that pass information to each other via a 
[topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) in the form of string messages. The example used here is a simple "talker" and "listener" system; one node publishes data and the other subscribes to the topic to receive that data.

Since Rust doesn't have inheritance, it's not possible to inherit from `Node` as is common practice in [`rclcpp`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) or [`rclpy`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

The code used in these examples can be found [here](https://github.com/ros2-rust/ros2_rust/tree/main/examples/rust_pubsub)
<div style="margin-left:20px;">
<details><summary>Side-note on dependencies</summary>

You may be wondering why you can't just add all your ROS 2-specific dependencies to `Cargo.toml` with `cargo add YOUR_DEPENDENCIES` and have to edit this file manually. Here is why:
Almost none of the ROS 2 dependencies you'll need for your ROS 2 Rust node development currently exist on [crates.io](https://crates.io/), the 
main source for Rust dependencies. So the add command simply can't find the dependency targets. What colcon does by compiling 
the ROS 2 Rust dependencies and your ROS 2 Rust project is redirect the cargo search for dependencies directly into your 
`workspace/install` folder, where it'll find locally generated Rust projects to use as dependencies. In particular, almost 
all message types will be called as dependencies for your ROS 2 Rust project this way.

</details></div>

</details>

<details><summary>Prerequisites </summary> 

Basic concepts of development with ROS 2 should be known:
* [workspaces](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html)
* [packages](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

A basic understanding of [Rust](https://doc.rust-lang.org/book/) is recommended, but not entirely necessary.
Before developing [ros2-rust](https://github.com/ros2-rust/ros2_rust) nodes, you must follow the 
[installation instructions](https://github.com/ros2-rust/ros2-rust/blob/main/README.md) for [`rclrs`](https://docs.rs/rclrs/latest/rclrs/).
For a full Introduction into Rust, please read the very good [Rust book](https://doc.rust-lang.org/book/title-page.html).

</details>

<details><summary>Tasks </summary> 
<div style="margin-left:20px;"><details><summary>Create a Package</summary>

Currently, building a package for ros2-rust is different 
from building packages for [Python](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html) or [C/C++](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html).  

First, you'll need to create a standard [cargo](https://doc.rust-lang.org/cargo/) 
package:
```sh
cargo new rust_pubsub && cd rust_pubsub
```
In the [`Cargo.toml`](https://doc.rust-lang.org/book/ch01-03-hello-cargo.html) file, add a dependency on `rclrs = "*"` and `std_msgs = "*"` by editing this file. Your `Cargo.toml` should now look like this:
```toml
[package]
name = "rust_pubsub"
version = "0.1.0"
edition = "2021"
[dependencies]
rclrs = "*"
std_msgs = "*"
```
Additionally, create a new `package.xml` if you want your node to be buildable with [`colcon`](https://colcon.readthedocs.io/en/released/user/installation.html). Make sure to change the build type to `ament_cargo` and to include the two packages mentioned above in the dependencies, as such:
```xml
<package format="3">
  <name>rust_pubsub</name>
  <version>0.0.0</version>
  <description>TODO: Package description.</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration.</license>

  <depend>rclrs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```
Your package should now have a similar structure:  
```sh
├── Cargo.toml
├── package.xml
└── src
    └── main.rs
```

</details>

<details><summary>Write the publisher node</summary>

To construct a node, replace the code in your `main.rs` file with the following:  
```rust
/// Creates a SimplePublisherNode, initializes a node and publisher, and provides
/// methods to publish a simple "Hello World" message on a loop in separate threads.
use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{env, sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
/// SimplePublisherNode struct contains node and publisher members.
/// Used to initialize a ROS 2 node and publisher, and publish messages.
struct SimplePublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<StringMsg>>,
}
impl SimplePublisherNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, publisher })
    }
    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self.publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
    });
    rclrs::spin(publisher.node.clone())
}
```

<details><summary>Examining the code in detail:</summary>

#### Importing 
The first 3 lines of the Rust code imports tools for thread synchronization, time 
handling, iteration, threading, ROS 2 communication, and string message publishing.
```rust
use rclrs::{create_node, Context, Node, Publisher, RclrsError, QOS_PROFILE_DEFAULT};
use std::{env, sync::Arc, thread, time::Duration};
use std_msgs::msg::String as StringMsg;
```
* `use std::{sync::Arc, time::Duration, iter, thread};`: Imports specific features from the standard library: 
    - `Arc` is for thread-safe shared ownership of data. 
    - `Duration` represents a time span. 
    - `thread` enables creating and managing threads.
* `use rclrs::{RclrsError, QOS_PROFILE_DEFAULT, Context, create_node, Node, Publisher};`: 
    - Imports elements for ROS 2 communication: 
        - `RclrsError` for handling errors. 
        - `QOS_PROFILE_DEFAULT` for default Quality of Service settings. 
- `Context, create_node, Node, Publisher` are for ROS 2 node creation and publishing.
* `use std_msgs::msg::String as StringMsg;`: Imports the `StringMsg` type for publishing string messages.  

#### `SimplePublisherNode`
Next, this structure defines a `SimplePublisherNode` which holds references to a ROS 2 node and a publisher for string messages.
```rust
struct SimplePublisherNode {
    node: Arc<Node>,
    publisher: Arc<Publisher<StringMsg>>,
}
```
1. Structure:  
`struct SimplePublisherNode`: This line defines a new [`struct`](https://doc.rust-lang.org/rust-by-example/custom_types/structs.html) named `SimplePublisherNode`. It serves as a blueprint for creating objects that hold information related to a simple publisher node in ROS 2.  

2. Members:
* `node: Arc<Node>`: This member stores a reference to a ROS 2 node, wrapped in an [`Arc` (Atomic Reference Counted)](https://doc.rust-lang.org/std/sync/struct.Arc.html) smart pointer. This allows for safe sharing of the node reference across multiple threads.  
* `_publisher: Arc<Publisher<StringMsg>>`: This member stores a reference to a publisher specifically for string messages (`StringMsg`), also wrapped in an `Arc` for thread safety. The publisher is responsible for sending string messages to other nodes in the ROS 2 system.  
#### `impl SimplePublisher`
This code defines methods for the `SimplePublisherNode` `struct`. The `new` method creates a ROS 2 node and publisher, storing them in the `struct`. The `publish_data` method publishes a string message with a `counter` and returns the incremented `counter`.
```rust
impl SimplePublisherNode {
    fn new(context: &context) -> result<self, RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let publisher = node
            .create_publisher("publish_hello", qos_profile_default)
            .unwrap();
        ok(self { node, publisher })
    }
    fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> {
        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}", increment),
        };
        self.publisher.publish(msg).unwrap();
        Ok(increment + 1_i32)
    }
}
```

1. Implementation Block:   
`impl SimplePublisherNode { ... }`: This line indicates that methods are being defined for the `SimplePublisherNode` struct.  
2. Constructor Method:  
* `fn new(context: &Context) -> Result<Self, RclrsError> { ... }`: This method serves as a constructor for creating instances of `SimplePublisherNode`.  
    * It takes a Context object as input, which is necessary for interacting with the ROS 2 system.
    * It returns a Result type, indicating either a successful Self (the created `SimplePublisherNode` object) or an `RclrsError` if something goes wrong.  
    * Inside the new method:  
        * `let node = create_node(context, "simple_publisher").unwrap();`: Creates a new ROS 2 node named `"simple_publisher"` within the given context. The [`unwrap()`](https://doc.rust-lang.org/rust-by-example/error/option_unwrap.html) unwraps the [`Result`](https://doc.rust-lang.org/std/result/), handling any errors immediately by forcing the program to abort ([`panic`](https://doc.rust-lang.org/book/ch09-01-unrecoverable-errors-with-panic.html)) if something goes wrong. Since your code can't function properly if the node is not able to be created, this is a valid error-handling response for our use-case.
        * `let _publisher = node.create_publisher("publish_hello", QOS_PROFILE_DEFAULT).unwrap();`: Creates a publisher for string messages on the topic `"publish_hello"` with default quality of service settings.  
        * `Ok(Self { node, _publisher, })`: Returns an [`Ok(T)`](https://doc.rust-lang.org/std/result/) Result with the newly created `SimplePublisherNode` as `T` object, containing the node and publisher references.  
3. Publishing Method:
* `fn publish_data(&self, increment: i32) -> Result<i32, RclrsError> { ... }`: This method publishes a string message and increments a `counter`.
    * It takes an increment value (an integer) as input, which is used for counting purposes within the message content.
    * It also returns a Result type, indicating either the incremented value or an [`RclrsError`](https://docs.rs/rclrs/latest/rclrs/enum.RclrsError.html) if publishing fails.
    * Inside the publish_data method:
        * `let msg: StringMsg = StringMsg { data: format!("Hello World {}", increment), };`: Creates a string message with the content `"Hello World"` followed by the increment value.
        * `self._publisher.publish(msg).unwrap();`: Publishes the created message onto the topic associated with the publisher.
        * `Ok(increment + 1_i32)`: Returns a Result with the incremented increment value.  

#### main
The main Method creates a ROS 2 node that publishes string messages at a rate of 1 Hz.
```rust
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut count: i32 = 0;
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        count = publisher_other_thread.publish_data(count).unwrap();
    });
    rclrs::spin(publisher.node.clone())
}
```

1. Main Function:
`fn main() -> Result<(), RclrsError> { ... }`: This defines the main entry point of the program. It returns a [`Result`](https://doc.rust-lang.org/std/result/) type, indicating either successful execution or an [`RclrsError`](https://docs.rs/rclrs/latest/rclrs/enum.RclrsError.html).  
2. Context and Node Setup:  
* `let context = Context::new(std::env::args()).unwrap();`: Creates a ROS 2 context using command-line arguments.  
* `let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());`:  
    * Creates an [Arc (atomic reference counted)](https://doc.rust-lang.org/std/sync/struct.Arc.html) pointer to a `SimplePublisherNode` object.  
    * Calls the new method on `SimplePublisherNode` to construct the node and publisher within the context.  
3. Thread and Iterator:  
* `let publisher_other_thread = Arc::clone(&publisher);`: Clones the shared publisher pointer for use in a separate thread.  
* `let mut iterator: i32 = 0;`: Initializes a counter variable for message content.  
* `thread::spawn(move || -> () { ... });`: Spawns a new [thread](https://doc.rust-lang.org/std/thread/index.html) with a [closure](https://doc.rust-lang.org/book/ch13-01-closures.html): `loop { ... }`: Creates an infinite loop using [`loop`](https://doc.rust-lang.org/std/keyword.loop.html).  
4. Publishing Loop within Thread:  
* `thread::sleep(Duration::from_millis(1000));`: Pauses the thread for 1 second (1 Hz publishing rate).  
* `iterator = publisher_other_thread.publish_data(count).unwrap();`: Calls the `publish_data` method on the `publisher_other_thread` to publish a message with the current counter value. Increments the iterator for the next message.  
5. Main Thread Spin:  
* `rclrs::spin(publisher.node.clone());`: Keeps the main thread running, processing ROS 2 events and messages. Uses a cloned reference to the node to ensure it remains active even with other threads.  

</details>
</details>
<details><summary>Having several ROS 2 Rust nodes in one Package</summary>

Of course, you can write for each node you want to implement its own package, and that can have it's advantages. I implore you to use some cargo tricks and add some binary targets to your `cargo.toml`. That could look like this:
```toml
[package]
name = "rust_pubsub"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[[bin]]
name="simple_publisher"
path="src/simple_publisher.rs"
[dependencies]
rclrs = "*"
std_msgs = "*"
```

You'll find the name of your executable and the corresponding file name under the `[[bin]]` tag. As you can see, the filename and the name you want to call your node don't have to match. Please remember to include your executable name with snake_cases. The Rust compiler will be a bit grumpy if you don't.  
Now, by recompiling the package from the previous chapter and making it usable:  
```sh
cd WORKSPACE
colcon build
source install/setub.bash
```
Running the node will look like this:
```sh
ros2 run rust_pubsub simple_publisher
```
As you can see, you are now calling your node by the name declared in `[[bin]]` using the `name` variable.

</details>
<details><summary>Write the subscriber node</summary> 

Of course, you can implement a new ROS 2 Rust package for this node. You can find out how to do this in the section called 'Create a package'.
Or you can add a new binary target to your package. To do so, just add a new `FILE.rs` to your source directory - for simplicity I'll call this file `simple_subscriber.rs` - and add a corresponding binary target to your `Cargo.toml`:
```toml
[[bin]]
name="simple_subscriber"
path="src/simple_subscriber.rs"
```
To construct the subscriber node, put the following code into a `FILE.rs` - in my case its the `src/simple_subscriber.rs`:
```rust
use rclrs::{create_node, Context, Node, RclrsError, Subscription, QOS_PROFILE_DEFAULT};
use std::{
    env,
    sync::{Arc, Mutex},
    thread,
    time::Duration,
};
use std_msgs::msg::String as StringMsg;
pub struct SimpleSubscriptionNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}
impl SimpleSubscriptionNode {
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_subscription").unwrap();
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);
        let _subscriber = node
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Ok(Self {
            node,
            _subscriber,
            data,
        })
    }
    fn data_callback(&self) -> Result<(), RclrsError> {
        if let Some(data) = self.data.lock().unwrap().as_ref() {
            println!("{}", data.data);
        } else {
            println!("No message available yet.");
        }
        Ok(())
    }
}
fn main() -> Result<(), RclrsError> {
    let context = Context::new(env::args()).unwrap();
    let subscription = Arc::new(SimpleSubscriptionNode::new(&context).unwrap());
    let subscription_other_thread = Arc::clone(&subscription);
    thread::spawn(move || loop {
        thread::sleep(Duration::from_millis(1000));
        subscription_other_thread.data_callback().unwrap()
    });
    rclrs::spin(subscription.node.clone())
}
```
<details><summary>Examining the code in detail:</summary>

#### SimpleSubscriptionNode:
```rust
pub struct SimpleSubscriptionNode {
    node: Arc<Node>,
    _subscriber: Arc<Subscription<StringMsg>>,
    data: Arc<Mutex<Option<StringMsg>>>,
}
```
Instead of a Publisher, there is a Subscription object in the Subscriber node. The data needs to be an `Arc<Mutex<Option<StringMsg>>>` because there can be errors in the data transfer process and this can be caught by including the value of the incoming subscription in an optional.  
#### impl SimpleSubscriptionNode
This code defines methods for the `SimpleSubscriptionNode` `struct`.  

##### new
The `new` method creates a ROS 2 node, subscriber, and a storage location for received messages, storing them in the `struct`.
```rust
    fn new(context: &Context) -> Result<Self, RclrsError> {
        let node = create_node(context, "simple_subscription").unwrap();
        let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));
        let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data);
        let _subscriber = node
            .create_subscription::<StringMsg, _>(
                "publish_hello",
                QOS_PROFILE_DEFAULT,
                move |msg: StringMsg| {
                    *data_mut.lock().unwrap() = Some(msg);
                },
            )
            .unwrap();
        Ok(Self {
            node,
            _subscriber,
            data,
        })
    }
```
A few special features:  
1. Initializing Shared Data:  
    * `let data: Arc<Mutex<Option<StringMsg>>> = Arc::new(Mutex::new(None));`  
        This line creates a shared data structure that will hold the received message.  
        * `Arc<Mutex<Option<StringMsg>>>`: This is a complex type combining several functionalities:  
            * `Arc<T>`: An atomically reference-counted pointer ([`Arc`](https://doc.rust-lang.org/std/sync/struct.Arc.html)) allows multiple parts of the code to safely access the same data (`T`).
            * `Mutex<T>`: A mutual exclusion lock ([`Mutex`](https://doc.rust-lang.org/std/sync/struct.Mutex.html)) ensures only one thread can modify the data (`T`) at a time, preventing race conditions.  
            * `Option<StringMsg>`: This represents an optional value that can either hold a message of type `StringMsg` or be `None` if no message has been received yet.
    * `Arc::new(Mutex::new(None))`: This creates a new instance of `Arc<Mutex<Option<StringMsg>>>` and initializes the inner `Mutex` with `None`.
2. Creating a Subscription:  
    * `let _subscriber = node.create_subscription::<StringMsg, _>(...`  
        This line attempts to create a subscription using the created ROS node (`node`).  
        * `create_subscription`: This is creates a subscription to a specific topic.  
        * `<StringMsg, _>`: This specifies the type of message the subscription is interested in (`StringMsg`) and a placeholder (`_`) for the callback [`closure`](https://doc.rust-lang.org/book/ch13-01-closures.html) type.  
            `"publish_hello"`: This is the name of the ROS topic this node wants to subscribe to. Messages of type `StringMsg` are expected on this topic.  
        * `move |msg: StringMsg| { ... }`: This is a closure ([anonymous function](https://en.wikipedia.org/wiki/Anonymous_function)) that will be called whenever a new message arrives on the subscribed topic.
        * `msg: StringMsg`: This parameter receives the received message of type `StringMsg`. The closure body (`{...}`) uses the `Mutex` to access and update the shared data (`data_mut`) with the received message.  
3. Cloning the Shared Data:
    * `let data_mut: Arc<Mutex<Option<StringMsg>>> = Arc::clone(&data)`; This line creates another `Arc` reference (`data_mut`) pointing to the same underlying data structure as data. This allows the closure to access and modify the shared data.  
##### data_callback
This function provides a way to access and potentially use the received message data stored within the `data` member variable of the `SimpleSubscriptionNode`. It checks if a message exists, prints it if available, or informs the user there's no message yet.
```rust
fn data_callback(&self) -> Result<(), RclrsError> {
    if let Some(data) = self.data.lock().unwrap().as_ref() {
         println!("{}", data.data);
    } else {
        println!("No message available yet.");
    }
    Ok(())
}

```
A few special features:  
1. Checking for Received Message:  
    * `if let Some(data) = self.data.lock().unwrap().as_ref() { ... }`: This is an [`if-let`](https://doc.rust-lang.org/rust-by-example/flow_control/if_let.html) statement used for pattern matching on optional values.  
    * `self.data`: This accesses the member variable data of the `struct` (likely the `Arc<Mutex<Option<StringMsg>>>` created earlier).  
    * `.lock().unwrap()`: This calls the lock method on the `Mutex` to gain exclusive access to the shared data. If another thread already holds the lock, lock might block until the lock is released.  
        `.as_ref()`: This converts the borrowed `MutexGuard` (returned by `.lock()`) into a reference to the inner value (`Option<StringMsg>`).  
    * `Some(data)`: This pattern attempts to match the value inside the Option with `Some(data)`. If there's a message (`Some(data)`), the code block after the if is executed, and data is bound to the actual message content of type `StringMsg`.  

</details>
</details>
<details><summary>Build and Run</summary>

Once you have implemented the code, you are ready to make it runnable:
```sh
cd WORKSPACE
colcon build
```
Please note that you'll need to run your nodes in separate terminals. In each terminal, you'll need to source your ROS 2 installation separately. So for each of the two nodes you've built so far, open a terminal and type the following:
```sh
cd WORKSPACE
source install/setup.bash
ros2 run rust_pubsub your_node_name
```
In my case, the nodes are called `simple_publisher` and `simple_subscriber`. You can name your nodes whatever you like. It is important that the publisher and subscriber use the same topic type and name.  
If you haven't had any errors so far and have successfully started the Publisher and Subscriber, you should see something similar in the Subscriber's Terminal window:
```sh
Hello World 230
Hello World 231
Hello World 232
Hello World 233
Hello World 234
Hello World 235
Hello World 236
Hello World 237
Hello World 238
Hello World 239
Hello World 240
Hello World 241
Hello World 242
Hello World 243
Hello World 244
Hello World 245
Hello World 246
```
(My nodes have been running for some time.)  
Enter `Ctrl+c` in each terminal to stop the nodes from spinning.
</details></div>
</details>

<details><summary>Summary</summary>

You created two nodes to publish and subscribe to data over a topic. Before running them, you added their dependencies and entry points to the package configuration files.

</details></details>

<details><summary>Last thoughts</summary>

At the end of the day, tools must not only work more safely and efficiently from a purely rational point of view, but they must also give the end user, as well as the developer, a good time. Hopefully you had fun developing the two nodes. Without fun, software development can be boring and will often prevent you from using this tool again. 

</details>
