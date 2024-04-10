# Writing a simple publisher and subscriber (RUST)
* Goal: Create and run a publisher and subscriber node using Python.
* Tutorial level: Beginner
* Time: 20 minutes
<details><summary>Background</summary>

In this tutorial you will create a 
[nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that pass information to each other via a 
[topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) in the form of string messages. The example used here is a simple "talker" and "listener" system; one node publishes data and the other subscribes to the topic to receive that data.

Since Rust doesn't have inheritance, it's not possible to inherit from `Node` as is common practice in [`rclcpp`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html) or [`rclpy`](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html).

The code used in these examples can be found [here](https://gitlab.com/ros21923912/simple_ros2_node)  
<div style="margin-left:20px;">
<details><summary>Sidenode to dependencies</summary>

You may be wondering why you can't just add all your ros2-specific dependencies to `cargo.toml` with `cargo add ${dependencie}` and have to edit this file manually. Here is why:
Almost none of the ROS2 dependencies you'll need for your ros2 rust node development currently exist on [crates.io](https://crates.io/), the main source for rust depencies. So the add command simply can't find the dependency targets. What colcon does by compiling the ros2 rust dependencies and your ros2 rust project is redirect the cargo search for dependencies directly into your `workspace/install` folder, where it'll find locally generated rust projects to use as dependencies. In particular, almost all message types will be called as dependencies for your ros2 rust project this way.

</details></div>

</details>

<details><summary>Prerequisites </summary> 

In previous tutorials, you learned how to create a [workspace](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html) and [create a package](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html).

A basic understanding of [RUST](https://www.rust-lang.org/) is recommended, but not entirely necessary.
Before developing ROS2 RUST nodes, you must follow the 
[installation instructions](https://github.com/ros2-rust/ros2_rust/blob/main/README.md) for them.


</details>

<details><summary>Tasks </summary> 
<div style="margin-left:20px;"><details><summary>Create a Package</summary>

Currently, building a package for ROS2 RUST is different 
from building packages for Python or C/C++.  

First, you'll need to create and go into a standard [cargo](https://doc.rust-lang.org/cargo/) 
project as follows:
```
cargo new your_project_name && your_project_name
```
In the [`Cargo.toml`](https://doc.rust-lang.org/book/ch01-03-hello-cargo.html) file, add a dependency on `rclrs = "*"` and `std_msgs = "*"` by editing this file. For a full Introduction into RUST, please read the very good [RUST book](https://doc.rust-lang.org/book/title-page.html). Your `Cargo.toml` could now look like this:
```
[package]
name = "your_package_name"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

[dependencies]
rclrs = "*"
std_msgs = "*"
```


Additionally, create a new `package.xml` if you want your node to be buildable with [`colcon`](https://colcon.readthedocs.io/en/released/user/installation.html). Make sure to change the build type to `ament_cargo` and to include the two packages mentioned above in the dependencies, as such:
```xml
<package format="3">
  <name>your_project_name</name>
  <version>0.0.0</version>
  <description>TODO: Package description. Seriously. Please make a Package description. We all will thank for it.</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>TODO: License declaration. Licenses are Great. Please add a Licence</license>

  <depend>rclrs</depend>
  <depend>std_msgs</depend>

  <export>
    <build_type>ament_cargo</build_type>
  </export>
</package>
```
By taking a look at your package, for example by typing [`tree .`](https://www.geeksforgeeks.org/tree-command-unixlinux/) inside your package, and you'll see a structure similar to the following:  
```
├── Cargo.toml
├── package.xml
└── src
    └── main.rs

2 directories, 3 files
```
Of course, you can use any capable editor or even your file explorer to do this.  

</details>

<details><summary>Write the publisher node</summary>



To construct a node, replace the code in your main.rs with the [following](https://gitlab.com/ros21923912/simple_ros2_node/-/raw/more_simple_nodes/src/simple_publisher.rs?ref_type=heads):  
```
/// Creates a SimplePublisherNode, initializes a node and publisher, and provides
/// methods to publish a simple "Hello World" message on a loop in separate threads.

/// Imports the Arc type from std::sync, used for thread-safe reference counting pointers,
/// and the StringMsg message type from std_msgs for publishing string messages.
use std::{sync::Arc,time::Duration,iter,thread};
use rclrs::{RclrsError,QOS_PROFILE_DEFAULT,Context,create_node,Node,Publisher};
use std_msgs::msg::String as StringMsg;
// / SimplePublisherNode struct contains node and publisher members.
// / Used to initialize a ROS 2 node and publisher, and publish messages.
struct SimplePublisherNode {
    node: Arc<Node>,
    _publisher: Arc<Publisher<StringMsg>>,
}
/// Creates a new SimplePublisherNode by initializing a node and publisher.
///
/// The `new` function takes a context and returns a Result containing the
/// initialized SimplePublisherNode or an error. It creates a node with the
/// given name and creates a publisher on the "publish_hello" topic.
///
/// The SimplePublisherNode contains the node and publisher members.
impl SimplePublisherNode {
    /// Creates a new SimplePublisherNode by initializing a node and publisher.
    ///
    /// This function takes a context and returns a Result containing the
    /// initialized SimplePublisherNode or an error. It creates a node with the
    /// given name and creates a publisher on the "publish_hello" topic.
    ///
    /// The SimplePublisherNode contains the node and publisher members.
    fn new(context: &Context) -> Result<Self,RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let _publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, _publisher, })
    }

    /// Publishes a "Hello World" message on the publisher.
    ///
    /// Creates a StringMsg with "Hello World" as the data, publishes it on
    /// the `_publisher`, and returns a Result. This allows regularly publishing
    /// a simple message on a loop.
    fn publish_data(&self,inkrement:i32) -> Result<i32,RclrsError> {

        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}",inkrement),
        };
        self._publisher.publish(msg).unwrap();
        Ok(inkrement+1_i32)
    }
}

/// The main function initializes a ROS 2 context, node and publisher,
/// spawns a thread to publish messages repeatedly, and spins the node
/// to receive callbacks.
/// 
/// It creates a context, initializes a SimplePublisherNode which creates
/// a node and publisher, clones the publisher to pass to the thread,  
/// spawns a thread to publish "Hello World" messages repeatedly, and
/// calls spin() on the node to receive callbacks. This allows publishing
/// messages asynchronously while spinning the node.
fn main() -> Result<(),RclrsError> {
    let context = Context::new(std::env::args()).unwrap();
    let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
    let publisher_other_thread = Arc::clone(&publisher);
    let mut iterator: i32=0;
    thread::spawn(move || -> () {
        iter::repeat(()).for_each(|()| {
            thread::sleep(Duration::from_millis(1000));
            iterator=publisher_other_thread.publish_data(iterator).unwrap();
        });
    });
    rclrs::spin(publisher.node.clone())
}
```

<details><summary>Examine the Code:</summary>

#### This first 3 lines of the Rust code imports tools for thread synchronization, time handling, iteration, threading, ROS 2 communication, and string message publishing. It's likely setting up a ROS 2 node that publishes string messages.
```
use std::{sync::Arc,time::Duration,iter,thread};
use rclrs::{RclrsError,QOS_PROFILE_DEFAULT,Context,create_node,Node,Publisher};
use std_msgs::msg::String as StringMsg;
```
* use std::{sync::Arc, time::Duration, iter, thread};: - Imports specific features from the standard library: - Arc is for thread-safe shared ownership of data. - Duration represents a time span. - iter provides tools for working with iterators. - thread enables creating and managing threads.
* use rclrs::{RclrsError, QOS_PROFILE_DEFAULT, Context, create_node, Node, Publisher};: - Imports elements for ROS 2 communication: - RclrsError for handling errors. - QOS_PROFILE_DEFAULT likely for default Quality of Service settings. - Context, create_node, Node, Publisher are for ROS 2 node creation and publishing.
* use std_msgs::msg::String as StringMsg;: - Imports the StringMsg type for publishing string messages.  

#### Next this struct defines a SimplePublisherNode which holds references to a ROS 2 node and a publisher for string messages.
```
struct SimplePublisherNode {
    node: Arc<Node>,
    _publisher: Arc<Publisher<StringMsg>>,
}
```
1. Structure:
struct SimplePublisherNode: This line defines a new struct named SimplePublisherNode. It serves as a blueprint for creating objects that hold information related to a simple publisher node in ROS 2.

2. Members:
* node: Arc<Node>: This member stores a reference to a ROS 2 node, wrapped in an Arc (Atomic Reference Counted) smart pointer. This allows for safe sharing of the node reference across multiple threads.  
* _publisher: Arc<Publisher<StringMsg>>: This member stores a reference to a publisher specifically for string messages (StringMsg), also wrapped in an Arc for thread safety. The publisher is responsible for sending string messages to other nodes in the ROS 2 system.  

3. This code defines methods for the SimplePublisherNode struct. The new method creates a ROS 2 node and publisher, storing them in the struct. The publish_data method publishes a string message with a counter and returns the incremented counter.
```
impl SimplePublisherNode {
    fn new(context: &Context) -> Result<Self,RclrsError> {
        let node = create_node(context, "simple_publisher").unwrap();
        let _publisher = node
            .create_publisher("publish_hello", QOS_PROFILE_DEFAULT)
            .unwrap();
        Ok(Self { node, _publisher, })
    }
    fn publish_data(&self,inkrement:i32) -> Result<i32,RclrsError> {

        let msg: StringMsg = StringMsg {
            data: format!("Hello World {}",inkrement),
        };
        self._publisher.publish(msg).unwrap();
        Ok(inkrement+1_i32)
    }
}
```

1. Implementation Block:
`impl SimplePublisherNode { ... }`: This line indicates that methods are being defined for the `SimplePublisherNode` struct.

2. Constructor Method:
* `fn new(context: &Context) -> Result<Self, RclrsError> { ... }`: This method serves as a constructor for creating instances of SimplePublisherNode.
    * It takes a Context object as input, which is necessary for interacting with the ROS 2 system.
    * It returns a Result type, indicating either a successful Self (the created SimplePublisherNode object) or an RclrsError if something goes wrong.
    * Inside the new method:
        * `let node = create_node(context, "simple_publisher").unwrap();`: Creates a new ROS 2 node named "simple_publisher" within the given context. The unwrap() unwraps the result, handling any errors immediately.
        * `let _publisher = node.create_publisher("publish_hello", QOS_PROFILE_DEFAULT).unwrap();`: Creates a publisher for string messages on the topic "publish_hello" with default quality of service settings.
        * `Ok(Self { node, _publisher, })`: Returns a Result with the newly created SimplePublisherNode object, containing the node and publisher references.

3. Publishing Method:
* `fn publish_data(&self, inkrement: i32) -> Result<i32, RclrsError> { ... }`: This method publishes a string message and increments a counter.
    * It takes an inkrement value (an integer) as input, which is likely used for counting purposes within the message content.
    * It also returns a Result type, indicating either the incremented inkrement value or an RclrsError if publishing fails.
    * Inside the publish_data method:
        * `let msg: StringMsg = StringMsg { data: format!("Hello World {}", inkrement), };`:tCreates a string message with the content "Hello World" followed by the inkrement value.
        * self._publisher.publish(msg).unwrap();: Publishes the created message onto the topic associated with the publisher.
        * Ok(inkrement + 1_i32): Returns a Result with the incremented inkrement value.  

#### The main Method creates a ROS 2 node that publishes string messages at a rate of 1 Hz.

```
fn main() -> Result<(),RclrsError> {
   let context = Context::new(std::env::args()).unwrap();
   let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());
   let publisher_other_thread = Arc::clone(&publisher);
   let mut iterator: i32=0;
   thread::spawn(move || -> () {
       iter::repeat(()).for_each(|()| {
           thread::sleep(Duration::from_millis(1000));
           iterator=publisher_other_thread.publish_data(iterator).unwrap();
       });
   });
   rclrs::spin(publisher.node.clone())
}
```

1. Main Function:
* `fn main() -> Result<(), RclrsError> { ... }`: This defines the main entry point of the program. It returns a Result type, indicating either successful execution or an RclrsError.

2. Context and Node Setup:

* `let context = Context::new(std::env::args()).unwrap();`: Creates a ROS 2 context using command-line arguments.
* `let publisher = Arc::new(SimplePublisherNode::new(&context).unwrap());`:
    * Creates an [Arc (atomic reference counted)](https://doc.rust-lang.org/std/sync/struct.Arc.html) pointer to a `SimplePublisherNode` object.
    * Calls the new method on SimplePublisherNode to construct the node and publisher within the context.

3. Thread and Iterator:
* `let publisher_other_thread = Arc::clone(&publisher);`: Clones the shared publisher pointer for use in a separate thread.
* `let mut iterator: i32 = 0;`: Initializes a counter variable for message content.
* `thread::spawn(move || -> () { ... });`: Spawns a new thread with a closure: `iter::repeat(()).for_each(|()| { ... });`: Creates an infinite loop using `iter::repeat`.

4. Publishing Loop within Thread:

* `thread::sleep(Duration::from_millis(1000));`: Pauses the thread for 1 second (1 Hz publishing rate).
* `iterator = publisher_other_thread.publish_data(iterator).unwrap();`: Calls the publish_data method on the publisher_other_thread to publish a message with the current counter value. Increments the iterator for the next message.

5. Main Thread Spin:
* `rclrs::spin(publisher.node.clone());`: Keeps the main thread running, processing ROS 2 events and messages. Uses a cloned reference to the node to ensure it remains active even with other threads.

</details>

Once you have implemented the code, you are ready to run it:
```
cd ${MainFolderOfWorkspace}
colcon build
source install/setub.bash
```
And finally run with:
```
ros2 run your_project_name your_project_name
```
(Please give your package a better name than me ;) )
</details>
<details><summary>Having several Ros2 rust nodes in one Package</summary>

Of course, you can write for each node you want to implement its own package, and that can have it's advantages. I implore you to use some cargo tricks and add some binary targets to your `cargo.toml`. this could look like this:
```
[package]
name = "your_package_name"
version = "0.1.0"
edition = "2021"

# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html
[[bin]]
name="simple_publisher"
path="src/main.rs"
[dependencies]
rclrs = "*"
std_msgs = "*"
```
You'll find the name of your executable and the corresponding file name under the `[[bin]]` tag. As you can see, the filename and the name you want to call your node don't have to match. Please remember to include your executable name with snake_cases. The rust compiler will be a bit grumpy if you don't.  
Now, by recompiling the package from the previous chapter and making it usable:  
```
cd ${MainFolderOfWorkspace}
colcon build
source install/setub.bash
```
node will look like this:
```
ros2 run your_package_name simple_publisher
```
As you can see, you are now calling your node by the name declared in `[[bin]]` using the `name` variable.
</details>
<details><summary>Write the subscriber node</summary> 

Of course, you can implement a new ros2 rust package for this node. You can find out how to do this in the section called 'Create a package'.
Or you can add a new binary target to your package. Then just add a new `<file>.rs` to your source directory - for simplicity I'll call this file `simple_subscriber.rs` - and add a corresponding binary target to your `Cargo.toml`:
```

```

</details>
</details></div>
