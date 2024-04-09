# Writing a simple publisher and subscriber (RUST)
* Goal: Create and run a publisher and subscriber node using Python.
* Tutorial level: Beginner
* Time: 20 minutes
<details><summary>Background</summary>

In this tutorial you will create 
[nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html) that pass information to each other via a 
[topic](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html) in the form of string messages. The example used here is a simple "talker" and "listener" system; one node publishes data and the other subscribes to the topic to receive that data.

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
<div style="margin-left:20px;">
<details><summary>Create a Package</summary>

Currently, building a package for ROS2 RUST is different 
from building packages for Python or C/C++.  

First, you'll need to create and go into a standard [cargo](https://doc.rust-lang.org/cargo/) 
project as follows:
```
cargo new your_project_name && your_project_name
```
In the [`Cargo.toml`](https://doc.rust-lang.org/book/ch01-03-hello-cargo.html) file, add a dependency on `rclrs = "*"` and `std_msgs = "*"` by editing this file. For a full Introduction into RUST, please read the very good [RUST book](https://doc.rust-lang.org/book/title-page.html)


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
<details><summary>Write the publisher node</summary><details>


</details>
</div>
</details>
