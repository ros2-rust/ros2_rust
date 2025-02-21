# Differences between the [ROS2](https://docs.ros.org/en/jazzy/index.html) logger and standard [`println!()`](https://doc.rust-lang.org/std/macro.println.html)

The [ROS2 logger](https://docs.ros.org/en/jazzy/Tutorials/Demos/Logging-and-logger-configuration.html) and
[classic stdout (e.g., `println!`)](https://en.wikipedia.org/wiki/Standard_streams) differ in several key ways:

1. **Severity Levels**:  
   ROS2 provides structured log levels (`DEBUG`, `INFO`, `WARN`, `ERROR`, `FATAL`), enabling filtering based on urgency. `println!` outputs raw text without categorization.

2. **Dynamic Control**:  
   ROS2 allows runtime adjustment of logging levels (e.g., enabling `DEBUG` on-the-fly). `println!` statements cannot be disabled without code changes.

3. **Metadata**:  
   ROS2 automatically appends context (timestamp, node name, file/line number) to logs. `println!` requires manual addition of such details.

4. **Integration**:  
   ROS2 logs are captured by tools like `rqt_console` and can be routed to files/network. `println!` outputs only to stdout unless manually redirected.

5. **Performance**:  
   ROS2 optimizes by skipping disabled log levels (e.g., `DEBUG` if not active). `println!` always executes, incurring overhead regardless of need.

6. **Configuration**:  
   ROS2 logging is configurable via launch files/parameters (e.g., per-node log levels). `println!` offers no built-in configuration.

**Example**:

```rust
// ROS2 logger (Rust example with rclrs)
log!(node.info(), "Sensor value: {}", sensor_data);  // Adds metadata, severity, and runtime control

// Classic stdout
println!("Sensor value: {}", sensor_data);  // Simple, unstructured output
```

**Use ROS2 logger** for structured, controllable logging within the ROS2 ecosystem; use `println!` for quick, simple text output.
