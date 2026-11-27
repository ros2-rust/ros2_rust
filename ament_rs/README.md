# ament_rs

Utilities for querying the ament resource index used by ROS 2 packages.

[![crate.io](https://img.shields.io/crates/v/ament_rs.svg)](https://crates.io/crates/ament_rs)
[![docs.rs](https://docs.rs/ament_rs/badge.svg)](https://docs.rs/ament_rs)

## Examples
```rust
println!("{:#?}", ament_rs::packages()?);
```

This snippet prints a list of packages with the prefixes they were found in, depending on the value of the `AMENT_PREFIX_PATH` environment variable on your system.

```text
{
    "ros_core": [
        "/opt/ros/rolling",
    ],
    "rcl_interfaces": [
        "/your/workspace/install/rcl_interfaces",
        "/opt/ros/rolling",
    ],
    ...
}
```

### `*_in`
The `*_in` function variants take an explicit prefix list instead of reading `AMENT_PREFIX_PATH`.

```rust 
let prefixes = ament_rs::prefixes()?;

assert_eq!(ament_rs::find_package_in("rcl_interfaces", &prefixes), 
           ament_rs::find_package("rcl_interfaces")?);
```
