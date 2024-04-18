[package]
name = "@(package_name)"
version = "@(package_version)"
edition = "2021"
description = "@(package_name) ros2 rust generated dependencies"
license = "Apache-2.0"

[dependencies]
rosidl_runtime_rs = "0.4"
serde = { version = "1", optional = true, features = ["derive"] }
serde-big-array = { version = "0.5.1", optional = true }
@[for dep in dependency_packages]@
@(dep) = "*"
@[end for]@

[features]
@{
serde_features = ["dep:serde", "dep:serde-big-array", "rosidl_runtime_rs/serde"]
for dep in dependency_packages:
	serde_features.append("{}/serde".format(dep))
}@
serde = @(serde_features)
