[package]
name = "@(package_name)"
@# The version should ideally be taken from package.xml, see
@# https://github.com/ros2-rust/ros2_rust/issues/116
version = "0.2.0"
edition = "2021"

[dependencies]
libc = "0.2"
rosidl_runtime_rs = "*"
serde = { version = "1", optional = true, features = ["derive"] }
@[for dep in dependency_packages]@
@(dep) = "*"
@[end for]@

[features]
@{
serde_features = ["dep:serde", "rosidl_runtime_rs/serde"]
for dep in dependency_packages:
	serde_features.append("{}/serde".format(dep))
}@
serde = @(serde_features)
