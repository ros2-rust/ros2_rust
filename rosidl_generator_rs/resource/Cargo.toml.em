[package]
name = "@(package_name)"
version = "@(package_version)"
edition = "2021"

[dependencies]
rosidl_runtime_rs = "0.3"
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
