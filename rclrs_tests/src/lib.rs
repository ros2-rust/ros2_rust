#![cfg(test)]

mod client_service_tests;
// Disabled in Foxy due to https://github.com/ros2/rosidl/issues/598
#[cfg(all(not(ros_distro = "foxy"), not(ros_distro = "galactic")))]
mod dynamic_message_tests;
mod graph_tests;
mod pub_sub_tests;
