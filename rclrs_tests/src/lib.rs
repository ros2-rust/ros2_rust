#![cfg(test)]

mod client_service_tests;
// Disabled in Foxy due to https://github.com/ros2/rosidl/issues/598
#[cfg(not(ros_distro = "foxy"))]
mod dynamic_message_tests;
mod graph_tests;
mod pub_sub_tests;
