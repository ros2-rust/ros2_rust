// Copyright 2026 Open Source Robotics Foundation, Inc.
// SPDX-License-Identifier: Apache-2.0

use std::borrow::Cow;

use ros_env::{sensor_msgs::msg::Image, test_msgs::msg::BoundedSequences};
use rosidl_runtime_rs::{Message, RmwMessage};

#[test]
fn cpu_messages_round_trip_without_requiring_buffer_support() {
    let image = Image {
        data: vec![11, 22, 33],
        ..Default::default()
    };
    let native = Image::into_rmw_message(Cow::Borrowed(&image))
        .into_owned()
        .try_into_cpu()
        .unwrap();
    assert_eq!(Image::try_from_rmw_message(native).unwrap(), image);

    let bounded = BoundedSequences {
        uint8_values: vec![11, 22, 33].try_into().unwrap(),
        ..Default::default()
    };
    let native = BoundedSequences::into_rmw_message(Cow::Borrowed(&bounded))
        .into_owned()
        .try_into_cpu()
        .unwrap();
    assert_eq!(
        BoundedSequences::try_from_rmw_message(native).unwrap(),
        bounded
    );
}
