<!--
Copyright 2026 Open Source Robotics Foundation, Inc.
SPDX-License-Identifier: Apache-2.0
-->

# CPU image buffers

These snippets use an existing ROS `node`. `process_pixels` represents CPU work
on the received slice.

## Publisher

```rust,ignore
use ros_env::sensor_msgs::msg::buffer::Image;

let publisher = node.create_publisher::<Image>("image")?;
let image = Image {
    height: 1,
    width: 3,
    encoding: "mono8".into(),
    step: 3,
    data: vec![10, 20, 30].into(),
    ..Default::default()
};
publisher.publish(image)?;
```

## Subscriber

```rust,ignore
use rclrs::SubscriptionOptions;
use ros_env::sensor_msgs::msg::buffer::Image;

let subscription = node.create_subscription::<Image, _>(
    SubscriptionOptions::new("image").acceptable_buffer_backends("cpu"),
    |image: Image| {
        let pixels = image.data.as_slice().unwrap();
        process_pixels(pixels);
    },
)?;
```

The callback borrows the CPU payload without copying it. CUDA publishers use
CPU fallback for this subscription.

Existing applications can keep `sensor_msgs::msg::Image` with its `Vec<u8>`
field. Both Rust representations publish and subscribe to the same ROS topic.

Runnable examples: [publisher](buffer_image_publisher.rs),
[subscriber](buffer_image_subscriber.rs), and
[existing CPU subscriber](cpu_image_subscriber.rs).
