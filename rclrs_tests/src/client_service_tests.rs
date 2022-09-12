use rclrs::{Client, Service};

fn assert_send<T: Send>() {}
fn assert_sync<T: Sync>() {}

#[test]
fn client_is_send_and_sync() {
    assert_send::<Client<test_msgs::srv::Arrays>>();
    assert_sync::<Client<test_msgs::srv::Arrays>>();
}

#[test]
fn service_is_send_and_sync() {
    assert_send::<Service<test_msgs::srv::Arrays>>();
    assert_sync::<Service<test_msgs::srv::Arrays>>();
}
