use rclrs::{LoanedMessage, Publisher, ReadOnlyLoanedMessage, Subscription};

fn assert_send<T: Send>() {}
fn assert_sync<T: Sync>() {}

#[test]
fn publisher_is_send_and_sync() {
    assert_send::<Publisher<test_msgs::msg::BoundedSequences>>();
    assert_sync::<Publisher<test_msgs::msg::BoundedSequences>>();
}

#[test]
fn subscription_is_send_and_sync() {
    assert_send::<Subscription<test_msgs::msg::BoundedSequences>>();
    assert_sync::<Subscription<test_msgs::msg::BoundedSequences>>();
}

#[test]
fn loaned_message_is_send_and_sync() {
    assert_send::<LoanedMessage<test_msgs::msg::rmw::BoundedSequences>>();
    assert_sync::<LoanedMessage<test_msgs::msg::rmw::BoundedSequences>>();
}

#[test]
fn readonly_loaned_message_is_send_and_sync() {
    assert_send::<ReadOnlyLoanedMessage<test_msgs::msg::rmw::BoundedSequences>>();
    assert_sync::<ReadOnlyLoanedMessage<test_msgs::msg::rmw::BoundedSequences>>();
}
