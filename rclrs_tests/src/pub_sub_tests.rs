use rclrs::{
    AnySubscriptionCallback, LoanedMessage, MessageInfo, Publisher, ReadOnlyLoanedMessage,
    Subscription, SubscriptionCallback,
};

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

#[test]
fn callback_conversion() {
    type Message = test_msgs::msg::BoundedSequences;
    let cb = |_msg: Message| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::Regular(_)
    ));
    let cb = |_msg: Message, _info: MessageInfo| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::RegularWithMessageInfo(_)
    ));
    let cb = |_msg: Box<Message>| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::Boxed(_)
    ));
    let cb = |_msg: Box<Message>, _info: MessageInfo| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::BoxedWithMessageInfo(_)
    ));
    let cb = |_msg: ReadOnlyLoanedMessage<'_, Message>| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::Loaned(_)
    ));
    let cb = |_msg: ReadOnlyLoanedMessage<'_, Message>, _info: MessageInfo| {};
    assert!(matches!(
        cb.into_callback(),
        AnySubscriptionCallback::<Message>::LoanedWithMessageInfo(_)
    ));
}
