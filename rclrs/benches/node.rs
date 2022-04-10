use rclrs::{Context, Publisher, Subscription, QOS_PROFILE_DEFAULT, RclReturnCode};
use std::{env, println};
use cstr_core::CString;
use std_msgs;
use criterion::{criterion_group, criterion_main, Criterion, SamplingMode};

fn default_context() -> Context {
    let args: Vec<CString> = env::args()
        .filter_map(|arg| CString::new(arg).ok())
        .collect();
    println!("<test_rclrs_mod> Context args: {:?}", args);
    Context::default(args)
}

fn bench_publisher(c: &mut Criterion)->Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Publisher");
    group.sampling_mode(SamplingMode::Auto);

    let context = default_context();
    let node =  context.create_node("bench_publisher")?;


    // Benchmark publisher creation
    group.bench_function("Create publisher", |b| {
        b.iter(||->Result<(), RclReturnCode>  {
            let _temp_publisher =
                Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;
            Ok(())
        })});

    let mut message = std_msgs::msg::String {
        data: "Hello world!".to_owned(),
    };
    let mut publish_count: u32 = 1;
    let publisher = Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;

    // Benchmark publish simple string 100 times
    group.bench_function("Publish", |b| {
        b.iter(||->Result<(), RclReturnCode>  {
            // Publish 100 times
            publish_count = 0;
            while context.ok()? {
                message.data = format!("Hello, world! {}", publish_count);
                publisher.publish(&message)?;
                publish_count += 1;
                if publish_count >= 100{
                    break;
                }
            }
            Ok(())
        })});

    group.finish();
    Ok(())

}

fn bench_subscriber(c: &mut Criterion)->Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Subscriber");
    group.sampling_mode(SamplingMode::Auto);

    let context = default_context();
    let mut node_sub = context.create_node("bench_subscriber")?;
    let node_pub = context.create_node("bench_publisher")?;

    // Benchmark subscriber creation
    group.bench_function("Create subscriber", |b| {
        b.iter(||->Result<(), RclReturnCode>  {
            let _temp_subscriber = Subscription::<std_msgs::msg::String>::new(
                &node_pub,
                "test",
                QOS_PROFILE_DEFAULT,
                move |msg: &std_msgs::msg::String| {
                    println!("Received message: '{}'", msg.data);
                },
            )?;
            Ok(())
        })});

    let mut message = std_msgs::msg::String {
        data: "Hello world!".to_owned(),
    };
    let mut num_messages_received: usize = 0;
    let _subscriber = node_sub
        .create_subscription::<std_msgs::msg::String, _>(
        "test",
        QOS_PROFILE_DEFAULT,
        move |_msg: &std_msgs::msg::String| {
            num_messages_received = &num_messages_received + 1;
        },
    )?;

    let publisher = Publisher::<std_msgs::msg::String>::new(&node_pub, "test", QOS_PROFILE_DEFAULT)?;


    let mut publish_count: u32 = 1;
    // Benchmark publish and receive simple string 100 times
    group.bench_function("Publish/Receive", |b| {
        b.iter(||->Result<(), RclReturnCode>  {
            // Publish 100 times
            publish_count = 0;
            while context.ok()? {
                message.data = format!("Hello, world! {}", publish_count);
                publisher.publish(&message)?;
                publish_count += 1;
                if publish_count >= 100{
                    break;
                }
                rclrs::spin_once(&node_sub, 500);
            }
            Ok(())
        })});
    group.finish();
    Ok(())

}
criterion_group!(benches, bench_publisher, bench_subscriber);
criterion_main!(benches);