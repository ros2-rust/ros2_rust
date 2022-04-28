use std::env;

use criterion::{Criterion, criterion_group, criterion_main, SamplingMode};
use cstr_core::CString;
use std_msgs;

use rclrs::{Context, Publisher, QOS_PROFILE_DEFAULT, RclReturnCode, Subscription};

fn default_context() -> Context {
    let args: Vec<CString> = env::args()
        .filter_map(|arg| CString::new(arg).ok())
        .collect();

    Context::default(args)
}

fn bench_publisher(c: &mut Criterion) -> Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Publisher");
    group.sampling_mode(SamplingMode::Auto);

    let context = default_context();
    let node = context.create_node("bench_publisher")?;


    // Benchmark publisher creation
    group.bench_function("Create publisher", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            let _temp_publisher =
                Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;
            Ok(())
        })
    });

    let mut message = std_msgs::msg::String {
        data: "Hello world!".to_owned(),
    };
    let mut publish_count: u32 = 1;
    let publisher = Publisher::<std_msgs::msg::String>::new(&node, "test", QOS_PROFILE_DEFAULT)?;

    // Benchmark publish simple string 100 times
    group.bench_function("Publish", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            // Publish 100 times
            publish_count = 0;
            while context.ok()? {
                message.data = format!("Hello, world! {}", publish_count);
                publisher.publish(&message)?;
                publish_count += 1;
                if publish_count >= 100 {
                    break;
                }
            }
            Ok(())
        })
    });

    group.finish();
    Ok(())
}

fn bench_node_creation(c: &mut Criterion) -> Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Node");
    group.sampling_mode(SamplingMode::Auto);

    // Benchmark publisher creation
    group.bench_function("Create node", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            let context = default_context();
            let _node = context.create_node("node_test")?;
            Ok(())
        })
    });


    group.finish();
    Ok(())
}

fn bench_context_creation(c: &mut Criterion) -> Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Context");
    group.sampling_mode(SamplingMode::Auto);

    // Benchmark publisher creation
    group.bench_function("Create context", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            let _context = default_context();
            Ok(())
        })
    });


    group.finish();
    Ok(())
}

fn bench_subscriber(c: &mut Criterion) -> Result<(), RclReturnCode> {
    let mut group = c.benchmark_group("Subscriber");
    group.sampling_mode(SamplingMode::Auto);

    let context = default_context();
    let mut node_sub = context.create_node("bench_subscriber")?;
    let node_pub = context.create_node("bench_publisher")?;

    // Benchmark subscriber creation
    group.bench_function("Create subscriber", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            let _temp_subscriber = Subscription::<std_msgs::msg::String>::new(
                &node_pub,
                "test",
                QOS_PROFILE_DEFAULT,
                move |msg: &std_msgs::msg::String| {
                    let _temp_msg = msg;
                },
            )?;
            Ok(())
        })
    });

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
    group.bench_function("Publish/Receive String", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            // Publish 100 times
            publish_count = 0;
            while context.ok()? {
                message.data = format!("Hello, world! {}", publish_count);
                publisher.publish(&message)?;
                publish_count += 1;
                if publish_count >= 100 {
                    break;
                }
                let _ = rclrs::spin_once(&node_sub, 500);
            }
            Ok(())
        })
    });
    num_messages_received = 0;
    let _subscriber = node_sub
        .create_subscription::<std_msgs::msg::Int32, _>(
            "test_int",
            QOS_PROFILE_DEFAULT,
            move |_msg: &std_msgs::msg::Int32| {
                num_messages_received = &num_messages_received + 1;
            },
        )?;

    let publisher = Publisher::<std_msgs::msg::Int32>::new(&node_pub, "test_int", QOS_PROFILE_DEFAULT)?;

    let mut message = std_msgs::msg::Int32 {
        data: 0,
    };

    // Benchmark publish and receive trivially copyable object 100 times
    group.bench_function("Publish/Receive i32", |b| {
        b.iter(|| -> Result<(), RclReturnCode>  {
            // Publish 100 times
            publish_count = 0;
            while context.ok()? {
                message.data = 42;
                publisher.publish(&message)?;
                publish_count += 1;
                if publish_count >= 100 {
                    break;
                }
                let _ = rclrs::spin_once(&node_sub, 500);
            }
            Ok(())
        })
    });
    group.finish();
    Ok(())
}
criterion_group!(benches, bench_publisher, bench_subscriber, bench_node_creation, bench_context_creation);
criterion_main!(benches);