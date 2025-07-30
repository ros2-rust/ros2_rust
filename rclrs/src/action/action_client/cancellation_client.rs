use super::GoalClientLifecycle;
use rosidl_runtime_rs::Action;
use tokio::sync::mpsc::UnboundedReceiver;
use std::{
    ops::{Deref, DerefMut},
    sync::Arc,
};

pub struct CancellationClient<A: Action> {
    board: Arc<GoalClientLifecycle<A>>,
}
