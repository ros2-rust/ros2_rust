use std::sync::Arc;
use crate::{WorkerChannel};

pub type Worker<Payload> = Arc<WorkerState<Payload>>;

pub struct WorkerState<Payload> {
    channel: Box<dyn WorkerChannel>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<T> WorkerState<T> {
    pub(crate) fn new(channel: Box<dyn WorkerChannel>) -> Self {
        Self { channel, _ignore: Default::default() }
    }
}
