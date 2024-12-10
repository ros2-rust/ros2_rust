use std::sync::Arc;
use crate::{WorkerCommands};

pub type Worker<Payload> = Arc<WorkerState<Payload>>;

pub struct WorkerState<Payload> {
    commands: Arc<WorkerCommands>,
    _ignore: std::marker::PhantomData<Payload>,
}

impl<T> WorkerState<T> {
    pub(crate) fn new(commands: Arc<WorkerCommands>) -> Self {
        Self { commands, _ignore: Default::default() }
    }
}
