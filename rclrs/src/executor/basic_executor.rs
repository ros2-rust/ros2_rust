use crate::executor::{ExecutorRuntime, ExecutorChannel, SpinConditions};
use futures::future::BoxFuture;

#[derive(Default)]
pub struct BasicExecutorRuntime {

}

impl ExecutorRuntime for BasicExecutorRuntime {
    fn spin(&mut self, conditions: SpinConditions) {

    }

    fn spin_async(
        self: Box<Self>,
        conditions: SpinConditions,
    ) -> BoxFuture<'static, Box<dyn ExecutorRuntime>> {
        Box::pin(async move { self as Box<dyn ExecutorRuntime> })
    }

    fn channel(&self) -> Box<dyn ExecutorChannel> {
        Box::new(BasicExecutorChannel { })
    }
}

pub struct BasicExecutorChannel {

}

impl ExecutorChannel for BasicExecutorChannel {
    fn add(&self, f: futures::future::BoxFuture<'static, ()>) {

    }

    fn wakeup(&self) {

    }
}
