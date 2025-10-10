use super::ActionServerHandle;
use crate::{
    log_error,
    rcl_bindings::*,
    vendor::{
        action_msgs::{msg::GoalInfo, srv::CancelGoal_Response},
        unique_identifier_msgs::msg::UUID,
    },
    CancelResponseCode, GoalUuid, Node, RclrsErrorFilter, ToResult,
};
use futures::{
    future::{select, Either},
    pin_mut,
};
use futures_lite::future::race;
use rosidl_runtime_rs::{Action, Message};
use std::{
    borrow::Cow,
    collections::HashSet,
    future::Future,
    sync::{Arc, Mutex},
};
use tokio::sync::watch::{channel as watch_channel, Receiver, Sender};

pub(super) struct CancellationState<A: Action> {
    receiver: Receiver<bool>,
    sender: Sender<bool>,

    /// We put a mutex on the mode because when we respond to cancellation
    /// requests we need to ensure that we update the cancellation mode
    /// atomically
    mode: Mutex<CancellationMode<A>>,
}

impl<A: Action> CancellationState<A> {
    pub(super) fn until_cancel_requested<F: Future + Unpin>(
        &self,
        f: F,
    ) -> impl Future<Output = Result<F::Output, F>> {
        let mut watcher = self.receiver.clone();
        async move {
            let cancel_requested = watcher.wait_for(|request_received| *request_received);
            pin_mut!(cancel_requested);
            match select(f, cancel_requested).await {
                Either::Left((result, _)) => Ok(result),
                Either::Right((_, f)) => Err(f),
            }
        }
    }

    pub(super) fn unless_cancel_requested<F: Future>(
        &self,
        f: F,
    ) -> impl Future<Output = Result<F::Output, ()>> {
        let mut watcher = self.receiver.clone();
        race(async move { Ok(f.await) }, async move {
            let _ = watcher.wait_for(|request_received| *request_received).await;
            Err(())
        })
    }

    /// Check if a cancellation is currently being requested.
    pub(super) fn cancel_requested(&self) -> bool {
        *self.receiver.borrow()
    }

    pub(super) fn request_cancellation(&self, request: CancellationRequest<A>, uuid: &GoalUuid) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::None => {
                let requests = Vec::from_iter([request]);
                *mode = CancellationMode::CancelRequested(requests);
                self.change_cancel_requested_status(true);
            }
            CancellationMode::CancelRequested(requests) => {
                requests.push(request);
                self.change_cancel_requested_status(true);
            }
            CancellationMode::Cancelling => {
                request.accept(*uuid);
            }
        }
    }

    /// Tell current cancellation requesters that their requests are rejected
    pub(super) fn reject_cancellation(&self, uuid: &GoalUuid) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::CancelRequested(requesters) => {
                for requester in requesters.drain(..) {
                    requester.reject(*uuid);
                }

                // Revert to not having any cancellation mode
                *mode = CancellationMode::None;
                // We do not need to worry about errors from sending this state
                // since it is okay for the receiver to be dropped.
                let _ = self.change_cancel_requested_status(false);
            }
            CancellationMode::None => {
                // Do nothing
            }
            CancellationMode::Cancelling => {
                // Do nothing. We will not revert a cancellation state.
            }
        }
    }

    /// Tell current and future cancellation requesters that their requests are
    /// accepted
    pub(super) fn accept_cancellation(&self, uuid: &GoalUuid) {
        let mut mode = self.mode.lock().unwrap();
        match &mut *mode {
            CancellationMode::CancelRequested(requesters) => {
                for requester in requesters.drain(..) {
                    requester.accept(*uuid);
                }

                // Progress to cancelling mode
                *mode = CancellationMode::Cancelling;
                // Just in case this signal was never sent, make sure we have
                // a true value in the cancel requested channel. We can ignore
                // errors from this because it is okay for the receiver to be
                // dropped.
                let _ = self.change_cancel_requested_status(true);
            }
            CancellationMode::None => {
                // Skip straight to cancellation mode since the user has accepted
                // a cancellation even though it wasn't requested externally.
                *mode = CancellationMode::Cancelling;
                // Make sure the cancellation is signalled. We can ignore errors
                // from this because it is okay for the receiver to be dropped.
                let _ = self.change_cancel_requested_status(true);
            }
            CancellationMode::Cancelling => {
                // Do nothing
            }
        }
    }

    fn change_cancel_requested_status(&self, cancel_requested: bool) {
        self.sender.send_if_modified(|status| {
            let previously_requested = *status;
            *status = cancel_requested;
            // Only notify the listeners if a cancellation has been requested
            // and was not previously requested. We do not need to notify when
            // a cancellation has been rejected (i.e. reverting back to no
            // cancellation requested).
            cancel_requested && !previously_requested
        });
    }
}

impl<A: Action> Default for CancellationState<A> {
    fn default() -> Self {
        let (sender, receiver) = watch_channel(false);
        Self {
            receiver,
            sender,
            mode: Mutex::new(CancellationMode::None),
        }
    }
}

pub(super) enum CancellationMode<A: Action> {
    None,
    CancelRequested(Vec<CancellationRequest<A>>),
    Cancelling,
}

/// This struct exists to deal with the fact that a single cancellation request
/// can trigger multiple goal cancellations at once. This allows us to
/// asynchronously receive the accept/reject results from all the different goals
/// and then issue the reply once all are received.
pub(super) struct CancellationRequest<A: Action> {
    inner: Arc<Mutex<CancellationRequestInner<A>>>,
}

impl<A: Action> Clone for CancellationRequest<A> {
    fn clone(&self) -> Self {
        Self {
            inner: Arc::clone(&self.inner),
        }
    }
}

impl<A: Action> CancellationRequest<A> {
    pub(super) fn new(
        id: rmw_request_id_t,
        waiting_for: Vec<GoalUuid>,
        server: Arc<ActionServerHandle<A>>,
        node: Node,
    ) -> Self {
        Self {
            inner: Arc::new(Mutex::new(CancellationRequestInner {
                id,
                waiting_for,
                server,
                node,
                received: Default::default(),
                accepted: Vec::new(),
                response_sent: false,
            })),
        }
    }

    pub(super) fn accept(&self, uuid: GoalUuid) {
        let mut inner = self.inner.lock().unwrap();
        if !inner.received.insert(uuid) {
            return;
        }

        let stamp = inner
            .node
            .get_clock()
            .now()
            .to_ros_msg()
            .unwrap_or_default();
        let info = GoalInfo {
            goal_id: UUID { uuid: *uuid },
            stamp,
        };

        inner.accepted.push(info);
        inner.respond_if_ready();
    }

    fn reject(&self, uuid: GoalUuid) {
        let mut inner = self.inner.lock().unwrap();
        if !inner.received.insert(uuid) {
            return;
        }

        inner.respond_if_ready();
    }
}

struct CancellationRequestInner<A: Action> {
    id: rmw_request_id_t,
    waiting_for: Vec<GoalUuid>,
    received: HashSet<GoalUuid>,
    accepted: Vec<GoalInfo>,
    response_sent: bool,
    server: Arc<ActionServerHandle<A>>,
    node: Node,
}

impl<A: Action> CancellationRequestInner<A> {
    fn respond_if_ready(&mut self) {
        for expected in &self.waiting_for {
            if !self.received.contains(expected) {
                return;
            }
        }

        self.respond();
    }

    fn respond(&mut self) {
        if self.response_sent {
            return;
        }

        self.response_sent = true;

        let mut response = CancelGoal_Response::default();
        response.goals_canceling = self.accepted.drain(..).collect();
        if response.goals_canceling.is_empty() {
            response.return_code = CancelResponseCode::Reject as i8;
        } else {
            response.return_code = CancelResponseCode::Accept as i8;
        }

        let mut response_rmw =
            CancelGoal_Response::into_rmw_message(Cow::Owned(response)).into_owned();
        let r = unsafe {
            // SAFETY: The action server handle is locked and so synchronized with other functions.
            // The request_id and response are both uniquely owned or borrowed, and so neither will
            // mutate during this function call.
            rcl_action_send_cancel_response(
                &*self.server.lock(),
                &mut self.id,
                &mut response_rmw as *mut _ as *mut _,
            )
        }
        .ok()
        .timeout_ok();

        if let Err(err) = r {
            log_error!(
                "CancellationRequest.respond",
                "Error occurred while responding to a cancellation request: {err}"
            )
        }
    }
}

impl<A: Action> Drop for CancellationRequestInner<A> {
    fn drop(&mut self) {
        if !self.response_sent {
            // As a last resort, send the response if all possible responders
            // have dropped.
            self.respond();
        }
    }
}
