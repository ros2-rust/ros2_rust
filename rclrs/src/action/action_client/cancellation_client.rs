

pub struct CancellationClient<A: Action> {
    board: Arc<ActionClientGoalBoard<A>>,
}
