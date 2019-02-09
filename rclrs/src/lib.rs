pub use self::context::*;
pub use self::error::*;
pub use self::node::*;
pub use self::qos::*;
use std::ops::{Deref, DerefMut};

pub trait Handle<T> {
    type DerefT: Deref<Target = T>;
    type DerefMutT: DerefMut<Target = T>;

    fn get(self) -> Self::DerefT;
    fn get_mut(self) -> Self::DerefMutT;
}

pub mod context;
pub mod error;
pub mod node;
pub mod qos;
