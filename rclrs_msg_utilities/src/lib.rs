// Copyright 2020 DCS Corporation, All Rights Reserved.

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// DISTRIBUTION A. Approved for public release; distribution unlimited.
// OPSEC #4584.

#![no_std]


// use downcast::{downcast, downcast_methods, downcast_methods_core, impl_downcast, Any};

// #[cfg(feature = "std")]
// use downcast::downcast_methods_std;


pub mod traits {
    extern crate downcast_rs;
    extern crate libc;

    use downcast_rs::{Downcast, impl_downcast};

    use libc::uintptr_t;

    pub trait Message: Downcast {
        fn get_native_message(&self) -> uintptr_t;
        fn destroy_native_message(&self, message_handle: uintptr_t);
        fn read_handle(&mut self, message_handle: uintptr_t);
    }

    impl_downcast!(Message);

    pub trait MessageDefinition<T>: Message {
        fn get_type_support() -> uintptr_t;
        fn static_get_native_message(message: &T) -> uintptr_t;
        fn static_destroy_native_message(message_handle: uintptr_t);
    }

}
