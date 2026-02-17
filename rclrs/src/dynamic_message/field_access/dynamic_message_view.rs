use std::fmt::{self, Debug};
use std::ops::Deref;

use super::super::MessageStructure;
use super::{DynamicSequenceElementMut, Proxy, ProxyMut, ProxySequence, Value, ValueMut};

/// A view of a single message. Used for nested messages.
///
/// This allows reading the fields of the message, but not modifying them.
#[derive(PartialEq, Eq)]
pub struct DynamicMessageView<'msg> {
    pub(crate) structure: &'msg MessageStructure,
    pub(crate) storage: &'msg [u8],
}

/// A mutable view of a single message. Used for nested messages.
///
/// This allows reading and modifying the fields of the message.
#[derive(PartialEq, Eq)]
pub struct DynamicMessageViewMut<'msg> {
    pub(crate) structure: &'msg MessageStructure,
    pub(crate) storage: &'msg mut [u8],
}

// ========================= impl for DynamicMessageView =========================

impl<'msg> Debug for DynamicMessageView<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        let mut struct_ = f.debug_struct(&self.structure().type_name);
        for field in &self.structure().fields {
            let value = self.get(&field.name).unwrap();
            struct_.field(&field.name, &value as &dyn Debug);
        }
        struct_.finish()
    }
}

impl<'msg> Deref for DynamicMessageView<'msg> {
    type Target = MessageStructure;
    fn deref(&self) -> &Self::Target {
        self.structure
    }
}

unsafe impl<'msg> Proxy<'msg> for DynamicMessageView<'msg> {
    type Metadata = &'msg MessageStructure;

    fn size_in_memory(structure: Self::Metadata) -> usize {
        structure.size
    }

    unsafe fn new(bytes: &'msg [u8], structure: Self::Metadata) -> Self {
        DynamicMessageView {
            structure,
            storage: bytes,
        }
    }
}

impl<'msg> DynamicMessageView<'msg> {
    /// Tries to access a field in the message.
    ///
    /// If no field of that name exists, `None` is returned.
    pub fn get(&self, field_name: &str) -> Option<Value<'msg>> {
        let field_info = self.structure.get_field_info(field_name)?;
        // For the unwrap_or, see DynamicMessageViewMut::get_mut
        let size = field_info.size().unwrap_or(1);
        let bytes = &self.storage[field_info.offset..field_info.offset + size];
        // SAFETY: The bytes contain a valid field of the type recorded in field_info.
        unsafe { Value::new(bytes, field_info) }
    }

    /// Returns a description of the message structure.
    pub fn structure(&self) -> &MessageStructure {
        self.structure
    }

    /// Iterate over all fields in declaration order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, Value<'_>)> + '_ {
        self.structure.fields.iter().map(|field_info| {
            let value = self.get(&field_info.name).unwrap();
            (field_info.name.as_str(), value)
        })
    }
}

// ========================= impl for DynamicMessageViewMut =========================

impl<'msg> Debug for DynamicMessageViewMut<'msg> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> Result<(), fmt::Error> {
        DynamicMessageView {
            structure: self.structure,
            storage: &*self.storage,
        }
        .fmt(f)
    }
}

impl<'msg> Deref for DynamicMessageViewMut<'msg> {
    type Target = MessageStructure;
    fn deref(&self) -> &Self::Target {
        self.structure
    }
}

impl<'msg> DynamicSequenceElementMut<'msg> for DynamicMessageViewMut<'msg> {
    type InnerSequence = ProxySequence<'msg, Self>;
}

unsafe impl<'msg> ProxyMut<'msg> for DynamicMessageViewMut<'msg> {
    type Metadata = &'msg MessageStructure;

    fn size_in_memory(structure: Self::Metadata) -> usize {
        structure.size
    }

    unsafe fn new(bytes: &'msg mut [u8], structure: Self::Metadata) -> Self {
        DynamicMessageViewMut {
            structure,
            storage: bytes,
        }
    }
}

impl<'msg> DynamicMessageViewMut<'msg> {
    /// Tries to access a field in the message.
    ///
    /// If no field of that name exists, `None` is returned.
    pub fn get(&self, field_name: &str) -> Option<Value<'_>> {
        let field_info = self.structure.get_field_info(field_name)?;
        // For the unwrap_or, see DynamicMessageViewMut::get_mut
        let size = field_info.size().unwrap_or(1);
        let bytes = &self.storage[field_info.offset..field_info.offset + size];
        // SAFETY: The bytes contain a valid field of the type recorded in field_info.
        unsafe { Value::new(bytes, field_info) }
    }

    /// Tries to mutably access a field in the message.
    ///
    /// If no field of that name exists, `None` is returned.
    pub fn get_mut(&mut self, field_name: &str) -> Option<ValueMut<'_>> {
        let field_info = self.structure.get_field_info(field_name)?;
        // The size is None for LongDouble, which has platform-dependent size.
        // It's fine to pass in 1 here – the length of the slice isn't strictly needed
        // by this function, especially not for a LongDouble value.
        let size = field_info.size().unwrap_or(1);
        let bytes = &mut self.storage[field_info.offset..field_info.offset + size];
        // SAFETY: The bytes contain a valid field of the type recorded in field_info.
        Some(unsafe { ValueMut::new(bytes, field_info) })
    }

    /// Returns a description of the message structure.
    pub fn structure(&self) -> &MessageStructure {
        self.structure
    }

    /// Iterate over all fields in declaration order.
    pub fn iter(&self) -> impl Iterator<Item = (&str, Value<'_>)> + '_ {
        self.structure.fields.iter().map(|field_info| {
            let value = self.get(&field_info.name).unwrap();
            (field_info.name.as_str(), value)
        })
    }

    /// Iterate over all fields in declaration order (mutable version).
    ///
    /// Note that, unusually for an `iter_mut()` method, this method takes `self`
    /// and not `&mut self`. This is because the values should borrow directly from
    /// the message, not the `MessageViewMut`.
    pub fn iter_mut(self) -> impl Iterator<Item = (&'msg str, ValueMut<'msg>)> + 'msg {
        // This looks different from iter() because naively calling self.get_mut() for
        // each field would create multiple mutable references into the same slice.
        // Create an iterator that contains the _end_ offset of each field.
        let offsets_next = self
            .structure
            .fields
            .iter()
            .map(|field_info| field_info.offset)
            .chain([self.structure.size])
            .skip(1);
        // By zipping, we have info about the start offset and end offset of each field.
        self.structure.fields.iter().zip(offsets_next).scan(
            self.storage,
            |remainder: &mut &'msg mut [u8], (field_info, next_field_offset)| {
                // Chop off bytes of the field's size from the front.
                // remainder is of type &'closure mut &'a mut [i32],
                // and calling remainder.split_at_mut would move out of
                // the outer reference, so it's forbidden
                let rem = std::mem::take(remainder);
                let (value_bytes, rem) = rem.split_at_mut(next_field_offset - field_info.offset);
                *remainder = rem;

                // SAFETY: The bytes contain a valid field of the type recorded in field_info.
                let value = unsafe { ValueMut::new(value_bytes, field_info) };
                Some((field_info.name.as_str(), value))
            },
        )
    }
}
