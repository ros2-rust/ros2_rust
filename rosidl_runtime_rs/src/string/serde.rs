use std::fmt;

use serde::{
    de::{Error, SeqAccess, Visitor},
    Deserialize, Deserializer, Serialize, Serializer,
};

use super::{
    rosidl_runtime_c__String__assignn, rosidl_runtime_c__U16String__assignn, BoundedString,
    BoundedWString, String, WString,
};

struct StringVisitor;
struct WStringVisitor;

impl<'de> Visitor<'de> for StringVisitor {
    type Value = String;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a string")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(String::from(v))
    }

    fn visit_bytes<E>(self, v: &[u8]) -> Result<Self::Value, E>
    where
        E: Error,
    {
        let mut msg = String {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        // SAFETY: This is doing the same thing as rosidl_runtime_c__String__copy.
        unsafe {
            rosidl_runtime_c__String__assignn(&mut msg, v.as_ptr() as *const _, v.len());
        }
        Ok(msg)
    }

    // We don't implement visit_bytes_buf, since the data in a string must always be managed by C.
}

impl<'de> Visitor<'de> for WStringVisitor {
    type Value = WString;

    fn expecting(&self, formatter: &mut fmt::Formatter) -> fmt::Result {
        formatter.write_str("a string")
    }

    fn visit_str<E>(self, v: &str) -> Result<Self::Value, E>
    where
        E: Error,
    {
        Ok(WString::from(v))
    }

    fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
    where
        A: SeqAccess<'de>,
    {
        let mut buf = if let Some(size) = seq.size_hint() {
            Vec::with_capacity(size)
        } else {
            Vec::new()
        };
        while let Some(el) = seq.next_element::<u16>()? {
            buf.push(el);
        }
        let mut msg = WString {
            data: std::ptr::null_mut(),
            size: 0,
            capacity: 0,
        };
        // SAFETY: This is doing the same thing as rosidl_runtime_c__U16String__copy.
        unsafe {
            rosidl_runtime_c__U16String__assignn(&mut msg, buf.as_ptr(), buf.len());
        }
        Ok(msg)
    }

    // We don't implement visit_bytes_buf, since the data in a string must always be managed by C.
}

impl<'de> Deserialize<'de> for String {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_string(StringVisitor)
    }
}

impl Serialize for String {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Not particularly efficient
        // SAFETY: See the Display implementation.
        let u8_slice = unsafe { std::slice::from_raw_parts(self.data as *mut u8, self.size) };
        let s = std::string::String::from_utf8_lossy(u8_slice);
        serializer.serialize_str(&s)
    }
}

impl<'de> Deserialize<'de> for WString {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_string(WStringVisitor)
    }
}

impl Serialize for WString {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        // Not particularly efficient
        // SAFETY: See the Display implementation.
        let u16_slice = unsafe { std::slice::from_raw_parts(self.data as *mut u16, self.size) };
        let s = std::string::String::from_utf16_lossy(u16_slice);
        serializer.serialize_str(&s)
    }
}

impl<'de, const N: usize> Deserialize<'de> for BoundedString<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        std::string::String::deserialize(deserializer)
            .and_then(|s| Self::try_from(s.as_str()).map_err(D::Error::custom))
    }
}

impl<const N: usize> Serialize for BoundedString<N> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.inner.serialize(serializer)
    }
}

impl<'de, const N: usize> Deserialize<'de> for BoundedWString<N> {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        std::string::String::deserialize(deserializer)
            .and_then(|s| Self::try_from(s.as_str()).map_err(D::Error::custom))
    }
}

impl<const N: usize> Serialize for BoundedWString<N> {
    fn serialize<S>(&self, serializer: S) -> Result<S::Ok, S::Error>
    where
        S: Serializer,
    {
        self.inner.serialize(serializer)
    }
}

#[cfg(test)]
mod tests {
    use quickcheck::quickcheck;

    use crate::{BoundedString, BoundedWString, String, WString};

    quickcheck! {
        fn test_json_roundtrip_string(s: String) -> bool {
            let value = serde_json::to_value(s.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            s == recovered
        }
    }

    quickcheck! {
        fn test_json_roundtrip_wstring(s: WString) -> bool {
            let value = serde_json::to_value(s.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            s == recovered
        }
    }

    quickcheck! {
        fn test_json_roundtrip_bounded_string(s: BoundedString<256>) -> bool {
            let value = serde_json::to_value(s.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            s == recovered
        }
    }

    quickcheck! {
        fn test_json_roundtrip_bounded_wstring(s: BoundedWString<256>) -> bool {
            let value = serde_json::to_value(s.clone()).unwrap();
            let recovered = serde_json::from_value(value).unwrap();
            s == recovered
        }
    }
}
