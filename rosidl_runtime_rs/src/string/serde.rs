use serde::{de::Error, Deserialize, Deserializer, Serialize, Serializer};

use super::{BoundedString, BoundedWString, String, WString};

impl<'de> Deserialize<'de> for String {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        std::string::String::deserialize(deserializer).map(|s| Self::from(s.as_str()))
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
        std::string::String::deserialize(deserializer).map(|s| Self::from(s.as_str()))
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
