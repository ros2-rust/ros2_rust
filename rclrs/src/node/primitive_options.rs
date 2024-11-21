use crate::{
    QoSDurabilityPolicy, QoSDuration, QoSHistoryPolicy, QoSLivelinessPolicy, QoSProfile,
    QoSReliabilityPolicy,
};

use std::{borrow::Borrow, time::Duration};

/// `PrimitiveOptions` are the subset of options that are relevant across all
/// primitives (e.g. [`Subscription`][1], [`Publisher`][2], [`Client`][3], and
/// [`Service`][4]).
///
/// Each different primitive type may have its own defaults for the overall
/// quality of service settings, and we cannot know what the default will be
/// until the `PrimitiveOptions` gets converted into the more specific set of
/// options. Therefore we store each quality of service field separately so that
/// we will only override the settings that the user explicitly asked for, and
/// the rest will be determined by the default settings for each primitive.
///
/// [1]: crate::Subscription
/// [2]: crate::Publisher
/// [3]: crate::Client
/// [4]: crate::Service
#[derive(Debug, Clone, Copy)]
#[non_exhaustive]
pub struct PrimitiveOptions<'a> {
    /// The name that will be used for the primitive
    pub name: &'a str,
    /// Override the default [`QoSProfile::history`] for the primitive.
    pub history: Option<QoSHistoryPolicy>,
    /// Override the default [`QoSProfile::reliability`] for the primitive.
    pub reliability: Option<QoSReliabilityPolicy>,
    /// Override the default [`QoSProfile::durability`] for the primitive.
    pub durability: Option<QoSDurabilityPolicy>,
    /// Override the default [`QoSProfile::deadline`] for the primitive.
    pub deadline: Option<QoSDuration>,
    /// Override the default [`QoSProfile::lifespan`] for the primitive.
    pub lifespan: Option<QoSDuration>,
    /// Override the default [`QoSProfile::liveliness`] for the primitive.
    pub liveliness: Option<QoSLivelinessPolicy>,
    /// Override the default [`QoSProfile::liveliness_lease`] for the primitive.
    pub liveliness_lease: Option<QoSDuration>,
    /// Override the default [`QoSProfile::avoid_ros_namespace_conventions`] for the primitive.
    pub avoid_ros_namespace_conventions: Option<bool>,
}

/// Trait to implicitly convert a compatible object into [`PrimitiveOptions`].
pub trait IntoPrimitiveOptions<'a>: Sized {
    /// Convert the object into [`PrimitiveOptions`] with default settings.
    fn into_primitive_options(self) -> PrimitiveOptions<'a>;

    /// Override all the quality of service settings for the primitive.
    fn qos(self, profile: QoSProfile) -> PrimitiveOptions<'a> {
        self.into_primitive_options().history(profile.history)
    }

    /// Use the default topics quality of service profile.
    fn topics_qos(self) -> PrimitiveOptions<'a> {
        self.qos(QoSProfile::topics_default())
    }

    /// Use the default sensor data quality of service profile.
    fn sensor_data_qos(self) -> PrimitiveOptions<'a> {
        self.qos(QoSProfile::sensor_data_default())
    }

    /// Use the default services quality of service profile.
    fn services_qos(self) -> PrimitiveOptions<'a> {
        self.qos(QoSProfile::services_default())
    }

    /// Use the system-defined default quality of service profile. This profile
    /// is determined by the underlying RMW implementation, so you cannot rely
    /// on this profile being consistent or appropriate for your needs.
    fn system_qos(self) -> PrimitiveOptions<'a> {
        self.qos(QoSProfile::system_default())
    }

    /// Override the default [`QoSProfile::history`] for the primitive.
    fn history(self, history: QoSHistoryPolicy) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.history = Some(history);
        options
    }

    /// Keep the last `depth` messages for the primitive.
    fn keep_last(self, depth: u32) -> PrimitiveOptions<'a> {
        self.history(QoSHistoryPolicy::KeepLast { depth })
    }

    /// Keep all messages for the primitive.
    fn keep_all(self) -> PrimitiveOptions<'a> {
        self.history(QoSHistoryPolicy::KeepAll)
    }

    /// Override the default [`QoSProfile::reliability`] for the primitive.
    fn reliability(self, reliability: QoSReliabilityPolicy) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.reliability = Some(reliability);
        options
    }

    /// Set the primitive to have [reliable][QoSReliabilityPolicy::Reliable] communication.
    fn reliable(self) -> PrimitiveOptions<'a> {
        self.reliability(QoSReliabilityPolicy::Reliable)
    }

    /// Set the primitive to have [best-effort][QoSReliabilityPolicy::BestEffort] communication.
    fn best_effort(self) -> PrimitiveOptions<'a> {
        self.reliability(QoSReliabilityPolicy::BestEffort)
    }

    /// Override the default [`QoSProfile::durability`] for the primitive.
    fn durability(self, durability: QoSDurabilityPolicy) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.durability = Some(durability);
        options
    }

    /// Set the primitive to have [volatile][QoSDurabilityPolicy::Volatile] durability.
    fn volatile(self) -> PrimitiveOptions<'a> {
        self.durability(QoSDurabilityPolicy::Volatile)
    }

    /// Set the primitive to have [transient local][QoSDurabilityPolicy::TransientLocal] durability.
    fn transient_local(self) -> PrimitiveOptions<'a> {
        self.durability(QoSDurabilityPolicy::TransientLocal)
    }

    /// Override the default [`QoSProfile::lifespan`] for the primitive.
    fn lifespan(self, lifespan: QoSDuration) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.lifespan = Some(lifespan);
        options
    }

    /// Set a custom duration for the [lifespan][QoSProfile::lifespan] of the primitive.
    fn lifespan_duration(self, duration: Duration) -> PrimitiveOptions<'a> {
        self.lifespan(QoSDuration::Custom(duration))
    }

    /// Make the [lifespan][QoSProfile::lifespan] of the primitive infinite.
    fn infinite_lifespan(self) -> PrimitiveOptions<'a> {
        self.lifespan(QoSDuration::Infinite)
    }

    /// Override the default [`QoSProfile::deadline`] for the primitive.
    fn deadline(self, deadline: QoSDuration) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.deadline = Some(deadline);
        options
    }

    /// Set the [`QoSProfile::deadline`] to a custom finite value.
    fn deadline_duration(self, duration: Duration) -> PrimitiveOptions<'a> {
        self.deadline(QoSDuration::Custom(duration))
    }

    /// Do not use a deadline for liveliness for this primitive.
    fn no_deadline(self) -> PrimitiveOptions<'a> {
        self.deadline(QoSDuration::Infinite)
    }

    /// Override the default [`QoSProfile::liveliness_lease`] for the primitive.
    fn liveliness_lease(self, lease: QoSDuration) -> PrimitiveOptions<'a> {
        let mut options = self.into_primitive_options();
        options.liveliness_lease = Some(lease);
        options
    }

    /// Set a custom duration for the [liveliness lease][QoSProfile::liveliness_lease].
    fn liveliness_lease_duration(self, duration: Duration) -> PrimitiveOptions<'a> {
        self.liveliness_lease(QoSDuration::Custom(duration))
    }
}

impl<'a> IntoPrimitiveOptions<'a> for PrimitiveOptions<'a> {
    fn into_primitive_options(self) -> PrimitiveOptions<'a> {
        self
    }
}

impl<'a> IntoPrimitiveOptions<'a> for &'a str {
    fn into_primitive_options(self) -> PrimitiveOptions<'a> {
        PrimitiveOptions::new(self)
    }
}

impl<'a, T: Borrow<str>> IntoPrimitiveOptions<'a> for &'a T {
    fn into_primitive_options(self) -> PrimitiveOptions<'a> {
        self.borrow().into_primitive_options()
    }
}

impl<'a> PrimitiveOptions<'a> {
    /// Begin building a new set of `PrimitiveOptions` with only the name set.
    pub fn new(name: &'a str) -> Self {
        Self {
            name,
            history: None,
            reliability: None,
            durability: None,
            deadline: None,
            lifespan: None,
            liveliness: None,
            liveliness_lease: None,
            avoid_ros_namespace_conventions: None,
        }
    }

    /// Apply the user-specified options to a pre-initialized [`QoSProfile`].
    pub fn apply(&self, qos: &mut QoSProfile) {
        if let Some(history) = self.history {
            qos.history = history;
        }

        if let Some(reliability) = self.reliability {
            qos.reliability = reliability;
        }

        if let Some(durability) = self.durability {
            qos.durability = durability;
        }

        if let Some(deadline) = self.deadline {
            qos.deadline = deadline;
        }

        if let Some(lifespan) = self.lifespan {
            qos.lifespan = lifespan;
        }

        if let Some(liveliness) = self.liveliness {
            qos.liveliness = liveliness;
        }

        if let Some(liveliness_lease) = self.liveliness_lease {
            qos.liveliness_lease = liveliness_lease;
        }

        if let Some(convention) = self.avoid_ros_namespace_conventions {
            qos.avoid_ros_namespace_conventions = convention;
        }
    }
}
