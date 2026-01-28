//! Tracking wheel implementation for odometry.
//!
//! This module provides support for tracking wheels used in odometry calculations,
//! allowing robots to track their position and movement using wheel encoders.
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::wheel::{OmniWheel, TrackingWheel};
//! use kernelvex::si::QLength;
//! use kernelvex::wheel::Tracking;
//! use vexide_devices::math::Direction;
//! use vexide_devices::smart::SmartPort;
//! use vexide_devices::smart::rotation::RotationSensor;
//!
//! let encoder = RotationSensor::new(unsafe {SmartPort::new(1)}, Direction::Forward);
//!
//! let mut tracking_wheel = TrackingWheel::new(
//!     encoder,
//!     OmniWheel::Omni275,
//!     QLength::from_inches(5.0),
//!     Some(1.0), // gearing ratio
//! );
//!
//! let distance = tracking_wheel.distance();
//! println!("Distance traveled: {} inches", distance.as_inches());
//! ```

use crate::sensors::Encoder;
use crate::si::QLength;
use crate::utils::Orientation;

/// Types of omni wheels available for tracking.
///
/// Omni wheels come in different sizes (diameter). The size affects the
/// distance calculation when converting encoder rotations to linear distance.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(unused)]
pub enum OmniWheel {
    /// 2.75-inch omni wheel
    Omni275,
    /// 3.25-inch omni wheel
    Omni325,
    /// 4.125-inch omni wheel
    Omni4,
    /// 2.75-inch anti-static wheel
    Anti275,
    /// 3.25-inch anti-static wheel
    Anti325,
    /// 4-inch anti-static wheel
    Anti4,
    /// Custom Length wheel
    Custom(QLength),
}

impl OmniWheel {
    /// Returns the diameter of the wheel.
    ///
    /// # Returns
    ///
    /// The wheel diameter as a [`QLength`].
    #[allow(unused)]
    fn size(&self) -> QLength {
        match *self {
            OmniWheel::Omni275 => QLength::from_inches(2.75),
            OmniWheel::Omni325 => QLength::from_inches(3.25),
            OmniWheel::Omni4 => QLength::from_inches(4.125),
            OmniWheel::Anti275 => QLength::from_inches(2.75),
            OmniWheel::Anti325 => QLength::from_inches(3.25),
            OmniWheel::Anti4 => QLength::from_inches(4.),
            OmniWheel::Custom(d) => d,
        }
    }
}

pub trait Tracking {
    /// Returns the offset of the tracking wheel from the robot's center.
    ///
    /// This is the perpendicular distance from the wheel to the robot's center
    /// of rotation, used in odometry calculations.
    fn offset(&self) -> QLength;

    /// Returns the total distance traveled since initialization or last reset.
    ///
    /// This method accumulates distance and returns the total. For incremental
    /// distance changes, use [`delta`](Tracking::delta).
    fn distance(&mut self) -> QLength;

    /// Resets the accumulated distance counter to zero.
    ///
    /// After calling this method, subsequent calls to `distance()` will measure
    /// from this reset point.
    fn reset(&mut self);
    /// Returns the change in distance since the last call to `delta`.
    ///
    /// This method tracks the incremental distance traveled between calls,
    /// useful for periodic odometry updates.
    ///
    /// # Returns
    ///
    /// The change in distance since the last call, as a [`QLength`].
    fn delta(&mut self) -> QLength;

    fn orientation(&self) -> Orientation;
}

/// A tracking wheel implementation using an encoder.
///
/// `TrackingWheel` converts encoder rotations into linear distance measurements
/// using the wheel diameter and optional gearing ratio. This is the primary
/// implementation of the [`Tracking`] trait for odometry calculations.
///
/// # Type Parameters
///
/// * `T` - The encoder type implementing the [`Encoder`] trait
///

pub struct TrackingWheel<T: Encoder> {
    encoder: T,
    wheel: OmniWheel,
    dist: QLength,
    orientation: Orientation,
    total: QLength,
    gearing: f64,
}

impl<T: Encoder> TrackingWheel<T> {
    /// Creates a new tracking wheel.
    ///
    /// # Arguments
    ///
    /// * `encoder` - The encoder used to measure wheel rotation
    /// * `wheel` - The type of omni wheel being used
    /// * `dist` - The perpendicular offset distance from the robot's center.
    ///            Positive values indicate right side, negative indicates left side
    /// * `gearing` - Optional gearing ratio. If `None`, assumes 1:1 gearing.
    ///              A value of 2.0 means the encoder rotates twice per wheel rotation.
    ///
    /// # Returns
    ///
    /// A new `TrackingWheel` instance with the encoder reset to zero.
    #[allow(unused)]
    pub fn new(encoder: T, wheel: OmniWheel, dist: QLength, ratio: Option<f64>) -> Self {
        let gearing: f64 = ratio.unwrap_or(1.);
        if dist.as_meters() > 0. {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Right,
                total: Default::default(),
                gearing,
            }
        } else {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Left,
                total: Default::default(),
                gearing,
            }
        }
    }
}

impl<T: Encoder> Tracking for TrackingWheel<T> {
    fn offset(&self) -> QLength {
        self.dist
    }

    fn distance(&mut self) -> QLength {
        let circumference = self.wheel.size() * std::f64::consts::PI;

        let distance = circumference * self.gearing * (self.encoder.rotations().as_radians())
            / std::f64::consts::TAU;

        self.total = distance;

        self.total
    }

    fn reset(&mut self) {
        self.total = Default::default();
        let _ = self.encoder.reset();
    }

    fn delta(&mut self) -> QLength {
        let circumference = self.wheel.size() * std::f64::consts::PI;

        let distance = circumference * self.gearing * (self.encoder.rotations().as_radians())
            / std::f64::consts::TAU;

        let ret = distance - self.total;

        self.total = distance;

        ret
    }

    fn orientation(&self) -> Orientation {
        self.orientation
    }
}
