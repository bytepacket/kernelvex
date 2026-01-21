//! Tracking wheel implementation for odometry.
//!
//! This module provides support for tracking wheels used in odometry calculations,
//! allowing robots to track their position and movement using wheel encoders.
//!
//! # Overview
//!
//! Tracking wheels are passive omni-directional wheels mounted on a robot with encoders
//! to measure their rotation. By tracking the rotation of these wheels, you can calculate
//! the robot's movement and position over time (odometry).
//!
//! ## Common Configurations
//!
//! - **Single Parallel Wheel**: Measures forward/backward motion only
//! - **Dual Parallel Wheels**: Two wheels on opposite sides for better accuracy
//! - **Three-Wheel Odometry**: Two parallel + one perpendicular for full 2D tracking (x, y, heading)
//!
//! # Examples
//!
//! ## Basic Usage
//!
//! ```no_run
//! use kernelvex::omniwheel::{OmniWheel, TrackingWheel, Tracking};
//! use kernelvex::si::QLength;
//! # use kernelvex::sensors::Encoder;
//! # let encoder: impl Encoder = todo!();
//!
//! // Create a tracking wheel
//! let mut tracking_wheel = TrackingWheel::new(
//!     encoder,
//!     OmniWheel::Omni275,              // 2.75" wheel
//!     QLength::from_inches(5.0),        // 5" offset from center
//!     Some(1.0),                        // 1:1 gearing ratio
//! );
//!
//! // Read total distance traveled
//! let distance = tracking_wheel.distance();
//!
//! // Get incremental change since last reading
//! let delta = tracking_wheel.delta();
//!
//! // Reset the tracking wheel
//! tracking_wheel.reset();
//! ```
//!
//! ## Using the Builder Pattern
//!
//! ```no_run
//! use kernelvex::omniwheel::{OmniWheel, TrackingWheelBuilder};
//! use kernelvex::si::QLength;
//! # use kernelvex::sensors::Encoder;
//! # let encoder: impl Encoder = todo!();
//!
//! let mut wheel = TrackingWheelBuilder::new(encoder)
//!     .wheel_type(OmniWheel::Omni325)
//!     .offset(QLength::from_inches(6.0))
//!     .gearing(2.0)
//!     .reversed(false)
//!     .build();
//! ```
//!
//! ## Dual Wheel Configuration
//!
//! ```no_run
//! use kernelvex::omniwheel::{OmniWheel, TrackingWheel, Tracking};
//! use kernelvex::si::QLength;
//! # use kernelvex::sensors::Encoder;
//! # let left_encoder: impl Encoder = todo!();
//! # let right_encoder: impl Encoder = todo!();
//!
//! // Create left and right tracking wheels
//! let mut left_wheel = TrackingWheel::new(
//!     left_encoder,
//!     OmniWheel::Omni275,
//!     QLength::from_inches(-6.0),  // Negative = left side
//!     None,
//! );
//!
//! let mut right_wheel = TrackingWheel::new(
//!     right_encoder,
//!     OmniWheel::Omni275,
//!     QLength::from_inches(6.0),   // Positive = right side
//!     None,
//! );
//!
//! // Average both wheels for better accuracy
//! let avg_distance = (left_wheel.distance() + right_wheel.distance()) / 2.0;
//! ```
//!
//! ## Encoder Reversal
//!
//! If an encoder is reading backwards (wrong mounting direction), you can reverse it:
//!
//! ```no_run
//! use kernelvex::omniwheel::{OmniWheel, TrackingWheel};
//! use kernelvex::si::QLength;
//! # use kernelvex::sensors::Encoder;
//! # let encoder: impl Encoder = todo!();
//!
//! let mut wheel = TrackingWheel::new(
//!     encoder,
//!     OmniWheel::Omni275,
//!     QLength::from_inches(5.0),
//!     None,
//! );
//!
//! // Reverse the encoder direction
//! wheel.set_reversed(true);
//! ```
//!
//! # Tips
//!
//! 1. **Calibration**: Measure actual wheel diameters - manufacturing tolerances matter
//! 2. **Gearing**: Account for external gearing between encoder and wheel
//! 3. **Direction**: Use `set_reversed()` if encoder reads backwards
//! 4. **Mounting**: Ensure wheels spin freely with good field contact
//! 5. **Update Rate**: Poll at 10-50ms intervals for smooth odometry


use crate::sensors::Encoder;
use crate::si::QLength;
use crate::utils::Orientation;

/// Types of omni wheels available for tracking.
///
/// Omni wheels come in different sizes (diameter). The size affects the
/// distance calculation when converting encoder rotations to linear distance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(unused)]
pub enum OmniWheel {
    /// 2.75 inch omni wheel
    Omni275,
    /// 3.25 inch omni wheel
    Omni325,
    /// 4.125 inch omni wheel
    Omni4,
    /// 2.75 inch anti-static wheel
    Anti275,
    /// 3.25 inch anti-static wheel
    Anti325,
    /// 4 inch anti-static wheel
    Anti4,
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
        }
    }
}

/// Trait for tracking sensors that measure distance traveled.
///
/// Tracking sensors are used in odometry calculations to determine the robot's
/// position. They provide methods to get the offset from the robot's c
/// measure distance traveled, and reset the accumulated distance.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::omniwheel::{TrackingWheel, Tracking, OmniWheel};
/// use kernelvex::si::QLength;
/// # use kernelvex::sensors::Encoder;
/// # let encoder: impl Encoder = todo!();
///
/// let mut tracker: TrackingWheel<_> = TrackingWheel::new(
///     encoder,
///     OmniWheel::Omni275,
///     QLength::from_inches(5.0),
///     None,
/// );
///
/// let offset = tracker.offset();
/// let distance = tracker.distance();
/// ```
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
/// # Examples
///
/// ```no_run
/// use kernelvex::omniwheel::{TrackingWheel, OmniWheel};
/// use kernelvex::si::QLength;
/// # use kernelvex::sensors::Encoder;
/// # let encoder: impl Encoder = todo!();
///
/// // Create a tracking wheel 5 inches from the robot center
/// let mut wheel = TrackingWheel::new(
///     encoder,
///     OmniWheel::Omni275,
///     QLength::from_inches(5.0),
///     Some(1.5), // 1.5:1 gearing ratio
/// );
/// ```

pub struct TrackingWheel<T: Encoder> {
    encoder: T,
    wheel: OmniWheel,
    dist: QLength,
    orientation: Orientation,
    total: QLength,
    gearing: Option<f64>,
    reversed: bool,
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
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheel, OmniWheel};
    /// use kernelvex::si::QLength;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// // Right side tracking wheel, 5 inches from center, no gearing
    /// let wheel = TrackingWheel::new(
    ///     encoder,
    ///     OmniWheel::Omni275,
    ///     QLength::from_inches(5.0),
    ///     None,
    /// );
    ///
    /// // Left side tracking wheel with 2:1 gearing
    /// let left_wheel = TrackingWheel::new(
    ///     encoder,
    ///     OmniWheel::Omni325,
    ///     QLength::from_inches(-6.0),
    ///     Some(2.0),
    /// );
    /// ```
    #[allow(unused)]
    pub fn new(encoder: T, wheel: OmniWheel, dist: QLength, gearing: Option<f64>) -> Self {
        if dist.as_meters() > 0. {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Right,
                total: Default::default(),
                gearing,
                reversed: false,
            }
        } else {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Left,
                total: Default::default(),
                gearing,
                reversed: false,
            }
        }
    }

    /// Sets whether the encoder direction should be reversed.
    ///
    /// This is useful when the encoder is mounted in the opposite direction
    /// from what is expected. When reversed, positive encoder rotations will
    /// be interpreted as negative distance traveled.
    ///
    /// # Arguments
    ///
    /// * `reversed` - `true` to reverse the encoder direction, `false` for normal operation
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheel, OmniWheel};
    /// use kernelvex::si::QLength;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let mut wheel = TrackingWheel::new(
    ///     encoder,
    ///     OmniWheel::Omni275,
    ///     QLength::from_inches(5.0),
    ///     None,
    /// );
    /// 
    /// // If the encoder is reading backwards, reverse it
    /// wheel.set_reversed(true);
    /// ```
    #[allow(unused)]
    pub fn set_reversed(&mut self, reversed: bool) {
        self.reversed = reversed;
    }

    /// Returns whether the encoder direction is reversed.
    ///
    /// # Returns
    ///
    /// `true` if the encoder is reversed, `false` otherwise.
    #[allow(unused)]
    pub fn is_reversed(&self) -> bool {
        self.reversed
    }

    /// Returns the orientation of the tracking wheel.
    ///
    /// # Returns
    ///
    /// The orientation (`Left` or `Right`) based on the wheel's offset position.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheel, OmniWheel};
    /// use kernelvex::si::QLength;
    /// use kernelvex::utils::Orientation;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let wheel = TrackingWheel::new(
    ///     encoder,
    ///     OmniWheel::Omni275,
    ///     QLength::from_inches(5.0),
    ///     None,
    /// );
    /// 
    /// assert_eq!(wheel.orientation(), Orientation::Right);
    /// ```
    #[allow(unused)]
    pub fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// Returns the raw encoder angle reading.
    ///
    /// This method provides direct access to the encoder's current rotation
    /// without any conversion to linear distance. Useful for debugging and
    /// diagnostics.
    ///
    /// # Returns
    ///
    /// The encoder's current rotation as a [`QAngle`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheel, OmniWheel};
    /// use kernelvex::si::QLength;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let wheel = TrackingWheel::new(
    ///     encoder,
    ///     OmniWheel::Omni275,
    ///     QLength::from_inches(5.0),
    ///     None,
    /// );
    /// 
    /// let angle = wheel.encoder_angle();
    /// println!("Encoder at {} degrees", angle.as_degrees());
    /// ```
    #[allow(unused)]
    pub fn encoder_angle(&self) -> crate::si::QAngle {
        self.encoder.rotations()
    }

    /// Returns the wheel type used by this tracking wheel.
    ///
    /// # Returns
    ///
    /// The [`OmniWheel`] type.
    #[allow(unused)]
    pub fn wheel_type(&self) -> OmniWheel {
        self.wheel
    }

    /// Returns the gearing ratio of this tracking wheel.
    ///
    /// # Returns
    ///
    /// The gearing ratio, or 1.0 if no gearing was specified.
    #[allow(unused)]
    pub fn gearing(&self) -> f64 {
        self.gearing.unwrap_or(1.0)
    }
}

impl<T: Encoder> Tracking for TrackingWheel<T> {
    fn offset(&self) -> QLength {
        self.dist
    }

    fn distance(&mut self) -> QLength {
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let mut distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().as_radians())
                / core::f64::consts::TAU;

        if self.reversed {
            distance = -distance;
        }

        self.total = distance;

        distance
    }

    fn reset(&mut self) {
        self.total = Default::default();
    }

    fn delta(&mut self) -> QLength {
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let mut distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().as_radians())
                / core::f64::consts::TAU;

        if self.reversed {
            distance = -distance;
        }

        let ret = distance - self.total;

        self.total = distance;

        ret
    }
}

/// Builder for creating [`TrackingWheel`] instances with a fluent API.
///
/// The builder pattern provides a more ergonomic way to construct tracking wheels,
/// especially when you want to set optional parameters or when the construction
/// becomes more complex.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::omniwheel::{TrackingWheelBuilder, OmniWheel};
/// use kernelvex::si::QLength;
/// # use kernelvex::sensors::Encoder;
/// # let encoder: impl Encoder = todo!();
///
/// // Basic usage
/// let wheel = TrackingWheelBuilder::new(encoder)
///     .wheel_type(OmniWheel::Omni275)
///     .offset(QLength::from_inches(5.0))
///     .build();
///
/// // With all options
/// let advanced_wheel = TrackingWheelBuilder::new(encoder)
///     .wheel_type(OmniWheel::Omni325)
///     .offset(QLength::from_inches(-6.0))
///     .gearing(2.0)
///     .reversed(true)
///     .build();
/// ```
pub struct TrackingWheelBuilder<T: Encoder> {
    encoder: T,
    wheel: OmniWheel,
    dist: QLength,
    gearing: Option<f64>,
    reversed: bool,
}

impl<T: Encoder> TrackingWheelBuilder<T> {
    /// Creates a new builder with default values.
    ///
    /// # Arguments
    ///
    /// * `encoder` - The encoder to use for this tracking wheel
    ///
    /// # Defaults
    ///
    /// - Wheel type: `OmniWheel::Omni275`
    /// - Offset: 0.0 inches
    /// - Gearing: 1:1 (no gearing)
    /// - Reversed: false
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::TrackingWheelBuilder;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let builder = TrackingWheelBuilder::new(encoder);
    /// ```
    #[allow(unused)]
    pub fn new(encoder: T) -> Self {
        Self {
            encoder,
            wheel: OmniWheel::Omni275,
            dist: QLength::from_inches(0.0),
            gearing: None,
            reversed: false,
        }
    }

    /// Sets the wheel type.
    ///
    /// # Arguments
    ///
    /// * `wheel` - The type of omni wheel to use
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheelBuilder, OmniWheel};
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let builder = TrackingWheelBuilder::new(encoder)
    ///     .wheel_type(OmniWheel::Omni325);
    /// ```
    #[allow(unused)]
    pub fn wheel_type(mut self, wheel: OmniWheel) -> Self {
        self.wheel = wheel;
        self
    }

    /// Sets the offset distance from the robot's center.
    ///
    /// Positive values indicate the right side, negative values indicate the left side.
    ///
    /// # Arguments
    ///
    /// * `dist` - The perpendicular offset distance
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::TrackingWheelBuilder;
    /// use kernelvex::si::QLength;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let builder = TrackingWheelBuilder::new(encoder)
    ///     .offset(QLength::from_inches(5.5));
    /// ```
    #[allow(unused)]
    pub fn offset(mut self, dist: QLength) -> Self {
        self.dist = dist;
        self
    }

    /// Sets the gearing ratio.
    ///
    /// A value of 2.0 means the encoder rotates twice per wheel rotation.
    ///
    /// # Arguments
    ///
    /// * `ratio` - The gearing ratio
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::TrackingWheelBuilder;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let builder = TrackingWheelBuilder::new(encoder)
    ///     .gearing(2.0);
    /// ```
    #[allow(unused)]
    pub fn gearing(mut self, ratio: f64) -> Self {
        self.gearing = Some(ratio);
        self
    }

    /// Sets whether the encoder direction should be reversed.
    ///
    /// # Arguments
    ///
    /// * `reversed` - `true` to reverse the encoder direction
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::TrackingWheelBuilder;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let builder = TrackingWheelBuilder::new(encoder)
    ///     .reversed(true);
    /// ```
    #[allow(unused)]
    pub fn reversed(mut self, reversed: bool) -> Self {
        self.reversed = reversed;
        self
    }

    /// Builds the [`TrackingWheel`].
    ///
    /// # Returns
    ///
    /// A new `TrackingWheel` instance with the configured parameters.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::omniwheel::{TrackingWheelBuilder, OmniWheel};
    /// use kernelvex::si::QLength;
    /// # use kernelvex::sensors::Encoder;
    /// # let encoder: impl Encoder = todo!();
    ///
    /// let wheel = TrackingWheelBuilder::new(encoder)
    ///     .wheel_type(OmniWheel::Omni275)
    ///     .offset(QLength::from_inches(5.0))
    ///     .build();
    /// ```
    #[allow(unused)]
    pub fn build(self) -> TrackingWheel<T> {
        let orientation = if self.dist.as_meters() > 0.0 {
            Orientation::Right
        } else {
            Orientation::Left
        };

        TrackingWheel {
            encoder: self.encoder,
            wheel: self.wheel,
            dist: self.dist,
            orientation,
            total: Default::default(),
            gearing: self.gearing,
            reversed: self.reversed,
        }
    }
}
