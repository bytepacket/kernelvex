//! Tracking wheel implementation for odometry.
//!
//! This module provides support for tracking wheels used in odometry calculations,
//! allowing robots to track their position and movement using wheel encoders.
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::omniwheel::{OmniWheel, TrackingWheel, Tracking};
//! use kernelvex::si::QLength;
//! # use kernelvex::sensors::Encoder;
//! # let encoder: impl Encoder = todo!();
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
/// position. They provide methods to get the offset from the robot's center,
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
    ///              Must be positive if provided.
    ///
    /// # Returns
    ///
    /// A new `TrackingWheel` instance with the encoder reset to zero.
    ///
    /// # Panics
    ///
    /// Panics if `gearing` is provided and is less than or equal to zero.
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
        // Validate gearing ratio
        if let Some(g) = gearing {
            assert!(g > 0.0, "Gearing ratio must be positive, got {}", g);
        }

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
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().as_radians())
                / core::f64::consts::TAU;

        distance
    }

    fn reset(&mut self) {
        self.total = Default::default();
    }

    fn delta(&mut self) -> QLength {
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().as_radians())
                / core::f64::consts::TAU;

        let ret = distance - self.total;

        self.total = distance;

        ret
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::si::QAngle;

    /// Mock encoder for testing
    struct MockEncoder {
        rotation: QAngle,
    }

    impl MockEncoder {
        fn new(rotation: QAngle) -> Self {
            Self { rotation }
        }

        fn set_rotation(&mut self, rotation: QAngle) {
            self.rotation = rotation;
        }
    }

    impl Encoder for MockEncoder {
        fn rotations(&self) -> QAngle {
            self.rotation
        }
    }

    #[test]
    fn test_omniwheel_sizes() {
        assert_eq!(OmniWheel::Omni275.size(), QLength::from_inches(2.75));
        assert_eq!(OmniWheel::Omni325.size(), QLength::from_inches(3.25));
        assert_eq!(OmniWheel::Omni4.size(), QLength::from_inches(4.125));
        assert_eq!(OmniWheel::Anti275.size(), QLength::from_inches(2.75));
        assert_eq!(OmniWheel::Anti325.size(), QLength::from_inches(3.25));
        assert_eq!(OmniWheel::Anti4.size(), QLength::from_inches(4.0));
    }

    #[test]
    fn test_tracking_wheel_creation_right() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            None,
        );

        assert_eq!(wheel.offset(), QLength::from_inches(5.0));
        assert_eq!(wheel.orientation, Orientation::Right);
    }

    #[test]
    fn test_tracking_wheel_creation_left() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(-5.0),
            None,
        );

        assert_eq!(wheel.offset(), QLength::from_inches(-5.0));
        assert_eq!(wheel.orientation, Orientation::Left);
    }

    #[test]
    #[should_panic(expected = "Gearing ratio must be positive")]
    fn test_negative_gearing_panics() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let _wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            Some(-1.0),
        );
    }

    #[test]
    #[should_panic(expected = "Gearing ratio must be positive")]
    fn test_zero_gearing_panics() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let _wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            Some(0.0),
        );
    }

    #[test]
    fn test_distance_calculation_no_gearing() {
        let encoder = MockEncoder::new(QAngle::from_radians(core::f64::consts::TAU)); // 1 full rotation
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            None,
        );

        let distance = wheel.distance();
        let circumference = OmniWheel::Omni275.size() * core::f64::consts::PI;
        
        // After 1 rotation, distance should equal circumference
        assert!((distance.as_inches() - circumference.as_inches()).abs() < 0.001);
    }

    #[test]
    fn test_distance_calculation_with_gearing() {
        let encoder = MockEncoder::new(QAngle::from_radians(core::f64::consts::TAU)); // 1 encoder rotation
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            Some(2.0), // 2:1 gearing means encoder rotates twice per wheel rotation
        );

        let distance = wheel.distance();
        let circumference = OmniWheel::Omni275.size() * core::f64::consts::PI;
        
        // With 2:1 gearing, 1 encoder rotation = 0.5 wheel rotations
        let expected = circumference * 2.0;
        assert!((distance.as_inches() - expected.as_inches()).abs() < 0.001);
    }

    #[test]
    fn test_delta_calculation() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            None,
        );

        // First delta should be 0
        let delta1 = wheel.delta();
        assert!((delta1.as_inches()).abs() < 0.001);

        // Rotate the encoder by 1 radian
        wheel.encoder.set_rotation(QAngle::from_radians(1.0));
        let delta2 = wheel.delta();
        
        // Delta should reflect the change
        let circumference = OmniWheel::Omni275.size() * core::f64::consts::PI;
        let expected = circumference * 1.0 / core::f64::consts::TAU;
        assert!((delta2.as_inches() - expected.as_inches()).abs() < 0.001);

        // Another rotation
        wheel.encoder.set_rotation(QAngle::from_radians(2.0));
        let delta3 = wheel.delta();
        
        // Delta should be the same (another 1 radian)
        assert!((delta3.as_inches() - expected.as_inches()).abs() < 0.001);
    }

    #[test]
    fn test_reset() {
        let encoder = MockEncoder::new(QAngle::from_radians(core::f64::consts::TAU));
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            None,
        );

        // Get initial distance
        let _ = wheel.distance();
        
        // Reset should clear total
        wheel.reset();
        assert_eq!(wheel.total.as_inches(), 0.0);
    }

    #[test]
    fn test_distance_does_not_accumulate() {
        let encoder = MockEncoder::new(QAngle::from_radians(core::f64::consts::TAU));
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            None,
        );

        let distance1 = wheel.distance();
        let distance2 = wheel.distance();
        
        // Calling distance() multiple times should return the same value
        // (it should not accumulate on each call)
        assert!((distance1.as_inches() - distance2.as_inches()).abs() < 0.001);
    }

    #[test]
    fn test_delta_with_gearing() {
        let encoder = MockEncoder::new(QAngle::from_radians(0.0));
        let mut wheel = TrackingWheel::new(
            encoder,
            OmniWheel::Omni275,
            QLength::from_inches(5.0),
            Some(2.0),
        );

        // First delta
        let _ = wheel.delta();

        // Rotate encoder by π radians (half rotation)
        wheel.encoder.set_rotation(QAngle::from_radians(core::f64::consts::PI));
        let delta = wheel.delta();

        let circumference = OmniWheel::Omni275.size() * core::f64::consts::PI;
        // With 2:1 gearing and π radians rotation, we expect 1 full wheel rotation
        let expected = circumference * 2.0 * core::f64::consts::PI / core::f64::consts::TAU;
        assert!((delta.as_inches() - expected.as_inches()).abs() < 0.001);
    }
}
