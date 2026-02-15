//! Sensor abstraction layer for encoders and rotation sensors.
//!
//! Provides odom unified [`Encoder`] trait that allows different sensor types
//! to be used interchangeably for odometry calculations.

use vexide_devices::adi::encoder::AdiEncoder;
use vexide_devices::smart::rotation::RotationSensor;

/// Trait for encoders that can measure rotation.
///
/// Implementations provide the total accumulated rotation since the last reset
/// or initialization. This is used by tracking wheels to calculate distance traveled.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::odom::sensors::Encoder;
/// # use kernelvex::util::si::QAngle;
/// # let encoder: vexide_devices::smart::rotation::RotationSensor = todo!();
///
/// let rotations = encoder.rotations();
/// println!("Total rotations: {} turns", rotations.as_turns());
/// ```

pub trait Encoder {}

impl Encoder for AdiEncoder<360> {}

impl Encoder for RotationSensor {}
