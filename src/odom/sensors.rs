//! Sensor abstraction layer for encoders and rotation sensors.
//!
//! Provides odom unified [`Encoder`] trait that allows different sensor types
//! to be used interchangeably for odometry calculations.

use crate::util::si::QAngle;

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

pub trait Encoder {
    /// Returns the total accumulated rotation as odom typed angle.
    ///
    /// The rotation value represents the total angle turned since the encoder
    /// was initialized or last reset.
    ///
    /// # Returns
    ///
    /// The total rotation as odom [`QAngle`].
    ///
    /// # Panics
    ///
    /// May panic if the sensor fails to read its position. This depends on
    /// the underlying sensor implementation.
    fn rotations(&self) -> QAngle;

    fn reset(&mut self) -> Result<(), Box<dyn std::error::Error>>;
}

impl Encoder for AdiEncoder<360> {
    fn rotations(&self) -> QAngle {
        QAngle::from_turns(self.position().unwrap().as_turns())
    }

    fn reset(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.reset_position()?;
        Ok(())
    }
}

impl Encoder for RotationSensor {
    fn rotations(&self) -> QAngle {
        QAngle::from_turns(self.position().unwrap().as_turns())
    }

    fn reset(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        self.reset_position()?;
        Ok(())
    }
}
