//! Differential (tank-style) drivetrain implementation.
//!
//! This module provides [`DifferentialDrive`], a controller for robots with independent
//! left and right motor groups. It supports multiple drive modes (tank, arcade, curvature)
//! and provides velocity estimation from motor encoders (IME) for odometry fallback.
//!
//! # Overview
//!
//! A differential drivetrain has motors on the left and right sides that can be driven
//! independently. By varying the speeds of each side, the robot can move forward, turn,
//! or spin in place.
//!
//! # Drive Modes
//!
//! - **Tank Drive**: Direct left/right control (`drive_tank`)
//! - **Arcade Drive**: Forward/turn control with expo scaling (`drive_arcade`)
//! - **Curvature Drive**: Throttle + curvature for smooth driving (`drive_curvature`)
//!
//! # IME Velocity Estimation
//!
//! When no tracking rig is available, `DifferentialDrive` can estimate linear and angular
//! velocity from motor encoder readings using the wheel size, gear ratio, and track width:
//!
//! - Linear velocity: `(left_vel + right_vel) / 2`
//! - Angular velocity: `(right_vel - left_vel) / track_width`
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{DifferentialDrive, MotorGroup, OmniWheel, QLength};
//! use kernelvex::dt::differential::ExpoDrive;
//!
//! let left = MotorGroup::new([left_motor1, left_motor2]);
//! let right = MotorGroup::new([right_motor1, right_motor2]);
//! let expo = ExpoDrive::new(2.0, 1.0);
//!
//! let drivetrain = DifferentialDrive::new(
//!     left,
//!     right,
//!     expo,
//!     OmniWheel::Omni4,
//!     QLength::from_inches(12.0),  // track width
//!     0.6,  // gear ratio
//! );
//!
//! // Tank drive: left at 50%, right at 50%
//! drivetrain.drive_tank(0.5, 0.5).await?;
//!
//! // Curvature drive: 80% throttle, slight right turn
//! drivetrain.drive_curvature(0.8, 0.2).await?;
//! ```

use crate::dt::model::{Arcade, CurvatureDrive, Drivetrain, Tank};
use crate::util::utils::GroupErrors;
use crate::{MotorGroup, OmniWheel, QLength, Vector2};

/// A differential (tank-style) drivetrain with left and right motor groups.
///
/// `DifferentialDrive` controls a robot with independent left and right sides,
/// supporting tank, arcade, and curvature drive modes. It also provides velocity
/// estimation from motor encoders (IME) for use when no tracking rig is available.
///
/// # Fields
///
/// - `left` / `right`: Motor groups for each side
/// - `wheel`: The wheel type used (for velocity calculations)
/// - `width`: Track width (distance between wheel centers)
/// - `ratio`: Gear ratio (motor rotations per wheel rotation)
/// - `expo`: Exponential drive scaling for smoother control
///
/// # Velocity Estimation
///
/// The drivetrain can estimate linear and angular velocity from motor RPM:
///
/// ```text
/// wheel_vel = motor_rpm * ratio * (2*PI / 60) * wheel_diameter / 2
/// linear_vel = (left_wheel_vel + right_wheel_vel) / 2
/// angular_vel = (right_wheel_vel - left_wheel_vel) / track_width
/// ```
pub struct DifferentialDrive {
    /// Left motor group.
    left: MotorGroup,
    /// Right motor group.
    right: MotorGroup,
    /// Wheel type for size calculations.
    wheel: OmniWheel,
    /// Track width (distance between wheel centers).
    pub(crate) width: QLength,
    /// Gear ratio (motor rotations per wheel rotation).
    ratio: f64,
    /// Exponential drive scaling configuration.
    expo: ExpoDrive
}

/// Exponential drive scaling for smoother joystick control.
///
/// `ExpoDrive` applies an exponential curve to joystick inputs, providing
/// fine control at low speeds while maintaining full power at high inputs.
/// This makes the robot easier to drive precisely.
///
/// # Formula
///
/// The scaling uses the formula:
/// ```text
/// m = (|x|^(n+2) + |y|^(n+2))^(1/(n+2))
/// f = m^k / sqrt(x^2 + y^2)
/// output = (f*x, f*y)
/// ```
///
/// # Parameters
///
/// - `n`: Controls the curve shape (higher = more aggressive near center)
/// - `k`: Controls the overall scaling factor
///
/// # Example
///
/// ```no_run
/// let expo = ExpoDrive::new(2.0, 1.0);
/// let (x, y) = expo.calculate(0.5, 0.5).as_tuple();
/// ```
pub struct ExpoDrive {
    /// Curve shape parameter (higher = more aggressive near center).
    n: f64,
    /// Overall scaling factor.
    k: f64,
    /// Epsilon value.
    eps: f64,
}

// https://www.desmos.com/calculator/tks2ea2u77
impl ExpoDrive {
    /// Creates a new exponential drive configuration.
    ///
    /// # Arguments
    ///
    /// * `n` - Curve shape parameter (typically 1.0 to 3.0)
    /// * `k` - Scaling factor (typically 1.0)
    pub fn new(n: f64, k: f64, eps: Option<f64>) -> Self {
        Self { n, k, eps: eps.unwrap_or(0.0) }
    }

    /// Applies exponential scaling to joystick inputs.
    ///
    /// # Arguments
    ///
    /// * `x` - X-axis input in range [-1, 1]
    /// * `y` - Y-axis input in range [-1, 1]
    ///
    /// # Returns
    ///
    /// Scaled output as a [`Vector2<f64>`]
    ///
    /// # Panics
    ///
    /// Panics if inputs are outside [-1, 1] range.
    pub fn calculate(&self, x: f64, y: f64) -> Vector2<f64> {
        {
            assert!(-1.0 <= x && x <= 1.0, "x must be between [-1, 1]");
        }

        {
            assert!(-1.0 <= y && y <= 1.0, "y must be between [-1, 1]");
        }

        if x == 0. && y == 0. {
            return Vector2::<f64>::new(0.0, 0.0);
        }

        let m = libm::pow(libm::pow(libm::fabs(x), self.n+2.) + libm::pow(libm::fabs(y), self.n+2.), 1./(self.n+2.));
        let f = libm::pow(m, self.k)/libm::sqrt(x*x + y*y);

        let mut fx = f*x;
        let mut fy = f*y;

        if fx <= self.eps {
            fx = 0.;
        }

        if fy <= self.eps {
            fy = 0.;
        }

        Vector2::<f64>::new(fx, fy)
    }
}

impl DifferentialDrive {
    /// Creates a new differential drivetrain.
    ///
    /// # Arguments
    ///
    /// * `left` - Motor group for the left side
    /// * `right` - Motor group for the right side
    /// * `expo` - Exponential drive configuration for input scaling
    /// * `wheel` - Wheel type used (for velocity calculations)
    /// * `width` - Track width (distance between left and right wheel centers)
    /// * `ratio` - Gear ratio (motor rotations per wheel rotation, e.g., 0.6 for 36:60)
    ///
    /// # Example
    ///
    /// ```no_run
    /// let drivetrain = DifferentialDrive::new(
    ///     left_motors,
    ///     right_motors,
    ///     ExpoDrive::new(2.0, 1.0),
    ///     OmniWheel::Omni4,
    ///     QLength::from_inches(12.0),
    ///     0.6,
    /// );
    /// ```
    #[inline]
    pub fn new(left: MotorGroup, right: MotorGroup, expo: ExpoDrive, wheel: OmniWheel, width: QLength, ratio: f64) -> Self {
        Self { left, right, expo, wheel, width, ratio }
    }

    /// Returns the track width of the drivetrain.
    pub fn width(&self) -> QLength {
        self.width
    }

    /// Returns a reference to the wheel type.
    pub fn wheel(&self) -> &OmniWheel {
        &self.wheel
    }
}

impl Arcade for DifferentialDrive {
    /// Drives using arcade control with exponential scaling.
    ///
    /// Arcade drive combines forward/backward and left/right inputs into
    /// differential wheel speeds. Inputs are scaled using the configured
    /// [`ExpoDrive`] for smoother control.
    ///
    /// # Arguments
    ///
    /// * `left` - Forward/backward input (-1.0 to 1.0)
    /// * `right` - Turn input (-1.0 to 1.0)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Command sent successfully
    /// * `Err(GroupErrors)` - Motor communication error
    async fn drive_arcade(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        let maximum = libm::fmax(libm::fabs(left), libm::fabs(right));
        let (x, y) = self.expo.calculate(left, right).as_tuple();
        let total = x + y;
        let difference = x - y;

        if x >= 0. {
            if y >= 0. {
                self.left.set_voltage(maximum).await?;
                self.right.set_voltage(difference).await?;
            } else {
                self.left.set_voltage(total).await?;
                self.right.set_voltage(maximum).await?;
            }
        } else if y >= 0. {
            self.left.set_voltage(total).await?;
            self.right.set_voltage(-maximum).await?;
        } else {
            self.left.set_voltage(-maximum).await?;
            self.right.set_voltage(difference).await?;
        }
        Ok(())
    }
}

impl Tank for DifferentialDrive {
    /// Drives using tank control with exponential scaling.
    ///
    /// Tank drive controls each side independently. The inputs are scaled
    /// using the configured [`ExpoDrive`] and converted to voltage (0-12V).
    ///
    /// # Arguments
    ///
    /// * `left` - Left side power (-1.0 to 1.0)
    /// * `right` - Right side power (-1.0 to 1.0)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Command sent successfully
    /// * `Err(GroupErrors)` - Motor communication error
    async fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        let (x, y) = self.expo.calculate(left, right).as_tuple();
        self.left.set_voltage(x * 12.0).await?;
        self.right.set_voltage(y * 12.0).await?;

        Ok(())
    }
}

impl CurvatureDrive for DifferentialDrive {
    /// Drives using curvature control.
    ///
    /// Curvature drive decouples throttle from turn rate, allowing sharp turns
    /// even at low speeds. Unlike arcade drive where turn rate scales with
    /// throttle, curvature drive maintains consistent turning regardless of speed.
    ///
    /// # Arguments
    ///
    /// * `throttle` - Forward/backward speed (-1.0 to 1.0)
    /// * `curvature` - Turn rate (-1.0 to 1.0), where 0 = straight
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Command sent successfully
    /// * `Err(GroupErrors)` - Motor communication error
    ///
    /// # Formula
    ///
    /// ```text
    /// left_speed = throttle + curvature
    /// right_speed = throttle - curvature
    /// ```
    ///
    /// Outputs are normalized if they exceed [-1, 1].
    async fn drive_curvature(&mut self, throttle: f64, curvature: f64) -> Result<(), GroupErrors> {
        let left_speed = throttle + curvature;
        let right_speed = throttle - curvature;

        let max_magnitude = libm::fmax(libm::fabs(left_speed), libm::fabs(right_speed));
        let (left_normalized, right_normalized) = if max_magnitude > 1.0 {
            (left_speed / max_magnitude, right_speed / max_magnitude)
        } else {
            (left_speed, right_speed)
        };

        self.left.set_voltage(left_normalized * 12.0).await?;
        self.right.set_voltage(right_normalized * 12.0).await?;

        Ok(())
    }
}

impl Drivetrain for DifferentialDrive {
    /// Estimates linear velocity from motor encoders (IME fallback).
    ///
    /// Computes the average wheel velocity from left and right motor RPM,
    /// converting to meters per second using the gear ratio and wheel size.
    ///
    /// # Formula
    ///
    /// ```text
    /// wheel_vel = motor_rpm * ratio * (2*PI / 60) * wheel_diameter
    /// linear_vel = (left_wheel_vel + right_wheel_vel) / 2
    /// ```
    ///
    /// # Returns
    ///
    /// * `Ok(f64)` - Linear velocity in meters per second
    /// * `Err(GroupErrors)` - Motor encoder read error
    async fn linear_velocity(&self) -> Result<f64, GroupErrors> {
        let left = self.left.velocity().await? as f64;
        let right = self.right.velocity().await? as f64;
        let avg = (left+right)/2.;

        let vel = avg * self.ratio * (std::f64::consts::TAU / 60.) * self.wheel.size().as_meters();

        Ok(vel)


    }

    /// Estimates angular velocity from motor encoders (IME fallback).
    ///
    /// Computes the rotational velocity from the difference between left and
    /// right wheel velocities, divided by the track width.
    ///
    /// # Formula
    ///
    /// ```text
    /// angular_vel = (right_wheel_vel - left_wheel_vel) / track_width
    /// ```
    ///
    /// # Returns
    ///
    /// * `Ok(f64)` - Angular velocity in radians per second
    /// * `Err(GroupErrors)` - Motor encoder read error
    async fn angular_velocity(&self) -> Result<f64, GroupErrors> {
        let left_rpm = self.left.velocity().await? as f64;
        let right_rpm = self.right.velocity().await? as f64;

        let vel = |rpm: f64| -> f64 {
            let wheel_rpm = rpm * self.ratio;
            let angular_vel = wheel_rpm * (2.0 * core::f64::consts::PI / 60.0);
            angular_vel * self.wheel.size().as_meters()
        };

        let left = vel(left_rpm);
        let right = vel(right_rpm);

        let angular_vel = (right - left) / self.width.as_meters();

        Ok(angular_vel)
    }
}

