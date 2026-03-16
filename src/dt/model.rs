//! Drivetrain control traits for different drive modes.
//!
//! This module defines the traits that drivetrains can implement to support
//! different control modes: tank, arcade, and curvature drive. It also defines
//! the [`Drivetrain`] trait for velocity estimation.
//!
//! # Drive Modes
//!
//! ## Tank Drive
//! Direct control of left and right sides independently. Each joystick controls
//! one side of the robot.
//!
//! ## Arcade Drive
//! Combined forward/turn control. One stick controls forward/backward motion,
//! another controls turning. Turn rate typically scales with forward speed.
//!
//! ## Curvature Drive
//! Throttle and curvature control. Unlike arcade, the turn rate is independent
//! of throttle, allowing sharp turns at low speeds. Ideal for precise maneuvering.
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{Tank, Arcade, CurvatureDrive};
//!
//! // Tank drive: direct left/right control
//! drivetrain.drive_tank(left_stick, right_stick).await?;
//!
//! // Arcade drive: forward/turn control
//! drivetrain.drive_arcade(forward_stick, turn_stick).await?;
//!
//! // Curvature drive: throttle/curvature control
//! drivetrain.drive_curvature(throttle, curvature).await?;
//! ```

use crate::util::utils::GroupErrors;

/// Tank drive control trait.
///
/// Tank drive provides direct control of left and right sides independently.
/// Each side receives a power value from -1.0 to 1.0.
///
/// # Example
///
/// ```no_run
/// // Drive forward at 50% power
/// drivetrain.drive_tank(0.5, 0.5).await?;
///
/// // Turn left (right side faster)
/// drivetrain.drive_tank(0.3, 0.7).await?;
///
/// // Spin in place (opposite directions)
/// drivetrain.drive_tank(-0.5, 0.5).await?;
/// ```
pub trait Tank {
    /// Drives using tank control.
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
    fn drive_tank(
        &mut self,
        left: f64,
        right: f64,
    ) -> impl Future<Output = Result<(), GroupErrors>> + Send;
}

/// Arcade drive control trait.
///
/// Arcade drive combines forward/backward and turn inputs into wheel speeds.
/// This is often more intuitive for new drivers than tank drive.
///
/// # Example
///
/// ```no_run
/// // Drive forward at 50%
/// drivetrain.drive_arcade(0.5, 0.0).await?;
///
/// // Turn right while moving forward
/// drivetrain.drive_arcade(0.5, 0.3).await?;
/// ```
pub trait Arcade {
    /// Drives using arcade control.
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
    fn drive_arcade(
        &mut self,
        left: f64,
        right: f64,
    ) -> impl Future<Output = Result<(), GroupErrors>> + Send;
}

/// Curvature drive control trait.
///
/// Curvature drive decouples throttle from turn rate, allowing precise control
/// at all speeds. Unlike arcade drive where turn rate scales with throttle,
/// curvature maintains consistent turning regardless of speed.
///
/// This is particularly useful for:
/// - Precise alignment at low speeds
/// - Sharp turns without needing high throttle
/// - Smooth high-speed curves
///
/// # Example
///
/// ```no_run
/// // Drive forward at 80% with slight right turn
/// drivetrain.drive_curvature(0.8, 0.2).await?;
///
/// // Sharp turn at low speed (impossible with arcade)
/// drivetrain.drive_curvature(0.2, 0.8).await?;
/// ```
pub trait CurvatureDrive {
    /// Drives using curvature control.
    ///
    /// # Arguments
    ///
    /// * `throttle` - Forward/backward speed (-1.0 to 1.0)
    /// * `curvature` - Turn rate (-1.0 to 1.0), where 0 = straight
    ///
    /// Unlike arcade drive, turning is not scaled by throttle, allowing
    /// sharp turns even at low speeds.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Command sent successfully
    /// * `Err(GroupErrors)` - Motor communication error
    fn drive_curvature(
        &mut self,
        throttle: f64,
        curvature: f64,
    ) -> impl Future<Output = Result<(), GroupErrors>> + Send;
}

/// Velocity estimation trait for drivetrains.
///
/// Provides methods to estimate linear and angular velocity from motor encoders
/// (Integrated Motor Encoders / IME). This is used as a fallback when no
/// external tracking system is available.
///
/// # Velocity Formulas
///
/// For a differential drivetrain:
/// - Linear velocity: `(left_vel + right_vel) / 2`
/// - Angular velocity: `(right_vel - left_vel) / track_width`
pub trait Drivetrain {
    /// Returns the estimated linear velocity in meters per second.
    ///
    /// This is the forward/backward velocity of the robot's center.
    fn linear_velocity(&self) -> impl Future<Output = Result<f64, GroupErrors>>;

    /// Returns the estimated angular velocity in radians per second.
    ///
    /// Positive values indicate counter-clockwise rotation.
    fn angular_velocity(&self) -> impl Future<Output = Result<f64, GroupErrors>>;
}
