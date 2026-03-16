//! RAMSETE controller for trajectory tracking.
//!
//! This module implements the RAMSETE (Rapidly-exploring Adaptive Motion
//! Controller for Smooth Trajectory Execution) controller, a nonlinear
//! feedback controller for tracking trajectories with differential drivetrains.
//!
//! # Overview
//!
//! RAMSETE computes velocity commands that drive the robot toward a reference
//! trajectory while correcting for position and heading errors. It is more
//! robust than pure pursuit for trajectories with significant curvature changes.
//!
//! # Algorithm
//!
//! The controller computes errors in the robot frame:
//! - `e_x`: Forward error (distance ahead/behind reference)
//! - `e_y`: Lateral error (distance left/right of reference)
//! - `e_theta`: Heading error (angle difference from reference)
//!
//! The output velocities are computed using the RAMSETE equations:
//! ```text
//! k = 2 * zeta * sqrt(w_d^2 + b * v_d^2)
//! v = v_d * cos(e_theta) + k * e_x
//! w = w_d + k * e_theta + b * v_d * sinc(e_theta) * e_y
//! ```
//!
//! # Tuning Parameters
//!
//! - `b`: Convergence gain (higher = more aggressive correction, typical: 2.0)
//! - `zeta`: Damping ratio (typical: 0.7, like a second-order system)
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{RamseteController, RamseteReference, Pose};
//!
//! let ramsete = RamseteController::new().set(2.0, 0.7);
//!
//! let reference = RamseteReference::new(target_pose, 1.0, 0.5);
//! let (linear_vel, angular_vel) = ramsete.calculate(current_pose, reference);
//! ```

use crate::motion::trajectory::TrajectoryPoint;
use crate::odom::pose::Pose;
use crate::util::si::QAngle;

/// RAMSETE controller configuration and calculation.
///
/// The RAMSETE controller is a nonlinear trajectory tracker that computes
/// velocity commands to follow a reference trajectory while correcting for
/// position and heading errors.
///
/// # Parameters
///
/// - `b`: Convergence gain that determines how aggressively the controller
///   corrects lateral and heading errors. Higher values = faster convergence
///   but potentially more oscillation. Typical value: 2.0
/// - `zeta`: Damping ratio similar to a second-order system. Higher values
///   reduce oscillation. Typical value: 0.7
/// - `epsilon`: Small-angle threshold for sinc function to avoid division
///   by zero. Default: 1e-6
#[derive(Debug, Clone, Copy)]
pub struct RamseteController {
    /// Convergence gain (typical: 2.0).
    b: f64,
    /// Damping ratio (typical: 0.7).
    zeta: f64,
    /// Small-angle epsilon for sinc calculation.
    epsilon: f64,
}

/// A trajectory reference point for RAMSETE tracking.
///
/// Contains the desired pose and velocities at a point along the trajectory.
/// This is what the RAMSETE controller tries to track.
#[derive(Debug, Clone, Copy)]
pub struct RamseteReference {
    /// Desired pose at this trajectory point.
    pub pose: Pose,
    /// Desired linear velocity in meters per second.
    pub linear_velocity: f64,
    /// Desired angular velocity in radians per second.
    pub angular_velocity: f64,
}

impl RamseteReference {
    /// Creates a new RAMSETE reference point.
    ///
    /// # Arguments
    ///
    /// * `pose` - Desired pose (position and heading)
    /// * `linear_velocity` - Desired forward velocity in m/s
    /// * `angular_velocity` - Desired rotational velocity in rad/s
    #[inline]
    pub fn new(pose: Pose, linear_velocity: f64, angular_velocity: f64) -> Self {
        Self {
            pose,
            linear_velocity,
            angular_velocity,
        }
    }
}

impl From<TrajectoryPoint> for RamseteReference {
    /// Converts a trajectory point to a RAMSETE reference.
    fn from(point: TrajectoryPoint) -> Self {
        Self {
            pose: point.pose,
            linear_velocity: point.linear_velocity,
            angular_velocity: point.angular_velocity,
        }
    }
}

impl RamseteController {
    /// Creates a RAMSETE controller with zero gains.
    ///
    /// Use [`set`](Self::set) to configure the tuning parameters.
    ///
    /// # Example
    ///
    /// ```no_run
    /// let ramsete = RamseteController::new().set(2.0, 0.7);
    /// ```
    #[inline]
    pub fn new() -> Self {
        Self {
            b: 0.,
            zeta: 0.,
            epsilon: 1e-6,
        }
    }

    /// Sets the RAMSETE tuning parameters.
    ///
    /// # Arguments
    ///
    /// * `b` - Convergence gain (typical: 2.0)
    /// * `zeta` - Damping ratio (typical: 0.7)
    ///
    /// # Returns
    ///
    /// A new controller with the specified gains.
    pub fn set(self, b: f64, zeta: f64) -> Self {
        Self {
            b,
            zeta,
            epsilon: 1e-6,
        }
    }

    /// Sets the small-angle epsilon for the sinc function.
    ///
    /// When the heading error is smaller than epsilon, sinc(theta) returns 1.0
    /// to avoid numerical instability from dividing by near-zero values.
    ///
    /// # Arguments
    ///
    /// * `epsilon` - Small-angle threshold (default: 1e-6)
    #[inline]
    pub fn with_epsilon(mut self, epsilon: f64) -> Self {
        self.epsilon = epsilon;
        self
    }

    /// Computes the linear and angular velocity commands.
    ///
    /// # Arguments
    ///
    /// * `current` - The robot's current pose
    /// * `reference` - The desired trajectory reference point
    ///
    /// # Returns
    ///
    /// A tuple `(v, w)` where:
    /// - `v` is the commanded linear velocity in m/s
    /// - `w` is the commanded angular velocity in rad/s
    ///
    /// # Algorithm
    ///
    /// 1. Computes errors in the robot frame (e_x, e_y, e_theta)
    /// 2. Calculates adaptive gain k based on reference velocities
    /// 3. Applies RAMSETE equations to compute corrected velocities
    pub fn calculate(&self, current: Pose, reference: RamseteReference) -> (f64, f64) {
        let coords = current.position();
        let refer = reference.pose.position();

        let dx = refer.x - coords.x;
        let dy = refer.y - coords.y;

        let heading = current.heading();
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        let e_x = cos_h * dx + sin_h * dy;
        let e_y = -sin_h * dx + cos_h * dy;

        let e_theta = (reference.pose.heading() - heading).remainder(QAngle::TAU);

        let v_d = reference.linear_velocity;
        let w_d = reference.angular_velocity;

        let k = 2.0 * self.zeta * libm::sqrt(w_d * w_d + self.b * v_d * v_d);
        let sinc = sinc(e_theta.as_radians(), self.epsilon);

        let v = v_d * e_theta.cos() + k * e_x;
        let w = w_d + k * e_theta.as_radians() + self.b * v_d * sinc * e_y;

        (v, w)
    }
}

/// Computes sinc(theta) = sin(theta)/theta with small-angle handling.
///
/// For |theta| < epsilon, returns 1.0 to avoid division by near-zero.
fn sinc(theta: f64, epsilon: f64) -> f64 {
    if libm::fabs(theta) < epsilon {
        1.0
    } else {
        libm::sin(theta) / theta
    }
}
