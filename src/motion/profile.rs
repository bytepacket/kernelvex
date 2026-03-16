//! Trapezoidal motion profile generation.
//!
//! This module provides tools for generating smooth motion profiles that respect
//! velocity and acceleration constraints. A trapezoidal profile is the standard
//! approach for point-to-point motion in robotics.
//!
//! # Overview
//!
//! A trapezoidal motion profile has three phases:
//! 1. **Acceleration**: Ramp up from zero to max velocity
//! 2. **Cruise**: Maintain max velocity (may be skipped for short distances)
//! 3. **Deceleration**: Ramp down from max velocity to zero
//!
//! ```text
//! Velocity
//!    ^
//! max|    ___________
//!    |   /           \
//!    |  /             \
//!    | /               \
//!    |/                 \
//!    +--------------------> Time
//!      accel cruise decel
//! ```
//!
//! For short distances where max velocity can't be reached, the profile becomes
//! triangular (no cruise phase).
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{TrapezoidalConstraints, QLength};
//!
//! let constraints = TrapezoidalConstraints::new()
//!     .set_gains(1.0, 2.0);  // 1 m/s max vel, 2 m/s^2 max accel
//!
//! let profile = constraints.generate_profile(QLength::from_meters(2.0));
//!
//! for state in profile {
//!     println!("t={:.2}s, pos={:.2}m, vel={:.2}m/s",
//!         state.time.as_sec(),
//!         state.position.as_meters(),
//!         state.velocity);
//! }
//! ```

use crate::util::si::{QLength, QTime};
use libm::sqrt;

/// A single time-indexed state during motion profile execution.
///
/// Represents the desired position, velocity, and acceleration at a specific
/// time point. Used by motion controllers to track the profile.
///
/// # Units
///
/// - `time`: Seconds from profile start
/// - `position`: Meters from start position
/// - `velocity`: Meters per second
/// - `acceleration`: Meters per second squared
#[derive(Debug, Clone, Copy)]
pub struct MotionState {
    /// Time from profile start.
    pub time: QTime,
    /// Position (distance) from start.
    pub position: QLength,
    /// Velocity at this time point (m/s).
    pub velocity: f64,
    /// Acceleration at this time point (m/s^2).
    pub acceleration: f64,
}

/// Constraints for trapezoidal motion profile generation.
///
/// Defines the maximum velocity and acceleration that the motion profile
/// will respect. These should be set based on your robot's physical capabilities.
///
/// # Example
///
/// ```no_run
/// let constraints = TrapezoidalConstraints::new()
///     .set_gains(1.5, 3.0);  // 1.5 m/s max, 3 m/s^2 accel
/// ```
#[derive(Debug, Clone, Copy)]
pub struct TrapezoidalConstraints {
    /// Maximum achievable velocity (m/s).
    pub max_velocity: f64,
    /// Maximum achievable acceleration/deceleration (m/s^2).
    pub max_acceleration: f64,
}

impl TrapezoidalConstraints {
    /// Creates new constraints with zero values.
    ///
    /// Use [`set_gains`](Self::set_gains) to configure the constraints.
    pub fn new() -> Self {
        Self {
            max_velocity: 0.,
            max_acceleration: 0.,
        }
    }

    /// Sets the maximum velocity and acceleration constraints.
    ///
    /// # Arguments
    ///
    /// * `max_vel` - Maximum velocity in m/s
    /// * `max_acc` - Maximum acceleration in m/s^2
    ///
    /// # Returns
    ///
    /// Self for builder pattern chaining.
    pub fn set_gains(mut self, max_vel: f64, max_acc: f64) -> Self {
        self.max_velocity = max_vel;
        self.max_acceleration = max_acc;
        self
    }

    /// Generates a trapezoidal motion profile for a given distance.
    ///
    /// Returns a sequence of 100 [`MotionState`] points representing the
    /// complete motion from start to end.
    ///
    /// # Arguments
    ///
    /// * `total_distance` - The total distance to travel
    ///
    /// # Returns
    ///
    /// A vector of motion states sampled evenly across the profile duration.
    ///
    /// # Profile Types
    ///
    /// - **Trapezoidal**: For longer distances where max velocity is reached
    /// - **Triangular**: For shorter distances where max velocity isn't reached
    ///
    /// # Example
    ///
    /// ```no_run
    /// let profile = constraints.generate_profile(QLength::from_meters(1.0));
    /// for state in profile.windows(2) {
    ///     let target_v = state[0].velocity;
    ///     let target_a = (state[1].velocity - state[0].velocity) / dt;
    ///     // Use for feedforward + PID control
    /// }
    /// ```
    pub fn generate_profile(&self, total_distance: QLength) -> Vec<MotionState> {
        let distance = total_distance.as_meters();
        let max_v = self.max_velocity;
        let max_a = self.max_acceleration;
        let t_accel = max_v / max_a;
        let d_accel = 0.5 * max_a * t_accel.powi(2); // or max_v^2 / (2*max_a)
        let d_cruise = distance - 2.0 * d_accel;
        // no cruise
        if d_cruise < 0.0 {
            let v_peak_tri = sqrt(distance * max_a);
            let t_accel_only = v_peak_tri / max_a;
            let t_total = 2.0 * t_accel_only;
            let d_half = 0.5 * distance;
            let samples = 100;
            let dt = t_total / (samples as f64 - 1.0);
            let mut states = Vec::with_capacity(samples);
            for i in 0..samples {
                let t = dt * i as f64;
                let (velocity, acceleration, position) = if t <= t_accel_only {
                    let v = max_a * t;
                    let a = max_a;
                    let p = 0.5 * max_a * t.powi(2);
                    (v, a, p)
                } else {
                    // Decelerating phase
                    let t_dec = t - t_accel_only;
                    let v = v_peak_tri - max_a * t_dec;
                    let a = -max_a;
                    let p_dec = v_peak_tri * t_dec - 0.5 * max_a * t_dec.powi(2);
                    (v, a, d_half + p_dec)
                };
                states.push(MotionState {
                    time: QTime::from_sec(t),
                    position: QLength::from_meters(position),
                    velocity,
                    acceleration,
                });
            }
            states
        } else {
            // Case 2: Full Trapezoidal profile (with cruise phase)
            let t_cruise = d_cruise / max_v;
            let t_total = 2.0 * t_accel + t_cruise;
            let samples = 100; // Fixed sampling rate
            let dt = t_total / (samples as f64 - 1.0);
            let mut states = Vec::with_capacity(samples);
            for i in 0..samples {
                let t = dt * i as f64;
                let (velocity, acceleration, position) = if t < t_accel {
                    // Accelerating
                    let v = max_a * t;
                    let a = max_a;
                    let p = 0.5 * max_a * t.powi(2);
                    (v, a, p)
                } else if t < t_accel + t_cruise {
                    // Cruising
                    let t_cruise_elapsed = t - t_accel;
                    let v = max_v;
                    let a = 0.0;
                    let p = d_accel + max_v * t_cruise_elapsed;
                    (v, a, p)
                } else {
                    // Decelerating
                    let t_dec_elapsed = t - (t_accel + t_cruise);
                    let v = max_v - max_a * t_dec_elapsed;
                    let a = -max_a;
                    let p_accel_cruise = d_accel + max_v * t_cruise;
                    let p_dec = max_v * t_dec_elapsed - 0.5 * max_a * t_dec_elapsed.powi(2);
                    (v, a, p_accel_cruise + p_dec)
                };
                states.push(MotionState {
                    time: QTime::from_sec(t),
                    position: QLength::from_meters(position),
                    velocity,
                    acceleration,
                });
            }
            states
        }
    }
}
