//! Feedforward controllers for motion control.
//!
//! This module provides feedforward controllers that compute motor output based on
//! desired velocity and acceleration, without feedback. When combined with PID,
//! feedforward provides proactive control that anticipates required motor output.
//!
//! # Overview
//!
//! Feedforward uses a physics-based model to predict the motor voltage needed to
//! achieve a desired motion:
//!
//! ```text
//! voltage = ks * sign(v) + kv * v + ka * a
//! ```
//!
//! Where:
//! - `ks`: Static friction compensation (voltage to overcome stiction)
//! - `kv`: Velocity gain (voltage per unit velocity)
//! - `ka`: Acceleration gain (voltage per unit acceleration)
//!
//! # Controllers
//!
//! - [`FeedForward`]: Standard feedforward for linear motion
//! - [`ArmFeedForward`]: Extended feedforward with gravity compensation for arms
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::FeedForward;
//!
//! let mut ff = FeedForward::new();
//! ff.set_ks(0.1);  // Static friction
//! ff.set_kv(0.5);  // Velocity gain
//! ff.set_ka(0.01); // Acceleration gain
//!
//! // Calculate voltage for 2 m/s velocity, 0.5 m/s^2 acceleration
//! let voltage = ff.calculate(2.0, 0.5);
//! ```

use crate::QAngle;

/// Feedforward controller for linear motion.
///
/// Computes motor voltage based on desired velocity and acceleration using:
///
/// ```text
/// voltage = ks * sign(v) + kv * v + ka * a
/// ```
///
/// # Gains
///
/// - `ks`: Static friction compensation. The minimum voltage needed to start
///   moving from rest. Applied in the direction of motion.
/// - `kv`: Velocity feedforward. Voltage required per unit velocity to maintain
///   constant speed against friction and back-EMF.
/// - `ka`: Acceleration feedforward. Voltage required per unit acceleration to
///   overcome inertia.
///
/// # Example
///
/// ```no_run
/// let ff = FeedForward::new()
///     .set_gains(0.1, 0.5, 0.01);
///
/// let voltage = ff.calculate(target_velocity, target_acceleration);
/// ```
pub struct FeedForward {
    /// Static friction compensation (voltage to overcome stiction).
    ks: f64,
    /// Velocity feedforward gain (voltage per unit velocity).
    kv: f64,
    /// Acceleration feedforward gain (voltage per unit acceleration).
    ka: f64,
}

impl FeedForward {
    /// Creates a new feedforward controller with zero gains.
    ///
    /// Use [`set_gains`](Self::set_gains) or individual setters to configure.
    #[inline]
    pub const fn new() -> FeedForward {
        Self {
            ks: 0.,
            kv: 0.,
            ka: 0.,
        }
    }

    /// Returns the static friction gain.
    pub const fn ks(&self) -> f64 {
        self.ks
    }

    /// Returns the velocity feedforward gain.
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the acceleration feedforward gain.
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Sets all feedforward gains at once.
    ///
    /// # Arguments
    ///
    /// * `ks` - Static friction compensation
    /// * `kv` - Velocity feedforward gain
    /// * `ka` - Acceleration feedforward gain
    ///
    /// # Returns
    ///
    /// A new `FeedForward` with the specified gains.
    pub const fn set_gains(&mut self, ks: f64, kv: f64, ka: f64) -> Self {
        Self { ks, kv, ka }
    }

    /// Sets the static friction gain.
    pub const fn set_ks(&mut self, ks: f64) {
        self.ks = ks
    }

    /// Sets the velocity feedforward gain.
    pub const fn set_kv(&mut self, kv: f64) {
        self.kv = kv
    }

    /// Sets the acceleration feedforward gain.
    pub const fn set_ka(&mut self, ka: f64) {
        self.ka = ka
    }

    /// Calculates the feedforward voltage for the given velocity and acceleration.
    ///
    /// # Arguments
    ///
    /// * `velocity` - Desired velocity (units depend on your system)
    /// * `acceleration` - Desired acceleration (units depend on your system)
    ///
    /// # Returns
    ///
    /// The feedforward voltage output.
    ///
    /// # Formula
    ///
    /// ```text
    /// voltage = ks * sign(velocity) + kv * velocity + ka * acceleration
    /// ```
    pub fn calculate(&self, velocity: f64, acceleration: f64) -> f64 {
        self.ks * velocity.signum() + self.kv * velocity + self.ka * acceleration
    }
}

/// Feedforward controller for arm mechanisms with gravity compensation.
///
/// Extends [`FeedForward`] with a gravity compensation term that accounts for
/// the arm's position. This is essential for arms that must hold position
/// against gravity.
///
/// # Formula
///
/// ```text
/// voltage = ks * sign(v) + kv * v + ka * a + kg * g(angle)
/// ```
///
/// Where `g(angle)` is a function that returns the gravity effect at the given
/// angle (typically `cos(angle)` for a simple arm).
///
/// # Example
///
/// ```no_run
/// use kernelvex::{ArmFeedForward, QAngle};
///
/// let ff = ArmFeedForward::new(0.1, 0.5, 0.01, 0.3);
///
/// // Calculate with gravity compensation using cosine
/// let voltage = ff.calculate(
///     QAngle::from_degrees(45.0),
///     target_velocity,
///     target_acceleration,
///     |angle| angle.cos(),
/// );
/// ```
pub struct ArmFeedForward {
    /// Static friction compensation.
    ks: f64,
    /// Velocity feedforward gain.
    kv: f64,
    /// Acceleration feedforward gain.
    ka: f64,
    /// Gravity compensation gain.
    kg: f64,
}

impl ArmFeedForward {
    /// Creates a new arm feedforward controller with the given gains.
    ///
    /// # Arguments
    ///
    /// * `ks` - Static friction compensation
    /// * `kv` - Velocity feedforward gain
    /// * `ka` - Acceleration feedforward gain
    /// * `kg` - Gravity compensation gain
    #[inline]
    pub const fn new(ks: f64, kv: f64, ka: f64, kg: f64) -> Self {
        Self { ks, kv, ka, kg }
    }

    /// Returns the static friction gain.
    #[inline]
    pub const fn ks(&self) -> f64 {
        self.ks
    }

    /// Returns the velocity feedforward gain.
    #[inline]
    pub const fn kv(&self) -> f64 {
        self.kv
    }

    /// Returns the acceleration feedforward gain.
    #[inline]
    pub const fn ka(&self) -> f64 {
        self.ka
    }

    /// Returns the gravity compensation gain.
    #[inline]
    pub const fn kg(&self) -> f64 {
        self.kg
    }

    /// Sets all gains at once, returning a new instance.
    #[inline]
    pub const fn set_gains(self, ks: f64, kv: f64, ka: f64, kg: f64) -> Self {
        Self { ks, kv, ka, kg }
    }

    /// Sets the static friction gain, returning a new instance.
    #[inline]
    pub const fn set_ks(self, ks: f64) -> Self {
        Self { ks, ..self }
    }

    /// Sets the velocity gain, returning a new instance.
    #[inline]
    pub const fn set_kv(self, kv: f64) -> Self {
        Self { kv, ..self }
    }

    /// Sets the acceleration gain, returning a new instance.
    #[inline]
    pub const fn set_ka(self, ka: f64) -> Self {
        Self { ka, ..self }
    }

    /// Sets the gravity gain, returning a new instance.
    #[inline]
    pub const fn set_kg(self, kg: f64) -> Self {
        Self { kg, ..self }
    }

    /// Calculates the feedforward voltage with gravity compensation.
    ///
    /// # Arguments
    ///
    /// * `angle` - Current arm angle
    /// * `velocity` - Desired velocity
    /// * `acceleration` - Desired acceleration
    /// * `g` - Gravity function that takes angle and returns gravity effect
    ///
    /// # Returns
    ///
    /// The feedforward voltage output including gravity compensation.
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Using cosine for simple arm
    /// let voltage = ff.calculate(angle, vel, acc, |a| a.cos());
    ///
    /// // Using custom gravity function
    /// let voltage = ff.calculate(angle, vel, acc, |a| {
    ///     // Custom gravity calculation for 4-bar linkage
    ///     (a + offset).cos() * linkage_factor
    /// });
    /// ```
    #[inline]
    pub fn calculate(
        &self,
        angle: QAngle,
        velocity: f64,
        acceleration: f64,
        g: impl Fn(QAngle) -> f64,
    ) -> f64 {
        self.ks * velocity.signum()
            + self.kv * velocity
            + self.ka * acceleration
            + self.kg * g(angle)
    }
}
