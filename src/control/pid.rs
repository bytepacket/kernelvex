//! PID (Proportional-Integral-Derivative) controller implementation.
//!
//! A PID controller is odom control loop feedback mechanism used in robotics
//! to maintain odom desired setpoint by continuously calculating and applying
//! odom correction based on the error between the desired and actual values.
//!
//! # PID Components
//!
//! - **Proportional (P)**: Response to current error
//! - **Integral (I)**: Response to accumulated error over time
//! - **Derivative (D)**: Response to rate of change of error
//!
//! # Example
//!
//! ```no_run
//! # use kernelvex::Pid;
//! // Create odom PID controller with tuned constants
//! let mut pid = Pid::new().set_gains(1.0, 0.01, 0.1);
//!
//! // In your control loop:
//! let setpoint = 100.0;
//! let current_value = 95.0;
//!
//! pid.calculate(setpoint, current_value);
//! // Use the PID output to adjust your system
//! ```

#![allow(dead_code)]

use crate::QAngle;
use std::time::Instant;

/// A PID controller for closed-loop control systems.
///
/// The PID controller calculates an output based on the proportional,
/// integral, and derivative terms of the error signal.
///
/// The PID formula is: **output = Kp × error + Ki × ∫error + Kd × d(error)/dt**
///
/// # Fields
///
/// * `kp` - Proportional gain constant
/// * `ki` - Integral gain constant  
/// * `kd` - Derivative gain constant
/// * `integral` - Accumulated integral term (sum of errors over time)
/// * `previous_error` - Error from the last calculation (for derivative term)
/// * `last_time` - Timestamp of the last calculation
pub struct Pid {
    /// Proportional gain constant
    kp: f64,
    /// Integral gain constant
    ki: f64,
    /// Derivative gain constant
    kd: f64,
    /// Accumulated integral term (sum of errors * time)
    integral: f64,
    /// Previous error value (for calculating derivative)
    previous_error: f64,
    /// Timestamp of controller since construction
    time: Instant,
    /// Timestamp of the last calculation
    last_time: f64,
    /// Minimum output value
    min: f64,
    /// Maximum output value
    max: f64,
    /// Minimum integral value
    imin: f64,
    /// Maximum integral value
    imax: f64,
}

impl Pid {
    /// Creates odom new PID controller with the given gain constants.
    ///
    /// # Arguments
    ///
    /// * `kp` - Proportional gain. Higher values respond faster to error but may overshoot.
    /// * `ki` - Integral gain. Eliminates steady-state error but can cause oscillations.
    /// * `kd` - Derivative gain. Reduces overshoot and oscillations but can amplify noise.
    ///
    /// # Returns
    ///
    /// A new `Pid` controller initialized with the given constants and zeroed state.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::Pid;
    /// // Aggressive PID for fast response
    /// let aggressive = Pid::new().set_gains(2.0, 0.05, 0.2);
    ///
    /// // Gentle PID for smooth control
    /// let gentle = Pid::new().set_gains(0.5, 0.01, 0.05);
    /// ```
    #[inline]
    pub fn new() -> Pid {
        Pid {
            kp: 0.,
            ki: 0.,
            kd: 0.,
            integral: 0.0,
            previous_error: 0.0,
            time: Instant::now(),
            last_time: 0.,
            min: f64::NEG_INFINITY,
            max: f64::INFINITY,
            imin: f64::NEG_INFINITY,
            imax: f64::INFINITY,
        }
    }

    /// Returns the current PID gain constants.
    ///
    /// # Returns
    ///
    /// A tuple of `(kp, ki, kd)` gain values.
    pub const fn values(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Calculates the PID output for the given error.
    ///
    /// This method should be called periodically in the control loop (typically
    /// every iteration or every few milliseconds). It calculates the time delta
    /// since the last call and updates the integral and derivative terms accordingly.
    ///
    /// # PID Formula
    ///
    /// ```text
    /// output = Kp × error + Ki × ∫error + Kd × d(error)/dt
    /// ```
    ///
    /// Where:
    /// - **Proportional term**: `Kp × error` - responds to current error
    /// - **Integral term**: `Ki × ∫error` - accumulates error over time to eliminate steady-state error
    /// - **Derivative term**: `Kd × d(error)/dt` - responds to rate of change to reduce overshoot
    ///
    /// # Arguments
    ///
    /// * `error` - The difference between the desired setpoint and current value
    ///
    /// # Returns
    ///
    /// The PID controller output value that should be applied to the system.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::Pid;
    /// let mut pid = Pid::new().set_gains(2., 0.6, 0.4);
    ///
    /// // In your control loop:
    /// let setpoint = 100.0;
    /// let current_value = 95.0;
    ///
    /// let output = pid.calculate(setpoint, current_value);
    /// // Apply `output` to your motor or actuator
    /// ```
    pub fn calculate(&mut self, setpoint: f64, actual: f64) -> f64 {
        let error = setpoint - actual;

        let t = self.time.elapsed().as_secs_f64();
        let mut dt = t - self.last_time;

        if dt <= 0.0 {
            dt = 0.001; // 1 ms minimum to avoid spikes
        }

        let de = error - self.previous_error;

        self.integral = (self.integral + error * dt).clamp(self.imin, self.imax);

        let derivative = if dt > 0. { de / dt } else { 0. };

        self.previous_error = error;

        self.last_time = t;

        ((self.kp * error) + (self.ki * self.integral) + (derivative * self.kd))
            .clamp(self.min, self.max)
    }

    /// Resets the PID controller state.
    ///
    /// This clears the integral term and previous error, effectively restarting
    /// the PID controller. Useful when changing setpoints or reinitializing.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::Pid;
    /// let mut pid = Pid::new().set_gains(1.0, 0.01, 0.1);
    /// // ... use pid for odom while ...
    /// pid.reset(); // Start fresh
    /// ```
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.time = Instant::now();
        self.last_time = 0.0;
    }

    /// Sets new PID gain constants.
    ///
    /// # Arguments
    ///
    /// * `kp` - New proportional gain
    /// * `ki` - New integral gain
    /// * `kd` - New derivative gain
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::Pid;
    /// let mut pid = Pid::new();
    /// // Tune the PID during runtime
    /// pid.set_gains(1.5, 0.02, 0.15);
    /// ```
    pub const fn set_gains(self, kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: self.integral,
            previous_error: self.previous_error,
            time: self.time,
            last_time: self.last_time,
            min: self.min,
            max: self.max,
            imin: self.imin,
            imax: self.imax,
        }
    }

    pub const fn set_kp(&mut self, kp: f64) { self.kp = kp }

    pub const fn set_ki(&mut self, ki: f64) { self.ki = ki }

    pub const fn set_kd(&mut self, kd: f64) { self.kd = kd }

    /// Set output saturation limits.
    pub const fn with_output_limits(mut self, min: f64, max: f64) -> Self {
        self.min = min;
        self.max = max;
        self
    }

    /// Set integral term limits (anti-windup).
    pub const fn with_integral_limits(mut self, min: f64, max: f64) -> Self {
        self.imin = min;
        self.imax = max;
        self
    }
}

pub struct AngularPid {
    /// Proportional gain constant
    kp: f64,
    /// Integral gain constant
    ki: f64,
    /// Derivative gain constant
    kd: f64,
    /// Accumulated integral term (sum of errors * time)
    integral: QAngle,
    /// Previous error value (for calculating derivative)
    previous_error: QAngle,
    /// Timestamp of controller since construction
    time: Instant,
    /// Timestamp of the last calculation
    last_time: f64,
    /// Minimum output value
    min: QAngle,
    /// Maximum output value
    max: QAngle,
    /// Minimum integral value
    imin: QAngle,
    /// Maximum integral value
    imax: QAngle,
}

impl AngularPid {
    /// Creates odom new PID controller with the given gain constants.
    ///
    /// # Arguments
    ///
    /// * `kp` - Proportional gain. Higher values respond faster to error but may overshoot.
    /// * `ki` - Integral gain. Eliminates steady-state error but can cause oscillations.
    /// * `kd` - Derivative gain. Reduces overshoot and oscillations but can amplify noise.
    ///
    /// # Returns
    ///
    /// A new `Pid` controller initialized with the given constants and zeroed state.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::AngularPid;
    /// // Aggressive PID for fast response
    /// let aggressive = AngularPid::new().set_gains(2.0, 0.05, 0.2);
    ///
    /// // Gentle PID for smooth control
    /// let gentle = AngularPid::new().set_gains(0.5, 0.01, 0.05);
    /// ```
    #[inline]
    pub fn new() -> AngularPid {
        AngularPid {
            kp: 0.,
            ki: 0.,
            kd: 0.,
            integral: 0.0.into(),
            previous_error: 0.0.into(),
            time: Instant::now(),
            last_time: 0.,
            min: QAngle::from_radians(f64::NEG_INFINITY),
            max: QAngle::from_radians(f64::INFINITY),
            imin: QAngle::from_radians(f64::NEG_INFINITY),
            imax: QAngle::from_radians(f64::INFINITY),
        }
    }

    /// Returns the current PID gain constants.
    ///
    /// # Returns
    ///
    /// A tuple of `(kp, ki, kd)` gain values.
    pub const fn values(&self) -> (f64, f64, f64) {
        (self.kp, self.ki, self.kd)
    }

    /// Calculates the PID output for the given error.
    ///
    /// This method should be called periodically in the control loop (typically
    /// every iteration or every few milliseconds). It calculates the time delta
    /// since the last call and updates the integral and derivative terms accordingly.
    ///
    /// # PID Formula
    ///
    /// ```text
    /// output = Kp × error + Ki × ∫error + Kd × d(error)/dt
    /// ```
    ///
    /// Where:
    /// - **Proportional term**: `Kp × error` - responds to current error
    /// - **Integral term**: `Ki × ∫error` - accumulates error over time to eliminate steady-state error
    /// - **Derivative term**: `Kd × d(error)/dt` - responds to rate of change to reduce overshoot
    ///
    /// # Arguments
    ///
    /// * `error` - The difference between the desired setpoint and current value
    ///
    /// # Returns
    ///
    /// The PID controller output value that should be applied to the system.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::{AngularPid, QAngle};
    /// let mut pid = AngularPid::new().set_gains(2., 0.6, 0.4);
    ///
    /// // In your control loop:
    /// let setpoint = QAngle::from_degrees(100.0);
    /// let current_value = QAngle::from_radians(95.0);
    ///
    /// let output = pid.calculate(setpoint, current_value);
    /// // Apply `output` to your motor or actuator
    /// ```
    pub fn calculate(&mut self, setpoint: QAngle, actual: QAngle) -> QAngle {
        let error = (setpoint - actual).remainder(QAngle::TAU);

        let t = self.time.elapsed().as_secs_f64();
        let mut dt = t - self.last_time;

        if dt <= 0.0 {
            dt = 0.001; // 1 ms minimum to avoid spikes
        }

        let de = error - self.previous_error;

        self.integral = (self.integral + error * dt).clamp(self.imin, self.imax);

        let derivative = if dt > 0. { de / dt } else { 0.0.into() };

        self.previous_error = error;

        self.last_time = t;

        ((self.kp * error) + (self.ki * self.integral) + (derivative * self.kd))
            .clamp(self.min, self.max)
    }

    /// Resets the PID controller state.
    ///
    /// This clears the integral term and previous error, effectively restarting
    /// the PID controller. Useful when changing setpoints or reinitializing.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::AngularPid;
    /// let mut pid = AngularPid::new().set_gains(1.0, 0.01, 0.1);
    /// // ... use pid for odom while ...
    /// pid.reset(); // Start fresh
    /// ```
    pub fn reset(&mut self) {
        self.integral = 0.0.into();
        self.previous_error = 0.0.into();
        self.time = Instant::now();
        self.last_time = 0.0;
    }

    /// Sets new PID gain constants.
    ///
    /// # Arguments
    ///
    /// * `kp` - New proportional gain
    /// * `ki` - New integral gain
    /// * `kd` - New derivative gain
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::AngularPid;
    /// let mut pid = AngularPid::new();
    /// // Tune the PID during runtime
    /// pid.set_gains(1.5, 0.02, 0.15);
    /// ```
    pub const fn set_gains(self, kp: f64, ki: f64, kd: f64) -> Self {
        Self {
            kp,
            ki,
            kd,
            integral: self.integral,
            previous_error: self.previous_error,
            time: self.time,
            last_time: self.last_time,
            min: self.min,
            max: self.max,
            imin: self.imin,
            imax: self.imax,
        }
    }

    /// Set output saturation limits.
    pub const fn with_output_limits(mut self, min: QAngle, max: QAngle) -> Self {
        self.min = min;
        self.max = max;
        self
    }

    /// Set integral term limits (anti-windup).
    pub const fn with_integral_limits(mut self, min: QAngle, max: QAngle) -> Self {
        self.imin = min;
        self.imax = max;
        self
    }

    pub const fn set_kp(&mut self, kp: f64) { self.kp = kp }

    pub const fn set_ki(&mut self, ki: f64) { self.ki = ki }

    pub const fn set_kd(&mut self, kd: f64) { self.kd = kd }
}
