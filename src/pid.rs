//! PID (Proportional-Integral-Derivative) controller implementation.
//!
//! A PID controller is a control loop feedback mechanism used in robotics
//! to maintain a desired setpoint by continuously calculating and applying
//! a correction based on the error between the desired and actual values.
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
//! # use kernelvex::pid::Pid;
//! // Create a PID controller with tuned constants
//! let mut pid = Pid::new(1.0, 0.01, 0.1);
//!
//! // In your control loop:
//! let setpoint = 100.0;
//! let current_value = 95.0;
//! let error = setpoint - current_value;
//!
//! pid.calculate(error);
//! // Use the PID output to adjust your system
//! ```

#![allow(dead_code)]

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
    kp: f32,
    /// Integral gain constant
    ki: f32,
    /// Derivative gain constant
    kd: f32,
    /// Accumulated integral term (sum of errors * time)
    integral: f32,
    /// Previous error value (for calculating derivative)
    previous_error: f32,
    /// Timestamp of the last calculation
    last_time: Instant,
}

impl Pid {
    /// Creates a new PID controller with the given gain constants.
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
    /// # use kernelvex::pid::Pid;
    /// // Aggressive PID for fast response
    /// let aggressive = Pid::new(2.0, 0.05, 0.2);
    ///
    /// // Gentle PID for smooth control
    /// let gentle = Pid::new(0.5, 0.01, 0.05);
    /// ```
    #[inline]
    pub fn new(kp: f32, ki: f32, kd: f32) -> Pid {
        Pid {
            kp,
            ki,
            kd,
            integral: 0.0,
            previous_error: 0.0,
            last_time: Instant::now(),
        }
    }

    /// Returns the current PID gain constants.
    ///
    /// # Returns
    ///
    /// A tuple of `(kp, ki, kd)` gain values.
    pub fn values(&self) -> (f32, f32, f32) {
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
    /// # use kernelvex::pid::Pid;
    /// let mut pid = Pid::new(1.0, 0.01, 0.1);
    ///
    /// // In your control loop:
    /// let setpoint = 100.0;
    /// let current_value = 95.0;
    /// let error = setpoint - current_value;
    ///
    /// let output = pid.calculate(error);
    /// // Apply `output` to your motor or actuator
    /// ```
    pub fn calculate(&mut self, error: f32) -> f32 {
        // Calculate time delta in seconds
        let now = Instant::now();
        let dt = now.duration_since(self.last_time).as_secs_f32();
        self.last_time = now;

        let dt = if dt > 0.0 { dt } else { 0.001 }; // Default to 1ms if dt is 0

        // Proportional term: current error
        let proportional = self.kp * error;

        // Integral term: accumulate error over time
        self.integral += error * dt;
        let integral = self.ki * self.integral;

        // Derivative term: rate of change of error
        // This helps reduce overshoot and oscillations
        let derivative = if dt > 0.0 {
            self.kd * (error - self.previous_error) / dt
        } else {
            0.0
        };

        // Store current error for next calculation
        self.previous_error = error;

        // Calculate and return PID output
        proportional + integral + derivative
    }

    /// Resets the PID controller state.
    ///
    /// This clears the integral term and previous error, effectively restarting
    /// the PID controller. Useful when changing setpoints or reinitializing.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// # use kernelvex::pid::Pid;
    /// let mut pid = Pid::new(1.0, 0.01, 0.1);
    /// // ... use pid for a while ...
    /// pid.reset(); // Start fresh
    /// ```
    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.previous_error = 0.0;
        self.last_time = Instant::now();
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
    /// # use kernelvex::pid::Pid;
    /// let mut pid = Pid::new(1.0, 0.01, 0.1);
    /// // Tune the PID during runtime
    /// pid.set_gains(1.5, 0.02, 0.15);
    /// ```
    pub fn set_gains(&mut self, kp: f32, ki: f32, kd: f32) {
        self.kp = kp;
        self.ki = ki;
        self.kd = kd;
    }
}
