use crate::util::si::{QLength, QTime};
use libm::{sqrt};

/// A single time-indexed state during motion.
/// Velocity and acceleration are stored as f64, as their dimensional analysis
/// is handled implicitly by the profile generation logic.
#[derive(Debug, Clone, Copy)]
pub struct MotionState {
    pub time: QTime,
    /// Position (distance) from start.
    pub position: QLength,
    /// Velocity at this time point (m/s).
    pub velocity: f64,
    /// Acceleration at this time point (m/s^2).
    pub acceleration: f64,
}
/// Constraints for defining the motion envelope.
#[derive(Debug, Clone, Copy)]
pub struct TrapezoidalConstraints {
    /// Maximum achievable velocity (m/s).
    pub max_velocity: f64,
    /// Maximum achievable acceleration/deceleration (m/s^2).
    pub max_acceleration: f64,
}
impl TrapezoidalConstraints {
    /// Generates a trapezoidal motion profile for a given distance.
    ///
    /// Returns a sequence of `MotionState` points, including acceleration/velocity
    /// at discrete time steps.
    ///
    /// The number of samples is fixed for simplicity in this initial implementation.
    /// In a real system, this would be determined by the control loop frequency.
    pub fn generate_profile(&self, total_distance: QLength) -> Vec<MotionState> {
        let distance = total_distance.as_meters();
        let max_v = self.max_velocity;
        let max_a = self.max_acceleration;
        let t_accel = max_v / max_a;
        let d_accel = 0.5 * max_a * t_accel.powi(2); // or max_v^2 / (2*max_a)
        let d_cruise = distance - 2.0 * d_accel;
        // no cruise
        if d_cruise < 0.0 {
            let v_peak_tri = sqrt(2.0 * max_a * distance);
            let t_total = 2.0 * (v_peak_tri / max_a);
            let samples = 100;
            let dt = t_total / (samples as f64 - 1.0);
            let mut states = Vec::with_capacity(samples);
            for i in 0..samples {
                let t = dt * i as f64;
                let pos_at_t = 0.5 * max_a * t.powi(2);
                let (velocity, acceleration, position) = if t <= t_total / 2.0 {
                    let v = max_a * t;
                    let a = max_a;
                    let p = 0.5 * max_a * t.powi(2);
                    (v, a, p)
                } else {
                    // Decelerating phase
                    let t_dec = t - t_total / 2.0;
                    let v = v_peak_tri - max_a * t_dec;
                    let a = -max_a;
                    let p_accel = d_accel;
                    let p_dec = v_peak_tri * t_dec - 0.5 * max_a * t_dec.powi(2);
                    (v, a, p_accel + p_dec)
                };
                states.push(MotionState {
                    time: QTime::from_sec(t),
                    position: QLength::from_meters(position),
                    velocity,
                    acceleration,
                });
            }
            return states;
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

