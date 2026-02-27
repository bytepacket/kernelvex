//! RAMSETE controller for trajectory tracking.

use crate::motion::trajectory::TrajectoryPoint;
use crate::odom::pose::Pose;
use crate::util::si::QAngle;

/// RAMSETE controller configuration and calculation.
#[derive(Debug, Clone, Copy)]
pub struct RamseteController {
    b: f64,
    zeta: f64,
    epsilon: f64,
}

/// A trajectory reference for RAMSETE tracking.
#[derive(Debug, Clone, Copy)]
pub struct RamseteReference {
    pub pose: Pose,
    /// Desired linear velocity in meters per second.
    pub linear_velocity: f64,
    /// Desired angular velocity in radians per second.
    pub angular_velocity: f64,
}

impl RamseteReference {
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
    fn from(point: TrajectoryPoint) -> Self {
        Self {
            pose: point.pose,
            linear_velocity: point.linear_velocity,
            angular_velocity: point.angular_velocity,
        }
    }
}

impl RamseteController {
    /// Creates a RAMSETE controller with the given tuning parameters.
    ///
    /// Common starting values are b = 2.0, zeta = 0.7.
    #[inline]
    pub fn new(b: f64, zeta: f64) -> Self {
        Self {
            b,
            zeta,
            epsilon: 1e-6,
        }
    }

    /// Sets the small-angle epsilon for the sinc term.
    #[inline]
    pub fn with_epsilon(mut self, epsilon: f64) -> Self {
        self.epsilon = epsilon;
        self
    }

    /// Computes the linear and angular velocity commands.
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

fn sinc(theta: f64, epsilon: f64) -> f64 {
    if libm::fabs(theta) < epsilon {
        1.0
    } else {
        libm::sin(theta) / theta
    }
}
