//! Odometry chassis implementation.
//!
//! Combines a drivetrain with an inertial sensor and tracking wheel
//! to provide position tracking capabilities.

use crate::model::Drivetrain;
use crate::pose::Pose;
use crate::sensors::Encoder;
use crate::si::QAngle;
use crate::wheel::{Tracking, TrackingWheel};
use vexide_devices::smart::imu::InertialSensor;

/// An odometry-enabled chassis that tracks robot position.
///
/// `OdomChassis` wraps a drivetrain with an inertial sensor and tracking
/// wheel to continuously estimate the robot's position on the field.
///
/// # Type Parameters
///
/// * `D` - The drivetrain type (must implement [`Drivetrain`])
/// * `T` - The encoder type used by the tracking wheel (must implement [`Encoder`])
pub struct OdomChassis<D: Drivetrain, T: Encoder> {
    dt: D,
    imu: InertialSensor,
    wheel: TrackingWheel<T>,
    pose: Pose,
    prev_heading: QAngle,
}

impl<D: Drivetrain, T: Encoder> OdomChassis<D, T> {
    /// Creates a new odometry chassis.
    ///
    /// # Arguments
    ///
    /// * `dt` - The drivetrain to control
    /// * `imu` - The inertial sensor for heading measurement
    /// * `wheel` - The tracking wheel for distance measurement
    pub fn new(dt: D, imu: InertialSensor, wheel: TrackingWheel<T>) -> Self {
        Self {
            dt,
            imu,
            wheel,
            pose: Pose::default(),
            prev_heading: QAngle::default(),
        }
    }

    /// Returns a reference to the underlying drivetrain.
    pub fn drivetrain(&self) -> &D {
        &self.dt
    }

    /// Returns a mutable reference to the underlying drivetrain.
    pub fn drivetrain_mut(&mut self) -> &mut D {
        &mut self.dt
    }

    /// Returns the current estimated pose.
    pub fn pose(&self) -> Pose {
        self.pose
    }

    /// Resets the pose to a given position.
    pub fn set_pose(&mut self, pose: Pose) {
        self.pose = pose;
    }

    /// Updates the odometry estimate.
    ///
    /// Call this method periodically (e.g., every control loop iteration)
    /// to keep the pose estimate up to date.
    pub fn update(&mut self) {
        let imu_heading = self.imu.heading().unwrap_or(
            vexide_devices::math::Angle::from_degrees(0.0),
        );
        let heading = QAngle::from(imu_heading);

        let delta_dist = self.wheel.delta();
        let delta_heading = heading - self.prev_heading;

        let avg_heading = self.prev_heading + delta_heading / 2.0;

        let dx = delta_dist.as_meters() * avg_heading.cos();
        let dy = delta_dist.as_meters() * avg_heading.sin();

        let delta_pose = Pose::new(dx, dy, QAngle::default());
        self.pose = self.pose + delta_pose;
        self.pose = Pose::new(
            self.pose.position().0,
            self.pose.position().1,
            heading,
        );

        self.prev_heading = heading;
    }
}