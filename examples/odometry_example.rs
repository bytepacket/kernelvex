//! Complete odometry example using tracking wheels
//!
//! This example demonstrates how to set up and use tracking wheels for
//! odometry calculations in a VEX robot. It shows best practices for:
//! - Setting up multiple tracking wheels
//! - Reading and processing encoder data
//! - Calculating robot position using wheel deltas
//! - Handling different wheel configurations
//!
//! # Robot Configuration
//!
//! This example assumes a three-wheel odometry setup:
//! - Two parallel wheels (left and right) for forward/backward motion
//! - One perpendicular wheel for sideways motion
//!
//! # Hardware Setup
//!
//! ```text
//!     ┌─────────────┐
//!     │     ▲       │  ← Perpendicular tracking wheel
//!     │     │       │     (measures sideways motion)
//!     │     │       │
//! ◄───┼─────┼───────┼───►
//!     │             │
//!     │   Robot     │
//!     │             │
//!     ●─────────────●  ← Parallel tracking wheels
//!    Left          Right  (measure forward/backward motion)
//! ```

#![no_main]
#![no_std]

use kernelvex::{
    omniwheel::{OmniWheel, Tracking, TrackingWheel},
    pose::Pose,
    si::{QAngle, QLength},
};
use vexide_devices::{
    adi::encoder::AdiEncoder,
    smart::rotation::RotationSensor,
};

/// Represents a complete odometry system with three tracking wheels
pub struct OdometrySystem {
    left_wheel: TrackingWheel<AdiEncoder<360>>,
    right_wheel: TrackingWheel<AdiEncoder<360>>,
    perpendicular_wheel: TrackingWheel<RotationSensor>,
    current_pose: Pose,
}

impl OdometrySystem {
    /// Creates a new odometry system
    ///
    /// # Arguments
    ///
    /// * `left_encoder` - Encoder for the left tracking wheel
    /// * `right_encoder` - Encoder for the right tracking wheel
    /// * `perp_encoder` - Encoder for the perpendicular tracking wheel
    /// * `wheel_separation` - Distance between left and right wheels
    /// * `perp_offset` - Offset of perpendicular wheel from center of rotation
    pub fn new(
        left_encoder: AdiEncoder<360>,
        right_encoder: AdiEncoder<360>,
        perp_encoder: RotationSensor,
        wheel_separation: QLength,
        perp_offset: QLength,
    ) -> Self {
        // Create left tracking wheel (negative offset for left side)
        let left_wheel = TrackingWheel::new(
            left_encoder,
            OmniWheel::Omni275,
            -wheel_separation / 2.0,
            None, // No gearing
        );

        // Create right tracking wheel (positive offset for right side)
        let right_wheel = TrackingWheel::new(
            right_encoder,
            OmniWheel::Omni275,
            wheel_separation / 2.0,
            None,
        );

        // Create perpendicular tracking wheel
        let perpendicular_wheel = TrackingWheel::new(
            perp_encoder,
            OmniWheel::Omni325,
            perp_offset,
            Some(1.5), // Example: 1.5:1 gearing ratio
        );

        Self {
            left_wheel,
            right_wheel,
            perpendicular_wheel,
            current_pose: Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        }
    }

    /// Updates the robot's position based on encoder deltas
    ///
    /// This should be called frequently (e.g., every 10ms) in a control loop
    /// to maintain accurate odometry.
    ///
    /// # Returns
    ///
    /// The updated robot pose
    pub fn update(&mut self) -> Pose {
        // Get the change in distance for each wheel since last update
        let left_delta = self.left_wheel.delta();
        let right_delta = self.right_wheel.delta();
        let perp_delta = self.perpendicular_wheel.delta();

        // Calculate the change in heading (rotation)
        // Using the difference between left and right wheels
        let wheel_base = self.right_wheel.offset() - self.left_wheel.offset();
        let delta_heading = QAngle::from_radians(
            (right_delta.as_meters() - left_delta.as_meters()) / wheel_base.as_meters(),
        );

        // Calculate the local change in position
        // Average of left and right wheels gives forward motion
        let forward_delta = (left_delta + right_delta) / 2.0;

        // For perpendicular wheel, we need to account for the offset during rotation
        // The perpendicular wheel moves in an arc when the robot rotates
        let perp_offset = self.perpendicular_wheel.offset();
        let arc_correction =
            perp_offset.as_meters() * delta_heading.as_radians();
        let sideways_delta = QLength::from_meters(
            perp_delta.as_meters() - arc_correction,
        );

        // Create a local displacement vector
        // This represents the movement in the robot's local coordinate frame
        let local_displacement = Pose::new(
            forward_delta.as_meters(),
            sideways_delta.as_meters(),
            QAngle::from_degrees(0.0),
        );

        // Transform the local displacement to global coordinates
        // We need to rotate by the average heading during this time step
        let avg_heading = self.current_pose.heading() + (delta_heading / 2.0);
        let global_displacement = local_displacement.rotate(avg_heading);

        // Update the current pose
        self.current_pose = self.current_pose + global_displacement;
        self.current_pose = self.current_pose.rotate(delta_heading);

        self.current_pose
    }

    /// Gets the current robot pose
    pub fn pose(&self) -> Pose {
        self.current_pose
    }

    /// Resets the odometry to a new pose
    ///
    /// This is useful for setting the initial position or resetting
    /// position after a known location is reached.
    pub fn reset(&mut self, pose: Pose) {
        self.current_pose = pose;
        self.left_wheel.reset();
        self.right_wheel.reset();
        self.perpendicular_wheel.reset();
    }

    /// Gets the distance traveled by each wheel (for debugging)
    pub fn wheel_distances(&mut self) -> (QLength, QLength, QLength) {
        (
            self.left_wheel.distance(),
            self.right_wheel.distance(),
            self.perpendicular_wheel.distance(),
        )
    }
}

// Example usage (would be in main function in real code)
// Note: This is a no_run example since we can't actually run it without hardware
//
// ```no_run
// // In your main function or control loop:
// let mut odometry = OdometrySystem::new(
//     left_encoder,
//     right_encoder,
//     perp_encoder,
//     QLength::from_inches(10.0),  // 10 inches between wheels
//     QLength::from_inches(5.0),   // 5 inches perpendicular offset
// );
//
// // In your periodic control loop (e.g., every 10ms):
// loop {
//     let current_pose = odometry.update();
//     let (x, y) = current_pose.position();
//     let heading = current_pose.heading();
//     
//     println!("Position: ({:.2}, {:.2}) inches", x * 39.37, y * 39.37);
//     println!("Heading: {:.2} degrees", heading.as_degrees());
//     
//     // Use position for autonomous navigation, etc.
//     
//     vexide_devices::sleep(vexide_devices::time::Duration::from_millis(10));
// }
// ```
