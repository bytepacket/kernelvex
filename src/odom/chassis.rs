//! Odometry-enabled chassis control for autonomous robot movement.
//!
//! This module provides the [`OdomChassis`] struct, a high-level controller that combines
//! a differential drivetrain with odometry tracking, PID control, feedforward, and trajectory
//! following capabilities using RAMSETE.
//!
//! # Overview
//!
//! `OdomChassis` unifies all the components needed for precise autonomous robot control:
//! - **Drivetrain**: Controls motor outputs via [`DifferentialDrive`]
//! - **Odometry**: Tracks robot position using either a [`TrackingRig`] or IME fallback
//! - **Motion Profiles**: Generates smooth velocity profiles via [`TrapezoidalConstraints`]
//! - **Trajectory Following**: Uses RAMSETE controller for curved path tracking
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{OdomChassis, DifferentialDrive, Pose, QLength, QAngle};
//!
//! // Create chassis with builder pattern
//! let chassis = OdomChassis::new(drivetrain, imu, Some(tracking_rig))
//!     .with_linear_pid(linear_pid)
//!     .with_angular_pid(angular_pid)
//!     .with_ff(feedforward)
//!     .with_constraints(constraints);
//!
//! // Drive straight 1 meter
//! chassis.shoot(QLength::from_meters(1.0)).await?;
//!
//! // Turn to 90 degrees
//! chassis.turn(QAngle::from_degrees(90.0)).await?;
//!
//! // Drive to a specific pose (turn + shoot)
//! chassis.shoot_to_pose(target_pose).await?;
//! ```

use crate::control::feedforward::FeedForward;
use crate::control::pid::{AngularPid, Pid};
use crate::control::purepursuit::PurePursuit;
use crate::control::ramsete::{RamseteController, RamseteReference};
use crate::dt::model::Tank;
use crate::motion::profile::TrapezoidalConstraints;
use crate::motion::trajectory::Trajectory;
use crate::util::si::{QAngle, QLength, QTime};
use crate::util::utils::GroupErrors;
use crate::{DifferentialDrive, Drivetrain, Pose, TrackingRig};
use core::time::Duration;
use vexide_async::time::sleep;
use vexide_devices::smart::imu::InertialSensor;
use vexide_devices::smart::motor::Motor;

/// Unified error type for drive operations.
///
/// Wraps motor-related errors that can occur during chassis movement commands.
///
/// # Variants
///
/// * `Motor` - Contains a collection of motor port errors from the drivetrain
#[derive(Debug)]
pub enum DriveError {
    /// Motor group encountered one or more port errors.
    Motor(GroupErrors),
}

/// A unified chassis controller with odometry, PID, feedforward, and trajectory support.
///
/// `OdomChassis` provides a high-level API for autonomous robot movement, combining:
/// - Differential drivetrain control
/// - IMU-based heading measurement
/// - Optional tracking rig for full pose estimation
/// - Linear and angular PID controllers
/// - Feedforward for velocity/acceleration compensation
/// - RAMSETE controller for curved trajectory following
/// - Trapezoidal motion profile generation
///
/// # Velocity Feedback
///
/// When a [`TrackingRig`] is provided, velocity feedback comes from the tracking wheels.
/// Otherwise, the chassis falls back to Integrated Motor Encoder (IME) velocity estimation
/// from the drivetrain motors.
///
/// # Builder Pattern
///
/// Use the `with_*` methods to configure the chassis:
///
/// ```no_run
/// let chassis = OdomChassis::new(dt, imu, tracking)
///     .with_linear_pid(Pid::new().with_kp(2.0))
///     .with_angular_pid(AngularPid::new().with_kp(1.5))
///     .with_ff(FeedForward::new().set_gains(0.1, 0.5, 0.01))
///     .with_constraints(TrapezoidalConstraints::new().set_gains(1.0, 2.0));
/// ```
pub struct OdomChassis {
    /// The differential drivetrain for motor control.
    dt: DifferentialDrive,
    /// Inertial sensor for heading measurement.
    imu: InertialSensor,
    /// Optional tracking rig for full pose estimation.
    tracking: Option<TrackingRig>,
    /// Current pose estimate (used when no tracking rig is present).
    pose: Pose,
    /// PID controller for linear (forward/backward) motion.
    linear_pid: Pid,
    /// PID controller for left-side velocity during trajectory following.
    left_pid: Pid,
    /// PID controller for right-side velocity during trajectory following.
    right_pid: Pid,
    /// PID controller for angular (turning) motion.
    angular_pid: AngularPid,
    /// Feedforward controller for velocity/acceleration compensation.
    ff: FeedForward,
    /// RAMSETE controller for curved trajectory following.
    ramsete: RamseteController,
    /// Motion profile constraints (max velocity and acceleration).
    constraints: TrapezoidalConstraints
}

impl OdomChassis {
    /// Creates an `OdomChassis` with default configuration.
    ///
    /// All PID gains and feedforward constants are initialized to zero.
    /// Use the `with_*` builder methods to configure the chassis.
    ///
    /// # Arguments
    ///
    /// * `dt` - The differential drivetrain to control
    /// * `imu` - Inertial sensor for heading measurement
    /// * `tracking` - Optional tracking rig for full pose estimation. If `None`,
    ///   velocity feedback falls back to motor encoders (IME).
    ///
    /// # Example
    ///
    /// ```no_run
    /// let chassis = OdomChassis::new(drivetrain, imu, Some(tracking_rig));
    /// ```
    pub fn new(dt: DifferentialDrive, imu: InertialSensor, tracking: Option<TrackingRig>) -> Self {
        Self::with_config(dt, imu, tracking)
    }

    /// Creates an `OdomChassis` with custom configuration.
    ///
    /// This is the internal constructor that initializes all controllers with
    /// default values. Use the builder methods to customize.
    ///
    /// # Arguments
    ///
    /// * `dt` - The differential drivetrain to control
    /// * `imu` - Inertial sensor for heading measurement
    /// * `tracking` - Optional tracking rig for pose estimation
    pub fn with_config(
        dt: DifferentialDrive,
        imu: InertialSensor,
        tracking: Option<TrackingRig>,
    ) -> Self {
        let linear_pid = Pid::new()
            .with_output_limits(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
        let left_pid = Pid::new()
            .with_output_limits(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
        let right_pid = Pid::new()
            .with_output_limits(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
        let angular_pid = AngularPid::new()
            .with_output_limits(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
        let ff = FeedForward::new();
        let ramsete = RamseteController::new();

        Self {
            dt,
            imu,
            tracking,
            pose: Default::default(),
            linear_pid,
            left_pid,
            right_pid,
            angular_pid,
            ff,
            ramsete,
            constraints: TrapezoidalConstraints::new()
        }
    }

    /// Sets the linear PID controller for straight-line motion.
    ///
    /// This PID controller is used by [`shoot`](Self::shoot) to regulate velocity
    /// during straight-line movements.
    ///
    /// # Arguments
    ///
    /// * `pid` - The PID controller to use for linear motion
    pub fn with_linear_pid(mut self, pid: Pid) -> Self {
        self.linear_pid = pid;
        self
    }

    /// Sets the angular PID controller for turning.
    ///
    /// This PID controller is used by [`turn`](Self::turn) to regulate heading
    /// during in-place turns.
    ///
    /// # Arguments
    ///
    /// * `pid` - The angular PID controller to use
    pub fn with_angular_pid(mut self, pid: AngularPid) -> Self {
        self.angular_pid = pid;
        self
    }

    /// Sets the RAMSETE controller for trajectory following.
    ///
    /// The RAMSETE controller computes velocity commands to track curved
    /// trajectories while correcting for position and heading errors.
    ///
    /// # Arguments
    ///
    /// * `ramsete` - The RAMSETE controller configuration
    pub fn with_ramsete(mut self, ramsete: RamseteController) -> Self {
        self.ramsete = ramsete;
        self
    }

    /// Sets the left-side PID controller for trajectory following.
    ///
    /// During trajectory following, separate PID controllers regulate the
    /// left and right wheel velocities.
    ///
    /// # Arguments
    ///
    /// * `pid` - The PID controller for left-side velocity
    pub fn with_left_pid(mut self, pid: Pid) -> Self {
        self.left_pid = pid;
        self
    }

    /// Sets the right-side PID controller for trajectory following.
    ///
    /// During trajectory following, separate PID controllers regulate the
    /// left and right wheel velocities.
    ///
    /// # Arguments
    ///
    /// * `pid` - The PID controller for right-side velocity
    pub fn with_right_pid(mut self, pid: Pid) -> Self {
        self.right_pid = pid;
        self
    }

    /// Sets the feedforward controller for velocity/acceleration compensation.
    ///
    /// Feedforward provides proactive motor voltage based on desired velocity
    /// and acceleration, improving motion profile tracking.
    ///
    /// # Arguments
    ///
    /// * `ff` - The feedforward controller with ks, kv, ka gains
    pub fn with_ff(mut self, ff: FeedForward) -> Self {
        self.ff = ff;
        self
    }

    /// Sets the motion profile constraints.
    ///
    /// These constraints define the maximum velocity and acceleration for
    /// trapezoidal motion profiles used by [`shoot`](Self::shoot).
    ///
    /// # Arguments
    ///
    /// * `constraints` - The trapezoidal profile constraints
    pub fn with_constraints(mut self, constraints: TrapezoidalConstraints) -> Self {
        self.constraints = constraints;
        self
    }

    /// Returns the current heading from the IMU.
    ///
    /// If the IMU read fails, returns 0 radians.
    ///
    /// # Returns
    ///
    /// The current heading as a [`QAngle`]
    pub fn heading(&self) -> QAngle {
        self.imu
            .heading()
            .map(|a| QAngle::from_radians(a.as_radians()))
            .unwrap_or(QAngle::from_radians(0.0))
    }


    /// Drives the robot straight for a specified distance using a trapezoidal motion profile.
    ///
    /// This method generates a trapezoidal velocity profile and executes it using
    /// feedforward + PID control. Velocity feedback comes from the tracking rig if
    /// available, otherwise from motor encoders (IME fallback).
    ///
    /// # Arguments
    ///
    /// * `distance` - The distance to travel (positive = forward, negative = backward)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Movement completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Drive forward 1 meter
    /// chassis.shoot(QLength::from_meters(1.0)).await?;
    ///
    /// // Drive backward 50 centimeters
    /// chassis.shoot(QLength::from_meters(-0.5)).await?;
    /// ```
    pub async fn shoot(&mut self, distance: QLength) -> Result<(), DriveError> {
        let constraints = TrapezoidalConstraints {
            max_velocity: self.constraints.max_velocity,
            max_acceleration: self.constraints.max_acceleration,
        };
        let profile = constraints.generate_profile(distance);

        self.linear_pid.reset();

        for window in profile.windows(2) {
            let current = &window[0];
            let next = &window[1];

            let target_v = current.velocity;
            let dt = (next.time - current.time).as_sec().max(1e-3);
            let target_a = (next.velocity - current.velocity) / dt;

            let measured_v = if let Some(tracking) = self.tracking.as_ref() {
                tracking.linear_velocity()
            } else {
                self.dt.linear_velocity().await.unwrap_or(0.0)
            };

            let volts_pid = self.linear_pid.calculate(target_v, measured_v);
            let volts_ff = self.ff.calculate(target_v, target_a);
            let volts = (volts_pid + volts_ff).clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);

            let fraction = volts / Motor::V5_MAX_VOLTAGE;
            self.dt.drive_tank(fraction, fraction).await.map_err(DriveError::Motor)?;

            sleep(Duration::from_secs_f64(dt)).await;
        }

        self.dt.drive_tank(0.0, 0.0).await.map_err(DriveError::Motor)?;
        Ok(())
    }

    /// Turns the robot in place to the specified absolute heading.
    ///
    /// Uses the angular PID controller to turn until the heading error is within
    /// a 2-degree tolerance. The turn direction is automatically chosen to take
    /// the shortest path.
    ///
    /// # Arguments
    ///
    /// * `target` - The target absolute heading (not relative to current heading)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Turn completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Turn to face 90 degrees
    /// chassis.turn(QAngle::from_degrees(90.0)).await?;
    ///
    /// // Turn to face north (0 degrees)
    /// chassis.turn(QAngle::from_degrees(0.0)).await?;
    /// ```
    pub async fn turn(&mut self, target: QAngle) -> Result<(), DriveError> {
        let angle_tolerance = QAngle::from_degrees(2.0);

        self.angular_pid.reset();

        loop {
            let current_heading = self.heading();
            let error = (target - current_heading).remainder(QAngle::TAU);

            if error.abs().as_radians() <= angle_tolerance.as_radians() {
                self.dt.drive_tank(0.0, 0.0).await.map_err(DriveError::Motor)?;
                break;
            }

            let output = self.angular_pid.calculate(target, current_heading);
            let turn = (output / Motor::V5_MAX_VOLTAGE).clamp(-1.0, 1.0);

            self.dt.drive_tank(turn, -turn).await.map_err(DriveError::Motor)?;
            sleep(Duration::from_millis(10)).await;
        }

        Ok(())
    }

    /// Follows a pre-generated trajectory using RAMSETE control.
    ///
    /// This method uses the RAMSETE controller to compute velocity commands that
    /// track the trajectory while correcting for position and heading errors.
    /// Differential wheel velocities are computed from the RAMSETE output and
    /// regulated with per-wheel PID + feedforward.
    ///
    /// # Arguments
    ///
    /// * `traj` - The trajectory to follow (must have time-parameterized points)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Trajectory completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Panics
    ///
    /// This method requires a tracking rig for pose feedback. If no tracking rig
    /// is present, pose estimation defaults to heading-only (position at origin).
    ///
    /// # Example
    ///
    /// ```no_run
    /// let trajectory = Trajectory::from_cubic_bezier(
    ///     Vector2::new(0.0, 0.0),
    ///     Vector2::new(0.5, 0.0),
    ///     Vector2::new(0.5, 1.0),
    ///     Vector2::new(1.0, 1.0),
    ///     QTime::from_sec(3.0),
    ///     100,
    ///     0.5,
    /// );
    /// chassis.trajectory(&trajectory).await?;
    /// ```
    pub async fn trajectory(&mut self, traj: &Trajectory) -> Result<(), DriveError> {
        let track_width_m = self.dt.width.as_meters();
        let total_time = traj.total_time().unwrap_or(QTime::from_sec(0.0)).as_sec();
        let mut last_left_target = 0.0;
        let mut last_right_target = 0.0;
        let mut last_time = 0.0;
        let start = std::time::Instant::now();

        self.left_pid.reset();
        self.right_pid.reset();

        loop {
            let t = QTime::from_sec(start.elapsed().as_secs_f64());
            if t.as_sec() > total_time + 0.05 {
                break;
            }

            let point = match traj.sample(t) {
                Some(p) => p,
                None => break,
            };

            let heading = self.heading();
            let pose = if let Some(tracking) = self.tracking.as_ref() {
                Pose::new(tracking.pose().position(), heading)
            } else {
                Pose::new(Default::default(), heading)
            };

            let reference = RamseteReference::from(point);
            let (v, w) = self.ramsete.calculate(pose, reference);

            let left_target = v - w * (track_width_m * 0.5);
            let right_target = v + w * (track_width_m * 0.5);

            let dt = (t.as_sec() - last_time).max(1e-3);
            let left_accel = (left_target - last_left_target) / dt;
            let right_accel = (right_target - last_right_target) / dt;
            last_left_target = left_target;
            last_right_target = right_target;
            last_time = t.as_sec();

            let (meas_v, meas_w) = if let Some(tracking) = self.tracking.as_ref() {
                (tracking.linear_velocity(), tracking.angular_velocity())
            } else {
                let v = self.dt
                    .linear_velocity()
                    .await
                    .unwrap_or(0.0);
                let w = self.dt
                    .angular_velocity()
                    .await
                    .unwrap_or(0.0);
                (v, w)
            };
            let left_meas = meas_v - meas_w * (track_width_m * 0.5);
            let right_meas = meas_v + meas_w * (track_width_m * 0.5);

            let left_volts = (self.left_pid.calculate(left_target, left_meas)
                + self.ff.calculate(left_target, left_accel))
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
            let right_volts = (self.right_pid.calculate(right_target, right_meas)
                + self.ff.calculate(right_target, right_accel))
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);

            let left = left_volts / Motor::V5_MAX_VOLTAGE;
            let right = right_volts / Motor::V5_MAX_VOLTAGE;

            self.dt.drive_tank(left, right).await.map_err(DriveError::Motor)?;
            sleep(Duration::from_millis(10)).await;
        }

        self.dt.drive_tank(0.0, 0.0).await.map_err(DriveError::Motor)?;
        Ok(())
    }

    /// Follows a trajectory using pure pursuit control.
    ///
    /// Pure pursuit is a geometric path follower that steers toward a lookahead
    /// point on the trajectory. It's simpler than RAMSETE and works well for
    /// smooth paths.
    ///
    /// # Algorithm
    ///
    /// 1. Find the lookahead point (circle-trajectory intersection)
    /// 2. Compute curvature to reach that point
    /// 3. Convert (velocity, curvature) to differential wheel speeds
    /// 4. Apply PID + feedforward control
    ///
    /// Uses the same `left_pid`, `right_pid`, and feedforward as [`trajectory()`](Self::trajectory).
    ///
    /// # Arguments
    ///
    /// * `path` - The pure pursuit controller containing trajectory and lookahead distance
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Path completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Panics
    ///
    /// Panics if no tracking rig is present (required for position feedback).
    ///
    /// # Example
    ///
    /// ```no_run
    /// let trajectory = Trajectory::from_points(points);
    /// let pursuit = PurePursuit::new(trajectory, 0.3); // 30cm lookahead
    /// chassis.pursuit(&pursuit).await?;
    /// ```
    pub async fn pursuit(&mut self, path: &PurePursuit) -> Result<(), DriveError> {
        const EXIT_TOLERANCE: f64 = 0.05;

        assert!(self.tracking.is_some(), "pure pursuit requires tracking rig");

        let track_width = self.dt.width.as_meters();
        let trajectory = path.trajectory();
        let final_point = match trajectory.points().last() {
            Some(p) => p.pose.position(),
            None => return Ok(()), // Empty trajectory
        };

        self.left_pid.reset();
        self.right_pid.reset();

        let mut last_left_target = 0.0;
        let mut last_right_target = 0.0;
        let mut last_time = std::time::Instant::now();

        loop {
            let heading = self.heading();
            let tracking = self.tracking.as_ref().unwrap();
            let position = tracking.pose().position();
            let pose = Pose::new(position, heading);

            let dx = final_point.x - position.x;
            let dy = final_point.y - position.y;
            let dist_to_end = libm::sqrt(dx * dx + dy * dy);
            if dist_to_end < EXIT_TOLERANCE {
                break;
            }

            let target_point = match path.intersect(pose) {
                Some(p) => p,
                None => break,
            };

            let curvature = path.curvature(pose, target_point.pose.position());

            let target_v = target_point.linear_velocity;


            let w = curvature * target_v;
            let left_target = target_v - w * (track_width * 0.5);
            let right_target = target_v + w * (track_width * 0.5);

            let now = std::time::Instant::now();
            let dt = now.duration_since(last_time).as_secs_f64().max(1e-3);
            let left_accel = (left_target - last_left_target) / dt;
            let right_accel = (right_target - last_right_target) / dt;
            last_left_target = left_target;
            last_right_target = right_target;
            last_time = now;

            let (meas_v, meas_w) = (tracking.linear_velocity(), tracking.angular_velocity());
            let left_meas = meas_v - meas_w * (track_width * 0.5);
            let right_meas = meas_v + meas_w * (track_width * 0.5);

            let left_volts = (self.left_pid.calculate(left_target, left_meas)
                + self.ff.calculate(left_target, left_accel))
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);
            let right_volts = (self.right_pid.calculate(right_target, right_meas)
                + self.ff.calculate(right_target, right_accel))
                .clamp(-Motor::V5_MAX_VOLTAGE, Motor::V5_MAX_VOLTAGE);

            let left = left_volts / Motor::V5_MAX_VOLTAGE;
            let right = right_volts / Motor::V5_MAX_VOLTAGE;

            self.dt.drive_tank(left, right).await.map_err(DriveError::Motor)?;
            sleep(Duration::from_millis(10)).await;
        }

        self.dt.drive_tank(0.0, 0.0).await.map_err(DriveError::Motor)?;
        Ok(())
    }

    /// Sets the robot's current pose estimate.
    ///
    /// Use this to initialize the pose at the start of autonomous or to
    /// correct drift during operation.
    ///
    /// # Arguments
    ///
    /// * `pose` - The new pose to set
    pub fn set_pose(&mut self, pose: &Pose) {
        self.pose = pose.clone();
    }

    /// Returns the current pose estimate.
    ///
    /// If a tracking rig is present, prefer using `tracking.pose()` directly
    /// for the most up-to-date estimate.
    ///
    /// # Returns
    ///
    /// The current pose estimate
    pub fn get_pose(&self) -> Pose {
        self.pose
    }

    /// Turns the robot to face a target pose.
    ///
    /// Computes the angle from the robot's current position to the target pose
    /// and turns to face that direction. Requires a tracking rig for position.
    ///
    /// # Arguments
    ///
    /// * `pose` - The target pose to face
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Turn completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Panics
    ///
    /// Panics if no tracking rig is present (required for position feedback).
    ///
    /// # Example
    ///
    /// ```no_run
    /// let target = Pose::new(Vector2::new(1.0, 2.0), QAngle::from_degrees(0.0));
    /// chassis.turn_to_pose(target).await?;
    /// ```
    pub async fn turn_to_pose(&mut self, pose: Pose) -> Result<(), DriveError> {
        {
            assert!(self.tracking.is_some(), "must have tracking");
        }

        let pos = self.tracking.as_ref().unwrap().pose().position();

        let dx = pose.position().x - pos.x;
        let dy = pose.position().y - pos.y;

        let angle = QAngle::from_radians(libm::atan2(dy, dx));

        self.turn(angle).await
    }

    /// Drives to a target pose using a two-step motion: turn then shoot.
    ///
    /// This method first turns to face the target pose using [`turn_to_pose`](Self::turn_to_pose),
    /// then drives straight to it using [`shoot`](Self::shoot). This is simpler than
    /// trajectory following but only works for point-to-point movements.
    ///
    /// # Arguments
    ///
    /// * `pose` - The target pose to reach
    ///
    /// # Returns
    ///
    /// * `Ok(())` - Movement completed successfully
    /// * `Err(DriveError::Motor)` - Motor communication error
    ///
    /// # Panics
    ///
    /// Panics if no tracking rig is present (required for position feedback).
    ///
    /// # Example
    ///
    /// ```no_run
    /// // Drive to position (1, 2) meters
    /// let target = Pose::new(Vector2::new(1.0, 2.0), QAngle::from_degrees(0.0));
    /// chassis.shoot_to_pose(target).await?;
    /// ```
    pub async fn shoot_to_pose(&mut self, pose: Pose) -> Result<(), DriveError> {
        self.turn_to_pose(pose).await?;
        let pos = self.tracking.as_ref().unwrap().pose().position();
        let dx = pose.position().x - pos.x;
        let dy = pose.position().y - pos.y;
        let dist = libm::sqrt(dx*dx + dy*dy);
        self.shoot(QLength::from_meters(dist)).await
    }
}


