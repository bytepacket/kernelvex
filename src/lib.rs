//! # kernelvex
//!
//! A VEX robotics control library for the Vexide platform.
//!
//! `kernelvex` provides high-level abstractions for building autonomous and
//! driver-controlled robot programs. It includes odometry, motion control,
//! trajectory following, and drivetrain management.
//!
//! ## Features
//!
//! - **Odometry**: Track robot position using [`TrackingRig`] with wheel encoders and IMU
//! - **Drivetrains**: [`DifferentialDrive`] with tank, arcade, and curvature control
//! - **Motion Profiles**: [`TrapezoidalConstraints`] for smooth acceleration
//! - **Trajectory Following**: [`PurePursuit`] and [`RamseteController`] for path tracking
//! - **PID Control**: [`Pid`] and [`AngularPid`] for closed-loop control
//! - **Feedforward**: [`FeedForward`] and [`ArmFeedForward`] for model-based control
//! - **Type-Safe Units**: [`QLength`], [`QAngle`], [`QTime`] prevent unit errors
//!
//! ## Quick Start
//!
//! ```ignore
//! use kernelvex::*;
//!
//! // Create a differential drivetrain
//! let left_motors = MotorGroup::new(vec![left1, left2]);
//! let right_motors = MotorGroup::new(vec![right1, right2]);
//! let drivetrain = DifferentialDrive::new(left_motors, right_motors, track_width);
//!
//! // Set up odometry
//! let rig = TrackingRig::new(
//!     Pose::default(),
//!     [horizontal_wheel],
//!     [left_wheel, right_wheel],
//!     Some(imu),
//! );
//!
//! // Create an odometry chassis for autonomous
//! let chassis = OdomChassis::new(drivetrain, rig, linear_pid, angular_pid);
//!
//! // Drive to a point
//! chassis.shoot(target_pose, constraints).await?;
//! ```
//!
//! ## Module Overview
//!
//! | Module | Description |
//! |--------|-------------|
//! | [`control`] | PID controllers, feedforward, RAMSETE, pure pursuit |
//! | [`dt`] | Drivetrain models and motor groups |
//! | [`motion`] | Motion profiles and trajectories |
//! | [`odom`] | Odometry, pose estimation, tracking wheels |
//! | [`util`] | Type-safe units, logging, solenoid groups |


pub use odom::wheel::{OmniWheel, TrackingWheel, TrackingRig};
pub use odom::chassis::{
    DriveError, OdomChassis,
};

pub use control::ramsete::{RamseteController, RamseteReference};
pub use util::controller::*;
pub use motion::profile::TrapezoidalConstraints;
pub use motion::trajectory::{Trajectory, TrajectoryPoint};
pub use odom::{pose::Pose, wheel::*};

pub use util::si::{QAngle, QLength, QTime};

pub use dt::model::*;
pub use util::solenoidgroup::SolenoidGroup;

pub use dt::differential::DifferentialDrive;
pub use dt::motorgroup::MotorGroup;

pub mod control;
pub mod dt;

pub use util::logger::Logger;
pub use util::{si::*, utils::*};

pub mod odom;

pub mod motion;

pub use control::pid::{AngularPid, Pid};

pub use control::purepursuit::PurePursuit;
pub mod util;

pub use control::feedforward::*;

pub use dt::differential::*;
