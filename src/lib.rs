//! # Kernelvex
//!
//! A VEX robotics control library providing high-level abstractions for
//! robot control systems including motor control, odometry, PID controllers,
//! and drive mechanisms.
//!
//! ## Modules
//!
//! - [`wheel`] - Tracking wheel implementation for odometry
//! - [`pid`] - PID controller for closed-loop control
//! - [`pose`] - 2D pose representation and transformations
//! - [`sensors`] - Encoder trait for sensor abstraction
//! - [`si`] - Type-safe unit system (length, angle, time)
//! - [`utils`] - Utility types and helpers
//! - [`motorgroup`] - Motor group abstraction
//! - [`differential`] - Differential drive implementation
//! - [`solenoidgroup`] - Pneumatic solenoid group control
//! - [`model`] - Drive model traits (Tank, Arcade, CurvatureDrive)
//! - [`odom`] - Odometry chassis for position tracking

pub mod wheel;
pub use crate::wheel::{OmniWheel, TrackingWheel};

pub mod pid;
pub use crate::pid::Pid;

pub mod pose;
pub use crate::pose::Pose;

pub mod sensors;
pub use crate::sensors::Encoder;

pub mod si;
pub use crate::si::{QAngle, QLength, QTime};

pub mod utils;

pub mod motorgroup;
pub use crate::motorgroup::MotorGroup;

pub mod differential;
pub use crate::differential::DifferentialDrive;

pub mod solenoidgroup;
pub use crate::solenoidgroup::SolenoidGroup;

pub mod model;
pub use crate::model::*;

pub mod odom;
pub use crate::odom::OdomChassis;
