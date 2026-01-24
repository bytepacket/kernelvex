//! # Kernelvex
//!
//! ## Modules
//!
//! - [`wheel`] - Tracking wheel implementation for odometry
//! - [`pid`] - PID controller for closed-loop control
//! - [`pose`] - 2D pose representation and transformations
//! - [`sensors`] - Encoder trait for sensor abstraction
//! - [`si`] - Type-safe unit system (length, angle, time)
//! - [`utils`] - Utility types and helpers

pub mod wheel;

pub use crate::wheel::{OmniWheel, TrackingWheel};

pub mod pid;

pub use crate::pid::Pid;

pub mod pose;

pub use crate::pose::Pose;

pub mod sensors;

pub mod si;

pub use crate::si::{QAngle, QLength, QTime};

pub mod utils;
mod solenoidgroup;

pub use crate::solenoidgroup::SolenoidGroup;
mod differential;
mod motorgroup;

pub use crate::motorgroup::MotorGroup;
