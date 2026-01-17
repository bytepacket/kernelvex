//! # Kernelvex
//!
//! ## Modules
//!
//! - [`omniwheel`] - Tracking wheel implementation for odometry
//! - [`pid`] - PID controller for closed-loop control
//! - [`pose`] - 2D pose representation and transformations
//! - [`sensors`] - Encoder trait for sensor abstraction
//! - [`si`] - Type-safe unit system (length, angle, time)
//! - [`utils`] - Utility types and helpers

pub mod omniwheel;

pub use crate::omniwheel::{OmniWheel, TrackingWheel};

pub mod pid;

pub use crate::pid::Pid;

pub mod pose;

pub use crate::pose::Pose;

pub mod sensors;

pub mod si;

pub use crate::si::{QAngle, QLength, QTime};

pub mod utils;
