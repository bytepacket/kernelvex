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

pub use odom::wheel::{OmniWheel, TrackingWheel, TrackingRig};
pub use odom::chassis::{OdomChassis, TurnError, ShootError};

pub use control::ramsete::{RamseteController, RamseteReference};
pub use motion::trajectory::{Trajectory, TrajectoryPoint};
pub use odom::pose::Pose;

pub use util::si::{QAngle, QLength, QTime};

pub use dt::model::*;
pub use util::solenoidgroup::SolenoidGroup;

pub use dt::differential::DifferentialDrive;
pub use dt::motorgroup::MotorGroup;
use odom::{pose, wheel};

pub mod control;
pub mod dt;

pub use util::logger::Logger;
use util::{si, utils};

pub mod odom;

pub mod motion;

pub use control::pid::{AngularPid, Pid};

pub use control::purepursuit::PurePursuit;
pub mod util;

pub use util::si::*;

pub use control::feedforward::*;