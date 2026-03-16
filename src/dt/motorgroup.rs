//! Motor group for controlling multiple motors as a single unit.
//!
//! This module provides [`MotorGroup`], which wraps multiple motors and applies
//! operations to all of them simultaneously. This is essential for drivetrains
//! where multiple motors drive the same side.
//!
//! # Thread Safety
//!
//! `MotorGroup` uses `Arc<Mutex<...>>` for shared ownership, allowing multiple
//! references to the same motor group across async tasks.
//!
//! # Error Handling
//!
//! Operations return `Result<(), GroupErrors>` where `GroupErrors` is a collection
//! of individual motor port errors. If any motor fails, the errors are collected
//! but the operation continues for the remaining motors.
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::MotorGroup;
//! use vexide_devices::smart::motor::Motor;
//!
//! // Create a motor group with 2 motors
//! let mut group = MotorGroup::new([motor1, motor2]);
//!
//! // Set voltage for all motors
//! group.set_voltage(6.0).await?;
//!
//! // Get average velocity
//! let rpm = group.velocity().await?;
//! ```

#![allow(dead_code)]
// TODO: remove results with warns (no errors)

use crate::util::si::QAngle;
use crate::util::utils::GroupErrors;
use std::sync::Arc;
use vexide_async::sync::Mutex;
use vexide_devices::smart::motor::MotorControl;
use vexide_devices::{
    math::Direction,
    smart::motor::{BrakeMode, Motor},
};

use heapless::Vec;

/// A group of motors that can be controlled as a single unit.
///
/// `MotorGroup` wraps a fixed-size array of motors with shared ownership,
/// allowing multiple references to the same motor group. All operations
/// are applied to every motor in the group, and errors are collected.
///
/// # Capacity
///
/// The internal storage supports up to 6 motors per group, which covers
/// most VEX robotics applications.
///
/// # Example
///
/// ```no_run
/// let mut group = MotorGroup::new([motor1, motor2, motor3]);
///
/// // All motors receive 8V
/// group.set_voltage(8.0).await?;
///
/// // Get average velocity across all motors
/// let avg_rpm = group.velocity().await?;
/// ```
#[derive(Clone)]
pub struct MotorGroup {
    /// Thread-safe storage for up to 6 motors.
    motors: Arc<Mutex<Vec<Motor, 6>>>,
}

impl MotorGroup {
    /// Runs a function on a specific motor by index.
    ///
    /// # Arguments
    ///
    /// * `index` - The motor index (0-based)
    /// * `f` - Function to execute on the motor
    ///
    /// # Panics
    ///
    /// Panics if `index` is out of bounds.
    pub async fn use_at<F, R>(&self, index: usize, f: F) -> R
    where
        F: FnOnce(&mut Motor) -> R
    {
        let mut guard = self.motors.lock().await;
        f(&mut guard[index])
    }

    /// Returns the number of motors in the group.
    pub async fn count(&self) -> usize {
        self.motors.lock().await.len()
    }

    /// Creates a new motor group from an array of motors.
    ///
    /// # Arguments
    ///
    /// * `motors` - Array of motors (up to 6)
    ///
    /// # Type Parameters
    ///
    /// * `N` - Number of motors in the array (compile-time constant)
    ///
    /// # Example
    ///
    /// ```no_run
    /// let group = MotorGroup::new([motor1, motor2]);
    /// ```
    pub fn new<const N: usize>(motors: [Motor; N]) -> Self {
        MotorGroup { motors: Arc::new(Mutex::new(Vec::from(motors))) }
    }

    /// Sets the voltage for all motors in the group.
    ///
    /// # Arguments
    ///
    /// * `volts` - Voltage to apply (typically -12.0 to 12.0 for V5 motors)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_voltage(&mut self, volts: f64) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_voltage(volts).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets the velocity target for all motors in the group.
    ///
    /// Uses the motor's internal velocity PID controller.
    ///
    /// # Arguments
    ///
    /// * `rpm` - Target velocity in RPM
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_velocity(&mut self, rpm: i32) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_velocity(rpm).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Applies the given brake mode to all motors.
    ///
    /// # Arguments
    ///
    /// * `brake` - The brake mode (Coast, Brake, or Hold)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn brake(&mut self, brake: BrakeMode) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.brake(brake).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets the direction for all motors in the group.
    ///
    /// # Arguments
    ///
    /// * `direction` - Forward or Reverse
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_direction(&mut self, direction: Direction) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_direction(direction).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets a position target for all motors with the given velocity.
    ///
    /// Uses the motor's internal position PID controller.
    ///
    /// # Arguments
    ///
    /// * `position` - Target position as an angle
    /// * `velocity` - Maximum velocity in RPM
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_position_target(
        &mut self,
        position: QAngle,
        velocity: i32,
    ) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_position_target(position.into(), velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets a profiled velocity target for all motors.
    ///
    /// Uses the motor's internal motion profiling.
    ///
    /// # Arguments
    ///
    /// * `velocity` - Target velocity in RPM
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_profiled_velocity(velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets a motor control target for all motors.
    ///
    /// # Arguments
    ///
    /// * `target` - The motor control target (voltage, velocity, or position)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_target(&mut self, target: MotorControl) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_target(target).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Sets the encoder position for all motors.
    ///
    /// # Arguments
    ///
    /// * `position` - The position to set as an angle
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors set successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn set_position(&mut self, position: QAngle) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.set_position(position.into()).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Resets the encoder position of all motors to zero.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All motors reset successfully
    /// * `Err(GroupErrors)` - One or more motors failed
    pub async fn reset_position(&mut self) -> Result<(), GroupErrors> {
        let mut guard = self.motors.lock().await;
        
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|motor| motor.reset_position().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Returns the average velocity of all motors in the group.
    ///
    /// # Returns
    ///
    /// * `Ok(i32)` - Average velocity in RPM
    /// * `Err(GroupErrors)` - One or more motors failed to read
    pub async fn velocity(&self) -> Result<i32, GroupErrors> {
        let guard = self.motors.lock().await;
        let mut errors = GroupErrors::new();
        let mut total_rpm = 0i32;
        
        for motor in guard.iter() {
            match motor.velocity() {
                Ok(rpm) => total_rpm += rpm as i32,
                Err(e) => errors.push(e),
            }
        }
        
        if errors.is_empty() {
            Ok(total_rpm / guard.len() as i32)
        } else {
            Err(errors)
        }
    }
    
}


