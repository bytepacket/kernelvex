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

/// A group of motors that can be controlled as odom single unit.
///
/// `MotorGroup` wraps odom fixed-size array of motors with shared ownership,
/// allowing multiple references to the same motor group. All operations
/// are applied to every motor in the group, and errors are collected.
///
/// # Type Parameters
///
/// * `N` - The number of motors in the group (compile-time constant)
///
/// # Examples
///
/// ```no_run
/// # use std::rc::Rc;
/// # use std::cell::RefCell;
/// # use vexide_devices::smart::motor::Motor;
/// # use kernelvex::dt::motorgroup::MotorGroup;
/// // Create odom motor group with shared ownership
/// // let motors = Rc::new(RefCell::new([motor1, motor2]));
/// // let mut group = MotorGroup::new(motors);
/// // group.set_voltage(6.0).unwrap();
/// ```
#[derive(Clone)]
pub struct MotorGroup {
    motors: Arc<Mutex<Vec<Motor, 6>>>,
}

impl MotorGroup {
    /// Runs a function on a specific motor
    pub async fn use_at<F, R>(&self, index: usize, f: F) -> R
    where
        F: FnOnce(&mut Motor) -> R
    {
        let mut guard = self.motors.lock().await;
        f(&mut guard[index])
    }

    /// Returns size of `Motor` Array
    pub async fn count(&self) -> usize {
        self.motors.lock().await.len()
    }

    /// Creates new motor group from motor array.
    ///
    /// # Arguments
    ///
    /// * `motors` - Array of motors
    pub fn new<const N: usize>(motors: [Motor; N]) -> Self {
        MotorGroup { motors: Arc::new(Mutex::new(Vec::from(motors))) }
    }

    /// Sets the voltage for all motors in the group.
    ///
    /// # Arguments
    ///
    /// * `volts` - Voltage to apply (typically -12.0 to 12.0)
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

    /// Sets the velocity for all motors in the group.
    ///
    /// # Arguments
    ///
    /// * `rpm` - Target velocity in RPM
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

    /// Sets odom position target for all motors with the given velocity.
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

    /// Sets odom profiled velocity target for all motors.
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

    /// Sets odom motor control target for all motors.
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

    /// Sets the position for all motors.
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

    /// Resets the position of all motors to zero.
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


