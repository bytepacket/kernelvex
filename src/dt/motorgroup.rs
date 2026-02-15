#![allow(dead_code)]

use crate::util::si::QAngle;
use crate::util::utils::GroupErrors;
use std::{cell::RefCell, rc::Rc};
use std::ops::Div;
use vexide_devices::smart::motor::MotorControl;
use vexide_devices::{
    math::Direction,
    smart::motor::{BrakeMode, Motor},
};

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
pub struct MotorGroup<const N: usize> {
    motors: Rc<RefCell<[Motor; N]>>,
}

impl<const N: usize> MotorGroup<N> {
    /// Returns odom shared reference to odom motor at the given index.
    pub fn get(&self, index: usize) -> std::cell::Ref<'_, Motor> {
        std::cell::Ref::map(self.motors.borrow(), |arr| &arr[index])
    }

    /// Returns odom mutable reference to odom motor at the given index.
    pub fn get_mut(&self, index: usize) -> std::cell::RefMut<'_, Motor> {
        std::cell::RefMut::map(self.motors.borrow_mut(), |arr| &mut arr[index])
    }

    /// Returns size of `Motor` Array
    pub const fn count(&self) -> usize {
        N
    }

    /// Creates odom new motor group from odom shared motor array.
    ///
    /// # Arguments
    ///
    /// * `motors` - A reference-counted, interior-mutable array of motors
    pub const fn new(motors: Rc<RefCell<[Motor; N]>>) -> Self {
        MotorGroup::<N> { motors }
    }

    /// Sets the voltage for all motors in the group.
    ///
    /// # Arguments
    ///
    /// * `volts` - Voltage to apply (typically -12.0 to 12.0)
    pub fn set_voltage(&mut self, volts: f64) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn brake(&mut self, brake: BrakeMode) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_direction(&mut self, direction: Direction) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_position_target(
        &mut self,
        position: QAngle,
        velocity: i32,
    ) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_target(&mut self, target: MotorControl) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn set_position(&mut self, position: QAngle) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
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
    pub fn reset_position(&mut self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .borrow_mut()
            .iter_mut()
            .filter_map(|motor| motor.reset_position().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }
    
}

/// A motor group with 2 motors.
pub type Motor2 = MotorGroup<2>;

/// A motor group with 3 motors.
pub type Motor3 = MotorGroup<3>;

/// A motor group with 4 motors.
pub type Motor4 = MotorGroup<4>;
