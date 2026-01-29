#![allow(dead_code)]

use crate::si::QAngle;
use std::{cell::RefCell, rc::Rc};
use vexide_devices::smart::motor::{MotorControl};
use vexide_devices::{
    math::Direction,
    smart::motor::{BrakeMode, Motor},
};
use crate::utils::GroupErrors;

#[derive(Clone)]
pub struct MotorGroup<const N: usize> {
    motors: Rc<RefCell<[Motor; N]>>,
}

impl<const N: usize> MotorGroup<N> {
    pub fn get(&self, index: usize) -> std::cell::Ref<'_, Motor> {
        std::cell::Ref::map(self.motors.borrow(), |arr| &arr[index])
    }

    pub fn get_mut(&self, index: usize) -> std::cell::RefMut<'_, Motor> {
        std::cell::RefMut::map(self.motors.borrow_mut(), |arr| &mut arr[index])
    }
    pub fn new(motors: Rc<RefCell<[Motor; N]>>) -> Self {
        MotorGroup::<N> {
            motors
        }
    }

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

pub type Motor2 = MotorGroup<2>;

pub type Motor3 = MotorGroup<3>;

pub type Motor4 = MotorGroup<4>;


