#![allow(dead_code)]


use std::{cell::RefCell, rc::Rc};
use vexide_devices::{
    math::Direction,
    smart::motor::{BrakeMode, Motor},
};
use vexide_devices::smart::motor::MotorControl;
use crate::si::QAngle;

use crate::utils::GroupErrors;

pub struct MotorGroup<const N: usize> {
    motors: Rc<RefCell<[Motor; N]>>,
}

impl<const N: usize> MotorGroup<N> {
    pub fn new(motors: Rc<RefCell<[Motor; N]>>) -> Self {
        Self { motors }
    }

    pub fn get(&self, index: usize) -> std::cell::Ref<'_, Motor> {
        std::cell::Ref::map(self.motors.borrow(), |arr| &arr[index])
    }

    pub fn get_mut(&self, index: usize) -> std::cell::RefMut<'_, Motor> {
        std::cell::RefMut::map(self.motors.borrow_mut(), |arr| &mut arr[index])
    }
}

pub struct Motor2 {
    motors: MotorGroup<2>,
}

impl Motor2 {
    pub fn new(motor: Rc<RefCell<[Motor; 2]>>) -> Self {
        Motor2 {
            motors: MotorGroup::new(motor),
        }
    }

    pub fn set_voltage(&mut self, volts: f64) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_voltage(volts).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_velocity(rpm).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn brake(&mut self, brake: BrakeMode) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.brake(brake).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_direction(&mut self, direction: Direction) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_direction(direction).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position_target(&mut self, position: QAngle, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position_target(position.into(), velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_profiled_velocity(velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_target(&mut self, target: MotorControl) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_target(target).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position(&mut self, position: QAngle) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position(position.into()).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn reset_position(&mut self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.reset_position().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }
}

pub struct Motor3 {
    motors: MotorGroup<3>,
}

impl Motor3 {
    pub fn new(motor: Rc<RefCell<[Motor; 3]>>) -> Self {
        Motor3 {
            motors: MotorGroup::new(motor),
        }
    }

    pub fn set_voltage(&mut self, volts: f64) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_voltage(volts).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_velocity(rpm).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn brake(&mut self, brake: BrakeMode) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.brake(brake).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_direction(&mut self, direction: Direction) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_direction(direction).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position_target(&mut self, position: QAngle, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position_target(position.into(), velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_profiled_velocity(velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_target(&mut self, target: MotorControl) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_target(target).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position(&mut self, position: QAngle) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position(position.into()).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn reset_position(&mut self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.reset_position().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }    
}

pub struct Motor4 {
    motors: MotorGroup<4>,
}

impl Motor4 {
    pub fn new(motor: Rc<RefCell<[Motor; 4]>>) -> Self {
        Motor4 {
            motors: MotorGroup::new(motor),
        }
    }

    pub fn set_voltage(&mut self, volts: f64) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_voltage(volts).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_velocity(&mut self, rpm: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_velocity(rpm).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn brake(&mut self, brake: BrakeMode) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.brake(brake).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_direction(&mut self, direction: Direction) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_direction(direction).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position_target(&mut self, position: QAngle, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position_target(position.into(), velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_profiled_velocity(&mut self, velocity: i32) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_profiled_velocity(velocity).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_target(&mut self, target: MotorControl) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_target(target).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn set_position(&mut self, position: QAngle) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.set_position(position.into()).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn reset_position(&mut self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .motors
            .motors
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, motor)| motor.reset_position().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }
}
