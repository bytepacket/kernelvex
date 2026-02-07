#![allow(dead_code)]

//! Solenoid group abstraction for controlling multiple pneumatic actuators.

use crate::utils::GroupErrors;
use std::cell::RefCell;
use std::rc::Rc;
use vexide_devices::adi::digital::*;

/// A group of pneumatic solenoids that can be controlled as a single unit.
///
/// `SolenoidGroup` wraps a vector of ADI digital outputs with shared
/// ownership, allowing multiple references and unified control of
/// pneumatic actuators.
pub struct SolenoidGroup {
    pneumatics: Rc<RefCell<Vec<AdiDigitalOut>>>,
}

impl SolenoidGroup {
    /// Creates a new solenoid group from a vector of digital outputs.
    pub fn new(pneumatics: Vec<AdiDigitalOut>) -> Self {
        SolenoidGroup {
            pneumatics: Rc::new(RefCell::new(pneumatics)),
        }
    }

    /// Returns a shared reference to a solenoid at the given index.
    pub fn get(&self, index: usize) -> std::cell::Ref<'_, AdiDigitalOut> {
        std::cell::Ref::map(self.pneumatics.borrow(), |t| &t[index])
    }

    /// Sets the logic level for all solenoids.
    pub fn set_level(&self, logic: LogicLevel) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .pneumatics
            .borrow_mut()
            .iter_mut()
            .filter_map(|sol| sol.set_level(logic).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Extends all solenoids (sets to high).
    pub fn extend(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .pneumatics
            .borrow_mut()
            .iter_mut()
            .filter_map(|sol| sol.set_level(LogicLevel::High).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Retracts all solenoids (sets to low).
    pub fn retract(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .pneumatics
            .borrow_mut()
            .iter_mut()
            .filter_map(|sol| sol.set_level(LogicLevel::Low).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Toggles all solenoids.
    pub fn toggle(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self
            .pneumatics
            .borrow_mut()
            .iter_mut()
            .filter_map(|sol| sol.toggle().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        } else {
            Err(ret)
        }
    }

    /// Checks if all solenoids are at the given logic level.
    pub fn is_level(&self, logic: LogicLevel) -> bool {
        for sol in self.pneumatics.borrow().iter() {
            match sol.level().unwrap() == logic {
                true => (),
                false => return false,
            }
        }
        true
    }
}
