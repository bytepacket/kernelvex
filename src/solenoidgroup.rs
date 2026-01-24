#![allow(dead_code)]

use std::cell::RefCell;
use std::rc::Rc;
use vexide_devices::adi::digital::*;
use crate::utils::GroupErrors;

pub struct SolenoidGroup {
    pneumatics: Rc<RefCell<Vec<AdiDigitalOut>>>,
}

impl SolenoidGroup {
    pub fn new(pneumatics: Vec<AdiDigitalOut>) -> Self {
        SolenoidGroup {
            pneumatics: Rc::new(RefCell::new(pneumatics)),
        }
    }

    pub fn get(&self, index: usize) -> std::cell::Ref<'_, AdiDigitalOut> {
        std::cell::Ref::map(self.pneumatics.borrow(), |t| &t[index])
    }

    pub fn set_level(&self, logic: LogicLevel) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self.pneumatics
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, sol)| sol.set_level(logic).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn extend(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self.pneumatics
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, sol)| sol.set_level(LogicLevel::High).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn retract(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self.pneumatics
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, sol)| sol.set_level(LogicLevel::Low).err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

    pub fn toggle(&self) -> Result<(), GroupErrors> {
        let ret: GroupErrors = self.pneumatics
            .borrow_mut()
            .iter_mut()
            .enumerate()
            .filter_map(|(_, sol)| sol.toggle().err())
            .collect();
        if ret.is_empty() {
            Ok(())
        }
        else {
            Err(ret)
        }
    }

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
