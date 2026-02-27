#![allow(dead_code)]

use crate::util::utils::GroupErrors;
use std::sync::Arc;
use vexide_async::sync::Mutex;
use vexide_devices::adi::digital::*;

#[derive(Clone)]
pub struct SolenoidGroup {
    pneumatics: Arc<Mutex<Vec<AdiDigitalOut>>>,
}

impl SolenoidGroup {
    pub fn new(pneumatics: Vec<AdiDigitalOut>) -> Self {
        SolenoidGroup {
            pneumatics: Arc::new(Mutex::new(pneumatics)),
        }
    }

    /// Runs a function on a specific solenoid
    pub async fn use_at<F, R>(&self, index: usize, f: F) -> R
    where
        F: FnOnce(&mut AdiDigitalOut) -> R
    {
        let mut guard = self.pneumatics.lock().await;
        f(&mut guard[index])
    }

    /// Sets the logic level for all solenoids.
    pub async fn set_level(&self, logic: LogicLevel) -> Result<(), GroupErrors> {
        let mut guard = self.pneumatics.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|sol| sol.set_level(logic).err())
            .collect();

        if ret.is_empty() { Ok(()) } else { Err(ret) }
    }

    /// Extends all solenoids (sets to high).
    pub async fn extend(&self) -> Result<(), GroupErrors> {
        self.set_level(LogicLevel::High).await
    }

    /// Retracts all solenoids (sets to low).
    pub async fn retract(&self) -> Result<(), GroupErrors> {
        self.set_level(LogicLevel::Low).await
    }

    /// Toggles all solenoids.
    pub async fn toggle(&self) -> Result<(), GroupErrors> {
        let mut guard = self.pneumatics.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|sol| sol.toggle().err())
            .collect();

        if ret.is_empty() { Ok(()) } else { Err(ret) }
    }

    /// Checks if all solenoids are at the given logic level.
    pub async fn is_level(&self, logic: LogicLevel) -> bool {
        let guard = self.pneumatics.lock().await;
        guard.iter().all(|sol| sol.level().unwrap_or(!logic) == logic)
    }
}