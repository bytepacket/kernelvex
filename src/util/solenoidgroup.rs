//! Solenoid group for controlling multiple pneumatic cylinders.
//!
//! This module provides [`SolenoidGroup`] for managing multiple pneumatic
//! solenoids as a single unit, useful for pistons that need to actuate together.
//!
//! # Example
//!
//! ```ignore
//! use kernelvex::util::solenoidgroup::SolenoidGroup;
//! use vexide_devices::adi::digital::AdiDigitalOut;
//!
//! let group = SolenoidGroup::new(vec![solenoid1, solenoid2]);
//!
//! // Extend all pistons
//! group.extend().await?;
//!
//! // Retract all pistons
//! group.retract().await?;
//! ```

#![allow(dead_code)]

use crate::util::utils::GroupErrors;
use std::sync::Arc;
use vexide_async::sync::Mutex;
use vexide_devices::adi::digital::*;

/// A group of pneumatic solenoids that can be controlled together.
///
/// `SolenoidGroup` wraps multiple [`AdiDigitalOut`] devices and provides
/// methods to control them as a single unit. All operations are async and
/// thread-safe using an `Arc<Mutex<>>` wrapper.
///
/// # Use Cases
///
/// - **Dual-acting pistons**: Control both sides of a piston simultaneously
/// - **Synchronized mechanisms**: Ensure multiple pistons extend/retract together
/// - **Redundant systems**: Control backup solenoids alongside primary ones
///
/// # Error Handling
///
/// Group operations return `Result<(), GroupErrors>` where `GroupErrors` is a
/// vector of individual port errors. If all solenoids succeed, `Ok(())` is
/// returned. If any fail, `Err(errors)` contains all failures.
#[derive(Clone)]
pub struct SolenoidGroup {
    pneumatics: Arc<Mutex<Vec<AdiDigitalOut>>>,
}

impl SolenoidGroup {
    /// Creates a new solenoid group from a vector of digital outputs.
    ///
    /// # Arguments
    ///
    /// * `pneumatics` - Vector of [`AdiDigitalOut`] devices to control together
    ///
    /// # Returns
    ///
    /// A new `SolenoidGroup` wrapping the provided solenoids.
    ///
    /// # Example
    ///
    /// ```ignore
    /// let group = SolenoidGroup::new(vec![solenoid1, solenoid2]);
    /// ```
    pub fn new(pneumatics: Vec<AdiDigitalOut>) -> Self {
        SolenoidGroup {
            pneumatics: Arc::new(Mutex::new(pneumatics)),
        }
    }

    /// Runs a function on a specific solenoid by index.
    ///
    /// This method provides direct access to an individual solenoid within
    /// the group for custom operations.
    ///
    /// # Arguments
    ///
    /// * `index` - Index of the solenoid to access (0-based)
    /// * `f` - Closure that receives a mutable reference to the solenoid
    ///
    /// # Returns
    ///
    /// The return value of the provided closure.
    ///
    /// # Panics
    ///
    /// Panics if `index` is out of bounds.
    pub async fn use_at<F, R>(&self, index: usize, f: F) -> R
    where
        F: FnOnce(&mut AdiDigitalOut) -> R
    {
        let mut guard = self.pneumatics.lock().await;
        f(&mut guard[index])
    }

    /// Sets the logic level for all solenoids.
    ///
    /// # Arguments
    ///
    /// * `logic` - The [`LogicLevel`] to set (`High` or `Low`)
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All solenoids set successfully
    /// * `Err(errors)` - One or more solenoids failed; contains all errors
    pub async fn set_level(&self, logic: LogicLevel) -> Result<(), GroupErrors> {
        let mut guard = self.pneumatics.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|sol| sol.set_level(logic).err())
            .collect();

        if ret.is_empty() { Ok(()) } else { Err(ret) }
    }

    /// Extends all solenoids (sets to high).
    ///
    /// Equivalent to `set_level(LogicLevel::High)`. Use this to extend
    /// pneumatic cylinders.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All solenoids extended successfully
    /// * `Err(errors)` - One or more solenoids failed
    pub async fn extend(&self) -> Result<(), GroupErrors> {
        self.set_level(LogicLevel::High).await
    }

    /// Retracts all solenoids (sets to low).
    ///
    /// Equivalent to `set_level(LogicLevel::Low)`. Use this to retract
    /// pneumatic cylinders.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All solenoids retracted successfully
    /// * `Err(errors)` - One or more solenoids failed
    pub async fn retract(&self) -> Result<(), GroupErrors> {
        self.set_level(LogicLevel::Low).await
    }

    /// Toggles all solenoids.
    ///
    /// Each solenoid switches to its opposite state (high → low, low → high).
    /// Note that solenoids toggle independently, so if they were in different
    /// states, they will swap states rather than synchronize.
    ///
    /// # Returns
    ///
    /// * `Ok(())` - All solenoids toggled successfully
    /// * `Err(errors)` - One or more solenoids failed
    pub async fn toggle(&self) -> Result<(), GroupErrors> {
        let mut guard = self.pneumatics.lock().await;
        let ret: GroupErrors = guard
            .iter_mut()
            .filter_map(|sol| sol.toggle().err())
            .collect();

        if ret.is_empty() { Ok(()) } else { Err(ret) }
    }

    /// Checks if all solenoids are at the given logic level.
    ///
    /// # Arguments
    ///
    /// * `logic` - The [`LogicLevel`] to check against
    ///
    /// # Returns
    ///
    /// `true` if all solenoids are at the specified level, `false` otherwise.
    /// If any solenoid's level cannot be read, it is treated as not matching.
    pub async fn is_level(&self, logic: LogicLevel) -> bool {
        let guard = self.pneumatics.lock().await;
        guard.iter().all(|sol| sol.level().unwrap_or(!logic) == logic)
    }
}