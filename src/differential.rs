use crate::model::{Arcade, Tank, CurvatureDrive, Drivetrain};
use crate::MotorGroup;
use crate::utils::GroupErrors;

/// A differential (tank-style) drivetrain with left and right motor groups.
///
/// `DifferentialDrive` controls a robot with independent left and right sides,
/// supporting tank, arcade, and curvature drive modes.
///
/// # Type Parameters
///
/// * `N` - The number of motors per side
pub struct DifferentialDrive<const N: usize> {
    left: MotorGroup<N>,
    right: MotorGroup<N>,
}


impl<const N: usize> DifferentialDrive<N> {
    #[inline]
    pub fn new(left: MotorGroup<N>, right: MotorGroup<N>) -> Self {
        Self { left, right }
    }
}

impl<const N: usize> Arcade for DifferentialDrive<N> {
    fn drive_arcade(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        let maximum = libm::fmax(libm::fabs(left), libm::fabs(right));
        let total = left + right;
        let difference = left - right;

        if left >= 0. {
            if right >= 0. {
                self.left.set_voltage(maximum)?;
                self.right.set_voltage(difference)?;
            } else {
                self.left.set_voltage(total)?;
                self.right.set_voltage(maximum)?;
            }
        }
        else if right >= 0. {
                  self.left.set_voltage(total)?;
                  self.right.set_voltage(-maximum)?;
              }
              else {
                  self.left.set_voltage(-maximum)?;
                  self.right.set_voltage(difference)?;
              }
        Ok(())
        }
    }

impl<const N: usize> Tank for DifferentialDrive<N> {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        self.left.set_voltage(left * 12.0)?;
        self.right.set_voltage(right * 12.0)?;

        Ok(())
    }
}

impl<const N: usize> CurvatureDrive for DifferentialDrive<N> {
    fn drive_curvature(&mut self, throttle: f64, curvature: f64) -> Result<(), GroupErrors> {
        let left = (throttle + (curvature * throttle.abs())).clamp(-12.0, 12.0);
        let right = (throttle - (curvature * throttle.abs())).clamp(-12.0, 12.0);

        self.left.set_voltage(left)?;
        self.right.set_voltage(right)?;

        Ok(())
    }
}

impl<const N: usize> Drivetrain for DifferentialDrive<N> {}