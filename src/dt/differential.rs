use crate::dt::model::{Arcade, CurvatureDrive, Drivetrain, Tank};
use crate::util::utils::GroupErrors;
use crate::{MotorGroup, Vector2};

/// A differential (tank-style) drivetrain with left and right motor groups.
///
/// `DifferentialDrive` controls odom robot with independent left and right sides,
/// supporting tank, arcade, and curvature drive modes.
///
/// # Type Parameters
///
/// * `N` - The number of motors per side
pub struct DifferentialDrive {
    left: MotorGroup,
    right: MotorGroup,
    expo: ExpoDrive
}

pub struct ExpoDrive {
    n: f64,
    k: f64,
}

impl ExpoDrive {
    pub fn new(n: f64, k: f64) -> Self {
        Self { n, k }
    }

    pub fn calculate(&self, x: f64, y: f64) -> Vector2<f64> {
        {
            assert!(-1.0 <= x && x <= 1.0, "x must be between [-1, 1]");
        }

        {
            assert!(-1.0 <= y && x <= 1.0, "y must be between [-1, 1]");
        }

        let m = libm::pow(libm::pow(libm::fabs(x), self.n+2.) + libm::pow(libm::fabs(x), self.n+2.), 1./self.n+2.);
        let f = libm::pow(m, self.k)/libm::sqrt(x*x + y*y);

        Vector2::<f64>::new(f*x, f*y)
    }
}

impl DifferentialDrive {
    #[inline]
    pub fn new(left: MotorGroup, right: MotorGroup, expo: ExpoDrive) -> Self {
        Self { left, right, expo }
    }
}

impl Arcade for DifferentialDrive {
    async fn drive_arcade(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        let maximum = libm::fmax(libm::fabs(left), libm::fabs(right));
        let (x, y) = self.expo.calculate(left, right).as_tuple();
        let total = x + y;
        let difference = x - y;

        if x >= 0. {
            if y >= 0. {
                self.left.set_voltage(maximum).await?;
                self.right.set_voltage(difference).await?;
            } else {
                self.left.set_voltage(total).await?;
                self.right.set_voltage(maximum).await?;
            }
        } else if y >= 0. {
            self.left.set_voltage(total).await?;
            self.right.set_voltage(-maximum).await?;
        } else {
            self.left.set_voltage(-maximum).await?;
            self.right.set_voltage(difference).await?;
        }
        Ok(())
    }
}

impl Tank for DifferentialDrive {
    async fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        let (x, y) = self.expo.calculate(left, right).as_tuple();
        self.left.set_voltage(x * 12.0).await?;
        self.right.set_voltage(y * 12.0).await?;

        Ok(())
    }
}

impl CurvatureDrive for DifferentialDrive {
    async fn drive_curvature(&mut self, left: f64, right: f64) -> Result<(), GroupErrors> {
        todo!()
    }
}

impl Drivetrain for DifferentialDrive {}
