use crate::util::utils::GroupErrors;

pub trait Tank {
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), GroupErrors>;
}

pub trait Arcade {
    fn drive_arcade(&mut self, left: f64, right: f64) -> Result<(), GroupErrors>;
}

pub trait CurvatureDrive {
    fn drive_curvature(&mut self, left: f64, right: f64) -> Result<(), GroupErrors>;
}

pub trait Drivetrain {}
