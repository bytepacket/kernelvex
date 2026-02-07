use crate::utils::GroupErrors;

/// Trait for tank drive control.
///
/// Tank drive allows independent control of left and right sides,
/// providing direct voltage or speed control to each side.
pub trait Tank {
    /// Drive the robot using tank controls.
    ///
    /// # Arguments
    ///
    /// * `left` - Left side power (-1.0 to 1.0)
    /// * `right` - Right side power (-1.0 to 1.0)
    fn drive_tank(&mut self, left: f64, right: f64) -> Result<(), GroupErrors>;
}

/// Trait for arcade drive control.
///
/// Arcade drive uses a forward/backward axis and a turn axis,
/// mixing them to produce left and right motor outputs.
pub trait Arcade {
    /// Drive the robot using arcade controls.
    ///
    /// # Arguments
    ///
    /// * `forward` - Forward/backward power (-1.0 to 1.0)
    /// * `turn` - Turn power (-1.0 to 1.0)
    fn drive_arcade(&mut self, forward: f64, turn: f64) -> Result<(), GroupErrors>;
}

/// Trait for curvature drive control.
///
/// Curvature drive scales the turn rate by the throttle, making it
/// easier to drive in a straight line at high speeds while still
/// allowing tight turns at low speeds.
pub trait CurvatureDrive {
    /// Drive the robot using curvature controls.
    ///
    /// # Arguments
    ///
    /// * `throttle` - Forward/backward power (-1.0 to 1.0)
    /// * `curvature` - Curvature of the path (-1.0 to 1.0)
    fn drive_curvature(&mut self, throttle: f64, curvature: f64) -> Result<(), GroupErrors>;
}

/// Marker trait for drivetrain implementations.
///
/// Types implementing this trait can be used as drivetrains in
/// higher-level control systems like odometry.
pub trait Drivetrain {}