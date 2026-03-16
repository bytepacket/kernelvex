//! Utility enums and helpers used across the library.
//!
//! This module provides common types and macros used throughout kernelvex:
//!
//! - [`Orientation`]: Left/right or CW/CCW direction indicators
//! - [`TrackingWheelOrientation`]: Vertical or horizontal wheel mounting with offset
//! - [`shared_motor!`]: Macro for creating shared motor arrays
//! - [`GroupErrors`]: Type alias for collections of port errors
use crate::util::si::QLength;

use vexide_devices::smart::PortError;
/// Represents the orientation or direction of a tracking wheel or component.
///
/// This enum is used in two contexts:
///
/// 1. **Spatial orientation**: `Left` and `Right` indicate which side of the
///    robot center a component is mounted on (determined by offset sign).
///
/// 2. **Rotational direction**: `CW` (clockwise) and `CCW` (counter-clockwise)
///    indicate rotation sense for turning operations.
///
/// # Example
///
/// ```
/// use kernelvex::util::utils::Orientation;
///
/// let wheel_side = Orientation::Left;
/// let turn_direction = Orientation::CW;
/// ```
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// Left side of the robot (negative offset from center)
    Left,
    /// Right side of the robot (positive offset from center)
    Right,
    /// Clockwise rotation
    CW,
    /// Counter-clockwise rotation
    CCW,
}

/// Specifies the mounting orientation of a tracking wheel with its offset distance.
///
/// Tracking wheels can be mounted in two orientations:
///
/// - **Vertical**: Measures forward/backward movement. The offset is the
///   perpendicular distance from the robot's center to the wheel (positive = right).
///
/// - **Horizontal**: Measures lateral (sideways) movement. The offset is the
///   distance from the robot's center along the forward axis (positive = forward).
///
/// # Example
///
/// ```
/// use kernelvex::util::utils::TrackingWheelOrientation;
/// use kernelvex::util::si::QLength;
///
/// // Vertical wheel 5 inches to the right of center
/// let right_wheel = TrackingWheelOrientation::Vertical(QLength::from_inches(5.0));
///
/// // Horizontal wheel 3 inches behind center
/// let back_wheel = TrackingWheelOrientation::Horizontal(QLength::from_inches(-3.0));
/// ```
#[derive(Copy, Clone, Debug)]
pub enum TrackingWheelOrientation {
    /// Vertical (forward-facing) wheel with perpendicular offset from center
    Vertical(QLength),
    /// Horizontal (sideways-facing) wheel with forward offset from center
    Horizontal(QLength),
}

/// Creates a shared motor array wrapped in `Rc<RefCell<[Motor; N]>>`.
///
/// This macro simplifies creating motor arrays that can be shared between
/// multiple components (e.g., drivetrain and autonomous routines) while
/// allowing interior mutability.
///
/// # Arguments
///
/// Accepts one or more `Motor` instances, separated by commas.
///
/// # Returns
///
/// `Rc<RefCell<[Motor; N]>>` where `N` is the number of motors provided.
///
/// # Example
///
/// ```ignore
/// use kernelvex::shared_motor;
/// use vexide_devices::smart::motor::Motor;
///
/// let motors = shared_motor!(
///     Motor::new(port1, Gearset::Green, Direction::Forward),
///     Motor::new(port2, Gearset::Green, Direction::Forward),
/// );
/// ```
#[macro_export]
macro_rules! shared_motor {
    ($($motor:expr), + $(,)?) => {{
        const N: usize = <[()]>::len(&[$(shared_motor!(@replace $motor)),*]);

        let motors: [Motor; N] = [$($motor),+];
        Rc::new(RefCell::new(motors))
    }};
    (@replace $_:expr) => (());
}

/// A collection of port errors from group operations.
///
/// When performing operations on motor groups or solenoid groups, individual
/// devices may fail while others succeed. This type collects all errors that
/// occurred during a group operation.
///
/// # Example
///
/// ```ignore
/// use kernelvex::util::utils::GroupErrors;
///
/// async fn operate_group(group: &SolenoidGroup) -> Result<(), GroupErrors> {
///     group.extend().await
/// }
/// ```
pub type GroupErrors = Vec<PortError>;
