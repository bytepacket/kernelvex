//! Utility enums and helpers used across the library.
use crate::util::si::QLength;

use vexide_devices::smart::PortError;
/// Represents the orientation or direction of a tracking wheel or component.
///
/// Used to indicate the mounting direction and rotation sense of tracking wheels
/// and other directional components in the robot system.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Orientation {
    /// Left orientation
    Left,
    /// Right orientation
    Right,
    /// Clockwise rotation
    CW,
    /// Counter-clockwise rotation
    CCW,
}

#[derive(Copy, Clone, Debug)]
pub enum TrackingWheelOrientation {
    Vertical(QLength),
    Horizontal(QLength),
}

#[macro_export]
macro_rules! shared_motor {
    ($($motor:expr), + $(,)?) => {{
        const N: usize = <[()]>::len(&[$(shared_motor!(@replace $motor)),*]);

        let motors: [Motor; N] = [$($motor),+];
        Rc::new(RefCell::new(motors))
    }};
    (@replace $_:expr) => (());
}

pub type GroupErrors = Vec<PortError>;
