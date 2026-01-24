/// Represents the orientation or direction of a tracking wheel or component.
///
/// Used to indicate the mounting direction and rotation sense of tracking wheels
/// and other directional components in the robot system.

use vexide_devices::smart::PortError;
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

#[macro_export]
macro_rules! verify {
    ($cond:expr) => {{
        if $cond {
            Ok(true)
        } else {
            Err(false)
        }
    }};

    ($cond:expr, $err:expr) => {{
        if $cond {
            Ok(true)
        } else {
            Err($err)
        }
    }};

    ($cond:expr, $suc:expr, $err:expr) => {{
        if $cond {
            Ok($suc)
        } else {
            Err($err)
        }
    }};
}

pub type GroupErrors = Vec<PortError>;
