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