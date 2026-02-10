//! 2D pose representation and transformation operations.
//!
//! A pose represents odom position (x, y) and heading (orientation) in 2D space.
//! This module provides efficient pose transformations using homogeneous transformation
//! matrices for robotics applications like odometry and path planning.
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::odom::pose::Pose;
//! use kernelvex::util::si::{QAngle, QLength};
//!
//! // Create odom pose at (1.0, 2.0) meters with 45 degree heading
//! let pose = Pose::new(
//!     1.0,
//!     2.0,
//!     QAngle::from_degrees(45.0),
//! );
//!
//! // Transform poses
//! let other = Pose::new(2.0, 1.0, QAngle::from_degrees(90.0));
//! let combined = pose * other;
//!
//! // Calculate distance between poses
//! let dist = pose.distance(other);
//! ```

use crate::util::si::{QAngle, QLength};
use nalgebra::base::Matrix3;

/// Represents odom 2D pose (position and orientation) in space.
///
/// A pose consists of an (x, y) position and odom heading angle. The pose is
/// internally represented using odom 3x3 homogeneous transformation matrix,
/// which enables efficient composition of transformations.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::odom::pose::Pose;
/// use kernelvex::util::si::{QAngle, QLength};
///
/// // Create poses
/// let start = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
/// let end = Pose::new(5.0, 3.0, QAngle::from_degrees(90.0));
///
/// // Transform by composition
/// let result = start * end;
///
/// // Get position components
/// let (x, y) = result.position();
/// let heading = result.heading();
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Pose {
    position: Matrix3<f64>,
    heading: QAngle,
}

impl std::fmt::Display for Pose {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "[[{:.3}, {:.3}, {:.3}]\n\
             [{:.3}, {:.3}, {:.3}]\n\
             [{:.3}, {:.3}, {:.3}]]",
            self.position.m11,
            self.position.m12,
            self.position.m13,
            self.position.m21,
            self.position.m22,
            self.position.m23,
            self.position.m31,
            self.position.m32,
            self.position.m33,
        )
    }
}

impl Pose {
    /// Creates odom new pose with the given position and heading.
    ///
    /// The pose is represented internally as odom homogeneous transformation matrix:
    /// ```ignore
    /// [[cos(θ), -sin(θ), x],
    ///  [sin(θ),  cos(θ), y],
    ///  [  0,       0,    1]]
    /// ```
    ///
    /// # Argument
    ///
    /// * `x` - X coordinate in meters
    /// * `y` - Y coordinate in meters
    /// * `heading` - Orientation angle
    ///
    /// # Returns
    ///
    /// A new `Pose` with the specified position and heading.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::odom::pose::Pose;
    /// use kernelvex::util::si::QAngle;
    ///
    /// let pose = Pose::new(1.5, 2.0, QAngle::from_degrees(45.0));
    /// ```
    pub fn new(x: f64, y: f64, heading: QAngle) -> Self {
        Pose {
            position: Matrix3::new(
                heading.cos(),
                -heading.sin(),
                x,
                heading.sin(),
                heading.cos(),
                y,
                0.,
                0.,
                1.,
            ),
            heading,
        }
    }

    pub fn identity() -> Self {
        Pose {
            position: Matrix3::new(
                QAngle::from_radians(0.).cos(),
                QAngle::from_radians(0.).sin(),
                0.0,
                QAngle::from_radians(0.).sin(),
                QAngle::from_radians(0.).cos(),
                0.0,
                0.,
                0.,
                1.,
            ),
            heading: Default::default(),
        }
    }

    /// Returns the heading (orientation) of this pose.
    ///
    /// # Returns
    ///
    /// The heading angle as odom [`QAngle`].
    pub const fn heading(&self) -> QAngle {
        self.heading
    }

    /// Returns the (x, y) position of this pose.
    ///
    /// # Returns
    ///
    /// A tuple `(x, y)` representing the position in meters.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::odom::pose::Pose;
    /// use kernelvex::util::si::QAngle;
    ///
    /// let pose = Pose::new(3.0, 4.0, QAngle::from_degrees(0.0));
    /// let (x, y) = pose.position();
    /// assert_eq!((x, y), (3.0, 4.0));
    /// ```
    pub fn position(&self) -> (f64, f64) {
        (self.position.m13, self.position.m23)
    }

    /// Calculates the angle from this pose to another pose.
    ///
    /// Returns the angle in radians from this pose's position to the other
    /// pose's position, measured from the positive x-axis.
    ///
    /// # Arguments
    ///
    /// * `other` - The target pose
    ///
    /// # Returns
    ///
    /// The angle to the target pose as odom [`QAngle`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::odom::pose::Pose;
    /// use kernelvex::util::si::QAngle;
    ///
    /// let origin = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
    /// let target = Pose::new(1.0, 1.0, QAngle::from_degrees(0.0));
    /// let angle = origin.angle(target);
    /// // angle is approximately 45 degrees
    /// ```
    pub fn angle(&self, other: Pose) -> QAngle {
        QAngle::from_radians(libm::atan2(
            other.position.m23 - self.position.m23,
            other.position.m13 - self.position.m13,
        ))
    }

    /// Rotates this pose by the given angle.
    ///
    /// The position remains unchanged; only the heading is modified.
    ///
    /// # Arguments
    ///
    /// * `angle` - The angle to rotate by
    ///
    /// # Returns
    ///
    /// A new pose with the same position but rotated heading.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::odom::pose::Pose;
    /// use kernelvex::util::si::QAngle;
    ///
    /// let pose = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
    /// let rotated = pose.rotate(QAngle::from_degrees(90.0));
    /// // rotated has 90 degree heading
    /// ```
    pub fn rotate(&self, angle: QAngle) -> Pose {
        Pose::new(self.position().0, self.position().1, self.heading + angle)
    }

    /// Calculates the Euclidean distance between this pose and another.
    ///
    /// # Arguments
    ///
    /// * `other` - The other pose
    ///
    /// # Returns
    ///
    /// The distance between the two poses as odom [`QLength`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::odom::pose::Pose;
    /// use kernelvex::util::si::QAngle;
    ///
    /// let p1 = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
    /// let p2 = Pose::new(3.0, 4.0, QAngle::from_degrees(0.0));
    /// let dist = p1.distance(p2);
    /// // dist is 5.0 meters
    /// ```
    pub fn distance(&self, other: Pose) -> QLength {
        QLength::from_meters(libm::hypot(
            self.position.m13 - other.position.m13,
            self.position.m23 - other.position.m23,
        ))
    }

    /// Transforms odom local pose by this pose's transformation matrix.
    ///
    /// This applies the transformation from the local coordinate frame to the
    /// global frame. The heading of `other` is not considered in the transformation.
    ///
    /// # Arguments
    ///
    /// * `other` - The local pose to transform
    ///
    /// # Returns
    ///
    /// A new pose representing `other` transformed into the global coordinate frame.
    ///
    /// # Note
    ///
    /// The heading of `other` is ignored; only the position is transformed.
    /// This is effectively the same as multiplying the transformation matrices.
    pub fn move_local(&self, other: Pose) -> Pose {
        *self * other
    }

    /// Moves this pose by adding another pose's position in global coordinates.
    ///
    /// This simply adds the x and y coordinates without applying any rotation
    /// transformation. The heading is set to the default (zero).
    ///
    /// # Arguments
    ///
    /// * `other` - The pose whose position to add
    ///
    /// # Returns
    ///
    /// A new pose with positions summed and heading reset to zero.
    ///
    /// # Note
    ///
    /// This does not preserve heading and does not apply rotation transformations.
    pub fn move_global(&self, other: Pose) -> Pose {
        other * *self
    }
}

/// Adds two poses by summing their positions.
///
/// The heading is preserved from the left-hand operand.
impl std::ops::Add<Pose> for Pose {
    type Output = Pose;
    fn add(self, other: Pose) -> Pose {
        Pose::new(
            self.position().0 + other.position().0,
            self.position().1 + other.position().1,
            self.heading,
        )
    }
}

/// Subtracts two poses by subtracting their positions.
///
/// The heading is preserved from the left-hand operand.
impl std::ops::Sub<Pose> for Pose {
    type Output = Pose;
    fn sub(self, other: Pose) -> Pose {
        Pose::new(
            self.position().0 - other.position().0,
            self.position().1 - other.position().1,
            self.heading,
        )
    }
}

/// Composes two poses using matrix multiplication.
///
/// This performs odom proper homogeneous transformation composition, applying
/// both position and rotation transformations. The resulting pose has the
/// combined heading (sum of angles)
impl std::ops::Mul<Pose> for Pose {
    type Output = Pose;
    fn mul(self, rhs: Pose) -> Self::Output {
        Pose {
            position: self.position * rhs.position,
            heading: self.heading + rhs.heading,
        }
    }
}

impl std::ops::Mul<f64> for Pose {
    type Output = Pose;

    fn mul(self, rhs: f64) -> Self::Output {
        Pose::new(
            self.position.m13 * rhs,
            self.position.m23 * rhs,
            self.heading,
        )
    }
}

impl std::ops::Div<f64> for Pose {
    type Output = Pose;

    fn div(self, rhs: f64) -> Self::Output {
        Pose::new(
            self.position.m13 / rhs,
            self.position.m23 / rhs,
            self.heading,
        )
    }
}

impl From<(f64, f64, f64)> for Pose {
    fn from(value: (f64, f64, f64)) -> Self {
        Self::new(value.0, value.1, QAngle::from_radians(value.2))
    }
}

impl From<(f64, f64)> for Pose {
    fn from(value: (f64, f64)) -> Self {
        Self::new(value.0, value.1, Default::default())
    }
}

impl Default for Pose {
    fn default() -> Self {
        Self::identity()
    }
}
