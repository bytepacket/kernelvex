//! 2D pose representation and transformation operations.
//!
//! A pose represents a position (x, y) and heading (orientation) in 2D space.
//! This module provides efficient pose transformations using homogeneous transformation
//! matrices for robotics applications like odometry and path planning.
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::pose::Pose;
//! use kernelvex::si::{QAngle, QLength};
//!
//! // Create a pose at (1.0, 2.0) meters with 45 degree heading
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
//! let dist = pose.distance(&other);
//! ```

use crate::si::{QAngle, QLength};
use nalgebra::base::Matrix3;

/// Represents a 2D pose (position and orientation) in space.
///
/// A pose consists of an (x, y) position and a heading angle. The pose is
/// internally represented using a 3x3 homogeneous transformation matrix,
/// which enables efficient composition of transformations.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::pose::Pose;
/// use kernelvex::si::{QAngle, QLength};
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

impl core::fmt::Display for Pose {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
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
    /// Creates a new pose with the given position and heading.
    ///
    /// The pose is represented internally as a homogeneous transformation matrix:
    /// ```
    /// [[cos(θ), -sin(θ), x],
    ///  [sin(θ),  cos(θ), y],
    ///  [  0,       0,    1]]
    /// ```
    ///
    /// # Arguments
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
    /// use kernelvex::pose::Pose;
    /// use kernelvex::si::QAngle;
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

    /// Returns the heading (orientation) of this pose.
    ///
    /// # Returns
    ///
    /// The heading angle as a [`QAngle`].
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
    /// use kernelvex::pose::Pose;
    /// use kernelvex::si::QAngle;
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
    /// The angle to the target pose as a [`QAngle`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::pose::Pose;
    /// use kernelvex::si::QAngle;
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
    /// use kernelvex::pose::Pose;
    /// use kernelvex::si::QAngle;
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
    /// The distance between the two poses as a [`QLength`].
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use kernelvex::pose::Pose;
    /// use kernelvex::si::QAngle;
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

    /// Transforms a local pose by this pose's transformation matrix.
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
        Pose {
            position: self.position * other.position,
            heading: self.heading,
        }
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
    /// For proper pose composition, use the `*` operator instead.
    pub fn move_global(&self, other: Pose) -> Pose {
        Pose::new(
            self.position.m13 + other.position.m13,
            self.position.m23 + other.position.m23,
            Default::default(),
        )
    }
}

/// Adds two poses by summing their positions.
///
/// The heading is preserved from the left-hand operand.
impl core::ops::Add<Pose> for Pose {
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
impl core::ops::Sub<Pose> for Pose {
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
/// This performs a proper homogeneous transformation composition, applying
/// both position and rotation transformations. The resulting pose has the
/// combined heading (sum of angles)
impl core::ops::Mul<Pose> for Pose {
    type Output = Pose;
    fn mul(self, rhs: Pose) -> Self::Output {
        Pose {
            position: self.position * rhs.position,
            heading: self.heading + rhs.heading,
        }
    }
}

impl core::ops::Mul<f64> for Pose {
    type Output = Pose;

    fn mul(self, rhs: f64) -> Self::Output {
        Pose::new(
            self.position.m13 * rhs,
            self.position.m23 * rhs,
            self.heading,
        )
    }
}

impl core::ops::Div<f64> for Pose {
    type Output = Pose;

    fn div(self, rhs: f64) -> Self::Output {
        Pose::new(
            self.position.m13 / rhs,
            self.position.m23 / rhs,
            self.heading,
        )
    }
}
