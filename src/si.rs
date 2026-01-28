//! Type-safe unit system for robotics calculations.
//!
//! This module provides compile-time type checking for physical quantities
//! (length, angle, time) to prevent unit errors in calculations. The system
//! is based on the Okapi library design for VEX robotics.
//!
//! # Design Philosophy
//!
//! Inspired by the C++11 `std::ratio` approach to dimensional analysis at compile-time.
//! See: <https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/>
//!
//! # Type Parameters
//!
//! The `RQuantity` type uses integer type-level numbers to represent dimensions:
//! - `L` - Length dimension (P1 = meters, Z0 = dimensionless)
//! - `T` - Time dimension (P1 = seconds, Z0 = dimensionless)
//! - `A` - Angle dimension (P1 = radians, Z0 = dimensionless)
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::si::{QLength, QAngle};
//!
//! // Type-safe unit conversions
//! let length = QLength::from_meters(1.5);
//! let inches = length.as_inches();
//!
//! // Compile-time checked operations
//! let dist1 = QLength::from_inches(12.0);
//! let dist2 = QLength::from_centimeters(30.0);
//! let total = dist1 + dist2; // OK: both are lengths
//! // let invalid = dist1 + QAngle::from_degrees(45.0); // Compile error!
//! ```

// TODO: implement time

use typenum::{Diff, Integer, Negate, Sum, P1, Z0};
use vexide_devices::math::Angle;

/// A typed quantity with compile-time checked dimensions.
///
/// `RQuantity` represents a physical quantity with type-safe dimensions.
/// The type parameters `L`, `T`, and `A` are integer types representing
/// the length, time, and angle dimensions respectively.
///
/// # Type Safety
///
/// Operations between quantities are only allowed when dimensions match
/// (for addition/subtraction) or when dimensions are compatible (for
/// multiplication/division). This prevents common unit errors at compile-time.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::si::QLength;
///
/// let a = QLength::from_meters(1.0);
/// let b = QLength::from_inches(12.0);
/// let sum = a + b; // Type-safe addition
/// ```
#[derive(Debug, Clone, Copy)]
pub struct RQuantity<L, T, A>
where
    L: Integer,
    T: Integer,
    A: Integer,
{
    value: f64,
    _phantom: std::marker::PhantomData<(L, T, A)>,
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Add for RQuantity<L, T, A> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value + rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Sub for RQuantity<L, T, A> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Mul<f64> for RQuantity<L, T, A> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value * rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Mul<RQuantity<L, T, A>> for f64 {
    type Output = RQuantity<L, T, A>;

    fn mul(self, rhs: RQuantity<L, T, A>) -> Self::Output {
        Self::Output {
            value: self * rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L, T, A, L2, T2, A2> std::ops::Mul<RQuantity<L2, T2, A2>> for RQuantity<L, T, A>
where
    L: Integer + std::ops::Add<L2>,
    T: Integer + std::ops::Add<T2>,
    A: Integer + std::ops::Add<A2>,
    L2: Integer,
    T2: Integer,
    A2: Integer,
    Sum<L, L2>: Integer,
    Sum<T, T2>: Integer,
    Sum<A, A2>: Integer,
{
    type Output = RQuantity<Sum<L, L2>, Sum<T, T2>, Sum<A, A2>>;

    fn mul(self, rhs: RQuantity<L2, T2, A2>) -> Self::Output {
        Self::Output {
            value: self.value * rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Div<f64> for RQuantity<L, T, A> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value / rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L, T, A> std::ops::Div<RQuantity<L, T, A>> for f64
where
    L: Integer + std::ops::Neg,
    T: Integer + std::ops::Neg,
    A: Integer + std::ops::Neg,

    Negate<L>: Integer,
    Negate<T>: Integer,
    Negate<A>: Integer,
{
    type Output = RQuantity<Negate<L>, Negate<T>, Negate<A>>;

    fn div(self, rhs: RQuantity<L, T, A>) -> Self::Output {
        Self::Output {
            value: self / rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L, T, A, L2, T2, A2> std::ops::Div<RQuantity<L2, T2, A2>> for RQuantity<L, T, A>
where
    L: Integer + std::ops::Sub<L2>,
    T: Integer + std::ops::Sub<T2>,
    A: Integer + std::ops::Sub<A2>,
    L2: Integer,
    T2: Integer,
    A2: Integer,
    Diff<L, L2>: Integer,
    Diff<T, T2>: Integer,
    Diff<A, A2>: Integer,
{
    type Output = RQuantity<Diff<L, L2>, Diff<T, T2>, Diff<A, A2>>;

    fn div(self, rhs: RQuantity<L2, T2, A2>) -> Self::Output {
        Self::Output {
            value: self.value / rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::AddAssign<RQuantity<L, T, A>>
    for RQuantity<L, T, A>
{
    fn add_assign(&mut self, rhs: RQuantity<L, T, A>) {
        *self = Self {
            value: self.value + rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::SubAssign<RQuantity<L, T, A>>
    for RQuantity<L, T, A>
{
    fn sub_assign(&mut self, rhs: RQuantity<L, T, A>) {
        *self = Self {
            value: self.value - rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::MulAssign<f64> for RQuantity<L, T, A> {
    fn mul_assign(&mut self, rhs: f64) {
        *self = Self {
            value: self.value * rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::DivAssign<f64> for RQuantity<L, T, A> {
    fn div_assign(&mut self, rhs: f64) {
        *self = Self {
            value: self.value / rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> std::ops::Neg for RQuantity<L, T, A> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::Output {
            value: -self.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> PartialEq for RQuantity<L, T, A> {
    fn eq(&self, other: &Self) -> bool {
        (self.value - other.value).abs() < f64::EPSILON
    }
}

impl<L: Integer, T: Integer, A: Integer> Default for RQuantity<L, T, A> {
    fn default() -> Self {
        Self {
            value: 0.,
            _phantom: std::marker::PhantomData,
        }
    }
}
/// A dimensionless number (no units).
///
/// This is useful for ratios and dimensionless constants.
#[allow(dead_code)]
pub type QNumber = RQuantity<Z0, Z0, Z0>;

/// A length quantity (dimension: length^1).
///
/// Represents distances and lengths with compile-time unit checking.
/// Can be created from meters, centimeters, or inches and converted
/// between these units.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::si::QLength;
///
/// let meters = QLength::from_meters(2.5);
/// let inches = meters.as_inches();
/// ```
#[allow(dead_code)]
pub type QLength = RQuantity<P1, Z0, Z0>;

/// A time quantity (dimension: time^1).
///
/// Represents durations and time intervals.
///
/// # Note
///
/// Time operations are not yet fully implemented.
#[allow(dead_code)]
pub type QTime = RQuantity<Z0, P1, Z0>;

/// An angle quantity (dimension: angle^1).
///
/// Represents angles and rotations with compile-time unit checking.
/// Can be created from radians, degrees, or turns and converted between
/// these units. Also provides trigonometric functions.
///
/// # Examples
///
/// ```no_run
/// use kernelvex::si::QAngle;
///
/// let degrees = QAngle::from_degrees(45.0);
/// let radians = degrees.as_radians();
/// let sine = degrees.sin();
/// ```
#[allow(dead_code)]
pub type QAngle = RQuantity<Z0, Z0, P1>;

impl QLength {
    /// Creates a length from a value in meters.
    ///
    /// # Arguments
    ///
    /// * `m` - Length in meters
    ///
    /// # Returns
    ///
    /// A `QLength` representing the given distance.
    #[allow(dead_code)]
    #[inline]
    pub const fn from_meters(m: f64) -> Self {
        Self {
            value: m,
            _phantom: std::marker::PhantomData,
        }
    }

    /// Creates a length from a value in centimeters.
    ///
    /// # Arguments
    ///
    /// * `cm` - Length in centimeters
    ///
    /// # Returns
    ///
    /// A `QLength` representing the given distance.
    #[allow(dead_code)]
    #[inline]
    pub const fn from_centimeters(cm: f64) -> Self {
        Self::from_meters(cm / 100.)
    }

    /// Creates a length from a value in inches.
    ///
    /// # Arguments
    ///
    /// * `ic` - Length in inches
    ///
    /// # Returns
    ///
    /// A `QLength` representing the given distance.
    #[allow(dead_code)]
    #[inline]
    pub const fn from_inches(ic: f64) -> Self {
        Self::from_centimeters(ic * 2.54)
    }

    /// Converts this length to meters.
    ///
    /// # Returns
    ///
    /// The length value in meters as an `f64`.
    #[inline]
    pub const fn as_meters(&self) -> f64 {
        self.value
    }

    /// Converts this length to centimeters.
    ///
    /// # Returns
    ///
    /// The length value in centimeters as an `f64`.
    #[inline]
    pub const fn as_centimeters(&self) -> f64 {
        self.value * 100.
    }

    /// Converts this length to inches.
    ///
    /// # Returns
    ///
    /// The length value in inches as an `f64`.
    #[inline]
    pub const fn as_inches(&self) -> f64 {
        self.value * 39.3701
    }
}

impl QTime {
    const SECOND: QTime = Self::from_sec(1.);

    const MILLISECOND: QTime = Self::from_msec(1.);

    const MINUTE: QTime = Self::from_minute(1.);

    #[allow(dead_code)]
    #[inline]
    pub const fn from_sec(s: f64) -> Self {
        Self {
            value: s,
            _phantom: std::marker::PhantomData,
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub const fn from_msec(ms: f64) -> Self {
        Self::from_sec(ms / 1000.)
    }

    #[allow(dead_code)]
    #[inline]
    pub const fn from_minute(s: f64) -> Self {
        Self::from_sec(s * 60.)
    }

    #[inline]
    pub const fn as_sec(&self) -> f64 {
        self.value
    }

    #[inline]
    pub const fn as_msec(&self) -> f64 {
        self.value * 1000.
    }

    #[inline]
    pub const fn as_minute(&self) -> f64 {
        self.value / 60.
    }
}

impl From<std::time::Duration> for QTime {
    fn from(value: std::time::Duration) -> Self {
        QTime::from_sec(value.as_secs_f64())
    }
}

impl QAngle {
    /// Creates an angle from a value in radians.
    ///
    /// # Arguments
    ///
    /// * `rad` - Angle in radians
    ///
    /// # Returns
    ///
    /// A `QAngle` representing the given angle.
    #[allow(dead_code)]
    #[inline]
    pub const fn from_radians(rad: f64) -> Self {
        Self {
            value: rad,
            _phantom: std::marker::PhantomData,
        }
    }

    /// Creates an angle from a value in degrees.
    ///
    /// # Arguments
    ///
    /// * `deg` - Angle in degrees
    ///
    /// # Returns
    ///
    /// A `QAngle` representing the given angle.
    #[inline]
    pub const fn from_degrees(deg: f64) -> Self {
        Self::from_radians(deg.to_radians())
    }

    /// Converts this angle to radians.
    ///
    /// # Returns
    ///
    /// The angle value in radians as an `f64`.
    #[inline]
    pub const fn as_radians(&self) -> f64 {
        self.value
    }

    /// Converts this angle to degrees.
    ///
    /// # Returns
    ///
    /// The angle value in degrees as an `f64`.
    #[inline]
    pub fn as_degrees(&self) -> f64 {
        // Use standard library function for better precision and consistency
        // Note: Not const because to_degrees() is not const in stable Rust
        self.value.to_degrees()
    }

    /// Creates an angle from a value in turns (revolutions).
    ///
    /// One turn equals 2π radians (360 degrees).
    ///
    /// # Arguments
    ///
    /// * `turns` - Angle in turns/revolutions
    ///
    /// # Returns
    ///
    /// A `QAngle` representing the given angle.
    #[inline]
    pub const fn from_turns(turns: f64) -> Self {
        Self::from_radians(turns * 2.0 * std::f64::consts::PI)
    }

    #[inline]
    pub fn asin(v: f64) -> Self {
        Self::from_radians(libm::asin(v))
    }

    #[inline]
    pub fn acos(v: f64) -> Self {
        Self::from_radians(libm::acos(v))
    }

    #[inline]
    pub fn atan(v: f64) -> Self {
        Self::from_radians(libm::atan(v))
    }

    #[inline]
    pub fn asinh(v: f64) -> Self {
        Self::from_radians(libm::asinh(v))
    }

    #[inline]
    pub fn acosh(v: f64) -> Self {
        Self::from_radians(libm::acosh(v))
    }

    #[inline]
    pub fn atanh(v: f64) -> Self {
        Self::from_radians(libm::atanh(v))
    }

    #[inline]
    pub fn atan2(y: f64, x: f64) -> Self {
        Self::from_radians(libm::atan2(y, x))
    }

    /// Converts this angle to turns (revolutions).
    ///
    /// # Returns
    ///
    /// The angle value in turns as an `f64`.
    #[inline]
    pub const fn as_turns(&self) -> f64 {
        self.value / (2.0 * std::f64::consts::PI)
    }

    /// Returns the absolute value of this angle.
    ///
    /// # Returns
    ///
    /// A new `QAngle` with the absolute value.
    #[allow(dead_code)]
    #[inline]
    pub fn abs(&self) -> Self {
        Self {
            value: libm::fabs(self.value),
            _phantom: std::marker::PhantomData,
        }
    }

    /// Calculates the sine of this angle.
    ///
    /// # Returns
    ///
    /// The sine value as an `f64` in the range [-1.0, 1.0].
    #[allow(dead_code)]
    #[inline]
    pub fn sin(&self) -> f64 {
        libm::sin(self.value)
    }

    /// Calculates the cosine of this angle.
    ///
    /// # Returns
    ///
    /// The cosine value as an `f64` in the range [-1.0, 1.0].
    #[allow(dead_code)]
    #[inline]
    pub fn cos(&self) -> f64 {
        libm::cos(self.value)
    }

    /// Calculates the tangent of this angle.
    ///
    /// # Returns
    ///
    /// The tangent value as an `f64`.
    #[allow(dead_code)]
    #[inline]
    pub fn tan(&self) -> f64 {
        libm::tan(self.value)
    }

    /// Calculates the hyperbolic sine of this angle.
    ///
    /// # Returns
    ///
    /// The hyperbolic sine value as an `f64`.
    #[allow(dead_code)]
    #[inline]
    pub fn sinh(&self) -> f64 {
        libm::sinh(self.value)
    }

    /// Calculates the hyperbolic cosine of this angle.
    ///
    /// # Returns
    ///
    /// The hyperbolic cosine value as an `f64`.
    #[allow(dead_code)]
    #[inline]
    pub fn cosh(&self) -> f64 {
        libm::cosh(self.value)
    }

    /// Calculates the hyperbolic tangent of this angle.
    ///
    /// # Returns
    ///
    /// The hyperbolic tangent value as an `f64`.
    #[allow(dead_code)]
    #[inline]
    pub fn tanh(&self) -> f64 {
        libm::tanh(self.value)
    }

    /// Calculates the floating-point remainder of dividing this angle by another.
    ///
    /// This is similar to the modulo operation but handles floating-point values.
    ///
    /// # Arguments
    ///
    /// * `other` - The divisor angle
    ///
    /// # Returns
    ///
    /// The remainder as a new `QAngle`.
    #[allow(dead_code)]
    #[inline]
    pub fn fmod(&self, other: Self) -> Self {
        Self {
            value: libm::fmod(self.value, other.value),
            _phantom: std::marker::PhantomData,
        }
    }

    /// Calculates the IEEE 754 floating-point remainder.
    ///
    /// This is more accurate than `fmod` for certain calculations.
    ///
    /// # Arguments
    ///
    /// * `other` - The divisor angle
    ///
    /// # Returns
    ///
    /// The remainder as a new `QAngle`.
    #[allow(dead_code)]
    #[inline]
    pub fn remainder(&self, other: Self) -> Self {
        Self {
            value: libm::remainder(self.value, other.value),
            _phantom: std::marker::PhantomData,
        }
    }

    /// Copies the sign from another angle to this angle.
    ///
    /// Returns a new angle with the magnitude of `self` and the sign of `other`.
    ///
    /// # Arguments
    ///
    /// * `other` - The angle from which to copy the sign
    ///
    /// # Returns
    ///
    /// A new `QAngle` with `self`'s magnitude and `other`'s sign.
    #[allow(dead_code)]
    #[inline]
    pub fn copysign(&self, other: Self) -> Self {
        Self {
            value: libm::copysign(self.value, other.value),
            _phantom: std::marker::PhantomData,
        }
    }
}

/// Conversion from Vexide `Angle` type to `QAngle`.
///
/// This allows seamless integration with Vexide's angle types.
impl From<Angle> for QAngle {
    fn from(value: Angle) -> Self {
        Self::from_radians(value.as_radians())
    }
}

/// Conversion from `QAngle` to Vexide `Angle` type.
///
/// This allows seamless integration with Vexide's angle types.
impl From<QAngle> for Angle {
    fn from(value: QAngle) -> Self {
        Angle::from_radians(value.as_radians())
    }
}

impl From<f64> for QAngle {
    fn from(value: f64) -> Self {
        Self::from_radians(value)
    }
}
