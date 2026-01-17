// based on okapi
//
// https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/#the-c11-stdratio-stl-template

// TODO: add cmp functions
// TODO: add math functions for angles and length types
// TODO: add pow functions maybe

use typenum::{Diff, Integer, Negate, Sum, P1, Z0};
use vexide_devices::math::Angle;

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

impl<L: Integer, T: Integer, A: Integer> core::ops::Add for RQuantity<L, T, A> {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value + rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::Sub for RQuantity<L, T, A> {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            value: self.value - rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::Mul<f64> for RQuantity<L, T, A> {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value * rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::Mul<RQuantity<L, T, A>> for f64 {
    type Output = RQuantity<L, T, A>;

    fn mul(self, rhs: RQuantity<L, T, A>) -> Self::Output {
        Self::Output {
            value: self * rhs.value,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L, T, A, L2, T2, A2> core::ops::Mul<RQuantity<L2, T2, A2>> for RQuantity<L, T, A>
where
    L: Integer + core::ops::Add<L2>,
    T: Integer + core::ops::Add<T2>,
    A: Integer + core::ops::Add<A2>,
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
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::Div<f64> for RQuantity<L, T, A> {
    type Output = Self;

    fn div(self, rhs: f64) -> Self::Output {
        Self {
            value: self.value / rhs,
            _phantom: std::marker::PhantomData,
        }
    }
}

impl<L, T, A> core::ops::Div<RQuantity<L, T, A>> for f64
where
    L: Integer + core::ops::Neg,
    T: Integer + core::ops::Neg,
    A: Integer + core::ops::Neg,

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

impl<L, T, A, L2, T2, A2> core::ops::Div<RQuantity<L2, T2, A2>> for RQuantity<L, T, A>
where
    L: Integer + core::ops::Sub<L2>,
    T: Integer + core::ops::Sub<T2>,
    A: Integer + core::ops::Sub<A2>,
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
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::AddAssign<RQuantity<L, T, A>>
    for RQuantity<L, T, A>
{
    fn add_assign(&mut self, rhs: RQuantity<L, T, A>) {
        *self = Self {
            value: self.value + rhs.value,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::SubAssign<RQuantity<L, T, A>>
    for RQuantity<L, T, A>
{
    fn sub_assign(&mut self, rhs: RQuantity<L, T, A>) {
        *self = Self {
            value: self.value - rhs.value,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::MulAssign<f64> for RQuantity<L, T, A> {
    fn mul_assign(&mut self, rhs: f64) {
        *self = Self {
            value: self.value * rhs,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::DivAssign<f64> for RQuantity<L, T, A> {
    fn div_assign(&mut self, rhs: f64) {
        *self = Self {
            value: self.value / rhs,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> core::ops::Neg for RQuantity<L, T, A> {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::Output {
            value: -self.value,
            _phantom: core::marker::PhantomData,
        }
    }
}

impl<L: Integer, T: Integer, A: Integer> PartialEq for RQuantity<L, T, A> {
    fn eq(&self, other: &Self) -> bool {
        self.value == other.value
    }
}

impl <L: Integer, T: Integer, A: Integer> Default for RQuantity<L, T, A> {
    fn default() -> Self {
        Self {
            value: 0.,
            _phantom: core::marker::PhantomData,
        }
    }
}
#[allow(dead_code)]
pub type QNumber = RQuantity<Z0, Z0, Z0>;

#[allow(dead_code)]
pub type QLength = RQuantity<P1, Z0, Z0>;

#[allow(dead_code)]
pub type QTime = RQuantity<Z0, P1, Z0>;

#[allow(dead_code)]
pub type QAngle = RQuantity<Z0, Z0, P1>;

impl QLength {
    #[allow(dead_code)]
    #[inline]
    pub fn from_meters(m: f64) -> Self {
        Self {
            value: m,
            _phantom: std::marker::PhantomData,
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub fn from_centimeters(cm: f64) -> Self {
        Self::from_meters(cm / 100.)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn from_inches(ic: f64) -> Self {
        Self::from_centimeters(ic * 2.54)
    }

    #[inline]
    pub fn as_meters(&self) -> f64 {
        self.value
    }

    #[inline]
    pub fn as_centimeters(&self) -> f64 {
        self.value * 100.
    }

    #[inline]
    pub fn as_inches(&self) -> f64 {
        self.value * 39.3701
    }
}
impl QAngle {
    #[allow(dead_code)]
    #[inline]
    pub fn from_radians(rad: f64) -> Self {
        Self {
            value: rad,
            _phantom: core::marker::PhantomData,
        }
    }

    #[inline]
    pub fn from_degrees(deg: f64) -> Self {
        Self::from_radians(deg.to_radians())
    }

    #[inline]
    pub fn as_radians(&self) -> f64 {
        self.value
    }

    #[inline]
    pub fn as_degrees(&self) -> f64 {
        self.value.to_degrees()
    }

    #[inline]
    pub fn from_turns(turns: f64) -> Self {
        Self::from_radians(turns * 2.0 * std::f64::consts::PI)
    }

    #[inline]
    pub fn as_turns(&self) -> f64 {
        self.value / (2.0 * std::f64::consts::PI)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn abs(&mut self) -> Self {
        Self {
            value: libm::fabs(self.value),
            _phantom: core::marker::PhantomData,
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub fn sin(&self) -> f64 {
        libm::sin(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn cos(&self) -> f64 {
        libm::cos(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn tan(&self) -> f64 {
        libm::tan(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn sinh(&self) -> f64 {
        libm::sinh(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn cosh(&self) -> f64 {
        libm::cosh(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn tanh(&self) -> f64 {
        libm::tanh(self.value)
    }

    #[allow(dead_code)]
    #[inline]
    pub fn fmod(&self, other: Self) -> Self {
        Self {
            value: libm::fmod(self.value, other.value),
            _phantom: core::marker::PhantomData,
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub fn remainder(&self, other: Self) -> Self {
        Self {
            value: libm::remainder(self.value, other.value),
            _phantom: core::marker::PhantomData,
        }
    }

    #[allow(dead_code)]
    #[inline]
    pub fn copysign(&self, other: Self) -> Self {
        Self {
            value: libm::copysign(self.value, other.value),
            _phantom: core::marker::PhantomData,
        }
    }
}

// adding From so vexide_devices::math::Angle -> QAngle

impl From<Angle> for QAngle {
    fn from(value: Angle) -> Self {
        Self::from_radians(value.as_radians())
    }
}

impl From<QAngle> for Angle {
    fn from(value: QAngle) -> Self {
        // vexide angles can be created from radians
        Angle::from_radians(value.as_radians())
    }
}
