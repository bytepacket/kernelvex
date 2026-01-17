// effectively the measurement implementation with extra uses

mod angles {
    #[allow(dead_code)]
    pub trait Unit {
        const FACTOR: f64;
        const NAME: &'static str;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Degrees;

    impl Unit for Degrees {
        const FACTOR: f64 = core::f64::consts::PI / 180.;
        const NAME: &'static str = "deg";
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Radians;

    impl Unit for Radians {
        const FACTOR: f64 = 1.0;
        const NAME: &'static str = "rad";
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Angle<U: Unit> {
        _si: f64,
        _unit: core::marker::PhantomData<U>,
    }

    impl Into<Angle<Radians>> for f64 {
        fn into(self) -> Angle<Radians> {
            Angle::<Radians> {
                _si: self,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl Into<Angle<Radians>> for Angle<Degrees> {
        fn into(self) -> Angle<Radians> {
            Angle::<Radians> {
                _si: self._si,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit> Angle<U> {
        #[allow(dead_code)]
        pub fn new(v: f64) -> Self {
            Self {
                _si: v * U::FACTOR,
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        pub fn to<T: Unit>(self) -> Angle<T> {
            Angle {
                _si: self._si,
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn value(&self) -> f64 {
            self._si / U::FACTOR
        }

        #[allow(dead_code)]
        #[inline]
        pub fn normalize(&self) -> Self {
            self.fmod(Angle {
                _si: core::f64::consts::PI,
                _unit: core::marker::PhantomData,
            })
        }

        #[allow(dead_code)]
        #[inline]
        pub fn abs(&self) -> Self {
            Self {
                _si: libm::fabs(self._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn fmod(&self, other: Self) -> Self {
            Self {
                _si: libm::fmod(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn remainder(&self, other: Self) -> Self {
            Self {
                _si: libm::remainder(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn copysign(&self, other: Self) -> Self {
            Self {
                _si: libm::copysign(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn cos(&self) -> f64 {
            libm::cos(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn sin(&self) -> f64 {
            libm::sin(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn tan(&self) -> f64 {
            libm::tan(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn approx_eq<T: Unit>(&self, other: Angle<T>, tolerance: Option<f64>) -> bool {
            let eps = tolerance.unwrap_or(f64::EPSILON);
            (self._si - other._si).abs() < eps
        }
    }
    impl<U: Unit, T: Unit> core::cmp::PartialEq<Angle<T>> for Angle<U> {
        fn eq(&self, other: &Angle<T>) -> bool {
            self._si == other._si
        }
    }

    impl<U: Unit> core::fmt::Display for Angle<U> {
        fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
            write!(f, "{} {}", self.value(), U::NAME)
        }
    }

    impl<U: Unit> core::ops::Mul<f64> for Angle<U> {
        type Output = Angle<U>;

        fn mul(self, rhs: f64) -> Self::Output {
            Self::Output {
                _si: self._si * rhs,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit> core::ops::Div<f64> for Angle<U> {
        type Output = Angle<U>;

        fn div(self, rhs: f64) -> Self::Output {
            Self::Output {
                _si: self._si / rhs,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit, T: Unit> core::ops::Div<Angle<T>> for Angle<U> {
        type Output = f64;

        fn div(self, rhs: Angle<T>) -> Self::Output {
            self._si / rhs._si
        }
    }

    impl<U: Unit, T: Unit> core::ops::AddAssign<Angle<T>> for Angle<U> {
        fn add_assign(&mut self, rhs: Angle<T>) {
            *self = Self {
                _si: self._si + rhs._si,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit, T: Unit> core::ops::Add<Angle<T>> for Angle<U> {
        type Output = Angle<U>;

        fn add(self, rhs: Angle<T>) -> Self::Output {
            Self::Output {
                _si: self._si + rhs._si,
                _unit: core::marker::PhantomData,
            }
        }
    }
}

pub use angles::*;
