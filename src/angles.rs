// effectively the measurement implementation with extra uses

mod angles {
    #[allow(dead_code)]
    pub trait Unit {
        const FACTOR: f32;
        const NAME: &'static str;
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Degrees;

    impl Unit for Degrees {
        const FACTOR: f32 = core::f32::consts::PI/180.;
        const NAME: &'static str = "deg";
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Radians;

    impl Unit for Radians {
        const FACTOR: f32 = 1.0;
        const NAME: &'static str = "rad";
    }

    #[derive(Copy, Clone, Debug)]
    pub struct Angle<U: Unit> {
        _si: f32,
        _unit: core::marker::PhantomData<U>,
    }


    impl Into<Angle<Radians>> for f32 {
        fn into(self) -> Angle<Radians> {
            Angle::<Radians> {_si: self, _unit: core::marker::PhantomData}
        }
    }

    impl Into<Angle<Radians>> for Angle<Degrees> {
        fn into(self) -> Angle<Radians> {
            Angle::<Radians> {_si: self._si, _unit: core::marker::PhantomData}
        }
    }

    impl<U: Unit> Angle<U> {
        #[allow(dead_code)]
        pub fn new(v: f32) -> Self {
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
        pub fn value(&self) -> f32 {
            self._si / U::FACTOR
        }

        #[allow(dead_code)]
        #[inline]
        pub fn normalize(&self) -> Self {
            self.fmod(Angle {_si: core::f32::consts::PI, _unit: core::marker::PhantomData})
        }

        #[allow(dead_code)]
        #[inline]
        pub fn abs(&self) -> Self {
            Self {
                _si: libm::fabsf(self._si),
                _unit: core::marker::PhantomData,
            }
        }


        #[allow(dead_code)]
        #[inline]
        pub fn fmod(&self, other: Self) -> Self {
            Self {
                _si: libm::fmodf(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn remainder(&self, other: Self) -> Self {
            Self {
                _si: libm::remainderf(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn copysign(&self, other: Self) -> Self {
            Self {
                _si: libm::copysignf(self._si, other._si),
                _unit: core::marker::PhantomData,
            }
        }

        #[allow(dead_code)]
        #[inline]
        pub fn cos(&self) -> f32 {
            libm::cosf(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn sin(&self) -> f32 {
            libm::sinf(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn tan(&self) -> f32 {
            libm::tanf(self._si)
        }

        #[allow(dead_code)]
        #[inline]
        pub fn approx_eq<T: Unit>(&self, other: Angle<T>, tolerance: Option<f32>) -> bool {
            let eps = tolerance.unwrap_or(f32::EPSILON);
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

    impl<U: Unit> core::ops::Mul<f32> for Angle<U> {
        type Output = Angle<U>;

        fn mul(self, rhs: f32) -> Self::Output {
            Self::Output {
                _si: self._si * rhs,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit> core::ops::Div<f32> for Angle<U> {
        type Output = Angle<U>;

        fn div(self, rhs: f32) -> Self::Output {
            Self::Output {
                _si: self._si / rhs,
                _unit: core::marker::PhantomData,
            }
        }
    }

    impl<U: Unit, T: Unit> core::ops::Div<Angle<T>> for Angle<U> {
        type Output = f32;


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