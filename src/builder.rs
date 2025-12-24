#[allow(dead_code)]
pub trait Unit {
    const FACTOR: f32;
    const NAME: &'static str;
}


#[macro_export]
macro_rules! unit {
    (
        $name:ident,

        Types {
            $( $U:ident => $conv:literal ),+ $(,)?
        }
    ) => {
        #[derive(Copy, Clone, Debug)]
        pub struct $name<U> {
            _si: f32,
            _unit: core::marker::PhantomData<U>,
        }

        impl<U: Unit> $name<U> {
            #[allow(dead_code)]
            pub fn new(v: f32) -> Self {
                Self {
                    _si: v * U::FACTOR,
                    _unit: core::marker::PhantomData,
                }
            }

            #[allow(dead_code)]
            pub fn to<T: Unit>(self) -> $name<T> {
                $name {
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
            pub fn abs(&self) -> Self {
                Self{
                    _si: libm::fabsf(self._si),
                    _unit: core::marker::PhantomData,
                }
            }

            #[allow(dead_code)]
            #[inline]
            pub fn ceil(&self) -> Self {
                Self {
                    _si: libm::ceilf(self._si),
                    _unit: core::marker::PhantomData,
                }
            }

            #[allow(dead_code)]
            #[inline]
            pub fn floor(&self) -> Self {
                Self {
                    _si: libm::floorf(self._si),
                    _unit: core::marker::PhantomData,
                }
            }

            #[allow(dead_code)]
            #[inline]
            pub fn sqrt(&self) -> Self {
                Self {
                    _si: libm::sqrtf(self._si),
                    _unit: core::marker::PhantomData,
                }
            }

            /*
            // won't add cbrt, precision is kinda buns
            pub fn cbrt(&self) -> Self {
                Self {
                    _si: self._si.cbrt(),
                    _unit: core::marker::PhantomData,
                }
            }
            */

            #[allow(dead_code)]
            #[inline]
            pub fn hypotf(&self, other: Self) -> Self {
                Self {
                    _si: libm::hypotf(self._si, other._si),
                    _unit: core::marker::PhantomData,
                }
            }

            #[allow(dead_code)]
            #[inline]
            pub fn modf(&self, other: Self) -> Self {
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
            pub fn approx_eq<T: Unit>(&self, other: $name<T>, tolerance: Option<f32>) -> bool {
                let eps = tolerance.unwrap_or(core::f32::EPSILON);
                (self._si - other._si).abs() < eps
            }
        }



        impl<U: Unit, T: Unit> core::cmp::PartialEq<$name<T>> for $name<U> {
            fn eq(&self, other: &$name<T>) -> bool {
                self._si == other._si
            }
        }

        /*
        impl<U: Unit, T: Unit> core::cmp::Ord<$name<T>> for $name<U> {
            fn cmp(&self, other: &$name<T>) -> core::cmp::Ordering {
                self._si.total_cmp(&other._si)
            }
        }
        */

        impl<U: Unit> core::fmt::Display for $name<U> {
            fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
                write!(f, "{} {}", self.value(), U::NAME)
            }
        }

        $(
            #[allow(dead_code)]
            #[derive(Copy, Clone, Debug)]
            pub struct $U;

            impl Unit for $U {
                const FACTOR: f32 = $conv;
                const NAME: &'static str = stringify!($U);
            }
        )+

        impl<U, T> core::ops::Add<$name<T>> for $name<U> {
            type Output = $name<U>;

            fn add(self, rhs: $name<T>) -> Self::Output {
                Self::Output {
                    _si: self._si + rhs._si,
                    _unit: core::marker::PhantomData,
                }
            }
        }

        impl<U, T> core::ops::Sub<$name<T>> for $name<U> {
            type Output = $name<U>;

            fn sub(self, rhs: $name<T>) -> Self::Output {
                Self::Output {
                    _si: self._si - rhs._si,
                    _unit: core::marker::PhantomData,
                }
            }
        }

        impl<U> core::ops::Mul<f32> for $name<U> {
            type Output = $name<U>;

            fn mul(self, rhs: f32) -> Self::Output {
                Self::Output {
                    _si: self._si * rhs,
                    _unit: core::marker::PhantomData,
                }
            }
        }

        impl<U> core::ops::Div<f32> for $name<U> {
            type Output = $name<U>;

            fn div(self, rhs: f32) -> Self::Output {
                Self::Output {
                    _si: self._si / rhs,
                    _unit: core::marker::PhantomData,
                }
            }
        }

        impl<U, T> core::ops::Div<$name<T>> for $name<U> {
            type Output = f32;

            fn div(self, rhs: $name<T>) -> Self::Output {
                self._si / rhs._si
            }
        }
    };
}

#[macro_export]
macro_rules! derive_unit {

    (
        $name:ident,

        Types {
            $first:ident => $first_conv:literal,
            $( $U:ident => $conv:literal ),+ $(,)?
        }

        Conv {
            $L:ident / $R:ident //div arm
        }
    ) => {
        unit!(
            $name,

            Types {
                $first => $first_conv,
                $( $U => $conv ),+
            }
        );

        impl<LU: Unit, RU: Unit> core::ops::Div<$R<RU>> for $L<LU> {
            type Output = $name<$first>;

            fn div(self, rhs: $R<RU>) -> Self::Output {
                $name {
                    _si: self._si / rhs._si,
                    _unit: core::marker::PhantomData,
                }
            }
        }
    };

    (
        $name:ident,

        Types {
            $first:ident => $first_conv:literal,
            $( $U:ident => $conv:literal ),+ $(,)?
        }

        Conv {
            $L:ident * $R:ident //mul arm
        }
    ) => {
        unit!(
            $name,

            Types {
                $first:ident => $first_conv:literal,
                $( $U => $conv ),+
            }
        );

        impl<LU: Unit, RU: Unit> core::ops::Mul<$R<RU>> for $L<LU> {
            type Output = $name<$first>;

            fn mul(self, rhs: $R<RU>) -> Self::Output {
                $name {
                    _si: self._si * rhs._si,
                    _unit: core::marker::PhantomData,
                }
            }
        }
    };
}