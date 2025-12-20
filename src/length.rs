use std::cmp::PartialEq;
use std::ops::{Add, Div, Mul, Sub};

#[derive(PartialEq, Copy, Clone)]
enum LengthUnit {
    Meter(),

    Centimeter(),

    Millimeter(),

    Kilometer(),

    Inches(),

    Mile(),

    Feet(),

    Yards(),
}

#[derive(PartialEq, Copy, Clone)]
enum SpeedUnit {
    MPS,
    MPH,
    KPH,
}

#[derive(PartialEq, Copy, Clone)]
enum TimeUnit {
    Seconds,
    Minutes,
}

#[derive(PartialEq, Copy, Clone)]
enum AngleUnit {
    Radians,
    Degrees,
}

#[derive(PartialEq, Copy, Clone)]
enum AccelUnit {
    MPS2,
    G,
}

#[derive(Copy, Clone)]
pub struct Length {
    _si: f32,
    _type: LengthUnit,
}

pub struct Time {
    _si: f32,
    _type: TimeUnit,
}

pub struct Speed {
    _si: f32,
    _type: SpeedUnit,
}

pub struct Accel {
    _si: f32,
    _type: AccelUnit,
}

impl Length {
    #[inline]
    pub fn meters(v: f32) -> Self {
        Self {
            _si: v,
            _type: LengthUnit::Meter(),
        }
    }

    #[inline]
    pub fn cm(v: f32) -> Self {
        Self {
            _si: v / 100.0,
            _type: LengthUnit::Centimeter(),
        }
    }

    #[inline]
    pub fn mm(v: f32) -> Self {
        Self {
            _si: v / 1000.0,
            _type: LengthUnit::Millimeter(),
        }
    }

    #[inline]
    pub fn km(v: f32) -> Self {
        Self {
            _si: v * 1000.0,
            _type: LengthUnit::Kilometer(),
        }
    }

    #[inline]
    pub fn inches(v: f32) -> Self {
        Self {
            _si: v / 39.37,
            _type: LengthUnit::Inches(),
        }
    }

    #[inline]
    pub fn ft(v: f32) -> Self {
        Self {
            _si: v / 3.281,
            _type: LengthUnit::Feet(),
        }
    }

    #[inline]
    pub fn yd(v: f32) -> Self {
        Self {
            _si: v / 1.094,
            _type: LengthUnit::Yards(),
        }
    }

    #[inline]
    pub fn mi(v: f32) -> Self {
        Self {
            _si: v * 1609.344,
            _type: LengthUnit::Mile(),
        }
    }
}

impl Add for Length {
    type Output = Length;

    #[inline]
    fn add(self, rhs: Length) -> Length {
        if self._type == rhs._type {
            return Length {
                _si: self._si + rhs._si,
                _type: self._type,
            };
        }
        Length {
            _si: self._si + rhs._si,
            _type: LengthUnit::Meter(),
        }
    }
}

impl Sub for Length {
    type Output = Length;

    #[inline]
    fn sub(self, rhs: Length) -> Length {
        if self._type == rhs._type {
            return Length {
                _si: self._si + rhs._si,
                _type: self._type,
            };
        }
        Length {
            _si: self._si - rhs._si,
            _type: LengthUnit::Meter(),
        }
    }
}

impl Mul<f32> for Length {
    type Output = Length;

    fn mul(self, rhs: f32) -> Length {
        Length {
            _si: self._si * rhs,
            _type: self._type,
        }
    }
}

impl Mul<Length> for Length {
    type Output = Area;
    #[inline]
    fn mul(self, rhs: Length) -> Area {
        if self._type == rhs._type {
            return Area {
                _si: self._si * rhs._si,
                _type: self._type,
            };
        }
        Area {
            _si: self._si * rhs._si,
            _type: LengthUnit::Meter(),
        }
    }
}

impl Div<f32> for Length {
    type Output = Length;

    fn div(self, rhs: f32) -> Length {
        Length {
            _si: self._si / rhs,
            _type: self._type,
        }
    }
}

impl Div<Time> for Length {
    type Output = Speed;
    fn div(self, rhs: Time) -> Speed {
        Speed {
            _si: self._si / rhs._si,
            _type: SpeedUnit::MPS,
        }
    }
}

pub trait LengthExt {
    fn meters(self) -> Length;
    fn cm(self) -> Length;
    fn mm(self) -> Length;
    fn km(self) -> Length;
    fn inches(self) -> Length;
    fn ft(self) -> Length;
    fn yd(self) -> Length;
    fn mi(self) -> Length;
}

impl LengthExt for f32 {
    fn meters(self) -> Length {
        Length::meters(self)
    }

    fn cm(self) -> Length {
        Length::cm(self)
    }

    fn mm(self) -> Length {
        Length::mm(self)
    }

    fn km(self) -> Length {
        Length::km(self)
    }

    fn inches(self) -> Length {
        Length::inches(self)
    }

    fn ft(self) -> Length {
        Length::ft(self)
    }

    fn yd(self) -> Length {
        Length::yd(self)
    }

    fn mi(self) -> Length {
        Length::mi(self)
    }
}

pub struct Area {
    _si: f32,
    _type: LengthUnit,
}

impl Area {
    #[inline]
    pub fn m2(v: f32) -> Self {
        Self {
            _si: v,
            _type: LengthUnit::Meter(),
        }
    }
    #[inline]
    pub fn cm2(v: f32) -> Self {
        Self {
            _si: v / 10000.0,
            _type: LengthUnit::Centimeter(),
        }
    }
    #[inline]
    pub fn mm2(v: f32) -> Self {
        Self {
            _si: v / 1_000_000.0,
            _type: LengthUnit::Millimeter(),
        }
    }
    #[inline]
    pub fn km2(v: f32) -> Self {
        Self {
            _si: v * 1_000_000.0,
            _type: LengthUnit::Kilometer(),
        }
    }
    #[inline]
    pub fn in2(v: f32) -> Self {
        Self {
            _si: v / 1550.0,
            _type: LengthUnit::Inches(),
        }
    }
    #[inline]
    pub fn ft2(v: f32) -> Self {
        Self {
            _si: v / 10.764,
            _type: LengthUnit::Feet(),
        }
    }
    #[inline]
    pub fn yd2(v: f32) -> Self {
        Self {
            _si: v / 1.196,
            _type: LengthUnit::Yards(),
        }
    }
    #[inline]
    pub fn mi2(v: f32) -> Self {
        Self {
            _si: v * 2_590_000.0,
            _type: LengthUnit::Mile(),
        }
    }
}

impl Add for Area {
    type Output = Area;
    fn add(self, rhs: Area) -> Area {
        Area {
            _si: self._si + rhs._si,
            _type: self._type,
        }
    }
}

impl Sub for Area {
    type Output = Area;
    fn sub(self, rhs: Area) -> Area {
        Area {
            _si: self._si - rhs._si,
            _type: self._type,
        }
    }
}

impl Mul<f32> for Area {
    type Output = Area;
    fn mul(self, rhs: f32) -> Area {
        Area {
            _si: self._si * rhs,
            _type: self._type,
        }
    }
}

impl Div<f32> for Area {
    type Output = Area;
    fn div(self, rhs: f32) -> Area {
        Area {
            _si: self._si / rhs,
            _type: self._type,
        }
    }
}

impl Div<Length> for Area {
    type Output = Length;
    fn div(self, rhs: Length) -> Length {
        Length {
            _si: self._si / rhs._si,
            _type: LengthUnit::Meter(),
        }
    }
}

pub trait AreaExt {
    fn m2(self) -> Area;

    fn cm2(self) -> Area;

    fn mm2(self) -> Area;

    fn km2(self) -> Area;

    fn in2(self) -> Area;

    fn ft2(self) -> Area;

    fn yd2(self) -> Area;

    fn mi2(self) -> Area;
}

impl AreaExt for f32 {
    fn m2(self) -> Area {
        Area::m2(self)
    }
    fn cm2(self) -> Area {
        Area::cm2(self)
    }
    fn mm2(self) -> Area {
        Area::mm2(self)
    }
    fn km2(self) -> Area {
        Area::km2(self)
    }
    fn in2(self) -> Area {
        Area::in2(self)
    }
    fn ft2(self) -> Area {
        Area::ft2(self)
    }
    fn yd2(self) -> Area {
        Area::yd2(self)
    }
    fn mi2(self) -> Area {
        Area::mi2(self)
    }
}

impl Time {
    #[inline]
    pub fn seconds(v: f32) -> Self {
        Self {
            _si: v,
            _type: TimeUnit::Seconds,
        }
    }

    #[inline]
    pub fn minutes(v: f32) -> Self {
        Self {
            _si: v * 60.0,
            _type: TimeUnit::Minutes,
        }
    }
}

impl Add for Time {
    type Output = Time;

    fn add(self, rhs: Self) -> Self::Output {
        if self._type == rhs._type {
            return Self {
                _si: self._si + rhs._si,
                _type: self._type,
            };
        }
        Self {
            _si: self._si + rhs._si,
            _type: TimeUnit::Seconds,
        }
    }
}

impl Sub for Time {
    type Output = Time;

    fn sub(self, rhs: Self) -> Self::Output {
        if self._type == rhs._type {
            return Self {
                _si: self._si - rhs._si,
                _type: self._type,
            };
        }
        Self {
            _si: self._si - rhs._si,
            _type: TimeUnit::Seconds,
        }
    }
}

impl Mul<f32> for Time {
    type Output = Time;
    fn mul(self, rhs: f32) -> Self::Output {
        Self {
            _si: self._si * rhs,
            _type: self._type,
        }
    }
}

impl Mul<Speed> for Time {
    type Output = Length;
    fn mul(self, rhs: Speed) -> Self::Output {
        Length {
            _si: self._si * rhs._si,
            _type: LengthUnit::Meter(),
        }
    }
}

impl Div<f32> for Time {
    type Output = Time;

    fn div(self, rhs: f32) -> Self::Output {
        Self {
            _si: self._si / rhs,
            _type: self._type,
        }
    }
}

impl Div<Time> for Time {
    type Output = f32;
    fn div(self, rhs: Time) -> f32 {
        self._si / rhs._si
    }
}
