use nalgebra::base::Matrix3;

use crate::angles::{Angle, Radians};

#[derive(Debug)]
pub struct Pose {
    position: Matrix3<f32>,
    heading: Angle<Radians>,
}

impl core::fmt::Display for Pose {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "[[{:.3}, {:.3}, {:.3}]\n \
              [{:.3}, {:.3}, {:.3}]\n \
              [{:.3}, {:.3}, {:.3}]]",
            self.position.m11, self.position.m12, self.position.m13,
            self.position.m21, self.position.m22, self.position.m23,
            self.position.m31, self.position.m32, self.position.m33,
        )
    }
}


impl Pose {
    pub fn new(x: f32, y: f32, heading: Angle<Radians>) -> Self {

        Pose {
            position: Matrix3::new(heading.cos(), -heading.sin(), x,
                                  heading.sin(), heading.cos(), y,
                                  0., 0., 1.),
            heading,
        }
    }

    pub const fn heading(&self) -> Angle<Radians> {
        self.heading
    }

    pub fn position(&self) -> (f32, f32) {
        (self.position.m13, self.position.m23)
    }

    pub fn angle(&self, other: Pose) -> f32 {
        libm::atan2f(other.position.m23 - self.position.m23, other.position.m13 - self.position.m13)
    }

    pub fn rotate(&self, angle: Angle<Radians>) -> Pose {
        Pose::new(self.position().0, self.position().1, self.heading + angle)
    }

    pub fn distance(&self, other: Pose) -> f32 {
        libm::hypotf((self.position.m13 - other.position.m13),(self.position.m23 - other.position.m23))
    }


    // effectively the same as mul two poses, but this does not take heading into account
    pub fn move_local(&self, other: Pose) -> Pose {
        Pose {
            position: self.position * other.position,
            heading: self.heading,
        }
    }

    // moves in global coordinates (just adds the poses), also doesnt take heading into account
    pub fn move_global(&self, other: Pose) -> Pose {
        Pose::new(self.position.m13 + other.position.m13, self.position.m23 + other.position.m23, 0.0.into())
    }

}

// does not take heading into consideration
impl core::ops::Add<Pose> for Pose {
    type Output = Pose;
    fn add(self, other: Pose) -> Pose {
        Pose::new(self.position().0 + other.position().0, self.position().1 + other.position().1, self.heading)
    }
}

impl core::ops::Sub<Pose> for Pose {
    type Output = Pose;
    fn sub(self, other: Pose) -> Pose {
        Pose::new(self.position().0 - other.position().0, self.position().1 - other.position().1, self.heading)
    }
}

impl core::ops::Mul<Pose> for Pose {
    type Output = Pose;
    fn mul(self, rhs: Pose) -> Self::Output {
        Pose {
            position: self.position * rhs.position,
            heading: self.heading + rhs.heading,
        }
    }
}

impl core::ops::Mul<f32> for Pose {
    type Output = Pose;

    fn mul(self, rhs: f32) -> Self::Output {
        Pose::new(self.position.m13 * rhs, self.position.m23 * rhs, self.heading)
    }
}

impl core::ops::Div<f32> for Pose {
    type Output = Pose;

    fn div(self, rhs: f32) -> Self::Output {
        Pose::new(self.position.m13 / rhs, self.position.m23 / rhs, self.heading)
    }
}



