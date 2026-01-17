use crate::angles::{Angle, Radians};

use vexide_devices::adi::encoder::AdiEncoder;
use vexide_devices::smart::rotation::RotationSensor;

// based off evian impl
pub trait Encoder {
    fn rotations(&self) -> Angle<Radians>;

}

impl Encoder for AdiEncoder<360> {
    fn rotations(&self) -> Angle<Radians>{
        Angle::<Radians>::new(self.position().unwrap().as_radians())
    }
}

impl Encoder for RotationSensor {
    fn rotations(&self) -> Angle<Radians>{
        Angle::<Radians>::new(self.position().unwrap().as_radians())
    }
}