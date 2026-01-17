// TODO add tracking wheels

use crate::sensors::Encoder;
use crate::si::QLength;
use crate::utils::Orientation;
use vexide::math::Direction;
use vexide::smart::SmartPort;
use vexide_devices::adi::encoder::AdiEncoder;
use vexide_devices::smart::expander::AdiExpander;
use vexide_devices::smart::rotation::RotationSensor;

#[allow(unused)]
pub enum OmniWheel {
    Omni275,
    Omni325,
    Omni4,
    Anti275,
    Anti325,
    Anti4,
}

impl OmniWheel {
    #[allow(unused)]
    fn size(&self) -> QLength {
        match *self {
            OmniWheel::Omni275 => QLength::from_inches(2.75),
            OmniWheel::Omni325 => QLength::from_inches(3.25),
            OmniWheel::Omni4 => QLength::from_inches(4.125),
            OmniWheel::Anti275 => QLength::from_inches(2.75),
            OmniWheel::Anti325 => QLength::from_inches(3.25),
            OmniWheel::Anti4 => QLength::from_inches(4.),
        }
    }
}

pub trait Tracking {
    fn offset(&self) -> QLength;

    fn distance(&mut self) -> QLength;

    fn reset(&mut self);

    fn delta(&mut self) -> QLength;
}

pub struct TrackingWheel<T: Encoder> {
    encoder: T,
    wheel: OmniWheel,
    dist: QLength,
    orientation: Orientation,
    total: QLength,
    gearing: Option<f64>,
}

impl<T: Encoder> TrackingWheel<T> {
    #[allow(unused)]
    pub fn new(encoder: T, wheel: OmniWheel, dist: QLength, gearing: Option<f64>) -> Self {
        if dist.as_meters() > 0. {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Right,
                total: QLength::from_inches(0.),
                gearing,
            }
        } else {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Left,
                total: QLength::from_inches(0.),
                gearing,
            }
        }
    }
}

impl<T: Encoder> Tracking for TrackingWheel<T> {
    fn offset(&self) -> QLength {
        self.dist
    }

    fn distance(&mut self) -> QLength {
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().value())
                / core::f64::consts::TAU;

        self.total += distance;

        distance
    }

    fn reset(&mut self) {
        self.total = QLength::from_inches(0.);
    }

    fn delta(&mut self) -> QLength {
        let circumference = self.wheel.size() * core::f64::consts::PI;

        let distance =
            circumference * self.gearing.unwrap_or(1.) * (self.encoder.rotations().value())
                / core::f64::consts::TAU;

        let ret = self.total - distance;

        self.total += distance;

        ret
    }
}

fn test() {
    let expander = AdiExpander::new(unsafe { SmartPort::new(1) });

    let x = TrackingWheel::new(
        RotationSensor::new(unsafe { SmartPort::new(2) }, Direction::Forward),
        OmniWheel::Omni275,
        QLength::from_inches(1.),
        Some(12.),
    );

    let z = TrackingWheel::new(
        AdiEncoder::new(expander.adi_f, expander.adi_e),
        OmniWheel::Anti4,
        QLength::from_inches(-5.),
        Some(1.),
    );
}
