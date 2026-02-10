use crate::dt::model::Drivetrain;
use crate::odom::sensors::Encoder;
use crate::TrackingWheel;
use vexide_devices::smart::imu::InertialSensor;

struct OdomChassis<D: Drivetrain, T: Encoder> {
    dt: D,
    imu: InertialSensor,
    wheel: TrackingWheel<T>,
}
