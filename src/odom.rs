use crate::model::Drivetrain;
use vexide_devices::smart::imu::InertialSensor;
use crate::sensors::Encoder;
use crate::TrackingWheel;

struct OdomChassis<D: Drivetrain, T: Encoder> {
    dt: D,
    imu: InertialSensor,
    wheel: TrackingWheel<T>
}