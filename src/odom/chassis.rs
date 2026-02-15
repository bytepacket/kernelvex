use crate::{DifferentialDrive, TrackingWheel};
use vexide_devices::smart::imu::InertialSensor;

struct OdomChassis<const N: usize> {
    dt: DifferentialDrive<N>,
    imu: InertialSensor,
    wheel: TrackingWheel,
}
