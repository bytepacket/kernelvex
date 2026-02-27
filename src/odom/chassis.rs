use crate::control::pid::{AngularPid, Pid};
use crate::dt::model::Tank;
use crate::util::si::QAngle;
use crate::{DifferentialDrive, MotorGroup, TrackingRig, TrackingWheel};
use vexide_async::time::sleep;
use vexide_devices::smart::imu::InertialSensor;
use core::time::Duration;

pub struct OdomChassis {
    dt: DifferentialDrive,
    imu: InertialSensor,
    tracking: TrackingRig,
}

impl OdomChassis {
    pub fn new(dt: DifferentialDrive, imu: InertialSensor, tracking: TrackingRig) -> Self {
        Self { dt, imu,  tracking}
    }

    pub async fn turn(&mut self, target: QAngle) -> Result<(), TurnError> {
        let angle_tolerance = QAngle::from_degrees(2.0);
        let max_output = 6.0;

        let mut pid = AngularPid::new()
            .set_gains(1.0, 0.0, 0.5)
            .with_output_limits(
                QAngle::from_radians(-max_output),
                QAngle::from_radians(max_output),
            );

        loop {
            let current_heading = self.heading();
            let error = (target - current_heading).remainder(QAngle::TAU);

            if error.abs().as_radians() <= angle_tolerance.as_radians() {
                self.dt.drive_tank(0.0, 0.0).await.map_err(TurnError::Drive)?;
                break;
            }

            let output = pid.calculate(target, current_heading);

            let turn_rate = output.as_radians() / max_output;
            let left = turn_rate;
            let right = -turn_rate;

            self.dt.drive_tank(left, right).await.map_err(TurnError::Drive)?;
        }

        Ok(())
    }

    pub async fn shoot(&mut self, target_rpm: i32, motor: &mut MotorGroup) -> Result<(), ShootError> {
        let mut pid = Pid::new()
            .set_gains(0.1, 0.001, 0.05)
            .with_output_limits(-12.0, 12.0);

        let mut stable_count = 0;
        let stable_threshold = 10;

        loop {
            let current_rpm = motor.velocity().await.unwrap_or(0);

            let error = (target_rpm - current_rpm) as f64;

            if error.abs() <= 10.0 {
                stable_count += 1;
                if stable_count >= stable_threshold {
                    break;
                }
            } else {
                stable_count = 0;
            }

            let output = pid.calculate(target_rpm as f64, current_rpm as f64);

            motor.set_voltage(output).await.map_err(ShootError::Motor)?;
            sleep(Duration::from_millis(10)).await;
        }

        Ok(())
    }

    pub fn heading(&self) -> QAngle {
        self.imu.heading()
            .map(|a| QAngle::from_radians(a.as_radians()))
            .unwrap_or(QAngle::from_radians(0.0))
    }
}

#[derive(Debug)]
pub enum TurnError {
    Drive(crate::util::utils::GroupErrors),
}

#[derive(Debug)]
pub enum ShootError {
    Motor(crate::util::utils::GroupErrors),
}
