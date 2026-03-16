extern crate kernelvex as kv;
use kv::{
    DifferentialDrive, MotorGroup, OdomChassis, Pose, QAngle, QLength,
    QTime, Tank, Trajectory, TrajectoryPoint,
};
use kv::ExpoDrive;
use vexide::prelude::*;
use kv::OmniWheel::Omni325;

struct Robot {
    controller: Controller,
    dt: DifferentialDrive,
    chassis: OdomChassis,
}

impl Compete for Robot {
    async fn autonomous(&mut self) {
        // Drive straight 2 meters using trapezoidal profile
        let _ = self
            .chassis
            .shoot(QLength::from_meters(2.))
            .await;

        // Turn to 90 degrees
        let _ = self
            .chassis
            .turn(QAngle::from_degrees(90.))
            .await;

        let traj = Trajectory::from_points(vec![
            TrajectoryPoint::new(
                Pose::new(Default::default(), QAngle::from_degrees(0.0)),
                0.0,
                0.0,
                QTime::from_sec(0.0),
            ),
            TrajectoryPoint::new(
                Pose::new(Default::default(), QAngle::from_degrees(0.0)),
                0.6,
                0.0,
                QTime::from_sec(2.0),
            ),
        ]);

        // Follow path
        let _ = self
            .chassis
            .trajectory(&traj)
            .await;
    }

    async fn driver(&mut self) {
        loop {
            let state = self.controller.state().unwrap_or_default();
            _ = self
                .dt
                .drive_tank(state.left_stick.y(), state.right_stick.x());

            sleep(Controller::UPDATE_INTERVAL).await;
        }
    }
}

#[vexide::main(banner(enabled = true))]
async fn main(peripherals: Peripherals) {
    let l_motor = MotorGroup::new([
        Motor::new(peripherals.port_11, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_6, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_12, Gearset::Blue, Direction::Reverse),
    ]);
    let r_motor = MotorGroup::new([
        Motor::new(peripherals.port_19, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_10, Gearset::Blue, Direction::Reverse),
        Motor::new(peripherals.port_9, Gearset::Blue, Direction::Reverse),
    ]);

    let mut imu = InertialSensor::new(peripherals.port_1);
    let _ = imu.calibrate().await;

    let chassis = OdomChassis::with_config(
        DifferentialDrive::new(l_motor.clone(),
                               r_motor.clone(),
                               ExpoDrive::new(1.5, 1.7, None),
                               Omni325,
                               QLength::from_meters(0.35),
                               1.),
        imu,
        None,
    );

    let robot = Robot {
        controller: peripherals.primary_controller,
        dt: DifferentialDrive::new(l_motor.clone(),
                                   r_motor.clone(),
                                   ExpoDrive::new(1.5, 1.7, None),
                                   Omni325,
                                   QLength::from_meters(0.35),
                                   1.),
        chassis,
    };

    robot.compete().await;
}
