use kernelvex::motion::trajectory::{Trajectory, TrajectoryPoint};
use kernelvex::odom::pose::Pose;
use kernelvex::util::si::{QAngle, QTime};

#[test]
fn test_trajectory_sample_endpoints() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(1.0, 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let start = traj.sample(QTime::from_sec(0.0)).unwrap();
    let end = traj.sample(QTime::from_sec(1.0)).unwrap();

    let (sx, sy) = start.pose.position();
    let (ex, ey) = end.pose.position();

    assert!((sx - 0.0).abs() < 1e-6);
    assert!((sy - 0.0).abs() < 1e-6);
    assert!((ex - 1.0).abs() < 1e-6);
    assert!((ey - 0.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_sample_intermediate() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(2.0, 0.0, QAngle::from_degrees(0.0)),
        2.0,
        0.0,
        QTime::from_sec(2.0),
    ));

    let mid = traj.sample(QTime::from_sec(1.0)).unwrap();
    let (mx, my) = mid.pose.position();

    assert!((mx - 1.0).abs() < 1e-6);
    assert!((my - 0.0).abs() < 1e-6);
    assert!((mid.linear_velocity - 1.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_sample_out_of_bounds() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(1.0, 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let before = traj.sample(QTime::from_sec(-1.0)).unwrap();
    let after = traj.sample(QTime::from_sec(2.0)).unwrap();

    let (bx, by) = before.pose.position();
    let (ax, ay) = after.pose.position();

    assert!((bx - 0.0).abs() < 1e-6);
    assert!((by - 0.0).abs() < 1e-6);
    assert!((ax - 1.0).abs() < 1e-6);
    assert!((ay - 0.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_heading_interpolation_shortest_arc() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(350.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(10.0)),
        0.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let mid = traj.sample(QTime::from_sec(0.5)).unwrap();
    let heading = mid.pose.heading().remainder(QAngle::TAU).as_degrees();
    let heading_abs = heading.abs();

    assert!(heading_abs < 1e-6 || (360.0 - heading_abs).abs() < 1e-6);
}

#[test]
fn test_trajectory_total_time() {
    let traj = Trajectory::from_points(vec![
        TrajectoryPoint::new(
            Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
            0.0,
            0.0,
            QTime::from_sec(0.0),
        ),
        TrajectoryPoint::new(
            Pose::new(1.0, 0.0, QAngle::from_degrees(0.0)),
            1.0,
            0.0,
            QTime::from_sec(3.0),
        ),
    ]);

    let total = traj.total_time().unwrap();
    assert!((total.as_sec() - 3.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_sample_empty() {
    let traj = Trajectory::new();
    assert!(traj.sample(QTime::from_sec(0.0)).is_none());
}
