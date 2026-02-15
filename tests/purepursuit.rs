use kernelvex::PurePursuit;
use kernelvex::{Trajectory, TrajectoryPoint};
use kernelvex::Pose;
use kernelvex::{QAngle, QTime, Vector2};

#[test]
fn test_pure_pursuit_lookahead_on_line() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(2.0, 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));

    let pp = PurePursuit::new(traj, 1.0);
    let pose = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
    let target = pp.intersect(pose).expect("lookahead point");

    let (x, y) = target.pose.position();
    assert!((x - 1.0).abs() < 1e-9);
    assert!(y.abs() < 1e-9);
}

#[test]
fn test_pure_pursuit_tangent() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(-18., 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(22.0, 30.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));
    let pp = PurePursuit::new(traj, 5.0);
    let pose = Pose::new(5.0, 11.0, QAngle::from_degrees(0.0));
    let target = pp.intersect(pose).expect("lookahead point");
    let (x, y) = target.pose.position();
    assert_eq!(x, 2.);
    assert_eq!(y, 15.);
}

#[test]
fn test_pure_pursuit_curvature_sign() {
    let traj = Trajectory::from_points(vec![TrajectoryPoint::new(
        Pose::new(0.0, 0.0, QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    )]);
    let pp = PurePursuit::new(traj, 1.0);

    let pose = Pose::new(0.0, 0.0, QAngle::from_degrees(0.0));
    let target = Vector2::<f64>::new(1.0, 1.0);
    let curvature = pp.curvature(pose, target);

    assert!(curvature > 0.0);
}
