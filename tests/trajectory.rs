use kernelvex::motion::trajectory::{Bezier, Trajectory, TrajectoryPoint};
use kernelvex::odom::pose::Pose;
use kernelvex::util::si::Vector2;
use kernelvex::util::si::{QAngle, QTime};

#[test]
fn test_trajectory_sample_endpoints() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vector2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let start = traj.sample(QTime::from_sec(0.0)).unwrap();
    let end = traj.sample(QTime::from_sec(1.0)).unwrap();

    let (sx, sy) = (start.pose.position().x, start.pose.position().y);
    let (ex, ey) = (end.pose.position().x, end.pose.position().y);

    assert!((sx - 0.0).abs() < 1e-6);
    assert!((sy - 0.0).abs() < 1e-6);
    assert!((ex - 1.0).abs() < 1e-6);
    assert!((ey - 0.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_sample_intermediate() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vector2::<f64>::new(2., 0.), QAngle::from_degrees(0.0)),
        2.0,
        0.0,
        QTime::from_sec(2.0),
    ));

    let mid = traj.sample(QTime::from_sec(1.0)).unwrap();
    let m = mid.pose.position();

    assert!((m.x - 1.0).abs() < 1e-6);
    assert!((m.y - 0.0).abs() < 1e-6);
    assert!((mid.linear_velocity - 1.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_sample_out_of_bounds() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vector2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let before = traj.sample(QTime::from_sec(-1.0)).unwrap();
    let after = traj.sample(QTime::from_sec(2.0)).unwrap();

    let b = before.pose.position();
    let a = after.pose.position();

    assert!((b.x - 0.0).abs() < 1e-6);
    assert!((b.y - 0.0).abs() < 1e-6);
    assert!((a.x - 1.0).abs() < 1e-6);
    assert!((a.y - 0.0).abs() < 1e-6);
}

#[test]
fn test_trajectory_heading_interpolation_shortest_arc() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(350.0)),
        0.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(10.0)),
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
            Pose::new(Default::default(), QAngle::from_degrees(0.0)),
            0.0,
            0.0,
            QTime::from_sec(0.0),
        ),
        TrajectoryPoint::new(
            Pose::new(Vector2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
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

#[test]
fn test_bezier_to_trajectory_endpoints() {
    let bezier = Bezier::new(
        Vector2::<f64>::new(0.0, 0.0),
        Vector2::<f64>::new(1.0, 0.0),
        Vector2::<f64>::new(1.0, 1.0),
        Vector2::<f64>::new(2.0, 1.0),
    );

    let traj = bezier.to_trajectory(QTime::from_sec(2.0), 5, 1.0);
    let start = traj.sample(QTime::from_sec(0.0)).unwrap();
    let end = traj.sample(QTime::from_sec(2.0)).unwrap();

    let s = start.pose.position();
    let e = end.pose.position();

    assert!((s.x - 0.0).abs() < 1e-9);
    assert!((s.y - 0.0).abs() < 1e-9);
    assert!((e.x - 2.0).abs() < 1e-9);
    assert!((e.y - 1.0).abs() < 1e-9);
}

#[test]
fn test_bezier_point() {
    let bezier = Bezier::new(Vector2::<f64>::new(1., 0.),
                                    Vector2::<f64>::new(2., 3.),
                                    Vector2::<f64>::new(3., 4.),
                                    Vector2::<f64>::new(5., 6.));

    println!("{:?}", bezier.point(0.7));
}
