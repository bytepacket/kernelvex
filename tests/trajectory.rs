use kernelvex::motion::trajectory::{Bezier, Trajectory, TrajectoryPoint};
use kernelvex::odom::pose::Pose;
use kernelvex::util::si::Vec2;
use kernelvex::util::si::{QAngle, QTime};

const EPS: f64 = 1e-6;

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
        Pose::new(Vec2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
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
        Pose::new(Vec2::<f64>::new(2., 0.), QAngle::from_degrees(0.0)),
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
        Pose::new(Vec2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
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
            Pose::new(Vec2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
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
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 0.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(2.0, 1.0),
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
    let bezier = Bezier::new(
        Vec2::<f64>::new(1., 0.),
        Vec2::<f64>::new(2., 3.),
        Vec2::<f64>::new(3., 4.),
        Vec2::<f64>::new(5., 6.),
    );

    println!("{:?}", bezier.point(0.7));
}

// =============================================================================
// Additional Bezier Tests
// =============================================================================

#[test]
fn test_bezier_midpoint() {
    // Symmetric curve - midpoint should be predictable
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(0.0, 1.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(1.0, 0.0),
    );

    let mid = bezier.point(0.5);
    // For this symmetric curve, midpoint should be at (0.5, 0.75)
    // B(0.5) = (1-t)^3*P0 + 3*(1-t)^2*t*P1 + 3*(1-t)*t^2*P2 + t^3*P3
    // = 0.125*(0,0) + 0.375*(0,1) + 0.375*(1,1) + 0.125*(1,0)
    // = (0, 0) + (0, 0.375) + (0.375, 0.375) + (0.125, 0)
    // = (0.5, 0.75)
    assert!((mid.x - 0.5).abs() < EPS, "Expected x=0.5, got {}", mid.x);
    assert!((mid.y - 0.75).abs() < EPS, "Expected y=0.75, got {}", mid.y);
}

#[test]
fn test_bezier_derivative_at_endpoints() {
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 0.0),
        Vec2::<f64>::new(2.0, 1.0),
        Vec2::<f64>::new(3.0, 1.0),
    );

    // At t=0, derivative should point from P0 toward P1: 3*(P1-P0) = 3*(1,0) = (3,0)
    let d0 = bezier.derivative(0.0);
    assert!(
        (d0.x - 3.0).abs() < EPS,
        "Expected dx=3.0 at t=0, got {}",
        d0.x
    );
    assert!(d0.y.abs() < EPS, "Expected dy=0 at t=0, got {}", d0.y);

    // At t=1, derivative should point from P2 toward P3: 3*(P3-P2) = 3*(1,0) = (3,0)
    let d1 = bezier.derivative(1.0);
    assert!(
        (d1.x - 3.0).abs() < EPS,
        "Expected dx=3.0 at t=1, got {}",
        d1.x
    );
    assert!(d1.y.abs() < EPS, "Expected dy=0 at t=1, got {}", d1.y);
}

#[test]
fn test_bezier_symmetry() {
    // For a symmetric curve, point(t) and point(1-t) should be symmetric
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(0.5, 1.0),
        Vec2::<f64>::new(0.5, 1.0),
        Vec2::<f64>::new(1.0, 0.0),
    );

    let p1 = bezier.point(0.25);
    let p2 = bezier.point(0.75);

    // x coordinates should be symmetric around 0.5
    assert!(
        (p1.x + p2.x - 1.0).abs() < EPS,
        "Expected symmetric x: {} + {} = 1.0",
        p1.x,
        p2.x
    );
    // y coordinates should be equal
    assert!(
        (p1.y - p2.y).abs() < EPS,
        "Expected equal y: {} == {}",
        p1.y,
        p2.y
    );
}

#[test]
fn test_bezier_straight_line() {
    // All control points collinear -> straight line
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(2.0, 2.0),
        Vec2::<f64>::new(3.0, 3.0),
    );

    // All points should be on the line y = x
    for i in 0..=10 {
        let t = i as f64 / 10.0;
        let p = bezier.point(t);
        assert!(
            (p.x - p.y).abs() < EPS,
            "Expected y=x at t={}: got ({}, {})",
            t,
            p.x,
            p.y
        );
    }
}

#[test]
fn test_bezier_to_trajectory_point_count() {
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 0.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(2.0, 1.0),
    );

    let samples = 10;
    let traj = bezier.to_trajectory(QTime::from_sec(2.0), samples, 1.0);

    assert_eq!(
        traj.points().len(),
        samples,
        "Expected {} points, got {}",
        samples,
        traj.points().len()
    );
}

#[test]
fn test_bezier_to_trajectory_velocity() {
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 0.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(2.0, 1.0),
    );

    let velocity = 1.5;
    let traj = bezier.to_trajectory(QTime::from_sec(2.0), 5, velocity);

    // All points should have the specified linear velocity
    for (i, point) in traj.points().iter().enumerate() {
        assert!(
            (point.linear_velocity - velocity).abs() < EPS,
            "Point {} has velocity {}, expected {}",
            i,
            point.linear_velocity,
            velocity
        );
    }
}

#[test]
fn test_bezier_to_trajectory_time_spacing() {
    let bezier = Bezier::new(
        Vec2::<f64>::new(0.0, 0.0),
        Vec2::<f64>::new(1.0, 0.0),
        Vec2::<f64>::new(1.0, 1.0),
        Vec2::<f64>::new(2.0, 1.0),
    );

    let total_time = 4.0;
    let samples = 5;
    let traj = bezier.to_trajectory(QTime::from_sec(total_time), samples, 1.0);
    let points = traj.points();

    // Time should be evenly spaced: 0, 1, 2, 3, 4
    let dt = total_time / (samples as f64 - 1.0);
    for (i, point) in points.iter().enumerate() {
        let expected_time = dt * i as f64;
        assert!(
            (point.time.as_sec() - expected_time).abs() < EPS,
            "Point {} has time {}, expected {}",
            i,
            point.time.as_sec(),
            expected_time
        );
    }

    // First point at t=0, last at total_time
    assert!(points.first().unwrap().time.as_sec().abs() < EPS);
    assert!((points.last().unwrap().time.as_sec() - total_time).abs() < EPS);
}
