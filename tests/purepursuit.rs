use kernelvex::Pose;
use kernelvex::PurePursuit;
use kernelvex::{QAngle, QTime, Vec2};
use kernelvex::{Trajectory, TrajectoryPoint};

const EPS: f64 = 1e-9;

#[test]
fn test_pure_pursuit_lookahead_on_line() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(2., 0.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));

    let pp = PurePursuit::new(traj, 1.0);
    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let target = pp.intersect(pose).expect("lookahead point");

    let pos = target.pose.position();
    assert!((pos.x - 1.0).abs() < 1e-9);
    assert!(pos.y.abs() < 1e-9);
}

#[test]
fn test_pure_pursuit_tangent() {
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(-18., 0.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(22., 30.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));
    let pp = PurePursuit::new(traj, 5.0);
    let pose = Pose::new(Vec2::<f64>::new(5., 11.), QAngle::from_degrees(0.0));
    let target = pp.intersect(pose).expect("lookahead point");
    let pos = target.pose.position();
    assert_eq!(pos.x, 2.);
    assert_eq!(pos.y, 15.);
}

#[test]
fn test_pure_pursuit_curvature_sign() {
    let traj = Trajectory::from_points(vec![TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    )]);
    let pp = PurePursuit::new(traj, 1.0);

    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let target = Vec2::<f64>::new(1.0, 1.0);
    let curvature = pp.curvature(pose, target);

    assert!(curvature > 0.0);
}

// =============================================================================
// Additional Coverage Tests
// =============================================================================

#[test]
fn test_pure_pursuit_no_intersection() {
    // Robot far from trajectory - lookahead circle doesn't intersect
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(100.0, 100.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(102.0, 100.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));

    let pp = PurePursuit::new(traj, 1.0); // 1m lookahead
    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0)); // at origin

    // No intersection because robot is 100m+ away from trajectory
    let result = pp.intersect(pose);
    assert!(
        result.is_none(),
        "Expected no intersection when robot is far from path"
    );
}

#[test]
fn test_pure_pursuit_multiple_segments() {
    // Multi-segment trajectory - should find the furthest intersection
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(2.0, 0.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(2.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(4.0, 0.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(4.0),
    ));

    let pp = PurePursuit::new(traj, 3.0); // 3m lookahead
    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));

    let target = pp.intersect(pose).expect("should find intersection");
    let pos = target.pose.position();

    // With 3m lookahead from origin, should intersect at x=3
    assert!((pos.x - 3.0).abs() < 0.01, "Expected x=3.0, got {}", pos.x);
    assert!(pos.y.abs() < EPS, "Expected y=0, got {}", pos.y);
}

#[test]
fn test_pure_pursuit_curvature_straight() {
    // Target directly ahead - zero curvature
    let traj = Trajectory::from_points(vec![TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    )]);
    let pp = PurePursuit::new(traj, 1.0);

    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let target = Vec2::<f64>::new(1.0, 0.0); // directly ahead
    let curvature = pp.curvature(pose, target);

    assert!(
        curvature.abs() < EPS,
        "Expected ~0 curvature for straight path, got {}",
        curvature
    );
}

#[test]
fn test_pure_pursuit_curvature_right_turn() {
    // Target to the right - negative curvature
    let traj = Trajectory::from_points(vec![TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    )]);
    let pp = PurePursuit::new(traj, 1.0);

    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let target = Vec2::<f64>::new(1.0, -1.0); // ahead and to the right
    let curvature = pp.curvature(pose, target);

    assert!(
        curvature < 0.0,
        "Expected negative curvature for right turn, got {}",
        curvature
    );
}

#[test]
fn test_pure_pursuit_small_lookahead() {
    // Very small lookahead - should still work
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(1.0),
    ));

    let pp = PurePursuit::new(traj, 0.01); // 1cm lookahead
    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));

    let target = pp.intersect(pose).expect("should find intersection");
    let pos = target.pose.position();

    // Should find intersection at x=0.01
    assert!(
        (pos.x - 0.01).abs() < 0.001,
        "Expected x=0.01, got {}",
        pos.x
    );
}

#[test]
fn test_pure_pursuit_zero_lookahead() {
    // Zero lookahead - curvature should return 0 to avoid division by zero
    let traj = Trajectory::from_points(vec![TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    )]);
    let pp = PurePursuit::new(traj, 0.0); // zero lookahead

    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let target = Vec2::<f64>::new(1.0, 1.0);
    let curvature = pp.curvature(pose, target);

    // Should return 0 to avoid division by zero
    assert!(
        curvature.abs() < EPS,
        "Expected 0 curvature for zero lookahead, got {}",
        curvature
    );
}

#[test]
fn test_pure_pursuit_robot_on_path() {
    // Robot exactly on the path
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(5.0, 0.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
        QTime::from_sec(5.0),
    ));

    let pp = PurePursuit::new(traj, 2.0);
    // Robot at (1,0) which is on the line segment
    let pose = Pose::new(Vec2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));

    let target = pp.intersect(pose).expect("should find intersection");
    let pos = target.pose.position();

    // Lookahead of 2m from (1,0) -> should hit (3,0)
    assert!((pos.x - 3.0).abs() < 0.01, "Expected x=3.0, got {}", pos.x);
}

#[test]
fn test_pure_pursuit_interpolates_velocity() {
    // Check that velocity is interpolated along segment
    let mut traj = Trajectory::new();
    traj.push(TrajectoryPoint::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        0.0, // start velocity
        0.0,
        QTime::from_sec(0.0),
    ));
    traj.push(TrajectoryPoint::new(
        Pose::new(Vec2::<f64>::new(2.0, 0.0), QAngle::from_degrees(0.0)),
        2.0, // end velocity
        0.0,
        QTime::from_sec(2.0),
    ));

    let pp = PurePursuit::new(traj, 1.0); // 1m lookahead
    let pose = Pose::new(Default::default(), QAngle::from_degrees(0.0));

    let target = pp.intersect(pose).expect("should find intersection");

    // At t=0.5 along segment (x=1.0), velocity should be lerp(0.0, 2.0, 0.5) = 1.0
    assert!(
        (target.linear_velocity - 1.0).abs() < 0.01,
        "Expected velocity ~1.0, got {}",
        target.linear_velocity
    );
}
