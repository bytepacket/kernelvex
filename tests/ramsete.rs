use kernelvex::control::ramsete::{RamseteController, RamseteReference};
use kernelvex::odom::pose::Pose;
use kernelvex::util::si::QAngle;
use kernelvex::Vector2;

#[test]
fn test_ramsete_zero_error() {
    let controller = RamseteController::new(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(Pose::new(Default::default(), QAngle::from_degrees(0.0)), 1.5, 0.2);

    let (v, w) = controller.calculate(current, reference);
    assert!((v - 1.5).abs() < 1e-6);
    assert!((w - 0.2).abs() < 1e-6);
}

#[test]
fn test_ramsete_heading_error() {
    let controller = RamseteController::new(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference =
        RamseteReference::new(Pose::new(Default::default(), QAngle::from_degrees(10.0)), 1.0, 0.0);

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}

#[test]
fn test_ramsete_position_error_forward() {
    let controller = RamseteController::new(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(Pose::new(Vector2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)), 1.0, 0.0);

    let (v, _w) = controller.calculate(current, reference);
    assert!(v > 0.0);
}

#[test]
fn test_ramsete_lateral_error_induces_turn() {
    let controller = RamseteController::new(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(Pose::new(Vector2::<f64>::new(0., 1.), QAngle::from_degrees(0.0)), 1.0, 0.0);

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}

#[test]
fn test_ramsete_sinc_small_angle() {
    let controller = RamseteController::new(2.0, 0.7).with_epsilon(1e-3);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference =
        RamseteReference::new(Pose::new(Vector2::<f64>::new(0., 1.), QAngle::from_degrees(0.01)), 1.0, 0.0);

    let (_v, w) = controller.calculate(current, reference);
    assert!(w.is_finite());
}

#[test]
fn test_ramsete_wraparound_heading() {
    let controller = RamseteController::new(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(350.0));
    let reference =
        RamseteReference::new(Pose::new(Default::default(), QAngle::from_degrees(10.0)), 1.0, 0.0);

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}
