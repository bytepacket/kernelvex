use kernelvex::Pose;
use kernelvex::QAngle;
use kernelvex::Vec2;
use kernelvex::{RamseteController, RamseteReference};

const EPS: f64 = 1e-6;

#[test]
fn test_ramsete_zero_error() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Default::default(), QAngle::from_degrees(0.0)),
        1.5,
        0.2,
    );

    let (v, w) = controller.calculate(current, reference);
    assert!((v - 1.5).abs() < 1e-6);
    assert!((w - 0.2).abs() < 1e-6);
}

#[test]
fn test_ramsete_heading_error() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Default::default(), QAngle::from_degrees(10.0)),
        1.0,
        0.0,
    );

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}

#[test]
fn test_ramsete_position_error_forward() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(1., 0.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
    );

    let (v, _w) = controller.calculate(current, reference);
    assert!(v > 0.0);
}

#[test]
fn test_ramsete_lateral_error_induces_turn() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(0., 1.), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
    );

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}

#[test]
fn test_ramsete_sinc_small_angle() {
    let controller = RamseteController::new().set(2.0, 0.7).with_epsilon(1e-3);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(0., 1.), QAngle::from_degrees(0.01)),
        1.0,
        0.0,
    );

    let (_v, w) = controller.calculate(current, reference);
    assert!(w.is_finite());
}

#[test]
fn test_ramsete_wraparound_heading() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(350.0));
    let reference = RamseteReference::new(
        Pose::new(Default::default(), QAngle::from_degrees(10.0)),
        1.0,
        0.0,
    );

    let (_v, w) = controller.calculate(current, reference);
    assert!(w > 0.0);
}

// =============================================================================
// Edge Case Tests
// =============================================================================

#[test]
fn test_ramsete_backwards_motion() {
    let controller = RamseteController::new().set(2.0, 0.7);
    // Robot at origin, reference behind it with negative velocity
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(-1.0, 0.0), QAngle::from_degrees(180.0)),
        -1.0, // negative velocity = backwards
        0.0,
    );

    let (v, _w) = controller.calculate(current, reference);
    // Output should be negative (backwards motion)
    assert!(
        v < 0.0,
        "Expected negative velocity for backwards motion, got {}",
        v
    );
}

#[test]
fn test_ramsete_pure_rotation() {
    let controller = RamseteController::new().set(2.0, 0.7);
    // Robot and reference at same position, only heading difference
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Default::default(), QAngle::from_degrees(90.0)),
        0.0, // zero linear velocity
        1.0, // non-zero angular velocity
    );

    let (_v, w) = controller.calculate(current, reference);
    // With zero v_d, output v should be close to zero (only e_x correction)
    // Angular velocity should be positive (turning CCW)
    assert!(w > 0.0, "Expected positive angular velocity, got {}", w);
}

#[test]
fn test_ramsete_high_curvature() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    // High angular velocity reference (tight turn)
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(0.5, 0.5), QAngle::from_degrees(45.0)),
        1.0,
        2.0, // high angular velocity
    );

    let (v, w) = controller.calculate(current, reference);
    // Both should be finite and reasonable
    assert!(v.is_finite(), "Linear velocity should be finite");
    assert!(w.is_finite(), "Angular velocity should be finite");
    // Angular velocity should be positive (turning left)
    assert!(w > 0.0, "Expected positive angular velocity, got {}", w);
}

#[test]
fn test_ramsete_zero_gains() {
    let controller = RamseteController::new(); // zero gains (b=0, zeta=0)
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(1.0, 1.0), QAngle::from_degrees(45.0)),
        1.5,
        0.5,
    );

    let (v, w) = controller.calculate(current, reference);
    // With zero gains (b=0, zeta=0), k = 2*0*sqrt(...) = 0
    // v = v_d * cos(e_theta) + 0 * e_x = v_d * cos(e_theta)
    // w = w_d + 0 * e_theta + 0 * v_d * sinc * e_y = w_d
    assert!(v.is_finite(), "Linear velocity should be finite");
    assert!(
        (w - 0.5).abs() < EPS,
        "Angular velocity should equal w_d={}, got {}",
        0.5,
        w
    );
}

#[test]
fn test_ramsete_large_position_error() {
    let controller = RamseteController::new().set(2.0, 0.7);
    let current = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    // Large position error (10 meters ahead)
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(10.0, 0.0), QAngle::from_degrees(0.0)),
        1.0,
        0.0,
    );

    let (v, w) = controller.calculate(current, reference);
    // Large forward error should result in increased velocity
    assert!(
        v > 1.0,
        "Expected velocity > v_d=1.0 due to forward error, got {}",
        v
    );
    // No lateral error, so angular velocity should be near zero
    assert!(w.abs() < 0.1, "Expected small angular velocity, got {}", w);
}

#[test]
fn test_ramsete_combined_errors() {
    let controller = RamseteController::new().set(2.0, 0.7);
    // Robot off to the side, behind, and with wrong heading
    let current = Pose::new(Vec2::<f64>::new(-1.0, -1.0), QAngle::from_degrees(-30.0));
    let reference = RamseteReference::new(
        Pose::new(Vec2::<f64>::new(1.0, 1.0), QAngle::from_degrees(45.0)),
        2.0,
        0.3,
    );

    let (v, w) = controller.calculate(current, reference);
    // Should produce finite, reasonable outputs
    assert!(v.is_finite(), "Linear velocity should be finite");
    assert!(w.is_finite(), "Angular velocity should be finite");
    // With all errors present, both v and w should be non-zero
    assert!(v.abs() > EPS, "Expected non-zero linear velocity");
    assert!(w.abs() > EPS, "Expected non-zero angular velocity");
}
