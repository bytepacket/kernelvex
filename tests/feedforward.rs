use kernelvex::{ArmFeedForward, FeedForward, QAngle};

const EPS: f64 = 1e-9;

#[test]
fn test_feedforward_zero_inputs() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.1);
    ff.set_kv(0.5);
    ff.set_ka(0.01);

    // v=0, a=0 -> output = ks * sign(0) + kv*0 + ka*0
    // Note: In Rust, 0.0_f64.signum() = 1.0 (positive zero has positive sign)
    // So output = 0.1 * 1 + 0 + 0 = 0.1
    let output = ff.calculate(0.0, 0.0);
    let expected = 0.1; // ks * signum(0) = 0.1 * 1.0
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_feedforward_velocity_only() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.1);
    ff.set_kv(0.5);
    ff.set_ka(0.01);

    let output = ff.calculate(2.0, 0.0);
    let expected = 0.1 + 0.5 * 2.0;
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_feedforward_acceleration_only() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.1);
    ff.set_kv(0.5);
    ff.set_ka(0.01);

    // v=0, a=5.0 -> output = ks*sign(0) + kv*0 + ka*a = 0.1*1 + 0 + 0.01*5 = 0.15
    // Note: signum(0.0) = 1.0 in Rust
    let output = ff.calculate(0.0, 5.0);
    let expected = 0.1 + 0.01 * 5.0; // ks*signum(0) + ka*a
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_feedforward_positive_velocity() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.2);
    ff.set_kv(0.0);
    ff.set_ka(0.0);

    let output = ff.calculate(3.0, 0.0);
    assert!((output - 0.2).abs() < EPS, "Expected 0.2, got {}", output);
}

#[test]
fn test_feedforward_negative_velocity() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.2);
    ff.set_kv(0.0);
    ff.set_ka(0.0);

    let output = ff.calculate(-3.0, 0.0);
    assert!(
        (output - (-0.2)).abs() < EPS,
        "Expected -0.2, got {}",
        output
    );
}

#[test]
fn test_feedforward_combined() {
    let mut ff = FeedForward::new();
    ff.set_ks(0.1);
    ff.set_kv(0.5);
    ff.set_ka(0.02);

    let output = ff.calculate(2.0, 3.0);
    let expected = 0.1 * 1.0 + 0.5 * 2.0 + 0.02 * 3.0;
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_feedforward_zero_gains() {
    let ff = FeedForward::new();

    assert!(ff.calculate(0.0, 0.0).abs() < EPS);
    assert!(ff.calculate(5.0, 0.0).abs() < EPS);
    assert!(ff.calculate(0.0, 5.0).abs() < EPS);
    assert!(ff.calculate(10.0, 10.0).abs() < EPS);
}

#[test]
fn test_arm_ff_zero_inputs() {
    let ff = ArmFeedForward::new(0.1, 0.5, 0.01, 0.3);

    // v=0, a=0, angle=0 -> output = ks*sign(0) + kv*0 + ka*0 + kg*cos(0)
    // = 0.1*1 + 0 + 0 + 0.3*1 = 0.4
    // Note: signum(0.0) = 1.0 in Rust
    let output = ff.calculate(QAngle::from_radians(0.0), 0.0, 0.0, |a| a.cos());
    let expected = 0.1 + 0.3 * 1.0; // ks*signum(0) + kg*cos(0)
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_arm_ff_gravity_horizontal() {
    let ff = ArmFeedForward::new(0.0, 0.0, 0.0, 1.0);

    let output = ff.calculate(QAngle::from_degrees(0.0), 0.0, 0.0, |a| a.cos());
    assert!((output - 1.0).abs() < EPS, "Expected 1.0, got {}", output);
}

#[test]
fn test_arm_ff_gravity_vertical() {
    let ff = ArmFeedForward::new(0.0, 0.0, 0.0, 1.0);

    let output = ff.calculate(QAngle::from_degrees(90.0), 0.0, 0.0, |a| a.cos());
    assert!(output.abs() < EPS, "Expected 0.0, got {}", output);
}

#[test]
fn test_arm_ff_gravity_downward() {
    let ff = ArmFeedForward::new(0.0, 0.0, 0.0, 1.0);

    let output = ff.calculate(QAngle::from_degrees(180.0), 0.0, 0.0, |a| a.cos());
    assert!(
        (output - (-1.0)).abs() < EPS,
        "Expected -1.0, got {}",
        output
    );
}

#[test]
fn test_arm_ff_velocity_term() {
    let ff = ArmFeedForward::new(0.0, 2.0, 0.0, 0.0);

    let output = ff.calculate(QAngle::from_degrees(0.0), 3.0, 0.0, |a| a.cos());
    assert!((output - 6.0).abs() < EPS, "Expected 6.0, got {}", output);
}

#[test]
fn test_arm_ff_acceleration_term() {
    let ff = ArmFeedForward::new(0.0, 0.0, 0.5, 0.0);

    let output = ff.calculate(QAngle::from_degrees(0.0), 0.0, 4.0, |a| a.cos());
    assert!((output - 2.0).abs() < EPS, "Expected 2.0, got {}", output);
}

#[test]
fn test_arm_ff_combined() {
    let ff = ArmFeedForward::new(0.1, 0.5, 0.02, 0.3);

    let output = ff.calculate(QAngle::from_degrees(60.0), 2.0, 3.0, |a| a.cos());
    let expected = 0.1 * 1.0 + 0.5 * 2.0 + 0.02 * 3.0 + 0.3 * 0.5;
    assert!(
        (output - expected).abs() < EPS,
        "Expected {}, got {}",
        expected,
        output
    );
}

#[test]
fn test_arm_ff_custom_gravity_function() {
    let ff = ArmFeedForward::new(0.0, 0.0, 0.0, 1.0);

    let output = ff.calculate(QAngle::from_degrees(90.0), 0.0, 0.0, |a| a.sin());
    assert!((output - 1.0).abs() < EPS, "Expected 1.0, got {}", output);

    let output_zero = ff.calculate(QAngle::from_degrees(0.0), 0.0, 0.0, |a| a.sin());
    assert!(output_zero.abs() < EPS, "Expected 0.0, got {}", output_zero);

    let output_offset = ff.calculate(QAngle::from_degrees(45.0), 0.0, 0.0, |a| {
        (a + QAngle::from_degrees(45.0)).cos()
    });
    assert!(
        output_offset.abs() < EPS,
        "Expected ~0.0, got {}",
        output_offset
    );
}
