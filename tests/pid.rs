use kernelvex::{AngularPid, Pid, QAngle};

#[test]
fn test_pid_zero_error_output() {
    let mut pid = Pid::new().set_gains(1.0, 0.5, 0.1);
    let output = pid.calculate(1.0, 1.0);
    assert!(output.abs() < 1e-9);
}

#[test]
fn test_pid_positive_error_output() {
    let mut pid = Pid::new().set_gains(2.0, 0.0, 0.0);
    let output = pid.calculate(1.5, 0.0);
    assert!((output - 3.0).abs() < 1e-9);
}

#[test]
fn test_pid_negative_error_output() {
    let mut pid = Pid::new().set_gains(2.0, 0.0, 0.0);
    let output = pid.calculate(0.0, 1.5);
    assert!((output + 3.0).abs() < 1e-9);
}

#[test]
fn test_pid_output_limits() {
    let mut pid = Pid::new()
        .set_gains(10.0, 0.0, 0.0)
        .with_output_limits(-5.0, 5.0);
    let output = pid.calculate(1.0, 0.0);
    assert!((output - 5.0).abs() < 1e-9);

    let output = pid.calculate(0.0, 1.0);
    assert!((output + 5.0).abs() < 1e-9);
}

#[test]
fn test_pid_integral_accumulates() {
    let mut pid = Pid::new().set_gains(0.0, 1.0, 0.0);
    let first = pid.calculate(1.0, 0.0);
    let second = pid.calculate(1.0, 0.0);
    assert!(second > first);
}

#[test]
fn test_pid_derivative_response() {
    let mut pid = Pid::new().set_gains(0.0, 0.0, 1.0);
    let _ = pid.calculate(0.0, 0.0);
    let output = pid.calculate(1.0, 0.0);
    assert!(output > 0.0);
}

// =============================================================================
// AngularPid Tests
// =============================================================================

#[test]
fn test_angular_pid_zero_error_output() {
    let mut pid = AngularPid::new().set_gains(1.0, 0.5, 0.1);
    let output = pid.calculate(QAngle::from_degrees(90.0), QAngle::from_degrees(90.0));
    assert!(output.abs() < 1e-9, "Expected ~0, got {}", output);
}

#[test]
fn test_angular_pid_positive_error() {
    let mut pid = AngularPid::new().set_gains(2.0, 0.0, 0.0);
    // setpoint > actual -> positive error -> positive output
    let output = pid.calculate(QAngle::from_degrees(100.0), QAngle::from_degrees(90.0));
    assert!(output > 0.0, "Expected positive output, got {}", output);
}

#[test]
fn test_angular_pid_negative_error() {
    let mut pid = AngularPid::new().set_gains(2.0, 0.0, 0.0);
    // setpoint < actual -> negative error -> negative output
    let output = pid.calculate(QAngle::from_degrees(80.0), QAngle::from_degrees(90.0));
    assert!(output < 0.0, "Expected negative output, got {}", output);
}

#[test]
fn test_angular_pid_output_limits() {
    let mut pid = AngularPid::new()
        .set_gains(100.0, 0.0, 0.0)
        .with_output_limits(-5.0, 5.0);

    // Large positive error should clamp to max
    let output = pid.calculate(QAngle::from_degrees(180.0), QAngle::from_degrees(0.0));
    assert!((output - 5.0).abs() < 1e-9, "Expected 5.0, got {}", output);

    // Large negative error should clamp to min
    pid.reset();
    let output = pid.calculate(QAngle::from_degrees(0.0), QAngle::from_degrees(180.0));
    assert!(
        (output - (-5.0)).abs() < 1e-9,
        "Expected -5.0, got {}",
        output
    );
}

#[test]
fn test_angular_pid_wraparound_positive() {
    // 350° -> 10° should take the short path (+20°), not the long path (-340°)
    let mut pid = AngularPid::new().set_gains(1.0, 0.0, 0.0);
    let output = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(350.0));

    // Short path is +20° (0.349 rad), so output should be positive and proportional to ~0.349
    assert!(
        output > 0.0,
        "Expected positive output for short path, got {}",
        output
    );

    // The error magnitude should be about 20° = 0.349 rad, so output ~ 0.349
    let expected_error_rad = 20.0_f64.to_radians();
    assert!(
        (output - expected_error_rad).abs() < 0.01,
        "Expected ~{}, got {}",
        expected_error_rad,
        output
    );
}

#[test]
fn test_angular_pid_wraparound_negative() {
    // 10° -> 350° should take the short path (-20°), not the long path (+340°)
    let mut pid = AngularPid::new().set_gains(1.0, 0.0, 0.0);
    let output = pid.calculate(QAngle::from_degrees(350.0), QAngle::from_degrees(10.0));

    // Short path is -20° (-0.349 rad), so output should be negative
    assert!(
        output < 0.0,
        "Expected negative output for short path, got {}",
        output
    );

    // The error magnitude should be about -20° = -0.349 rad
    let expected_error_rad = -20.0_f64.to_radians();
    assert!(
        (output - expected_error_rad).abs() < 0.01,
        "Expected ~{}, got {}",
        expected_error_rad,
        output
    );
}

#[test]
fn test_angular_pid_integral_accumulates() {
    let mut pid = AngularPid::new().set_gains(0.0, 1.0, 0.0);
    let first = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));
    let second = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));
    assert!(
        second > first,
        "Integral should accumulate: first={}, second={}",
        first,
        second
    );
}

#[test]
fn test_angular_pid_reset_clears_state() {
    let mut pid = AngularPid::new().set_gains(0.0, 1.0, 0.0);

    // Accumulate some integral
    let _ = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));
    let _ = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));
    let before_reset = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));

    pid.reset();

    // After reset, integral should start fresh
    let after_reset = pid.calculate(QAngle::from_degrees(10.0), QAngle::from_degrees(0.0));

    // before_reset had accumulated integral, after_reset should be smaller
    assert!(
        after_reset < before_reset,
        "Reset should clear state: before={}, after={}",
        before_reset,
        after_reset
    );
}
