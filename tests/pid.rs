use kernelvex::Pid;

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
