use kernelvex::motion::profile::TrapezoidalConstraints;
use kernelvex::QLength;
const EPS: f64 = 1e-6;

fn is_non_decreasing(values: &[f64]) -> bool {
    values.windows(2).all(|w| w[1] + EPS >= w[0])
}

#[test]
fn triangular_profile_respects_limits_and_distance() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(1.0);
    let profile = constraints.generate_profile(distance);

    assert!(!profile.is_empty());
    let final_pos = profile.last().unwrap().position.as_meters();
    assert!((final_pos - distance.as_meters()).abs() < 1e-3);

    let max_velocity_seen = profile.iter().map(|s| s.velocity).fold(f64::MIN, f64::max);
    let v_peak_expected = (2.0 * constraints.max_acceleration * distance.as_meters()).sqrt();
    assert!(max_velocity_seen <= v_peak_expected + 1e-6);

    let positions: Vec<f64> = profile.iter().map(|s| s.position.as_meters()).collect();
    assert!(is_non_decreasing(&positions));

    assert!(profile.iter().all(|s| s.velocity >= -EPS));
}

#[test]
fn trapezoidal_profile_hits_cruise_and_stays_within_limits() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(10.0);
    let profile = constraints.generate_profile(distance);

    assert!(!profile.is_empty());
    let final_pos = profile.last().unwrap().position.as_meters();
    assert!((final_pos - distance.as_meters()).abs() < 1e-3);

    let positions: Vec<f64> = profile.iter().map(|s| s.position.as_meters()).collect();
    assert!(is_non_decreasing(&positions));

    let max_velocity_seen = profile.iter().map(|s| s.velocity).fold(f64::MIN, f64::max);
    assert!(max_velocity_seen <= constraints.max_velocity + 1e-6);

    let has_cruise = profile.iter().any(|s| {
        (s.velocity - constraints.max_velocity).abs() < 1e-3 && s.acceleration.abs() < 1e-3
    });
    assert!(has_cruise);
}
