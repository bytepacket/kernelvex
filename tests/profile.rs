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

// =============================================================================
// Additional Profile Tests
// =============================================================================

#[test]
fn test_profile_zero_distance() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(0.0);
    let profile = constraints.generate_profile(distance);

    // With zero distance, all positions should be 0 and velocities should be 0
    assert!(!profile.is_empty());
    for state in &profile {
        assert!(
            state.position.as_meters().abs() < EPS,
            "Expected position 0, got {}",
            state.position.as_meters()
        );
    }
}

#[test]
fn test_profile_starts_at_zero_velocity() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(5.0);
    let profile = constraints.generate_profile(distance);

    let first = profile.first().unwrap();
    assert!(
        first.velocity.abs() < EPS,
        "Expected initial velocity 0, got {}",
        first.velocity
    );
    assert!(
        first.time.as_sec().abs() < EPS,
        "Expected initial time 0, got {}",
        first.time.as_sec()
    );
    assert!(
        first.position.as_meters().abs() < EPS,
        "Expected initial position 0, got {}",
        first.position.as_meters()
    );
}

#[test]
fn test_profile_ends_at_zero_velocity() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(5.0);
    let profile = constraints.generate_profile(distance);

    let last = profile.last().unwrap();
    // Final velocity should be approximately 0 (within tolerance due to discretization)
    assert!(
        last.velocity.abs() < 0.1,
        "Expected final velocity ~0, got {}",
        last.velocity
    );
}

#[test]
fn test_profile_respects_max_acceleration() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.5,
    };

    let distance = QLength::from_meters(10.0);
    let profile = constraints.generate_profile(distance);

    // All accelerations should be within bounds
    for state in &profile {
        assert!(
            state.acceleration.abs() <= constraints.max_acceleration + EPS,
            "Acceleration {} exceeds max {}",
            state.acceleration,
            constraints.max_acceleration
        );
    }
}

#[test]
fn test_profile_very_short_distance() {
    // Very short distance - should be triangular (no cruise phase)
    let constraints = TrapezoidalConstraints {
        max_velocity: 10.0, // high max velocity
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(0.5); // very short
    let profile = constraints.generate_profile(distance);

    assert!(!profile.is_empty());
    let final_pos = profile.last().unwrap().position.as_meters();
    assert!(
        (final_pos - distance.as_meters()).abs() < 1e-3,
        "Expected final position {}, got {}",
        distance.as_meters(),
        final_pos
    );

    // Should never reach max velocity (triangular profile)
    let max_velocity_seen = profile.iter().map(|s| s.velocity).fold(f64::MIN, f64::max);
    assert!(
        max_velocity_seen < constraints.max_velocity,
        "Triangular profile should not reach max velocity: {} >= {}",
        max_velocity_seen,
        constraints.max_velocity
    );
}

#[test]
fn test_profile_very_long_distance() {
    // Very long distance - should have extended cruise phase
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(100.0); // very long
    let profile = constraints.generate_profile(distance);

    assert!(!profile.is_empty());
    let final_pos = profile.last().unwrap().position.as_meters();
    assert!(
        (final_pos - distance.as_meters()).abs() < 1e-3,
        "Expected final position {}, got {}",
        distance.as_meters(),
        final_pos
    );

    // Should have many points at max velocity (cruise phase)
    let cruise_count = profile
        .iter()
        .filter(|s| {
            (s.velocity - constraints.max_velocity).abs() < 0.1 && s.acceleration.abs() < 0.1
        })
        .count();
    assert!(
        cruise_count > profile.len() / 2,
        "Long profile should have extended cruise phase: {} / {} points cruising",
        cruise_count,
        profile.len()
    );
}

#[test]
fn test_profile_time_is_monotonic() {
    let constraints = TrapezoidalConstraints {
        max_velocity: 2.0,
        max_acceleration: 1.0,
    };

    let distance = QLength::from_meters(5.0);
    let profile = constraints.generate_profile(distance);

    let times: Vec<f64> = profile.iter().map(|s| s.time.as_sec()).collect();
    assert!(
        is_non_decreasing(&times),
        "Time should be monotonically increasing"
    );
}
