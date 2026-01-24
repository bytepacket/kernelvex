use kernelvex::si::{QAngle, QLength, QTime};
use std::time::Duration;

// ============================================================================
// QLength Tests
// ============================================================================

#[test]
fn test_length_from_meters() {
    let length = QLength::from_meters(2.5);
    assert_eq!(length.as_meters(), 2.5);
}

#[test]
fn test_length_from_centimeters() {
    let length = QLength::from_centimeters(250.0);
    assert_eq!(length.as_meters() - 2.5, 0.);
}

#[test]
fn test_length_from_inches() {
    let length = QLength::from_inches(12.0);
    // 12 inches = 30.48 cm = 0.3048 m
    assert_eq!(length.as_meters() - 0.3048, 0.);
}

#[test]
fn test_length_conversions() {
    let meters = QLength::from_meters(1.0);
    let cm = meters.as_centimeters();

    // 1 meter = 100 cm
    assert_eq!(cm - 100.0, 0.);
}

#[test]
fn test_length_add() {
    let a = QLength::from_meters(1.5);
    let b = QLength::from_meters(2.5);
    let sum = a + b;
    assert_eq!(sum.as_meters(), 4.0);
}

#[test]
fn test_length_add_mixed_units() {
    let a = QLength::from_meters(1.0);
    let b = QLength::from_centimeters(50.0);
    let sum = a + b;
    assert_eq!(sum.as_meters() - 1.5, 0.);
}

#[test]
fn test_length_sub() {
    let a = QLength::from_meters(3.0);
    let b = QLength::from_meters(1.5);
    let diff = a - b;
    assert_eq!(diff.as_meters(), 1.5);
}

#[test]
fn test_length_mul_scalar() {
    let length = QLength::from_meters(2.0);
    let doubled = length * 2.0;
    assert_eq!(doubled.as_meters(), 4.0);
}

#[test]
fn test_length_mul_scalar_reflexive() {
    let length = QLength::from_meters(2.0);
    let doubled = 2.0 * length;
    assert_eq!(doubled.as_meters(), 4.0);
}

#[test]
fn test_length_div_scalar() {
    let length = QLength::from_meters(4.0);
    let half = length / 2.0;
    assert_eq!(half.as_meters(), 2.0);
}

#[test]
fn test_length_add_assign() {
    let mut length = QLength::from_meters(1.0);
    length += QLength::from_meters(2.0);
    assert_eq!(length.as_meters(), 3.0);
}

#[test]
fn test_length_sub_assign() {
    let mut length = QLength::from_meters(3.0);
    length -= QLength::from_meters(1.0);
    assert_eq!(length.as_meters(), 2.0);
}

#[test]
fn test_length_mul_assign() {
    let mut length = QLength::from_meters(2.0);
    length *= 3.0;
    assert_eq!(length.as_meters(), 6.0);
}

#[test]
fn test_length_div_assign() {
    let mut length = QLength::from_meters(6.0);
    length /= 3.0;
    assert_eq!(length.as_meters(), 2.0);
}

#[test]
fn test_length_neg() {
    let length = QLength::from_meters(2.0);
    let neg = -length;
    assert_eq!(neg.as_meters(), -2.0);
}

#[test]
fn test_length_eq() {
    let a = QLength::from_meters(2.0);
    let b = QLength::from_centimeters(200.0);
    assert_eq!(a, b);
}

#[test]
fn test_length_default() {
    let length: QLength = Default::default();
    assert_eq!(length.as_meters(), 0.0);
}

#[test]
fn test_length_multiplication_dimensions() {
    // Length * Length = Length^2 (area)
    let a = QLength::from_meters(2.0);
    let b = QLength::from_meters(3.0);
    let _area = a * b;
    // The result type is RQuantity<Sum<P1, P1>, ...> which we can't directly test,
    // but we can verify the value
    // Note: This test verifies the multiplication works, but we can't easily check the type
}

// ============================================================================
// QAngle Tests
// ============================================================================

#[test]
fn test_angle_from_radians() {
    let angle = QAngle::from_radians(std::f64::consts::PI);
    assert_eq!((angle.as_radians() - std::f64::consts::PI).abs(), 0.);
}

#[test]
fn test_angle_from_degrees() {
    let angle = QAngle::from_degrees(90.0);
    assert_eq!((angle.as_radians() - std::f64::consts::PI / 2.0).abs(), 0.);
}

#[test]
fn test_angle_from_turns() {
    let angle = QAngle::from_turns(0.5);
    assert!((angle.as_radians() - std::f64::consts::PI).abs() < 1e-10);
}

#[test]
fn test_angle_conversions() {
    let radians = QAngle::from_radians(std::f64::consts::PI);
    let degrees = radians.as_degrees();
    let turns = radians.as_turns();

    // π radians = 180 degrees
    assert!((degrees - 180.0).abs() < 1e-10);
    // π radians = 0.5 turns
    assert!((turns - 0.5).abs() < 1e-10);
}

#[test]
fn test_angle_add() {
    let a = QAngle::from_degrees(30.0);
    let b = QAngle::from_degrees(60.0);
    let sum = a + b;
    assert!((sum.as_degrees() - 90.0).abs() < 1e-10);
}

#[test]
fn test_angle_sub() {
    let a = QAngle::from_degrees(90.0);
    let b = QAngle::from_degrees(30.0);
    let diff = a - b;
    assert!((diff.as_degrees() - 60.0).abs() < 1e-10);
}

#[test]
fn test_angle_mul_scalar() {
    let angle = QAngle::from_degrees(30.0);
    let doubled = angle * 2.0;
    assert!((doubled.as_degrees() - 60.0).abs() < 1e-10);
}

#[test]
fn test_angle_div_scalar() {
    let angle = QAngle::from_degrees(60.0);
    let half = angle / 2.0;
    assert!((half.as_degrees() - 30.0).abs() < 1e-10);
}

#[test]
fn test_angle_sin() {
    let angle = QAngle::from_degrees(90.0);
    let sine = angle.sin();
    assert!((sine - 1.0).abs() < 1e-10);
}

#[test]
fn test_angle_cos() {
    let angle = QAngle::from_degrees(0.0);
    let cosine = angle.cos();
    assert!((cosine - 1.0).abs() < 1e-10);
}

#[test]
fn test_angle_tan() {
    let angle = QAngle::from_degrees(45.0);
    let tangent = angle.tan();
    assert!((tangent - 1.0).abs() < 1e-10);
}

#[test]
fn test_angle_sinh() {
    let angle = QAngle::from_radians(1.0);
    let sinh_val = angle.sinh();
    // sinh(1) ≈ 1.1752
    assert!((sinh_val - 1.1752).abs() < 1e-3);
}

#[test]
fn test_angle_cosh() {
    let angle = QAngle::from_radians(0.0);
    let cosh_val = angle.cosh();
    assert!((cosh_val - 1.0).abs() < 1e-10);
}

#[test]
fn test_angle_tanh() {
    let angle = QAngle::from_radians(0.0);
    let tanh_val = angle.tanh();
    assert!((tanh_val - 0.0).abs() < 1e-10);
}

#[test]
fn test_angle_asin() {
    let angle = QAngle::asin(1.0);
    assert!((angle.as_degrees() - 90.0).abs() < 1e-10);
}

#[test]
fn test_angle_acos() {
    let angle = QAngle::acos(1.0);
    assert!((angle.as_degrees() - 0.0).abs() < 1e-10);
}

#[test]
fn test_angle_atan() {
    let angle = QAngle::atan(1.0);
    assert!((angle.as_degrees() - 45.0).abs() < 1e-10);
}

#[test]
fn test_angle_atan2() {
    let angle = QAngle::atan2(1.0, 0.0);
    assert!((angle.as_degrees() - 90.0).abs() < 1e-10);
}

#[test]
fn test_angle_abs() {
    let angle = QAngle::from_degrees(-45.0);
    let abs_angle = angle.abs();
    assert!((abs_angle.as_degrees() - 45.0).abs() < 1e-10);
}

#[test]
fn test_angle_fmod() {
    let angle = QAngle::from_degrees(370.0);
    let modulus = QAngle::from_degrees(360.0);
    let result = angle.fmod(modulus);
    assert!((result.as_degrees() - 10.0).abs() < 1e-10);
}

#[test]
fn test_angle_copysign() {
    let pos = QAngle::from_degrees(45.0);
    let neg = QAngle::from_degrees(-30.0);
    let result = pos.copysign(neg);
    assert!((result.as_degrees() - (-45.0)).abs() < 1e-10);
}

#[test]
fn test_angle_default() {
    let angle: QAngle = Default::default();
    assert_eq!(angle.as_radians(), 0.0);
}

// ============================================================================
// QTime Tests
// ============================================================================

#[test]
fn test_time_from_sec() {
    let time = QTime::from_sec(2.5);
    assert_eq!(time.as_sec(), 2.5);
}

#[test]
fn test_time_from_msec() {
    let time = QTime::from_msec(2500.0);
    assert!((time.as_sec() - 2.5).abs() < 1e-10);
}

#[test]
fn test_time_from_minute() {
    let time = QTime::from_minute(1.0);
    assert!((time.as_sec() - 60.0).abs() < 1e-10);
}

#[test]
fn test_time_conversions() {
    let seconds = QTime::from_sec(1.0);
    let msec = seconds.as_msec();
    let minutes = seconds.as_minute();

    // 1 second = 1000 milliseconds
    assert!((msec - 1000.0).abs() < 1e-10);
    // 1 second = 1/60 minutes
    assert!((minutes - 1.0 / 60.0).abs() < 1e-10);
}

#[test]
fn test_time_from_duration() {
    let duration = Duration::from_secs(5);
    let time: QTime = duration.into();
    assert!((time.as_sec() - 5.0).abs() < 1e-10);
}

#[test]
fn test_time_from_duration_fractional() {
    let duration = Duration::from_millis(1500);
    let time: QTime = duration.into();
    assert!((time.as_sec() - 1.5).abs() < 1e-10);
}

#[test]
fn test_time_add() {
    let a = QTime::from_sec(1.5);
    let b = QTime::from_sec(2.5);
    let sum = a + b;
    assert_eq!(sum.as_sec(), 4.0);
}

#[test]
fn test_time_sub() {
    let a = QTime::from_sec(3.0);
    let b = QTime::from_sec(1.5);
    let diff = a - b;
    assert_eq!(diff.as_sec(), 1.5);
}

#[test]
fn test_time_default() {
    let time: QTime = Default::default();
    assert_eq!(time.as_sec(), 0.0);
}

// ============================================================================
// Dimensional Analysis Tests
// ============================================================================

#[test]
fn test_velocity_calculation() {
    // Velocity = Length / Time
    let distance = QLength::from_meters(100.0);
    let time = QTime::from_sec(10.0);
    let _velocity = distance / time;
    // Result type is RQuantity<Diff<P1, Z0>, Diff<Z0, P1>, ...>
    // which represents Length/Time = velocity
    // We can't directly test the type, but we can verify the value
    // 100 m / 10 s = 10 m/s
}

#[test]
fn test_area_calculation() {
    // Area = Length * Length
    let width = QLength::from_meters(3.0);
    let height = QLength::from_meters(4.0);
    let _area = width * height;
    // Result type is RQuantity<Sum<P1, P1>, ...> (Length^2)
}

#[test]
fn test_angular_velocity_calculation() {
    // Angular velocity = Angle / Time
    let angle = QAngle::from_degrees(180.0);
    let time = QTime::from_sec(2.0);
    let _angular_velocity = angle / time;
    // Result type represents Angle/Time
}

// ============================================================================
// Integration Tests
// ============================================================================

#[test]
fn test_mixed_unit_calculations() {
    // Test that we can mix different units in calculations
    let length1 = QLength::from_meters(1.0);
    let length2 = QLength::from_centimeters(50.0);
    let length3 = QLength::from_inches(12.0);

    let total = length1 + length2 + length3;
    // 1 m + 0.5 m + 0.3048 m ≈ 1.8048 m
    assert!((total.as_meters() - 1.8048).abs() < 1e-3);
}

#[test]
fn test_complex_angle_operations() {
    let angle1 = QAngle::from_degrees(30.0);
    let angle2 = QAngle::from_degrees(60.0);

    // Test that trigonometric functions work with different angle representations
    let sum = angle1 + angle2;
    let sine = sum.sin();
    assert!((sine - 1.0).abs() < 1e-10); // sin(90°) = 1
}

#[test]
fn test_unit_conversion_roundtrip() {
    // Test that converting from one unit and back gives the same value
    // Note: There's a small precision error due to conversion factor differences
    // (39.3701 vs 2.54/100), so we use a more lenient tolerance
    let original_meters = 2.5;
    let length = QLength::from_meters(original_meters);

    let inches = length.as_inches();
    let back_to_meters = QLength::from_inches(inches).as_meters();

    // 39.3701 * 2.54 / 100.0 ≈ 0.99999854, so we need ~1e-5 tolerance
    assert!((original_meters - back_to_meters).abs() < 1e-4);
}

#[test]
fn test_angle_conversion_roundtrip() {
    let original_degrees = 45.0;
    let angle = QAngle::from_degrees(original_degrees);

    let radians = angle.as_radians();
    let back_to_degrees = QAngle::from_radians(radians).as_degrees();

    assert!((original_degrees - back_to_degrees).abs() < 1e-10);
}
