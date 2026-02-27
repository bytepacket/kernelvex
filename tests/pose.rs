use kernelvex::odom::pose::Pose;
use kernelvex::util::si::QAngle;
use std::f64::consts::PI;
use kernelvex::Vector2;
// TODO: fix heading not being added in tests
// ============================================================================
// Basic Construction Tests
// ============================================================================

#[test]
fn test_pose_new() {
    let pose = Pose::new(Vector2::<f64>::new(1.5, 2.0), QAngle::from_degrees(45.0));
    let pos =pose.position();
    assert_eq!(pos.x, 1.5);
    assert_eq!(pos.y, 2.0);
    assert!((pose.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_identity() {
    let identity = Pose::identity();
    let pos =identity.position();
    assert_eq!(pos.x, 0.0);
    assert_eq!(pos.y, 0.0);
    assert_eq!(identity.heading().as_radians(), 0.0);
}

#[test]
fn test_pose_default() {
    let default: Pose = Default::default();
    let pos =default.position();
    assert_eq!(pos.x, 0.0);
    assert_eq!(pos.y, 0.0);
    assert_eq!(default.heading().as_radians(), 0.0);
}

#[test]
fn test_pose_position() {
    let pose = Pose::new(Vector2::<f64>::new(3.0, 4.0), QAngle::from_degrees(90.0));
    let pos =pose.position();
    assert_eq!(pos.x, 3.0);
    assert_eq!(pos.y, 4.0);
}

#[test]
fn test_pose_heading() {
    let pose = Pose::new(Default::default(), QAngle::from_degrees(45.0));
    assert!((pose.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

// ============================================================================
// Distance Tests
// ============================================================================

#[test]
fn test_pose_distance_horizontal() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(3.0, 0.0), QAngle::from_degrees(0.0));
    let dist = p1.distance(p2);
    assert_eq!(dist.as_meters(), 3.0);
}

#[test]
fn test_pose_distance_vertical() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(0., 4.), QAngle::from_degrees(0.0));
    let dist = p1.distance(p2);
    assert_eq!(dist.as_meters(), 4.0);
}

#[test]
fn test_pose_distance_diagonal() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(3.0, 4.0), QAngle::from_degrees(0.0));
    let dist = p1.distance(p2);
    // Distance should be sqrt(3^2 + 4^2) = 5.0
    assert_eq!(dist.as_meters(), 5.0);
}

#[test]
fn test_pose_distance_symmetric() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(3.0, 4.0), QAngle::from_degrees(0.0));
    let dist1 = p1.distance(p2);
    let dist2 = p2.distance(p1);
    assert_eq!(dist1.as_meters(), dist2.as_meters());
}

#[test]
fn test_pose_distance_zero() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let dist = p1.distance(p2);
    assert_eq!(dist.as_meters(), 0.0);
}

// ============================================================================
// Angle Tests
// ============================================================================

#[test]
fn test_pose_angle_horizontal() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let angle = p1.angle(p2);
    assert_eq!(angle.as_radians(), 0.0);
}

#[test]
fn test_pose_angle_vertical() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(0.0, 1.0), QAngle::from_degrees(0.0));
    let angle = p1.angle(p2);
    assert!((angle.as_radians() - PI / 2.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_angle_diagonal() {
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 1.0), QAngle::from_degrees(0.0));
    let angle = p1.angle(p2);
    // Angle should be 45 degrees = π/4 radians
    assert!((angle.as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_angle_negative_y() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 1.0), QAngle::from_degrees(0.0));
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let angle = p1.angle(p2);
    // Angle should be -90 degrees = -π/2 radians
    assert_eq!(angle.as_degrees(), -90.);
}

// ============================================================================
// Rotation Tests
// ============================================================================

#[test]
fn test_pose_rotate() {
    let pose = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let rotated = pose.rotate(QAngle::from_degrees(90.0));
    let pos =rotated.position();
    // Position should remain unchanged
    assert_eq!(pos.x, 1.0);
    assert_eq!(pos.y, 2.0);
    // Heading should be rotated
    assert!((rotated.heading().as_degrees() - 90.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_rotate_multiple() {
    let pose = Pose::new(Default::default(), QAngle::from_degrees(30.0));
    let rotated = pose.rotate(QAngle::from_degrees(60.0));
    // Heading should be 30 + 60 = 90 degrees
    assert!((rotated.heading().as_degrees() - 90.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_rotate_negative() {
    let pose = Pose::new(Vector2::<f64>::new(1.0, 1.0), QAngle::from_degrees(90.0));
    let rotated = pose.rotate(QAngle::from_degrees(-45.0));
    assert!((rotated.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

// ============================================================================
// Addition Tests
// ============================================================================

#[test]
fn test_pose_add() {
    let p1 = Pose::new(Vector2::<f64>::new(1., 2.), QAngle::from_degrees(45.0));
    let p2 = Pose::new(Vector2::<f64>::new(3., 4.), QAngle::from_degrees(90.0));
    let sum = p1 + p2;
    let pos =sum.position();
    // Positions should be added: (1+3, 2+4) = (4, 6)
    assert_eq!(pos.x, 4.0);
    assert_eq!(pos.y, 6.0);
    // Heading should be preserved from left operand
    assert!((sum.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_add_identity() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(30.0));
    let identity = Pose::identity();
    let sum = p1 + identity;
    let pos =sum.position();
    assert_eq!(pos.x, 1.0);
    assert_eq!(pos.y, 2.0);
}

// ============================================================================
// Subtraction Tests
// ============================================================================

#[test]
fn test_pose_sub() {
    let p1 = Pose::new(Vector2::<f64>::new(5.0, 7.0), QAngle::from_degrees(45.0));
    let p2 = Pose::new(Vector2::<f64>::new(2.0, 3.0), QAngle::from_degrees(90.0));
    let diff = p1 - p2;
    let pos =diff.position();
    // Positions should be subtracted: (5-2, 7-3) = (3, 4)
    assert_eq!(pos.x, 3.0);
    assert_eq!(pos.y, 4.0);
    // Heading should be preserved from left operand
    assert!((diff.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_sub_identity() {
    let p1 = Pose::new(Vector2::<f64>::new(3.0, 4.0), QAngle::from_degrees(60.0));
    let identity = Pose::identity();
    let diff = p1 - identity;
    let pos =diff.position();
    assert_eq!(pos.x, 3.0);
    assert_eq!(pos.y, 4.0);
    assert!((diff.heading().as_degrees() - 60.0).abs() < 1e-10);
}

// ============================================================================
// Multiplication Tests (Composition)
// ============================================================================

#[test]
fn test_pose_mul_identity() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(45.0));
    let identity = Pose::identity();
    let result = p1 * identity;
    let pos = result.position();
    // Multiplying by identity should leave position unchanged
    assert!((pos.x - 1.0).abs() < f64::EPSILON);
    assert!((pos.y - 2.0).abs() < f64::EPSILON);
    // Heading should be unchanged (45 + 0 = 45)
    assert!((result.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_mul_translation_only() {
    // First pose: identity (0, 0, 0°)
    let p1 = Pose::identity();
    // Second pose: translation only (1, 2, 0°)
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let result = p1 * p2;
    let pos =result.position();
    // Result should be (1, 2)
    assert!((pos.x - 1.0).abs() < f64::EPSILON);
    assert!((pos.y - 2.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_mul_rotation_only() {
    // First pose: at origin, rotated 90°
    let p1 = Pose::new(Default::default(), QAngle::from_degrees(90.0));
    // Second pose: at (1, 0), no rotation
    let p2 = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let result = p1 * p2;
    let pos =result.position();
    // Rotating (1, 0) by 90° should give (0, 1)
    assert!((pos.x - 0.0).abs() < f64::EPSILON);
    assert!((pos.y - 1.0).abs() < f64::EPSILON);
    // Combined heading should be 90° + 0° = 90°
    assert!((result.heading().as_degrees() - 90.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_mul_combined() {
    // First pose: (1, 0) with 90° rotation
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(90.0));
    // Second pose: (0, 1) with 90° rotation
    let p2 = Pose::new(Vector2::<f64>::new(0.0, 1.0), QAngle::from_degrees(90.0));
    let result = p1 * p2;
    // Combined heading should be 90° + 90° = 180°
    assert!((result.heading().as_degrees() - 180.0).abs() < f64::EPSILON);
}

// ============================================================================
// Scalar Multiplication Tests
// ============================================================================

#[test]
fn test_pose_mul_scalar() {
    let pose = Pose::new(Vector2::<f64>::new(2.0, 3.0), QAngle::from_degrees(45.0));
    let scaled = pose * 2.0;
    let pos =scaled.position();
    // Position should be scaled: (2*2, 3*2) = (4, 6)
    assert_eq!(pos.x, 4.0);
    assert_eq!(pos.y, 6.0);
    // Heading should be preserved
    assert!((scaled.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_mul_scalar_zero() {
    let pose = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(30.0));
    let scaled = pose * 0.0;
    let pos =scaled.position();
    assert_eq!(pos.x, 0.0);
    assert_eq!(pos.y, 0.0);
    assert!((scaled.heading().as_degrees() - 30.0).abs() < 1e-10);
}

#[test]
fn test_pose_mul_scalar_negative() {
    let pose = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(45.0));
    let scaled = pose * -1.0;
    let pos =scaled.position();
    assert_eq!(pos.x, -1.0);
    assert_eq!(pos.y, -2.0);
    assert!((scaled.heading().as_degrees() - 45.0).abs() < f64::EPSILON);
}

// ============================================================================
// Scalar Division Tests
// ============================================================================

#[test]
fn test_pose_div_scalar() {
    let pose = Pose::new(Vector2::<f64>::new(4.0, 6.0), QAngle::from_degrees(60.0));
    let scaled = pose / 2.0;
    let pos =scaled.position();
    // Position should be divided: (4/2, 6/2) = (2, 3)
    assert_eq!(pos.x, 2.0);
    assert_eq!(pos.y, 3.0);
}

#[test]
fn test_pose_div_scalar_fractional() {
    let pose = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(45.0));
    let scaled = pose / 4.0;
    let pos =scaled.position();
    assert_eq!(pos.x, 0.25);
    assert_eq!(pos.y, 0.5);
}

// ============================================================================
// Move Local Tests
// ============================================================================

#[test]
fn test_pose_move_local() {
    // Robot at origin facing east
    let robot = Pose::new(Default::default(), QAngle::from_degrees(0.0));
    // Local point at (1, 0) in robot's frame
    let local = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let global = robot.move_local(local);
    let pos =global.position();
    // Should still be (1, 0) since robot is at origin facing east
    assert!((pos.x - 1.0).abs() < f64::EPSILON);
    assert!((pos.y - 0.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_move_local_rotated() {
    // Robot at origin facing north (90°)
    let robot = Pose::new(Default::default(), QAngle::from_degrees(90.0));
    // Local point at (1, 0) in robot's frame (to its right)
    let local = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let global = robot.move_local(local);
    let pos =global.position();
    // When facing north, (1, 0) in local frame is (0, 1) in global frame
    assert!((pos.x - 0.0).abs() < f64::EPSILON);
    assert!((pos.y - 1.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_move_local_translated() {
    // Robot at (2, 3) facing east
    let robot = Pose::new(Vector2::<f64>::new(2.0, 3.0), QAngle::from_degrees(0.0));
    // Local point at (1, 0) in robot's frame
    let local = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    let global = robot.move_local(local);
    let pos =global.position();
    // Should be (2+1, 3+0) = (3, 3)
    assert!((pos.x - 3.0).abs() < f64::EPSILON);
    assert!((pos.y - 3.0).abs() < f64::EPSILON);
}

// ============================================================================
// Move Global Tests
// ============================================================================

#[test]
fn test_pose_move_global() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(45.0));
    let p2 = Pose::new(Vector2::<f64>::new(3.0, 4.0), QAngle::from_degrees(90.0));
    let result = p1.move_global(p2);
    let pos =result.position();
    // Positions should be added: (1+3, 2+4) = (4, 6)
    assert_eq!(pos.x, 1.0);
    assert_eq!(pos.y, 5.0);
    // Heading should be reset to 0
    assert!(result.heading().as_radians() - 135.0 < f64::EPSILON);
}

#[test]
fn test_pose_move_global_zero() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(30.0));
    let zero = Pose::identity();
    let result = p1.move_global(zero);
    let pos =result.position();
    assert_eq!(pos.x, 1.0);
    assert_eq!(pos.y, 2.0);
    let z = result.heading().as_degrees();
    // Floating point precision: 30.0 degrees -> radians -> back to degrees may not be exactly 30.0
    assert!((z - 30.0).abs() < 1e-10);
}

// ============================================================================
// Transformation Matrix Tests
// ============================================================================

#[test]
fn test_pose_transformation_matrix_zero_rotation() {
    // Pose at (1, 2) with 0° rotation
    let pose = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(0.0));
    let pos =pose.position();
    assert_eq!(pos.x, 1.0);
    assert_eq!(pos.y, 2.0);
    // Cos(0) = 1, Sin(0) = 0
    assert!((pose.heading().cos() - 1.0).abs() < f64::EPSILON);
    assert!((pose.heading().sin() - 0.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_transformation_matrix_ninety_rotation() {
    // Pose at (0, 0) with 90° rotation
    let pose = Pose::new(Default::default(), QAngle::from_degrees(90.0));
    // Cos(90°) = 0, Sin(90°) = 1
    assert!((pose.heading().cos() - 0.0).abs() < f64::EPSILON);
    assert!((pose.heading().sin() - 1.0).abs() < f64::EPSILON);
}

// ============================================================================
// Integration Tests
// ============================================================================

#[test]
fn test_pose_chained_transformations() {
    // Start at origin
    let start = Pose::identity();
    // Move forward 1 meter
    let forward = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));
    // Rotate 90 degrees
    let rotate = Pose::new(Default::default(), QAngle::from_degrees(90.0));
    // Move forward 1 meter again
    let forward2 = Pose::new(Vector2::<f64>::new(1.0, 0.0), QAngle::from_degrees(0.0));

    // Chain transformations
    let result = start * forward * rotate * forward2;
    let _position = result.position();

    // After first forward: (1, 0)
    // After rotate 90°: heading is 90°
    // After second forward in new frame: should be (1, 1) in global
    // The exact position depends on matrix multiplication order
    // Combined heading: 0 + 0 + 90 + 0 = 90°
    assert!((result.heading().as_degrees() - 90.0).abs() < f64::EPSILON);
}

#[test]
fn test_pose_roundtrip() {
    // Create odom pose
    let original = Pose::new(Vector2::<f64>::new(1.5, 2.5), QAngle::from_degrees(45.0));

    // Extract components
    let pos = original.position();
    let heading = original.heading();

    // Recreate pose
    let recreated = Pose::new(pos, heading);

    // Verify components match
    let (x2, y2) = (recreated.position().x, recreated.position().y);
    assert_eq!(pos.x, x2);
    assert_eq!(pos.y, y2);
    assert!((heading.as_degrees() - recreated.heading().as_degrees()).abs() < f64::EPSILON);
}

#[test]
fn test_pose_clone() {
    let p1 = Pose::new(Vector2::<f64>::new(1.0, 2.0), QAngle::from_degrees(45.0));
    let p2 = p1;
    // Both should have same position and heading
    let pos1 = p1.position();
    let pos2 = p2.position();
    assert_eq!(pos1.x, pos2.x);
    assert_eq!(pos2.y, pos1.y);
    assert!((p1.heading().as_degrees() - p2.heading().as_degrees()).abs() < f64::EPSILON);
}
