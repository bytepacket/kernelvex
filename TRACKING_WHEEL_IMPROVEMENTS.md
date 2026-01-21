# Tracking Wheel Implementation - Ideas and Improvements

This document summarizes the analysis, bug fixes, and improvements made to the tracking wheel implementation in the kernelvex library.

## Overview

The tracking wheel implementation provides a foundation for odometry calculations in VEX robotics. The implementation includes:

- **Encoder abstraction** (`sensors.rs`) - Unified interface for different encoder types
- **Tracking wheel core** (`omniwheel.rs`) - Main tracking wheel implementation with distance and delta calculations
- **Type-safe units** (`si.rs`) - Compile-time checked physical quantities
- **Pose system** (`pose.rs`) - 2D pose representation for robot position tracking

## Critical Bugs Fixed

### 1. `distance()` Method Accumulation Bug
**Problem:** The `distance()` method was incorrectly accumulating to `self.total` on each call, causing it to return ever-increasing values instead of the current encoder position-based distance.

**Fix:** Removed the accumulation line. The method now correctly calculates and returns the current distance without side effects.

```rust
// Before: ❌
fn distance(&mut self) -> QLength {
    let distance = /* calculation */;
    self.total += distance;  // Wrong! Accumulates on each call
    distance
}

// After: ✅
fn distance(&mut self) -> QLength {
    let distance = /* calculation */;
    distance  // Returns current distance based on encoder position
}
```

### 2. `delta()` Method Logic Error
**Problem:** The delta calculation was inverted (`self.total - distance` instead of `distance - self.total`) and then incorrectly added to total instead of setting it.

**Fix:** Corrected the subtraction order and changed to assignment instead of addition.

```rust
// Before: ❌
fn delta(&mut self) -> QLength {
    let distance = /* calculation */;
    let ret = self.total - distance;  // Wrong order!
    self.total += distance;  // Wrong! Should be assignment
    ret
}

// After: ✅
fn delta(&mut self) -> QLength {
    let distance = /* calculation */;
    let ret = distance - self.total;  // Correct: new - old
    self.total = distance;  // Correct: update to new value
    ret
}
```

### 3. RotationSensor Encoder Unit Mismatch
**Problem:** The `RotationSensor` implementation of the `Encoder` trait was using `as_turns()` inside `from_radians()`, causing a unit mismatch.

**Fix:** Changed to use `as_radians()` for correct unit handling.

```rust
// Before: ❌
impl Encoder for RotationSensor {
    fn rotations(&self) -> QAngle {
        QAngle::from_radians(self.position().unwrap().as_turns())
    }
}

// After: ✅
impl Encoder for RotationSensor {
    fn rotations(&self) -> QAngle {
        QAngle::from_radians(self.position().unwrap().as_radians())
    }
}
```

## Improvements Implemented

### 4. Gearing Ratio Validation
Added validation to ensure gearing ratios are positive values, preventing silent errors from invalid configurations.

```rust
pub fn new(..., gearing: Option<f64>) -> Self {
    if let Some(g) = gearing {
        assert!(g > 0.0, "Gearing ratio must be positive, got {}", g);
    }
    // ...
}
```

### 5. Comprehensive Unit Tests
Added 11 unit tests covering:
- Wheel size calculations for all wheel types
- Tracking wheel creation (left/right orientation)
- Gearing ratio validation (panic tests)
- Distance calculations with and without gearing
- Delta calculations with and without gearing
- Reset functionality
- Verification that `distance()` doesn't accumulate incorrectly

All tests pass successfully.

### 6. Complete Documentation Examples
Added two example files demonstrating proper usage:

- **`examples/odometry_example.rs`** - Complete three-wheel odometry system showing:
  - Setting up multiple tracking wheels
  - Calculating robot position from wheel deltas
  - Handling rotation and translation
  - Compensating for wheel offset during rotation
  
- **`examples/basic_tracking.rs`** - Simple single-wheel usage examples:
  - Basic tracking wheel setup
  - Using geared wheels
  - Multiple wheel configurations

### 7. Documentation Improvements
- Fixed incomplete documentation in the `Tracking` trait
- Added panic documentation for gearing validation
- Improved clarity of docstrings throughout

## Additional Improvement Ideas

### Future Enhancements (Not Yet Implemented)

1. **Error Handling Instead of Panics**
   - Currently, encoder implementations use `unwrap()` which panics on sensor failures
   - Consider making the `Encoder` trait return `Result<QAngle, EncoderError>` for graceful error handling
   - This would allow robots to handle sensor failures without crashing

2. **Orientation Enum Usage**
   - The `Orientation` field in `TrackingWheel` is stored but not actively used
   - Could be used for automatic sign flipping or validation
   - Could help with debugging by indicating which side of the robot a wheel is on

3. **Encoder Reset Support**
   - Add a `reset_encoder()` method to the `Encoder` trait
   - Would allow zeroing encoders without recreating tracking wheels
   - Useful for calibration routines

4. **Velocity Estimation**
   - Add `velocity()` method that estimates wheel velocity from deltas and time
   - Would be useful for velocity-based control loops
   - Could use a simple finite difference or a filtered approach

5. **Filtering and Noise Reduction**
   - Add optional filtering (e.g., moving average, low-pass) for encoder readings
   - Would help reduce noise in velocity calculations
   - Could be implemented as a wrapper type around encoders

6. **Calibration Support**
   - Add methods to calibrate wheel diameter by driving known distances
   - Automatic gearing ratio detection
   - Drift compensation for long-term accuracy

7. **Integration with IMU**
   - Add methods to fuse encoder odometry with IMU data
   - Would provide more accurate heading estimates
   - Could use complementary or Kalman filtering

## Best Practices Learned

1. **Distance vs Delta**: 
   - `distance()` returns the current absolute distance from encoder position
   - `delta()` returns incremental change since last call - use this for odometry updates

2. **Gearing Ratios**: 
   - Specify ratio as encoder rotations per wheel rotation
   - E.g., 2.0 means encoder rotates twice per wheel rotation

3. **Wheel Offset Sign Convention**:
   - Positive offset = right side of robot
   - Negative offset = left side of robot
   - This affects rotation calculations in odometry

4. **Update Frequency**:
   - Call `delta()` frequently (10-20ms) for accurate odometry
   - Slower updates can miss fast movements and cause integration errors

5. **Perpendicular Wheel Offset Correction**:
   - When robot rotates, perpendicular wheel moves in an arc
   - Must subtract `offset × rotation` from perpendicular delta
   - See `examples/odometry_example.rs` for implementation

## Testing

Run the test suite with:
```bash
cargo test --lib
```

All 11 tests should pass:
```
test omniwheel::tests::test_delta_calculation ... ok
test omniwheel::tests::test_delta_with_gearing ... ok
test omniwheel::tests::test_distance_calculation_no_gearing ... ok
test omniwheel::tests::test_distance_calculation_with_gearing ... ok
test omniwheel::tests::test_distance_does_not_accumulate ... ok
test omniwheel::tests::test_negative_gearing_panics - should panic ... ok
test omniwheel::tests::test_omniwheel_sizes ... ok
test omniwheel::tests::test_reset ... ok
test omniwheel::tests::test_tracking_wheel_creation_left ... ok
test omniwheel::tests::test_tracking_wheel_creation_right ... ok
test omniwheel::tests::test_zero_gearing_panics - should panic ... ok
```

## Conclusion

The tracking wheel implementation is now more robust with critical bugs fixed, comprehensive tests added, and clear documentation provided. The implementation provides a solid foundation for odometry calculations in VEX robotics applications.

For complete usage examples, see:
- `examples/odometry_example.rs` - Full odometry system
- `examples/basic_tracking.rs` - Simple usage patterns
