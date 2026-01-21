# Kernelvex Examples

This directory contains example code demonstrating how to use the kernelvex library for VEX robotics applications.

## Tracking Wheel Usage

The `tracking_wheel_usage.rs` example shows:

- How to set up single and dual tracking wheel configurations
- Using both the direct constructor and builder pattern APIs
- Common tracking wheel configurations (parallel, perpendicular, geared)
- Using both ADI encoders and rotation sensors
- Diagnostic and debugging techniques
- Integration with continuous control loops

### Building and Running

To build an example for VEX V5:

```bash
cargo build --example tracking_wheel_usage --target thumbv7em-none-eabihf
```

Note: This library is designed for VEX V5 robots and requires the nightly Rust toolchain with the appropriate embedded target.

## Common Tracking Wheel Configurations

### Single Parallel Wheel
- Measures forward/backward motion
- Simple and reliable
- Cannot detect lateral movement

### Dual Parallel Wheels
- Two wheels on opposite sides
- Average readings for better accuracy
- Can detect rotation

### Three-Wheel Odometry
- Two parallel wheels + one perpendicular
- Full 2D position tracking (x, y, heading)
- Most common configuration for competitions

### Geared Tracking Wheels
- External gearing increases resolution
- Useful for high-precision applications
- 2:1 or 3:1 ratios are common

## Tips

1. **Calibration**: Measure your actual wheel diameters - manufacturing tolerances can affect accuracy
2. **Gearing**: Account for any external gearing between encoder and wheel
3. **Direction**: Use `set_reversed()` if encoder reads backwards
4. **Mounting**: Ensure wheels spin freely and have good contact with the field
5. **Update Rate**: Poll tracking wheels at 10-50ms intervals for smooth odometry
