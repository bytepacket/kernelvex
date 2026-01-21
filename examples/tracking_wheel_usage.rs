//! Example showing common tracking wheel configurations for VEX odometry.
//!
//! This example demonstrates how to set up tracking wheels for different
//! robot configurations commonly used in VEX competitions.

#![no_std]
#![no_main]

use kernelvex::omniwheel::{OmniWheel, Tracking, TrackingWheel, TrackingWheelBuilder};
use kernelvex::si::QLength;
use vexide_devices::prelude::*;

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // === Single Tracking Wheel Configuration ===
    // A simple parallel tracking wheel (perpendicular to forward motion)
    // mounted on the right side of the robot
    
    let encoder_right = peripherals.port_1.into_adi().into_encoder(
        peripherals.port_2.into_adi()
    ).expect("Failed to create encoder");
    
    let mut parallel_wheel = TrackingWheel::new(
        encoder_right,
        OmniWheel::Omni275,              // 2.75" tracking wheel
        QLength::from_inches(5.0),        // 5" to the right of center
        None,                             // 1:1 gearing (no external gearing)
    );

    // === Using the Builder Pattern ===
    // The builder pattern provides a more ergonomic API, especially
    // when you need to set multiple options
    
    let encoder_builder_example = peripherals.port_9.into_adi().into_encoder(
        peripherals.port_10.into_adi()
    ).expect("Failed to create encoder");
    
    let mut builder_wheel = TrackingWheelBuilder::new(encoder_builder_example)
        .wheel_type(OmniWheel::Omni325)
        .offset(QLength::from_inches(6.0))
        .gearing(1.5)
        .reversed(false)
        .build();

    // Read distance traveled
    let distance = parallel_wheel.distance();
    println!("Distance traveled: {} inches", distance.as_inches());

    // Get incremental change
    let delta = parallel_wheel.delta();
    println!("Change since last reading: {} inches", delta.as_inches());

    // === Dual Tracking Wheel Configuration ===
    // Two parallel wheels for better accuracy and redundancy
    
    let encoder_left = peripherals.port_3.into_adi().into_encoder(
        peripherals.port_4.into_adi()
    ).expect("Failed to create encoder");
    
    let mut left_parallel = TrackingWheel::new(
        encoder_left,
        OmniWheel::Omni325,               // 3.25" tracking wheel
        QLength::from_inches(-6.0),       // 6" to the left of center (negative)
        None,
    );

    // Average readings from both wheels for better accuracy
    let avg_distance = (parallel_wheel.distance() + left_parallel.distance()) / 2.0;
    println!("Average distance: {} inches", avg_distance.as_inches());

    // === Perpendicular Tracking Wheel ===
    // A perpendicular wheel (parallel to forward motion) for strafing detection
    
    let encoder_perp = peripherals.port_5.into_adi().into_encoder(
        peripherals.port_6.into_adi()
    ).expect("Failed to create encoder");
    
    let mut perpendicular_wheel = TrackingWheel::new(
        encoder_perp,
        OmniWheel::Omni275,
        QLength::from_inches(7.0),        // Offset from center
        None,
    );

    let strafe_distance = perpendicular_wheel.distance();
    println!("Strafe distance: {} inches", strafe_distance.as_inches());

    // === Geared Tracking Wheel Configuration ===
    // External gearing can be used for higher resolution
    // For example, a 2:1 ratio means the encoder turns twice per wheel rotation
    
    let encoder_geared = peripherals.port_7.into_adi().into_encoder(
        peripherals.port_8.into_adi()
    ).expect("Failed to create encoder");
    
    let mut geared_wheel = TrackingWheel::new(
        encoder_geared,
        OmniWheel::Omni4,                 // 4.125" tracking wheel
        QLength::from_inches(5.5),
        Some(2.0),                        // 2:1 gearing ratio
    );

    // === Using Rotation Sensors Instead of Encoders ===
    // Rotation sensors provide higher resolution than ADI encoders
    
    let rotation_sensor = RotationSensor::new(peripherals.port_9);
    
    let mut rotation_wheel = TrackingWheel::new(
        rotation_sensor,
        OmniWheel::Anti275,               // Anti-static wheel
        QLength::from_inches(-5.0),
        Some(1.0),
    );

    // === Diagnostic Information ===
    // Get diagnostic info about the tracking wheel configuration
    
    println!("\n=== Tracking Wheel Diagnostics ===");
    println!("Orientation: {:?}", parallel_wheel.orientation());
    println!("Wheel type: {:?}", parallel_wheel.wheel_type());
    println!("Gearing ratio: {}", parallel_wheel.gearing());
    println!("Raw encoder angle: {} degrees", parallel_wheel.encoder_angle().as_degrees());
    println!("Offset from center: {} inches", parallel_wheel.offset().as_inches());

    // === Resetting Tracking Wheels ===
    // Reset the accumulated distance when needed (e.g., at start of autonomous)
    parallel_wheel.reset();
    left_parallel.reset();
    perpendicular_wheel.reset();
    
    println!("\nTracking wheels reset!");

    // === Continuous Tracking Loop ===
    // Typical usage in a control loop
    loop {
        // Get incremental changes since last loop iteration
        let forward_delta = parallel_wheel.delta();
        let strafe_delta = perpendicular_wheel.delta();
        
        // Use these deltas to update robot pose/position
        // (In a real application, you'd integrate with pose tracking)
        
        println!("Forward: {:.3} in, Strafe: {:.3} in", 
                 forward_delta.as_inches(), 
                 strafe_delta.as_inches());
        
        // Update at ~10ms interval (typical for odometry)
        vexide_devices::smart::time::sleep(core::time::Duration::from_millis(10)).await;
    }
}
