//! Basic tracking wheel example
//!
//! This example shows the simplest way to use a single tracking wheel
//! to measure distance traveled.

#![no_main]
#![no_std]

use kernelvex::{
    omniwheel::{OmniWheel, Tracking, TrackingWheel},
    si::QLength,
};
use vexide_devices::adi::encoder::AdiEncoder;

// Example: Create and use a single tracking wheel
pub fn simple_tracking_example() {
    // Initialize the encoder (this would come from your hardware setup)
    // let encoder = AdiEncoder::new(...);
    
    // Create a tracking wheel using a 2.75" omni wheel
    // positioned 5 inches from the robot's center
    // let mut tracking_wheel = TrackingWheel::new(
    //     encoder,
    //     OmniWheel::Omni275,
    //     QLength::from_inches(5.0),
    //     None, // No gearing (1:1)
    // );
    
    // Get the total distance traveled
    // let total_distance = tracking_wheel.distance();
    // println!("Total distance: {:.2} inches", total_distance.as_inches());
    
    // In a control loop, get incremental changes:
    // loop {
    //     let delta = tracking_wheel.delta();
    //     println!("Moved {:.3} inches since last check", delta.as_inches());
    //     
    //     // Process the delta for odometry calculations
    //     
    //     vexide_devices::sleep(vexide_devices::time::Duration::from_millis(10));
    // }
}

// Example: Tracking wheel with gearing
pub fn geared_tracking_example() {
    // If your tracking wheel has a gear ratio (encoder doesn't rotate 1:1 with wheel),
    // specify the ratio. For example, if the encoder rotates twice per wheel rotation:
    
    // let mut geared_wheel = TrackingWheel::new(
    //     encoder,
    //     OmniWheel::Omni325,
    //     QLength::from_inches(6.0),
    //     Some(2.0), // 2:1 gearing ratio
    // );
    
    // The tracking wheel will automatically account for the gearing when
    // calculating distances
}

// Example: Multiple tracking wheels for different purposes
pub fn multi_wheel_example() {
    // Left side tracking wheel (negative offset)
    // let mut left_wheel = TrackingWheel::new(
    //     left_encoder,
    //     OmniWheel::Omni275,
    //     QLength::from_inches(-5.0),
    //     None,
    // );
    
    // Right side tracking wheel (positive offset)
    // let mut right_wheel = TrackingWheel::new(
    //     right_encoder,
    //     OmniWheel::Omni275,
    //     QLength::from_inches(5.0),
    //     None,
    // );
    
    // In a control loop:
    // loop {
    //     let left_delta = left_wheel.delta();
    //     let right_delta = right_wheel.delta();
    //     
    //     // Calculate robot movement from both wheels
    //     let forward_movement = (left_delta + right_delta) / 2.0;
    //     let rotation = (right_delta - left_delta) / QLength::from_inches(10.0); // wheel base
    //     
    //     println!("Forward: {:.2} inches, Rotation: {:.4} rad",
    //         forward_movement.as_inches(), rotation.as_meters());
    //     
    //     vexide_devices::sleep(vexide_devices::time::Duration::from_millis(10));
    // }
}
