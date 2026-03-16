//! Tracking wheel implementation for odometry.
//!
//! This module provides support for tracking wheels used in odometry calculations,
//! allowing robots to track their position and movement using wheel encoders.
//!
//! # Overview
//!
//! Tracking wheels are unpowered wheels with encoders that measure distance traveled.
//! By using multiple tracking wheels at known offsets from the robot center, the
//! robot's position and heading can be calculated.
//!
//! # Components
//!
//! - [`TrackingWheel`]: A single wheel with encoder for distance measurement
//! - [`TrackingRig`]: A complete odometry system with multiple wheels and optional IMU
//! - [`OmniWheel`]: Enum of standard VEX wheel sizes
//! - [`Encoder`]: Either ADI quadrature or V5 rotation sensor
//!
//! # Tracking Rig
//!
//! The [`TrackingRig`] runs a background task that continuously updates the robot's
//! pose estimate. It supports:
//! - IMU for heading (preferred)
//! - Two parallel forward wheels for heading (fallback)
//! - Horizontal wheels for lateral movement detection
//!
//! # Examples
//!
//! ```no_run
//! use kernelvex::odom::wheel::{Encoder, OmniWheel, TrackingWheel};
//! use kernelvex::util::si::QLength;
//! use vexide_devices::math::Direction;
//! use vexide_devices::smart::SmartPort;
//! use vexide_devices::smart::rotation::RotationSensor;
//! use kernelvex::util::utils::TrackingWheelOrientation;
//!
//! let encoder = RotationSensor::new(unsafe {SmartPort::new(1)}, Direction::Forward);
//!
//! let mut tracking_wheel = TrackingWheel::new(
//!     Encoder::Smart(encoder),
//!     OmniWheel::Omni275,
//!     TrackingWheelOrientation::Vertical(QLength::from_inches(5.)),
//!     Some(1.0), // gearing ratio
//! );
//!
//! let distance = tracking_wheel.distance();
//! println!("Distance traveled: {} inches", distance.as_inches());
//! ```

use crate::odom::pose::Pose;
use crate::odom::wheel::Encoder::{Adi, Smart};
use crate::util::si::QLength;
use crate::util::utils::{TrackingWheelOrientation, Orientation};
use crate::{QAngle, Vector2};
use std::cell::RefCell;
use std::rc::Rc;
use std::time::{Duration, Instant};
use vexide_async::task::{spawn, Task};
use vexide_async::time::sleep;
use vexide_devices::adi::encoder::AdiEncoder;
use vexide_devices::smart::imu::InertialSensor;
use vexide_devices::smart::rotation::RotationSensor;
use heapless::Vec;

#[derive(Debug)]
pub enum Encoder {
    Adi(AdiEncoder<360>),
    Smart(RotationSensor),
}

/// Types of omni wheels available for tracking.
///
/// Omni wheels come in different sizes (diameter). The size affects the
/// distance calculation when converting encoder rotations to linear distance.
#[derive(Debug, Clone, Copy, PartialEq)]
#[allow(unused)]
pub enum OmniWheel {
    /// 2.75-inch omni wheel
    Omni275,
    /// 3.25-inch omni wheel
    Omni325,
    /// 4.125-inch omni wheel
    Omni4,
    /// 2.75-inch anti-static wheel
    Anti275,
    /// 3.25-inch anti-static wheel
    Anti325,
    /// 4-inch anti-static wheel
    Anti4,
    /// Custom Length wheel
    Custom(QLength),
}

impl OmniWheel {
    /// Returns the diameter of the wheel.
    ///
    /// # Returns
    ///
    /// The wheel diameter as [`QLength`].
    #[allow(unused)]
    pub(crate) fn size(&self) -> QLength {
        match *self {
            OmniWheel::Omni275 => QLength::from_inches(2.75),
            OmniWheel::Omni325 => QLength::from_inches(3.25),
            OmniWheel::Omni4 => QLength::from_inches(4.125),
            OmniWheel::Anti275 => QLength::from_inches(2.75),
            OmniWheel::Anti325 => QLength::from_inches(3.25),
            OmniWheel::Anti4 => QLength::from_inches(4.),
            OmniWheel::Custom(d) => d,
        }
    }
}

/// A tracking wheel implementation using an encoder.
///
/// `TrackingWheel` converts encoder rotations into linear distance measurements
/// using the wheel diameter and optional gearing ratio. This is the primary
/// implementation of the [`Tracking`] trait for odometry calculations.
///
/// # Type Parameters
///
/// * `T` - The encoder type implementing the [`Encoder`] trait
///

#[derive(Debug)]
pub struct TrackingWheel {
    encoder: Encoder,
    wheel: OmniWheel,
    dist: TrackingWheelOrientation,
    orientation: Orientation,
    total: QLength,
    gearing: f64,
}

impl TrackingWheel {
    /// Creates odom new tracking wheel.
    ///
    /// # Arguments
    ///
    /// * `encoder` - The encoder used to measure wheel rotation
    /// * `wheel` - The type of omni wheel being used
    /// * `dist` - The perpendicular offset distance from the robot's center.
    ///            Positive values indicate right side, negative indicates left side
    /// * `gearing` - Optional gearing ratio. If `None`, assumes 1:1 gearing.
    ///              A value of 2.0 means the encoder rotates twice per wheel rotation.
    ///
    /// # Returns
    ///
    /// A new `TrackingWheel` instance with the encoder reset to zero.
    #[allow(unused)]
    #[inline]
    pub fn new(
        encoder: Encoder,
        wheel: OmniWheel,
        dist: TrackingWheelOrientation,
        ratio: Option<f64>,
    ) -> Self {
        let gearing: f64 = ratio.unwrap_or(1.);

        let _v = match dist {
            TrackingWheelOrientation::Vertical(v) => v,
            TrackingWheelOrientation::Horizontal(v) => v,
        };

        if _v.as_meters() > 0. {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Right,
                total: Default::default(),
                gearing,
            }
        } else {
            TrackingWheel {
                encoder,
                wheel,
                dist,
                orientation: Orientation::Left,
                total: Default::default(),
                gearing,
            }
        }
    }

    /// Returns the perpendicular offset distance from the robot's center.
    ///
    /// This is the distance from the robot's center of rotation to the wheel,
    /// measured perpendicular to the wheel's rolling direction.
    ///
    /// # Returns
    ///
    /// The offset distance as [`QLength`]. Positive values indicate right side
    /// or forward offset, negative values indicate left side or backward offset.
    pub const fn offset(&self) -> QLength {
        match self.dist {
            TrackingWheelOrientation::Vertical(v) => v,
            TrackingWheelOrientation::Horizontal(v) => v,
        }
    }

    /// Returns the total distance traveled by this wheel since the last reset.
    ///
    /// Calculates distance using the formula:
    /// ```text
    /// distance = (circumference * rotations_in_radians) / gearing
    /// ```
    ///
    /// Where:
    /// - `circumference = wheel_diameter * π`
    /// - `rotations_in_radians` is the encoder position converted to radians
    /// - `gearing` is the gear ratio between encoder and wheel
    ///
    /// # Returns
    ///
    /// The total distance traveled as [`QLength`].
    ///
    /// # Panics
    ///
    /// Panics if the encoder position cannot be read.
    pub fn distance(&self) -> QLength {
        let circumference = self.wheel.size() * std::f64::consts::PI;

        let rotations = match &self.encoder {
            Adi(encoder) => {
                QAngle::from_turns(encoder.position().unwrap().as_turns())
            }
            Smart(encoder) => {
                QAngle::from_turns(encoder.position().unwrap().as_turns())
            }
        };

        let distance =
            (circumference * std::f64::consts::PI * rotations.as_radians()) / self.gearing;

        distance
    }

    /// Returns the distance traveled since the last call to `delta`.
    ///
    /// This method tracks the cumulative distance internally and returns
    /// the difference from the previous reading. Useful for calculating
    /// incremental position changes in odometry loops.
    ///
    /// # Returns
    ///
    /// The distance traveled since the last call as [`QLength`].
    ///
    /// # Note
    ///
    /// The first call after construction or [`reset`](Self::reset) will return
    /// the total distance, as the previous value is initialized to zero.
    pub fn delta(&mut self) -> QLength {
        let previous = self.total;
        let current = self.distance();
        self.total = current;
        current - previous
    }

    /// Resets the encoder position and internal tracking state to zero.
    ///
    /// This clears both the encoder's position register and the internal
    /// cumulative distance tracker used by [`delta`](Self::delta).
    ///
    /// Call this method when repositioning the robot or starting a new
    /// autonomous routine.
    pub fn reset(&mut self) {
        self.total = Default::default();
        match &mut self.encoder {
            Adi(encoder) => {
                let _ = encoder.reset_position();
            }
            Smart(encoder) => {
                let _ = encoder.reset_position();
            }
        }
    }

    /// Sets the encoder position to a specific angle.
    ///
    /// This directly sets the encoder's position register without affecting
    /// the internal tracking state. Useful for calibration or synchronization.
    ///
    /// # Arguments
    ///
    /// * `position` - The angle to set the encoder position to.
    pub fn set(&mut self, position: QAngle) {
        match &mut self.encoder {
            Adi(encoder) => {
                let _ = encoder.set_position(position.into());
            }
            Smart(encoder) => {
                let _ = encoder.set_position(position.into());
            }
        }
    }

    /// Returns the orientation of this tracking wheel.
    ///
    /// The orientation is determined by the sign of the offset distance:
    /// - Positive offset → [`Orientation::Right`]
    /// - Negative offset → [`Orientation::Left`]
    ///
    /// # Returns
    ///
    /// The [`Orientation`] of this wheel (Left or Right).
    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    /// Returns the tracking wheel orientation (vertical or horizontal).
    ///
    /// Vertical wheels measure forward/backward movement, while horizontal
    /// wheels measure lateral (sideways) movement.
    ///
    /// # Returns
    ///
    /// The [`TrackingWheelOrientation`] including the offset distance.
    pub const fn direction(&self) -> TrackingWheelOrientation {
        self.dist
    }
}

/// A complete odometry tracking system with multiple wheels and optional IMU.
///
/// `TrackingRig` runs a background task that continuously updates the robot's
/// pose estimate using tracking wheel data and optional IMU heading. It supports:
///
/// - **IMU heading** (preferred): Uses an inertial sensor for accurate heading
/// - **Wheel-based heading** (fallback): Uses two parallel forward wheels
/// - **Horizontal wheels**: For detecting lateral (sideways) movement
///
/// # Architecture
///
/// The tracking rig spawns an asynchronous task that runs at approximately 100Hz
/// (10ms intervals). This task:
/// 1. Reads encoder positions from all tracking wheels
/// 2. Computes heading from IMU or wheel differential
/// 3. Calculates local displacement using arc-based odometry
/// 4. Transforms local displacement to field coordinates
/// 5. Updates the shared pose estimate
///
/// # Odometry Algorithm
///
/// The algorithm uses arc-based position tracking:
/// - When turning, the robot follows an arc rather than a straight line
/// - The chord length formula `2 * sin(Δθ/2)` accounts for this arc
/// - Local coordinates are rotated by the average heading to get field coordinates
///
/// # Example
///
/// ```no_run
/// use kernelvex::odom::wheel::{TrackingRig, TrackingWheel, Encoder, OmniWheel};
/// use kernelvex::odom::pose::Pose;
/// use kernelvex::util::utils::TrackingWheelOrientation;
/// use kernelvex::util::si::QLength;
///
/// // Create tracking wheels
/// // let left_wheel = TrackingWheel::new(...);
/// // let right_wheel = TrackingWheel::new(...);
/// // let horizontal_wheel = TrackingWheel::new(...);
///
/// // Create tracking rig with IMU
/// // let rig = TrackingRig::new(
/// //     Pose::default(),
/// //     [horizontal_wheel],  // horizontal wheels
/// //     [left_wheel, right_wheel],  // vertical wheels
/// //     Some(imu),
/// // );
///
/// // Get current pose
/// // let pose = rig.pose();
/// ```
// based off evian
pub struct TrackingRig {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl TrackingRig {
    /// Creates a new tracking rig and starts the background odometry task.
    ///
    /// # Arguments
    ///
    /// * `origin` - The initial pose of the robot (position and heading)
    /// * `horizontal` - Array of horizontal tracking wheels (measure lateral movement).
    ///                  Maximum of 2 wheels allowed.
    /// * `vertical` - Array of vertical tracking wheels (measure forward/backward movement).
    ///                Maximum of 2 wheels allowed.
    /// * `imu` - Optional inertial sensor for heading. If `None`, two parallel forward
    ///           wheels are required to compute heading from wheel differential.
    ///
    /// # Type Parameters
    ///
    /// * `N` - Number of horizontal tracking wheels (0-2)
    /// * `U` - Number of vertical tracking wheels (0-2)
    ///
    /// # Panics
    ///
    /// Panics if:
    /// - More than 2 horizontal or vertical tracking wheels are provided
    /// - No IMU is provided AND no pair of parallel forward wheels exists
    ///
    /// # Returns
    ///
    /// A new `TrackingRig` that immediately begins updating pose estimates.
    #[inline]

    pub fn new<const N: usize, const U: usize>(
        origin: Pose,
        horizontal: [TrackingWheel; N],
        vertical: [TrackingWheel; U],
        imu: Option<InertialSensor>,
    ) -> Self {

        const {
            assert!(N <= 2 || U <= 2 , "cannot have over 2 tracking wheels each");
        }

        let mut h_wheels: Vec<TrackingWheel, 2> = Vec::from_array(horizontal);


        let mut v_wheels: Vec<TrackingWheel, 2> = Vec::from_array(vertical);
 

        let parallel_indices = find_parallel_forward_indices(&h_wheels);


            assert!(
                imu.is_some() || parallel_indices.is_some(),
                "gyro or two parallel forward wheels are required to determine heading"
            );

        let initial_heading = compute_raw_heading(imu.as_ref(), parallel_indices.as_ref(), &mut h_wheels[..])
            .unwrap_or_default();
        let initial_forward: Vec<f64, 2> = v_wheels
            .iter_mut()
            .map(|wheel| wheel.distance().as_meters())
            .collect();
        let initial_sideways: f64 = h_wheels
            .iter_mut()
            .map(|wheel| wheel.distance().as_meters())
            .sum();

        let initial_forward_travel = if initial_forward.is_empty() { 0.0 } else { initial_forward.iter().sum::<f64>() / initial_forward.len() as f64 };

        let data = Rc::new(RefCell::new(TrackingData {
            pose: origin,
            raw_heading: initial_heading,
            heading_offset: origin.heading(),
            forward_travel: initial_forward_travel,
            linear_velocity: 0.0,
            angular_velocity: 0.0,
        }));

        let task_data = Rc::clone(&data);


        let task = spawn(async move {
            Self::task(
                &mut v_wheels[..],
                &mut h_wheels[..],
                imu,
                task_data,
                parallel_indices,
                Vec::from_slice(&initial_forward).unwrap(),
                Vec::from_slice(&[initial_sideways]).unwrap(),
                initial_heading,
                initial_forward_travel,
            ).await;
        });

        Self { data, _task: task}
    }

    /// Returns the latest pose estimate.
    ///
    /// The pose is continuously updated by the background task at approximately
    /// 100Hz. This method returns the most recent estimate.
    ///
    /// # Returns
    ///
    /// The current [`Pose`] estimate (position and heading).
    pub fn pose(&self) -> Pose {
        self.data.borrow().pose
    }

    /// Returns the latest linear velocity estimate in meters per second.
    ///
    /// Computed from the change in forward wheel travel over time.
    ///
    /// # Returns
    ///
    /// Linear velocity in m/s. Positive values indicate forward movement.
    pub fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }

    /// Returns the latest angular velocity estimate in radians per second.
    ///
    /// If an IMU is available, uses the gyroscope's Z-axis rate directly.
    /// Otherwise, computes from the change in heading over time.
    ///
    /// # Returns
    ///
    /// Angular velocity in rad/s. Positive values indicate counter-clockwise rotation.
    pub fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }

    /// Background odometry task that continuously updates the pose estimate.
    ///
    /// This async task runs in a loop at approximately 100Hz and:
    /// 1. Reads current positions from all tracking wheels
    /// 2. Computes heading change from IMU or wheel differential
    /// 3. Calculates local displacement using arc-based odometry
    /// 4. Transforms to field coordinates and updates the shared pose
    ///
    /// # Arc-Based Odometry
    ///
    /// When the robot turns, it follows an arc rather than a straight line.
    /// The algorithm uses the chord length formula:
    /// ```text
    /// unit_chord = 2 * sin(Δθ / 2)
    /// ```
    ///
    /// Local displacement is computed as:
    /// ```text
    /// local_x = unit_chord * (Δforward / Δθ + offset)  // forward direction
    /// local_y = unit_chord * (Δsideways / Δθ + offset) // lateral direction
    /// ```
    ///
    /// When `Δθ ≈ 0`, the robot moved straight and the formula simplifies to
    /// just using the raw delta values.
    ///
    /// # Field Coordinate Transformation
    ///
    /// Local coordinates are transformed to field coordinates using:
    /// ```text
    /// dx_field = local_x * cos(avg_heading) - local_y * sin(avg_heading)
    /// dy_field = local_x * sin(avg_heading) + local_y * cos(avg_heading)
    /// ```
    ///
    /// Where `avg_heading` is the midpoint heading during the movement.
    #[allow(clippy::too_many_arguments)]
    async fn task(
        forward: &mut [TrackingWheel],
        sideways: &mut [TrackingWheel],
        mut imu: Option<InertialSensor>,
        data: Rc<RefCell<TrackingData>>,
        parallel_indices: Option<(usize, usize)>,
        mut prev_forward: Vec<f64, 2>,
        mut prev_sideways: Vec<f64, 2>,
        mut prev_raw_heading: QAngle,
        mut prev_forward_travel: f64,
    ) {
        let mut prev_time = Instant::now();

        loop {
            sleep(Duration::from_millis(10)).await;

        let forward_data: Vec<(f64, f64), 2> = forward
            .iter_mut()
            .map(|wheel| (wheel.distance().as_meters(), wheel.offset().as_meters()))
            .collect();
        let sideways_data: Vec<(f64, f64), 2> = sideways
            .iter_mut()
            .map(|wheel| (wheel.distance().as_meters(), wheel.offset().as_meters()))
            .collect();

        let raw_heading = match compute_raw_heading(imu.as_ref(), parallel_indices.as_ref(), forward) {
                Ok(heading) => heading,
                Err(HeadingError::Imu(fallback)) => {
                    imu = None;
                    if let Some(fallback_heading) = fallback {
                        fallback_heading
                    } else {
                        return;
                    }
                }
                Err(HeadingError::Rotary) if imu.is_some() => {
                    imu = None;
                    continue;
                }
                Err(_) => continue,
            };

            let delta_heading = (raw_heading - prev_raw_heading).remainder(QAngle::TAU);
            let avg_heading = raw_heading + delta_heading * 0.5 + data.borrow().heading_offset;
            prev_raw_heading = raw_heading;

            let unit_chord = 2.0 * libm::sin(delta_heading.as_radians() / 2.0);

            let mut local_x_sum = 0.0;
            let mut local_y_sum = 0.0;
            let mut forward_count = 0.0;
            let mut sideways_count = 0.0;
            let mut travel_sum = 0.0;

            for (i, (travel, offset)) in forward_data.iter().enumerate() {
                let delta = *travel - prev_forward[i];
                travel_sum += *travel;
                forward_count += 1.0;

                local_x_sum += if delta_heading.as_radians() == 0.0 {
                    delta
                } else {
                    unit_chord * (delta / delta_heading.as_radians() + offset)
                };

                prev_forward[i] = *travel;
            }

            for (i, (travel, offset)) in sideways_data.iter().enumerate() {
                let delta = *travel - prev_sideways[i];
                sideways_count += 1.0;

                local_y_sum += if delta_heading.as_radians() == 0.0 {
                    delta
                } else {
                    unit_chord * (delta / delta_heading.as_radians() + offset)
                };

                prev_sideways[i] = *travel;
            }

            let local_x = if forward_count > 0.0 {
                local_x_sum / forward_count
            } else {
                0.0
            };
            let local_y = if sideways_count > 0.0 {
                local_y_sum / sideways_count
            } else {
                0.0
            };

            let dt = prev_time.elapsed().as_secs_f64();
            prev_time = Instant::now();

            let forward_travel = if forward_count > 0.0 {
                travel_sum / forward_count
            } else {
                prev_forward_travel
            };

            let linear_velocity = if dt > 0.0 {
                (forward_travel - prev_forward_travel) / dt
            } else {
                0.0
            };
            prev_forward_travel = forward_travel;

            let angular_velocity = if let Some(imu_ref) = imu.as_ref() {
                imu_ref.gyro_rate().ok().map(|v| v.z.to_radians()).unwrap_or(0.0)
            } else if dt > 0.0 {
                delta_heading.as_radians() / dt
            } else {
                0.0
            };

            let dx_field = local_x * libm::cos(avg_heading.as_radians())
                - local_y * libm::sin(avg_heading.as_radians());
            let dy_field = local_x * libm::sin(avg_heading.as_radians())
                + local_y * libm::cos(avg_heading.as_radians());

            let mut state = data.borrow_mut();
            let (x, y) = (state.pose.position().x, state.pose.position().y);
            state.pose = Pose::new(
                Vector2::<f64>::new(x + dx_field,
                y + dy_field),
                raw_heading + state.heading_offset,
            );
            state.raw_heading = raw_heading;
            state.forward_travel = forward_travel;
            state.linear_velocity = linear_velocity;
            state.angular_velocity = angular_velocity;
        }
    }
}

/// Internal state for the tracking rig's background task.
///
/// This structure is shared between the main thread and the odometry task
/// via `Rc<RefCell<TrackingData>>`.
#[derive(Debug, Clone, Copy)]
struct TrackingData {
    /// Current pose estimate (position and heading)
    pose: Pose,
    /// Raw heading from IMU or wheel differential (without offset)
    raw_heading: QAngle,
    /// Offset applied to raw heading to get final heading
    heading_offset: QAngle,
    /// Cumulative forward travel distance (meters)
    forward_travel: f64,
    /// Current linear velocity (m/s)
    linear_velocity: f64,
    /// Current angular velocity (rad/s)
    angular_velocity: f64,
}

/// Error type for heading computation failures.
///
/// Used internally to handle fallback from IMU to wheel-based heading.
#[derive(Debug, Clone, Copy)]
enum HeadingError {
    /// IMU read failed; contains optional fallback heading from wheels
    Imu(Option<QAngle>),
    /// Rotation sensor read failed
    Rotary,
}


/// Finds indices of two parallel forward wheels suitable for heading calculation.
///
/// Two wheels are considered "parallel" if their offsets are approximately
/// symmetric about the robot center (i.e., `offset_i + offset_j ≈ 0`).
///
/// # Arguments
///
/// * `forward` - Slice of forward-facing tracking wheels
///
/// # Returns
///
/// `Some((left_index, right_index))` if a parallel pair is found, where the
/// left wheel has the smaller (more negative) offset. Returns `None` if no
/// suitable pair exists.
///
/// # Tolerance
///
/// Uses a tolerance of 0.5 meters for symmetry check.
fn find_parallel_forward_indices(forward: &Vec<TrackingWheel, 2>) -> Option<(usize, usize)> {
    const OFFSET_TOLERANCE: f64 = 0.5;
    let n = forward.len();
    if n < 2 {
        return None;
    }

    for i in 0..n {
        for j in (i + 1)..n {
            let i_offset = forward[i].offset().as_meters();
            let j_offset = forward[j].offset().as_meters();
            if (i_offset + j_offset).abs() <= OFFSET_TOLERANCE {
                return Some(if i_offset < j_offset { (i, j) } else { (j, i) });
            }
        }
    }

    None
}

/// Computes the robot's raw heading from available sensors.
///
/// Attempts to read heading in the following priority order:
/// 1. IMU heading (if available and working)
/// 2. Wheel differential heading (if parallel wheels exist)
///
/// # Arguments
///
/// * `imu` - Optional reference to an inertial sensor
/// * `parallel_indices` - Optional indices of parallel forward wheels
/// * `forward` - Mutable slice of forward tracking wheels
///
/// # Returns
///
/// * `Ok(heading)` - Successfully computed heading
/// * `Err(HeadingError::Imu(fallback))` - IMU failed; contains optional wheel-based fallback
/// * `Err(HeadingError::Rotary)` - Rotation sensor failed
///
/// # Note
///
/// IMU heading is negated to match the convention (positive = counter-clockwise).
fn compute_raw_heading (
    imu: Option<&InertialSensor>,
    parallel_indices: Option<&(usize, usize)>,
    forward: &mut [TrackingWheel],
) -> Result<QAngle, HeadingError> {
    if let Some(imu_ref) = imu {
        return match imu_ref.heading() {
            Ok(heading) => Ok(QAngle::from_degrees(heading.as_degrees() * -1.0)),
            Err(_) => {
                Err(HeadingError::Imu(parallel_indices.and_then(|(l, r)| {
                    wheel_heading(forward, *l, *r)
                })))
            }
        }
    }

    if let Some((left, right)) = parallel_indices {
        if let Some(heading) = wheel_heading(forward, *left, *right) {
            return Ok(heading);
        }
        return Err(HeadingError::Rotary);
    }

    Err(HeadingError::Imu(None))
}

/// Computes heading from the differential of two parallel tracking wheels.
///
/// Uses the formula:
/// ```text
/// heading = (right_travel - left_travel) / track_width
/// ```
///
/// Where `track_width` is the distance between the two wheels.
///
/// # Arguments
///
/// * `forward` - Mutable slice of forward tracking wheels
/// * `left_index` - Index of the left wheel (negative offset)
/// * `right_index` - Index of the right wheel (positive offset)
///
/// # Returns
///
/// `Some(heading)` if computation succeeds, `None` if indices are invalid
/// or track width is zero.
fn wheel_heading (
    forward: &mut [TrackingWheel],
    left_index: usize,
    right_index: usize,
) -> Option<QAngle> {
    if left_index >= forward.len() || right_index >= forward.len() {
        return None;
    }

    let left_offset = forward[left_index].offset();
    let right_offset = forward[right_index].offset();
    let track_width = (right_offset - left_offset).as_meters().abs();
    if track_width == 0.0 {
        return None;
    }

    let left_travel = forward[left_index].distance().as_meters();
    let right_travel = forward[right_index].distance().as_meters();

    Some(QAngle::from_radians((right_travel - left_travel) / track_width))
}
