//! Tracking wheel implementation for odometry.
//!
//! This module provides support for tracking wheels used in odometry calculations,
//! allowing robots to track their position and movement using wheel encoders.
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
    fn size(&self) -> QLength {
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

    pub const fn offset(&self) -> QLength {
        match self.dist {
            TrackingWheelOrientation::Vertical(v) => v,
            TrackingWheelOrientation::Horizontal(v) => v,
        }
    }

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
    pub fn delta(&mut self) -> QLength {
        let previous = self.total;
        let current = self.distance();
        self.total = current;
        current - previous
    }

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

    pub const fn orientation(&self) -> Orientation {
        self.orientation
    }

    pub const fn direction(&self) -> TrackingWheelOrientation {
        self.dist
    }
}

// based off evian
pub struct TrackingRig {
    data: Rc<RefCell<TrackingData>>,
    _task: Task<()>,
}

impl TrackingRig {
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
    pub fn pose(&self) -> Pose {
        self.data.borrow().pose
    }

    /// Returns the latest linear velocity estimate in meters per second.
    pub fn linear_velocity(&self) -> f64 {
        self.data.borrow().linear_velocity
    }

    /// Returns the latest angular velocity estimate in radians per second.
    pub fn angular_velocity(&self) -> f64 {
        self.data.borrow().angular_velocity
    }

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

#[derive(Debug, Clone, Copy)]
struct TrackingData {
    pose: Pose,
    raw_heading: QAngle,
    heading_offset: QAngle,
    forward_travel: f64,
    linear_velocity: f64,
    angular_velocity: f64,
}

#[derive(Debug, Clone, Copy)]
enum HeadingError {
    Imu(Option<QAngle>),
    Rotary,
}


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
