//! Trajectory representation and sampling utilities.
//!
//! This module provides data structures for representing time-parameterized
//! trajectories and utilities for generating them from curves like Bezier splines.
//!
//! # Overview
//!
//! A trajectory is a sequence of time-indexed points, each containing:
//! - **Pose**: Position (x, y) and heading
//! - **Linear velocity**: Forward speed in m/s
//! - **Angular velocity**: Rotational speed in rad/s
//! - **Time**: Timestamp from trajectory start
//!
//! Trajectories are used by controllers like RAMSETE and Pure Pursuit to guide
//! the robot along curved paths.
//!
//! # Creating Trajectories
//!
//! ## From Bézier curves
//!
//! ```no_run
//! use kernelvex::{Trajectory, Vector2, QTime};
//!
//! let trajectory = Trajectory::from_cubic_bezier(
//!     Vector2::new(0.0, 0.0),   // Start
//!     Vector2::new(0.5, 0.0),   // Control 1
//!     Vector2::new(0.5, 1.0),   // Control 2
//!     Vector2::new(1.0, 1.0),   // End
//!     QTime::from_sec(3.0),     // Total time
//!     100,                       // Sample count
//!     0.5,                       // Linear velocity
//! );
//! ```
//!
//! ## Manually
//!
//! ```no_run
//! let mut trajectory = Trajectory::new();
//! trajectory.push(TrajectoryPoint::new(pose1, 0.5, 0.0, QTime::from_sec(0.0)));
//! trajectory.push(TrajectoryPoint::new(pose2, 0.5, 0.1, QTime::from_sec(1.0)));
//! ```

// TODO: add QTime instead of normal f64 type
use crate::odom::pose::Pose;
use crate::util::si::{QAngle, QTime, Vector2};

/// A single time-indexed point along a trajectory.
///
/// Contains the desired robot state (pose and velocities) at a specific time
/// during trajectory execution.
///
/// # Fields
///
/// - `pose`: Desired position and heading
/// - `linear_velocity`: Desired forward speed (m/s)
/// - `angular_velocity`: Desired rotation rate (rad/s)
/// - `time`: Time from trajectory start
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPoint {
    /// Desired pose (position and heading) at this point.
    pub pose: Pose,
    /// Desired linear velocity in meters per second.
    pub linear_velocity: f64,
    /// Desired angular velocity in radians per second.
    pub angular_velocity: f64,
    /// Time from trajectory start.
    pub time: QTime,
}

impl TrajectoryPoint {
    /// Creates a new trajectory point.
    ///
    /// # Arguments
    ///
    /// * `pose` - Desired position and heading
    /// * `linear_velocity` - Forward speed in m/s
    /// * `angular_velocity` - Rotation rate in rad/s
    /// * `time` - Time from trajectory start
    #[inline]
    pub const fn new(pose: Pose, linear_velocity: f64, angular_velocity: f64, time: QTime) -> Self {
        Self {
            pose,
            linear_velocity,
            angular_velocity,
            time,
        }
    }
}

/// A time-parameterized trajectory with sampling support.
///
/// Stores a sequence of [`TrajectoryPoint`]s and provides methods for
/// sampling the trajectory at arbitrary times with interpolation.
///
/// # Sampling
///
/// Use [`sample`](Self::sample) to get the trajectory state at any time.
/// The method interpolates between stored points for smooth tracking.
#[derive(Debug, Clone)]
pub struct Trajectory {
    /// Time-ordered trajectory points.
    points: Vec<TrajectoryPoint>,
}

impl Trajectory {
    /// Creates an empty trajectory.
    #[inline]
    pub const fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Creates a trajectory from time-ordered points.
    ///
    /// # Arguments
    ///
    /// * `points` - Vector of trajectory points, must be in ascending time order
    #[inline]
    pub const fn from_points(points: Vec<TrajectoryPoint>) -> Self {
        Self { points }
    }

    /// Returns a read-only view of trajectory points.
    #[inline]
    pub fn points(&self) -> &[TrajectoryPoint] {
        &self.points
    }

    /// Returns the total trajectory duration.
    ///
    /// # Returns
    ///
    /// The time of the last point, or `None` if the trajectory is empty.
    pub fn total_time(&self) -> Option<QTime> {
        self.points.last().map(|p| p.time)
    }

    /// Adds a point to the end of the trajectory.
    ///
    /// Points should be added in ascending time order.
    pub fn push(&mut self, point: TrajectoryPoint) {
        self.points.push(point);
    }

    /// Samples the trajectory at the given time with interpolation.
    ///
    /// Returns the interpolated trajectory state at the specified time.
    /// If the time is outside the trajectory bounds, the nearest endpoint
    /// is returned.
    ///
    /// # Arguments
    ///
    /// * `time` - Time from trajectory start
    ///
    /// # Returns
    ///
    /// The interpolated trajectory point, or `None` if the trajectory is empty.
    ///
    /// # Interpolation
    ///
    /// Position and heading are linearly interpolated. Heading interpolation
    /// takes the shortest angular path.
    pub fn sample(&self, time: QTime) -> Option<TrajectoryPoint> {
        let first = self.points.first()?;
        let last = self.points.last()?;

        if time.as_sec() <= first.time.as_sec() {
            return Some(*first);
        }
        if time.as_sec() >= last.time.as_sec() {
            return Some(*last);
        }

        for window in self.points.windows(2) {
            let a = window[0];
            let b = window[1];
            if time.as_sec() >= a.time.as_sec() && time.as_sec() <= b.time.as_sec() {
                let span = b.time.as_sec() - a.time.as_sec();
                let t = if span <= 0.0 {
                    0.0
                } else {
                    (time.as_sec() - a.time.as_sec()) / span
                };

                return Some(TrajectoryPoint {
                    pose: interpolate_pose(a.pose, b.pose, t),
                    linear_velocity: lerp(a.linear_velocity, b.linear_velocity, t),
                    angular_velocity: lerp(a.angular_velocity, b.angular_velocity, t),
                    time,
                });
            }
        }

        Some(*last)
    }

    /// Creates a trajectory by sampling a cubic Bézier curve.
    ///
    /// Generates a trajectory by sampling a cubic Bezier spline at regular
    /// intervals. The heading is derived from the curve tangent, and angular
    /// velocity is estimated from heading changes between samples.
    ///
    /// # Arguments
    ///
    /// * `p0` - Start point (meters)
    /// * `p1` - First control point (meters)
    /// * `p2` - Second control point (meters)
    /// * `p3` - End point (meters)
    /// * `total_time` - Total trajectory duration
    /// * `samples` - Number of points to generate
    /// * `linear_velocity` - Constant linear velocity for all points (m/s)
    ///
    /// # Returns
    ///
    /// A trajectory with `samples` points along the Bézier curve.
    ///
    /// # Example
    ///
    /// ```no_run
    /// // S-curve trajectory
    /// let traj = Trajectory::from_cubic_bezier(
    ///     Vector2::new(0.0, 0.0),
    ///     Vector2::new(1.0, 0.0),
    ///     Vector2::new(0.0, 1.0),
    ///     Vector2::new(1.0, 1.0),
    ///     QTime::from_sec(2.0),
    ///     50,
    ///     0.8,
    /// );
    /// ```
    pub fn from_cubic_bezier(
        p0: Vector2<f64>,
        p1: Vector2<f64>,
        p2: Vector2<f64>,
        p3: Vector2<f64>,
        total_time: QTime,
        samples: usize,
        linear_velocity: f64,
    ) -> Self {
        Bezier::new(p0, p1, p2, p3).to_trajectory(total_time, samples, linear_velocity)
    }
}

impl Default for Trajectory {
    fn default() -> Self {
        Self::new()
    }
}

impl Default for TrajectoryPoint {
    fn default() -> Self {
        Self {
            pose: Pose::identity(),
            linear_velocity: 0.,
            angular_velocity: 0.,
            time: QTime::default(),
        }
    }
}
/// Linear interpolation between two values.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

/// Interpolates between two angles, taking the shortest path.
fn lerp_angle(a: QAngle, b: QAngle, t: f64) -> QAngle {
    let delta = (b - a).remainder(QAngle::TAU);
    a + delta * t
}

/// Interpolates between two poses.
fn interpolate_pose(a: Pose, b: Pose, t: f64) -> Pose {
    let (ax, ay) = (a.position().x, a.position().y);
    let (bx, by) = (b.position().x, b.position().y);
    let heading = lerp_angle(a.heading(), b.heading(), t);
    Pose::new(
        Vector2::<f64>::new(lerp(ax, bx, t), lerp(ay, by, t)),
        heading,
    )
}

/// A cubic Bézier curve defined by start, end, and two control points.
///
/// Bézier curves provide smooth, controllable paths for trajectory generation.
/// The curve starts at `start`, ends at `end`, and is "pulled" toward the
/// control points without necessarily passing through them.
///
/// # Curve Properties
///
/// - Always passes through start and end points
/// - Tangent at start points toward control1
/// - Tangent at end points away from control2
/// - Entire curve lies within convex hull of control points
///
/// # Example
///
/// ```no_run
/// let curve = Bezier::new(
///     Vector2::new(0.0, 0.0),   // Start
///     Vector2::new(1.0, 0.0),   // Control 1 - pulls curve right
///     Vector2::new(0.0, 1.0),   // Control 2 - pulls curve up
///     Vector2::new(1.0, 1.0),   // End
/// );
///
/// // Sample point at t=0.5 (middle of curve)
/// let midpoint = curve.point(0.5);
/// let heading = curve.heading(0.5);
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Bezier {
    /// Starting point of the curve.
    start: Vector2<f64>,
    /// First control point (influences curve near start).
    control1: Vector2<f64>,
    /// Second control point (influences curve near end).
    control2: Vector2<f64>,
    /// Ending point of the curve.
    end: Vector2<f64>,
}

impl Bezier {
    /// Maximum parameter value (curves are parameterized from 0 to 1).
    const T_MAX: f64 = 1.0;

    /// Creates a new cubic Bézier curve.
    ///
    /// # Arguments
    ///
    /// * `start` - Starting point
    /// * `control1` - First control point
    /// * `control2` - Second control point
    /// * `end` - Ending point
    #[inline]
    pub const fn new(
        start: Vector2<f64>,
        control1: Vector2<f64>,
        control2: Vector2<f64>,
        end: Vector2<f64>,
    ) -> Self {
        Self {
            start,
            control1,
            control2,
            end,
        }
    }

    /// Computes the point on the curve at parameter t.
    ///
    /// # Arguments
    ///
    /// * `t` - Parameter value in range [0, 1]
    ///
    /// # Returns
    ///
    /// The (x, y) position on the curve at t.
    ///
    /// # Panics
    ///
    /// Panics if t > 1.0.
    #[inline]
    pub fn point(&self, t: f64) -> Vector2<f64> {
        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;

        self.start * (uu * u)
            + self.control1 * (3.0 * uu * t)
            + self.control2 * (3.0 * u * tt)
            + self.end * (tt * t)
    }

    /// Computes the tangent vector at parameter t.
    ///
    /// The tangent points in the direction of motion along the curve.
    ///
    /// # Arguments
    ///
    /// * `t` - Parameter value in range [0, 1]
    ///
    /// # Returns
    ///
    /// The tangent vector (not normalized).
    ///
    /// # Panics
    ///
    /// Panics if t > 1.0.
    #[inline]
    pub fn tangent(&self, t: f64) -> Vector2<f64> {
        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let u = 1.0 - t;
        let tt = t * t;
        let uu = u * u;

        (self.control1 - self.start) * (3.0 * uu)
            + (self.control2 - self.control1) * (6.0 * u * t)
            + (self.end - self.control2) * (3.0 * tt)
    }

    /// Computes the heading (direction of motion) at parameter t.
    ///
    /// # Arguments
    ///
    /// * `t` - Parameter value in range [0, 1]
    ///
    /// # Returns
    ///
    /// The heading angle derived from the tangent vector.
    ///
    /// # Panics
    ///
    /// Panics if t > 1.0.
    #[inline]
    pub fn heading(&self, t: f64) -> QAngle {
        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let tan = self.tangent(t);
        QAngle::from_radians(libm::atan2(tan.y, tan.x))
    }

    /// Computes the first derivative (velocity vector) at parameter t.
    ///
    /// This is equivalent to the tangent vector and represents the rate of
    /// change of position with respect to the parameter t.
    ///
    /// # Arguments
    ///
    /// * `t` - Parameter value in range [0, 1]
    ///
    /// # Returns
    ///
    /// The derivative vector.
    ///
    /// # Panics
    ///
    /// Panics if t > 1.0.
    pub fn derivative(&self, t: f64) -> Vector2<f64> {
        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let u = 1.0 - t;

        let q0 = (self.control1 - self.start) * 3.0;
        let q1 = (self.control2 - self.control1) * 3.0;
        let q2 = (self.end - self.control2) * 3.0;

        let velocity = q0 * (u * u) + q1 * (2.0 * u * t) + q2 * (t * t);

        velocity
    }

    /// Converts the Bézier curve to a trajectory.
    ///
    /// Samples the curve at regular intervals and computes angular velocity
    /// from heading changes between samples.
    ///
    /// # Arguments
    ///
    /// * `total_time` - Total trajectory duration
    /// * `samples` - Number of points to generate (must be >= 2)
    /// * `linear_velocity` - Constant linear velocity for all points (m/s)
    ///
    /// # Returns
    ///
    /// A trajectory with the specified number of samples, or an empty
    /// trajectory if `samples < 2`.
    pub fn to_trajectory(
        &self,
        total_time: QTime,
        samples: usize,
        linear_velocity: f64,
    ) -> Trajectory {
        if samples < 2 {
            return Trajectory::new();
        }

        let dt = total_time.as_sec() / (samples as f64 - 1.0);
        let mut points = Vec::with_capacity(samples);
        let mut headings = Vec::with_capacity(samples);

        for i in 0..samples {
            let t = i as f64 / (samples as f64 - 1.0);
            let pos = self.point(t);
            let heading = self.heading(t);
            headings.push(heading);
            points.push(TrajectoryPoint::new(
                Pose::new(pos, heading),
                linear_velocity,
                0.0,
                QTime::from_sec(dt * i as f64),
            ));
        }

        for i in 0..samples {
            let angular_velocity = if i + 1 < samples {
                let dtheta = (headings[i + 1] - headings[i]).remainder(QAngle::TAU);
                dtheta.as_radians() / dt
            } else if i > 0 {
                let dtheta = (headings[i] - headings[i - 1]).remainder(QAngle::TAU);
                dtheta.as_radians() / dt
            } else {
                0.0
            };
            points[i].angular_velocity = angular_velocity;
        }

        Trajectory::from_points(points)
    }
}
