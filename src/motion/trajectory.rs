//! Trajectory representation and sampling utilities.

use crate::odom::pose::Pose;
use crate::util::si::{QAngle, QTime};

/// A single time-indexed point along a trajectory.
#[derive(Debug, Clone, Copy)]
pub struct TrajectoryPoint {
    pub pose: Pose,
    /// Desired linear velocity in meters per second.
    pub linear_velocity: f64,
    /// Desired angular velocity in radians per second.
    pub angular_velocity: f64,
    /// Time from trajectory start.
    pub time: QTime,
}

impl TrajectoryPoint {
    #[inline]
    pub fn new(pose: Pose, linear_velocity: f64, angular_velocity: f64, time: QTime) -> Self {
        Self {
            pose,
            linear_velocity,
            angular_velocity,
            time,
        }
    }
}

/// A time-parameterized trajectory with sampling support.
#[derive(Debug, Clone)]
pub struct Trajectory {
    points: Vec<TrajectoryPoint>,
}

impl Trajectory {
    /// Creates an empty trajectory.
    #[inline]
    pub fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Creates a trajectory from time-ordered points.
    #[inline]
    pub fn from_points(points: Vec<TrajectoryPoint>) -> Self {
        Self { points }
    }

    /// Returns a read-only view of trajectory points.
    #[inline]
    pub fn points(&self) -> &[TrajectoryPoint] {
        &self.points
    }

    /// Returns the total trajectory time.
    pub fn total_time(&self) -> Option<QTime> {
        self.points.last().map(|p| p.time)
    }

    /// Adds a point to the trajectory.
    pub fn push(&mut self, point: TrajectoryPoint) {
        self.points.push(point);
    }

    /// Samples the trajectory at the given time.
    ///
    /// If the time is outside the trajectory bounds, the nearest endpoint is returned.
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
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn lerp_angle(a: QAngle, b: QAngle, t: f64) -> QAngle {
    let delta = (b - a).remainder(QAngle::TAU);
    a + delta * t
}

fn interpolate_pose(a: Pose, b: Pose, t: f64) -> Pose {
    let (ax, ay) = a.position();
    let (bx, by) = b.position();
    let heading = lerp_angle(a.heading(), b.heading(), t);
    Pose::new(lerp(ax, bx, t), lerp(ay, by, t), heading)
}
