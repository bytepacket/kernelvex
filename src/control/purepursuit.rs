//! Pure pursuit controller for trajectory tracking.
//!
//! This module implements the pure pursuit algorithm, a geometric path tracking
//! controller that steers the robot toward a lookahead point on the trajectory.
//!
//! # Overview
//!
//! Pure pursuit works by:
//! 1. Finding a lookahead point on the trajectory at a fixed distance ahead
//! 2. Computing the curvature needed to reach that point
//! 3. Converting curvature to wheel speeds
//!
//! The lookahead distance determines tracking behavior:
//! - **Shorter lookahead**: Tighter tracking but more oscillation
//! - **Longer lookahead**: Smoother tracking but may cut corners
//!
//! # Algorithm
//!
//! The lookahead point is found by intersecting a circle (centered at the robot)
//! with the trajectory. The curvature to reach this point is:
//!
//! ```text
//! curvature = 2 * y_robot / lookahead^2
//! ```
//!
//! Where `y_robot` is the lateral offset of the lookahead point in the robot frame.
//!
//! # Example
//!
//! ```no_run
//! use kernelvex::{PurePursuit, Trajectory, Pose};
//!
//! let controller = PurePursuit::new(trajectory, 0.3); // 30cm lookahead
//!
//! // Find lookahead point
//! if let Some(target) = controller.intersect(current_pose) {
//!     let curvature = controller.curvature(current_pose, target.pose.position());
//!     // Convert curvature to wheel speeds
//! }
//! ```

use crate::motion::trajectory::{Trajectory, TrajectoryPoint};
use crate::odom::pose::Pose;
use crate::util::si::Vector2;

/// Pure pursuit controller for path following.
///
/// Pure pursuit is a geometric controller that steers toward a lookahead point
/// on the trajectory. It's simpler than RAMSETE but works well for smooth paths.
///
/// # Fields
///
/// - `trajectory`: The path to follow
/// - `lookahead`: Distance ahead to look for the target point (meters)
#[derive(Debug, Clone)]
pub struct PurePursuit {
    /// The trajectory to follow.
    trajectory: Trajectory,
    /// Lookahead distance in meters.
    lookahead: f64,
}

impl PurePursuit {
    /// Creates a pure pursuit controller with the given trajectory and lookahead distance.
    ///
    /// # Arguments
    ///
    /// * `trajectory` - The path to follow
    /// * `lookahead` - Distance ahead to look for target point (meters)
    ///
    /// # Example
    ///
    /// ```no_run
    /// let controller = PurePursuit::new(trajectory, 0.3); // 30cm lookahead
    /// ```
    #[inline]
    pub fn new(trajectory: Trajectory, lookahead: f64) -> Self {
        Self {
            trajectory,
            lookahead,
        }
    }

    /// Returns the current lookahead distance in meters.
    #[inline]
    pub const fn lookahead(&self) -> f64 {
        self.lookahead
    }

    /// Sets a new lookahead distance in meters.
    ///
    /// Adjust during operation to trade off between tracking accuracy
    /// and smoothness.
    #[inline]
    pub fn set_lookahead(&mut self, lookahead: f64) {
        self.lookahead = lookahead;
    }

    /// Returns a reference to the underlying trajectory.
    #[inline]
    pub const fn trajectory(&self) -> &Trajectory {
        &self.trajectory
    }

    /// Finds the lookahead point on the trajectory using circle intersection.
    ///
    /// Draws a circle of radius `lookahead` centered at the robot's position
    /// and finds where it intersects the trajectory. Returns the furthest
    /// intersection point along the path.
    ///
    /// # Arguments
    ///
    /// * `pose` - The robot's current pose
    ///
    /// # Returns
    ///
    /// The trajectory point at the lookahead intersection, or `None` if no
    /// intersection is found (robot too far from path).
    ///
    /// # Algorithm
    ///
    /// For each trajectory segment:
    /// 1. Solve circle-line intersection equations
    /// 2. Filter intersections to valid segment range [0, 1]
    /// 3. Keep the furthest point along the trajectory
    pub fn intersect(&self, pose: Pose) -> Option<TrajectoryPoint> {
        let points = self.trajectory.points();
        if points.len() < 2 {
            return points.first().copied();
        }

        let center = pose.position();
        let radius = self.lookahead;

        let mut best: Option<(usize, f64, Vector2<f64>)> = None;

        for (index, window) in points.windows(2).enumerate() {
            let a_pose = window[0].pose;
            let b_pose = window[1].pose;
            let a = a_pose.position();
            let b = b_pose.position();

            let candidates = segment_circle_intersections(a, b, center, radius);
            for (t, point) in candidates {
                let should_replace = match best {
                    None => true,
                    Some((best_index, best_t, _)) => {
                        index > best_index || (index == best_index && t > best_t)
                    }
                };

                if should_replace {
                    best = Some((index, t, point));
                }
            }
        }

        let (index, t, point) = best?;
        let segment = &points[index..=index + 1];
        let a = segment[0];
        let b = segment[1];

        Some(TrajectoryPoint::new(
            Pose::new(point, a.pose.heading()),
            lerp(a.linear_velocity, b.linear_velocity, t),
            lerp(a.angular_velocity, b.angular_velocity, t),
            a.time + (b.time - a.time) * t,
        ))
    }

    /// Computes the curvature needed to reach the lookahead point.
    ///
    /// # Arguments
    ///
    /// * `pose` - The robot's current pose
    /// * `target` - The lookahead point position
    ///
    /// # Returns
    ///
    /// The curvature (1/radius) needed to reach the target. Positive curvature
    /// turns left, negative turns right. Returns 0.0 if lookahead is too small
    /// to avoid division by zero.
    ///
    /// # Formula
    ///
    /// ```text
    /// curvature = 2 * y_robot / lookahead^2
    /// ```
    ///
    /// Where `y_robot` is the lateral offset in the robot's coordinate frame.
    pub fn curvature(&self, pose: Pose, target: Vector2<f64>) -> f64 {
        /// Minimum lookahead distance to avoid division by near-zero values.
        /// 1mm is small enough to be effectively zero.
        const LOOKAHEAD_EPSILON: f64 = 1e-3;

        let coords = pose.position();
        let dx = target.x - coords.x;
        let dy = target.y - coords.y;

        let heading = pose.heading();
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        let y_r = -sin_h * dx + cos_h * dy;
        if self.lookahead.abs() < LOOKAHEAD_EPSILON {
            0.0
        } else {
            2.0 * y_r / (self.lookahead * self.lookahead)
        }
    }
}

/// Finds circle-line segment intersections.
///
/// # Arguments
///
/// * `a` - Segment start point
/// * `b` - Segment end point
/// * `center` - Circle center
/// * `radius` - Circle radius
///
/// # Returns
///
/// Vector of (t, point) tuples where t is the parameter [0, 1] along the segment.
fn segment_circle_intersections(
    a: Vector2<f64>,
    b: Vector2<f64>,
    center: Vector2<f64>,
    radius: f64,
) -> Vec<(f64, Vector2<f64>)> {
    let d = b - a;
    let f = a - center;

    let a_coef = d.dot(d);
    let b_coef = 2.0 * f.dot(d);
    let c_coef = f.dot(f) - radius * radius;

    let discriminant = b_coef * b_coef - 4.0 * a_coef * c_coef;
    if discriminant < 0.0 || a_coef == 0.0 {
        return Vec::new();
    }

    let sqrt_disc = libm::sqrt(discriminant);
    let t1 = (-b_coef - sqrt_disc) / (2.0 * a_coef);
    let t2 = (-b_coef + sqrt_disc) / (2.0 * a_coef);

    let mut results = Vec::new();
    if (0.0..=1.0).contains(&t1) {
        results.push((t1, a + d * t1));
    }
    if (0.0..=1.0).contains(&t2) && t2 != t1 {
        results.push((t2, a + d * t2));
    }

    results
}

/// Linear interpolation between two values.
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}
