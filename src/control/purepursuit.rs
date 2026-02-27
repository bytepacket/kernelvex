//! Pure pursuit controller for trajectory tracking.

use crate::motion::trajectory::{Trajectory, TrajectoryPoint};
use crate::odom::pose::Pose;
use crate::util::si::Vector2;

#[derive(Debug, Clone)]
pub struct PurePursuit {
    trajectory: Trajectory,
    lookahead: f64,
}

impl PurePursuit {
    /// Creates a pure pursuit controller with the given trajectory and lookahead distance.
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

    /// Computes curvature toward the lookahead point in the robot frame.
    pub fn curvature(&self, pose: Pose, target: Vector2<f64>) -> f64 {
        let coords = pose.position();
        let dx = target.x - coords.x;
        let dy = target.y - coords.y;

        let heading = pose.heading();
        let cos_h = heading.cos();
        let sin_h = heading.sin();

        let y_r = -sin_h * dx + cos_h * dy;
        if self.lookahead == 0.0 {
            0.0
        } else {
            2.0 * y_r / (self.lookahead * self.lookahead)
        }
    }
}

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

fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

