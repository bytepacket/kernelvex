//! Trajectory representation and sampling utilities.
// TODO: add QTime instead of normal f64 type
use crate::odom::pose::Pose;
use crate::util::si::{QAngle, QTime, Vector2};

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
#[derive(Debug, Clone)]
pub struct Trajectory {
    points: Vec<TrajectoryPoint>,
}

impl Trajectory {
    /// Creates an empty trajectory.
    #[inline]
    pub const fn new() -> Self {
        Self { points: Vec::new() }
    }

    /// Creates a trajectory from time-ordered points.
    #[inline]
    pub const fn from_points(points: Vec<TrajectoryPoint>) -> Self {
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

    /// Creates a trajectory by sampling a cubic Bézier curve.
    ///
    /// Control points are specified in meters. Heading is derived from the curve
    /// tangent, linear velocity is constant, and angular velocity is estimated
    /// from successive heading changes.
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
fn lerp(a: f64, b: f64, t: f64) -> f64 {
    a + (b - a) * t
}

fn lerp_angle(a: QAngle, b: QAngle, t: f64) -> QAngle {
    let delta = (b - a).remainder(QAngle::TAU);
    a + delta * t
}

fn interpolate_pose(a: Pose, b: Pose, t: f64) -> Pose {
    let (ax, ay) = (a.position().x, a.position().y);
    let (bx, by) = (b.position().x, b.position().y);
    let heading = lerp_angle(a.heading(), b.heading(), t);
    Pose::new(Vector2::<f64>::new(lerp(ax, bx, t), lerp(ay, by, t)), heading)
}

/// A cubic Bézier curve defined by start, end, and two control points.
#[derive(Debug, Clone, Copy)]
pub struct Bezier {
    start: Vector2<f64>,
    control1: Vector2<f64>,
    control2: Vector2<f64>,
    end: Vector2<f64>,
}

impl Bezier {

    const T_MAX: f64 = 1.0;
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

    #[inline]
    pub fn heading(&self, t: f64) -> QAngle {

        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let tan = self.tangent(t);
        QAngle::from_radians(libm::atan2(tan.y, tan.x))
    }

    pub fn derivative(&self, t: f64) -> Vector2<f64> {

        {
            assert!(t <= Self::T_MAX, "time cannot exceed 1");
        }

        let u = 1.0 - t;

        let q0 = (self.control1 - self.start) * 3.0;
        let q1 = (self.control2 - self.control1) * 3.0;
        let q2 = (self.end - self.control2) * 3.0;

        let velocity = q0 * (u * u)
            + q1 * (2.0 * u * t)
            + q2 * (t * t);

        velocity
    }

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
