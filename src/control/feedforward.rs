use crate::QAngle;

pub struct FeedForward {
    ks: f64,
    kv: f64,
    ka: f64
}

impl FeedForward {
    #[inline]
    pub const fn new(ks: f64, kv: f64, ka: f64) -> FeedForward {
        Self { ks, kv, ka }
    }

    pub const fn ks(&self) -> f64 { self.ks }

    pub const fn kv(&self) -> f64 { self.kv }

    pub const fn ka(&self) -> f64 { self.ka }

    pub const fn set_gains(&mut self, ks: f64, kv: f64, ka: f64) -> Self {
        Self {ks, kv, ka}
    }

    pub const fn set_ks(&mut self, ks: f64) { self.ks = ks }

    pub const fn set_kv(&mut self, kv: f64) { self.kv = kv }

    pub const fn set_ka(&mut self, ka: f64) { self.ka = ka }

    pub fn calculate(&self, velocity: f64, acceleration: f64) -> f64 {
        self.ks * velocity.signum()
            + self.kv * velocity
            + self.ka * acceleration
    }
}

pub struct ArmFeedForward {
    ks: f64,
    kv: f64,
    ka: f64,
    kg: f64,
}

impl ArmFeedForward {
    #[inline]
    pub const fn new(ks: f64, kv: f64, ka: f64, kg: f64) -> Self {
        Self { ks, kv, ka, kg }
    }

    #[inline]
    pub const fn ks(&self) -> f64 { self.ks }

    #[inline]
    pub const fn kv(&self) -> f64 { self.kv }

    #[inline]
    pub const fn ka(&self) -> f64 { self.ka }

    #[inline]
    pub const fn kg(&self) -> f64 { self.kg }

    #[inline]
    pub const fn set_gains(self, ks: f64, kv: f64, ka: f64, kg: f64) -> Self {
        Self { ks, kv, ka, kg }
    }

    #[inline]
    pub const fn set_ks(self, ks: f64) -> Self { Self { ks, ..self } }

    #[inline]
    pub const fn set_kv(self, kv: f64) -> Self { Self { kv, ..self } }

    #[inline]
    pub const fn set_ka(self, ka: f64) -> Self { Self { ka, ..self } }

    #[inline]
    pub const fn set_kg(self, kg: f64) -> Self { Self { kg, ..self } }

    #[inline]
    pub fn calculate(&self, angle: QAngle, velocity: f64, acceleration: f64, g: impl Fn(QAngle) -> f64) -> f64 {
        self.ks * velocity.signum()
            + self.kv * velocity
            + self.ka * acceleration
            + self.kg * g(angle)
    }
}