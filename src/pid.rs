#![allow(dead_code)]

use chrono::{NaiveTime, Utc};

struct Pid {
    kp: f32,
    ki: f32,
    kd: f32,
    time: NaiveTime,
}

impl Pid {
    #[inline]
    pub fn new(kp: f32, ki: f32, kd: f32) -> Pid {
        Pid {
            kp,
            ki,
            kd,
            time: Utc::now().time(),
        }
    }

    pub fn values(&self) -> (f32, f32, f32) {
        (self.kp, self.ki, self.kd)
    }

    pub fn calculate(&mut self, error: f32) {
        let diff = (Utc::now().time() - self.time);
        self.time = Utc::now().time();
        self.ki += error * (diff.num_seconds() as f32);
        todo!("implement");
    }
}
