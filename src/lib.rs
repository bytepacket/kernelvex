mod angles;
mod omniwheel;
mod pid;
mod pose;
mod sensors;
mod si;
mod utils;

#[cfg(test)]
mod tests {
    use crate::angles::{Angle, Degrees, Radians};
    use crate::pose::Pose;
    use crate::si::{QAngle, QLength};

    #[test]
    fn it_works() {
        let a = QLength::from_meters(1.0);
        let b = QLength::from_centimeters(50.0);
        let c = QLength::from_inches(12.0);
        let sum = a + b + c;

        let sum_cm = sum.as_centimeters();
        let sum_in = sum.as_inches();

        println!("{:?}", sum);
        println!("{}", sum_cm);
        println!("{}", sum_in);


        let ratio = c / b;
        println!("{:?}", ratio);

        let x = QAngle::from_degrees(1.);

        let y = x.as_radians();

        let _ = x.sin();

        let mut z = Angle::<Radians>::new(2.0);

        z += Angle::<Radians>::new(3.);

        println!("{}", z);
        println!("{}", y);

        let p = Pose::new(-1., -2., Default::default());

        let r = Pose::new(2., 1., Default::default());

        let k = p * r;

        println!("{}", k);
    }
}
