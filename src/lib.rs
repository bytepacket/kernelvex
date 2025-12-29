mod si;
mod angles;
mod pose;
mod omniwheel;

#[cfg(test)]
mod tests {
    use std::ops::AddAssign;
    use crate::angles::{Angle, Degrees, Radians};
    use crate::si::*;
    use crate::pose::Pose;


    #[test]
    fn it_works() {
        let a = Length::<Meter>::new(1.0);
        let b = Length::<Centimeter>::new(50.0);
        let c = Length::<Inch>::new(12.0);
        let d = Length::<Foot>::new(3.0);
        let e = Length::<Inch>::new(36.0);
        let f: Time<Seconds> = Time::<Seconds>::new(60.);
        let g : Time<Minutes> = Time::<Minutes>::new(1.);
        assert_eq!(f, g);
        assert!(d.approx_eq(e, None));

        let f = g.abs();

        let sum = a + b + c;


        let sum_cm = sum.to::<Centimeter>();
        let sum_in = sum.to::<Inch>();


        println!("{}", sum);
        println!("{}", sum_cm);
        println!("{}", sum_in);


        let doubled = d * 2.0;
        println!("{}", doubled);


        let ratio = c / b;
        println!("{}", ratio);

        let x = Angle::<Degrees>::new(1.);
        let y = x.to::<Radians>();

        let _ = x.sin();

        let mut z = Angle::<Radians>::new(2.0);

        z += Angle::<Radians>::new(3.);

        println!("{}", z);

        println!("{}", y);

        let p = Pose::new(-1., -2., 0.0.into());

        let r = Pose::new(2., 1., 0.0.into());

        let k = p * r;

        println!("{}", k);
    }
}
