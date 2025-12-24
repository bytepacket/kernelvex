mod builder;


#[cfg(test)]
mod tests {
    use crate::builder::*;
    use crate::unit;
    use crate::derive_unit;

    #[test]
    fn it_works() {
        unit!(
            Length,

            Types {
                Meter => 1.0,
                Centimeter => 0.01,
                Millimeter => 0.001,
                Inch => 0.0254,
                Foot => 0.3048,
            }
        );

        unit!(
            Time,
            Types {
                Seconds => 1.0,
                Minutes => 60.0,
                Hours => 3600.0,
            }
        );

        derive_unit!(
            Speed,

            Types {
                MeterPerSecond => 1.0,
                CentimeterPerSecond => 0.01,
                MillimeterPerSecond => 0.001,
                FeetPerSecond => 0.3048,
                InchPerSecond => 0.0254,
            }
            Conv {
                Length / Time
            }
        );



        let a = Length::<Meter>::new(1.0);
        let b = Length::<Centimeter>::new(50.0);
        let c = Length::<Inch>::new(12.0);
        let d = Length::<Foot>::new(3.0);
        let e = Length::<Inch>::new(36.0);
        let f: Time<Seconds> = Time::<Seconds>::new(60.);
        let g : Time<Minutes> = Time::<Minutes>::new(1.);
        assert_eq!(f, g);
        assert!(d.approx_eq(e, None));


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

        let speed = a/f;
        println!("{}", speed);

    }
}
