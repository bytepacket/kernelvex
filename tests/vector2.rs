use kernelvex::util::si::{QLength, Vector2};
use nalgebra::Vector2 as NaVector2;

#[test]
fn test_vector2_add_sub() {
    let a = Vector2::<f64>::new(1.0, 2.0);
    let b = Vector2::<f64>::new(3.0, 4.0);
    let c = a + b;
    let d = c - a;

    assert!((c.x - 4.0).abs() < 1e-9);
    assert!((c.y - 6.0).abs() < 1e-9);
    assert!((d.x - 3.0).abs() < 1e-9);
    assert!((d.y - 4.0).abs() < 1e-9);
}

#[test]
fn test_vector2_dot_cross() {
    let a = Vector2::<f64>::new(1.0, 0.0);
    let b = Vector2::<f64>::new(0.0, 2.0);

    let dot = a.dot(b);
    let cross = a.cross(b);

    assert!((dot - 0.0).abs() < 1e-9);
    assert!((cross - 2.0).abs() < 1e-9);
}

#[test]
fn test_vector2_norm_normalize() {
    let v = Vector2::<f64>::new(3.0, 4.0);
    let norm = v.norm();
    assert!((norm - 5.0).abs() < 1e-9);

    let unit = v.normalize();
    let mag = libm::sqrt(unit.x * unit.x + unit.y * unit.y);
    assert!((mag - 1.0).abs() < 1e-9);
}

#[test]
fn test_vector2_distance_lerp() {
    let a = Vector2::<f64>::new(0.0, 0.0);
    let b = Vector2::<f64>::new(2.0, 0.0);

    let dist = a.distance(b);
    assert!((dist - 2.0).abs() < 1e-9);

    let mid = a.lerp(b, 0.5);
    assert!((mid.x - 1.0).abs() < 1e-9);
    assert!((mid.y - 0.0).abs() < 1e-9);
}

#[test]
fn test_vector2_nalgebra_interop() {
    let a = Vector2::<f64>::new(1.25, -2.5);
    let v: NaVector2<f64> = a.into();
    let b: Vector2<f64> = v.into();

    assert!((b.x - 1.25).abs() < 1e-9);
    assert!((b.y + 2.5).abs() < 1e-9);
}

#[test]
fn test_vector2_quantity_ops() {
    let a = Vector2::new(QLength::from_meters(1.0), QLength::from_meters(2.0));
    let b = Vector2::new(QLength::from_meters(3.0), QLength::from_meters(4.0));

    let c = a + b;
    assert!((c.x.as_meters() - 4.0).abs() < 1e-9);
    assert!((c.y.as_meters() - 6.0).abs() < 1e-9);

    let dot = a.dot(b);
    assert!((dot.raw() - 11.0).abs() < 1e-9);

    let unit = a.normalize();
    let mag = libm::sqrt(unit.x.raw() * unit.x.raw() + unit.y.raw() * unit.y.raw());
    assert!((mag - 1.0).abs() < 1e-9);
}
