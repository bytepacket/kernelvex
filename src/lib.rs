mod length;
mod filters;


#[cfg(test)]
mod tests {
    use std::ops::Add;
    use crate::length::*;
    use super::*;

    #[test]
    fn it_works() {
        let result = 5.0.cm() * 10.0.meters();
        let area = 5.0.m2();
    }
}
