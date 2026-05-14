use easy_color::HSV;
use std::env;

pub enum BallColor {
    Lower,
    Higher,
}

pub fn get_ball_color(ball_color: BallColor) -> HSV {
    let e = match ball_color {
        BallColor::Lower => "LOWER",
        BallColor::Higher => "HIGHER",
    };
    match env::var(format!("BALL_{}_COLOR", e)) {
        Ok(value) => value
            .as_str()
            .try_into()
            .unwrap_or_else(|_| panic!("BALL_{}_COLOR is not a valid HSV value", e)),
        Err(_) => panic!("BALL_{}_COLOR is not set", e),
    }
}
