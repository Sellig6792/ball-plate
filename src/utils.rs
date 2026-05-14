use easy_color::HSV;
use opencv::Error;
use opencv::core::Mat;
use opencv::core::{Point as CvPoint, Scalar};
use opencv::imgproc;
use std::env;

pub enum CircleType {
    Circle = 1,
    Point = -1,
}

#[derive(Debug, Clone)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

pub fn draw_circle(
    edges_map: &mut Mat,
    center: &Point,
    radius: i32,
    circle_type: CircleType,
    color: Scalar,
) -> Result<(), Error> {
    imgproc::circle(
        edges_map,
        CvPoint::new(center.x, center.y),
        radius,
        color,
        circle_type as i32,
        imgproc::LINE_AA,
        0,
    )
}

pub fn draw_vector(edges_map: &mut Mat, origin: Point, destination: Point) -> Result<(), Error> {
    // 1. Convert your custom Point to OpenCV's Point
    let origin_point = CvPoint::new(origin.x, origin.y);
    let destination_point = CvPoint::new(destination.x, destination.y);

    // 2. Use the built-in line function
    // Scalar is (B, G, R, A)
    imgproc::line(
        edges_map,
        origin_point,
        destination_point,
        Scalar::new(255.0, 255.0, 0.0, 0.0), // Cyan
        1,                                   //
        imgproc::LINE_8,                     // Line type
        0,                                   // Shift (fractional bits)
    )?;

    // 2. Draw a red circle at the destination point
    draw_circle(
        edges_map,
        &destination,
        3,
        CircleType::Point,
        Scalar::new(0.0, 0.0, 255.0, 0.0),
    )?;

    Ok(())
}

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
