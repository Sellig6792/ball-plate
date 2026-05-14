use opencv::Error;
use opencv::core::Point as CvPoint;
use opencv::core::{Mat, Scalar};
use opencv::imgproc;

use crate::utils::Point;

pub enum CircleType {
    Circle = 1,
    Point = -1,
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
