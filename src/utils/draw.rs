use opencv::Error;
use opencv::core::{Mat, Scalar};
use opencv::core::{MatTraitConst, Point as CvPoint, Size};
use opencv::imgproc;
use std::env;

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

pub fn draw_line(
    img: &mut Mat,
    pt1: Point,
    pt2: Point,
    color: Scalar,
) -> Result<(), Error> {
    let thickness = 1;
    let line_type = imgproc::LINE_8;
    let shift = 0;

    imgproc::line(img, CvPoint::new(pt1.x, pt1.y), CvPoint::new(pt2.x, pt2.y), color, thickness, line_type, shift)
        .map_err(|e| e)
}


pub fn upscale_mat(src: &mut Mat) -> Result<(), Error> {
    let upscale_factor: f64 = env::var("UPSCALE_FACTOR")
        .expect("UPSCALE_FACTOR not set in .env")
        .parse()
        .expect("UPSCALE_FACTOR must be a valid f64");

    let mut dst = Mat::default();

    let new_width = (src.cols() as f64 * upscale_factor).round() as i32;
    let new_height = (src.rows() as f64 * upscale_factor).round() as i32;
    let target_size = Size::new(new_width, new_height);

    imgproc::resize(src, &mut dst, target_size, 0.0, 0.0, imgproc::INTER_CUBIC)?;

    dst.copy_to(src)?;

    Ok(())
}
