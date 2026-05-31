use opencv::Error;
use opencv::core::{Mat, Scalar};
use opencv::core::{MatTraitConst, Point as CvPoint, Size};
use opencv::imgproc;
use std::env;
use crate::pid::Pid;
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

    draw_line(
        edges_map,
        origin,
        destination.clone(),
        Scalar::new(255.0, 255.0, 0.0, 0.0), // Cyan
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

pub fn draw_plate_guidelines(frame_mat: &mut Mat, pid: &Pid) {
    let _ = draw_circle(
        frame_mat,
        &Point::new(pid.center_x_pixel as i32, pid.center_y_pixel as i32),
        2,
        CircleType::Point,
        Scalar::new(197.0, 73.0, 137.0, 0.0),
    );

    if let Ok(plate_width_str) = std::env::var("PLATE_WIDTH_PIXELS") {
        if let Ok(plate_width) = plate_width_str.parse::<i32>() {
            let window_width = frame_mat.cols();
            let frame_height = frame_mat.rows();
            let x_1 = (window_width - plate_width) / 2;
            let x_2 = x_1 + plate_width;

            let _ = draw_line(frame_mat, Point::new(x_1, 0), Point::new(x_1, frame_height), Scalar::new(0.0, 0.0, 0.0, 0.0));
            let _ = draw_line(frame_mat, Point::new(x_2, 0), Point::new(x_2, frame_height), Scalar::new(0.0, 0.0, 0.0, 0.0));
        }
    }
}