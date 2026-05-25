use crate::utils::{Point, ball};
use nokhwa::NokhwaError;
use nokhwa::pixel_format::LumaFormat;
use nokhwa::utils::{
    CameraFormat, CameraIndex, FrameFormat, RequestedFormat, RequestedFormatType, Resolution,
};
use opencv::core::{Mat, Point2f, Scalar, Size, Vector, in_range};
use opencv::imgcodecs::{IMREAD_COLOR, imdecode};
use opencv::imgproc::{
    CHAIN_APPROX_SIMPLE, COLOR_BGR2HSV, RETR_EXTERNAL, cvt_color, find_contours, gaussian_blur,
    min_enclosing_circle,
};
use std::env;

pub struct Camera {
    camera: nokhwa::Camera,
}

impl Camera {
    /// Initialize the camera with resolution and framerate from .env file
    pub fn init() -> Result<Self, NokhwaError> {
        let index = CameraIndex::Index(0);
        let frame_rate: u32 = env::var("FRAME_RATE")
            .expect("FRAME_RATE must be set in .env")
            .parse()
            .expect("FRAME_RATE is not a valid u32");

        let res_vec = env::var("RESOLUTION")
            .expect("RESOLUTION must be set in .env")
            .split("x")
            .map(|x| x.parse::<u32>().expect("RESOLUTION is not a valid u32"))
            .collect::<Vec<u32>>();
        let resolution = Resolution::new(res_vec[0], res_vec[1]);

        let requested = RequestedFormat::new::<LumaFormat>(RequestedFormatType::Exact(
            CameraFormat::new(resolution, FrameFormat::MJPEG, frame_rate),
        ));

        let mut camera = nokhwa::Camera::new(index, requested)?;
        camera.open_stream()?;
        Ok(Self { camera })
    }

    /// Retrieve raw image from camera and decode it to OpenCV BGR Matrix
    pub fn get_frame(&mut self) -> Result<Mat, NokhwaError> {
        let frame_buffer = &self.camera.frame()?;
        Ok(imdecode(
            &Mat::from_slice(frame_buffer.buffer()).expect("Unable to read image buffer"),
            IMREAD_COLOR,
        )
        .expect("Failed to decode MJPEG stream"))
    }

    /// Isolate the ball color (typically fluorescent orange) in HSV color space
    pub fn threshold(image_bgr: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut hsv = Mat::default();
        let mut mask = Mat::default();
        cvt_color(image_bgr, &mut hsv, COLOR_BGR2HSV, 0)?;

        // HSV OpenCV conversion coefficient (0-100% -> 0-255)
        const K: f64 = 2.55;

        let ball_lower_color = ball::get_ball_color(ball::BallColor::Lower);
        let ball_higher_color = ball::get_ball_color(ball::BallColor::Higher);

        let lower_color = Scalar::new(
            ball_lower_color.hue() as f64,
            ball_lower_color.saturation() as f64 * K,
            ball_lower_color.value() as f64 * K,
            0.0,
        );
        let higher_color = Scalar::new(
            ball_higher_color.hue() as f64,
            ball_higher_color.saturation() as f64 * K,
            ball_higher_color.value() as f64 * K,
            0.0,
        );

        in_range(&hsv, &lower_color, &higher_color, &mut mask)?;
        Ok(mask)
    }

    /// Apply a slight Gaussian blur to eliminate digital noise from the image
    pub fn blur(mask: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut blurred = Mat::default();
        gaussian_blur(mask, &mut blurred, Size::new(5, 5), 0., 0., 0)?;
        Ok(blurred)
    }

    /// Analyze contours to calculate the center (X,Y) and radius of the ball
    pub fn get_circle(
        &self,
        image_bgr: &Mat,
    ) -> Result<Option<(Point, i32)>, Box<dyn std::error::Error>> {
        let mask = Self::threshold(image_bgr)?;
        let clean_mask = Self::blur(&mask)?;

        let mut contours = Vector::<Vector<opencv::core::Point>>::new();
        find_contours(
            &clean_mask,
            &mut contours,
            RETR_EXTERNAL,
            CHAIN_APPROX_SIMPLE,
            opencv::core::Point::new(0, 0),
        )?;

        let mut max_area = 0.0;
        let mut best_contour = None;

        // Find the largest contour matching the ball color
        for contour in contours.iter() {
            let area = opencv::imgproc::contour_area(&contour, false)?;
            if area > 50.0 && area > max_area {
                max_area = area;
                best_contour = Some(contour);
            }
        }

        if let Some(contour) = best_contour {
            let mut center = Point2f::default();
            let mut radius = 0.0;
            let mut hull = Vector::<opencv::core::Point>::new();
            opencv::imgproc::convex_hull(&contour, &mut hull, false, true)?;
            min_enclosing_circle(&hull, &mut center, &mut radius)?;

            return Ok(Some((
                Point::new(center.x.round() as i32, center.y.round() as i32),
                radius as i32,
            )));
        }
        Ok(None)
    }

    /// Properly close the camera hardware stream
    pub fn close(&mut self) -> Result<(), NokhwaError> {
        self.camera.stop_stream()
    }
}
