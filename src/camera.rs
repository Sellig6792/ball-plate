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
    /// Initialise la caméra avec la résolution et le framerate du fichier .env
    pub fn init() -> Result<Self, NokhwaError> {
        let index = CameraIndex::Index(0);
        let frame_rate: u32 = match env::var("FRAME_RATE") {
            Ok(value) => value
                .parse::<u32>()
                .expect("FRAME_RATE n'est pas un u32 valide"),
            Err(_) => 20,
        };
        let resolution: Resolution = match env::var("RESOLUTION") {
            Ok(value) => {
                let vec = value
                    .split("x")
                    .map(|x| {
                        x.parse::<u32>()
                            .expect("RESOLUTION n'est pas un u32 valide")
                    })
                    .collect::<Vec<u32>>();
                Resolution::new(vec[0], vec[1])
            }
            Err(_) => Resolution::new(640, 480),
        };

        let requested = RequestedFormat::new::<LumaFormat>(RequestedFormatType::Exact(
            CameraFormat::new(resolution, FrameFormat::MJPEG, frame_rate),
        ));

        let mut camera = nokhwa::Camera::new(index, requested)?;
        camera.open_stream()?;
        Ok(Self { camera })
    }

    /// Récupère l'image brute de la caméra et la décode en Matrice BGR OpenCV
    pub fn get_frame(&mut self) -> Result<Mat, NokhwaError> {
        let frame_buffer = &self.camera.frame()?;
        Ok(imdecode(
            &Mat::from_slice(frame_buffer.buffer()).expect("Impossible de lire le buffer image"),
            IMREAD_COLOR,
        )
        .expect("Échec du décodage du flux MJPEG"))
    }

    /// Isole la couleur de la bille (généralement orange fluo) dans l'espace HSV
    pub fn threshold(image_bgr: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut hsv = Mat::default();
        let mut mask = Mat::default();
        cvt_color(image_bgr, &mut hsv, COLOR_BGR2HSV, 0)?;

        // Coefficient de conversion HSV OpenCV (0-100% -> 0-255)
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

    /// Applique un léger flou gaussien pour éliminer le bruit numérique de l'image
    pub fn blur(mask: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut blurred = Mat::default();
        gaussian_blur(mask, &mut blurred, Size::new(5, 5), 0., 0., 0)?;
        Ok(blurred)
    }

    /// Analyse les contours pour calculer le centre (X,Y) et le rayon de la bille
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

        // On cherche le plus grand contour correspondant à la couleur de la balle
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

    /// Ferme proprement le flux matériel de la caméra
    pub fn close(&mut self) -> Result<(), NokhwaError> {
        self.camera.stop_stream()
    }
}
