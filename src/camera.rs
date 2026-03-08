use crate::utils::Point;
use nokhwa::NokhwaError;
use nokhwa::pixel_format::LumaFormat;
use nokhwa::utils::{CameraIndex, RequestedFormat, RequestedFormatType};
use opencv::core::{Mat, Point2f, Scalar, Size, Vector, in_range};
use opencv::imgcodecs::{IMREAD_COLOR, imdecode};
use opencv::imgproc::{
    CHAIN_APPROX_SIMPLE, COLOR_BGR2HSV, RETR_EXTERNAL, cvt_color,
    find_contours, gaussian_blur, min_enclosing_circle,
};

pub struct Camera {
    camera: nokhwa::Camera,
}

impl Camera {
    /// Initialise la caméra et ouvre le flux
    pub fn init() -> Result<Self, NokhwaError> {
        let index = CameraIndex::Index(0);
        let requested =
            RequestedFormat::new::<LumaFormat>(RequestedFormatType::AbsoluteHighestFrameRate);
        let mut camera = nokhwa::Camera::new(index, requested)?;
        camera.open_stream()?;
        Ok(Self { camera })
    }

    /// Récupère une image brute en couleur (BGR)
    pub fn get_frame(&mut self) -> Result<Mat, NokhwaError> {
        let frame_buffer = &self.camera.frame()?;
        Ok(imdecode(
            &Mat::from_slice(frame_buffer.buffer()).expect("Échec lecture buffer"),
            IMREAD_COLOR,
        )
        .expect("Échec décodage MJPG"))
    }

    /// ÉTAPE 1: Isole l'orange fluo via l'espace HSV
    pub fn threshold(image_bgr: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut hsv = Mat::default();
        let mut mask = Mat::default();
        cvt_color(image_bgr, &mut hsv, COLOR_BGR2HSV, 0)?;

        // Plage Orange Fluo :
        // H (Teinte) : 5 à 22 | S (Saturation) : 150 à 255 | V (Valeur) : 100 à 255
        let lower_orange = Scalar::new(10.0, 100.0, 30.0, 0.0);
        let upper_orange = Scalar::new(40.0, 255.0, 240.0, 0.0);

        in_range(&hsv, &lower_orange, &upper_orange, &mut mask)?;
        Ok(mask)
    }

    /// ÉTAPE 2: Lissage pour supprimer le bruit autour de la balle
    pub fn blur(mask: &Mat) -> Result<Mat, Box<dyn std::error::Error>> {
        let mut blurred = Mat::default();
        gaussian_blur(mask, &mut blurred, Size::new(5, 5), 0., 0., 0)?;
        Ok(blurred)
    }

    /// ÉTAPE 3: Détecte la balle et génère les points d'un cercle parfait
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

        // On cherche le plus gros objet orange
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
            // On calcule le cercle mathématique qui englobe la forme
            min_enclosing_circle(&hull, &mut center, &mut radius)?;

            return Ok(Some((
                Point::new(center.x.round() as i32, center.y.round() as i32),
                radius as i32,
            )));
        }
        Ok(None)
    }

    pub fn close(&mut self) -> Result<(), NokhwaError> {
        self.camera.stop_stream()
    }
}
