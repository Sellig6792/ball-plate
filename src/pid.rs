use dotenv::dotenv;
use std::env;

#[derive(Debug, Clone, Copy)]
pub enum Axe {
    X,
    Y,
}

pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub dt: f32,
    pub invert_x: bool,
    pub invert_y: bool,
}

struct PidState {
    error_previous: f32,
    integral_sum: f32,
}

impl PidState {
    fn new() -> Self {
        Self {
            error_previous: 0.0,
            integral_sum: 0.0,
        }
    }
}

pub struct Pid {
    config: PidConfig,
    state_x: PidState,
    state_y: PidState,
    pub center_x_pixel: f32,
    pub center_y_pixel: f32,
    pixels_per_cm: f32,
}

impl Pid {
    /// Construit le régulateur PID à partir des variables d'environnement globales et manuelles
    pub fn from_env() -> Self {
        dotenv().ok();

        // Récupération stricte des paramètres du PID depuis le fichier .env
        let kp: f32 = env::var("PID_KP")
            .expect("The environment variable 'PID_KP' is missing.")
            .parse()
            .unwrap();
        let ki: f32 = env::var("PID_KI")
            .expect("The environment variable 'PID_KI' is missing.")
            .parse()
            .unwrap();
        let kd: f32 = env::var("PID_KD")
            .expect("The environment variable 'PID_KD' is missing.")
            .parse()
            .unwrap();
        let fps: f32 = env::var("FRAME_RATE")
            .expect("The environment variable 'FRAME_RATE' is missing.")
            .parse()
            .unwrap();

        let invert_x: bool = env::var("INVERT_X")
            .expect("The environment variable 'INVERT_X' is missing.")
            .parse()
            .unwrap();
        let invert_y: bool = env::var("INVERT_Y")
            .expect("The environment variable 'INVERT_Y' is missing.")
            .parse()
            .unwrap();

        let center_x_raw = env::var("TARGET_CENTER_X")
            .expect("The environment variable 'TARGET_CENTER_X' is missing.")
            .parse()
            .unwrap();
        let center_y_raw = env::var("TARGET_CENTER_Y")
            .expect("The environment variable 'TARGET_CENTER_Y' is missing.")
            .parse()
            .unwrap();
        let plate_size_pixel: f32 = env::var("PLATE_WIDTH_PIXELS")
            .expect("The environment variable 'PLATE_WIDTH_PIXELS' is missing.")
            .parse()
            .unwrap();

        // Dimensions physiques réelles de la plaque de jeu (40x40 cm)
        let plate_physical_size_cm: f32 = env::var("PLATE_PHYSICAL_SIZE_CM")
            .expect("The environment variable 'PLATE_PHYSICAL_SIZE_CM' is missing.")
            .parse()
            .unwrap();

        let pixels_per_cm = plate_size_pixel / plate_physical_size_cm;

        Self {
            config: PidConfig {
                kp,
                ki,
                kd,
                dt: 1.0 / fps,
                invert_x,
                invert_y,
            },
            state_x: PidState::new(),
            state_y: PidState::new(),
            center_x_pixel: center_x_raw,
            center_y_pixel: center_y_raw,
            pixels_per_cm,
        }
    }

    /// Calcule la consigne d'inclinaison nécessaire pour corriger la dérive de la bille
    pub fn calculer_inclinaison(&mut self, axe: Axe, ball_position_pixel: f32) -> f32 {
        let dt = self.config.dt;

        let (state, center_pixel, invert) = match axe {
            Axe::X => (&mut self.state_x, self.center_x_pixel, self.config.invert_x),
            Axe::Y => (&mut self.state_y, self.center_y_pixel, self.config.invert_y),
        };

        // Calcul de l'écart spatial en centimètres
        let mut pixel_offset = center_pixel - ball_position_pixel;

        if invert {
            pixel_offset = -pixel_offset;
        }

        let error_cm = (pixel_offset / self.pixels_per_cm).clamp(-20.0, 20.0);

        // 1. Terme Proportionnel (P)
        let p = self.config.kp * error_cm;

        // 2. Terme Intégral (I) avec système de butée (anti-windup)
        if self.config.ki > 0.0 {
            state.integral_sum += error_cm * dt;
            state.integral_sum = state.integral_sum.clamp(-5.0, 5.0);
        }
        let i = self.config.ki * state.integral_sum;

        // 3. Terme Dérivé (D) basé sur la vitesse de déplacement de la bille
        let d = if dt > 0.0 {
            self.config.kd * ((error_cm - state.error_previous) / dt)
        } else {
            0.0
        };

        // Sauvegarde de l'erreur courante pour le prochain cycle
        state.error_previous = error_cm;

        // Normalisation de la sortie du bloc vers un ratio centré autour de 0.5 (plaque plane)
        let pid_output = (p + i + d) / 60.0;
        let plate_inclination = 0.5 + pid_output;

        // Limitation physique pour protéger la course mécanique de vos servomoteurs
        plate_inclination.clamp(0., 1.)
    }

    pub fn angle_from_height(h: f32) -> Result<u16, String> {
        if !(0.0..=1.0).contains(&h) {
            return Err("The normalized height h must be between 0.0 and 1.0.".to_string());
        }

        let a_str = env::var("ARM")
            .map_err(|_| "The environment variable 'ARM' is missing.".to_string())?;
        let arm: f32 = a_str
            .parse()
            .map_err(|_| "Failed to parse 'ARM' into a valid float (f32).".to_string())?;

        let r_str = env::var("ROD")
            .map_err(|_| "The environment variable 'ROD' is missing.".to_string())?;
        let rod: f32 = r_str
            .parse()
            .map_err(|_| "Failed to parse 'ROD' into a valid float (f32).".to_string())?;

        if rod <= arm {
            return Err(
                "Mechanical error: the rod must be strictly longer than the arm."
                    .to_string(),
            );
        }

        let height = (rod - arm) + (2.0 * arm * h);

        // let argument = (height.powi(2) - arm.powi(2) + rod.powi(2)) / (2.0 * rod * height);
        let argument = (height.powi(2) + arm.powi(2) - rod.powi(2)) / (2.0 * arm * height);
        let argument_clamped = argument.clamp(-1.0, 1.0);

        // let theta_base_rad = argument_clamped.asin();
        let theta_base_rad = argument_clamped.acos();
        let theta_base_degrees = theta_base_rad.to_degrees();

        // Transformation pour la plage 90 -> 270
        // let theta_degrees: u16 = (theta_base_degrees + 180.0).round() as u16;
        let theta_degrees: u16 = (270. - theta_base_degrees).round() as u16;

        Ok(theta_degrees)
    }
}
