use dotenv::dotenv;
use std::env;

#[derive(Debug, Clone, Copy)]
pub enum Axe {
    X = 0,
    Y = 1,
}

pub struct PidConfig {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub dt: f32,
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

pub struct BallAndPlatePid {
    config: PidConfig,
    state_x: PidState,
    state_y: PidState,
    center_x_pixel: f32,
    center_y_pixel: f32,
    pixels_per_cm: f32,
}

impl BallAndPlatePid {
    /// Construit le régulateur PID à partir des variables d'environnement globales et manuelles
    pub fn from_env(center_x_raw: f32, center_y_raw: f32, plate_size_pixel: f32) -> Self {
        dotenv().ok();

        let kp: f32 = env::var("PID_KP")
            .unwrap_or_else(|_| "1.5".to_string())
            .parse()
            .unwrap();
        let ki: f32 = env::var("PID_KI")
            .unwrap_or_else(|_| "0.0".to_string())
            .parse()
            .unwrap();
        let kd: f32 = env::var("PID_KD")
            .unwrap_or_else(|_| "0.3".to_string())
            .parse()
            .unwrap();
        let fps: f32 = env::var("FRAME_RATE")
            .unwrap_or_else(|_| "20".to_string())
            .parse()
            .unwrap();

        // Dimensions physiques réelles de la plaque de jeu (40x40 cm)
        let plate_physical_size_cm = 40.0;
        let pixels_per_cm = plate_size_pixel / plate_physical_size_cm;

        Self {
            config: PidConfig {
                kp,
                ki,
                kd,
                dt: 1.0 / fps,
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

        let (state, center_pixel) = match axe {
            Axe::X => (&mut self.state_x, self.center_x_pixel),
            Axe::Y => (&mut self.state_y, self.center_y_pixel),
        };

        // Calcul de l'écart spatial en centimètres (inversion de signe automatique selon le côté)
        let pixel_offset = center_pixel - ball_position_pixel;
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
        let pid_output = (p + i + d) / 100.0;
        let plate_inclination = 0.5 + pid_output;

        // Limitation physique pour protéger la course mécanique de vos servomoteurs
        plate_inclination.clamp(0.2, 0.8)
    }
}
