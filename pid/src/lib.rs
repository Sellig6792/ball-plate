mod point;

pub use point::Point;
use std::collections::VecDeque;
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
    pub config: PidConfig,
    state_x: PidState,
    state_y: PidState,
    pub plate_size_in_cm: f32,
    pub center: Point,
    pub target: Point,
    pub target_queue: VecDeque<Point>,
    pub pixels_per_cm: f32,
}

impl Pid {
    pub fn from_env() -> Self {
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
            plate_size_in_cm: plate_physical_size_cm,
            state_x: PidState::new(),
            state_y: PidState::new(),
            center: Point::new(center_x_raw, center_y_raw),
            target: Point::new(center_x_raw, center_y_raw),
            target_queue: VecDeque::new(),
            pixels_per_cm,
        }
    }

    /// Consumes the next waypoint in O(1) if the ball has neared the active target
    pub fn update_trajectory_target(&mut self, current_ball: &Point) {
        if !self.target_queue.is_empty() {
            let distance = (((current_ball.x - self.target.x).pow(2)
                + (current_ball.y - self.target.y).pow(2)) as f32)
                .sqrt();

            let distance_threshold = env::var("TARGET_DISTANCE_THRESHOLD")
                .unwrap_or_else(|_| "15".to_string())
                .parse::<f32>()
                .unwrap();

            if distance < distance_threshold
                && let Some(next_pt) = self.target_queue.pop_front()
            {
                self.target = next_pt;
            }
        }
    }

    pub fn calculate_inclination(&mut self, mut axe: Axe, ball_position_pixel: i32) -> f32 {
        let invert_x_with_y: bool = env::var("INVERT_X_WITH_Y")
            .unwrap_or_else(|_| "false".to_string())
            .parse()
            .unwrap();

        if invert_x_with_y {
            match axe {
                Axe::X => {
                    axe = Axe::Y;
                }
                Axe::Y => {
                    axe = Axe::X;
                }
            }
        }
        let dt = self.config.dt;

        let (state, center_pixel, invert) = match axe {
            Axe::X => (&mut self.state_x, self.target.x, self.config.invert_x),
            Axe::Y => (&mut self.state_y, self.target.y, self.config.invert_y),
        };

        let mut pixel_offset = center_pixel - ball_position_pixel;

        if invert {
            pixel_offset = -pixel_offset;
        }

        let error_cm = (pixel_offset as f32 / self.pixels_per_cm)
            .clamp(-self.plate_size_in_cm / 2.0, self.plate_size_in_cm / 2.0);

        let p = self.config.kp * error_cm;

        if self.config.ki > 0.0 {
            state.integral_sum += error_cm * dt;

            // Scaled Anti-Windup Clamping:
            // Since the final total adjustment power relies on `/ 60.0` scaling,
            // clamping `integral_sum` around +/- 3.0 to 5.0 keeps the max potential
            // integral contribution tightly bound within 10% - 15% of total servo throw,
            // entirely neutralizing windup latency.
            let clamp_integral: f32 = env::var("CLAMP_INTEGRAL")
                .unwrap_or_else(|_| "0.".to_string())
                .parse()
                .unwrap();

            state.integral_sum = state.integral_sum.clamp(-clamp_integral, clamp_integral);
        }
        let i = self.config.ki * state.integral_sum;

        let d = if dt > 0.0 {
            self.config.kd * ((error_cm - state.error_previous) / dt)
        } else {
            0.0
        };

        state.error_previous = error_cm;

        let pid_output = (p + i + d) / (self.plate_size_in_cm / 2.0);
        let plate_inclination = 0.5 + pid_output;

        let clamp_command: f32 = env::var("CLAMP_COMMAND")
            .unwrap_or_else(|_| "0.".to_string())
            .parse()
            .unwrap();
        plate_inclination.clamp(0. + clamp_command, 1. - clamp_command)
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
                "Mechanical error: the rod must be strictly longer than the arm.".to_string(),
            );
        }

        let height = (rod - arm) + (2.0 * arm * h);
        let argument = (height.powi(2) + arm.powi(2) - rod.powi(2)) / (2.0 * arm * height);
        let argument_clamped = argument.clamp(-1.0, 1.0);

        let theta_base_rad = argument_clamped.acos();
        let theta_base_degrees = theta_base_rad.to_degrees();
        let theta_degrees: u16 = (270. - theta_base_degrees).round() as u16;

        Ok(theta_degrees)
    }
}

impl Default for PidConfig {
    fn default() -> Self {
        Self {
            kp: 0.8,
            ki: 0.1,
            kd: 0.35,
            dt: 1.0 / 10.0,
            invert_x: false,
            invert_y: false,
        }
    }
}

impl Default for Pid {
    fn default() -> Self {
        let center_x_raw = 320;
        let center_y_raw = 240;
        let plate_physical_size_cm = 40.0;
        let plate_size_pixel = 640.0;
        let pixels_per_cm = plate_size_pixel / plate_physical_size_cm;

        Self {
            config: PidConfig::default(),
            plate_size_in_cm: plate_physical_size_cm,
            state_x: PidState::new(),
            state_y: PidState::new(),
            center: Point::new(center_x_raw, center_y_raw),
            target: Point::new(center_x_raw, center_y_raw),
            target_queue: VecDeque::new(),
            pixels_per_cm,
        }
    }
}
