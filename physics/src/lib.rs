// physics/src/lib.rs
// Core 2D Physics Library with integrated pixel translation layer and structural crosstalk

use std::f32::consts::PI;

pub struct Physics {
    // Internal Physical State (Centimeters)
    pub pos_x_cm: f32,
    pub vel_x: f32,
    pub current_servo_degrees_x: f32,

    pub pos_y_cm: f32,
    pub vel_y: f32,
    pub current_servo_degrees_y: f32,

    // Linkage Dimensions & Scale Configuration
    pub arm: f32,
    pub rod: f32,
    pub pixels_per_cm: f32,

    // Structural Imperfection / Cross-coupling coefficients (Percentage of bleed)
    pub crosstalk_x_into_y: f32, // E.g., 0.05 means 5% of X tilt bleeds into Y acceleration
    pub crosstalk_y_into_x: f32, // E.g., 0.02 means 2% of Y tilt bleeds into X acceleration
}

impl Physics {
    /// Creates a new 2D physics instance. Converts starting positions from pixels to cm internally.
    pub fn new(start_pixel_x: f32, start_pixel_y: f32, pixels_per_cm: f32) -> Self {
        let _ = dotenvy::dotenv();

        let arm: f32 = std::env::var("ARM")
            .expect("Environment variable 'ARM' is missing.")
            .parse()
            .expect("Failed to parse 'ARM' into f32.");

        let rod: f32 = std::env::var("ROD")
            .expect("Environment variable 'ROD' is missing.")
            .parse()
            .expect("Failed to parse 'ROD' into f32.");

        if rod <= arm {
            panic!("Mechanical error: 'ROD' must be strictly longer than 'ARM'.");
        }

        // Define the structural imperfection parameters.
        // You can also pull these from environment variables if desired.
        let crosstalk_x_into_y = 0.06; // 6% bleed effect
        let crosstalk_y_into_x = 0.03; // 3% bleed effect

        Self {
            pos_x_cm: start_pixel_x / pixels_per_cm,
            vel_x: 0.0,
            current_servo_degrees_x: 180.0,

            pos_y_cm: start_pixel_y / pixels_per_cm,
            vel_y: 0.0,
            current_servo_degrees_y: 180.0,

            arm,
            rod,
            pixels_per_cm,
            crosstalk_x_into_y,
            crosstalk_y_into_x,
        }
    }

    /// Returns the current X position transformed into camera pixels
    pub fn get_pixel_pos_x(&self) -> i32 {
        (self.pos_x_cm * self.pixels_per_cm).round() as i32
    }

    /// Returns the current Y position transformed into camera pixels
    pub fn get_pixel_pos_y(&self) -> i32 {
        (self.pos_y_cm * self.pixels_per_cm).round() as i32
    }

    /// Advances the simulation. Inputs are normalized inclinations [0.0 - 1.0].
    pub fn step(&mut self, cmd_h_x: f32, cmd_h_y: f32, dt: f32) {
        let max_degrees_this_frame = 333.0 * dt;
        let g = 981.0;
        let max_plate_tilt_rad = 12.0 * (PI / 180.0);
        let stiction_threshold = 1.2;
        let rolling_resistance = 0.91;

        // ==========================================
        // 1. RESOLVE INDEPENDENT SERVO POSITIONING
        // ==========================================

        // --- X-AXIS SERVO kinematics ---
        let height_x = (self.rod - self.arm) + (2.0 * self.arm * cmd_h_x);
        let argument_x =
            (height_x.powi(2) + self.arm.powi(2) - self.rod.powi(2)) / (2.0 * self.arm * height_x);
        let target_theta_x = 270.0 - argument_x.clamp(-1.0, 1.0).acos().to_degrees();
        let diff_x = target_theta_x - self.current_servo_degrees_x;
        self.current_servo_degrees_x +=
            diff_x.clamp(-max_degrees_this_frame, max_degrees_this_frame);

        // --- Y-AXIS SERVO kinematics ---
        let height_y = (self.rod - self.arm) + (2.0 * self.arm * cmd_h_y);
        let argument_y =
            (height_y.powi(2) + self.arm.powi(2) - self.rod.powi(2)) / (2.0 * self.arm * height_y);
        let target_theta_y = 270.0 - argument_y.clamp(-1.0, 1.0).acos().to_degrees();
        let diff_y = target_theta_y - self.current_servo_degrees_y;
        self.current_servo_degrees_y +=
            diff_y.clamp(-max_degrees_this_frame, max_degrees_this_frame);

        // ==========================================
        // 2. COMPUTE PURE PHYSICAL PLATE TILTS
        // ==========================================
        let tilt_dev_rad_x = (self.current_servo_degrees_x - 180.0).to_radians();
        let pure_plate_tilt_rad_x = tilt_dev_rad_x.sin() * max_plate_tilt_rad;

        let tilt_dev_rad_y = (self.current_servo_degrees_y - 180.0).to_radians();
        let pure_plate_tilt_rad_y = tilt_dev_rad_y.sin() * max_plate_tilt_rad;

        // ==========================================
        // 3. APPLY STRUCTURAL CROSS-COUPLING
        // ==========================================
        // The effective tilt driving acceleration on each axis is now composite
        let effective_tilt_x =
            pure_plate_tilt_rad_x + (pure_plate_tilt_rad_y * self.crosstalk_y_into_x);
        let effective_tilt_y =
            pure_plate_tilt_rad_y + (pure_plate_tilt_rad_x * self.crosstalk_x_into_y);

        // ==========================================
        // 4. CALCULATE DYNAMICS AND STEP POSITIONS
        // ==========================================

        // --- X-AXIS DYNAMICS ---
        let mut accel_x = (2.0 / 3.0) * g * effective_tilt_x.sin();
        if self.vel_x.abs() < 0.05 && accel_x.abs() < stiction_threshold {
            accel_x = 0.0;
        }
        self.vel_x += accel_x * dt;
        self.vel_x *= rolling_resistance;
        self.pos_x_cm += self.vel_x * dt;

        // --- Y-AXIS DYNAMICS ---
        let mut accel_y = (2.0 / 3.0) * g * effective_tilt_y.sin();
        if self.vel_y.abs() < 0.05 && accel_y.abs() < stiction_threshold {
            accel_y = 0.0;
        }
        self.vel_y += accel_y * dt;
        self.vel_y *= rolling_resistance;
        self.pos_y_cm += self.vel_y * dt;
    }
}
