#![allow(dead_code)]

/// A simplified 2D physics engine simulating a ping-pong ball on a tilted plate.
pub struct PhysicsEngine2D {
    pub pos_x_cm: f32,
    pub pos_y_cm: f32,
    pub vel_x_cm_s: f32,
    pub vel_y_cm_s: f32,
    pub plate_size_cm: f32,
    pub pixels_per_cm: f32,
}

impl PhysicsEngine2D {
    pub fn new(start_x: f32, start_y: f32, plate_size_cm: f32, pixels_per_cm: f32) -> Self {
        Self {
            pos_x_cm: start_x,
            pos_y_cm: start_y,
            vel_x_cm_s: 0.0,
            vel_y_cm_s: 0.0,
            plate_size_cm,
            pixels_per_cm,
        }
    }

    /// Advances the simulation by one time step dt
    pub fn step(&mut self, incl_x: f32, incl_y: f32, dt: f32) {
        // Translate inclinations (0.5 = flat, 0.9 = max tilt right/up, 0.1 = max tilt left/down)
        let angle_x_rad = (incl_x - 0.5) * 2.0 * 0.26;
        let angle_y_rad = (incl_y - 0.5) * 2.0 * 0.26;

        // Acceleration = gravity component + air/friction resistance
        let acc_x = (981.0 * angle_x_rad.sin()) + (-0.15 * self.vel_x_cm_s);
        let acc_y = (981.0 * angle_y_rad.sin()) + (-0.15 * self.vel_y_cm_s);

        // Euler integration for X
        self.vel_x_cm_s += acc_x * dt;
        self.pos_x_cm += self.vel_x_cm_s * dt;

        // Euler integration for Y
        self.vel_y_cm_s += acc_y * dt;
        self.pos_y_cm += self.vel_y_cm_s * dt;

        // Boundaries' clamps (ball cannot leave the physical plate)
        let max_bound = self.plate_size_cm / 2.0;
        self.pos_x_cm = self.pos_x_cm.clamp(-max_bound, max_bound);
        self.pos_y_cm = self.pos_y_cm.clamp(-max_bound, max_bound);
    }

    pub fn get_pixel_pos_x(&self, center_x: i32) -> i32 {
        center_x + (self.pos_x_cm * self.pixels_per_cm).round() as i32
    }

    pub fn get_pixel_pos_y(&self, center_y: i32) -> i32 {
        center_y + (self.pos_y_cm * self.pixels_per_cm).round() as i32
    }
}
