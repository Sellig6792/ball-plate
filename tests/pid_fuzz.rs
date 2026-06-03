use physics::Physics;
use pid::{Axe, Pid, Point};

#[test]
fn test_random_positions_converge_to_center_under_30_seconds() {
    dotenvy::dotenv().ok();
    use std::time::{SystemTime, UNIX_EPOCH};

    // Initialize a lightweight pseudo-random number generator (LCG)
    let mut seed = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;

    let mut next_random = || {
        seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        (seed >> 33) as f32 / 2147483647.0 // Returns a float between 0.0 and 1.0
    };

    // Number of randomized fuzzing scenarios to evaluate
    let num_scenarios = 20;

    // Fixed layout and mechanical configuration matching standard defaults

    let plate_size_cm = 40.0;
    let plate_size_pixels = 640.0;
    let pixels_per_cm = plate_size_pixels / plate_size_cm;

    for scenario in 0..num_scenarios {
        // Initialize PID controller via Default (No Env Vars)
        let mut pid = Pid::default();
        pid.plate_size_in_cm = plate_size_cm;
        pid.pixels_per_cm = pixels_per_cm;

        pid.config.dt = 1.0 / 10.0; // 10Hz update rate
        pid.config.invert_x = false;
        pid.config.invert_y = false;

        // Target center is fixed at (320, 240)
        let center_x = 320;
        let center_y = 240;
        pid.center = Point::new(center_x, center_y);
        pid.target = Point::new(center_x, center_y);

        let border_margin = 40.0;
        let ball_start_x_px =
            border_margin + (next_random() * (plate_size_pixels - (border_margin * 2.0)));
        let ball_start_y_px =
            border_margin + (next_random() * (plate_size_pixels - (border_margin * 2.0)));

        // 2. RANDOM VELOCITY: Random initial speed vectors between -15.0 cm/s and +15.0 cm/s
        let ball_start_vel_x = -15.0 + (next_random() * 30.0);
        let ball_start_vel_y = -15.0 + (next_random() * 30.0);

        // Manually assemble Physics to bypass environment lookups entirely
        let mut physics = Physics {
            pos_x_cm: ball_start_x_px / pixels_per_cm,
            vel_x: ball_start_vel_x,
            current_servo_degrees_x: 180.0,

            pos_y_cm: ball_start_y_px / pixels_per_cm,
            vel_y: ball_start_vel_y,
            current_servo_degrees_y: 180.0,

            arm: std::env::var("ARM").unwrap().parse().unwrap(),
            rod: std::env::var("ROD").unwrap().parse().unwrap(),
            pixels_per_cm,
            crosstalk_x_into_y: 0.04,
            crosstalk_y_into_x: 0.02,
        };

        // Closed-loop simulation constraints
        let max_seconds = 20.0;
        let max_steps = (max_seconds / pid.config.dt).round() as usize;

        let acceptable_error_pixels = 4.0;
        let consecutive_steps_required = 10;
        let mut stabilization_counter = 0;
        let mut converged_in_time = false;

        for _step in 0..max_steps {
            let ball_x = physics.get_pixel_pos_x();
            let ball_y = physics.get_pixel_pos_y();

            // Run closed-loop iteration
            let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
            let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);
            physics.step(cmd_x, cmd_y, pid.config.dt);

            // Calculate Euclidean distance error to the target center
            let error_x = (ball_x - pid.target.x) as f32;
            let error_y = (ball_y - pid.target.y) as f32;
            let current_distance = (error_x.powi(2) + error_y.powi(2)).sqrt();

            // Check if the ball remains within the acceptable error bounds
            if current_distance <= acceptable_error_pixels {
                stabilization_counter += 1;
            } else {
                stabilization_counter = 0; // Reset if it slips or overshoots out of bounds
            }

            if stabilization_counter >= consecutive_steps_required {
                converged_in_time = true;
                break;
            }
        }

        // Assertions checking that this specific physical state successfully converged
        assert!(
            converged_in_time,
            "Fuzz Failure: Scenario #{} failed to reach target center within {} seconds.\n\
             Initial Position -> X: {:.1}px, Y: {:.1}px\n\
             Initial Velocity -> X: {:.2} cm/s, Y: {:.2} cm/s\n\
             Final Position   -> X: {}px, Y: {}px",
            scenario,
            max_seconds,
            ball_start_x_px,
            ball_start_y_px,
            ball_start_vel_x,
            ball_start_vel_y,
            physics.get_pixel_pos_x(),
            physics.get_pixel_pos_y()
        );
    }
}
