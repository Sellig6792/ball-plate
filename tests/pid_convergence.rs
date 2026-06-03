use physics::Physics;
use pid::{Axe, Pid, Point};
use rand::distr::{Distribution, Uniform};
use std::thread::sleep;
use std::time::Duration;

#[test]
fn test_physics_driven_waypoint_stepping() {
    let mut pid = Pid::default();

    // Establish a waypoint queue
    let wp1 = Point::new(330, 240); // Close to the start point
    let wp2 = Point::new(400, 240); // Farther out
    pid.target_queue.push_back(wp1);
    pid.target_queue.push_back(wp2);

    // Start the ball exactly at the original center (320, 240)
    let mut physics = Physics::new(320.0, 240.0, pid.pixels_per_cm);

    // Initial evaluation populates wp1 as the active target because distance <= 10
    pid.update_trajectory_target(&Point::new(
        physics.get_pixel_pos_x(),
        physics.get_pixel_pos_y(),
    ));
    assert_eq!(pid.target.x, wp1.x, "Failed to load the first waypoint");

    let mut transitioned_to_wp2 = false;

    // Run a closed-loop execution loop to see if the physical movement triggers the next target shift
    for _ in 0..40 {
        let ball_x = physics.get_pixel_pos_x();
        let ball_y = physics.get_pixel_pos_y();

        // Evaluate waypoint conditions based on the ball's current physical position
        pid.update_trajectory_target(&Point::new(ball_x, ball_y));

        if pid.target.x == wp2.x {
            transitioned_to_wp2 = true;
            break;
        }

        let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
        let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);
        physics.step(cmd_x, cmd_y, pid.config.dt);
    }

    assert!(
        transitioned_to_wp2,
        "The physical rolling path of the ball failed to advance the target queue to Waypoint 2"
    );
}

#[test]
fn test_ball_converges_and_settles() {
    let mut pid = Pid::default(); // Keeping default configuration untouched

    // --- CALCULATE RANDOM POSITION ON A SQUARE PLATE ---
    let mut rng = rand::rng();

    // The plate boundary extends from -half_length to +half_length from the center
    let half_length_cm = pid.plate_size_in_cm / 2.0;
    let safety_padding_cm = 2.0;
    let max_bound_cm = half_length_cm - safety_padding_cm;

    // Uniformly sample across the square surface boundaries
    let range = Uniform::try_from(-max_bound_cm..max_bound_cm).unwrap();
    let random_offset_x_cm = range.sample(&mut rng);
    let random_offset_y_cm = range.sample(&mut rng);

    // Map the random centimeter offsets relative to the target center pixel (320, 240)
    let start_pixel_x = 320.0 + (random_offset_x_cm * pid.pixels_per_cm);
    let start_pixel_y = 240.0 + (random_offset_y_cm * pid.pixels_per_cm);

    let mut physics = Physics::new(start_pixel_x, start_pixel_y, pid.pixels_per_cm);

    // Warm up/settle virtual servos using the default loop interval
    for _ in 0..10 {
        physics.step(0.5, 0.5, pid.config.dt);
    }

    // Reset positions after warmup
    physics.pos_x_cm = start_pixel_x / pid.pixels_per_cm;
    physics.pos_y_cm = start_pixel_y / pid.pixels_per_cm;
    physics.vel_x = 0.0;
    physics.vel_y = 0.0;

    // --- EXTENDED TO 60 SECONDS MAX ---
    // 60.0 seconds / 0.033 dt = ~1818 frames maximum capacity
    let max_duration_secs = 60.0;
    let max_frames = (max_duration_secs / pid.config.dt).floor() as usize;

    let mut converged = false;
    let tolerance = 8;

    // --- SETTLEMENT TRACKING VARIABLES ---
    let mut consecutive_stable_frames = 0;
    let required_stable_frames = 15; // Must stay at the center for 15 frames (~0.5 seconds)

    for _frame in 0..max_frames {
        // --- STRICT BOUNDARY GUARDRAIL ---
        // Calculate the ball's current physical distance from the center of the plate
        let center_x_cm = 320.0 / pid.pixels_per_cm;
        let center_y_cm = 240.0 / pid.pixels_per_cm;

        let distance_x_cm = (physics.pos_x_cm - center_x_cm).abs();
        let distance_y_cm = (physics.pos_y_cm - center_y_cm).abs();

        // Immediately panic/fail if the ball exceeds the edge of the square plate
        assert!(
            distance_x_cm <= half_length_cm && distance_y_cm <= half_length_cm,
            "Test failed: The ball rolled off the plate! X distance: {:.2}cm, Y distance: {:.2}cm (Limit: {:.2}cm)",
            distance_x_cm,
            distance_y_cm,
            half_length_cm
        );

        let ball_x = physics.get_pixel_pos_x();
        let ball_y = physics.get_pixel_pos_y();

        // Evaluate if the ball is within the target center boundaries
        if (ball_x - 320).abs() <= tolerance && (ball_y - 240).abs() <= tolerance {
            consecutive_stable_frames += 1;

            // Success condition: Ball came to a rest/maintained center position long enough
            if consecutive_stable_frames >= required_stable_frames {
                converged = true;
                break;
            }
        } else {
            // Reset the counter if the ball overshoots or rolls out of the tolerance zone
            consecutive_stable_frames = 0;
        }

        // Execute closed-loop control step
        let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
        let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);
        physics.step(cmd_x, cmd_y, pid.config.dt);
    }

    assert!(
        converged,
        "Ball failed to reach and settle on the center within the 60-second limit."
    );
}

#[test]
fn test_ball_converges_with_velocity() {
    // --- DECODE CUSTOM COMMAND LINE ARGUMENTS ---
    // Check if the user appended "--show-visuals" to the cargo test command
    let show_visuals = std::env::var("SHOW_VISUALS").is_ok();

    let mut pid = Pid::default(); // Keeping default configuration untouched

    // --- CALCULATE RANDOM POSITION ON A SQUARE PLATE ---
    let mut rng = rand::rng();

    let half_length_cm = pid.plate_size_in_cm / 2.0;
    let safety_padding_cm = 2.0;
    let max_bound_cm = half_length_cm - safety_padding_cm;

    let pos_range = Uniform::try_from(-max_bound_cm..max_bound_cm).unwrap();
    let random_offset_x_cm = pos_range.sample(&mut rng);
    let random_offset_y_cm = pos_range.sample(&mut rng);

    let start_pixel_x = 320.0 + (random_offset_x_cm * pid.pixels_per_cm);
    let start_pixel_y = 240.0 + (random_offset_y_cm * pid.pixels_per_cm);

    let mut physics = Physics::new(start_pixel_x, start_pixel_y, pid.pixels_per_cm);

    // Warm up/settle virtual servos
    for _ in 0..10 {
        physics.step(0.5, 0.5, pid.config.dt);
    }

    // Inject initial velocity bounds (-25.0 cm/s to +25.0 cm/s)
    let vel_range = Uniform::try_from(-25.0..25.0).unwrap();
    let initial_vel_x = vel_range.sample(&mut rng);
    let initial_vel_y = vel_range.sample(&mut rng);

    physics.pos_x_cm = start_pixel_x / pid.pixels_per_cm;
    physics.pos_y_cm = start_pixel_y / pid.pixels_per_cm;
    physics.vel_x = initial_vel_x;
    physics.vel_y = initial_vel_y;

    let max_duration_secs = 60.0;
    let max_frames = (max_duration_secs / pid.config.dt).floor() as usize;

    let mut converged = false;
    let tolerance = 8;

    let mut consecutive_stable_frames = 0;
    let required_stable_frames = 15;

    // CLI Grid Rendering Dimensions
    let grid_width = 40;
    let grid_height = 15;

    for frame in 0..max_frames {
        // --- STRICT BOUNDARY GUARDRAIL ---
        let center_x_cm = 320.0 / pid.pixels_per_cm;
        let center_y_cm = 240.0 / pid.pixels_per_cm;

        let distance_x_cm = (physics.pos_x_cm - center_x_cm).abs();
        let distance_y_cm = (physics.pos_y_cm - center_y_cm).abs();

        assert!(
            distance_x_cm <= half_length_cm && distance_y_cm <= half_length_cm,
            "Test failed: The ball rolled off the plate! X distance: {:.2}cm, Y distance: {:.2}cm (Limit: {:.2}cm)",
            distance_x_cm,
            distance_y_cm,
            half_length_cm
        );

        let ball_x = physics.get_pixel_pos_x();
        let ball_y = physics.get_pixel_pos_y();

        // --- OPTIONAL TERMINAL VISUALIZATION ENGINE ---
        if show_visuals {
            let ball_col = ((ball_x as f32 / 640.0) * grid_width as f32) as i32;
            let ball_row = ((ball_y as f32 / 480.0) * grid_height as f32) as i32;
            let center_col = ((320.0 / 640.0) * grid_width as f32) as i32;
            let center_row = ((240.0 / 480.0) * grid_height as f32) as i32;

            // Clear frame and dump grid matrix to stdout
            print!("\x1B[2J\x1B[1;1H");
            println!(
                "Frame: {}/{} | Position: ({}, {})",
                frame, max_frames, ball_x, ball_y
            );
            println!("{}+", "-".repeat(grid_width as usize));

            for r in 0..grid_height {
                print!("|");
                for c in 0..grid_width {
                    if r == ball_row && c == ball_col {
                        print!("●");
                    } else if r == center_row && c == center_col {
                        print!("+");
                    } else {
                        print!(" ");
                    }
                }
                println!("|");
            }
            println!("{}+", "-".repeat(grid_width as usize));

            // Slow down the execution loops to mimic natural simulation frame delays
            sleep(Duration::from_millis(33));
        }

        // Evaluate settlement window
        if (ball_x - 320).abs() <= tolerance && (ball_y - 240).abs() <= tolerance {
            consecutive_stable_frames += 1;
            if consecutive_stable_frames >= required_stable_frames {
                converged = true;
                break;
            }
        } else {
            consecutive_stable_frames = 0;
        }

        // Execute closed-loop control step
        let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
        let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);
        physics.step(cmd_x, cmd_y, pid.config.dt);
    }

    assert!(
        converged,
        "Ball failed to reach and settle on the center within 60 seconds."
    );
}
