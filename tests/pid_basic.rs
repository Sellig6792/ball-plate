use physics::Physics;
use pid::{Axe, Pid};

#[test]
fn test_neutral_equilibrium_physics_and_pid() {
    let mut pid = Pid::default();

    // Spawn a ball directly at the system target center (320, 240)
    let mut physics = Physics::new(320.0, 240.0, pid.pixels_per_cm);

    // Warm up/settle the internal servo positions so they don't jump from their default 180.0
    for _ in 0..10 {
        physics.step(0.5, 0.5, pid.config.dt);
    }

    // Teleport the ball back to the center in case the servo alignment nudged it
    physics.pos_x_cm = 320.0 / pid.pixels_per_cm;
    physics.pos_y_cm = 240.0 / pid.pixels_per_cm;
    physics.vel_x = 0.0;
    physics.vel_y = 0.0;

    // Calculate the recommended controller actions
    let cmd_x = pid.calculate_inclination(Axe::X, physics.get_pixel_pos_x());
    let cmd_y = pid.calculate_inclination(Axe::Y, physics.get_pixel_pos_y());

    // An error-free state should command exactly 0.5 (neutral tilt/height)
    assert_eq!(
        cmd_x, 0.5,
        "PID should not tilt the plate if the ball is on target"
    );
    assert_eq!(
        cmd_y, 0.5,
        "PID should not tilt the plate if the ball is on target"
    );

    // Step the physics engine with neutral commands
    physics.step(cmd_x, cmd_y, pid.config.dt);

    // Allow a small pixel gap tolerance due to float-to-integer conversion and friction
    let error_x = (physics.get_pixel_pos_x() - 320).abs();
    let error_y = (physics.get_pixel_pos_y() - 240).abs();

    assert!(error_x <= 3, "Ball settled too far from center X: gap was {} px", error_x);
    assert!(error_y <= 3, "Ball settled too far from center Y: gap was {} px", error_y);
}

#[test]
fn test_extreme_displacement_command_clamping() {
    let mut pid = Pid::default();

    // Spawn a ball completely off the physical edge of the plate
    let mut physics = Physics::new(-1000.0, -1000.0, pid.pixels_per_cm);

    let cmd_x = pid.calculate_inclination(Axe::X, physics.get_pixel_pos_x());
    let cmd_y = pid.calculate_inclination(Axe::Y, physics.get_pixel_pos_y());

    // With a default clamp of 0.0, the output can saturate completely to 0.0 or 1.0
    assert!(
        cmd_x <= 1.0 && cmd_x >= 0.0,
        "Output exceeded raw mechanical constraints"
    );
    assert!(
        cmd_y <= 1.0 && cmd_y >= 0.0,
        "Output exceeded raw mechanical constraints"
    );

    // Ensure the system steps smoothly under saturated conditions without blowing up
    physics.step(cmd_x, cmd_y, pid.config.dt);
    assert!(
        physics.get_pixel_pos_x() > -1000,
        "Ball failed to accelerate inwards"
    );
}