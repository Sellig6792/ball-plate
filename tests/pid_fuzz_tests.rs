// tests/pid_fuzz_tests.rs
use pid::{Axe, Pid};

mod common;
use common::PhysicsEngine2D;

fn generate_pseudo_random(seed: &mut u32, min: f32, max: f32) -> f32 {
    *seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
    let raw = (*seed & 0x7FFFFFFF) as f32 / 0x7FFFFFFF as f32;
    min + raw * (max - min)
}

#[test]
fn test_monte_carlo_100_random_stabilizations() {
    let mut seed = 1234;
    let mut failed_runs = 0;

    for _ in 0..100 {
        let mut pid = Pid::default();
        let dt = pid.config.dt;

        let start_x = generate_pseudo_random(&mut seed, -15.0, 15.0);
        let start_y = generate_pseudo_random(&mut seed, -15.0, 15.0);

        let mut physics =
            PhysicsEngine2D::new(start_x, start_y, pid.plate_size_in_cm, pid.pixels_per_cm);

        // Run the 2D simulation for 200 frames
        for _ in 0..200 {
            let pixel_x = physics.get_pixel_pos_x(pid.target.x);
            let pixel_y = physics.get_pixel_pos_y(pid.target.y);

            let incl_x = pid.calculate_inclination(Axe::X, pixel_x);
            let incl_y = pid.calculate_inclination(Axe::Y, pixel_y);

            let phys_incl_x = if pid.config.invert_x {
                1.0 - incl_x
            } else {
                incl_x
            };
            let phys_incl_y = if pid.config.invert_y {
                1.0 - incl_y
            } else {
                incl_y
            };

            physics.step(phys_incl_x, phys_incl_y, dt);
        }

        // Check if the final position is centered within 0.5 cm
        let is_centered_x = physics.pos_x_cm.abs() < 0.5;
        let is_centered_y = physics.pos_y_cm.abs() < 0.5;

        if !is_centered_x || !is_centered_y {
            failed_runs += 1;
        }
    }

    assert_eq!(
        failed_runs, 0,
        "Out of 100 random runs, {} failed to settle at the center.",
        failed_runs
    );
}
