use physics::Physics;
use pid::{Axe, Pid, Point};

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