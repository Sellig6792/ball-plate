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

#[test]
fn test_moving_trajectory_tracking() {
    let mut pid = Pid::default();

    let initial_x = 320.0;
    let initial_y = 240.0;
    let mut physics = Physics::new(initial_x, initial_y, pid.pixels_per_cm);

    let radius = 50.0;
    let steps = 60; // Réduit pour une animation CLI fluide

    // Vérification de la variable d'environnement pour activer le rendu
    let show_visualizer = std::env::var("CLI_VIEW")
        .unwrap_or_else(|_| "false".to_string())
        .parse::<bool>()
        .unwrap_or(false);

    for i in 0..steps {
        let angle = (i as f32) * 0.15;
        let target_x = (initial_x + radius * angle.cos()).round() as i32;
        let target_y = (initial_y + radius * angle.sin()).round() as i32;

        pid.target = Point::new(target_x, target_y);

        let ball_x = physics.get_pixel_pos_x();
        let ball_y = physics.get_pixel_pos_y();

        let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
        let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);

        physics.step(cmd_x, cmd_y, pid.config.dt);

        // --- BLOC DE VISUALISATION CLI ---
        if show_visualizer {
            // Nettoie le terminal et replace le curseur en haut à gauche
            print!("{}[2J{}[1;1H", 27 as char, 27 as char);

            println!("=== SIMULATION TRAJECTOIRE (Étape {}/{}) ===", i + 1, steps);
            println!("Cible  : X: {:3}, Y: {:3}", target_x, target_y);
            println!("Bille  : X: {:3}, Y: {:3}", ball_x, ball_y);
            println!("-----------------------------------------");

            // Dimensions de la grille de rendu CLI
            let grid_width = 40;
            let grid_height = 20;

            // Projection des coordonnées (0-640 et 0-480) vers la taille de la grille CLI
            let scale_x = grid_width as f32 / 640.0;
            let scale_y = grid_height as f32 / 480.0;

            let b_grid_x = ((ball_x as f32) * scale_x).round() as i32;
            let b_grid_y = ((ball_y as f32) * scale_y).round() as i32;

            let t_grid_x = ((target_x as f32) * scale_x).round() as i32;
            let t_grid_y = ((target_y as f32) * scale_y).round() as i32;

            // Rendu de la matrice
            for y in 0..grid_height {
                for x in 0..grid_width {
                    if x == b_grid_x && y == b_grid_y {
                        print!("🔴"); // La bille (ou 'O' si votre terminal ne supporte pas les emoji)
                    } else if x == t_grid_x && y == t_grid_y {
                        print!("🎯"); // La cible (ou 'X')
                    } else if x == 0 || x == grid_width - 1 || y == 0 || y == grid_height - 1 {
                        print!("#"); // Les bordures de la plaque
                    } else {
                        print!(" "); // Espace vide
                    }
                }
                println!();
            }

            // Ralentit l'affichage pour le rendre visible à l'œil humain
            std::thread::sleep(std::time::Duration::from_millis(50));
        }
        // ---------------------------------
    }

    let final_ball_x = physics.get_pixel_pos_x();
    let final_ball_y = physics.get_pixel_pos_y();
    let distance_from_center = (((final_ball_x - initial_x as i32).pow(2)
        + (final_ball_y - initial_y as i32).pow(2)) as f32)
        .sqrt();

    assert!(distance_from_center > 10.0);
}
