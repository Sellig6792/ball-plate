use physics::Physics;
use pid::{Axe, Pid, Point};
pub use plotters::prelude::*;

#[derive(Clone, Copy)]
pub struct Situation {
    pub start_pos: f32, // Starting displacement from center in cm
    pub start_vel: f32, // Starting velocity in cm/s
}

// Fixed E0277 error by adding Clone and Copy traits
#[derive(Clone, Copy)]
struct EvaluationResult {
    avg_success_time_ms: f32,
    success_count: usize,
}

/// Simulates a single scenario using the actual Pid and Physics crates.
fn simulate_single_situation(kp: f32, ki: f32, kd: f32, dt: f32, situation: Situation) -> Option<f32> {
    let mut pid = Pid::default();
    pid.config.kp = kp;
    pid.config.ki = ki;
    pid.config.kd = kd;
    pid.config.dt = dt;

    // Aligned with working diagnostic trace setup
    pid.config.invert_x = false;
    pid.config.invert_y = false;
    pid.target = Point::new(320, 240);

    // Map physical positions properly relative to the center pixel coordinate space
    let start_pixel_x = 320.0 + (situation.start_pos * pid.pixels_per_cm);
    let mut physics = Physics::new(start_pixel_x, 240.0, pid.pixels_per_cm);

    // Settle servo math states before tracking
    for _ in 0..10 {
        physics.step(0.5, 0.5, pid.config.dt);
    }

    // Set scenario conditions
    physics.pos_x_cm = start_pixel_x / pid.pixels_per_cm;
    physics.pos_y_cm = 240.0 / pid.pixels_per_cm;
    physics.vel_x = situation.start_vel;
    physics.vel_y = 0.0;

    let max_frames = 250;
    let limit_cm = pid.plate_size_in_cm / 2.0;

    let precision_threshold_cm = 0.15;
    let velocity_threshold_cms = 0.5;
    let mut consecutive_stable_frames = 0;
    let required_stable_frames = 15;

    for frame in 1..=max_frames {
        let ball_x = physics.get_pixel_pos_x();
        let ball_y = physics.get_pixel_pos_y();

        let cmd_x = pid.calculate_inclination(Axe::X, ball_x);
        let cmd_y = pid.calculate_inclination(Axe::Y, ball_y);

        physics.step(cmd_x, cmd_y, pid.config.dt);

        let center_cm = 320.0 / pid.pixels_per_cm;
        let distance_from_center_cm = (physics.pos_x_cm - center_cm).abs();

        // Out of bounds safety check
        if distance_from_center_cm > limit_cm {
            return None;
        }

        // Convergence evaluation criteria
        if distance_from_center_cm < precision_threshold_cm && physics.vel_x.abs() < velocity_threshold_cms {
            consecutive_stable_frames += 1;
            if consecutive_stable_frames >= required_stable_frames {
                let stable_frame = frame - required_stable_frames;
                return Some((stable_frame as f32) * pid.config.dt * 1000.0);
            }
        } else {
            consecutive_stable_frames = 0;
        }
    }

    None
}

fn evaluate_pid_performance(kp: f32, ki: f32, kd: f32, dt: f32, situations: &[Situation; 100]) -> EvaluationResult {
    let mut total_time = 0.0;
    let mut success_count = 0;

    for &situation in situations {
        if let Some(time) = simulate_single_situation(kp, ki, kd, dt, situation) {
            total_time += time;
            success_count += 1;
        }
    }

    let avg_success_time_ms = if success_count > 0 {
        total_time / success_count as f32
    } else {
        8000.0
    };

    EvaluationResult { avg_success_time_ms, success_count }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let dt = 0.033;

    // --- SCENARIO SETUPS ---
    let mut situations = [Situation { start_pos: 0.0, start_vel: 0.0 }; 100];
    for i in 0..100 {
        let progression = i as f32 / 99.0;
        let direction = if i % 2 == 0 { 1.0 } else { -1.0 };

        situations[i] = Situation {
            start_pos: direction * (1.0 + progression * 12.0),
            start_vel: -direction * (progression * 50.0),
        };
    }

    // --- EXPANDED UPPER BOUND MATRIX SWEEP RANGES ---
    let mut kp_values = Vec::new();
    for i in 1..=30 { kp_values.push(7.5 + (i as f32 * 0.5)); }

    let mut kd_values = Vec::new();
    for i in 1..=15 { kd_values.push(0.3 + (i as f32 * 0.08)); }

    let ki_values = [0.000, 0.001, 0.002];

    let total_x_cells = kd_values.len() * ki_values.len();
    let total_y_cells = kp_values.len();

    // --- TRACKING PERFECT CONFIGURATIONS ---
    let mut fastest_perfect_time = 8000.0;
    let mut perfect_winner: Option<(f32, f32, f32)> = None;

    // --- RUN FAST SINGLE PASS EXECUTION ---
    let mut results_grid = vec![vec![None; total_x_cells]; total_y_cells];
    let mut best_avg_time = 8000.0;
    let minimum_acceptable_successes = 50;

    println!("Running matrix optimization sweep...");
    for (y_idx, &kp) in kp_values.iter().enumerate() {
        for (d_idx, &kd) in kd_values.iter().enumerate() {
            for (i_idx, &ki) in ki_values.iter().enumerate() {
                let x_idx = d_idx * ki_values.len() + i_idx;

                let res = evaluate_pid_performance(kp, ki, kd, dt, &situations);

                if res.success_count >= minimum_acceptable_successes && res.avg_success_time_ms < best_avg_time {
                    best_avg_time = res.avg_success_time_ms;
                }

                // Log the absolute fastest configuration that hits 100% success rate
                if res.success_count == 100 && res.avg_success_time_ms < fastest_perfect_time {
                    fastest_perfect_time = res.avg_success_time_ms;
                    perfect_winner = Some((kp, ki, kd));
                }

                results_grid[y_idx][x_idx] = Some(res);
            }
        }
    }

    // --- RENDERING CONFIGURATION ---
    let root = BitMapBackend::new("pid_massive_heatmap.png", (1600, 1000)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Massive PID Heatmap (Average Time of Successful Runs Only)", ("sans-serif", 24).into_font())
        .margin(30)
        .x_label_area_size(70)
        .y_label_area_size(70)
        .build_cartesian_2d(0..total_x_cells, 0..total_y_cells)?;

    chart.configure_mesh()
        .x_desc("Derivative Gain (Kd) / Integral Gain (Ki)")
        .y_desc("Proportional Gain (Kp)")
        .x_label_formatter(&|&x| {
            let d_idx = x / ki_values.len();
            let i_idx = x % ki_values.len();
            if d_idx < kd_values.len() && i_idx == 0 {
                format!("{:.3} / {:.3}", kd_values[d_idx], ki_values[i_idx])
            } else {
                "".to_string()
            }
        })
        .y_label_formatter(&|&y| {
            if y < kp_values.len() {
                format!("{:.3}", kp_values[y])
            } else {
                "".to_string()
            }
        })
        .label_style(("sans-serif", 11))
        .disable_x_mesh()
        .disable_y_mesh()
        .draw()?;

    // --- MAP CANVAS BLOCKS ---
    for y_idx in 0..total_y_cells {
        let kp = kp_values[y_idx];
        for d_idx in 0..kd_values.len() {
            let kd = kd_values[d_idx];
            for i_idx in 0..ki_values.len() {
                let ki = ki_values[i_idx];
                let x_idx = d_idx * ki_values.len() + i_idx;

                let res = results_grid[y_idx][x_idx].as_ref().unwrap();

                let color = if res.success_count == 0 {
                    RED.mix(0.9)
                } else if (res.avg_success_time_ms - best_avg_time).abs() < 0.01 && best_avg_time < 8000.0 && res.success_count >= minimum_acceptable_successes {
                    println!("=> ROBUST SUCCESS WINNER: Kp: {:.3} | Ki: {:.3} | Kd: {:.3} -> Avg time when successful: {:.1} ms (Passed {}/100)",
                             kp, ki, kd, res.avg_success_time_ms, res.success_count);
                    BLUE.mix(1.0)
                } else if res.avg_success_time_ms < 1200.0 {
                    GREEN.mix(0.9)
                } else if res.avg_success_time_ms < 2200.0 {
                    CYAN.mix(0.8)
                } else if res.avg_success_time_ms < 4000.0 {
                    YELLOW.mix(0.8)
                } else {
                    MAGENTA.mix(0.8)
                };

                chart.draw_series(std::iter::once(Rectangle::new(
                    [(x_idx, y_idx), (x_idx + 1, y_idx + 1)],
                    color.filled(),
                )))?;
            }
        }
    }

    println!("\n[SUCCESS] Heatmap optimization output rendered at 'pid_massive_heatmap.png'.");

    // --- PRINT THE FLAWLESS 100/100 SPEED RUNNER ---
    match perfect_winner {
        Some((kp, ki, kd)) => {
            println!("🥇 UNSTOPPABLE PERFECT WINNER (100/100): Kp: {:.3} | Ki: {:.3} | Kd: {:.3} -> Flawless Avg Time: {:.1} ms",
                     kp, ki, kd, fastest_perfect_time);
        }
        None => {
            println!("⚠️  No configuration achieved a perfect 100/100 baseline within these limits yet.");
        }
    }

    Ok(())
}