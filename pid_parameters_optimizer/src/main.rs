use physics::Physics;
use pid::{Axe, Pid, Point};
pub use plotters::prelude::*;
use rayon::prelude::*;
use std::sync::atomic::{AtomicUsize, Ordering};

#[derive(Clone, Copy)]
pub struct Situation {
    pub start_pos: f32,
    pub start_vel: f32,
}

#[derive(Clone, Copy)]
struct EvaluationResult {
    avg_success_time_ms: f32,
    success_count: usize,
}

fn simulate_single_situation(kp: f32, ki: f32, kd: f32, dt: f32, situation: Situation) -> Option<f32> {
    let mut pid = Pid::default();
    pid.config.kp = kp;
    pid.config.ki = ki;
    pid.config.kd = kd;
    pid.config.dt = dt;
    pid.config.invert_x = false;
    pid.config.invert_y = false;
    pid.target = Point::new(320, 240);

    let start_pixel_x = 320.0 + (situation.start_pos * pid.pixels_per_cm);
    let mut physics = Physics::new(start_pixel_x, 240.0, pid.pixels_per_cm);

    for _ in 0..10 {
        physics.step(0.5, 0.5, pid.config.dt);
    }

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

        if distance_from_center_cm > limit_cm {
            return None;
        }

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

fn evaluate_pid_performance(
    kp: f32,
    ki: f32,
    kd: f32,
    dt: f32,
    situations: &[Situation; 100],
) -> EvaluationResult {
    let mut total_time = 0.0;
    let mut success_count = 0;

    for (idx, &situation) in situations.iter().enumerate() {
        // Early pruning for totally unstable configurations
        if idx == 50 && success_count < 5 {
            return EvaluationResult { avg_success_time_ms: 8000.0, success_count };
        }

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

    let mut situations = [Situation { start_pos: 0.0, start_vel: 0.0 }; 100];
    for i in 0..100 {
        let progression = i as f32 / 99.0;
        let direction = if i % 2 == 0 { 1.0 } else { -1.0 };
        situations[i] = Situation {
            start_pos: direction * (1.0 + progression * 12.0),
            start_vel: -direction * (progression * 50.0),
        };
    }

    let x_resolution = 60;
    let y_resolution = 50;

    let kp_min = 0.0; let kp_max = 5.0;
    let kd_min = 0.0; let kd_max = 10.0;

    let ki_slices = [0.0, 0.5, 1.0];
    let total_x_cells = x_resolution * ki_slices.len();
    let total_tasks = y_resolution * x_resolution * ki_slices.len();

    println!("Running multi-threaded optimization sweep across {} combinations...", total_tasks);

    // Atomic counter to track completed tasks across multiple CPU cores
    let completed_counter = AtomicUsize::new(0);
    // Track the last reported percentage interval to avoid console flooding
    let last_reported_percentage = AtomicUsize::new(0);

    // Pre-flattening the work grid allows Rayon to distribute tasks perfectly evenly
    let mut work_items = Vec::with_capacity(total_tasks);
    for y_idx in 0..y_resolution {
        for x_res_idx in 0..x_resolution {
            for ki_idx in 0..ki_slices.len() {
                work_items.push((y_idx, x_res_idx, ki_idx));
            }
        }
    }

    // --- PARALLEL EXECUTION MATRIX ---
    // .par_iter() automatically uses your computer's available CPU cores
    let processed_results: Vec<(usize, usize, usize, EvaluationResult)> = work_items
        .par_iter()
        .map(|&(y_idx, x_res_idx, ki_idx)| {
            let kp = kp_min + (kp_max - kp_min) * (y_idx as f32 / y_resolution as f32);
            let kd = kd_min + (kd_max - kd_min) * (x_res_idx as f32 / x_resolution as f32);
            let ki = ki_slices[ki_idx];

            let res = evaluate_pid_performance(kp, ki, kd, dt, &situations);

            // Update progress
            let current = completed_counter.fetch_add(1, Ordering::Relaxed) + 1;
            let percent = (current * 100) / total_tasks;

            // Log progress cleanly at every 10% milestone
            if percent % 10 == 0 && percent > last_reported_percentage.load(Ordering::Relaxed) {
                if last_reported_percentage.compare_exchange(percent - 10, percent, Ordering::Relaxed, Ordering::Relaxed).is_ok() {
                    println!("[Progress] {}% complete...", percent);
                }
            }

            (y_idx, x_res_idx, ki_idx, res)
        })
        .collect();

    // Reconstruct the 2D grid structure from the parallel results
    let mut results_grid = vec![vec![None; total_x_cells]; y_resolution];
    let mut best_avg_time = 8000.0;
    let mut fastest_perfect_time = 8000.0;
    let mut perfect_winner: Option<(f32, f32, f32)> = None;
    let minimum_acceptable_successes = 50;

    for (y_idx, x_res_idx, ki_idx, res) in processed_results {
        let kp = kp_min + (kp_max - kp_min) * (y_idx as f32 / y_resolution as f32);
        let kd = kd_min + (kd_max - kd_min) * (x_res_idx as f32 / x_resolution as f32);
        let ki = ki_slices[ki_idx];
        let x_idx = x_res_idx * ki_slices.len() + ki_idx;

        if res.success_count >= minimum_acceptable_successes && res.avg_success_time_ms < best_avg_time {
            best_avg_time = res.avg_success_time_ms;
        }

        if res.success_count == 100 && res.avg_success_time_ms < fastest_perfect_time {
            fastest_perfect_time = res.avg_success_time_ms;
            perfect_winner = Some((kp, ki, kd));
        }

        results_grid[y_idx][x_idx] = Some(res);
    }

    // --- RENDER CONTINUOUS HEATMAP ---
    let root = BitMapBackend::new("pid_continuous_heatmap.png", (1800, 1000)).into_drawing_area();
    root.fill(&WHITE)?;

    let mut chart = ChartBuilder::on(&root)
        .caption("Continuous PID Sweeper Performance Grid (Parallelized)", ("sans-serif", 24).into_font())
        .margin(30)
        .x_label_area_size(70)
        .y_label_area_size(70)
        .build_cartesian_2d(0..total_x_cells, 0..y_resolution)?;

    chart.configure_mesh()
        .x_desc("Derivative Gain (Kd Range 0-10) / Sliced Integral Gain (Ki)")
        .y_desc("Proportional Gain (Kp Range 0-5)")
        .x_label_formatter(&|&x| {
            let x_res_idx = x / ki_slices.len();
            let ki_idx = x % ki_slices.len();
            if x_res_idx < x_resolution && x_res_idx % 10 == 0 && ki_idx == 0 {
                let current_kd = kd_min + (kd_max - kd_min) * (x_res_idx as f32 / x_resolution as f32);
                format!("{:.1} / Ki:{:.1}", current_kd, ki_slices[ki_idx])
            } else {
                "".to_string()
            }
        })
        .y_label_formatter(&|&y| {
            if y < y_resolution && y % 5 == 0 {
                let current_kp = kp_min + (kp_max - kp_min) * (y as f32 / y_resolution as f32);
                format!("{:.2}", current_kp)
            } else {
                "".to_string()
            }
        })
        .label_style(("sans-serif", 10))
        .disable_x_mesh()
        .disable_y_mesh()
        .draw()?;

    for y_idx in 0..y_resolution {
        let kp = kp_min + (kp_max - kp_min) * (y_idx as f32 / y_resolution as f32);
        for x_res_idx in 0..x_resolution {
            let kd = kd_min + (kd_max - kd_min) * (x_res_idx as f32 / x_resolution as f32);
            for ki_idx in 0..ki_slices.len() {
                let ki = ki_slices[ki_idx];
                let x_idx = x_res_idx * ki_slices.len() + ki_idx;

                if let Some(res) = &results_grid[y_idx][x_idx] {
                    let color = if res.success_count == 0 {
                        RED.mix(0.85)
                    } else if (res.avg_success_time_ms - best_avg_time).abs() < 0.01 && best_avg_time < 8000.0 && res.success_count >= minimum_acceptable_successes {
                        println!("=> OPTIMAL NODE FOUND: Kp: {:.3} | Ki: {:.3} | Kd: {:.3} -> Success Avg: {:.1} ms ({}/100)",
                                 kp, ki, kd, res.avg_success_time_ms, res.success_count);
                        BLUE.mix(1.0)
                    } else if res.avg_success_time_ms < 1200.0 {
                        GREEN.mix(0.9)
                    } else if res.avg_success_time_ms < 2200.0 {
                        CYAN.mix(0.75)
                    } else if res.avg_success_time_ms < 4000.0 {
                        YELLOW.mix(0.75)
                    } else {
                        MAGENTA.mix(0.7)
                    };

                    chart.draw_series(std::iter::once(Rectangle::new(
                        [(x_idx, y_idx), (x_idx + 1, y_idx + 1)],
                        color.filled(),
                    )))?;
                }
            }
        }
    }

    println!("\n[SUCCESS] Heatmap optimization output rendered at 'pid_continuous_heatmap.png'.");

    if let Some((kp, ki, kd)) = perfect_winner {
        println!("🥇 UNSTOPPABLE PERFECT WINNER (100/100): Kp: {:.3} | Ki: {:.3} | Kd: {:.3} -> Flawless Avg Time: {:.1} ms",
                 kp, ki, kd, fastest_perfect_time);
    }

    Ok(())
}