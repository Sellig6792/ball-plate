use std::time::{Instant, Duration};
use std::thread::sleep;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use opencv::core::{Mat, MatTraitConst};

use hh_ball_plate::camera::Camera;
use hh_ball_plate::usb;
use pid::{Axe, Pid, Point};

struct OptimizationState {
    params: [f32; 3],
    dp: [f32; 3],
    param_idx: usize,
    stage: usize,
    best_score: f32,
}

struct TrialState {
    consecutive_stable_frames: i32,
    start_step: Instant,
    stabilized: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenvy::dotenv().ok();

    let mut camera = Camera::init()?;
    let mut arduino = usb::UsbController::new(
        &std::env::var("USB_PORT").expect("USB_PORT must be set in .env"),
        std::env::var("USB_BAUD_RATE").expect("USB_BAUD_RATE must be set in .env").parse()?,
    )?;

    // Handle Ctrl+C interruptions cleanly
    let running = Arc::new(AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, Ordering::SeqCst);
    })?;

    let mut opt = init_optimization_state()?;
    let mut pid = Pid::from_env();
    let targets = [Point::new(220, 240), Point::new(420, 240)];
    let mut target_idx = 0;

    println!("=========================================================================");
    println!(" RUNNING CONTINUOUS AUTOMATIC OPTIMIZATION ENGINE                       ");
    println!("=========================================================================");
    println!("Place the ball on the plate. The system will take over immediately.");
    println!("Press Ctrl+C to safely exit and save current values.");
    println!("-------------------------------------------------------------------------");

    let mut trial = TrialState {
        consecutive_stable_frames: 0,
        start_step: Instant::now(),
        stabilized: false,
    };

    let target_interval = get_target_interval();
    let mut last_loop_time = Instant::now();
    pid.target = targets[target_idx];

    while running.load(Ordering::SeqCst) {
        let start_loop = Instant::now();
        pid.config.dt = calculate_dt(&mut last_loop_time, target_interval);

        let mut frame_mat = camera.get_frame().unwrap_or_else(|_| Mat::default());
        if frame_mat.empty() {
            continue;
        }

        if let Ok(Some((center, _))) = camera.get_circle(&mut frame_mat) {
            if is_outside_guardrails(center.x, center.y) {
                reset_plate_neutral(&mut arduino);
                continue;
            }

            pid.update_trajectory_target(&center);

            // Check if ball settled at current destination target
            if ((center.x - pid.target.x).abs() as f32) <= 8.0 && ((center.y - pid.target.y).abs() as f32) <= 8.0 {
                trial.consecutive_stable_frames += 1;
                if trial.consecutive_stable_frames >= 8 && !trial.stabilized {
                    trial.stabilized = true;
                    let score = trial.start_step.elapsed().as_secs_f32() * 1000.0;

                    apply_twiddle_step(&mut opt, score, &mut pid);

                    target_idx = (target_idx + 1) % targets.len();
                    pid.target = targets[target_idx];

                    trial.consecutive_stable_frames = 0;
                    trial.stabilized = false;
                    trial.start_step = Instant::now();
                }
            } else {
                trial.consecutive_stable_frames = 0;
            }

            let command_x = pid.calculate_inclination(Axe::X, center.x);
            let command_y = pid.calculate_inclination(Axe::Y, center.y);
            arduino.send(Pid::angle_from_height(command_x).unwrap_or(180), Pid::angle_from_height(command_y).unwrap_or(180));
        }

        // Handle timeout parameters if ball is trapped or drops out of tracking sequence
        if trial.start_step.elapsed() > Duration::from_secs(8) {
            apply_twiddle_step(&mut opt, 99999.0, &mut pid);
            target_idx = (target_idx + 1) % targets.len();
            pid.target = targets[target_idx];
            trial.start_step = Instant::now();
            trial.consecutive_stable_frames = 0;
        }

        enforce_loop_pacing(start_loop, target_interval);
    }

    // Clean exit sequence
    println!("\nStopping calibration engine safely...");
    reset_plate_neutral(&mut arduino);

    println!("=========================================================================");
    println!(" FINAL CALIBRATED COEFFICIENTS                                           ");
    println!("=========================================================================");
    println!("PID_KP={:.4}", opt.params[0]);
    println!("PID_KI={:.4}", opt.params[1]);
    println!("PID_KD={:.4}", opt.params[2]);
    println!("=========================================================================");

    Ok(())
}

fn init_optimization_state() -> Result<OptimizationState, Box<dyn std::error::Error>> {
    Ok(OptimizationState {
        params: [
            std::env::var("PID_KP").unwrap_or_else(|_| "0.6".to_string()).parse::<f32>()?,
            std::env::var("PID_KI").unwrap_or_else(|_| "0.1".to_string()).parse::<f32>()?,
            std::env::var("PID_KD").unwrap_or_else(|_| "0.25".to_string()).parse::<f32>()?,
        ],
        dp: [0.04, 0.008, 0.015],
        param_idx: 0,
        stage: 0,
        best_score: 99999.0,
    })
}

fn apply_twiddle_step(opt: &mut OptimizationState, score: f32, pid: &mut Pid) {
    let idx = opt.param_idx;

    if opt.best_score == 99999.0 {
        opt.best_score = score;
        opt.params[idx] += opt.dp[idx];
        update_pid_gains(pid, opt.params);
        return;
    }

    if opt.stage == 0 {
        if score < opt.best_score {
            opt.best_score = score;
            opt.dp[idx] *= 1.1;
            next_parameter(opt);
            opt.params[opt.param_idx] += opt.dp[opt.param_idx];
        } else {
            opt.params[idx] -= 2.0 * opt.dp[idx];
            opt.stage = 1;
        }
    } else {
        if score < opt.best_score {
            opt.best_score = score;
            opt.dp[idx] *= 1.1;
        } else {
            opt.params[idx] += opt.dp[idx];
            opt.dp[idx] *= 0.9;
        }
        next_parameter(opt);
        opt.params[opt.param_idx] += opt.dp[opt.param_idx];
        opt.stage = 0;
    }

    update_pid_gains(pid, opt.params);

    println!("New Parameters -> Kp: {:.4} | Ki: {:.4} | Kd: {:.4} (Step Sum: {:.4})",
             opt.params[0], opt.params[1], opt.params[2], opt.dp[0] + opt.dp[1] + opt.dp[2]);
}

fn next_parameter(opt: &mut OptimizationState) {
    opt.param_idx = (opt.param_idx + 1) % 3;
}

fn update_pid_gains(pid: &mut Pid, params: [f32; 3]) {
    pid.config.kp = params[0];
    pid.config.ki = params[1];
    pid.config.kd = params[2];
}

fn is_outside_guardrails(x: i32, y: i32) -> bool {
    x < 100 || x > 540 || y < 30 || y > 450
}

fn reset_plate_neutral(arduino: &mut usb::UsbController) {
    let neutral = Pid::angle_from_height(0.5).unwrap_or(180);
    arduino.send(neutral, neutral);
}

fn get_target_interval() -> Duration {
    let fps: f32 = std::env::var("FRAME_RATE").unwrap_or_else(|_| "10".to_string()).parse().unwrap_or(10.0);
    Duration::from_secs_f32(1.0 / fps)
}

fn calculate_dt(last_loop_time: &mut Instant, target_interval: Duration) -> f32 {
    let elapsed = last_loop_time.elapsed().as_secs_f32();
    *last_loop_time = Instant::now();
    if elapsed > 0.0 { elapsed } else { target_interval.as_secs_f32() }
}

fn enforce_loop_pacing(start_loop: Instant, target_interval: Duration) {
    if start_loop.elapsed() < target_interval {
        sleep(target_interval - start_loop.elapsed());
    }
}