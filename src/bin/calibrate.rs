use std::time::{Instant, Duration};
use std::thread::sleep;
use opencv::core::{Mat, MatTraitConst};

use hh_ball_plate::camera::Camera;
use hh_ball_plate::usb;
use pid::{Axe, Pid};

struct Evaluation {
    stabilized: bool,
    score_ms: f32,
}

struct TrialState {
    consecutive_stable_frames: i32,
    stabilized_at_ms: f32,
    stabilized: bool,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenvy::dotenv().ok();

    let mut camera = Camera::init()?;
    let mut arduino = usb::UsbController::new(
        &std::env::var("USB_PORT").expect("USB_PORT must be set in .env"),
        std::env::var("USB_BAUD_RATE")
            .expect("USB_BAUD_RATE must be set in .env")
            .parse()?,
    )?;

    let mut params = [
        std::env::var("PID_KP").unwrap_or_else(|_| "0.6".to_string()).parse::<f32>()?,
        std::env::var("PID_KI").unwrap_or_else(|_| "0.1".to_string()).parse::<f32>()?,
        std::env::var("PID_KD").unwrap_or_else(|_| "0.25".to_string()).parse::<f32>()?,
    ];

    let mut dp = [0.05, 0.01, 0.02];
    let optimization_tolerance = 0.005;

    println!("=========================================================================");
    println!(" INITIALIZING LIVE HARDWARE TWIDDLE OPTIMIZATION ENGINE                 ");
    println!("=========================================================================");
    println!("Starting Parameters -> Kp: {:.3}, Ki: {:.3}, Kd: {:.3}", params[0], params[1], params[2]);

    let mut best_score = evaluate_system(&mut camera, &mut arduino, params[0], params[1], params[2]).score_ms;

    while (dp[0] + dp[1] + dp[2]) > optimization_tolerance {
        for i in 0..3 {
            params[i] += dp[i];
            let res = evaluate_system(&mut camera, &mut arduino, params[0], params[1], params[2]);

            if res.stabilized && res.score_ms < best_score {
                best_score = res.score_ms;
                dp[i] *= 1.1;
            } else {
                params[i] -= 2.0 * dp[i];
                let res = evaluate_system(&mut camera, &mut arduino, params[0], params[1], params[2]);

                if res.stabilized && res.score_ms < best_score {
                    best_score = res.score_ms;
                    dp[i] *= 1.1;
                } else {
                    params[i] += dp[i];
                    dp[i] *= 0.9;
                }
            }
        }
        println!("-------------------------------------------------------------------------");
        println!("Current Optimum -> Kp: {:.4}, Ki: {:.4}, Kd: {:.4} | Score: {:.1} ms", params[0], params[1], params[2], best_score);
    }

    println!("=========================================================================");
    println!(" RUN COMPLETE                                                            ");
    println!("=========================================================================");
    println!("Update these values inside your production .env file:");
    println!("  -> PID_KP={:.4}", params[0]);
    println!("  -> PID_KI={:.4}", params[1]);
    println!("  -> PID_KD={:.4}", params[2]);
    println!("=========================================================================");

    Ok(())
}

fn evaluate_system(
    camera: &mut Camera,
    arduino: &mut usb::UsbController,
    kp: f32,
    ki: f32,
    kd: f32,
) -> Evaluation {
    let mut pid = initialize_trial_pid(kp, ki, kd);
    reset_plate_neutral(arduino);

    println!("\n[Trial Configuration] Kp: {:.3} | Ki: {:.3} | Kd: {:.3}", kp, ki, kd);
    println!("Place the ball on the plate... Running trial loop in 4 seconds.");
    sleep(Duration::from_secs(4));

    let mut state = TrialState {
        consecutive_stable_frames: 0,
        stabilized_at_ms: 0.0,
        stabilized: false,
    };

    run_evaluation_loop(camera, arduino, &mut pid, &mut state);
    reset_plate_neutral(arduino);

    build_evaluation_verdict(state)
}

fn run_evaluation_loop(
    camera: &mut Camera,
    arduino: &mut usb::UsbController,
    pid: &mut Pid,
    state: &mut TrialState,
) {
    let target_interval = get_target_interval();
    let start_test = Instant::now();
    let mut last_loop_time = Instant::now();

    while start_test.elapsed() < Duration::from_secs(12) {
        let start_loop = Instant::now();
        pid.config.dt = calculate_dt(&mut last_loop_time, target_interval);

        let mut frame_mat = camera.get_frame().unwrap_or_else(|_| Mat::default());
        if !frame_mat.empty() {
            let should_break = handle_camera_frame(pid, arduino, camera, &mut frame_mat, start_test, state);
            if should_break {
                break;
            }
        }

        enforce_loop_pacing(start_loop, target_interval);
    }
}

fn handle_camera_frame(
    pid: &mut Pid,
    arduino: &mut usb::UsbController,
    camera: &mut Camera,
    frame_mat: &mut Mat,
    start_test: Instant,
    state: &mut TrialState,
) -> bool {
    if let Ok(Some((center, _))) = camera.get_circle(frame_mat) {
        if is_outside_guardrails(center.x, center.y) {
            println!("  -> Guardrail tripped: Ball neared physical limit. Run canceled.");
            return true;
        }
        return process_ball_frame(pid, arduino, center, start_test, state);
    }
    false
}

fn get_target_interval() -> Duration {
    let fps: f32 = std::env::var("FRAME_RATE")
        .unwrap_or_else(|_| "10".to_string())
        .parse()
        .unwrap_or(10.0);
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

fn initialize_trial_pid(kp: f32, ki: f32, kd: f32) -> Pid {
    let mut pid = Pid::from_env();
    pid.config.kp = kp;
    pid.config.ki = ki;
    pid.config.kd = kd;
    pid.target_queue.clear();
    pid.target = pid.center;
    pid
}

fn reset_plate_neutral(arduino: &mut usb::UsbController) {
    let neutral_x = Pid::angle_from_height(0.5).unwrap_or(180);
    let neutral_y = Pid::angle_from_height(0.5).unwrap_or(180);
    arduino.send(neutral_x, neutral_y);
}

fn is_outside_guardrails(x: i32, y: i32) -> bool {
    x < 100 || x > 540 || y < 30 || y > 450
}

fn build_evaluation_verdict(state: TrialState) -> Evaluation {
    if state.stabilized {
        println!("  -> Verdict: SUCCESS | Settled in {:.1} ms", state.stabilized_at_ms);
        Evaluation { stabilized: true, score_ms: state.stabilized_at_ms }
    } else {
        println!("  -> Verdict: FAILED | Timeout or boundary breach.");
        Evaluation { stabilized: false, score_ms: 99999.0 }
    }
}

fn process_ball_frame(
    pid: &mut Pid,
    arduino: &mut usb::UsbController,
    center: pid::Point,
    start_test: Instant,
    state: &mut TrialState,
) -> bool {
    pid.update_trajectory_target(&center);

    let ball_x = center.x as f32;
    let ball_y = center.y as f32;
    let target_x = pid.target.x as f32;
    let target_y = pid.target.y as f32;

    if (ball_x - target_x).abs() <= 8.0 && (ball_y - target_y).abs() <= 8.0 {
        state.consecutive_stable_frames += 1;
        if state.consecutive_stable_frames >= 8 && !state.stabilized {
            state.stabilized = true;
            state.stabilized_at_ms = start_test.elapsed().as_secs_f32() * 1000.0;
        }
    } else {
        state.consecutive_stable_frames = 0;
        state.stabilized = false;
    }

    let command_x = pid.calculate_inclination(Axe::X, center.x);
    let command_y = pid.calculate_inclination(Axe::Y, center.y);

    let angle_x = Pid::angle_from_height(command_x).unwrap_or(180);
    let angle_y = Pid::angle_from_height(command_y).unwrap_or(180);

    arduino.send(angle_x, angle_y);
    false
}