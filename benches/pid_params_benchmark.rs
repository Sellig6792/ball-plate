// benches/pid_params_benchmark.rs
// Standalone Advanced PID Parametric Scenario Benchmarking Tool

use std::env;
use std::time::Instant;

// Import the official PID implementation from your workspace
use pid::{Axe, Pid};
// Import the decoupled 2D workspace physics library
use physics::Physics;

#[derive(Clone, Copy, Debug)]
pub struct PhysicalScenario {
    pub id: usize,
    pub start_pixel_x: f32,
    pub start_pixel_y: f32,
    pub description: &'static str,
}

#[derive(Debug)]
pub struct ScenarioResult {
    pub scenario_id: usize,
    pub success: bool,
    pub stabilization_time_ms: f32,
    pub final_pos_x_cm: f32,
    pub final_pos_y_cm: f32,
    pub failure_reason: &'static str,
}

/// Simulates a 2D multi-axis physical scenario using the centralized workspace engine and your Pid struct.
fn execute_scenario(kp: f32, ki: f32, kd: f32, scenario: PhysicalScenario, pixels_per_cm: f32) -> ScenarioResult {
    // Instantiate the 2D physics tracker using the shared workspace library
    let mut physics = Physics::new(scenario.start_pixel_x, scenario.start_pixel_y, pixels_per_cm);

    // Initialize your official Pid struct
    let mut pid = Pid::default();
    pid.config.kp = kp;
    pid.config.ki = ki;
    pid.config.kd = kd;

    // Explicitly align with your known physical axis configuration
    pid.config.invert_x = false;
    pid.config.invert_y = false;

    let dt = pid.config.dt;
    let plate_half_bound_cm = pid.plate_size_in_cm / 2.0; // Dynamically grab configuration limits

    let precision_threshold_cm = 0.12;
    let velocity_threshold_cms = 0.40;
    let mut consecutive_stable_frames = 0;
    let required_stable_frames = 18;

    let max_frames = 350;

    for frame in 1..=max_frames {
        // Retrieve positions transformed to camera pixels for the PID controller
        let current_pixel_x = physics.get_pixel_pos_x();
        let current_pixel_y = physics.get_pixel_pos_y();

        // Calculate normalized servo commands [0.0 - 1.0] from PID tracking loop
        let incl_x = pid.calculate_inclination(Axe::X, current_pixel_x);
        let incl_y = pid.calculate_inclination(Axe::Y, current_pixel_y);

        // Advance physical world state by directly supplying raw kinematics commands to handle crosstalk
        physics.step(incl_x, incl_y, dt);

        // --- STRICT BOUNDARY GUARDRAIL ---
        // Calculate the physical distance from the center of the plate
        let center_x_cm = (pid.target.x as f32) / pixels_per_cm;
        let center_y_cm = (pid.target.y as f32) / pixels_per_cm;

        let distance_x_cm = (physics.pos_x_cm - center_x_cm).abs();
        let distance_y_cm = (physics.pos_y_cm - center_y_cm).abs();

        // Failure Check: Exceeded physical plate constraints
        if distance_x_cm > plate_half_bound_cm || distance_y_cm > plate_half_bound_cm {
            return ScenarioResult {
                scenario_id: scenario.id,
                success: false,
                stabilization_time_ms: 0.0,
                final_pos_x_cm: physics.pos_x_cm,
                final_pos_y_cm: physics.pos_y_cm,
                failure_reason: "BALL_DROPPED_OFF_PLATE",
            };
        }

        // Stability Check: Evaluated relative to the absolute setpoint delta
        let stable_x = distance_x_cm < precision_threshold_cm && physics.vel_x.abs() < velocity_threshold_cms;
        let stable_y = distance_y_cm < precision_threshold_cm && physics.vel_y.abs() < velocity_threshold_cms;

        if stable_x && stable_y {
            consecutive_stable_frames += 1;
            if consecutive_stable_frames >= required_stable_frames {
                let convergence_frame = frame - required_stable_frames;
                return ScenarioResult {
                    scenario_id: scenario.id,
                    success: true,
                    stabilization_time_ms: (convergence_frame as f32) * dt * 1000.0,
                    final_pos_x_cm: physics.pos_x_cm,
                    final_pos_y_cm: physics.pos_y_cm,
                    failure_reason: "NONE",
                };
            }
        } else {
            consecutive_stable_frames = 0;
        }
    }

    ScenarioResult {
        scenario_id: scenario.id,
        success: false,
        stabilization_time_ms: 0.0,
        final_pos_x_cm: physics.pos_x_cm,
        final_pos_y_cm: physics.pos_y_cm,
        failure_reason: "TIMEOUT_STABILIZATION_FAILED",
    }
}

fn main() {
    let args: Vec<String> = env::args().collect();

    // Re-tuned to use your expanded sweet spot bounds discovered during optimization sweeps
    let default_kp = 9.500;
    let default_ki = 0.001;
    let default_kd = 0.510;

    if args.len() < 4 {
        println!("[NOTICE] Missing arguments. Usage: cargo bench -- <Kp> <Ki> <Kd>");
        println!("Executing structural imperfection fallback benchmark suite...\n");
        execute_benchmark(default_kp, default_ki, default_kd, 100);
        return;
    }

    let kp: f32 = args[1].parse().unwrap_or(default_kp);
    let ki: f32 = args[2].parse().unwrap_or(default_ki);
    let kd: f32 = args[3].parse().unwrap_or(default_kd);

    execute_benchmark(kp, ki, kd, 100);
}

fn execute_benchmark(kp: f32, ki: f32, kd: f32, count: usize) {
    let mut default_pid = Pid::default();
    let dt = default_pid.config.dt;
    let pixels_per_cm = default_pid.pixels_per_cm;

    println!("=========================================================================");
    println!(" INITIALIZING MASSIVE MULTI-SCENARIO 2D PID BENCHMARK ENGINE              ");
    println!("=========================================================================");
    println!(" Target Evaluation Metrics (X & Y Axes Integration via pid::Pid):");
    println!("   -> Proportional Gain (Kp):   {:.4}", kp);
    println!("   -> Integral Gain (Ki):       {:.4}", ki);
    println!("   -> Derivative Gain (Kd):     {:.4}", kd);
    println!("   -> Processing Step (dt):     {:.4} seconds", dt);
    println!("   -> Total 2D Stress Scenarios: {}", count);
    println!("-------------------------------------------------------------------------");

    let mut scenarios = Vec::with_capacity(count);
    for i in 0..count {
        let progression = i as f32 / (count - 1) as f32;
        let direction = if i % 2 == 0 { 1.0 } else { -1.0 };

        // Generate start locations inside the physical square perimeter (40cm square -> max 20cm)
        let start_x_cm = direction * (0.5 + progression * 17.5);
        let start_y_cm = -direction * (0.5 + progression * 17.5);

        let description = if progression < 0.25 {
            "Soft Center-Proximity Perturbation"
        } else if progression < 0.60 {
            "Moderate Step-Response Displacement"
        } else if progression < 0.85 {
            "High-Velocity Kinetic Impact Incursion"
        } else {
            "Extreme Outer Perimeter Boundary Recovery"
        };

        scenarios.push(PhysicalScenario {
            id: i + 1,
            start_pixel_x: (default_pid.target.x as f32) + (start_x_cm * pixels_per_cm),
            start_pixel_y: (default_pid.target.y as f32) + (start_y_cm * pixels_per_cm),
            description,
        });
    }

    let benchmark_start = Instant::now();

    let mut success_count = 0;
    let mut total_stabilization_time = 0.0;
    let mut max_stabilization_time: f32 = 0.0;
    let mut min_stabilization_time: f32 = 99999.0;
    let mut drop_fails = 0;
    let mut timeout_fails = 0;

    println!("{:<6} | {:<42} | {:<10} | {:<12}", "ID", "Scenario Profile Type Description", "Status", "Settled Time");
    println!("-------------------------------------------------------------------------");

    for scenario in &scenarios {
        let result = execute_scenario(kp, ki, kd, *scenario, pixels_per_cm);

        let status_str = if result.success {
            success_count += 1;
            total_stabilization_time += result.stabilization_time_ms;
            if result.stabilization_time_ms > max_stabilization_time { max_stabilization_time = result.stabilization_time_ms; }
            if result.stabilization_time_ms < min_stabilization_time { min_stabilization_time = result.stabilization_time_ms; }
            "SUCCESS"
        } else {
            if result.failure_reason == "BALL_DROPPED_OFF_PLATE" {
                drop_fails += 1;
                "CRASH"
            } else {
                timeout_fails += 1;
                "TIMEOUT"
            }
        };

        let time_str = if result.success {
            format!("{:.1} ms", result.stabilization_time_ms)
        } else {
            result.failure_reason.to_string()
        };

        if scenario.id <= 15 || scenario.id == count || !result.success {
            println!(
                "{:<6} | {:<42} | {:<10} | {:<12}",
                scenario.id,
                if scenario.id <= 15 || scenario.id == count { scenario.description } else { "[Mid-Range Scenario Run]" },
                status_str,
                time_str
            );
        } else if scenario.id == 16 {
            println!("... [Processing remaining multi-tier physical profiles sequentially] ...");
        }
    }

    let benchmark_duration = benchmark_start.elapsed();
    let success_rate = (success_count as f32 / count as f32) * 100.0;

    println!("-------------------------------------------------------------------------");
    println!(" PERFORMANCE OVERVIEW & COMPILATION RESULTS SUMMARY                      ");
    println!("-------------------------------------------------------------------------");
    println!(" CPU Processing Execution Clock Speed : {:?}", benchmark_duration);
    println!(" Global Robustness Success Rate       : {:.2}% ({}/{} Scenarios Stabilized)", success_rate, success_count, count);
    println!(" Boundary Crashes (Dropped Balls)     : {}", drop_fails);
    println!(" Continuous Over-Oscillation Timeouts : {}", timeout_fails);

    if success_count > 0 {
        println!(" Average Successful Settling Time     : {:.2} ms", total_stabilization_time / success_count as f32);
        println!(" Peak Optimal Response Time           : {:.2} ms", min_stabilization_time);
        println!(" Worst-Case Recovery Settle Time      : {:.2} ms", max_stabilization_time);
    } else {
        println!(" Average Successful Settling Time     : N/A (Zero scenarios survived tuning window)");
    }
    println!("=========================================================================");
}