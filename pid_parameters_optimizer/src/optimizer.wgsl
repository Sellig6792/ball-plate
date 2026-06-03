struct SimulationParams {
    kp_min: f32,
    kp_max: f32,
    kd_min: f32,
    kd_max: f32,
    ki_min: f32,
    ki_max: f32,
    dt: f32,
    x_resolution: u32,
    y_resolution: u32,
    ki_resolution: u32,
    arm: f32,
    rod: f32,
    plate_physical_size_cm: f32,
    clamp_integral: f32,
    clamp_command: f32,
    invert_x_with_y: u32,
}

struct EvaluationResult {
    avg_success_time_ms: f32,
    success_count: u32,
}

@group(0) @binding(0) var<uniform> params: SimulationParams;
@group(0) @binding(1) var<storage, read_write> output_grid: array<EvaluationResult>;

struct Situation {
    start_pos_x_cm: f32,
    start_pos_y_cm: f32,
}

struct Physics {
    pos_x_cm: f32,
    vel_x: f32,
    current_servo_degrees_x: f32,
    pos_y_cm: f32,
    vel_y: f32,
    current_servo_degrees_y: f32,
    crosstalk_x_into_y: f32,
    crosstalk_y_into_x: f32,
}

struct PidConfig {
    kp: f32,
    ki: f32,
    kd: f32,
    dt: f32,
    invert_x: bool,
    invert_y: bool,
}

struct PidState {
    error_previous: f32,
    integral_sum: f32,
}

fn to_degrees(rad: f32) -> f32 {
    return rad * 57.29577951308232;
}

fn to_radians(deg: f32) -> f32 {
    return deg * 0.017453292519943295;
}

fn get_pixel_pos_x(p: Physics, pixels_per_cm: f32) -> i32 {
    return i32(floor(p.pos_x_cm * pixels_per_cm + 0.5));
}

fn get_pixel_pos_y(p: Physics, pixels_per_cm: f32) -> i32 {
    return i32(floor(p.pos_y_cm * pixels_per_cm + 0.5));
}

fn physics_step(p: ptr<function, Physics>, cmd_h_x: f32, cmd_h_y: f32, dt: f32) {
    let max_degrees_this_frame = 333.0 * dt;
    let g = 981.0;
    let max_plate_tilt_rad = 12.0 * (3.141592653589793 / 180.0);
    let stiction_threshold = 1.2;
    let rolling_resistance = 0.91;

    // 1. RESOLVE INDEPENDENT SERVO POSITIONING
    let height_x = (params.rod - params.arm) + (2.0 * params.arm * cmd_h_x);
    let argument_x = (pow(height_x, 2.0) + pow(params.arm, 2.0) - pow(params.rod, 2.0)) / (2.0 * params.arm * height_x);
    let target_theta_x = 270.0 - to_degrees(acos(clamp(argument_x, -1.0, 1.0)));
    let diff_x = target_theta_x - (*p).current_servo_degrees_x;
    (*p).current_servo_degrees_x += clamp(diff_x, -max_degrees_this_frame, max_degrees_this_frame);

    let height_y = (params.rod - params.arm) + (2.0 * params.arm * cmd_h_y);
    let argument_y = (pow(height_y, 2.0) + pow(params.arm, 2.0) - pow(params.rod, 2.0)) / (2.0 * params.arm * height_y);
    let target_theta_y = 270.0 - to_degrees(acos(clamp(argument_y, -1.0, 1.0)));
    let diff_y = target_theta_y - (*p).current_servo_degrees_y;
    (*p).current_servo_degrees_y += clamp(diff_y, -max_degrees_this_frame, max_degrees_this_frame);

    // 2. COMPUTE PURE PHYSICAL PLATE TILTS
    let tilt_dev_rad_x = to_radians((*p).current_servo_degrees_x - 180.0);
    let pure_plate_tilt_rad_x = sin(tilt_dev_rad_x) * max_plate_tilt_rad;

    let tilt_dev_rad_y = to_radians((*p).current_servo_degrees_y - 180.0);
    let pure_plate_tilt_rad_y = sin(tilt_dev_rad_y) * max_plate_tilt_rad;

    // 3. APPLY STRUCTURAL CROSS-COUPLING
    let effective_tilt_x = pure_plate_tilt_rad_x + (pure_plate_tilt_rad_y * (*p).crosstalk_y_into_x);
    let effective_tilt_y = pure_plate_tilt_rad_y + (pure_plate_tilt_rad_x * (*p).crosstalk_x_into_y);

    // 4. CALCULATE DYNAMICS AND STEP POSITIONS
    var accel_x = (2.0 / 3.0) * g * sin(effective_tilt_x);
    if (abs((*p).vel_x) < 0.05 && abs(accel_x) < stiction_threshold) {
        accel_x = 0.0;
    }
    (*p).vel_x += accel_x * dt;
    (*p).vel_x *= rolling_resistance;
    (*p).pos_x_cm += (*p).vel_x * dt;

    var accel_y = (2.0 / 3.0) * g * sin(effective_tilt_y);
    if (abs((*p).vel_y) < 0.05 && abs(accel_y) < stiction_threshold) {
        accel_y = 0.0;
    }
    (*p).vel_y += accel_y * dt;
    (*p).vel_y *= rolling_resistance;
    (*p).pos_y_cm += (*p).vel_y * dt;
}

fn calculate_inclination(
    axe_param: u32,
    ball_position_pixel: i32,
    config: PidConfig,
    state: ptr<function, PidState>,
    pixels_per_cm: f32,
    target_x: i32,
    target_y: i32
) -> f32 {
    var axe = axe_param;
    if (params.invert_x_with_y == 1u) {
        if (axe == 0u) { axe = 1u; }
        else { axe = 0u; }
    }

    var center_pixel = target_x;
    var invert = config.invert_x;
    if (axe == 1u) {
        center_pixel = target_y;
        invert = config.invert_y;
    }

    var pixel_offset = center_pixel - ball_position_pixel;
    if (invert) {
        pixel_offset = -pixel_offset;
    }

    let error_cm = clamp(f32(pixel_offset) / pixels_per_cm, -params.plate_physical_size_cm / 2.0, params.plate_physical_size_cm / 2.0);
    let p = config.kp * error_cm;

    if (config.ki > 0.0) {
        (*state).integral_sum += error_cm * config.dt;
        if (params.clamp_integral > 0.0) {
            (*state).integral_sum = clamp((*state).integral_sum, -params.clamp_integral, params.clamp_integral);
        }
    }
    let i_term = config.ki * (*state).integral_sum;

    var d = 0.0;
    if (config.dt > 0.0) {
        d = config.kd * ((error_cm - (*state).error_previous) / config.dt);
    }

    (*state).error_previous = error_cm;

    let pid_output = (p + i_term + d) / (params.plate_physical_size_cm / 2.0);
    let plate_inclination = 0.5 + pid_output;

    return clamp(plate_inclination, 0.0 + params.clamp_command, 1.0 - params.clamp_command);
}

fn simulate_single_situation(kp: f32, ki: f32, kd: f32, dt: f32, situation: Situation) -> f32 {
    let plate_size_pixel = 640.0;
    let pixels_per_cm = plate_size_pixel / params.plate_physical_size_cm;

    let target_x = 320;
    let target_y = 240;

    let center_x_cm = f32(target_x) / pixels_per_cm;
    let center_y_cm = f32(target_y) / pixels_per_cm;

    var physics: Physics;
    physics.pos_x_cm = center_x_cm + situation.start_pos_x_cm;
    physics.pos_y_cm = center_y_cm + situation.start_pos_y_cm;
    physics.vel_x = 0.0;
    physics.vel_y = 0.0;
    physics.current_servo_degrees_x = 180.0;
    physics.current_servo_degrees_y = 180.0;
    physics.crosstalk_x_into_y = 0.06;
    physics.crosstalk_y_into_x = 0.03;

    var config: PidConfig;
    config.kp = kp; config.ki = ki; config.kd = kd; config.dt = dt;
    config.invert_x = false; config.invert_y = false;

    var state_x: PidState; state_x.error_previous = 0.0; state_x.integral_sum = 0.0;
    var state_y: PidState; state_y.error_previous = 0.0; state_y.integral_sum = 0.0;

    let max_frames = 350;
    let limit_cm = params.plate_physical_size_cm / 2.0;
    let precision_threshold_cm = 0.12;
    let velocity_threshold_cms = 0.40;
    let required_stable_frames = 18;
    var consecutive_stable_frames = 0;

    for (var frame = 1; frame <= max_frames; frame++) {
        if ((max_frames - frame) < (required_stable_frames - consecutive_stable_frames)) {
            return -1.0;
        }

        let ball_x = get_pixel_pos_x(physics, pixels_per_cm);
        let ball_y = get_pixel_pos_y(physics, pixels_per_cm);

        let cmd_x = calculate_inclination(0u, ball_x, config, &state_x, pixels_per_cm, target_x, target_y);
        let cmd_y = calculate_inclination(1u, ball_y, config, &state_y, pixels_per_cm, target_x, target_y);

        physics_step(&physics, cmd_x, cmd_y, dt);

        let distance_x_cm = abs(physics.pos_x_cm - center_x_cm);
        let distance_y_cm = abs(physics.pos_y_cm - center_y_cm);

        if (distance_x_cm > limit_cm || distance_y_cm > limit_cm) {
            return -1.0;
        }

        let stable_x = distance_x_cm < precision_threshold_cm && abs(physics.vel_x) < velocity_threshold_cms;
        let stable_y = distance_y_cm < precision_threshold_cm && abs(physics.vel_y) < velocity_threshold_cms;

        if (stable_x && stable_y) {
            consecutive_stable_frames++;
            if (consecutive_stable_frames >= required_stable_frames) {
                return f32(frame - required_stable_frames) * dt * 1000.0;
            }
        } else {
            consecutive_stable_frames = 0;
        }
    }
    return -1.0;
}

@compute @workgroup_size(16, 16, 1)
fn main(@builtin(global_invocation_id) global_id: vec3<u32>) {
    let total_x_cells = params.x_resolution * params.ki_resolution;
    if (global_id.x >= total_x_cells || global_id.y >= params.y_resolution) { return; }

    let y_idx = global_id.y;
    let x_res_idx = global_id.x / params.ki_resolution;
    let ki_idx = global_id.x % params.ki_resolution;

    let kp = params.kp_min + (params.kp_max - params.kp_min) * (f32(y_idx) / f32(params.y_resolution));
    let kd = params.kd_min + (params.kd_max - params.kd_min) * (f32(x_res_idx) / f32(params.x_resolution));
    let ki = params.ki_min + (params.ki_max - params.ki_min) * (f32(ki_idx) / f32(params.ki_resolution));

    var total_time = 0.0;
    var success_count = 0u;

    for (var i = 0u; i < 100u; i++) {
        // Optimisation précoce : Si à la moitié de la boucle le taux est catastrophique, on coupe
        if (i == 50u && success_count < 5u) {
            break;
        }

        let progression = f32(i) / 99.0;
        var direction = 1.0;
        if (i % 2u == 1u) { direction = -1.0; }

        var sit: Situation;
        sit.start_pos_x_cm = direction * (0.5 + progression * 17.5);
        sit.start_pos_y_cm = -direction * (0.5 + progression * 17.5);

        let outcome = simulate_single_situation(kp, ki, kd, params.dt, sit);
        if (outcome >= 0.0) {
            total_time += outcome;
            success_count++;
        }
    }

    let flat_idx = y_idx * total_x_cells + global_id.x;
    output_grid[flat_idx].success_count = success_count;

    // CORRECTION CRUCIALE : Calcul de la moyenne réelle non saturée à 8000
    if (success_count > 0u) {
        output_grid[flat_idx].avg_success_time_ms = total_time / f32(success_count);
    } else {
        output_grid[flat_idx].avg_success_time_ms = 999999.0;
    }
}