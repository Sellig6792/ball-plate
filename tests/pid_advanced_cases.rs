use pid::{Axe, Pid};

#[test]
fn test_perfect_mathematical_symmetry() {
    let mut pid_left = Pid::default();
    let mut pid_right = Pid::default();

    // Mirror errors relative to the center target
    let out_right = pid_right.calculate_inclination(Axe::X, pid_right.target.x + 20);
    let out_left = pid_left.calculate_inclination(Axe::X, pid_left.target.x - 20);

    // Deviation from neutral baseline (0.5) must be perfectly identical on both sides
    let deviation_right = out_right - 0.5;
    let deviation_left = 0.5 - out_left;

    assert!((deviation_right - deviation_left).abs() < 0.0001);
}

#[test]
fn test_proportional_linearity() {
    // We use two fresh instances to evaluate pure proportional reaction
    // without previous state (derivative/integral) interference.
    let mut pid_step_1 = Pid::default();
    let mut pid_step_2 = Pid::default();

    let center_incl = pid_step_1.calculate_inclination(Axe::X, pid_step_1.target.x);
    let _ = pid_step_2.calculate_inclination(Axe::X, pid_step_2.target.x);

    // Step 1: Measure displacement deviation for 1 pixel
    let incl_1 = pid_step_1.calculate_inclination(Axe::X, pid_step_1.target.x + 1);
    let deviation_1 = incl_1 - center_incl;

    // Step 2: Measure displacement deviation for 2 pixels
    let incl_2 = pid_step_2.calculate_inclination(Axe::X, pid_step_2.target.x + 2);
    let deviation_2 = incl_2 - center_incl;

    // A true PID linear proportional term means: 2x error == 2x output change
    // We use a small delta to allow for minor floating point rounding issues
    assert!(
        (deviation_2 - (deviation_1 * 2.0)).abs() < 0.001,
        "PID response is not linear. 1px dev: {}, 2px dev: {}",
        deviation_1,
        deviation_2
    );
}

#[test]
fn test_instantaneous_error_sign_flip() {
    let mut pid = Pid::default();

    // Frame 1: Ball is far on one side
    let _ = pid.calculate_inclination(Axe::X, pid.target.x + 300);

    // Frame 2: Teleport ball instantly to the completely opposite side
    let incl_x = pid.calculate_inclination(Axe::X, pid.target.x - 300);

    // If your code goes down on negative error: it hits 0.0. If it goes up: it hits 1.0.
    // We check for absolute saturation at one of the hard limits without using ||
    let distance_to_limits = (incl_x - 0.0) * (incl_x - 1.0);
    assert!(
        (distance_to_limits).abs() < 0.001,
        "Expected output to hit either 0.0 or 1.0, got: {}",
        incl_x
    );
}

#[test]
fn test_frequency_doubling_changes_derivative_impact() {
    let mut pid_slow = Pid::default();
    let mut pid_fast = Pid::default();

    pid_slow.config.dt = 0.1;
    pid_fast.config.dt = 0.05;

    // Step 1: Establish base error zero-point
    let _ = pid_slow.calculate_inclination(Axe::X, pid_slow.target.x);
    let _ = pid_fast.calculate_inclination(Axe::X, pid_fast.target.x);

    // Step 2: Apply a small pixel jump to prevent instant clamping to 1.0 or 0.0
    let out_slow = pid_slow.calculate_inclination(Axe::X, pid_slow.target.x + 5);
    let out_fast = pid_fast.calculate_inclination(Axe::X, pid_fast.target.x + 5);

    // Calculate how far each output moved from the neutral 0.5 center point
    let deviation_slow = (out_slow - 0.5).abs();
    let deviation_fast = (out_fast - 0.5).abs();

    // Because pid_fast has a smaller dt, its de/dt is larger, meaning its absolute deviation must be greater
    assert!(
        deviation_fast > deviation_slow,
        "Fast dt deviation ({}) should be greater than slow dt deviation ({})",
        deviation_fast,
        deviation_slow
    );
}
