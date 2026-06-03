use pid::{Axe, Pid};

mod common;

#[test]
fn test_ball_already_at_center_stays_flat() {
    let mut pid = Pid::default();

    let inclination_x = pid.calculate_inclination(Axe::X, pid.target.x);
    let inclination_y = pid.calculate_inclination(Axe::Y, pid.target.y);

    assert!((inclination_x - 0.5).abs() < 0.001);
    assert!((inclination_y - 0.5).abs() < 0.001);
}

#[test]
fn test_extreme_overflow_hits_hardware_clamps() {
    let mut pid = Pid::default();

    // Positive error forces a strict specific boundary condition
    let absurd_pixel_x = pid.target.x + 5000;
    let inclination_x = pid.calculate_inclination(Axe::X, absurd_pixel_x);

    assert_eq!(inclination_x, 0.0);
}

#[test]
fn test_corner_top_right_saturates_both_axes() {
    let mut pid = Pid::default();

    let corner_x = pid.target.x + 2000;
    let corner_y = pid.target.y + 2000;

    let incl_x = pid.calculate_inclination(Axe::X, corner_x);
    let incl_y = pid.calculate_inclination(Axe::Y, corner_y);

    assert_eq!(incl_x, 0.0);
    assert_eq!(incl_y, 0.0);
}

#[test]
fn test_corner_top_left_saturates_both_axes() {
    let mut pid = Pid::default();

    let corner_x = pid.target.x - 2000;
    let corner_y = pid.target.y + 2000;

    let incl_x = pid.calculate_inclination(Axe::X, corner_x);
    let incl_y = pid.calculate_inclination(Axe::Y, corner_y);

    assert_eq!(incl_x, 1.0);
    assert_eq!(incl_y, 0.0);
}

#[test]
fn test_corner_bottom_right_saturates_both_axes() {
    let mut pid = Pid::default();

    let corner_x = pid.target.x + 2000;
    let corner_y = pid.target.y - 2000;

    let incl_x = pid.calculate_inclination(Axe::X, corner_x);
    let incl_y = pid.calculate_inclination(Axe::Y, corner_y);

    assert_eq!(incl_x, 0.0);
    assert_eq!(incl_y, 1.0);
}

#[test]
fn test_corner_bottom_left_saturates_both_axes() {
    let mut pid = Pid::default();

    let corner_x = pid.target.x - 2000;
    let corner_y = pid.target.y - 2000;

    let incl_x = pid.calculate_inclination(Axe::X, corner_x);
    let incl_y = pid.calculate_inclination(Axe::Y, corner_y);

    assert_eq!(incl_x, 1.0);
    assert_eq!(incl_y, 1.0);
}

#[test]
fn test_high_speed_moving_away_forces_max_saturation() {
    let mut pid = Pid::default();

    let _ = pid.calculate_inclination(Axe::X, pid.target.x + 50);
    let incl_x = pid.calculate_inclination(Axe::X, pid.target.x + 150);

    assert_eq!(incl_x, 0.0);
}

#[test]
fn test_high_speed_moving_away_forces_min_saturation() {
    let mut pid = Pid::default();

    let _ = pid.calculate_inclination(Axe::X, pid.target.x - 50);
    let incl_x = pid.calculate_inclination(Axe::X, pid.target.x - 150);

    assert_eq!(incl_x, 1.0);
}

#[test]
fn test_high_speed_diagonal_crossing_saturates_both_axes() {
    let mut pid = Pid::default();

    let _ = pid.calculate_inclination(Axe::X, pid.target.x - 100);
    let _ = pid.calculate_inclination(Axe::Y, pid.target.y - 100);

    let incl_x = pid.calculate_inclination(Axe::X, pid.target.x + 150);
    let incl_y = pid.calculate_inclination(Axe::Y, pid.target.y + 150);

    assert_eq!(incl_x, 0.0);
    assert_eq!(incl_y, 0.0);
}

#[test]
fn test_sudden_velocity_reversal() {
    let mut pid = Pid::default();

    let _ = pid.calculate_inclination(Axe::X, pid.target.x + 10);
    let _ = pid.calculate_inclination(Axe::X, pid.target.x + 20);

    let incl_x = pid.calculate_inclination(Axe::X, pid.target.x - 100);

    assert_eq!(incl_x, 1.0);
}
