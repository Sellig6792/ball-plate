use criterion::{Criterion, criterion_group, criterion_main};
use pid::{Axe, Pid};
use std::hint::black_box; // Adjust if your package name differs

/// Benchmark 1: Standard single-axis execution
fn bench_pid_single_axis(c: &mut Criterion) {
    let mut pid = Pid::default();
    let test_pixel_pos = pid.target.x + 150;

    c.bench_function("pid_single_axis_evaluation", |b| {
        b.iter(|| pid.calculate_inclination(black_box(Axe::X), black_box(test_pixel_pos)))
    });
}

/// Benchmark 2: Simulates real runtime execution by alternating between X and Y axes
fn bench_pid_axis_alternation(c: &mut Criterion) {
    let mut pid = Pid::default();
    let test_pos_x = pid.target.x + 80;
    let test_pos_y = pid.target.y - 120;

    c.bench_function("pid_axis_alternation_loop", |b| {
        b.iter(|| {
            let x_out = pid.calculate_inclination(black_box(Axe::X), black_box(test_pos_x));
            let y_out = pid.calculate_inclination(black_box(Axe::Y), black_box(test_pos_y));
            black_box((x_out, y_out));
        })
    });
}

/// Benchmark 3: Measures performance under heavy state changes (high derivative spikes)
fn bench_pid_dynamic_state_changes(c: &mut Criterion) {
    let mut pid = Pid::default();
    let mut toggle = false;

    c.bench_function("pid_dynamic_state_fluctuation", |b| {
        b.iter(|| {
            // Alternate inputs every iteration to prevent the derivative term from flattening out
            let input = if toggle {
                pid.target.x + 200
            } else {
                pid.target.x - 200
            };
            toggle = !toggle;

            pid.calculate_inclination(black_box(Axe::X), black_box(input))
        })
    });
}

/// Benchmark 4: Mass processing performance (e.g., buffering or smoothing 100 frames)
fn bench_pid_batch_processing(c: &mut Criterion) {
    let mut pid = Pid::default();

    // Create a mock buffer of 100 sequential sensor readings moving across the plate
    let mock_frame_stream: Vec<i32> = (0..100).map(|i| pid.target.x + (i * 2)).collect();

    c.bench_function("pid_batch_process_100_frames", |b| {
        b.iter(|| {
            for &pixel_pos in &mock_frame_stream {
                let out = pid.calculate_inclination(black_box(Axe::X), black_box(pixel_pos));
                black_box(out);
            }
        })
    });
}

// Register all benchmarks into the harness group
criterion_group!(
    benches,
    bench_pid_single_axis,
    bench_pid_axis_alternation,
    bench_pid_dynamic_state_changes,
    bench_pid_batch_processing
);
criterion_main!(benches);
