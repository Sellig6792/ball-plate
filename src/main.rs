mod app;
mod camera;
mod pid;

#[cfg(not(feature = "arduino-less"))]
mod usb;
mod utils;

use crate::app::UserEvent::ChangeImage;
use crate::app::{App, UserEvent};
use crate::utils::Point;
use camera::Camera;
use cprint::{ceprintln, cprintln};
use opencv::core::MatTraitConst;
use opencv::core::{Mat, Scalar};
use pid::{Axe, Pid};
use std::env;
use std::time::Instant;
use tokio::sync::mpsc;
use winit::event_loop::EventLoop;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();

    // 1. READ ENVIRONMENT CONFIGURATIONS FOR WINDOW RESOLUTION
    // If not set in .env, defaults to standard 2x scaling configurations (1280x960)
    let window_res_vec = env::var("WINDOW_RESOLUTION")
        .expect("WINDOW_RESOLUTION must be set in .env")
        .split("x")
        .map(|x| {
            x.parse::<u32>()
                .expect("WINDOW_RESOLUTION is not a valid [u32]x[u32]")
        })
        .collect::<Vec<u32>>();

    // 2. WINIT INITIALIZATION (Main thread)
    let event_loop: EventLoop<UserEvent> = EventLoop::with_user_event().build()?;
    let proxy = event_loop.create_proxy();

    // 3. STARTING TOKIO IN A DEDICATED THREAD
    std::thread::spawn({
        let proxy = proxy.clone();

        move || {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap();

            rt.block_on(async move {
                let (tx, mut rx) = mpsc::channel::<Mat>(100);

                let update_app = proxy.clone();
                tokio::spawn(async move {
                    while let Some(frame) = rx.recv().await {
                        let proxy_task = update_app.clone();

                        let _ = tokio::task::spawn_blocking(move || {
                            let _ = proxy_task.send_event(ChangeImage(frame));
                        })
                        .await;
                    }
                });

                tokio::task::spawn_blocking(move || {
                    if let Err(e) = run_camera_capture(tx) {
                        ceprintln!("Error", format!("while camera capture: {:?}", e));
                    }
                })
                .await
                .unwrap();
            });
        }
    });

    // 4. LAUNCHING THE GRAPHICAL APPLICATION (Using parsed variables)
    let mut app = App {
        window_graphics: None,
        pixels: Vec::new(),
        width: window_res_vec[0],
        height: window_res_vec[1],
    };

    event_loop.run_app(&mut app)?;
    Ok(())
}

fn run_camera_capture(tx: mpsc::Sender<Mat>) -> Result<(), Box<dyn std::error::Error>> {
    let mut camera = Camera::init()?;

    #[cfg(not(feature = "arduino-less"))]
    let mut arduino = usb::UsbController::new(
        &std::env::var("USB_PORT").expect("USB_PORT must be set"),
        std::env::var("USB_BAUD_RATE")
            .expect("USB_BAUD_RATE must be set")
            .parse()?,
    )?;

    let mut pid = Pid::from_env();
    let mut last_center: Option<Point> = None;

    // --- ENVIRONMENT CONFIGURATIONS FOR TELEMETRY PLOT ---
    #[cfg(not(feature = "no-graph"))]
    let mut telemetry_plot = {
        let width: i32 = std::env::var("GRAPH_WIDTH").unwrap_or_else(|_| "250".to_string()).parse()?;
        let height: i32 = std::env::var("GRAPH_HEIGHT").unwrap_or_else(|_| "125".to_string()).parse()?;
        utils::graph::TelemetryGraph::new(120, width, height)
    };
    // -----------------------------------------------------

    #[cfg(all(not(feature = "no-graph"), feature = "arduino-less"))]
    let mut current_feedback = (180i16, 180i16);
    #[cfg(all(not(feature = "arduino-less"), not(feature = "no-graph")))]
    let mut current_feedback = (180i16, 180i16);

    #[cfg(not(feature = "no-graph"))]
    let mut current_target = (180u16, 180u16);

    #[cfg(all(not(feature = "arduino-less"), not(feature = "no-graph")))]
    let mut serial_buffer: Vec<u8> = Vec::new();

    let mut frame_mat = camera.get_frame()?;
    let mut last_loop_time = Instant::now();
    cprintln!("Log", "Capture and control loops started..." => Cyan);

    loop {
        let start_loop = Instant::now();
        let dt = last_loop_time.elapsed().as_secs_f32();
        last_loop_time = start_loop;
        pid.config.dt = if dt > 0.0 { dt } else { 0.01 };

        if let Err(mpsc::error::TrySendError::Closed(_)) = tx.try_send(frame_mat.clone()) {
            cprintln!("Log", "The graphical receiver was closed. Stopping." => Cyan);
            break;
        }

        frame_mat = camera.get_frame()?;
        if frame_mat.empty() {
            continue;
        }

        #[cfg(all(not(feature = "arduino-less"), not(feature = "no-graph")))]
        if let Some((fb_x, fb_y)) = arduino.read_feedback(&mut serial_buffer) {
            current_feedback = (fb_x, fb_y);
        }

        // Forward raw items to our frame processor
        process_frame(
            &mut frame_mat,
            &mut camera,
            &mut pid,
            &mut last_center,
            dt,
            #[cfg(not(feature = "no-graph"))] &mut telemetry_plot,
            #[cfg(not(feature = "no-graph"))] &mut current_target,
            #[cfg(not(feature = "no-graph"))] current_feedback,
            #[cfg(not(feature = "arduino-less"))] &mut arduino,
        )?;
    }

    camera.close().expect("Error while closing the camera");
    Ok(())
}

/// Extracted helper to isolate the processing loop mechanics
fn process_frame(
    frame_mat: &mut Mat,
    camera: &mut Camera,
    pid: &mut Pid,
    last_center: &mut Option<Point>,
    dt: f32,
    #[cfg(not(feature = "no-graph"))] telemetry_plot: &mut utils::graph::TelemetryGraph,
    #[cfg(not(feature = "no-graph"))] current_target: &mut (u16, u16),
    #[cfg(not(feature = "no-graph"))] current_feedback: (i16, i16),
    #[cfg(not(feature = "arduino-less"))] arduino: &mut usb::UsbController,
) -> Result<(), Box<dyn std::error::Error>> {
    utils::draw::draw_plate_guidelines(frame_mat, pid);

    let ball = camera.get_circle(frame_mat)?;
    if ball.is_none() {
        #[cfg(not(feature = "no-graph"))]
        telemetry_plot.log_and_draw(frame_mat, *current_target, current_feedback);
        return Ok(());
    }

    let (center, radius) = ball.unwrap();
    cprintln!("Ball", format!("X: {:4.} , Y: {:4.}", center.x, center.y) => Yellow);

    let _ = utils::draw::draw_circle(frame_mat, &center, radius, utils::draw::CircleType::Circle, Scalar::new(0.0, 255.0, 0.0, 0.0));

    let command_x = pid.calculate_inclination(Axe::X, center.x as f32);
    let command_y = pid.calculate_inclination(Axe::Y, center.y as f32);
    cprintln!("PID", format!("X: {:.2} ; Y: {:.2} ==> dt: {:.2}", command_x, command_y, dt) => Magenta);

    let angle_x = Pid::angle_from_height(command_x)?;
    let angle_y = Pid::angle_from_height(command_y)?;

    #[cfg(not(feature = "no-graph"))]
    {
        *current_target = (angle_x, angle_y);
        telemetry_plot.log_and_draw(frame_mat, *current_target, current_feedback);
    }

    #[cfg(not(feature = "arduino-less"))]
    arduino.send(angle_x, angle_y);

    if let Some(last_center_pt) = *last_center {
        let in_a_second = utils::computing::in_a_second(last_center_pt, center.clone(), dt);
        let _ = utils::draw::draw_vector(frame_mat, center.clone(), in_a_second);
    }
    *last_center = Some(center);

    Ok(())
}