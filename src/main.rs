mod app;
mod camera;
mod pid;
mod usb; // 1. Déclaration du module usb.rs
mod utils;

use crate::app::UserEvent::ChangeImage;
use crate::app::{App, UserEvent};
use crate::utils::Point;
use crate::utils::draw::upscale_mat;
use camera::Camera;
use cprint::{ceprintln, cprintln};
use opencv::core::MatTraitConst;
use opencv::core::{Mat, Scalar};
use pid::{Axe, Pid};
use std::env;
use std::time::Instant;
use tokio::sync::mpsc;
use usb::UsbController; // 2. Import du contrôleur USB
use winit::event_loop::EventLoop;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();

    // 1. INITIALISATION DE WINIT (Thread principal)
    let event_loop: EventLoop<UserEvent> = EventLoop::with_user_event().build()?;
    let proxy = event_loop.create_proxy();

    // 2. DÉMARRAGE DE TOKIO DANS UN THREAD DÉDIÉ
    std::thread::spawn({
        let proxy = proxy.clone();

        move || {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap();

            rt.block_on(async move {
                let (tx, mut rx) = mpsc::channel::<Mat>(100);

                // Tâche Async d'envoi d'image vers l'UI
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

                // Tâche de traitement caméra et PID
                tokio::task::spawn_blocking(move || {
                    if let Err(e) = run_camera_capture(tx) {
                        ceprintln!(
                            "Error",
                            format!("while camera capture : {:?}", e)
                        );
                    }
                })
                .await
                .unwrap();
            });
        }
    });

    // 3. LANCEMENT DE L'APPLICATION GRAPHIQUE
    let mut app = App {
        window_graphics: None,
        image_pixels: Vec::new(),
        img_width: 640 * 2,
        img_height: 480 * 2,
    };

    event_loop.run_app(&mut app)?;
    Ok(())
}

fn run_camera_capture(tx: mpsc::Sender<Mat>) -> Result<(), Box<dyn std::error::Error>> {
    let mut camera = Camera::init()?;

    let usb_port = env::var("USB_PORT").expect("USB_PORT must be set");
    let baud_rate: u32 = env::var("USB_BAUD_RATE")
        .expect("USB_BAUD_RATE must be set")
        .parse()?;

    #[cfg(not(feature = "arduino-less"))]
    let mut arduino = UsbController::new(&usb_port, baud_rate)?;

    let mut pid = Pid::from_env();

    cprintln!("Log", "Capture et asservissement démarrés..." => Cyan);
    let mut last_center: Option<Point> = None;

    // Buffer pour accumuler les caractères reçus de l'Arduino
    #[cfg(not(feature = "arduino-less"))]
    let mut serial_buffer: Vec<u8> = Vec::new();

    let mut frame_mat = camera.get_frame()?;

    loop {
        let start_loop = Instant::now();

        #[cfg(not(feature = "arduino-less"))]
        arduino.println(&mut serial_buffer);

        if let Err(e) = tx.try_send(frame_mat.clone()) {
            match e {
                mpsc::error::TrySendError::Full(_) => {
                    // UI is too slow, skipping this frame to prevent std::bad_alloc
                }
                mpsc::error::TrySendError::Closed(_) => {
                    cprintln!("Log", "The graphical receiver was closed. Stopping." => Cyan);
                    break;
                }
            }
        }

        frame_mat = camera.get_frame()?;

        if frame_mat.empty() {
            continue;
        }

        let _ = utils::draw::draw_circle(
            &mut frame_mat,
            &Point::new(pid.center_x_pixel as i32, pid.center_y_pixel as i32),
            2,
            utils::draw::CircleType::Point,
            Scalar::new(197.0, 73.0, 137.0, 0.0),
        );

        let ball = camera.get_circle(&frame_mat)?;

        match ball {
            Some(_) => {}
            None => continue,
        }
        let (center, mut radius) = ball.unwrap();
        cprintln!("Ball", format!("Center: ({:.2}, {:.2})", center.x, center.y) => Yellow);

        if let Ok(defined_radius) = env::var("RADIUS") {
            radius = defined_radius
                .parse::<i32>()
                .expect("RADIUS must be a number");
        }

        let _ = utils::draw::draw_circle(
            &mut frame_mat,
            &center,
            radius,
            utils::draw::CircleType::Circle,
            Scalar::new(0.0, 255.0, 0.0, 0.0),
        );

        let command_x = pid.calculer_inclinaison(Axe::X, center.x as f32);
        let command_y = pid.calculer_inclinaison(Axe::Y, center.y as f32);

        cprintln!("PID", format!("X: {:.2} ; Y: {:.2} ", command_x, command_y) => Magenta);

        let angle_x = Pid::angle_from_height(command_x)?;
        let angle_y = Pid::angle_from_height(command_y)?;

        #[cfg(not(feature = "arduino-less"))]
        arduino.send(angle_x, angle_y);

        if let Some(last_center_pt) = last_center {
            let dt = start_loop.elapsed().as_secs_f32();
            let in_a_second = utils::computing::in_a_second(last_center_pt, center.clone(), dt);
            let _ = utils::draw::draw_vector(&mut frame_mat, center.clone(), in_a_second);
        }
        last_center = Some(center);
    }

    camera
        .close()
        .expect("Erreur lors de la fermeture de la caméra");
    Ok(())
}
