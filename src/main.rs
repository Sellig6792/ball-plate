mod app;
mod camera;
mod pid;
mod utils;

use crate::app::UserEvent::ChangeImage;
use crate::app::{App, UserEvent};
use crate::utils::draw::upscale_mat;
use camera::Camera;
use opencv::core::MatTraitConst;
use opencv::core::{Mat, Scalar};
use pid::{Axe, BallAndPlatePid};
use std::env;
use std::time::Instant;
use tokio::sync::mpsc;
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
                            let _ = proxy_task.send_event(ChangeImage(
                                upscale_mat(&frame, 2.).expect("COULDNT UPSCALE"),
                            ));
                        })
                        .await;
                    }
                });

                // Tâche de traitement caméra et PID
                tokio::task::spawn_blocking(move || {
                    if let Err(e) = run_camera_capture(tx) {
                        eprintln!("Erreur lors de la capture caméra : {:?}", e);
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
        img_width: 640,
        img_height: 480,
    };

    event_loop.run_app(&mut app)?;
    Ok(())
}

fn run_camera_capture(tx: mpsc::Sender<Mat>) -> Result<(), Box<dyn std::error::Error>> {
    let mut camera = Camera::init()?;

    // Lecture des paramètres du centre définis manuellement dans le .env
    let center_x: f32 = env::var("TARGET_CENTER_X")
        .unwrap_or_else(|_| "320.0".to_string())
        .parse()
        .unwrap();
    let center_y: f32 = env::var("TARGET_CENTER_Y")
        .unwrap_or_else(|_| "240.0".to_string())
        .parse()
        .unwrap();
    let plate_width_px: f32 = env::var("PLATE_WIDTH_PIXELS")
        .unwrap_or_else(|_| "450.0".to_string())
        .parse()
        .unwrap();

    // Initialisation directe du PID avec le centre manuel
    let mut pid = BallAndPlatePid::from_env(center_x, center_y, plate_width_px);

    println!(
        "Capture et asservissement démarrés (Centre ciblé : X={}, Y={})...",
        center_x, center_y
    );
    let mut count = 0;
    let mut last_center: Option<utils::Point> = None;

    loop {
        let start_loop = Instant::now();
        let mut frame_mat = camera.get_frame()?;

        if frame_mat.empty() {
            continue;
        }

        // Recherche de la bille (cercle)
        if let Some((center, mut radius)) = camera.get_circle(&frame_mat)? {
            if let Ok(r) = env::var("RADIUS") {
                radius = r.parse::<i32>().expect("RADIUS must be a number");
            }

            // Dessin du cercle autour de la bille
            let _ = utils::draw::draw_circle(
                &mut frame_mat,
                &center,
                radius,
                utils::draw::CircleType::Circle,
                Scalar::new(0.0, 255.0, 0.0, 0.0),
            );

            // --- CALCULS DU PID ---
            let command_x = pid.calculer_inclinaison(Axe::X, center.x as f32);
            let command_y = pid.calculer_inclinaison(Axe::Y, center.y as f32);
            println!("PID -> X: {:.2}, Y: {:.2}", command_x, command_y);
            // ICI : Tu peux envoyer command_x et command_y via ton port Série à l'Arduino !
            // Exemple: println!("Servos -> X: {:.2}, Y: {:.2}", command_x, command_y);

            if let Some(last_center_pt) = last_center {
                let diff =
                    utils::Point::new(center.x - last_center_pt.x, center.y - last_center_pt.y);
                let delta_time = start_loop.elapsed().as_secs_f64();

                let in_a_second = utils::Point::new(
                    center.x + (diff.x as f64 / delta_time) as i32,
                    center.y + (diff.y as f64 / delta_time) as i32,
                );

                let _ = utils::draw::draw_vector(&mut frame_mat, center.clone(), in_a_second);
            }
            last_center = Some(center);
            count += 1;
        }

        // Si la fenêtre graphique se ferme, on arrête proprement la boucle de capture
        if tx.blocking_send(frame_mat.clone()).is_err() {
            println!("Le récepteur graphique a été fermé. Arrêt de la capture.");
            break;
        }
    }

    camera
        .close()
        .expect("Erreur lors de la fermeture de la caméra");
    println!("Fin de la capture. {} images traitées.", count);
    Ok(())
}
