mod app;
mod camera;
mod utils;
mod pid;

use crate::app::UserEvent::ChangeImage;
use crate::app::{App, UserEvent};
use camera::Camera;
use opencv::core::{Mat, Scalar};
use std::env;
use std::time::Instant;
use tokio::sync::mpsc;
use winit::event_loop::EventLoop;
use crate::utils::draw::upscale_mat;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();

    // 1. INITIALISATION DE WINIT (Thread principal)
    let event_loop: EventLoop<UserEvent> = EventLoop::with_user_event().build()?;
    let proxy = event_loop.create_proxy();

    // 2. DÉMARRAGE DE TOKIO DANS UN THREAD DÉDIÉ
    // Cela garantit que le runtime reste en vie et ne gèle pas avec run_app
    std::thread::spawn({
        let proxy = proxy.clone();

        move || {
            let rt = tokio::runtime::Builder::new_multi_thread()
                .enable_all()
                .build()
                .unwrap();

            rt.block_on(async move {
                let (tx, mut rx) = mpsc::channel::<Mat>(100);

                // Tâche Async de sauvegarde
                let update_app = proxy.clone();
                tokio::spawn(async move {
                    while let Some(frame) = rx.recv().await {
                        let proxy_task = update_app.clone();

                        let _ = tokio::task::spawn_blocking(move || {
                            let _ = proxy_task.send_event(ChangeImage(upscale_mat(&frame, 2.).expect("COULDNT UPSCALE")));
                        })
                        .await;
                    }
                });

                // Tâche de capture caméra
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
        img_width: 640 * 2,
        img_height: 480 * 2,
    };

    event_loop.run_app(&mut app)?;
    Ok(())
}

fn run_camera_capture(tx: mpsc::Sender<Mat>) -> Result<(), Box<dyn std::error::Error>> {
    let mut camera = Camera::init()?;

    println!("Capture démarrée en arrière-plan...");
    let mut count = 0;

    let mut last_center: Option<utils::Point> = None;

    loop {
        let start_loop = Instant::now();

        let mut frame_mat = camera.get_frame()?;

        if let Some((center, mut radius)) = camera.get_circle(&frame_mat)? {
            if let Ok(r) = env::var("RADIUS") {
                radius = r.parse::<i32>().expect("RADIUS must be a number");
            }

            let _ = utils::draw::draw_circle(
                &mut frame_mat,
                &center,
                radius,
                utils::draw::CircleType::Circle,
                Scalar::new(0.0, 255.0, 0.0, 0.0),
            );
            if let Some(last_center) = last_center {
                let diff = utils::Point::new(center.x - last_center.x, center.y - last_center.y);

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
        // Si le récepteur est fermé (ex: fermeture de la fenêtre), on arrête proprement
        if tx.blocking_send(frame_mat.clone()).is_err() {
            println!("Le récepteur a été fermé. Arrêt de la capture.");
            break;
        }

        // Petite pause pour éviter de saturer le CPU et le canal MPSC
        // std::thread::sleep(std::time::Duration::from_millis(50));
    }

    camera.close().expect("Error closing camera");
    println!("Fin de la capture. {} images traitées.", count);
    Ok(())
}
