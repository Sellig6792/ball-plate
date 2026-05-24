mod app;
mod camera;
mod pid;
mod usb; // 1. Déclaration du module usb.rs
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

    let usb_port = env::var("USB_PORT").unwrap_or_else(|_| "/dev/ttyS3".to_string());
    let baud_rate: u32 = env::var("USB_BAUD_RATE")
        .unwrap_or_else(|_| "115200".to_string())
        .parse()
        .unwrap_or(115200);

    let mut arduino = match UsbController::new(&usb_port, baud_rate) {
        Ok(controller) => {
            println!("Connecté à l'Arduino sur {}", usb_port);
            // Pause essentielle pour laisser l'Arduino finir son auto-reset
            std::thread::sleep(std::time::Duration::from_secs(2));
            Some(controller)
        }
        Err(e) => {
            eprintln!("Attention : Impossible de se connecter à l'Arduino sur {} ({})", usb_port, e);
            None
        }
    };

    let center_x: f32 = env::var("TARGET_CENTER_X").unwrap_or_else(|_| "320.0".to_string()).parse().unwrap();
    let center_y: f32 = env::var("TARGET_CENTER_Y").unwrap_or_else(|_| "240.0".to_string()).parse().unwrap();
    let plate_width_px: f32 = env::var("PLATE_WIDTH_PIXELS").unwrap_or_else(|_| "450.0".to_string()).parse().unwrap();

    let mut pid = BallAndPlatePid::from_env(center_x, center_y, plate_width_px);

    println!("Capture et asservissement démarrés...");
    let mut count = 0;
    let mut last_center: Option<utils::Point> = None;

    // Buffer pour accumuler les caractères reçus de l'Arduino
    let mut serial_buffer = Vec::new();

    loop {
        let start_loop = Instant::now();
        let mut frame_mat = camera.get_frame()?;

        if frame_mat.empty() {
            continue;
        }

        if let Some((center, mut radius)) = camera.get_circle(&frame_mat)? {
            if let Ok(r) = env::var("RADIUS") {
                radius = r.parse::<i32>().expect("RADIUS must be a number");
            }

            let _ = utils::draw::draw_circle(&mut frame_mat, &center, radius, utils::draw::CircleType::Circle, Scalar::new(0.0, 255.0, 0.0, 0.0));

            let command_x = 1. - pid.calculer_inclinaison(Axe::X, center.x as f32);
            let command_y = 1. - pid.calculer_inclinaison(Axe::Y, center.y as f32);

            println!("PID: X: {:2} Y: {:2} ", command_x, command_y);

            // --- CONVERSION DE LA SORTIE NORMALISÉE (0.0 à 1.0) EN DEGRÉS (0 à 180) ---
            // On sature d'abord la commande entre 0.0 et 1.0 par sécurité
            let command_x_clamped = command_x.clamp(0.0, 1.0);

            // Produit en croix : valeur * 180.0
            let angle_x_absolu = command_x_clamped * 180.0;

            // --- ENVOI À L'ARDUINO ---
            if let Some(ref mut port) = arduino {
                // Conversion finale en octet entier (u8)
                let angle_x_byte = angle_x_absolu as u8;

                let packet: [u8; 3] = [0xFF, 0x01, angle_x_byte];

                if let Err(e) = port.send_bytes(&packet) {
                    eprintln!("Erreur lors de l'envoi USB : {:?}", e);
                }

                // --- NOUVEAU : LECTURE DES MESSAGES DE L'ARDUINO ---
                // On regarde si l'Arduino a écrit quelque chose dans le buffer série
                let mut read_buf = [0u8; 64];
                // On utilise le port sous-jacent pour lire de manière non-bloquante (grâce au timeout bas du UsbController)
                if let Ok(bytes_read) = port.port.read(&mut read_buf) {
                    if bytes_read > 0 {
                        // On ajoute les octets lus à notre buffer global
                        serial_buffer.extend_from_slice(&read_buf[..bytes_read]);

                        // Si on trouve un retour à la ligne '\n', on affiche le message complet
                        if let Some(pos) = serial_buffer.iter().position(|&x| x == b'\n') {
                            let line_bytes = serial_buffer.drain(..=pos).collect::<Vec<u8>>();
                            if let Ok(message) = String::from_utf8(line_bytes) {
                                // Affiche le message de l'Arduino proprement dans la console Rust
                                print!("[Arduino] {}", message);
                            }
                        }
                    }
                }            }

            if let Some(last_center_pt) = last_center {
                let diff = utils::Point::new(center.x - last_center_pt.x, center.y - last_center_pt.y);
                let delta_time = start_loop.elapsed().as_secs_f64();
                let in_a_second = utils::Point::new(center.x + (diff.x as f64 / delta_time) as i32, center.y + (diff.y as f64 / delta_time) as i32);
                let _ = utils::draw::draw_vector(&mut frame_mat, center.clone(), in_a_second);
            }
            last_center = Some(center);
            count += 1;
        }

        if tx.blocking_send(frame_mat.clone()).is_err() {
            println!("Le récepteur graphique a été fermé. Arrêt de la capture.");
            break;
        }
    }

    camera.close().expect("Erreur lors de la fermeture de la caméra");
    Ok(())
}