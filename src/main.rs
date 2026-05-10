mod camera;
mod utils;

use std::env;
use std::fs;
use std::sync::mpsc;
use std::thread;
use std::time::{Duration, Instant};

use opencv::core::{Mat, Vector};
use opencv::imgcodecs::imwrite;

use camera::Camera;
use utils::get_circle_points;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    dotenv::dotenv().ok();

    let mut camera = Camera::init()?;
    let duration = Duration::from_secs(5);
    let start = Instant::now();

    let output_dir = "./_canny_frames/";
    fs::create_dir_all(output_dir)?;

    // Canal pour la sauvegarde (Path, Image)
    let (tx, rx) = mpsc::channel::<(String, Mat)>();

    // THREAD DE SAUVEGARDE (Consommateur)
    let saver_thread = thread::spawn(move || {
        while let Ok((path, frame)) = rx.recv() {
            let _ = imwrite(&path, &frame, &Vector::<i32>::new());
        }
    });

    println!("Capture en cours (5s)... Appuyez sur 'q' pour quitter.");
    let mut count = 0;
    let mut durations = Vec::new();
    while start.elapsed() < duration {
        let start_loop = Instant::now();
        let mut frame_mat = camera.get_frame()?;

        // DETECTION
        if let Some((center, mut radius)) = camera.get_circle(&frame_mat)? {
            match env::var("RADIUS") {
                Ok(r) => {
                    radius = r.parse::<i32>().expect("RADIUS must be a number");
                }
                _ => {
                    println!("RADIUS is not set, using computed value: {}", { radius });
                }
            }
            let circle_points = get_circle_points(center, radius);
            let _ = utils::draw_circle(&mut frame_mat, &circle_points);

            // On envoie à la sauvegarde uniquement si on a un cercle
            let file_path = format!("{}/{}.jpg", output_dir, count);
            tx.send((file_path, frame_mat.clone()))?;
            count += 1;
        }

        durations.push(start_loop.elapsed());
    }

    // Fermeture propre
    drop(tx);
    let _ = saver_thread.join();
    camera.close()?;

    println!("{} images sauvegardées.", count);
    println!(
        "Latence moyenne par frame: {:?}",
        durations.into_iter().sum::<Duration>() / count
    );
    Ok(())
}
