mod camera;
mod utils;


use std::fs;
use std::time::{Duration, Instant};
use std::sync::mpsc;
use std::thread;

use opencv::core::{Mat, Vector};
use opencv::imgcodecs::imwrite;

use camera::Camera;
use utils::get_circle_points;


const RADIUS: Option<i32> = Some(160);


fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    while start.elapsed() < duration {
        let start_loop = Instant::now();
        let mut frame_mat = camera.get_frame()?;

        // DETECTION
        if let Some((center, radius)) = camera.get_circle(&frame_mat)? {
            let circle_points = get_circle_points(center, RADIUS.or(Some(radius)).unwrap());
            let _ = utils::draw_circle(&mut frame_mat, &circle_points);

            // On envoie à la sauvegarde uniquement si on a un cercle
            let file_path = format!("{}/{}.jpg", output_dir, count);
            tx.send((file_path, frame_mat.clone()))?;
            count += 1;
        }

        let duration_loop = start_loop.elapsed();
        println!("Frame {} - Latence boucle: {:?}", count, duration_loop);
    }

    // Fermeture propre
    drop(tx);
    let _ = saver_thread.join();
    camera.close()?;

    println!("Fin. {} images sauvegardées.", count);
    Ok(())
}
