use serialport::SerialPort;
use std::io::Write;
use std::time::Duration;

pub struct UsbController {
    pub port: Box<dyn SerialPort>,
}

impl UsbController {
    /// Initialise la connexion USB avec l'Arduino.
    /// * `port_name` : Le chemin du port (ex: "/dev/ttyUSB0" sous WSL/Linux ou "COM3" sous Windows)
    /// * `baud_rate` : La vitesse, généralement 9600 ou 115200 (doit correspondre au `Serial.begin()` de l'Arduino)
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, serialport::Error> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(10))
            .open();

        match port {
            Ok(p) => {
                println!("Connecté à l'Arduino sur {}", port_name);
                // Pause essentielle pour laisser l'Arduino finir son auto-reset
                std::thread::sleep(std::time::Duration::from_secs(2));
                Ok(Self { port: p })
            }

            Err(e) => {
                panic!(
                    "Attention : Impossible de se connecter à l'Arduino sur {} ({})",
                    port_name, e
                );
            }
        }
    }

    /// Envoie des octets bruts (pratique pour envoyer des coordonnées de tracking)
    pub fn send_bytes(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    pub fn send(&mut self, command_x: f32, command_y: f32) {
        let command_x = (command_x * 255.0) as u8;
        let command_y = (command_y * 255.0) as u8;

        let packet: [u8; 3] = [0xFF, command_x, command_y];

        if let Err(e) = self.send_bytes(&packet) {
            eprintln!("Erreur lors de l'envoi USB : {:?}", e);
        }
    }

    pub fn println(&mut self, serial_buffer: &mut Vec<u8>) {
        let mut read_buf = [0u8; 64];
        // On utilise le port sous-jacent pour lire de manière non-bloquante (grâce au timeout bas du UsbController)
        if let Ok(bytes_read) = self.port.read(&mut read_buf) {
            if bytes_read == 0 {
                return;
            }
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
    }
}

// Adapt values from 0.2 to 0.8 to values from 0 to 1
pub fn adjust(value: f32) -> f32 {
    (value - 0.2) / 0.6
}
