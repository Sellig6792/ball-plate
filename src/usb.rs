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
            .open()?;

        Ok(Self { port })
    }

    /// Envoie une chaîne de caractères (texte) à l'Arduino
    pub fn send_string(&mut self, data: &str) -> std::io::Result<()> {
        self.port.write_all(data.as_bytes())?;
        self.port.flush()?; // Force l'envoi immédiat des données
        Ok(())
    }

    /// Envoie des octets bruts (pratique pour envoyer des coordonnées de tracking)
    pub fn send_bytes(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }
}