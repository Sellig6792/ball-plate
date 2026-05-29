use cprint::{cprint, cprintln};
use serialport::SerialPort;
use std::io::Write;
use std::time::Duration;

pub struct UsbController {
    pub port: Box<dyn SerialPort>,
}

impl UsbController {
    /// Initializes the USB connection with the Arduino.
    /// * `port_name`: The port path (e.g., "/dev/ttyUSB0" on WSL/Linux or "COM3" on Windows)
    /// * `baud_rate`: The speed, typically 9600 or 115200 (must match the Arduino's `Serial.begin()`)
    pub fn new(port_name: &str, baud_rate: u32) -> Result<Self, serialport::Error> {
        let port = serialport::new(port_name, baud_rate)
            .timeout(Duration::from_millis(10))
            .open();

        match port {
            Ok(p) => {
                cprintln!("Log", format!("Connected to Arduino on {}", port_name) => Cyan);
                // Essential delay to let the Arduino complete its auto-reset
                std::thread::sleep(std::time::Duration::from_secs(2));
                Ok(Self { port: p })
            }

            Err(e) => {
                panic!(
                    "Warning: Unable to connect to Arduino on {} ({})",
                    port_name, e
                );
            }
        }
    }

    /// Sends raw bytes (useful for sending tracking coordinates)
    pub fn send_bytes(&mut self, bytes: &[u8]) -> std::io::Result<()> {
        self.port.write_all(bytes)?;
        self.port.flush()?;
        Ok(())
    }

    pub fn send(&mut self, angle_x: u16, angle_y: u16) {
        // Angle is from 0 to 180
        let high_x = (angle_x >> 8) as u8;
        let low_x = (angle_x & 0xFF) as u8;
        let high_y = (angle_y >> 8) as u8;
        let low_y = (angle_y & 0xFF) as u8;

        // Calculate a simple checksum (sum of bytes)
        let checksum = high_x
            .wrapping_add(low_x)
            .wrapping_add(high_y)
            .wrapping_add(low_y);

        let packet: [u8; 6] = [
            0xFF, // Unique header
            high_x, low_x, high_y, low_y, checksum, // Replaces 0xFE for data integrity
        ];

        if let Err(e) = self.send_bytes(&packet) {
            eprintln!("Error during USB transmission: {:?}", e);
        }
    }

    pub fn println(&mut self, serial_buffer: &mut Vec<u8>) {
        let mut read_buf = [0u8; 64];
        // Use the underlying port to read in a non-blocking manner (thanks to the low timeout of UsbController)
        if let Ok(bytes_read) = self.port.read(&mut read_buf) {
            if bytes_read == 0 {
                return;
            }
            // Append the read bytes to our global buffer
            serial_buffer.extend_from_slice(&read_buf[..bytes_read]);

            // If a newline '\n' is found, display the complete message
            if let Some(pos) = serial_buffer.iter().position(|&x| x == b'\n') {
                let line_bytes = serial_buffer.drain(..=pos).collect::<Vec<u8>>();
                if let Ok(message) = String::from_utf8(line_bytes) {
                    // Display the message from the Arduino cleanly in the Rust console
                    cprint!("Arduino", message => Blue);
                }
            }
        }
    }
}
