use cprint::cprintln;
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

    /// Sends X and Y target angles to the Arduino (Header: 0xFF)
    pub fn send(&mut self, angle_x: u16, angle_y: u16) {
        let high_x = (angle_x >> 8) as u8;
        let low_x = (angle_x & 0xFF) as u8;
        let high_y = (angle_y >> 8) as u8;
        let low_y = (angle_y & 0xFF) as u8;

        // Calculate a simple checksum (sum of data bytes)
        let checksum = high_x
            .wrapping_add(low_x)
            .wrapping_add(high_y)
            .wrapping_add(low_y);

        let packet: [u8; 6] = [
            0xFF, // Unique header for Rust -> Arduino
            high_x, low_x, high_y, low_y, checksum,
        ];

        if let Err(e) = self.send_bytes(&packet) {
            eprintln!("Error during USB transmission: {:?}", e);
        }
    }

    /// Reads the serial port, parses the feedback packet from Arduino (Header: 0xEE)
    /// Returns `Some((angle_x, angle_y))` if a valid packet is processed.
    pub fn read_feedback(&mut self, serial_buffer: &mut Vec<u8>) -> Option<(i16, i16)> {
        let mut read_buf = [0u8; 64];

        // Non-blocking read
        if let Ok(bytes_read) = self.port.read(&mut read_buf) && bytes_read > 0 {
                serial_buffer.extend_from_slice(&read_buf[..bytes_read]);
        }

        // Process the buffer (a valid packet requires at least 6 bytes)
        while serial_buffer.len() >= 6 {
            if serial_buffer[0] == 0xEE {
                let high_x = serial_buffer[1];
                let low_x = serial_buffer[2];
                let high_y = serial_buffer[3];
                let low_y = serial_buffer[4];
                let received_checksum = serial_buffer[5];

                let calculated_checksum = high_x
                    .wrapping_add(low_x)
                    .wrapping_add(high_y)
                    .wrapping_add(low_y);

                if calculated_checksum == received_checksum {
                    let angle_x = i16::from_be_bytes([high_x, low_x]);
                    let angle_y = i16::from_be_bytes([high_y, low_y]);

                    // Consume the 6 bytes of the valid packet
                    serial_buffer.drain(..6);
                    return Some((angle_x, angle_y));
                } else {
                    // Invalid checksum: discard header byte to look for next frame
                    serial_buffer.remove(0);
                }
            } else {
                // Not the header: discard byte
                serial_buffer.remove(0);
            }
        }

        None
    }
}