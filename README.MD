# HH-ball-plate — Controller (Rust)

This submodule contains the main control application written in Rust. It serves as the "brain" of the **HH-ball-plate** system, handling computer vision processing, trajectory calculation, and the PID regulation loop.

## 🛠️ Key Responsibilities
* **Ball Tracking (OpenCV):** Captures the video stream from the camera, processes frames to recognize the ping-pong ball, and computes its exact coordinates in real-time.
* **PID Control Loop:** Processes the position data to compute the required adjustments and angles needed to counteract gravity and stabilize the ball.
* **Serial Communication:** Sends the computed target angles to the Arduino driver via serial interface.
* **GUI & Visualization:** Displays a live window showing the tracked ball, its trajectory, and real-time graphs plotting the servomotor angles.

## 🔧 Prerequisites & Setup

### Dependencies
Ensure you have the following installed on your host system or WSL distribution:
* **Rust toolchain:** Installable via `rustup`.
* **OpenCV:** Required for image processing (`opencv` crate bindings).
* **Build tools:** System tools like `pkg-config`, `cmake`, and a C++ compiler to compile the Rust-OpenCV dependencies.

### Configuration
1. Create a `.env` file in this directory (`controller-rust/`).
2. Define your environment variables (such as serial port configurations, camera indices, or PID coefficients).

### WSL Users (Crucial)
If running inside WSL, USB devices are sandboxed by default. You must use `usbipd` from your Windows host to attach:
1. The **camera** (before launching the application).
2. The **Arduino board** (after uploading the driver code).

## 🚀 How to Run
Once your environment variables are configured and your USB hardware is attached, launch the controller using:
```bash
cargo run --release
```

---
© 2026 Cyril Asselborn, Eléonore Mernier, Victoria Douchamps, Gilles Meunier. All rights reserved.