use opencv::core::{Mat, MatTraitConst, Vec3b};
use softbuffer::{Context, Surface};
use std::num::NonZeroU32;
use std::rc::Rc;
use winit::application::ApplicationHandler;
use winit::event::WindowEvent;
use winit::event_loop::ActiveEventLoop;
use winit::window::Window;

// Événement personnalisé pour notifier l'application qu'une nouvelle image doit être affichée
#[derive(Debug)]
pub enum UserEvent {
    ChangeImage(Mat), // Contient le chemin de la nouvelle image
}

type RcWin = Rc<Window>;
pub struct App {
    pub window_graphics: Option<(RcWin, Surface<RcWin, RcWin>)>,
    pub image_pixels: Vec<u32>,
    pub img_width: u32,
    pub img_height: u32,
}

impl App {
    fn load_new_frame(&mut self, frame: &Mat) {
        let size = frame.size().unwrap();
        let width = size.width as u32;
        let height = size.height as u32;

        // 1. Prepare the pixel buffer
        let mut new_pixels = Vec::with_capacity((width * height) as usize);

        // 2. Iterate through the Mat and convert BGR to u32 hex
        // Using at_2d is safe here, but note that OpenCV is usually BGR
        for y in 0..size.height {
            for x in 0..size.width {
                if let Ok(pixel) = frame.at_2d::<Vec3b>(y, x) {
                    let b = pixel[0] as u32;
                    let g = pixel[1] as u32;
                    let r = pixel[2] as u32;
                    // Format: 0x00RRGGBB
                    new_pixels.push((r << 16) | (g << 8) | b);
                }
            }
        }

        // 3. Update state
        self.image_pixels = new_pixels;
        self.img_width = width;
        self.img_height = height;

        // 4. Trigger redraw if the window exists
        if let Some((window, _)) = &self.window_graphics {
            window.request_redraw();
        }
    }
}

impl ApplicationHandler<UserEvent> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window_graphics.is_none() {
            let size = winit::dpi::PhysicalSize::new(self.img_width, self.img_height);
            let attrs = Window::default_attributes()
                .with_inner_size(size)
                .with_title("Wayland Real-time Image Viewer");

            let window = Rc::new(event_loop.create_window(attrs).unwrap());
            let context = Context::new(window.clone()).unwrap();
            let mut surface = Surface::new(&context, window.clone()).unwrap();

            let width = NonZeroU32::new(self.img_width).unwrap();
            let height = NonZeroU32::new(self.img_height).unwrap();
            surface.resize(width, height).unwrap();

            self.window_graphics = Some((window, surface));
        }
    }

    // Capture de notre événement personnalisé en temps réel
    fn user_event(&mut self, _event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::ChangeImage(frame) => {
                // println!("Changement d'image en cours : {}", path);
                self.load_new_frame(&frame);
            }
        }
    }

    fn window_event(
        &mut self,
        event_loop: &ActiveEventLoop,
        _: winit::window::WindowId,
        event: WindowEvent,
    ) {
        match event {
            WindowEvent::Resized(physical_size) => {
                if physical_size.width > 0
                    && physical_size.height > 0
                    && let Some((_, surface)) = &mut self.window_graphics
                {
                    let w = NonZeroU32::new(physical_size.width).unwrap();
                    let h = NonZeroU32::new(physical_size.height).unwrap();
                    surface.resize(w, h).unwrap();
                }
            }
            WindowEvent::RedrawRequested => {
                if let Some((window, surface)) = &mut self.window_graphics {
                    let mut buffer = surface.buffer_mut().unwrap();

                    // 1. Always clear the buffer to avoid artifacts
                    buffer.fill(0);

                    // 2. SAFETY CHECK: Ensure image_pixels is not empty
                    if self.image_pixels.is_empty() {
                        let _ = buffer.present();
                        return; // Exit early if there's nothing to draw
                    }

                    let win_size = window.inner_size();
                    let win_w = win_size.width as usize;
                    let win_h = win_size.height as usize;

                    // 3. Calculate bounds safely
                    let draw_w = win_w.min(self.img_width as usize);
                    let draw_h = win_h.min(self.img_height as usize);
                    let img_w = self.img_width as usize;

                    // 4. Row-by-row copy with boundary protection
                    for y in 0..draw_h {
                        let buffer_start = y * win_w;
                        let img_start = y * img_w;

                        // Ensure we don't slice past the end of the image_pixels vector
                        let img_end = img_start + draw_w;
                        if img_end <= self.image_pixels.len() {
                            let buffer_row = &mut buffer[buffer_start..buffer_start + draw_w];
                            let img_row = &self.image_pixels[img_start..img_end];
                            buffer_row.copy_from_slice(img_row);
                        }
                    }

                    let _ = buffer.present();
                }
            }

            WindowEvent::CloseRequested => event_loop.exit(),
            _ => {}
        }
    }
}
