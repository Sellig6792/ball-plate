use crate::utils::draw::upscale_mat;
use crate::utils::Point;
use opencv::core::{Mat, MatTraitConst, MatTraitConstManual};
use softbuffer::{Context, Surface};
use std::num::NonZeroU32;
use std::rc::Rc;
use winit::application::ApplicationHandler;
use winit::event::{WindowEvent, MouseButton, ElementState};
use winit::event_loop::ActiveEventLoop;
use winit::window::Window;

// Enum to specify what action the background processing thread should execute
#[derive(Debug, Clone)]
pub enum TargetMessage {
    AppendWaypoint(Point),  // Left click variant: schedules an additional point
    InstantTarget(Point),   // Right click variant: instantly overrides the pathing
}

#[derive(Debug)]
pub enum UserEvent {
    ChangeImage(Mat),
}

type RcWin = Rc<Window>;
pub struct App {
    pub window_graphics: Option<(RcWin, Surface<RcWin, RcWin>)>,
    pub pixels: Vec<u32>,
    pub width: u32,
    pub height: u32,
    pub click_tx: crossbeam_channel::Sender<TargetMessage>,
    pub current_cursor: Point,
    pub is_mouse_down: bool,
    pub is_right_mouse_down: bool, // Added to track right mouse button state
}

impl App {
    fn load_new_frame(&mut self, frame: &mut Mat) {
        upscale_mat(frame).expect("Error while upscaling the frame");

        let size = frame.size().unwrap();
        let width = size.width as u32;
        let height = size.height as u32;

        if let Ok(data) = frame.data_bytes() {
            let total_pixels = (width * height) as usize;
            let mut new_pixels = Vec::with_capacity(total_pixels);

            for chunk in data.chunks_exact(3) {
                let b = chunk[0] as u32;
                let g = chunk[1] as u32;
                let r = chunk[2] as u32;

                new_pixels.push((r << 16) | (g << 8) | b);
            }

            self.pixels = new_pixels;
            self.width = width;
            self.height = height;

            if let Some((window, _)) = &self.window_graphics {
                window.request_redraw();
            }
        }
    }
}

impl ApplicationHandler<UserEvent> for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.window_graphics.is_none() {
            let size = winit::dpi::LogicalSize::new(self.width as f64, self.height as f64);

            let attrs = Window::default_attributes()
                .with_inner_size(size)
                .with_resizable(false)
                .with_title("Ball Tracking Visualisation");

            let window = Rc::new(event_loop.create_window(attrs).unwrap());
            let context = Context::new(window.clone()).unwrap();
            let mut surface = Surface::new(&context, window.clone()).unwrap();

            let width = NonZeroU32::new(self.width).unwrap();
            let height = NonZeroU32::new(self.height).unwrap();
            surface.resize(width, height).unwrap();

            self.window_graphics = Some((window, surface));
        }
    }

    fn user_event(&mut self, _event_loop: &ActiveEventLoop, event: UserEvent) {
        match event {
            UserEvent::ChangeImage(mut frame) => {
                self.load_new_frame(&mut frame);
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
            WindowEvent::CursorMoved { position, .. } => {
                let upscale_factor: f32 = std::env::var("UPSCALE_FACTOR")
                    .expect("UPSCALE_FACTOR must be set in .env")
                    .parse()
                    .expect("UPSCALE_FACTOR must be a f32");

                self.current_cursor = Point::new(
                    (position.x as f32 / upscale_factor) as i32,
                    (position.y as f32 / upscale_factor) as i32,
                );

                if self.is_mouse_down {
                    let _ = self.click_tx.send(TargetMessage::AppendWaypoint(self.current_cursor.clone()));
                } else if self.is_right_mouse_down {
                    // While dragging with right-click, continuously override the target location
                    let _ = self.click_tx.send(TargetMessage::InstantTarget(self.current_cursor.clone()));
                }
            }
            WindowEvent::MouseInput { state, button, .. } => {
                match button {
                    MouseButton::Left => {
                        if state == ElementState::Pressed {
                            self.is_mouse_down = true;
                            let _ = self.click_tx.send(TargetMessage::AppendWaypoint(self.current_cursor.clone()));
                        } else {
                            self.is_mouse_down = false;
                        }
                    }
                    MouseButton::Right => {
                        if state == ElementState::Pressed {
                            self.is_right_mouse_down = true;
                            let _ = self.click_tx.send(TargetMessage::InstantTarget(self.current_cursor.clone()));
                        } else {
                            self.is_right_mouse_down = false;
                        }
                    }
                    _ => {}
                }
            }
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
                    buffer.fill(0);

                    if self.pixels.is_empty() {
                        let _ = buffer.present();
                        return;
                    }

                    let win_size = window.inner_size();
                    let win_w = win_size.width as usize;
                    let win_h = win_size.height as usize;

                    let draw_w = win_w.min(self.width as usize);
                    let draw_h = win_h.min(self.height as usize);
                    let img_w = self.width as usize;

                    for y in 0..draw_h {
                        let buffer_start = y * win_w;
                        let img_start = y * img_w;
                        let img_end = img_start + draw_w;

                        if img_end <= self.pixels.len() {
                            let buffer_row = &mut buffer[buffer_start..buffer_start + draw_w];
                            let img_row = &self.pixels[img_start..img_end];
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