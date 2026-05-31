use opencv::core::{Mat, MatTraitConst, Point as CvPoint, Rect, Scalar, add_weighted};
use opencv::imgproc::{FONT_HERSHEY_SIMPLEX, LINE_8, line, put_text};
use std::collections::VecDeque;
use std::env;

pub struct TelemetryGraph {
    target_x_history: VecDeque<i16>,
    feedback_x_history: VecDeque<i16>,
    target_y_history: VecDeque<i16>,
    feedback_y_history: VecDeque<i16>,
    max_points: usize,
    width: i32,
    height: i32,
}

impl TelemetryGraph {
    pub fn new(max_points: usize, width: i32, height: i32) -> Self {
        Self {
            target_x_history: VecDeque::with_capacity(max_points),
            feedback_x_history: VecDeque::with_capacity(max_points),
            target_y_history: VecDeque::with_capacity(max_points),
            feedback_y_history: VecDeque::with_capacity(max_points),
            max_points,
            width,
            height,
        }
    }

    pub fn push(&mut self, target_x: i16, feedback_x: i16, target_y: i16, feedback_y: i16) {
        if self.target_x_history.len() >= self.max_points {
            self.target_x_history.pop_front();
            self.feedback_x_history.pop_front();
            self.target_y_history.pop_front();
            self.feedback_y_history.pop_front();
        }
        self.target_x_history.push_back(target_x);
        self.feedback_x_history.push_back(feedback_x);
        self.target_y_history.push_back(target_y);
        self.feedback_y_history.push_back(feedback_y);
    }

    pub fn draw(&self, frame: &mut Mat) -> Result<(), opencv::Error> {
        let window_res_vec = env::var("WINDOW_RESOLUTION")
            .expect("WINDOW_RESOLUTION must be set in .env")
            .split('x')
            .map(|x| {
                x.parse::<u32>()
                    .expect("WINDOW_RESOLUTION is not a valid [u32]x[u32]")
            })
            .collect::<Vec<u32>>();

        let win_width = window_res_vec[0] as f64;

        let base_reference_width = 1280.0;
        let resolution_scale = win_width / base_reference_width;

        // Fetch user variable from .env matching the new naming structure

        let zoomed_width = (self.width as f64 * resolution_scale).round() as i32;
        let zoomed_height = (self.height as f64 * resolution_scale).round() as i32;
        let margin = (20.0 * resolution_scale).round() as i32;
        let top_y = (20.0 * resolution_scale).round() as i32;

        let frame_width = frame.cols();

        let x_pos_graph_x = margin;
        let x_pos_graph_y = frame_width - margin - zoomed_width;

        let rect_x = Rect::new(x_pos_graph_x, top_y, zoomed_width, zoomed_height);
        let rect_y = Rect::new(x_pos_graph_y, top_y, zoomed_width, zoomed_height);

        self.draw_axis(frame, rect_x, resolution_scale, true)?;
        self.draw_axis(frame, rect_y, resolution_scale, false)?;

        Ok(())
    }

    pub fn draw_axis(
        &self,
        frame: &mut Mat,
        rect: Rect,
        scale_factor: f64,
        is_axis_x: bool,
    ) -> Result<(), opencv::Error> {
        let (targets, feedbacks, label) = if is_axis_x {
            (&self.target_x_history, &self.feedback_x_history, "Servo X")
        } else {
            (&self.target_y_history, &self.feedback_y_history, "Servo Y")
        };

        if targets.is_empty() {
            return Ok(());
        }

        let mut sub_mat = Mat::roi_mut(frame, rect)?;

        let overlay = Mat::new_size_with_default(
            rect.size(),
            sub_mat.typ(),
            Scalar::new(25.0, 25.0, 25.0, 0.0),
        )?;

        let mut blended = Mat::default();
        let alpha = 0.60;
        let beta = 1.0 - alpha;
        add_weighted(&sub_mat, alpha, &overlay, beta, 0.0, &mut blended, -1)?;
        blended.copy_to(&mut sub_mat)?;

        let min_val = 90.0f32;
        let max_val = 270.0f32;
        let val_range = max_val - min_val;

        let map_point = |index: usize, value: i16| -> CvPoint {
            let pct_x = index as f32 / (self.max_points - 1) as f32;
            let pct_y = (value as f32 - min_val) / val_range;
            let pt_x = rect.x + (pct_x * rect.width as f32) as i32;
            let pt_y = rect.y + rect.height - (pct_y * rect.height as f32) as i32;
            CvPoint::new(pt_x, pt_y)
        };

        let line_thickness = if scale_factor > 1.5 {
            3
        } else if scale_factor < 0.7 {
            1
        } else {
            2
        };
        let font_scale = 0.35 * scale_factor;
        let title_font_scale = 0.5 * scale_factor;

        let graduation_levels = [90, 135, 180, 225, 270];
        for &level in &graduation_levels {
            let left_pt = map_point(0, level);
            let right_pt = map_point(self.max_points - 1, level);

            line(
                frame,
                left_pt,
                right_pt,
                Scalar::new(60.0, 60.0, 60.0, 0.0),
                1,
                LINE_8,
                0,
            )?;

            let text_val = format!("{}*", level);
            let text_x_offset = (45.0 * scale_factor).round() as i32;
            let text_pos = CvPoint::new(rect.x + rect.width - text_x_offset, left_pt.y + 5);

            put_text(
                frame,
                &text_val,
                text_pos,
                FONT_HERSHEY_SIMPLEX,
                font_scale,
                Scalar::new(180.0, 180.0, 180.0, 0.0),
                1,
                LINE_8,
                false,
            )?;
        }

        for i in 0..targets.len() - 1 {
            let pt_target_start = map_point(i, targets[i]);
            let pt_target_end = map_point(i + 1, targets[i + 1]);
            line(
                frame,
                pt_target_start,
                pt_target_end,
                Scalar::new(255.0, 0.0, 255.0, 0.0),
                line_thickness,
                LINE_8,
                0,
            )?;

            let pt_feed_start = map_point(i, feedbacks[i]);
            let pt_feed_end = map_point(i + 1, feedbacks[i + 1]);
            line(
                frame,
                pt_feed_start,
                pt_feed_end,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                line_thickness,
                LINE_8,
                0,
            )?;

            let delta_start = (targets[i] - feedbacks[i]).abs() + min_val as i16;
            let delta_end = (targets[i + 1] - feedbacks[i + 1]).abs() + min_val as i16;
            let pt_delta_start = map_point(i, delta_start);
            let pt_delta_end = map_point(i + 1, delta_end);
            line(
                frame,
                pt_delta_start,
                pt_delta_end,
                Scalar::new(0.0, 255.0, 255.0, 0.0),
                1,
                LINE_8,
                0,
            )?;
        }

        let title_x_offset = (10.0 * scale_factor).round() as i32;
        let title_y_offset = (20.0 * scale_factor).round() as i32;
        put_text(
            frame,
            label,
            CvPoint::new(rect.x + title_x_offset, rect.y + title_y_offset),
            FONT_HERSHEY_SIMPLEX,
            title_font_scale,
            Scalar::new(255.0, 255.0, 255.0, 0.0),
            1,
            LINE_8,
            false,
        )?;

        Ok(())
    }
}
