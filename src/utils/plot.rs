use opencv::core::{Mat, MatTraitConst, Point as CvPoint, Rect, Scalar, add_weighted};
use opencv::imgproc::{FONT_HERSHEY_SIMPLEX, LINE_8, line, put_text};
use std::collections::VecDeque;

pub struct TelemetryPlot {
    target_x_history: VecDeque<i16>,
    feedback_x_history: VecDeque<i16>,
    target_y_history: VecDeque<i16>,
    feedback_y_history: VecDeque<i16>,
    max_points: usize,
    width: i32,
    height: i32,
}

impl TelemetryPlot {
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

    /// Appends target and real state updates for both channels into rolling queues
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

    /// Automatically handles symmetric layout math and draws both graphs
    pub fn draw(&self, frame: &mut Mat) -> Result<(), opencv::Error> {
        let margin = 20;
        let frame_width = frame.cols();

        // Compute starting positions for horizontal layout
        let x_pos_graph_x = margin;
        let x_pos_graph_y = frame_width - margin - self.width;
        let top_y = 20;

        self.draw_axis(frame, x_pos_graph_x, top_y, true)?;
        self.draw_axis(frame, x_pos_graph_y, top_y, false)?;

        Ok(())
    }

    /// Blits telemetry lines of a specific selection onto the frame with a transparent card
    pub fn draw_axis(
        &self,
        frame: &mut Mat,
        x: i32,
        y: i32,
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

        // --- TRANSPARENT BACKGROUND MATRIX BLENDING ---
        let roi_rect = Rect::new(x, y, self.width, self.height);
        let mut sub_mat = Mat::roi_mut(frame, roi_rect)?;

        let overlay = Mat::new_size_with_default(
            roi_rect.size(),
            sub_mat.typ(),
            Scalar::new(25.0, 25.0, 25.0, 0.0),
        )?;

        let mut blended = Mat::default();
        let alpha = 0.60;
        let beta = 1.0 - alpha;
        add_weighted(&sub_mat, alpha, &overlay, beta, 0.0, &mut blended, -1)?;
        blended.copy_to(&mut sub_mat)?;
        // --------------------------------------------------------------------

        // Scale setup ranging from 90 to 270 degrees
        let min_val = 90.0f32;
        let max_val = 270.0f32;
        let val_range = max_val - min_val;

        // Maps absolute data into localized chart coordinates
        let map_point = |index: usize, value: i16| -> CvPoint {
            let pct_x = index as f32 / (self.max_points - 1) as f32;
            let pct_y = (value as f32 - min_val) / val_range;
            let pt_x = x + (pct_x * self.width as f32) as i32;
            let pt_y = y + self.height - (pct_y * self.height as f32) as i32;
            CvPoint::new(pt_x, pt_y)
        };

        // --- DRAW GRADUATIONS & GRIDLINES ---
        // We'll draw dashed/subtle lines for 90, 135, 180, 225, and 270 degrees
        let graduation_levels = [90, 135, 180, 225, 270];
        for &level in &graduation_levels {
            // Get the Y position for this specific angle level
            let left_pt = map_point(0, level);
            let right_pt = map_point(self.max_points - 1, level);

            // Subtle dark gray gridline
            line(
                frame,
                left_pt,
                right_pt,
                Scalar::new(60.0, 60.0, 60.0, 0.0),
                1,
                LINE_8,
                0,
            )?;

            // Graduation value text (e.g., "180°")
            let text_val = format!("{}*", level); // Using '*' or 'deg' since standard OpenCV fonts don't render '°' well
            let text_pos = CvPoint::new(x + self.width - 45, left_pt.y + 5);
            put_text(
                frame,
                &text_val,
                text_pos,
                FONT_HERSHEY_SIMPLEX,
                0.35,
                Scalar::new(180.0, 180.0, 180.0, 0.0), // Muted gray text
                1,
                LINE_8,
                false,
            )?;
        }

        // --- DRAW TELEMETRY DATASET CURVES ---
        for i in 0..targets.len() - 1 {
            // 1. Requested Target Curve (Magenta)
            let pt_target_start = map_point(i, targets[i]);
            let pt_target_end = map_point(i + 1, targets[i + 1]);
            line(
                frame,
                pt_target_start,
                pt_target_end,
                Scalar::new(255.0, 0.0, 255.0, 0.0),
                2,
                LINE_8,
                0,
            )?;

            // 2. Hardware Response Curve (Cyan)
            let pt_feed_start = map_point(i, feedbacks[i]);
            let pt_feed_end = map_point(i + 1, feedbacks[i + 1]);
            line(
                frame,
                pt_feed_start,
                pt_feed_end,
                Scalar::new(255.0, 255.0, 0.0, 0.0),
                2,
                LINE_8,
                0,
            )?;

            // 3. Absolute Delta Curve (Yellow)
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

        // Overlay axis title text
        put_text(
            frame,
            label,
            CvPoint::new(x + 10, y + 20),
            FONT_HERSHEY_SIMPLEX,
            0.5,
            Scalar::new(255.0, 255.0, 255.0, 0.0),
            1,
            LINE_8,
            false,
        )?;

        Ok(())
    }
}
