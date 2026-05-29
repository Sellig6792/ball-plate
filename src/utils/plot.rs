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

    /// Blits telemetry lines of a specific selection onto the frame with a transparent card
    pub fn draw_axis(
        &self,
        frame: &mut Mat,
        x: i32,
        y: i32,
        is_axis_x: bool,
    ) -> Result<(), opencv::Error> {
        let (targets, feedbacks, label) = if is_axis_x {
            (
                &self.target_x_history,
                &self.feedback_x_history,
                "Servo X (Deg)",
            )
        } else {
            (
                &self.target_y_history,
                &self.feedback_y_history,
                "Servo Y (Deg)",
            )
        };

        if targets.is_empty() {
            return Ok(());
        }

        // --- TRANSPARENT BACKGROUND MATRIX BLENDING (FIXED BORROW CHECKER) ---
        // 1. Define the region of interest (ROI)
        let roi_rect = Rect::new(x, y, self.width, self.height);

        // 2. Extract a mutable sub-matrix view of the region
        let mut sub_mat = Mat::roi_mut(frame, roi_rect)?;

        // 3. Create the dark tint overlay layer matching the dimensions of the ROI
        let overlay = Mat::new_size_with_default(
            roi_rect.size(),
            sub_mat.typ(),
            Scalar::new(25.0, 25.0, 25.0, 0.0), // Tint color
        )?;

        // 4. Blend to a temporary Mat first to prevent simultaneous mutable/immutable borrowing
        let mut blended = Mat::default();
        let alpha = 0.60;
        let beta = 1.0 - alpha;
        add_weighted(&sub_mat, alpha, &overlay, beta, 0.0, &mut blended, -1)?;

        // 5. Safely copy the blended pixels back into the sub_mat frame area
        blended.copy_to(&mut sub_mat)?;
        // --------------------------------------------------------------------

        // Scale setup ranging from 0 to 300 degrees
        let min_val = 0.0f32;
        let max_val = 300.0f32;
        let val_range = max_val - min_val;

        let map_point = |index: usize, value: i16| -> CvPoint {
            let pct_x = index as f32 / (self.max_points - 1) as f32;
            let pct_y = (value as f32 - min_val) / val_range;
            let pt_x = x + (pct_x * self.width as f32) as i32;
            let pt_y = y + self.height - (pct_y * self.height as f32) as i32;
            CvPoint::new(pt_x, pt_y)
        };

        // Draw opaque telemetry dataset curves directly on top of the blended zone
        for i in 0..targets.len() - 1 {
            // Requested Target Curve (Magenta)
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

            // Hardware Response Curve (Cyan)
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
        }

        // Overlay axis label text
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
