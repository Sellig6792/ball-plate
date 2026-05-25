use crate::utils::Point;

pub fn in_a_second(previous: Point, current: Point, dt: f32) -> Point {
    let diff = Point::new(current.x - previous.x, current.y - previous.y);
    Point::new(
        current.x + (diff.x as f32 / dt) as i32,
        current.y + (diff.y as f32 / dt) as i32,
    )
}
