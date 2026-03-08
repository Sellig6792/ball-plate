use opencv::Error;
use opencv::core::{Mat, MatTrait, Vec3b};

#[derive(Debug, Clone)]
pub struct Point {
    pub x: i32,
    pub y: i32,
}

impl Point {
    pub fn new(x: i32, y: i32) -> Self {
        Self { x, y }
    }
}

pub fn draw_circle(edges_map: &mut Mat, circle_points: &Vec<Point>) -> Result<(), Error> {
    for circle_point in circle_points {
        if !(0..1079).contains(&circle_point.y) || !(0..1919).contains(&circle_point.x) {
            continue;
        }
        *edges_map.at_2d_mut::<Vec3b>(circle_point.y, circle_point.x)? = Vec3b::from([0, 255, 0]);
    }

    Ok(())
}

pub fn get_circle_points(center: Point, radius: i32) -> Vec<Point> {
    let mut points = Vec::new();

    // Si le rayon est 0, on ne retourne que le centre
    if radius == 0 {
        points.push(center);
        return points;
    }

    let mut x = 0;
    let mut y = radius;
    let mut d = 3 - 2 * radius;

    while x <= y {
        // Ajout des 8 points symétriques (les 8 octants)
        // Ces variantes couvrent tout le périmètre sans aucun trou
        points.push(Point::new(center.x + x, center.y + y)); // Octant 1
        points.push(Point::new(center.x - x, center.y + y)); // Octant 2
        points.push(Point::new(center.x + x, center.y - y)); // Octant 3
        points.push(Point::new(center.x - x, center.y - y)); // Octant 4
        points.push(Point::new(center.x + y, center.y + x)); // Octant 5
        points.push(Point::new(center.x - y, center.y + x)); // Octant 6
        points.push(Point::new(center.x + y, center.y - x)); // Octant 7
        points.push(Point::new(center.x - y, center.y - x)); // Octant 8

        // Mise à jour de la variable de décision
        if d < 0 {
            d = d + 4 * x + 6;
        } else {
            d = d + 4 * (x - y) + 10;
            y -= 1;
        }
        x += 1;
    }

    points
}
