use num_traits::FromPrimitive;
use num_traits::ToPrimitive;
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

pub fn get_distances_from_average(edge_points: &Vec<Point>, average_point: &Point) -> Vec<i32> {
    let v: Vec<_> = edge_points
        .iter()
        .enumerate()
        .flat_map(|(_, point)| {
            let dx = point.x - average_point.x;
            let dy = point.y - average_point.y;

            // Filtrage des distances extrêmes
            // println!("DX: {}, DY: {}", dx, dy);
            let d = 30;
            if dx.abs() > 160 + d || dx.abs() < 160 - d || dy.abs() > 160 + d || dy.abs() < 160 - d
            {
                return None;
            }

            // Calcul de l'hypoténuse : sqrt(dx² + dy²)
            let distance = ((dx.pow(2) + dy.pow(2)) as f32).sqrt();

            Some(distance.round() as i32)
        })
        .collect();

    if v.len() > 0 {
        return v;
    };
    // None.unwrap()
    let mut v = edge_points
        .iter()
        .map(|point| {
            let dx = point.x - average_point.x;
            let dy = point.y - average_point.y;
            ((dx.pow(2) + dy.pow(2)) as f32).sqrt().round() as i32
        })
        .collect::<Vec<_>>();
    v.sort();
    v
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

pub fn average<T>(values: &[T]) -> Option<T>
where
    T: ToPrimitive + FromPrimitive + Copy,
{
    if values.is_empty() {
        return None;
    }

    let sum: f64 = values.iter().map(|v| v.to_f64().unwrap()).sum();
    let avg = sum / (values.len() as f64);

    T::from_f64(avg)
}

pub fn filtered_average<T>(values: &[T], sensitivity: f64) -> Option<T>
where
    T: ToPrimitive + FromPrimitive + Copy,
{
    // return average(values);
    if values.is_empty() {
        return None;
    }

    // 1. Work in f64 internally for all math
    let f_values: Vec<f64> = values.iter().filter_map(|v| v.to_f64()).collect();

    // 2. Initial mean
    let mean = f_values.iter().sum::<f64>() / f_values.len() as f64;

    // 3. Standard Deviation
    let variance =
        f_values.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / f_values.len() as f64;

    let std_dev = variance.sqrt();

    // 4. Filter
    let filtered: Vec<f64> = f_values
        .into_iter()
        .filter(|&x| (x - mean).abs() <= sensitivity * std_dev)
        .collect();

    if filtered.is_empty() {
        return T::from_f64(mean);
    }

    // 5. Final average converted back to T
    let final_avg = filtered.iter().sum::<f64>() / filtered.len() as f64;
    T::from_f64(final_avg)
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
