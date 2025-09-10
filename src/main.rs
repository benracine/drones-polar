//! You are given a list of 2D drone positions as (x, y) coordinates relative
//! to a radar located at the origin (0, 0). The radar has a limited field of
//! view (FOV) specified in degrees (e.g., 90.0 means the radar can "see" a 90°
//! arc at a time). The radar can be rotated to any direction, and a drone is
//! considered visible if it is within the FOV of the radar.
//!
//! Implement the optimal_radar_direction function below.
//!
//! This function should determine the optimal direction (in degrees) to point
//! the radar such that it can see the maximum number of drones within its FOV.
//!
//! You may assume:
//!     Drones directly on the boundary of the FOV are considered visible.
//!     FOV will always be ≤ 360°.
//!     Input list can contain up to 10⁵ drone positions.
//!
//!             y ↑
//!               |
//!     (-3, 4)   |      (2, 4)
//!             \ |   *
//!              \|   .
//! (-4, 2)  *    \   .   *  (3, 3)
//!               Radar (0,0) --------> x
//!              /|  *
//! (-3,-3)     / |
//!      *     /  |    *  (3,-2)
//!           /   |
//!   (-2,-4)    (0,-3)

use itertools::Itertools;
use std::f64::consts::PI;

#[derive(Debug, Clone, Copy)]
struct DronePosition {
    x: f64,
    y: f64,
}

impl From<(f64, f64)> for DronePosition {
    fn from((x, y): (f64, f64)) -> Self {
        Self { x, y }
    }
}

#[derive(Debug, Clone, Copy)]
struct DronePositionPolar {
    theta: f64, // angle in degrees
}

impl From<DronePosition> for DronePositionPolar {
    fn from(p: DronePosition) -> Self {
        let theta_deg = p.y.atan2(p.x) * 180.0 / PI;
        let normalized = (theta_deg + 360.0) % 360.0;
        Self { theta: normalized }
    }
}

/*
fn count_drones_in_range(start_angle: f64, drones: &[DronePositionPolar], fov_degrees: f64) -> usize {
    let end_angle = (start_angle + fov_degrees) % 360.0;
    if start_angle < end_angle {
        drones
            .iter()
            .filter(|&&drone| drone.theta >= start_angle && drone.theta <= end_angle)
            .count()
    } else {
        // Wrap around case
        drones
            .iter()
            .filter(|&&drone| drone.theta >= start_angle || drone.theta <= end_angle)
            .count()
    }
}

fn optimal_radar_direction(drones: &[DronePosition], fov_degrees: f64) -> f64 {
    // TODO: Implement the function.
    let radial_done_positions: Vec<DronePositionPolar> =
      drones.iter().map(|&drone| { drone.into() }).collect();

    let mut counter = HashMap::new();
    for theta_i = 0..=360 {
      let count = count_drones_in_range(theta_i, &radial_drone_positions);
      counter.insert(theta_i, count);
    }

    let max_angle = counter.max_by_key(|&(_, &val)| val);

    println!("max_angle = {:?}", max_angle);

    let sorted_drone_positions = radial_drone_positions.sortBy(|&(_, &val)| val);

    let mut max = 0;
    for (position, index) in sorted_drone_positions.enumerate() {
      let start = position.theta;
      let end = position.theta + 90;
      let mut counter = 0;

      for i index..=sorted_drone_positions.len() {
        let in_window = start < position.theta && position.theta < end;
        if in_window {
          counter += 1;
        }
      }
      if (counter > max)  {
        max = counter;
      }
    }
}
*/

fn main() {
    let coords = vec![
        (2.3, 4.1),
        (-3.7, 1.5),
        (5.5, -0.9),
        (-1.2, -3.4),
        (0.8, -2.7),
        (4.0, 2.2),
        (-2.8, -4.5),
        (3.3, 3.8),
        (-5.1, 0.4),
        (1.6, -1.1),
        (-3.9, 2.6),
        (2.0, 5.0),
        (4.4, -3.3),
        (-0.7, 2.9),
        (0.1, -4.8),
        (2.7, -1.6),
        (-4.6, -2.0),
        (1.9, 3.2),
        (-2.2, 4.7),
        (3.6, -2.5),
    ];

    let sorted_radial_drone_positions = coords
        .into_iter()
        .map(DronePosition::from)
        .map(DronePositionPolar::from)
        .sorted_by(|a, b| {
            a.theta
                .partial_cmp(&b.theta)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .collect::<Vec<_>>();

    // Ok, now invoke a function that computes the optimal radar direction
    let fov_degrees = 90.0;
    let optimal_direction = find_optimal_radar_direction(&sorted_radial_drone_positions, &fov_degrees);

}
