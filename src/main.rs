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

const FOV_DEGREES: f64 = 90.0;

/// Cartesian coordinates of a drone.
#[derive(Debug, Clone, Copy)]
struct DronePosition {
    x: f64,
    y: f64,
}

/// From tuple (x, y) to DronePosition
impl From<(f64, f64)> for DronePosition {
    fn from((x, y): (f64, f64)) -> Self {
        Self { x, y }
    }
}

/// Polar coordinates of a drone.
#[derive(Debug, Clone, Copy)]
struct DronePositionPolar {
    theta: f64, // angle in degrees
}

/// From Cartesian to Polar coordinates
impl From<DronePosition> for DronePositionPolar {
    fn from(p: DronePosition) -> Self {
        // atan2 returns angle in radians
        // convert to degrees 
        // Theta could be negative, so we add 360 and then take modulo 360 to ensure it's in [0, 360)
        let theta = (p.y.atan2(p.x) * 180.0 / PI + 360.0) % 360.0;
        Self { theta }
    }
}

/// Get the raw drone data from the field in the form of a vector of tuples
fn collect_raw_drone_data() -> Vec<(f64, f64)> {
    vec![
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
    ]
}

/// Calculate the center angle of an FOV window, handling wraparound
fn calculate_fov_center(start_angle: f64, fov: f64) -> f64 {
    let end_angle = start_angle + fov;
    if end_angle <= 360.0 {
        (start_angle + end_angle) / 2.0
    } else {
        ((start_angle + end_angle) / 2.0) % 360.0
    }
}

/// Find the optimal radar direction to maximize the number of visible drones
fn find_optimal_radar_direction(sorted_positions: Vec<DronePositionPolar>, fov: f64) -> f64 {
    let n = sorted_positions.len();

    // Duplicate with +360° offsets to handle wraparound
    let mut angles: Vec<f64> = sorted_positions.iter().map(|p| p.theta).collect();
    let extra_angles: Vec<f64> = angles.iter().map(|a| a + 360.0).collect();
    angles.extend(extra_angles);

    let mut max_visible = 0;
    let mut best_start_angle = 0.0;

    let mut j = 0;

    // Starting with the position of each drone, see how many fit in the FOV
    for i in 0..n {
        let start_angle = angles[i];
        let end_angle = start_angle + fov;

        // Gets us to O(n * log(n)) because at least we're not iterating over the
        // whole array each time like we were with the filter
        while j < angles.len() && angles[j] <= end_angle {
            j += 1;
        }
        let visible = j - i;

        if visible > max_visible {
            max_visible = visible;
            best_start_angle = start_angle;
        }
    }

    calculate_fov_center(best_start_angle, fov)
}

fn main() {
    // Get the raw data from the field
    let coords = collect_raw_drone_data();

    // Deal with the empty data case
    let n = coords.len();
    if n == 0 {
        println!("Not enough data to determine optimal direction.");
        return;
    }

    // Reformat the drone data to be in polar coordinates and sorted from low to high theta angles
    // Map from tuple -> DronePosition -> DronePositionPolar
    // Then sort by low to high theta angle
    let sorted_radial_drone_positions: Vec<DronePositionPolar> = coords
        .into_iter()
        .map(DronePosition::from)
        .map(DronePositionPolar::from)
        .sorted_by(|a, b| {
            a.theta
                .partial_cmp(&b.theta)
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .collect();

    let optimal_direction =
        find_optimal_radar_direction(sorted_radial_drone_positions, FOV_DEGREES);

    println!("Optimal radar direction: {:.2}°", optimal_direction);
}
