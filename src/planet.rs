extern crate kiss3d;
extern crate nalgebra as na;

use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Point3, Translation3, Vector3};

use crate::utility::*;
use crate::{GRAV_CONSTANT, ROCKET_FACTOR};

pub struct Planet {
    pub radius: f64,
    pub mass: f64,
    pub velocity: Point,
    pub position: Point,
    pub scene_node: Option<SceneNode>,
}

impl Planet {
    pub fn new(mass: f64, radius: f64, velocity: Point, position: Point) -> Self {
        Self {
            mass,
            radius,
            velocity,
            position,
            scene_node: None,
        }
    }
    pub fn gravitational_force(&self, other_planet: &Self) -> Point {
        let force_mag = GRAV_CONSTANT
            * self.mass
            * other_planet.mass
            * (self.position - other_planet.position)
                .squared_mag()
                .recip();
        (self.position - other_planet.position) / (self.position - other_planet.position).mag()
            * force_mag
    }

    pub fn rocket_force(&self, other_planet: &Self) -> Point {
        let force_mag = GRAV_CONSTANT
            * self.mass
            * other_planet.mass
            * (self.position - other_planet.position)
                .squared_mag()
                .recip()
            * ((self.position - other_planet.position).angle() * 2.).sin()
            * ROCKET_FACTOR;

        (self.position - other_planet.position) / (self.position - other_planet.position).mag()
            * force_mag
    }

    pub fn kinematics_update(&mut self, delta_t: f64) {
        self.position += self.velocity * delta_t;
    }

    pub fn separate_planets(&mut self, other_planet: &mut Self) {
        let other_position = other_planet.position;
        let position = self.position;
        let dist = position - other_position;
        let length = dist.mag();
        let unit_dist = dist / length;
        let extension = (other_planet.radius + self.radius - length) / 2.;

        if extension > 0. {
            other_planet.position = other_position + unit_dist * (-extension);
            self.position = position + unit_dist * extension;
            other_planet.velocity = Point::default();
            self.velocity = Point::default();
        }
    }

    pub fn clear(&mut self, window: &mut Window) {
        if let Some(scene_node) = &mut self.scene_node {
            window.remove_node(scene_node);
            self.scene_node = None;
        }
    }

    pub fn render(&mut self, window: &mut Window, color: Option<Point>) {
        self.clear(window);
        let mut scene_node = Some(window.add_sphere(self.radius as f32));
        if self.position.mag() != 0. {
            let up = Vector3::<f32>::y();
            scene_node.as_mut().unwrap().reorient(
                &self.position.na_point(),
                &Point::new(0., 1., 0.).na_point(),
                &up,
            );
        }

        if let Some(col) = color {
            scene_node
                .as_mut()
                .unwrap()
                .set_color(col.x as f32, col.y as f32, col.z as f32);
        }

        self.scene_node = scene_node;
    }
    pub fn update_render(&mut self, old_position: Point) {
        let up = Vector3::<f32>::y();
        if let Some(scene_node) = &mut self.scene_node {
            scene_node.append_translation(&Translation3::new(
                (self.position.x - old_position.x) as f32,
                (self.position.y - old_position.y) as f32,
                (self.position.z - old_position.z) as f32,
            ));
            // scene_node.reorient(
            //     &self.position.na_point(),
            //     &Point::new(0., 1., 0.).na_point(),
            //     &up,
            // );
        }
    }
}
