extern crate image;
extern crate kiss3d;
extern crate nalgebra as na;

use csv::Writer;
use image::{save_buffer, ColorType::Rgb8};
use kiss3d::camera::ArcBall;
use kiss3d::light::Light;
use kiss3d::resource::Material;
use kiss3d::scene::SceneNode;
use kiss3d::window::Window;
use na::{Point3, Translation3};
use planetary_motion::planet::Planet;
use planetary_motion::utility::Point;
use planetary_motion::GRAV_CONSTANT;
use std::cell::RefCell;
use std::convert::TryInto;
use std::env;
use std::f64::consts::PI;
use std::fs;
use std::path::Path;
use std::process::Command;
use std::rc::Rc;
use std::thread;
use std::time::{Duration, Instant};

const DELTA_T: f64 = 0.001;
const FPS: u64 = 100;
const DELAY: u64 = (1f64 / (DELTA_T * (FPS as f64))) as u64;
const DURATION: f64 = 50.;
const STILL_FRAME: bool = true;

fn main() {
    // Mass of the Earth in kg
    let m: f64 = 5e16;
    let r: f64 = 10.;
    let eccentricity = 0.;
    let distance: f64 = 1e2;
    let mut center_planet = Planet::new(
        m,
        r,
        Point::new(
            0.,
            -(GRAV_CONSTANT * m * (distance.recip() * (1. + eccentricity)) / 2.).sqrt(),
            0.,
        ),
        Point::new(0.0000, 0., 0.),
    );
    let mut other_planet = Planet::new(
        m,
        r,
        Point::new(
            0.,
            (GRAV_CONSTANT * m * (distance.recip() * (1. + eccentricity)) / 2.).sqrt(),
            0.,
        ),
        Point::new(distance, 0., 0.),
    );

    let mut window = Window::new("Simulation! 😎");
    window.set_light(Light::StickToCamera);
    window.set_background_color(1.0, 1.0, 1.0);

    let eye = Point3::new(0., 5000., 0.);
    let mut arc_ball = ArcBall::new(eye, Point::default().na_point());
    arc_ball.set_yaw(-PI as f32 / 2.);
    arc_ball.set_pitch(0.5 * PI as f32);

    center_planet.render(&mut window, Some(Point::new(0., 1., 0.)));
    other_planet.render(&mut window, Some(Point::new(1., 0., 0.)));

    let mut last_time: Instant = Instant::now();
    let mut time_elapsed;
    let mut total_time_elapsed = 0.;
    let mut counter: u64 = 0;
    let mut other_old_position = other_planet.position;
    let mut center_old_position = center_planet.position;

    let mut wtr = Writer::from_writer(vec![]);
    wtr.write_record(&["time", "x", "y"]).ok();

    fs::remove_dir_all("images").ok();
    fs::create_dir("images").expect("Directory not created correctly.");
    while window.render_with_camera(&mut arc_ball) {
        if (counter as f64) * DELTA_T > DURATION {
            break;
        }
        if counter % DELAY == DELAY - 1 {
            let mut out: Vec<u8> = vec![];
            window.snap(&mut out);
            save_buffer(
                &Path::new(&format!("images/img_{:0>5}.png", counter / DELAY)),
                &mut out,
                1600,
                1200,
                Rgb8,
            )
            .unwrap();
            if center_planet.position != center_old_position {
                center_planet.update_render(center_old_position);
            }

            if other_planet.position != other_old_position {
                other_planet.update_render(other_old_position);
            }
            other_old_position = other_planet.position;
            center_old_position = center_planet.position;
        }

        // println!(
        //     "center_planet: {:?}, other_planet: {:?}",
        //     center_planet.position, other_planet.position
        // );
        let force = Planet::gravitational_force(&center_planet, &other_planet);
        // println!("force: {:?}", force);

        center_planet.velocity -= force * DELTA_T / center_planet.mass;
        center_planet.kinematics_update(DELTA_T);

        other_planet.velocity += force * DELTA_T / other_planet.mass;
        other_planet.kinematics_update(DELTA_T);

        Planet::separate_planets(&mut center_planet, &mut other_planet);
        let position = center_planet.position;
        if STILL_FRAME {
            center_planet.position -= position;
            other_planet.position -= position;
        }
        wtr.write_record(&[
            total_time_elapsed.to_string(),
            other_planet.position.x.to_string(),
            other_planet.position.y.to_string(),
        ])
        .ok();

        time_elapsed = last_time.elapsed().as_secs_f64();
        if counter % 100 == 0 {
            println!("Time elapsed: {:.3}.", total_time_elapsed);
        }
        total_time_elapsed += DELTA_T;
        // assert!((DELTA_T - time_elapsed).max(0.) != 0.);
        // thread::sleep(Duration::from_secs_f64((DELTA_T - time_elapsed).max(0.)));
        last_time = Instant::now();

        counter += 1;
    }

    let data = String::from_utf8(wtr.into_inner().unwrap()).unwrap();
    fs::write("csv/test_circle.csv", data).expect("Unable to write file");

    Command::new("ffmpeg")
        .args([
            "-y",
            "-r",
            &format!("{}", FPS),
            "-f",
            "image2",
            "-s",
            "1600x1200",
            "-i",
            "images/img_%05d.png",
            "-vcodec",
            "libx264",
            "-crf",
            "25",
            "-pix_fmt",
            "yuv420p",
            "videos/temp.mp4",
        ])
        .output()
        .expect("failed to execute process");

    Command::new("ffmpeg")
        .args([
            "-y",
            "-i",
            "videos/temp.mp4",
            "-vf",
            "vflip",
            "-c:a",
            "copy",
            "videos/test_circle.mp4",
        ])
        .output()
        .expect("failed to execute process");
    fs::remove_file("videos/temp.mp4").expect("Temp file removed incorrectly.");
    fs::remove_dir_all("images").expect("Directory removed incorrectly.");
}
