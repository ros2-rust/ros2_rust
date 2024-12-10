use crate::turtle::Turtle;
use core::time;
use eframe::egui::{self, ColorImage, TextureOptions};
use eframe::egui::{Image, Ui, Vec2};
use std::collections::BTreeMap;
use std::f32::consts::FRAC_PI_2;
use std::sync::mpsc::{channel, Receiver, Sender};
use std::sync::Arc;
use std::{env, thread};
use tiny_skia::{Color, Pixmap};

pub const FRAME_WIDTH: u32 = 500;
pub const FRAME_HEIGHT: u32 = 500;

pub const TURTLE_IMG_WIDTH: f32 = 45.0;
pub const TURTLE_IMG_HEIGHT: f32 = 45.0;

const BACKGROUND_R: u8 = 69;
const BACKGROUND_G: u8 = 86;
const BACKGROUND_B: u8 = 255;
const BACKGROUND_ALPHA: u8 = 255;

pub const UPDATE_INTERVAL_MS: u64 = 16;

enum ServiceMsg {
    Clear,
    Reset,
    Kill(String),
    Spawn(f32, f32, f32, String, Sender<String>),
}

#[allow(unused)]
pub struct TurtleFrame<'a> {
    ctx: egui::Context,
    image_handle: egui::TextureHandle,
    turtle_images: Vec<egui::Image<'a>>,
    path_image: Pixmap,

    turtles: BTreeMap<String, Turtle<'a>>,
    id_count: u32,
    frame_count: u64,

    meter: f32,
    width_in_meters: f32,
    height_in_meters: f32,

    context: rclrs::Context,
    nh: Arc<rclrs::Node>,
    last_turtle_update: rclrs::Time,

    bg_r_param: rclrs::OptionalParameter<i64>,
    bg_g_param: rclrs::OptionalParameter<i64>,
    bg_b_param: rclrs::OptionalParameter<i64>,

    srv_rx: Receiver<ServiceMsg>,

    clear_srv: Arc<rclrs::Service<std_srvs::srv::Empty>>,
    kill_srv: Arc<rclrs::Service<turtlesim_rs_msgs::srv::Kill>>,
    reset_srv: Arc<rclrs::Service<std_srvs::srv::Empty>>,
    spawn_srv: Arc<rclrs::Service<turtlesim_rs_msgs::srv::Spawn>>,
}

impl<'a> TurtleFrame<'a> {
    pub fn new(ctx: egui::Context) -> Self {
        let mut turtle_images = vec![];
        load_turtle_images(&mut turtle_images);

        let turtles = BTreeMap::new();
        let frame_count = 0;
        let id_count = 0;

        let meter = TURTLE_IMG_HEIGHT;
        let width_in_meters = (FRAME_WIDTH as f32 - 1.0) / meter;
        let height_in_meters = (FRAME_HEIGHT as f32 - 1.0) / meter;
        let context = rclrs::Context::new(env::args()).unwrap();

        let nh = rclrs::create_node(&context, "turtlesim_rs").unwrap();
        println!("Starting turtlesim_rs with node name {}", nh.name());

        let last_turtle_update = nh.get_clock().now();

        let bg_r_param = nh
            .declare_parameter("background_r")
            .default(BACKGROUND_R as i64)
            .optional()
            .unwrap();

        let bg_g_param = nh
            .declare_parameter("background_g")
            .default(BACKGROUND_G as i64)
            .optional()
            .unwrap();

        let bg_b_param = nh
            .declare_parameter("background_b")
            .default(BACKGROUND_B as i64)
            .optional()
            .unwrap();

        let bg_r = bg_r_param.get().unwrap() as u8;
        let bg_g = bg_g_param.get().unwrap() as u8;
        let bg_b = bg_b_param.get().unwrap() as u8;

        let mut path_image = Pixmap::new(FRAME_WIDTH, FRAME_HEIGHT).unwrap();
        path_image.fill(Color::from_rgba8(bg_r, bg_g, bg_b, BACKGROUND_ALPHA));

        let color_image = ColorImage::from_rgba_unmultiplied(
            [FRAME_WIDTH as usize, FRAME_HEIGHT as usize],
            path_image.data(),
        );

        let image_handle = ctx.load_texture("background", color_image, Default::default());

        let (srv_tx, srv_rx) = channel();

        let clear_srv_tx = srv_tx.clone();
        let clear_srv = nh
            .create_service("clear", move |_, _| {
                clear_srv_tx.send(ServiceMsg::Clear).unwrap();
                std_srvs::srv::Empty_Response::default()
            })
            .unwrap();

        let kill_srv_tx = srv_tx.clone();
        let kill_srv = nh
            .create_service(
                "kill",
                move |_, req: turtlesim_rs_msgs::srv::Kill_Request| {
                    let turtle_name = req.name;
                    kill_srv_tx.send(ServiceMsg::Kill(turtle_name)).unwrap();
                    turtlesim_rs_msgs::srv::Kill_Response::default()
                },
            )
            .unwrap();

        let reset_srv_tx = srv_tx.clone();
        let reset_srv = nh
            .create_service("reset", move |_, _| {
                reset_srv_tx.send(ServiceMsg::Reset).unwrap();
                std_srvs::srv::Empty_Response::default()
            })
            .unwrap();

        let spawn_srv_tx = srv_tx.clone();
        let spawn_srv = nh
            .create_service(
                "spawn",
                move |_, req: turtlesim_rs_msgs::srv::Spawn_Request| {
                    let (name_tx, name_rx) = channel();
                    let (x, y, theta, turtle_name) = (req.x, req.y, req.theta, req.name);

                    spawn_srv_tx
                        .send(ServiceMsg::Spawn(x, y, theta, turtle_name, name_tx))
                        .unwrap();
                    let turtle_realname = name_rx.recv().unwrap();

                    turtlesim_rs_msgs::srv::Spawn_Response {
                        name: turtle_realname,
                    }
                },
            )
            .unwrap();

        let nh_clone = Arc::clone(&nh);
        thread::spawn(move || loop {
            std::thread::sleep(time::Duration::from_millis(UPDATE_INTERVAL_MS / 2));
            let _v = rclrs::spin_once(nh_clone.clone(), Some(time::Duration::ZERO));
        });

        TurtleFrame {
            ctx,
            image_handle,
            turtle_images,
            path_image,

            turtles,
            id_count,
            frame_count,

            context,
            nh,
            last_turtle_update,

            meter,
            width_in_meters,
            height_in_meters,

            bg_r_param,
            bg_g_param,
            bg_b_param,

            srv_rx,

            clear_srv,
            kill_srv,
            reset_srv,
            spawn_srv,
        }
    }

    pub fn get_frame_center(&self) -> (f32, f32) {
        (self.width_in_meters / 2.0, self.height_in_meters / 2.0)
    }
    pub fn spawn(&mut self, name: &str, x: f32, y: f32, angle: f32) -> String {
        let rand_usize = rand::random::<usize>() % self.turtle_images.len();
        let turtle_img = self.turtle_images[rand_usize].clone();
        self.spawn_internal(name, x, y, angle, turtle_img)
    }

    fn spawn_internal(
        &mut self,
        name: &str,
        x: f32,
        y: f32,
        angle: f32,
        image: egui::Image<'a>,
    ) -> String {
        let mut real_name = name.to_owned();
        if name.is_empty() {
            self.id_count += 1;
            let mut new_name = format!("turtle{}", self.id_count);

            while self.has_turtle(&new_name) {
                self.id_count += 1;
                new_name = format!("turtle{}", self.id_count);
            }
            real_name = new_name;
        } else if self.has_turtle(name) {
            return String::new();
        }

        let turtle = Turtle::new(
            self.nh.clone(),
            &real_name.clone(),
            image,
            egui::Pos2::new(x, self.height_in_meters - y),
            angle,
        );
        self.turtles.insert(real_name.clone(), turtle);

        self.ctx.request_repaint();
        println!(
            "Spawning turtle [{}] at x=[{}], y=[{}], theta=[{}]",
            real_name, x, y, angle
        );

        real_name
    }

    pub fn has_turtle(&self, name: &str) -> bool {
        self.turtles.contains_key(name)
    }

    pub fn update(&mut self, ui: &mut Ui) {
        self.image_handle
            .set(self.get_color_image(), TextureOptions::default());
        ui.image((self.image_handle.id(), self.image_handle.size_vec2()));
        for turtle in self.turtles.values() {
            turtle.paint(ui);
        }
    }

    pub fn update_turtles(&mut self) {
        let mut modified = false;

        for turtle in self.turtles.values_mut() {
            modified |= turtle.update(
                UPDATE_INTERVAL_MS as f64 * 0.001,
                &mut self.path_image,
                self.width_in_meters,
                self.height_in_meters,
            );
        }

        if modified {
            self.ctx.request_repaint();
        }

        self.frame_count += 1;
    }

    pub fn handle_service_requests(&mut self) {
        let service_requests = self.srv_rx.try_iter().collect::<Vec<_>>();

        if service_requests.is_empty() {
            return;
        }

        for srv_req in service_requests {
            match srv_req {
                ServiceMsg::Clear => self.clear_callback(),
                ServiceMsg::Reset => self.reset_callback(),
                ServiceMsg::Kill(turtle_name) => self.kill_callback(turtle_name),
                ServiceMsg::Spawn(x, y, theta, turtle_name, name_tx) => {
                    self.spawn_callback(x, y, theta, turtle_name, name_tx)
                }
            }
        }

        self.ctx.request_repaint();
    }

    fn get_color_image(&self) -> ColorImage {
        ColorImage::from_rgba_unmultiplied(
            [FRAME_WIDTH as usize, FRAME_HEIGHT as usize],
            self.path_image.data(),
        )
    }

    fn clear_callback(&mut self) {
        let bg_r = self.bg_r_param.get().unwrap() as u8;
        let bg_g = self.bg_g_param.get().unwrap() as u8;
        let bg_b = self.bg_b_param.get().unwrap() as u8;

        self.path_image
            .fill(Color::from_rgba8(bg_r, bg_g, bg_b, BACKGROUND_ALPHA))
    }

    fn reset_callback(&mut self) {
        self.turtles.clear();
        self.id_count = 0;
        self.spawn(
            "",
            self.width_in_meters / 2.0,
            self.height_in_meters / 2.0,
            0.0,
        );
        self.clear_callback();
    }

    fn kill_callback(&mut self, turtle_name: String) {
        let has_turtle = self.has_turtle(&turtle_name);
        if has_turtle {
            self.turtles.remove(&turtle_name);
        } else {
            println!("Tried to kill turtle {}, which does not exist", turtle_name);
        }
    }

    fn spawn_callback(
        &mut self,
        x: f32,
        y: f32,
        theta: f32,
        turtle_name: String,
        name_tx: Sender<String>,
    ) {
        let turtle_realname = self.spawn(&turtle_name, x, y, theta);
        name_tx.send(turtle_realname).unwrap();
    }
}

fn load_turtle_images(turtle_images: &mut Vec<Image<'_>>) {
    let turtles = vec![
        egui::include_image!("../images/box-turtle.png"),
        egui::include_image!("../images/robot-turtle.png"),
        egui::include_image!("../images/sea-turtle.png"),
        egui::include_image!("../images/diamondback.png"),
        egui::include_image!("../images/electric.png"),
        egui::include_image!("../images/fuerte.png"),
        egui::include_image!("../images/groovy.png"),
        egui::include_image!("../images/hydro.svg"),
        egui::include_image!("../images/indigo.svg"),
        egui::include_image!("../images/jade.png"),
        egui::include_image!("../images/kinetic.png"),
        egui::include_image!("../images/lunar.png"),
        egui::include_image!("../images/melodic.png"),
    ];

    turtle_images.reserve(turtles.len());
    for img in turtles {
        let image = egui::Image::new(img);
        turtle_images.push(
            image
                .max_size(Vec2::new(TURTLE_IMG_WIDTH, TURTLE_IMG_HEIGHT))
                .rotate(FRAC_PI_2, Vec2::splat(0.5)),
        );
    }
}
