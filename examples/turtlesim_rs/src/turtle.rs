use eframe::egui::{Image, Pos2, Rect, Ui, Vec2};
use rclrs::{Node, Time, QOS_PROFILE_DEFAULT};
use std::sync::mpsc::{channel, Receiver};
use std::time;
use std::{
    f32::consts::{FRAC_PI_2, PI},
    sync::{Arc, Mutex},
};
use tiny_skia::{LineCap, Paint, PathBuilder, Pixmap, Stroke, Transform};

use crate::turtle_frame::{TURTLE_IMG_HEIGHT, TURTLE_IMG_WIDTH};

const DEFAULT_PEN_R: u8 = 0xb3;
const DEFAULT_PEN_G: u8 = 0xb8;
const DEFAULT_PEN_B: u8 = 0xff;
const DEFAULT_PEN_ALPHA: u8 = 255;
const DEFAULT_STROKE_WIDTH: f32 = 3.0;

enum TurtleSrvs {
    SetPen(u8, u8, u8, u8, u8),
    TeleportAbsolute(f32, f32, f32),
    TeleportRelative(f32, f32),
}

pub struct TurtleVel {
    lin_vel: f64,
    ang_vel: f64,
    last_command_time: Time,
}

pub struct Pen<'a> {
    paint: Paint<'a>,
    stroke: Stroke,
}

#[allow(unused)]
pub struct Turtle<'a> {
    node: Arc<Node>,
    image: Image<'a>,
    pos: Pos2,
    orient: f32,
    meter: f32,

    pen_: Pen<'a>,
    pen_on: bool,

    turtle_vel: Arc<Mutex<TurtleVel>>,

    srv_rx: Receiver<TurtleSrvs>,

    velocity_sub: Arc<rclrs::Subscription<geometry_msgs::msg::Twist>>,
    pose_pub: Arc<rclrs::Publisher<turtlesim_rs_msgs::msg::Pose>>,
    color_pub: Arc<rclrs::Publisher<turtlesim_rs_msgs::msg::Color>>,
    set_pen_srv: Arc<rclrs::Service<turtlesim_rs_msgs::srv::SetPen>>,
    teleport_absolute_srv: Arc<rclrs::Service<turtlesim_rs_msgs::srv::TeleportAbsolute>>,
    teleport_relative_srv: Arc<rclrs::Service<turtlesim_rs_msgs::srv::TeleportRelative>>,
}

impl<'a> Turtle<'a> {
    pub fn new(node: Arc<Node>, real_name: &str, image: Image<'a>, pos: Pos2, orient: f32) -> Self {
        let meter = TURTLE_IMG_HEIGHT;

        let turtle_vel = Arc::new(Mutex::new(TurtleVel {
            lin_vel: 0.0,
            ang_vel: 0.0,
            last_command_time: node.get_clock().now(),
        }));

        let turtle_vel_clone = Arc::clone(&turtle_vel);
        let node_clone = Arc::clone(&node);

        let velocity_sub = node
            .create_subscription(
                &(real_name.to_owned() + "/cmd_vel"),
                QOS_PROFILE_DEFAULT,
                move |msg: geometry_msgs::msg::Twist| {
                    let mut vel = turtle_vel_clone.lock().unwrap();
                    vel.lin_vel = msg.linear.x;
                    vel.ang_vel = msg.angular.z;
                    vel.last_command_time = node_clone.get_clock().now();
                },
            )
            .unwrap();

        let pose_pub = node
            .create_publisher(&(real_name.to_owned() + "/pose"), QOS_PROFILE_DEFAULT)
            .unwrap();

        let color_pub = node
            .create_publisher(
                &(real_name.to_owned() + "/color_sensor"),
                QOS_PROFILE_DEFAULT,
            )
            .unwrap();

        let (srv_tx, srv_rx) = channel();

        let set_pen_srv_tx = srv_tx.clone();
        let set_pen_srv = node
            .create_service(
                &(real_name.to_owned() + "/set_pen"),
                move |_, srv: turtlesim_rs_msgs::srv::SetPen_Request| {
                    let (r, g, b, width, off) = (srv.r, srv.g, srv.b, srv.width, srv.off);
                    set_pen_srv_tx
                        .send(TurtleSrvs::SetPen(r, g, b, width, off))
                        .unwrap();
                    turtlesim_rs_msgs::srv::SetPen_Response::default()
                },
            )
            .unwrap();

        let teleport_absolute_srv_tx = srv_tx.clone();
        let teleport_absolute_srv = node
            .create_service(
                &(real_name.to_owned() + "/teleport_absolute"),
                move |_, srv: turtlesim_rs_msgs::srv::TeleportAbsolute_Request| {
                    let x = srv.x;
                    let y = srv.y;
                    let theta = srv.theta;
                    teleport_absolute_srv_tx
                        .send(TurtleSrvs::TeleportAbsolute(x, y, theta))
                        .unwrap();
                    turtlesim_rs_msgs::srv::TeleportAbsolute_Response::default()
                },
            )
            .unwrap();

        let teleport_relative_srv_tx = srv_tx.clone();
        let teleport_relative_srv = node
            .create_service(
                &(real_name.to_owned() + "/teleport_relative"),
                move |_, srv: turtlesim_rs_msgs::srv::TeleportRelative_Request| {
                    let linear = srv.linear;
                    let angular = srv.angular;
                    teleport_relative_srv_tx
                        .send(TurtleSrvs::TeleportRelative(linear, angular))
                        .unwrap();
                    turtlesim_rs_msgs::srv::TeleportRelative_Response::default()
                },
            )
            .unwrap();

        let stroke = Stroke {
            width: DEFAULT_STROKE_WIDTH,
            line_cap: LineCap::Round,
            ..Default::default()
        };

        let mut paint = Paint::default();
        paint.set_color_rgba8(
            DEFAULT_PEN_R,
            DEFAULT_PEN_B,
            DEFAULT_PEN_G,
            DEFAULT_PEN_ALPHA,
        );
        paint.anti_alias = true;

        let pen_ = Pen { paint, stroke };
        let pen_on = true;

        Turtle {
            node,
            image,
            pos,
            orient,
            meter,

            pen_,
            pen_on,

            turtle_vel,

            srv_rx,

            velocity_sub,
            pose_pub,
            color_pub,
            set_pen_srv,
            teleport_absolute_srv,
            teleport_relative_srv,
        }
    }

    fn rotate_image(&mut self) {
        let image = self.image.clone();
        self.image = image.rotate(-self.orient + FRAC_PI_2, Vec2::splat(0.5));
    }

    pub fn update(
        &mut self,
        dt: f64,
        path_image: &mut Pixmap,
        canvas_width: f32,
        canvas_height: f32,
    ) -> bool {
        let mut modified = false;

        let old_orient = self.orient;
        let old_pos = self.pos;

        modified |= self.handle_service_requests(path_image, old_pos, canvas_height);

        let mut turtle_vel = self.turtle_vel.lock().unwrap();
        let is_old_command = self
            .node
            .get_clock()
            .now()
            .compare_with(&turtle_vel.last_command_time, |now_ns, command_ns| {
                let diff_ns = (now_ns - command_ns) as u64;
                time::Duration::from_nanos(diff_ns) > time::Duration::from_secs(1)
            })
            .unwrap();

        if is_old_command {
            turtle_vel.lin_vel = 0.0;
            turtle_vel.ang_vel = 0.0;
        }

        let lin_vel_ = turtle_vel.lin_vel;
        let ang_vel_ = turtle_vel.ang_vel;

        drop(turtle_vel);

        self.orient += (ang_vel_ * dt) as f32;
        // Keep orient between -pi and +pi
        self.orient -= 2.0 * PI * ((self.orient + PI) / (2.0 * PI)).floor();

        self.pos.x += self.orient.cos() * (lin_vel_ * dt) as f32;
        self.pos.y += -self.orient.sin() * (lin_vel_ * dt) as f32;

        // Clamp to screen size
        if self.pos.x < 0.0
            || self.pos.x > canvas_width
            || self.pos.y < 0.0
            || self.pos.y > canvas_height
        {
            println!(
                "Oh no! I hit the wall! (Clamping from [x={}, y={}])",
                self.pos.x, self.pos.y
            );
        }

        self.pos.x = f32::min(f32::max(self.pos.x, 0.0), canvas_width);
        self.pos.y = f32::min(f32::max(self.pos.y, 0.0), canvas_height);

        let pose_msg = turtlesim_rs_msgs::msg::Pose {
            x: self.pos.x,
            y: canvas_height - self.pos.y,
            theta: self.orient,
            linear_velocity: lin_vel_ as f32,
            angular_velocity: ang_vel_ as f32,
        };

        self.pose_pub.publish(pose_msg).unwrap();

        let pixel_color = path_image.pixel(
            (self.pos.x * self.meter) as u32,
            ((canvas_height - self.pos.y) * self.meter) as u32,
        );

        if let Some(color) = pixel_color {
            let color_msg = turtlesim_rs_msgs::msg::Color {
                r: color.red(),
                g: color.green(),
                b: color.blue(),
            };
            self.color_pub.publish(color_msg).unwrap();
        }

        if self.orient != old_orient {
            modified = true;
            self.rotate_image();
        }

        if self.pos != old_pos {
            modified = true;

            if self.pen_on {
                self.draw_line_on_path_image(path_image, old_pos, self.pos);
            }
        }

        modified
    }

    fn handle_service_requests(
        &mut self,
        path_image: &mut Pixmap,
        old_pos: Pos2,
        canvas_height: f32,
    ) -> bool {
        let mut modified = false;

        for srvs in self.srv_rx.try_iter() {
            match srvs {
                TurtleSrvs::SetPen(r, g, b, width, off) => {
                    self.pen_.paint.set_color_rgba8(r, g, b, 255);
                    self.pen_.stroke.width = width as f32;
                    self.pen_on = off == 0;
                }
                TurtleSrvs::TeleportAbsolute(x, y, theta) => {
                    self.pos.x = x;
                    self.pos.y = canvas_height - y;
                    self.orient = theta;
                    self.draw_line_on_path_image(path_image, old_pos, self.pos);
                    modified = true;
                }
                TurtleSrvs::TeleportRelative(linear, angular) => {
                    self.orient += angular;
                    self.pos.x += self.orient.cos() * linear;
                    self.pos.y += -self.orient.sin() * linear;
                    self.draw_line_on_path_image(path_image, old_pos, self.pos);
                    modified = true;
                }
            }
        }

        modified
    }

    pub fn paint(&self, ui: &mut Ui) {
        let top_left_pos = Pos2 {
            x: self.pos.x * self.meter - TURTLE_IMG_WIDTH / 2.0,
            y: self.pos.y * self.meter - TURTLE_IMG_HEIGHT / 2.0,
        };
        let image_rect =
            Rect::from_min_size(top_left_pos, Vec2::new(TURTLE_IMG_WIDTH, TURTLE_IMG_HEIGHT));
        self.image.paint_at(ui, image_rect);
    }

    fn draw_line_on_path_image(&self, path_image: &mut Pixmap, pos1: Pos2, pos2: Pos2) {
        let mut path_builder = PathBuilder::new();
        path_builder.move_to(pos1.x * self.meter, pos1.y * self.meter);
        path_builder.line_to(pos2.x * self.meter, pos2.y * self.meter);

        if let Some(path) = path_builder.finish() {
            path_image.stroke_path(
                &path,
                &self.pen_.paint,
                &self.pen_.stroke,
                Transform::identity(),
                None,
            );
        }
    }
}
