use core::time;
use eframe::egui::{self, CentralPanel, Frame, ViewportBuilder};
use std::sync::{Arc, Mutex};
use std::thread;

use egui_extras::install_image_loaders;
use turtlesim_rs::turtle_frame::{TurtleFrame, FRAME_HEIGHT, FRAME_WIDTH, UPDATE_INTERVAL_MS};

fn main() {
    let viewport = ViewportBuilder::default()
        .with_resizable(false)
        .with_inner_size((FRAME_WIDTH as f32, FRAME_HEIGHT as f32));

    let native_options = eframe::NativeOptions {
        viewport,
        ..Default::default()
    };

    let _ = eframe::run_native(
        "TurtleSim_rs",
        native_options,
        Box::new(|cc| {
            install_image_loaders(&cc.egui_ctx);
            Box::new(MyEguiApp::new(cc))
        }),
    );
}

struct MyEguiApp {
    turtle_frame: Arc<Mutex<TurtleFrame<'static>>>,
}

impl MyEguiApp {
    fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let mut turtle_frame = TurtleFrame::new(cc.egui_ctx.clone());

        let (x, y) = turtle_frame.get_frame_center();
        let theta = 0.0;
        let turtle_name = "";
        turtle_frame.spawn(turtle_name, x, y, theta);

        let turtle_frame = Arc::new(Mutex::new(turtle_frame));

        let turtle_frame_clone = Arc::clone(&turtle_frame);
        thread::spawn(move || loop {
            std::thread::sleep(time::Duration::from_millis(UPDATE_INTERVAL_MS));
            let mut frame = turtle_frame_clone.lock().unwrap();
            frame.update_turtles();
            frame.handle_service_requests();
        });

        MyEguiApp { turtle_frame }
    }
}

impl eframe::App for MyEguiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        CentralPanel::default()
            .frame(Frame::default())
            .show(ctx, |ui| {
                let mut frame = self.turtle_frame.lock().unwrap();
                frame.update(ui)
            });
    }
}
