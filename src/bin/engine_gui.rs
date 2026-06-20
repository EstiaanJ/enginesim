use std::time::Instant;

use eframe::egui::{self, Color32, RichText};
use egui_plot::{Line, Plot, PlotPoints, Points};

use enginesim::engine_config::EngineDefinition;
use enginesim::profiles::SimulationProfile;
use enginesim::single_cylinder::{SingleCylinderEngine, rad_per_s_to_rpm};
use enginesim::telemetry::{
    AngleTraceSnapshot, EngineControls, EngineFrameTelemetry, TelemetryAggregator,
    TelemetryAvailability, default_profile_for_gui,
};

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions::default();
    eframe::run_native(
        "EngineSim GUI",
        options,
        Box::new(|_cc| Ok(Box::new(EngineGuiApp::new()))),
    )
}

struct EngineGuiApp {
    definition: EngineDefinition,
    profile: SimulationProfile,
    engine: SingleCylinderEngine,
    controls: EngineControls,
    telemetry: TelemetryAggregator,
    latest_frame: EngineFrameTelemetry,
    running: bool,
    last_wall_clock: Instant,
}

impl EngineGuiApp {
    fn new() -> Self {
        let definition =
            EngineDefinition::from_json_str(include_str!("../../data/engines/gn250.json"))
                .expect("bundled GN250 JSON should parse");
        let profile = default_profile_for_gui(&definition);
        let controls = EngineControls::default();
        let engine =
            SingleCylinderEngine::from_definition_with_profile(definition.clone(), profile);
        let telemetry = TelemetryAggregator::new(definition.clone(), controls);
        let latest_frame = telemetry.snapshot();

        Self {
            definition,
            profile,
            engine,
            controls,
            telemetry,
            latest_frame,
            running: true,
            last_wall_clock: Instant::now(),
        }
    }

    fn reset_engine(&mut self) {
        self.engine = SingleCylinderEngine::from_definition_with_profile(
            self.definition.clone(),
            self.profile,
        );
        self.telemetry = TelemetryAggregator::new(self.definition.clone(), self.controls);
        self.latest_frame = self.telemetry.snapshot();
        self.last_wall_clock = Instant::now();
    }

    fn simulate_until_now(&mut self) {
        let now = Instant::now();
        let mut wall_delta_seconds = (now - self.last_wall_clock).as_secs_f64();
        self.last_wall_clock = now;
        if !self.running {
            return;
        }

        wall_delta_seconds = wall_delta_seconds.clamp(0.0, 0.05);
        let timestep_seconds = self.profile.timestep_seconds;
        let steps = (wall_delta_seconds / timestep_seconds).ceil() as usize;

        self.telemetry.set_controls(self.controls);
        for _ in 0..steps.max(1) {
            let output = self.engine.step(
                self.controls
                    .to_step_inputs(rad_per_s_to_rpm(self.engine.crank_speed_rad_per_s())),
            );
            self.telemetry.ingest_step_output(output, timestep_seconds);
            if let Some(frame) = self.telemetry.publish_ready() {
                self.latest_frame = frame;
            }
        }
    }
}

impl eframe::App for EngineGuiApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.simulate_until_now();

        egui::SidePanel::left("controls")
            .resizable(true)
            .default_width(260.0)
            .show(ctx, |ui| {
                ui.heading("Controls");
                ui.checkbox(&mut self.running, "Run");
                if ui.button("Reset").clicked() {
                    self.reset_engine();
                }
                ui.separator();
                ui.label("Profile");
                let mut profile_kind = self.profile.kind;
                egui::ComboBox::from_label("")
                    .selected_text(format!("{profile_kind:?}"))
                    .show_ui(ui, |ui| {
                        ui.selectable_value(
                            &mut profile_kind,
                            enginesim::profiles::SimulationProfileKind::RealTime,
                            "RealTime",
                        );
                        ui.selectable_value(
                            &mut profile_kind,
                            enginesim::profiles::SimulationProfileKind::Render,
                            "Render",
                        );
                    });
                if profile_kind != self.profile.kind {
                    self.profile = match profile_kind {
                        enginesim::profiles::SimulationProfileKind::RealTime => {
                            default_profile_for_gui(&self.definition)
                        }
                        enginesim::profiles::SimulationProfileKind::Render => {
                            SimulationProfile::render()
                        }
                    };
                    self.reset_engine();
                }
                ui.separator();
                ui.add(
                    egui::Slider::new(&mut self.controls.throttle_position, 0.0..=1.0)
                        .text("Throttle")
                        .suffix(" frac"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.idle_leak_fraction, 0.0..=0.25)
                        .text("Idle Leak")
                        .suffix(" frac"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.lambda_target, 0.8..=1.2)
                        .text("Lambda Target"),
                );
                ui.separator();
                ui.checkbox(&mut self.controls.starter_enabled, "Starter");
                ui.add(
                    egui::Slider::new(&mut self.controls.starter_torque_nm, 0.0..=60.0)
                        .text("Starter Torque")
                        .suffix(" Nm"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.starter_speed_limit_rpm, 0.0..=3000.0)
                        .text("Starter Limit")
                        .suffix(" rpm"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.added_inertia_kg_m2, 0.0..=0.05)
                        .text("Added Inertia")
                        .suffix(" kg m^2"),
                );
                ui.add(
                    egui::Slider::new(&mut self.controls.added_torque_load_nm, 0.0..=40.0)
                        .text("Load")
                        .suffix(" Nm"),
                );
                ui.checkbox(&mut self.controls.spark_enabled, "Spark");
                ui.checkbox(&mut self.controls.fuel_enabled, "Fuel");
                ui.add(
                    egui::Slider::new(&mut self.controls.redline_spark_cut_rpm, 2000.0..=12000.0)
                        .text("Spark Cut")
                        .suffix(" rpm"),
                );
                ui.separator();
                ui.checkbox(&mut self.controls.dyno_mode_enabled, "Dyno Mode");
                ui.add(
                    egui::Slider::new(&mut self.controls.dyno_target_rpm, 500.0..=9000.0)
                        .text("Dyno Target")
                        .suffix(" rpm"),
                );
                ui.separator();
                ui.small("Throttle, idle leak, and lambda target are placeholders until intake and species models exist.");
            });

        egui::TopBottomPanel::top("summary")
            .resizable(false)
            .show(ctx, |ui| {
                ui.horizontal_wrapped(|ui| {
                    metric(
                        ui,
                        "RPM",
                        format!("{:.0}", self.latest_frame.engine_data.rpm),
                    );
                    metric(
                        ui,
                        "RPM Delta",
                        format!("{:.1}", self.latest_frame.engine_data.rpm_delta),
                    );
                    metric(
                        ui,
                        "Torque",
                        format!("{:.2} Nm", self.latest_frame.engine_data.torque_nm),
                    );
                    metric(
                        ui,
                        "Power",
                        format!("{:.2} kW", self.latest_frame.engine_data.power_kw),
                    );
                    metric(
                        ui,
                        "Fuel Flow",
                        format!(
                            "{:.1} mg/s",
                            self.latest_frame.engine_data.fuel_flow_mg_per_s
                        ),
                    );
                    metric(
                        ui,
                        "Air Flow",
                        format!("{:.2} g/s", self.latest_frame.engine_data.air_flow_g_per_s),
                    );
                    scalar_metric(
                        ui,
                        "Chamber Lambda",
                        self.latest_frame.engine_data.chamber_lambda,
                    );
                    scalar_metric(
                        ui,
                        "Exhaust Lambda",
                        self.latest_frame.engine_data.exhaust_lambda,
                    );
                    metric(
                        ui,
                        "Peak Temp",
                        format!(
                            "{:.0} K",
                            self.latest_frame.engine_data.peak_cylinder_temperature_k
                        ),
                    );
                    metric(
                        ui,
                        "Peak Pressure",
                        format!(
                            "{:.0} kPa",
                            self.latest_frame.engine_data.peak_cylinder_pressure_pa / 1000.0
                        ),
                    );
                    metric(
                        ui,
                        "MAP",
                        format!("{:.0} kPa", self.latest_frame.engine_data.map_pa / 1000.0),
                    );
                    metric(
                        ui,
                        "Combustion Ratio",
                        format!(
                            "{:.2}",
                            self.latest_frame.engine_data.combustion_event_ratio
                        ),
                    );
                });
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            egui::Grid::new("main_grid")
                .num_columns(2)
                .spacing([12.0, 12.0])
                .show(ui, |ui| {
                    angle_plot(
                        ui,
                        "Cylinder Pressure",
                        &self.latest_frame.angle_trace,
                        |trace| &trace.pressure_pa,
                        "Pa",
                    );
                    angle_plot(
                        ui,
                        "Cylinder Temperature",
                        &self.latest_frame.angle_trace,
                        |trace| &trace.temperature_k,
                        "K",
                    );
                    ui.end_row();
                    angle_plot(
                        ui,
                        "Valve Effective Area",
                        &self.latest_frame.angle_trace,
                        |trace| &trace.intake_effective_area_m2,
                        "m^2",
                    );
                    angle_plot(
                        ui,
                        "Chamber Air Mass",
                        &self.latest_frame.angle_trace,
                        |trace| &trace.chamber_air_mass_g,
                        "g",
                    );
                    ui.end_row();
                    time_plot(
                        ui,
                        "RPM Over Time",
                        &self.latest_frame.time_history,
                        |point| [point.time_seconds, point.rpm],
                    );
                    time_plot(
                        ui,
                        "Torque Over Time",
                        &self.latest_frame.time_history,
                        |point| [point.time_seconds, point.torque_nm],
                    );
                    ui.end_row();
                    time_plot(
                        ui,
                        "MAP Over Time",
                        &self.latest_frame.time_history,
                        |point| [point.time_seconds, point.map_pa / 1000.0],
                    );
                    rpm_history_plot(ui, "Torque/Power vs RPM", &self.latest_frame);
                });
        });

        ctx.request_repaint();
    }
}

fn metric(ui: &mut egui::Ui, label: &str, value: String) {
    ui.group(|ui| {
        ui.label(RichText::new(label).small());
        ui.label(RichText::new(value).strong());
    });
}

fn scalar_metric(ui: &mut egui::Ui, label: &str, scalar: enginesim::telemetry::TelemetryScalar) {
    let suffix = match scalar.availability {
        TelemetryAvailability::Measured => "",
        TelemetryAvailability::Placeholder => " placeholder",
        TelemetryAvailability::Unavailable => " unavailable",
    };
    let text = scalar
        .value
        .map(|value| format!("{value:.2}{suffix}"))
        .unwrap_or_else(|| suffix.trim().to_string());
    metric(ui, label, text);
}

fn angle_plot(
    ui: &mut egui::Ui,
    title: &str,
    trace: &AngleTraceSnapshot,
    select: impl Fn(&AngleTraceSnapshot) -> &Vec<[f64; 2]>,
    y_label: &str,
) {
    let points = PlotPoints::from(select(trace).clone());
    Plot::new(title)
        .view_aspect(1.8)
        .height(220.0)
        .x_axis_label("Crank Angle (deg)")
        .y_axis_label(y_label)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new(title, points));
            if title == "Valve Effective Area" {
                let exhaust = PlotPoints::from(trace.exhaust_effective_area_m2.clone());
                plot_ui.line(Line::new("Exhaust", exhaust).color(Color32::LIGHT_RED));
            }
            if title == "Chamber Air Mass" {
                let fuel = PlotPoints::from(trace.fuel_mass_placeholder_mg.clone());
                plot_ui.line(Line::new("Fuel placeholder", fuel).color(Color32::LIGHT_YELLOW));
            }
        });
}

fn time_plot(
    ui: &mut egui::Ui,
    title: &str,
    history: &[enginesim::telemetry::TimePlotPoint],
    map: impl Fn(&enginesim::telemetry::TimePlotPoint) -> [f64; 2],
) {
    let points: Vec<[f64; 2]> = history.iter().map(map).collect();
    Plot::new(title)
        .view_aspect(1.8)
        .height(220.0)
        .show(ui, |plot_ui| {
            plot_ui.line(Line::new(title, PlotPoints::from(points)));
        });
}

fn rpm_history_plot(ui: &mut egui::Ui, title: &str, telemetry: &EngineFrameTelemetry) {
    Plot::new(title)
        .view_aspect(1.8)
        .height(220.0)
        .x_axis_label("RPM")
        .show(ui, |plot_ui| {
            for point in &telemetry.rpm_history {
                let fade = (255_i32 - (point.age_index as i32 * 18)).clamp(48, 255) as u8;
                plot_ui.points(
                    Points::new(
                        "Torque",
                        PlotPoints::from(vec![[point.rpm, point.torque_nm]]),
                    )
                    .radius(3.0)
                    .color(Color32::from_rgba_unmultiplied(100, 200, 255, fade)),
                );
                plot_ui.points(
                    Points::new("Power", PlotPoints::from(vec![[point.rpm, point.power_kw]]))
                        .radius(3.0)
                        .color(Color32::from_rgba_unmultiplied(255, 180, 80, fade)),
                );
            }
        });
}
