use std::path::PathBuf;

use eframe::egui::{self, Color32, DragValue, RichText};
use egui_plot::{Line, Plot, PlotPoints, Polygon, VLine};

use enginesim::engine_config::{EngineDefinition, ValveDefinition, ValveLiftProfileModel};
use enginesim::engine_handling::EngineHandlingDefinition;
use enginesim::engine_loader::{
    EngineCatalogEntry, LoadSource, default_engine_directory, default_engine_path,
    discover_engine_files, load_engine_and_handling, save_engine_and_handling,
};
use enginesim::tuning::TuningSession;
use enginesim::valve::{
    ValveEvent, normalized_segment_fractions, positive_cycle_delta_rad,
    segmented_cubic_lift_fraction,
};

const ENGINE_CYCLE_DEG: f64 = 720.0;
const OVERLAP_TDC_DEG: f64 = 360.0;

fn main() -> eframe::Result<()> {
    eframe::run_native(
        "Valve Lift Designer",
        eframe::NativeOptions::default(),
        Box::new(|_cc| Ok(Box::new(ValveLiftDesignerApp::new()))),
    )
}

struct ValveLiftDesignerApp {
    engine_path: PathBuf,
    engine_options: Vec<EngineCatalogEntry>,
    session: TuningSession,
    load_status: String,
}

fn bundled_definition() -> EngineDefinition {
    EngineDefinition::from_json_str(include_str!("../../data/engines/gn250.json"))
        .expect("bundled GN250 JSON should parse")
}

fn bundled_handling() -> EngineHandlingDefinition {
    EngineHandlingDefinition::from_json_str(include_str!("../../data/engines/gn250.handling.json"))
        .expect("bundled GN250 handling JSON should parse")
}

fn engine_options_or_default() -> Vec<EngineCatalogEntry> {
    match discover_engine_files(&default_engine_directory()) {
        Ok(entries) if !entries.is_empty() => entries,
        _ => vec![EngineCatalogEntry::from_path(default_engine_path())],
    }
}

fn initial_engine_path(options: &[EngineCatalogEntry]) -> PathBuf {
    let default_path = default_engine_path();
    options
        .iter()
        .find(|entry| entry.path == default_path)
        .or_else(|| options.first())
        .map(|entry| entry.path.clone())
        .unwrap_or(default_path)
}

fn describe_source(label: &str, source: &LoadSource) -> String {
    match source {
        LoadSource::Disk(path) => format!("{label}: {}", path.display()),
        LoadSource::BundledFallback { reason } => format!("{label}: bundled fallback ({reason})"),
    }
}

fn load_status(engine_source: &LoadSource, handling_source: &LoadSource) -> String {
    format!(
        "{} | {}",
        describe_source("engine", engine_source),
        describe_source("handling", handling_source)
    )
}

impl ValveLiftDesignerApp {
    fn new() -> Self {
        let engine_options = engine_options_or_default();
        let engine_path = initial_engine_path(&engine_options);
        let loaded =
            load_engine_and_handling(&engine_path, &bundled_definition(), &bundled_handling());
        let load_status = load_status(&loaded.engine_source, &loaded.handling_source);
        Self {
            engine_path,
            engine_options,
            session: TuningSession::new(loaded.definition, loaded.handling),
            load_status,
        }
    }

    fn load_engine(&mut self, engine_path: PathBuf) {
        let loaded =
            load_engine_and_handling(&engine_path, &bundled_definition(), &bundled_handling());
        self.engine_path = engine_path;
        self.load_status = load_status(&loaded.engine_source, &loaded.handling_source);
        self.session.replace(loaded.definition, loaded.handling);
    }

    fn selected_engine_label(&self) -> String {
        self.engine_options
            .iter()
            .find(|entry| entry.path == self.engine_path)
            .map(|entry| entry.label.clone())
            .unwrap_or_else(|| self.session.committed_engine().metadata.name.clone())
    }

    fn save_to_disk(&mut self) {
        match save_engine_and_handling(
            &self.engine_path,
            self.session.committed_engine(),
            self.session.committed_handling(),
        ) {
            Ok(()) => self.load_status = format!("saved to {}", self.engine_path.display()),
            Err(err) => self.load_status = format!("save failed: {err}"),
        }
    }
}

impl eframe::App for ValveLiftDesignerApp {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        egui::TopBottomPanel::top("top").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.heading("Valve Lift Designer");
                ui.separator();
                ui.label(self.selected_engine_label());
            });
            let dirty = self.session.is_dirty();
            ui.horizontal(|ui| {
                if ui
                    .add_enabled(dirty, egui::Button::new("Write Changes"))
                    .clicked()
                {
                    self.session.write_changes();
                }
                if ui
                    .add_enabled(dirty, egui::Button::new("Undo Changes"))
                    .clicked()
                {
                    self.session.undo_changes();
                }
                if ui
                    .button("Save to disk")
                    .on_hover_text("Writes the committed config; Write Changes first")
                    .clicked()
                {
                    self.save_to_disk();
                }
                ui.small(self.load_status.clone());
            });
        });

        egui::SidePanel::left("controls")
            .resizable(true)
            .default_width(360.0)
            .show(ctx, |ui| {
                let mut selected_path = self.engine_path.clone();
                ui.label("Engine");
                egui::ComboBox::from_id_salt("engine_select")
                    .selected_text(self.selected_engine_label())
                    .show_ui(ui, |ui| {
                        for entry in &self.engine_options {
                            ui.selectable_value(
                                &mut selected_path,
                                entry.path.clone(),
                                &entry.label,
                            );
                        }
                    });
                if selected_path != self.engine_path && !self.session.is_dirty() {
                    self.load_engine(selected_path);
                } else if selected_path != self.engine_path {
                    ui.colored_label(
                        Color32::YELLOW,
                        "Write or undo changes before switching engines.",
                    );
                }

                ui.separator();
                let draft = &mut self.session.draft_engine;
                valve_designer_controls(ui, "Intake Valve", &mut draft.valves.intake);
                ui.separator();
                valve_designer_controls(ui, "Exhaust Valve", &mut draft.valves.exhaust);
            });

        egui::CentralPanel::default().show(ctx, |ui| {
            let definition = &self.session.draft_engine;
            ui.label(RichText::new(&definition.metadata.name).strong());
            draw_valve_plot(ui, definition);
            draw_overlap_plot(ui, definition);
            if self.session.is_dirty() {
                ui.colored_label(
                    Color32::YELLOW,
                    "Draft differs from committed config. Write Changes applies it to this session; Save to disk persists it.",
                );
            }
        });
    }
}

fn valve_designer_controls(ui: &mut egui::Ui, label: &str, valve: &mut ValveDefinition) {
    ui.label(RichText::new(label).strong());
    let mut segmented = valve.lift_profile.model == ValveLiftProfileModel::SegmentedCubic;
    if ui
        .checkbox(&mut segmented, "Use segmented cubic lift profile")
        .changed()
    {
        valve.lift_profile.model = if segmented {
            ValveLiftProfileModel::SegmentedCubic
        } else {
            ValveLiftProfileModel::LegacyCosine
        };
    }

    let total_duration_deg = valve_duration_deg(valve).max(1.0);
    let (ramp_fraction, main_fraction, dwell_fraction) =
        normalized_segment_fractions(valve.lift_profile);
    let mut ramp_deg = ramp_fraction * total_duration_deg;
    let mut main_deg = main_fraction * total_duration_deg;
    let mut dwell_deg = dwell_fraction * total_duration_deg;
    let mut ramp_lift_mm = valve.lift_profile.ramp_lift_fraction * valve.max_lift_m * 1000.0;
    let mut max_lift_mm = valve.max_lift_m * 1000.0;

    ui.horizontal(|ui| {
        ui.label("Open absolute");
        let mut open_deg = valve.open_angle_deg.rem_euclid(ENGINE_CYCLE_DEG);
        if ui
            .add(
                DragValue::new(&mut open_deg)
                    .speed(0.5)
                    .range(0.0..=ENGINE_CYCLE_DEG)
                    .suffix(" deg"),
            )
            .changed()
        {
            valve.open_angle_deg = open_deg.rem_euclid(ENGINE_CYCLE_DEG);
            valve.close_angle_deg =
                (valve.open_angle_deg + total_duration_deg).rem_euclid(ENGINE_CYCLE_DEG);
        }
    });

    ui.add_enabled_ui(segmented, |ui| {
        if labelled_drag(ui, "Ramp up/down", &mut ramp_deg, 0.5, 0.0..=180.0, " deg") {
            set_segment_degrees(valve, ramp_deg, main_deg, dwell_deg);
        }
        if labelled_drag(
            ui,
            "Main lift up/down",
            &mut main_deg,
            0.5,
            0.0..=240.0,
            " deg",
        ) {
            set_segment_degrees(valve, ramp_deg, main_deg, dwell_deg);
        }
        if labelled_drag(ui, "Dwell", &mut dwell_deg, 0.5, 0.0..=240.0, " deg") {
            set_segment_degrees(valve, ramp_deg, main_deg, dwell_deg);
        }
        if labelled_drag(
            ui,
            "Ramp lift",
            &mut ramp_lift_mm,
            0.01,
            0.0..=max_lift_mm.max(0.1),
            " mm",
        ) {
            valve.lift_profile.ramp_lift_fraction = if valve.max_lift_m > 0.0 {
                (ramp_lift_mm / (valve.max_lift_m * 1000.0)).clamp(0.0, 1.0)
            } else {
                0.0
            };
        }
    });
    if labelled_drag(ui, "Max lift", &mut max_lift_mm, 0.01, 0.1..=25.0, " mm") {
        valve.max_lift_m = max_lift_mm / 1000.0;
        valve.lift_profile.ramp_lift_fraction = if max_lift_mm > 0.0 {
            (ramp_lift_mm / max_lift_mm).clamp(0.0, 1.0)
        } else {
            0.0
        };
    }

    let duration_deg = valve_duration_deg(valve);
    valve.close_angle_deg = (valve.open_angle_deg + duration_deg).rem_euclid(ENGINE_CYCLE_DEG);
    ui.small(format!(
        "close abs {:.1} deg | duration {:.1} deg | internal open {:.4} rad",
        valve.close_angle_deg,
        duration_deg,
        absolute_deg_to_internal_rad(valve.open_angle_deg)
    ));
}

fn labelled_drag(
    ui: &mut egui::Ui,
    label: &str,
    value: &mut f64,
    speed: f64,
    range: std::ops::RangeInclusive<f64>,
    suffix: &str,
) -> bool {
    let mut changed = false;
    ui.horizontal(|ui| {
        ui.label(label);
        changed = ui
            .add(
                DragValue::new(value)
                    .speed(speed)
                    .range(range)
                    .suffix(suffix),
            )
            .changed();
    });
    changed
}

fn set_segment_degrees(valve: &mut ValveDefinition, ramp_deg: f64, main_deg: f64, dwell_deg: f64) {
    let ramp_deg = ramp_deg.max(0.0);
    let main_deg = main_deg.max(0.0);
    let dwell_deg = dwell_deg.max(0.0);
    let total_deg = (2.0 * ramp_deg + 2.0 * main_deg + dwell_deg).max(1.0);
    valve.lift_profile.ramp_duration_fraction = ramp_deg / total_deg;
    valve.lift_profile.main_lift_duration_fraction = main_deg / total_deg;
    valve.lift_profile.dwell_duration_fraction = dwell_deg / total_deg;
    valve.close_angle_deg = (valve.open_angle_deg + total_deg).rem_euclid(ENGINE_CYCLE_DEG);
}

fn valve_duration_deg(valve: &ValveDefinition) -> f64 {
    if valve.lift_profile.model == ValveLiftProfileModel::SegmentedCubic {
        let (ramp, main, dwell) = normalized_segment_fractions(valve.lift_profile);
        let current = (valve.close_angle_deg - valve.open_angle_deg).rem_euclid(ENGINE_CYCLE_DEG);
        let nominal = 2.0 * ramp + 2.0 * main + dwell;
        if nominal > 0.0 { current.max(1.0) } else { 1.0 }
    } else {
        (valve.close_angle_deg - valve.open_angle_deg).rem_euclid(ENGINE_CYCLE_DEG)
    }
}

fn absolute_deg_to_internal_rad(angle_deg: f64) -> f64 {
    angle_deg.rem_euclid(ENGINE_CYCLE_DEG).to_radians()
}

fn draw_valve_plot(ui: &mut egui::Ui, definition: &EngineDefinition) {
    ui.label(RichText::new("Valve lift over engine cycle").strong());
    let intake = lift_points(&definition.valves.intake, 0.0, ENGINE_CYCLE_DEG, 1.0);
    let exhaust = lift_points(&definition.valves.exhaust, 0.0, ENGINE_CYCLE_DEG, 1.0);
    Plot::new("valve_lift_plot")
        .height(340.0)
        .x_axis_label("Absolute crank angle (deg)")
        .y_axis_label("Valve lift (mm)")
        .default_x_bounds(0.0, ENGINE_CYCLE_DEG)
        .allow_drag(false)
        .allow_zoom(false)
        .show(ui, |plot_ui| {
            plot_ui.set_plot_bounds_x(0.0..=ENGINE_CYCLE_DEG);
            for &tdc in &[0.0, 360.0, 720.0] {
                plot_ui.vline(VLine::new("TDC", tdc).color(Color32::GRAY));
            }
            for &bdc in &[180.0, 540.0] {
                plot_ui.vline(VLine::new("BDC", bdc).color(Color32::DARK_GRAY));
            }
            plot_ui.line(Line::new("Intake", PlotPoints::from(intake)).color(Color32::LIGHT_BLUE));
            plot_ui.line(Line::new("Exhaust", PlotPoints::from(exhaust)).color(Color32::LIGHT_RED));
        });
}

fn draw_overlap_plot(ui: &mut egui::Ui, definition: &EngineDefinition) {
    ui.label(RichText::new("Valve overlap").strong());
    let start = 300.0;
    let end = 430.0;
    let intake = lift_points(&definition.valves.intake, start, end, 0.5);
    let exhaust = lift_points(&definition.valves.exhaust, start, end, 0.5);
    let overlap = overlap_polygon(
        &definition.valves.intake,
        &definition.valves.exhaust,
        start,
        end,
    );
    Plot::new("valve_overlap_plot")
        .height(260.0)
        .x_axis_label("Absolute crank angle (deg)")
        .y_axis_label("Valve lift (mm)")
        .default_x_bounds(start, end)
        .allow_drag(false)
        .allow_zoom(false)
        .show(ui, |plot_ui| {
            plot_ui.set_plot_bounds_x(start..=end);
            plot_ui.vline(VLine::new("Overlap TDC", OVERLAP_TDC_DEG).color(Color32::GRAY));
            if !overlap.is_empty() {
                plot_ui.polygon(
                    Polygon::new("Overlap period", PlotPoints::from(overlap))
                        .fill_color(Color32::from_rgba_unmultiplied(80, 180, 110, 70)),
                );
            }
            plot_ui.line(Line::new("Intake", PlotPoints::from(intake)).color(Color32::LIGHT_BLUE));
            plot_ui.line(Line::new("Exhaust", PlotPoints::from(exhaust)).color(Color32::LIGHT_RED));
        });
}

fn lift_points(
    valve: &ValveDefinition,
    start_deg: f64,
    end_deg: f64,
    step_deg: f64,
) -> Vec<[f64; 2]> {
    let event = ValveEvent::from_definition(*valve);
    let mut points = Vec::new();
    let mut angle = start_deg;
    while angle <= end_deg {
        let lift_mm = event.max_lift_m * 1000.0 * event.lift_fraction(angle.to_radians());
        points.push([angle, lift_mm]);
        angle += step_deg;
    }
    points
}

fn overlap_polygon(
    intake: &ValveDefinition,
    exhaust: &ValveDefinition,
    start_deg: f64,
    end_deg: f64,
) -> Vec<[f64; 2]> {
    let intake_event = ValveEvent::from_definition(*intake);
    let exhaust_event = ValveEvent::from_definition(*exhaust);
    let mut top = Vec::new();
    let mut bottom = Vec::new();
    let mut angle = start_deg;
    while angle <= end_deg {
        let intake_lift =
            intake_event.max_lift_m * 1000.0 * intake_event.lift_fraction(angle.to_radians());
        let exhaust_lift =
            exhaust_event.max_lift_m * 1000.0 * exhaust_event.lift_fraction(angle.to_radians());
        let shared = intake_lift.min(exhaust_lift);
        if shared > 0.0 {
            top.push([angle, shared]);
            bottom.push([angle, 0.0]);
        }
        angle += 0.5;
    }
    bottom.reverse();
    top.extend(bottom);
    top
}

#[allow(dead_code)]
fn segmented_lift_at_absolute_deg(valve: &ValveDefinition, angle_deg: f64) -> f64 {
    let open = absolute_deg_to_internal_rad(valve.open_angle_deg);
    let angle = absolute_deg_to_internal_rad(angle_deg);
    let duration =
        positive_cycle_delta_rad(open, absolute_deg_to_internal_rad(valve.close_angle_deg));
    if duration <= 0.0 {
        return 0.0;
    }
    let progress = positive_cycle_delta_rad(open, angle) / duration;
    segmented_cubic_lift_fraction(progress, valve.lift_profile) * valve.max_lift_m
}
