use crate::physics::chamber::{
    ChamberBoundary, ChamberProperties, ChamberSpeciesMasses, DRY_AIR_INERT_MASS_FRACTION,
    DRY_AIR_OXYGEN_MASS_FRACTION, chamber_pressure_pa, mixture_chamber_properties,
};
use crate::physics::flow::{
    FlowEndpoint, FlowOrifice, GasFlowProperties, bidirectional_isentropic_mass_flow,
};
use crate::physics::gas::{cp, cv};
use crate::throttle;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeCellGeometry {
    pub length_m: f64,
    pub area_m2: f64,
}

impl PipeCellGeometry {
    pub fn volume_m3(self) -> f64 {
        self.length_m * self.area_m2
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeCellState {
    pub mass_kg: f64,
    pub momentum_kg_m_per_s: f64,
    pub total_energy_j: f64,
    pub species: ChamberSpeciesMasses,
}

impl PipeCellState {
    pub fn from_pressure_temperature_velocity(
        pressure_pa: f64,
        temperature_k: f64,
        velocity_m_per_s: f64,
        geometry: PipeCellGeometry,
        properties: GasFlowProperties,
    ) -> Self {
        assert!(pressure_pa > 0.0, "pipe pressure must be positive");
        assert!(temperature_k > 0.0, "pipe temperature must be positive");
        assert!(geometry.length_m > 0.0, "pipe cell length must be positive");
        assert!(geometry.area_m2 > 0.0, "pipe cell area must be positive");
        assert!(
            properties.gas_constant_j_per_kg_k > 0.0 && properties.specific_heat_ratio > 1.0,
            "pipe gas properties must be valid"
        );

        let volume_m3 = geometry.volume_m3();
        let mass_kg =
            pressure_pa * volume_m3 / (properties.gas_constant_j_per_kg_k * temperature_k);
        let momentum_kg_m_per_s = mass_kg * velocity_m_per_s;
        let internal_energy_j = mass_kg
            * cv(
                properties.gas_constant_j_per_kg_k,
                properties.specific_heat_ratio,
            )
            * temperature_k;
        let kinetic_energy_j = 0.5 * mass_kg * velocity_m_per_s * velocity_m_per_s;

        Self {
            mass_kg,
            momentum_kg_m_per_s,
            total_energy_j: internal_energy_j + kinetic_energy_j,
            species: ChamberSpeciesMasses::from_dry_air_mass(mass_kg),
        }
    }

    pub fn density_kg_per_m3(self, geometry: PipeCellGeometry) -> f64 {
        if geometry.volume_m3() <= 0.0 {
            return 0.0;
        }

        self.mass_kg.max(0.0) / geometry.volume_m3()
    }

    pub fn velocity_m_per_s(self) -> f64 {
        if self.mass_kg <= 0.0 {
            return 0.0;
        }

        self.momentum_kg_m_per_s / self.mass_kg
    }

    pub fn pressure_pa(self, geometry: PipeCellGeometry, properties: GasFlowProperties) -> f64 {
        let volume_m3 = geometry.volume_m3();
        if self.mass_kg <= 0.0 || volume_m3 <= 0.0 || properties.specific_heat_ratio <= 1.0 {
            return 0.0;
        }

        let velocity_m_per_s = self.velocity_m_per_s();
        let kinetic_energy_j = 0.5 * self.mass_kg * velocity_m_per_s * velocity_m_per_s;
        let internal_energy_density_j_per_m3 =
            (self.total_energy_j - kinetic_energy_j).max(0.0) / volume_m3;
        (properties.specific_heat_ratio - 1.0) * internal_energy_density_j_per_m3
    }

    pub fn temperature_k(self, geometry: PipeCellGeometry, properties: GasFlowProperties) -> f64 {
        let pressure_pa = self.pressure_pa(geometry, properties);
        if pressure_pa <= 0.0 || self.mass_kg <= 0.0 {
            return 0.0;
        }

        pressure_pa * geometry.volume_m3() / (self.mass_kg * properties.gas_constant_j_per_kg_k)
    }

    pub fn sound_speed_m_per_s(
        self,
        geometry: PipeCellGeometry,
        properties: GasFlowProperties,
    ) -> f64 {
        let pressure_pa = self.pressure_pa(geometry, properties);
        let density_kg_per_m3 = self.density_kg_per_m3(geometry);
        if pressure_pa <= 0.0 || density_kg_per_m3 <= 0.0 || properties.specific_heat_ratio <= 1.0 {
            return 0.0;
        }

        (properties.specific_heat_ratio * pressure_pa / density_kg_per_m3).sqrt()
    }

    fn conserved_density(self, geometry: PipeCellGeometry) -> ConservedDensity {
        let volume_m3 = geometry.volume_m3();
        ConservedDensity {
            mass_kg_per_m3: self.mass_kg / volume_m3,
            momentum_kg_per_m2_s: self.momentum_kg_m_per_s / volume_m3,
            energy_j_per_m3: self.total_energy_j / volume_m3,
        }
    }

    fn species_fraction(self) -> ChamberSpeciesMasses {
        if self.mass_kg <= 0.0 {
            return ChamberSpeciesMasses::default();
        }

        ChamberSpeciesMasses {
            oxygen_kg: self.species.oxygen_kg / self.mass_kg,
            fuel_kg: self.species.fuel_kg / self.mass_kg,
            inert_kg: self.species.inert_kg / self.mass_kg,
            products_kg: self.species.products_kg / self.mass_kg,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeBoundaryFlux {
    pub mass_kg_per_s: f64,
    pub momentum_n: f64,
    pub energy_w: f64,
    pub species_kg_per_s: ChamberSpeciesMasses,
}

impl PipeBoundaryFlux {
    pub fn zero() -> Self {
        Self {
            mass_kg_per_s: 0.0,
            momentum_n: 0.0,
            energy_w: 0.0,
            species_kg_per_s: ChamberSpeciesMasses::default(),
        }
    }

    pub fn scaled(self, scale: f64) -> Self {
        Self {
            mass_kg_per_s: self.mass_kg_per_s * scale,
            momentum_n: self.momentum_n * scale,
            energy_w: self.energy_w * scale,
            species_kg_per_s: ChamberSpeciesMasses {
                oxygen_kg: self.species_kg_per_s.oxygen_kg * scale,
                fuel_kg: self.species_kg_per_s.fuel_kg * scale,
                inert_kg: self.species_kg_per_s.inert_kg * scale,
                products_kg: self.species_kg_per_s.products_kg * scale,
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PipeBoundary {
    Closed,
    OpenPressure {
        pressure_pa: f64,
        temperature_k: f64,
    },
}

#[derive(Debug, Clone, PartialEq)]
pub struct Pipe1D {
    pub cells: Vec<PipeCellState>,
    pub geometry: PipeCellGeometry,
    pub properties: GasFlowProperties,
}

impl Pipe1D {
    pub fn uniform(
        cell_count: usize,
        geometry: PipeCellGeometry,
        properties: GasFlowProperties,
        pressure_pa: f64,
        temperature_k: f64,
    ) -> Self {
        assert!(cell_count > 0, "pipe must have at least one cell");
        let cell = PipeCellState::from_pressure_temperature_velocity(
            pressure_pa,
            temperature_k,
            0.0,
            geometry,
            properties,
        );

        Self {
            cells: vec![cell; cell_count],
            geometry,
            properties,
        }
    }

    pub fn max_stable_timestep_seconds(&self, cfl_number: f64) -> f64 {
        max_stable_timestep_seconds(&self.cells, self.geometry, self.properties, cfl_number)
    }

    pub fn step(
        &mut self,
        left_boundary: PipeBoundary,
        right_boundary: PipeBoundary,
        timestep_seconds: f64,
    ) -> PipeStepBoundaryFluxes {
        step_pipe_cells_with_boundaries(
            &mut self.cells,
            self.geometry,
            self.properties,
            left_boundary,
            right_boundary,
            timestep_seconds,
        )
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PipeStepBoundaryFluxes {
    pub left: PipeBoundaryFlux,
    pub right: PipeBoundaryFlux,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlenumState {
    pub volume_m3: f64,
    pub chamber_state: crate::physics::chamber::ChamberState,
    pub species: ChamberSpeciesMasses,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ThrottlePlenumInput {
    pub throttle_position: f64,
    pub minimum_area_m2: f64,
    pub maximum_added_area_m2: f64,
    pub maximum_throttle_position: f64,
    pub discharge_coefficient: f64,
    pub upstream_pressure_pa: f64,
    pub upstream_temperature_k: f64,
}

impl PlenumState {
    pub fn from_pressure_temperature(
        pressure_pa: f64,
        temperature_k: f64,
        volume_m3: f64,
        properties: ChamberProperties,
    ) -> Self {
        assert!(pressure_pa > 0.0, "plenum pressure must be positive");
        assert!(temperature_k > 0.0, "plenum temperature must be positive");
        assert!(volume_m3 > 0.0, "plenum volume must be positive");
        let mass_kg =
            pressure_pa * volume_m3 / (properties.gas_constant_j_per_kg_k * temperature_k);

        Self {
            volume_m3,
            chamber_state: crate::physics::chamber::ChamberState {
                mass_kg,
                temperature_k,
            },
            species: ChamberSpeciesMasses::from_dry_air_mass(mass_kg),
        }
    }

    pub fn pressure_pa(&self, fallback: ChamberProperties) -> f64 {
        chamber_pressure_pa(
            self.chamber_state,
            ChamberBoundary {
                volume_m3: self.volume_m3,
                volume_rate_m3_per_s: 0.0,
                heat_rate_w: 0.0,
            },
            mixture_chamber_properties(self.species, fallback),
        )
    }

    pub fn step_throttle(
        &mut self,
        input: ThrottlePlenumInput,
        fallback: ChamberProperties,
        timestep_seconds: f64,
    ) -> f64 {
        let properties = mixture_chamber_properties(self.species, fallback);
        let pressure_pa = self.pressure_pa(fallback);
        let effective_area_m2 = throttle::effective_area_m2(
            input.minimum_area_m2,
            input.maximum_added_area_m2,
            input.throttle_position,
            input.maximum_throttle_position,
        );
        let mass_flow_kg_per_s = bidirectional_isentropic_mass_flow(
            FlowEndpoint {
                pressure_pa: input.upstream_pressure_pa,
                temperature_k: input.upstream_temperature_k,
            },
            FlowEndpoint {
                pressure_pa,
                temperature_k: self.chamber_state.temperature_k,
            },
            FlowOrifice {
                discharge_coefficient: input.discharge_coefficient,
                area_m2: effective_area_m2,
            },
            GasFlowProperties {
                specific_heat_ratio: properties.specific_heat_ratio,
                gas_constant_j_per_kg_k: properties.gas_constant_j_per_kg_k,
            },
        );

        let cv_j_per_kg_k = cv(
            properties.gas_constant_j_per_kg_k,
            properties.specific_heat_ratio,
        );
        let cp_j_per_kg_k = cp(
            properties.gas_constant_j_per_kg_k,
            properties.specific_heat_ratio,
        );
        let mass_delta_kg = mass_flow_kg_per_s * timestep_seconds;
        let internal_energy_j =
            self.chamber_state.mass_kg * cv_j_per_kg_k * self.chamber_state.temperature_k;
        let updated_internal_energy_j = if mass_flow_kg_per_s >= 0.0 {
            internal_energy_j
                + mass_flow_kg_per_s
                    * cp_j_per_kg_k
                    * input.upstream_temperature_k
                    * timestep_seconds
        } else {
            internal_energy_j
                + mass_flow_kg_per_s
                    * cp_j_per_kg_k
                    * self.chamber_state.temperature_k
                    * timestep_seconds
        };
        let updated_mass_kg = (self.chamber_state.mass_kg + mass_delta_kg).max(0.0);
        self.chamber_state.mass_kg = updated_mass_kg;
        self.chamber_state.temperature_k = if updated_mass_kg > 0.0 && cv_j_per_kg_k > 0.0 {
            (updated_internal_energy_j / (updated_mass_kg * cv_j_per_kg_k))
                .max(fallback.minimum_temperature_k)
        } else {
            fallback.minimum_temperature_k
        };
        if mass_flow_kg_per_s >= 0.0 {
            self.species.add_dry_air(mass_delta_kg);
        } else {
            self.species.remove_proportional(-mass_delta_kg);
        }
        self.chamber_state.mass_kg = self.species.total_mass_kg();

        mass_flow_kg_per_s
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct ConservedDensity {
    mass_kg_per_m3: f64,
    momentum_kg_per_m2_s: f64,
    energy_j_per_m3: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
struct EulerFluxDensity {
    mass_kg_per_m2_s: f64,
    momentum_n_per_m2: f64,
    energy_w_per_m2: f64,
}

pub fn max_stable_timestep_seconds(
    cells: &[PipeCellState],
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
    cfl_number: f64,
) -> f64 {
    if cells.is_empty() || geometry.length_m <= 0.0 || cfl_number <= 0.0 {
        return 0.0;
    }

    let max_signal_speed = cells
        .iter()
        .map(|cell| cell.velocity_m_per_s().abs() + cell.sound_speed_m_per_s(geometry, properties))
        .fold(0.0, f64::max);
    if max_signal_speed <= 0.0 {
        return f64::INFINITY;
    }

    cfl_number * geometry.length_m / max_signal_speed
}

pub fn assert_cfl_stable(
    cells: &[PipeCellState],
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
    timestep_seconds: f64,
    cfl_number: f64,
) {
    let max_timestep_seconds = max_stable_timestep_seconds(cells, geometry, properties, cfl_number);
    assert!(
        timestep_seconds <= max_timestep_seconds,
        "pipe timestep {timestep_seconds} exceeds CFL limit {max_timestep_seconds}"
    );
}

pub fn rusanov_flux(
    left: PipeCellState,
    right: PipeCellState,
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
) -> PipeBoundaryFlux {
    let left_u = left.conserved_density(geometry);
    let right_u = right.conserved_density(geometry);
    let left_flux = physical_flux_density(left, geometry, properties);
    let right_flux = physical_flux_density(right, geometry, properties);
    let max_signal_speed = (left.velocity_m_per_s().abs()
        + left.sound_speed_m_per_s(geometry, properties))
    .max(right.velocity_m_per_s().abs() + right.sound_speed_m_per_s(geometry, properties));
    let area_m2 = geometry.area_m2;

    let mass_flux_density = 0.5 * (left_flux.mass_kg_per_m2_s + right_flux.mass_kg_per_m2_s)
        - 0.5 * max_signal_speed * (right_u.mass_kg_per_m3 - left_u.mass_kg_per_m3);
    let momentum_flux_density = 0.5 * (left_flux.momentum_n_per_m2 + right_flux.momentum_n_per_m2)
        - 0.5 * max_signal_speed * (right_u.momentum_kg_per_m2_s - left_u.momentum_kg_per_m2_s);
    let energy_flux_density = 0.5 * (left_flux.energy_w_per_m2 + right_flux.energy_w_per_m2)
        - 0.5 * max_signal_speed * (right_u.energy_j_per_m3 - left_u.energy_j_per_m3);
    let species_fraction = if mass_flux_density >= 0.0 {
        left.species_fraction()
    } else {
        right.species_fraction()
    };
    let mass_kg_per_s = mass_flux_density * area_m2;

    PipeBoundaryFlux {
        mass_kg_per_s,
        momentum_n: momentum_flux_density * area_m2,
        energy_w: energy_flux_density * area_m2,
        species_kg_per_s: ChamberSpeciesMasses {
            oxygen_kg: species_fraction.oxygen_kg * mass_kg_per_s,
            fuel_kg: species_fraction.fuel_kg * mass_kg_per_s,
            inert_kg: species_fraction.inert_kg * mass_kg_per_s,
            products_kg: species_fraction.products_kg * mass_kg_per_s,
        },
    }
}

pub fn step_pipe_cells(
    cells: &mut [PipeCellState],
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
    timestep_seconds: f64,
) {
    assert_cfl_stable(cells, geometry, properties, timestep_seconds, 1.0);
    if cells.len() < 2 || timestep_seconds <= 0.0 {
        return;
    }

    let mut fluxes = Vec::with_capacity(cells.len() - 1);
    for pair in cells.windows(2) {
        fluxes.push(rusanov_flux(pair[0], pair[1], geometry, properties));
    }

    for (face_index, flux) in fluxes.into_iter().enumerate() {
        apply_flux(&mut cells[face_index], flux, -timestep_seconds);
        apply_flux(&mut cells[face_index + 1], flux, timestep_seconds);
    }
}

pub fn step_pipe_cells_with_boundaries(
    cells: &mut [PipeCellState],
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
    left_boundary: PipeBoundary,
    right_boundary: PipeBoundary,
    timestep_seconds: f64,
) -> PipeStepBoundaryFluxes {
    assert_cfl_stable(cells, geometry, properties, timestep_seconds, 1.0);
    if cells.is_empty() || timestep_seconds <= 0.0 {
        return PipeStepBoundaryFluxes {
            left: PipeBoundaryFlux::zero(),
            right: PipeBoundaryFlux::zero(),
        };
    }

    let left_ghost = boundary_ghost_cell(left_boundary, cells[0], geometry, properties);
    let right_ghost =
        boundary_ghost_cell(right_boundary, cells[cells.len() - 1], geometry, properties);
    let left_flux = rusanov_flux(left_ghost, cells[0], geometry, properties);
    let right_flux = rusanov_flux(cells[cells.len() - 1], right_ghost, geometry, properties);

    step_pipe_cells(cells, geometry, properties, timestep_seconds);
    apply_flux(&mut cells[0], left_flux, timestep_seconds);
    let last_index = cells.len() - 1;
    apply_flux(&mut cells[last_index], right_flux, -timestep_seconds);

    PipeStepBoundaryFluxes {
        left: left_flux,
        right: right_flux,
    }
}

pub fn closed_end_ghost_cell(cell: PipeCellState) -> PipeCellState {
    PipeCellState {
        momentum_kg_m_per_s: -cell.momentum_kg_m_per_s,
        ..cell
    }
}

pub fn open_pressure_ghost_cell(
    pressure_pa: f64,
    temperature_k: f64,
    adjacent_velocity_m_per_s: f64,
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
) -> PipeCellState {
    PipeCellState::from_pressure_temperature_velocity(
        pressure_pa,
        temperature_k,
        adjacent_velocity_m_per_s,
        geometry,
        properties,
    )
}

pub fn pipe_cell_to_chamber_orifice_flux(
    pipe_cell: PipeCellState,
    chamber_pressure_pa: f64,
    chamber_temperature_k: f64,
    pipe_geometry: PipeCellGeometry,
    valve_area_m2: f64,
    properties: GasFlowProperties,
) -> PipeBoundaryFlux {
    if valve_area_m2 <= 0.0 {
        return PipeBoundaryFlux::zero();
    }

    let flux_geometry = PipeCellGeometry {
        length_m: pipe_geometry.length_m,
        area_m2: valve_area_m2.min(pipe_geometry.area_m2),
    };
    let pipe_flux_cell =
        resize_cell_to_geometry(pipe_cell, pipe_geometry, flux_geometry, properties);
    let chamber_cell = PipeCellState::from_pressure_temperature_velocity(
        chamber_pressure_pa,
        chamber_temperature_k,
        0.0,
        flux_geometry,
        properties,
    );
    rusanov_flux(pipe_flux_cell, chamber_cell, flux_geometry, properties)
}

pub fn apply_boundary_flux_to_cell(
    cell: &mut PipeCellState,
    flux_into_cell: PipeBoundaryFlux,
    timestep_seconds: f64,
) {
    let timestep_seconds =
        limited_boundary_timestep(cell.mass_kg, flux_into_cell, timestep_seconds);
    apply_flux(cell, flux_into_cell, timestep_seconds);
}

fn physical_flux_density(
    cell: PipeCellState,
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
) -> EulerFluxDensity {
    let density_kg_per_m3 = cell.density_kg_per_m3(geometry);
    let velocity_m_per_s = cell.velocity_m_per_s();
    let pressure_pa = cell.pressure_pa(geometry, properties);
    let energy_density_j_per_m3 = cell.total_energy_j / geometry.volume_m3();

    EulerFluxDensity {
        mass_kg_per_m2_s: density_kg_per_m3 * velocity_m_per_s,
        momentum_n_per_m2: density_kg_per_m3 * velocity_m_per_s * velocity_m_per_s + pressure_pa,
        energy_w_per_m2: (energy_density_j_per_m3 + pressure_pa) * velocity_m_per_s,
    }
}

fn boundary_ghost_cell(
    boundary: PipeBoundary,
    adjacent: PipeCellState,
    geometry: PipeCellGeometry,
    properties: GasFlowProperties,
) -> PipeCellState {
    match boundary {
        PipeBoundary::Closed => closed_end_ghost_cell(adjacent),
        PipeBoundary::OpenPressure {
            pressure_pa,
            temperature_k,
        } => open_pressure_ghost_cell(
            pressure_pa,
            temperature_k,
            adjacent.velocity_m_per_s(),
            geometry,
            properties,
        ),
    }
}

fn resize_cell_to_geometry(
    cell: PipeCellState,
    source_geometry: PipeCellGeometry,
    target_geometry: PipeCellGeometry,
    properties: GasFlowProperties,
) -> PipeCellState {
    let mut resized = PipeCellState::from_pressure_temperature_velocity(
        cell.pressure_pa(source_geometry, properties).max(1.0),
        cell.temperature_k(source_geometry, properties).max(1.0),
        cell.velocity_m_per_s(),
        target_geometry,
        properties,
    );
    let fractions = cell.species_fraction();
    resized.species = ChamberSpeciesMasses {
        oxygen_kg: fractions.oxygen_kg * resized.mass_kg,
        fuel_kg: fractions.fuel_kg * resized.mass_kg,
        inert_kg: fractions.inert_kg * resized.mass_kg,
        products_kg: fractions.products_kg * resized.mass_kg,
    };
    resized
}

fn apply_flux(cell: &mut PipeCellState, flux: PipeBoundaryFlux, signed_timestep_seconds: f64) {
    cell.mass_kg += flux.mass_kg_per_s * signed_timestep_seconds;
    cell.momentum_kg_m_per_s += flux.momentum_n * signed_timestep_seconds;
    cell.total_energy_j += flux.energy_w * signed_timestep_seconds;
    cell.species.oxygen_kg += flux.species_kg_per_s.oxygen_kg * signed_timestep_seconds;
    cell.species.fuel_kg += flux.species_kg_per_s.fuel_kg * signed_timestep_seconds;
    cell.species.inert_kg += flux.species_kg_per_s.inert_kg * signed_timestep_seconds;
    cell.species.products_kg += flux.species_kg_per_s.products_kg * signed_timestep_seconds;
    normalize_cell(cell);
}

fn limited_boundary_timestep(
    available_mass_kg: f64,
    flux_into_cell: PipeBoundaryFlux,
    timestep_seconds: f64,
) -> f64 {
    if timestep_seconds <= 0.0 || flux_into_cell.mass_kg_per_s >= 0.0 {
        return timestep_seconds;
    }

    let requested_removal_kg = -flux_into_cell.mass_kg_per_s * timestep_seconds;
    let removable_mass_kg = available_mass_kg.max(0.0);
    if requested_removal_kg <= removable_mass_kg || requested_removal_kg <= 0.0 {
        timestep_seconds
    } else {
        timestep_seconds * removable_mass_kg / requested_removal_kg
    }
}

fn normalize_cell(cell: &mut PipeCellState) {
    cell.mass_kg = cell.mass_kg.max(0.0);
    cell.total_energy_j = cell.total_energy_j.max(0.0);
    cell.species = cell.species.normalized();
    let species_mass_kg = cell.species.total_mass_kg();
    if cell.mass_kg > 0.0 && species_mass_kg > 0.0 {
        let correction = cell.mass_kg / species_mass_kg;
        cell.species.oxygen_kg *= correction;
        cell.species.fuel_kg *= correction;
        cell.species.inert_kg *= correction;
        cell.species.products_kg *= correction;
    } else {
        cell.species = ChamberSpeciesMasses {
            oxygen_kg: cell.mass_kg * DRY_AIR_OXYGEN_MASS_FRACTION,
            fuel_kg: 0.0,
            inert_kg: cell.mass_kg * DRY_AIR_INERT_MASS_FRACTION,
            products_kg: 0.0,
        };
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1.0e-9;

    fn properties() -> GasFlowProperties {
        GasFlowProperties {
            specific_heat_ratio: 1.4,
            gas_constant_j_per_kg_k: 287.0,
        }
    }

    fn geometry() -> PipeCellGeometry {
        PipeCellGeometry {
            length_m: 0.01,
            area_m2: 0.001,
        }
    }

    fn cell(pressure_pa: f64, velocity_m_per_s: f64) -> PipeCellState {
        PipeCellState::from_pressure_temperature_velocity(
            pressure_pa,
            300.0,
            velocity_m_per_s,
            geometry(),
            properties(),
        )
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        let difference = (actual - expected).abs();
        assert!(
            difference <= tolerance,
            "expected {expected}, got {actual}; difference {difference} exceeded {tolerance}",
        );
    }

    #[test]
    fn cell_state_round_trips_pressure_temperature_and_velocity() {
        let state = cell(101_325.0, 25.0);

        assert_close(
            state.pressure_pa(geometry(), properties()),
            101_325.0,
            EPSILON,
        );
        assert_close(
            state.temperature_k(geometry(), properties()),
            300.0,
            EPSILON,
        );
        assert_close(state.velocity_m_per_s(), 25.0, EPSILON);
        assert_close(state.species.total_mass_kg(), state.mass_kg, EPSILON);
    }

    #[test]
    fn equal_states_have_only_pressure_momentum_flux() {
        let state = cell(101_325.0, 0.0);
        let flux = rusanov_flux(state, state, geometry(), properties());

        assert_close(flux.mass_kg_per_s, 0.0, EPSILON);
        assert_close(flux.energy_w, 0.0, EPSILON);
        assert_close(flux.momentum_n, 101_325.0 * geometry().area_m2, EPSILON);
    }

    #[test]
    fn pressure_gradient_drives_mass_flux_to_lower_pressure() {
        let high = cell(120_000.0, 0.0);
        let low = cell(100_000.0, 0.0);

        let forward = rusanov_flux(high, low, geometry(), properties());
        let reverse = rusanov_flux(low, high, geometry(), properties());

        assert!(forward.mass_kg_per_s > 0.0);
        assert!(reverse.mass_kg_per_s < 0.0);
    }

    #[test]
    fn cell_pair_step_conserves_mass_species_and_energy() {
        let mut cells = [cell(120_000.0, 0.0), cell(100_000.0, 0.0)];
        let initial_mass: f64 = cells.iter().map(|cell| cell.mass_kg).sum();
        let initial_energy: f64 = cells.iter().map(|cell| cell.total_energy_j).sum();
        let initial_species: f64 = cells.iter().map(|cell| cell.species.total_mass_kg()).sum();

        step_pipe_cells(&mut cells, geometry(), properties(), 1.0e-6);

        let final_mass: f64 = cells.iter().map(|cell| cell.mass_kg).sum();
        let final_energy: f64 = cells.iter().map(|cell| cell.total_energy_j).sum();
        let final_species: f64 = cells.iter().map(|cell| cell.species.total_mass_kg()).sum();
        assert_close(final_mass, initial_mass, 1.0e-12);
        assert_close(final_energy, initial_energy, 1.0e-9);
        assert_close(final_species, initial_species, 1.0e-12);
    }

    #[test]
    fn cfl_limit_uses_velocity_plus_sound_speed() {
        let state = cell(101_325.0, 10.0);
        let sound_speed = state.sound_speed_m_per_s(geometry(), properties());
        let expected = 0.5 * geometry().length_m / (sound_speed + 10.0);

        assert_close(
            max_stable_timestep_seconds(&[state], geometry(), properties(), 0.5),
            expected,
            1.0e-12,
        );
    }

    #[test]
    #[should_panic(expected = "exceeds CFL limit")]
    fn cfl_assertion_rejects_unstable_timestep() {
        let state = cell(101_325.0, 0.0);

        assert_cfl_stable(&[state], geometry(), properties(), 1.0, 1.0);
    }

    #[test]
    fn closed_end_ghost_cell_reflects_velocity() {
        let state = cell(101_325.0, 40.0);
        let ghost = closed_end_ghost_cell(state);

        assert_close(ghost.velocity_m_per_s(), -40.0, EPSILON);
        assert_close(
            ghost.pressure_pa(geometry(), properties()),
            101_325.0,
            EPSILON,
        );
    }

    #[test]
    fn open_pressure_ghost_cell_sets_boundary_pressure() {
        let ghost = open_pressure_ghost_cell(90_000.0, 310.0, 12.0, geometry(), properties());

        assert_close(
            ghost.pressure_pa(geometry(), properties()),
            90_000.0,
            EPSILON,
        );
        assert_close(
            ghost.temperature_k(geometry(), properties()),
            310.0,
            EPSILON,
        );
        assert_close(ghost.velocity_m_per_s(), 12.0, EPSILON);
    }

    #[test]
    fn pressure_wave_moves_about_one_sound_speed_timestep() {
        let mut cells = [
            cell(102_000.0, 0.0),
            cell(102_000.0, 0.0),
            cell(102_000.0, 0.0),
            cell(100_000.0, 0.0),
            cell(100_000.0, 0.0),
            cell(100_000.0, 0.0),
        ];
        let timestep_seconds = max_stable_timestep_seconds(&cells, geometry(), properties(), 0.5);

        step_pipe_cells(&mut cells, geometry(), properties(), timestep_seconds);

        assert!(cells[2].pressure_pa(geometry(), properties()) < 102_000.0);
        assert!(cells[3].pressure_pa(geometry(), properties()) > 100_000.0);
    }

    #[test]
    fn pressure_wave_response_converges_with_mesh_refinement() {
        let coarse = pressure_step_center_pressure_after_time(6, 0.06, 5.0e-5);
        let fine = pressure_step_center_pressure_after_time(12, 0.06, 5.0e-5);
        let initial_high = 102_000.0;
        let initial_low = 100_000.0;

        assert!(coarse > initial_low && coarse < initial_high);
        assert!(fine > initial_low && fine < initial_high);
        assert!((fine - coarse).abs() < 1_000.0);
    }

    #[test]
    fn chamber_boundary_flux_conserves_species_fraction_from_upstream_pipe() {
        let mut pipe = cell(120_000.0, 0.0);
        pipe.species.fuel_kg = pipe.mass_kg * 0.02;
        pipe.species.oxygen_kg = pipe.mass_kg * 0.20;
        pipe.species.inert_kg = pipe.mass_kg * 0.78;
        pipe.species.products_kg = 0.0;
        let flux = pipe_cell_to_chamber_orifice_flux(
            pipe,
            100_000.0,
            300.0,
            geometry(),
            geometry().area_m2,
            properties(),
        );

        assert!(flux.mass_kg_per_s > 0.0);
        assert_close(
            flux.species_kg_per_s.fuel_kg / flux.mass_kg_per_s,
            0.02,
            1.0e-12,
        );
    }

    #[test]
    fn boundary_step_reflects_from_closed_end() {
        let mut pipe = Pipe1D {
            cells: vec![cell(101_325.0, -50.0)],
            geometry: geometry(),
            properties: properties(),
        };

        pipe.step(PipeBoundary::Closed, PipeBoundary::Closed, 1.0e-6);

        assert!(pipe.cells[0].velocity_m_per_s() > -50.0);
    }

    #[test]
    fn open_boundary_drives_pipe_toward_external_pressure() {
        let mut pipe = Pipe1D::uniform(3, geometry(), properties(), 100_000.0, 300.0);

        let fluxes = pipe.step(
            PipeBoundary::OpenPressure {
                pressure_pa: 120_000.0,
                temperature_k: 300.0,
            },
            PipeBoundary::OpenPressure {
                pressure_pa: 100_000.0,
                temperature_k: 300.0,
            },
            1.0e-6,
        );

        assert!(fluxes.left.mass_kg_per_s > 0.0);
        assert!(pipe.cells[0].pressure_pa(geometry(), properties()) > 100_000.0);
    }

    #[test]
    fn pipe_to_chamber_orifice_flux_respects_valve_area_and_reversal() {
        let pipe = cell(120_000.0, 0.0);
        let open = pipe_cell_to_chamber_orifice_flux(
            pipe,
            100_000.0,
            300.0,
            geometry(),
            geometry().area_m2 * 0.5,
            properties(),
        );
        let closed = pipe_cell_to_chamber_orifice_flux(
            pipe,
            100_000.0,
            300.0,
            geometry(),
            0.0,
            properties(),
        );
        let reverse = pipe_cell_to_chamber_orifice_flux(
            cell(90_000.0, 0.0),
            110_000.0,
            300.0,
            geometry(),
            geometry().area_m2 * 0.5,
            properties(),
        );

        assert!(open.mass_kg_per_s > 0.0);
        assert_eq!(closed, PipeBoundaryFlux::zero());
        assert!(reverse.mass_kg_per_s < 0.0);
    }

    #[test]
    fn pipe_to_chamber_orifice_flux_handles_empty_pipe_cell() {
        let empty_cell = PipeCellState {
            mass_kg: 0.0,
            momentum_kg_m_per_s: 0.0,
            total_energy_j: 0.0,
            species: ChamberSpeciesMasses::default(),
        };

        let flux = pipe_cell_to_chamber_orifice_flux(
            empty_cell,
            101_325.0,
            300.0,
            geometry(),
            geometry().area_m2,
            properties(),
        );

        assert!(flux.mass_kg_per_s.is_finite());
        assert!(flux.energy_w.is_finite());
    }

    #[test]
    fn boundary_flux_cannot_remove_more_than_cell_mass() {
        let mut state = cell(101_325.0, 0.0);
        let initial_mass_kg = state.mass_kg;
        let initial_energy_j = state.total_energy_j;
        let initial_species = state.species;

        apply_boundary_flux_to_cell(
            &mut state,
            PipeBoundaryFlux {
                mass_kg_per_s: -initial_mass_kg * 10.0,
                momentum_n: 0.0,
                energy_w: -initial_energy_j * 10.0,
                species_kg_per_s: ChamberSpeciesMasses {
                    oxygen_kg: -initial_species.oxygen_kg * 10.0,
                    fuel_kg: -initial_species.fuel_kg * 10.0,
                    inert_kg: -initial_species.inert_kg * 10.0,
                    products_kg: -initial_species.products_kg * 10.0,
                },
            },
            1.0,
        );

        assert_close(state.mass_kg, 0.0, EPSILON);
        assert_close(state.total_energy_j, 0.0, EPSILON);
        assert_close(state.species.total_mass_kg(), 0.0, EPSILON);
    }

    #[test]
    fn plenum_throttle_flow_raises_pressure_when_upstream_is_higher() {
        let fallback = ChamberProperties {
            gas_constant_j_per_kg_k: 287.0,
            specific_heat_ratio: 1.4,
            minimum_mass_kg: 1.0e-9,
            minimum_temperature_k: 1.0,
        };
        let mut plenum = PlenumState::from_pressure_temperature(90_000.0, 300.0, 0.002, fallback);
        let initial_pressure = plenum.pressure_pa(fallback);

        let flow = plenum.step_throttle(
            ThrottlePlenumInput {
                throttle_position: 1.0,
                minimum_area_m2: 0.0,
                maximum_added_area_m2: 0.0002,
                maximum_throttle_position: 1.0,
                discharge_coefficient: 0.7,
                upstream_pressure_pa: 101_325.0,
                upstream_temperature_k: 300.0,
            },
            fallback,
            1.0e-4,
        );

        assert!(flow > 0.0);
        assert!(plenum.pressure_pa(fallback) > initial_pressure);
    }

    fn pressure_step_center_pressure_after_time(
        cell_count: usize,
        total_length_m: f64,
        elapsed_seconds: f64,
    ) -> f64 {
        let geometry = PipeCellGeometry {
            length_m: total_length_m / cell_count as f64,
            area_m2: 0.001,
        };
        let mut pipe = Pipe1D {
            cells: (0..cell_count)
                .map(|index| {
                    let pressure_pa = if index < cell_count / 2 {
                        102_000.0
                    } else {
                        100_000.0
                    };
                    PipeCellState::from_pressure_temperature_velocity(
                        pressure_pa,
                        300.0,
                        0.0,
                        geometry,
                        properties(),
                    )
                })
                .collect(),
            geometry,
            properties: properties(),
        };
        let mut remaining_seconds = elapsed_seconds;
        while remaining_seconds > f64::EPSILON {
            let step_seconds = pipe.max_stable_timestep_seconds(0.5).min(remaining_seconds);
            pipe.step(PipeBoundary::Closed, PipeBoundary::Closed, step_seconds);
            remaining_seconds -= step_seconds;
        }

        pipe.cells[cell_count / 2].pressure_pa(geometry, properties())
    }
}
