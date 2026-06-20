# Wiebe Burn Model

The Wiebe model should be used as the crank-angle burn-shape model inside the 0D chamber. It should not be treated as the whole combustion model. The chamber must still conserve species, consume oxygen and fuel, generate combustion products, release heat, apply heat losses and update mixture properties.

## Role In The Current Plan

For the first single-cylinder vertical slice, Wiebe can drive the fraction of burnable fuel that has released heat by crank angle. The surrounding combustion model must decide:

- whether the mixture is combustible,
- how much fuel is burnable from available fuel and oxygen,
- how much combustion efficiency to apply,
- how much heat is lost to the chamber walls,
- how much unburned fuel remains after incomplete combustion or misfire,
- how generated combustion products change chamber composition.

The recommended split is:

- `x_b(theta)` describes normalized cumulative burn shape.
- `eta_comb` describes combustion efficiency and incomplete burn.
- species conversion handles fuel, oxygen, inert gas and products.
- chamber energy update receives heat release after losses.
- ignition delay determines when combustion starts after spark.
- burn duration determines how quickly the burn progresses after start of combustion.

## Cumulative Burn Fraction

The standard cumulative Wiebe burn fraction is:

```text
x_b(theta) = 1 - exp[-a * ((theta - theta_0) / delta_theta)^(m + 1)]
```

Where:

- `theta` is current crank angle.
- `theta_0` is start of combustion.
- `delta_theta` is combustion duration.
- `a` is the Wiebe efficiency coefficient.
- `m` is the shape factor.

Do not force `x_b` to exactly `1.0` at `delta_theta` unless the model explicitly chooses to normalize the curve. With `a = 5`, the standard expression reaches about 99.3 percent at the duration boundary.

## Event Timing

Spark timing and start of combustion are not the same event. A simple starting model is:

```text
theta_soc = theta_spark + ignition_delay(p_spark, T_spark, phi, residual_fraction)
```

The state used for ignition delay must be the physically correct chamber state at spark timing, not a stale state from before intake, exhaust or injection events in the same step.

The current single-cylinder implementation uses chamber pressure, chamber temperature, equivalence ratio and residual fraction to decide whether an event starts. Fueling at spark is based on trapped air mass and the requested lambda target, and the mixture limits suppress combustion outside the combustible range.

Ignition delay should eventually depend on:

- unburned-gas pressure near spark,
- unburned-gas temperature near spark,
- equivalence ratio,
- residual/EGR fraction,
- spark energy,
- turbulence/flame-kernel conditions.

For the current 0D implementation, use the chamber pressure and chamber temperature at spark timing.

A practical state-dependent structure is:

```text
ignition_delay =
    ignition_delay_ref
    * pressure_factor
    * temperature_factor
    * equivalence_ratio_factor
    * residual_fraction_factor
    * spark_energy_factor
    * turbulence_factor
```

Typical ignition-delay behavior:

| Variable | Expected effect on ignition delay |
|---|---|
| Higher pressure | Shorter delay |
| Higher temperature | Shorter delay |
| Near-stoichiometric mixture | Shorter delay |
| Lean mixture | Longer delay |
| High residual/EGR fraction | Longer delay |
| Higher turbulence | Usually shorter delay |

A simple first approximation may use a fixed delay, such as 5 to 15 degCA, but the API should leave room for state-dependent delay.

## Combustion Duration

Combustion duration should eventually be computed from in-cylinder state and operating condition. A practical structure is:

```text
delta_theta =
    delta_theta_ref
    * speed_factor
    * pressure_factor
    * temperature_factor
    * equivalence_ratio_factor
    * residual_fraction_factor
    * turbulence_factor
```

For spark-ignition combustion, duration is related to turbulent flame speed. Faster burn means shorter `delta_theta`.

Combustion duration should eventually depend on:

- engine speed,
- pressure and temperature at ignition or early burn,
- equivalence ratio,
- residual/EGR fraction,
- turbulence intensity,
- chamber geometry or characteristic flame travel distance,
- spark timing through its effect on pressure, temperature and turbulence.

Reasonable starting values if duration means start-of-combustion to about CA90:

| Operating condition | Duration |
|---|---:|
| Fast burn, high load, near stoich | 20 to 35 degCA |
| Normal stoichiometric SI | 30 to 50 degCA |
| Lean, high residual, or low load | 45 to 80 degCA |
| Very slow or unstable combustion | 70 to 100 degCA |

## Equivalence Ratio And Mixture Limits

Equivalence ratio is:

```text
phi = AFR_stoich / AFR_actual
```

Spark-ignition combustion is usually fastest near stoichiometric to slightly rich mixtures, often around `phi = 1.0` to `1.1`.

Example duration multipliers:

| Mixture | phi | Burn-duration multiplier |
|---|---:|---:|
| Slightly rich / fastest | 1.05 to 1.10 | 0.9 |
| Stoich | 1.0 | 1.0 |
| Mild lean | 0.9 | 1.1 to 1.3 |
| Lean | 0.8 | 1.4 to 2.0 |
| Very lean | below 0.75 | misfire or partial burn likely |

Early combustible bounds may use approximate AFR limits, but future species-aware combustion should make the oxygen limit explicit instead of relying only on air/fuel ratio.
phi above and including 1.96 should not combust at all
phi below and including 0.563 should not combust at all

## Combustion Efficiency And Incomplete Burn

Incomplete combustion should be modeled separately from the Wiebe curve shape:

```text
Q_released = eta_comb * m_fuel_burnable * LHV * delta_x_b
```

Where `eta_comb` may depend on:

- equivalence ratio,
- oxygen availability,
- residual/EGR fraction,
- pressure and temperature,
- wall quench,
- misfire,
- cyclic variability,
- turbulence or flame speed quality.

This split keeps `x_b` responsible for timing/shape and `eta_comb` responsible for how much of the available chemical energy is actually released.

Misfire and partial burn behavior should leave physically meaningful remaining species:

- unburned fuel can remain in the chamber or leave through exhaust flow,
- unused oxygen should remain available to the chamber state,
- products should only be generated by burned fuel,
- heat release should correspond to actual burned fuel mass.

## Avoiding Double Counting

Do not apply the same physical effect in multiple places. For example:

- If pressure and temperature affect ignition delay, avoid also applying the same correction twice to start-of-combustion timing.
- If equivalence ratio affects both burn duration and combustion efficiency, use each correction for a distinct purpose: duration changes timing, efficiency changes released energy.
- If spark timing changes pressure and temperature at ignition, avoid adding a separate spark-timing multiplier unless it represents a separate calibrated effect.
- If turbulence is introduced through a 1D/flow or chamber turbulence estimate, use that same signal consistently for delay/duration rather than adding an unrelated speed-only correction.

## Suggested Starting Parameters

For a basic spark-ignition 0D model:

- `a = 5`
- `m = 2`
- fixed ignition delay initially, then state-dependent delay later
- duration from a fixed calibration initially, then state-dependent duration later

Incomplete combustion should be modeled through combustion efficiency and species conversion, not by distorting the Wiebe burn shape unless that is an intentional calibration choice.

## Integration Requirements

- Evaluate heat release over the crank-angle interval being stepped, not only at the step start or end.
- Use the delta in cumulative burned fraction for each step.
- Keep burn fraction monotonic during a combustion event.
- Stop the event when elapsed crank angle reaches the configured duration or the remaining burnable fuel is exhausted.
- Do not start a new combustion event from stale AFR, pressure, temperature or residual-fraction state.
- Keep all crank-angle values in radians internally.

## Testing Requirements

Unit tests should cover:

- `x_b(0) = 0`
- monotonic burn fraction over the event,
- `x_b(delta_theta)` equals the standard expression value unless normalization is explicitly enabled,
- shape-factor and `a` parameter effects,
- no burn when duration, coefficient or burnable fuel is invalid,
- correct stepwise burned-fuel delta from cumulative burn fraction.

Integration tests should cover:

- heat release raises pressure at fixed volume,
- fired cycle produces positive indicated work with sane timing,
- overly lean/rich or oxygen-limited cases produce partial burn or misfire behavior,
- incomplete combustion leaves appropriate unburned fuel and oxygen/products state,
- changing duration or start-of-combustion timing changes pressure trace and torque in the expected direction.
