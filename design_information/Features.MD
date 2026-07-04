# Requested Features

Status note: the tuning GUI described below is implemented as `src/bin/tuning_gui.rs`, backed by `src/tuning.rs::TuningSession` (draft/committed config with "Write Changes"/"Undo Changes") and `src/engine_handling.rs::EngineHandlingDefinition` (the sim-handling JSON split from the engine JSON, including `set_points_rpm`). 1D pipe geometry uses `total_length_m`/`number_of_cells` (`src/engine_config.rs`). The requirements below are kept as the original spec/rationale rather than rewritten, since the implementation still tracks them closely; treat unchecked-sounding items (e.g. per-cylinder MAP override behavior at extreme values, full input-box coverage for every listed parameter) as worth spot-checking against the current GUI rather than assumed complete.

# Tuning GUI
This will be a GUI for tuning engine performance and various aspects of the engine.

## Changes needed to overall architecture
Significant changes will be needed so create a new git branch.
Also make sure you write tests that ensure current accuracy and behaviour is maintained! Analise in detail things that might be impacted by changes and write coverage tests for those impacts! 
Changes fall into two main catagories


**Changes needed to JSON configs**
I want properties that define the ENGINE to be in the engine json
I want properties that define the way the ENGINE IS HANDLED IN SIM to go in a new json file
For example, the max added inertia should go in the new file, the bore should go in the engine json.

**Changes needed to core code**
Basically I noticed that when the json is changed and I run the program again there's some compilation that is happening
but I want to be able to change things like intake runner length or intake runner grid size, or throttle size and so on quickly
Now real time changes while the sim running is ideal, but I don't want to go chasing real time changes if it means massively increased complexity or issues with the 1D side of things. If I need to compromise and we need to "reset" the sim to implement the changes that the user made in the Thuning GUI then that's fine, we can have a button that's like "Write Changes" and "Undo Changes", once write changes is pressed the changes are made and the sim resets. But I don't want to close the GUI and recompile, that's a goal failed state.

**1D Grid**
Instead of defining the pipes as `cell_length` and `cell_count`, I want to define it as `total_length` and `number_of_cells`

## Set points
Set points are rpm dyno points where the tuning will happen
Control: A slider that slides between the set points (ie snaps to them)
Set points defined in JSON
For GN250: 1000rpm, 2500rpm, 5000rpm

## Input boxes for tunable paramaters
- For each intake & exhaust pipe (runners and final pipe)
	- diameter in mm (convert to m^2)
	- length
	- number of cells
- Compression Ratio
- valve start times (from 0 to 720 absolute engine position)
- valve duration (from 0 to 720, relative and in addition to start time)
- RPM entry box (should support negative RPMS if the simulation does, test the sim first!)
- throttle body size
- intake plenum details
- wiebe paramaters
- other combustion parameters
- spark parameters and timing
 - low speed rpm
 - low speed advance (0 to 720)
 - high speed rpm
 - high speed advance (0 to 720) rpm
 - for angle entry: give a toggle for absolute engine position (default) , or relative to btdc (as in the conventional way)


I also want a direct MAP overide mode, where I can toggle a box and then force the MAP to be a specific value, support any positive value up to 2000 kPa. Defualt is atmospheric pressure.

## Data displayed
With respect to engine angle
 - Cylinder pressure
 - Cylinder temperature
 - Charge air vs Fuel x AFR

Data boxes (starred ones * should be more prominent):
 - Torque *
 - Power *
 - BMEP Est.
 - VE *
 - Chamber Lambda
 - Exhaust Lambda *
 - Peak Temp
 - Peak pressure
 - Combustion Ratio
 - MAP *

## Other controls
A throttle slider and input box, default WOT






