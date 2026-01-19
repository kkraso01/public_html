# Drone directories overview

## `drones/`
This folder contains the simulation building blocks and scenario-specific logic for the drone experiences:
- **Cave exploration (`drones/cave/`)** combines the physics engine, a cave environment, LIDAR sensing, and occupancy-grid mapping utilities for autonomous frontier selection and path planning.
- **Race demo (`drones/race/`)** pairs the physics engine with a generated race track, reference trajectories, and a HUD to showcase high-speed control loops.
- **Obstacle sandbox (`drones/obstacles/`)** provides draggable rigid-body obstacles, dynamic obstacle management, and a demo wrapper that uses the shared physics utilities for interactive experimentation.
- **Core physics (`drones/physics/`)** exposes shared drone dynamics models and parameters used by the above demos.

Each scenario exports an `init...Demo` helper that takes a DOM container and returns lifecycle controls (`pause`, `resume`, `restart`, etc.), enabling consistent orchestration from the UI layer.

## `drone-sim/`
The `drone-sim/ui` module hosts the browser integration. It wires the page DOM into the simulation demos, sets up visibility-based pausing, and manages toggles for each experience so the heavy simulations only run when the related section is on-screen.
