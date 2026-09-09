# Mobile Robot Navigation Simulation (Theta\* + APF + Pure Pursuit + LIDAR + Occupancy Mapping)

## How to run

1. Open `robot\\\_navigation\\\_sim.m` in MATLAB.
2. Keep all the other `.m` files in the same folder (they're the "modular"
function files the main script calls — see file list below).
3. Press Run. A 2×2 animated figure opens and, by default, an MP4 is written
next to the script (`recordVideo = true` near the top of the script).

No add-on toolboxes are required — only base MATLAB (`inpolygon`, `conv2`,
`VideoWriter`, `imagesc`, etc.).

## What's actually implemented

* **Theta\* global planner** on an inflated occupancy grid, with a proper
any-angle line-of-sight check (Bresenham + corner-cut prevention), not just
plain A\*.
* **Path smoothing** by arc-length resampling with `pchip` interpolation.
* **Simulated 360° LIDAR** via ray-marching against the true occupancy grid
(72 beams, 2.6 m range).
* **Pure-pursuit-style path tracking** (lookahead point + heading controller)
**blended with APF repulsion** from live LIDAR hits, so the robot swerves
around obstacles it wasn't told about in advance and returns to the planned
path once clear.
* **Adaptive speed**: slows for sharp turns and for close obstacles.
* **Live occupancy-grid mapping** from LIDAR sweeps (simplified log-odds
counter), shown as a grey/black/white map panel.
* **Stall detection and Theta\* re-planning**: if the robot makes no real
progress for \~2.5 s, it backs up, turns, and re-plans from where it is.
* **Multi-panel live animation** (global view, zoomed view, occupancy map,
speed/distance plot) and MP4 recording, plus a small 4-wheel schematic
robot icon (not a full vehicle-dynamics model).

## File list

|File|Purpose|
|-|-|
|`robot\\\_navigation\\\_sim.m`|Main script — parameters, environment build, main simulation loop, figure/video setup|
|`define\\\_obstacles.m`|Hand-placed obstacle layout (circles, rects, polygons)|
|`build\\\_environment.m`|Rasterizes obstacles into a ground-truth occupancy grid|
|`inflate\\\_occ.m`|Dilates obstacles by robot radius for safe planning|
|`theta\\\_star.m`|Any-angle Theta\* path planner|
|`line\\\_of\\\_sight.m`|Bresenham line-of-sight check used by Theta\*|
|`heuristic.m`|Euclidean distance heuristic|
|`resample\\\_path.m`|Arc-length resampling / smoothing of the planned path|
|`world\\\_to\\\_grid.m` / `grid\\\_to\\\_world.m`|Coordinate conversions|
|`lidar\\\_scan.m`|Simulated 360° LIDAR ray casting|
|`update\\\_belief.m`|Occupancy-grid mapping update from a LIDAR scan|
|`prob\\\_from\\\_logodds.m`|Converts the belief grid to a 0–1 image for display|
|`find\\\_lookahead\\\_point.m`|Pure-pursuit lookahead point search|
|`apf\\\_repulsion.m`|Artificial Potential Field repulsive force from LIDAR hits|
|`wrap\\\_to\\\_pi.m`|Angle wrapping helper|
|`robot\\\_shape\\\_points.m` / `wheel\\\_shape\\\_points.m`|4-wheel robot schematic geometry for plotting|
|`preview\\\_frame.png`|Screenshot of the final animation frame from my own Octave test run|

Tweak `startPose`, `goalPos`, `obstacles` (in `define\\\_obstacles.m`), or the
tuning parameters at the top of `robot\\\_navigation\\\_sim.m` (lookahead
distance, APF gains, LIDAR range/beam count, etc.) to change the scenario.

