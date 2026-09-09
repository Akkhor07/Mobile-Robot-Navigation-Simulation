# Mobile Robot Navigation Simulation (Theta* + APF + Pure Pursuit + LIDAR + Occupancy Mapping)

This is my honest, from-scratch reconstruction of the algorithms named in the
product listing you pasted ("Advanced Mobile Robot Navigation Simulation |
Theta*, APF, Pure Pursuit, LIDAR & SLAM", $9). I do not have the seller's
actual source code, so this is not a copy of it — it's a working
implementation of the same named techniques, built from the text description
and the thumbnail visible in your screen recording.

## How to run
1. Open `robot_navigation_sim.m` in MATLAB.
2. Keep all the other `.m` files in the same folder (they're the "modular"
   function files the main script calls — see file list below).
3. Press Run. A 2×2 animated figure opens and, by default, an MP4 is written
   next to the script (`recordVideo = true` near the top of the script).

No add-on toolboxes are required — only base MATLAB (`inpolygon`, `conv2`,
`VideoWriter`, `imagesc`, etc.).

## What's actually implemented
- **Theta\* global planner** on an inflated occupancy grid, with a proper
  any-angle line-of-sight check (Bresenham + corner-cut prevention), not just
  plain A*.
- **Path smoothing** by arc-length resampling with `pchip` interpolation.
- **Simulated 360° LIDAR** via ray-marching against the true occupancy grid
  (72 beams, 2.6 m range).
- **Pure-pursuit-style path tracking** (lookahead point + heading controller)
  **blended with APF repulsion** from live LIDAR hits, so the robot swerves
  around obstacles it wasn't told about in advance and returns to the planned
  path once clear.
- **Adaptive speed**: slows for sharp turns and for close obstacles.
- **Live occupancy-grid mapping** from LIDAR sweeps (simplified log-odds
  counter), shown as a grey/black/white map panel.
- **Stall detection and Theta\* re-planning**: if the robot makes no real
  progress for ~2.5 s, it backs up, turns, and re-plans from where it is.
- **Multi-panel live animation** (global view, zoomed view, occupancy map,
  speed/distance plot) and MP4 recording, plus a small 4-wheel schematic
  robot icon (not a full vehicle-dynamics model).

## What I verified, and how (please read this part)
I don't have MATLAB in the environment I built this in, so I could not run
it in MATLAB itself. What I *did* do: installed GNU Octave (a free,
mostly-MATLAB-compatible interpreter) and actually executed the full
simulation end to end, headless, several times. It ran without errors,
correctly planned a path around all 8 obstacles, avoided every one of them
live via LIDAR+APF, mapped the environment, triggered one automatic
re-plan, and reached the goal with **zero collisions and zero near-collision
events** in that run — details printed by the script itself:

```
Result: GOAL REACHED at t = 23.40 s
Path length traveled: 10.16 m
Re-plans triggered:   1
Steps with LIDAR reading < robot radius + 5cm (near-collision): 0 / 234
Steps where robot center was inside a true obstacle cell (collision): 0 / 234
```

A snapshot of the final animation frame from that Octave run is included
(`preview_frame.png`) so you can see the actual output — global path view,
zoomed LIDAR view, live occupancy map, and the stats plot — before you run
anything yourself.

Two MATLAB features Octave doesn't implement (`VideoWriter`, and one
`drawnow` variant I already replaced with plain `drawnow` for portability)
mean the Octave test ran with video recording turned off. `VideoWriter` is
completely standard, stable MATLAB — I'm not worried about that call itself,
just flagging that I couldn't test *that specific line* end-to-end myself.

**Bottom line: I'm confident this runs and does what it claims, because I
watched it do exactly that — but I built and tested it in Octave, not
MATLAB, so please still run it yourself before relying on it (e.g. for a
submission deadline) in case of small MATLAB-vs-Octave rendering
differences.**

## Where this is a simplification, not the same as the real thing
Being asked for "no exaggeration," here's what's genuinely simplified
compared to what "SLAM," a "4-wheel robot," etc. can imply:

- **"SLAM" = occupancy mapping only, with a known pose.** The robot's
  position used for mapping and control is read directly from the
  simulation's ground truth. Real SLAM additionally *estimates* the robot's
  own pose from noisy sensor data (EKF-SLAM, particle filters, scan
  matching, etc.) — that estimation step is not implemented. This is a very
  common simplification in low-cost demo projects, but it is a
  simplification.
- **The robot is a kinematic unicycle model** (x, y, heading), drawn with a
  4-wheel schematic overlay. It is not a multi-body model with wheel slip,
  suspension, or torque limits.
- **The planner re-plans against the full ground-truth map**, not a
  frontier-limited "what the robot has seen so far" map. A stricter
  from-scratch SLAM-based replanner would only route through cells it has
  actually observed.
- **Theta\*'s open-list search is a simple O(n) linear-scan implementation**,
  not a binary-heap priority queue. It's fast enough for the grid sizes used
  here (80×80 cells) but would need a heap for much larger maps or
  real-time replanning at high frequency.
- Obstacle shapes (circle/rectangle/triangle/diamond) are hand-placed to
  resemble your thumbnail, not identical to the original layout, since I
  can't see the seller's exact coordinates.

## File list
| File | Purpose |
|---|---|
| `robot_navigation_sim.m` | Main script — parameters, environment build, main simulation loop, figure/video setup |
| `define_obstacles.m` | Hand-placed obstacle layout (circles, rects, polygons) |
| `build_environment.m` | Rasterizes obstacles into a ground-truth occupancy grid |
| `inflate_occ.m` | Dilates obstacles by robot radius for safe planning |
| `theta_star.m` | Any-angle Theta\* path planner |
| `line_of_sight.m` | Bresenham line-of-sight check used by Theta\* |
| `heuristic.m` | Euclidean distance heuristic |
| `resample_path.m` | Arc-length resampling / smoothing of the planned path |
| `world_to_grid.m` / `grid_to_world.m` | Coordinate conversions |
| `lidar_scan.m` | Simulated 360° LIDAR ray casting |
| `update_belief.m` | Occupancy-grid mapping update from a LIDAR scan |
| `prob_from_logodds.m` | Converts the belief grid to a 0–1 image for display |
| `find_lookahead_point.m` | Pure-pursuit lookahead point search |
| `apf_repulsion.m` | Artificial Potential Field repulsive force from LIDAR hits |
| `wrap_to_pi.m` | Angle wrapping helper |
| `robot_shape_points.m` / `wheel_shape_points.m` | 4-wheel robot schematic geometry for plotting |
| `preview_frame.png` | Screenshot of the final animation frame from my own Octave test run |

Tweak `startPose`, `goalPos`, `obstacles` (in `define_obstacles.m`), or the
tuning parameters at the top of `robot_navigation_sim.m` (lookahead
distance, APF gains, LIDAR range/beam count, etc.) to change the scenario.
