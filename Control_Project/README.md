# ARL Autonomous Vehicle Control Track

**Autotronics Research Lab (ARL) — Ain Shams University**  
*Course: Autonomous Vehicles & Drive-by-Wire Systems | Individual Project*

<p align="center">
  <img src="assets/demo.gif" alt="Autonomous Vehicle Simulation Demo" width="100%" />
</p>

---

## 🧠 What Is This Project About?

In this project you will build the control system for a self-driving car in a ROS 2 simulation. The car drives around a racetrack and your job is to make it stay on the path, control its speed, and complete laps as fast and accurately as possible.

You will work through a series of milestones, each building on the last:

1. **Explore the system** — Learn what topics the car publishes and subscribes to.
2. **Bring the car to life** — Implement the physics equations that describe how the car moves.
3. **Drive it manually** — Build a keyboard teleoperation node to drive the car yourself.
4. **Add cruise control** — Implement a PID speed controller so the car holds a steady speed.
5. **Make it autonomous** — Implement three different steering controllers (Lateral PID, Pure Pursuit, and MPC) so the car drives itself around the track.
6. **Monitor performance** — Build a lap analyzer that logs lap times, tracking error, and shows live graphs and 3D overlays in RViz.
7. **Report your results** — Compare your controllers and document your findings.

The car model is realistic: it has velocity as a state (not a direct input), meaning it accelerates and decelerates due to drag and friction — just like a real vehicle.



---



---

## 🚀 Quickstart

### 1. Build the Workspace
```bash
# Source ROS 2 Humble
source /opt/ros/humble/setup.bash

# Install build, simulation, and controller dependencies
sudo apt update && sudo apt install -y python3-colcon-common-extensions \
  python3-numpy python3-scipy ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-xacro

# Build the workspace (bicycle_sim, bicycle_control, track_environment)
cd /path/to/bicycle_gym-main
colcon build --symlink-install
source install/setup.bash
```

In every new terminal, source the ROS distribution and built workspace:

```bash
source /opt/ros/humble/setup.bash
cd /path/to/bicycle_gym-main
source install/setup.bash
```

### 2. Launch Modes

| Mode | Launch Command | Section |
|---|---|---|
| **Base Simulation (CLI Testing)** | `ros2 launch bicycle_sim bicycle_sim.launch.py` | Milestones 1 & 2 |
| **Interactive Keyboard Teleop** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` | Milestones 3 & 4 |
| **Lateral PID (Reactive)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` | Milestone 5.2 |
| **Pure Pursuit (Geometric Preview)**| `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit` | Milestone 5.3 |
| **Extended Kinematic MPC (Optimal Preview)** | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc` | Milestone 5.4 |

For keyboard teleoperation, start the keyboard driver in a second sourced terminal:

```bash
sudo apt install -y ros-humble-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

To request closed-loop cruise control in teleoperation mode:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
```

The launch file also supports `rviz:=false` and `analyzer:=false` to disable those nodes. For example:

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit rviz:=false
```

### 3. Inspecting the ROS Graph

Run these commands from a second sourced terminal while the simulation is running:

```bash
ros2 node list
ros2 topic list
ros2 topic info /state
ros2 topic info /throttle
ros2 topic info /steer
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_msgs/msg/Float32
ros2 interface show geometry_msgs/msg/Twist
ros2 topic echo /state
```

### 4. Direct Actuator Testing

With the base simulation running, send actuator commands from another sourced terminal. Throttle/brake uses `[-1.0, 1.0]`; steering is in radians, with positive values turning left.

```bash
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic pub --once /steer std_msgs/msg/Float32 "{data: 0.30}"
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: -1.0}"
```

### 5. Real-Time Telemetry & Graphing
```bash
# Install plotting and telemetry visualization tools
sudo apt update && sudo apt install -y ros-humble-plotjuggler-ros rqt-plot

# Inspect live signals in rqt_plot:
ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed

# Or launch PlotJuggler for multi-topic time-series analysis:
ros2 run plotjuggler plotjuggler
```

Some telemetry topics are only available after the corresponding analyzer work is complete.

---

## 🎯 Milestones at a Glance

- **Milestone 1**: Topic Discovery, Graph Inspection & Telemetry Plotting (`ros2 topic list / info`, `rqt_plot`, `plotjuggler`)
- **Milestone 2**: Extended Kinematic Bicycle Model & Euler Integration (`src/bicycle_sim/bicycle_sim/bicycle_model.py`)
- **Milestone 3**: Teleoperation Bridge & Open-Loop Driving (`src/bicycle_control/bicycle_control/teleop_bridge.py`)
- **Milestone 4**: Low-Level Powertrain Cruise Control (`src/bicycle_control/bicycle_control/longitudinal_pid.py`)
- **Milestone 5**: Autonomous Path Tracking — It's Time to Get the Car to Drive Autonomously!
  - **5.1**: High-Level Velocity Profiler & Path Curvature (`src/bicycle_control/bicycle_control/velocity_profiler.py`)
  - **5.2**: Steer Using Reactive Feedback (`src/bicycle_control/bicycle_control/lateral_pid.py`)
  - **5.3**: Steer Using Geometric Preview (`src/bicycle_control/bicycle_control/pure_pursuit.py`)
  - **5.4**: Steer Using Constrained Optimal Preview (Extended Kinematic MPC) (`src/bicycle_control/bicycle_control/mpc.py`)
  - **5.5**: Real-Time Telemetry, Graphing & RViz Dashboard Engineering (`src/track_environment/track_environment/lap_analyzer.py`)
- **Milestone 6**: Free Exploration & Reference Resources (Ackermann Kinematics, 3D Simulation, Nav2 MPPI)
- **Milestone 7**: Deliverable 1 — Repository Documentation (`README.md` Benchmark Report)
- **Milestone 8**: Deliverable 2 — Technical Video Walkthrough (3–5 Minute Demo)

---
## Milestone 6: Free Exploration & Reference Resources

To connect your work in this lab to industrial autonomous vehicle systems, modern simulators, and production ROS 2 frameworks, explore the following organized learning resources. These materials illustrate how the 2D planar kinematic bicycle model extends into multi-body dynamics, 3D physics engines, and advanced sampling-based predictive control.

---

### 1. Four-Wheel Ackermann Kinematics & `ros2_control`
*Explore multi-body steering geometry and industrial ROS 2 controller architectures.*

In a physical 4-wheel vehicle navigating a turn, the inside front wheel must turn sharper than the outside wheel because it follows a smaller turning radius ($R - W/2$ vs $R + W/2$). Forcing both wheels to the same angle causes tire scrub, tread wear, and energy loss.

$$\tan\delta_{inner} = \frac{L}{R - \frac{W}{2}}, \quad \tan\delta_{outer} = \frac{L}{R + \frac{W}{2}}$$

#### Curated Resources:
- [ROS 2 Control Mobile Robot Kinematics Guide](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html) — Guide on modeling 4-wheel kinematics and visualizing full car models instead of simplified bicycle models.
- [ROS 2 Steering Controllers Library](https://control.ros.org/kilted/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html) — Official documentation for Ackermann and bicycle steering controllers in `ros2_control`.
- [ros2_control_demos Example 11: Steered Wheel Base](https://control.ros.org/humble/doc/ros2_control_demos/example_11/doc/userdoc.html) — Industrial demonstration of steered-wheel bases and hardware interfaces.
- [ros2_control_demos Repository](https://github.com/ros-controls/ros2_control_demos) — Comprehensive reference suite for `ros2_control` implementations.
- [ROS 2 Controllers Official Repository](https://github.com/ros-controls/ros2_controllers/tree/master) — Upstream implementations of vehicle and chassis controllers.
- [Four-Wheel AMR Reference Implementation](https://github.com/abubakar-mughal97/four_wheel_amr) — 4-wheel mobile robot package with Ackermann steering.

---

### 2. Modern 3D Simulation Environments (Gazebo & MVSim)
*Bridge the gap between 2D planar kinematics and full 3D physics engines with tire friction dynamics.*

While kinematic models assume zero tire slip, physical vehicles experience tire deflection and friction saturation (Pacejka Magic Formula). 3D physics engines simulate suspension compliance, tire contact patches, sensor noise, and terrain.

#### Curated Resources:
- [Ackermann Vehicle in Modern Gazebo (Gz-Sim) & ROS 2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) ([Main Branch](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2/tree/main)) — Autonomous Ackermann vehicle simulation using modern Gazebo (Gz-Sim / Ignition) and ROS 2.
- [Classic Gazebo Ackermann Simulation](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle) — Classic Gazebo simulation showcasing physical Ackermann steering linkages.
- [Ackermann Autonomous Car Simulation](https://github.com/armando-genis/Ackermann-Autonomous-Car-Simulation) — Autonomous driving stack with Ackermann kinematics in simulation.
- [MVSim — Multi-Vehicle Simulator for ROS 2 Humble](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/MVSim/Simulation-MVSim.html) — Lightweight, fast multi-vehicle dynamic simulator tailored for mobile robots and autonomous vehicles.

---

### 3. Stochastic Sampling-Based Predictive Control (Nav2 MPPI)
*Explore model predictive path integral control for non-linear vehicle systems.*

Model Predictive Path Integral (MPPI) control is an advanced algorithm that generates thousands of randomized candidate trajectories in parallel (using GPU or multi-core CPU) and averages them using path integral weighting to produce optimal controls without needing gradient-based solvers.

#### Curated Resources:
- [Nav2 MPPI Controller](https://index.ros.org/p/nav2_mppi_controller/) — Production real-time MPPI controller package in the ROS 2 Navigation stack with dynamic obstacle avoidance and customizable cost functions.

---

### 💡 Synthesis Task for Your Report:
In your `README.md` report, synthesize your takeaways from exploring these organized resources:
1. **Kinematics vs Multi-Body**: How 4-wheel Ackermann kinematics accounts for differing inner and outer wheel turning radii ($\delta_{inner}$ vs $\delta_{outer}$), and how this is modeled in `ros2_control`.
2. **2D vs 3D Simulation**: The computational and modeling trade-offs between lightweight 2D kinematic simulation and full 3D physics engines (Gazebo / MVSim).
3. **Deterministic vs Sampling Control**: How modern sampling-based controllers (Nav2 MPPI) differ in flexibility, obstacle handling, and compute requirements compared to deterministic optimization (MPC).

---
## Student Information

| | |
|---|---|
| **Name** | Abdalrhman Ahmed Zakaria *(taken from the supplied study report; edit if needed)* |
| **Department** | Senior Mechatronics, Ain Shams University |
| **Course** | Autonomous Vehicles & Drive-by-Wire Systems (ARL) |

## System Architecture

```
 teleop_twist_keyboard -> /cmd_vel -> teleop_bridge --+
                                                       +--> /throttle, /steer --> kinematic_bicycle (sim_node)
 path_gen --> /path ---> controller (PID | PP | MPC) --+            |  -> /state (Odometry), /joint_states, TF map->base_link
          \-> /track_bounds (cones)    ^---------------- /state ----+
 /path + /state --> lap_analyzer --> /telemetry/{cte,speed,heading_err_deg,lap_time}, /lap/metrics (JSON), /lap/visualization
 robot_state_publisher <- /joint_states      RViz2: RobotModel, Path, /track_bounds, /lap/visualization
```

| Package | Contents |
|---|---|
| `bicycle_sim` | `bicycle_model.py` (plant), `sim_node.py`, URDF/Xacro, `bicycle.rviz`, launch files |
| `bicycle_control` | `teleop_bridge.py`, `longitudinal_pid.py`, `velocity_profiler.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `path_utils.py`, `controller_node.py` |
| `track_environment` | `track.py`, `path_gen.py`, `lap_analyzer.py`, track CSVs |

Topics: `/throttle` and `/steer` are `std_msgs/Float32`; `/state` is `nav_msgs/Odometry`; `/path` is `nav_msgs/Path`; `/lap/metrics` is a JSON `std_msgs/String`.

## Mathematical Formulations

**Plant (forward Euler, dt = 0.1 s, L = 1.25 m, state [x, y, θ, v] at the rear axle).**
ẋ = v cosθ, ẏ = v sinθ, θ̇ = (v/L) tanδ, v̇ = k_a·u − c_d v² − c_r v, with k_a = 4 m/s², c_d = 0.005, c_r = 0.05.
Each step: `x ← x + ẋ·dt`, `θ ← atan2(sinθ, cosθ)` (heading wrapping), `v ← clip(v, 0, v_max = 25)` (speed clamping). Steering is clamped to ±35°, throttle to [−1, 1].

**Teleop mapping.** `u = clip(linear.x / 5, −1, 1)`, `δ = clip(angular.z / 1.0 · 0.6109, ±0.6109)`; if no Twist arrives for 0.5 s, `u = δ = 0`. With cruise control, `linear.x` is a target speed fed to the speed PID.

**Longitudinal PID.** `u = clip(Kp e + Ki I + Kd ė, −1, 1)`, `I ← clip(I + e·dt, ±I_max)` (anti-windup clamp). Kp = 1.0, Ki = 0.1, Kd = 0, I_max = 2.

**Curvature and velocity profile.** Waypoints are non-uniformly spaced (3 mm to 0.96 m), so heading differences between neighbours are noise. Curvature is the Menger curvature of three points ±1.5 m of arc length around each waypoint: κ = 2(b−a)×(c−b) / (|b−a||c−b||c−a|), positive for left turns. Speed limit: `v = min(v_cruise, sqrt(a_lat / |κ|))` with a_lat = 5 m/s², evaluated as the worst κ over a preview window `max(3 m, 1.5 s·v)` so the car brakes before the corner.

**Lateral PID.** With cte > 0 meaning the car is left of the path and `e_ψ = ψ − ψ_path`: `δ = −(Kp·cte + Ki∫cte + Kd·d(cte)/dt) − K_ψ·e_ψ`, with Kp = 0.48, Kd = 0.15, K_ψ = 0.7. The negative signs make it negative feedback (left of path → steer right).

**Pure Pursuit.** Look-ahead `L_d = clip(L_d0 + k_v·v, L_min, L_max)` (0.8 + 0.5v, in [0.8, 3.5] m). The target is the intersection of the path with a circle of radius L_d around the rear axle; with α the bearing to it, `δ = atan(2 L sinα / L_d)`.

**Extended kinematic MPC.** Variables are the steering-rate sequence and the acceleration sequence over N = 10 steps (dt = 0.1 s), with box bounds (|δ| ≤ 35°, |δ̇| ≤ 1.5 rad/s, −3 ≤ a ≤ 2.5 m/s²). The plant model is rolled out exactly as in the simulator (including drag, so throttle = (a + drag)/k_a). The cost is expressed in the Frenet frame of each reference point:
`J = Σ_k [ w_lat e_lat² + w_lon e_lon² + w_ψ e_ψ² + w_v (v − v_ref)² ] + w_a a² + w_δ Δδ² + w_Δa Δa²`
with `e_lon = cosψ_r·Δx + sinψ_r·Δy`, `e_lat = −sinψ_r·Δx + cosψ_r·Δy`. It is solved by L-BFGS-B with a batched forward-difference gradient; the previous plan, shifted by one step, is the **warm start**; one step of actuator delay is compensated by propagating the state. The reference speed is curvature-limited and acceleration-reachable, and reference points are spaced by the integrated reference speed.

## Reproduction Guide

```bash
source /opt/ros/humble/setup.bash
cd /path/to/Control_Project && colcon build --symlink-install && source install/setup.bash

ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=pure_pursuit
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=mpc
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true   # + teleop_twist_keyboard in a 2nd terminal
# options: target_speed:=5.0 velocity_mode:=constant rviz:=false analyzer:=false summary_file:=/tmp/pp.json
```

The lap analyzer logs a banner after every lap and writes `/tmp/lap_summary.json` (per-lap time, mean/max/RMS CTE, speeds). To fill the table below from a ROS run, let each mode complete 3 laps and copy the values from that file. Telemetry: `ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed`.

Unit tests (no ROS needed): `cd bicycle_control && PYTHONPATH=. python3 -m pytest test/test_pure_pursuit.py test/test_longitudinal_pid.py test/test_lateral_pid.py test/test_velocity_profiler.py test/test_mpc.py test/test_path_utils.py test/test_pid_antiwindup.py`.
ROS-free closed-loop benchmark: `python3 tools/offline_benchmark.py --laps 3`.

## Telemetry Benchmark Leaderboard

**These numbers come from the OFFLINE harness `tools/offline_benchmark.py`, not from a ROS 2 run.** It uses the repository's own controller classes and the plant equations of `bicycle_model.py`, 3 complete laps of `centerline_0.csv` (528.2 m) per controller, target speed 4 m/s, 10 Hz loop, no DDS latency or jitter. Raw data: `benchmark_offline.json`. Mean/RMS/max CTE are over all three laps; best lap is the fastest of the three. **ROS 2 results have not been measured yet and must be re-collected on a ROS machine (see above).**

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
|---|---|---|---|---|---|---|
| **Manual Teleoperation** | not measured | not measured | not measured | not measured | not measured | not run |
| **Lateral PID (Reactive)** | 117.70 | 4.12 | 0.349 | 3.680 | 0.492 | 3 / OK |
| **Pure Pursuit (Preview)** | 112.00 | 4.12 | 0.075 | 0.364 | 0.109 | 3 / OK |
| **Extended Kinematic MPC (Optimal)** | 119.50 | 4.05 | 0.093 | 0.437 | 0.123 | 3 / OK |

Lap times (s): Lateral PID 117.8 / 118.5 / 117.7; Pure Pursuit 112.3 / 112.0 / 112.1; MPC 120.7 / 119.5 / 119.7. MPC solve time: mean 20.4 ms, max 83.0 ms per 100 ms tick (workstation CPU of the test sandbox).

## Critical Comparison

* **Lateral PID** is the cheapest and needs no model, but it only reacts to the error that already exists. It was by far the worst here (RMS 0.49 m; one lap peaked at 3.68 m in a tight corner) and its max CTE varies from lap to lap, which shows that it is sensitive to the corners.
* **Pure Pursuit** was the best in this test: lowest CTE and fastest laps. The simulated plant is a no-slip kinematic bicycle, which is exactly the model Pure Pursuit's arc geometry assumes, so it is close to ideal here. Its weaknesses (corner cutting at large look-ahead, no actuator limits, no speed awareness) are not exposed by this plant.
* **MPC** tracked far better than Lateral PID (RMS 0.12 vs 0.49 m) but **did not beat Pure Pursuit** in this measurement (0.12 vs 0.11 m RMS) and was ~7 s per lap slower because its reference speed is limited to what the car can reach with bounded acceleration and its speed term is weighted against tracking. It is also ~20 ms (up to 83 ms) per solve. I did not tune MPC weights further, so this is the result of the current untuned weights, not a ceiling.

## Why MPC Can Track Better Than Pure Pursuit and Lateral PID (theory)

1. **Preview of the road and of the dynamics.** Lateral PID uses only the present error. Pure Pursuit uses one preview point but assumes the car instantly follows a constant-curvature arc. MPC predicts the whole horizon with the actual vehicle model (steering rate limit, acceleration lag, drag, delay) and chooses all steering and acceleration inputs jointly, so it starts steering early enough for the real response.
2. **Constraints in the optimisation.** Steering angle, steering rate and acceleration limits are enforced inside the optimiser. PID and PP clip afterwards, and clipping breaks their design assumptions (integral windup, wrong arc).
3. **Joint speed and steering decision.** The speed profile and the lateral error share one cost, so MPC trades speed for accuracy in corners; the cascade of PID/PP treats them separately.
4. **Explicit weights and feedback.** The Frenet cost separates lateral from longitudinal error, so cross-track error can be weighted more heavily than lag along the path; re-solving every tick with the warm-started plan gives closed-loop correction.

These advantages grow when the plant has lag, limits or slip. On this kinematic, slip-free plant, with a loose MPC tuning and a 10-step horizon, they were not enough to beat a well-tuned Pure Pursuit, as the table shows.

## Milestone 6: Free Exploration (Four-wheel Ackermann kinematics)

The chosen topic is four-wheel Ackermann geometry, and the quantities below were computed numerically for this vehicle (L = 1.25 m, track W = 1.18 m). For a bicycle-model steering angle δ the turning radius is R = L / tanδ. To make both front wheels roll about the same centre, `tanδ_in = L / (R − W/2)` and `tanδ_out = L / (R + W/2)`:

| Bicycle δ | R (m) | δ_inner | δ_outer |
|---|---|---|---|
| 10° | 7.09 | 10.89° | 9.25° |
| 20° | 3.43 | 23.72° | 17.26° |
| 35° (limit) | 1.79 | 46.28° | 27.76° |

Findings: the inner/outer difference reaches 18.5° at the steering limit, so with the 35° bicycle limit the inner wheel in a real Ackermann car would be at 46°. The track's tightest corner (minimum radius ~2.17 m, measured from the CSV) needs a bicycle angle of about 30° (δ = atan(L/R)), so the bicycle model is only a mild approximation here for the controllers (they use the single-track angle at the rear-axle centre) but the URDF's two front wheels both use the same angle, which is a visual simplification. In `ros2_control` this geometry is handled by the steering controllers (Ackermann/bicycle steering controller libraries) via the wheelbase and track parameters. The Gazebo variant of the launch file was kept in `bicycle_sim/launch/archive/` as an unverified starting point for a 3D simulation. **I did not run Gazebo, MVSim or Nav2 MPPI, so I make no measured claim about them.** The qualitative trade-off (MPPI samples thousands of trajectories and handles non-differentiable costs and obstacles at the price of GPU/CPU load and noisier output, whereas the gradient MPC used here is smooth and cheap but local) is from the cited resources and the algorithms' definitions, not from experiments.

## Deliverable 2: Video Requirements (3–5 min)

~1 min of code (anti-windup clamp in `longitudinal_pid.py`, heading wrapping in `bicycle_model.py`, preview/curvature in `path_utils.py` and `pure_pursuit.py`); ~2 min of live RViz2 demos of all four modes (teleop with cruise control, Lateral PID, Pure Pursuit, MPC) with the HUD and `rqt_plot` telemetry visible; 1–2 min of analysis of the benchmark table. The video is not part of this repository and still has to be recorded.

## Known Limitations

* The ROS 2 launch, node integration, RViz2 display and the telemetry topics were **not run** in the environment where the changes were made (no ROS 2); they were only syntax-checked. The results above are offline.
* Manual teleoperation has no benchmark entry.
* MPC weights are untuned and its mean tick time is 20 ms (max 83 ms); the Python implementation is not real-time safe on a slow CPU.
