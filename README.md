# Bicycle Gym — ARL Autonomous Vehicle Control Project

**Autotronics Research Lab (ARL) — Ain Shams University** · *Autonomous Vehicles & Drive-by-Wire Systems — individual project*

<p align="center">
  <img src="assets/demo.gif" alt="Simulation demo" width="100%" />
</p>

| | |
|---|---|
| **Student** | Abdalrhman Ahmed Zakaria *(name and department taken from the supplied study report — edit if needed)* |
| **Department** | Senior Mechatronics |
| **ROS distribution** | **ROS 2 Jazzy Jalisco** (Ubuntu 24.04, Python 3.12) |
| **Packages** | `bicycle_sim`, `bicycle_control`, `track_environment` (all `ament_python`) |
| **Track** | `track_environment/tracks/centerline_0.csv` — 1,001 waypoints, closed loop; 528.2 m is the summed polyline length of the CSV, while the distance actually driven per lap is ≈ 445 m (see [§6.5](#65-data-quality-notes-and-open-points)) |

> **Honesty note.** Every implementation statement below was checked against the source code in this repository. The results section now contains **two clearly separated kinds of evidence**: (1) **ROS 2 simulation results** — 3 completed laps per controller (Lateral PID, Pure Pursuit, MPC) copied from the `lap_analyzer` console banners of real launches (raw text in [`docs/results/ros2_lap_analyzer_logs.txt`](docs/results/ros2_lap_analyzer_logs.txt)); these are the **primary results**; and (2) the earlier **offline, ROS-free replay** of the control loop (`tools/offline_benchmark.py`), kept only as a cross-check. The two are never mixed in a table row. The 3-lap aggregates of the ROS runs were **derived from the per-lap banners** (the `lap_summary.json` file itself was not supplied). Items that are unverified are marked ⚠️.

---

## Table of Contents

1. [Project overview](#1-project-overview)
2. [Environment setup (ROS 2 Jazzy)](#2-environment-setup-ros-2-jazzy)
3. [Package architecture](#3-package-architecture)
4. [Build and launch](#4-build-and-launch)
5. [Milestone documentation (M1–M8)](#5-milestone-documentation)
6. [Results and assessment](#6-results-and-assessment)
7. [Why MPC can track better than Pure Pursuit and Lateral PID](#7-why-mpc-can-track-better-than-pure-pursuit-and-lateral-pid)
8. [Requirement status summary](#8-requirement-status-summary)
9. [Troubleshooting](#9-troubleshooting)
10. [Repository layout and submission checklist](#10-repository-layout-and-submission-checklist)

---

## 1. Project overview

Bicycle Gym is a ROS 2 project in which a simulated self-driving car is driven around a racetrack. The goal is to keep the car on the reference path, regulate its speed and complete laps quickly and accurately. The car is deliberately realistic: **velocity is a state, not an input**. It changes through throttle, aerodynamic drag and rolling resistance, so the controllers must cope with powertrain lag and with the coupling between speed and steering.

The work is organised as eight milestones: graph discovery, vehicle kinematics, teleoperation, cruise control, autonomous path tracking (velocity profiler, Lateral PID, Pure Pursuit, MPC, lap analyzer), free exploration, documentation and a video.

**Vehicle (verified in code and URDF).**

| Parameter | Value | Where |
|---|---|---|
| Wheelbase $L$ | 1.25 m | `bicycle_model.py`, controllers, URDF (rear axle x = 0, front axle x = 1.25) |
| Track width | 1.18 m | URDF wheel offsets y = ±0.59 m |
| Wheel radius | 0.5 m | `wheel_radius` parameter |
| Max steering | ±35° (0.6109 rad) | `max_steer_rad` |
| $k_a$, $c_{drag}$, $c_{roll}$ | 4.0 m/s², 0.005, 0.05 | `bicycle_model.py` (MPC uses the same values) |
| Speed limit | 0 … 25 m/s, no reverse | `max_speed` |

⚠️ The 0.3 m wheel width from the project description was not checked in the URDF.

### Live RViz2 view

<p align="center"><img src="docs/images/rviz_lap1_start.png" alt="RViz2 view at the start of lap 1" width="90%"></p>

*Figure 1 — RViz2 (screenshot supplied by the student): the car (robot model), the cyan reference path, yellow/blue boundary cones, the green start gate and the floating HUD (`LAP 1  4.10 s`, `SPEED 3.85 m/s`, `CTE +0.04 m`). The controller mode used for this screenshot was not recorded.*

<p align="center"><img src="docs/images/rviz_lap1_mid.png" alt="RViz2 view later in lap 1" width="90%"></p>

*Figure 2 — Same RViz2 configuration later in lap 1 (`36.10 s`, `3.05 m/s`, `CTE +0.01 m`): the car is on the path while cornering and the HUD follows it. This is a single instant, not a completed lap.*

---

## 2. Environment setup (ROS 2 Jazzy)

The project was developed with **ROS 2 Jazzy**. The package manifests use `package.xml` format 3 with no distro-specific dependencies, so they are not tied to a particular release. ⚠️ The build in this repository was not re-run by the README author; the Jazzy statement and the screenshots come from the student's own machine.

### 2.1 Install ROS 2 Jazzy and dependencies

```bash
# Ubuntu 24.04 with ROS 2 Jazzy already installed from the official apt repository
source /opt/ros/jazzy/setup.bash

sudo apt update
sudo apt install -y \
  ros-jazzy-desktop \
  ros-jazzy-robot-state-publisher ros-jazzy-rviz2 ros-jazzy-xacro \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-rqt-graph ros-jazzy-rqt-plot ros-jazzy-plotjuggler-ros \
  python3-colcon-common-extensions python3-numpy python3-scipy python3-pytest
```

| Dependency | Used by |
|---|---|
| `rclpy`, `std_msgs`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `visualization_msgs`, `tf2_ros` | all nodes |
| `robot_state_publisher`, `rviz2`, `xacro` | URDF, RViz2 display |
| `numpy`, `scipy` | controllers; MPC uses `scipy.optimize.minimize` (L-BFGS-B) |
| `teleop_twist_keyboard` | keyboard driving (M3/M4) |
| `rqt_plot`, `PlotJuggler` | live signal plotting (M1, M5.5) |

### 2.2 Every new terminal

```bash
source /opt/ros/jazzy/setup.bash
cd ~/path/to/Control_Project
source install/setup.bash
```

---

## 3. Package architecture

```
Control_Project/
├── bicycle_sim/          plant, URDF/Xacro, RViz config, launch files
├── bicycle_control/      teleop bridge, PID, profiler, Lateral PID, Pure Pursuit, MPC, controller node
├── track_environment/    track loader, path publisher, lap analyzer, track CSVs
├── tools/                offline_benchmark.py (ROS-free replay)
├── docs/images/          screenshots used in this README
├── docs/results/         raw lap_analyzer console output of the ROS 2 runs
└── assets/demo.gif
```

| Package | Responsibility | Key files |
|---|---|---|
| `bicycle_sim` | Extended kinematic bicycle plant, TF, joint states, URDF, RViz | `bicycle_model.py`, `sim_node.py`, `urdf/racecar.urdf`, `launch/bicycle.rviz`, `launch/bicycle_sim.launch.py` |
| `bicycle_control` | Teleop bridge, longitudinal and lateral controllers | `teleop_bridge.py`, `longitudinal_pid.py`, `velocity_profiler.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `path_utils.py`, `controller_node.py` |
| `track_environment` | CSV track loading, `/path`, cone markers, lap analysis | `track.py`, `path_gen.py`, `lap_analyzer.py` |

### 3.1 ROS graph (verified from `rqt_graph` screenshots)

<p align="center"><img src="docs/images/rqt_graph_teleop.png" alt="rqt_graph in teleoperation mode" width="95%"></p>

*Figure 3 — `rqt_graph`, `controller:=teleop`: `/teleop_bridge` → `/throttle`, `/steer` → `/kinematic_bicycle` → `/state`, `/joint_states` (→ `/robot_state_publisher` → `/robot_description`); `/path_gen` → `/path`, `/track_bounds`; `/path` and `/state` → `/lap_analyzer` → `/lap/visualization`.*

<p align="center"><img src="docs/images/rqt_graph_controller.png" alt="rqt_graph in autonomous mode" width="95%"></p>

*Figure 4 — `rqt_graph`, autonomous mode: `/controller` subscribes to `/path` and `/state` and publishes `/throttle` and `/steer`, closing the loop through `/kinematic_bicycle`. The mode (Lateral PID / Pure Pursuit / MPC) used for this capture was not recorded.*

### 3.2 Topics and message types

Verified in the source code; the ones drawn in the graphs are also confirmed by the screenshots.

| Topic | Type | Publisher → Subscriber | Meaning / units |
|---|---|---|---|
| `/throttle` | `std_msgs/Float32` | bridge or controller → sim | normalised throttle/brake, −1…1 |
| `/steer` | `std_msgs/Float32` | bridge or controller → sim | front steering angle, rad, + = left |
| `/cmd_vel` | `geometry_msgs/Twist` | keyboard → `teleop_bridge` | `linear.x` m/s, `angular.z` rad/s |
| `/state` | `nav_msgs/Odometry` | sim → controller, bridge, analyzer | pose (m, quaternion), `twist.linear.x` = speed (m/s) |
| `/joint_states` | `sensor_msgs/JointState` | sim → `robot_state_publisher` | steering hinges (rad), wheel rotation (rad) |
| `/path` | `nav_msgs/Path` | `path_gen` → controller, analyzer | centerline poses, frame `map`, 10 Hz |
| `/track_bounds` | `visualization_msgs/MarkerArray` | `path_gen` → RViz | boundary cones |
| `/lap/visualization` | `visualization_msgs/MarkerArray` | analyzer → RViz | start gate, CTE whisker, HUD |
| `/telemetry/cte` | `std_msgs/Float32` | analyzer | signed cross-track error, m (+ = left of path) |
| `/telemetry/speed` | `std_msgs/Float32` | analyzer | m/s |
| `/telemetry/heading_err_deg` | `std_msgs/Float32` | analyzer | degrees |
| `/telemetry/lap_time` | `std_msgs/Float32` | analyzer | current lap time, s |
| `/lap/metrics` | `std_msgs/String` (JSON) | analyzer | `lap, current_lap_time, elapsed_time, last_lap_time, best_lap_time, speed, current_cte, rms_cte, heading_err_deg` |

⚠️ The `/telemetry/*` and `/lap/metrics` topics are verified from the code only; they do not appear in the supplied graph screenshots (rqt_graph hides topics without subscribers).

### 3.3 Data-flow summary

```
 teleop_twist_keyboard -> /cmd_vel -> teleop_bridge --+
                                                      +--> /throttle, /steer --> kinematic_bicycle (sim_node)
 path_gen --> /path ---> controller (PID | PP | MPC) -+            |  -> /state (Odometry), /joint_states, TF map->base_link
          \-> /track_bounds (cones)    ^---------------- /state ----+
 /path + /state --> lap_analyzer --> /telemetry/{cte,speed,heading_err_deg,lap_time}, /lap/metrics (JSON), /lap/visualization
 robot_state_publisher <- /joint_states      RViz2: RobotModel, Path, /track_bounds, /lap/visualization
```

In teleoperation mode `teleop_bridge` drives `/throttle` and `/steer` (Figure 3); in the autonomous modes `/controller` replaces it (Figure 4). Both are never active together.

---

## 4. Build and launch

```bash
source /opt/ros/jazzy/setup.bash
cd ~/path/to/Control_Project
colcon build --symlink-install
source install/setup.bash
```

All modes use one launch file (`bicycle_sim/launch/bicycle_sim.launch.py`); the mode is selected with `controller:=`:

| Mode | Command | Milestone |
|---|---|---|
| Base simulation only | `ros2 launch bicycle_sim bicycle_sim.launch.py` | M1, M2 |
| Teleoperation (open loop) | `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop` | M3 |
| Teleoperation + cruise control | `… controller:=teleop use_cruise_control:=true` | M4 |
| Lateral PID | `… controller:=lateral_pid` | M5.2 |
| Pure Pursuit | `… controller:=pure_pursuit` | M5.3 |
| Extended kinematic MPC | `… controller:=mpc` | M5.4 |

Other arguments: `rviz:=false`, `analyzer:=false`, `target_speed:=4.0`, `velocity_mode:=curvature|constant`, `track_file:=centerline_0.csv`, `summary_file:=/tmp/lap_summary.json`.

For teleoperation, in a **second sourced terminal**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

The earlier Gazebo launch variant is archived in `bicycle_sim/launch/archive/gazebo_launch_commented.py.txt` and is **not** used by the main launch file.

---

## 5. Milestone documentation

Status legend: ✅ implemented and checked in code/unit tests · 🟡 implemented, checked only offline or by reading code · ⚠️ not verified in ROS 2 / not done.

### Milestone 1 — Discovery

**Objective.** Launch the base simulation, identify active topics, message types and units with the ROS 2 command-line tools, and set up live plotting with `rqt_plot` and PlotJuggler.

**What is in the project.** The base launch (`controller:=none`) starts `robot_state_publisher`, the plant `/kinematic_bicycle`, `/path_gen`, `/lap_analyzer` and RViz2. The topic table in [§3.2](#32-topics-and-message-types) lists every topic, type and unit, taken from the code; the ROS graph is documented by Figures 3 and 4.

**Run / verify.**

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py
ros2 node list
ros2 topic list -t
ros2 topic info /state
ros2 topic info /throttle
ros2 topic info /steer
ros2 interface show nav_msgs/msg/Odometry
ros2 interface show std_msgs/msg/Float32
ros2 interface show geometry_msgs/msg/Twist
ros2 topic echo /state
ros2 run rqt_graph rqt_graph
ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed
ros2 run plotjuggler plotjuggler          # then: Streaming → ROS 2 Topic Subscriber
```

**Status.** ✅ graph discovery (screenshots). ⚠️ No screenshot of `rqt_plot` or PlotJuggler was supplied, so those two tools are documented but not evidenced.

### Milestone 2 — Vehicle kinematics and powertrain resistance

**Objective.** Implement the extended kinematic bicycle equations, integrate with forward Euler (with heading wrapping and speed clamping) and verify the physics with direct throttle/steer commands.

**Implementation** (`bicycle_sim/bicycle_sim/bicycle_model.py`, class `Car`). State $[x, y, \theta, v]$ at the **rear axle**; inputs $u\in[-1,1]$ (throttle/brake) and $\delta$ (rad). In `update_x_dot()`:

$$
\dot x = v\cos\theta,\quad \dot y = v\sin\theta,\quad \dot\theta = \frac{v}{L}\tan\delta,\quad
\dot v = k_a u - c_{drag}v^2 - c_{roll}v
$$

with $L=1.25$ m, $k_a=4$ m/s², $c_{drag}=0.005$, $c_{roll}=0.05$. In `update_x()` (dt = 0.1 s from a 10 Hz timer):

1. Forward Euler: $\mathbf{x}_{k+1}=\mathbf{x}_k+\dot{\mathbf{x}}\,\Delta t$.
2. **Heading wrap:** $\theta \leftarrow \operatorname{atan2}(\sin\theta,\cos\theta)$.
3. **Speed clamp:** $v \leftarrow \min(\max(v,0),\,v_{max})$ — no reverse, $v_{max}=25$ m/s.

Because $v\ge 0$, drag and rolling resistance always oppose motion. Incoming commands are clamped (steering ±35°, throttle ±1) with a throttled warning. The node publishes `/state`, `/joint_states` (steering and wheel spin) and the TF `map → ego_racecar/base_link`. `sim_node.py` reads the start pose from the track CSV.

**Run / verify.**

```bash
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: 0.5}"
ros2 topic pub --once /steer    std_msgs/msg/Float32 "{data: 0.30}"
ros2 topic pub --once /throttle std_msgs/msg/Float32 "{data: -1.0}"
ros2 topic echo /state
pytest bicycle_sim/test/test_bicycle_model.py      # needs ROS (rclpy)
```

Expected: with $u=0.5$ the speed approaches the terminal value where $k_a u = c_{drag}v^2+c_{roll}v$ (about 15.6 m/s, from solving $2 = 0.005v^2 + 0.05v$), steering 0.3 rad produces a left turn, and negative throttle brakes to a stop without reversing.

**Status.** ✅ code inspected against the equations. ⚠️ `test_bicycle_model.py` needs `rclpy` and was not run by the README author.

### Milestone 3 — Keyboard teleoperation node

**Objective.** Translate `Twist` messages into throttle and steering using open-loop mapping equations with a 0.5 s safety watchdog.

**Implementation** (`bicycle_control/bicycle_control/teleop_bridge.py`, node `teleop_bridge`). Subscribes `/cmd_vel`, publishes `/throttle` and `/steer` at 10 Hz.

$$
u = \operatorname{clip}\!\left(\frac{v_{cmd}}{v_{lin,max}},-1,1\right),\qquad
\delta = \operatorname{clip}\!\left(\frac{\omega_{cmd}}{\omega_{max}}\,\delta_{max},-\delta_{max},\delta_{max}\right)
$$

Defaults: $v_{lin,max}=5$ m/s, $\omega_{max}=1$ rad/s, $\delta_{max}=0.6109$ rad (35°), all ROS parameters. **Watchdog:** each callback stores the receive time; the 10 Hz timer computes the age and, if it exceeds `auto_zero_timeout` = 0.5 s, sets throttle, steering and target speed to zero.

**Run / verify.**

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic echo /throttle      # release the keys: returns to 0.0 after ≈0.5 s
```

**Status.** 🟡 code inspected; Figure 3 shows the node wired to `/throttle` and `/steer`. ⚠️ Driving by keyboard and the watchdog timing were not demonstrated in a supplied recording. *Note:* `/steer` is published in every cycle (an earlier revision of this node did not publish it; that is fixed in the current code).

### Milestone 4 — Low-level cruise control

**Objective.** Implement a longitudinal PID with anti-windup, treat the teleop linear velocity as a target speed, and show closed-loop regulation against drag.

**Implementation** (`longitudinal_pid.py`, class `PIDLongitudinalController`):

$$
e = v_{target}-v,\quad I \leftarrow \operatorname{clip}(I+e\,\Delta t,\,-I_{max},\,I_{max}),\quad
u=\operatorname{clip}\!\left(K_p e+K_i I+K_d\tfrac{\dot e}{1},\,-1,\,1\right)
$$

Gains $K_p=1.0$, $K_i=0.1$, $K_d=0$, $I_{max}=2$. **Anti-windup** is integrator clamping (the integral cannot grow without bound while the throttle is saturated). The output is clipped to the actuator range −1…1 (brake to full throttle). `reset()` clears the integrator.

**Integration.** With `use_cruise_control:=true`, `teleop_bridge` subscribes `/state`, treats `linear.x` as the target speed, and calls `pid.compute(target, speed)`. When target and speed are both below 0.05 m/s the PID is reset and throttle is zero (prevents integral build-up while stopped). The same PID class is reused by the autonomous controller node.

**Run / verify.**

```bash
ros2 launch bicycle_sim bicycle_sim.launch.py controller:=teleop use_cruise_control:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run rqt_plot rqt_plot /telemetry/speed
cd bicycle_control && PYTHONPATH=. python3 -m pytest test/test_longitudinal_pid.py test/test_pid_antiwindup.py
```

**Status.** ✅ unit tests (clamped output and bounded integrator under saturation). ⚠️ A speed-step plot against drag in ROS was not supplied.

### Milestone 5 — Autonomous path tracking

The autonomous modes all run in one node, `controller_node.py` (`/controller`), at 10 Hz. It subscribes `/state` and `/path`, caches the path, and each tick computes steering with the selected lateral controller and throttle with the longitudinal PID (MPC computes both). The controller is chosen by the `control_mode` parameter (set by `controller:=`).

#### Milestone 5.1 — Velocity profiler and path curvature

**Objective.** Compute path curvature and apply curvature-limited speed constraints.

**Implementation.**

* *Curvature* (`path_utils.path_curvature`). The track CSV has highly non-uniform spacing (≈3 mm to 0.96 m between waypoints), so differencing neighbouring headings is dominated by noise. Instead, for each waypoint $b$ the points $a$ and $c$ located 1.5 m of arc length behind and ahead are interpolated, and the signed Menger curvature is used (positive = left turn):

$$
\kappa=\frac{2\,(b-a)\times(c-b)}{\lVert b-a\rVert\,\lVert c-b\rVert\,\lVert c-a\rVert}
$$

  On `centerline_0.csv` this gives a maximum $|\kappa|\approx0.46$ 1/m (minimum radius ≈ 2.17 m).
* *Profile* (`velocity_profiler.py`): with a lateral-acceleration budget $a_{lat}=5$ m/s²,

$$
v_{target}=\min\!\Big(v_{cruise},\ \sqrt{a_{lat}/|\kappa|}\Big),\qquad v_{target}\in[0,\,7.5]
$$

  When no cruise cap is given, a straight returns `max_speed`.
* *Preview:* `controller_node.preview_speed()` evaluates the **worst** curvature over a window of $\max(3\,\text{m},\,1.5\,\text{s}\cdot v)$ ahead, so the car decelerates before a corner.

**Verify.** `cd bicycle_control && PYTHONPATH=. python3 -m pytest test/test_velocity_profiler.py test/test_path_utils.py` (circle → $1/R$, straight → 0, sign positive for left). **Status.** ✅ unit tests pass.

#### Milestone 5.2 — Lateral PID

**Objective.** Steer from cross-track and heading error with correct sign conventions.

**Implementation** (`lateral_pid.py`, `LateralPIDController`; errors from `controller_node.compute_track_errors()`). The cross-track error (CTE) is the signed distance to the nearest path segment, **positive when the car is to the left of the path direction**; the heading error is $e_\psi=\operatorname{wrap}(\psi-\psi_{path})$.

$$
\delta=-\Big(K_p\,\mathrm{cte}+K_i\!\int\!\mathrm{cte}\,dt+K_d\frac{d\,\mathrm{cte}}{dt}\Big)-K_\psi e_\psi
$$

$K_p=0.48,\ K_i=0,\ K_d=0.15,\ K_\psi=0.7$; integral clamped to ±1; output clipped to ±35°. Both terms carry a minus sign (negative feedback: left of the path → steer right; heading rotated left → steer right). The derivative is initialised on the first sample so there is no start-up kick. Speed is set by the curvature profiler and the speed PID.

**Run / verify.** `ros2 launch bicycle_sim bicycle_sim.launch.py controller:=lateral_pid` and `ros2 run rqt_plot rqt_plot /telemetry/cte`; unit test `test/test_lateral_pid.py`. **Status.** ✅ unit tests; ✅ 3 laps in ROS 2 (RMS CTE 0.406 m, §6.2); offline laps as cross-check (§6.4).

#### Milestone 5.3 — Pure Pursuit

**Objective.** Geometric steering with an adaptive look-ahead distance.

**Implementation** (`pure_pursuit.py`, `PurePursuitController`).

* **Adaptive look-ahead:** $L_d=\operatorname{clip}(L_{d0}+k_v v,\ L_{min},\ L_{max})$ with $L_{d0}=0.8$ m, $k_v=0.5$ s, $L_{min}=0.8$ m, $L_{max}=3.5$ m. These are ROS parameters (`pp_l0`, `pp_kv`, `pp_l_min`, `pp_l_max`) and can be changed at run time (`ros2 param set /controller pp_kv 0.7`).
* **Target point:** the nearest waypoint is searched in a window around the previous index (global search if lost); the path is then walked forward and the circle of radius $L_d$ around the rear axle is intersected with the path segment, giving a continuous target (no waypoint jumps).
* **Arc steering:** with $\alpha$ the bearing to the target in the body frame,

$$
\kappa_{arc}=\frac{2\sin\alpha}{L_d},\qquad \delta=\arctan(L\,\kappa_{arc})=\arctan\!\Big(\frac{2L\sin\alpha}{L_d}\Big),\quad |\delta|\le35^\circ
$$

**Run / verify.** `… controller:=pure_pursuit`; `test/test_pure_pursuit.py`. **Status.** ✅ unit tests; ✅ 3 laps in ROS 2 (RMS CTE 0.085 m, §6.2); offline laps as cross-check (§6.4).

#### Milestone 5.4 — Extended kinematic MPC

**Objective.** Frenet-frame cost, receding horizon and warm-start initialisation.

**Implementation** (`mpc.py`, `KinematicBicycleMPC`).

* **Prediction model:** the same extended kinematic bicycle as the simulator, including drag and rolling resistance, forward Euler, $N=10$, $\Delta t=0.1$ s. Commanded acceleration is converted to throttle by $u=(a+c_{drag}v^2+c_{roll}v)/k_a$.
* **Decision variables:** steering-rate sequence (normalised to $|\dot\delta|\le1.5$ rad/s) and acceleration sequence ($-3\le a\le2.5$ m/s²), all box-bounded, so L-BFGS-B stays feasible; $\delta$ is additionally clipped to ±35°.
* **Frenet-frame cost.** For every predicted pose, the error to the reference point (position $x_r,y_r$, heading $\psi_r$) is rotated into the reference frame:

$$
e_{lon}=\cos\psi_r\,\Delta x+\sin\psi_r\,\Delta y,\qquad e_{lat}=-\sin\psi_r\,\Delta x+\cos\psi_r\,\Delta y
$$

$$
J=\sum_{k=1}^{N}\Big[w_{lat}e_{lat}^2+w_{lon}e_{lon}^2+w_\psi e_\psi^2+w_v(v-v_{ref})^2\Big]c_k+\sum_k\big(w_a a_k^2+w_{\delta}\,\Delta\delta_k^2+w_{\Delta a}\,\Delta a_k^2\big)
$$

  $w_{lat}=30,\ w_{lon}=1,\ w_\psi=10,\ w_v=3,\ w_a=0.1,\ w_\delta=3,\ w_{\Delta a}=0.5$; terminal stage weight $c_N=2$.
* **Receding horizon:** the problem is re-solved every tick and only the first steering angle and throttle are applied. The gradient is a forward difference evaluated for all perturbations in one batched NumPy rollout (no hand-written adjoint).
* **Warm start:** the previous solution is shifted by one step (last element repeated) and used as the initial guess; with no previous plan, steering is held and a P-speed acceleration is used. `maxiter=30`, `maxfun=100`.
* **Delay compensation:** one step of actuator delay is propagated before optimising.
* **Reference** (`path_utils.mpc_reference`): rows $[x, y, \psi, v_{ref}]$ where $v_{ref}$ is curvature-limited **and reachable** ($v_k\le v_0+1.5\,(k{+}1)\Delta t$), and rows are spaced by the integrated reference speed $\sum v_{ref}\Delta t$ (not the measured speed, otherwise a slow car asks for nearby targets and never accelerates).

**Run / verify.** `… controller:=mpc`; `test/test_mpc.py`; MPC solve time is available in `mpc.last_info`. **Status.** ✅ unit test; ✅ 3 laps in ROS 2 (RMS CTE 0.103 m, §6.2); offline laps as cross-check (§6.4). MPC weights are not tuned. ⚠️ The MPC solve time was not recorded in the ROS 2 run; the only timing figure (mean 20.4 ms, max 83.0 ms) is from the offline harness.

#### Milestone 5.5 — Lap analyzer

**Objective.** Lap timing, real-time telemetry topics and RViz dashboard markers.

**Implementation** (`track_environment/lap_analyzer.py`, node `/lap_analyzer`).

* **Projection:** each `/state` message is projected onto the nearest path segment, giving the signed CTE, heading error and arc position $s$.
* **Lap detection:** a lap is counted when $s$ wraps from $>75\%$ to $<25\%$ of the track length while $v>0.1$ m/s; the crossing time is interpolated between ticks. The lap clock is held at 0 until the car first moves.
* **Per-lap metrics** (logged as a banner and stored): lap time, mean/max/RMS $|CTE|$, mean heading error, mean/top speed. Buffers reset every lap. ⚠️ The `Distance` line of the banner is the **cumulative** odometric distance since the start of the run (`total_distance` is never reset), not the distance of that lap; per-lap distance is the difference of consecutive banners (§6.2). After each lap `lap_summary.json` (`summary_file` parameter, default `/tmp/lap_summary.json`) is rewritten with laps completed, best lap, top speed, mean/max/RMS CTE and the per-lap list — this is the source for the ROS benchmark table.
* **Telemetry topics:** `/telemetry/cte`, `/telemetry/speed`, `/telemetry/heading_err_deg`, `/telemetry/lap_time` (Float32) and `/lap/metrics` (JSON), all at 10 Hz.
* **RViz markers** (`/lap/visualization`): start/finish gate (green cylinder), **CTE whisker** (line from the rear axle to the projected path point, green → red with error; saturates at 1 m), and a **text HUD** above the car: `LAP n  <time> s / BEST / SPEED / CTE` (visible in Figures 1 and 2).

**Run / verify.** `ros2 topic echo /lap/metrics`, `ros2 run rqt_plot rqt_plot /telemetry/cte /telemetry/speed /telemetry/heading_err_deg`, and `cat /tmp/lap_summary.json` after a lap. **Status.** ✅ completed laps in ROS 2 are evidenced by the lap banners of all three autonomous controllers (§6.2); the HUD and path are shown in Figures 1–2. ⚠️ The `/tmp/lap_summary.json` file itself and the `rqt_plot` / PlotJuggler views were not supplied.

### Milestone 6 — Free exploration

**Objective.** Investigate one of: four-wheel Ackermann kinematics, 3D simulation (Gazebo/MVSim) or Nav2 MPPI, and document the findings.

**Chosen topic: four-wheel Ackermann kinematics** (the topic that could be evaluated with the project's own numbers). A bicycle angle $\delta$ corresponds to a turning radius $R=L/\tan\delta$ measured at the rear-axle centre. For both front wheels to roll about the same centre,

$$
\tan\delta_{in}=\frac{L}{R-W/2},\qquad \tan\delta_{out}=\frac{L}{R+W/2}
$$

Computed for this vehicle ($L=1.25$ m, $W=1.18$ m):

| Bicycle $\delta$ | $R$ (m) | $\delta_{inner}$ | $\delta_{outer}$ |
|---|---|---|---|
| 10° | 7.09 | 10.89° | 9.25° |
| 20° | 3.43 | 23.72° | 17.26° |
| 35° (limit) | 1.79 | 46.28° | 27.76° |

**Findings.** (1) At the steering limit the inner and outer wheels differ by about 18.5°, so identical front-wheel angles (as in this URDF, where both hinges use `δ`) force tyre scrub; the single-track model is a faithful *kinematic* description of the vehicle centre but hides this. (2) The tightest corner on `centerline_0.csv` has $R\approx2.17$ m (from the curvature estimate), which needs $\delta=\arctan(L/R)\approx29.9^\circ$ — inside the 35° limit but close to it; a real Ackermann car would need ≈ 38° on the inner wheel there. (3) In `ros2_control` this geometry is handled by the steering-controller library, which takes wheelbase and track-width parameters.

**Gazebo (3D) — present but not verified.** The repository contains an unmerged Gazebo variant: `bicycle_sim/bicycle_sim/gz_adapter.py` (integrates throttle with the same longitudinal model and publishes a Gazebo `Twist`), `config/gazebo_bridge.yaml`, `urdf/edges_gazebo.xacro`, `gazebo_fix.patch`, and the archived launch code. **I did not run Gazebo, MVSim or Nav2 MPPI, so no measured claim is made about them.** The qualitative contrast (sampling-based MPPI handles non-differentiable costs and obstacles but needs many rollouts; gradient MPC as used here is smooth and cheap but local) comes from the algorithm definitions, not from experiments.

**Curated resources reviewed** *(organised as in the course material; the links were not re-checked by the README author)*

| Topic | Resource | What it covers |
|---|---|---|
| Ackermann / `ros2_control` | [ROS 2 Control Mobile Robot Kinematics Guide](https://control.ros.org/humble/doc/ros2_controllers/doc/mobile_robot_kinematics.html) | modelling 4-wheel kinematics and full car models instead of the simplified bicycle model |
| | [ROS 2 Steering Controllers Library](https://control.ros.org/kilted/doc/ros2_controllers/steering_controllers_library/doc/userdoc.html) | Ackermann and bicycle steering controllers in `ros2_control` |
| | [ros2_control_demos Example 11: Steered Wheel Base](https://control.ros.org/humble/doc/ros2_control_demos/example_11/doc/userdoc.html) | steered-wheel bases and hardware interfaces |
| | [ros2_control_demos](https://github.com/ros-controls/ros2_control_demos) · [ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/master) | reference suite and upstream chassis/vehicle controllers |
| | [Four-Wheel AMR](https://github.com/abubakar-mughal97/four_wheel_amr) | 4-wheel robot package with Ackermann steering |
| 3D simulation | [Ackermann Vehicle in Gz-Sim & ROS 2](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2) ([main branch](https://github.com/alitekes1/ackermann-vehicle-gzsim-ros2/tree/main)) | autonomous Ackermann vehicle in modern Gazebo |
| | [Classic Gazebo Ackermann Simulation](https://github.com/lucasmazzetto/gazebo_ackermann_steering_vehicle) | physical Ackermann steering linkages |
| | [Ackermann Autonomous Car Simulation](https://github.com/armando-genis/Ackermann-Autonomous-Car-Simulation) | autonomous driving stack with Ackermann kinematics |
| | [MVSim for ROS 2](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/MVSim/Simulation-MVSim.html) | lightweight multi-vehicle dynamic simulator |
| Sampling-based control | [Nav2 MPPI Controller](https://index.ros.org/p/nav2_mppi_controller/) | real-time MPPI controller with obstacle avoidance and custom costs |

**Synthesis (from the resources and the algorithm definitions — not from experiments).**

1. *Kinematics vs multi-body.* A 4-wheel Ackermann car gives the inner wheel a sharper angle than the outer one because the two follow radii $R-W/2$ and $R+W/2$; forcing equal angles causes tyre scrub, tread wear and energy loss. `ros2_control` handles this through its steering-controller library, parameterised by wheelbase and track width.
2. *2D vs 3D simulation.* The kinematic plant used here assumes zero tyre slip and is cheap and deterministic. 3D physics engines (Gazebo, MVSim) add tyre friction saturation (e.g. Pacejka-type models), suspension compliance, sensor noise and terrain, at a much higher modelling and compute cost — which is also why the controllers here were not exposed to slip.
3. *Deterministic vs sampling control.* The gradient-based MPC used here is smooth and cheap but local; Nav2 MPPI samples thousands of randomised trajectories in parallel and weights them by path-integral weighting, so it handles non-differentiable costs and obstacles without a gradient solver, at the price of GPU/CPU load and noisier output.

**Status.** 🟡 Ackermann analysis (computed numerically, not simulated); ⚠️ 3D simulation and MPPI not investigated.

### Milestone 7 — Deliverable 1: documentation

**Objective.** README with student information, architecture, mathematics, a benchmark table over ≥3 laps, controller comparison, the MPC theory and the Milestone 6 summary and reproduction guide.

| Required item | Where |
|---|---|
| Student information | top of this file |
| System architecture | [§3](#3-package-architecture) |
| Mathematical formulations | [§5](#5-milestone-documentation) (M2–M5.5) |
| Benchmark table, ≥3 laps | [§6.2](#62-benchmark-table) — **ROS 2** results (primary) plus offline cross-check |
| Critical comparison | [§6.3](#63-critical-comparison) |
| Theory of why MPC can track better | [§7](#7-why-mpc-can-track-better-than-pure-pursuit-and-lateral-pid) |
| Milestone 6 summary | [Milestone 6](#milestone-6--free-exploration) |
| Reproduction guide | [§2](#2-environment-setup-ros-2-jazzy), [§4](#4-build-and-launch), [§6.1](#61-how-the-numbers-were-obtained) |
| Source files | [§10](#10-repository-layout-and-submission-checklist) |

**Status.** ✅ complete in content: the 3-lap benchmark of all three controllers was measured in ROS 2 (derived from the lap banners; `lap_summary.json` not supplied) and cross-checked offline (see §6).

### Milestone 8 — Deliverable 2: video walkthrough

**Objective.** A 3–5 minute video: ≈1 min of code (anti-windup, heading wrapping, preview calculations), ≈2 min of live RViz2 demos of **all four controller modes** with the telemetry display, and 1–2 min of analysis.

**Suggested script.**

| Time | Content | Where in the code |
|---|---|---|
| 0:00–1:00 | Code: integrator clamp, heading wrap, preview/curvature | `longitudinal_pid.py` (`np.clip` on `integral`), `bicycle_model.py::update_x`, `path_utils.path_curvature` + `controller_node.preview_speed`, `pure_pursuit.find_target_waypoint` |
| 1:00–3:00 | Live RViz2: teleop+cruise, Lateral PID, Pure Pursuit, MPC with HUD, CTE whisker, `rqt_plot` | launch commands in §4 |
| 3:00–5:00 | Analysis of the benchmark table and the MPC theory | §6, §7 |

**Status.** ⚠️ **Not done** — the video is not part of the repository and still has to be recorded. Record it from a real ROS 2 run; the numbers shown on screen should agree with the ROS 2 benchmark table in §6.2.

---

## 6. Results and assessment

### 6.1 How the numbers were obtained

There are two kinds of evidence, and they must not be mixed up:

| Kind | Source | Status |
|---|---|---|
| **ROS 2 (Jazzy) — primary** | Console banners printed by the `lap_analyzer` node (`LAP n COMPLETE`) during real launches of `controller:=pure_pursuit`, `controller:=mpc` and `controller:=lateral_pid`, 3 completed laps each on `centerline_0.csv` (`target_speed` 4 m/s). Raw text: [`docs/results/ros2_lap_analyzer_logs.txt`](docs/results/ros2_lap_analyzer_logs.txt). | ✅ measured (supplied by the student) |
| **Offline** | `tools/offline_benchmark.py`, which imports this repository's controller classes (PID, profiler, Lateral PID, Pure Pursuit, MPC) and the plant equations of `bicycle_model.py`, replays the 10 Hz loop for 3 complete laps per controller on `centerline_0.csv` at 4 m/s. No DDS latency, no timer jitter; each command is applied on the next tick. Raw data: `benchmark_offline.json`. | ✅ measured, used only as a cross-check (§6.4) |

Reproduce the offline numbers: `python3 tools/offline_benchmark.py --laps 3`.
Reproduce the ROS 2 numbers: launch a mode (§4), let it finish 3 laps, read the three banners (and `cat /tmp/lap_summary.json`).

### 6.2 Benchmark table

**ROS 2 simulation — 3 laps per controller (primary result).** Best lap is the fastest of the 3 laps and top speed the highest of the 3 per-lap top speeds. Mean/RMS CTE are the 3-lap aggregates *derived from the three banners* (weighted by lap time, since the analyzer samples at 10 Hz, so sample count ∝ lap time; RMS combined as $\sqrt{\sum w_i\,\mathrm{RMS}_i^{2}}$); max CTE is the largest per-lap maximum. CTE is $|{\rm CTE}|$ in metres.

| Controller mode | Best lap (s) | Top speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps completed / status |
|---|---|---|---|---|---|---|
| Manual teleoperation | — | — | — | — | — | not measured |
| Lateral PID (reactive) | 116.40 | 4.12 | 0.289 | 2.374 | 0.406 | 3 / OK *(ROS 2)* |
| Pure Pursuit (preview) | **112.01** | 4.12 | **0.056** | **0.382** | **0.085** | 3 / OK *(ROS 2)* |
| Extended kinematic MPC (optimal) | 119.70 | 4.09 | 0.079 | 0.411 | 0.103 | 3 / OK *(ROS 2)* |

**Per-lap values exactly as printed by the lap analyzer** (distance per lap = difference of the cumulative `Distance` values of consecutive banners, see §6.5):

| Controller | Lap | Lap time (s) | Mean CTE (m) | RMS CTE (m) | Max CTE (m) | Mean speed (m/s) | Top speed (m/s) | Distance in lap (m) |
|---|---|---|---|---|---|---|---|---|
| Lateral PID | 1 | 117.80 | 0.317 | 0.464 | 2.374 | 3.96 | 4.12 | 467.4 |
| | 2 | 116.90 | 0.291 | 0.395 | 1.775 | 3.98 | 4.00 | 464.8 |
| | 3 | 116.40 | 0.258 | 0.349 | 1.403 | 3.98 | 4.00 | 462.8 |
| Pure Pursuit | 1 | 112.30 | 0.054 | 0.083 | 0.352 | 3.97 | 4.12 | 445.7 |
| | 2 | 112.09 | 0.057 | 0.087 | 0.336 | 3.98 | 4.00 | 445.8 |
| | 3 | 112.01 | 0.057 | 0.086 | 0.382 | 3.98 | 4.00 | 445.4 |
| MPC | 1 | 120.70 | 0.078 | 0.101 | 0.398 | 3.69 | 4.07 | 445.4 |
| | 2 | 119.70 | 0.078 | 0.101 | 0.390 | 3.72 | 4.06 | 445.5 |
| | 3 | 119.80 | 0.080 | 0.108 | 0.411 | 3.72 | 4.09 | 445.7 |

Lap-time averages over the 3 laps: Lateral PID 117.03 s, Pure Pursuit 112.13 s, MPC 120.07 s. Aggregate mean speed (lap-time weighted): Lateral PID 3.97 m/s, Pure Pursuit 3.98 m/s, MPC 3.71 m/s. Mean distance per lap: Lateral PID 465.0 m, Pure Pursuit 445.6 m, MPC 445.5 m.

⚠️ The ROS 2 run did not log MPC solve time, so no ROS 2 compute figure exists. The only one is the offline value: mean 20.4 ms, maximum 83.0 ms per 100 ms tick.

### 6.3 Critical comparison

| Aspect | Lateral PID | Pure Pursuit | Extended kinematic MPC |
|---|---|---|---|
| Information used | present CTE and heading error | one look-ahead point | whole horizon + vehicle model |
| Preview | none | yes (one point) | yes (10 steps) |
| Actuator limits | clipped afterwards | clipped afterwards | inside the optimisation |
| Model needed | none | wheelbase only | full model (incl. drag, delay) |
| Tuning parameters | $K_p,K_i,K_d,K_\psi$ | $L_{d0},k_v,L_{min},L_{max}$ | weights, horizon, rate limits |
| Compute (offline only) | negligible | negligible | ≈20 ms mean, 83 ms max |
| **ROS 2:** best lap | 116.40 s | **112.01 s** | 119.70 s |
| **ROS 2:** RMS CTE | 0.406 m | **0.085 m** | 0.103 m |
| **ROS 2:** max CTE | 2.374 m | **0.382 m** | 0.411 m |
| **ROS 2:** mean speed | 3.97 m/s | 3.98 m/s | 3.71 m/s |
| **ROS 2:** distance per lap | ≈465.0 m | ≈445.6 m | ≈445.5 m |

**Which controller performed best?** According to the ROS 2 measurements, **Pure Pursuit is best on every reported metric**: fastest lap (112.01 s), lowest mean (0.056 m), RMS (0.085 m) and maximum (0.382 m) CTE. The ranking by tracking accuracy is Pure Pursuit < MPC < Lateral PID (lower is better); the ranking by lap time is Pure Pursuit (112.01 s) < Lateral PID (116.40 s) < MPC (119.70 s). The same orderings were obtained offline (§6.4).

* **Lateral PID** was clearly the least accurate: RMS CTE 0.406 m is about 4.8× that of Pure Pursuit and 3.9× that of MPC, and its worst lap peaked at 2.374 m (6.2× Pure Pursuit's worst). It also drove ≈19 m more per lap than the other two (465 vs 445.6 m) at almost the same mean speed (3.97 vs 3.98 m/s), which accounts for its ≈5 s slower lap than Pure Pursuit (distance ÷ mean speed ≈ 117 s). The design has no preview and can only react to an error that already exists. Over the three laps its errors fell (mean CTE 0.317 → 0.291 → 0.258 m, max 2.374 → 1.775 → 1.403 m) and its lap time dropped from 117.80 s to 116.40 s; the reason for this lap-to-lap improvement was not investigated (the integral gain is $K_i=0$), so it is reported as observed only.
* **Pure Pursuit** gave the lowest error and the fastest laps and was very repeatable (lap times within 0.29 s, RMS CTE 0.083–0.087 m). The simulated plant is a slip-free kinematic bicycle — the exact assumption behind the pure-pursuit arc — so it is close to ideal here. Its weaknesses (no actuator limits, no speed awareness, corner-cutting at large look-ahead) are not exercised by this plant.
* **MPC** tracked far better than Lateral PID (RMS 0.103 vs 0.406 m; max 0.411 vs 2.374 m) but **did not beat Pure Pursuit** (RMS 0.103 vs 0.085 m, i.e. about 21 % higher, max 0.411 vs 0.382 m). It drove the same distance per lap as Pure Pursuit (≈445.5 m) but at a lower mean speed (3.71 vs 3.98 m/s), which is why its laps were ≈7.7 s slower and the slowest of the three. This is consistent with the design: the MPC speed reference is limited to what the car can reach with bounded acceleration and speed is traded against tracking inside the cost. No experiment isolated that cause, and the weights are untuned, so this is the current result, not a ceiling.

### 6.4 Offline cross-check (ROS-free replay)

The earlier offline benchmark (3 laps, same track and 4 m/s target; mean/RMS/max CTE over all 3 laps, best lap = fastest of 3). It is **not** a ROS 2 measurement.

| Controller mode | Best lap (s) | Top speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps completed / status |
|---|---|---|---|---|---|---|
| Lateral PID (reactive) | 117.70 | 4.12 | 0.349 | 3.680 | 0.492 | 3 / OK *(offline)* |
| Pure Pursuit (preview) | 112.00 | 4.12 | 0.075 | 0.364 | 0.109 | 3 / OK *(offline)* |
| Extended kinematic MPC (optimal) | 119.50 | 4.05 | 0.093 | 0.437 | 0.123 | 3 / OK *(offline)* |

Individual offline lap times (s): Lateral PID 117.8 / 118.5 / 117.7 · Pure Pursuit 112.3 / 112.0 / 112.1 · MPC 120.7 / 119.5 / 119.7. MPC solve time in the offline run: mean 20.4 ms, maximum 83.0 ms per 100 ms tick.

**ROS 2 vs offline (same metric, same track):**

| Controller | Best lap ROS 2 / offline (s) | RMS CTE ROS 2 / offline (m) | Max CTE ROS 2 / offline (m) |
|---|---|---|---|
| Lateral PID | 116.40 / 117.70 | 0.406 / 0.492 | 2.374 / 3.680 |
| Pure Pursuit | 112.01 / 112.00 | 0.085 / 0.109 | 0.382 / 0.364 |
| MPC | 119.70 / 119.50 | 0.103 / 0.123 | 0.411 / 0.437 |

Lap times agree within 1.3 s for all three controllers and the controller ranking is identical. The ROS 2 mean/RMS CTE are lower than the offline values for all three controllers (RMS by 16–22 %); the reason was **not determined** (see §6.5). The ROS 2 numbers are the ones used for the conclusions above.

### 6.5 Data-quality notes and open points

1. **`Distance` in the banner is cumulative.** The analyzer never resets `total_distance`, so the lap 2 and lap 3 banners show roughly 2× and 3× the lap distance. The per-lap values in §6.2 are differences of consecutive banners.
2. **Perimeter vs driven distance (unresolved discrepancy, flagged).** The analyzer and `offline_benchmark.py` report a track length of 528.2 m, which is the sum of the 1,001 CSV segments plus the closing segment. Every controller that tracks the path closely drove only ≈445 m per lap (445.4–445.8 m for Pure Pursuit and MPC in ROS 2; 445.7 m for Pure Pursuit offline), and 4 m/s × 112 s is also ≈448 m. Re-measuring the CSV shows why: using every 2nd waypoint gives a polyline of 444.4 m (every 5th: 443.5 m), so the 528.2 m figure is inflated by waypoint-to-waypoint zig-zag in the file. The lap counter works on fractions of this length and registered three laps in every run, but the effect of the zig-zag on the CTE values (they are measured to the same polyline) was **not assessed**.
3. **Why ROS 2 CTE is lower than offline** is unknown. Differences between the two set-ups are listed in §6.1 (timing/latency model); no experiment was run to attribute the gap to any of them.
4. **Aggregates are derived.** The ROS 2 mean/RMS CTE in §6.2 were computed from per-lap banners using lap-time weights, because `/tmp/lap_summary.json` (which uses all samples directly) was not supplied. They may differ from that file in the last digit. Best lap, top speed and max CTE are exact (min / max of the printed values).
5. **Lap-1 top speed.** Lateral PID and Pure Pursuit show a top speed of 4.12 m/s in lap 1 only (4.00 m/s afterwards), i.e. a small overshoot of the 4 m/s target while accelerating from standstill; MPC peaks at 4.06–4.09 m/s in every lap. The cause was not examined.
6. **Screenshots.** Figures 1–2 (RViz2) and Figures 3–4 (`rqt_graph`) do not record which controller mode was running; they are illustrations, not part of the benchmark.

### 6.6 Assessment rubric — status

| Component (weight) | Evidence in this repository | Status |
|---|---|---|
| Kinematic model and integration (15 %) | `bicycle_model.py`: Euler, heading wrap, speed clamp | ✅ code; ⚠️ ROS test not run here |
| Teleop bridge and cruise integration (10 %) | `teleop_bridge.py`: mapping, 0.5 s watchdog, PID on `/state` | 🟡 |
| Longitudinal PID (15 %) | `longitudinal_pid.py`, anti-windup tests | ✅ tests; ⚠️ no speed-step plot supplied |
| M5.1 + M5.2 (15 %) | `path_utils.py`, `velocity_profiler.py`, `lateral_pid.py` | ✅ tests; ✅ 3 ROS 2 laps (Lateral PID) |
| M5.3 + M5.4 (20 %) | `pure_pursuit.py`, `mpc.py` | ✅ tests; ✅ 3 ROS 2 laps each; offline cross-check |
| Milestone 6 (10 %) | Ackermann analysis, resource synthesis | 🟡 Ackermann only; ⚠️ Gazebo/MPPI not run |
| Milestone 7 documentation and telemetry (10 %) | this README, analyzer topics and markers, ROS 2 lap banners | ✅ benchmark measured in ROS 2; ⚠️ `lap_summary.json` not supplied |
| Milestone 8 video (5 %) | — | https://drive.google.com/drive/folders/1H6tG2lg1RjPsa1RHcwUSzVcHeZ00JBR8?usp=drive_link |

---

## 7. Why MPC can track better than Pure Pursuit and Lateral PID

1. **Preview of the road and of the dynamics.** Lateral PID sees only the current error. Pure Pursuit uses one preview point but assumes the car instantly follows a constant-curvature arc. MPC predicts the entire horizon with the actual plant model (steering-rate limit, acceleration lag, drag, one tick of delay) and chooses all steering and acceleration inputs jointly, so it starts to steer early enough for the real response.
2. **Constraints inside the optimisation.** Steering angle, steering rate and acceleration limits are enforced by the optimiser. PID and Pure Pursuit only clip afterwards, which breaks their design assumptions (integrator windup, a different arc than the one computed).
3. **One cost for speed and steering.** MPC trades speed against accuracy in corners within one objective; the PID/Pure Pursuit cascade handles them separately.
4. **Explicit error weighting in the Frenet frame.** Cross-track error ($w_{lat}=30$) is penalised far more than longitudinal lag ($w_{lon}=1$), and re-solving every tick from a warm-started plan gives closed-loop correction.

These advantages grow when the plant has lag, limits or slip. On the slip-free kinematic plant used here, with untuned weights and a 1 s horizon (N = 10, Δt = 0.1 s), they did **not** overcome a well-tuned Pure Pursuit — neither in the ROS 2 runs (RMS CTE 0.103 m vs 0.085 m) nor in the offline cross-check (0.123 m vs 0.109 m); see §6.2–§6.3. The MPC's computational cost (≈20 ms mean per solve, measured offline) is the price paid.

---

## 8. Requirement status summary

| # | Requirement | Status |
|---|---|---|
| M1 | Topics/types/units identified; rqt_plot and PlotJuggler set up | 🟡 graph evidenced; plotting tools documented only |
| M2 | Extended kinematic model, Euler, wrap, clamp | ✅ |
| M3 | Teleop bridge, 0.5 s watchdog | 🟡 |
| M4 | Longitudinal PID + anti-windup + cruise integration | ✅ unit tests / ⚠️ no ROS speed plot |
| M5.1 | Curvature + curvature-limited profile | ✅ |
| M5.2 | Lateral PID, correct signs | ✅ unit tests / ✅ 3 ROS 2 laps |
| M5.3 | Pure Pursuit, adaptive look-ahead | ✅ unit tests / ✅ 3 ROS 2 laps |
| M5.4 | MPC: Frenet cost, receding horizon, warm start | ✅ unit test / ✅ 3 ROS 2 laps |
| M5.5 | Lap analyzer: timing, telemetry topics, RViz markers | ✅ lap banners from ROS 2 runs; HUD seen in screenshots; ⚠️ `lap_summary.json` not supplied |
| M6 | Free exploration | 🟡 Ackermann analysis; ⚠️ Gazebo/MPPI not run |
| M7 | README with ≥3-lap benchmark | ✅ ROS 2 benchmark (derived from lap banners) + offline cross-check |
| M8 | 3–5 min video | ⚠️ not done |

### 8.1 Known limitations

* The README author did **not** run ROS 2 (launch, node integration, RViz2, telemetry topics were checked by reading the code and by the student's screenshots/logs). The ROS 2 results are the student's own lap-analyzer output.
* Manual teleoperation has no benchmark entry.
* MPC weights are untuned. Its mean solve time was 20 ms (max 83 ms) in the offline harness; the Python implementation may not be real-time safe on a slow CPU. No solve-time measurement exists for the ROS 2 run.
* The ROS 2 3-lap aggregates are derived from per-lap banners; the controller mode of the RViz/`rqt_graph` screenshots was not recorded.
* The Gazebo variant is present but unverified, and Gazebo / MVSim / Nav2 MPPI were not run.

---

## 9. Troubleshooting

| Symptom | Likely cause and fix |
|---|---|
| `Package 'bicycle_sim' not found` | `source install/setup.bash` after `colcon build`; check that `source /opt/ros/jazzy/setup.bash` came first |
| RViz2 shows no robot | `robot_state_publisher` must be running; set Fixed Frame to `map`; check `ros2 topic echo /robot_description --once` |
| No cones/path in RViz | `/path_gen` failed to load the CSV: check the launch log; `track_file` must be a CSV in `track_environment/tracks` or an absolute path; rebuild after adding files |
| Car does not move in teleop | `ros2 topic echo /cmd_vel` — the keyboard node must publish `geometry_msgs/Twist` (Jazzy's `teleop_twist_keyboard` can publish `TwistStamped` if `stamped:=true`; keep the default). The bridge zeros commands after 0.5 s without messages, so keys must be repeated |
| Car cannot steer in teleop | Confirm `/steer` is being published (`ros2 topic hz /steer`); `angular.z` of 1.0 rad/s equals full lock |
| Controller does nothing | It waits for both `/state` and `/path`; check `ros2 topic hz /path /state` |
| Lap counter stays at 0 | The lap is counted when the arc position wraps while moving at >0.1 m/s; drive a full lap on the same track |
| Lap banner `Distance` looks too large for lap 2/3 | It is cumulative since launch (never reset); subtract the previous banner to get the per-lap distance |
| MPC log warnings / lag | MPC solve time up to ≈80 ms was seen offline (not measured in the ROS 2 run); close other CPU loads or reduce horizon/`maxiter` |
| `ModuleNotFoundError: scipy` | `sudo apt install python3-scipy` |
| `colcon build` warns about `setup.py install` deprecation | Harmless with `--symlink-install` on Jazzy; if tests fail to find modules, rebuild the affected package |
| rqt_plot / PlotJuggler not found | install `ros-jazzy-rqt-plot` / `ros-jazzy-plotjuggler-ros` (names may vary — check with `apt-cache search plotjuggler`) |

---

## 10. Repository layout and submission checklist

```
Control_Project/
├── README.md
├── assets/demo.gif
├── docs/images/{rviz_lap1_start,rviz_lap1_mid,rqt_graph_teleop,rqt_graph_controller}.png
├── docs/results/ros2_lap_analyzer_logs.txt
├── benchmark_offline.json
├── tools/offline_benchmark.py
├── bicycle_sim/       {bicycle_sim/{bicycle_model,sim_node,gz_adapter}.py, urdf/, config/, launch/}
├── bicycle_control/   {bicycle_control/{teleop_bridge,longitudinal_pid,velocity_profiler,lateral_pid,pure_pursuit,mpc,path_utils,controller_node}.py, test/}
└── track_environment/ {track_environment/{track,path_gen,lap_analyzer}.py, tracks/, test/}
```

Submission checklist (from the Project Description):

- [x] `README.md` at the repository root *(this document is delivered as `README3.md`; rename it to `README.md` when submitting)*
- [x] Source files `bicycle_model.py`, `teleop_bridge.py`, `longitudinal_pid.py`, `lateral_pid.py`, `pure_pursuit.py`, `mpc.py`, `lap_analyzer.py`
- [x] ROS 2 Jazzy benchmark (3 laps per controller) — from the `lap_analyzer` banners ([§6.2](#62-benchmark-table), raw text in `docs/results/`); ⚠️ attach `/tmp/lap_summary.json` of each run if the course requires the file itself
- [ ] 3–5 minute video (all four modes, telemetry display)
- [ ] Submission form: https://forms.gle/8LDeRLjurrWSKBTa7

Tests that need no ROS: `cd bicycle_control && PYTHONPATH=. python3 -m pytest test/test_pure_pursuit.py test/test_longitudinal_pid.py test/test_lateral_pid.py test/test_velocity_profiler.py test/test_mpc.py test/test_path_utils.py test/test_pid_antiwindup.py` (14 passed in the environment where they were run) and `cd track_environment && PYTHONPATH=. python3 -m pytest test/test_track.py` (3 passed).

*Documentation sources and corrections:* this README follows the layout of `README2`; implementation details were merged from `README1` (the README shipped in the project ZIP). `README1` still said "source /opt/ros/humble" and `ros-humble-*` packages in its quick-start commands; this document uses **ROS 2 Jazzy** throughout, as stated by the student and shown by the setup in §2.

*Reference:* the vehicle model and MPC/LQR/PID comparison background are in the student's study report `Control_Task_Report_ARL2027.pdf` (Varma, Swamy & Mukherjee, ICSTCEE 2020); its numbers belong to a different vehicle and test and are **not** used as benchmarks here.
