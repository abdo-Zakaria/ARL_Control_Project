"""
Lap Analyzer Node:
Performance evaluation, real-time telemetry, lap timing, and RViz HUD visualization.
Decoupled observer monitoring /path and /state to compute cross-track error, heading error,
lap times, and dynamic metrics.
"""

import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class LapAnalyzer(Node):
    def __init__(self):
        super().__init__('lap_analyzer')
        self.get_logger().info('Initializing Lap Analyzer Node...')

        # Subscriptions
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.state_sub = self.create_subscription(Odometry, '/state', self.state_callback, 10)

        # Publishers
        self.metrics_pub = self.create_publisher(String, '/lap/metrics', 10)
        self.viz_pub = self.create_publisher(MarkerArray, '/lap/visualization', 10)

        # Standardized Plottable Telemetry Publishers (for rqt_plot & PlotJuggler)
        self.cte_pub = self.create_publisher(Float32, '/telemetry/cte', 10)
        self.speed_pub = self.create_publisher(Float32, '/telemetry/speed', 10)
        self.heading_err_pub = self.create_publisher(
            Float32, '/telemetry/heading_err_deg', 10
        )
        self.lap_time_pub = self.create_publisher(Float32, '/telemetry/lap_time', 10)

        # Path storage
        self.path_points = []  # [(x, y, psi)]
        self.path_cum_dist = []
        self.track_length = 0.0
        self.path_received = False

        # State & Timing
        self.start_sim_time = None
        self.last_state_time = None
        self.lap_start_time = None
        self.has_started = False  # lap clock is held at 0 until the car first moves

        # Lap Tracking
        self.lap_count = 0
        self.last_s = 0.0
        self.total_distance = 0.0
        self.last_xy = None

        # Lap Times
        self.current_lap_time = 0.0
        self.last_lap_time = None
        self.best_lap_time = None
        self.lap_times = []

        # Error & Speed Statistics (Per Lap)
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

        # Global Statistics
        self.global_ctes = []
        self.global_max_speed = 0.0
        self.lap_stats = []
        self.declare_parameter('summary_file', '/tmp/lap_summary.json')
        self.summary_file = str(self.get_parameter('summary_file').value)

        # Current live metrics
        self.current_cte = 0.0
        self.current_heading_err = 0.0
        self.current_speed = 0.0
        self.proj_xy = (0.0, 0.0)

        # Publish periodic summary and HUD at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_telemetry)

    def path_callback(self, msg: Path):
        """Processes received path and precomputes cumulative distance."""
        if self.path_received and len(msg.poses) == len(self.path_points):
            return  # Path already loaded and unchanged

        pts = []
        for p in msg.poses:
            x = p.pose.position.x
            y = p.pose.position.y
            # Extract yaw from quaternion
            qz = p.pose.orientation.z
            qw = p.pose.orientation.w
            yaw = 2.0 * math.atan2(qz, qw)
            pts.append((x, y, yaw))

        if len(pts) < 2:
            return

        self.path_points = pts
        # Compute cumulative distance
        cum = [0.0]
        for i in range(1, len(pts)):
            d = math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
            cum.append(cum[-1] + d)

        self.path_cum_dist = cum
        self.track_length = cum[-1]
        self.path_received = True
        self.get_logger().info(
            f"Lap Analyzer: Loaded path with {len(pts)} waypoints, "
            f"perimeter: {self.track_length:.2f} m"
        )

    def state_callback(self, msg: Odometry):
        """Processes vehicle odometry and updates progress, lap timing, and errors."""
        now_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.start_sim_time is None:
            self.start_sim_time = now_sec
            self.lap_start_time = now_sec

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        yaw = 2.0 * math.atan2(qz, qw)
        v = msg.twist.twist.linear.x

        self.current_speed = v
                # Hold the lap clock at zero until the car actually starts moving
        if not self.has_started and v < 0.1:
            self.lap_start_time = now_sec
        else:
            self.has_started = True
        
        self.global_max_speed = max(self.global_max_speed, v)

        # Distance traveled
        if self.last_xy is not None:
            step_d = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
            self.total_distance += step_d
        self.last_xy = (x, y)

        if not self.path_received or len(self.path_points) < 2:
            return

        # Find nearest point & projection
        proj_x, proj_y, s, cte, heading_err = self.project_to_path(x, y, yaw)
        self.proj_xy = (proj_x, proj_y)
        self.current_cte = cte
        self.current_heading_err = heading_err

        # Accumulate metrics
        abs_cte = abs(cte)
        self.lap_ctes.append(abs_cte)
        self.lap_heading_errors.append(abs(heading_err))
        self.lap_speeds.append(v)
        self.global_ctes.append(abs_cte)

        self.current_lap_time = now_sec - self.lap_start_time

        # Lap Crossing Detection (s wrapped around track_length while moving forward).
        # e.g., last_s near end (> 70% length) and current s near start (< 30% length).
        if self.track_length > 5.0 and v > 0.1:
            if self.last_s > 0.75 * self.track_length and s < 0.25 * self.track_length:
                # Sub-tick lap time interpolation
                ds_total = (self.track_length - self.last_s) + s
                dt_step = max(now_sec - (self.last_state_time or now_sec), 1e-4)
                frac = (self.track_length - self.last_s) / max(ds_total, 1e-4)
                t_crossing = (self.last_state_time or now_sec) + frac * dt_step

                lap_duration = t_crossing - self.lap_start_time
                self.record_lap_completion(lap_duration, now_sec)
                self.lap_start_time = t_crossing

        self.last_s = s
        self.last_state_time = now_sec

    def project_to_path(self, x, y, yaw):
        """Finds closest segment and projects (x, y) to compute exact orthogonal CTE."""
        pts = self.path_points
        n = len(pts)

        # 1. Find nearest waypoint
        min_dist_sq = float('inf')
        nearest_idx = 0
        for i in range(n):
            dx = pts[i][0] - x
            dy = pts[i][1] - y
            d_sq = dx * dx + dy * dy
            if d_sq < min_dist_sq:
                min_dist_sq = d_sq
                nearest_idx = i

        # 2. Check candidate segments: (prev, nearest) and (nearest, next)
        best_dist = float('inf')
        best_proj = (pts[nearest_idx][0], pts[nearest_idx][1])
        best_s = self.path_cum_dist[nearest_idx]
        best_seg_yaw = pts[nearest_idx][2]
        best_signed_cte = 0.0

        candidate_segments = [
            ((nearest_idx - 1) % n, nearest_idx),
            (nearest_idx, (nearest_idx + 1) % n)
        ]
        for prev_i, next_i in candidate_segments:
            x1, y1, yaw1 = pts[prev_i]
            x2, y2, _ = pts[next_i]
            dx = x2 - x1
            dy = y2 - y1
            seg_len_sq = dx * dx + dy * dy
            if seg_len_sq < 1e-6:
                continue

            t = max(0.0, min(1.0, ((x - x1) * dx + (y - y1) * dy) / seg_len_sq))
            px = x1 + t * dx
            py = y1 + t * dy
            dist = math.hypot(x - px, y - py)

            if dist < best_dist:
                best_dist = dist
                best_proj = (px, py)
                best_s = self.path_cum_dist[prev_i] + t * math.sqrt(seg_len_sq)
                best_seg_yaw = math.atan2(dy, dx)

                # Signed cross track error: positive if car is to the left of path
                cross = dx * (y - y1) - dy * (x - x1)
                best_signed_cte = math.copysign(dist, cross)

        # Heading error in [-pi, pi]
        heading_err = math.atan2(math.sin(yaw - best_seg_yaw), math.cos(yaw - best_seg_yaw))

        return best_proj[0], best_proj[1], best_s, best_signed_cte, heading_err

    def record_lap_completion(self, lap_duration, now_sec):
        """Records finished lap and prints summary."""
        self.lap_count += 1
        self.last_lap_time = lap_duration
        self.lap_times.append(lap_duration)

        if self.best_lap_time is None or lap_duration < self.best_lap_time:
            self.best_lap_time = lap_duration

        ctes = np.asarray(self.lap_ctes, dtype=float)
        speeds = np.asarray(self.lap_speeds, dtype=float)
        heads = np.asarray(self.lap_heading_errors, dtype=float)
        stats = {
            'lap': self.lap_count,
            'lap_time': float(lap_duration),
            'mean_cte': float(ctes.mean()) if ctes.size else 0.0,
            'max_cte': float(ctes.max()) if ctes.size else 0.0,
            'rms_cte': float(np.sqrt(np.mean(ctes ** 2))) if ctes.size else 0.0,
            'mean_heading_err_deg': float(np.degrees(heads.mean())) if heads.size else 0.0,
            'mean_speed': float(speeds.mean()) if speeds.size else 0.0,
            'max_speed': float(speeds.max()) if speeds.size else 0.0,
        }
        self.lap_stats.append(stats)
        self.get_logger().info(
            '\n==================== LAP %d COMPLETE ====================\n'
            '  Lap time : %.2f s   (best %.2f s)\n'
            '  CTE      : mean %.3f m | RMS %.3f m | max %.3f m\n'
            '  Speed    : mean %.2f m/s | top %.2f m/s\n'
            '  Distance : %.1f m\n'
            '=========================================================' % (
                self.lap_count, lap_duration, self.best_lap_time,
                stats['mean_cte'], stats['rms_cte'], stats['max_cte'],
                stats['mean_speed'], stats['max_speed'], self.total_distance))
        self.write_summary()

        # Reset per-lap buffers so the next lap starts fresh
        self.lap_ctes = []
        self.lap_heading_errors = []
        self.lap_speeds = []

    def write_summary(self):
        """Dumps all completed-lap statistics to JSON (benchmark source for the README)."""
        summary = {
            'laps_completed': self.lap_count,
            'best_lap_time': self.best_lap_time,
            'top_speed': self.global_max_speed,
            'mean_cte': float(np.mean(self.global_ctes)) if self.global_ctes else 0.0,
            'max_cte': float(np.max(self.global_ctes)) if self.global_ctes else 0.0,
            'rms_cte': (float(np.sqrt(np.mean(np.square(self.global_ctes))))
                        if self.global_ctes else 0.0),
            'laps': self.lap_stats,
        }
        try:
            with open(self.summary_file, 'w') as f:
                json.dump(summary, f, indent=2)
        except OSError as exc:
            self.get_logger().warn(f'Could not write {self.summary_file}: {exc}',
                                   throttle_duration_sec=10.0)

    def publish_telemetry(self):
        """Periodically publishes numerical telemetry and RViz visual markers at 10 Hz."""
        # Baseline start-gate visualization hook
        # 1. Plottable Float32 signals (rqt_plot / PlotJuggler)
        cte_msg = Float32()
        cte_msg.data = float(self.current_cte)
        self.cte_pub.publish(cte_msg)

        speed_msg = Float32()
        speed_msg.data = float(self.current_speed)
        self.speed_pub.publish(speed_msg)

        heading_msg = Float32()
        heading_msg.data = float(math.degrees(self.current_heading_err))
        self.heading_err_pub.publish(heading_msg)

        lap_time_msg = Float32()
        lap_time_msg.data = float(self.current_lap_time)
        self.lap_time_pub.publish(lap_time_msg)

        # 2. JSON summary on /lap/metrics
        rms_cte = (
            float(np.sqrt(np.mean(np.square(self.lap_ctes)))) if self.lap_ctes else 0.0
        )
        elapsed = (
            (self.last_state_time - self.start_sim_time)
            if self.last_state_time is not None and self.start_sim_time is not None else 0.0
        )
        telemetry = {
            'lap': self.lap_count,
            'current_lap_time': self.current_lap_time,
            'elapsed_time': elapsed,
            'last_lap_time': self.last_lap_time,
            'best_lap_time': self.best_lap_time,
            'speed': self.current_speed,
            'current_cte': self.current_cte,
            'rms_cte': rms_cte,
            'heading_err_deg': math.degrees(self.current_heading_err),
        }
        metrics_msg = String()
        metrics_msg.data = json.dumps(telemetry)
        self.metrics_pub.publish(metrics_msg)

        # 3. RViz markers (start gate + CTE whisker + HUD)
        self.publish_rviz_markers(telemetry)

    def publish_rviz_markers(self, telemetry=None):
        """Renders start gate, error whisker, and on-screen HUD text in RViz."""
        ma = MarkerArray()
        now = self.get_clock().now().to_msg()

        # ----------------------------------------------------------------------
        # Marker 1: Start/Finish Gate Line (Provided for Track Origin Reference)
        # ----------------------------------------------------------------------
        if self.path_points:
            p0 = self.path_points[0]
            gate = Marker()
            gate.header.frame_id = 'map'
            gate.header.stamp = now
            gate.ns = 'start_gate'
            gate.id = 0
            gate.type = Marker.CYLINDER
            gate.action = Marker.ADD
            gate.pose.position.x = p0[0]
            gate.pose.position.y = p0[1]
            gate.pose.position.z = 0.5
            gate.pose.orientation.w = 1.0
            gate.scale.x = 0.1
            gate.scale.y = 1.2
            gate.scale.z = 1.0
            gate.color.r = 0.1
            gate.color.g = 0.9
            gate.color.b = 0.2
            gate.color.a = 0.7
            ma.markers.append(gate)

        t = telemetry or {}
        cte = t.get('current_cte', self.current_cte)
        speed = t.get('speed', self.current_speed)

        # ----------------------------------------------------------------------
        # Marker 2: Cross-track error whisker (rear axle -> projected path point)
        # ----------------------------------------------------------------------
        if self.path_received and self.last_xy is not None:
            sev = min(abs(cte) / 1.0, 1.0)  # 0 m = green, >= 1 m = red
            whisker = Marker()
            whisker.header.frame_id = 'map'
            whisker.header.stamp = now
            whisker.ns = 'cte_whisker'
            whisker.id = 1
            whisker.type = Marker.LINE_STRIP
            whisker.action = Marker.ADD
            whisker.pose.orientation.w = 1.0
            whisker.scale.x = 0.08
            whisker.color.r = float(sev)
            whisker.color.g = float(1.0 - sev)
            whisker.color.b = 0.0
            whisker.color.a = 1.0
            for px, py in (self.last_xy, self.proj_xy):
                pt = Point()
                pt.x = float(px)
                pt.y = float(py)
                pt.z = 0.1
                whisker.points.append(pt)
            ma.markers.append(whisker)

        # ----------------------------------------------------------------------
        # Marker 3: 3D HUD scoreboard floating above the vehicle
        # ----------------------------------------------------------------------
        best = t.get('best_lap_time', self.best_lap_time)
        lap_t = t.get('current_lap_time', self.current_lap_time)
        hud_xy = self.last_xy if self.last_xy is not None else (0.0, 0.0)
        hud = Marker()
        hud.header.frame_id = 'map'
        hud.header.stamp = now
        hud.ns = 'lap_hud'
        hud.id = 2
        hud.type = Marker.TEXT_VIEW_FACING
        hud.action = Marker.ADD
        hud.pose.position.x = float(hud_xy[0])
        hud.pose.position.y = float(hud_xy[1])
        hud.pose.position.z = 2.5
        hud.pose.orientation.w = 1.0
        hud.scale.z = 0.7
        hud.color.r = 1.0
        hud.color.g = 1.0
        hud.color.b = 1.0
        hud.color.a = 1.0
        hud.text = (
            f"LAP {self.lap_count + 1}   {lap_t:6.2f} s\n"
            f"BEST  {'--' if best is None else f'{best:.2f} s'}\n"
            f"SPEED {speed:5.2f} m/s\n"
            f"CTE   {cte:+.2f} m"
        )
        ma.markers.append(hud)
        self.viz_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    analyzer = LapAnalyzer()
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
