#!/usr/bin/env python3
"""
Waypoint Recorder Perfect v3.1 - Multi-Lap Support (FSDS)

- Records waypoints for multiple laps WITHOUT overlap
- Uses hysteresis state machine (SEARCHING → IN_LAP → SEARCHING)
- Publishes visualization to RViz2:
  - /recorded_waypoints (MarkerArray)
  - /recorded_path (nav_msgs/Path)
- Saves to: /workspace/ros2_ws/waypoints.json

Fixes:
- Subscribe to /testing_only/odom using BEST_EFFORT QoS (common for simulator/sensor topics).
- Optional: snap-close loop by appending the start waypoint at the end.
"""

import json
import math
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray, Marker


def yaw_from_quaternion(quat) -> float:
    """Extract yaw from quaternion."""
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class WaypointRecorderPerfect(Node):
    """Record clean waypoints for multiple laps without overlap."""

    def __init__(self):
        super().__init__("waypoint_recorder_perfect")

        # ---------- Parameters ----------
        self.declare_parameter("odom_topic", "/testing_only/odom")
        self.declare_parameter("frame_id", "fsds/FSCar")

        self.declare_parameter("lap_distance_threshold", 1.5)        # meters
        self.declare_parameter("min_waypoints_before_close", 150)    # points
        self.declare_parameter("distance_before_in_lap", 10.0)       # meters
        self.declare_parameter("waypoint_spacing", 0.2)              # meters

        self.declare_parameter("snap_close_loop", True)              # append start at end
        self.declare_parameter("output_file", "/workspace/ros2_ws/waypoints.json")

        self.odom_topic = self.get_parameter("odom_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.lap_distance_threshold = float(self.get_parameter("lap_distance_threshold").value)
        self.min_waypoints_before_close = int(self.get_parameter("min_waypoints_before_close").value)
        self.distance_before_in_lap = float(self.get_parameter("distance_before_in_lap").value)
        self.waypoint_spacing = float(self.get_parameter("waypoint_spacing").value)

        self.snap_close_loop = bool(self.get_parameter("snap_close_loop").value)
        self.output_file = self.get_parameter("output_file").value

        # ---------- State ----------
        self.waypoints: List[List[float]] = []
        self.start_x: Optional[float] = None
        self.start_y: Optional[float] = None

        self.lap_state = "SEARCHING"  # SEARCHING or IN_LAP
        self.recording_active = True

        self.current_pose = None
        self.current_yaw = 0.0
        self.last_odom_time = None

        # ---------- Publishers ----------
        self.path_pub = self.create_publisher(Path, "/recorded_path", 10)
        self.markers_pub = self.create_publisher(MarkerArray, "/recorded_waypoints", 10)

        # ---------- Subscriber QoS (IMPORTANT) ----------
        # If publisher is BEST_EFFORT and subscriber is RELIABLE => incompatible. [web:16]
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            sensor_qos,
        )

        # ---------- Timers ----------
        self.timer = self.create_timer(0.1, self.record_waypoint)      # 10 Hz
        self.status_timer = self.create_timer(2.0, self.status_check)  # 0.5 Hz

        # ---------- Logging ----------
        self.get_logger().info("=" * 80)
        self.get_logger().info("🎯 WAYPOINT RECORDER PERFECT v3.1")
        self.get_logger().info("=" * 80)
        self.get_logger().info(f"Odometry topic: {self.odom_topic}")
        self.get_logger().info(f"Frame id: {self.frame_id}")
        self.get_logger().info(f"Waypoint spacing: {self.waypoint_spacing} m")
        self.get_logger().info(f"Close threshold: {self.lap_distance_threshold} m")
        self.get_logger().info(f"Enter IN_LAP after: {self.distance_before_in_lap} m travel")
        self.get_logger().info(f"Min waypoints before close: {self.min_waypoints_before_close}")
        self.get_logger().info(f"Snap close loop: {self.snap_close_loop}")
        self.get_logger().info(f"Output: {self.output_file}")
        self.get_logger().info("=" * 80)
        self.get_logger().info("RViz2: Fixed Frame should be 'fsds/FSCar' (or provide TF).")
        self.get_logger().info("Add displays: MarkerArray(/recorded_waypoints), Path(/recorded_path).")

    def odom_callback(self, msg: Odometry):
        self.current_pose = msg.pose.pose
        self.current_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.last_odom_time = self.get_clock().now()

    def status_check(self):
        # Helpful warning if nothing is happening
        if self.last_odom_time is None:
            self.get_logger().warn(
                f"No odometry received yet from {self.odom_topic}. "
                f"If topics exist but recorder publishes nothing, check QoS: "
                f"`ros2 topic info -v {self.odom_topic}`"
            )

    def distance_to_start(self) -> float:
        if self.start_x is None or self.current_pose is None:
            return float("inf")
        dx = self.current_pose.position.x - self.start_x
        dy = self.current_pose.position.y - self.start_y
        return math.hypot(dx, dy)

    def distance_traveled(self) -> float:
        # Conservative estimate; enough to trigger leaving the start area
        if self.start_x is None or self.current_pose is None:
            return 0.0
        dx = self.current_pose.position.x - self.start_x
        dy = self.current_pose.position.y - self.start_y
        return abs(dx) + abs(dy)

    def record_waypoint(self):
        if self.current_pose is None or not self.recording_active:
            return

        x = float(self.current_pose.position.x)
        y = float(self.current_pose.position.y)

        # First waypoint sets start position
        if self.start_x is None:
            self.start_x = x
            self.start_y = y
            self.waypoints.append([x, y])
            self.get_logger().info(f"✅ START recorded: ({x:.2f}, {y:.2f})")
            self.get_logger().info("State: SEARCHING → waiting to travel away from start")
            self.publish_visualization()
            return

        dist_traveled = self.distance_traveled()
        dist_to_start = self.distance_to_start()

        # State machine
        if self.lap_state == "SEARCHING":
            if dist_traveled > self.distance_before_in_lap:
                self.lap_state = "IN_LAP"
                self.get_logger().info(f"✅ ENTERED IN_LAP (traveled {dist_traveled:.1f}m)")
            return  # do not record points while SEARCHING

        # IN_LAP: check closure
        if (
            len(self.waypoints) > self.min_waypoints_before_close
            and dist_to_start < self.lap_distance_threshold
        ):
            self.get_logger().info(f"\n🏁 LOOP DETECTED! dist_to_start={dist_to_start:.2f}m")
            self.get_logger().info(f"✅ Recorded {len(self.waypoints)} waypoints so far")

            if self.snap_close_loop and (self.start_x is not None):
                # Make the saved loop explicitly closed
                self.waypoints.append([float(self.start_x), float(self.start_y)])

            self.recording_active = False
            self.save_waypoints()
            self.timer.cancel()
            return

        # Add waypoint if far enough from last point
        last = self.waypoints[-1]
        if math.hypot(x - last[0], y - last[1]) > self.waypoint_spacing:
            self.waypoints.append([x, y])
            self.publish_visualization()

            if len(self.waypoints) % 20 == 0:
                self.get_logger().info(
                    f"📍 Recorded {len(self.waypoints)} waypoints | "
                    f"Dist to start: {dist_to_start:.2f}m | "
                    f"State: IN_LAP"
                )

    def publish_visualization(self):
        now = self.get_clock().now().to_msg()

        # Markers
        markers = MarkerArray()
        for i, (wx, wy) in enumerate(self.waypoints):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp = now
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.1
            m.scale.x = m.scale.y = m.scale.z = 0.15

            if i == 0:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0  # start = red
            elif self.start_x is not None and math.hypot(wx - self.start_x, wy - self.start_y) < 2.0:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0  # near start = yellow
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 1.0, 0.0  # normal = green
            m.color.a = 0.8
            markers.markers.append(m)

        # Start target zone (cyan disk)
        if self.start_x is not None:
            target = Marker()
            target.header.frame_id = self.frame_id
            target.header.stamp = now
            target.id = 9999
            target.type = Marker.CYLINDER
            target.action = Marker.ADD
            target.pose.position.x = float(self.start_x)
            target.pose.position.y = float(self.start_y)
            target.pose.position.z = 0.2
            target.scale.x = target.scale.y = 2.0
            target.scale.z = 0.1
            target.color.r, target.color.g, target.color.b = 0.0, 1.0, 1.0
            target.color.a = 0.3
            markers.markers.append(target)

        self.markers_pub.publish(markers)

        # Path
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = now
        for wx, wy in self.waypoints:
            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = float(wx)
            p.pose.position.y = float(wy)
            path.poses.append(p)
        self.path_pub.publish(path)

    def save_waypoints(self):
        try:
            with open(self.output_file, "w") as f:
                json.dump(self.waypoints, f, indent=2)

            if len(self.waypoints) >= 2:
                gap = math.hypot(
                    self.waypoints[-1][0] - self.waypoints[0][0],
                    self.waypoints[-1][1] - self.waypoints[0][1],
                )
            else:
                gap = float("inf")

            self.get_logger().info(f"💾 SAVED: {self.output_file}")
            self.get_logger().info(f"✅ Waypoints: {len(self.waypoints)}")
            self.get_logger().info(f"📏 Endpoint gap (last→first): {gap:.3f} m")

        except Exception as e:
            self.get_logger().error(f"❌ Save failed: {e}")

    def destroy_node(self):
        # If user stops early, still save what was recorded
        if self.recording_active and self.waypoints:
            self.get_logger().info("🛑 Recorder shutting down early; saving partial waypoints.")
            self.save_waypoints()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderPerfect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\n🛑 Interrupted by user.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

