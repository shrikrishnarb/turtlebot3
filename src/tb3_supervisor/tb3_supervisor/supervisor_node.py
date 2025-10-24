#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SupervisorNode: safety supervision between Nav2 and base controller.

- Listens to Nav2 velocity (/cmd_vel_raw) and lidar (/scan).
- Enforces distance-based slow/stop in a configurable frontal field of view.
- FSM to handle blocking:
    PASS_THROUGH / SLOW       -> normal behavior
    BLOCKED_WAIT              -> stop linear, hold still for N seconds
    UNBLOCK_ROTATE            -> rotate deterministically to clear the path
- Publishes safe velocity to base controller topic (/cmd_vel).
"""

from typing import Optional
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class SupervisorNode(Node):
    def __init__(self):
        super().__init__('tb3_safety_supervisor')

        # ---------------- Parameters ----------------
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('input_cmd_topic', '/cmd_vel_raw')
        self.declare_parameter('output_cmd_topic', '/cmd_vel')

        self.declare_parameter('slow_down_range', 0.60)   # m
        self.declare_parameter('stop_range', 0.30)        # m
        self.declare_parameter('fov_deg', 60.0)           # forward ± fov/2
        self.declare_parameter('slow_scale_min', 0.2)

        self.declare_parameter('cmd_timeout_sec', 0.50)
        self.declare_parameter('control_rate_hz', 20)

        # Scale angular in slow zone (optional)
        self.declare_parameter('scale_angular', False)
        self.declare_parameter('angular_scale_min', 0.4)

        # --------- Unblocking & hysteresis parameters ----------
        self.declare_parameter('unblock_wait_sec', 3.0)          # wait before rotating
        self.declare_parameter('release_margin', 0.05)           # resume when >= stop+margin
        self.declare_parameter('unblock_min_rotate_sec', 1.5)    # hold rotation at least this long
        self.declare_parameter('unblock_max_rotate_sec', 8.0)    # rotate no longer than this
        self.declare_parameter('unblock_rotate_speed_max', 0.40) # rad/s clamp
        self.declare_parameter('unblock_rotate_speed_min', 0.20) # rad/s minimum
        self.declare_parameter('nav2_ang_threshold', 0.05)       # rad/s to trust Nav2 angular sign

        # Sector-based direction probe (fallback if Nav2 angular ~0)
        self.declare_parameter('side_sector_center_deg', 45.0)   # ± center for L/R sectors
        self.declare_parameter('side_sector_half_deg', 25.0)     # half width of L/R sectors

        # Read params
        gp = self.get_parameter
        self.scan_topic       = gp('scan_topic').value
        self.input_cmd_topic  = gp('input_cmd_topic').value
        self.output_cmd_topic = gp('output_cmd_topic').value

        self.slow_down_range  = gp('slow_down_range').value
        self.stop_range       = gp('stop_range').value
        self.fov_deg          = gp('fov_deg').value
        self.slow_scale_min   = gp('slow_scale_min').value

        self.cmd_timeout_sec  = gp('cmd_timeout_sec').value
        self.control_rate_hz  = int(gp('control_rate_hz').value)

        self.scale_angular    = gp('scale_angular').value
        self.angular_scale_min = gp('angular_scale_min').value

        self.unblock_wait_sec = gp('unblock_wait_sec').value
        self.release_margin   = gp('release_margin').value
        self.unblock_min_rotate_sec = gp('unblock_min_rotate_sec').value
        self.unblock_max_rotate_sec = gp('unblock_max_rotate_sec').value
        self.unblock_rotate_speed_max = gp('unblock_rotate_speed_max').value
        self.unblock_rotate_speed_min = gp('unblock_rotate_speed_min').value
        self.nav2_ang_threshold = gp('nav2_ang_threshold').value

        self.side_center_deg  = gp('side_sector_center_deg').value
        self.side_half_deg    = gp('side_sector_half_deg').value

        # Derived hysteresis threshold
        self.hysteresis_release = self.stop_range + self.release_margin

        # ---------------- State ----------------
        self._last_cmd: Optional[Twist] = None
        self._last_cmd_time = self.get_clock().now()
        self._last_min_front: Optional[float] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_raw_angular: float = 0.0

        # FSM
        self._state = 'PASS'               # PASS | SLOW | BLOCKED_WAIT | UNBLOCK_ROTATE
        self._blocked_since = None         # rclpy.time.Time
        self._unblock_started = None       # rclpy.time.Time
        self._unblock_dir = 0              # -1 (CW) | +1 (CCW)
        self._unblock_speed = 0.0

        # ---------------- QoS & ROS I/O ----------------
        qos_scan = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_cmd  = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                              reliability=ReliabilityPolicy.RELIABLE)

        self.create_subscription(LaserScan, self.scan_topic, self._on_scan, qos_scan)
        self.create_subscription(Twist, self.input_cmd_topic, self._on_cmd, qos_cmd)
        self._pub = self.create_publisher(Twist, self.output_cmd_topic, qos_cmd)

        self._timer = self.create_timer(1.0 / self.control_rate_hz, self._control_loop)

        self.get_logger().info(
            f"Supervisor: slow≤{self.slow_down_range:.2f}m, stop≤{self.stop_range:.2f}m, "
            f"fov={self.fov_deg}°, hysteresis={self.hysteresis_release:.2f}m; "
            f"unblock wait={self.unblock_wait_sec:.1f}s, rotate=[{self.unblock_rotate_speed_min:.2f}, "
            f"{self.unblock_rotate_speed_max:.2f}] rad/s"
        )

    # ---------------- Callbacks ----------------
    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg
        self._last_min_front = self._min_in_sector(msg, 0.0, self.fov_deg/2.0)

    def _on_cmd(self, msg: Twist):
        self._last_cmd = msg
        self._last_cmd_time = self.get_clock().now()
        self._last_raw_angular = msg.angular.z

    # ---------------- Helpers ----------------
    def _min_in_sector(self, scan: LaserScan, center_deg: float, half_deg: float) -> Optional[float]:
        """Minimum range in a sector [center_deg ± half_deg]. Degrees, 0 = forward."""
        if scan is None:
            return None
        c = math.radians(center_deg)
        h = math.radians(half_deg)
        start = c - h
        end   = c + h

        def angle_to_index(theta: float) -> int:
            idx = int(round((theta - scan.angle_min) / scan.angle_increment))
            return max(0, min(len(scan.ranges) - 1, idx))

        i0 = angle_to_index(start)
        i1 = angle_to_index(end)
        if i0 > i1:
            i0, i1 = i1, i0

        vals = [r for r in scan.ranges[i0:i1+1] if math.isfinite(r)]
        if not vals:
            return None
        return min(vals)

    def _pick_unblock_direction(self) -> int:
        """Choose rotation direction: prefer Nav2 angular sign; else choose side with more clearance."""
        # If Nav2 has a clear angular intent, trust it.
        if abs(self._last_raw_angular) >= self.nav2_ang_threshold:
            return 1 if self._last_raw_angular > 0.0 else -1

        # Else probe left/right sectors and choose where clearance is larger
        left  = self._min_in_sector(self._last_scan,  +self.side_center_deg, self.side_half_deg)
        right = self._min_in_sector(self._last_scan,  -self.side_center_deg, self.side_half_deg)

        # Default to CCW if equal/None
        if left is None and right is None:
            return 1
        if right is None:
            return 1
        if left is None:
            return -1
        return 1 if (left >= right) else -1

    # ---------------- Core loop ----------------
    def _control_loop(self):
        now = self.get_clock().now()

        # Fail-safe: no recent cmd -> publish zero
        if self._last_cmd is None or (now - self._last_cmd_time) > Duration(seconds=self.cmd_timeout_sec):
            self._pub.publish(Twist())
            return

        cmd = self._last_cmd
        min_front = self._last_min_front

        # Default: pass-through
        safe = Twist()
        safe.linear.x  = cmd.linear.x
        safe.linear.y  = cmd.linear.y
        safe.angular.z = cmd.angular.z
        safe.linear.z  = 0.0
        safe.angular.x = 0.0
        safe.angular.y = 0.0

        # If no scan, just pass-through
        if min_front is None:
            self._state = 'PASS'
            self._pub.publish(safe)
            return

        # ---------------- FSM ----------------
        # 1) Hard-block region
        if min_front <= self.stop_range:
            if self._state not in ('BLOCKED_WAIT', 'UNBLOCK_ROTATE'):
                self._state = 'BLOCKED_WAIT'
                self._blocked_since = now
                self._unblock_started = None
                self._unblock_dir = 0
                self._unblock_speed = 0.0
                self.get_logger().debug("ENTER BLOCKED_WAIT")

            # Always stop linear while blocked
            safe.linear.x = 0.0

            if self._state == 'BLOCKED_WAIT':
                # Hold still (avoid angular jitter before commit)
                safe.angular.z = 0.0
                wait_s = (now - self._blocked_since).nanoseconds / 1e9
                if wait_s >= self.unblock_wait_sec:
                    self._unblock_dir = self._pick_unblock_direction()
                    # Choose rotation speed based on last cmd or defaults
                    spd = abs(self._last_raw_angular)
                    if spd < self.unblock_rotate_speed_min:
                        spd = self.unblock_rotate_speed_min
                    spd = clamp(spd, 0.0, self.unblock_rotate_speed_max)
                    self._unblock_speed = spd
                    self._unblock_started = now
                    self._state = 'UNBLOCK_ROTATE'
                    self.get_logger().debug(f"ENTER UNBLOCK_ROTATE dir={self._unblock_dir} spd={self._unblock_speed:.2f}rad/s")
                self._pub.publish(safe)
                return

            if self._state == 'UNBLOCK_ROTATE':
                safe.linear.x = 0.0
                # Hold chosen direction & speed (ignore Nav2 jitter)
                safe.angular.z = self._unblock_dir * self._unblock_speed
                rot_s = (now - self._unblock_started).nanoseconds / 1e9

                # Exit conditions:
                # - got enough clearance and rotated at least a little time
                if (min_front >= self.hysteresis_release) and (rot_s >= self.unblock_min_rotate_sec):
                    self._state = 'PASS'
                    self._blocked_since = None
                    self._unblock_started = None
                    self.get_logger().debug("EXIT UNBLOCK_ROTATE -> PASS (clearance reached)")
                # - rotated too long -> fall back to wait (let Nav2 rethink)
                elif rot_s >= self.unblock_max_rotate_sec:
                    self._state = 'BLOCKED_WAIT'
                    self._blocked_since = now
                    self._unblock_started = None
                    safe.angular.z = 0.0
                    self.get_logger().debug("UNBLOCK_ROTATE timeout -> BLOCKED_WAIT")
                self._pub.publish(safe)
                return

        # 2) If we recently were blocked, require hysteresis clearance to resume
        if self._state in ('BLOCKED_WAIT', 'UNBLOCK_ROTATE'):
            if min_front >= self.hysteresis_release:
                self._state = 'PASS'
                self._blocked_since = None
                self._unblock_started = None
                self.get_logger().debug("PASS (hysteresis clear)")
            else:
                # Keep linear stopped; allow gentle alignment (bounded angular)
                safe.linear.x = 0.0
                rot = clamp(self._last_raw_angular,
                            -self.unblock_rotate_speed_max,
                            +self.unblock_rotate_speed_max)
                safe.angular.z = rot
                self._pub.publish(safe)
                return

        # 3) Slow-down zone
        if self.stop_range < min_front <= self.slow_down_range:
            self._state = 'SLOW'
            k = (min_front - self.stop_range) / max(1e-6, (self.slow_down_range - self.stop_range))
            k = max(self.slow_scale_min, min(1.0, k))
            safe.linear.x *= k
            if self.scale_angular:
                safe.angular.z *= max(self.angular_scale_min, k)
            self._pub.publish(safe)
            return

        # 4) Clear → pass-through
        self._state = 'PASS'
        self._pub.publish(safe)


def main(args=None):
    rclpy.init(args=args)
    node = SupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()