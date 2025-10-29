#!/usr/bin/env python
import rospy
import numpy as np
import math
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String

# --- Constants ---
FEET_PER_METER = 3.28084
METERS_PER_FEET = 1.0 / FEET_PER_METER

FRONT_ESCAPE_DISTANCE_FEET = 0.5
CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER = 0.087
CAMERA_TO_BUMPER_OFFSET_METER = 0.40

COLLISION_TIMEOUT_SEC = 3.0
BUMPER_DEBOUNCE_SEC = 0.5
TELEOP_IDLE_SEC = 2.0
TELEOP_EPS = 1e-3

# Navigation tuning
FORWARD_SPEED = 0.2       # m/s
ANGULAR_SPEED = 0.5       # rad/s
TURN_TOLERANCE = 0.1      # radians (~5.7 deg)


class NavigationController(object):
    def __init__(self):
        self.state = 'IDLE'

        rospy.init_node('navigation_controller', anonymous=True)
        
        # Get robot starting position in feet from launch file
        self.start_x_feet = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet = rospy.get_param("~start_y_feet", 0.0)
        self.start_theta_deg = rospy.get_param("~start_theta_deg", 0.0)
        self.yaw_offset_rad = math.radians(self.start_theta_deg)

        # --- Publishers / Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',
                                           Twist, queue_size=1)

        self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop',
                                           Twist, self.teleop_callback)

        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',
                                           BumperEvent, self.bumper_callback)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        # Subscribe to task planner's waypoint list
        self.waypoints_sub = rospy.Subscriber('/task_planner/waypoints',
                                              String, self._waypoints_callback)

        # Talk to execution monitor
        self.target_point_pub = rospy.Publisher('/execution_monitor/set_target',
                                                Point, queue_size=1)
        self.execution_status_sub = rospy.Subscriber('/execution_monitor/status',
                                                     String, self.on_execution_status)

        # --- Internal nav state ---
        self.odom_data = None
        self.laser_data = None

        self.collision_detected = False
        self.bumper_pressed = False
        self.last_nonzero_teleop_time = None
        self.collision_time = None
        self.collision_release_time = None

        self.current_target = None
        self.waypoints = []
        self.current_waypoint_index = 0

        self.meters_per_foot = 0.3048   # constant

        # Execution monitor readiness
        self.execution_monitor_ready = False

        # Control loop rate
        self.rate = rospy.Rate(10)  # 10 Hz

        rospy.loginfo("Navigation Controller initialized")
        rospy.loginfo("Starting position: (%.2f, %.2f) feet", 
                      self.start_x_feet, self.start_y_feet)

    def run(self):
        rospy.loginfo("Navigation Controller starting...")

        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.navigate_to_target()
            elif self.state == 'COMPLETED':
                rospy.loginfo("Navigation completed")
                break
            self.rate.sleep()

    # ---------------- Waypoint interface ----------------
    def set_waypoints(self, waypoints):
        """
        waypoints: list of (x, y) tuples in FEET from the task planner.
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0

        if waypoints:
            if self.wait_for_execution_monitor():
                self.set_next_target()
                self.state = 'NAVIGATING'
                rospy.loginfo("Starting navigation with %d waypoints (feet).",
                              len(waypoints))
            else:
                rospy.logwarn("Execution monitor not ready - navigation not started")
                self.state = 'IDLE'
        else:
            self.state = 'IDLE'
            rospy.loginfo("No waypoints provided")

    def _waypoints_callback(self, msg):
        """
        Get semicolon-separated 'x,y' in FEET from TaskPlanner:
            '2.0,3.0; 9.0,8.0; 12.0,9.0; 4.0,14.0'
        Parse to [(2.0,3.0), (9.0,8.0), ...] then call set_waypoints().
        """
        text = msg.data.strip()
        if not text:
            rospy.logwarn("NavigationController: received empty waypoints list.")
            return

        try:
            pts = []
            for chunk in text.split(';'):
                chunk = chunk.strip()
                if not chunk:
                    continue
                x_str, y_str = chunk.split(',')
                pts.append((float(x_str), float(y_str)))

            self.set_waypoints(pts)
            rospy.loginfo("NavigationController: received %d waypoints from Task Planner.",
                          len(pts))

        except Exception as e:
            rospy.logerr("NavigationController: failed to parse waypoints '%s': %s",
                         text, e)

    def set_next_target(self):
        """
        Tell the execution monitor about the next waypoint,
        and update self.current_target.
        """
        if self.current_waypoint_index < len(self.waypoints):
            self.current_target = self.waypoints[self.current_waypoint_index]

            target_msg = Point()
            target_msg.x = self.current_target[0]
            target_msg.y = self.current_target[1]
            target_msg.z = 0.0
            self.target_point_pub.publish(target_msg)

            rospy.loginfo("Target set to waypoint %d: (%.2f, %.2f) feet",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])
        else:
            self.current_target = None
            target_msg = Point()
            target_msg.x = 0.0
            target_msg.y = 0.0
            target_msg.z = -1.0  # 'no target'
            self.target_point_pub.publish(target_msg)

    # ---------------- Execution monitor feedback ----------------
    def on_execution_status(self, msg):
        status = msg.data.strip()

        if not self.execution_monitor_ready:
            self.execution_monitor_ready = True
            rospy.loginfo("Execution Monitor is now ready!")

        if status.startswith("SUCCESS") and self.current_target is not None:
            rospy.loginfo("Reached waypoint %d: (%.2f, %.2f) feet",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])

            self.current_waypoint_index += 1

            if self.current_waypoint_index < len(self.waypoints):
                self.set_next_target()
                rospy.loginfo("Moving to next waypoint")
            else:
                self.state = 'COMPLETED'
                self.current_target = None
                rospy.loginfo("All waypoints completed!")

        elif status.startswith("FAILURE") and self.current_target is not None:
            rospy.logwarn("Failed waypoint %d: (%.2f, %.2f) feet",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])

            # simple behavior: skip to next point
            self.current_waypoint_index += 1
            if self.current_waypoint_index < len(self.waypoints):
                self.set_next_target()
                rospy.loginfo("Attempting next waypoint after failure")
            else:
                self.state = 'COMPLETED'
                self.current_target = None
                rospy.logwarn("Navigation completed with failures")

        elif status.startswith("PROGRESS:") or status.startswith("STALLED:"):
            rospy.loginfo("Execution Monitor: %s", status)

        elif status.startswith("READY:"):
            rospy.loginfo("Execution Monitor: %s", status)

    def wait_for_execution_monitor(self, timeout=10.0):
        rospy.loginfo("Waiting for execution monitor to be ready...")
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.execution_monitor_ready:
                return True

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("Timeout waiting for execution monitor after %.1f seconds",
                              timeout)
                return False

            try:
                topics = rospy.get_published_topics()
                execution_monitor_topics = [
                    topic for topic in topics
                    if 'execution_monitor' in topic[0]
                ]
                if execution_monitor_topics:
                    rospy.loginfo("Found execution monitor topics: %s",
                                  [t[0] for t in execution_monitor_topics])
                else:
                    rospy.loginfo("Execution monitor topics not found yet...")
            except Exception:
                rospy.loginfo("Checking for execution monitor topics...")

            rospy.sleep(1.0)

        return False

    # ---------------- Motion helpers ----------------
    def calculate_angle_to_target(self):
        if self.odom_data is None or self.current_target is None:
            return 0.0

        current_x_ft = (self.odom_data.pose.pose.position.x / self.meters_per_foot) + self.start_x_feet
        current_y_ft = (self.odom_data.pose.pose.position.y / self.meters_per_foot) + self.start_y_feet

        target_x_ft, target_y_ft = self.current_target

        dx = target_x_ft - current_x_ft
        dy = target_y_ft - current_y_ft

        desired_angle = math.atan2(dy, dx)
        
        # Debug: Log current position, target, and desired heading
        rospy.loginfo_throttle(2.0, "Current: (%.2f, %.2f) ft, Target: (%.2f, %.2f) ft, Delta: (%.2f, %.2f), Desired angle: %.1f deg",
                               current_x_ft, current_y_ft, target_x_ft, target_y_ft, 
                               dx, dy, math.degrees(desired_angle))

        # Extract yaw (radians) from quaternion orientation
        current_orientation = self.odom_data.pose.pose.orientation
        current_yaw = self.quaternion_to_yaw(current_orientation)
        
        current_yaw += self.yaw_offset_rad

        # Normalize to [-pi, pi]
        while current_yaw > math.pi:
            current_yaw -= 2 * math.pi
        while current_yaw < -math.pi:
            current_yaw += 2 * math.pi
        
        # Debug: Log current orientation vs desired
        rospy.loginfo_throttle(2.0, "Yaw(off=%.1f): curr=%.1f, desired=%.1f", 
                               math.degrees(self.yaw_offset_rad),
                               math.degrees(current_yaw), math.degrees(desired_angle))

        angle_diff = desired_angle - current_yaw

        # normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        rospy.loginfo_throttle(2.0, "Angle difference: %.1f deg", math.degrees(angle_diff))

        return angle_diff

    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def navigate_to_target(self):
        if self.current_target is None:
            self.stop_robot()
            return

        angle_diff = self.calculate_angle_to_target()

        # check obstacles
        if self.laser_data is not None:
            self.check_obstacles_ahead()

        if hasattr(self, 'obstacle_detected') and self.obstacle_detected:
            self.handle_obstacle()
            return

        # pause for teleop override
        if self._teleop_active():
            self.stop_robot()
            return

        if abs(angle_diff) > TURN_TOLERANCE:
            self.turn_towards_target(angle_diff)
        else:
            self.move_forward()

    def turn_towards_target(self, angle_diff):
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED if angle_diff > 0.0 else -ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Turning towards target. Angle diff: %.2f rad", angle_diff)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Moving forward towards target")

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def check_obstacles_ahead(self):
        if self.laser_data is None:
            return

        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]

        # Approx front -30deg to +30deg
        angle_min = self.laser_data.angle_min
        angle_inc = self.laser_data.angle_increment

        front_start_idx = int((-math.pi/6 - angle_min) / angle_inc)
        front_end_idx   = int(( math.pi/6 - angle_min) / angle_inc)

        front_start_idx = max(0, front_start_idx)
        front_end_idx   = min(len(ranges), front_end_idx)

        front_ranges = ranges[front_start_idx:front_end_idx]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges < self.laser_data.range_max]

        # feet -> meters, plus camera->bumper offset
        dist_thresh_m = (FRONT_ESCAPE_DISTANCE_FEET * METERS_PER_FEET
                         + CAMERA_TO_BUMPER_OFFSET_METER
                         + CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER)

        if len(front_ranges) > 0 and np.min(front_ranges) < dist_thresh_m:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected at %.2f m", np.min(front_ranges))
        else:
            self.obstacle_detected = False

    def handle_obstacle(self):
        # Simple placeholder. Could spin in place, etc.
        return

    # ---------------- Callbacks ----------------
    def bumper_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            time_since_release = (rospy.Time.now() - self.collision_release_time).to_sec() \
                if self.collision_release_time else float('inf')
            if self.collision_release_time is None or time_since_release > BUMPER_DEBOUNCE_SEC:
                rospy.loginfo('Collision detected! Bumper: %s', data.bumper)
                self.bumper_pressed = True
                self.collision_detected = True
                self.collision_time = rospy.Time.now()
                self.stop_robot()
        elif data.state == BumperEvent.RELEASED:
            rospy.loginfo("Bumper released: %s", data.bumper)
            self.bumper_pressed = False
            self.collision_release_time = rospy.Time.now()

    def laser_callback(self, data):
        self.laser_data = data

    def odom_callback(self, data):
        self.odom_data = data

    def teleop_callback(self, msg):
        if self._is_nonzero_twist(msg):
            time_since_collision = (rospy.Time.now() - self.collision_time).to_sec() \
                if self.collision_time else float('inf')
            if time_since_collision > COLLISION_TIMEOUT_SEC:
                self.last_nonzero_teleop_time = rospy.Time.now()
                if self.collision_detected:
                    self.collision_detected = False

    def _is_nonzero_twist(self, msg):
        return (abs(msg.linear.x) > TELEOP_EPS or
                abs(msg.linear.y) > TELEOP_EPS or
                abs(msg.linear.z) > TELEOP_EPS or
                abs(msg.angular.x) > TELEOP_EPS or
                abs(msg.angular.y) > TELEOP_EPS or
                abs(msg.angular.z) > TELEOP_EPS)

    def _teleop_active(self):
        if self.last_nonzero_teleop_time is None:
            return False
        return (rospy.Time.now() - self.last_nonzero_teleop_time).to_sec() < TELEOP_IDLE_SEC


def main():
    ctrl = NavigationController()
    try:
        ctrl.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
