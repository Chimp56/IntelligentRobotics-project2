#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import random
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# --- Constants ---
FEET_PER_METER = 3.28084
METERS_PER_FEET = 1.0 / FEET_PER_METER

FRONT_ESCAPE_DISTANCE_FEET = 1.0  # Increased from 0.5 to be less sensitive
CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER = 0.087
CAMERA_TO_BUMPER_OFFSET_METER = 0.40
FRONT_SCAN_ANGLE_DEG = 20.0  # Narrower scan angle (reduced from ~30 degrees)
MIN_OBSTACLE_READINGS = 8  # Require multiple readings for obstacle detection

COLLISION_TIMEOUT_SEC = 3.0
BUMPER_DEBOUNCE_SEC = 0.5
TELEOP_IDLE_SEC = 2.0
TELEOP_EPS = 1e-3

FORWARD_SPEED = 0.2    # m/s
ANGULAR_SPEED = 0.5    # rad/s
TURN_TOLERANCE = 0.1   # rad (~5.7 deg)
BACKOFF_SPEED = 0.2    # m/s (reverse speed on bumper)
BACKOFF_DURATION_SEC = 0.8
ADVANCE_AFTER_TURN_SEC = 0.8
ADVANCE_SPEED_SCALE = 0.6


# Obstacle avoidance constants
ESCAPE_TURN_DEGREE_ANGLE = 45.0  # degrees
ESCAPE_TURN_DEGREE_ANGLE_VARIANCE = 15.0  # degrees
SYMMETRIC_THRESHOLD = 0.05  # ratio difference threshold for symmetric detection (lower = more asymmetric)
AVOID_ARC_SPEED_SCALE = 0.4  # percent of FORWARD_SPEED while arcing around
AVOID_TIMEOUT_SEC = 3.0      # stop avoidance after this many seconds
CLEARANCE_EXIT_SCALE = 1.2   # exit when min distance > threshold * this


class NavigationController(object):
    def __init__(self):
        rospy.init_node('navigation_controller', anonymous=True)

        # --- Spawn pose (feet/degrees) from launch ---
        self.start_x_feet = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet = rospy.get_param("~start_y_feet", 0.0)
        self.start_theta_deg = rospy.get_param("~start_theta_deg", 0.0)

        # yaw_offset maps Gazebo/base_link yaw (odom frame) into your classroom frame
        # You already had -90 baked in. We keep that behavior.
        self.yaw_offset_rad = math.radians(self.start_theta_deg - 90.0)

        # --- Publishers / Subscribers ---
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi',
                                           Twist, queue_size=1)

        self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop',
                                           Twist, self.teleop_callback)

        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper',
                                           BumperEvent, self.bumper_callback)

        self.laser_sub = rospy.Subscriber('/scan',
                                          LaserScan, self.laser_callback)

        self.odom_sub = rospy.Subscriber('/odom',
                                         Odometry, self.odom_callback)

        # TaskPlanner sends global waypoint list here (feet coords)
        self.waypoints_sub = rospy.Subscriber('/task_planner/waypoints',
                                              String, self._waypoints_callback)

        # ExecutionMonitor bridge
        self.target_point_pub = rospy.Publisher('/execution_monitor/set_target',
                                                Point, queue_size=1)
        self.execution_status_sub = rospy.Subscriber('/execution_monitor/status',
                                                     String, self.on_execution_status)

        # --- State ---
        self.state = 'WAITING'  # WAITING, NAVIGATING, COMPLETED
        self.execution_monitor_ready = False

        self.odom_data = None
        self.laser_data = None

        self.collision_detected = False
        self.bumper_pressed = False
        self.last_nonzero_teleop_time = None
        self.collision_time = None
        self.collision_release_time = None
        self.backoff_until = None

        # active plan
        self.waypoints = []            # [(x_ft,y_ft), ...]
        self.current_waypoint_index = 0
        self.current_target = None     # (x_ft,y_ft)

        self.meters_per_foot = 0.3048

        self.rate = rospy.Rate(10)     # 10 Hz

        rospy.loginfo("NavigationController: init complete.")
        rospy.loginfo("NavigationController: declared spawn (%.2f, %.2f) feet, %.1f deg",
                      self.start_x_feet, self.start_y_feet, self.start_theta_deg)

    # ------------------------------------------------------------------
    # Main loop
    # ------------------------------------------------------------------
    def run(self):
        rospy.loginfo("NavigationController: starting control loop.")

        rospy.loginfo("Waiting 10 seconds for Gazebo to launch...")
        rospy.sleep(10.0)
        rospy.loginfo("Starting navigation...")
        
        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.navigate_to_target()
            elif self.state == 'COMPLETED':
                # We're done with this batch of waypoints.
                # Stay alive and wait for /task_planner/waypoints to publish
                # a new batch (runtime retasking).
                rospy.loginfo_throttle(2.0,
                    "NavigationController: COMPLETED current plan; waiting for new tasks.")
                # let ExecutionMonitor know we're idle (no active target)
                self._publish_no_target()
                self.state = 'WAITING'
            elif self.state == 'WAITING':
                # idle; just spin
                pass

            self.rate.sleep()

    # ------------------------------------------------------------------
    # Waypoint / planning interface
    # ------------------------------------------------------------------
    def _waypoints_callback(self, msg):
        """
        Incoming semicolon-separated "x,y" list, in FEET global frame:
           "2.0,3.0; 9.0,8.0; ..."
        This can happen at program start OR again later (retask).
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

            rospy.loginfo("NavigationController: got %d waypoints from TaskPlanner.",
                          len(pts))

            self._start_new_plan(pts)

        except Exception as e:
            rospy.logerr("NavigationController: failed to parse waypoints '%s': %s",
                         text, e)

    def _start_new_plan(self, pts):
        """
        Reset navigation with a brand new set of waypoints from the planner.
        """
        self.waypoints = list(pts)
        self.current_waypoint_index = 0

        if not self.waypoints:
            rospy.logwarn("NavigationController: empty new plan, staying WAITING.")
            self.state = 'WAITING'
            self._publish_no_target()
            return

        if not self.wait_for_execution_monitor():
            rospy.logwarn("NavigationController: ExecMonitor not ready, can't start nav.")
            self.state = 'WAITING'
            self._publish_no_target()
            return

        # Kick off the first target and enter NAVIGATING
        self.set_next_target()
        self.state = 'NAVIGATING'
        rospy.loginfo("NavigationController: NAVIGATING with %d waypoints.",
                      len(self.waypoints))

    def set_next_target(self):
        """
        Move to the next waypoint in the list (if any), publish it to ExecutionMonitor.
        """
        if self.current_waypoint_index < len(self.waypoints):
            self.current_target = self.waypoints[self.current_waypoint_index]

            tgt = Point()
            tgt.x = self.current_target[0]
            tgt.y = self.current_target[1]
            tgt.z = 0.0
            self.target_point_pub.publish(tgt)

            rospy.loginfo("NavigationController: new target %d -> (%.2f, %.2f) ft",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])
        else:
            # No more targets left in this batch
            self.current_target = None
            self._publish_no_target()
            self.state = 'COMPLETED'
            rospy.loginfo("NavigationController: all waypoints complete.")

    def _publish_no_target(self):
        """
        Tell the ExecutionMonitor there is currently NO active target,
        so it can announce READY_FOR_TASKS.
        """
        msg = Point()
        msg.x = 0.0
        msg.y = 0.0
        msg.z = -1.0
        self.target_point_pub.publish(msg)

    # ------------------------------------------------------------------
    # ExecutionMonitor feedback
    # ------------------------------------------------------------------
    def on_execution_status(self, msg):
        """
        React to SUCCESS / FAILURE from ExecutionMonitor.
        """
        status = msg.data.strip()

        if not self.execution_monitor_ready:
            self.execution_monitor_ready = True
            rospy.loginfo("NavigationController: ExecutionMonitor is now ready.")

        if status.startswith("SUCCESS") and self.current_target is not None:
            rospy.loginfo("NavigationController: waypoint %d succeeded (%.2f, %.2f) ft",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])

            self.current_waypoint_index += 1
            self.set_next_target()  # this will flip us to COMPLETED if we ran out

        elif status.startswith("FAILURE") and self.current_target is not None:
            rospy.logwarn("NavigationController: waypoint %d FAILED (%.2f, %.2f) ft",
                          self.current_waypoint_index,
                          self.current_target[0],
                          self.current_target[1])

            # naive fallback: skip this point
            self.current_waypoint_index += 1
            self.set_next_target()

        elif status.startswith("READY_FOR_TASKS"):
            # ExecutionMonitor telling us it's idle. We can stay WAITING.
            rospy.loginfo_throttle(2.0,
                "NavigationController: monitor READY_FOR_TASKS (idle).")

        # PROGRESS / STALLED we just log periodically
        elif status.startswith("PROGRESS") or status.startswith("STALLED"):
            rospy.loginfo_throttle(2.0,
                "ExecutionMonitor says: %s", status)

    def wait_for_execution_monitor(self, timeout=10.0):
        """
        Block until we've heard from ExecutionMonitor at least once,
        or until timeout.
        """
        start_time = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.execution_monitor_ready:
                return True
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                return False
            rospy.sleep(0.2)
        return False

    # ------------------------------------------------------------------
    # Motion / control
    # ------------------------------------------------------------------
    def navigate_to_target(self):
        """
        Reactive local controller:
        1. turn to face target,
        2. go forward,
        3. avoid obstacle crudely,
        4. pause if teleop override is active.
        """
        # Handle bumper back-off first
        if self.backoff_until is not None:
            if (rospy.Time.now() - self.backoff_until).to_sec() < 0.0:
                twist = Twist()
                twist.linear.x = -BACKOFF_SPEED
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo_throttle(0.5, "NavCtrl: backing off due to bumper")
                return
            else:
                # Backoff complete
                self.backoff_until = None
                self.collision_detected = False
                self.bumper_pressed = False
        if self.current_target is None:
            self.stop_robot()
            return

        # compute heading correction
        angle_diff = self.calculate_angle_to_target()

        # simple collision / teleop arbitration
        self.check_obstacles_ahead()
        if getattr(self, 'obstacle_detected', False):
            self.handle_obstacle()
            self.move_forward()
            return
        

        if self._teleop_active():
            self.stop_robot()
            return

        if abs(angle_diff) > TURN_TOLERANCE:
            self.turn_towards_target(angle_diff)
        else:
            self.move_forward()

    def calculate_angle_to_target(self):
        """
        Convert /odom (meters) -> global classroom feet using spawn offset,
        then compute heading error to current_target in radians.
        """
        if self.odom_data is None or self.current_target is None:
            return 0.0

        # odom -> feet, then offset by declared spawn
        cur_x_ft = (self.odom_data.pose.pose.position.x / self.meters_per_foot) \
                   + self.start_x_feet
        cur_y_ft = (self.odom_data.pose.pose.position.y / self.meters_per_foot) \
                   + self.start_y_feet

        tgt_x_ft, tgt_y_ft = self.current_target
        dx = tgt_x_ft - cur_x_ft
        dy = tgt_y_ft - cur_y_ft

        desired_angle = math.atan2(dy, dx)

        # robot yaw in odom frame
        q = self.odom_data.pose.pose.orientation
        yaw_now = self.quaternion_to_yaw(q)

        # apply the static offset you were using
        yaw_now += self.yaw_offset_rad
        while yaw_now > math.pi:
            yaw_now -= 2.0 * math.pi
        while yaw_now < -math.pi:
            yaw_now += 2.0 * math.pi

        angle_diff = desired_angle - yaw_now
        while angle_diff > math.pi:
            angle_diff -= 2.0 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2.0 * math.pi

        # debug throttle
        rospy.loginfo_throttle(
            2.0,
            "NavCtrl pos(%.2f,%.2f)ft tgt(%.2f,%.2f)ft -> hdg_err %.1f deg",
            cur_x_ft, cur_y_ft,
            tgt_x_ft, tgt_y_ft,
            math.degrees(angle_diff)
        )

        return angle_diff

    def quaternion_to_yaw(self, q):
        x = q.x
        y = q.y
        z = q.z
        w = q.w
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def turn_towards_target(self, angle_diff):
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED if angle_diff > 0.0 else -ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo_throttle(1.0,
            "NavCtrl: turning (%.2f rad error)", angle_diff)

    def move_forward(self):
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo_throttle(1.0,
            "NavCtrl: moving forward")

    def _advance_after_turn(self, seconds=ADVANCE_AFTER_TURN_SEC):
        """Push forward briefly after a turn to commit around the obstacle."""
        end_time = rospy.Time.now() + rospy.Duration(seconds)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            twist = Twist()
            twist.linear.x = FORWARD_SPEED * ADVANCE_SPEED_SCALE
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def check_obstacles_ahead(self):
        """
        Check if obstacles are within FRONT_ESCAPE_DISTANCE_FEET
        """
        front_ranges, distance_threshold = self._compute_front_ranges_and_threshold()
        if front_ranges is None:
            return

        # Debug logging
        if len(front_ranges) > 0:
            rospy.loginfo("Closest obstacle: {:.2f}m".format(np.min(front_ranges)))

        # Within threshold AND sufficient readings? (requires multiple readings to avoid false positives)
        obstacle_count = len(front_ranges)
        if obstacle_count >= MIN_OBSTACLE_READINGS and np.min(front_ranges) < distance_threshold:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected (count: {}, min: {:.2f}m, threshold: {:.2f}m)".format(
                obstacle_count, np.min(front_ranges), distance_threshold))

            # Determine if obstacle is symmetric or asymmetric
            if self.is_obstacle_symmetric(front_ranges):
                self.on_symmetric_obstacle_ahead()
            else:
                self.on_asymmetric_obstacle_ahead()
            self.move_forward()
        else:
            self.obstacle_detected = False

    def on_symmetric_obstacle_ahead(self):
        """
        Handle symmetric obstacle by turning based on target direction.
        If dy < dx: turn toward (current_x, target_y)
        If dx < dy: turn toward (target_x, current_y)
        """
        rospy.loginfo("Symmetric obstacle detected - turning based on target direction")
        
        # Get current position and target
        if self.odom_data is None or self.current_target is None:
            # Fall back to random turn if no data
            turn_angle = random.uniform(30, 60)
            turn_radians = np.radians(turn_angle)
            rospy.logwarn("No position/target data, using random turn")
        else:
            # Current position
            cur_x_ft = (self.odom_data.pose.pose.position.x / self.meters_per_foot) + self.start_x_feet
            cur_y_ft = (self.odom_data.pose.pose.position.y / self.meters_per_foot) + self.start_y_feet
            
            # Target position
            tgt_x_ft, tgt_y_ft = self.current_target
            
            # Calculate deltas
            dx = tgt_x_ft - cur_x_ft
            dy = tgt_y_ft - cur_y_ft
            
            # Determine intermediate waypoint based on which dimension is larger
            if abs(dy) < abs(dx):
                # Horizontal movement is longer - turn toward (current_x, target_y)
                inter_x = cur_x_ft
                inter_y = tgt_y_ft
                rospy.loginfo("Horizontal path longer (dx=%.2f > dy=%.2f), turning toward (%.2f, %.2f)", 
                             abs(dx), abs(dy), inter_x, inter_y)
            else:
                # Vertical movement is longer - turn toward (target_x, current_y)
                inter_x = tgt_x_ft
                inter_y = cur_y_ft
                rospy.loginfo("Vertical path longer (dy=%.2f > dx=%.2f), turning toward (%.2f, %.2f)", 
                             abs(dy), abs(dx), inter_x, inter_y)
            
            # Calculate angle to intermediate waypoint
            inter_dx = inter_x - cur_x_ft
            inter_dy = inter_y - cur_y_ft
            desired_angle = math.atan2(inter_dy, inter_dx)
            
            # Get current orientation
            q = self.odom_data.pose.pose.orientation
            yaw_now = self.quaternion_to_yaw(q)
            yaw_now += self.yaw_offset_rad
            
            # Normalize
            while yaw_now > math.pi:
                yaw_now -= 2.0 * math.pi
            while yaw_now < -math.pi:
                yaw_now += 2.0 * math.pi
            
            # Calculate turn angle
            turn_radians = desired_angle - yaw_now
            
            # Normalize turn angle
            while turn_radians > math.pi:
                turn_radians -= 2.0 * math.pi
            while turn_radians < -math.pi:
                turn_radians += 2.0 * math.pi
        
        rospy.loginfo("Turning {:.1f} degrees ({:.2f} radians)".format(
            math.degrees(turn_radians), turn_radians))
        
        # Execute turn
        self.execute_turn(turn_radians)
        
        # After turning, push forward to start skirting the obstacle
        self._advance_after_turn()

        # Reset obstacle detection after turn
        self.obstacle_detected = False

    def on_asymmetric_obstacle_ahead(self):
        """
        Handle asymmetric obstacle by turning towards the clearer side only while
        asymmetric obstacles remain within the front threshold.
        """
        rospy.loginfo("Asymmetric obstacle detected - arcing toward open side")

        rate = rospy.Rate(10)
        angular_velocity = 0.5  # rad/s
        start_time = rospy.Time.now()

        while not rospy.is_shutdown() and not self.collision_detected:
            # Timeout safeguard to prevent getting stuck
            if (rospy.Time.now() - start_time).to_sec() > AVOID_TIMEOUT_SEC:
                rospy.loginfo("Avoidance timeout reached; resuming course")
                break

            front_ranges, distance_threshold = self._compute_front_ranges_and_threshold()
            if front_ranges is None or len(front_ranges) == 0:
                break

            nearest = np.min(front_ranges)
            rospy.loginfo("Closest obstacle (asym arc): {:.2f}m".format(nearest))

            # Exit if we have gained sufficient clearance or obstacle looks symmetric now
            if nearest >= distance_threshold * CLEARANCE_EXIT_SCALE or self.is_obstacle_symmetric(front_ranges):
                break

            # Determine which side is clearer using min distance per half.
            mid = len(front_ranges) // 2
            right_half = front_ranges[:mid]   # negative angles (right side)
            left_half  = front_ranges[mid:]   # positive angles (left side)

            left_min = float(np.min(left_half)) if len(left_half) > 0 else 0.0
            right_min = float(np.min(right_half)) if len(right_half) > 0 else 0.0

            # Arc: move forward slowly while turning toward the clearer side (larger min distance)
            twist = Twist()
            twist.linear.x = FORWARD_SPEED * AVOID_ARC_SPEED_SCALE
            twist.angular.z = angular_velocity if left_min > right_min else -angular_velocity
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        # Stop turning, then push forward a bit to commit around obstacle
        self.reset_velocity()
        self._set_position_after_turn()
        self._advance_after_turn()
        self.obstacle_detected = False

    def is_obstacle_symmetric(self, front_ranges=None):
        """
        Determine symmetry using more robust cues:
        - Compare left/right minimum distances (strong signal of which side is clearer)
        - Compare left/right counts (how many points are within threshold on each side)

        Returns True only when both sides are very similar.
        """
        if front_ranges is None:
            front_ranges, _ = self._compute_front_ranges_and_threshold()
            if front_ranges is None:
                return True

        # Split into halves
        mid = len(front_ranges) // 2
        left = front_ranges[:mid]
        right = front_ranges[mid:]

        # If no readings at all, treat as symmetric (no evidence)
        if len(left) == 0 and len(right) == 0:
            return True
        # If readings only on one side, clearly asymmetric
        if len(left) == 0 or len(right) == 0:
            return False

        # Metrics
        left_min = float(np.min(left)) if len(left) > 0 else float('inf')
        right_min = float(np.min(right)) if len(right) > 0 else float('inf')
        left_count = len(left)
        right_count = len(right)

        # 1) Min-distance similarity (relative difference)
        denom = (left_min + right_min) / 2.0 if (left_min + right_min) > 0.0 else 1.0
        min_ratio_diff = abs(left_min - right_min) / denom

        # 2) Count balance (how even are the returns on each side?)
        total = float(left_count + right_count)
        count_imbalance = abs(left_count - right_count) / total if total > 0 else 0.0

        # Thresholds: keep SYMMETRIC_THRESHOLD as distance similarity gate,
        # and require counts to be reasonably balanced (< 40% imbalance)
        COUNT_IMBALANCE_THRESHOLD = 0.4

        # Symmetric only if both distance similarity and count balance are satisfied
        is_symmetric = (min_ratio_diff < SYMMETRIC_THRESHOLD) and (count_imbalance < COUNT_IMBALANCE_THRESHOLD)

        rospy.loginfo_throttle(2.0,
            "Symmetry check: left_min=%.2fm right_min=%.2fm min_ratio=%.2f count_l=%d count_r=%d imbalance=%.2f -> %s",
            left_min, right_min, min_ratio_diff, left_count, right_count, count_imbalance,
            "SYMMETRIC" if is_symmetric else "ASYMMETRIC")

        return is_symmetric

    def _compute_front_ranges_and_threshold(self):
        """
        Compute front-sector laser ranges and the effective distance threshold.
        Returns (front_ranges, distance_threshold). front_ranges is a 1-D numpy array
        of valid readings within the front sector. Returns (None, None) if laser data is missing.
        """
        if self.laser_data is None:
            return None, None

        # Effective threshold in meters (feet + camera-to-bumper + camera-to-base offsets)
        distance_threshold = FRONT_ESCAPE_DISTANCE_FEET * METERS_PER_FEET
        distance_threshold = distance_threshold + CAMERA_TO_BUMPER_OFFSET_METER + CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER

        ranges = np.array(self.laser_data.ranges)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment

        # Filter out invalid readings (inf, nan)
        ranges = ranges[np.isfinite(ranges)]

        # Front sector using configured scan angle (narrower for less sensitivity)
        scan_angle_rad = math.radians(FRONT_SCAN_ANGLE_DEG)
        front_start_idx = int((-scan_angle_rad - angle_min) / angle_increment)
        front_end_idx = int((scan_angle_rad - angle_min) / angle_increment)
        front_start_idx = max(0, front_start_idx)
        front_end_idx = min(len(ranges), front_end_idx)


        front_ranges = ranges[front_start_idx:front_end_idx]
        front_ranges = front_ranges[front_ranges < distance_threshold]

        return front_ranges, distance_threshold

    def _split_left_right(self, front_ranges):
        """
        Split front ranges into left and right halves and return average distances.
        Returns (left_avg, right_avg) in meters.
        """
        if len(front_ranges) == 0:
            return 0.0, 0.0
        
        mid = len(front_ranges) // 2
        left_ranges = front_ranges[:mid]
        right_ranges = front_ranges[mid:]
        
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else 0.0
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else 0.0
        
        return left_avg, right_avg

    def execute_turn(self, turn_radians):
        """
        Execute a turn for the specified angle in radians.
        """
        if self.odom_data is None:
            return
        
        # Record starting yaw
        current_orientation = self.odom_data.pose.pose.orientation
        self.turn_start_yaw = self.quaternion_to_yaw(current_orientation)
        self.turn_in_progress = True
        
        # Determine turn direction (always take shortest path)
        if turn_radians < 0:
            angular_velocity = -ANGULAR_SPEED
            target_turn = abs(turn_radians)
        else:
            angular_velocity = ANGULAR_SPEED
            target_turn = turn_radians
        
        rate = rospy.Rate(10)
        total_turned = 0.0
        
        while not rospy.is_shutdown() and total_turned < target_turn and not self.collision_detected:
            if self.odom_data is None:
                break
                
            current_orientation = self.odom_data.pose.pose.orientation
            current_yaw = self.quaternion_to_yaw(current_orientation)
            
            # Calculate how much we've turned
            angle_diff = current_yaw - self.turn_start_yaw
            
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            
            total_turned = abs(angle_diff)
            
            # Continue turning
            twist = Twist()
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        # Stop turning
        self.reset_velocity()
        self._set_position_after_turn()
        self.turn_in_progress = False
        self.turn_start_yaw = None

    def reset_velocity(self):
        """Stop all movement"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def _set_position_after_turn(self):
        """
        Placeholder for updating position after turn if needed.
        Currently just logs that turn completed.
        """
        rospy.loginfo("Turn completed")

    def handle_obstacle(self):
        """
        Handle obstacle avoidance - this is called from navigate_to_target
        when obstacle_detected is True. The actual handling is done in
        check_obstacles_ahead() via on_symmetric_obstacle_ahead() or
        on_asymmetric_obstacle_ahead().
        """
        # The obstacle handling is already done in check_obstacles_ahead()
        # This method exists for compatibility but doesn't need to do anything
        pass

    # ------------------------------------------------------------------
    # Sensors / teleop callbacks
    # ------------------------------------------------------------------
    def bumper_callback(self, data):
        if data.state == BumperEvent.PRESSED:
            since_rel = (rospy.Time.now() - self.collision_release_time).to_sec() \
                        if hasattr(self, 'collision_release_time') and self.collision_release_time \
                        else float('inf')
            if not hasattr(self, 'collision_release_time') or since_rel > BUMPER_DEBOUNCE_SEC:
                rospy.loginfo("NavCtrl: collision! bumper=%s", data.bumper)
                self.bumper_pressed = True
                self.collision_detected = True
                self.collision_time = rospy.Time.now()
                # Initiate timed back-off
                self.backoff_until = rospy.Time.now() + rospy.Duration(BACKOFF_DURATION_SEC)
                # Start backing immediately
                twist = Twist()
                twist.linear.x = -BACKOFF_SPEED
                self.cmd_vel_pub.publish(twist)
        elif data.state == BumperEvent.RELEASED:
            rospy.loginfo("NavCtrl: bumper released %s", data.bumper)
            self.bumper_pressed = False
            self.collision_release_time = rospy.Time.now()

    def laser_callback(self, data):
        self.laser_data = data

    def odom_callback(self, data):
        self.odom_data = data

    def teleop_callback(self, msg):
        if self._is_nonzero_twist(msg):
            since_collision = (rospy.Time.now() - self.collision_time).to_sec() \
                              if self.collision_time else float('inf')
            if since_collision > COLLISION_TIMEOUT_SEC:
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
        return ((rospy.Time.now() - self.last_nonzero_teleop_time).to_sec()
                < TELEOP_IDLE_SEC)


def main():
    ctrl = NavigationController()
    try:
        ctrl.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
