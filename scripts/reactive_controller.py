#!/usr/bin/env python
import rospy
import random
import numpy as np
from kobuki_msgs.msg import BumperEvent
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry



# constants
MAX_RANDOM_TURN_DEGREE_ANGLE = 15
FRONT_ESCAPE_DISTANCE_FEET = 1
FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN = 1
ESCAPE_TURN_DEGREE_ANGLE = 180
ESCAPE_TURN_DEGREE_ANGLE_VARIANCE = 30
FEET_PER_METER = 3.28084
METERS_PER_FEET = 1 / FEET_PER_METER
CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER = 0.087
CAMERA_TO_BUMPER_OFFSET_METER = 0.40
COLLISION_TIMEOUT_SEC = 3.0
BUMPER_DEBOUNCE_SEC = 0.5

TELEOP_IDLE_SEC = 2.0
TELEOP_EPS = 1e-3

class ReactiveController:
    def __init__(self):
        self.state = 'DRIVE_FORWARD'

        rospy.init_node('reactive_controller', anonymous=True)
        
        # Publisher for velocity commands (use navi to send controls)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        
        # Subscriber for teleop twists to detect keyboard control
        self.last_nonzero_teleop_time = None
        self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleop_callback)
        
        # Subscriber for bumper events
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        
        # Subscriber for laser scan data
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)

        # Subscriber for odometry data
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.collision_detected = False
        self.obstacle_detected = False
        self.laser_data = None
        self.odom_data = None
        self.x_position_after_turn = None
        self.y_position_after_turn = None
        self.bumper_pressed = False
        self.collision_release_time = None
        self.collision_time = None

    def run(self):
        """
        High level control loop
        """
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            # Check for collision first
            if self.collision_detected or self.state == 'HALT':
                # Continuously publish zero velocities while in collision
                self.halt_robot()
                rate.sleep()
                continue
            
            # Pause autonomy if teleop recently active
            if self._teleop_active():
                rate.sleep()
                continue
            
            # Check for obstacles ahead
            if self.laser_data is not None:
                self.check_obstacles_ahead()
            
            # if drove FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN turn
            self.turn_randomly()

            # Drive forward if no obstacles
            if not self.obstacle_detected and not self.collision_detected:
                self.drive_forward()
            
            rate.sleep()

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

        # Within threshold?
        if len(front_ranges) > 0 and np.min(front_ranges) < distance_threshold:
            self.obstacle_detected = True
            rospy.loginfo("Obstacle detected")
            rospy.loginfo("Obstacle detected at {:.2f}m (threshold: {:.2f}m)".format(np.min(front_ranges), distance_threshold))

            # Determine if obstacle is symmetric or asymmetric
            if self.is_obstacle_symmetric(front_ranges):
                self.on_symmetric_obstacle_ahead()
            else:
                self.on_asymmetric_obstacle_ahead()
        else:
            self.obstacle_detected = False

    def on_symmetric_obstacle_ahead(self):
        """
        Handle symmetric obstacle by turning a random degree angle
        """
        rospy.loginfo("Symmetric obstacle detected - executing escape turn")
        
        # Calculate random turn angle
        min_angle = ESCAPE_TURN_DEGREE_ANGLE - ESCAPE_TURN_DEGREE_ANGLE_VARIANCE
        max_angle = ESCAPE_TURN_DEGREE_ANGLE + ESCAPE_TURN_DEGREE_ANGLE_VARIANCE
        turn_angle = random.uniform(min_angle, max_angle)
        
        # Convert to radians
        turn_radians = np.radians(turn_angle)
        
        rospy.loginfo("Turning {:.1f} degrees ({:.2f} radians)".format(turn_angle, turn_radians))
        
        # Execute turn
        self.execute_turn(turn_radians)
        
        # Reset obstacle detection after turn
        self.obstacle_detected = False

    def on_asymmetric_obstacle_ahead(self):
        """
        Handle asymmetric obstacle by turning towards the clearer side only while
        asymmetric obstacles remain within the front threshold.
        """
        rospy.loginfo("Asymmetric obstacle detected - turning while asymmetric obstacle ahead")

        rate = rospy.Rate(10)
        angular_velocity = 0.5  # rad/s

        while not rospy.is_shutdown() and not self.collision_detected:
            front_ranges, distance_threshold = self._compute_front_ranges_and_threshold()
            if front_ranges is None or len(front_ranges) == 0:
                break

            nearest = np.min(front_ranges)
            rospy.loginfo("Closest obstacle (asym turn): {:.2f}m".format(nearest))

            # Stop if no longer within threshold or if obstacle becomes symmetric
            if nearest >= distance_threshold or self.is_obstacle_symmetric(front_ranges):
                break

            left_avg, right_avg = self._split_left_right(front_ranges)

            turn_msg = Twist()
            turn_msg.angular.z = angular_velocity if left_avg > right_avg else -angular_velocity
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()

        # Stop turning
        self.reset_velocity()
        self._set_position_after_turn()
        self.obstacle_detected = False

    def turn_randomly(self):
        """
        Turn randomly
        """
        # i need to check if the robot has driven FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN
        if self.odom_data is None or self.x_position_after_turn is None or self.y_position_after_turn is None:
            return
        
        # calculate distance from position_after_turn to odom_data
        distance = np.sqrt((self.x_position_after_turn - self.odom_data.pose.pose.position.x)**2 + (self.y_position_after_turn - self.odom_data.pose.pose.position.y)**2)
        if distance > FORWARD_MOVMENT_DISTANCE_FEET_BEFORE_TURN * METERS_PER_FEET:
            turn_angle = random.uniform(-MAX_RANDOM_TURN_DEGREE_ANGLE, MAX_RANDOM_TURN_DEGREE_ANGLE)
            self.execute_turn(np.radians(turn_angle))

    def drive_forward(self):
        forward_msg = Twist()
        forward_msg.linear.x = .2
        forward_msg.angular.x = 0.0
        self.cmd_vel_pub.publish(forward_msg)

    def execute_turn(self, angle_radians):
        """
        Execute a turn by the specified angle
        """
        # Calculate turn duration based on angular velocity
        angular_velocity = 0.5  # rad/s
        turn_duration = abs(angle_radians) / angular_velocity
        
        # Create turn command
        turn_msg = Twist()
        turn_msg.angular.z = angular_velocity if angle_radians > 0 else -angular_velocity
        
        # Execute turn, but break if teleop activates or collision happens
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < turn_duration:
            if self.collision_detected or self._teleop_active():
                break
            self.cmd_vel_pub.publish(turn_msg)
            rate.sleep()
        
        # Stop after turn
        self.reset_velocity()

        # record position after turn
        self._set_position_after_turn()

    def is_obstacle_symmetric(self, front_ranges=None):
        """
        Determine if obstacle is symmetric by comparing left and right sides
        """
        if front_ranges is None:
            front_ranges, _ = self._compute_front_ranges_and_threshold()
            if front_ranges is None:
                return True

        if len(front_ranges) < 3:
            return True  # Default to symmetric if not enough data

        left_avg, right_avg = self._split_left_right(front_ranges)

        # Consider symmetric if difference is small
        avg_distance = (left_avg + right_avg) / 2
        if avg_distance == float('inf'):
            return True

        difference = abs(left_avg - right_avg)
        threshold = 0.1 * avg_distance  # 10% tolerance

        is_symmetric = difference < threshold
        rospy.loginfo("Obstacle symmetry check: left={:.2f}m, right={:.2f}m, symmetric={}".format(left_avg, right_avg, is_symmetric))
        return is_symmetric

    def halt_robot(self):
        """
        Handle collision by immediately halting the robot
        """
        halt_msg = Twist()
        self.cmd_vel_pub.publish(halt_msg)
        
        # Set state to HALT - robot will stay halted until non-zero teleop command
        self.state = 'HALT'

        rospy.loginfo("Robot halted - waiting for non-zero teleop command to resume")
    
    def reset_velocity(self):
        """
        Reset the velocity of the robot
        """
        reset_msg = Twist()
        self.cmd_vel_pub.publish(reset_msg)

    def bumper_callback(self, data):
        """
        Callback function for bumper events
        """
        if data.state == BumperEvent.PRESSED:
            # Check if enough time has passed since last release (debounce)
            time_since_release = (rospy.Time.now() - self.collision_release_time).to_sec() if self.collision_release_time else float('inf')
            if self.collision_release_time is None or time_since_release > BUMPER_DEBOUNCE_SEC:
                collision_detected_str = 'Collision detected! Bumper:' + str(data.bumper) + ' (0=LEFT, 1=CENTER, 2=RIGHT)'
                rospy.loginfo(collision_detected_str)
                self.bumper_pressed = True
                self.collision_detected = True
                self.state = 'COLLISION'
                self.collision_time = rospy.Time.now()
                self.halt_robot()
        elif data.state == BumperEvent.RELEASED:
            bumper_relaesed_str = "Bumper released: " + str(data.bumper)
            rospy.loginfo(bumper_relaesed_str)
            self.bumper_pressed = False
            # Start debounce timer; run loop will clear collision after stable release
            self.collision_release_time = rospy.Time.now()

    def laser_callback(self, data):
        """
        Callback function for laser scan data
        """
        self.laser_data = data

    def odom_callback(self, data):
        """
        Callback function for odometry data
        """
        self.odom_data = data

    def teleop_callback(self, msg):
        if self._is_nonzero_twist(msg):
            time_since_collision = (rospy.Time.now() - self.collision_time).to_sec() if self.collision_time else float('inf')
            if time_since_collision > COLLISION_TIMEOUT_SEC:
                self.last_nonzero_teleop_time = rospy.Time.now()
                # If robot is in HALT state, resume autonomous operation
                if self.state == 'HALT':
                    self.state = 'DRIVE_FORWARD'
                    self.collision_detected = False

    def _set_position_after_turn(self):
        self.x_position_after_turn = self.odom_data.pose.pose.position.x
        self.y_position_after_turn = self.odom_data.pose.pose.position.y

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

        # Front sector (roughly -30 to +30 degrees)
        front_start_idx = int((-np.pi/6 - angle_min) / angle_increment)
        front_end_idx = int((np.pi/6 - angle_min) / angle_increment)
        front_start_idx = max(0, front_start_idx)
        front_end_idx = min(len(ranges), front_end_idx)

        front_ranges = ranges[front_start_idx:front_end_idx]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges < self.laser_data.range_max]

        return front_ranges, distance_threshold

    def _split_left_right(self, front_ranges):
        """
        Split the front-sector ranges into left and right halves and return their means.
        Returns (left_avg, right_avg). If side has no readings, 0 is returned for that side.
        NOTE: LaserScan angles increase from right (-) to left (+), and our front_ranges
        slice is ordered from right to left. Therefore, the SECOND half is LEFT.
        """
        if front_ranges is None or len(front_ranges) == 0:
            return 0.0, 0.0
        mid = len(front_ranges) // 2
        right_ranges = front_ranges[:mid]
        left_ranges = front_ranges[mid:]
        left_avg = np.mean(left_ranges) if len(left_ranges) > 0 else 0.0
        right_avg = np.mean(right_ranges) if len(right_ranges) > 0 else 0.0
        return left_avg, right_avg


if __name__ == '__main__':
    try:
        controller = ReactiveController()
        controller.run()
    except rospy.ROSInterruptException:
        pass


