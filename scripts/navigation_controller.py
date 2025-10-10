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

# Constants
FEET_PER_METER = 3.28084
METERS_PER_FEET = 1 / FEET_PER_METER
FRONT_ESCAPE_DISTANCE_FEET = .5
CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER = 0.087
CAMERA_TO_BUMPER_OFFSET_METER = 0.40
COLLISION_TIMEOUT_SEC = 3.0
BUMPER_DEBOUNCE_SEC = 0.5
TELEOP_IDLE_SEC = 2.0
TELEOP_EPS = 1e-3

# Navigation constants
FORWARD_SPEED = 0.2  # m/s
ANGULAR_SPEED = 0.5  # rad/s
TURN_TOLERANCE = 0.1  # radians (~5.7 degrees)

class NavigationController:
    def __init__(self):
        self.state = 'IDLE'
        
        rospy.init_node('navigation_controller', anonymous=True)
        
        # Publishers and subscribers
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1)
        self.teleop_sub = rospy.Subscriber('/cmd_vel_mux/input/teleop', Twist, self.teleop_callback)
        self.bumper_sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.bumper_callback)
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # State variables
        self.odom_data = None
        self.laser_data = None
        self.collision_detected = False
        self.bumper_pressed = False
        self.last_nonzero_teleop_time = None
        self.collision_time = None
        self.collision_release_time = None
        
        # Navigation variables
        self.current_target = None
        self.waypoints = []
        self.current_waypoint_index = 0
        
        # Coordinate system setup
        self.startup_position = None  # Robot's actual starting position in odom frame (meters)
        self.meters_per_foot = 0.3048  # Conversion factor
        
        # Execution monitor communication
        self.target_point_pub = rospy.Publisher('/execution_monitor/set_target', Point, queue_size=1)
        self.execution_status_sub = rospy.Subscriber('/execution_monitor/status', String, self.on_execution_status)
        
        # Rate for control loop
        self.rate = rospy.Rate(10)  # 10 Hz
        
        # Flag to track if execution monitor is ready
        self.execution_monitor_ready = False
        
        rospy.loginfo("Navigation Controller initialized")
        
    def run(self):
        """
        Main control loop
        """
        rospy.loginfo("Navigation Controller starting...")
        
        while not rospy.is_shutdown():
            if self.state == 'NAVIGATING':
                self.navigate_to_target()
            elif self.state == 'COMPLETED':
                rospy.loginfo("Navigation completed")
                break
            
            self.rate.sleep()
            
    def set_waypoints(self, waypoints):
        """
        Set a list of waypoints to navigate to (in feet)
        waypoints: list of (x, y) tuples in feet
        """
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        if waypoints:
            # Check if execution monitor is ready before setting target
            if self.wait_for_execution_monitor():
                self.set_next_target()
                self.state = 'NAVIGATING'
                rospy.loginfo("Starting navigation with {} waypoints (in feet)".format(len(waypoints)))
            else:
                rospy.logwarn("Execution monitor not ready - navigation not started")
                self.state = 'IDLE'
        else:
            self.state = 'IDLE'
            rospy.loginfo("No waypoints provided")
    
    def set_next_target(self):
        """
        Set the next waypoint as the current target
        """
        if self.current_waypoint_index < len(self.waypoints):
            self.current_target = self.waypoints[self.current_waypoint_index]
            
            # Send target point to execution monitor node
            target_msg = Point()
            target_msg.x = self.current_target[0]
            target_msg.y = self.current_target[1]
            target_msg.z = 0.0  # Not used
            self.target_point_pub.publish(target_msg)
            
            rospy.loginfo("Target set to waypoint {}: ({:.2f}, {:.2f}) feet".format(
                self.current_waypoint_index, self.current_target[0], self.current_target[1]))
        else:
            self.current_target = None
            # Send empty target to execution monitor
            target_msg = Point()
            target_msg.x = 0.0
            target_msg.y = 0.0
            target_msg.z = -1.0  # Special value to indicate no target
            self.target_point_pub.publish(target_msg)
    
    def on_execution_status(self, msg):
        """
        Callback to receive status updates from execution monitor
        """
        status = msg.data.strip()
        
        # Mark execution monitor as ready when we receive any status message
        if not self.execution_monitor_ready:
            self.execution_monitor_ready = True
            rospy.loginfo("Execution Monitor is now ready!")
        
        if status.startswith("SUCCESS") and self.current_target is not None:
                rospy.loginfo("Successfully reached waypoint {}: ({:.2f}, {:.2f}) feet".format(
                    self.current_waypoint_index, self.current_target[0], self.current_target[1]))
                
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index < len(self.waypoints):
                    # Move to next waypoint
                    self.set_next_target()
                    rospy.loginfo("Moving to next waypoint")
                else:
                    # All waypoints completed
                    self.state = 'COMPLETED'
                    self.current_target = None
                    rospy.loginfo("All waypoints completed!")
                
        elif status.startswith("FAILURE") and self.current_target is not None:
                rospy.logwarn("Failed to reach waypoint {}: ({:.2f}, {:.2f}) feet".format(
                    self.current_waypoint_index, self.current_target[0], self.current_target[1]))
                
                # For now, just move to the next waypoint
                # In a full implementation, this would handle the specific failure scenarios
                self.current_waypoint_index += 1
                
                if self.current_waypoint_index < len(self.waypoints):
                    self.set_next_target()
                    rospy.loginfo("Attempting next waypoint after failure")
                else:
                    self.state = 'COMPLETED'
                    self.current_target = None
                    rospy.logwarn("Navigation completed with failures")
                
        elif status.startswith("PROGRESS:") or status.startswith("STALLED:"):
            # Just log progress/stalled status, no action needed
            rospy.loginfo("Execution Monitor: {}".format(status))
            
        elif status.startswith("READY:"):
            # Execution monitor is ready - this is handled by the readiness flag above
            rospy.loginfo("Execution Monitor: {}".format(status))
    
    def wait_for_execution_monitor(self, timeout=10.0):
        """
        Wait for execution monitor to be ready
        Returns True if ready, False if timeout
        """
        rospy.loginfo("Waiting for execution monitor to be ready...")
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            if self.execution_monitor_ready:
                return True
            
            # Check for timeout
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > timeout:
                rospy.logwarn("Timeout waiting for execution monitor after {:.1f} seconds".format(timeout))
                return False
            
            # Check if execution monitor topic exists
            try:
                # Try to get topic info to see if execution monitor is publishing
                topics = rospy.get_published_topics()
                execution_monitor_topics = [topic for topic in topics if 'execution_monitor' in topic[0]]
                
                if execution_monitor_topics:
                    rospy.loginfo("Found execution monitor topics: {}".format([t[0] for t in execution_monitor_topics]))
                else:
                    rospy.loginfo("Execution monitor topics not found yet...")
                    
            except Exception as e:
                rospy.loginfo("Checking for execution monitor topics...")
            
            rospy.sleep(1.0)  # Wait 1 second before checking again
        
        return False
    
    def calculate_angle_to_target(self):
        """
        Calculate the angle from current position to target
        """
        if self.odom_data is None or self.current_target is None:
            return 0
        
        # Set startup position on first odometry reading
        if self.startup_position is None:
            self.startup_position = (
                self.odom_data.pose.pose.position.x,
                self.odom_data.pose.pose.position.y
            )
            rospy.loginfo("Controller startup position set to: ({:.3f}, {:.3f}) meters".format(
                self.startup_position[0], self.startup_position[1]))
        
        # Convert current odom position to feet, with (0,0) at startup position
        current_x_meters = self.odom_data.pose.pose.position.x - self.startup_position[0]
        current_y_meters = self.odom_data.pose.pose.position.y - self.startup_position[1]
        
        current_x_feet = current_x_meters / self.meters_per_foot
        current_y_feet = current_y_meters / self.meters_per_foot
        
        # Target is already in feet
        target_x, target_y = self.current_target
        
        # Calculate desired angle in feet coordinate system
        desired_angle = math.atan2(target_y - current_y_feet, target_x - current_x_feet)
        
        # Get current orientation (yaw)
        current_orientation = self.odom_data.pose.pose.orientation
        current_yaw = self.quaternion_to_yaw(current_orientation)
        
        # Calculate angle difference
        angle_diff = desired_angle - current_yaw
        
        # Normalize to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        return angle_diff
    
    def quaternion_to_yaw(self, quaternion):
        """
        Convert quaternion to yaw angle
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Roll and pitch calculations not needed for yaw, but kept for completeness
        # sinr_cosp = 2 * (w * x + y * z)
        # cosr_cosp = 1 - 2 * (x * x + y * y)
        # roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # sinp = 2 * (w * y - z * x)
        # if abs(sinp) >= 1:
        #     pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
        # else:
        #     pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def navigate_to_target(self):
        """
        Navigate towards the current target
        """
        if self.current_target is None:
            self.stop_robot()
            return
        
        # Calculate angle to target
        angle_diff = self.calculate_angle_to_target()
        
        # Check for obstacles
        if self.laser_data is not None:
            self.check_obstacles_ahead()
        
        # If obstacle detected, handle it
        if hasattr(self, 'obstacle_detected') and self.obstacle_detected:
            self.handle_obstacle()
            return
        
        # If collision detected, stop
        # if self.collision_detected:
        #     self.stop_robot()
        #     return
        
        # If teleop is active, pause navigation
        if self._teleop_active():
            self.stop_robot()
            return
        
        # Navigate based on angle difference
        if abs(angle_diff) > TURN_TOLERANCE:
            # Turn towards target
            self.turn_towards_target(angle_diff)
        else:
            # Move forward towards target
            self.move_forward()
    
    def turn_towards_target(self, angle_diff):
        """
        Turn towards the target
        """
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED if angle_diff > 0 else -ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Turning towards target. Angle diff: {:.2f} rad".format(angle_diff))
    
    def move_forward(self):
        """
        Move forward towards target
        """
        twist = Twist()
        twist.linear.x = FORWARD_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Moving forward towards target")
    
    def stop_robot(self):
        """
        Stop the robot
        """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def check_obstacles_ahead(self):
        """
        Check if obstacles are within FRONT_ESCAPE_DISTANCE_FEET
        """
        if self.laser_data is None:
            return
        
        ranges = np.array(self.laser_data.ranges)
        ranges = ranges[np.isfinite(ranges)]
        
        # Front sector (roughly -30 to +30 degrees)
        angle_min = self.laser_data.angle_min
        angle_increment = self.laser_data.angle_increment
        
        front_start_idx = int((-math.pi/6 - angle_min) / angle_increment)
        front_end_idx = int((math.pi/6 - angle_min) / angle_increment)
        front_start_idx = max(0, front_start_idx)
        front_end_idx = min(len(ranges), front_end_idx)
        
        front_ranges = ranges[front_start_idx:front_end_idx]
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges < self.laser_data.range_max]
        
        # Calculate threshold
        distance_threshold = FRONT_ESCAPE_DISTANCE_FEET * METERS_PER_FEET
        distance_threshold += CAMERA_TO_BUMPER_OFFSET_METER + CAMERA_TO_BASE_FOOTPRINT_OFFSET_METER
        
        # Check for obstacles
        if len(front_ranges) > 0 and np.min(front_ranges) < distance_threshold:
            self.obstacle_detected = False
            rospy.loginfo("Obstacle detected at {:.2f}m".format(np.min(front_ranges)))
        else:
            self.obstacle_detected = False
    
    def handle_obstacle(self):
        """
        Handle obstacle by turning away
        """
        return
        # Simple obstacle avoidance - turn left
        twist = Twist()
        twist.angular.z = ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Avoiding obstacle")

    
    # Callback functions
    def bumper_callback(self, data):
        """
        Callback function for bumper events
        """
        if data.state == BumperEvent.PRESSED:
            time_since_release = (rospy.Time.now() - self.collision_release_time).to_sec() if self.collision_release_time else float('inf')
            if self.collision_release_time is None or time_since_release > BUMPER_DEBOUNCE_SEC:
                rospy.loginfo('Collision detected! Bumper: {}'.format(data.bumper))
                self.bumper_pressed = True
                self.collision_detected = True
                self.collision_time = rospy.Time.now()
                self.stop_robot()
        elif data.state == BumperEvent.RELEASED:
            rospy.loginfo("Bumper released: {}".format(data.bumper))
            self.bumper_pressed = False
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
        """
        Callback for teleop commands
        """
        if self._is_nonzero_twist(msg):
            time_since_collision = (rospy.Time.now() - self.collision_time).to_sec() if self.collision_time else float('inf')
            if time_since_collision > COLLISION_TIMEOUT_SEC:
                self.last_nonzero_teleop_time = rospy.Time.now()
                if self.collision_detected:
                    self.collision_detected = False
    
    def _is_nonzero_twist(self, msg):
        """
        Check if twist message has non-zero values
        """
        return (abs(msg.linear.x) > TELEOP_EPS or
                abs(msg.linear.y) > TELEOP_EPS or
                abs(msg.linear.z) > TELEOP_EPS or
                abs(msg.angular.x) > TELEOP_EPS or
                abs(msg.angular.y) > TELEOP_EPS or
                abs(msg.angular.z) > TELEOP_EPS)
    
    def _teleop_active(self):
        """
        Check if teleop is currently active
        """
        if self.last_nonzero_teleop_time is None:
            return False
        return (rospy.Time.now() - self.last_nonzero_teleop_time).to_sec() < TELEOP_IDLE_SEC


def main():
    """
    Main function with dummy waypoints for testing
    """
    # Create controller
    controller = NavigationController()
    
    # dummy waypoints for testing (in feet)
    # These are some example points within a typical room
    # Robot will start at (0,0) and navigate to these points
    dummy_waypoints = [
        (10.0, 2.0),   # 10 feet right, 5 feet up from start, starting at (0,0) this should be a success
        (4.0, 10),   # 4 feet right, 2 feet up from start, this should be a failure
        (1.0, 3.0),   # 1 foot right, 3 feet up from start
        (3.0, 4.0),   # 3 feet right, 4 feet up from start
    ]
    
    # Set waypoints and start navigation
    controller.set_waypoints(dummy_waypoints)
    
    # Run the controller
    try:
        controller.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()