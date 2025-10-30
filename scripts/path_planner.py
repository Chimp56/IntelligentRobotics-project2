#!/usr/bin/env python
import rospy
import math
from nav_msgs.msg import Odometry
from project2.srv import GetPathSegment, GetPathSegmentResponse


class PathPlanner:
    def __init__(self):
        rospy.init_node('path_planner', anonymous=True)
        
        # Get robot starting position in feet from launch file
        self.start_x_feet = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet = rospy.get_param("~start_y_feet", 0.0)
        self.start_theta_deg = rospy.get_param("~start_theta_deg", 0.0)
        
        # Yaw offset to convert from Gazebo frame to assignment frame
        self.yaw_offset_rad = math.radians(self.start_theta_deg - 90.0)
        
        # Create the service
        self.service = rospy.Service('/path_planner/get_segment', 
                                    GetPathSegment, 
                                    self.handle_get_segment)
        
        # Subscriber for odometry
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # State variables
        self.odom_data = None
        self.current_position_feet = None  # (x, y) in feet
        self.current_orientation_rad = 0.0  # yaw in radians
        self.meters_per_foot = 0.3048
        
        rospy.loginfo("Path Planner initialized (service mode)")
        rospy.loginfo("Starting position: (%.2f, %.2f) feet, heading %.1f deg",
                     self.start_x_feet, self.start_y_feet, self.start_theta_deg)
        rospy.loginfo("Service available at: /path_planner/get_segment")
    
    def handle_get_segment(self, req):
        """
        Service handler - calculates path segment to requested target.
        
        Path plans are calculated from the current position, which is continuously
        updated via odometry. This means:
        - For first target: plans from starting position
        - After success: plans from the reached waypoint (current position)
        - After failure: plans from current position where failure occurred
        """
        rospy.loginfo("Path planning service request: target (%.2f, %.2f) feet",
                     req.target_x, req.target_y)
        
        response = GetPathSegmentResponse()
        
        # Check if we have position data
        if self.current_position_feet is None:
            response.success = False
            response.message = "No position data available yet"
            rospy.logwarn(response.message)
            return response
        
        try:
            # Calculate path segment
            target = (req.target_x, req.target_y)
            turn_angle, distance = self.calculate_path_segment(
                self.current_position_feet,
                self.current_orientation_rad,
                target
            )
            
            # Fill response
            response.turn_angle = turn_angle
            response.distance = distance
            response.success = True
            response.message = "Path segment calculated successfully"
            
            rospy.loginfo("Path Planner: From (%.2f, %.2f) to (%.2f, %.2f) - Turn %.2f rad (%.1f deg), Distance %.2f ft",
                         self.current_position_feet[0], self.current_position_feet[1],
                         req.target_x, req.target_y,
                         turn_angle, math.degrees(turn_angle), distance)
            
        except Exception as e:
            response.success = False
            response.message = "Error calculating path segment: {}".format(str(e))
            rospy.logerr(response.message)
        
        return response
    
    def calculate_path_segment(self, from_pos, from_orientation, to_pos):
        """
        Calculate turn angle and straight-line distance from current position to target.
        """
        # Calculate straight-line distance (Pythagorean theorem)
        dx = to_pos[0] - from_pos[0]
        dy = to_pos[1] - from_pos[1]
        distance = math.sqrt(dx * dx + dy * dy)
        
        # Calculate desired heading to target
        desired_heading = math.atan2(dy, dx)
        
        # Calculate turn angle needed
        turn_angle = desired_heading - from_orientation
        
        # Normalize turn angle to [-pi, pi] range
        while turn_angle > math.pi:
            turn_angle -= 2 * math.pi
        while turn_angle < -math.pi:
            turn_angle += 2 * math.pi
        
        return turn_angle, distance
    
    def update_position(self):
        """
        Update current position and orientation from odometry.
        """
        if self.odom_data is None:
            return
        
        # Convert odometry position to feet, accounting for starting position
        current_x_feet = (self.odom_data.pose.pose.position.x / self.meters_per_foot) + self.start_x_feet
        current_y_feet = (self.odom_data.pose.pose.position.y / self.meters_per_foot) + self.start_y_feet
        
        self.current_position_feet = (current_x_feet, current_y_feet)
        
        # Get current orientation (yaw) and apply offset for coordinate frame conversion
        current_yaw = self.quaternion_to_yaw(self.odom_data.pose.pose.orientation)
        self.current_orientation_rad = current_yaw + self.yaw_offset_rad
        
        # Normalize orientation to [-pi, pi]
        while self.current_orientation_rad > math.pi:
            self.current_orientation_rad -= 2 * math.pi
        while self.current_orientation_rad < -math.pi:
            self.current_orientation_rad += 2 * math.pi
    
    def quaternion_to_yaw(self, quaternion):
        """
        Convert quaternion orientation to yaw angle (rotation about z-axis).
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w
        
        # Calculate yaw from quaternion
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def odom_callback(self, data):
        """
        Callback for odometry data.
        Updates robot position every time new odometry is received.
        """
        self.odom_data = data
        self.update_position()


if __name__ == '__main__':
    try:
        planner = PathPlanner()
        rospy.loginfo("Path Planner service ready and waiting for requests...")
        rospy.spin()  # Keep the service running
    except rospy.ROSInterruptException:
        pass
