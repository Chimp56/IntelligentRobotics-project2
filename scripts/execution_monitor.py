#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math

class ExecutionMonitor:
    def __init__(self):
        rospy.init_node('execution_monitor', anonymous=True)
        
        # Get robot starting position in feet from launch file
        self.start_x_feet = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet = rospy.get_param("~start_y_feet", 0.0)
        
        # Odometry subscriber
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # Publisher for status communication with navigation controller
        self.status_pub = rospy.Publisher('/execution_monitor/status', String, queue_size=1)
        
        # Subscriber for target point from navigation controller
        self.target_point_sub = rospy.Subscriber('/execution_monitor/set_target', Point, self.set_target_callback)
        
        # Current state
        self.odom_data = None
        self.current_position = None
        self.target_point = None
        self.previous_distance = float('inf')
        self.stuck_counter = 0
        self.max_stuck_iterations = 100  # 10 seconds at 10Hz
        self.success_threshold = 1.0  # 1 foot
        
        # Coordinate system setup
        self.meters_per_foot = 0.3048  # Conversion factor
        
        # Node name for logging
        self.node_name = rospy.get_name()
        
        # Rate for monitoring loop
        self.rate = rospy.Rate(10)  # 10 Hz
        
        rospy.loginfo("Execution Monitor initialized")
        rospy.loginfo("Starting position: (%.2f, %.2f) feet", 
                      self.start_x_feet, self.start_y_feet)
        
        # Publish initial status to indicate readiness
        rospy.sleep(0.5)  # Brief delay to ensure publishers are ready
        self.publish_status("READY: Execution Monitor initialized")

    def run(self):
        """
        Main execution loop
        """
        rospy.loginfo("Execution Monitor starting...")
        
        while not rospy.is_shutdown():
            self.monitor_execution()
            self.rate.sleep()

    def monitor_execution(self):
        """
        Main monitoring loop - checks progress and determines success/failure
        """
        if self.target_point is None:
            rospy.loginfo("No target point set - skipping monitoring")
            return
        
        if self.current_position is None:
            rospy.loginfo("No position data available - skipping monitoring")
            return
        
        current_distance = self.calculate_distance_to_target()
        
        rospy.loginfo_throttle(2.0, "Current: (%.2f, %.2f) ft, Target: (%.2f, %.2f) ft, Delta: (%.2f, %.2f)",
                               self.current_position[0], self.current_position[1], self.target_point[0], self.target_point[1], 
                               self.target_point[0] - self.current_position[0], self.target_point[1] - self.current_position[1])

        # Check if target reached
        if self.is_target_reached():
            status = "SUCCESS: Target reached! Distance: {:.2f} feet".format(current_distance)
            # rospy.loginfo(status)
            self.publish_status(status)
            return True
        
        # Check if making progress
        if self.is_making_progress():
            status = "PROGRESS: Moving towards target. Distance: {:.2f} feet".format(current_distance)
            # rospy.loginfo(status)
            self.publish_status(status)
            return False
        
        # Check if stuck
        if self.is_stuck():
            status = "FAILURE: Robot appears stuck. Distance: {:.2f} feet".format(current_distance)
            # rospy.logwarn(status)
            self.publish_status(status)
            return True
        
        # Still working on it
        status = "STALLED: Progress stalled but not stuck yet. Distance: {:.2f} feet".format(current_distance)
        rospy.loginfo(status)
        self.publish_status(status)
        return False
    
    
    def set_target_callback(self, msg):
        """
        Callback to receive target point from navigation controller
        """
        self.target_point = (msg.x, msg.y)
        self.previous_distance = float('inf')
        self.stuck_counter = 0
        # rospy.loginfo("Target point received: ({:.2f}, {:.2f}) feet".format(msg.x, msg.y))
        self.publish_status("Target set to ({:.2f}, {:.2f}) feet".format(msg.x, msg.y))
    
    def publish_status(self, message):
        """
        Publish status message
        """
        status_msg = String()
        status_msg.data = message
        self.status_pub.publish(status_msg)
    
    def odom_callback(self, data):
        """
        Update current position from odometry and convert to feet
        """
        self.odom_data = data
        if data is not None:
            current_x_feet = (data.pose.pose.position.x / self.meters_per_foot) + self.start_x_feet
            current_y_feet = (data.pose.pose.position.y / self.meters_per_foot) + self.start_y_feet
            
            self.current_position = (current_x_feet, current_y_feet)
    
    def calculate_distance_to_target(self):
        """
        Calculate distance from current position to target point
        """
        if self.current_position is None or self.target_point is None:
            return float('inf')
        
        dx = self.target_point[0] - self.current_position[0]
        dy = self.target_point[1] - self.current_position[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def is_making_progress(self):
        """
        Check if robot is making progress towards target
        """
        if self.current_position is None or self.target_point is None:
            return False
        
        current_distance = self.calculate_distance_to_target()
        
        # Check if we're getting closer
        if current_distance < self.previous_distance:
            self.stuck_counter = 0
            self.previous_distance = current_distance
            return True
        else:
            # Not making progress
            self.stuck_counter += 1
            return False
    
    def is_target_reached(self):
        """
        Check if robot is within 1 foot of target point
        """
        if self.current_position is None or self.target_point is None:
            return False
        
        distance = self.calculate_distance_to_target()
        return distance <= self.success_threshold
    
    def is_stuck(self):
        """
        Check if robot has been stuck (not making progress) for too long
        """
        return self.stuck_counter >= self.max_stuck_iterations
    


if __name__ == '__main__':
    try:
        monitor = ExecutionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass