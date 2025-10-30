#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import String
import math


class ExecutionMonitor(object):
    def __init__(self):
        rospy.init_node('execution_monitor', anonymous=True)
        
        # --- Spawn pose (passed from launch in FEET) ---
        # This is the robot's declared global starting position.
        self.start_x_feet = rospy.get_param("~start_x_feet", 0.0)
        self.start_y_feet = rospy.get_param("~start_y_feet", 0.0)

        # --- ROS I/O ---
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        # NavigationController listens on this
        self.status_pub = rospy.Publisher('/execution_monitor/status',
                                          String, queue_size=1)

        # NavigationController publishes the current/next target here
        self.target_point_sub = rospy.Subscriber('/execution_monitor/set_target',
                                                 Point,
                                                 self.set_target_callback)

        # --- State ---
        self.odom_data = None
        self.current_position = None        # (x_ft, y_ft)
        self.target_point = None            # (x_ft, y_ft) OR None
        self.has_active_target = False      # becomes False when NavController sends z = -1.0

        self.previous_distance = float('inf')
        self.stuck_counter = 0
        self.max_stuck_iterations = 150     # ~15s at 10Hz
        self.success_threshold = 1.0        # feet

        self.meters_per_foot = 0.3048

        self.rate = rospy.Rate(10)          # 10 Hz

        rospy.loginfo("ExecutionMonitor: init complete.")
        rospy.loginfo("ExecutionMonitor: declared spawn (%.2f, %.2f) feet",
                      self.start_x_feet, self.start_y_feet)

        # small delay just to let pubs connect
        rospy.sleep(0.5)
        self.publish_status("READY: Execution Monitor initialized")

    def run(self):
        rospy.loginfo("ExecutionMonitor: starting main loop.")
        while not rospy.is_shutdown():
            self.monitor_execution()
            self.rate.sleep()

    # ------------------------------------------------------------------
    # Core monitoring logic
    # ------------------------------------------------------------------
    def monitor_execution(self):
        """
        Decide whether we're making progress, stuck, success, etc.
        Also announce READY_FOR_TASKS when navigation is idle.
        """
        # If NavController told us "no active target", advertise READY_FOR_TASKS.
        if not self.has_active_target:
            # Only spam this ~2s so log doesn't explode
            rospy.loginfo_throttle(2.0,
                                   "ExecutionMonitor: idle / waiting for new tasks.")
            self.publish_status("READY_FOR_TASKS: waiting for new task list")
            return

        # Past here, we *do* have an active target.
        if self.target_point is None:
            rospy.loginfo("ExecutionMonitor: target unset, skipping check.")
            return
        
        if self.current_position is None:
            rospy.loginfo("ExecutionMonitor: no odometry yet, skipping check.")
            return
        
        current_distance = self.calculate_distance_to_target()
        
        rospy.loginfo_throttle(
            2.0,
            "ExecMon pos(%.2f, %.2f)ft -> tgt(%.2f, %.2f)ft d=%.2f",
            self.current_position[0],
            self.current_position[1],
            self.target_point[0],
            self.target_point[1],
            current_distance
        )

        # Success?
        if self.is_target_reached():
            msg = "SUCCESS: Target reached! Distance: {:.2f} feet".format(
                current_distance
            )
            self.publish_status(msg)
            # After success, NavController will advance to next waypoint
            return

        # Progress?
        if self.is_making_progress():
            msg = "PROGRESS: Moving towards target. Distance: {:.2f} feet".format(
                current_distance
            )
            self.publish_status(msg)
            return

        # Stuck?
        if self.is_stuck():
            msg = "FAILURE: Robot appears stuck. Distance: {:.2f} feet".format(
                current_distance
            )
            self.publish_status(msg)
            # After failure, NavController will skip waypoint / advance
            return

        # Otherwise: stalled but not long enough to call failure
        msg = "STALLED: Progress stalled but not stuck yet. Distance: {:.2f} feet".format(
            current_distance
        )
        rospy.loginfo(msg)
        self.publish_status(msg)

    # ------------------------------------------------------------------
    # ROS Callbacks
    # ------------------------------------------------------------------
    def set_target_callback(self, msg):
        """
        NavigationController publishes the next target waypoint here.
        Convention:
          - normal target: z == 0.0  (valid x,y in feet)
          - "no more targets": z == -1.0
        """
        if msg.z < 0.0:
            # special "no target" signal
            self.target_point = None
            self.has_active_target = False
            self.previous_distance = float('inf')
            self.stuck_counter = 0
            rospy.loginfo("ExecutionMonitor: received NO TARGET (navigation idle).")
            # We don't immediately publish READY_FOR_TASKS here; monitor_execution will.
            return

        # else: an actual target
        self.target_point = (msg.x, msg.y)
        self.has_active_target = True
        self.previous_distance = float('inf')
        self.stuck_counter = 0

        self.publish_status(
            "Target set to ({:.2f}, {:.2f}) feet".format(msg.x, msg.y)
        )
    
    def odom_callback(self, data):
        """
        Convert /odom position (meters in Gazebo base frame)
        into world feet coordinates consistent with TaskPlanner.
        We assume /odom (0,0) == spawn pose in Gazebo. We then
        add the known spawn feet offset from launch.
        """
        self.odom_data = data
        if data is None:
            return

        # Convert meters -> feet, then offset by declared spawn feet
        x_ft = (data.pose.pose.position.x / self.meters_per_foot) + self.start_x_feet
        y_ft = (data.pose.pose.position.y / self.meters_per_foot) + self.start_y_feet
        self.current_position = (x_ft, y_ft)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def publish_status(self, message):
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)
    
    def calculate_distance_to_target(self):
        if self.current_position is None or self.target_point is None:
            return float('inf')
        dx = self.target_point[0] - self.current_position[0]
        dy = self.target_point[1] - self.current_position[1]
        return math.sqrt(dx * dx + dy * dy)

    def is_target_reached(self):
        if self.current_position is None or self.target_point is None:
            return False
        return self.calculate_distance_to_target() <= self.success_threshold
    
    def is_making_progress(self):
        if self.current_position is None or self.target_point is None:
            return False
        
        d = self.calculate_distance_to_target()
        # got closer?
        if d < self.previous_distance:
            self.previous_distance = d
            self.stuck_counter = 0
            return True

        # didn't get closer
            self.stuck_counter += 1
            return False
    
    def is_stuck(self):
        return self.stuck_counter >= self.max_stuck_iterations


if __name__ == '__main__':
    try:
        monitor = ExecutionMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        pass
