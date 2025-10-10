#!/usr/bin/env python
import rospy
import sys
from project2.srv import SetWaypoints

def send_waypoints_direct(x_coords, y_coords):
    """
    Send waypoints directly via ROS service
    """
    rospy.init_node('waypoint_sender', anonymous=True)
    
    try:
        # Wait for service to be available
        rospy.loginfo("Waiting for navigation controller service...")
        rospy.wait_for_service('/navigation_controller/set_waypoints', timeout=10.0)
        
        # Create service proxy
        set_waypoints = rospy.ServiceProxy('/navigation_controller/set_waypoints', SetWaypoints)
        
        rospy.loginfo("Sending {} waypoints to navigation controller...".format(len(x_coords)))
        
        # Send request
        response = set_waypoints(x_coords, y_coords)
        
        if response.success:
            rospy.loginfo("Success: {}".format(response.message))
            return True
        else:
            rospy.logerr("Error: {}".format(response.message))
            return False
            
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: {}".format(e))
        return False
    except rospy.ROSException as e:
        rospy.logerr("ROS error: {}".format(e))
        return False

def main():
    if len(sys.argv) < 3:
        print("Usage:")
        print("  python send_waypoints.py <x1> <y1> [x2] [y2] ...")
        print("")
        print("Examples:")
        print("  python send_waypoints.py 2.0 1.0")
        print("  python send_waypoints.py 2.0 1.0 4.0 2.0 1.0 3.0")
        print("  python send_waypoints.py 10.0 5.0 4.0 10.0 1.0 3.0")
        return
    
    try:
        # Parse command line arguments
        args = sys.argv[1:]
        if len(args) % 2 != 0:
            print("Error: Must provide even number of coordinates (x1 y1 x2 y2 ...)")
            return
        
        x_coords = []
        y_coords = []
        
        for i in range(0, len(args), 2):
            x = float(args[i])
            y = float(args[i+1])
            x_coords.append(x)
            y_coords.append(y)
        
        print("Waypoints to send:")
        for i, (x, y) in enumerate(zip(x_coords, y_coords)):
            print("  {}: ({:.1f}, {:.1f}) feet".format(i+1, x, y))
        
        # Send waypoints
        success = send_waypoints_direct(x_coords, y_coords)
        
        if success:
            print("Waypoints sent successfully!")
        else:
            print("Failed to send waypoints")
            
    except ValueError:
        print("Error: Invalid coordinates provided")
    except KeyboardInterrupt:
        print("\nCancelled by user")

if __name__ == '__main__':
    main()
