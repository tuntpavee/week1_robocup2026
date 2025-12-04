#!/usr/bin/env python3
import rospy
import csv
import math
import os
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
# from turtlesim.ServiceProxy import SetPen

# Global variables
current_pose = Pose()
cmd_vel = Twist()

def pose_callback(msg):
    """Callback function to update the turtle's current position."""
    global current_pose
    current_pose = msg

def get_waypoints_from_csv(filename):
    """Reads x, y, theta from CSV."""
    waypoints = []
    script_dir = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(script_dir, filename)
    
    try:
        with open(file_path, 'r') as csvfile:
            csv_reader = csv.reader(csvfile)
            for row in csv_reader:
                if len(row) >= 3:
                    point = {
                        'x': float(row[0].strip()),
                        'y': float(row[1].strip()),
                        'theta': float(row[2].strip())
                    }
                    waypoints.append(point)
        rospy.loginfo(f"Loaded {len(waypoints)} waypoints from {filename}")
    except Exception as e:
        rospy.logerr(f"Error reading CSV: {e}")
    return waypoints

def normalize_angle(angle):
    """Keeps angles within -pi to pi range to prevent spinning the long way."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def rotate_to_heading(target_theta, publisher, rate):
    """Rotates the turtle to a specific heading (theta) in place."""
    global cmd_vel, current_pose
    
    rospy.loginfo(f"Rotating to heading: {target_theta}")
    
    while not rospy.is_shutdown():
        # Calculate angular error
        angle_error = normalize_angle(target_theta - current_pose.theta)
        
        # Stop if we are close enough (tolerance 0.01 rad)
        if abs(angle_error) < 0.01:
            break
            
        # P-Controller for rotation
        cmd_vel.linear.x = 0.0
        cmd_vel.angular.z = 2.0 * angle_error
        
        publisher.publish(cmd_vel)
        rate.sleep()

def move_to_goal(target_x, target_y, publisher, rate):
    """Drives the turtle to the X, Y coordinates."""
    global cmd_vel, current_pose
    
    rospy.loginfo(f"Driving to: x={target_x}, y={target_y}")
    
    while not rospy.is_shutdown():
        # Calculate distance to goal
        delta_x = target_x - current_pose.x
        delta_y = target_y - current_pose.y
        distance = math.sqrt(delta_x**2 + delta_y**2)
        
        # Stop if we are close enough (tolerance 0.1)
        if distance < 0.1:
            break
            
        # Calculate steering angle towards the target
        path_angle = math.atan2(delta_y, delta_x)
        angle_error = normalize_angle(path_angle - current_pose.theta)
        
        # P-Controller:
        # Linear speed depends on distance
        cmd_vel.linear.x = 1.5 * distance
        # Angular speed depends on how much we need to turn to face the target
        cmd_vel.angular.z = 6.0 * angle_error
        
        publisher.publish(cmd_vel)
        rate.sleep()

def main():
    global cmd_vel
    rospy.init_node('closed_loop_square')
    
    # Subscribe to pose
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    
    # Publisher for movement
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    # Wait for simulator connection
    rospy.loginfo("Waiting for pose...")
    rospy.wait_for_message('/turtle1/pose', Pose)
    
    rate = rospy.Rate(10)
    set_pen_service = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    set_pen_service(0, 0, 0, 3, 0)
    
    # 1. Load Goals
    goals = get_waypoints_from_csv('waypoints.csv')
    
    # 2. Logic to Close the Loop
    # If the last point is not the same as the first, append the first point.
    if goals:
        first_point = goals[0]
        last_point = goals[-1]
        
        # Check if coordinates match
        if (first_point['x'] != last_point['x']) or (first_point['y'] != last_point['y']):
            rospy.loginfo("Adding closing waypoint to loop back to start.")
            goals.append(first_point)

    # 3. Execution Loop
    for goal in goals:
        # Step A: Drive to the X, Y position
        move_to_goal(goal['x'], goal['y'], pub, rate)
        
        # Step B: Once arrived, rotate to match the desired Theta
        rotate_to_heading(goal['theta'], pub, rate)
        
    # Stop everything when done
    rospy.loginfo("Path Finished. Loop Closed.")
    cmd_vel.linear.x = 0
    cmd_vel.angular.z = 0
    pub.publish(cmd_vel)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass