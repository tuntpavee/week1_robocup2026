#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

current_pose = Pose()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def main():
    rospy.init_node('stop_at_10')
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    rate = rospy.Rate(10)
    cmd_vel = Twist()
    
    while not rospy.is_shutdown():
        # CHALLENGE 1 LOGIC:
        if current_pose.x >= 10.0:
            rospy.loginfo("Reached x=10! Stopping.")
            cmd_vel.linear.x = 0.0
            pub.publish(cmd_vel)
            break
        else:
            cmd_vel.linear.x = 1.0 # Move forward
            pub.publish(cmd_vel)
            
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass