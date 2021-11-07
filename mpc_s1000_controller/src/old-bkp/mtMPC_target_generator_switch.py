#!/usr/bin/env python3
# ROS python API
import rospy
from geometry_msgs.msg import Point

x_goal = Point()

# Main function
def main():
    # initiate node
    rospy.init_node('targetGen', anonymous=True)
    # ROS loop rate
    rate = rospy.Rate(20.0)
    target_goal_pub = rospy.Publisher('/mtMPC/target',Point,queue_size=1)
    x_goal.x = 30#10
    x_goal.y = -25#-5
    x_goal.z = 10#5
    #x_goal.x = 50
    #x_goal.y = -50
    #x_goal.z = 20
    while not rospy.is_shutdown():
    # This is where we calculate the trajectory
        target_goal_pub.publish(x_goal)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
