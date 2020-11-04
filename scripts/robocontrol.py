#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan



def callback(data):
    ranges = data.ranges
    
    move(ranges)

    # 267 -> left, 0 -> middle, 89 -> right
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)


def move(ranges):
    z = 0
    x = 0

    turn_left = 0.5
    turn_right = -0.5

    left_30 = ranges[29]
    left_60 = ranges[59]
    left_90 = ranges[89]

    front = ranges[0]

    right_30 = ranges[329]
    right_60 = ranges[299]
    right_90 = ranges[269]

    print("L " + str(left_90) + " " + str(left_60) + " " + str(left_30) + " | "
     + str(front) + " | "
     + str(right_30) + " " + str(right_60) + " " + str(right_90) + " R")

    if front > 3.0 and left_30 > 3.0 and right_30  > 3.0:
        z = 0
        x = 0.1
    elif front > left_30 and front > right_30:
        z = 0
        x = 0.1
    elif left_30 > front and left_30 > right_30:
        z = turn_left
        x = 0.1
    elif right_30 > front and right_30 > left_30:
        z = turn_right
        x = 0.1

        

    twist = Twist()

    twist.linear.x = x
    twist.angular.z = z
    pub.publish(twist)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('ifl_turtlebot1/scan', LaserScan, callback)
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

pub = rospy.Publisher('/ifl_turtlebot1/cmd_vel', Twist, queue_size=10)

if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
