#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
    ranges = data.ranges

    left_30 = ranges[29]
    left_60 = ranges[59]
    left_90 = ranges[89]

    front = ranges[0]

    right_30 = ranges[329]
    right_60 = ranges[299]
    right_90 = ranges[269]
    

    # 267 -> left, 0 -> middle, 89 -> right
    print("L " + str(left_90) + " " + str(left_60) + " " + str(left_30) + " | "
     + str(front) + " | "
     + str(right_30) + " " + str(right_60) + " " + str(right_90) + " R")
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)


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

if __name__ == '__main__':
    listener()
