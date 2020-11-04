#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan

def callback(data):
    ranges = data.ranges
    # 267 -> left, 0 -> middle, 89 -> right
    print(str(ranges[267]) + " " + str(ranges[0]) + " " + str(ranges[89]) )
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

     pub = rospy.Publisher('/ifl_turtlebot1/cmd_vel', Twist, queue_size=10)


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
