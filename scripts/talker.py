#!/usr/bin/env python


import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist

def talker():
    rospy.init_node('turtlebot3_teleop')
    pub = rospy.Publisher('/ifl_turtlebot1/cmd_vel', Twist, queue_size=10)

    twist = Twist()


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twist.linear.x = 1

        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
