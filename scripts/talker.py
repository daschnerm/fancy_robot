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
        # 1 / 2 relation

        twist.linear.x = 0.1
        twist.angular.z = 0.2
        #twist.angular.z = 0.02

        # if ranges[29] 


        rospy.loginfo(twist)
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
