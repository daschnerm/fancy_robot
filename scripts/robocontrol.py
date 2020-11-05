#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class RoboControl:

    def __init__(self):
        rospy.init_node('listener', anonymous=True)
        self.scan_sub = rospy.Subscriber('ifl_turtlebot1/scan', LaserScan, self.scan_callback)
        self.odom_sub = rospy.Subscriber('ifl_turtlebot1/odom', Odometry, self.odom_callback)

        self.controll_pub = rospy.Publisher('/ifl_turtlebot1/cmd_vel', Twist, queue_size=10)
        
        self.front = 0
        self.left_30 = self.left_60 = self.left_90 = 0
        self.right_30 = self.right_60 = self.right_90 = 0

        self.desired_speed = 0.8
        self.actual_speed = 0

        self.z = 0
        self.x = 0


    def odom_callback(self, data):
        self.actual_speed = data.twist.twist.linear.x

    def scan_callback(self, data):
        ranges = data.ranges
        
        self.move(ranges)

        #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    def check_edges(self):
        z = None
        if self.left_30 < 0.15:
            z = self.turn_right
            print("To far left")
        if self.right_30 < 0.15:
            z = self.turn_left
            print("To far right")
        return z
    
    def check_inital_state(self):
        return self.front < 0.20 or (self.left_90 > 0.7 or self.right_90 > 0.7) and self.relative_speed < 0.2

    def move(self, ranges):
	x = self.x
	z = self.z

        self.relative_speed = abs(self.actual_speed) / self.desired_speed

        self.turn_left = 1.1 * self.relative_speed
        self.turn_right = -1.1 * self.relative_speed

        self.turn_left_init = 1 * self.desired_speed
        self.turn_right_init = -1 * self.desired_speed


        forward = self.desired_speed

        self.left_30 = ranges[29]
        self.left_60 = ranges[59]
        self.left_90 = ranges[89]

        self.front = ranges[0]

        self.right_30 = ranges[329]
        self.right_60 = ranges[299]
        self.right_90 = ranges[269]

        initial_state = self.check_inital_state()

        # Init | if 
        if initial_state:
            x = -0.1 * self.desired_speed
            if self.front > 0.45:
                z = 2
                x = 0
            print("init")
        elif self.left_90 < 0.15 or self.left_60 < 0.15:
            z = self.turn_right * 0.5
            if self.right_60 < 0.15:
                print("left_60")
            elif self.right_90 < 0.15:
                print("left_90")
        elif self.right_90 < 0.15 or self.right_60 < 0.15:
            z = self.turn_left * 0.5
            if self.right_60 < 0.15:
                print("right_60")
            elif self.right_90 < 0.15:
                print("right_90")
        elif self.front > 3.0 and self.left_30 > 3.0 and self.right_30  > 3.0:
            z = 0
            print("forward | inf")
        elif self.front > self.left_30 and self.front > self.right_30:
            z = 0
            print("forward")
        elif self.left_30 > self.front and self.left_30 > self.right_30:
            z = self.turn_left
            print("left")
        elif self.right_30 > self.front and self.right_30 > self.left_30:
            z = self.turn_right
            
            print("right")
        
	if x >= 0 and x < forward:
           x = x + (forward / 30)
           print("Accelerating " + str(x))

        z_emergency = self.check_edges()
        if not initial_state and z_emergency:
            z = z_emergency

	print("x: " + str(x) + "L " + str(self.left_90) + " " + str(self.left_30) + " | "
        + str(self.front) + " | "
        + str(self.right_30) + " " + str(self.right_90) + " R | s " + str(self.actual_speed) + " r:" + str(self.relative_speed) + " z: " + str(z))

        twist = Twist()

        twist.linear.x = x
        twist.angular.z = z
        self.controll_pub.publish(twist)
	self.x = x
 	self.z = z



if __name__ == '__main__':
    try:
        robo_control = RoboControl()
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
