#!/usr/bin/env python

from slam_robot.srv import *
from geometry_msgs.msg import Twist
from time import sleep
import rospy

cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
ZERO_TWIST = Twist()
ZERO_TWIST.linear.x = 0
ZERO_TWIST.linear.y = 0
ZERO_TWIST.linear.z = 0
ZERO_TWIST.angular.x = 0
ZERO_TWIST.angular.y = 0
ZERO_TWIST.angular.z = 0

def handle_move(data):
    print "got: [%s, %s]" % (data.twist.linear.x, data.time)
    cmd_vel_pub.publish(data.twist)
    sleep(data.time)
    cmd_vel_pub.publish(ZERO_TWIST)
    return MoveResponse()

def move_server():
    rospy.init_node('command_server')
    s = rospy.Service('command_server/move', Move, handle_move)
    print "Ready"
    rospy.spin()

if __name__ == "__main__":
    move_server()
