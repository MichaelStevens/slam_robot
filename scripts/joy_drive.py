#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

MAX_LINEAR_SPEED = 0.5
MAX_ANGULAR_SPEED = 1

def create_msg_handler():
  cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
  def handler(data):
    twist = Twist()
    twist.linear.x = MAX_LINEAR_SPEED * data.axes[1]
    twist.linear.y = 0
    twist.linear.z = 0
    twist.angular.x = 0
    twist.angular.y = 0
    twist.angular.z = MAX_ANGULAR_SPEED * data.axes[2]

    cmd_vel_pub.publish(twist)

  return handler

if __name__ == '__main__':
  try:
    rospy.init_node('tank_drive', anonymous=True)
    handler = create_msg_handler()
    rospy.Subscriber("/joy", Joy, handler)
    rospy.spin()
  except rospy.ROSInterruptException: pass
