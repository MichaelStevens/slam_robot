#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, String


def create_pose_handler():
  path_pub = rospy.Publisher('/path', Path, queue_size=10)
  poses = []

  def handler(odom):
    path = Path()


    path.header = odom.header
    pose_stamped = PoseStamped()
    pose_stamped.pose = odom.pose.pose
    pose_stamped.header = odom.header

    poses.append(pose_stamped)
    path.poses = poses


    path_pub.publish(path)


  return handler

if __name__ == '__main__':
  try:
    rospy.init_node('path_builder', anonymous=True)
    handler = create_pose_handler()
    rospy.Subscriber("/RosAria/pose", Odometry, handler)
    rospy.spin()
  except rospy.ROSInterruptException: pass
