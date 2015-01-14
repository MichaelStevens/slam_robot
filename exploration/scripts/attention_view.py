#!/usr/bin/env python
import roslib
roslib.load_manifest('slam_robot')
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from attention import attention_map


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_mono",Image,self.callback)

  def callback(self,data):
    try:
      img_orig = self.bridge.imgmsg_to_cv2(data)
    except CvBridgeError, e:
      print e

    img = attention_map(img_orig)
    p = np.argmax(img)

    cv2.circle(img_orig, (p % img.shape[1], int(p / img.shape[1]),),50, 200, 5)


    cv2.imshow("Image window", img_orig)
    cv2.waitKey(3)


    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(img))
    except CvBridgeError, e:
      print e

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
