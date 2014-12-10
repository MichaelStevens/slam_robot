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
from matplotlib import pyplot as plt


class image_converter:
  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    cv2.namedWindow("Image window", 1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      img_orig = self.bridge.imgmsg_to_cv2(data, "mono8")
    except CvBridgeError, e:
      print e

    img = img_orig
    #img = np.uint8((img_orig / 4000.0) * 255.0)
    #cv2.cv.ConvertScale(cv_image, cv_image, 1.0/256.0, 0)

    #gray_img = cv_image / 4000.0#cv2.cvtColor(cv_image, cv2.cv.CV_BGR2GRAY)

    img = cv2.GaussianBlur(img, (9, 9), 3)
    img = cv2.Canny(img,80,70)

    img2 =  cv2.Laplacian(img_orig,cv2.CV_8U)
    #img_x = cv2.Sobel(cv_image, cv2.CV_8U, 1,0,ksize=5)
    #img_y = cv2.Sobel(cv_image, cv2.CV_8U, 0,1,ksize=5)
    #img_x = cv2.convertScaleAbs(img_x)
    #img_y = cv2.convertScaleAbs(img_y)
    #img = cv2.addWeighted(img_x, 0.5, img_y, 0.5, 0)
    #img = cv2.bitwise_and(img1, img1, mask = img2)
    #img = cv2.inRange(img, 40, 50)
    img = cv2.blur(img, (10, 10)) * 12
    img = cv2.GaussianBlur(img, (129, 129), 20)
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
