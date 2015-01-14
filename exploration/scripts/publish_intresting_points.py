#!/usr/bin/env python
import roslib
roslib.load_manifest('exploration')
import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PointStamped
from exploration.msg import PointList
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge, CvBridgeError
from attention import attention_map

class MovingAverage:
    def __init__(self, n):
        self.n = n
        self.values = [0]

    def update(self, x):
        if len(self.values) < self.n:
            self.values.append(float(x))
        else:
            self.values = self.values[1:] + [x]

    def value(self):
        return sum(self.values) / len(self.values)


class IntPntFinder:
    def __init__(self):
        self.point_pub = rospy.Publisher("attention_points", PointList, queue_size=10)
        self.int_point_pub = rospy.Publisher("attention_point_markers", Marker, queue_size=10)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_mono", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.camera_image = None
        self.depth_image = None
        self.fx = 572.882768
        self.fy = 542.739980
        self.cx = 314.649173
        self.cy = 240.160459
        self.x_smooth = MovingAverage(20)
        self.y_smooth = MovingAverage(20)
        self.z_smooth = MovingAverage(20)

    def camera_callback(self, data):
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e

        if self.depth_image != None:
            attention_image = attention_map(self.camera_image)
            p = np.argmax(attention_image)
            (x, y, z) = self.index2point(p)

            if not (x == 0 and y == 0 and z == 0) and y > 0:
                self.x_smooth.update(x)
                self.y_smooth.update(y)
                self.z_smooth.update(z)

            point = PointStamped()
            point.header.frame_id = "/camera_depth_optical_frame"
            # TODO this stamp uses Time(0) to avoid the tf time difference problem
            #      is there a better way of doing this?
            point.header.stamp = rospy.Time(0)
            print point.header
            point.header.stamp.secs -= 0
            point.point.x = self.x_smooth.value()
            point.point.y = self.y_smooth.value()
            point.point.z = self.z_smooth.value()
            pointList = PointList()
            pointList.points = [point]
            self.point_pub.publish(pointList)

            # publish markers for visualization
            marker = Marker()
            marker.header.frame_id = "/camera_depth_optical_frame"
            marker.header.stamp = rospy.get_rostime()
            marker.id = 0
            marker.type = Marker.SPHERE_LIST
            marker.action = Marker.ADD
            marker.color.a = 1.0
            marker.color.r = 255
            marker.color.g = 1
            marker.color.b = 1
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.points = [p.point for p in pointList.points]
            self.int_point_pub.publish(marker)



    def depth_callback(self, data):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e
            return

    def index2point(self, index):
        Z = self.depth_image.item(index)
        x = index % self.depth_image.shape[1]
        y = int(index / self.depth_image.shape[1])
        X = (Z / self.fx) * (x - self.cx)
        Y = (Z / self.fy) * (y - self.cy)
        return (X / 1000., Y / 1000., Z / 1000.)

def main(args):
  intPntFinder = IntPntFinder()
  rospy.init_node('int_point_pub', anonymous=True)

  try:
    rospy.spin()
  except KeyboardInterrupt:
    print "Shutting down"


if __name__ == '__main__':
    main(sys.argv)
