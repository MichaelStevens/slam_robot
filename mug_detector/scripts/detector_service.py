#!/usr/bin/env python
import roslib
roslib.load_manifest('mug_detector')
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from mug_detector.srv import *
import detector

class Detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber("/camera/depth/image_rect_raw", Image, self.depth_callback)
        self.camera_image = None
        self.depth_image = None
        self.fx = 572.882768
        self.fy = 542.739980
        self.cx = 314.649173
        self.cy = 240.160459
        detector.init_detector()
        rospy.Service('get_mug_detections', Detections, self.get_mug_detections)


    def get_mug_detections(self, data):
        if self.camera_image != None and self.depth_image != None:
            image = cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB)
            image = image.astype('float32') / 255.0
            print "getting detections..."
            dets = detector.detect_mugs(image)
            print "done!"
            #import pdb; pdb.set_trace()
            points = []
            for det in dets:
                # find center of the square
                x = int((det[0] + det[2])/2.)
                y = int((det[1] + det[3])/2.)

                # convert the subscripts to an index
                i = np.ravel_multi_index((y, x), self.depth_image.shape)

                # convert the index to a 3d point
                (x, y, z) = self.index2point(i)
                points.append(Point(x, y, z))
            return DetectionsResponse(points)
        else:
            print "No camera or depth image!"
            return DetectionsResponse()

    def camera_callback(self, data):
        try:
            self.camera_image = self.bridge.imgmsg_to_cv2(data)
        except CvBridgeError, e:
            print e

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
    rospy.init_node('mug_detector', anonymous=True)
    detector = Detector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"



if __name__ == '__main__':
    main(sys.argv)
