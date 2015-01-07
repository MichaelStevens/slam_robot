#!/usr/bin/env python
import roslib
roslib.load_manifest('exploration')
import sys
import rospy
import tf
from exploration.msg import PointList

int_pt = None
def make_int_pt_callback(listener):
    def int_pt_callback(data):
        global int_pt
        int_pt = listener.transformPoint("map", data.points[0])

    return int_pt_callback



def main(args):
    rospy.init_node('explore', anonymous=True)
    listener = tf.TransformListener()
    callback = make_int_pt_callback(listener)
    int_pt_sub = rospy.Subscriber("/interesting_points", PointList, callback)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if int_pt != None and int_pt_sub != None:
            pass
            #int_pt_sub.unregister()
            #int_pt_sub = None
        elif int_pt != None and int_pt_sub == None:
            pass
        print int_pt



        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
