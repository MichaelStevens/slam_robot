#!/usr/bin/env python
import roslib
roslib.load_manifest('exploration')
import sys
import rospy
import tf
import actionlib
import math
from exploration.msg import PointList
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus


int_pt = None
def make_int_pt_callback(listener):
    def int_pt_callback(data):
        global int_pt
        int_pt = listener.transformPoint("map", data.points[0])

    return int_pt_callback

def robot_distance2(point, listener):
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    return math.sqrt((point.point.x - trans[0])**2 + (point.point.y - trans[1])**2)




def main(args):
    rospy.init_node('explore', anonymous=True)

    # tf listner
    listener = tf.TransformListener()

    # subscribe to interesting points
    callback = make_int_pt_callback(listener)
    int_pt_sub = rospy.Subscriber("/interesting_points", PointList, callback)
    ac_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac_client.wait_for_server()


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if int_pt != None and int_pt_sub != None:
            # stop subscribing to interesting points
            int_pt_sub.unregister()
            int_pt_sub = None

            # send goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose.position.x = int_pt.point.x
            goal.target_pose.pose.position.y = int_pt.point.y
            goal.target_pose.pose.orientation.w = 1.0


            ac_client.send_goal(goal)

        if int_pt == None: continue

        distance = robot_distance2(int_pt, listener)
        if distance != None:
            print "%s away from target!" % (distance)
            if distance <= 1.3:
                if ac_client != None:
                    ac_client.cancel_goal()
                    ac_client = None
                print "finished!"
                



        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
