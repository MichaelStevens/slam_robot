#!/usr/bin/env python
import roslib
roslib.load_manifest('exploration')
import sys
import rospy
import tf
import actionlib
import math
from math import sqrt, acos
from exploration.msg import PointList
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState

class RobotHead:
    def __init__(self, tf_listener):
        self.tf_listener = tf_listener
        self.ptu_publisher = rospy.Publisher("/ptu/cmd", JointState)
        # TODO for whatever reason, the ptu driver fails to publish state
        #      so this class only works if ptu starts from (0, 0)
        self.ptu_subscriber = rospy.Subscriber("/ptu/state", JointState, self.update_ptu_angles)
        self.ptu_pan = 0
        self.ptu_tilt = 0

    def update_ptu_angles(self, data):
        self.ptu_pan = data.position[0]
        self.ptu_tilt = data.position[1]

    def reset(self):
        msg = JointState()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.name = ["ptu_pan", "ptu_tilt"]
        msg.velocity = [0.6, 0.6]
        msg.position = [0, 0]
        print "setting position : ", msg.position
        self.ptu_publisher.publish(msg)

    def look_at(self, point):
        # transform point into kinect frame
        point_kinect = self.tf_listener.transformPoint("/camera_rgb_optical_frame", point)

        # make vector v from origin to point_kinect
        v = (point_kinect.point.x, point_kinect.point.y, point_kinect.point.z)

        # find angle between the projection of v onto the xz axis and the z axis
        mag_v_xz = sqrt(v[0]**2 + v[2]**2)
        pan = acos(v[2] / mag_v_xz)

        # find the angle between the projection and v
        mag_v = sqrt(v[0]**2 + v[1]**2 + v[2]**2)
        tilt = acos((v[0]**2 + v[2]**2) / (mag_v * mag_v_xz))

        print "at %s, %s" % (self.ptu_pan, self.ptu_tilt)
        print "pan: %s, tilt: %s" % (pan, tilt)

        # publish angle values to the ptu
        msg = JointState()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.name = ["ptu_pan", "ptu_tilt"]
        msg.velocity = [0.6, 0.6]
        msg.position = [self.ptu_pan-pan, self.ptu_tilt-tilt]
        print "setting position : ", msg.position
        self.ptu_publisher.publish(msg)


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

    # create function to move Kinect
    head = RobotHead(listener)

    # subscribe to attention points
    callback = make_int_pt_callback(listener)
    int_pt_sub = rospy.Subscriber("/attention_points", PointList, callback)
    ac_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac_client.wait_for_server()


    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if int_pt != None and int_pt_sub != None:
            # stop subscribing to attention points
            int_pt_sub.unregister()
            int_pt_sub = None
            head.look_at(int_pt)


            # send goal
            #goal = MoveBaseGoal()
            #goal.target_pose.header.frame_id = "map"
            #goal.target_pose.header.stamp = rospy.Time.now()
            #goal.target_pose.pose.position.x = int_pt.point.x
            #goal.target_pose.pose.position.y = int_pt.point.y
            #goal.target_pose.pose.orientation.w = 1.0


            #ac_client.send_goal(goal)

        #if int_pt == None: continue

        #distance = robot_distance2(int_pt, listener)
        #if distance != None:
        #    print "%s away from target!" % (distance)
        #    if distance <= 1.3:
        #        if ac_client != None:
        #            ac_client.cancel_goal()
        #            ac_client = None
        #        print "finished!"




        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
