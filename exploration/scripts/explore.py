#!/usr/bin/env python
import roslib
roslib.load_manifest('exploration')
import sys
import rospy
import tf
import actionlib
import math
from math import sqrt, acos, pi
from exploration.msg import PointList
from move_base_msgs.msg import MoveBaseGoal, MoveBaseAction
from actionlib_msgs.msg import GoalStatus
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty

class AttentionPointFinder:
    def __init__(self, tf_listener, robot_head):
        self.tf_listener = tf_listener
        self.old_points = []
        self.robot_head = robot_head
        self.attention_point_sub = rospy.Subscriber("/attention_points", PointList, self.attention_point_callback)
        self.attention_point = None
        self.dist_thresh = 0.5
        rospy.wait_for_service('clear_attention_smoother')
        self.clear_attention_smoother = rospy.ServiceProxy('clear_attention_smoother', Empty)




    def attention_point_callback(self, data):
        if len(data.points) > 0:
            self.attention_point = self.tf_listener.transformPoint("map", data.points[0])
        else:
            self.attention_point = None

    def distance(self, p1, p2):
        sum = (p1.point.x - p2.point.x)**2
        sum += (p1.point.y - p2.point.y)**2
        sum += (p1.point.z - p2.point.z)**2
        return sqrt(sum)

    def find_attention_points(self):
        # create array to store all attention points found
        points = []

        # Check forward
        self.robot_head.reset()
        rospy.sleep(5)
        self.clear_attention_smoother()
        rospy.sleep(3)
        if self.attention_point != None:
            points.append(self.attention_point)
        print "Found point 1"

        # check left
        self.robot_head.rotate(pi / 2, 0)
        rospy.sleep(5)
        self.clear_attention_smoother()
        rospy.sleep(3)
        if self.attention_point != None:
            points.append(self.attention_point)
        print "Found point 2"

        # check right
        self.robot_head.rotate(-pi / 2, 0)
        rospy.sleep(8)
        self.clear_attention_smoother()
        rospy.sleep(3)
        if self.attention_point != None:
            points.append(self.attention_point)
        print "Found point 3"
        self.robot_head.reset()

        # [float("inf")] is appended to the begining of the list so an empty list will not cause an error
        point_filter = lambda p: min([float("inf")] + [self.distance(p, old) for old in self.old_points]) > self.dist_thresh

        points = [p for p in points if point_filter(p)]
        print "filtered down to %s points" % (len(points))
        self.old_points.extend(points)
        return points


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
        self.rotate(0, 0)


    def rotate(self, pan, tilt):
        # publish angle values to the ptu
        msg = JointState()
        msg.header.frame_id = "map"
        msg.header.stamp = rospy.Time.now()
        msg.name = ["ptu_pan", "ptu_tilt"]
        msg.velocity = [0.6, 0.6]
        msg.position = [pan, tilt]
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
        self.rotate(self.ptu_pan-pan, self.ptu_tilt-tilt)


def robot_distance2(point, listener):
    try:
        (trans,rot) = listener.lookupTransform('/map', '/base_link', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None
    return math.sqrt((point.point.x - trans[0])**2 + (point.point.y - trans[1])**2)

def drive_to_point(point, ac_client, listener, thresh):
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = point.point.x
    goal.target_pose.pose.position.y = point.point.y
    goal.target_pose.pose.orientation.w = 1.0
    ac_client.send_goal(goal)
    distance = thresh + 1
    while distance > thresh:
        distance = robot_distance2(point, listener)


    print 'cancel goal'
    ac_client.cancel_goal()




def main(args):
    rospy.init_node('explore', anonymous=True)

    # tf listner
    listener = tf.TransformListener()

    # create function to move Kinect
    head = RobotHead(listener)
    attention_finder = AttentionPointFinder(listener, head)

    # this ac_client allows us to send commands to the nav stack
    ac_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    ac_client.wait_for_server()

    print 'finding points...'

    points = attention_finder.find_attention_points()

    while len(points) > 0:
        for point in points:
            print 'driving to point'
            drive_to_point(points[0], ac_client, listener, 1)
            head.look_at(points[0])
            rospy.sleep(1)
            head.reset()
        points = attention_finder.find_attention_points()

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main(sys.argv)
    except rospy.ROSInterruptException:
        pass
