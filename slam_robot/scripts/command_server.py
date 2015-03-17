#!/usr/bin/env python

from slam_robot.srv import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from time import sleep
from math import sin, cos, pi
import rospy

reqest_time = 0
ptu_request = SpinPtuRequest(0, 0, 0, 0, 0)

ptu_data = JointState()
ptu_data.header.frame_id = "map"
ptu_data.name = ["ptu_pan", "ptu_tilt"]
ptu_data.velocity = [0, 0]
ptu_data.position = [0, 0]

ptu_publisher = rospy.Publisher("/ptu/cmd", JointState, queue_size=10)

cmd_vel_pub = rospy.Publisher('/RosAria/cmd_vel', Twist, queue_size=10)
ZERO_TWIST = Twist()
ZERO_TWIST.linear.x = 0
ZERO_TWIST.linear.y = 0
ZERO_TWIST.linear.z = 0
ZERO_TWIST.angular.x = 0
ZERO_TWIST.angular.y = 0
ZERO_TWIST.angular.z = 0

def publish_ptu_data(data, request, ptu_pub):
    if ptu_data.velocity[0] < 0:
        ptu_data.velocity[0] = -ptu_data.velocity[0]
        ptu_data.position[0] = request.center_pan - request.amplitude_pan
    else:
        ptu_data.position[0] = request.center_pan + request.amplitude_pan

    if ptu_data.velocity[1] < 0:
        ptu_data.velocity[1] = -ptu_data.velocity[1]
        ptu_data.position[1] = request.center_tilt - request.amplitude_tilt
    else:
        ptu_data.position[1] = request.center_tilt + request.amplitude_tilt

    ptu_data.header.stamp = rospy.Time.now()
    ptu_pub.publish(ptu_data)

def handle_move(data):
    print "got: [%s, %s]" % (data.twist.linear.x, data.time)
    cmd_vel_pub.publish(data.twist)
    sleep(data.time)
    cmd_vel_pub.publish(ZERO_TWIST)
    return MoveResponse()

def handle_spin_ptu(data):
    global ptu_request, reqest_time

    # stop circleing
    ptu_request.speed = 0

    # if speed is 0 stop at return to (0, 0)
    if data.speed == 0:
        ptu_data.velocity = [0.6, 0.6]
        ptu_data.position = [0, 0]
        ptu_data.header.stamp = rospy.Time.now()
        ptu_publisher.publish(ptu_data)
        ptu_request = data
        reqest_time = rospy.Time.now()
        return SpinPtuResponse()

    # convert degrees to radians
    data.center_pan *= pi / 180.0
    data.center_tilt *= pi / 180.0
    data.amplitude_pan *= pi / 180.0
    data.amplitude_tilt *= pi / 180.0
    data.speed *= pi / 180.0

    #print "looking at center"
    #ptu_data.velocity = [0.6, 0.6]
    #ptu_data.position = [data.center_pan, data.center_tilt]
    #ptu_data.header.stamp = rospy.Time.now()
    #ptu_publisher.publish(ptu_data)
    #sleep(3)

    print "moving to edge"
    ptu_data.position = [data.center_pan, data.center_tilt + data.amplitude_tilt]
    ptu_data.velocity = [0.6, 0.6]
    ptu_data.header.stamp = rospy.Time.now()
    ptu_publisher.publish(ptu_data)
    sleep(3)

    print "begin circle"
    ptu_request = data
    reqest_time = rospy.Time.now()
    return SpinPtuResponse()

if __name__ == "__main__":
    rospy.init_node('command_server')

    # init move server
    s0 = rospy.Service('command_server/move', Move, handle_move)
    print "Move server ready"

    # init ptu server
    s1 = rospy.Service('command_server/spin_ptu', SpinPtu, handle_spin_ptu)
    print "PTU server ready"
    print "Ready"

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if ptu_request.speed != 0:
            delta_t = (rospy.Time.now() - reqest_time).to_sec()

            ptu_data.velocity[0] = ptu_request.amplitude_pan * ptu_request.speed * cos(ptu_request.speed * delta_t)
            ptu_data.velocity[1] = ptu_request.amplitude_tilt * ptu_request.speed * -sin(ptu_request.speed * delta_t)
            publish_ptu_data(ptu_data, ptu_request, ptu_publisher)
            if delta_t >= 1 / (ptu_request.speed / (2*pi)):
                ptu_request.speed = 0
                ptu_data.velocity = [0, 0]
                ptu_data.position = [ptu_request.center_pan, ptu_request.center_tilt + ptu_request.amplitude_tilt]
                ptu_data.header.stamp = rospy.Time.now()
                ptu_publisher.publish(ptu_data)
                print 'finished'
        rate.sleep()
