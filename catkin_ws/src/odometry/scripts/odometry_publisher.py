#!/usr/bin/env python3

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import digitalio
import board

#Inputs - motor encoders
enc_rf = digitalio.DigitalInOut(board.D4)
enc_rr = digitalio.DigitalInOut(board.D17)
enc_lr = digitalio.DigitalInOut(board.D22)
enc_lf = digitalio.DigitalInOut(board.D27)
#Setting as inputs
enc_rf.direction = digitalio.Direction.INPUT
enc_rr.direction = digitalio.Direction.INPUT
enc_lr.direction = digitalio.Direction.INPUT
enc_lf.direction = digitalio.Direction.INPUT
#Wheel properties
pulses_per_rev = 20
wheel_diameter = 8
tick = math.pi * wheel_diameter / 20 / 100
ang_tick_coeff = 1.0
tick_angular = ang_tick_coeff * math.pi / 2 / 37

l_x = 0.208
l_y = 0.104

linear = 0



cmd_x = 0
cmd_y = 0
cmd_z = 0


def callback(cmd_vel: Twist):
    global cmd_x
    global cmd_y
    global cmd_z
    cmd_x = cmd_vel.linear.x
    cmd_y = cmd_vel.linear.y
    cmd_z = cmd_vel.angular.z


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
odom_sub = rospy.Subscriber("/cmd_vel", Twist, callback)
odom_broadcaster = tf.TransformBroadcaster()

x = 0.0
y = 0.0
th = 0.0

delta_x = 0
delta_y = 0
delta_th = 0

vx = 0.0
vy = 0.0
vth = 0.0

current_time = rospy.Time.now()
last_time = rospy.Time.now()

curr_num_pulses_lf = 0
prev_value_lf = False
curr_num_pulses_lr = 0
prev_value_lr = False
curr_num_pulses_rf = 0
prev_value_rf = False
curr_num_pulses_rr = 0
prev_value_rr = False

r = rospy.Rate(50.0)
while not rospy.is_shutdown():
    current_time = rospy.Time.now()

    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()


    
    curr_value_lf = enc_lf.value
    curr_value_lr = enc_lr.value
    curr_value_rf = enc_rf.value
    curr_value_rr = enc_rr.value

    if enc_lf.value == True and curr_value_lf != prev_value_lf:
        if cmd_x > 0 or cmd_y < 0 or (cmd_x > 0 and cmd_y < 0) or cmd_z < 0:
            curr_num_pulses_lf += 1
        else:
            curr_num_pulses_lf -= 1
    prev_value_lf = curr_value_lf

    if enc_rf.value == True and curr_value_rf != prev_value_rf:
        if cmd_x > 0 or cmd_y > 0 or (cmd_x > 0 and cmd_y >0) or (cmd_z > 0 and cmd_x == 0):
            curr_num_pulses_rf +=1
        else:
            curr_num_pulses_rf -=1
    prev_value_rf = curr_value_rf

    if enc_lr.value == True and curr_value_lr != prev_value_lr:
        if cmd_x > 0 or cmd_y > 0 or (cmd_x > 0 and cmd_y > 0) or cmd_z < 0:
            curr_num_pulses_lr += 1
        else:
            curr_num_pulses_lr -= 1
    prev_value_lr = curr_value_lr

    if enc_rr.value == True and curr_value_rr != prev_value_rr:
        if cmd_x > 0 or cmd_y < 0 or (cmd_x > 0 and cmd_y < 0) or (cmd_z > 0 and cmd_x == 0):
            curr_num_pulses_rr += 1
        else:
            curr_num_pulses_rr -= 1
    prev_value_rr = curr_value_rr


   
    delta_x = tick * 0.25 *  cos(th) * (curr_num_pulses_lf + curr_num_pulses_rf + curr_num_pulses_lr + curr_num_pulses_rr) - tick * 0.25 * sin(th) * (- curr_num_pulses_lf + curr_num_pulses_rf + curr_num_pulses_lr - curr_num_pulses_rr)
    delta_y = tick * 0.25 *  sin(th) * (curr_num_pulses_lf + curr_num_pulses_rf + curr_num_pulses_lr + curr_num_pulses_rr) + tick * 0.25 * cos(th) * (- curr_num_pulses_lf + curr_num_pulses_rf + curr_num_pulses_lr - curr_num_pulses_rr)
    delta_th = tick * 0.25 * ( - curr_num_pulses_lf / (l_x + l_y) + curr_num_pulses_rf / (l_x + l_y) - curr_num_pulses_lr / (l_x +l_y) + curr_num_pulses_rr / (l_x + l_y))

    curr_num_pulses_lf = 0
    curr_num_pulses_lr = 0
    curr_num_pulses_rf = 0
    curr_num_pulses_rr = 0

    x += delta_x
    y += delta_y
    th += delta_th
    if cmd_x >= 0.23:
        vx = 0.23
    elif cmd_x < 0.23 and cmd_x > 0.16:
        vx = cmd_x
    elif cmd_x < 0.16 and cmd_x > 0:
        vx = 0.16
    elif cmd_x <= -0.16 and cmd_x > -0.23:
        vx = cmd_x
    elif cmd_x <= -0.23:
        vx = -0.23
    elif cmd_x > -0.16 and cmd_x < 0:
        cmd_x = -0.16

    if cmd_y >= 0.23:
        vy = 0.23
    elif cmd_y < 0.23 and cmd_y > 0.16:
        vy = cmd_y
    elif cmd_y < 0.16 and cmd_y > 0:
        vy = 0.16
    elif cmd_y <= -0.16 and cmd_y > -0.23:
        vy = cmd_y
    elif cmd_y <= -0.23:
        vy = -0.23
    elif cmd_y > -0.16 and cmd_y < 0:
        cmd_y = -0.16

    vth = cmd_z

    rospy.loginfo(x) 
    rospy.loginfo(y)
     
    
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

    # first, we'll publish the transform over tf
    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    r.sleep()
