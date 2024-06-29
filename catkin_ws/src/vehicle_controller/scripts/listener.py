#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import board
import digitalio
from adafruit_pca9685 import PCA9685

i2c = board.I2C() 
pca = PCA9685(i2c)
pca.frequency = 60

all_wheels_speed = 0x8FFF

#Right Front wheel RPM
pca.channels[2].duty_cycle = all_wheels_speed
#Right rear wheel RPM
pca.channels[3].duty_cycle = all_wheels_speed
#Left rear wheel PWM
pca.channels[1].duty_cycle = all_wheels_speed
#Left front wheel RPM
pca.channels[0].duty_cycle = all_wheels_speed

#Outputs - H-bridges
w1_rf = digitalio.DigitalInOut(board.D19)
w2_rf = digitalio.DigitalInOut(board.D26)
w1_rr = digitalio.DigitalInOut(board.D6)
w2_rr = digitalio.DigitalInOut(board.D13)
w1_lr = digitalio.DigitalInOut(board.D9)
w2_lr = digitalio.DigitalInOut(board.D10)
w1_lf = digitalio.DigitalInOut(board.D5)
w2_lf = digitalio.DigitalInOut(board.D11)


#Setting as outputs
w1_rf.direction = digitalio.Direction.OUTPUT
w2_rf.direction = digitalio.Direction.OUTPUT
w1_rr.direction = digitalio.Direction.OUTPUT
w2_rr.direction = digitalio.Direction.OUTPUT
w1_lr.direction = digitalio.Direction.OUTPUT
w2_lr.direction = digitalio.Direction.OUTPUT
w1_lf.direction = digitalio.Direction.OUTPUT
w2_lf.direction = digitalio.Direction.OUTPUT

def callback(data: Twist):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data)

    if data.linear.x > 0:
        w1_lf.value = False
        w2_lf.value = True
        w1_lr.value = False
        w2_lr.value = True
        w1_rf.value = False
        w2_rf.value = True
        w1_rr.value = False
        w2_rr.value = True
    else:
        w1_lf.value = False
        w2_lf.value = False
        w1_lr.value = False
        w2_lr.value = False
        w1_rf.value = False
        w2_rf.value = False
        w1_rr.value = False
        w2_rr.value = False  

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('/cmd_vel', Twist, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()