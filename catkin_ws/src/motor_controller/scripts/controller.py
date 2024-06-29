#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import board
import digitalio
from adafruit_pca9685 import PCA9685

i2c = board.I2C() 
pca = PCA9685(i2c)
pca.frequency = 500

#Dimensions
r = 0.04
l_x = 0.208
l_y = 0.104

min_ang_vel = 4


#Right Front wheel RPM
pca.channels[2].duty_cycle = 15400
#Right rear wheel RPM
pca.channels[3].duty_cycle = 14500
#Left rear wheel PWM
pca.channels[1].duty_cycle = 15800
#Left front wheel RPM
pca.channels[0].duty_cycle = 15000

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

def callback(cmd_vel: Twist):
    # rospy.loginfo(cmd_vel)
    angular_coord = cmd_vel.angular
    linear_coord = cmd_vel.linear

    v_x = linear_coord.x
    v_y = linear_coord.y
    w_z = angular_coord.z

    if w_z != 0:
        min_ang_vel = 4.4
    else:
        min_ang_vel = 4.8

    #Stop the motors
    if linear_coord.x == 0 and linear_coord.y == 0 and angular_coord.z == 0:
        w1_rf.value = False
        w2_rf.value = False
        w1_rr.value = False
        w2_rr.value = False
        w1_lr.value = False
        w2_lr.value = False
        w1_lf.value = False
        w2_lf.value = False

    else:

        #Wheel 1

        wheel_1_w = min(5.6, v_x / r + - v_y / r - (w_z * (l_x + l_y) / r))
        if wheel_1_w < 0:
            wheel_1_backwards = True
            wheel_1_w = max(min_ang_vel,min(5.6, abs(v_x / r + - v_y / r - (w_z * (l_x + l_y) / r))))
            
        else:
            wheel_1_backwards = False
            if wheel_1_w != 0:
                wheel_1_w = max(min_ang_vel,min(5.6, v_x / r + - v_y / r - (w_z * (l_x + l_y) / r)))

        if wheel_1_w <= 0.01:
            wheel_1_pwm = 0
        else:
            wheel_1_pwm = round(26009.3690 * pow(abs(wheel_1_w), 3) - 356492.2134 * pow(abs(wheel_1_w),2) + 1634606.5012 * abs(wheel_1_w) - 2487505.3783)
        
        pca.channels[0].duty_cycle = wheel_1_pwm
        

        if wheel_1_backwards == False:
            w1_lf.value = True
            w2_lf.value = False
        else:
            w1_lf.value = False
            w2_lf.value = True

        #Wheel 2
        
        wheel_2_w = min(5.6, v_x / r + v_y / r + (w_z * (l_x + l_y) / r))
        
        if wheel_2_w < 0:
            wheel_2_backwards = True
            wheel_2_w = max(min_ang_vel,min(5.6, abs(v_x / r +  v_y / r + (w_z * (l_x + l_y) / r))))
        else:
            wheel_2_backwards = False
            if wheel_2_w != 0:
                wheel_2_w = max(min_ang_vel,min(5.6, abs(v_x / r +  v_y / r + (w_z * (l_x + l_y) / r))))
       

        if wheel_2_w <= 0.01:
            wheel_2_pwm = 0
        else:
            wheel_2_pwm = round(14012.5465 * pow(abs(wheel_2_w), 3) - 201622.0264 * pow(abs(wheel_2_w),2) + 975590.4166 * abs(wheel_2_w) - 1565859.2545)


        pca.channels[2].duty_cycle = wheel_2_pwm
        
        
        if wheel_2_backwards == False:
            w1_rf.value = True
            w2_rf.value = False
        else:
            w1_rf.value = False
            w2_rf.value = True

        #Wheel 3
        
        wheel_3_w = min(5.6, v_x / r + v_y / r - (w_z * (l_x + l_y) / r))
        if wheel_3_w < 0:
            wheel_3_backwards = True
            wheel_3_w = max(min_ang_vel,min(5.6, abs(v_x / r + v_y / r - (w_z * (l_x + l_y) / r))))
        else:
            wheel_3_backwards = False
            if wheel_3_w != 0:
                wheel_3_w = max(min_ang_vel,min(5.6, abs(v_x / r + v_y / r - (w_z * (l_x + l_y) / r))))
       

        if wheel_3_w <= 0.01:
            wheel_3_pwm = 0
        else:
            wheel_3_pwm = round(25910.2423 * pow(abs(wheel_3_w), 3) - 357877.5271 * pow(abs(wheel_3_w),2) + 1654417.8054 * abs(wheel_3_w) - 2539164.7591)
        
        pca.channels[1].duty_cycle = wheel_3_pwm

        if wheel_3_backwards == False:
            w1_lr.value = True
            w2_lr.value = False
        else:
            w1_lr.value = False
            w2_lr.value = True

    
        #Wheel 4
        
        wheel_4_w = min(5.6, v_x / r - v_y / r + (w_z * (l_x +l_y) / r))
        if wheel_4_w < 0:
            wheel_4_backwards = True
            wheel_4_w = max(min_ang_vel,min(5.6, abs(v_x / r + - v_y / r + (w_z * (l_x + l_y) / r))))
        else:
            wheel_4_backwards = False
            if wheel_4_w != 0:
                wheel_4_w = max(min_ang_vel,min(5.6, abs(v_x / r + - v_y / r + (w_z * (l_x + l_y) / r))))
       
        
    
        if wheel_4_w <= 0.01:
            wheel_4_pwm = 0
        else:
            wheel_4_pwm = round(10125.8663 * pow(abs(wheel_4_w), 3) - 139716.8605 * pow(abs(wheel_4_w),2) + 650199.6983 * abs(wheel_4_w) - 1001597.5852)
        
        pca.channels[3].duty_cycle = wheel_4_pwm

        if wheel_4_backwards == False:
            w1_rr.value = True
            w2_rr.value = False
        else:
            w1_rr.value = False
            w2_rr.value = True
        
    

            

def listener():
    rospy.init_node("Motor_controller")
    rospy.Subscriber('/cmd_vel', Twist, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
