#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3
import tf
import pyrealsense2 as rs
import numpy as np

rospy.init_node('imu_publisher')
pub = rospy.Publisher('imu_data', Imu, queue_size=10)

def initialize_camera():
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel)
    conf.enable_stream(rs.stream.gyro)
    prof = p.start(conf)
    return p
    


def gyro_data(gyro):
    return np.asarray([gyro.x, gyro.y, gyro.z])


def accel_data(accel):
    return np.asarray([accel.x, accel.y, accel.z])

p = initialize_camera()

rate = rospy.Rate(10) # 10hz
while not rospy.is_shutdown():

    imu = Imu()
    imu.header.stamp = rospy.Time.now()
    imu.header.frame_id = "base_link"
    imu.orientation_covariance = [-1.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0]
    imu.angular_velocity_covariance = [0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0]
    imu.linear_acceleration_covariance = [0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0,
                                       0.0, 0.0, 0.0]
    
    f = p.wait_for_frames()
    accel = accel_data(f[0].as_motion_frame().get_motion_data())
    gyro = gyro_data(f[1].as_motion_frame().get_motion_data())
    
    imu.linear_acceleration.x = accel[2]
    imu.linear_acceleration.y = -accel[0]
    imu.linear_acceleration.z = accel[1]
    imu.angular_velocity.x = gyro[2]
    imu.angular_velocity.y = -gyro[0]
    imu.angular_velocity.z = gyro[1]

    pub.publish(imu)
    
    
    rate.sleep()
