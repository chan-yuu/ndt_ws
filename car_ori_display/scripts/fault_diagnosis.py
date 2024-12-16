#!/usr/bin/env python3
'''
Author: CYUN && cyun@tju.enu.cn
Date: 2024-10-16 18:21:37
LastEditors: CYUN && cyun@tju.enu.cn
LastEditTime: 2024-11-03 20:06:21
FilePath: /undefined/home/nvidia/clamp_forklift_ws2/src/car_ori_display/scripts/fault_diagnosis.py
Description: 

Copyright (c) 2024 by Tianjin University, All Rights Reserved. 
'''

import rospy
import random
import requests
import can
from car_interfaces.msg import FaultDiagnosisInterface,GpsImuInterface  # 替换为你的实际包名
from sensor_msgs.msg import PointCloud2, Image

lidar_state = 0
camera_state = 0
gps_state = 0

def lidar_state_callback(msg):
    global lidar_state
    lidar_state = 1
    

def camera_state_callback(msg):
    global camera_state
    camera_state = 1
    
def gps_state_callback(msg):
    global gps_state
    gps_state = 1

def check_internet_connection():
    try:
        response = requests.get("https://www.baidu.com", timeout=3)
        return response.status_code == 200
    except requests.RequestException:
        return False

def check_can_state():
    try:
        bus = can.interface.Bus(channel='can0', bustype='socketcan')
        msg = bus.recv(timeout=1.0)  # 5秒超时
        if msg: # 待修改
            return True
        return False
    except (OSError, can.CanError):
        return False
    
def check_gps_can_state():
    try:
        bus = can.interface.Bus(channel='can1', bustype='socketcan')
        msg = bus.recv(timeout=1.0)  # 5秒超时
        if msg: # 待修改
            return True
        return False
    except (OSError, can.CanError):
        return False

def fault_diagnosis_publisher():
    global lidar_state, camera_state, gps_state
    rospy.init_node('fault_diagnosis_test', anonymous=True)
    rate = rospy.Rate(100)  # 1 Hz
    pub = rospy.Publisher('fault_diagnosis_data', FaultDiagnosisInterface, queue_size=10)


    rospy.Subscriber('/rslidar_points', PointCloud2, lidar_state_callback)
    rospy.Subscriber('/camera/color/image_raw', Image, camera_state_callback)
    rospy.Subscriber('/gps_imu', GpsImuInterface, gps_state_callback)

    while not rospy.is_shutdown():
        msg = FaultDiagnosisInterface()
        msg.can_state = 1 if check_can_state() else 0  # 检查CAN状态
        msg.gps_can_state = 1 if check_gps_can_state() else 0  # 检查CAN状态
        msg.internet_state = 1 if check_internet_connection() else 0  # 检查互联网连接
        # msg.lidar_state = random.randint(0, 1)  # 随机生成0或1
        # msg.gps_system_state = random.randint(0, 1)
        # msg.camera_state = random.randint(0, 1)
        msg.lidar_state = lidar_state
        msg.gps_system_state = gps_state
        msg.camera_state = camera_state

        # msg.can_state = 1 
        # msg.gps_can_state = 1 
        # msg.internet_state = 1 
        # msg.lidar_state = 1
        # msg.gps_system_state = 1
        # msg.camera_state = 1
        # rospy.loginfo("Publishing fault diagnosis data: %s", msg)
        pub.publish(msg)
        lidar_state = 0
        gps_state = 0
        camera_state = 0
        # rate.sleep()
    # rospy.spin()

if __name__ == '__main__':
    try:
        fault_diagnosis_publisher()
    except rospy.ROSInterruptException:
        pass
