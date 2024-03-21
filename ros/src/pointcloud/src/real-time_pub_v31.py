#!/usr/bin/env python


import rospy
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header, String
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud2, PointField

import socket
import numpy as np
import lib_v31


rospy.init_node('PointCloud', anonymous=True)
scanPublisher = rospy.Publisher('/Point_Cloud', PointCloud2, queue_size=1)
# server_ip = '192.168.11.100'
# lidar_port = 8031
lidar_port = rospy.get_param("lidar_port")

# intensity_mode = 0
intensity_mode = rospy.get_param("intensity_mode")
if intensity_mode == 1:
    pub_int = rospy.Publisher('intensity_1', String, queue_size=10)
else:
    pass

ANGLE_TO_RADI = 3.14/180
MAX_TDC = 4087
LIGHT_SPEED = 0.299792458
TIME_TO_DEPTH = 0.3125 *LIGHT_SPEED/2 
DEPTH_BIAS = 0
TCP_BUF_SIZE = 8200

# pkg_num = 60 # 6.25
pkg_num = 55 # 7.8125
lidar_G3 = 256
def get_rawDataLen_G3(lidar_G2, pkg_num):
    pkg_frame = lidar_G2*4 # 1024
    pkg_frame *= 8
    pkg_frame *= pkg_num

    tcp_frame = lidar_G2*4 + 1 # 1025
    tcp_frame *= 8
    tcp_frame *= pkg_num

    data_points = lidar_G2*pkg_num
    data_points_ = lidar_G2*(pkg_num-1)
    data_points_1 = lidar_G2*(pkg_num-2)

    return tcp_frame, pkg_frame, data_points, data_points_, data_points_1

int_data_size, bufferSize, data_points, data_points_, data_points_1 = get_rawDataLen_G3(lidar_G3, pkg_num)

def get_indexList_G3(dataNum):
    index_list = [0]
    d_counter = 0
    data_cycle = 1025
    rangeNum = int(data_cycle/4)
    for i in range(dataNum-1):
        d_counter += 1
        if d_counter < rangeNum:
            index_list.append(index_list[i] + 32)
        elif d_counter == rangeNum:
            index_list.append(index_list[i] + 40)
            d_counter = 0
    return index_list
index_list = get_indexList_G3(data_points)
index_list = np.array(index_list)

def main_v3(dataXYZ_1, dataXYZ_2, dataXYZ_3, dataXYZ_4, dataXYZ_5, dataXYZ_6):
    dataXYZ_12 = np.concatenate((dataXYZ_1, dataXYZ_2), axis=0)
    dataXYZ_34 = np.concatenate((dataXYZ_3, dataXYZ_4), axis=0)
    dataXYZ_56 = np.concatenate((dataXYZ_5, dataXYZ_6), axis=0)
    dataXYZ_total = np.concatenate((dataXYZ_12, dataXYZ_34, dataXYZ_56), axis=0)

    return dataXYZ_total

FIELDS_XYZ = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)]

def convertCloudFromOpen3dToROS(dataXYZ_total, frame_id=None):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = frame_id

    fields=FIELDS_XYZ
    
    return pc2.create_cloud(header, fields, dataXYZ_total)


def main(lidar_port):
    cms_server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    cms_server.bind(("", lidar_port))
    str_port = '192.168.11.' + str(lidar_port)

    HORIZONTAL_FOV = 20.0
    VERTICAL_FOV = 20.0

    ### 192.168.11.31
    Decrease_tdc_value_1 = 106
    Decrease_tdc_value_2 = 112
    Decrease_tdc_value_3 = 106
    Decrease_tdc_value_4 = 119
    Decrease_tdc_value_5 = 101
    Decrease_tdc_value_6 = 110

    ### 192.168.11.30
    # Decrease_tdc_value_1 = 120
    # Decrease_tdc_value_2 = 120
    # Decrease_tdc_value_3 = 106
    # Decrease_tdc_value_4 = 106
    # Decrease_tdc_value_5 = 120
    # Decrease_tdc_value_6 = 120

    ros_rate = rospy.Rate(10.0)

    int_data = []
    zero_data = np.array([0,0,0,0])

    while not rospy.is_shutdown():
        if len(int_data) < bufferSize:
            recv_data, addr = cms_server.recvfrom(TCP_BUF_SIZE)
            
            for data in recv_data:
                int_data.append(ord(data))

        if len(int_data) > bufferSize-1:
            int_data_array = np.array(int_data)

            check_num = lib_v31.check_index_G3(int_data_array)
            if check_num == 1:
                int_data_ = lib_v31.rawData_sorting_G3(int_data_array, pkg_num)
            elif check_num == 2:
                int_data_ = lib_v31.rawData_sorting_G3_2Points(int_data_array)
            elif check_num == 3:
                int_data_ = lib_v31.rawData_sorting_G3_2Points_2rows(int_data_array)
            elif check_num == 4:
                int_data_ = lib_v31.rawData_sorting_G3_2rows(int_data_array, pkg_num)

            dataXYZ_1, dataXYZ_2, \
            dataXYZ_3, dataXYZ_4, \
            dataXYZ_5, dataXYZ_6, \
            int_1_intensity, int_2_intensity, \
            int_3_intensity, int_4_intensity, \
            int_5_intensity, int_6_intensity = lib_v31.rawData_to_xyz_G3(
                int_data_, index_list, 
                HORIZONTAL_FOV, VERTICAL_FOV, 
                Decrease_tdc_value_1, Decrease_tdc_value_2, 
                Decrease_tdc_value_3, Decrease_tdc_value_4, 
                Decrease_tdc_value_5, Decrease_tdc_value_6
            )

            dataXYZ_total = main_v3(dataXYZ_1, dataXYZ_2, dataXYZ_3, dataXYZ_4, dataXYZ_5, dataXYZ_6)

            laserScan = convertCloudFromOpen3dToROS(dataXYZ_total,  
                                                    frame_id="lidar")
            scanPublisher.publish(laserScan)

            if intensity_mode == 1:
                intensity_data = int_1_intensity + int_2_intensity + int_3_intensity + int_4_intensity + int_5_intensity + int_6_intensity
                str_intensity_data = ','.join(map(str, intensity_data))
                pub_int.publish(str_intensity_data)
            else:
                pass

            print('Point Cloud: ',  str_port, ' is publishing.', rospy.get_time())
            int_data = []

            ros_rate.sleep()
    

if __name__ == '__main__':
    try:
        main(lidar_port)
    except rospy.ROSInterruptException:
        pass
