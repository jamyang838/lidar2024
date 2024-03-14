import math
import numpy as np
import scipy.linalg as linalg
from numba import jit


######################### Data Sorting G3 #########################
@jit(nopython=True)
def rawData_sorting_G3(int_data_array, pkg_num):
    TCP_BUF_SIZE = 8200

    sort_data = []
    for i in range(0, len(int_data_array), TCP_BUF_SIZE):
        sort_data.append(int_data_array[i : i+TCP_BUF_SIZE])

    new_data = []
    zero_ = [0, 0, 0, 0, 0, 0, 0, 0]
    check_data = [250, 251, 255, 255, 0, 0, 0, 0]
    for j in range(len(sort_data)-1):
        for k in range(8, len(sort_data[j])-8):
            new_data.append(sort_data[j][k])
        for l in range(8):
            new_data.append(sort_data[j+1][l])
        new_data += check_data

    for m in range(8, TCP_BUF_SIZE-32):
        new_data.append(sort_data[pkg_num-1][m])
    for n in range(4):
        new_data += zero_
    new_data += check_data
    new_data = np.array(new_data)
    
    return new_data

@jit(nopython=True)
def rawData_sorting_G3_2Points(int_data_array):
    TCP_BUF_SIZE = 8200

    sort_data = []
    for i in range(0, len(int_data_array), TCP_BUF_SIZE):
        sort_data.append(int_data_array[i : i+TCP_BUF_SIZE])

    new_data = []
    for j in range(len(sort_data)):
        for k in range(64):
            new_data.append(0)
        for l in range(64, len(sort_data[j])):
            new_data.append(sort_data[j][l])
    new_data = np.array(new_data)
    
    return new_data

@jit(nopython=True)
def rawData_sorting_G3_2Points_2rows(int_data_array):
    TCP_BUF_SIZE = 8200

    sort_data = []
    for i in range(0, len(int_data_array), TCP_BUF_SIZE):
        sort_data.append(int_data_array[i : i+TCP_BUF_SIZE])

    new_data = []
    for j in range(len(sort_data)):
        for k in range(80):
            new_data.append(0)
        for l in range(80, len(sort_data[j])):
            new_data.append(sort_data[j][l])
    new_data = np.array(new_data)
    
    return new_data

@jit(nopython=True)
def rawData_sorting_G3_2rows(int_data_array, pkg_num):
    TCP_BUF_SIZE = 8200

    sort_data = []
    for i in range(0, len(int_data_array), TCP_BUF_SIZE):
        sort_data.append(int_data_array[i : i+TCP_BUF_SIZE])

    new_data = []
    zero_ = [0, 0, 0, 0, 0, 0, 0, 0]
    check_data = [250, 251, 255, 255, 0, 0, 0, 0]
    for j in range(len(sort_data)-1):
        for k in range(16, len(sort_data[j])-8):
            new_data.append(sort_data[j][k])
        for l in range(16):
            new_data.append(sort_data[j+1][l])
        new_data += check_data

    for m in range(16, TCP_BUF_SIZE-24):
        new_data.append(sort_data[pkg_num-1][m])
    for n in range(4):
        new_data += zero_
    new_data += check_data
    new_data = np.array(new_data)
    
    return new_data

@jit(nopython=True)
def check_index_G3(rawData):
    check_num = 0
    if rawData[8] == rawData[16] == rawData[24] == rawData[32]:
        if rawData[32] != rawData[40]:
            check_num += 1
        elif rawData[64] == rawData[72] == rawData[80] == rawData[88]:
            check_num += 2
        elif rawData[80] == rawData[88] == rawData[96] == rawData[104]:
            check_num += 3
    elif rawData[16] == rawData[24] == rawData[32] == rawData[40]:
        check_num += 4

    return check_num
######################### Data Sorting G3 #########################


############################## Data decoding G3 ##############################
@jit(nopython=True)
def gen_xyz_3d(data_xyz):
    dataLen = int(len(data_xyz) / 3)
    data_3d = np.zeros((dataLen, 3))
    for i in range(dataLen):
        data_3d[i][0] = data_xyz[i*3]
        data_3d[i][1] = data_xyz[i*3 + 1]
        data_3d[i][2] = data_xyz[i*3 + 2]
    return data_3d

########################### Rotate Matrix ###########################
### G3 default
# LiDAR_1_yaw_angle_A    =  math.radians(float(45.66))
# LiDAR_1_yaw_angle_A_up =  math.radians(float(0.0))
# LiDAR_2_yaw_angle_2    =  math.radians(float(27.385))
# LiDAR_2_yaw_angle_A_up =  math.radians(float(0.0))
# LiDAR_3_yaw_angle_B    =  math.radians(float(9.125))
# LiDAR_3_yaw_angle_B_up =  math.radians(float(0.0))
# LiDAR_4_yaw_angle_B    = -math.radians(float(9.125))
# LiDAR_4_yaw_angle_B_up =  math.radians(float(0.0))
# LiDAR_5_yaw_angle_B    = -math.radians(float(27.385))
# LiDAR_5_yaw_angle_B_up =  math.radians(float(0.0))
# LiDAR_6_yaw_angle_B    = -math.radians(float(45.66))
# LiDAR_6_yaw_angle_B_up =  math.radians(float(0.0))

# G2-31
LiDAR_1_yaw_angle_A    =  math.radians(float(44.46))
LiDAR_1_yaw_angle_A_up =  math.radians(float(0.0))
LiDAR_2_yaw_angle_2    =  math.radians(float(27.085))
LiDAR_2_yaw_angle_A_up =  math.radians(float(0.0))
LiDAR_3_yaw_angle_B    =  math.radians(float(9.125))
LiDAR_3_yaw_angle_B_up =  math.radians(float(0.0))
LiDAR_4_yaw_angle_B    = -math.radians(float(9.125))
LiDAR_4_yaw_angle_B_up =  math.radians(float(0.0))
LiDAR_5_yaw_angle_B    = -math.radians(float(26.385))
LiDAR_5_yaw_angle_B_up =  math.radians(float(0.0))
LiDAR_6_yaw_angle_B    = -math.radians(float(45.66))
LiDAR_6_yaw_angle_B_up =  math.radians(float(0.0))

def rotate_mat(axis, radian):
    rot_matrix = linalg.expm(np.cross(np.eye(3), axis / linalg.norm(axis) * radian))
    return rot_matrix
axis_x, axis_y, axis_z = [1,0,0], [0,1,0], [0, 0, 1]

rot_matrix_1 = rotate_mat(axis_y, LiDAR_1_yaw_angle_A)
rot_matrix_up_1 = rotate_mat(axis_x, LiDAR_1_yaw_angle_A_up)
########################### LiDAR 2 ###########################
rot_matrix_2 = rotate_mat(axis_y, LiDAR_2_yaw_angle_2)
rot_matrix_up_2 = rotate_mat(axis_x, LiDAR_2_yaw_angle_A_up)
########################### LiDAR 3 ###########################
rot_matrix_3 = rotate_mat(axis_y, LiDAR_3_yaw_angle_B)
rot_matrix_up_3 = rotate_mat(axis_x, LiDAR_3_yaw_angle_B_up)
########################### LiDAR 4 ###########################
rot_matrix_4 = rotate_mat(axis_y, LiDAR_4_yaw_angle_B)
rot_matrix_up_4 = rotate_mat(axis_x, LiDAR_4_yaw_angle_B_up)
########################### LiDAR 5 ###########################
rot_matrix_5 = rotate_mat(axis_y, LiDAR_5_yaw_angle_B)
rot_matrix_up_5 = rotate_mat(axis_x, LiDAR_5_yaw_angle_B_up)
########################### LiDAR 6 ###########################
rot_matrix_6 = rotate_mat(axis_y, LiDAR_6_yaw_angle_B)
rot_matrix_up_6 = rotate_mat(axis_x, LiDAR_6_yaw_angle_B_up)

@jit(nopython=True)
def lidar1_rotate(x, y, z):

    x_ = (x*rot_matrix_1[0][0] + 
          y*rot_matrix_1[0][1] + 
          z*rot_matrix_1[0][2]) * rot_matrix_up_1[0][0] + \
         (x*rot_matrix_1[1][0] + 
          y*rot_matrix_1[1][1] + 
          z*rot_matrix_1[1][2]) * rot_matrix_up_1[0][1] + \
         (x*rot_matrix_1[2][0] + 
          y*rot_matrix_1[2][1] + 
          z*rot_matrix_1[2][2]) * rot_matrix_up_1[0][2]

    y_ = (x*rot_matrix_1[0][0] + 
          y*rot_matrix_1[0][1] + 
          z*rot_matrix_1[0][2]) * rot_matrix_up_1[1][0] + \
         (x*rot_matrix_1[1][0] + 
          y*rot_matrix_1[1][1] + 
          z*rot_matrix_1[1][2]) * rot_matrix_up_1[1][1] + \
         (x*rot_matrix_1[2][0] + 
          y*rot_matrix_1[2][1] + 
          z*rot_matrix_1[2][2]) * rot_matrix_up_1[1][2]

    z_ = (x*rot_matrix_1[0][0] + 
          y*rot_matrix_1[0][1] + 
          z*rot_matrix_1[0][2]) * rot_matrix_up_1[2][0] + \
         (x*rot_matrix_1[1][0] + 
          y*rot_matrix_1[1][1] + 
          z*rot_matrix_1[1][2]) * rot_matrix_up_1[2][1] + \
         (x*rot_matrix_1[2][0] + 
          y*rot_matrix_1[2][1] + 
          z*rot_matrix_1[2][2]) * rot_matrix_up_1[2][2]
    
    return x_, y_, z_

@jit(nopython=True)
def lidar2_rotate(x, y, z):

    x_ = (x*rot_matrix_2[0][0] + 
          y*rot_matrix_2[0][1] + 
          z*rot_matrix_2[0][2]) * rot_matrix_up_2[0][0] + \
         (x*rot_matrix_2[1][0] + 
          y*rot_matrix_2[1][1] + 
          z*rot_matrix_2[1][2]) * rot_matrix_up_2[0][1] + \
         (x*rot_matrix_2[2][0] + 
          y*rot_matrix_2[2][1] + 
          z*rot_matrix_2[2][2]) * rot_matrix_up_2[0][2]

    y_ = (x*rot_matrix_2[0][0] + 
          y*rot_matrix_2[0][1] + 
          z*rot_matrix_2[0][2]) * rot_matrix_up_2[1][0] + \
         (x*rot_matrix_2[1][0] + 
          y*rot_matrix_2[1][1] + 
          z*rot_matrix_2[1][2]) * rot_matrix_up_2[1][1] + \
         (x*rot_matrix_2[2][0] + 
          y*rot_matrix_2[2][1] + 
          z*rot_matrix_2[2][2]) * rot_matrix_up_2[1][2]

    z_ = (x*rot_matrix_2[0][0] + 
          y*rot_matrix_2[0][1] + 
          z*rot_matrix_2[0][2]) * rot_matrix_up_2[2][0] + \
         (x*rot_matrix_2[1][0] + 
          y*rot_matrix_2[1][1] + 
          z*rot_matrix_2[1][2]) * rot_matrix_up_2[2][1] + \
         (x*rot_matrix_2[2][0] + 
          y*rot_matrix_2[2][1] + 
          z*rot_matrix_2[2][2]) * rot_matrix_up_2[2][2]
    
    return x_, y_, z_

@jit(nopython=True)
def lidar3_rotate(x, y, z):

    x_ = (x*rot_matrix_3[0][0] + 
          y*rot_matrix_3[0][1] + 
          z*rot_matrix_3[0][2]) * rot_matrix_up_3[0][0] + \
         (x*rot_matrix_3[1][0] + 
          y*rot_matrix_3[1][1] + 
          z*rot_matrix_3[1][2]) * rot_matrix_up_3[0][1] + \
         (x*rot_matrix_3[2][0] + 
          y*rot_matrix_3[2][1] + 
          z*rot_matrix_3[2][2]) * rot_matrix_up_3[0][2]

    y_ = (x*rot_matrix_3[0][0] + 
          y*rot_matrix_3[0][1] + 
          z*rot_matrix_3[0][2]) * rot_matrix_up_3[1][0] + \
         (x*rot_matrix_3[1][0] + 
          y*rot_matrix_3[1][1] + 
          z*rot_matrix_3[1][2]) * rot_matrix_up_3[1][1] + \
         (x*rot_matrix_3[2][0] + 
          y*rot_matrix_3[2][1] + 
          z*rot_matrix_3[2][2]) * rot_matrix_up_3[1][2]

    z_ = (x*rot_matrix_3[0][0] + 
          y*rot_matrix_3[0][1] + 
          z*rot_matrix_3[0][2]) * rot_matrix_up_3[2][0] + \
         (x*rot_matrix_3[1][0] + 
          y*rot_matrix_3[1][1] + 
          z*rot_matrix_3[1][2]) * rot_matrix_up_3[2][1] + \
         (x*rot_matrix_3[2][0] + 
          y*rot_matrix_3[2][1] + 
          z*rot_matrix_3[2][2]) * rot_matrix_up_3[2][2]
    
    return x_, y_, z_

@jit(nopython=True)
def lidar4_rotate(x, y, z):

    x_ = (x*rot_matrix_4[0][0] + 
          y*rot_matrix_4[0][1] + 
          z*rot_matrix_4[0][2]) * rot_matrix_up_4[0][0] + \
         (x*rot_matrix_4[1][0] + 
          y*rot_matrix_4[1][1] + 
          z*rot_matrix_4[1][2]) * rot_matrix_up_4[0][1] + \
         (x*rot_matrix_4[2][0] + 
          y*rot_matrix_4[2][1] + 
          z*rot_matrix_4[2][2]) * rot_matrix_up_4[0][2]

    y_ = (x*rot_matrix_4[0][0] + 
          y*rot_matrix_4[0][1] + 
          z*rot_matrix_4[0][2]) * rot_matrix_up_4[1][0] + \
         (x*rot_matrix_4[1][0] + 
          y*rot_matrix_4[1][1] + 
          z*rot_matrix_4[1][2]) * rot_matrix_up_4[1][1] + \
         (x*rot_matrix_4[2][0] + 
          y*rot_matrix_4[2][1] + 
          z*rot_matrix_4[2][2]) * rot_matrix_up_4[1][2]

    z_ = (x*rot_matrix_4[0][0] + 
          y*rot_matrix_4[0][1] + 
          z*rot_matrix_4[0][2]) * rot_matrix_up_4[2][0] + \
         (x*rot_matrix_4[1][0] + 
          y*rot_matrix_4[1][1] + 
          z*rot_matrix_4[1][2]) * rot_matrix_up_4[2][1] + \
         (x*rot_matrix_4[2][0] + 
          y*rot_matrix_4[2][1] + 
          z*rot_matrix_4[2][2]) * rot_matrix_up_4[2][2]
    
    return x_, y_, z_

@jit(nopython=True)
def lidar5_rotate(x, y, z):

    x_ = (x*rot_matrix_5[0][0] + 
          y*rot_matrix_5[0][1] + 
          z*rot_matrix_5[0][2]) * rot_matrix_up_5[0][0] + \
         (x*rot_matrix_5[1][0] + 
          y*rot_matrix_5[1][1] + 
          z*rot_matrix_5[1][2]) * rot_matrix_up_5[0][1] + \
         (x*rot_matrix_5[2][0] + 
          y*rot_matrix_5[2][1] + 
          z*rot_matrix_5[2][2]) * rot_matrix_up_5[0][2]

    y_ = (x*rot_matrix_5[0][0] + 
          y*rot_matrix_5[0][1] + 
          z*rot_matrix_5[0][2]) * rot_matrix_up_5[1][0] + \
         (x*rot_matrix_5[1][0] + 
          y*rot_matrix_5[1][1] + 
          z*rot_matrix_5[1][2]) * rot_matrix_up_5[1][1] + \
         (x*rot_matrix_5[2][0] + 
          y*rot_matrix_5[2][1] + 
          z*rot_matrix_5[2][2]) * rot_matrix_up_5[1][2]

    z_ = (x*rot_matrix_5[0][0] + 
          y*rot_matrix_5[0][1] + 
          z*rot_matrix_5[0][2]) * rot_matrix_up_5[2][0] + \
         (x*rot_matrix_5[1][0] + 
          y*rot_matrix_5[1][1] + 
          z*rot_matrix_5[1][2]) * rot_matrix_up_5[2][1] + \
         (x*rot_matrix_5[2][0] + 
          y*rot_matrix_5[2][1] + 
          z*rot_matrix_5[2][2]) * rot_matrix_up_5[2][2]
    
    return x_, y_, z_

@jit(nopython=True)
def lidar6_rotate(x, y, z):

    x_ = (x*rot_matrix_6[0][0] + 
          y*rot_matrix_6[0][1] + 
          z*rot_matrix_6[0][2]) * rot_matrix_up_6[0][0] + \
         (x*rot_matrix_6[1][0] + 
          y*rot_matrix_6[1][1] + 
          z*rot_matrix_6[1][2]) * rot_matrix_up_6[0][1] + \
         (x*rot_matrix_6[2][0] + 
          y*rot_matrix_6[2][1] + 
          z*rot_matrix_6[2][2]) * rot_matrix_up_6[0][2]

    y_ = (x*rot_matrix_6[0][0] + 
          y*rot_matrix_6[0][1] + 
          z*rot_matrix_6[0][2]) * rot_matrix_up_6[1][0] + \
         (x*rot_matrix_6[1][0] + 
          y*rot_matrix_6[1][1] + 
          z*rot_matrix_6[1][2]) * rot_matrix_up_6[1][1] + \
         (x*rot_matrix_6[2][0] + 
          y*rot_matrix_6[2][1] + 
          z*rot_matrix_6[2][2]) * rot_matrix_up_6[1][2]

    z_ = (x*rot_matrix_6[0][0] + 
          y*rot_matrix_6[0][1] + 
          z*rot_matrix_6[0][2]) * rot_matrix_up_6[2][0] + \
         (x*rot_matrix_6[1][0] + 
          y*rot_matrix_6[1][1] + 
          z*rot_matrix_6[1][2]) * rot_matrix_up_6[2][1] + \
         (x*rot_matrix_6[2][0] + 
          y*rot_matrix_6[2][1] + 
          z*rot_matrix_6[2][2]) * rot_matrix_up_6[2][2]
    
    return x_, y_, z_
########################### Rotate Matrix ###########################

########################### raw data decoding ###########################
# HORIZONTAL_FOV = 20
# VERTICAL_FOV = 10
MAX_TDC = 4087
ANGLE_TO_RADI = 3.14/180
LIGHT_SPEED = 0.299792458
TIME_TO_DEPTH = 0.3125 *LIGHT_SPEED/2 
DEPTH_BIAS = 0

"""------------------------- Decoding Function -------------------------"""
@jit(nopython=True)
def get_radius(tdc_value, TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED):
    if tdc_value <= 3072:
        raidus = tdc_value * TIME_TO_DEPTH - DEPTH_BIAS
    else:
        raidus = (960 + (tdc_value - 3072)*0.625)*(LIGHT_SPEED/2)
    return raidus

@jit(nopython=True)
def delete_Blind_Space(radius, Decrease_tdc_value, TIME_TO_DEPTH, DEPTH_BIAS):
    radius -= Decrease_tdc_value * TIME_TO_DEPTH - DEPTH_BIAS
    if radius < 0:
        radius = 0
    else:
        pass
    return radius

@jit(nopython=True)
def rawData_to_xyz_G3(int_data, index_list, HORIZONTAL_FOV, VERTICAL_FOV, 
                      Decrease_tdc_value_1, Decrease_tdc_value_2, 
                      Decrease_tdc_value_3, Decrease_tdc_value_4, 
                      Decrease_tdc_value_5, Decrease_tdc_value_6):
    data_xyz_L1 = []
    int_1_intensity = []
    data_xyz_R1 = []
    int_2_intensity = []
    data_xyz_L2 = []
    int_3_intensity = []
    data_xyz_R2 = []
    int_4_intensity = []
    data_xyz_L3 = []
    int_5_intensity = []
    data_xyz_R3 = []
    int_6_intensity = []    
    for i in index_list:
        x_adc = int_data[i+2]
        x_adc += int_data[i+3] * 256
        if x_adc > 1023:
            x_adc = 0

        y_adc = int_data[i+4]
        y_adc += int_data[i+5] * 256

        tdc_1 = int_data[i+10]
        tdc_1 += int_data[i+11] * 256
        intensity_1 = int_data[i+14]

        tdc_2 = int_data[i+12]
        tdc_2 += int_data[i+13] * 256
        intensity_2 = int_data[i+15]

        tdc_3 = int_data[i+18]
        tdc_3 += int_data[i+19] * 256
        intensity_3 = int_data[i+22]

        tdc_4 = int_data[i+20]
        tdc_4 += int_data[i+21] * 256
        intensity_4 = int_data[i+23]

        tdc_5 = int_data[i+26]
        tdc_5 += int_data[i+27] * 256
        intensity_5 = int_data[i+30]

        tdc_6 = int_data[i+28]
        tdc_6 += int_data[i+29] * 256
        intensity_6 = int_data[i+31]

        new_x_adc_value = (-x_adc)+1023
        x_angle = (HORIZONTAL_FOV * (float(new_x_adc_value) - 512) / 1024) * ANGLE_TO_RADI
        y_angle = (VERTICAL_FOV * (float(y_adc) - 512) / 1024) * ANGLE_TO_RADI

        if float(tdc_1) > (MAX_TDC):
            tdc_1 = 0
        else:
            pass

        if float(tdc_2) > (MAX_TDC):
            tdc_2 = 0
        else:
            pass

        if float(tdc_3) > (MAX_TDC):
            tdc_3 = 0
        else:
            pass

        if float(tdc_4) > (MAX_TDC):
            tdc_4 = 0
        else:
            pass

        if float(tdc_5) > (MAX_TDC):
            tdc_5 = 0
        else:
            pass

        if float(tdc_6) > (MAX_TDC):
            tdc_6 = 0
        else:
            pass

        radius1 = get_radius(float(tdc_1), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)
        radius2 = get_radius(float(tdc_2), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)
        radius3 = get_radius(float(tdc_3), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)
        radius4 = get_radius(float(tdc_4), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)
        radius5 = get_radius(float(tdc_5), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)
        radius6 = get_radius(float(tdc_6), TIME_TO_DEPTH, DEPTH_BIAS, LIGHT_SPEED)

        radius1 = delete_Blind_Space(radius1, Decrease_tdc_value_1, TIME_TO_DEPTH, DEPTH_BIAS)
        radius2 = delete_Blind_Space(radius2, Decrease_tdc_value_2, TIME_TO_DEPTH, DEPTH_BIAS)
        radius3 = delete_Blind_Space(radius3, Decrease_tdc_value_3, TIME_TO_DEPTH, DEPTH_BIAS)
        radius4 = delete_Blind_Space(radius4, Decrease_tdc_value_4, TIME_TO_DEPTH, DEPTH_BIAS)
        radius5 = delete_Blind_Space(radius5, Decrease_tdc_value_5, TIME_TO_DEPTH, DEPTH_BIAS)
        radius6 = delete_Blind_Space(radius6, Decrease_tdc_value_6, TIME_TO_DEPTH, DEPTH_BIAS)

        data_x_1 = radius1 * math.cos(y_angle) * math.sin(x_angle)
        data_y_1 = radius1 * math.sin(y_angle)
        data_z_1 = radius1 * math.cos(y_angle) * math.cos(x_angle)

        data_x_2 = radius2 * math.cos(y_angle) * math.sin(x_angle)
        data_y_2 = radius2 * math.sin(y_angle)
        data_z_2 = radius2 * math.cos(y_angle) * math.cos(x_angle)

        data_x_3 = radius3 * math.cos(y_angle) * math.sin(x_angle)
        data_y_3 = radius3 * math.sin(y_angle)
        data_z_3 = radius3 * math.cos(y_angle) * math.cos(x_angle)

        data_x_4 = radius4 * math.cos(y_angle) * math.sin(x_angle)
        data_y_4 = radius4 * math.sin(y_angle)
        data_z_4 = radius4 * math.cos(y_angle) * math.cos(x_angle)

        data_x_5 = radius5 * math.cos(y_angle) * math.sin(x_angle)
        data_y_5 = radius5 * math.sin(y_angle)
        data_z_5 = radius5 * math.cos(y_angle) * math.cos(x_angle)

        data_x_6 = radius6 * math.cos(y_angle) * math.sin(x_angle)
        data_y_6 = radius6 * math.sin(y_angle)
        data_z_6 = radius6 * math.cos(y_angle) * math.cos(x_angle)

        data_x_1_, data_y_1_, data_z_1_ = lidar1_rotate(data_x_1, data_y_1, data_z_1)
        data_x_2_, data_y_2_, data_z_2_ = lidar2_rotate(data_x_2, data_y_2, data_z_2)
        data_x_3_, data_y_3_, data_z_3_ = lidar3_rotate(data_x_3, data_y_3, data_z_3)
        data_x_4_, data_y_4_, data_z_4_ = lidar4_rotate(data_x_4, data_y_4, data_z_4)
        data_x_5_, data_y_5_, data_z_5_ = lidar5_rotate(data_x_5, data_y_5, data_z_5)
        data_x_6_, data_y_6_, data_z_6_ = lidar6_rotate(data_x_6, data_y_6, data_z_6)

        if data_z_1_ != 0.0:
            data_xyz_L1.append(data_z_1_)
            data_xyz_L1.append(data_x_1_)
            data_xyz_L1.append(data_y_1_)
            # data_xyz_L1.append(data_z_1_)
            int_1_intensity.append(intensity_1)
        if data_z_2_ != 0.0:
            data_xyz_R1.append(data_z_2_)
            data_xyz_R1.append(data_x_2_)
            data_xyz_R1.append(data_y_2_)
            # data_xyz_R1.append(data_z_2_)
            int_2_intensity.append(intensity_2)
        if data_z_3_ != 0.0:
            data_xyz_L2.append(data_z_3_)
            data_xyz_L2.append(data_x_3_)
            data_xyz_L2.append(data_y_3_)
            # data_xyz_L2.append(data_z_3_)
            int_3_intensity.append(intensity_3)
        if data_z_4_ != 0.0:
            data_xyz_R2.append(data_z_4_)
            data_xyz_R2.append(data_x_4_)
            data_xyz_R2.append(data_y_4_)
            # data_xyz_R2.append(data_z_4_)
            int_4_intensity.append(intensity_4)
        if data_z_5_ != 0.0:
            data_xyz_L3.append(data_z_5_)
            data_xyz_L3.append(data_x_5_)
            data_xyz_L3.append(data_y_5_)
            # data_xyz_L3.append(data_z_5_)
            int_5_intensity.append(intensity_5)
        if data_z_6_ != 0.0:
            data_xyz_R3.append(data_z_6_)
            data_xyz_R3.append(data_x_6_)
            data_xyz_R3.append(data_y_6_)
            # data_xyz_R3.append(data_z_6_)
            int_6_intensity.append(intensity_6)

    data_xyz_L1 = np.array(data_xyz_L1)
    data_xyz_R1 = np.array(data_xyz_R1)
    data_xyz_L2 = np.array(data_xyz_L2)
    data_xyz_R2 = np.array(data_xyz_R2)
    data_xyz_L3 = np.array(data_xyz_L3)
    data_xyz_R3 = np.array(data_xyz_R3)

    dataXYZ_1 = gen_xyz_3d(data_xyz_L1)
    dataXYZ_2 = gen_xyz_3d(data_xyz_R1)
    dataXYZ_3 = gen_xyz_3d(data_xyz_L2)
    dataXYZ_4 = gen_xyz_3d(data_xyz_R2)
    dataXYZ_5 = gen_xyz_3d(data_xyz_L3)
    dataXYZ_6 = gen_xyz_3d(data_xyz_R3)

    int_1_intensity = np.array(int_1_intensity)
    int_2_intensity = np.array(int_2_intensity)
    int_3_intensity = np.array(int_3_intensity)
    int_4_intensity = np.array(int_4_intensity)
    int_5_intensity = np.array(int_5_intensity)
    int_6_intensity = np.array(int_6_intensity)

    return dataXYZ_1, dataXYZ_2, \
           dataXYZ_3, dataXYZ_4, \
           dataXYZ_5, dataXYZ_6, \
           int_1_intensity, int_2_intensity, \
           int_3_intensity, int_4_intensity, \
           int_5_intensity, int_6_intensity
############################## Data decoding G3 ##############################