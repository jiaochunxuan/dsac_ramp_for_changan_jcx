# coding=utf-8
# 该文件下代码可直接与package相替换
from __future__ import print_function
from ctypes import *
from _ctypes import FreeLibrary
import logging
# logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger(__name__)

"""Simulation Setting Value"""
DECISION_OUTPUT_TYPE = ['spatio-temporal trajectory',
                        'acceleration(m/s^2), front wheel angle(deg)',
                        'eng_torque(N*m), steering(deg), brake(Mpa)']  # 不能更改顺序
DECISION_TYPE = ['Map1_XinLong Planner', 'External', 'External',
                 'Map2_XinLong Planner']  # 不能更改顺序
DECISION_FILE_PATH = 'Modules/DecisionModule/Map1_XinLong.dll'
DYNAMIC_OUTPUT = 2
KINEMATIC_OUTPUT = 1
SPATIO_TEMPORAL_TRAJECTORY = 0

CAR_TYPE = ['CVT Car', 'AMT Car', 'Truck'] # 不能更改顺序
CVT_CAR = 0
AMT_CAR = 1
TRUCK = 2
CVT_MODEL_FILE_PATH = "Modules/CarModel_CVT.dll"
AMT_MODEL_FILE_PATH = "Modules/CarModel_AMT.dll"
TRUCK_MODEL_FILE_PATH = "Modules/CarModel_Truck.dll"
CAR_LIB = [CVT_MODEL_FILE_PATH, AMT_MODEL_FILE_PATH, TRUCK_MODEL_FILE_PATH] # 不能更改顺序

TRAFFIC_TYPE = ['No Traffic', 'Mixed Traffic', 'Vehicle Only Traffic'] # 不能更改顺序
NO_TRAFFIC = 0
MIXED_TRAFFIC = 1
VEHICLE_ONLY_TRAFFIC = 2
TRAFFIC_DENSITY = ['Sparse', 'Middle', 'Dense'] # 不能更改顺序
SPARSE = 0
MIDDLE = 1
DENSE = 2

CONTROLLER_TYPE = ['Preview PID', 'External', 'External'] # 不能更改顺序
CONTROLLER_FILE_PATH = "Modules/Controller.dll"
PID = 0
EXTERNAL = 1

FILE_TYPE = ['C/C++ DLL', 'Python Module'] # 不能更改顺序

MAPS = ['Map1_Urban Road', 'Map2_Highway', 'Map3_Shanghai Anting',
        'Map4_Beijing Changping', 'Map5_Mcity']  # 不能更改顺序


class CarParameter(Structure):
    """
    Car Position Structure for C/C++ interface
    """
    _fields_ = [
        ("LX_AXLE", c_float),  # 轴距，m
        ("LX_CG_SU", c_float),  # 悬上质量质心至前轴距离，m
        ("M_SU", c_float),  # 悬上质量，kg
        ("IZZ_SU", c_float),  # 转动惯量，kg*m^2
        ("A", c_float),  # 迎风面积，m^2
        ("CFx", c_float),  # 空气动力学侧偏角为零度时的纵向空气阻力系数
        ("AV_ENGINE_IDLE", c_float),  # 怠速转速，rpm
        ("IENG", c_float),  # 曲轴转动惯量，kg*m^2
        ("TAU", c_float),  # 发动机-变速箱输入轴 时间常数，s
        ("R_GEAR_TR1", c_float),  # 最低档变速箱传动比
        ("R_GEAR_FD", c_float),  # 主减速器传动比
        ("BRAK_COEF", c_float),  # 液压缸变矩系数,Nm/(MPa)
        ("Steer_FACTOR", c_float),  # 转向传动比
        ("M_US", c_float),  # 簧下质量，kg
        ("RRE", c_float),  # 车轮有效滚动半径，m
        ("CF", c_float),  # 前轮侧偏刚度，N/rad
        ("CR", c_float),  # 后轮侧偏刚度，N/rad
        ("ROLL_RESISTANCE", c_float)]  # 滚动阻力系数


class RoadParameter(Structure):
    """
    Car Position Structure for C/C++ interface
    """
    _fields_ = [("slope", c_float)]


class VehicleInfo(Structure):
    """车辆动力学参数结构体"""
    _fields_ = [
        ("AV_Eng", c_float),
        ("AV_Y", c_float),
        ("Ax", c_float),
        ("Ay", c_float),
        ("A", c_float),
        ("Beta", c_float),
        ("Bk_Pressure", c_float),
        ("Mfuel", c_float),  # 累计
        ("M_EngOut", c_float),
        ("Rgear_Tr", c_float),
        ("Steer_SW", c_float),
        ("StrAV_SW", c_float),
        ("Steer_L1", c_float),
        ("Throttle", c_float),
        ("Vx", c_float),
        ("Vy", c_float),
        ("Yaw", c_float),
        ("Qfuel", c_float)]  # rate


class VehInfo(Structure):
    """周车信息结构体
    """
    _fields_ = [
        ("veh_turn_signal", c_int),  # 车辆转向信号灯（0 不亮 1 亮）
        ("veh_brake_signal", c_bool),  # 车辆制动信号灯（0 无信号灯 1 制动灯）
        ("veh_emergency_signal", c_bool),  # 车辆紧急信号灯（0 无信号灯 1 双闪）
        ("veh_type", c_int),  # 车辆类型（0 小客车 1 卡车/巴士 2 摩托车/自行车/电瓶车）
        ("veh_id", c_int),  # 车辆id
        ("veh_width", c_float),  # 车辆宽度，m
        ("veh_length", c_float),  # 车辆长度，m
        ("veh_height", c_float),  # 车辆高度，m
        ("veh_x", c_float),  # 车辆形心x坐标，m
        ("veh_y", c_float),  # 车辆形心y坐标，m
        ("veh_z", c_float),  # 车辆形心z坐标，m
        ("veh_dx", c_float),  # 车辆形心dx，m/s
        ("veh_dy", c_float),  # 车辆形心dy，m/s
        ("veh_dz", c_float),  # 车辆形心dz，m/s
        ("veh_heading", c_float),  # 车辆偏航角，deg
        ("veh_pitch", c_float),  # 车辆俯仰角，deg
        ("car_follow_model", c_int),
        ("lane_change_model", c_int),
        ("max_acc", c_float),
        ("max_dec", c_float),
        ("render_flag", c_bool),
        ("turn_state", c_int),  # 车辆转向状态 0 直行 1 左转 2 右转
        ("winker_time", c_float)]

if __name__ == "__main__":
    a = VehInfo(veh_x=2)
    print(a.veh_x)


