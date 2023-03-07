# coding=utf-8
from __future__ import print_function
from ctypes import *
import os
# from _ctypes import FreeLibrary

DEFAULT_SETTING_FILE = 'Library/simulation_setting_file.xml'
"""Simulation Setting Value"""

current_path = os.path.dirname(__file__)

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

class VehicleInfo(Structure):
    """
    Car Position Structure for C/C++ interface
    """
    _fields_ = [
        ("AV_Eng", c_float),
        ("AV_Y", c_float),
        ("Ax", c_float),
        ("Ay", c_float),
        ("A", c_float),
        ("Beta", c_float),
        ("Bk_Pressure", c_float),
        ("Mfuel", c_float),
        ("M_EngOut", c_float),
        ("Rgear_Tr", c_float),
        ("Steer_SW", c_float),
        ("StrAV_SW", c_float),
        ("Steer_L1", c_float),
        ("Throttle", c_float),
        ("Vx", c_float),
        ("Vy", c_float),
        ("Yaw", c_float),
        ("Qfuel", c_float)]

class RoadParameter(Structure):
    """
    Car Position Structure for C/C++ interface
    """
    _fields_ = [("slope", c_float)]
