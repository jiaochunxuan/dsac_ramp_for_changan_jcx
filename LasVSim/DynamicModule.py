# coding=utf-8
from math import pi
from _ctypes import FreeLibrary
from LasVSim.setting import *
from LasVSim.data_structures import *
import math

class VehicleDynamicModel(object):  # 可以直接与package版本替换
    """
    Vehicle dynamic model for updating vehicle's state
    a interface with ctypes for the dynamic library
    compatible with both CVT and AMT controller model

    Attributes:
        ego_x: 自车x坐标，m
        ego_y: 自车y坐标，m
        heading: 自车偏航角，deg（平台坐标系下）
        ego_vel: 自车速度标量,m/s
        acc: 自车加速度标量，m/s^2
        engine_speed: 发动机转速，rpm
        drive_ratio: 挡位
        engine_torque: 发动机输出扭矩，Nm
        brake_pressure: 制动压力，Mpa
        steer_wheel: 方向盘转角，deg（顺时针为正）
        car_info: 自动动力学模型参数结构体
        car_length: 自车长度，m
        car_width: 自车宽度，m
        ...
    """
    def __init__(self, x, y, a, v, step_length, car_parameter=None,
                 model_type=None):
        if model_type is None or model_type == 'CVT Car':
            self.path = CVT_MODEL_FILE_PATH
            self.type = 'CVT Car'
        elif model_type == 'AMT Car':
            self.path = AMT_MODEL_FILE_PATH
            self.type = model_type
        elif model_type == 'Truck':
            self.path = TRUCK_MODEL_FILE_PATH
            self.type = model_type
        self.step_length = float(step_length)/1000
        self.car_length = 0
        self.car_width = 0

        self.car_para = car_parameter
        self.ego_x = x  # m
        self.ego_y = y  # m
        self.heading = a  # deg, 坐标系2
        self.ego_vel = v  # m/s
        self.acc = 0  # m/s^2
        self.engine_speed = self.car_para.AV_ENGINE_IDLE / 30 * pi  # rpm
        self.drive_ratio = 1.0
        self.engine_torque = 0.0  # N.m
        self.brake_pressure = 0.0  # Mpa
        self.steer_wheel = 0.0  # deg
        self.car_info = VehicleInfo()  # 自车信息结构体

        self.dll = CDLL(self.path)

        self.dll.init(c_float(self.ego_x), c_float(self.ego_y),
                      c_float(self.heading), c_float(self.ego_vel),
                      c_float(self.step_length), byref(self.car_para))
        self.pos_time = 0

    def __del__(self):
        FreeLibrary(self.dll._handle)
        del self.dll

    def sim_step(self, EngTorque=None, BrakPressure=None, SteerWheel=None):
        if EngTorque is None:
            ET = c_float(self.engine_torque)
        else:
            ET = c_float(EngTorque)
            self.engine_torque = EngTorque
        if BrakPressure is None:
            BP = c_float(self.brake_pressure)
        else:
            BP = c_float(BrakPressure)
            self.brake_pressure = BrakPressure
        if SteerWheel is None:
            SW = c_float(self.steer_wheel)
        else:
            SW = c_float(SteerWheel)
            self.steer_wheel = SteerWheel
        x = c_float()
        y = c_float()
        yaw = c_float()
        acc = c_float()
        v = c_float()
        r = c_float()
        i = c_float()
        road_info = RoadParameter()
        road_info.slope = 0.0
        self.dll.sim(byref(road_info), byref(ET), byref(BP), byref(SW),
                     byref(x), byref(y), byref(yaw), byref(acc), byref(v),
                     byref(r), byref(i))
        (self.ego_x, self.ego_y, self.ego_vel, self.heading, self.acc, self.engine_speed,
         self.drive_ratio) = (x.value, y.value, v.value, yaw.value, acc.value,
                              r.value, i.value)

    def get_pos(self):
        return (self.ego_x, self.ego_y, self.ego_vel, self.heading, self.acc,
                self.engine_speed, self.drive_ratio)

    # def set_model(self,a,b,m):
    #     self.dll.set_para(c_float(a),c_float(b),c_float(m))

    def set_control_input(self, eng_torque, brake_pressure, steer_wheel):
        self.engine_torque = eng_torque
        self.brake_pressure = brake_pressure
        self.steer_wheel = steer_wheel

    def get_info(self):
        self.dll.get_info(byref(self.car_info))
        return (self.car_info.Steer_SW,
                self.car_info.Throttle,
                self.car_info.Bk_Pressure,
                self.car_info.Rgear_Tr,
                self.car_info.AV_Eng,
                self.car_info.M_EngOut,
                self.car_info.A,
                self.car_info.Beta / pi * 180,
                self.car_info.AV_Y / pi * 180,
                self.car_info.Vy,
                self.car_info.Vx,
                self.car_info.Steer_L1,
                self.car_info.StrAV_SW,
                self.car_info.Qfuel,
                self.car_info.Ax,
                self.car_info.Ay,
                self.car_info.Mfuel)


if __name__ == "__main__":
    print(model_type)
    ego_car_dynamic = VehicleDynamicModel(0, 0, 0, 0, 50, carparameter,
                                          model_type)  # x, y, a, step_length(ms), car_parameter=None, model_type=None)

