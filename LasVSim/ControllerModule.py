# coding=utf-8
from ctypes import *
from _ctypes import FreeLibrary
import math
from LasVSim.setting import *


class CarPosition(Structure):
    """
    Car Position Structure for C/C++ interface
    """
    _fields_ = [("x", c_float),  # m
                ("y", c_float),  # m
                ("heading", c_float),  # deg, 坐标系1下
                ("velocity", c_float)]  # m/s

def get_prop_angle(a1, a2, k):
    """Get angle between a1 and a2 with proportion k.

    Unit: degree
    """
    while a1 - a2 > 180:
        a1 -= 360
    while a2 - a1 > 180:
        a2 -= 360
    a = a1 * (1 - k) + a2 * k
    a = a % 360
    if a > 180:
        a -= 360
    return a


def get_distance(p1,p2):
    """Get distance between p1 and p2.

    Unit: m
    """
    x1,y1=p1
    x2,y2=p2
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))


class CarControllerDLL(object):
    """Car Controller class for LasVSim.

    It's A interface with ctypes for the dynamic library compatible with both
    CVT and AMT controller model.
    """

    def __init__(self, path, model_type, step_length, input_type,
                 car_parameter=None, car_model=None):
        self.__path = path
        self.__step_length = float(step_length)/1000
        self.__model_type = model_type
        if input_type == DECISION_OUTPUT_TYPE[KINEMATIC_OUTPUT]:
            self.__input = 1
        elif input_type == DECISION_OUTPUT_TYPE[DYNAMIC_OUTPUT]:
            self.__input = 0
        else:
            self.__input = 2
        if car_model == 'CVT Car':
            self.__car_model = 0
        elif car_model == 'AMT Car':
            self.__car_model = 1
        else:
            self.__car_model = 2

        self.__car_para = car_parameter
        # print('self.__input', self.__input)
        self.dll = CDLL(self.__path)
        self.dll.init(c_float(self.__step_length), byref(self.__car_para),
                      c_int(self.__input), c_int(self.__car_model))
        self.amax=4.0
        self.plan_pos=None
        self.p = None  # 期望轨迹点

    def __del__(self):
        # FreeLibrary(self.dll._handle)
        del self.dll

    def sim_step(self, X, Y, YAW, A, V, R, I):
        _x = c_float(X)
        _y = c_float(Y)
        _a = c_float(A)
        _v = c_float(V)
        _yaw = c_float(YAW)
        _r = c_float(R)
        _i = c_float(I)
        _EngTorque = c_float(0)
        _BrakPressure = c_float(0)
        _SteerWheel = c_float(0)
        self.dll.sim(byref(_x), byref(_y), byref(_yaw), byref(_a), byref(_v),
                     byref(_r), byref(_i), byref(_EngTorque),
                     byref(_BrakPressure), byref(_SteerWheel))
        return _EngTorque.value, _BrakPressure.value, _SteerWheel.value


    def set_track(self, track):
        # track=self.resample_track(track)
        if len(track)>=2:
            self.plan_pos=track[1]
        else:
            self.plan_pos=None
        pos_arr = CarPosition * len(track)
        self.p = pos_arr()
        for i in range(len(track)):
            t, x, y, v, a = track[i]
            self.p[i].x = x
            self.p[i].y = y
            self.p[i].heading = a
            self.p[i].velocity = v
        self.dll.set_trajectory(len(self.p), byref(self.p))

    def resample_track(self, track):
        if len(track) < 2:
            return [track[0]]
        dt = 0.1
        t0, x0, y0, v0, a0 = track[0]
        t = t0
        resampled = []
        for i in range(1, len(track)):
            t1, x1, y1, v1, a1 = track[i]
            while t < t1 + ROUTER_MAX_ERR:
                k = (t - t0) / (t1 - t0)
                x = x0 * (1 - k) + x1 * k
                y = y0 * (1 - k) + y1 * k
                v = v0 * (1 - k) + v1 * k
                a = get_prop_angle(a0, a1, k)
                resampled.append([t, x, y, v, a])
                if len(resampled) > 100:
                    break
                t += dt
            t0, x0, y0, v0, a0 = track[i]
        if len(resampled) < 2:
            t0, x0, y0, v0, a0 = resampled[0]
            t1, x1, y1, v1, a1 = track[-1]
            v1 = v0 - self.amax * 0.1
            if v1 < 0:
                v1 = 0
            if v0 < ROUTER_MAX_ERR:
                x1, y1, a1 = x0, y0, a0
            else:
                d1 = get_distance((x0, y0), (x1, y1))
                if d1 < ROUTER_MAX_ERR:
                    x1, y1, a1 = x0, y0, a0
                else:
                    d2 = (v1 + v0) / 2 * 0.1
                    k = d2 / d1
                    x1 = x0 * (1 - k) + x1 * k
                    y1 = y0 * (1 - k) + y1 * k
                    a1 = get_prop_angle(a0, a1, k)
            resampled.append([t0 + 0.1, x1, y1, v1, a1])
        return resampled

    def get_plan_pos(self):
        return self.plan_pos

    def set_control_outputB(self,acc_desired,steering_desired):
        acc_des=c_float(acc_desired)
        str_des=c_float(steering_desired)
        self.dll.set_control_outputB(acc_des, str_des)

# test codes
if __name__ == "__main__":
    controller = CarControllerDLL()
    # track = [(0, -360.76, -616.375, 0.0, -90.0),
    #          (0.1, -360.88, -616.3875000476837, 0.25, -90.0),
    #          (0.2, -361.0, -616.4000000953674, 0.5, -90.0),
    #          (0.3, -361.1, -616.4000000953674, 0.75, -90.0),
    #          (0.4, -361.2, -616.4000000953674, 1.0, -90.0),
    #          (0.5, -361.40000915527344, -616.4000000953674, 1.25, -90.0),
    #          (0.6, -361.6000061035156, -616.4000000953674, 1.5, -90.0),
    #          (0.7, -361.8000030517578, -616.4000000953674, 1.75, -90.0),
    #          (0.7999999999999999, -362.0, -616.4000000953674, 2.0, -90.0),
    #          (0.8999999999999999, -362.3000030517578, -616.4000000953674, 2.25, -90.0),
    #          (0.9999999999999999, -362.6000061035156, -616.4000000953674, 2.5, -90.0),
    #          (1.0999999999999999, -362.90000915527344, -616.4000000953674, 2.75, -90.0),
    #          (1.2, -363.20001220703125, -616.4000000953674, 3.0, -90.0),
    #          (1.3, -363.6000061035156, -616.4000000953674, 3.25, -90.0),
    #          (1.4000000000000001, -364.0, -616.4000000953674, 3.5, -90.0),
    #          (1.5000000000000002, -364.3999938964844, -616.4000000953674, 3.75, -90.0),
    #          (1.6000000000000003, -364.79998779296875, -616.4000000953674, 4.0, -90.0),
    #          (1.7000000000000004, -365.29998779296875, -616.4000000953674, 4.25, -90.0),
    #          (1.8000000000000005, -365.79998779296875, -616.4000000953674, 4.5, -90.0),
    #          (1.9000000000000006, -366.2999954223633, -616.4000000953674, 4.75, -90.0),
    #          (2.0000000000000004, -366.8000030517578, -616.4000000953674, 5.0, -90.0),
    #          (2.1000000000000005, -367.4000015258789, -616.4000000953674, 5.25, -90.0),
    #          (2.2000000000000006, -368.0, -616.4000000953674, 5.5, -90.0),
    #          (2.3000000000000007, -368.5999984741211, -616.4000000953674, 5.75, -90.0),
    #          (2.400000000000001, -369.1999969482422, -616.4000000953674, 6.0, -90.0),
    #          (2.500000000000001, -369.9000015258789, -616.4000000953674, 6.25, -90.0),
    #          (2.600000000000001, -370.6000061035156, -616.4000000953674, 6.5, -90.0),
    #          (2.700000000000001, -371.3000030517578, -616.4000000953674, 6.75, -90.0),
    #          (2.800000000000001, -372.0, -616.4000000953674, 7.0, -90.0),
    #          (2.9000000000000012, -372.8000030517578, -616.4000000953674, 7.25, -90.0),
    #          (3.0000000000000013, -373.6000061035156, -616.4000000953674, 7.5, -90.0),
    #          (3.1000000000000014, -374.4000015258789, -616.4000000953674, 7.75, -90.0),
    #          (3.2000000000000015, -375.1999969482422, -616.4000000953674, 8.0, -90.0),
    #          (3.3000000000000016, -376.0999984741211, -616.4000000953674, 8.25, -90.0),
    #          (3.4000000000000017, -377.0, -616.4000000953674, 8.5, -90.0),
    #          (3.5000000000000018, -377.9000015258789, -616.4000000953674, 8.75, -90.0),
    #          (3.600000000000002, -378.8000030517578, -616.4000000953674, 9.0, -90.0),
    #          (3.700000000000002, -379.8000030517578, -616.4000000953674, 9.25, -90.0),
    #          (3.800000000000002, -380.8000030517578, -616.4000000953674, 9.5, -90.0),
    #          (3.900000000000002, -381.8000030517578, -616.4000000953674, 9.75, -90.0),
    #          (4.000000000000002, -382.8000030517578, -616.4000000953674, 10.0, -90.0),
    #          (4.100000000000001, -383.9000015258789, -616.4000000953674, 10.25, -90.0),
    #          (4.200000000000001, -385.0, -616.4000000953674, 10.5, -90.0)]
    # controller.set_track(track)
    controller.sim_step(-360.76, -616.375, -90.0, 0, 0, 0, 1.4)
    print('done')


