import gym
from LasVSim import lasvsim
from LasVSim import traffic_module
from gym.utils import seeding
import math
import numpy as np
from LasVSim.endtoend_env_utils import shift_coordination, rotate_coordination
from collections import deque
from matplotlib import pyplot as plt
from gym import spaces
from matplotlib.pyplot import MultipleLocator
from math import pi
from collections import OrderedDict
from LasVSim.ControllerModule import *
from LasVSim.DynamicModule import *
from LasVSim.setting import *

# env_closer = closer.Closer()

class HighwayEntranceEnv(gym.Env):

    def __init__(self):
        self.goal_length = 500  # episode ends on running 500m
        self.horizon = plan_horizon
        self.history_len = history_len
        self.setting_path = setting_path
        high = np.full((1, 44), -float('inf')) # 20 = 2dim * 6veh + 8ego
        self.action_space = spaces.Box(np.array([-3, -40]), np.array([3, 40]), dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        self.encode_vec_len = 44  # 6*6+ 8 ego
        self.obs_deque = deque(maxlen=history_len)
        self.simulation = lasvsim.create_simulation(setting_path + 'simulation_setting_file.xml')
        self.detected_vehicles = None
        self.all_vehicles = None
        self.ego_dynamics = None
        self.ego_info = None
        self.ego_road_related_info = None
        self.seed()  # call this for giving self.np_random
        self.interested_rear_dist = 30 #30
        self.interested_front_dist = 60  # 60 if you change this, you should change process action too
        self.interested_vehicles_4lane_list = []
        self.ego_dynamics_list = []
        self.all_vehicles_list = []
        self.interested_4lane_vehicles = []

        self.reset()


    def seed(self, seed=None):  # call this before only before training
        self.np_random, seed = seeding.np_random(seed)
        lasvsim.seed(seed)
        return [seed]

    def reset(self, **kwargs):  # if not assign 'init_state', it will generate random init state
        #  clear deque
        self.obs_deque.clear()
        self.flag_h0 = 0

        #reset initial states
        def random_init_state(flag=True):
            if flag:
                x = self.np_random.uniform(0, 1) * 10 + 5 # [5,220]
                v = self.np_random.uniform(0, 1) * 30
                y = -150 + math.tan(10* math.pi / 180) * (x - 100) - 3.75*3.5
                heading = 10
                init_state = [x, y, v, heading]
                
                # x = self.np_random.uniform(0, 1) * 10 + 5 # [5,220]
                # v = self.np_random.uniform(0, 1) * 30
                # if x < 100:
                #     y = -150 + math.tan(10* math.pi / 180) * (x - 100) - 3.75*3.5
                #     heading = 10
                # else:
                #     lane = self.np_random.choice([0, 1])
                #     y_fn = lambda lane: \
                #     [ -3.75 * 1.5, -3.75*2.5][lane]
                #     y = y_fn(lane) -150
                #     heading = 0
                # init_state = [x, y, v, heading]
            return init_state

        if 'init_state' in kwargs:
            self.init_state = kwargs['init_state']
        else:
            self.init_state = random_init_state()
        lasvsim.reset_simulation(overwrite_settings={'init_state': self.init_state},
                                 init_traffic_path=self.setting_path)
        self.simulation = lasvsim.simulation
        self.all_vehicles = lasvsim.get_all_objects()
        self.ego_dynamics, self.ego_info = lasvsim.get_ego_info()
        self.ego_road_related_info = self.get_ego_road_related_info(self.ego_dynamics['x'], self.ego_dynamics['y'], self.ego_dynamics['heading'])
        self.obs_deque.append([self.all_vehicles, self.ego_dynamics, self.ego_road_related_info])
        self.encoded_obs = np.zeros((self.history_len, self.encode_vec_len))
        self.ego_car_dynamic = VehicleDynamicModel(self.init_state[0], self.init_state[1], self.init_state[3], self.init_state[2],
                                                   step_length, carparameter, model_type)  # x, y, a, step_length(ms), car_parameter=None, model_type=None)

        self.ego_car_control = CarControllerDLL(CONTROLLER_MODEL_PATH, model_type, step_length,  'acceleration(m/s^2), front wheel angle(deg)',
                                           carparameter, model_type)

        return self.observation(self.obs_deque)

    def step(self, action):  # action is a np.array, [behavior, goal_delta_x, acc]

        self.all_vehicles_list = []
        self.ego_dynamics_list = []
        state_on_begin_of_step = [self.ego_dynamics['x'], self.ego_dynamics['y'], self.ego_dynamics['v'], self.ego_dynamics['heading']]
        control_acc, control_steer = action
        wheelsteering = control_steer
        # print('control_acc, wheelsteering',control_acc, wheelsteering)
        self.ego_car_control.set_control_outputB(control_acc, wheelsteering)

        [torque, breaking, wheelsteering] = self.ego_car_control.sim_step(self.ego_car_dynamic.ego_x, self.ego_car_dynamic.ego_y,
                                                                self.ego_car_dynamic.heading, self.ego_car_dynamic.acc,
                                                                self.ego_car_dynamic.ego_vel, self.ego_car_dynamic.engine_speed,
                                                                self.ego_car_dynamic.drive_ratio)
        # print('torque, breaking, wheelsteering', torque, breaking, wheelsteering)
        self.ego_car_dynamic.set_control_input(torque, breaking, wheelsteering)  # eng_torque, brake_pressure, steer_wheel
        self.ego_car_dynamic.sim_step()
        # print('--------------------')
        lasvsim.set_ego(self.ego_car_dynamic.ego_x, self.ego_car_dynamic.ego_y, self.ego_car_dynamic.ego_vel, self.ego_car_dynamic.heading)
        lasvsim.sim_step()
        self.ego_dynamics, self.ego_info = lasvsim.get_ego_info() #observation
        self.ego_dynamics_list.append(self.ego_dynamics)


        self.all_vehicles = lasvsim.get_all_objects()  # coordination 2
        self.all_vehicles_list.append(self.all_vehicles)
        self.ego_road_related_info = self.get_ego_road_related_info(self.ego_dynamics['x'], self.ego_dynamics['y'], self.ego_dynamics['heading'])
        self.obs_deque.append([self.all_vehicles, self.ego_dynamics, self.ego_road_related_info])
        # print('length of obs', len(self.obs_deque))
        obs = self.observation(self.obs_deque)


        reward = 0
        done = 0
        done_type = 3

        done, done_type = self.judge_done()
        # print('action', action)
        # print('dis2goal', self.ego_road_related_info['dis2goal'])
        # print('ego y', self.ego_dynamics['y']+150+7 )
        # print('ego heading', self.ego_dynamics['heading'] )
        # print('ego v', self.ego_dynamics['v'])
        # print('done_type', done_type)
        reward += self.compute_done_reward(done_type)

        lane_keep_reward = 0
        long_keep_reward = 0
        lateral_reward = 0
        dis_keep_reward = 0
        heading_reward = 0
        velocity_reward = 0
        comfort_reward = 0
        state_on_end_of_step = state_on_begin_of_step

        if not done:
            state_on_end_of_step = [self.ego_dynamics['x'], self.ego_dynamics['y'], self.ego_dynamics['v'], self.ego_dynamics['heading']]
            #lane_keep_reward = -5*(self.ego_road_related_info['dist2roadcenter'])**2
            #lane_keep_reward = -8*(self.ego_road_related_info['dist2roadcenter'])**2
            lane_keep_reward = -15*(self.ego_road_related_info['dist2roadcenter'])**2
            long_keep_reward = (self.ego_dynamics['x']/320)**2
            # print('lane_keep_reward', lane_keep_reward)
            if self.ego_dynamics['x'] > 100:

                lateral_reward = -0.5 * abs(self.ego_road_related_info['dis2goal']) **2  #  delta_y to goal
            # print('lateral_reward', lateral_reward)
            dis_keep_reward = 1*min(abs(obs[0]), abs(obs[6]), abs(obs[12]), abs(obs[18]), abs(obs[24]), abs(obs[30]))-10/30
            # print('dis keep',dis_keep_reward )
            heading_reward = -(self.ego_road_related_info['dif_heading']/20)**2
            velocity_reward = 0.5 * (state_on_end_of_step[2]-20)
            comfort_reward = -2*((state_on_end_of_step[3] - state_on_begin_of_step[3] )/20)**2

        reward +=   (lane_keep_reward + lateral_reward + long_keep_reward + dis_keep_reward +\
                     heading_reward + velocity_reward +  comfort_reward)
            # print('offset', self.ego_road_related_info['dist2current_lane_center'])
            # print('lane index', self.ego_road_related_info['egolane_index'])
        info = dict(done_rew=reward,
                    lane_keep_reward=lane_keep_reward,
                    long_keep_reward=long_keep_reward,
                    lateral_rew=lateral_reward,
                    dis_keep_reward=dis_keep_reward,
                    heading_reward=heading_reward,
                    vel_rew=velocity_reward,
                    cft_rew=comfort_reward,
                    done_type=done_type,
                    control_acc = control_acc,
                    control_steer = control_steer,
                    state_on_end_of_step=state_on_end_of_step)

        return obs, reward, done, info

    def observation(self, observation):
        for infos in observation:
            all_vehicles, ego_dynamics, ego_road_related_info = infos
            ego_x, ego_y, ego_v, ego_heading, ego_length, ego_width = ego_dynamics['x'], ego_dynamics['y'], \
                                                                      ego_dynamics['v'], ego_dynamics['heading'], \
                                                                      ego_dynamics['length'], ego_dynamics['width']

            ego_heading = ego_heading - 10 if ego_x < 100 else ego_heading
            dis2goal, dist2roadcenter, dist2roadleft, dist2roadright, dif_heading = ego_road_related_info['dis2goal'],\
                                                                                    ego_road_related_info['dist2roadcenter'],\
                                                                                    ego_road_related_info['dist2roadleft'],\
                                                                                    ego_road_related_info['dist2roadright'],\
                                                                                    ego_road_related_info['dif_heading']
            # print('ego lane index', ego_road_related_info['road_ID'], ego_road_related_info['lane_index'])

            self.interested_vehicles=[]
            for veh in all_vehicles:
                if self.is_in_interested_area(ego_x, ego_y, veh['x'], veh['y']):
                    # interested_lane = self._interested_lane_index(ego_road_related_info['lane_index'], ego_y)
                    # interested_lane = [0,0,0]
                    # print('interested_lane', interested_lane)
                    if  163 < veh['x'] < 320:
                        veh['lane_index'] = 0 if veh['lane_index']==1 else veh['lane_index']
                        veh['lane_index'] = 1 if veh['lane_index']==2 else veh['lane_index']
                        veh['lane_index'] = 2 if veh['lane_index']==3 else veh['lane_index']

                    if veh['lane_index'] in self._interested_lane_index(ego_road_related_info['lane_index'], ego_y):
                        self.interested_vehicles.append(veh)
                        # print('veh lane index', veh['lane_index'], veh['road_ID'])
            # print('self.interested_vehicles',  self.interested_vehicles)

            current_timestep_info = self._divide_6parts_and_encode(ego_x, ego_y, ego_v, ego_heading, ego_length, ego_width,
                                                                   ego_road_related_info['lane_index'],
                                                                   dis2goal, dist2roadcenter, dist2roadleft, dist2roadright, dif_heading)
            # print('current', current_timestep_info)
            # print('curren_shape', current_timestep_info.shape)
            self.encoded_obs = self.encoded_obs[1:]
            self.encoded_obs = np.append(self.encoded_obs, current_timestep_info,axis=0)
            # print('encoded', self.encoded_obs)
        return self.encoded_obs.reshape(1,-1).squeeze()  # [time_step, 6*8 + 8]

    def get_ego_road_related_info(self, ego_x, ego_y, ego_heading):
        ego_road_related_info = lasvsim.get_ego_road_related_info()
        # print('dist2current_lane_center', ego_road_related_info['dist2current_lane_center'])
        # print('egolane_index', ego_road_related_info['egolane_index'], ego_road_related_info['egolane_ID'])
        goal_y = -150-3.75*2.5

        if ego_x < 100:
            roadleft = -150+math.tan(10* math.pi / 180) * (ego_x-100) - 3.75*2.8
            roadright = -150 + math.tan(10* math.pi / 180) * (ego_x -100) - 3.75*4
            # roadcenter = -150 + math.tan(10* math.pi / 180) * (ego_x - 100) - 3.5*3.4
            dif_heading = ego_heading-10

        elif ego_y < -150-3.75*3:
            roadleft= -150-3.75*3
            roadright = -150 - 3.75*4
            # roadcenter = -150 - 3.75*3.5
            dif_heading = ego_heading
        else:
            roadleft= -150- 3.75*2
            roadright = -150-3.75*3
            # roadcenter = -150-3.75*2.5
            dif_heading = ego_heading


        return dict(dis2goal = ego_y - goal_y,
                    dist2roadcenter = ego_road_related_info['dist2current_lane_center'],
                    dist2roadleft= ego_y - roadleft,
                    dist2roadright= ego_y - roadright,
                    dif_heading= dif_heading,
                    lane_index = ego_road_related_info['egolane_index'],
                    road_ID = ego_road_related_info['egolane_ID'] )

            # center_line = -150-3.5*2 - 3.5*1.5
            # egolane_index = 0
            # return dict(dist2current_lane_center= ego_y - center_line,
            #             egolane_index=egolane_index)

    def is_in_interested_area(self, ego_x, ego_y, pos_x, pos_y):
        if ego_x < 100:
            if  ego_x - self.interested_rear_dist < pos_x < ego_x + self.interested_front_dist and  pos_y < -150-3.75*2:
                return True
            else:
                return False
        elif  ego_x - self.interested_rear_dist < pos_x < ego_x + self.interested_front_dist and  ego_y-3.75*2 <pos_y< ego_y+3.75*2:
            return True
        else:
            return False

    def is_in_ramp2highway_area(self, pos_x, pos_y):
        return True if 0 < pos_x < 350 and  pos_y < -150-3.75 else False

    def render(self, mode='human', **kwargs):
        self.interested_vehicles_4lane_list = []
        # print(' length of all_vehicles_list', len(self.all_vehicles_list) )
        for index, all_vehicles in enumerate(self.all_vehicles_list):
            # print('index', index)
            ego_x, ego_y, ego_v, ego_heading, \
            ego_length, ego_width = self.ego_dynamics_list[index]['x'], self.ego_dynamics_list[index]['y'],\
                                    self.ego_dynamics_list[index]['v'], self.ego_dynamics_list[index]['heading'],\
                                    self.ego_dynamics_list[index]['length'], self.ego_dynamics_list[index]['width']
            shifted_x, shifted_y = shift_coordination(ego_x, ego_y, 0, -150)
            ego_car = {'ego_x':shifted_x,'ego_y': shifted_y, 'ego_width': ego_length, 'ego_height': ego_width,
                       'ego_angle': ego_heading}
            # print('index', index)
            # self.interested_4lane_vehicles = [veh for veh in all_vehicles
            #                                   if self.is_in_ramp2highway_area(veh['x'], veh['y'])]
            self.interested_vehicles = []
            self.other_vehicles = []
            for veh in all_vehicles:
                if self.is_in_interested_area(self.ego_dynamics_list[index]['x'], self.ego_dynamics_list[index]['y'], veh['x'], veh['y']):
                    self.interested_vehicles.append(veh)
                else:
                    self.other_vehicles.append(veh)

            interested_vehicles = self._process(self.interested_vehicles, 0)
            other_vehicles = self._process(self.other_vehicles, 0)


            self._render(ego_car, interested_vehicles, other_vehicles, 0, 350)

    def _interested_lane_index(self, ego_lane_index, ego_y):
        if ego_y <= -150 - 3.75*3:
            return [0, 0, 0] #ramp and highway_0
        else:
            info_list = [[1, 0, None], [2, 1, 0], [None, 2, 1]]  # left, middle, right
            return info_list[ego_lane_index]

    @staticmethod
    def laneindex2disttoroadedgy(lane_index, dist2current_lane_center, dif_heading):  # dist2current_lane_center (left positive)
        lane_center2road_left = [3.75*3+3.75/2, 3.75*2+3.75/2, 3.75*1+3.75/2, 3.75*0+3.75/2]
        lane_center2road_right = [3.75*0+3.75/2, 3.75*1+3.75/2, 3.75*2+3.75/2, 3.75*3+3.75/2]
        return lane_center2road_left[lane_index] - dist2current_lane_center, \
               lane_center2road_right[lane_index] + dist2current_lane_center, \
               dif_heading

    @staticmethod
    def laneindex2centery(lane_index, ego_x, ego_y):
        center_y_list = [-150-5*3.75/2, -150-3*3.75/2, -150-1*3.75/2] # highway_0 1 2
        return center_y_list[lane_index]
        # if ego_x < 100:
        #     return -150 + math.tan(10* math.pi / 180) * (ego_x - 100) - 3.5*3.4
        # elif ego_y < -150 - 3.75*3:
        #     return -150 - 3.75*5/2
        # else:
        #     center_y_list = [-150-5*3.75/2, -150-3*3.75/2, -150-1*3.75/2]
        #     return center_y_list[lane_index]

    def _divide_6parts_and_encode(self, ego_x, ego_y, ego_v, ego_heading, ego_length, ego_width, egolane_index,
                                  dis2goal, dist2roadcenter, dist2roadleft, dist2roadright, dif_heading):

        dis2goal = 20 if dis2goal > 20 else dis2goal
        EGO_ENCODED_VECTOR = [ego_v, ego_heading, ego_length, ego_width,
                              dis2goal, dist2roadcenter, dist2roadleft, dist2roadright]  # 8 dim

        # print('ego info',ego_x, ego_y, ego_v, ego_heading)
        # print('road info', dis2goal, dist2roadcenter, dist2roadleft, dist2roadright)

        # egolane_index = 0
        #保证高速公路水平车道上，laneindex不变化
        if 100< ego_x <320:
            egolane_index = 0 if egolane_index==1 else egolane_index
            egolane_index = 1 if egolane_index==2 else egolane_index
            egolane_index = 2 if egolane_index==3 else egolane_index
        # print('egolane_index', egolane_index)
        # NO_ROAD_ENCODED_VECTOR = [0, 0, ego_v, 0, ego_length, ego_width]
        NO_LEFT_ROAD_FRONT_ENCODED_VECTOR = [self.interested_front_dist, dist2roadleft, ego_v, 0, ego_length, ego_width]
        NO_LEFT_ROAD_REAR_ENCODED_VECTOR = [-self.interested_rear_dist, dist2roadleft, ego_v, 0, ego_length, ego_width]
        NO_RIGHT_ROAD_FRONT_ENCODED_VECTOR = [self.interested_front_dist, dist2roadright, ego_v, 0, ego_length, ego_width]
        NO_RIGHT_ROAD_REAR_ENCODED_VECTOR = [-self.interested_rear_dist, dist2roadright, ego_v, 0, ego_length, ego_width]
        MIDDLE_FRONT_NO_CAR_ENCODED_VECTOR = [self.interested_front_dist, -dist2roadcenter, ego_v, 0, ego_length, ego_width]
        MIDDLE_REAR_NO_CAR_ENCODED_VECTOR = [-self.interested_rear_dist, -dist2roadcenter, 0, 0, ego_length, ego_width]
        if egolane_index != 2:
            # print('leftlane', self._interested_lane_index(egolane_index, ego_y )[0])
            center_y = self.laneindex2centery(self._interested_lane_index(egolane_index, ego_y )[0], ego_x, ego_y)
            # print('disy2leftlanecenter', center_y-ego_y)
            LEFT_FRONT_NO_CAR_ENCODED_VECTOR = [self.interested_front_dist, center_y-ego_y, ego_v, 0, ego_length, ego_width]  # delta_x, delta_y, v, heading(in coord2), length, width
            LEFT_REAR_NO_CAR_ENCODED_VECTOR = [-self.interested_rear_dist, center_y-ego_y, 0, 0, ego_length, ego_width]
        if egolane_index != 0:
            # print('rightlane', self._interested_lane_index(egolane_index, ego_y )[2])
            center_y = self.laneindex2centery(self._interested_lane_index(egolane_index, ego_y )[2], ego_x, ego_y)
            # print('disy2rightlanecenter', center_y-ego_y)
            RIGHT_FRONT_NO_CAR_ENCODED_VECTOR = [self.interested_front_dist, center_y-ego_y, ego_v, 0, ego_length, ego_width]
            RIGHT_REAR_NO_CAR_ENCODED_VECTOR = [-self.interested_rear_dist, center_y-ego_y, 0, 0, ego_length, ego_width]
        if egolane_index ==0 and ego_y > -150-3.75*3:
            center_y = -150-3.75*7/2
            # print('disy2rightlanecenter', center_y-ego_y)
            RIGHT_FRONT_NO_CAR_ENCODED_VECTOR = [self.interested_front_dist, center_y-ego_y, ego_v, 0, ego_length, ego_width]
            RIGHT_REAR_NO_CAR_ENCODED_VECTOR = [-self.interested_rear_dist, center_y-ego_y, 0, 0, ego_length, ego_width]

        left_front = []
        left_rear = []
        middle_front = []
        middle_rear = []
        right_front = []
        right_rear = []

        # divide 6 parts
        # print(' num interested_vehicles',  len(self.interested_vehicles))
        if egolane_index == 2:
            for veh in self.interested_vehicles:
                delta_x = veh['x'] - ego_x
                delta_y = veh['y'] - ego_y
                v = veh['v']
                heading = veh['angle']
                length = veh['length']
                width = veh['width']
                if veh['lane_index'] == 2 and veh['x'] > ego_x:
                    middle_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 2 and veh['x'] < ego_x:
                    middle_rear.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 1 and veh['x'] > ego_x:
                    right_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 1 and veh['x'] < ego_x:
                    right_rear.append([delta_x, delta_y, v, heading, length, width])
                else:
                    assert 0, 'interested vehicles error'
        elif egolane_index == 1:
            for veh in self.interested_vehicles:
                delta_x = veh['x'] - ego_x
                delta_y = veh['y'] - ego_y
                v = veh['v']
                heading = veh['angle']
                length = veh['length']
                width = veh['width']
                if veh['lane_index'] == 1 and veh['x'] > ego_x:
                    middle_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 1 and veh['x'] < ego_x:
                    middle_rear.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 2 and veh['x'] > ego_x:
                    left_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 2 and veh['x'] < ego_x:
                    left_rear.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 0 and veh['x'] > ego_x and veh['y'] > -150 - 3.75*3 :
                    right_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 0 and veh['x'] < ego_x and veh['y'] > -150 - 3.75*3 :
                    right_rear.append([delta_x, delta_y, v, heading, length, width])
                # else:
                #     assert 0, 'interested vehicles error'
        # ego car in 0 lane in highway
        elif egolane_index == 0 and ego_y > -150 - 3.75*3:
            for veh in self.interested_vehicles:
                delta_x = veh['x'] - ego_x
                delta_y = veh['y'] - ego_y
                v = veh['v']
                heading = veh['angle']
                length = veh['length']
                width = veh['width']
                if veh['lane_index'] == 1 and veh['x'] > ego_x:
                    left_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 1 and veh['x'] < ego_x:
                    left_rear.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 0 and veh['x'] > ego_x:
                    if veh['y'] > -150 - 3.75*3 :
                        middle_front.append([delta_x, delta_y, v, heading, length, width])
                    else:
                        right_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 0 and veh['x'] < ego_x:
                    if veh['y'] > -150 - 3.75*3 :
                        middle_rear.append([delta_x, delta_y, v, heading, length, width])
                    else:
                        right_rear.append([delta_x, delta_y, v, heading, length, width])
                # else:   # TODO
                #     assert 0, 'interested vehicles error'
        #ego car in ramp
        else:
             for veh in self.interested_vehicles:
                delta_x = veh['x'] - ego_x
                delta_y = veh['y'] - ego_y
                v = veh['v']
                heading = veh['angle']
                length = veh['length']
                width = veh['width']
                if veh['lane_index'] == 0 and veh['x'] > ego_x:
                    if veh['y'] > -150 - 3.75*3 :
                        left_front.append([delta_x, delta_y, v, heading, length, width])
                    else:
                        middle_front.append([delta_x, delta_y, v, heading, length, width])
                elif veh['lane_index'] == 0 and veh['x'] < ego_x:
                    if veh['y'] < -150 - 3.75*3 :
                        left_rear.append([delta_x, delta_y, v, heading, length, width])
                    else:
                        middle_rear.append([delta_x, delta_y, v, heading, length, width])
                else:
                    print('interested vehicles error', veh['lane_index'] )
                    print('ego lane index',  ego_x, ego_y, egolane_index)
                    assert 0, 'interested vehicles error'

        # sort 6 parts
        if left_front:
            left_front.sort(key=lambda y: y[0])
        if left_rear:
            left_rear.sort(key=lambda y: y[0], reverse=True)
        if middle_front:
            middle_front.sort(key=lambda y: y[0])
        if middle_rear:
            middle_rear.sort(key=lambda y: y[0], reverse=True)
        if right_front:
            right_front.sort(key=lambda y: y[0])
        if right_rear:
            right_rear.sort(key=lambda y: y[0], reverse=True)

        # encode
        if egolane_index == 2:
            # encode left front
            encode_left_front = NO_LEFT_ROAD_FRONT_ENCODED_VECTOR

            # encode left rear
            encode_left_rear = NO_LEFT_ROAD_REAR_ENCODED_VECTOR

            # encode middle front
            if not middle_front:
                encode_middle_front = MIDDLE_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_front = middle_front[0]

            # encode middle rear
            if not middle_rear:
                encode_middle_rear = MIDDLE_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_rear = middle_rear[0]

            # encode right front
            if not right_front:
                encode_right_front = RIGHT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_front = right_front[0]

            # encode right rear
            if not right_rear:
                encode_right_rear = RIGHT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_rear = right_rear[0]
        elif egolane_index == 1:
            # encode left front
            if not left_front:
                encode_left_front = LEFT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_front = left_front[0]

            # encode left rear
            if not left_rear:
                encode_left_rear = LEFT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_rear = left_rear[0]

            # encode middle front
            if not middle_front:
                encode_middle_front = MIDDLE_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_front = middle_front[0]

            # encode middle rear
            if not middle_rear:
                encode_middle_rear = MIDDLE_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_rear = middle_rear[0]

            if not right_front:
                encode_right_front = RIGHT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_front = right_front[0]

            # encode right rear
            if not right_rear:
                encode_right_rear = RIGHT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_rear = right_rear[0]
        elif egolane_index == 0 and ego_y > -150 - 3.75*3:
            # encode left front
            if not left_front:
                encode_left_front = LEFT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_front = left_front[0]

            # encode left rear
            if not left_rear:
                encode_left_rear = LEFT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_rear = left_rear[0]

            # encode middle front
            if not middle_front:
                encode_middle_front = MIDDLE_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_front = middle_front[0]

            # encode middle rear
            if not middle_rear:
                encode_middle_rear = MIDDLE_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_rear = middle_rear[0]

            # encode right front
            if not right_front:
                encode_right_front = RIGHT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_front = right_front[0]

            # encode right rear
            if not right_rear:
                encode_right_rear = RIGHT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_right_rear = right_rear[0]
        else: # ego in ramp
            # encode left front
            if not left_front:
                encode_left_front = LEFT_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_front = left_front[0]

            # encode left rear
            if not left_rear:
                encode_left_rear = LEFT_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_left_rear = left_rear[0]

            # encode middle front
            if not middle_front:
                encode_middle_front = MIDDLE_FRONT_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_front = middle_front[0]

            # encode middle rear
            if not middle_rear:
                encode_middle_rear = MIDDLE_REAR_NO_CAR_ENCODED_VECTOR
            else:
                encode_middle_rear = middle_rear[0]

            # encode right
            encode_right_front = NO_RIGHT_ROAD_FRONT_ENCODED_VECTOR
            encode_right_rear = NO_RIGHT_ROAD_REAR_ENCODED_VECTOR

        kveh = [1/30,1,1/15,1/180*math.pi,1/4,1/2]
        kego = [1/15,1/180*math.pi,1/4,1/2,1/5,1,1,1]
        combined = np.array(encode_left_front + encode_left_rear +encode_middle_front +\
                  encode_middle_front + encode_right_front + encode_right_rear + EGO_ENCODED_VECTOR)
        combined = combined.reshape((1, self.encode_vec_len))
        # print('combined', combined)

        k = np.array( kveh + kveh + kveh + kveh + kveh + kveh + kego )
        # print('state ', len(encode_left_front))
        # print('states num', combined.size)

        # k = np.array([1/30,1,1/15,1/180*math.pi,1/4,1/2,  1/30,1,1/15,1/180*math.pi,1/4,1/2,  1/15,1/180*math.pi,1/4,1/2,1/5,1,1,1])

        # combined = encode_left_front + encode_left_rear + encode_middle_front +\
        #            encode_middle_rear + encode_right_front + encode_right_rear + EGO_ENCODED_VECTOR
        return combined*k

    @property
    def unwrapped(self):
        """Completely unwrap this env.

        Returns:
            gym.Env: The base non-wrapped gym.Env instance
        """
        return self

    def __str__(self):
        if self.spec is None:
            return '<{} instance>'.format(type(self).__name__)
        else:
            return '<{}<{}>>'.format(type(self).__name__, self.spec.id)

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
        # propagate exception
        return False

    def _process(self, interested_vehicles, ego_x):
        vehicles = []
        for veh in interested_vehicles:
            veh_x, veh_y, veh_heading, veh_length, veh_width = veh['x'], veh['y'], veh['angle'], veh['length'], veh[
                'width']
            shifted_x, shifted_y = shift_coordination(veh_x, veh_y, ego_x, -150)
            vehicles.append(dict(car_x=shifted_x,
                                 car_y=shifted_y,
                                 car_width=veh_length,
                                 car_height=veh_width,
                                 car_angle=veh_heading))

        return vehicles

    def _render(self, ego_car, vehicles, other_vehicles, left_boder, right_boder):

        plt.cla()
        plt.title("Render")
        ax = plt.gca()  # ax为两条坐标轴的实例
        plt.xlim((left_boder, right_boder))
        plt.ylim((-60, 60))
        plt.title("test render")
        ax.spines['right'].set_color('none')
        ax.spines['top'].set_color('none')
        ax.xaxis.set_ticks_position('bottom')
        ax.yaxis.set_ticks_position('left')
        ax.spines['bottom'].set_position(('data', 0))
        ax.spines['left'].set_position(('data', 0))
        # 画车道线
        x = np.arange(left_boder, right_boder)
        x1 = np.arange(left_boder, left_boder+96)
        x11 = np.arange(left_boder, left_boder+100)
        x2 = np.arange(left_boder+100, left_boder+230)
        x3 = np.arange(left_boder+230, left_boder+320)
        x4 = np.arange(left_boder+320, right_boder)

        y0 = 0 *x
        y1 = 0 * x -3.75
        y2= 0 * x - 3.75*2
        y31 = 0 * x1 - 3.75*3
        y32 = 0 * x2 - 3.75*3
        y33 = 0 * x3 - 3.75*3
        y34 = 0 * x4 - 3.75*3
        y4 = math.tan(10* math.pi / 180) * (x1-(left_boder+100)) - 3.75*2.8
        y5 = math.tan(10* math.pi / 180)  * (x11-(left_boder+100)) - 3.75*4
        y6 =  0 * x2 - 3.75*4
        y7 = math.tan(2* math.pi / 180)  * (x3-(left_boder+230)) - 3.75*4
        plt.xlabel("x ")
        plt.ylabel("y ")
        plt.plot(x, y0, color='b', linewidth=1, linestyle='-')
        plt.plot(x, y1, color='b', linewidth=1, linestyle='--')
        plt.plot(x, y2, color='b', linewidth=1, linestyle='--')
        plt.plot(x1, y31, color='b', linewidth=1, linestyle='-')
        plt.plot(x2, y32, color='b', linewidth=1, linestyle='--')
        plt.plot(x3, y33, color='b', linewidth=1, linestyle='--')
        plt.plot(x4, y34, color='b', linewidth=1, linestyle='-')
        plt.plot(x1, y4, color='b', linewidth=1, linestyle='-')
        plt.plot(x11, y5, color='b', linewidth=1, linestyle='-')
        plt.plot(x2, y6, color='b', linewidth=1, linestyle='-')
        plt.plot(x3, y7, color='b', linewidth=1, linestyle='-')

        ego_x = ego_car['ego_x']
        ego_y = ego_car['ego_y']
        ego_width = ego_car['ego_width']
        ego_height = ego_car['ego_height']
        ego_angle = ego_car['ego_angle']

        egoxl = ego_x - 0.5*ego_width*math.cos(ego_angle/180*math.pi) + 0.5*ego_height*math.sin(ego_angle/180*math.pi)
        egoyl = ego_y - 0.5*ego_width*math.sin(ego_angle/180*math.pi) - 0.5*ego_height*math.cos(ego_angle/180*math.pi)
        rect = plt.Rectangle((egoxl, egoyl), ego_width, ego_height, ego_angle,
                             linewidth=1, edgecolor='r', facecolor='none')
        ax.add_patch(rect)
        # print('ego render', ego_x, ego_y)


        # 感兴趣的他车
        for veh in vehicles:
            car_x = veh['car_x']
            car_y = veh['car_y']
            car_width = veh['car_width']
            car_height = veh['car_height']
            car_angle = veh['car_angle']
            rect = plt.Rectangle((car_x - 1 / 2 * car_width, car_y - 1 / 2 * car_height), car_width, car_height,
                                 car_angle,
                                 linewidth=1, edgecolor='b', facecolor='none')
            ax.add_patch(rect)

         # 不感兴趣的他车
        for veh in other_vehicles:
            car_x = veh['car_x']
            car_y = veh['car_y']
            car_width = veh['car_width']
            car_height = veh['car_height']
            car_angle = veh['car_angle']
            rect = plt.Rectangle((car_x - 1 / 2 * car_width, car_y - 1 / 2 * car_height), car_width, car_height,
                                 car_angle,
                                 linewidth=1, edgecolor='lightsteelblue', facecolor='none')
            ax.add_patch(rect)
        plt.axis('off')
        # plt.axis('equal')
        plt.pause(0.1)
        # plt.show()

    def close(self):
        """Override close in your subclass to perform any necessary cleanup.

        Environments will automatically close() themselves when
        garbage collected or when the program exits.
        """
        pass

    def compute_done_reward(self, done):
        if done == 3:  # not end, just step reward
            return 1
        elif done == 2:  # good end reward
            return 200
        else:            # bad end reward
            return -1000

    def judge_done(self):
        '''
        :done type:
         flag, 0: bad done: violate road constrain
         flag, 1: bad done: collision
         flag, 2: good done: complete
         flag, 3: not done
        '''
        if self._is_road_violation(): # go outside rode
            return 1, 0
        elif self.simulation.stopped:  # collision
            return 1, 1
        elif self.ego_dynamics['v']==0:  # collision
            return 1, 1
        # TODO
        elif self.ego_dynamics['x'] > 300 and -3.75*3+1 < self.ego_dynamics['y']+150 < 0 and abs(self.ego_dynamics['heading'])< 3:  # complete whole journey
            return 1, 2
        else:
             return 0, 3  # not done

    def _is_road_violation(self):
        corner_points = self.ego_info
        for corner_point in corner_points:
            corner_points = self.ego_info
            if self.ego_dynamics['y'] + 150 > -3.75*2.8 and abs(self.ego_dynamics['heading'])<3:
                self.flag_h0 = 1
            # print(' self.ego_y',self.ego_dynamics['y'])
            # print('self.flag_h0', self.flag_h0 )
            # print(corner_point[0], corner_point[1])
            if not self.judge_feasible(corner_point[0], corner_point[1]):
                return True
        return False


    def judge_feasible(self, orig_x, orig_y):  # map dependant TODO)
        # return True
        if 0 < orig_x < 100:
            if math.tan(10* math.pi / 180) * (orig_x-100) - 3.75*4 < orig_y+150 < math.tan(10* math.pi / 180) * (orig_x-100) - 3.75*2.8:
                # print('ramp')
                return True
            else:
                # print('ramp false')
                return False
        elif 100 < orig_x < 230:
            if self.flag_h0 == 1:
                if -3.75*3 < orig_y + 150 <-3.75*2: #训练时是2，为了实际考虑，测试时设计为1.8
                    # print('highway 0 ')
                    return True
                else:
                    # print('highway 0 false')
                    return False
            elif -3.75*4 < orig_y + 150 <-3.75*2: #训练时是2，为了实际考虑，测试时设计为1.8
                # print('acc zone')
                return True
            else:
                # print('acc zone false')
                return False

        elif 230 < orig_x < 320:
            # TODO
            if self.flag_h0 == 1:
                if -3.75*3 < orig_y + 150 <-3.75*2:
                    # print('highway 01 ')
                    return True
                else:
                    # print('highway 01 false')
                    return False
            elif orig_y +150 > math.tan(2* math.pi / 180)  * (orig_x -230) - 3.75*4 and orig_y +150 < -3.75*2:
                # print('merge ')
                return True
            else:
                # print('merge false')
                return False

        elif orig_x > 320:
            if -3.75*3 < orig_y + 150 < -3.75*2:
                return True
            else:
                return False







