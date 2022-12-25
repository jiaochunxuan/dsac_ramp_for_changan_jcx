from __future__ import print_function
import torch
import numpy as np
import torch.multiprocessing as mp
from torch.multiprocessing import Process, Queue
import time
from Model import PolicyNet,QNet
import gym
import matplotlib.pyplot as plt
from HighwayEntranceEnv import HighwayEntranceEnv



class Simulation():
    def __init__(self, args,shared_value):
        super(Simulation, self).__init__()
        seed = args.seed
        np.random.seed(seed)
        torch.manual_seed(seed)
        self.stop_sign = shared_value[1]
        self.args = args
        self.env = HighwayEntranceEnv()
        self.device = torch.device("cpu")
        #self.load_index = self.args.max_train
        self.load_index = 1850000

        self.actor = PolicyNet(args).to(self.device)
        # self.actor = PolicyNet(args.state_dim, args.num_hidden_cell, args.action_high, args.action_low,args.NN_type).to(self.device)
        self.actor.load_state_dict(torch.load('./'+self.args.env_name+'/method_' + str(self.args.method) + '/model/policy1_' + str(self.load_index) + '.pkl',map_location='cpu'))

        self.Q_net1 = QNet(args).to(self.device)
        # self.Q_net1 = QNet(args.state_dim, args.action_dim, args.num_hidden_cell, args.NN_type).to(self.device)
        self.Q_net1.load_state_dict(torch.load('./'+self.args.env_name+'/method_' + str(self.args.method) + '/model/Q1_' + str(self.load_index) + '.pkl',map_location='cpu'))

        if self.args.double_Q:
            elf.Q_net2 = QNet(args).to(self.device)
            # self.Q_net2 = QNet(args.state_dim, args.action_dim, args.num_hidden_cell, args.NN_type).to(self.device)
            self.Q_net2.load_state_dict(torch.load('./'+self.args.env_name+'/method_' + str(self.args.method) + '/model/Q2_' + str(self.load_index) + '.pkl',map_location='cpu'))
        
        self.test_success = 0

    def run(self):
        for episode in range(0,1000,1):

            step = 0
            reward_history = []
            entropy_history = []
            done_history = []
            Q_history =[]
            x_history = []
            y_history = []
            v_history = []
            heading_history=[]
            acc_history = []
            steering_history = []

            self.state = self.env.reset()
            state_tensor = torch.FloatTensor(self.state.copy()).float().to(self.device)
            if self.args.NN_type == "CNN":
                state_tensor = state_tensor.permute(2, 0, 1)
            self.u, log_prob = self.actor.get_action(state_tensor.unsqueeze(0), False)

            while True:
                q = self.Q_net1(state_tensor.unsqueeze(0), torch.FloatTensor(self.u).to(self.device))[0]
                if self.args.double_Q:
                    q = torch.min(
                        self.Q_net1(state_tensor.unsqueeze(0), torch.FloatTensor(self.u).to(self.device))[0],
                        self.Q_net2(state_tensor.unsqueeze(0), torch.FloatTensor(self.u).to(self.device))[0])

                Q_history.append(q.detach().item())

                self.u = self.u.squeeze(0)
                self.state, self.reward, self.done, info = self.env.step(self.u)
                x_history.append(info['state_on_end_of_step'][0])
                y_history.append(info['state_on_end_of_step'][1])
                v_history.append(info['state_on_end_of_step'][2])
                heading_history.append(info['state_on_end_of_step'][3])

                acc_history.append(info['control_acc'])
                steering_history.append(info['control_steer'])

                reward_history.append(self.reward)
                done_history.append(self.done)
                entropy_history.append(log_prob)

                if step%10000 >=0 and step%10000 <=9999:
                    self.env.render(mode='human')
                state_tensor = torch.FloatTensor(self.state.copy()).float().to(self.device)
                if self.args.NN_type == "CNN":
                    state_tensor = state_tensor.permute(2, 0, 1)
                self.u, log_prob = self.actor.get_action(state_tensor.unsqueeze(0), False)

                if self.done == True or x_history[-1] > 295:
                    break
                step = step +1
            if x_history[-1] > 295:
                self.test_success = self.test_success + 1
                print('test_success', self.test_success, 'test_total', episode+1)
        
        # # 轨迹
        # plt.figure()
        # plt.plot(np.array(x_history), np.array(y_history))
        # plt.xlabel('X')
        # plt.ylabel('Y')
        # plt.title('trajectory')

        # # 速度
        # plt.figure()
        # T = np.arange(0, len(v_history),1)*0.1
        # plt.plot(T, np.array(v_history),'r', linewidth=2.0)
        # plt.xlabel('Time')
        # plt.ylabel('Speed')

        # # 朝向角
        # plt.figure()
        # T = np.arange(0, len(v_history),1)*0.1
        # plt.plot(T, np.array(heading_history),'r', linewidth=2.0)
        # plt.xlabel('Time')
        # plt.ylabel('Heading')

        # plt.figure()
        # plt.plot(T, np.array(acc_history))
        # plt.xlabel('Time')
        # plt.ylabel('Acceleration')

        # plt.figure()
        # plt.plot(T, np.array(steering_history))
        # plt.xlabel('Time')
        # plt.ylabel('Steering Angle')

        # plt.show()

def test():
    a = np.arange(0,10,1)
    print(a)


if __name__ == "__main__":
    test()



