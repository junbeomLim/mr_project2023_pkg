import rclpy
from rclpy.node import Node

from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol

#--------------------------------
import gymnasium as gym
import math
import random
from gym.envs.classic_control import utils
import matplotlib
import matplotlib.pyplot as plt
from collections import namedtuple, deque
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F

import gym
from gym import spaces
import numpy as np

import os

"""--------------------------설명---------------------------------------
state: 물병을 놓는 순간의 각 관절의 각도 및 각속도
action: 물병을 놓는 순간의 관절 속도의 변화(증가, 유지, 감소), 관절 하나 당 3개의 action이 존재하므로 action space 는 3*3 = 9의 크기를 지닌다.
로봇팔의 각 관절은 임의의 위치에서 무조건 물병을 던진다.물병이 착지할 때 물병의 각도와 각속도를 통해 reward를 계산한다.
물병을 던진 후, reward를 통해 다음 epsiode에 던질 때에는 각 관절의 위치(각도)를 더 높일 것인지, 줄일 것인지, 유지할 것인지 결정(Action)한다. 
-----------------------------------------------------------------------"""


plt.ion()

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

Transition = namedtuple('Transition',
                        ('state', 'action', 'next_state', 'reward'))


class ReplayMemory(object):

    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)

    def push(self, *args):
        """Save a transition"""
        self.memory.append(Transition(*args))

    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)

    def __len__(self):
        return len(self.memory)

class DQN(nn.Module):
    def __init__(self, n_observations, n_actions):
        super(DQN, self).__init__()
        self.layer1 = nn.Linear(n_observations, 128)
        self.layer2 = nn.Linear(128, 128)
        self.layer3 = nn.Linear(128, n_actions)

    # Called with either one element to determine next action, or a batch
    # during optimization. Returns tensor([[left0exp,right0exp]...]).
    def forward(self, x):
        x = F.relu(self.layer1(x))
        x = F.relu(self.layer2(x))
        return self.layer3(x)

class SimpleCustomEnv(gym.Env):
    def __init__(self):
        super(SimpleCustomEnv, self).__init__()

        # 환경 리셋 및 초기 상태 설정
        self.reset()
        
        #reward 관련 상수 camera 관련 변수
        self.a_1 = 90 #deg
        self.a_2 = 250 #deg/s
        #c_1:c_2 = 10:1
        self.c_1 = 0.01
        self.c_2 = 0.001
        self.reward = 0.0
        self.deg = -1.0
        self.w = -1.0

        # 관찰 공간과 행동 공간 정의
        self.observation_space = spaces.Box(low= 0.0, high= 1.0, shape=(2,), dtype=np.float32) #관절의 각도 2개, 관절의 속도 (무시) : shape = (2, )
        self.action_space = spaces.Discrete(9)  #J_1_pos: 증가 유지 감소 * J_2_pos: 증가 유지 감소

    def get_state(self, j_1_pos, j_2_pos):
        self.j_1_pos = j_1_pos
        self.j_2_pos = j_2_pos
        return
    
    def get_reward(self, deg, w):
        self.deg = deg
        self.w = w
        self.reward = -self.c_1*(self.a_1-self.deg)**2 -self.c_2*(self.a_2-self.w)**2
        return

    def step(self, action):
        self.state = (self.j_1_pos, self.j_2_pos)

        # 보상 계산
        reward = self.reward

        # 종료 조건 검사 (성공 시)
        done = abs(self.deg-self.a_1) <= 15.0 and abs(self.w-self.a_2) <= 50.0
        if done:
            reward += 5
        
        return np.array(self.state, dtype=np.float32), reward, done, {}

    def reset(self):
        # 초기 상태 설정
        self.j_1_pos = 0.0
        self.j_2_pos = 0.0
        self.state = np.array([self.j_1_pos, self.j_2_pos]) 

        return (self.state, {})
#-----------------------------------------------------------------------------------------------------

gym.register(id='SimpleCustomEnv-v0', entry_point=SimpleCustomEnv)
env = gym.make('SimpleCustomEnv-v0')

# 학습 설정        
BATCH_SIZE = 128
GAMMA = 0.99
EPS_START = 0.9
EPS_END = 0.05
EPS_DECAY = 1000
TAU = 0.005
LR = 1e-4

# Get number of actions from gym action space
n_actions = env.action_space.n
# Get the number of state observations
state,_ = env.reset()
n_observations = len(state)

policy_net = DQN(n_observations, n_actions).to(device)
target_net = DQN(n_observations, n_actions).to(device)
target_net.load_state_dict(policy_net.state_dict())

optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
memory = ReplayMemory(10000)
done = False

steps_done = 0

#-----------------------------------------------------------------------------------

def select_action(state):
    global steps_done
    sample = random.random()
    eps_threshold = EPS_END + (EPS_START - EPS_END) * \
        math.exp(-1. * steps_done / EPS_DECAY)
    steps_done += 1
    if sample > eps_threshold:
        with torch.no_grad():
            # t.max(1) will return the largest column value of each row.
            # second column on max result is index of where max element was
            # found, so we pick action with the larger expected reward.
            #import pdb; pdb.set_trace()
            return policy_net(state).max(1)[1].view(1,1)
    else:
        return torch.tensor([[env.action_space.sample()]], device=device, dtype=torch.long)

episode_durations = []

def plot_durations(show_result=False):
    plt.figure(1)
    durations_t = torch.tensor(episode_durations, dtype=torch.float)
    if show_result:
        plt.title('Result')
    else:
        plt.clf()
        plt.title('Training...')
    plt.xlabel('Episode')
    plt.ylabel('Duration')
    plt.plot(durations_t.numpy())
    # Take 100 episode averages and plot them too
    if len(durations_t) >= 100:
        means = durations_t.unfold(0, 100, 1).mean(1).view(-1)
        means = torch.cat((torch.zeros(99), means))
        plt.plot(means.numpy())

    plt.pause(0.001)  # pause a bit so that plots are updated

def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    # Transpose the batch (see https://stackoverflow.com/a/19343/3343043 for
    # detailed explanation). This converts batch-array of Transitions
    # to Transition of batch-arrays.
    batch = Transition(*zip(*transitions))

    # Compute a mask of non-final states and concatenate the batch elements
    # (a final state would've been the one after which simulation ended)
    non_final_mask = torch.tensor(tuple(map(lambda s: s is not None,
                                        batch.next_state)), device=device, dtype=torch.bool)
    non_final_next_states = torch.cat([s for s in batch.next_state
                                                if s is not None])
    state_batch = torch.cat(batch.state)
    action_batch = torch.cat(batch.action)
    reward_batch = torch.cat(batch.reward)

    # Compute Q(s_t, a) - the model computes Q(s_t), then we select the
    # columns of actions taken. These are the actions which would've been taken
    # for each batch state according to policy_net
    state_action_values = policy_net(state_batch).gather(1, action_batch)

    # Compute V(s_{t+1}) for all next states.
    # Expected values of actions for non_final_next_states are computed based
    # on the "older" target_net; selecting their best reward with max(1)[0].
    # This is merged based on the mask, such that we'll have either the expected
    # state value or 0 in case the state was final.
    next_state_values = torch.zeros(BATCH_SIZE, device=device)
    with torch.no_grad():
        next_state_values[non_final_mask] = target_net(non_final_next_states).max(1)[0]
    # Compute the expected Q values
    expected_state_action_values = (next_state_values * GAMMA) + reward_batch

    # Compute Huber loss
    criterion = nn.SmoothL1Loss()
    loss = criterion(state_action_values, expected_state_action_values.unsqueeze(1))

    # Optimize the model
    optimizer.zero_grad()
    loss.backward()
    # In-place gradient clipping
    torch.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
    optimizer.step()            

class getdata(Node):
    def __init__(self):
        super().__init__('getdata')

        self.subscription_camera = self.create_subscription(Cameradata, 'camera', self.get_camera, 10)
        self.subscription_camera # prevent unused variable warning

        self.camera_deg = 0.0
        self.camera_w = 0.0
        self.new_data_received = False
        
    def get_camera(self, camera_msg):
        #self.get_logger().info(f"camera: {camera_msg.deg} {camera_msg.w}")
        self.camera_deg = camera_msg.deg
        self.camera_w = camera_msg.w
        self.new_data_received = True

    def return_data(self):
        return self.camera_deg, self.camera_w

class senddata(Node):
    def __init__(self):
        super().__init__('senddata')
        self.publisher_ = self.create_publisher(Robotarmcontrol, 'robotarm', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.j_1_pos = 0.0
        self.j_2_pos = 0.0
        
    def get_robotarm_data(self, j_1_pos=-1.0, j_2_pos=-1.0):
        self.j_1_pos = j_1_pos
        self.j_2_pos = j_2_pos

    def timer_callback(self):
        msg = Robotarmcontrol()
        msg.j_1_pos = self.j_1_pos
        msg.j_2_pos = self.j_2_pos
        self.publisher_.publish(msg)
        self.get_logger().info(f"\n j_1: {msg.j_1_pos} pos \n j_2: {msg.j_2_pos} pos")

#------------------------------------------------------------------------------------------------

def main(args=None):
#-------------------------------------------------------
    global env
    global BATCH_SIZE
    global GAMMA
    global EPS_START
    global EPS_END
    global EPS_DECAY
    global TAU
    global LR
    global n_actions
    global state
    global n_observations
    global policy_net
    global target_net
    global optimizer
    global memory
    global done
    global steps_done

    SAVE_INTERVAL = 2  # 2 에피소드마다 모델 저장

    j_1_max = 1023.0 #모터 1 최대값
    j_1_min = 512.0 #모터 1 최소값
    j_2_max = 650.0 #모터 2 최대값
    j_2_min = 512.0 #모터 2 최소값
    pos_width = 150.0 #모터 position 변동 범위

    #initailize 
    # 물병의 각도 및 각속도는 모두 양수 (크기만 고려한다)
    camera_deg = 0.0
    camera_w = 0.0

    #initailize 
    #관절의 각도 및 각속도는 모두 양수 (크기만 고려한다)
    j_1_pos = 0.0
    j_2_pos = 0.0

    rclpy.init(args=args)

    getdata_node = getdata()
    senddata_node = senddata()

    if torch.cuda.is_available():
        num_episodes = 600
    else:
        num_episodes = 50

    # 저장된 가중치 불러오기
    save_path_policy_net = os.path.expanduser('~/ros2_ws/src/mr_project2023_pkg/policy_net_weights.pth') #저장 위치
    save_path_target_net = os.path.expanduser('~/ros2_ws/src/mr_project2023_pkg/target_net_weights.pth') #저장 위치

    #torch.save(policy_net.state_dict(), save_path_policy_net) #저장되어 있지 않을 때만, 오류 방지
    #torch.save(policy_net.state_dict(), save_path_target_net) #저장되어 있지 않을 때만, 오류 방지
    policy_net.load_state_dict(torch.load(save_path_policy_net))
    policy_net.load_state_dict(torch.load(save_path_target_net))
    
    for i_episode in range(num_episodes):
        # Initialize the environment and get it's state
        state, _ = env.reset()
        state = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
            
        for t in count():
#---------------------------------------------------------------------------------------
            #action 선택 및 전송
            action = select_action(state)
            getdata_node.get_logger().info(f"action: {action.item()}")

            #action 선택
            if action.item() == 0:
                j_1_pos += pos_width
                j_2_pos += pos_width
            elif action.item() == 1:
                j_1_pos += pos_width
                j_2_pos -= pos_width
            elif action.item() == 2:
                j_1_pos -= pos_width
                j_2_pos += pos_width
            elif action.item() == 3:
                j_1_pos -= pos_width
                j_2_pos -= pos_width
            elif action.item() == 4:
                j_1_pos += pos_width

            elif action.item() == 5:
                j_1_pos -= pos_width
                
            elif action.item() == 6:
                j_2_pos += pos_width
            
            elif action.item() == 7:
                j_2_pos -= pos_width
            
            elif action.item() == 8:
                pass
            
            j_1_pos = min(j_1_pos, j_1_max)
            j_1_pos = max(j_1_pos, j_1_min)

            j_2_pos = min(j_2_pos, j_2_max)
            j_2_pos = max(j_2_pos, j_2_min)
            
            #모터 값 발신
            senddata_node.get_robotarm_data(j_1_pos, j_2_pos)
            rclpy.spin_once(senddata_node)

            #step 함수
            #cameradata 값 얻기
            #물병을 던진 순간의 robot data 각 관절의 각속도 및 각도 얻기
            camera_deg = -1
            getdata_node.get_logger().info("wait for camera")
            while getdata_node.new_data_received == False:
                rclpy.spin_once(getdata_node)
            
            #카메라 값 수신
            camera_deg, camera_w = getdata_node.return_data()
            getdata_node.new_data_received = False
            #getdata_node.get_logger().info(f"{camera_deg}")
            
            #리워드 및 상태 업데이트
            env.get_reward(camera_deg, camera_w)
            env.get_state(j_1_pos, j_2_pos)          
            
            observation, reward, done, _ = env.step(action.item())
            #getdata_node.get_logger().info(f"{observation} {reward} {done}")

            reward = torch.tensor([reward], device=device)

            next_state = torch.tensor(observation, dtype=torch.float32, device=device).unsqueeze(0)

            # Store the transition in memory
            memory.push(state, action, next_state, reward)

            # Move to the next state
            state = next_state

            # Perform one step of the optimization (on the policy network)
            optimize_model()

            # Soft update of the target network's weights
            # θ′ ← τ θ + (1 −τ )θ′
            target_net_state_dict = target_net.state_dict()
            policy_net_state_dict = policy_net.state_dict()

            for key in policy_net_state_dict:
                target_net_state_dict[key] = policy_net_state_dict[key]*TAU + target_net_state_dict[key]*(1-TAU)
            target_net.load_state_dict(target_net_state_dict)
            
            if i_episode % SAVE_INTERVAL == 0:
                # 학습 중간에 모델 저장
                torch.save(policy_net.state_dict(), save_path_policy_net)
                torch.save(policy_net.state_dict(), save_path_target_net) 

            if done:
                episode_durations.append(t + 1)
                plot_durations()
                break
    
    # 최종 학습이 끝난 후에도 모델을 저장
    torch.save(policy_net.state_dict(), save_path_policy_net)
    torch.save(policy_net.state_dict(), save_path_target_net)

    print('Complete')
    plot_durations(show_result=True)
    plt.ioff()
    plt.show()
    
    getdata_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()