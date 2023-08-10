import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol

#--------------------------------
import gymnasium as gym
import math
import random
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

class learnNode(Node):
    def __init__(self):
        super().__init__('learn')

        self.publisher_ = self.create_publisher(Robotarmcontrol, 'robotarm', 10)
        self.send_msg = Robotarmcontrol()

        self.subscription_camera = self.create_subscription(Cameradata, 'camera', self.get_camera, 10)
        self.subscription_camera
        #initailize 
        # 물병의 각도 및 각속도는 모두 양수 (크기만 고려한다)
        self.camera_deg = -1
        self.camera_w = -1
        self.steps_done = 0

        self.subscription_robotarm = self.create_subscription(Robotarmcontrol, 'joint_1', self.callback, 10) 
        self.subscription_robotarm


    def get_camera(self, camera_msg):
        self.get_logger().info(f"camera: {camera_msg.deg} {camera_msg.w}")
        self.camera_deg = camera_msg.deg
        self.camera_w = camera_msg.w
    
    class SimpleCustomEnv(gym.Env):
        def __init__(self):
            super(SimpleCustomEnv, self).__init__()

            #reward 관련 상수
            self.a_1 = 90 #deg
            self.a_2 = 250 #deg/s
            #c_1:c_2 = 10:1
            self.c_1 = 10
            self.c_2 = 1
                
            self.action_space = spaces.Discrete(100)  # actions: 특정 각속도로 관절 움직임 (power 0 to 100)
            self.observation_space = spaces.Discrete(4)  # observation_space(State)는 던지는 순간 각 관절의 각도 및 각속도

            self.state = np.zeros(4)

        def reset(self):
            self.state = np.zeros(4)
            return self.state

        def step(self, action):
            assert self.action_space.contains(action), f"Invalid action: {action}"
            
            # Take action and update state
            
            ############################# 수정 필요: 상위 클래스 변수 하위 클래스에서 사용하기, 116번째 줄 수정 필요
            self.send_msg.j_1_w = action
            self.publisher_.publish(self.send_msg)

            reward = -self.c_1*(self.a_1-self.camera_deg)**2 -self.c_2*(self.a_2-self.camera_w)

            if self.camera_deg < self.a_1 and self.camera_w < self.a_2:
                done = True
                reward = reward + 10
            else:
                done = False

            self.state = np.array([msg.j_1_deg,msg.j_1_w,msg.j_2_deg,msg.j_2_w])

            #initialize
            self.camera_deg = -1
            self.camera_w = -1

            return self.state, reward, done, {}

        def render(self, mode='human'):
            pass


    def callback(self, msg):
        #---------------------------------------------------------------------
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
        state = env.reset()
        n_observations = len(state)

        policy_net = DQN(n_observations, n_actions).to(device)
        target_net = DQN(n_observations, n_actions).to(device)
        target_net.load_state_dict(policy_net.state_dict())

        optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
        memory = ReplayMemory(10000)


        def select_action(state):
            sample = random.random()
            eps_threshold = EPS_END + (EPS_START - EPS_END) * \
                math.exp(-1. * self.steps_done / EPS_DECAY)
            self.steps_done += 1
            if sample > eps_threshold:
                with torch.no_grad():
                    # t.max(1) will return the largest column value of each row.
                    # second column on max result is index of where max element was
                    # found, so we pick action with the larger expected reward.
                    return policy_net(state).max(1)[1].view(1, 1)
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

        #------------------------------------------------------------------------------------------------
        if torch.cuda.is_available():
            num_episodes = 600
        else:
            num_episodes = 50

        for i_episode in range(num_episodes):
            # Initialize the environment and get it's state
            state = env.reset()
            state = torch.tensor(state, dtype=torch.float32, device=device).unsqueeze(0)
            for t in count():
                action = select_action(state)
                observation, reward, terminated, truncated, _ = env.step(action.item())
                reward = torch.tensor([reward], device=device)
                done = terminated or truncated

                if terminated:
                    next_state = None
                else:
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

                if done:
                    episode_durations.append(t + 1)
                    plot_durations()
                    break

        print('Complete')
        plot_durations(show_result=True)
        plt.ioff()
        plt.show()

def main(args=None):
    rclpy.init(args=args)

    learn_node = learnNode()

    rclpy.spin(learn_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    learn_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__' :
    main()