from agent.Agent import Agent
from agent.ppoNetWork import PPO
from agent.params import *
from utils.RongAoUtils import RongAoUtils
from agent.obsToState import getStateRewardDone
from agent.normalization import Normalization, RewardScaling
import numpy as np
import random,math
from torch.utils.tensorboard import SummaryWriter

if flag_multiprocessing:
    ACTION = {
        "msg_info": "驾驶操控,1001,2,0,Delay,Force,0|0`0`0.8`0",
        "msg_type": "manu_ctrl",  # }
        "done": "0"}  # 是否结束战斗并附带原因[0-未结束 1-本方胜 2-敌方胜[本机摔或高度过低] 3-时间到]
else:
    ACTION = {
        "msg_info": "驾驶操控,1001,2,0,Delay,Force,0|0`0`0.8`0",
        "msg_type": "manu_ctrl"}

class PPOAGENT(Agent):
    def __init__(self, camp):
        super(PPOAGENT, self).__init__(camp)
        if flag_multiprocessing:
            self.done = 0
        else:
            self.done = False
        self.camp = camp
        self.episode = 11293
        self.nowInfo = None
        self.Epi_Rewards = 0
        self.Alt_Rewards = 0
        self.Attack_Rewards = 0
        self.Dist_Rewards = 0
        self.Alpha_Rewards = 0
        self.Beta_Rewards = 0
        self.preInfo = None
        self.nowState = None
        self.preState = None
        self.stepAction = None
        self.Action = ACTION.copy()
        self.ppoModel = PPO(StateDim, ActionDim, lr_actor, lr_critic, gamma, K_epochs, eps_clip, True,
                            action_std_init=0.6)  # 状态维度，动作维度，随机化探索概率
        if Train == 2 or Train == 1:
            self.ppoModel.load()
        self.planeID = []
        self.track = {}
        self.currentRewardBuffer = []
        if Train != 2:
            self.writer = SummaryWriter(log_dir=log_result_dir + '/SummaryWriterLogs')
        self.state_norm = Normalization(shape=StateDim)  # Trick 2:state normalization
        if use_reward_norm:  # Trick 3:reward normalization
            self.reward_norm = Normalization(shape=1)
        elif use_reward_scaling:  # Trick 4:reward scaling
            self.reward_scaling = RewardScaling(shape=1, gamma=gamma)

    def reset(self):
        if flag_multiprocessing:
            self.done = 0
        else:
            self.done = False
        self.nowInfo = None
        self.preInfo = None
        self.nowState = None
        self.preState = None
        self.Action = ACTION.copy()
        self.Epi_Rewards = 0
        self.Alt_Rewards = 0
        self.Attack_Rewards = 0
        self.Dist_Rewards = 0
        self.Alpha_Rewards = 0
        self.Beta_Rewards = 0
        self.currentRewardBuffer.clear()
        self.episode += 1
        if use_reward_scaling:
            self.reward_scaling.reset()

    def action(self, planeInfo, CurTime):
        """
        :param planeInfo: 敌我双方的飞机和导弹信息
        :return:仿真环境需要的动作信息
        """
        self.nowInfo = planeInfo
        self.CurTime = CurTime
        action = ACTION.copy()

        if self.preInfo == None:
            self.preInfo = self.nowInfo
        else:
            self.nowState, Reward, self.done, AltReward, DisReward, AttackReward, AlphaReward, BataReward = getStateRewardDone(self.nowInfo, self.preInfo,self.CurTime)
            if flag_multiprocessing:
                action["done"] = self.done  # 结束标志
            # if use_state_norm:
            #     self.nowState = self.state_norm(np.array(self.nowState))
            # if use_reward_norm:
            #     Reward = self.reward_norm(Reward)
            # elif use_reward_scaling:
            #     Reward = self.reward_scaling(Reward)
            self.Epi_Rewards += Reward
            self.Alt_Rewards += AltReward
            self.Dist_Rewards += DisReward
            self.Attack_Rewards += AttackReward
            self.Alpha_Rewards += AlphaReward
            self.Beta_Rewards += BataReward
            self.ppoModel.buffer.rewards.append(Reward)
            self.ppoModel.buffer.is_terminals.append(self.done)
            self.stepAction = self.ppoModel.select_action(self.nowState)
            self.currentRewardBuffer.append(Reward)
            Actions = []
            for a in self.stepAction:  # 输出裁剪
                if a <= -1:
                    a = -1
                elif a >= 1:
                    a = 1
                Actions.append(a)
            if Actions[2] < 0:
                Actions[2] = math.fabs(Actions[2])
            if (flag_multiprocessing and self.done == 0) or (not flag_multiprocessing and not self.done):
                action = RongAoUtils.moveRL(action, "1001", Actions[0], Actions[1], Actions[2], Actions[3])
                self.Action = action
                self.preInfo = self.nowInfo
            elif Train != 2:
                self.writer.add_scalars('Total_Rewards',
                                   {'avg_reward': np.mean(self.currentRewardBuffer),
                                    'episode_reward': self.Epi_Rewards,
                                    'Alt_Rewards': self.Alt_Rewards,
                                    'Dist_Rewards': self.Dist_Rewards,
                                    'Attack_Rewards': self.Attack_Rewards,
                                    'Alpha_Rewards': self.Alpha_Rewards,
                                    'Beta_Rewards': self.Beta_Rewards},
                                   self.episode)
                self.writer.flush()
                print(" ********************************** Rewards：", self.Epi_Rewards)
                filename = log_result_dir + 'Reward_list.txt'
                with open(filename, 'a') as file:
                    file.write("{} \n".format(self.Epi_Rewards))
                # filename1 = log_result_dir + 'CurTime_list.txt'
                # with open(filename1, 'a') as file:
                #     file.write("{} \n".format(self.CurTime))
                if self.episode % save_model_freq == 0:
                    self.writer1 = SummaryWriter(log_result_dir + f'/SummaryWriterLogs/experiment_{self.episode}')
                    for i, reward in enumerate(self.currentRewardBuffer):
                        self.writer1.add_scalar('Step_Reward', reward, i)
                    self.writer1.flush()
                    self.ppoModel.save()
                if self.episode % update_timestep == 0:
                    self.ppoModel.update(self.episode)
            else:
                print(".......")

        return action

    def getAction(self):
        return self.Action
