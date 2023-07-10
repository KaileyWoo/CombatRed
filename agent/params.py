import os
max_ep_len = 10000
max_training_timesteps = int(5e7)
print_freq = max_ep_len * 100  # print avg reward in the interval (in num timesteps)
log_freq = max_ep_len * 10  # log avg reward in the interval (in num timesteps)
save_model_freq = 250  # save model frequency (in num timesteps)
action_std = 0.6  # starting std for action distribution (Multivariate Normal)
action_std_decay_rate = 0.05  # linearly decay action_std (action_std = action_std - action_std_decay_rate)
min_action_std = 0.1  # minimum action_std (stop decay after action_std <= min_action_std)
action_std_decay_freq = int(2.5e5)  # action_std decay frequency (in num timesteps)
update_timestep = 25      # update policy every n timesteps
K_epochs = 160               # update policy for K epochs in one PPO update
eps_clip = 0.15          # clip parameter for PPO
gamma = 0.999            # discount factor
lr_actor = 0.0003 #0.0003       # learning rate for actor network
lr_critic = 0.001       # learning rate for critic network
random_seed = 0         # set random seed if required (0 = no random seed)

StateDim = 13 # 状态维度
ActionDim = 4  # 动作维度
Train = 1  # 0，1，2分别表示：0重新训练，1加载之前的训练，2测试模式

#Distance_To_Win = 1000  #设置对局获胜需要的尾追距离

#  for test
flag_multiprocessing = True
#use_adv_norm = True  # Trick 1:advantage normalization
use_state_norm = False  # Trick 2:state normalization
use_reward_norm = False   # Trick 3:reward normalization
use_reward_scaling = False  # Trick 4:reward scaling
use_lr_decay = False  # Trick 6:learning rate Decay
use_grad_clip = True  # Trick 7: Gradient clip

log_dir_1 = "agent/red_Agent/ppo_agent"
if not os.path.exists(log_dir_1):
    os.makedirs(log_dir_1)
log_dir = log_dir_1 + "/model_TB_ScenarioInfo1/003/"
if not os.path.exists(log_dir):
    os.makedirs(log_dir)
# else:
#     if Train==0 and os.path.exists(log_dir + "ppo.pkl"):
#         os.remove(log_dir + "ppo.pkl")

load_dir = log_dir_1 + "/model_TB_ScenarioInfo1/003/"
if not os.path.exists(load_dir):
    os.makedirs(load_dir)

log_result_dir = "./myResult/TB_ScenarioInfo1/003/"
if not os.path.exists(log_result_dir):
    os.makedirs(log_result_dir)
# else:
#     if Train==0 and os.path.exists(log_result_dir + 'Reward_list.txt'):
#         os.remove(log_result_dir + 'Reward_list.txt')
