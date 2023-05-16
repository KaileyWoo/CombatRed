from env.CombatEnv_Red import AirCombat_Red
from agent.ppo_agent import PPOAGENT
import time
EPISODE = 10000  # 进行训练的对局轮数

def main(Episode,red):
    for i in range(Episode):
        done = False
        print("第",i+1,"局开始")
        while not done:
            red.step()
            done = red.done()
        red.reset()
        print("第",i+1,"局结束")


if __name__ == '__main__':
    red = AirCombat_Red(PPOAGENT)
    main(EPISODE, red)
