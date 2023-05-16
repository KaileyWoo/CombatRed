from matplotlib import pyplot as plt
import numpy as np
import datetime
from statistics import mean
from agent.params import log_result_dir

def loadtxtmethod(filename):
    data = np.loadtxt(filename, dtype=np.float64, delimiter=',')
    return data

if __name__=="__main__":
    x = []
    y = []
    avg_y = []
    i = 1
    data = loadtxtmethod(log_result_dir + 'Reward_list.txt')
    for row in data:
        x.append(i)
        i += 1
        y.append(row)
        if i > 10:
            avg = mean(y[i-9:i])
            avg_y.append(avg)
        else:
            avg_y.append(row)

    plt.plot(x, avg_y, linewidth=1, color="blue", label="Mean value")
    # plt.plot(x, y, linewidth=1, color="blue", label="Mean value")
    plt.xlabel("Episode")
    plt.ylabel("Rewards")
    # plt.title("221228 PPO_with_GRU3 ")
    day = str(datetime.datetime.today().day)
    month = str(datetime.datetime.today().month)
    hour = str(datetime.datetime.today().hour)
    minute = str(datetime.datetime.today().minute)
    title = month+'-' +day+' '+hour+ ':'+minute+'  PPO '
    plt.title(title)
    # plt.yticks([-20, -15, -10, -5, 0, 5])

    plt.savefig(log_result_dir + "rewards.png")
    plt.show()