import math
from utils.RongAoUtils import RongAoUtils
from agent.params import  *

def getStateRewardDone(cur_observation,pre_observation,time):
    """
    根据观测信息（now+pre），返回状态，奖励函数，是否结束
    """
    if flag_multiprocessing:
        DONE = 0  # 是否结束战斗并附带原因[0-未结束 1-本方胜 2-敌方胜[本机摔或高度过低] 3-时间到]
    else:
        DONE = True
    CurTime = time
    dest_alt = 5000
    dest_spd=  200
    dest_heading = 60
    Distance_to_Winc = 1000
    REWARD = 0  # 总奖励

    #当前帧观测数据解析
    cur_DetectedInfo = {}
    cur_WeaponSystem = {}
    cur_track = {}
    cur_SFC = {}
    cur_MissileTrack = {}
    has_cur_DetectedInfo=0
    for data in cur_observation:
        #探测信息
        if data["data_tp"] == "DetectedInfo":
            if len(data["data_info"][0]['DetectedTargets']) > 0:
                cur_DetectedInfo = data["data_info"][0]['DetectedTargets'][0]
                has_cur_DetectedInfo = 1
        #武器信息
        elif data["data_tp"] == "WeaponSystem":
            cur_WeaponSystem = data["data_info"][0]
        #自身航迹信息
        elif data["data_tp"] == "track":
            cur_track = data["data_info"][0]
        #火控信息
        elif data["data_tp"] == "SFC":
            cur_SFC = data["data_info"]
        #导弹信息
        elif data["data_tp"] == "MissileTrack":
            cur_MissileTrack = data["data_info"]

    #解析上一帧数据
    pre_DetectedInfo = {}
    pre_WeaponSystem = {}
    pre_track = {}
    pre_SFC = {}
    pre_MissileTrack = {}
    has_pre_DetectedInfo =0
    for pre_data in pre_observation:
        #探测信息
        if pre_data["data_tp"] == "DetectedInfo":
            if len(pre_data["data_info"][0]['DetectedTargets']) > 0:
                pre_DetectedInfo = pre_data["data_info"][0]['DetectedTargets'][0]
                has_pre_DetectedInfo = 1
        #武器信息
        elif pre_data["data_tp"] == "WeaponSystem":
            pre_WeaponSystem = pre_data["data_info"][0]
        #自身航迹信息
        elif pre_data["data_tp"] == "track":
            pre_track = pre_data["data_info"][0]
        #火控信息
        elif pre_data["data_tp"] == "SFC":
            pre_SFC = pre_data["data_info"]
        #导弹信息
        elif pre_data["data_tp"] == "MissileTrack":
            pre_MissileTrack = pre_data["data_info"]

    #观测数据
    #1.自身航向[-180-180]
    cur_self_heading = cur_track['Heading']
    pre_self_heading = pre_track['Heading']
    #2.自身滚转[-180,180]
    cur_self_roll = cur_track['Roll']
    pre_self_roll = pre_track['Roll']
    #3.自身俯仰[-90,90]
    cur_self_pitch = cur_track['Pitch']
    pre_self_pitch = pre_track['Pitch']
    #4.自身高度[0-10000]
    cur_self_alt = cur_track['Altitude']
    pre_self_alt = pre_track['Altitude']
    #5.自身速度[0-500m/s]
    cur_self_v_n = cur_track['V_N']
    pre_self_v_n = pre_track['V_N']
    cur_self_v_e = cur_track['V_E']
    pre_self_v_e  = pre_track['V_E']
    cur_self_v_d = cur_track['V_D']
    pre_self_v_d = pre_track['V_D']
    cur_self_alpha = cur_track['alpha']
    pre_self_alpha = pre_track['alpha']
    cur_self_beta = cur_track['beta']
    pre_self_beta = pre_track['beta']
    cur_self_p = cur_track['p']  # 滚转角速度(弧度每秒)
    cur_self_q = cur_track['q']  # 俯仰角速度(弧度每秒)
    cur_self_r = cur_track['r']  # 侧滑角速度(弧度每秒)
    cur_self_v_real = (cur_self_v_e*cur_self_v_e + cur_self_v_d*cur_self_v_d + cur_self_v_n*cur_self_v_n)**0.5
    pre_self_v_real = (pre_self_v_e * pre_self_v_e + pre_self_v_d * pre_self_v_d + pre_self_v_n * pre_self_v_n) ** 0.5

    cur_deta_alt = 0
    if has_cur_DetectedInfo == 1:
        cur_deta_alt = math.fabs(cur_DetectedInfo['Altitude'] - cur_self_alt)
    # cur_deta_real_v = math.fabs(dest_spd - cur_self_v_real)
    # cur_deta_alpha = math.fabs(cur_self_alpha - pre_self_alpha)

    pre_deta_heading = math.fabs(dest_heading - pre_self_heading)
    if has_pre_DetectedInfo == 1:
        pre_deta_alt = math.fabs(pre_DetectedInfo['Altitude'] - pre_self_alt)
    # pre_deta_real_v = math.fabs(dest_spd - pre_self_v_real)

    distanceVector3D = [0,0,0]
    attackAngle = 0
    escapeAngle = 0
    if has_cur_DetectedInfo == 1:
        distance = RongAoUtils.getDistance3D(cur_track, cur_DetectedInfo)  # 计算两架飞机的距离
        distanceVector3D = RongAoUtils.getDistanceVector3D(cur_track, cur_DetectedInfo)
        myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_track)
        enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_DetectedInfo)
        enemySpeed = RongAoUtils.getSpeed(cur_DetectedInfo)

        # 计算我方攻击角[0-pi]---->越小越好
        attackAngle = math.acos((myPlaneSpeedVector[0] * distanceVector3D[0] + myPlaneSpeedVector[1] * distanceVector3D[1] +
                                 myPlaneSpeedVector[2] * distanceVector3D[2]) / distance)
        # 计算敌方逃逸角[0-pi]---->越小越好
        escapeAngle = math.acos((enemyPlaneSpeedVector[0] * distanceVector3D[0] + enemyPlaneSpeedVector[1] *
                                 distanceVector3D[1] + enemyPlaneSpeedVector[2] * distanceVector3D[2]) / distance)
    if has_pre_DetectedInfo == 1:
        pre_distance = RongAoUtils.getDistance3D(pre_track, pre_DetectedInfo)  # 计算两架飞机的距离
        pre_distanceVector3D = RongAoUtils.getDistanceVector3D(pre_track, pre_DetectedInfo)
        pre_myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_track)
        pre_enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_DetectedInfo)
        pre_enemySpeed = RongAoUtils.getSpeed(pre_DetectedInfo)

        # 计算我方攻击角[0-pi]---->越小越好
        pre_attackAngle = math.acos((pre_myPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_myPlaneSpeedVector[1] * pre_distanceVector3D[1] +
                                 pre_myPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)
        # 计算敌方逃逸角[0-pi]---->越小越好
        pre_escapeAngle = math.acos((pre_enemyPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_enemyPlaneSpeedVector[1] *
                                 pre_distanceVector3D[1] + pre_enemyPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)

    # STATE--维的状态观测信息->归一化
    STATE = [cur_self_heading/180, cur_self_pitch/90, cur_self_roll/180, cur_self_alt/10000, cur_self_v_n/300,
             cur_self_v_e/300, cur_self_v_d/300, cur_self_alpha/math.pi, cur_self_beta/math.pi , distanceVector3D[0] / 1000, distanceVector3D[1] / 1000,
             attackAngle, escapeAngle]

    #对局结束奖励
    if cur_self_alt > 10000 or cur_self_alt < 1000 :
        REWARD = -50 #-30
        if flag_multiprocessing:
            DONE = 2
        print("高度越界,当前高度为:", cur_self_alt)
    elif cur_self_alpha < -math.pi / 15 or cur_self_alpha > math.pi / 4.5: #[-12,40]
        REWARD = -30
        if flag_multiprocessing:
            DONE = 2
        print("迎角失控，当前迎角:", cur_self_alpha)
    elif CurTime > 599:
        if flag_multiprocessing:
            DONE = 2
        REWARD = -30
        print("时间到，结束   时间：", CurTime)
    elif  has_cur_DetectedInfo == 1 and attackAngle < math.pi / 6 and distance < Distance_to_Winc:
        print("达成目标，结束    时间：", CurTime , 'attackAngle: ' , attackAngle , 'escapeAngle: ', escapeAngle)
        if flag_multiprocessing:
            DONE = 1
        if escapeAngle < math.pi / 6:  # 大胜
            REWARD = 50
        elif math.pi / 6 <= escapeAngle < math.pi * 4 / 6:  # 小胜
            REWARD = 35
        else:  # 平局
            REWARD = 15
    else:
        if not flag_multiprocessing:
            DONE = False
        if pre_observation is None:
            REWARD = 0
        else:
            REWARD_ALT = 0        # 高度奖励
            # REWARD_SPD = 0      # 速度奖励
            REWARD_HEADING = 0    # 航向奖励
            REWARD_ATTACK = 0     # 攻击角奖励
            REWARD_ESCAPE = 0     # 逃逸角奖励
            REWARD_DISTANCE = 0   # 距离奖励
            REWARD_ALPHA = 0      # 迎角奖励

            #高度奖励
            if has_pre_DetectedInfo == 1:
                if cur_deta_alt > pre_deta_alt :
                    REWARD_ALT -= 1
                else:
                    REWARD_ALT += 1

            #速度奖励
            # if cur_deta_real_v < pre_deta_real_v:
            #     REWARD_SPD -= 1
            # else:
            #     REWARD_SPD += 1
            # if cur_self_v_real<dest_spd+20 and cur_self_v_real>dest_spd-20:
            #     REWARD_SPD += 1
            # else:
            #     REWARD_SPD -= 1

            if has_pre_DetectedInfo == 1:
                if cur_self_heading > math.pi:
                    cur_self_heading = math.pi*2 - cur_self_heading
                if pre_self_heading > math.pi:
                    pre_self_heading = math.pi*2 - pre_self_heading

                # 航向奖励
                if cur_self_heading > pre_self_heading:
                    REWARD_HEADING -= 1
                else:
                    REWARD_HEADING += 1

                # 攻击角奖励
                if pre_attackAngle > attackAngle:
                    if attackAngle > 0.12:
                        REWARD_ATTACK = 1
                    else:
                        REWARD_ATTACK = 0.1
                elif pre_attackAngle == attackAngle:
                    REWARD_ATTACK = 0
                else:
                    if attackAngle > 0.12:
                        REWARD_ATTACK = -1
                    else:
                        REWARD_ATTACK = -0.1

                # 逃逸角奖励
                if pre_escapeAngle > escapeAngle:
                    if escapeAngle > 0.12:
                        REWARD_ESCAPE = 1
                    else:
                        REWARD_ESCAPE = 0.1
                elif pre_escapeAngle == escapeAngle:
                    REWARD_ESCAPE = 0
                else:
                    if escapeAngle > 0.12:
                        REWARD_ESCAPE = -1
                    else:
                        REWARD_ESCAPE = -0.1

                # 距离奖励
                if distanceVector3D[0] > pre_distanceVector3D[0]:
                    REWARD_DISTANCE -= 5#1
                else:
                    REWARD_DISTANCE += 5#1
                if distanceVector3D[1] > pre_distanceVector3D[1]:
                    REWARD_DISTANCE -= 5#1
                else:
                    REWARD_DISTANCE += 5#1

            # 迎角奖励
            if cur_self_alpha < -math.pi / 18 or cur_self_alpha > math.pi / 6: # [-10,30]
                REWARD_ALPHA -= 1

            # 总奖励值
            #REWARD = (REWARD_ALT*0.01 + REWARD_HEADING*0.01 + REWARD_ALPHA * 0.05 + time*0.0001 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.015) * 0.1
            REWARD = (REWARD_ALT * 0.01 + REWARD_HEADING * 0.01 + REWARD_ALPHA * 0.05 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.015) * 0.1

    return STATE, REWARD, DONE
