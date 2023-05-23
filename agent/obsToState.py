import math
from utils.RongAoUtils import RongAoUtils
from agent.params import flag_multiprocessing

def getStateRewardDone(cur_observation,pre_observation,time):
    """
    根据观测信息（now+pre），返回状态，奖励函数，是否结束
    """
    if flag_multiprocessing:
        DONE = 0  #是否结束战斗并附带原因[0-未结束 1-本方胜 2-敌方胜[本机摔或高度过低] 3-时间到]
    else:
        DONE = True
    CurTime = time
    Distance_to_Winc = 1000
    Distance_Max = 30000

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
    cur_accelerations_x = cur_track['accelerations_x']
    pre_accelerations_x = pre_track['accelerations_x']
    cur_accelerations_y = cur_track['accelerations_y']
    pre_accelerations_y = pre_track['accelerations_y']
    cur_accelerations_z = cur_track['accelerations_z']
    pre_accelerations_z = pre_track['accelerations_z']
    cur_self_alpha = cur_track['alpha']
    pre_self_alpha = pre_track['alpha']
    cur_self_beta = cur_track['beta']
    pre_self_beta = pre_track['beta']
    cur_self_v_real = (cur_self_v_e*cur_self_v_e + cur_self_v_d*cur_self_v_d + cur_self_v_n*cur_self_v_n)**0.5
    pre_self_v_real = (pre_self_v_e * pre_self_v_e + pre_self_v_d * pre_self_v_d + pre_self_v_n * pre_self_v_n) ** 0.5

    #航向差
    heading_diff = 0
    #速度差
    speed_diff = 0
    #高度差
    alt_diff = 0
    #距离
    distance = 0
    #攻击角
    attackAngle = 0
    #逃逸角
    escapeAngle = 0
    if has_cur_DetectedInfo == 1:
        heading_diff = math.fabs(cur_self_heading - cur_DetectedInfo['Heading'])
        speed_diff = math.fabs(cur_self_v_real - RongAoUtils.getSpeed(cur_DetectedInfo))
        alt_diff = math.fabs(cur_self_alt - cur_DetectedInfo['Altitude'])

        distance = RongAoUtils.getDistance3D(cur_track, cur_DetectedInfo)
        distanceVector3D = RongAoUtils.getDistanceVector3D(cur_track, cur_DetectedInfo)
        myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_track)
        enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_DetectedInfo)
        # 计算我方攻击角[0-pi]---->越小越好
        attackAngle = math.acos((myPlaneSpeedVector[0] * distanceVector3D[0] + myPlaneSpeedVector[1] * distanceVector3D[1] +
                                 myPlaneSpeedVector[2] * distanceVector3D[2]) / distance)
        # 计算敌方逃逸角[0-pi]---->越小越好
        escapeAngle = math.acos((enemyPlaneSpeedVector[0] * distanceVector3D[0] + enemyPlaneSpeedVector[1] * distanceVector3D[1] +
                                 enemyPlaneSpeedVector[2] * distanceVector3D[2]) / distance)

    # 航向差
    pre_heading_diff = 0
    # 速度差
    pre_speed_diff = 0
    # 高度差
    Pre_alt_diff = 0
    # 距离
    pre_distance = 0
    # 攻击角
    pre_attackAngle = 0
    # 逃逸角
    pre_escapeAngle = 0
    if has_pre_DetectedInfo == 1:
        pre_heading_diff = math.fabs(pre_self_heading - pre_DetectedInfo['Heading'])
        pre_speed_diff = math.fabs(pre_self_v_real - RongAoUtils.getSpeed(pre_DetectedInfo))
        pre_alt_diff = math.fabs(pre_self_alt - pre_DetectedInfo['Altitude'])

        pre_distance = RongAoUtils.getDistance3D(pre_track, pre_DetectedInfo)
        pre_distanceVector3D = RongAoUtils.getDistanceVector3D(pre_track, pre_DetectedInfo)
        pre_myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_track)
        pre_enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_DetectedInfo)
        # 计算我方攻击角[0-pi]---->越小越好
        pre_attackAngle = math.acos(
            (pre_myPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_myPlaneSpeedVector[1] * pre_distanceVector3D[1] +
             pre_myPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)
        # 计算敌方逃逸角[0-pi]---->越小越好
        pre_escapeAngle = math.acos(
            (pre_enemyPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_enemyPlaneSpeedVector[1] * pre_distanceVector3D[1] +
             pre_enemyPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)

    # STATE--维的状态观测信息->归一化
    STATE = [cur_self_roll/math.pi, cur_self_pitch/math.pi, cur_self_alpha/math.pi, cur_self_beta/math.pi,
             heading_diff/(math.pi*2.0), speed_diff/600, alt_diff/10000, attackAngle/math.pi, escapeAngle/math.pi, distance/Distance_Max,
             distanceVector3D[0]/Distance_Max,distanceVector3D[1]/Distance_Max, distanceVector3D[2]/Distance_Max]

    TotalReward = 0
    EndReward = 0
    StepReward = 0
    #对局结束奖励/稀疏奖励
    if cur_self_alt > 10000 or cur_self_alt < 1000:
        TotalReward = -50
        if flag_multiprocessing:
            DONE = 2
        print("高度越界，结束  时间: ", CurTime, "当前高度为:", cur_self_alt)
    elif cur_self_alpha < -math.pi / 15 or cur_self_alpha > math.pi / 4.5:
        TotalReward = -30
        if flag_multiprocessing:
            DONE = 2
        print("迎角失控，当前迎角:", cur_self_alpha*180/math.pi)
    elif CurTime > 299:
        TotalReward = -40
        if flag_multiprocessing:
            DONE = 3
        print("--------------------------超时，当前时间：", CurTime)
    #elif has_cur_DetectedInfo == 1 and attackAngle < math.pi / 6 and distance < Distance_to_Winc:
    elif has_cur_DetectedInfo == 1 and distance < Distance_to_Winc:
        print("达成目标 时间：", CurTime, ' 距离:', distance, '高度:', cur_self_alt, 'attackAngle:', attackAngle*180/math.pi, 'escapeAngle:', escapeAngle*180/math.pi)
        if flag_multiprocessing:
            DONE = 1
        TotalReward = 50
        # if escapeAngle < math.pi / 6:  # 大胜
        #     EndReward = 50
        # elif math.pi / 6 <= escapeAngle < math.pi * 4 / 6:  # 小胜
        #     EndReward = 35
        # else:  # 平局
        #     EndReward = 15
    else:
        #单步奖励
        if not flag_multiprocessing:
            DONE = False
        if pre_observation is None:
            TotalReward = 0
        else:
            #角速度
            angular_speed = math.fabs(cur_self_heading - pre_self_heading)/math.pi
            #加速度变量
            acceleration_change = math.fabs(cur_accelerations_x - pre_accelerations_x) + math.fabs(cur_accelerations_y - pre_accelerations_y) + \
                                  math.fabs(cur_accelerations_z - pre_accelerations_z)

            # StepReward = 0.2*math.exp(-distance) - 0.01*angular_speed - 0.01*math.exp(-acceleration_change) + 0.2*math.exp(-attackAngle*180/math.pi) + 0.2*math.exp(-escapeAngle*180/math.pi) + \
            #              0.1*math.exp(-heading_diff*180/math.pi) + 0.2*math.exp(-speed_diff) + 0.1*math.exp(-alt_diff)

            dis_reward = 0
            spd_reward = 0
            alt_reward = 0
            attack_reward = 0
            alpha_reward = 0
            if has_cur_DetectedInfo == 1 and has_pre_DetectedInfo == 1:
                if distance < pre_distance:
                    dis_reward = math.exp(-(pre_distance-distance)/1000)
                elif distance > pre_distance:
                    dis_reward = -math.exp(-(distance-pre_distance)/1000)

                if cur_self_v_real < RongAoUtils.getSpeed(cur_DetectedInfo):
                    if speed_diff < pre_speed_diff:
                        spd_reward = -math.exp(-(pre_speed_diff-speed_diff)/100)
                    elif speed_diff > pre_speed_diff:
                        spd_reward = math.exp(-(speed_diff-pre_speed_diff)/100)

                if cur_self_alt < cur_DetectedInfo['Altitude']:
                    if alt_diff < pre_alt_diff:
                        alt_reward = math.exp(-(pre_alt_diff-alt_diff)/1000)
                    elif alt_diff > pre_alt_diff:
                        alt_reward = -math.exp(-(alt_diff-pre_alt_diff)/1000)

                if attackAngle < pre_attackAngle:
                    attack_reward = math.exp(-(pre_attackAngle-attackAngle)/math.pi)
                elif attackAngle > pre_attackAngle:
                    attack_reward = -math.exp(-(attackAngle-pre_attackAngle)/math.pi)

            if cur_self_alpha < -math.pi / 18 or cur_self_alpha > math.pi / 6:
                alpha_reward -= 1

            TotalReward = 0.1*dis_reward + 0.1*spd_reward + 0.1*alt_reward + 0.2*attack_reward + 0.5*alpha_reward
            if TotalReward < -1:
                TotalReward = -1
            elif TotalReward > 1:
                TotalReward = 1
            TotalReward *= 0.01

    #TotalReward = 0.1*StepReward + 0.9*EndReward  #[-25.31607, 25.5]
    #normalize [-1,1]
    #TotalReward = (TotalReward + 25.5) / 51 * 2 - 1
    #print(StepReward)

    return STATE, TotalReward, DONE
