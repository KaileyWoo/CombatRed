import math
from utils.RongAoUtils import RongAoUtils
from agent.params import flag_multiprocessing

# global step_step, step_speed, step_alt, step_attackangle, step_escapeangle
# step_step = 0
# step_speed = 0
# step_alt = 0
# step_attackangle = 0

def getStateRewardDone(cur_observation,pre_observation,time):
    """
    根据观测信息（now+pre），返回状态，奖励函数，是否结束
    """
    if flag_multiprocessing:
        DONE = 0  #是否结束战斗并附带原因[0-未结束 1-本方胜 2-敌方胜[本机摔或高度过低] 3-时间到]
    else:
        DONE = True
    CurTime = time
    dest_heading = 60
    Distance_to_Winc = 1000


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
    cur_self_v_real = (cur_self_v_e*cur_self_v_e + cur_self_v_d*cur_self_v_d + cur_self_v_n*cur_self_v_n)**0.5
    pre_self_v_real = (pre_self_v_e * pre_self_v_e + pre_self_v_d * pre_self_v_d + pre_self_v_n * pre_self_v_n) ** 0.5
    # 自身经纬度
    cur_self_Latitude = cur_track['Latitude']
    cur_self_Longitude = cur_track['Longitude']

    cur_deta_alt = 0
    pre_deta_alt = 0
    if has_cur_DetectedInfo == 1:
        cur_deta_alt = math.fabs(cur_DetectedInfo['Altitude'] - cur_self_alt)
    # cur_deta_real_v = math.fabs(dest_spd - cur_self_v_real)
    # cur_deta_alpha = math.fabs(cur_self_alpha - pre_self_alpha)

    pre_deta_heading = math.fabs(dest_heading - pre_self_heading)
    if has_pre_DetectedInfo == 1:
        pre_deta_alt = math.fabs(pre_DetectedInfo['Altitude'] - pre_self_alt)
    # pre_deta_real_v = math.fabs(dest_spd - pre_self_v_real)

    distanceVector3D = [0, 0, 0]
    attackAngle = 0
    escapeAngle = 0
    distance = 0
    mySpeed = RongAoUtils.getSpeed(cur_track)
    pre_mySpeed = RongAoUtils.getSpeed(pre_track)

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

        cur_deta_heading = cur_DetectedInfo['Heading']


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

        pre_deta_heading = pre_DetectedInfo['Heading']

    # STATE--维的状态观测信息->归一化
    STATE = [cur_self_heading/(math.pi*2.0), cur_self_pitch/math.pi, cur_self_roll/(math.pi*2.0), cur_self_alt/10000, cur_self_v_n/300,
             cur_self_v_e/300, cur_self_v_d/300, cur_self_alpha/math.pi, cur_self_beta/math.pi, distance / 30000, distanceVector3D[0] / 30000,
             distanceVector3D[1] / 30000, distanceVector3D[2] / 30000, attackAngle/math.pi, escapeAngle/math.pi]   #cur_self_Latitude/90, cur_self_Longitude/180

    #对局结束奖励
    if cur_self_alt > 10000 or cur_self_alt < 1000:
        REWARD = -50 #-30
        if flag_multiprocessing:
            DONE = 2
        print("高度越界，结束  时间: ", CurTime, "当前高度为:", cur_self_alt)
    elif cur_self_alpha < -math.pi / 15 or cur_self_alpha > math.pi / 4.5:
    #elif cur_self_alpha < 0 or cur_self_alpha > math.pi / 4.5:
        REWARD = -30
        if flag_multiprocessing:
            DONE = 2
        print("迎角失控，结束  时间: ", CurTime, "当前迎角:", cur_self_alpha*180/math.pi)
    elif cur_self_beta < -math.pi / 6 or cur_self_beta > math.pi / 6:
        REWARD = -30
        if flag_multiprocessing:
            DONE = 2
        print("滑移角失控，结束  时间: ", CurTime, "当前滑移角:", cur_self_beta*180/math.pi)
    elif CurTime > 590:
        REWARD = -30
        if flag_multiprocessing:
            DONE = 3
        print("----------------------------时间到：", CurTime)
    elif distance > 30000:
        REWARD = -30
        if flag_multiprocessing:
            DONE = 2
        print("两机距离大于30000m，当前距离： ", distance)
    elif has_cur_DetectedInfo == 1 and attackAngle < math.pi / 6 and distance < Distance_to_Winc:
    #elif step_step > 120:
        #print("达成目标 时间：", CurTime, " step: ", step_step, ' 距离:', distance, '高度:', cur_self_alt, 'attackAngle:', attackAngle*180/math.pi, 'escapeAngle:', escapeAngle*180/math.pi)
        print("达成目标 时间：", CurTime, ' 距离:', distance, '高度:', cur_self_alt, 'attackAngle:', attackAngle*180/math.pi, 'escapeAngle:', escapeAngle*180/math.pi)
        if flag_multiprocessing:
            DONE = 1
        #REWARD = 50
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

            # if has_cur_DetectedInfo == 1 and attackAngle < math.pi / 6 and distance < Distance_to_Winc and escapeAngle < math.pi * 4 / 6:
            #     # if escapeAngle < math.pi / 6:
            #     #     REWARD = 1
            #     # elif math.pi / 6 <= escapeAngle < math.pi * 4 / 6:
            #     #     REWARD = 0.5
            #     # else:
            #     #     REWARD = 0.2
            #     #
            #     # REWARD = REWARD * 0.1
            #     if step_step == 0:
            #         step_speed = enemySpeed
            #         step_alt = cur_DetectedInfo['Altitude']
            #
            #     step_step += 1
            #
            #     REWARD_DET_ALT = 0  # 相对高度奖励
            #     REWARD_SPD = 0  # 速度奖励
            #     REWARD_HEADING = 0  # 航向奖励
            #     REWARD_ATTACK = 0  # 攻击角奖励
            #     REWARD_ESCAPE = 0  # 逃逸角奖励
            #     REWARD_DISTANCE = 0  # 距离奖励
            #
            #     if has_pre_DetectedInfo == 1 and has_cur_DetectedInfo == 1:
            #         # 相对速度差奖励
            #         if math.fabs(mySpeed - enemySpeed) > (step_speed - 10) and math.fabs(mySpeed - enemySpeed) < (step_speed + 10):
            #             REWARD_SPD += 1
            #         else:
            #             REWARD_SPD -= 1
            #
            #         # 相对高度奖励
            #         if cur_deta_alt > pre_deta_alt:
            #             REWARD_DET_ALT -= 1
            #         elif cur_deta_alt < pre_deta_alt:
            #             REWARD_DET_ALT += 1
            #
            #         # 攻击角奖励
            #         if pre_attackAngle == attackAngle:
            #             REWARD_ATTACK += 1
            #         elif pre_attackAngle < attackAngle:
            #             REWARD_ATTACK -= 1
            #
            #         # 逃逸角奖励
            #         if pre_escapeAngle == escapeAngle:
            #             REWARD_ESCAPE += 1
            #         elif pre_escapeAngle < escapeAngle:
            #             REWARD_ESCAPE -= -1
            #
            #         # 距离奖励
            #         if distance == pre_distance:
            #             REWARD_DISTANCE += 1
            #         else:
            #             REWARD_DISTANCE -= 1
            #
            #         if math.fabs(cur_self_heading - cur_deta_heading) < math.fabs(pre_self_heading - pre_deta_heading):
            #             REWARD_HEADING += 1
            #         else:
            #             REWARD_HEADING -= 1
            #
            #     REWARD = (REWARD_DET_ALT * 0.01 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.01 + REWARD_SPD * 0.01 + REWARD_HEADING * 0.05) * 0.1
            #
            # else:
            #
            #     step_step = 0

                REWARD_ALT = 0  # 高度奖励
                REWARD_DET_ALT = 0  # 相对高度奖励
                REWARD_SPD = 0      # 速度奖励
                REWARD_HEADING = 0    # 航向奖励
                REWARD_ATTACK = 0  # 攻击角奖励
                REWARD_ESCAPE = 0  # 逃逸角奖励
                REWARD_DISTANCE = 0  # 距离奖励
                REWARD_ALPHA = 0  # 迎角奖励

                if has_pre_DetectedInfo == 1 and has_cur_DetectedInfo == 1:

                    # 相对速度差奖励
                    if distance > 5000:
                        if mySpeed > pre_mySpeed:
                            REWARD_SPD += 1
                        else:
                            REWARD_SPD -= 1
                    # if (mySpeed - enemySpeed) < (pre_mySpeed - pre_enemySpeed):
                    #     REWARD_SPD += 1
                    # else:
                    #     REWARD_SPD -= 1


                    # 相对高度奖励
                    # if cur_deta_alt > pre_deta_alt :
                    #     REWARD_DET_ALT -= 1
                    # elif cur_deta_alt < pre_deta_alt :
                    #     REWARD_DET_ALT += 1

                    # 攻击角奖励
                    # if attackAngle < pre_attackAngle:
                    #     REWARD_ATTACK += 1
                    # else:
                    #     REWARD_ATTACK -= 10
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
                    # if escapeAngle > pre_escapeAngle:
                    #     REWARD_ESCAPE += 1
                    # else:
                    #     REWARD_ESCAPE -= 1
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
                    if distance < pre_distance:
                        REWARD_DISTANCE += 1
                    elif distance > pre_distance:
                        REWARD_DISTANCE -= 1
                    elif distanceVector3D[0] > pre_distanceVector3D[0]:
                        REWARD_DISTANCE -= 1
                    elif distanceVector3D[0] < pre_distanceVector3D[0]:
                        REWARD_DISTANCE += 1
                    if distanceVector3D[1] > pre_distanceVector3D[1]:
                        REWARD_DISTANCE -= 1
                    elif distanceVector3D[1] < pre_distanceVector3D[1]:
                        REWARD_DISTANCE += 1
                    if distanceVector3D[2] > pre_distanceVector3D[2]:
                        REWARD_DISTANCE -= 1
                    elif distanceVector3D[2] < pre_distanceVector3D[2]:
                        REWARD_DISTANCE += 1

                    # 相对航向奖励
                    if distance > 5000:
                        if math.fabs(cur_self_heading - cur_deta_heading) < math.fabs(pre_self_heading - pre_deta_heading):
                            REWARD_HEADING += 1
                        else:
                            REWARD_HEADING -= 1

                # 航向奖励
                # if cur_self_heading > math.pi:
                #     cur_self_heading = math.pi * 2 - cur_self_heading
                # if pre_self_heading > math.pi:
                #     pre_self_heading = math.pi * 2 - pre_self_heading
                # if cur_self_heading > pre_self_heading:
                #     REWARD_HEADING -= 1
                # else:
                #     REWARD_HEADING += 1

                # 迎角奖励
                if cur_self_alpha < -math.pi / 18 or cur_self_alpha > math.pi / 6:
                    REWARD_ALPHA -= 1
                else:
                    REWARD_ALPHA += 0.1

                # 总奖励值
                #REWARD = (REWARD_ALT*0.01 + REWARD_HEADING*0.01 + REWARD_ALPHA * 0.05 + time*0.0001 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.015) * 0.1
                #REWARD = (REWARD_ALT * 0.01 + REWARD_HEADING * 0.01 + REWARD_ALPHA * 0.05 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.015) * 0.1
                #REWARD = (REWARD_DET_ALT * 0.015 + REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.01 + REWARD_ALPHA * 0.05 + REWARD_ALT * 0.005) * 0.1
                REWARD = (REWARD_ATTACK * 0.01 + REWARD_ESCAPE * 0.01 + REWARD_DISTANCE * 0.01 + REWARD_SPD * 0.01 + REWARD_HEADING * 0.01 + REWARD_ALPHA * 0.05) * 0.1

    return STATE, REWARD, DONE
