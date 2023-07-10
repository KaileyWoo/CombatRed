import math
from utils.RongAoUtils import RongAoUtils
from agent.params import flag_multiprocessing

#将角度切换至(-PI,PI]之间
def Check_Rad_FABS_PI(val):
    if val >= 3.14159:
        val = val - 2*3.14159
        return Check_Rad_FABS_PI(val)
    elif val < -3.14159:
        val = val + 2*3.14159
        return Check_Rad_FABS_PI(val)
    else:
        return val
#根据两点经纬度,求两点之间的直线距离[不够精确,但够用]
def Distance_P2P(lon1,lat1,lon2,lat2):
    lon1 = lon1*math.pi/180
    lat1 = lat1*math.pi/180
    lon2 = lon2*math.pi/180
    lat2 = lat2*math.pi/180

    deta_lon = lon2-lon1
    deta_lat = lat2-lat1
    a = math.sin(deta_lat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(deta_lon/2)**2
    c = 2 * math.asin(a**0.5)
    r = 6371000

    dis = c*r

    return dis

def getStateRewardDone(cur_observation,pre_observation,time):
    """
    根据观测信息（now+pre），返回状态，奖励函数，是否结束
    """
    if flag_multiprocessing:
        DONE = 0  #是否结束战斗并附带原因[0-未结束 1-本方胜 2-敌方胜[本机摔或高度过低] 3-时间到]
    else:
        DONE = True
    CurTime = time
    Distance_Target = 1000
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
    cur_self_alpha = cur_track['alpha']  # 迎角/攻角
    pre_self_alpha = pre_track['alpha']
    cur_self_beta = cur_track['beta']    # 侧滑角/滑移角
    pre_self_beta = pre_track['beta']
    cur_self_p = cur_track['p']  # 滚转角速度(弧度每秒)
    cur_self_q = cur_track['q']  # 俯仰角速度(弧度每秒)
    cur_self_r = cur_track['r']  # 侧滑角速度(弧度每秒)
    cur_self_v_real = (cur_self_v_e*cur_self_v_e + cur_self_v_d*cur_self_v_d + cur_self_v_n*cur_self_v_n)**0.5
    pre_self_v_real = (pre_self_v_e * pre_self_v_e + pre_self_v_d * pre_self_v_d + pre_self_v_n * pre_self_v_n) ** 0.5
    cur_self_lon = cur_track['Longitude']  # 经度(°)
    cur_self_lat = cur_track['Latitude']  # 纬度(°)
    pre_self_lon = pre_track['Longitude']  # 经度(°)
    pre_self_lat = pre_track['Latitude']  # 纬度(°)

    #敌方信息
    cur_enemy_lon = cur_DetectedInfo['Longitude']  # 经度(°)
    cur_enemy_lat = cur_DetectedInfo['Latitude']  # 纬度(°)
    pre_enemy_lon = pre_DetectedInfo['Longitude']  # 经度(°)
    pre_enemy_lat = pre_DetectedInfo['Latitude']  # 纬度(°)
    cur_enemy_alt = cur_DetectedInfo['Altitude']  # 高度(m)
    pre_enemy_alt = pre_DetectedInfo['Altitude']  # 高度(m)
    cur_enemy_heading = cur_DetectedInfo['Heading']
    pre_enemy_heading = pre_DetectedInfo['Heading']
    cur_enemy_v_n = cur_DetectedInfo['V_N']  # 北向速度(m/s)
    cur_enemy_v_e = cur_DetectedInfo['V_E']  # 东向速度(m/s)
    cur_enemy_v_d = cur_DetectedInfo['V_D']  # 地向速度(m/s)
    pre_enemy_v_n = pre_DetectedInfo['V_N']  # 北向速度(m/s)
    pre_enemy_v_e = pre_DetectedInfo['V_E']  # 东向速度(m/s)
    pre_enemy_v_d = pre_DetectedInfo['V_D']  # 地向速度(m/s)
    cur_enemy_v_real = (cur_enemy_v_n * cur_enemy_v_n + cur_enemy_v_e * cur_enemy_v_e + cur_enemy_v_d * cur_enemy_v_d) ** 0.5
    pre_enemy_v_real = (pre_enemy_v_n * pre_enemy_v_n + pre_enemy_v_e * pre_enemy_v_e + pre_enemy_v_d * pre_enemy_v_d) ** 0.5

    alt_diff = cur_self_alt - cur_enemy_alt  # 两机高度差(m)
    pre_alt_diff = pre_self_alt - pre_enemy_alt  # 两机高度差(m)
    heading_diff = math.pi - (cur_self_heading - cur_enemy_heading + math.pi) % (2 * math.pi) # 航向差
    distance = RongAoUtils.getDistance3D(cur_track, cur_DetectedInfo)  # 计算两架飞机的距离
    pre_distance = RongAoUtils.getDistance3D(pre_track, pre_DetectedInfo)  # 计算两架飞机的距离

    #cur_self_2_target_pos_heading = math.atan2(cur_enemy_lat - cur_self_lat, cur_enemy_lon - cur_self_lon)  # 计算敌相对我机的角度[纯位置]正东为0,逆时针为正(弧度)
    #cur_spd_heading = math.atan2(cur_self_v_n, cur_self_v_e)  # 我方速度方向 正东为0,逆时针为正(弧度)
    #attackAngle = Check_Rad_FABS_PI(cur_self_2_target_pos_heading - cur_spd_heading)  # 期望飞行方向与实际飞行方向的偏离程度, 转移到-PI到PI之间distanceVector3D = RongAoUtils.getDistanceVector3D(cur_track, cur_DetectedInfo)
    #cur_enemy_spd_heading = math.atan2(cur_enemy_v_n, cur_enemy_v_e)  # 敌方速度方向
    #escapeAngle = Check_Rad_FABS_PI(cur_self_2_target_pos_heading - cur_enemy_spd_heading)

    # pre_self_2_target_pos_heading = math.atan2(pre_enemy_lat - pre_self_lat, pre_enemy_lon - pre_self_lon)  # 计算敌相对我机的角度[纯位置]正东为0,逆时针为正(弧度)
    # pre_spd_heading = math.atan2(pre_self_v_n, pre_self_v_e)  # 我方速度方向 正东为0,逆时针为正(弧度)
    # pre_attackAngle = Check_Rad_FABS_PI(pre_self_2_target_pos_heading - pre_spd_heading)  # 期望飞行方向与实际飞行方向的偏离程度, 转移到-PI到PI之间

    distanceVector3D = RongAoUtils.getDistanceVector3D(cur_track, cur_DetectedInfo)
    myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_track)
    enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(cur_DetectedInfo)
    attackAngle = math.acos((myPlaneSpeedVector[0] * distanceVector3D[0] + myPlaneSpeedVector[1] * distanceVector3D[1] +
                             myPlaneSpeedVector[2] * distanceVector3D[2]) / distance)
    escapeAngle = math.acos((enemyPlaneSpeedVector[0] * distanceVector3D[0] + enemyPlaneSpeedVector[1] *
                             distanceVector3D[1] + enemyPlaneSpeedVector[2] * distanceVector3D[2]) / distance)

    pre_distanceVector3D = RongAoUtils.getDistanceVector3D(pre_track, pre_DetectedInfo)
    pre_myPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_track)
    #pre_enemyPlaneSpeedVector = RongAoUtils.getSpeedVector3D(pre_DetectedInfo)
    pre_attackAngle = math.acos((pre_myPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_myPlaneSpeedVector[1] * pre_distanceVector3D[1] +
                             pre_myPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)
    # pre_escapeAngle = math.acos((pre_enemyPlaneSpeedVector[0] * pre_distanceVector3D[0] + pre_enemyPlaneSpeedVector[1] *
    #                          pre_distanceVector3D[1] + pre_enemyPlaneSpeedVector[2] * pre_distanceVector3D[2]) / pre_distance)



    # STATE--维的状态观测信息->归一化
    STATE = [cur_self_roll/math.pi, cur_self_pitch/math.pi, cur_self_alpha/math.pi, cur_self_beta/math.pi,
             cur_self_p/math.pi, cur_self_q/math.pi, cur_self_r/math.pi, cur_self_v_real/600,
             heading_diff/math.pi, alt_diff/10000, attackAngle/math.pi, escapeAngle/math.pi, distance/Distance_Max]

    #print("heading_diff:", heading_diff * 180 / math.pi, "cur_self_heading:", cur_self_heading * 180 / math.pi, "cur_DetectedInfo['Heading']:", cur_DetectedInfo['Heading'] * 180 / math.pi)
    #print("distance:",distance," attackAngle:", attackAngle*180/math.pi, " escapeAngle:", escapeAngle*180/math.pi)
    TotalReward = 0
    AltReward = 0
    DisReward = 0
    AttackReward = 0
    AlphaReward = 0
    BataReward = 0
    #对局结束奖励/稀疏奖励
    if cur_self_alt > 10000 or cur_self_alt < 1000:
        TotalReward = -50 + math.tanh((1000 - distance) / 10000)
        if flag_multiprocessing:
            DONE = 2
        print("高度越界，结束  时间: ", CurTime, "高度:", cur_self_alt, ", TotalReward=", TotalReward, ", distance=", distance)
    elif cur_self_alpha < -math.pi / 12 or cur_self_alpha > math.pi / 4.5:   #[-15, 40]
        TotalReward = -50 + math.tanh((1000 - distance) / 10000)
        if flag_multiprocessing:
            DONE = 2
        print("攻角失控，结束  时间: ", CurTime, "攻角:", cur_self_alpha*180/math.pi, ", TotalReward=", TotalReward, ", distance=", distance)
    elif cur_self_beta < -math.pi / 6 or cur_self_beta > math.pi / 6:   #[-30, 30]
        TotalReward = -50 + math.tanh((1000 - distance) / 10000)
        if flag_multiprocessing:
            DONE = 2
        print("侧滑角失控，结束  时间: ", CurTime, "侧滑角:", cur_self_beta*180/math.pi, ", TotalReward=", TotalReward, ", distance=", distance)
    elif CurTime > 299:
        TotalReward = -50 + 0.8*math.tanh((1000 - distance) / 10000) + 0.2*math.tanh(-math.fabs(attackAngle) / math.pi)
        if flag_multiprocessing:
            DONE = 3
        print("--------------------------超时，", "TotalReward=", TotalReward, ", distance=", distance,", attackAngle=", attackAngle*180/math.pi)
    elif distance > Distance_Max:
        TotalReward = -50 + math.tanh(-math.fabs(attackAngle) / math.pi)
        if flag_multiprocessing:
            DONE = 3
        print("超出最大距离范围，", "TotalReward=", TotalReward, ", attackAngle=", attackAngle * 180 / math.pi)
    #elif has_cur_DetectedInfo == 1 and math.fabs(attackAngle) < math.pi / 6 and distance < Distance_Target:
    elif has_cur_DetectedInfo == 1 and distance < Distance_Target:
        #TotalReward = 50 + 0.8*math.exp(-CurTime / 300) + 0.2*math.exp(-(math.fabs(escapeAngle))/math.pi)
        TotalReward = 50 + math.exp(-CurTime / 300)
        if flag_multiprocessing:
            DONE = 1
        print("***********************************************达成目标***********************************************")
        print("达成目标 时间：", CurTime, "TotalReward=", TotalReward, ', 距离:', distance, ', 高度:', cur_self_alt,
              ', attackAngle:', attackAngle * 180 / math.pi,', escapeAngle:', escapeAngle * 180 / math.pi)
    else:
        #单步奖励
        if not flag_multiprocessing:
            DONE = False
        if pre_observation is None:
            TotalReward = 0
        else:
            dis_reward = 0
            spd_reward = 0
            alt_reward = 0
            attack_reward = 0
            alpha_reward = 0
            beta_reward = 0
            if has_cur_DetectedInfo == 1 and has_pre_DetectedInfo == 1:
                # 距离奖励
                pre_distance -= Distance_Target
                distance -= Distance_Target
                #if distance > 0:
                #    dis_reward = math.tanh((pre_distance-distance)/10)
                dis_reward = math.tanh((pre_distance - distance) / 10)

                # 速度奖励
                #spd_reward = math.tanh((distance/10000)*(cur_self_v_real-pre_self_v_real))

                # 高度奖励
                #if alt_diff > 1000 or alt_diff < 0:
                #    alt_reward = math.tanh((math.fabs(pre_alt_diff)-math.fabs(alt_diff))/10)
                alt_reward = math.tanh((math.fabs(pre_alt_diff) - math.fabs(alt_diff)) / 10)

                # 攻击角奖励
                #if math.fabs(attackAngle) > math.pi / 6:
                #    attack_reward = math.tanh((pre_attackAngle-attackAngle)*100)
                attack_reward = math.tanh((pre_attackAngle - attackAngle) * 100)
            # 迎角奖励 [-10,30]
            if cur_self_alpha < -math.pi / 18:
                pre_alpha_diff = -math.pi / 18 - pre_self_alpha
                cur_alpha_diff = -math.pi / 18 - cur_self_alpha
                alpha_reward = math.tanh((pre_alpha_diff - cur_alpha_diff)*10)
            elif cur_self_alpha > math.pi / 6:
                pre_alpha_diff = pre_self_alpha - math.pi / 6
                cur_alpha_diff = cur_self_alpha - math.pi / 6
                alpha_reward = math.tanh((pre_alpha_diff - cur_alpha_diff)*10)

            # 侧滑角奖励 [-20,20]
            if cur_self_beta < -math.pi / 9 or cur_self_beta > math.pi / 9:
                pre_beta_diff = math.fabs(pre_self_beta) - math.pi / 9
                cur_beta_diff = math.fabs(cur_self_beta) - math.pi / 9
                beta_reward = math.tanh((pre_beta_diff - cur_beta_diff)*10)

            TotalReward = 0.5*dis_reward + 0.1*alt_reward + 0.2*attack_reward + 0.1*alpha_reward + 0.1*beta_reward + math.tanh(-CurTime / 300)
            if TotalReward < -1:
                TotalReward = -1
            elif TotalReward > 1:
                TotalReward = 1
            TotalReward *= 0.01
            AltReward = alt_reward * 0.001
            DisReward = dis_reward * 0.001
            AttackReward = attack_reward * 0.001
            AlphaReward = alpha_reward * 0.001
            BataReward = beta_reward * 0.001

    return STATE, TotalReward, DONE, AltReward, DisReward, AttackReward, AlphaReward, BataReward
