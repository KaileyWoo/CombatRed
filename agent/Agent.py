# 这是一个基类智能体，所有的智能体都要继承这个智能体，并且实现其相关功能

class Agent(object):
    def __init__(self, camp):
        """必要的初始化"""
        self.done = False
        self.camp = camp
        self.planeID = []
        self.nowInfo = []
        self.DetectedInfo = {}
        self.WeaponSystem = {}
        self.track = {}
        self.SFC = {}
        self.MissileTrack = {}
        self.CurTime = 0


    def reset(self):
        """
        每一局对局结束后进行重置
        主要重置状态、动作、奖励等
        :return:
        """
        pass

    def action(self, planeInfo , CurTime):
        """
        传入飞机和导弹信息，进行动作选择
        """
        self.CurTime = CurTime
        self.nowInfo = planeInfo
        for data in planeInfo:
            if data["data_tp"] == "DetectedInfo":
                self.DetectedInfo = data["data_info"]
            elif data["data_tp"] == "WeaponSystem":
                self.WeaponSystem = data["data_info"][0]
            elif data["data_tp"] == "track":
                self.track = data["data_info"][0]
            elif data["data_tp"] == "SFC":
                self.SFC = data["data_info"]
            elif data["data_tp"] == "MissileTrack":
                self.MissileTrack = data["data_info"]
        manu_ctrl = {
                "msg_info": "PDI,15001,2,0,Delay,Force,0|1`1`0",
                "msg_type": "manu_ctrl"}

        return manu_ctrl

