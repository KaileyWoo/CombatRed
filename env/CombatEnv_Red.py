import datetime
import socket
import struct
import threading
import time
import os
import json
from agent.params import flag_multiprocessing
from utils.RongAoUtils import RongAoUtils


class AirCombat_Red:
    def __init__(self, RedAgent):
        """
        初始化红方方智能体，创建仿真链接，初始化环境交互类信息
        :param RedAgent: 红方智能体
        """
        self.RedAgent = RedAgent("red")
        # 端口连接
        self.Red_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.Red_client.connect(('127.0.0.1', 8868))

        self.Red_identify_dict = {
            "msg_type": "identify",
            "msg_info": {
                "identify": "red"}}
        self.manu_ctrl = {
            "msg_info": "驾驶操控,1001,2,0,Delay,Force,0|0`0`0.6`0",
            "msg_type": "manu_ctrl"}

        self.manu_ctrl_launch = {
            "msg_info": "发射, 1001, 2, 0, Delay, Force, 0 | 1002",
            "msg_type": "manu_ctrl"}

        self.msgInit()
        self.CurTime = 0
        self.rcv_one_game_end = False
        # 缓存接收到的报文
        self.rcv_msg = bytes()

    def msgInit(self):
        """
        初始化环境交互类信息
        """
        Red_json_str = json.dumps(self.Red_identify_dict)
        msg_len = len(Red_json_str.encode("utf-8"))
        msg_len_data = struct.pack('i', msg_len)
        self.Red_client.send(msg_len_data)
        self.Red_client.send(Red_json_str.encode("utf-8"))

    def reset(self):
        self.rcv_one_game_end = False
        #self.RedAgent.reset()

    def step(self):
        """
        环境更新,31代表机动，32代表打击
        """
        cur_msg = self.Red_client.recv(1024 * 10)
        self.rcv_msg = self.rcv_msg + cur_msg
        total_len = len(self.rcv_msg)
        while total_len > 4:
            msg_len = int.from_bytes(self.rcv_msg[0:4], byteorder="little", signed=False)
            if (msg_len + 4 <= total_len):
                # 一帧数据
                one_frame_msg = self.rcv_msg[4:4 + msg_len]
                # 去除一帧后的剩余数据
                self.rcv_msg = self.rcv_msg[4 + msg_len: total_len]
                total_len = len(self.rcv_msg)

                Red_json_str_recv = json.loads(one_frame_msg)
                Red_str_tp = (Red_json_str_recv["msg_type"])
                self.CurTime = Red_json_str_recv["msg_time"]

                if Red_str_tp == "result":
                    print("红方已收到对局结束指令")
                    self.RedAgent.reset()
                    self.rcv_one_game_end = True
                elif Red_str_tp == "red_out_put":
                    if flag_multiprocessing:
                        if self.RedAgent.done != 0:
                            self.RedAgent.reset()
                            self.rcv_one_game_end = True
                        if len(Red_json_str_recv["msg_info"]) != 0:
                            RedActionInfo = self.RedAgent.action(Red_json_str_recv["msg_info"], self.CurTime)
                        else:
                            RedActionInfo = self.RedAgent.getAction()
                        msg_len = len(json.dumps(RedActionInfo).encode("utf-8"))
                        msg_len_data = struct.pack('i', msg_len)
                        self.Red_client.send(msg_len_data)
                        self.Red_client.send(json.dumps(RedActionInfo).encode("utf-8"))
                    else:
                        if self.RedAgent.done:
                            msg_len = len(json.dumps(self.manu_ctrl).encode("utf-8"))
                            msg_len_data = struct.pack('i', msg_len)
                            self.Red_client.send(msg_len_data)
                            self.Red_client.send(json.dumps(self.manu_ctrl).encode("utf-8"))
                        else:
                            RedActionInfo = self.RedAgent.action(Red_json_str_recv["msg_info"], self.CurTime)
                            msg_len = len(json.dumps(RedActionInfo).encode("utf-8"))
                            msg_len_data = struct.pack('i', msg_len)
                            self.Red_client.send(msg_len_data)
                            self.Red_client.send(json.dumps(RedActionInfo).encode("utf-8"))

                else:
                    print("红方接收到不明信息")
            else:
                break
    def done(self):
        """
        判定对局是否结束
        :return: 返回布尔类型的结束标识符
        """
        return self.rcv_one_game_end
        #return self.RedAgent.done



