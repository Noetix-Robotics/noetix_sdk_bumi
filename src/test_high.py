import sys
import os
from time import sleep
import numpy as np
import math

# DDS 环境配置
os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"

sys.path.append(os.path.abspath("./build"))
from highcontrol_py import *

# EDU Bumi get joydata necessary init AoLionDriver
# al = AoLionDriver()
# al.init("/dev/input/js0", 115200)

# get HighController instance and init
ctrl = HighController.instance()
ctrl.init()

# 初始化状态
key_updown = [0] * 14
key_inuse = [0] * 14
fileindex = 0

Key1 = 1
Key2 = 2
Key5 = 5
Key6 = 6
Key7 = 7
Key8 = 8
Key9 = 9
Key10 = 10
Key12 = 12

lasmode = curmode = 0

while True:
    curmode = ctrl.get_mode()
    if curmode != lasmode:
        lasmode = curmode
        print(f"[PYTHON DEBUG]: curmode is {curmode}")

    # remote_data = al.getremotedata() # EDU Bumi get joydata funtion
    remote_data = ctrl.from_dds_get_joydata()  # standard Bumi get joydata funtion

    axes = remote_data.axes
    buttons = remote_data.button

    # 更新按键状态
    for i in range(14):
        if buttons[i] == 0:
            key_updown[i] = 0
            key_inuse[i] = 0
        elif buttons[i] == 1:
            key_updown[i] = 1

    action = ControlCmd.DEFAULT

    x = axes[1]
    yaw = axes[0]

    # ===== 动作逻辑 =====
    if (
        key_updown[Key2] == 0
        and key_updown[Key5] == 1
        and key_updown[Key1] == 0
        and key_inuse[Key5] == 0
    ):
        action = ControlCmd.STARTTEACH
        key_inuse[Key5] = 1
        print("[DEBUG]: STARTTEACH")

    elif (
        key_updown[Key6] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key6] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.SWING
        key_inuse[Key6] = 1
        print("[DEBUG]: SWING")

    elif (
        key_updown[Key7] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key7] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.SHAKE
        key_inuse[Key7] = 1
        print("[DEBUG]: SHAKE")

    elif (
        key_updown[Key8] == 1
        and key_updown[Key2] == 0
        and key_inuse[Key8] == 0
        and key_updown[Key1] == 0
    ):
        action = ControlCmd.CHEER
        key_inuse[Key8] = 1
        print("[DEBUG]: CHEER")

    elif key_updown[Key9] == 1 and key_inuse[Key9] == 0:
        action = ControlCmd.START
        key_inuse[Key9] = 1
        print("[DEBUG]: START")

    elif key_updown[Key10] == 1 and key_inuse[Key10] == 0:
        action = ControlCmd.SWITCH
        key_inuse[Key10] = 1
        print("[DEBUG]: SWITCH")

    elif key_updown[Key2] == 1 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.WALK
        key_inuse[Key5] = 1
        print("[DEBUG]: WALK")

    elif key_updown[Key1] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.SAVETEACH
        key_inuse[Key6] = 1
        fileindex += 1
        print("[DEBUG]: SAVETEACH")

    elif key_updown[Key1] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.ENDTEACH
        key_inuse[Key7] = 1
        print("[DEBUG]: ENDTEACH")

    elif key_updown[Key1] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.PLAYTEACH
        key_inuse[Key8] = 1
        fileindex = 1
        print("[DEBUG]: PLAYTEACH")

    elif key_updown[Key1] == 1 and key_updown[Key5] == 1 and key_inuse[Key5] == 0:
        action = ControlCmd.FALLTOSTAND
        key_inuse[Key5] = 1
        print("[DEBUG]: FALLTOSTAND")

    elif key_updown[Key2] == 1 and key_updown[Key6] == 1 and key_inuse[Key6] == 0:
        action = ControlCmd.DANCE
        key_inuse[Key6] = 1
        print("[DEBUG]: DANCE")

    elif key_updown[Key2] == 1 and key_updown[Key7] == 1 and key_inuse[Key7] == 0:
        action = ControlCmd.DANCE1
        key_inuse[Key7] = 1
        print("[DEBUG]: DANCE1")

    elif key_updown[Key2] == 1 and key_updown[Key8] == 1 and key_inuse[Key8] == 0:
        action = ControlCmd.DANCE2
        key_inuse[Key8] = 1
        print("[DEBUG]: DANCE2")

    elif key_updown[Key12] == 1 and key_updown[Key8] == 0 and key_inuse[Key12] == 0:
        action = ControlCmd.STANDTOFALL
        key_inuse[Key12] = 1
        print("[DEBUG]: STANDTOFALL")

    # ===== 发送指令 =====
    ctrl.publish_cmd(x, yaw, action, fileindex)

    sleep(0.01)  # 防止CPU打满
