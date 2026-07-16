import sys
import os

os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"

sys.path.append(os.path.abspath("./build"))
from lowcontrol_py import *

ctrl = LowController.instance()

ctrl.init()

cmds = [MotorCmd() for _ in range(21)]

for i in range(21):
    cmds[i].pos = 0.0
    cmds[i].vel = 0.0
    cmds[i].tau = 0.0
    cmds[i].kp = 1.0
    cmds[i].kd = 0.1
    cmds[i].motor_id = i
cmds[0].pos = -2
cmds[0].kp = 5
cmds[0].kd = 5

try:
    ctrl.set_joint(cmds)
except KeyboardInterrupt:
    print("stop")
