import time
import math
import copy
from typing import List
import sys
import os
import numpy as np

import yaml
import onnxruntime as ort
from dataclasses import dataclass, field
from typing import Dict

sys.path.append(os.path.abspath("./build"))
from lowcontrol_py import (
    LowController,
    MotorCmd,
    MotorState,
    NingImuData,
    JoyData,
    AoLionDriver,
)


@dataclass
class Proprioception:
    # joint position
    jointPos: np.ndarray = field(default_factory=lambda: np.zeros(21, dtype=np.float32))

    # joint velocity
    jointVel: np.ndarray = field(default_factory=lambda: np.zeros(21, dtype=np.float32))

    # imu angular velocity
    baseAngVel: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )

    # roll pitch yaw
    baseEulerXyz: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )

    # gravity projected into body frame
    projectedGravity: np.ndarray = field(
        default_factory=lambda: np.zeros(3, dtype=np.float32)
    )


@dataclass
class Command:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


@dataclass
class ControlCfg:
    stiffness: Dict[str, float] = field(default_factory=dict)
    damping: Dict[str, float] = field(default_factory=dict)

    actionScale: float = 0.0

    decimation: int = 0

    user_torque_limit: float = 0.0
    user_power_limit: float = 0.0

    cycle_time: float = 0.0


@dataclass
class ObsScales:
    linVel: float = 0.0
    angVel: float = 0.0

    dofPos: float = 0.0
    dofVel: float = 0.0

    quat: float = 0.0

    heightMeasurements: float = 0.0


@dataclass
class RobotCfg:
    encoder_nomalize: bool = False

    clipActions: float = 0.0
    clipObs: float = 0.0

    obsScales: ObsScales = field(default_factory=ObsScales)

    controlCfg: ControlCfg = field(default_factory=ControlCfg)

    loophz: int = 0

    cycletimeerrorThreshold: float = 0.0

    ThreadPriority: int = 0


jointNames = [
    "leg_l1_joint",
    "leg_r1_joint",
    "waist_1_joint",
    "leg_l2_joint",
    "leg_r2_joint",
    "arm_l1_joint",
    "arm_r1_joint",
    "leg_l3_joint",
    "leg_r3_joint",
    "arm_l2_joint",
    "arm_r2_joint",
    "leg_l4_joint",
    "leg_r4_joint",
    "arm_l3_joint",
    "arm_r3_joint",
    "leg_l5_joint",
    "leg_r5_joint",
    "arm_l4_joint",
    "arm_r4_joint",
    "leg_l6_joint",
    "leg_r6_joint",
]


modelname = ""
count = 0

robotconfig = RobotCfg()

onnx_env = None
policy_session = None

policy_input_names = []
policy_output_names = []

policy_input_shapes = []
policy_output_shapes = []


actuatedDofNum_ = 0

isfirstRecObs_ = True

actionsSize_ = 0
observationSize_ = 0
stackSize_ = 0

scalez = 0.0
scalex = 0.0
scaley = 0.0

actions_ = []
policyObservations_ = []


isfirstCompAct_ = True

remote_data = JoyData()


standPercent = 0.0
standDuration = 0.0


mode_ = None
isChangeMode_ = False
startcontrol = False
initfinish = 0


lieJointAngles_ = np.zeros(21, dtype=np.float32)
standJointAngles_ = np.zeros(21, dtype=np.float32)
currentJointAngles_ = np.zeros(21, dtype=np.float32)
lastActions_ = np.zeros(21, dtype=np.float32)
defaultJointAngles_ = np.zeros(21, dtype=np.float32)
command_ = np.zeros(21, dtype=np.float32)
propri_ = Proprioception()
proprioHistoryBuffer_ = np.zeros(observationSize_ * stackSize_, dtype=np.float32)
found_joint_names = False
found_default_joint_pos = False
found_action_scale = False
found_joint_stiffness = False
found_joint_damping = False
action_scale = []
joint_stiffness = []
joint_damping = []
default_joint_pos = []
joint_names = []
aoliondriver = AoLionDriver()
current_vel_limit_ = 1.5


def square(a):
    return a * a


def get_rotation_matrix_from_zyx_euler_angles(euler_angles):
    z = euler_angles[0]
    y = euler_angles[1]
    x = euler_angles[2]

    c1 = np.cos(z)
    c2 = np.cos(y)
    c3 = np.cos(x)

    s1 = np.sin(z)
    s2 = np.sin(y)
    s3 = np.sin(x)

    s2s3 = s2 * s3
    s2c3 = s2 * c3

    rotation_matrix = np.array(
        [
            [c1 * c2, c1 * s2s3 - s1 * c3, c1 * s2c3 + s1 * s3],
            [s1 * c2, s1 * s2s3 + c1 * c3, s1 * s2c3 - c1 * s3],
            [-s2, c2 * s3, c2 * c3],
        ],
        dtype=np.float32,
    )

    return rotation_matrix


def quat_to_zyx(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    zyx = np.zeros(3, dtype=np.float32)

    a = -2.0 * (x * z - w * y)
    a = min(a, 0.99999)

    zyx[0] = np.arctan2(
        2 * (x * y + w * z), square(w) + square(x) - square(y) - square(z)
    )

    zyx[1] = np.arcsin(a)

    zyx[2] = np.arctan2(
        2 * (y * z + w * x), square(w) - square(x) - square(y) + square(z)
    )

    return zyx


def quat_to_xyz(q):
    x = q[0]
    y = q[1]
    z = q[2]
    w = q[3]

    xyz = np.zeros(3, dtype=np.float32)

    # -----------------------------------
    # Roll (X)
    # -----------------------------------
    sinr_cosp = 2 * (w * x + y * z)

    cosr_cosp = 1 - 2 * (x * x + y * y)

    xyz[0] = np.arctan2(sinr_cosp, cosr_cosp)

    # -----------------------------------
    # Pitch (Y)
    # -----------------------------------
    sinp = 2 * (w * y - z * x)

    if abs(sinp) >= 1:
        xyz[1] = np.copysign(np.pi / 2, sinp)
    else:
        xyz[1] = np.arcsin(sinp)

    # -----------------------------------
    # Yaw (Z)
    # -----------------------------------
    siny_cosp = 2 * (w * z + x * y)

    cosy_cosp = 1 - 2 * (y * y + z * z)

    xyz[2] = np.arctan2(siny_cosp, cosy_cosp)

    return xyz


def split_string(s: str, delimiter: str) -> List[str]:
    tokens = []

    for token in s.split(delimiter):
        token = token.strip()
        if token:
            tokens.append(token)

    return tokens


def split_string_to_double(s: str, delimiter: str) -> List[float]:
    values = []

    for token in s.split(delimiter):
        token = token.strip()
        if token:
            values.append(float(token))

    return values


def init():
    global mode_, actuatedDofNum_
    global standDuration, standPercent
    global lieJointAngles_, standJointAngles_, currentJointAngles_
    global aoliondriver

    path = os.getcwd()

    mode_ = "DEFAULT"

    aoliondriver.init("/dev/input/js0", 115200)

    # DOF
    actuatedDofNum_ = 21

    # 时间参数
    standDuration = 1000
    standPercent = 0.0

    lieJointAngles_ = np.zeros(actuatedDofNum_)
    standJointAngles_ = np.zeros(actuatedDofNum_)
    currentJointAngles_ = np.zeros(actuatedDofNum_)

    standJointAngles_[:] = [
        -0.1495,
        -0.1495,  # leg_l1, leg_r1
        0.0,  # waist_1
        0.0,
        0.0,  # leg_l2, leg_r2
        0.0,
        0.0,  # arm_l1, arm_r1
        0.0,
        0.0,  # leg_l3, leg_r3
        0.2618,
        -0.2618,  # arm_l2, arm_r2
        0.3215,
        0.3215,  # leg_l4, leg_r4
        0.0,
        0.0,  # arm_l3, arm_r3
        -0.1720,
        -0.1720,  # leg_l5, leg_r5
        0.0,
        0.0,  # arm_l4, arm_r4
        0.0,
        0.0,  # leg_l6, leg_r6
    ]

    currentJointAngles_.fill(0.0)


def setparameter(cmd, isfirst):
    global isfirstRecObs_
    global isfirstCompAct_
    global command_

    isfirstRecObs_ = isfirst

    isfirstCompAct_ = isfirstRecObs_

    if command_ is None:
        command_ = np.zeros(3, dtype=np.float32)

    command_[0] = cmd.x
    command_[1] = cmd.y
    command_[2] = cmd.yaw


def updateStateEstimation():
    global propri_
    global ctrl
    global actuatedDofNum_
    global jointNames

    joint_pos = np.zeros(actuatedDofNum_, dtype=np.float32)
    joint_vel = np.zeros(actuatedDofNum_, dtype=np.float32)

    quat = np.zeros(4, dtype=np.float32)
    angular_vel = np.zeros(3, dtype=np.float32)
    linear_acc = np.zeros(3, dtype=np.float32)

    joint_state = ctrl.get_joint_state()

    for i in range(actuatedDofNum_):
        hw_idx = ctrl.getJointsIndex(jointNames[i])

        if 0 <= hw_idx < 21:
            joint_pos[i] = joint_state[hw_idx].pos
            joint_vel[i] = joint_state[hw_idx].vel

    imudata = ctrl.get_imu_data()

    quat[:] = np.array(imudata.ori, dtype=np.float32)

    angular_vel[:] = np.array(imudata.angular_vel, dtype=np.float32)

    linear_acc[:] = np.array(imudata.linear_acc, dtype=np.float32)

    if propri_ is None:
        propri_ = {}

    propri_.jointPos = joint_pos
    propri_.jointVel = joint_vel
    propri_.baseAngVel = angular_vel

    gravity_vector = np.array([0.0, 0.0, -1.0], dtype=np.float32)

    zyx = quat_to_zyx(quat)

    propri_.baseEulerXyz = np.array(
        [
            zyx[2],
            zyx[1],
            zyx[0],
        ],
        dtype=np.float32,
    )

    rot = get_rotation_matrix_from_zyx_euler_angles(zyx)

    inverse_rot = np.linalg.inv(rot)

    projected_gravity = inverse_rot @ gravity_vector

    propri_.projectedGravity = projected_gravity

    return True


def handle_default_mode():
    global ctrl
    global actuatedDofNum_
    global jointNames

    motorcmd = []

    for i in range(21):
        cmd = MotorCmd()
        cmd.pos = 0.0
        cmd.vel = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.1
        cmd.tau = 0.0
        cmd.motor_id = i
        motorcmd.append(cmd)

    for j in range(actuatedDofNum_):
        hw_idx = ctrl.getJointsIndex(jointNames[j])

        if 0 <= hw_idx < 21:
            motorcmd[hw_idx].pos = 0.0
            motorcmd[hw_idx].vel = 0.0
            motorcmd[hw_idx].kp = 0.0
            motorcmd[hw_idx].kd = 0.1
            motorcmd[hw_idx].tau = 0.0
            motorcmd[hw_idx].motor_id = hw_idx

    ctrl.set_joint(motorcmd)


def handle_stand_mode():
    global ctrl
    global standPercent
    global standDuration
    global lieJointAngles_
    global standJointAngles_
    global actuatedDofNum_
    global jointNames

    if standPercent > 1.0:
        return

    motorcmd = []

    for i in range(21):
        cmd = MotorCmd()
        cmd.motor_id = i
        motorcmd.append(cmd)

    for j in range(actuatedDofNum_):
        pos_des = (
            lieJointAngles_[j] * (1.0 - standPercent)
            + standJointAngles_[j] * standPercent
        )

        hw_idx = ctrl.getJointsIndex(jointNames[j])

        if 0 <= hw_idx < 21:
            motorcmd[hw_idx].pos = float(pos_des)
            motorcmd[hw_idx].vel = 0.0
            motorcmd[hw_idx].kp = 10.0
            motorcmd[hw_idx].kd = 0.5
            motorcmd[hw_idx].tau = 0.0
            motorcmd[hw_idx].motor_id = hw_idx

    ctrl.set_joint(motorcmd)

    standPercent += 1.0 / standDuration
    standPercent = min(standPercent, 1.0)


def handle_lie_mode():
    global ctrl
    global standPercent
    global standDuration
    global currentJointAngles_
    global lieJointAngles_
    global actuatedDofNum_
    global jointNames

    if standPercent > 1.0:
        return

    motorcmd = []

    for i in range(21):
        cmd = MotorCmd()
        cmd.motor_id = i
        motorcmd.append(cmd)

    for j in range(actuatedDofNum_):
        pos_des = (
            currentJointAngles_[j] * (1.0 - standPercent)
            + lieJointAngles_[j] * standPercent
        )

        hw_idx = ctrl.getJointsIndex(jointNames[j])

        if 0 <= hw_idx < 21:
            motorcmd[hw_idx].pos = float(pos_des)
            motorcmd[hw_idx].vel = 0.0
            motorcmd[hw_idx].kp = 10.0
            motorcmd[hw_idx].kd = 0.5
            motorcmd[hw_idx].tau = 0.0
            motorcmd[hw_idx].motor_id = hw_idx

    ctrl.set_joint(motorcmd)

    standPercent += 1.0 / standDuration
    standPercent = min(standPercent, 1.0)


def compute_actions():
    global policy_session
    global policyObservations_
    global actions_
    global actionsSize_
    global observationSize_
    global isfirstCompAct_

    if policy_session is None:
        raise RuntimeError("policy_session is None, ONNX model not loaded")

    obs = np.array(policyObservations_, dtype=np.float32).reshape(1, -1)

    if isfirstCompAct_:
        for i, v in enumerate(policyObservations_):
            print(v, end=" ")

            if (i + 1) % observationSize_ == 0:
                print()

        isfirstCompAct_ = False

    outputs = policy_session.run(None, {policy_input_names[0]: obs})

    action_output = outputs[0].reshape(-1)

    if len(actions_) != actionsSize_:
        actions_ = [0.0] * actionsSize_

    for i in range(actionsSize_):
        actions_[i] = float(action_output[i])


def compute_observation():
    global command_
    global scalex, scaley, scalez
    global current_vel_limit_

    global lastActions_
    global propri_

    global observationSize_
    global actionsSize_
    global stackSize_

    global defaultJointAngles_
    global proprioHistoryBuffer_
    global policyObservations_
    global isfirstRecObs_
    global robotconfig

    cmd_x = command_[0] * scalex
    cmd_y = command_[1] * scaley
    cmd_z = command_[2] * scalez

    if cmd_x < -0.5:
        cmd_x = -0.5

    if cmd_x < 0.3 and cmd_x >= 0:
        cmd_x = 0

    if abs(cmd_z) < 0.3:
        cmd_z = 0

    cmd_x = np.clip(cmd_x, -current_vel_limit_, current_vel_limit_)
    cmd_y = np.clip(cmd_y, -1.0, 1.0)
    cmd_z = np.clip(cmd_z, -1.0, 1.0)

    command = np.array([cmd_x, cmd_y, cmd_z], dtype=np.float32)

    actions = np.array(lastActions_, dtype=np.float32).copy()

    base_ang_vel = np.array(propri_.baseAngVel, dtype=np.float32)

    base_euler = np.array(
        [propri_.baseEulerXyz[0], propri_.baseEulerXyz[1]], dtype=np.float32
    )

    joint_pos = np.array(propri_.jointPos, dtype=np.float32)
    joint_vel = np.array(propri_.jointVel, dtype=np.float32)

    joint_pos = joint_pos - defaultJointAngles_

    proprioObs = np.concatenate(
        [
            command,  # 3
            base_ang_vel,  # 3
            base_euler,  # 2
            joint_pos,  # N
            joint_vel,  # N
            actions,  # N
        ]
    ).astype(np.float32)

    if isfirstRecObs_:
        for i in range(stackSize_):
            start = i * observationSize_
            end = start + observationSize_
            proprioHistoryBuffer_[start:end] = proprioObs

        isfirstRecObs_ = False

    proprioHistoryBuffer_[:-observationSize_] = proprioHistoryBuffer_[observationSize_:]
    proprioHistoryBuffer_[-observationSize_:] = proprioObs

    policyObservations_ = proprioHistoryBuffer_.copy()

    obs_min = -robotconfig.clipObs
    obs_max = robotconfig.clipObs

    policyObservations_ = np.clip(policyObservations_, obs_min, obs_max)

    return policyObservations_


def handle_user_mode():
    global count
    global mode_

    global actions_
    global lastActions_

    global action_scale
    global joint_stiffness
    global joint_damping
    global defaultJointAngles_

    global actionsSize_
    global jointNames

    global ctrl
    global robotconfig

    if updateStateEstimation() is False:
        return False

    if propri_.projectedGravity[2] >= -0.3:
        print("Fall Protection Triggered (UserMode)!")
        mode_ = "DEFAULT"
        return True

    if count % robotconfig.controlCfg.decimation == 0:
        count = 0

        compute_observation()
        compute_actions()

        # action clip
        action_min = -robotconfig.clipActions
        action_max = robotconfig.clipActions

        actions_ = np.clip(np.array(actions_, dtype=np.float32), action_min, action_max)

    motorcmd = []

    for i in range(21):
        cmd = MotorCmd()
        cmd.pos = 0.0
        cmd.vel = 0.0
        cmd.kp = 0.0
        cmd.kd = 0.0
        cmd.tau = 0.0
        cmd.motor_id = i
        motorcmd.append(cmd)

    for i in range(actionsSize_):
        if i >= len(jointNames):
            break

        part_name = jointNames[i]
        hw_idx = ctrl.getJointsIndex(part_name)

        pos_des = actions_[i] * action_scale[i] + defaultJointAngles_[i]

        if 0 <= i < 21:
            motorcmd[i].pos = float(pos_des)
            motorcmd[i].vel = 0.0
            motorcmd[i].kp = float(joint_stiffness[i])
            motorcmd[i].kd = float(joint_damping[i])
            motorcmd[i].tau = 0.0
            motorcmd[i].motor_id = hw_idx

        lastActions_[i] = actions_[i]

    ctrl.set_joint(motorcmd)

    count += 1
    return True


def process():
    global initfinish
    global remote_data
    global startcontrol
    global standPercent
    global mode_
    global currentJointAngles_
    global actuatedDofNum_
    global jointNames
    global isChangeMode_

    if not hasattr(process, "keyflag"):
        process.keyflag = [0] * 14

    keyflag = process.keyflag

    if initfinish == 0:
        return

    # -----------------------------
    # Command
    # -----------------------------
    class Command:
        def __init__(self):
            self.x = 0.0
            self.y = 0.0
            self.yaw = 0.0

    cmd = Command()

    l_jdata = aoliondriver.getremotedata()

    if l_jdata is None:
        return

    remote_data = l_jdata

    cmd.x = float(remote_data.axes[1])
    cmd.y = 0.0
    cmd.yaw = float(remote_data.axes[0])

    if remote_data.button[9] == 1 and keyflag[9] == 0:
        if not startcontrol:
            startcontrol = True
            standPercent = 0.0
            mode_ = "LIE"
            keyflag[9] = 1

            joint_state = ctrl.get_joint_state()

            for i in range(actuatedDofNum_):
                index = ctrl.getJointsIndex(jointNames[i])
                currentJointAngles_[i] = joint_state[index].pos

            print("start control")

        else:
            startcontrol = False
            mode_ = "DEFAULT"
            keyflag[9] = 1
            print("stop control")

    elif remote_data.button[9] == 0:
        keyflag[9] = 0

    if remote_data.button[10] == 1 and remote_data.button[2] == 1 and keyflag[10] == 0:
        if startcontrol is True:
            if mode_ != "STAND":
                standPercent = 0.0
                mode_ = "STAND"

                joint_state = ctrl.get_joint_state()

                for i in range(actuatedDofNum_):
                    index = ctrl.getJointsIndex(jointNames[i])
                    currentJointAngles_[i] = joint_state[index].pos

                print("STAND2LIE")

            elif mode_ == "LIE":
                standPercent = 0.0
                mode_ = "STAND"
                print("LIE2STAND")

    elif remote_data.button[10] == 0:
        keyflag[10] = 0

    if remote_data.button[5] == 1 and remote_data.button[2] == 1 and keyflag[5] == 0:
        if mode_ == "STAND":
            standPercent = 0.0
            isChangeMode_ = True
            mode_ = "USERMODE"
            keyflag[5] = 1

            print("TO USERWALK MODE")

    elif remote_data.button[5] == 0:
        keyflag[5] = 0

    if remote_data.button[11] == 1 and keyflag[11] == 0:
        if mode_ == "USERMODE":
            isChangeMode_ = True
            mode_ = "STAND"
            print("WALK2STAND")

        elif mode_ == "DEFAULT":
            standPercent = 0.0
            print("deftolie")
            isChangeMode_ = True
            mode_ = "LIE"

    if mode_ == "STAND":
        handle_stand_mode()

    elif mode_ == "LIE":
        handle_lie_mode()

    elif mode_ == "DEFAULT":
        handle_default_mode()

    elif mode_ == "USERMODE":
        setparameter(cmd, isChangeMode_)
        isChangeMode_ = False
        handle_user_mode()

    else:
        print(f"Unexpected mode encountered: {mode_}")


def onnx_data_init():
    global policy_session

    global policy_input_names
    global policy_output_names

    global policy_input_shapes
    global policy_output_shapes

    if policy_session is None:
        raise RuntimeError("policy_session is None, please load onnx model first")

    policy_input_names = []
    policy_input_shapes = []

    for input_meta in policy_session.get_inputs():
        name = input_meta.name
        shape = input_meta.shape

        policy_input_names.append(name)
        policy_input_shapes.append(shape)

    policy_output_names = []
    policy_output_shapes = []

    for output_meta in policy_session.get_outputs():
        name = output_meta.name
        shape = output_meta.shape

        policy_output_names.append(name)
        policy_output_shapes.append(shape)


def get_model_param():
    global modelname
    global robotconfig

    global standDuration
    global standPercent

    global actionsSize_
    global observationSize_
    global stackSize_

    global scalez
    global scaley
    global scalex

    global actions_
    global actuatedDofNum_

    global policyObservations_
    global lastActions_
    global proprioHistoryBuffer_
    global defaultJointAngles_

    global default_joint_pos

    conpath = os.getcwd()

    yaml_path = os.path.join(conpath, "config", "bumi_ac.yaml")

    with open(yaml_path, "r", encoding="utf-8") as f:
        acconfig = yaml.safe_load(f)

    cfg = acconfig[modelname]

    standDuration = 1000
    standPercent = 0.0

    robotconfig.controlCfg.decimation = int(cfg["control"]["decimation"])

    robotconfig.controlCfg.cycle_time = float(cfg["control"]["cycle_time"])

    robotconfig.clipObs = float(
        cfg["normalization"]["clip_scales"]["clip_observations"]
    )

    robotconfig.clipActions = float(cfg["normalization"]["clip_scales"]["clip_actions"])

    actionsSize_ = int(cfg["size"]["actions_size"])
    observationSize_ = int(cfg["size"]["observations_size"])
    stackSize_ = int(cfg["size"]["stack_size"])

    scalez = float(cfg["axis_mappings"]["scalez"])
    scaley = float(cfg["axis_mappings"]["scaley"])
    scalex = float(cfg["axis_mappings"]["scalex"])

    actions_ = np.zeros(actionsSize_, dtype=np.float32)

    actuatedDofNum_ = 21

    total_obs_size = observationSize_ * stackSize_

    policyObservations_ = np.zeros(total_obs_size, dtype=np.float32)

    lastActions_ = np.zeros(actionsSize_, dtype=np.float32)

    proprioHistoryBuffer_ = np.zeros(total_obs_size, dtype=np.float32)

    defaultJointAngles_ = np.zeros(actionsSize_, dtype=np.float32)

    for i in range(actionsSize_):
        defaultJointAngles_[i] = float(default_joint_pos[i])

    return True


def load_model(modelpath: str):
    global onnx_env
    global policy_session

    global policy_input_names
    global policy_output_names
    global policy_input_shapes
    global policy_output_shapes

    global modelname
    global command_

    global isfirstCompAct_
    global isfirstRecObs_
    global count

    global found_joint_names
    global found_default_joint_pos
    global found_action_scale
    global found_joint_stiffness
    global found_joint_damping

    global joint_names
    global joint_stiffness
    global joint_damping
    global default_joint_pos
    global action_scale

    global initfinish

    print(f"Load Onnx model from path: {modelpath}")

    session_options = ort.SessionOptions()
    session_options.inter_op_num_threads = 1

    try:
        policy_session = ort.InferenceSession(
            modelpath, sess_options=session_options, providers=["CPUExecutionProvider"]
        )
    except Exception as e:
        print("load run model failed")
        print(e)
        return False

    if policy_session is None:
        print("policy_session is None")
        return False

    policy_input_names = []
    policy_output_names = []
    policy_input_shapes = []
    policy_output_shapes = []

    modelname = "run"

    command_ = np.zeros(3, dtype=np.float32)

    isfirstCompAct_ = True

    isfirstRecObs_ = True

    count = 0

    meta = policy_session.get_modelmeta()

    custom_metadata = meta.custom_metadata_map

    for key, value in custom_metadata.items():
        if key == "joint_names":
            found_joint_names = True

            joint_names = split_string(value, ",")

        elif key == "joint_stiffness":
            found_joint_stiffness = True

            joint_stiffness = split_string_to_double(value, ",")

        elif key == "joint_damping":
            found_joint_damping = True

            joint_damping = split_string_to_double(value, ",")

        elif key == "default_joint_pos":
            found_default_joint_pos = True

            default_joint_pos = split_string_to_double(value, ",")

        elif key == "action_scale":
            found_action_scale = True

            action_scale = split_string_to_double(value, ",")

    onnx_data_init()

    ret = get_model_param()

    if not ret:
        print("get_model_param failed")
        return False

    initfinish = 1

    print("Load Onnx run model successfully !!!")

    return True


def main():
    global ctrl

    path = os.getcwd()

    os.environ["CYCLONEDDS_URI"] = "file://config/dds.xml"
    ctrl = LowController.instance()
    ctrl.init()

    ddsxml = "file://" + path + "/config/dds.xml"
    os.environ["CYCLONEDDS_URI"] = ddsxml
    print(f"cur path is {ddsxml}")

    load_model(path + "/policy/policy.onnx")

    init()

    while True:
        start_time = time.perf_counter()

        process()

        end_time = time.perf_counter()

        elapsed = end_time - start_time

        # C++:
        # sleep_for(2000us - elapsed)
        target = 0.002  # 2ms

        sleep_time = target - elapsed

        if sleep_time > 0:
            time.sleep(sleep_time)


if __name__ == "__main__":
    main()
