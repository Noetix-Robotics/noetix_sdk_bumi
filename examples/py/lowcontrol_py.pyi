# lowcontrol_py.pyi

from typing import Any, Callable, List
import numpy as np

class RobotBmsData:
    battery_temp: int
    battery_alarm: int
    battery_soc: int
    battery_soh: int

class MotorCmd:
    pos: float
    vel: float
    tau: float
    kp: float
    kd: float
    motor_id: int

    def __init__(self) -> None: ...

class JoyData:
    def __init__(self) -> None: ...
    @property
    def axes(self) -> np.ndarray: ...
    @property
    def button(self) -> np.ndarray: ...

class NingImuData:
    def __init__(self) -> None: ...
    @property
    def ori(self) -> np.ndarray: ...
    @property
    def ori_cov(self) -> np.ndarray: ...
    @property
    def angular_vel(self) -> np.ndarray: ...
    @property
    def angular_vel_cov(self) -> np.ndarray: ...
    @property
    def linear_acc(self) -> np.ndarray: ...
    @property
    def linear_acc_cov(self) -> np.ndarray: ...

class MotorState:
    def __init__(self) -> None: ...
    pos: float
    vel: float
    tau: float
    motor_id: int
    error: int
    temperature: float

class RobotHardwareStatus:
    def __init__(self) -> None: ...
    imu_data: NingImuData
    remote_data: JoyData
    motor_data: List[MotorState]
    bms_data: RobotBmsData
    workmode: int

RobotHardwareStatusCallback = Callable[[RobotHardwareStatus], None]

class LowController:
    @staticmethod
    def instance() -> "LowController": ...
    def init(self) -> bool: ...
    def set_joint(self, motorcmd: List[MotorCmd]) -> None: ...
    def getJointsIndex(self, jointname: str) -> int: ...
    def subscribe_robot_hardware_status(
        self, callback: RobotHardwareStatusCallback
    ) -> None: ...

class AoLionDriver:
    def __init__(self) -> None: ...
    def init(self, port: str, baudrate: int) -> bool: ...
    def getremotedata(self) -> JoyData: ...
