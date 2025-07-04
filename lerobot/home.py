
"""
Simple script to control a robot from teleoperation.

Example:

```shell
python -m lerobot.readout \
    --robot.type=so101_follower \
    --robot.port=/dev/tty.usbmodem58760431541 \
    --robot.cameras="{ front: {type: opencv, index_or_path: 0, width: 1920, height: 1080, fps: 30}}" \
    --robot.id=black \
```
"""

import logging
import time
from dataclasses import asdict, dataclass
from pprint import pformat

import draccus
import numpy as np
import rerun as rr

from lerobot.common.cameras.opencv.configuration_opencv import OpenCVCameraConfig  # noqa: F401
from lerobot.common.cameras.realsense.configuration_realsense import RealSenseCameraConfig  # noqa: F401
from lerobot.common.robots import (  # noqa: F401
    Robot,
    RobotConfig,
    koch_follower,
    make_robot_from_config,
    so100_follower,
    so101_follower,
)
from lerobot.common.teleoperators import (
    Teleoperator,
    TeleoperatorConfig,
    make_teleoperator_from_config,
)
from lerobot.common.utils.robot_utils import busy_wait
from lerobot.common.utils.utils import init_logging, move_cursor_up
from lerobot.common.utils.visualization_utils import _init_rerun

from .common.teleoperators import gamepad, koch_leader, so100_leader, so101_leader  # noqa: F401

from lerobot.common.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)


@dataclass
class ReadoutConfig:
    robot: RobotConfig
    # Limit the maximum frames per second.
    fps: int = 60
    # Display all cameras on screen
    display_data: bool = True


def home_once(
    robot: Robot, fps: int):

    print(robot)
    print(robot.bus)
    print(robot.action_features)
    # robot.bus.torque_enabled()
    # robot.bus.configure_motors()
    # for motor in robot.bus.motors:
    #     robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
    #     robot.bus.write("P_Coefficient", motor, 16)
    #     print (f"Motor: {motor}")

    print ("Talk enabled")


# FeetechMotorsBus(
#     Port: '/dev/tty.usbmodem5A680116271',
#     Motors: 
# {       'shoulder_pan': Motor(id=1,
#                               model='sts3215',
#                               norm_mode=<MotorNormMode.RANGE_M100_100: 'range_m100_100'>),
#         'shoulder_lift': Motor(id=2,
#                                model='sts3215',
#                                norm_mode=<MotorNormMode.RANGE_M100_100: 'range_m100_100'>),
#         'elbow_flex': Motor(id=3,
#                             model='sts3215',
#                             norm_mode=<MotorNormMode.RANGE_M100_100: 'range_m100_100'>),
#         'wrist_flex': Motor(id=4,
#                             model='sts3215',
#                             norm_mode=<MotorNormMode.RANGE_M100_100: 'range_m100_100'>),
#         'wrist_roll': Motor(id=5,
#                             model='sts3215',
#                             norm_mode=<MotorNormMode.RANGE_M100_100: 'range_m100_100'>),
#         'gripper': Motor(id=6,
#                          model='sts3215',
#                          norm_mode=<MotorNormMode.RANGE_0_100: 'range_0_100'>)},
# )',

# {shoulder_pan_pos}, {shoulder_lift_pos}, {elbow_flex_pos}, {wrist_flex_pos}, {wrist_roll_pos}, {gripper_pos}")
# -4.6786182772190585, -98.64003399915002, 99.81793354574421, 100.0, 5.006105006105017, 1.4208389715832206


    action = {}

    action["shoulder_pan.pos"] = -5
    action["shoulder_lift.pos"] = -95
    action["elbow_flex.pos"] = 95
    action["wrist_flex.pos"] = 95
    action["wrist_roll.pos"] = 5
    action["gripper.pos"] = 5
    
    ret = robot.send_action(action)

    busy_wait(1)

    print ("Done")



@draccus.wrap()
def home(cfg: ReadoutConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)

    robot.connect()

    try:
        home_once(robot, cfg.fps)
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        robot.disconnect()


if __name__ == "__main__":
    home()
