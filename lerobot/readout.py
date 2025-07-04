
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


def readout_loop(
    robot: Robot, fps: int):

    print(robot)
    print(robot.bus)
    with robot.bus.torque_disabled():
        robot.bus.configure_motors()
        for motor in robot.bus.motors:
            robot.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            robot.bus.write("P_Coefficient", motor, 16)

        print ("Torque disabled")

        start = time.perf_counter()
        while True:
            # robot.bus.torque_disabled();

            loop_start = time.perf_counter()

            observation = robot.get_observation()

            # 'shoulder_pan.pos': -26.716222125054657, 'shoulder_lift.pos': -25.626859328516787, 'elbow_flex.pos': 66.95493855257169, 'wrist_flex.pos': 76.02427921092564, 'wrist_roll.pos': 0.024420024420024333, 'gripper.pos': 27.46955345060893,
            shoulder_pan_pos = observation['shoulder_pan.pos']
            shoulder_lift_pos = observation['shoulder_lift.pos']
            elbow_flex_pos = observation['elbow_flex.pos']
            wrist_flex_pos = observation['wrist_flex.pos']
            wrist_roll_pos = observation['wrist_roll.pos']
            gripper_pos = observation['gripper.pos']


            dt_s = time.perf_counter() - loop_start
            busy_wait(1 / fps - dt_s)

            loop_s = time.perf_counter() - loop_start

            print(f"{loop_s * 1e3:.2f} ms: {shoulder_pan_pos}, {shoulder_lift_pos}, {elbow_flex_pos}, {wrist_flex_pos}, {wrist_roll_pos}, {gripper_pos}")


@draccus.wrap()
def readout(cfg: ReadoutConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    if cfg.display_data:
        _init_rerun(session_name="teleoperation")

    robot = make_robot_from_config(cfg.robot)

    robot.connect()

    try:
        readout_loop(robot, cfg.fps)
    except KeyboardInterrupt:
        pass
    finally:
        if cfg.display_data:
            rr.rerun_shutdown()
        robot.disconnect()


if __name__ == "__main__":
    readout()
