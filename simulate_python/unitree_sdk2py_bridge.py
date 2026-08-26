import mujoco
import numpy as np
import pygame
import sys
import struct
import time
from contextlib import nullcontext

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher

from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread

from unitree_go_image import Image_, unitree_go_msg_dds__Image_
from camera_randomizer import DepthRandomizer

import config

if config.ROBOT == "g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import (
        unitree_hg_msg_dds__LowState_ as LowState_default,
    )
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import (
        unitree_go_msg_dds__LowState_ as LowState_default,
    )

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"
TOPIC_IMAGE_COLOR = "/color/image_raw"
TOPIC_IMAGE_DEPTH = "/depth/image_raw"

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35
FOOT_ORDER = ("FR", "FL", "RR", "RL")


class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data, data_lock=None):
        self.mj_model = mj_model
        self.mj_data = mj_data
        # The simulation loop may update mjData from another thread.  Contact
        # arrays are resized during mj_step, so their traversal must share its
        # lock to avoid reading a stale ncon followed by contact[index].
        self._data_lock = data_lock

        self.num_motor = self.mj_model.nu
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.idl_type = self.num_motor > NUM_MOTOR_IDL_GO  # 0: unitree_go, 1: unitree_hg

        self.joystick = None

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        # The quadruped XML models name their collision geoms FR, FL, RR and RL.
        # Keep the IDs here rather than adding force sensors to sensordata: contact
        # forces are then independent of the number and ordering of other sensors.
        self.foot_geom_ids = {
            name: mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in FOOT_ORDER
        }
        self.foot_geom_to_index = {
            geom_id: index for index, geom_id in enumerate(self.foot_geom_ids.values()) if geom_id >= 0
        }
        self.have_foot_force_ = hasattr(LowState_default(), "foot_force") and all(
            geom_id >= 0 for geom_id in self.foot_geom_ids.values()
        )

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(interval=self.dt, target=self.PublishLowState, name="sim_lowstate")
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(interval=self.dt, target=self.PublishHighState, name="sim_highstate")
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(TOPIC_WIRELESS_CONTROLLER, WirelessController_)
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }

        if config.CAMERA_ENABLED:
            self._camera_setup()

    def _camera_setup(self):
        camera_id = mujoco.mj_name2id(self.mj_model, mujoco.mjtObj.mjOBJ_CAMERA, config.CAMERA_NAME)
        if camera_id < 0:
            print("Camera '{}' not found in model, disabling camera.".format(config.CAMERA_NAME))
            return

        self._cam_name = config.CAMERA_NAME
        self._cam_resolution = self.mj_model.cam_resolution[camera_id]
        self._cam_w = self._cam_resolution[0]
        self._cam_h = self._cam_resolution[1]

        self._cam_renderer = None

        self._depth_randomizer = DepthRandomizer(
            gaussian_std=config.DEPTH_GAUSSIAN_NOISE_STD,
            dropout_rate=config.DEPTH_DROPOUT_RATE,
            quantization=config.DEPTH_QUANTIZATION,
            edge_distortion=config.DEPTH_EDGE_DISTORTION,
            edge_shift_std=config.DEPTH_EDGE_SHIFT_STD,
            edge_kernel_size=config.DEPTH_EDGE_KERNEL_SIZE,
            systematic_missing_rate=config.DEPTH_SYSTEMATIC_MISSING_RATE,
            missing_patch_size=config.DEPTH_MISSING_PATCH_SIZE,
        )

        self._color_puber = ChannelPublisher(TOPIC_IMAGE_COLOR, Image_)
        self._color_puber.Init()
        self._depth_puber = ChannelPublisher(TOPIC_IMAGE_DEPTH, Image_)
        self._depth_puber.Init()

        interval = 1.0 / config.CAMERA_FPS
        self._camera_thread = RecurrentThread(
            interval=interval,
            target=self.PublishCameraImages,
            name="sim_camera",
        )
        self._camera_thread.Start()

    def PublishCameraImages(self):
        if self._cam_renderer is None:
            self._cam_renderer = mujoco.Renderer(self.mj_model, height=self._cam_h, width=self._cam_w)

        self._cam_renderer.update_scene(self.mj_data, camera=self._cam_name)

        color_img = self._cam_renderer.render()
        color_msg = unitree_go_msg_dds__Image_()
        color_msg.timestamp = int(time.time() * 1e6)
        color_msg.width = self._cam_w
        color_msg.height = self._cam_h
        color_msg.step = self._cam_w * 3
        color_msg.encoding = "bgr8"
        color_msg.data = color_img.copy().tobytes()
        self._color_puber.Write(color_msg)

        self._cam_renderer.enable_depth_rendering()
        depth = self._cam_renderer.render()
        self._cam_renderer.disable_depth_rendering()

        depth[depth > config.DEPTH_CLIP_FAR] = 0.0
        depth[depth < config.DEPTH_CLIP_NEAR] = 0.0

        depth = self._depth_randomizer.randomize(depth)

        depth_mm = (depth * 1000.0).clip(0, 65535).astype(np.uint16)
        depth_msg = unitree_go_msg_dds__Image_()
        depth_msg.timestamp = int(time.time() * 1e6)
        depth_msg.width = self._cam_w
        depth_msg.height = self._cam_h
        depth_msg.step = self._cam_w * 2
        depth_msg.encoding = "mono16"
        depth_msg.data = depth_mm.tobytes()
        self._depth_puber.Write(depth_msg)

    def Stop(self):
        self.lowStateThread.Wait()
        self.HighStateThread.Wait()
        self.WirelessControllerThread.Wait()
        if hasattr(self, "_camera_thread"):
            self._camera_thread.Wait()
        if hasattr(self, "_cam_renderer") and self._cam_renderer is not None:
            self._cam_renderer.close()

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.mj_data.ctrl[i] = (
                    msg.motor_cmd[i].tau
                    + msg.motor_cmd[i].kp * (msg.motor_cmd[i].q - self.mj_data.sensordata[i])
                    + msg.motor_cmd[i].kd * (msg.motor_cmd[i].dq - self.mj_data.sensordata[i + self.num_motor])
                )

    def PublishLowState(self):
        if self.mj_data != None:
            for i in range(self.num_motor):
                self.low_state.motor_state[i].q = self.mj_data.sensordata[i]
                self.low_state.motor_state[i].dq = self.mj_data.sensordata[i + self.num_motor]
                self.low_state.motor_state[i].tau_est = self.mj_data.sensordata[i + 2 * self.num_motor]

            self._UpdateFootForce()

            if self.have_frame_sensor_:

                self.low_state.imu_state.quaternion[0] = self.mj_data.sensordata[self.dim_motor_sensor + 0]
                self.low_state.imu_state.quaternion[1] = self.mj_data.sensordata[self.dim_motor_sensor + 1]
                self.low_state.imu_state.quaternion[2] = self.mj_data.sensordata[self.dim_motor_sensor + 2]
                self.low_state.imu_state.quaternion[3] = self.mj_data.sensordata[self.dim_motor_sensor + 3]

                self.low_state.imu_state.gyroscope[0] = self.mj_data.sensordata[self.dim_motor_sensor + 4]
                self.low_state.imu_state.gyroscope[1] = self.mj_data.sensordata[self.dim_motor_sensor + 5]
                self.low_state.imu_state.gyroscope[2] = self.mj_data.sensordata[self.dim_motor_sensor + 6]

                self.low_state.imu_state.accelerometer[0] = self.mj_data.sensordata[self.dim_motor_sensor + 7]
                self.low_state.imu_state.accelerometer[1] = self.mj_data.sensordata[self.dim_motor_sensor + 8]
                self.low_state.imu_state.accelerometer[2] = self.mj_data.sensordata[self.dim_motor_sensor + 9]

            if self.joystick != None:
                pygame.event.get()
                # Buttons
                self.low_state.wireless_remote[2] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                0,
                                0,
                                int(self.joystick.get_axis(self.axis_id["LT"]) > 0),
                                int(self.joystick.get_axis(self.axis_id["RT"]) > 0),
                                int(self.joystick.get_button(self.button_id["SELECT"])),
                                int(self.joystick.get_button(self.button_id["START"])),
                                int(self.joystick.get_button(self.button_id["LB"])),
                                int(self.joystick.get_button(self.button_id["RB"])),
                            ]
                        ]
                    ),
                    2,
                )
                self.low_state.wireless_remote[3] = int(
                    "".join(
                        [
                            f"{key}"
                            for key in [
                                int(self.joystick.get_hat(0)[0] < 0),  # left
                                int(self.joystick.get_hat(0)[1] < 0),  # down
                                int(self.joystick.get_hat(0)[0] > 0),  # right
                                int(self.joystick.get_hat(0)[1] > 0),  # up
                                int(self.joystick.get_button(self.button_id["Y"])),  # Y
                                int(self.joystick.get_button(self.button_id["X"])),  # X
                                int(self.joystick.get_button(self.button_id["B"])),  # B
                                int(self.joystick.get_button(self.button_id["A"])),  # A
                            ]
                        ]
                    ),
                    2,
                )
                # Axes
                sticks = [
                    self.joystick.get_axis(self.axis_id["LX"]),
                    self.joystick.get_axis(self.axis_id["RX"]),
                    -self.joystick.get_axis(self.axis_id["RY"]),
                    -self.joystick.get_axis(self.axis_id["LY"]),
                ]
                packs = list(map(lambda x: struct.pack("f", x), sticks))
                self.low_state.wireless_remote[4:8] = packs[0]
                self.low_state.wireless_remote[8:12] = packs[1]
                self.low_state.wireless_remote[12:16] = packs[2]
                self.low_state.wireless_remote[20:24] = packs[3]

            self.low_state_puber.Write(self.low_state)

    def _UpdateFootForce(self):
        """Publish per-foot normal contact loads in LowState.foot_force.

        MuJoCo reports the normal component of every active contact in index 0 of
        ``mj_contactForce``.  A foot can have multiple contact points, so their
        normal loads are accumulated.  The DDS field is int16, in newtons.
        """
        if not self.have_foot_force_:
            return

        foot_force = np.zeros(4, dtype=np.float64)
        contact_force = np.zeros(6, dtype=np.float64)

        data_lock = getattr(self, "_data_lock", None)
        with data_lock if data_lock is not None else nullcontext():
            for contact_id in range(self.mj_data.ncon):
                contact = self.mj_data.contact[contact_id]
                foot_index = self.foot_geom_to_index.get(contact.geom1)
                if foot_index is None:
                    foot_index = self.foot_geom_to_index.get(contact.geom2)
                if foot_index is None:
                    continue

                mujoco.mj_contactForce(self.mj_model, self.mj_data, contact_id, contact_force)
                foot_force[foot_index] += contact_force[0]

        for index, force in enumerate(foot_force):
            self.low_state.foot_force[index] = int(np.clip(np.rint(force), -32768, 32767))

    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[self.dim_motor_sensor + 10]
            self.high_state.position[1] = self.mj_data.sensordata[self.dim_motor_sensor + 11]
            self.high_state.position[2] = self.mj_data.sensordata[self.dim_motor_sensor + 12]

            self.high_state.velocity[0] = self.mj_data.sensordata[self.dim_motor_sensor + 13]
            self.high_state.velocity[1] = self.mj_data.sensordata[self.dim_motor_sensor + 14]
            self.high_state.velocity[2] = self.mj_data.sensordata[self.dim_motor_sensor + 15]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(self.button_id["RB"])
            key_state[self.key_map["L1"]] = self.joystick.get_button(self.button_id["LB"])
            key_state[self.key_map["start"]] = self.joystick.get_button(self.button_id["START"])
            key_state[self.key_map["select"]] = self.joystick.get_button(self.button_id["SELECT"])
            key_state[self.key_map["R2"]] = self.joystick.get_axis(self.axis_id["RT"]) > 0
            key_state[self.key_map["L2"]] = self.joystick.get_axis(self.axis_id["LT"]) > 0
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")
            sys.exit()

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i)
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 3])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
