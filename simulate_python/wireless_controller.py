from enum import Enum, unique
from unitree_sdk2py.core.channel import ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_


@unique
class KeyIdx(Enum):
    R1 = 0
    L1 = 1
    start = 2
    select = 3
    R2 = 4
    L2 = 5
    F1 = 6
    F2 = 7
    A = 8
    B = 9
    X = 10
    Y = 11
    up = 12
    right = 13
    down = 14
    left = 15


class Key:
    def __init__(self):
        self.pressed = False
        self.on_press = False
        self.on_release = False

    def update_key(self, state: bool):
        self.on_press = (self.pressed == False) if state else False
        self.on_release = False if state else (self.pressed == True)
        self.pressed = state


class WirelessController:
    def __init__(self, queue_length: int = 1):
        self.subscriber = ChannelSubscriber(
            "rt/wirelesscontroller", WirelessController_
        )
        self.subscriber.Init(self.callback, queue_length)

        # keys
        self.keys = [Key() for _ in range(len(KeyIdx))]

        # joystick (x, y)
        self.left_joystick = [0.0, 0.0]
        self.right_joystick = [0.0, 0.0]
        
        self.left_joystick_target = [0.0, 0.0]
        self.right_joystick_target = [0.0, 0.0]
        
        self.smooth_factor = 0.03
        self.dead_zone = 0.01

    def lx(self):
        return self.left_joystick[0]

    def ly(self):
        return self.left_joystick[1]

    def rx(self):
        return self.right_joystick[0]

    def ry(self):
        return self.right_joystick[1]

    def key_pressed(self, key_enum: KeyIdx):
        return self.keys[key_enum.value].pressed

    def key_down(self, key_enum: KeyIdx):
        return self.keys[key_enum.value].on_press

    def key_up(self, key_enum: KeyIdx):
        return self.keys[key_enum.value].on_release

    def update(self):
        self.left_joystick[0] = (
            self.left_joystick[0] * (1 - self.smooth_factor)
            + self.left_joystick_target[0] * self.smooth_factor
        )
        
        self.left_joystick[1] = (
            self.left_joystick[1] * (1 - self.smooth_factor)
            + self.left_joystick_target[1] * self.smooth_factor
        )

        self.right_joystick[0] = (
            self.right_joystick[0] * (1 - self.smooth_factor)
            + self.right_joystick_target[0] * self.smooth_factor
        )
        self.right_joystick[1] = (
            self.right_joystick[1] * (1 - self.smooth_factor)
            + self.right_joystick_target[1] * self.smooth_factor
        )
    
    def callback(self, msg: WirelessController_):
        self.left_joystick[0] = 0.0 if abs(msg.lx) < self.dead_zone else msg.lx
        self.left_joystick[1] = 0.0 if abs(msg.ly) < self.dead_zone else msg.ly
        self.right_joystick[0] = 0.0 if abs(msg.rx) < self.dead_zone else msg.rx
        self.right_joystick[1] = 0.0 if abs(msg.ry) < self.dead_zone else msg.ry
        
        # self.left_joystick_target[0] = 0.0 if abs(msg.lx) < self.dead_zone else msg.lx
        # self.left_joystick_target[1] = 0.0 if abs(msg.ly) < self.dead_zone else msg.ly
        # self.right_joystick_target[0] = 0.0 if abs(msg.rx) < self.dead_zone else msg.rx
        # self.right_joystick_target[1] = 0.0 if abs(msg.ry) < self.dead_zone else msg.ry
        
        for i in range(len(KeyIdx)):
            self.keys[i].update_key(((msg.keys & (1 << i)) > 0))
        pass
