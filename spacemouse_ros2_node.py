#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from collections import namedtuple
from easyhid import Enumeration

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

# Define named tuples
AxisSpec = namedtuple("AxisSpec", ["channel", "byte1", "byte2", "scale"])
ButtonSpec = namedtuple("ButtonSpec", ["channel", "byte", "bit"])
SpaceNavigator = namedtuple("SpaceNavigator", ["t", "x", "y", "z", "roll", "pitch", "yaw", "buttons"])

class ButtonState(list):
    def __int__(self):
        return sum((b << i) for (i, b) in enumerate(reversed(self)))

def to_int16(y1, y2):
    x = (y1) | (y2 << 8)
    if x >= 32768:
        x = -(65536 - x)
    return x

class DeviceSpec(object):
    def __init__(self, name, hid_id, led_id, mappings, button_mapping, axis_scale=350.0):
        self.name = name
        self.hid_id = hid_id
        self.led_id = led_id
        self.mappings = mappings
        self.button_mapping = button_mapping
        self.axis_scale = axis_scale
        self.device = None
        self.dict_state = {
            "t": -1, "x": 0, "y": 0, "z": 0,
            "roll": 0, "pitch": 0, "yaw": 0,
            "buttons": ButtonState([0] * len(self.button_mapping))
        }
        self.tuple_state = SpaceNavigator(**self.dict_state)

    def open(self):
        if self.device:
            self.device.open()

    def close(self):
        if self.device:
            self.device.close()

    def read(self):
        if not self.device:
            return None
        data = self.device.read(13)
        if data:
            self.process(data)
        return self.tuple_state

    def process(self, data):
        for name, (chan, b1, b2, flip) in self.mappings.items():
            if data[0] == chan:
                self.dict_state[name] = flip * to_int16(data[b1], data[b2]) / float(self.axis_scale)

        for button_index, (chan, byte, bit) in enumerate(self.button_mapping):
            if data[0] == chan:
                mask = 1 << bit
                self.dict_state["buttons"][button_index] = 1 if (data[byte] & mask) != 0 else 0

        self.dict_state["t"] = time.time()
        self.tuple_state = SpaceNavigator(**self.dict_state)

# SpaceMouse Compact spec (correct VID/PID)
spacemouse_compact_spec = DeviceSpec(
    name="SpaceNavigator",
    hid_id=[0x256f, 0xc635],  # SpaceMouse Compact
    led_id=[0x8, 0x4B],
    mappings={
        "x": AxisSpec(channel=1, byte1=1, byte2=2, scale=1),
        "y": AxisSpec(channel=1, byte1=3, byte2=4, scale=-1),
        "z": AxisSpec(channel=1, byte1=5, byte2=6, scale=-1),
        "pitch": AxisSpec(channel=2, byte1=1, byte2=2, scale=-1),
        "roll": AxisSpec(channel=2, byte1=3, byte2=4, scale=-1),
        "yaw": AxisSpec(channel=2, byte1=5, byte2=6, scale=1),
    },
    button_mapping=[
        ButtonSpec(channel=3, byte=1, bit=0),
        ButtonSpec(channel=3, byte=1, bit=1),
    ],
    axis_scale=350.0,
)

def open_device():
    hid = Enumeration()
    all_devices = hid.find()
    for device in all_devices:
        if "SpaceMouse" in (device.product_string or ""):
            print(f"Found {device.product_string} (VID: {hex(device.vendor_id)}, PID: {hex(device.product_id)})")
            spacemouse_compact_spec.device = device
            spacemouse_compact_spec.open()
            return spacemouse_compact_spec
    raise Exception("SpaceMouse not found")

class SpaceMouseNode(Node):
    def __init__(self):
        super().__init__('spacemouse_node')
        self.dev = open_device()
        self.get_logger().info("SpaceMouse Compact opened (ROS 2 publisher active).")

        self.twist_pub = self.create_publisher(Twist, 'spacemouse/twist', 10)
        self.button_pub = self.create_publisher(Int32, 'spacemouse/buttons', 10)

        self.timer = self.create_timer(0.01, self.read_and_publish)  # 100 Hz

    def read_and_publish(self):
        state = self.dev.read()
        if not state:
            return

        # Publish Twist
        twist = Twist()
        twist.linear.x = float(state.x)
        twist.linear.y = float(state.y)
        twist.linear.z = float(state.z)
        twist.angular.x = float(state.roll)
        twist.angular.y = float(state.pitch)
        twist.angular.z = float(state.yaw)
        self.twist_pub.publish(twist)

        # Publish Buttons
        buttons = Int32()
        buttons.data = (int(state.buttons[0]) << 0) | (int(state.buttons[1]) << 1)
        self.button_pub.publish(buttons)

def main(args=None):
    rclpy.init(args=args)
    node = SpaceMouseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
