#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
from collections import namedtuple
from easyhid import Enumeration
import time
import datetime

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

# SpaceMouse Compact spec
spacemouse_compact_spec = DeviceSpec(
    name="SpaceNavigator",
    hid_id=[0x46D, 0xC626],  # VID, PID
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


def main():
    dev = open_device()
    print("SpaceMouse Compact opened. Move device to see values (Ctrl+C to exit).")

    try:
        while True:
            state = dev.read()
            if state:
                data = {
                        'x': float("%.2f" % getattr(state, 'x')),
                        'y': float("%.2f" % getattr(state, 'y')),
                        'z': float("%.2f" % getattr(state, 'z')),
                        'roll': float("%.2f" % getattr(state, 'roll')),
                        'pitch': float("%.2f" % getattr(state, 'pitch')),
                        'yaw': float("%.2f" % getattr(state, 'yaw')),
                        'button0': int(state.buttons[0]),
                        'button1': int(state.buttons[1]),
                        # 'timeStamp': timeStamp.timestamp()  # Convert datetime to Unix timestamp
                    }
                print(data)
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        dev.close()

if __name__ == "__main__":
    main()
