#!/usr/bin/env python3
import rclpy
import rclpy.time
from std_msgs.msg import Byte
from itertools import cycle
from enum import IntEnum
import time

class trafficlight():
    class Color(IntEnum):
        RED     = 0
        YELLOW  = 1
        GREEN   = 2

    def mirrorLight(self, number):
        if number == 0:
            return 2
        if number == 2:
            return 0
        return number

    def __init__(self):
        self.TL_interval = 1.0  # seconds
        rclpy.init()
        self.node = rclpy.create_node('traffic_light_publisher_node')
        self.trafficlights = []
        tlma = self.node.create_publisher(Byte, '/automobile/trafficlight/master', 1)
        tlsl = self.node.create_publisher(Byte, '/automobile/trafficlight/slave', 1)
        tlam = self.node.create_publisher(Byte, '/automobile/trafficlight/antimaster', 1)
        tlst = self.node.create_publisher(Byte, '/automobile/trafficlight/start', 1)
        self.trafficlights.append(tlma)
        self.trafficlights.append(tlsl)
        self.trafficlights.append(tlam)
        self.trafficlights.append(tlst)
        print("[tl_talker] Traffic light publisher started")

    def sendState(self, id, state):
        self.trafficlights[id].publish(Byte(data=bytes([int(state)])))

    def run(self):
        self.pattern = [
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,
            self.Color.RED,
            self.Color.YELLOW,
            self.Color.YELLOW,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.GREEN,
            self.Color.YELLOW,
            self.Color.YELLOW
        ]
        self.maincycle = cycle(self.pattern)
        self.main_state = self.Color.RED
        last_time = time.time()

        print("[tl_talker] Running...")
        while rclpy.ok():
            current_time = time.time()
            if current_time - last_time >= self.TL_interval:
                self.main_state = next(self.maincycle)
                last_time = current_time
                print(f"[tl_talker] State: {self.main_state.name}")

            self.sendState(0, self.main_state)
            self.sendState(1, self.main_state)
            self.sendState(2, self.mirrorLight(self.main_state))
            self.sendState(3, self.mirrorLight(self.main_state))

            rclpy.spin_once(self.node, timeout_sec=0.0)
            time.sleep(0.1)  # 10Hz

def main():
    nod = trafficlight()
    nod.run()

if __name__ == "__main__":
    nod = trafficlight()
    nod.run()
