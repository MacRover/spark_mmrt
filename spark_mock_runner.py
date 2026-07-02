#!/usr/bin/env python3
import time
import random
import struct
import os
import sys
from typing import List
import yaml
from can import ThreadSafeBus, Message, Notifier
from threading import Thread, Lock

from enum import Enum

# -------------------------------------------------------------------- 
# -------------------------- Data Classes ----------------------------
# --------------------------------------------------------------------
class ControlType(Enum):
    DUTY_CYCLE = 0, 
    VELOCITY = 1, 
    VOLTAGE = 2, 
    POSITION = 3,
    SMARTMOTION = 4, 
    SMARTVELOCITY = 5, 
    MAXMOTION_POSITION = 6, 
    MAXMOTION_VELOCITY = 7

class Status0:
    applied_output = 0.0
    voltage = 12.0
    current = 0.5
    motor_temperature = 25.0
    
    hard_forward_limit = False
    hard_reverse_limit = False
    soft_forward_limit = False
    soft_reverse_limit = False
    primary_heartbeat_lock = False
    inverted = False

    def __call__(self):
        output = int(self.applied_output * 32442.6)
        voltage = int(self.voltage * 136.5)
        current = int(self.current * 27.3)
        temp = int(self.motor_temperature * 1.0)
        vct = ((voltage & 0xfff) | 
               ((current & 0xfff) << 12) | 
               ((temp & 0xff) << 24))
        flags = (self.hard_forward_limit | 
                 (self.hard_reverse_limit << 1) | 
                 (self.soft_forward_limit << 2) | 
                 (self.soft_reverse_limit << 3) |
                 (self.inverted << 4) | 
                 (self.primary_heartbeat_lock  << 5))
        return struct.pack("<hIH", output, vct, flags)

class Status1:
    raw_value = 0
    def __call__(self):
        return struct.pack("<Q", self.raw_value)

class Status2:
    encoder_pos = 0.0
    encoder_vel = 0.0
    def __call__(self):
        return struct.pack("<ff", self.encoder_vel, self.encoder_pos)


class Status3:
    analog_voltage = 0.0
    analog_velocity = 0.0
    analog_position = 0.0

    def __call__(self):
        voltage = int(self.analog_voltage * 204.2)
        velocity = int(self.analog_velocity * 128.0)
        volt_vel = ((voltage & 0x3ff) | ((velocity & 0x3fffff)) << 10)
        return struct.pack("<If", volt_vel, self.analog_position)

class Status4:
    alt_encoder_pos = 0.0
    alt_encoder_vel = 0.0
    def __call__(self):
        return struct.pack("<ff", self.alt_encoder_vel, self.alt_encoder_pos)

class Status5:
    abs_encoder_pos = 0.0
    abs_encoder_vel = 0.0
    def __call__(self):
        return struct.pack("<ff", self.abs_encoder_vel, self.abs_encoder_pos)

class Status6:
    duty_cycle = 0.0
    period = 0
    no_signal = False

    def __call__(self):
        duty = int(self.duty_cycle * 64886.14)
        return struct.pack("<2hI", duty, self.period, self.no_signal)


class Status7:
    integral_acc = 0.0
    def __call__(self):
        return struct.pack("<fI", self.integral_acc, 0)

class MessageAPI:
    def __init__(self, class_id, index_id):
        self.class_id = class_id
        self.index_id = index_id
    
    def __eq__(self, other):
        return self.class_id == other.class_id and self.index_id == other.index_id
# ---------------------------------------------------------------------------------

class SparkMock:
    DUTY_CYCLE_SETPOINT = MessageAPI(0, 2)
    VELOCITY_SETPOINT = MessageAPI(0, 0)
    POSITION_SETPOINT = MessageAPI(0, 4)

    def __init__(self, name, device_id, max_velocity):
        self.name = name
        self.device_id = device_id
        self.__lock = Lock()
        self.__statuses = [
            Status0(),
            Status1(),
            Status2(),
            Status3(),
            Status4(),
            Status5(),
            Status6(),
            Status7(),
        ]
        self.__setpoint = 0.0
        self.__control_type = ControlType.DUTY_CYCLE
        self.__max_velocity = max_velocity
    
    def __get_address(self, status_idx: int) -> int:
        # Class ID = 46 for status frames
        return (2 << 24) | (5 << 16) | (46 << 10) | (status_idx << 6) | self.device_id
    
    def get_status_frame(self, status_idx: int) -> Message:
        status_msg = self.__statuses[status_idx]
        return Message(
                arbitration_id=self.__get_address(status_idx), 
                data=status_msg(), 
                is_extended_id=True)
    
    def update_statuses(self, time_diff: float):
        with self.__lock:
            if self.__control_type == ControlType.DUTY_CYCLE:
                self.__statuses[0].applied_output = self.__setpoint
                self.__statuses[2].encoder_vel = 0.0
                self.__statuses[2].encoder_pos = 0.0
                self.__statuses[5].abs_encoder_vel = 0.0
                self.__statuses[5].abs_encoder_pos = 0.0

            elif self.__control_type == ControlType.VELOCITY:
                self.__statuses[2].encoder_vel = self.__setpoint + random.uniform(-0.005, 0.005)
                if self.__setpoint < -self.__max_velocity:
                    self.__statuses[2].encoder_vel = -self.__max_velocity
                elif self.__setpoint > self.__max_velocity:
                    self.__statuses[2].encoder_vel = self.__max_velocity
                if self.__setpoint != 0.0:
                    self.__statuses[2].encoder_pos += self.__statuses[2].encoder_vel * time_diff
                self.__statuses[0].applied_output = self.__setpoint / self.__max_velocity
            
            elif self.__control_type == ControlType.POSITION:
                pass
                
        self.__statuses[0].voltage = 12.0 + random.uniform(-0.03, 0.0)
        self.__statuses[0].current = 0.5 + random.uniform(-0.05, 0.05) + 20.0 * abs(self.__statuses[0].applied_output)
        self.__statuses[0].motor_temperature = 25.0 + 1.50 * self.__statuses[0].current

    
    def process_command(self, command: Message):
        device_id = command.arbitration_id & 0x3f
        device_type = (command.arbitration_id) >> 24
        manufacturer = (command.arbitration_id >> 16) & 0xff
        if device_id != self.device_id or device_type != 2 or manufacturer != 5:
            return
        with self.__lock:
            index_id = (command.arbitration_id >> 6) & 0x0f
            class_id = (command.arbitration_id >> 10) & 0x3f
            recv_msg = MessageAPI(class_id, index_id)
            if recv_msg == self.DUTY_CYCLE_SETPOINT:
                self.__control_type = ControlType.DUTY_CYCLE
                self.__setpoint, = struct.unpack("<f", command.data[:4])
            elif recv_msg == self.VELOCITY_SETPOINT:
                self.__control_type = ControlType.VELOCITY
                self.__setpoint, = struct.unpack("<f", command.data[:4])
            elif recv_msg == self.POSITION_SETPOINT:
                self.__control_type = ControlType.POSITION
                self.__setpoint, = struct.unpack("<f", command.data[:4])


class SparkMockRunner:
    def __init__(self, config):
        self.nodes = config["can_nodes"]
        self.bus = ThreadSafeBus(
            interface="socketcan", 
            channel=config["can_interface"], 
            bitrate=config["can_bitrate"],
            can_filters=[f | {"extended": True} for f in config["filters"]]
        )
        self.notifier = Notifier(
            self.bus, 
            listeners=[
                (lambda msg, id=node["device_id"]: self.can_rx_callback(id, msg)) 
                for node in self.nodes
            ]
        )
        self.timers = [
            Thread(
                target=self.can_tx_callback, 
                args=(node["device_id"], list(map(lambda x: 1/x if x != 0 else float('inf'), node["status_rates"]))),
                daemon=True
            )
            for node in self.nodes
        ]
        self.devices = {node["device_id"]: SparkMock(node["name"], node["device_id"], node["max_velocity"]) for node in self.nodes}

        for timer in self.timers:
            timer.start()
    
    def __del__(self):
        self.notifier.stop()
        self.bus.shutdown()

    def can_rx_callback(self, device_id: int, msg: Message) -> None:
        spark_mock = self.devices[device_id]
        spark_mock.process_command(msg)

    def can_tx_callback(self, device_id: int, status_periods: List[float]) -> None:
        prev_time = [0] * len(status_periods)
        spark_mock = self.devices[device_id]
        while True:
            current_time = time.time()
            for i in range(len(status_periods)):
                if current_time - prev_time[i] >= status_periods[i]:
                    prev_time[i] = current_time
                    status_msg = spark_mock.get_status_frame(i)
                    self.bus.send(status_msg)
            delta_time = time.time() - current_time
            spark_mock.update_statuses(delta_time)

def load_yaml(config_path) -> dict:
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config

def get_base_dir() -> str:
    dir = os.path.join(os.path.realpath(__file__), os.pardir, os.pardir)
    return os.path.abspath(dir)

def main():
    base_dir = get_base_dir()
    config_path = sys.argv[1] if len(sys.argv) > 1 else \
                    os.path.join(base_dir, "config", "default_runner.yaml")
    config = load_yaml(config_path)

    runners = []
    for runner_config in config["mock_runners"]:
        runners.append(SparkMockRunner(runner_config))
    
    try:
        while True:
            time.sleep(1)
    except:
        pass

if __name__ == "__main__":
    main()