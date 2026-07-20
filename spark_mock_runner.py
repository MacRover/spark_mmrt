#!/usr/bin/env python3
import time
from math import pi
import random
import struct
import os
import sys
from typing import List
import yaml
from can import ThreadSafeBus, Message, Notifier
from threading import Thread, Lock
import logging

from enum import Enum

# -------------------------------------------------------------------- 
# -------------------------- Data Classes ----------------------------
# --------------------------------------------------------------------
class ControlType(Enum):
    DUTY_CYCLE = 0
    VELOCITY = 1
    VOLTAGE = 2
    POSITION = 3
    SMARTMOTION = 4
    SMARTVELOCITY = 5
    MAXMOTION_POSITION = 6
    MAXMOTION_VELOCITY = 7

class SensorType(Enum):
    NONE = 0
    MAIN_ENCODER = 1
    ANALOG = 2
    ALT_ENCODER = 3
    DUTY_CYCLE = 4

class ParameterID(Enum):
    CONTROL_TYPE = 5
    SENSOR_TYPE = 9
    P0 = 13
    I0 = 14
    D0 = 15
    F0 = 16
    POSITION_FACTOR = 112
    VELOCITY_FACTOR = 113
    DUTY_CYCLE_POSITION_FACTOR = 139
    DUTY_CYCLE_VELOCITY_FACTOR = 140
    DUTY_CYCLE_INVERTED = 141

class ParameterType(Enum):
    UNUSED = 0
    INT = 1
    UINT = 2
    FLOAT = 3
    BOOL = 4

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

class Status8:
    setpoint = 0.0
    at_setpoint = False
    pid_slot = 0
    def __call__(self):
        pslot_flag = self.at_setpoint | ((self.pid_slot & 0xf) << 1)
        return struct.pack("<fI", self.setpoint, pslot_flag)

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
    PARAMETER_WRITE = MessageAPI(14, 0)

    def __init__(self, name, device_id, heartbeat, heartbeat_timeout, max_velocity, max_acceleration, params):
        self.name = name
        self.device_id = device_id
        self.heartbeat = heartbeat
        self.heartbeat_timeout = heartbeat_timeout
        self.logger = logging.getLogger(self.__class__.__name__ + f"_{self.name}")
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
            Status8(),
        ]
        self.__setpoint = 0.0
        self.__active_setpoint = 0.0
        self.__control_type = ControlType(params.get("control_type", 0))
        self.__sensor_type = SensorType(params.get("sensor_type", 1))
        self.__p = float(params.get("p", 0.0))
        self.__i = float(params.get("i", 0.0))
        self.__d = float(params.get("d", 0.0))
        self.__f = float(params.get("f", 0.0))
        self.__pos_factor = params.get("position_factor", 1.0)
        self.__vel_factor = params.get("velocity_factor", 1.0)
        self.__abs_pos_factor = params.get("abs_position_factor", 1.0)
        self.__abs_vel_factor = params.get("abs_velocity_factor", 1.0)
        self.__setpoint_pos_factor = 1.0
        self.__setpoint_vel_factor = 1.0
        # Convert units from rad/s to RPM and rad/s^2 to RPM^2 respectively, also
        # remember to apply the scaling factor in case we need to change the units
        self.__max_velocity = max_velocity * 60.0 / (2.0 * pi) * self.__vel_factor
        self.__acceleration = max_acceleration * 60.0 ** 2 / (2.0 * pi) * self.__vel_factor

        self.__heartbeat_timer = self.heartbeat_timeout
        self.__heartbeat_timed_out = False

        self.__moving = False
        self.__hold_position = 0.0
        self.__prev_setpoint = 0.0
        self.__rampdown_x = 0.0
        self.__sustained_x = 0.0
        self.__starting_pos = 0.0
        self.__direction = 1.0
        self.update_parameters(ParameterID.SENSOR_TYPE.value, self.__sensor_type.value)
        self.logger.info(f"Initialized {self.name} with ID {self.device_id}")
    
    def __get_address(self, cls: int, idx: int) -> int:
        return (2 << 24) | (5 << 16) | (cls << 10) | (idx << 6) | self.device_id
    
    def get_status_frame(self, status_idx: int) -> Message:
        status_msg = self.__statuses[status_idx]
        self.logger.debug(f"Preparing STATUS_{status_idx}, Address: {hex(self.__get_address(46, status_idx))}, Data: {status_msg()}")
        return Message(
                # Class ID = 46 for status frames
                arbitration_id=self.__get_address(46, status_idx), 
                data=status_msg(), 
                is_extended_id=True)
    
    def handle_position_state(self, time_diff: float):
        if self.__moving and self.__active_setpoint == 0.0:
            self.__hold_position = self.__statuses[2].encoder_pos
            self.__moving = False
        elif self.__active_setpoint != 0.0:
            self.__statuses[2].encoder_pos += self.__statuses[2].encoder_vel * (self.__pos_factor / self.__vel_factor) * time_diff
            self.__moving = True
        else:
            self.__statuses[2].encoder_pos = self.__hold_position + random.gauss(0.0, 0.0001 * self.__pos_factor)
    
    def update_statuses(self, time_diff: float):
        with self.__lock:
            if self.heartbeat:
                if self.__heartbeat_timer <= self.heartbeat_timeout:
                    self.__active_setpoint = self.__setpoint
                elif not self.__heartbeat_timed_out:
                    self.logger.info(f"Heartbeat timeout, stopping motor")
                    self.__heartbeat_timed_out = True
                    self.__moving = False
                    self.__hold_position = self.__statuses[2].encoder_pos
                    self.__active_setpoint = self.__hold_position if self.__control_type == ControlType.POSITION else 0.0
            else:
                self.__active_setpoint = self.__setpoint

            if self.__control_type == ControlType.DUTY_CYCLE:
                self.__statuses[0].applied_output = self.__active_setpoint
                if self.__active_setpoint < -1.0:
                    self.__statuses[0].applied_output = -1.0
                elif self.__active_setpoint > 1.0:
                    self.__statuses[0].applied_output = 1.0
                
                self.handle_position_state(time_diff)
                self.__statuses[2].encoder_vel = self.__active_setpoint * self.__max_velocity + random.gauss(0.0, 0.00025 * self.__vel_factor)
                self.__statuses[5].abs_encoder_vel = self.__statuses[2].encoder_vel * (self.__abs_vel_factor / self.__vel_factor)
                self.__statuses[5].abs_encoder_pos = (self.__statuses[2].encoder_pos * (self.__abs_pos_factor / self.__pos_factor)) % self.__abs_pos_factor
                self.__statuses[8].at_setpoint = (self.__active_setpoint == self.__setpoint)

            elif self.__control_type == ControlType.VELOCITY:
                self.__statuses[2].encoder_vel = self.__active_setpoint + random.gauss(0.0, 0.00025 * self.__vel_factor)
                if self.__active_setpoint < -self.__max_velocity:
                    self.__statuses[2].encoder_vel = -self.__max_velocity
                elif self.__active_setpoint > self.__max_velocity:
                    self.__statuses[2].encoder_vel = self.__max_velocity

                self.handle_position_state(time_diff)
                self.__statuses[0].applied_output = self.__active_setpoint / self.__max_velocity
                self.__statuses[5].abs_encoder_vel = self.__statuses[2].encoder_vel * (self.__abs_vel_factor / self.__vel_factor)
                self.__statuses[5].abs_encoder_pos = (self.__statuses[2].encoder_pos * (self.__abs_pos_factor / self.__pos_factor)) % self.__abs_pos_factor
                self.__statuses[8].at_setpoint = (self.__active_setpoint == self.__setpoint)
            
            elif self.__control_type == ControlType.POSITION:
                error_pos = self.__active_setpoint - self.__statuses[2].encoder_pos
                delta = self.__statuses[2].encoder_pos - self.__starting_pos
                if self.__active_setpoint != self.__prev_setpoint or (not self.__moving and abs(error_pos) > 0.005 * self.__pos_factor):
                    self.__moving = True
                    self.__statuses[8].at_setpoint = False
                    self.__prev_setpoint = self.__active_setpoint
                    self.__rampdown_x = self.__max_velocity ** 2 / (2.0 * self.__acceleration) * (self.__pos_factor / self.__vel_factor)
                    self.__sustained_x = abs(error_pos) - self.__rampdown_x
                    self.__starting_pos = self.__statuses[2].encoder_pos
                    self.__direction = (1.0 if error_pos > 0 else -1.0)
                elif self.__moving:
                    if abs(delta) <= self.__sustained_x:
                        self.__statuses[2].encoder_vel = self.__max_velocity * self.__direction
                    elif abs(delta) - self.__sustained_x <= self.__rampdown_x - 0.0005 * self.__pos_factor:
                        self.__statuses[2].encoder_vel = self.__max_velocity * self.__direction * (1.0 - (abs(delta) - self.__sustained_x) / self.__rampdown_x)
                    else:
                        self.logger.debug(f"Setpoint reached! Position: {self.__statuses[2].encoder_pos}, Setpoint: {self.__active_setpoint}")
                        self.__statuses[8].at_setpoint = True
                        self.__hold_position = self.__statuses[2].encoder_pos
                        self.__moving = False

                    self.__statuses[2].encoder_pos += self.__statuses[2].encoder_vel * (self.__pos_factor / self.__vel_factor) * time_diff
                else:
                    self.__statuses[2].encoder_vel = random.gauss(0.0, 0.00025 * self.__vel_factor)
                    self.__statuses[2].encoder_pos = self.__hold_position + random.gauss(0.0, 0.0001 * self.__pos_factor)

                self.__statuses[0].applied_output = self.__statuses[2].encoder_vel / self.__max_velocity
                self.__statuses[5].abs_encoder_vel = self.__statuses[2].encoder_vel * (self.__abs_vel_factor / self.__vel_factor)
                self.__statuses[5].abs_encoder_pos = (self.__statuses[2].encoder_pos * (self.__abs_pos_factor / self.__pos_factor)) % self.__abs_pos_factor

            self.__heartbeat_timer += time_diff * 60.0

        self.__statuses[0].voltage = 12.0 + random.uniform(-0.03, 0.0)
        self.__statuses[0].current = 0.5 + random.uniform(-0.05, 0.05) + 20.0 * abs(self.__statuses[0].applied_output)
        # self.__statuses[0].motor_temperature = 25.0 + 1.50 * self.__statuses[0].current
    
    def get_parameter(self, param_id: int) -> tuple[int | float | bool, ParameterType]:
        if param_id not in ParameterID._value2member_map_:
            self.logger.warning("Attempted to get an unknown parameter, is it supported?")
            return 0, ParameterType.UNUSED
        
        param_id = ParameterID(param_id)
        if param_id == ParameterID.CONTROL_TYPE:
            return self.__control_type.value, ParameterType.UINT
        elif param_id == ParameterID.P0:
            return self.__p, ParameterType.FLOAT
        elif param_id == ParameterID.I0:
            return self.__i, ParameterType.FLOAT
        elif param_id == ParameterID.D0:
            return self.__d, ParameterType.FLOAT
        elif param_id == ParameterID.F0:
            return self.__f, ParameterType.FLOAT
        elif param_id == ParameterID.POSITION_FACTOR:
            return self.__pos_factor, ParameterType.FLOAT
        elif param_id == ParameterID.VELOCITY_FACTOR:
            return self.__vel_factor, ParameterType.FLOAT
        elif param_id == ParameterID.DUTY_CYCLE_POSITION_FACTOR:
            return self.__abs_pos_factor, ParameterType.FLOAT
        elif param_id == ParameterID.DUTY_CYCLE_VELOCITY_FACTOR:
            return self.__abs_vel_factor, ParameterType.FLOAT
        elif param_id == ParameterID.DUTY_CYCLE_INVERTED:
            return False, ParameterType.BOOL
        elif param_id == ParameterID.SENSOR_TYPE:
            return self.__sensor_type.value, ParameterType.UINT

    def update_parameters(self, param_id: int, param_value: int) -> tuple[bool, ParameterType]:
        if param_id not in ParameterID._value2member_map_:
            self.logger.warning("Received unknown/unused parameter, is it supported?")
            return False, ParameterType.UNUSED
        
        param_id = ParameterID(param_id)
        type = ParameterType.UNUSED
        update_setpoint_factors = False

        if param_id == ParameterID.CONTROL_TYPE:
            self.__control_type = ControlType(param_value)
            type = ParameterType.UINT
            self.logger.info(f"Updated Control Type to {self.__control_type.name}")

        elif param_id == ParameterID.P0:
            self.__p, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated P0 to {self.__p}")

        elif param_id == ParameterID.I0:
            self.__i, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated I0 to {self.__i}")

        elif param_id == ParameterID.D0:
            self.__d, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated D0 to {self.__d}")

        elif param_id == ParameterID.F0:
            self.__f, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated F0 to {self.__f}")

        elif param_id == ParameterID.POSITION_FACTOR:
            self.__pos_factor, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated Position Factor to {self.__pos_factor}")
            update_setpoint_factors = True

        elif param_id == ParameterID.VELOCITY_FACTOR:
            new_vel_factor, = struct.unpack("<f", struct.pack("<I", param_value))
            self.__max_velocity = self.__max_velocity * (new_vel_factor / self.__vel_factor)
            self.__acceleration = self.__acceleration * (new_vel_factor / self.__vel_factor)
            self.__vel_factor = new_vel_factor
            type = ParameterType.FLOAT
            self.logger.info(f"Updated Velocity Factor to {self.__vel_factor}")
            update_setpoint_factors = True

        elif param_id == ParameterID.DUTY_CYCLE_POSITION_FACTOR:
            self.__abs_pos_factor, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated Duty Cycle Position Factor to {self.__abs_pos_factor}")
            update_setpoint_factors = True

        elif param_id == ParameterID.DUTY_CYCLE_VELOCITY_FACTOR:
            self.__abs_vel_factor, = struct.unpack("<f", struct.pack("<I", param_value))
            type = ParameterType.FLOAT
            self.logger.info(f"Updated Duty Cycle Velocity Factor to {self.__abs_vel_factor}")
            update_setpoint_factors = True

        elif param_id == ParameterID.DUTY_CYCLE_INVERTED:
            type = ParameterType.BOOL

        if update_setpoint_factors or param_id == ParameterID.SENSOR_TYPE:
            sensor = SensorType(param_value)
            if sensor == SensorType.MAIN_ENCODER:
                self.__setpoint_pos_factor = 1.0
                self.__setpoint_vel_factor = 1.0
            elif sensor == SensorType.DUTY_CYCLE:
                self.__setpoint_pos_factor = (self.__pos_factor / self.__abs_pos_factor)
                self.__setpoint_vel_factor = (self.__vel_factor / self.__abs_vel_factor)
            elif sensor == SensorType.NONE:
                self.__setpoint_pos_factor = 0.0
                self.__setpoint_vel_factor = 0.0

            self.__sensor_type = sensor
            type = ParameterType.UINT
            if param_id == ParameterID.SENSOR_TYPE:
                self.logger.info(f"Updated Sensor Type to {self.__sensor_type.name}")

        return True, type
    
    def process_command(self, command: Message) -> None | Message:
        device_id = command.arbitration_id & 0x3f
        api_id = (command.arbitration_id >> 6) & 0x3ff
        device_type = (command.arbitration_id) >> 24
        manufacturer = (command.arbitration_id >> 16) & 0xff

        # Listen for RIO heartbeat messages and reset the heartbeat timer on arrival
        if device_id == 0 and device_type == 1 and manufacturer == 1 and api_id == 0x061:
            self.logger.debug(f"Received Primary Heartbeat, refreshing timer")
            with self.__lock:
                self.__heartbeat_timer = 0.0
                self.__heartbeat_timed_out = False
                self.__statuses[0].primary_heartbeat_lock = True

        if device_id != self.device_id or device_type != 2 or manufacturer != 5:
            return
        
        self.logger.debug(f"Received CAN Frame -> {hex(command.arbitration_id)}")
        with self.__lock:
            index_id = (api_id) & 0x0f
            class_id = (api_id >> 4)
            recv_msg = MessageAPI(class_id, index_id)
            if recv_msg == self.DUTY_CYCLE_SETPOINT:
                self.__control_type = ControlType.DUTY_CYCLE
                self.__setpoint, = struct.unpack("<f", command.data[:4])
                self.__statuses[8].setpoint = self.__setpoint
            elif recv_msg == self.VELOCITY_SETPOINT:
                self.__control_type = ControlType.VELOCITY
                raw_setpoint, = struct.unpack("<f", command.data[:4])
                self.__setpoint = raw_setpoint * self.__setpoint_vel_factor
                self.__statuses[8].setpoint = raw_setpoint
            elif recv_msg == self.POSITION_SETPOINT:
                self.__control_type = ControlType.POSITION
                raw_setpoint, = struct.unpack("<f", command.data[:4])
                self.__setpoint = raw_setpoint * self.__setpoint_pos_factor
                self.__statuses[8].setpoint = raw_setpoint
            elif recv_msg == self.PARAMETER_WRITE:
                param_id, param_value = struct.unpack("<BI", command.data)
                self.logger.debug(f"Received Parameter Write -> Param ID: {param_id}, Raw Value: {param_value}")
                success, param_type = self.update_parameters(param_id, param_value)
                if success:
                    return Message(
                        arbitration_id=self.__get_address(14, 1),
                        data=struct.pack("<2BIB", param_id, param_type.value, param_value, 0),
                        is_extended_id=True)
                return Message(
                    arbitration_id=self.__get_address(14, 1),
                    data=struct.pack("<2BIB", param_id, param_type.value, param_value, 4),
                    is_extended_id=True)
            elif class_id >= 48 and command.dlc <= 1:
                param_id = (api_id) & 0xff
                self.logger.info(f"Parameter poll requested, ID: {param_id}")
                param_value, param_type = self.get_parameter(param_id)
                if param_type != ParameterType.UNUSED:
                    format_type = "f" if param_type == ParameterType.FLOAT else "I"
                    self.logger.info(f"Parameter request success, value: {param_value}")
                    return Message(
                        arbitration_id=self.__get_address(class_id, param_id),
                        data=struct.pack(f"<{format_type}2B", param_value, param_type.value, 0),
                        is_extended_id=True)
                return Message(
                    arbitration_id=self.__get_address(class_id, param_id),
                    data=struct.pack("<I2B", param_value, ParameterType.UNUSED.value, 0),
                    is_extended_id=True)


class SparkMockRunner:
    def __init__(self, config):
        self.name = config["name"]
        self.nodes = config["can_nodes"]
        self.logger = logging.getLogger(self.__class__.__name__ + f"_{self.name}")
        self.bus = ThreadSafeBus(
            interface="socketcan", 
            channel=config["can_interface"], 
            bitrate=config["can_bitrate"],
            can_filters=[f | {"extended": True} for f in config["filters"]] 
        )
        self.devices = {
            node["device_id"]: 
                SparkMock(
                    node["name"], 
                    node["device_id"], 
                    node.get("listen_for_heartbeat", True),
                    node.get("heartbeat_timeout", 1.0),
                    node.get("max_velocity", 5.0),
                    node.get("max_acceleration", 10.0),
                    node["flash"]
                ) 
            for node in self.nodes
        }
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

        for timer in self.timers:
            timer.start()
        self.logger.info(f"Initialized {self.name}")
    
    def __del__(self):
        self.notifier.stop()
        self.bus.shutdown()

    def can_rx_callback(self, device_id: int, msg: Message) -> None:
        spark_mock = self.devices[device_id]
        resp_msg = spark_mock.process_command(msg)
        if resp_msg:
            self.bus.send(resp_msg)

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
            spark_mock.update_statuses(delta_time / 60.0)

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

    log_level = {
        "debug": logging.DEBUG,
        "info": logging.INFO,
        "warning": logging.WARNING,
        "error": logging.ERROR,
        "critical": logging.CRITICAL
    }
    logging.basicConfig(
        level=log_level[config.get("log_level", "info").lower()],
        format="[%(levelname)s]: (%(name)s): %(message)s"
    )

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