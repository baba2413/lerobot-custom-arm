
from serial import Serial
import time
from mit_func import can2serial


class RobstrideMotor:
    def __init__(self, port: str, baudrate: int, host_id: int = 253):
        self.port = port
        self.baudrate = baudrate
        self.motor_can_id = None
        self.host_id = host_id
        self.init_serial_connection()
        self.pos = 0.0
        self.vel = 0.0
        self.last_message_print_time = 0

    def init_serial_connection(self):
        self.serial = Serial(self.port, self.baudrate, timeout=0.1)
        print(f"Connected to {self.port} at {self.baudrate} baud.")

    def send_message(self, encoded_message: bytes):
        self.serial.write(encoded_message)

    def set_motor_id(self, id: int):
        self.motor_can_id = id

    def set_mode(self, mode: int):
        mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
        mode_serial = can2serial("1200FD01", mode_data)
        self.send_message(mode_serial)
        time.sleep(0.05)
        raw_data = self.serial.read(self.serial.in_waiting) # empty buffer