from motors_bus import MotorsBusBase, Motor, MotorCalibration, Value
from robstride_mit import mit_func
from robstride_mit.robstride_motor.motor import RobstrideMotor

class RobstrideMotorsBus(MotorsBusBase):
    def __init__(
        self,
        port: str,
        motors: dict[str, Motor],
        calibration: dict[str, MotorCalibration] | None = None,
        baudrate: int = 921600,
        host_id: int = 253,
        kp: float = 20.0,
        kd: float = 0.5,
    ):
        super().__init__(port, motors, calibration)
        self.baudrate = baudrate
        self.host_id = host_id
        self.kp = kp
        self.kd = kd
        self.motor = None
        self._is_connected = False
        self.last_state = {}

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self, handshake: bool = True) -> None:
        self.motor = RobstrideMotor(
            port=self.port,
            baudrate=self.baudrate,
            host_id=self.host_id,
        )

        for motor_name, motor in self.motors.items():
            self.motor.set_motor_id(motor.id)
            self._set_mode(motor.id, mit_func.MIT_MODE)

        self._is_connected = True

    def disconnect(self, disable_torque: bool = True) -> None:
        if self.motor is not None and disable_torque:
            for motor_name, motor in self.motors.items():
                self._set_mode(motor.id, mit_func.MENU_MODE)

        if self.motor is not None:
            self.motor.serial.close()

        self._is_connected = False

    def read(self, data_name: str, motor: str) -> Value:
        state = self._read_state(motor)

        if data_name not in state:
            raise KeyError(f"Unsupported data_name: {data_name}")

        return state[data_name]

    def write(self, data_name: str, motor: str, value: Value) -> None:
        if data_name == "Goal_Position":
            self._write_position(motor, float(value))
        else:
            raise KeyError(f"Unsupported data_name: {data_name}")

    def sync_read(self, data_name: str, motors: str | list[str] | None = None) -> dict[str, Value]:
        motor_names = self._normalize_motor_names(motors)

        return {
            motor_name: self.read(data_name, motor_name)
            for motor_name in motor_names
        }

    def sync_write(self, data_name: str, values: dict[str, Value]) -> None:
        for motor_name, value in values.items():
            self.write(data_name, motor_name, value)

    def enable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        motor_names = self._normalize_motor_names(motors)

        for motor_name in motor_names:
            motor_id = self.motors[motor_name].id
            self._set_mode(motor_id, mit_func.MIT_MODE)

    def disable_torque(self, motors: str | list[str] | None = None, num_retry: int = 0) -> None:
        motor_names = self._normalize_motor_names(motors)

        for motor_name in motor_names:
            motor_id = self.motors[motor_name].id
            self._set_mode(motor_id, mit_func.MENU_MODE)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.calibration

    def write_calibration(
        self,
        calibration_dict: dict[str, MotorCalibration],
        cache: bool = True,
    ) -> None:
        if cache:
            self.calibration = calibration_dict

    def _write_position(self, motor_name: str, position: float) -> None:
        motor_id = self.motors[motor_name].id

        command_bytes = mit_func.pack_cmd(
            p=position,
            v=0.0,
            kp=self.kp,
            kd=self.kd,
            t_ff=0.0,
        )

        data = " ".join(f"{b:02X}" for b in command_bytes)
        can_id = self._make_mit_can_id(motor_id)
        serial_cmd = mit_func.can2serial(can_id, data)

        self.motor.send_message(serial_cmd)

    def _read_state(self, motor_name: str) -> dict[str, Value]:
        raw_data = self.motor.serial.read(self.motor.serial.in_waiting)

        if not raw_data:
            if motor_name in self.last_state:
                return self.last_state[motor_name]
            raise RuntimeError("No data received from Robstride motor.")

        can_data = raw_data[7:-2]
        decoded = mit_func.decode_reply(can_data)

        if decoded is None:
            if motor_name in self.last_state:
                return self.last_state[motor_name]
            raise RuntimeError("Failed to decode Robstride reply.")

        position, velocity, torque, vbus, temp = decoded

        state = {
            "Present_Position": position,
            "Present_Velocity": velocity,
            "Present_Torque": torque,
            "Present_Vbus": vbus,
            "Present_Temperature": temp,
        }

        self.last_state[motor_name] = state
        return state

    def _set_mode(self, motor_id: int, mode: int) -> None:
        mode_data = f"FF FF FF FF FF FF FF {mode:02X}"
        can_id = self._make_mode_can_id(motor_id)
        mode_serial = mit_func.can2serial(can_id, mode_data)
        self.motor.send_message(mode_serial)

        self.motor.serial.read(self.motor.serial.in_waiting)

    def _normalize_motor_names(self, motors: str | list[str] | None) -> list[str]:
        if motors is None:
            return list(self.motors.keys())
        if isinstance(motors, str):
            return [motors]
        return motors

    def _make_mit_can_id(self, motor_id: int) -> str:
        return f"1200FD{motor_id:02X}"

    def _make_mode_can_id(self, motor_id: int) -> str:
        return f"1200FD{motor_id:02X}"