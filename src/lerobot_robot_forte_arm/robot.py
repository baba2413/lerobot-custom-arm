from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from robstride_motors_bus import RobstrideMotorsBus
from lerobot.robots import Robot

class ForteArm(Robot):
    config_class = ForteArmConfig
    name = "forte_arm"

    def __init__(self, config: ForteArmConfig):
        super().__init__(config)
        self.bus = RobstrideMotorsBus(
            port = self.config.port,
            motors={
                "shoulder_yaw": Motor(),
                "shoulder_pitch": Motor(),
                "shoulder_roll": Motor(),
                "elbow_pitch": Motor(),
                "lower_arm_roll": Motor(),
                "wrist_pitch": Motor(),
                "LeftGripperJoint": Motor(),
            },
            calibration=self.calibration,
        )
        self.cameras = make_cameras_from_configs(config.cameras)

        @property
        def _motors_ft(self) -> dict[str, type]:
            return {
                "shoulder_yaw": float,
                "shoulder_pitch": float,
                "shoulder_roll": float,
                "elbow_pitch": float,
                "lower_arm_roll": float,
                "wrist_pitch": float,
                "LeftGripperJoint": float,
            }
        
        @property
        def _cameras_ft(self) -> dict[str, tuple]:
            return{
                cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
            }
        
        @property
        def observation_features(self)-> dict:
            return {**self._motors_ft, **self._cameras_ft}
        
        def action_features(self) -> dict:
            return self._motor_ft
        
        @property
        def is_connected(self) -> bool:
            return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())
        
        def connect(self, calibrate:bool = True) -> None:
            self.bus.connect()
            if not self.is_calibrated and calibrate:
                self.calibrate()

            for cam in self.camera.values():
                cam.connect()

            self.configure()

        def disconnect(self) -> None:
            self.bus.disconnect()
            for cam in self.cameras.values():
                cam.disconnect()

        @property
        def is_calibrated(self) -> bool:
            return True

        def calibrate(self) -> None:
            self.bus.disable_torque()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            input(f"Move {self} to the middle of its range of motion and press Enter.")
            homing_offset = self.bus.set_half_turn_homings()

        @property
        def is_calibrated(self) -> bool:
            return self.bus.is_calibrated
        
        def configure(self) -> None:
            with self.bus.torque_disabled():
                self.bus.configure_motors()
                for motor in self.bus.motors:
                    self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
                    self.bus.write("P_Coefficient", motor, 16)
                    self.bus.write("I_Coefficient", motor, 0)
                    self.bus.write("d_Coefficient", motor, 32)

        def get_observation(self) -> dict[str, Any]:
            if not self.is_connected:
                raise ConnectionError(f"{self} is not connected")
            
            obs_dict = self.bus.sync_read("Present_Position")
            obs_dict = {f"{motor}.pos":val for motor,val in obs_dict.items()}

            for cam_key, cam in self.cameras.items():
                obs_dict[cam_key] = cam.aync_read()

            return obs_dict
        
        def send_action(self, action: dict[str, Any]) -> dict[str,Any]:
            goal_pos = {key.removeduffix(".pos"): val for key,val in action.items()}

            self.bus.sync_write("Goal_Position", goal_pos)
            return action