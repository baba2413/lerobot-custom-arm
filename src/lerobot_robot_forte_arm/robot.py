import torch
from lerobot.common.robot_devices.robots.utils import Robot
from .config import MyHandmadeArmConfig

class MyHandmadeArm(Robot):
    config_class = MyHandmadeArmConfig

    def __init__(self, config: MyHandmadeArmConfig, teleport_device=None):
        super().__init__(config, teleport_device)
        self.config = config
        self._is_connected = False

    def connect(self):
        """PCB와 컴퓨터가 연결되었을 때 시리얼 포트를 여는 구간입니다."""
        print("🔗 수제 로봇 팔에 연결을 시도합니다... (하드웨어 대기 중)")
        self._is_connected = True

    def get_observation(self) -> dict[str, torch.Tensor]:
        """로봇의 현재 모터 각도와 카메라 화면을 읽어오는 함수입니다."""
        # 하드웨어가 없으므로 임시로 0으로 가득 찬 가짜 모터 각도를 반환합니다.
        mock_state = torch.zeros(self.config.state_dim)
        return {"observation.state": mock_state}

    def send_action(self, action: torch.Tensor):
        """AI 두뇌가 '이렇게 움직여'라고 명령(Action)하면 모터에 각도를 쏘는 함수입니다."""
        # 나중에 여기에 PCB 시리얼 통신(예: serial.write) 코드가 들어갑니다.
        pass

    def disconnect(self):
        """종료할 때 모터 힘을 풀고 포트를 닫는 구간입니다."""
        self._is_connected = False
        print("🔌 연결을 안전하게 해제했습니다.")