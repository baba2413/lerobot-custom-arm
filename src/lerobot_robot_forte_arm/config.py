from dataclasses import dataclass, field
from lerobot.common.robot_devices.robots.configs import RobotConfig

@dataclass
class ForteArmConfig(RobotConfig):
    # 로봇 팔의 모터 개수 (예: 6자유도 + 그리퍼 = 7)
    action_dim: int = 7
    state_dim: int = 7
    
    # 사용할 카메라 설정 (우선 웹캠 1대 세팅)
    cameras: dict = field(default_factory=lambda: {
        "front": {
            "type": "opencv",
            "index_or_path": 0,
            "width": 640,
            "height": 480,
            "fps": 30
        }
    })