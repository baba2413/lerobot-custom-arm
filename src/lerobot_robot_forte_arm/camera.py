from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.cameras.realsense.camera_realsense import RealSenseCamera
from lerobot.cameras.configs import ColorMode, Cv2Rotation

config = RealSenseCameraConfig(
    serial_number_or_name="825312072171",
    fps=15,
    width=640,
    height=480,
    color_mode=ColorMode.RGB,
    use_depth=True,
    rotation=Cv2Rotation.NO_ROTATION
)

camera = RealSenseCamera(config)
camera.connect()

try:
    color_frame = camera.read()
    depth_map = camera.read_depth()
    print("Color frame shape:", color_frame.shape)
    print("Depth map shape:", depth_map.shape)
finally:
    camera.disconnect()