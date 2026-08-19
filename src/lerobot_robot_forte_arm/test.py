from lerobot.motors import Motor, MotorNormMode
# from robstride_motors_bus import RobstrideMotorsBus
from lerobot.motors.robstride.robstride import RobstrideMotorsBus

bus = RobstrideMotorsBus(
    port="/dev/ttyUSB0",
    motors={
        "test_motor": Motor(
            id=1,
            model="robstride",
            norm_mode=MotorNormMode.DEGREES,
            motor_type_str="o0",
            recv_id=1,
        )
    },
    can_interface="slcan",
    bitrate=1000000,
    use_can_fd=False,
)

try:
    bus.connect(handshake=False)
    print("connected without handshake")

    # bus.configure_motors()
    # print("configured")
    bus._enable_motor("test_motor")

    print(bus.read("Present_Position", "test_motor"))
    # print(bus._read_state("test_motor"))
    bus.disconnect()

finally:
    if bus.is_connected:
        bus.disconnect()