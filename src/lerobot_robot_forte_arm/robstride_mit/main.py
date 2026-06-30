from robstride_motor.motor import RobstrideMotor
import numpy as np
import time
from time import sleep
import mit_func 
import sys


def main():
    # NOTE: set `port` to your device path on Ubuntu (e.g. /dev/ttyUSB0 or /dev/ttyACM0)
    motor = RobstrideMotor(port="/dev/ttyUSB0", baudrate=921600)
    motor.set_motor_id(1)

    # Set MIT Mode.
    motor.set_mode(mit_func.MIT_MODE)
    print(f"Sent MIT Mode command via serial. Motor is ready.")

    # Sine-wave parameters (gripper open -> close)
    open_pos = 6.15
    close_pos = 7.6
    center = (open_pos + close_pos) / 2.0
    amplitude = (close_pos - open_pos) / 2.0

    # Frequency in Hz (0.2 Hz = 5 second period). Adjust to taste.
    freq = 0.2

    # Control loop rate (seconds per command). 50 Hz is a reasonable default.
    dt = 0.02
    kp = 20  # Proportional gain (tune for your motor)
    kd = 0.5  # Derivative gain (tune for your motor)

    print(f"Starting sine-wave motion: center={center:.3f}, amp={amplitude:.3f}, freq={freq}Hz")
    start_t = time.time()
    printing = False
    try:
        for i in range(1000000):
            t = time.time() - start_t
            if int(i) % 50 == 0:
                printing = True
            else:
                printing=False

            pos = center + amplitude * np.cos(2 * np.pi * freq * t + np.pi)  # +pi to start at open position

            # optional: lightweight status print
            # if int(t) % 1 == 0:
            #     print(f"t={t:.2f}s target_pos={pos:.3f}", end='\r')
            
            if printing: print(f"Command State: p={pos:.3f}, v={0.0}, kp={kp:.3f}, kd={kd:.3f}, t_ff={0.0}")
            command_bytes = mit_func.pack_cmd(p=pos,v=0.0, kp=kp, kd=kd, t_ff=0.0)

            my_data = " ".join(f"{b:02X}" for b in command_bytes)
            # my_data = "05 70 00 00 01 00 00 00"
            if printing: print(f"my data: {my_data}")

            my_can_id = "1200FD01"

            serial_cmd = mit_func.can2serial(my_can_id, my_data, printing=printing)
            can_data = serial_cmd[7:-2]

            # printing can data we will send
            can_id = can_data[0]
            can_position = (can_data[1] << 8) | can_data[2]
            can_velocity = (can_data[3]<<4) | (can_data[4]>>4)
            can_torque = ((can_data[4] & 0x0F)<<8) | can_data[5]
            can_vbus = can_data[6]
            can_temp = can_data[7]

            if printing: 
                print(f"can_id: {hex(can_id)}")
                print(f"can_position: {hex(can_position)}")
                print(f"can_velocity: {hex(can_velocity)}")
                print(f"can_torque: {hex(can_torque)}")
                print(f"can_vbus: {hex(can_vbus)}")
                print(f"can_temp: {hex(can_temp)}")

            motor.send_message(serial_cmd)

            #recieving phase
            raw_data = motor.serial.read(motor.serial.in_waiting)

            if raw_data:
                if printing: print(f"Raw data received: {raw_data.hex()}")

            can_data = raw_data[7:-2]

            if can_data:

                if printing: print(f"can_data received: {can_data.hex()}")

                position, velocity, torque, vbus, temp = mit_func.decode_reply(can_data)
                if printing: print(f"Recieved State: p={position:.3f}, v={velocity:.3f}, torque={torque:.3f}, vbus={vbus:.3f}, temp={temp:.3f}")


                # print("-"*10 + " Recieved Motor State " + "-"*10)
                # print(f"Position : {position} rad")
                # print(f"Velocity : {velocity} rad/s")
                # print(f"Torque   : {torque} Nm")
                # print(f"Vbus     : {vbus} V")
                # print(f"Temp     : {temp} °C")
                # print("-" * 40)

            if printing: print("-"*40)
            time.sleep(dt)
                
    except KeyboardInterrupt:
        print("\nKeyboard interrupt received — stopping and disabling movement.")
        motor.set_mode(mit_func.MENU_MODE)
        print(f"Sent MENU Mode command via serial. Motor should be stopped.")

if __name__ == "__main__":
    main() 
