import serial
import time
import struct
import serial.tools.list_ports

# Print available COM ports
print("Available COM ports:", [port.device for port in serial.tools.list_ports.comports()])

# Initialize serial connection
ser = serial.Serial('COM4', 921600, timeout=5.0)  

# Frame headers and tails for CAN messages
frame_head = "4154"
frame_tail = "0d0a"

def read_response():
    """Reads response from the motor."""
    time.sleep(0.5)  
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)  
        print("Motor Response:", data.hex())
    else:
        print("No response received.")

def float_to_uint(x, x_min, x_max, bits):
    """Convert a float to an unsigned integer within the specified range."""
    span = x_max - x_min
    offset = x_min
    x = max(min(x, x_max), x_min)  # Clamp value
    return int((x - offset) * ((1 << bits) - 1) / span)

class Cybergear:
    def __init__(self, master_can, motor_can):
        self.master = master_can
        self.motor = motor_can

    def power_on(self):
        """Send power-on signal."""
        command = '41542b41540d0a'
        print("Sending Power On Command:", command)
        ser.write(bytes.fromhex(command))  
        ser.flush()
        time.sleep(1)
        read_response()

    def enable_motor(self):
        """Enable motor (Communication Type 3)."""
        bin_num = (3 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        print("Sending Enable Motor Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def control_motor(self, torque, mech_position, speed, kp, kd):
        """Control the motor (Communication Type 1)."""
        bin_num = (1 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")

        # Convert float values to uint16
        tx_data = [
            float_to_uint(mech_position, -12.5, 12.5, 16) >> 8,
            float_to_uint(mech_position, -12.5, 12.5, 16) & 0xFF,
            float_to_uint(speed, -30.0, 30.0, 16) >> 8,
            float_to_uint(speed, -30.0, 30.0, 16) & 0xFF,
            float_to_uint(kp, 0.0, 500.0, 16) >> 8,
            float_to_uint(kp, 0.0, 500.0, 16) & 0xFF,
            float_to_uint(kd, 0.0, 5.0, 16) >> 8,
            float_to_uint(kd, 0.0, 5.0, 16) & 0xFF,
        ]

        tx_data_hex = "".join(format(x, "02x") for x in tx_data)
        hex_can = frame_head + hex_str + "08" + tx_data_hex + frame_tail

        print("Sending Motor Control Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def stop_motor(self):
        """Stop motor (Communication Type 4)."""
        bin_num = (4 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        print("Sending Stop Motor Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

# Initialize motor with master CAN ID and motor CAN ID
motor_1 = Cybergear(0x00, 0x7F)  # Master CAN ID: 0x00, Motor CAN ID: 0x7F

# Power on the motor
motor_1.power_on()
time.sleep(1)

# Enable the motor
motor_1.enable_motor()
time.sleep(1)

# Control the motor with desired position, speed, torque, kp, and kd
motor_1.control_motor(torque=0.5, mech_position=0.0, speed=0.2, kp=5.0, kd=0.5)
time.sleep(5)

# Stop the motor
motor_1.stop_motor()