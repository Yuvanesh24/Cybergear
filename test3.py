import serial
import time
import struct
import serial.tools.list_ports

# Check available COM ports
print("Available COM ports:", [port.device for port in serial.tools.list_ports.comports()])

# Initialize Serial Communication (Set Correct COM Port)
ser = serial.Serial('COM3', 921600, timeout=5.0)

# CAN Communication Frame Head and Tail
FRAME_HEAD = "4154"
FRAME_TAIL = "0d0a"

def read_response():
    """Reads response from the motor and prints it."""
    time.sleep(0.5)
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print("Motor Response:", data.hex())
    else:
        print("No response received.")

def float_to_uint(value, min_value, max_value, bits):
    """Converts a float into an unsigned integer within the specified range."""
    value = max(min(value, max_value), min_value)  # Clamping
    return int((value - min_value) * ((1 << bits) - 1) / (max_value - min_value))

class MotorController:
    def __init__(self, master_can, motor_can):
        self.master = master_can
        self.motor = motor_can

    def power_on(self):
        """Power on the motor."""
        command = '41542b41540d0a'
        print("Sending Power On Command:", command)
        ser.write(bytes.fromhex(command))
        ser.flush()
        time.sleep(1)
        read_response()

    def enable_motor(self):
        """Enable the motor using CAN communication."""
        bin_num = (3 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = FRAME_HEAD + hex_str + "080000000000000000" + FRAME_TAIL
        print("Sending Enable Motor Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def control_motor(self, torque, mech_position, speed, kp, kd):
        """Control the motor with proper torque and speed settings."""
        bin_num = (1 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")

        tx_data = [
            float_to_uint(mech_position, -4 * 3.1416, 4 * 3.1416, 16) >> 8,
            float_to_uint(mech_position, -4 * 3.1416, 4 * 3.1416, 16) & 0xFF,
            float_to_uint(speed, -296.0, 296.0, 16) >> 8,  # Speed range from motor spec
            float_to_uint(speed, -296.0, 296.0, 16) & 0xFF,
            float_to_uint(kp, 0.0, 1000.0, 16) >> 8,  # Position Gain (KP)
            float_to_uint(kp, 0.0, 1000.0, 16) & 0xFF,
            float_to_uint(kd, 0.0, 10.0, 16) >> 8,  # Derivative Gain (KD)
            float_to_uint(kd, 0.0, 10.0, 16) & 0xFF,
        ]

        tx_data_hex = "".join(format(x, "02x") for x in tx_data)
        hex_can = FRAME_HEAD + hex_str + "08" + tx_data_hex + FRAME_TAIL

        print("Sending Motor Control Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def stop_motor(self):
        """Stop the motor safely."""
        bin_num = (4 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = FRAME_HEAD + hex_str + "080000000000000000" + FRAME_TAIL
        print("Sending Stop Motor Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

# Initialize motor with correct CAN IDs
motor = MotorController(master_can=253, motor_can=127)

# Power on sequence
motor.power_on()
time.sleep(2)
motor.enable_motor()
time.sleep(2)

# **Increase Torque & Speed According to Motor Specs**
motor.control_motor(
    torque=12.0,  # Max Torque (12N.m)
    mech_position=1.5,
    speed=240.0,  # Max Continuous Speed (240 RPM)
    kp=500.0,  # Increased KP for better control
    kd=5.0  # Adjusted KD
)
time.sleep(5)

# Stop the motor
motor.stop_motor()
