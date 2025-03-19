import serial
import time
import struct
import serial.tools.list_ports

# Check available COM ports
print("Available COM ports:", [port.device for port in serial.tools.list_ports.comports()])

# Initialize Serial Communication
ser = serial.Serial('COM3', 921600, timeout=5.0)

# CAN Frame Constants
frame_head = "4154"
frame_tail = "0d0a"

def read_response():
    """Reads response from the motor and returns raw data."""
    time.sleep(0.5)  # Allow some time for response
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)  # Read all available data
        print("Raw Response:", data.hex())
        return data
    else:
        print("No response received.")
        return None

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

        tx_data = [
            float_to_uint(mech_position, -4 * 3.1416, 4 * 3.1416, 16) >> 8,
            float_to_uint(mech_position, -4 * 3.1416, 4 * 3.1416, 16) & 0xFF,
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

    def read_motor_status(self):
        """Request and read position, velocity, torque, and temperature from the motor."""
        
        ser.reset_input_buffer()  # Flush old data

        bin_num = (2 << 24) | (self.master << 8) | self.motor  
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail

        print("Requesting Motor Status:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        
        time.sleep(0.5)  # Wait for new response
        data = read_response()
        
        if data and len(data) >= 8:
            # Extract 2-byte values (Big-Endian format)
            position_raw = int.from_bytes(data[0:2], byteorder='big', signed=True)
            velocity_raw = int.from_bytes(data[2:4], byteorder='big', signed=True)
            torque_raw = int.from_bytes(data[4:6], byteorder='big', signed=True)
            temperature_raw = int.from_bytes(data[6:8], byteorder='big', signed=True)

            # Convert raw values to real-world units
            position = (position_raw / 65535) * (4 * 3.1416)  # Convert to radians
            velocity = (velocity_raw / 65535) * (30.0)  # Convert to rad/s
            torque = (torque_raw / 65535) * (12.0)  # Convert to Nm
            temperature = temperature_raw / 10.0  # Convert to Celsius

           
            print(f"Motor Position: {position:.4f} rad")
            print(f"Motor Velocity: {velocity:.4f} rad/s")
            print(f"Motor Torque: {torque:.4f} Nm")
            print(f"Motor Temperature: {temperature:.1f} °C")
            
            return {
                "position": position,
                "velocity": velocity,
                "torque": torque,
                "temperature": temperature
            }
        else:
            print("Invalid response length.")
            return None

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

# Initialize motor
motor_1 = Cybergear(253, 127)

# Power on the motor
motor_1.power_on()
time.sleep(2)

# Enable the motor
motor_1.enable_motor()
time.sleep(2)

# Send control command to move the motor with higher torque & speed
motor_1.control_motor(
    torque=8.0,         
    mech_position=2.0,  
    speed=10.0,         
    kp=50.0,            
    kd=2.0             
)

# Wait for movement
time.sleep(2)

# Read motor status TWICE to confirm actual movement
motor_1.read_motor_status()
time.sleep(1)
motor_1.read_motor_status()  # Read again

# Stop the motor
time.sleep(5)
motor_1.stop_motor()
