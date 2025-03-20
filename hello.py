import threading
import serial
import time
import sys

# Initialize Serial Communication
ser = serial.Serial('COM3', 921600, timeout=5.0)

frame_head = "4154"
frame_tail = "0d0a"

def read_response():
    """Reads response from the motor and prints raw data."""
    time.sleep(0.2)  
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        sys.stdout.write(f"Raw Response: {data.hex()}\n")
        sys.stdout.flush()
        return data
    return None

def float_to_uint(x, x_min, x_max, bits):
    """Convert a float to an unsigned integer within the specified range."""
    span = x_max - x_min
    offset = x_min
    x = max(min(x, x_max), x_min)  
    return int((x - offset) * ((1 << bits) - 1) / span)

class Cybergear:
    def __init__(self, master_can, motor_can):
        self.master = master_can
        self.motor = motor_can
        self.running = True  # Thread control flag

    def power_on(self):
        """Send power-on signal to motor."""
        command = '41542b41540d0a'
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
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def control_motor(self, torque, mech_position, speed, kp, kd, absolute=True):
        """Send motor control commands (Communication Type 1)."""
        bin_num = (1 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")

        # Convert float values to uint16
        position_min, position_max = (-3.1416, 3.1416) if absolute else (-4 * 3.1416, 4 * 3.1416)
        
        tx_data = [
            float_to_uint(mech_position, position_min, position_max, 16) >> 8,
            float_to_uint(mech_position, position_min, position_max, 16) & 0xFF,
            float_to_uint(speed, -30.0, 30.0, 16) >> 8,
            float_to_uint(speed, -30.0, 30.0, 16) & 0xFF,
            float_to_uint(kp, 0.0, 500.0, 16) >> 8,
            float_to_uint(kp, 0.0, 500.0, 16) & 0xFF,
            float_to_uint(kd, 0.0, 5.0, 16) >> 8,
            float_to_uint(kd, 0.0, 5.0, 16) & 0xFF,
        ]

        tx_data_hex = "".join(format(x, "02x") for x in tx_data)
        hex_can = frame_head + hex_str + "08" + tx_data_hex + frame_tail

        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def read_motor_status(self):
        """Request and read motor position, velocity, and torque."""
        ser.reset_input_buffer()
        bin_num = (2 << 24) | (self.master << 8) | self.motor  
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail

        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        time.sleep(0.2)
        data = read_response()

        if data and len(data) >= 8:
            position_raw = int.from_bytes(data[0:2], byteorder='big', signed=True)
            velocity_raw = int.from_bytes(data[2:4], byteorder='big', signed=True)
            torque_raw = int.from_bytes(data[4:6], byteorder='big', signed=True)

            position = (position_raw / 65535) * (3.1416)  # Adjust scaling
            velocity = (velocity_raw / 65535) * 30.0
            torque = (torque_raw / 65535) * 10.0  # Adjust based on motor specs

            sys.stdout.write(f"Position: {position:.4f} rad, Velocity: {velocity:.2f} rad/s, Torque: {torque:.2f} Nm\n")
            sys.stdout.flush()
            return position, velocity, torque
        return None, None, None

    def continuous_position_read(self):
        """Thread function to continuously read motor status."""
        while self.running:
            self.read_motor_status()
            time.sleep(1)

    def stop_motor(self):
        """Stop motor (Communication Type 4)."""
        bin_num = (4 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def stop_now(self):
        """Emergency Stop - Immediately stops motor movement."""
        sys.stdout.write("🚨 Emergency Stop Triggered! 🚨\n")
        sys.stdout.flush()
        self.stop_motor()
        self.running = False

    def stop_thread(self):
        """Stops the continuous position reading thread."""
        self.running = False


motor_1 = Cybergear(253, 127)

motor_1.power_on()
time.sleep(2)

motor_1.enable_motor()
time.sleep(2)

position_thread = threading.Thread(target=motor_1.continuous_position_read)
position_thread.start()
motor_1.control_motor(10, 3.14, 5.0, 300.0, 2.0)

time.sleep(5)

position, velocity, torque = motor_1.read_motor_status()
print(f"Current Position: {position:.4f} rad")


motor_1.stop_motor()
motor_1.stop_thread()
position_thread.join()

sys.stdout.write("Motor stopped. Monitoring ended.\n")
sys.stdout.flush()
