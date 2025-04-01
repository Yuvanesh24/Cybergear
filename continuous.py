import serial
import time
import struct
import threading
import serial.tools.list_ports


print("Available COM ports:", [port.device for port in serial.tools.list_ports.comports()])


ser = serial.Serial('COM3', 921600, timeout=5.0)  


frame_head = "4154"
frame_tail = "0d0a"

def read_response():
    """Reads response from the motor."""
    time.sleep(0.1) 
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)  
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
        self.running = False  
        self.read_thread = None  

    def power_on(self):
        """Send power-on signal."""
        command = '41542b41540d0a'
        ser.write(bytes.fromhex(command))  
        ser.flush()
        time.sleep(1)

    def enable_motor(self):
        """Enable motor (Communication Type 3)."""
        bin_num = (3 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        ser.write(bytes.fromhex(hex_can))
        ser.flush()

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
        ser.write(bytes.fromhex(hex_can))
        ser.flush()

    def stop_motor(self):
        """Stop motor (Communication Type 4)."""
        bin_num = (4 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        ser.write(bytes.fromhex(hex_can))
        ser.flush()

    def read_motor_position(self):
        """Reads CyberGear motor position from the correct byte offset."""
        ser.reset_input_buffer()

        # Send read position command (Communication Type 2)
        bin_num = (2 << 24) | (self.master << 8) | self.motor  
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail

        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        time.sleep(0.05)

        data = read_response()
        if data and len(data) >= 10:
            print(f"🔎 Full Motor Response: {data.hex()}")

            # Extract bytes at offset 8-9 for position
            position_bytes = data[8:10]  
            position_int = int.from_bytes(position_bytes, byteorder='big', signed=True)

            # Convert to degrees (-180 to 180 range)
            position_deg = (position_int / 32768.0) * 180.0  

            print(f"✅ Motor Position: {position_deg:.2f}°")
            return position_deg

        return None

    def continuous_position_read(self):
        """Continuously read motor position in a separate thread."""
        self.running = True
        last_reported_deg = None

        while self.running:
            current_deg = self.read_motor_position()

            if current_deg is not None:
                rounded_deg = round(current_deg)

                if last_reported_deg is None or rounded_deg != last_reported_deg:
                    print(f"Motor is at {rounded_deg} degrees")
                    last_reported_deg = rounded_deg

            time.sleep(0.2) 

    def start_continuous_reading(self):
        """Start reading position in a new thread."""
        if not self.read_thread or not self.read_thread.is_alive():
            self.read_thread = threading.Thread(target=self.continuous_position_read, daemon=True)
            self.read_thread.start()

    def stop_continuous_reading(self):
        """Stop the continuous reading thread."""
        self.running = False
        if self.read_thread:
            self.read_thread.join()

# Initialize motor
motor_1 = Cybergear(253, 127)

# Power on the motor
print("Powering on..")
motor_1.power_on()
time.sleep(1)

# Enable the motor
print("Enabling motor...")
motor_1.enable_motor()
time.sleep(1)

# Start reading motor position
print("Starting position monitoring...")
motor_1.start_continuous_reading()

# Control motor
print("Controlling motor...")
motor_1.control_motor(torque=0.5, mech_position=3.14, speed=0.2, kp=100.0, kd=2.0)
time.sleep(5)

# Stop motor
print("Stopping motor...")
motor_1.stop_motor()
motor_1.stop_continuous_reading()

print("Motor control completed.")
