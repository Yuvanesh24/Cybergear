import serial
import time
import struct
import serial.tools.list_ports

# Check available COM ports
print("Available COM ports:", [port.device for port in serial.tools.list_ports.comports()])

try:
    ser = serial.Serial('COM3', 921600, timeout=5.0)  
except serial.SerialException as e:
    print("ERROR: Could not open COM3. Check permissions or device connection.")
    print(e)
    exit(1)

frame_head = "4154"
frame_tail = "0d0a"

def read_response():
    """Reads response from the motor."""
    time.sleep(0.5)  
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)  
        print(data)
        print("Motor Response:", data.hex())
    else:
        print("No response received.")

def float_to_ieee754(f):
    """Converts a float to IEEE 754 hex format."""
    return struct.pack(">f", f).hex()

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
        """Enable motor."""
        bin_num = (3 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "080000000000000000" + frame_tail
        print("Sending Enable Motor Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def set_run_mode(self, mode):
        """Set the motor's run mode before sending control commands."""
        mode_map = {"velocity": 0x08, "position": 0x10, "torque": 0x04}
        if mode not in mode_map:
            print("Invalid mode!")
            return
        bin_num = (7 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")
        hex_can = frame_head + hex_str + "08" + format(mode_map[mode], "02x") + "00000000000000" + frame_tail
        print(f"Setting Run Mode: {mode} ({hex_can})")
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def send_motor_sdo_float(self, index, value):
        """Send MOTOR_SDO_FLOAT command."""
        index_hex = format(index, "04x")  # 2-byte index
        value_hex = float_to_ieee754(value)  # 4-byte IEEE 754 float
        hex_can = frame_head + "70" + index_hex + value_hex + frame_tail
        print(f"Sending MOTOR_SDO_FLOAT: index = {index}, value = {value} ({hex_can})")
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def control_motor(self, torque, mech_position, speed, kp, kd):
        """Control the motor."""
        bin_num = (1 << 24) | (self.master << 8) | self.motor
        bin_num = (bin_num << 3) | 0b100
        hex_str = format(bin_num, "08x")

        tx_data = [
            float_to_ieee754(mech_position),
            float_to_ieee754(speed),
            float_to_ieee754(kp),
            float_to_ieee754(kd),
        ]

        tx_data_hex = "".join(tx_data)
        hex_can = frame_head + hex_str + "08" + tx_data_hex + frame_tail

        print("Sending Motor Control Command:", hex_can)
        ser.write(bytes.fromhex(hex_can))
        ser.flush()
        read_response()

    def stop_motor(self):
        """Stop motor."""
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

motor_1.power_on()
time.sleep(2)
motor_1.enable_motor()
time.sleep(2)

# Set mode before control commands
motor_1.set_run_mode("velocity")  
time.sleep(2)

# Set necessary parameters before control
motor_1.send_motor_sdo_float(28696, 23.0)  # Example parameter
motor_1.send_motor_sdo_float(28682, 10.18) # Example parameter

time.sleep(2)

# Send control command
motor_1.control_motor(torque=0.0, mech_position=0.0, speed=0.0, kp=0.0, kd=0.0)

time.sleep(5)
motor_1.stop_motor()
