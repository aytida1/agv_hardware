import serial
import crcmod
from serial.tools import list_ports
import time

class Feetech_motor:
    def __init__(self, serial_port = "/dev/ttyACM0", baud_rate = 115200):
        self.client = None
        self.serial_port = self.get_motor_serial_port(serial_port)
        self.baud_rate = baud_rate
        self.baud_rate_dict = {
            0: "256000",
            1: "128000",
            2: "115200",
            3: "57600",
            4: "56000",
            5: "38400",
            6: "19200",
            7: "14400",
            8: "9600",
            9: "4800"
        }
        self.initialize_connection()

    def generate_crc(self, data):
        crc_func = crcmod.mkCrcFun(0x18005, rev=True, initCrc=0xFFFF, xorOut=0x0000)
        crc_value = crc_func(bytes(data))
        crc_hex = format(crc_value, '04X')
        crc1 = int(crc_hex[0:2],16)
        crc2 = int(crc_hex[2:4],16)
        return crc2,crc1
    
    def get_motor_serial_port(self,serial_port):
        print("inside get motor srial port ========")
        dev = list_ports.comports()
        for i in dev:
            print(i.device)
            if serial_port in i.device_path:
                print(f"port found : {i.device}")
                return i.device
        return serial_port

    def initialize_connection(self):
        try:
            self.client = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
        except Exception as e:
            print(f"Error in serial connection!: {e}")
    
    def convert_int_to_byte_acceptable_range(self, input):
        if input >= 0:
            y = hex(input)
            # print(y)
            if len(y) == 6:
                byte1 = int(y[2:4], 16)
                byte2 = int(y[4:6], 16)
            elif len(y) == 5:
                byte1 = int(y[2:3], 16)
                byte2 = int(y[3:5], 16)
            elif len(y) == 4:
                byte1 = 0
                byte2 = int(y[2:4], 16)
            elif len(y) == 3:
                byte1 = 0
                byte2 = int(y[2:3], 16)
            return byte1, byte2
        else:
            y = hex(65535 - abs(input) + 1)
            print(y)
            if len(y) == 6:
                byte1 = int(y[2:4], 16)
                byte2 = int(y[4:6], 16)
            elif len(y) == 5:
                byte1 = int(y[2:3], 16)
                byte2 = int(y[3:5], 16)
            elif len(y) == 4:
                byte1 = 0
                byte2 = int(y[2:4], 16)
            elif len(y) == 3:
                byte1 = 0
                byte2 = int(y[2:3], 16)
            return byte1, byte2
    
    def serial_read(self, packet):
        try:
            self.client.reset_input_buffer()
            print(f'Entered serial_read function')
            self.client.write(packet)
            time.sleep(0.005)
            data = b''
            for i in range(7):
                data += self.client.read()
            data = bytes.hex(data)
            print(f'Data received from serial communication is {data}')
            data = data[6:10]
            print(f'Completed serial_read function')
            return data
        except Exception as e:
            print(f"serial_read: {e}")
            return 0
    
    def serial_write(self, packet):
        try:
            print(f'Entered serial_write function')
            self.client.write(packet)
            time.sleep(0.005)
            data = b''
            for i in range(7):
                data += self.client.read()
            data = bytes.hex(data)
            print(f'Data received from serial communication is {data}')
            data = data[8:12]
            print(f'Completed serial_write function')
            return data
        except Exception as e:
            print(f"serial_write: {e}")
            return 0

    def mm_to_pulse_converter(self, mm_position, ratio):
        pulse_position = ratio * mm_position
        return pulse_position
    
    def get_motor_id(self, current_motor_id):
        try:
            print(f'Entered get_motor_id function')
            packet = bytes([current_motor_id, 0x03, 0x00, 0x0A, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x00, 0x0A, 0x00, 0x01, crc1, crc2])
            m_id = self.serial_read(packet_with_checksum)
            m_id = int(m_id, 16)
            print(f'ID of the given motor is {m_id}')
            return m_id
        except Exception as e:
            print(f"get_motor_id: {e}")
            return 0
        
    def set_motor_id(self, current_motor_id, new_motor_id):
        try:
            print(f'Entered set_motor_id function')
            new_motor_id_byte1, new_motor_id_byte2 = self.convert_int_to_byte_acceptable_range(new_motor_id)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x0A, new_motor_id_byte1, new_motor_id_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x0A, new_motor_id_byte1, new_motor_id_byte2, crc1, crc2])
            new_id = self.serial_write(packet_with_checksum)
            new_id = int(new_id, 16)
            print(f'ID of the given motor changed to {new_id}')
        except Exception as e:
            print(f"set_motor_id: {e}")
    
    def get_baud_rate(self, current_motor_id):
        try:
            print(f'Entered get_baud_rate function')
            packet = bytes([current_motor_id, 0x03, 0x00, 0x0B, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x00, 0x0B, 0x00, 0x01, crc1, crc2])
            c_baudrate = self.serial_read(packet_with_checksum)
            c_baudrate = int(c_baudrate, 16)
            baudrate = self.baud_rate_dict[c_baudrate]
            print(f'Baud Rate of the given motor is {baudrate}')
            return baudrate
        except Exception as e:
            print(f"get_baud_rate: {e}")
            return 0
    
    def set_baud_rate(self, current_motor_id, new_baud_rate):
        try:
            print(f'Entered set_baud_rate function')
            new_baud_rate_byte1, new_baud_rate_byte2 = self.convert_int_to_byte_acceptable_range(new_baud_rate)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x0B, new_baud_rate_byte1, new_baud_rate_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x0B, new_baud_rate_byte1, new_baud_rate_byte2, crc1, crc2])
            new_baudrate = self.serial_write(packet_with_checksum)
            new_baudrate = int(new_baudrate, 16)
            print(f'Baud of the given motor changed to {new_baudrate}')
        except Exception as e:
            print(f"set_baud_rate: {e}")

    def get_motor_position(self, current_motor_id):
        try:
            print(f'Entered get_motor_position function')
            packet = bytes([current_motor_id, 0x03, 0x01, 0x01, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x01, 0x01, 0x00, 0x01, crc1, crc2])
            p_position = self.serial_read(packet_with_checksum)
            p_position = int(p_position, 16)
            print(f'Current position of the given motor is {p_position}')
            return p_position
        except Exception as e:
            print(f"get_motor_position: {e}")
            return 0
    
    def set_motor_position(self, current_motor_id, new_goal_position, position_type = "pulse", pulse_mm_ratio = 0):
        try:
            print(f'Entered set_motor_position function')
            if position_type == "mm":
                new_goal_position = self.mm_to_pulse_converter(new_goal_position, pulse_mm_ratio)
            new_goal_position_byte1, new_goal_position_byte2 = self.convert_int_to_byte_acceptable_range(new_goal_position)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x80, new_goal_position_byte1, new_goal_position_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x80, new_goal_position_byte1, new_goal_position_byte2, crc1, crc2])
            new_position = self.serial_write(packet_with_checksum)
            new_position = int(new_position, 16)
            print(f'Position of the given motor changed to {new_position}')
        except Exception as e:
            print(f"set_motor_position: {e}")
    
    def get_motor_velocity(self, current_motor_id):
        try:
            print(f'Entered get_motor_velocity function')
            packet = bytes([current_motor_id, 0x03, 0x01, 0x02, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x01, 0x02, 0x00, 0x01, crc1, crc2])
            p_velocity = self.serial_read(packet_with_checksum)
            p_velocity = int(p_velocity, 16)
            print(f'Current velocity of the given motor is {p_velocity}')
            return p_velocity
        except Exception as e:
            print(f"get_motor_velocity: {e}")
            return 0
    
    def set_motor_velocity(self, current_motor_id, new_goal_velocity):
        try:
            print(f'Entered set_motor_velocity function')
            new_goal_velocity_byte1, new_goal_velocity_byte2 = self.convert_int_to_byte_acceptable_range(new_goal_velocity)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x83, new_goal_velocity_byte1, new_goal_velocity_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x83, new_goal_velocity_byte1, new_goal_velocity_byte2, crc1, crc2])
            new_velocity = self.serial_write(packet_with_checksum)
            new_velocity = int(new_velocity, 16)
            print(f'Velocity of the given motor changed to {new_velocity}')
        except Exception as e:
            print(f"set_motor_velocity: {e}")

    def set_motor_accleration(self, current_motor_id, new_goal_accleration):
        try:
            print(f'Entered set_motor_acceleration function')
            new_goal_accleration_byte1, new_goal_accleration_byte2 = self.convert_int_to_byte_acceptable_range(new_goal_accleration)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x82, new_goal_accleration_byte1, new_goal_accleration_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x82, new_goal_accleration_byte1, new_goal_accleration_byte2, crc1, crc2])
            new_accleration = self.serial_write(packet_with_checksum)
            new_accleration = int(new_accleration, 16)
            print(f'Accleration of the given motor changed to {new_accleration}')
        except Exception as e:
            print(f"set_motor_accleration: {e}")
    
    def set_motor_offset(self, current_motor_id, new_motor_offset):
        try:
            print(f'set_motor_offset function')
            new_motor_offset_byte1, new_motor_offset_byte2 = self.convert_int_to_byte_acceptable_range(new_motor_offset)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x0F, new_motor_offset_byte1, new_motor_offset_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x0F, new_motor_offset_byte1, new_motor_offset_byte2, crc1, crc2])
            new_offset = self.serial_write(packet_with_checksum)
            new_offset = int(new_offset, 16)
            print(f'Offset of the given motor changed to {new_offset}')
        except Exception as e:
            print(f"set_motor_offset: {e}")

    def set_motor_max_torque(self, current_motor_id, new_motor_max_torque):
        try:
            print(f'set_motor_max_torque function')
            new_motor_max_torque_byte1, new_motor_max_torque_byte2 = self.convert_int_to_byte_acceptable_range(new_motor_max_torque)
            packet = bytes([current_motor_id, 0x06, 0x00, 0x84, new_motor_max_torque_byte1, new_motor_max_torque_byte2])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x84, new_motor_max_torque_byte1, new_motor_max_torque_byte2, crc1, crc2])
            new_max_torque = self.serial_write(packet_with_checksum)
            new_max_torque = int(new_max_torque, 16)
            print(f'Max torque limit of the given motor changed to {new_max_torque}')
        except Exception as e:
            print(f"set_motor_max_torque: {e}")

    def get_motor_moving_status(self, current_motor_id):
        try:
            print(f'Entered get_motor_moving_status function')
            packet = bytes([current_motor_id, 0x03, 0x01, 0x06, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x01, 0x06, 0x00, 0x01, crc1, crc2])
            moving_status = self.serial_read(packet_with_checksum)
            moving_status = int(moving_status, 16)
            if moving_status == 1:
                p_moving_status = True
            else:
                p_moving_status = False
            print(f'Moving Status of the given motor is {p_moving_status}')
            return p_moving_status
        except Exception as e:
            print(f"get_motor_moving_status: {e}")
            return False
    
    def get_motor_hardware_error_status(self, current_motor_id):
        try:
            print(f'Entered get_motor_hardware_error_status function')
            packet = bytes([current_motor_id, 0x03, 0x01, 0x00, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x03, 0x01, 0x00, 0x00, 0x01, crc1, crc2])
            hardware_error_status = self.serial_read(packet_with_checksum)
            hardware_error_status = int(hardware_error_status, 16)
            print(f'Hardware Error Status of the given motor is {hardware_error_status}')
            return hardware_error_status
        except Exception as e:
            print(f"get_motor_hardware_error_status: {e}")
            return 0
        
    def motor_torque_enable(self, current_motor_id):
        try:
            print(f'Entered motor_torque_enable function')
            packet = bytes([current_motor_id, 0x06, 0x00, 0x81, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x81, 0x00, 0x01, crc1, crc2])
            torque_status = self.serial_write(packet_with_checksum)
            torque_status = int(torque_status, 16)
            print(f'Torque Status of the given motor is {torque_status}')
        except Exception as e:
            print(f"motor_torque_enable: {e}")

    def motor_torque_disable(self, current_motor_id):
        try:
            print(f'Entered motor_torque_disable function')
            packet = bytes([current_motor_id, 0x06, 0x00, 0x81, 0x00, 0x00])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x81, 0x00, 0x00, crc1, crc2])
            torque_status = self.serial_write(packet_with_checksum)
            torque_status = int(torque_status, 16)
            print(f'Torque Status of the given motor is {torque_status}')
        except Exception as e:
            print(f"motor_torque_disable: {e}")

    def motor_work_mode_enable(self, current_motor_id):
        try:
            print(f'Entered motor_work_mode_enable function')
            packet = bytes([current_motor_id, 0x06, 0x00, 0x10, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x10, 0x00, 0x01, crc1, crc2])
            work_mode_status = self.serial_write(packet_with_checksum)
            print(f'Work Mode Status of the given motor is {work_mode_status}')
        except Exception as e:
            print(f"motor_work_mode_enable: {e}")
    
    def motor_work_mode_disable(self, current_motor_id):
        try:
            print(f'Entered motor_work_mode_disable function')
            packet = bytes([current_motor_id, 0x06, 0x00, 0x10, 0x00, 0x00])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x10, 0x00, 0x00, crc1, crc2])
            work_mode_status = self.serial_write(packet_with_checksum)
            print(f'Work Mode Status of the given motor is {work_mode_status}')
        except Exception as e:
            print(f"motor_work_mode_disable: {e}")

    def motor_error_reset(self, current_motor_id):
        try:
            print(f'Entered motor_error_reset function')
            packet = bytes([current_motor_id, 0x06, 0x00, 0x86, 0x00, 0x01])
            crc1, crc2 = self.generate_crc(packet)
            packet_with_checksum = bytes([current_motor_id, 0x06, 0x00, 0x86, 0x00, 0x01, crc1, crc2])
            error_reset = self.serial_write(packet_with_checksum)
            print(f'Motor Error Reset Done')
        except Exception as e:
            print(f"motor_error_reset: {e}")