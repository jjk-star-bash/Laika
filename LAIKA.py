# Edited by Joshua Kraus for Lika Project @ Lehigh University Rocketry 
# ORCID 0009-0006-3163-3312
# APACHE 2.0 LICENSE PLEASE READ & CITE


import serial
import struct
import time

__version__ = '2.0.7'
__last_modified__ = '05/30/25'

"""
ORDER is used to store the command address and corresponding data
"""
ORDER = {
    "BATTERY": [0x01, 100],
    "PERFORM": [0x03, 0],
    "CALIBRATION": [0x04, 0],
    "UPGRADE": [0x05, 0],
    "MOVE_TEST": [0x06, 1],
    "FIRMWARE_VERSION": [0x07],
    "GAIT_TYPE": [0x09, 0x00],
    "BT_NAME": [0x13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "UNLOAD_MOTOR": [0x20, 0],
    "LOAD_MOTOR": [0x20, 0],
    "VX": [0x30, 128],
    "VY": [0x31, 128],
    "VYAW": [0x32, 128],
    "TRANSLATION": [0x33, 0, 0, 0],
    "ATTITUDE": [0x36, 0, 0, 0],
    "PERIODIC_ROT": [0x39, 0, 0, 0],
    "MarkTime": [0x3C, 0],
    "MOVE_MODE": [0x3D, 0],
    "ACTION": [0x3E, 0],
    "PERIODIC_TRAN": [0x80, 0, 0, 0],
    "MOTOR_ANGLE": [0x50, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128],
    "MOTOR_SPEED": [0x5C, 1],
    "LEG_POS": [0x40, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
    "IMU": [0x61, 0],
    "ROLL": [0x62, 0],
    "PITCH": [0x63, 0],
    "YAW": [0x64, 0]
}

"""
PARAM is used to store the parameter limit range of the robot dog
"""

PARAM = {
    "TRANSLATION_LIMIT": [35, 18, [75, 115]], # X symetric Y symetric Z lower bound, upper bound (Scope of translation)
    "ATTITUDE_LIMIT": [20, 15, 11],           # Roll Pitch Yaw (Scope of posture)
    "LEG_LIMIT": [35, 18, [75, 115]],         # Scope of the leg
    "MOTOR_LIMIT": [[-73, 57], [-66, 93], [-31, 31]], # Lower, middle and upper steering gear range
    "PERIOD_LIMIT": [[1.5, 8]],
    "MARK_TIME_LIMIT": [10, 35],  # Stationary height range (of the feet marching)
    "VX_LIMIT": 25,    #  X velocity range
    "VY_LIMIT": 18,    #  Y velocity range
    "VYAW_LIMIT": 100  # Rotation speed range
}


def search(data, list):
    for i in range(len(list)):
        if data == list[i]:
            return i + 1
    return -1


def conver2u8(data, limit, mode=0):
    """
    Convert the actual parameters to single byte data from 0 to 255
    """
    max = 0xff
    if mode == 0:
        min = 0x00
    else:
        min = 0x01

    if not isinstance(limit, list):
        if data >= limit:
            return max
        elif data <= -limit:
            return min
        else:
            return int(128 + 128 * data / limit)
    else:
        limitmin = limit[0]
        limitmax = limit[1]
        if data >= limitmax:
            return max
        elif data <= limitmin:
            return min
        else:
            return int(255 / (limitmax - limitmin) * (data - limitmin))


def conver2float(data, limit):
    if not isinstance(limit, list):
        return (data - 128.0) / 255.0 * limit
    else:
        limitmin = limit[0]
        limitmax = limit[1]
        return data / 255.0 * (limitmax - limitmin) + limitmin


def Byte2Float(rawdata):
    a = bytearray()
    a.append(rawdata[3])
    a.append(rawdata[2])
    a.append(rawdata[1])
    a.append(rawdata[0])
    return struct.unpack("!f", a)[0]


class Laika():
    """
    When initilizing Laika, you need to specify the serial
    communication interface between the upper computer (a Rpi5) and the machine dog interface for the servos
    We are doing this over the uart bus connection via the gpio pins on the pi
    """

    def __init__(self, port="/dev/ttyAMA0"):
        self.ser = serial.Serial(port, 115200, timeout=0.5)
        self.rx_FLAG = 0
        self.rx_COUNT = 0
        self.rx_ADDR = 0
        self.rx_LEN = 0
        self.rx_data = bytearray(50)
        self.__delay = 0.05
        pass

    def __send(self, key, index=1, len=1):
        mode = 0x01
        order = ORDER[key][0] + index - 1
        value = []
        value_sum = 0
        for i in range(0, len):
            value.append(ORDER[key][index + i])
            value_sum = value_sum + ORDER[key][index + i]
        sum_data = ((len + 0x08) + mode + order + value_sum) % 256
        sum_data = 255 - sum_data
        tx = [0x55, 0x00, (len + 0x08), mode, order]
        tx.extend(value)
        tx.extend([sum_data, 0x00, 0xAA])
        self.ser.write(tx)

    def __read(self, addr, read_len=1):
        mode = 0x02
        sum_data = (0x09 + mode + addr + read_len) % 256
        sum_data = 255 - sum_data
        tx = [0x55, 0x00, 0x09, mode, addr, read_len, sum_data, 0x00, 0xAA]
        # time.sleep(0.1)
        self.ser.flushInput()
        self.ser.write(tx)

    def stop(self):
        self.move_x(0)
        self.move_y(0)
        self.mark_time(0)
        self.turn(0)

    def move(self, direction, step):
        if direction in ['x', 'X']:
            self.move_x(step)
        elif direction in ['y', 'Y']:
            self.move_y(step)
        else:
            print("ERROR!Invalid direction!")

    def move_x(self, step):
        if step > 20:
            step = 20
        elif step < -20:
            step = -20
        ORDER["VX"][1] = conver2u8(step, PARAM["VX_LIMIT"])
        self.__send("VX")

    def move_y(self, step):
        if step > 18:
            step = 18
        elif step < -18:
            step = -18
        ORDER["VY"][1] = conver2u8(step, PARAM["VY_LIMIT"])
        self.__send("VY")

    def turn(self, step):
        if step > 70:
            step = 70
        elif step < -70:
            step = -70
        elif 0 < step < 30:
            step = 30
        elif -30 < step < 0:
            step = -30
        ORDER["VYAW"][1] = conver2u8(step, PARAM["VYAW_LIMIT"])
        self.__send("VYAW")

    def forward(self, step):
        self.move_x(abs(step))

    def back(self, step):
        self.move_x(-abs(step))

    def left(self, step):
        self.move_y(abs(step))

    def right(self, step):
        self.move_y(-abs(step))

    def turnleft(self, step):
        self.turn(abs(step))

    def turnright(self, step):
        self.turn(-abs(step))

    def __translation(self, direction, data):
        index = search(direction, ['x', 'y', 'z'])
        if index == -1:
            print("ERROR!Direction must be 'x', 'y' or 'z'")
            return
        ORDER["TRANSLATION"][index] = conver2u8(data, PARAM["TRANSLATION_LIMIT"][index - 1])
        self.__send("TRANSLATION", index)

    def translation(self, direction, data):
        """
        Keep the robot's feet stationary and the body makes three-axis translation
        """
        if (isinstance(direction, list)):
            if (len(direction) != len(data)):
                print("ERROR!The length of direction and data don't match!")
                return
            for i in range(len(data)):
                self.__translation(direction[i], data[i])
        else:
            self.__translation(direction, data)

    def __attitude(self, direction, data):
        index = search(direction, ['r', 'p', 'y'])
        if index == -1:
            print("ERROR!Direction must be 'r', 'p' or 'y'")
            return
        ORDER["ATTITUDE"][index] = conver2u8(data, PARAM["ATTITUDE_LIMIT"][index - 1])
        self.__send("ATTITUDE", index)

    def attitude(self, direction, data):
        """
        Keep the robot's feet stationary and the body makes three-axis rotation
        """
        if (isinstance(direction, list)):
            if (len(direction) != len(data)):
                print("ERROR!The length of direction and data don't match!")
                return
            for i in range(len(data)):
                self.__attitude(direction[i], data[i])
        else:
            self.__attitude(direction, data)

    def action(self, action_id):
        """
        Make the robot do the specified preset action
        """
        if action_id <= 0 or action_id > 255:
            print("ERROR!Illegal Action ID!")
            return
        ORDER["ACTION"][1] = action_id
        self.__send("ACTION")

    def reset(self):
        """
        The robot dog stops moving and all parameters return to the initial state
        """
        self.action(255)
        time.sleep(0.2)

    def leg(self, leg_id, data):
        """
        Control the three-axis movement of a single leg of the robot
        """
        value = [0, 0, 0]
        if leg_id not in [1, 2, 3, 4]:
            print("Error!Illegal Index!")
            return
        if len(data) != 3:
            message = "Error!Illegal Value!"
            return
        for i in range(3):
            try:
                value[i] = conver2u8(data[i], PARAM["LEG_LIMIT"][i])
            except:
                print("Error!Illegal Value!")
        for i in range(3):
            index = 3 * (leg_id - 1) + i + 1
            ORDER["LEG_POS"][index] = value[i]
            self.__send("LEG_POS", index)

    def __motor(self, index, data):
        ORDER["MOTOR_ANGLE"][index] = conver2u8(data, PARAM["MOTOR_LIMIT"][index % 3 - 1])
        self.__send("MOTOR_ANGLE", index)

    def motor(self, motor_id, data):
        """
        Control the rotation of a single steering gear of the robot
        """
        MOTOR_ID = [11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]
        if isinstance(motor_id, list):
            if len(motor_id) != len(data):
                print("Error!Length Mismatching!")
                return
            index = []
            for i in range(len(motor_id)):
                temp_index = search(motor_id[i], MOTOR_ID)
                if temp_index == -1:
                    print("Error!Illegal Index!")
                    return
                index.append(temp_index)
            for i in range(len(index)):
                self.__motor(index[i], data[i])
        else:
            index = search(motor_id, MOTOR_ID)
            self.__motor(index, data)

    def unload_motor(self, leg_id):
        if leg_id not in [1, 2, 3, 4]:
            print('ERROR!leg_id must be 1, 2, 3 or 4')
            return
        ORDER["UNLOAD_MOTOR"][1] = 0x10 + leg_id
        self.__send("UNLOAD_MOTOR")

    def unload_allmotor(self):
        ORDER["UNLOAD_MOTOR"][1] = 0x01
        self.__send("UNLOAD_MOTOR")

    def load_motor(self, leg_id):
        if leg_id not in [1, 2, 3, 4]:
            print('ERROR!leg_id must be 1, 2, 3 or 4')
            return
        ORDER["LOAD_MOTOR"][1] = 0x20 + leg_id
        self.__send("LOAD_MOTOR")

    def load_allmotor(self):
        ORDER["LOAD_MOTOR"][1] = 0x00
        self.__send("LOAD_MOTOR")

    def __periodic_rot(self, direction, period):
        index = search(direction, ['r', 'p', 'y'])
        if index == -1:
            print("ERROR!Direction must be 'r', 'p' or 'y'")
            return
        if period == 0:
            ORDER["PERIODIC_ROT"][index] = 0
        else:
            ORDER["PERIODIC_ROT"][index] = conver2u8(period, PARAM["PERIOD_LIMIT"][0], mode=1)
        self.__send("PERIODIC_ROT", index)

    def periodic_rot(self, direction, period):
        """
        Make the robot rotate periodically
        """
        if (isinstance(direction, list)):
            if (len(direction) != len(period)):
                print("ERROR!The length of direction and data don't match!")
                return
            for i in range(len(period)):
                self.__periodic_rot(direction[i], period[i])
        else:
            self.__periodic_rot(direction, period)

    def __periodic_tran(self, direction, period):
        index = search(direction, ['x', 'y', 'z'])
        if index == -1:
            print("ERROR!Direction must be 'x', 'y' or 'z'")
            return
        if period == 0:
            ORDER["PERIODIC_TRAN"][index] = 0
        else:
            ORDER["PERIODIC_TRAN"][index] = conver2u8(period, PARAM["PERIOD_LIMIT"][0], mode=1)
        self.__send("PERIODIC_TRAN", index)

    def periodic_tran(self, direction, period):
        """
        Make the robot translate periodically
        """
        if (isinstance(direction, list)):
            if (len(direction) != len(period)):
                print("ERROR!The length of direction and data don't match!")
                return
            for i in range(len(period)):
                self.__periodic_tran(direction[i], period[i])
        else:
            self.__periodic_tran(direction, period)

    def mark_time(self, data):
        """
        Make the robot marks time
        """
        if data == 0:
            ORDER["MarkTime"][1] = 0
        else:
            ORDER["MarkTime"][1] = conver2u8(data, PARAM["MARK_TIME_LIMIT"], mode=1)
        self.__send("MarkTime")

    def pace(self, mode):
        """
        Change the step frequency of the robot
        """
        if mode == "normal":
            value = 0x00
        elif mode == "slow":
            value = 0x01
        elif mode == "high":
            value = 0x02
        else:
            print("ERROR!Illegal Value!")
            return
        ORDER["MOVE_MODE"][1] = value
        self.__send("MOVE_MODE")

    def gait_type(self, mode):
        """
        Change the gait of the robot
        """
        if mode == "trot":
            value = 0x00
        elif mode == "walk":
            value = 0x01
        elif mode == "high_walk":
            value = 0x02
        ORDER["GAIT_TYPE"][1] = value
        self.__send("GAIT_TYPE")

    def imu(self, mode):
        """
        Turn on / off the self stable state of the robot dog
        """
        if mode != 0 and mode != 1:
            print("ERROR!Illegal Value!")
            return
        ORDER["IMU"][1] = mode
        self.__send("IMU")

    def perform(self, mode):
        """
        Turn on / off the action status of the robot dog cycle
        """
        if mode != 0 and mode != 1:
            print("ERROR!Illegal Value!")
            return
        ORDER["PERFORM"][1] = mode
        self.__send("PERFORM")

    def motor_speed(self, speed):
        """
        Adjust the steering gear rotation speed,
        only effective when control the steering gear separately
        """
        if speed < 0 or speed > 255:
            print("ERROR!Illegal Value!The speed parameter needs to be between 0 and 255!")
            return
        if speed == 0:
            speed = 1
        ORDER["MOTOR_SPEED"][1] = speed
        self.__send("MOTOR_SPEED")

    def read_motor(self, out_int=False):
        """
        Read the angles of the 12 steering gear
        """
        self.__read(ORDER["MOTOR_ANGLE"][0], 12)
        time.sleep(self.__delay)
        angle = []
        if self.__unpack():
            for i in range(12):
                index = round(conver2float(self.rx_data[i], PARAM["MOTOR_LIMIT"][i % 3]), 2)
                if out_int:
                    if index > 0:
                        angle.append(int(index+0.5))
                    elif index < 0:
                        angle.append(int(index-0.5))
                    else:
                        angle.append(int(index))
                else:
                    angle.append(index)
        return angle

    def read_battery(self):
        self.__read(ORDER["BATTERY"][0], 1)
        time.sleep(self.__delay)
        battery = 0
        if self.__unpack():
            battery = int(self.rx_data[0])
        return battery

    def read_version(self):
        self.__read(ORDER["FIRMWARE_VERSION"][0], 10)
        time.sleep(self.__delay)
        firmware_version = 'Null'
        if self.__unpack():
            # data = self.rx_data[0:10]
            data = self.rx_data[2:10]
            firmware_version = data.decode("utf-8").strip('\0')
        return firmware_version

    def read_roll(self, out_int=False):
        self.__read(ORDER["ROLL"][0], 4)
        time.sleep(self.__delay)
        roll = 0
        if self.__unpack():
            roll = Byte2Float(self.rx_data)
        if out_int:
            tmp = int(roll)
            return tmp
        return round(roll, 2)

    def read_pitch(self, out_int=False):
        self.__read(ORDER["PITCH"][0], 4)
        time.sleep(self.__delay)
        pitch = 0
        if self.__unpack():
            pitch = Byte2Float(self.rx_data)
        if out_int:
            tmp = int(pitch)
            return tmp
        return round(pitch, 2)

    def read_yaw(self, out_int=False):
        self.__read(ORDER["YAW"][0], 4)
        time.sleep(self.__delay)
        yaw = 0
        if self.__unpack():
            yaw = Byte2Float(self.rx_data)
        if out_int:
            tmp = int(yaw)
            return tmp
        return round(yaw, 2)

    def __unpack(self):
        n = self.ser.inWaiting()
        rx_CHECK = 0
        if n:
            data = self.ser.read(n)
            for num in data:
                if self.rx_FLAG == 0:
                    if num == 0x55:
                        self.rx_FLAG = 1
                    else:
                        self.rx_FLAG = 0

                elif self.rx_FLAG == 1:
                    if num == 0x00:
                        self.rx_FLAG = 2
                    else:
                        self.rx_FLAG = 0

                elif self.rx_FLAG == 2:
                    self.rx_LEN = num
                    self.rx_FLAG = 3

                elif self.rx_FLAG == 3:
                    self.rx_TYPE = num
                    self.rx_FLAG = 4

                elif self.rx_FLAG == 4:
                    self.rx_ADDR = num
                    self.rx_FLAG = 5

                elif self.rx_FLAG == 5:
                    if self.rx_COUNT == (self.rx_LEN - 9):
                        self.rx_data[self.rx_COUNT] = num
                        self.rx_COUNT = 0
                        self.rx_FLAG = 6
                    elif self.rx_COUNT < self.rx_LEN - 9:
                        self.rx_data[self.rx_COUNT] = num
                        self.rx_COUNT = self.rx_COUNT + 1

                elif self.rx_FLAG == 6:
                    for i in self.rx_data[0:(self.rx_LEN - 8)]:
                        rx_CHECK = rx_CHECK + i
                    rx_CHECK = 255 - (self.rx_LEN + self.rx_TYPE + self.rx_ADDR + rx_CHECK) % 256
                    if num == rx_CHECK:
                        self.rx_FLAG = 7
                    else:
                        self.rx_FLAG = 0
                        self.rx_COUNT = 0
                        self.rx_ADDR = 0
                        self.rx_LEN = 0

                elif self.rx_FLAG == 7:
                    if num == 0x00:
                        self.rx_FLAG = 8
                    else:
                        self.rx_FLAG = 0
                        self.rx_COUNT = 0
                        self.rx_ADDR = 0
                        self.rx_LEN = 0

                elif self.rx_FLAG == 8:
                    if num == 0xAA:
                        self.rx_FLAG = 0
                        self.rx_COUNT = 0
                        return True
                    else:
                        self.rx_FLAG = 0
                        self.rx_COUNT = 0
                        self.rx_ADDR = 0
                        self.rx_LEN = 0
        return False

    def calibration(self, state):
        """
        For software calibration, please use with caution!!!
        """
        if state:
            ORDER["CALIBRATION"][1] = 1
        else:
            ORDER["CALIBRATION"][1] = 0
        self.__send("CALIBRATION")


if __name__ == '__main__':
    g_dog = Laika()
    version = g_dog.read_version()
    print("version:", version)
