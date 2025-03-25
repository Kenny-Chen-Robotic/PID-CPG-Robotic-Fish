# coding:UTF-8
#!/usr/bin/env python

import sys
sys.path.append(r"/home/pi/Desktop/ping-python")
sys.path.append(r"/home/pi/Desktop/ms5837-python-master")
from numpy import *
import smbus
import serial
import ms5837
from brping import Ping1D
import time
import RPi.GPIO as GPIO
import datetime

# ----------------- 硬件初始化 -----------------
GPIO.setmode(GPIO.BOARD)  # 物理引脚编号
GPIO.setup(11, GPIO.OUT)  # 蜂鸣器引脚
time.sleep(5)  # 等待系统稳定

class Pid():
    def __init__(self, exp_val, kp, ki, kd, value):
        self.KP = kp
        self.KI = ki
        self.KD = kd
        self.exp_val = exp_val
        self.control_val = 0
        self.value = value
        self.now_err = 0
        self.last_err = 0
        self.last_last_err = 0
        self.change_val = 0

    def cmd_pid(self):
        self.last_last_err = self.last_err
        self.last_err = self.now_err
        self.now_err = self.exp_val - self.value
        self.change_val = self.KP*(self.now_err - self.last_err) + \
                          self.KI*self.now_err + self.KD*(self.now_err - 2*self.last_err + self.last_last_err)
        self.control_val += self.change_val
        return self.control_val


# 陀螺仪数据
class Gyro(object):
    def __init__(self, addr, data):
        self.addr = addr
        self.i2c = smbus.SMBus(1)
        self.data = data

    def calibration(self):
        self.i2c.write_i2c_block_data(self.addr, 0x69, [0x88, 0xB5])
        time.sleep(1)
        self.i2c.write_i2c_block_data(self.addr, 0x01, [0x01, 0x00])
        time.sleep(6)

    def axis_set(self):
        self.i2c.write_i2c_block_data(self.addr, 0x69, [0x88, 0xB5])
        time.sleep(1)
        if self.data == 9:
            self.i2c.write_i2c_block_data(self.addr, 0x24, [0x00, 0x00])
        elif self.data == 6:
            self.i2c.write_i2c_block_data(self.addr, 0x24, [0x01, 0x00])
        time.sleep(1)

    def get_acc(self):
        try:
            self.raw_acc_x = self.i2c.read_i2c_block_data(self.addr, 0x34, 2)
            self.raw_acc_y = self.i2c.read_i2c_block_data(self.addr, 0x35, 2)
            self.raw_acc_z = self.i2c.read_i2c_block_data(self.addr, 0x36, 2)
        except IOError:
            print("ReadError: gyro_acc")
            return (0, 0, 0)
        else:
            self.k_acc = 16 * 9.8
            self.acc_x = (self.raw_acc_x[1] << 8 | self.raw_acc_x[0]) / 32768 * self.k_acc
            self.acc_y = (self.raw_acc_y[1] << 8 | self.raw_acc_y[0]) / 32768 * self.k_acc
            self.acc_z = (self.raw_acc_z[1] << 8 | self.raw_acc_z[0]) / 32768 * self.k_acc
            if self.acc_x >= self.k_acc:
                self.acc_x -= 2 * self.k_acc
            if self.acc_y >= self.k_acc:
                self.acc_y -= 2 * self.k_acc
            if self.acc_z >= self.k_acc:
                self.acc_z -= 2 * self.k_acc
            return (self.acc_x, self.acc_y, self.acc_z)

    def get_gyro(self):
        try:
            self.raw_gyro_x = self.i2c.read_i2c_block_data(self.addr, 0x37, 2)
            self.raw_gyro_y = self.i2c.read_i2c_block_data(self.addr, 0x38, 2)
            self.raw_gyro_z = self.i2c.read_i2c_block_data(self.addr, 0x39, 2)
        except IOError:
            print("ReadError: gyro_gyro")
            return (0, 0, 0)
        else:
            self.k_gyro = 2000
            self.gyro_x = (self.raw_gyro_x[1] << 8 | self.raw_gyro_x[0]) / 32768 * self.k_gyro
            self.gyro_y = (self.raw_gyro_y[1] << 8 | self.raw_gyro_y[0]) / 32768 * self.k_gyro
            self.gyro_z = (self.raw_gyro_z[1] << 8 | self.raw_gyro_z[0]) / 32768 * self.k_gyro
            if self.gyro_x >= self.k_gyro:
                self.gyro_x -= 2 * self.k_gyro
            if self.gyro_y >= self.k_gyro:
                self.gyro_y -= 2 * self.k_gyro
            if self.gyro_z >= self.k_gyro:
                self.gyro_z -= 2 * self.k_gyro
            return (self.gyro_z)

    def get_angle(self):
        try:
            self.raw_angle_x = self.i2c.read_i2c_block_data(self.addr, 0x3d, 2)
            self.raw_angle_y = self.i2c.read_i2c_block_data(self.addr, 0x3e, 2)
            self.raw_angle_z = self.i2c.read_i2c_block_data(self.addr, 0x3f, 2)
        except IOError:
            print("ReadError: gyro_angle")
            return (0, 0, 0)
        else:
            self.k_angle = 180
            self.angle_x = (self.raw_angle_x[1] << 8 | self.raw_angle_x[0]) / 32768 * self.k_angle
            self.angle_y = (self.raw_angle_y[1] << 8 | self.raw_angle_y[0]) / 32768 * self.k_angle
            self.angle_z = (self.raw_angle_z[1] << 8 | self.raw_angle_z[0]) / 32768 * self.k_angle
            if self.angle_x >= self.k_angle:
                self.angle_x -= 2 * self.k_angle
            if self.angle_y >= self.k_angle:
                self.angle_y -= 2 * self.k_angle
            if self.angle_z >= self.k_angle:
                self.angle_z -= 2 * self.k_angle
            return (self.angle_z)

# ----------------- 字符串处理 -----------------
def numberjudge(data):
    char_map = {
        '-': b'-', '.': b'.',
        '0': b'0', '1': b'1', '2': b'2',
        '3': b'3', '4': b'4', '5': b'5',
        '6': b'6', '7': b'7', '8': b'8',
        '9': b'9'
    }
    ser.write(char_map.get(data, b''))

# ----------------- 传感器初始化 -----------------
myPing = Ping1D()
if not myPing.initialize():
    print("Failed to initialize Ping!")
    exit(1)

ser = serial.Serial('/dev/ttyUSBB', 115200)
sensor = ms5837.MS5837_30BA()
if not sensor.init() or not sensor.read():
    print("Sensor init failed!")
    exit(1)


# ----------------- 主程序 -----------------
if __name__ == '__main__':
    # 蜂鸣器提示
    for _ in range(5):
        GPIO.output(11, GPIO.HIGH)
        time.sleep(1)
        GPIO.output(11, GPIO.LOW)
        time.sleep(1)

    # 初始化变量
    start_time = time.time()
    head_gyro = Gyro(0x50, 9)
    goal = head_gyro.get_angle()  # 初始目标角度
    goal_depth = 30  # 厘米
    T = 0.05  # 采样周期
    
    # 卡尔曼滤波参数
    xk1 = head_gyro.get_angle()
    Pk1 = 0
    Q = 1
    R_kalman = 0.01
    
    # 数据窗口
    step_number = 12
    direction_data = []
    
    # 创建日志文件
    log_file = f"/home/pi/Desktop/data/test_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.txt"
    with open(log_file, 'a') as f:
        f.write('real_angle\t\tmean_angle\t\tgoal_direction\t\tdirection_PID\t\tdepth\t\tgoal_depth\t\tdepth_PID\t\ttime\t\t\n')

    # 发送指令与STM32通讯
    def send_command(prefix, value, suffix):
        try:
            ser.write(prefix)
            time.sleep(0.01)
            for char in f"{value}":
                numberjudge(char)
            ser.write(suffix)
            time.sleep(0.01)
        except Exception as e:
            print(f"发送指令失败: {str(e)}")

    while True:
        # ----------------- 传感器处理 -----------------
        # 声呐处理
        obstacle_avoid = 0
        try:
            data = myPing.get_distance()
            if data and data["distance"] < 2500 and data["confidence"] > 80:
                obstacle_avoid = 3
        except:
            obstacle_avoid = 0
        
        # 深度处理
        depth_cm = 0
        try:
            if sensor.read():
                depth_cm = (sensor.depth() - sensor.freshwaterDepth) * 100
        except:
            pass

        # ----------------- 控制计算 -----------------
        try:
            # 卡尔曼滤波
            xk = xk1 + T * head_gyro.get_gyro()[2]  # 取Z轴角速度
            Pk = Pk1 + Q
            kp = Pk / (Pk + R_kalman)
            filter_angle = xk + kp * (head_gyro.get_angle()[2] - xk)
            Pk1 = (1 - kp) * Pk
            xk1 = filter_angle

            # 更新数据窗口
            if len(direction_data) >= step_number:
                direction_data.pop(0)
            direction_data.append(filter_angle)

            # 方向PID
            direction_pid = Pid(goal, 0.5, 0.2, 0.2, mean(direction_data))
            dir_output = direction_pid.cmd_pid()
            a = round(max(-30, min(30, dir_output) * 1.5, 1))

            # 深度PID
            depth_pid = Pid(goal_depth, 0.5, 0.05, 0.1, depth_cm)
            depth_output = depth_pid.cmd_pid()
            b = round((1 - min(8, abs(depth_output)) * (-1 if depth_output < 0 else 1), 2))
            
        except Exception as e:
            print(f"控制计算错误: {str(e)}")
            a, b = 0, 0

        # ----------------- 数据记录 -----------------
        try:
            with open(log_file, 'a') as f:
                f.write(f"{head_gyro.get_angle()[2]}\t\t{mean(direction_data)}\t\t{goal}\t\t{a}\t\t")
                f.write(f"{depth_cm}\t\t{goal_depth}\t\t{b}\t\t{round(time.time()-start_time,2)}\n")
        except Exception as e:
            print(f"日志写入失败: {str(e)}")

        # ----------------- 发送指令 -----------------
        send_command(b'a', a, b'A')  # 尾鳍B
        send_command(b'b', b, b'B')  # 胸鳍R1
        send_command(b'c', obstacle_avoid, b'C')  # 尾鳍R

        time.sleep(T)  # 控制周期
