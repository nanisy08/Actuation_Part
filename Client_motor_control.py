"""
File    : Motor_control.py
Author  : Sooyeon Kim
Date    : December 01, 2023
Update  : January 25, 2024
Description : 
            : Saving data in 10Hz
Protocol    : 
1. Stepper motor 연결 -- Arduino Due upload
2. Port num 확인 (Arduino, Dynamixel) + TCP server IP num 확인
3. 팬텀 옴니 서버를 먼저 열고, 실행할 것

position = 1000 * position; //micro meter
gimbalAngles = 1000 * gimbalAngles * 180/3.141592; //milli deg
int16_t data[6] = { pos[0], pos[2], pos[1], gimbAng[0], gimbAng[1], gimbAng[2] };
"""
import socket
import struct
import keyboard
import time
import serial
import os
from datetime import datetime
import threading
import csv

# 1. 데이터 저장 경로 + 파일 이름.csv
csv_file_path = "C:/Users/222BMG13/Desktop/MotorData.csv"

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

from dynamixel_sdk import * 

######################################################
##################### VARIABLES ######################
######################################################
# Motor setting
DXL1_ID                    = 8   # Steering(Flexion)
DXL2_ID                    = 9   # Yawing
DEVICENAME                 = 'COM6' # ex) Windows: "COM1", Linux: "/dev/ttyUSB0", Mac: "/dev/tty.usbserial-*"
BAUDRATE                   = 1000000

MOVING_SPEED               = 1023  # 0~1023 x 0.111 rpm (ex-1023:114 rpm)
                                   # AX-12 : No Load Speed	59 [rev/min] (at 12V)
DXL_MOVING_STATUS_THRESHOLD = 5

# Control table address
ADDR_MX_TORQUE_ENABLE      = 24
ADDR_MX_GOAL_POSITION      = 30
ADDR_MX_PRESENT_POSITION   = 36
ADDR_MX_MOVING_SPEED       = 32
ADDR_MX_PRESENT_SPEED      = 38

TORQUE_ENABLE              = 1
TORQUE_DISABLE             = 0

PROTOCOL_VERSION           = 1.0

# Data Byte Length
LEN_MX_GOAL_POSITION       = 4
LEN_MX_PRESENT_POSITION    = 4
LEN_MX_MOVING              = 1

# Needle angle
scale_factor = 2            # 모터의 각도 대비 사용자 input(gimbal angle)
                            # ex) 사용자 10deg 움직였는데 --> SF 2 --> 모터는 20deg 움직인다
control_limit_max = 100     # needle 움직이는 거 보고 control input의 범위 정할 것
control_limit_min = -100

# Motor initial position
dxl1_init = 340
dxl2_init = 690

### Data Initialize
disp, dxl_goal_angle, dxl_goal_position = 0, 0, 0

######################################################
################ FUNCTION AND THREAD #################
######################################################
### Thread for data logging
exit_flag = False
def log_data():
    global disp, dxl_goal_angle, dxl_goal_position
    with open(csv_file_path, mode='w', newline='') as csv_file:
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(['Time', 'User Control','',         'Motor', ''])
        csv_writer.writerow(['', '    Disp[mm]', 'Gimbal[deg]', 'Angle[deg]', 'Command[-]'])

        while not exit_flag:
            csv_writer.writerow([datetime.now(), round(disp,4), round(dxl_goal_angle,4),
                                    round(300/1024*dxl_goal_position,4), int(dxl_goal_position)])
            time.sleep(0.1)
        else:
            csv_file.close()

### Map function
def mapping(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


######################################################
##################### EXECUTION ######################
######################################################
time.sleep(0.2)

## 1. TCP/IP communication
HOST = '192.168.0.140'
PORT = 4578
PACKET_SIZE = 24

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    client_socket.connect((HOST, PORT))
    print('Connected to server!')
except ConnectionRefusedError:
    print("TCP 연결이 거부되었습니다. 호스트 또는 포트 번호를 확인하세요.")
    quit()
except socket.error as e:
    print(f"TCP 소켓 오류 발생: {e}")
    quit()

## 2. Stepper motor (Serial communication)
port = 'COM5'
baud_rate = 250000
try:
    ser = serial.Serial(port, baud_rate)
    print("Connected to Stepper motor!")
except serial.SerialException as e:
    print(f"Arduino serial 연결 실패: {e}")
    quit()

## 3. Dynamixel setting
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Initialize GroupSyncWrite/Read instance
groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port of Dynamixel")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Enable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL1_ID)

# Enable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
else:
    print("Dynamixel#%d has been successfully connected" % DXL2_ID)

# Write moving speed to maximize the response
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))
dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_MX_MOVING_SPEED, MOVING_SPEED)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

##################################################
### Thread
log_thread = threading.Thread(target=log_data)
log_thread.start()
time.sleep(0.2)


print("Press 'esc' to quit")
## Control Loop
while True:
    ## Receiving from server via TCP/IP
    byte_data = client_socket.recv(PACKET_SIZE)
    if len(byte_data) > 1:
        data = struct.unpack('<iiiiii', byte_data)
        ## 1. Stepper motor Control -- Serial Communication with Arduino Due
        serial_data = struct.pack('<h', int(data[1]/10))  # little endian, displacement[um]
        ser.write(serial_data)
        disp = data[1]/1000
        
        ## 2. Dynamixel Control
        dxl_goal_angle = data[3]/1000 #deg
        dxl_goal_position = float(mapping(dxl_goal_angle/scale_factor, -150, 150, 0, 1023)) - 512 # deg to control input

        # Range of angle for safety (unit: control input)
        dxl_goal_position = max(min(dxl_goal_position, control_limit_max), control_limit_min)
        
        # Offest 보정
        dxl1_goal_position = dxl1_init + (int(dxl_goal_position))
        dxl2_goal_position = dxl2_init + (int(dxl_goal_position))

        # Allocate goal position value into byte array
        param_goal_position1 = [ DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position)),
                                DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position)),
                                DXL_LOBYTE(DXL_HIWORD(dxl1_goal_position)),
                                DXL_HIBYTE(DXL_HIWORD(dxl1_goal_position)) ]
        param_goal_position2 = [ DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position)),
                                DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position)),
                                DXL_LOBYTE(DXL_HIWORD(dxl2_goal_position)),
                                DXL_HIBYTE(DXL_HIWORD(dxl2_goal_position)) ]

        # Add Dynamixel#1,2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addParam failed" % DXL1_ID)
            quit()
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addParam failed" % DXL2_ID)
            quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        # print("Disp: %.4f mm, Gimbal: %.4f deg || Goal: %.4f deg " % (disp, dxl_goal_angle, 300/1024*dxl_goal_position))

        # print("Displacement[mm]: %.4f" % (disp))
        # print("Gimbal Angle[deg]: %.4f" % (dxl_goal_angle))
        # print("Angle of disk[deg]: %.4f" % (300/1024*dxl_goal_position))

        
    ## Quit
    if keyboard.is_pressed('esc'):
        print("Exit the loop")
        exit_flag = True
        break



# Close the TCP/IP socket
client_socket.close()

# Disable Dynamixel#1 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL1_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Disable Dynamixel#2 Torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL2_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("%s" % packetHandler.getRxPacketError(dxl_error))

# Close port
portHandler.closePort()

# Close serial
ser.close()
