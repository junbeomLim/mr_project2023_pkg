#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import math
import cv2
import numpy as np

from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol
from rclpy.parameter import Parameter

#전처리기
robotarm_Connect = False #로봇 모터 연결되어 있을 때만 모터 함수 실행, 모터 연결: True 연결 안됨: False
simulation_mode = False #시뮬레이션 모드. 카메라 사용 안하고, 임의의 함수 식으로 물병의 각도 및 각속도 반환, 시뮬레이션 모드: True, 카메라 사용:False

GRIPPER_OPEN = 1023 #1023은 물병을 놓을 때, 다이나믹셀 position
GRIPPER_CLOSE = 512 #512은 물병을 놓을 때, 다이나믹셀 position

if robotarm_Connect:
    import os

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

    #--------------------------------------------------------------------#
    #motor control parameter
    from dynamixel_sdk import *                    # Uses Dynamixel SDK library

    # Control table address
    ADDR_MX_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
    ADDR_MX_GOAL_POSITION      = 30
    ADDR_MX_PRESENT_POSITION   = 36

    # Data Byte Length
    LEN_MX_GOAL_POSITION       = 2
    LEN_MX_PRESENT_POSITION    = 2

    # Protocol version
    PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

    # Default setting
    DXL1_ID                     = 1                 # Dynamixel#1 ID : 1
    DXL2_ID                     = 2                 # Dynamixel#2 ID : 2
    DXL3_ID                     = 3                 # Dynamixel#3 ID : 3
    BAUDRATE                    = 57600             # Dynamixel default baudrate : 57600
    DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                    # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"
    TORQUE_ENABLE               = 1                 # Value for enabling the torque
    TORQUE_DISABLE              = 0                 # Value for disabling the torque
    DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

    #----------------------------------------------------------------------------

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Initialize GroupSyncWrite instance
    groupSyncWrite = GroupSyncWrite(portHandler, packetHandler, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
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

    #gripper
    # Enable Dynamixel#3 Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel#%d has been successfully connected" % DXL3_ID)
#---------------------------------------------------------------------------------------------------#

def robotarm_move(dxl1_goal_pos, dxl2_goal_pos):
    global robotarm_Connect
    if robotarm_Connect:
        dxl1_goal_position = [dxl1_goal_pos]
        dxl2_goal_position = [dxl2_goal_pos]

        # Allocate goal position value into byte array
        param_goal_position1 = [DXL_LOBYTE(DXL_LOWORD(dxl1_goal_position[0])), DXL_HIBYTE(DXL_LOWORD(dxl1_goal_position[0]))]
        param_goal_position2 = [DXL_LOBYTE(DXL_LOWORD(dxl2_goal_position[0])), DXL_HIBYTE(DXL_LOWORD(dxl2_goal_position[0]))]

        # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL1_ID, param_goal_position1)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL1_ID)
            quit()

        # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL2_ID, param_goal_position2)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL2_ID)
            quit()

        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()

        while 1:
            dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
            if (abs(dxl1_goal_position[0] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                # Read Dynamixel#1 present position
                dxl1_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL1_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

            dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
            if (abs(dxl2_goal_position[0] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                # Read Dynamixel#2 present position
                dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL2_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

            #print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d" % (DXL1_ID, dxl_goal_position[index], dxl1_present_position, DXL2_ID, dxl_goal_position[index], dxl2_present_position))

            if not ((abs(dxl1_goal_position[0] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) and (abs(dxl2_goal_position[0] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break

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

def move_gripper(dxl3_goal_pos):
    global robotarm_Connect
    if robotarm_Connect:
        dxl3_goal_position = [dxl3_goal_pos]

        # Allocate goal position value into byte array
        param_goal_position3 = [DXL_LOBYTE(DXL_LOWORD(dxl3_goal_position[0])), DXL_HIBYTE(DXL_LOWORD(dxl3_goal_position[0]))]

        # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
        dxl_addparam_result = groupSyncWrite.addParam(DXL3_ID, param_goal_position3)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % DXL3_ID)
            quit()
        
        # Syncwrite goal position
        dxl_comm_result = groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))

        # Clear syncwrite parameter storage
        groupSyncWrite.clearParam()
        
        #gripper move
        while 1:
            dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
            if (abs(dxl3_goal_position[0] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                # Read Dynamixel#1 present position
                dxl3_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL3_ID, ADDR_MX_PRESENT_POSITION)
                if dxl_comm_result != COMM_SUCCESS:
                    print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
                elif dxl_error != 0:
                    print("%s" % packetHandler.getRxPacketError(dxl_error))

            if not (abs(dxl3_goal_position[0] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD):
                break

            # Disable Dynamixel#1 Torque
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL3_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))

class get_action(Node):
    def __init__(self):
        super().__init__('get_action')
        self.subscription = self.create_subscription(Robotarmcontrol,'robotarm', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        global simulation_mode

        self.j_1_pos = -1
        self.j_2_pos = -1
        
        self.camera_deg = -1        
        self.camera_w = -1

        self.new_data_received = False

    def get_camera_data(self):
        # 물체의 초기 각도와 각속도 초기화
        initial_angle = None
        angular_velocity = None

        #물병이 떨어지는 부분 인식 범위선:
        x_land = 500
        y_land = 870
        
        angle = 0
        preangle = 0

        # 칼만 필터 파라미터 초기화
        Q = 1e-5  # 프로세스 분산
        R = 0.01  # 측정 분산
        x_hat = 0  # 초기 예측값
        P = 1  # 초기 예측 오차

        #단위환산
        fps = 330 #opencv에서 각속도는 deg/frame으로 변환됨

        # 카메라 열기 노트북 카메라: 0, 카메라: 2
        cap = cv2.VideoCapture(2)

        while True:
            # 프레임 읽기
            _, frame = cap.read()

            # 빨간색 범위 설정 bgr
            lower_red = np.array([0, 0, 102])
            upper_red = np.array([51, 60, 255])

            # 이미지에서 빨간색 영역 추출
            mask = cv2.inRange(frame, lower_red, upper_red)

            # 빨간색 영역을 감싸는 사각형 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 최대 크기의 2개의 빨간색 사각형 찾기
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]

            centers = []

            for contour in contours:
                # 빨간색 영역을 감싸는 최대 크기의 사각형 찾기
                x, y, w, h = cv2.boundingRect(contour)

                # 중심 좌표 계산
                center_x = x + w // 2
                center_y = y + h // 2

                # 중심 좌표 저장
                centers.append((center_x, center_y))

                # 사각형 그리기
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 두 중심점이 존재할 때
            if len(centers) == 2:
                # 두 중심점 연결하는 선 그리기
                angular_velocity = angle - preangle
                cv2.line(frame, centers[0], centers[1], (0, 0, 255), 2)

                # 두 중심점 간의 각도 계산
                angle = np.arctan2(centers[1][1] - centers[0][1], centers[1][0] - centers[0][0]) * 180 / np.pi
                preangle = angle

                # 칼만 필터 적용
                # 예측 단계
                x_hat_minus = x_hat
                P_minus = P + Q

                # 측정 단계
                K = P_minus / (P_minus + R)
                x_hat = x_hat_minus + K * (angle - x_hat_minus)
                P = (1 - K) * P_minus

                # 각도 출력
                angle = x_hat
                print("각도:", angle, "각속도", preangle)

            # 프레임 출력
            cv2.imshow("Frame", frame)

            # 'q' 키로 루프 종료
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 카메라 해제 및 창 닫기
        cap.release()
        cv2.destroyAllWindows() 
        
        return float(angle), float(angular_velocity*fps)   
    
    def listener_callback(self, msg):
        #제어할 각도 값 수신
        self.get_logger().info(f'j_1_pos {msg.j_1_pos} j_2_pos {msg.j_2_pos}')
        self.j_1_pos = msg.j_1_pos
        self.j_2_pos = msg.j_2_pos
        
        #카메라 값 수신
        if simulation_mode:
            #임의로 지정, 역학적 분석과 무관
            self.camera_w = (self.j_1_pos+self.j_2_pos)/6
            self.camera_deg = (self.j_1_pos+self.j_2_pos)/17
        else:
            self.camera_deg, self.camera_w = self.get_camera_data()

        self.new_data_received = True
    
    def return_parameter(self):
        return self.j_1_pos, self.j_2_pos, self.camera_deg, self.camera_w

class send_data(Node):
    def __init__(self):
        super().__init__('send_data')
        timer_period = 0.5  # seconds

        self.pub_camera = self.create_publisher(Cameradata, 'camera', 10)
        self.timer_camera = self.create_timer(timer_period, self.timer_callback_camera)
        
    def get_camera_parameter(self, camera_deg, camera_w):
        self.camera_deg = camera_deg
        self.camera_w = camera_w
        
    def get_state_parameter(self, j_1_pos, j_2_pos):
        self.j_1_pos = j_1_pos
        self.j_2_pos = j_2_pos
        
    def timer_callback_camera(self):
        msg = Cameradata()
        msg.deg = self.camera_deg
        msg.w = self.camera_w
        self.pub_camera.publish(msg)
        self.get_logger().info(f'camera {msg.deg} deg {msg.w} deg/s')

    def timer_callback_state(self):
        msg = Robotarmcontrol()
        msg.j_1_pos = self.j_1_pos
        msg.j_2_pos = self.j_2_pos
        self.pub_state.publish(msg)

def main(args=None):
    camera_deg = -1.0
    camera_w = -1.0
    j_1_pos = -1.0
    j_2_pos = -1.0
    
    rclpy.init(args=args)

    get_action_node = get_action()
    send_data_node = send_data()

    while True:
        get_action_node.get_logger().info("wait for action")
        while get_action_node.new_data_received == False:
            rclpy.spin_once(get_action_node) #로봇팔 값 받기
        
        j_1_pos, j_2_pos, camera_deg, camera_w = get_action_node.return_parameter()
        get_action_node.new_data_received = False

        #그리퍼 조정하기
        #키보드 값 입력 받아 조정
        user_input = input('open: o, close: c, end: q :')
        while user_input != 1:
            if user_input == 'o' or user_input == 'O':
                user_input = input('open: o, close: c, end: q :')

            elif user_input == 'c' or user_input == 'C':
                user_input = input('open: o, close: c, end: q :')
                
            elif user_input == 'q' or user_input == 'Q':
                break

        #로봇팔 움직임
        robotarm_move(int(j_1_pos),int(j_2_pos))
        #물병 놓기(던지기)
        move_gripper(GRIPPER_OPEN)

        send_data_node.get_camera_parameter(camera_deg, camera_w)
        send_data_node.get_state_parameter(j_1_pos, j_2_pos)
        rclpy.spin_once(send_data_node) #카메라 값 전송 #로봇팔이 던지고 나서의 값 전송
        
        #로봇팔 원위치
        robotarm_move(512,512)
        move_gripper(GRIPPER_CLOSE)
        
    # Close port
    portHandler.closePort()
    # Destroy the node explicitly
    # (optional - selfotherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_action_node.destroy_node()
    send_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()