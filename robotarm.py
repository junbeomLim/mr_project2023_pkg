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
simulation_mode = True #시뮬레이션 모드. 카메라 사용 안하고, 임의의 함수 식으로 물병의 각도 및 각속도 반환, 시뮬레이션 모드: True, 카메라 사용:False

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

        #단위환산
        fps = 330 #opencv에서 각속도는 deg/frame으로 변환됨

        # 동영상 또는 웹캠에서 비디오를 읽습니다.
        cap = cv2.VideoCapture(0)  # 웹캠 사용 시 0, 동영상 파일의 경로를 지정하려면 파일 경로를 입력합니다.

        while True:
            ret, frame = cap.read()

            if not ret:
                break

            # 빨간색 물체를 추적하기 위한 HSV 색상 범위 지정
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])

            # 프레임을 HSV 색상 공간으로 변환합니다.
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # 빨간색 객체를 추출합니다.
            mask = cv2.inRange(hsv, lower_red, upper_red)

            # 추출된 객체의 경계를 찾습니다.
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                # 가장 큰 물체를 찾습니다.
                largest_contour = max(contours, key=cv2.contourArea)
                rect = cv2.minAreaRect(largest_contour)

                # 현재 물체의 중심 좌표와 각도를 계산합니다.
                current_position = rect[0]
                angle = rect[2]

                # 처음 물체를 감지한 경우 초기 각도를 설정합니다.
                if initial_angle is None:
                    initial_angle = angle

                # 현재 각도와 초기 각도의 차이를 계산하여 각속도를 구합니다.
                angular_velocity = angle - initial_angle

                # 현재 위치와 각도를 출력합니다.
                print("현재 위치:", current_position)
                print("현재 각도:", angle)
                print("각속도:", angular_velocity)

            # 현재 위치를 이전 위치로 업데이트합니다.
            previous_position = current_position

            # 프레임에 추적한 물체와 정보를 그립니다.
            box = cv2.boxPoints(rect).astype(int)
            cv2.drawContours(frame, [box], 0, (0, 255, 0), 2)
            cv2.putText(frame, f"각도: {angle:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            # 화면에 프레임을 표시합니다.
            cv2.imshow('Object Tracking', frame)

            # 물체가 바닥에 착지하면 루프 종료.
            if current_position[0] < x_land and current_position[1] > y_land:
                break

            # 'q' 키를 누르면 루프를 종료합니다.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # 사용한 자원을 해제하고 창을 닫습니다.
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
    
    #for i in range(50):
    while True:
        get_action_node.get_logger().info("wait for action")
        while get_action_node.new_data_received == False:
            rclpy.spin_once(get_action_node) #로봇팔 값 받기
        
        j_1_pos, j_2_pos, camera_deg, camera_w = get_action_node.return_parameter()
        get_action_node.new_data_received = False

        #로봇팔 움직임
        robotarm_move(int(j_1_pos),int(j_2_pos))
        #물병 놓기
        move_gripper(int(1023)) #1023은 물병을 놓을 때, 다이나믹셀 position

        send_data_node.get_camera_parameter(camera_deg, camera_w)
        send_data_node.get_state_parameter(j_1_pos, j_2_pos)
        rclpy.spin_once(send_data_node) #카메라 값 전송 #로봇팔이 던지고 나서의 값 전송
        #로봇팔 원위치
        robotarm_move(512,512)
        move_gripper(512)
        
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