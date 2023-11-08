#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
import cv2
import numpy as np

from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol
from rclpy.parameter import Parameter

#전처리기
robotarm_Connect = True #로봇 모터 연결되어 있을 때만 모터 함수 실행, 모터 연결: True 연결 안됨: False
simulation_mode = True #시뮬레이션 모드. 카메라 사용 안하고, 임의의 함수 식으로 물병의 각도 및 각속도 반환, 시뮬레이션 모드: True, 카메라 사용:False

if robotarm_Connect:
    from dynamixel_sdk import *  # Dynamixel SDK library

    # Protocol version
    PROTOCOL_VERSION = 1.0

    # Default setting
    DXL1_ID = 1  # Dynamixel ID: 1
    DXL2_ID = 2  # Dynamixel ID: 2
    DXL3_ID = 3  # Dynamixel ID: 3
    BAUDRATE = 57600
    DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller

    # Control table addresses
    ADDR_AX_TORQUE_ENABLE = 24
    ADDR_AX_GOAL_POSITION = 30
    ADDR_AX_MOVING_SPEED = 32
    DXL_MOVING_STATUS_THRESHOLD = 10
    GRIPPER_OPEN = 20 #20은 물병을 놓을 때, 다이나믹셀 position
    GRIPPER_CLOSE = 240 #240은 물병을 놓을 때, 다이나믹셀 position

    # Speed for maintaining position
    SPEED_FOR_MAINTENANCE = 0

    # Initialize PortHandler instance
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        quit()

    # Set port baud rate
    if not portHandler.setBaudRate(BAUDRATE):
        print("Failed to set the baudrate")
        quit()

    # Enable torque for all Dynamixels
    for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
        packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, 1)

    # Move Dynamixel#2 to position 240 and set its speed to 0 to maintain the position
    packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION, GRIPPER_CLOSE)
    packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_MOVING_SPEED, SPEED_FOR_MAINTENANCE)
def move_robotarm(dxl_goal_position, speed):
    global robotarm_Connect
    if robotarm_Connect:
        # Move Dynamixel#1 to position and Dynamixel#2 to position
        packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_GOAL_POSITION, dxl_goal_position)
        packetHandler.write2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_MOVING_SPEED, speed)
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION, 1023-dxl_goal_position)
        packetHandler.write2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_MOVING_SPEED, speed)
        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION, GRIPPER_CLOSE)

        # Wait for Dynamixel#1 and Dynamixel#2 to reach their goal positions
        while True:
            if abs(packetHandler.read2ByteTxRx(portHandler, DXL1_ID, ADDR_AX_GOAL_POSITION)[0] - dxl_goal_position) <= DXL_MOVING_STATUS_THRESHOLD and \
                abs(packetHandler.read2ByteTxRx(portHandler, DXL2_ID, ADDR_AX_GOAL_POSITION)[0] - (1023-dxl_goal_position)) <= DXL_MOVING_STATUS_THRESHOLD and \
                abs(packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION)[0] - GRIPPER_CLOSE) <= DXL_MOVING_STATUS_THRESHOLD:
                break

def move_gripper(dxl3_goal_position):
    global robotarm_Connect
    if robotarm_Connect:
        # Move Dynamixel#3 to position
        packetHandler.write2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION, dxl3_goal_position)
        # Wait for Dynamixel#1 and Dynamixel#3 to reach their goal positions
        while True:
            if abs(packetHandler.read2ByteTxRx(portHandler, DXL3_ID, ADDR_AX_GOAL_POSITION)[0] - dxl3_goal_position) <= DXL_MOVING_STATUS_THRESHOLD:
                break

class get_action(Node):
    def __init__(self):
        super().__init__('get_action')
        self.subscription = self.create_subscription(Robotarmcontrol,'robotarm', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.j_pos = -1

        self.new_data_received = False   
    
    def listener_callback(self, msg):
        #제어할 각도 값 수신
        self.get_logger().info(f'j_pos {msg.j_pos}')
        self.new_data_received = True
        self.j_pos = msg.j_pos
    
    def return_parameter(self):
        return self.j_pos

class send_data(Node):
    def __init__(self):
        super().__init__('send_data')
        timer_period = 0.5  # seconds

        self.pub_camera = self.create_publisher(Cameradata, 'camera', 10)
        self.timer_camera = self.create_timer(timer_period, self.timer_callback_camera)
        
    def get_camera_parameter(self, camera_deg, camera_w):
        self.camera_deg = camera_deg
        self.camera_w = camera_w
        
    def get_state_parameter(self, j_pos):
        self.j_pos = j_pos
        
    def timer_callback_camera(self):
        msg = Cameradata()
        msg.deg = self.camera_deg
        msg.w = self.camera_w
        self.pub_camera.publish(msg)
        self.get_logger().info(f'camera {msg.deg} deg {msg.w} deg/s')

    def timer_callback_state(self):
        msg = Robotarmcontrol()
        msg.j_pos = self.j_pos
        self.pub_state.publish(msg)

def main(args=None):
    camera_deg = -1.0
    camera_w = -1.0
    j_pos = -1.0
    
    # 물체의 초기 각도와 각속도 초기화
    initial_angle = None
    angular_velocity = None

    #물병이 떨어지는 부분 인식 범위선:
    y_land = 270
    
    angle = 0
    preangle = 0

    # 칼만 필터 파라미터 초기화
    Q = 1e-5  # 프로세스 분산
    R = 0.01  # 측정 분산
    x_hat = 0  # 초기 예측값
    P = 1  # 초기 예측 오차

    #단위환산
    fps = 330 #opencv에서 각속도는 deg/frame으로 변환됨

    rclpy.init(args=args)

    get_action_node = get_action()
    send_data_node = send_data()

    while True:
        get_action_node.get_logger().info("wait for action")
        while get_action_node.new_data_received == False:
            rclpy.spin_once(get_action_node) #로봇팔 값 받기
        
        j_pos = get_action_node.return_parameter()
        
        #그리퍼 조정하기
        #키보드 값 입력 받아 조정
        move_robotarm(200,1000)
        user_input = input('open: o, close: c, end: q :')
        while user_input != 1:
            if user_input == 'o' or user_input == 'O':
                move_gripper(GRIPPER_OPEN)
            elif user_input == 'c' or user_input == 'C':
                move_gripper(GRIPPER_CLOSE)
            elif user_input == 'q' or user_input == 'Q':
                move_gripper(GRIPPER_CLOSE)
                break
            user_input = input('open: o, close: c, end: q :')
        
        if simulation_mode:
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
                    length_y = centers[1][1] - centers[0][1]
                    length_x = centers[1][0] - centers[0][0]
                    angle = np.arctan2(length_y, length_x) * 180 / np.pi
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
                    print("각도:", angle, "각속도", angular_velocity, "중심점 y위치", (centers[0][1]+ centers[1][1])/2)
                    if (centers[0][1]+ centers[1][1])/2 >= y_land:
                        break
                
                # 프레임 출력
                cv2.imshow("Frame", frame)
                
                # 'q' 키로 루프 종료 (cv 창에서)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                
                # 'r' 키로 로봇팔 움직이기 (cv 창에서)
                if cv2.waitKey(1) & 0xFF == ord('r'):
                    if robotarm_Connect:
                        #로봇팔 움직임
                        move_robotarm(int(j_pos), 1000)
                        time.sleep(0.2)
                        #물병 놓기(던지기)
                        move_gripper(GRIPPER_OPEN)
                        move_robotarm(200, 1000)
                        # Disable torque for all Dynamixels
                        for dxl_id in [DXL1_ID, DXL2_ID, DXL3_ID]:
                            packetHandler.write1ByteTxRx(portHandler, dxl_id, ADDR_AX_TORQUE_ENABLE, 0)

            # 카메라 해제 및 창 닫기
            cap.release()
            cv2.destroyAllWindows() 

        else:
            #임의로 지정, 역학적 분석과 무관
            angle = (j_pos)/2.3
            angular_velocity = (j_pos)/6.4

        camera_deg = float(angle) 
        camera_w = float(angular_velocity*fps)

        #camera_deg, camera_w = get_camera_data(j_pos)
        
        send_data_node.get_camera_parameter(camera_deg, camera_w)
        send_data_node.get_state_parameter(j_pos)
        rclpy.spin_once(send_data_node) #카메라 값 전송 #로봇팔이 던지고 나서의 값 전송
        
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