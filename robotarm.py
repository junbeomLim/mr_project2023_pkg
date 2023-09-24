import rclpy
from rclpy.node import Node
import math
import cv2
import numpy as np

from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol
from rclpy.parameter import Parameter

class get_action(Node):
    def __init__(self):
        super().__init__('get_action')
        self.subscription = self.create_subscription(Robotarmcontrol,'robotarm', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        self.j_1_deg = -1
        self.j_1_w = -1
        self.j_2_deg = -1
        self.j_2_w = -1

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
        fps = 100 #opencv에서 각속도는 deg/frame으로 변환됨

        # 동영상 또는 웹캠에서 비디오를 읽습니다.
        cap = cv2.VideoCapture(0)  # 웹캠 사용 시 0, 동영상 파일의 경로를 지정하려면 파일 경로를 입력합니다.

        while True:
            ret, frame = cap.read()

            if not ret:
                break

            # 빨간색 물체를 추적하기 위한 HSV 색상 범위 지정
            lower_red = np.array([0, 100, 100])
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
        self.get_logger().info(f'j_1_w {msg.j_1_w} j_2_w {msg.j_2_w}')
        #약 3초간 움직인 후 물병을 던진다고 가정
        self.j_1_w = msg.j_1_w
        self.j_2_w = msg.j_2_w
        
        #임의로 가정
        c = 354 #모터 파워 100 = 59rpm -> deg/s로 환산
        j_1_ds = self.j_1_w*c
        j_2_ds = self.j_2_w*c
        self.j_1_deg = 3*j_1_ds
        self.j_2_deg = 3*j_2_ds
        
        self.camera_deg, self.camera_w = self.get_camera_data()

        self.new_data_received = True
    
    def return_parameter(self):
        return self.j_1_deg, self.j_1_w, self.j_2_deg, self.j_2_w, self.camera_deg, self.camera_w

class send_data(Node):
    def __init__(self):
        super().__init__('send_data')
        timer_period = 0.5  # seconds

        self.pub_camera = self.create_publisher(Cameradata, 'camera', 10)
        self.timer_camera = self.create_timer(timer_period, self.timer_callback_camera)
        
        self.pub_state = self.create_publisher(Robotarmcontrol, 'joint_1', 10)
        self.timer_state = self.create_timer(timer_period, self.timer_callback_state)

    def get_camera_parameter(self, camera_deg, camera_w):
        self.camera_deg = camera_deg
        self.camera_w = camera_w
        
    def get_state_parameter(self, j_1_deg, j_1_w, j_2_deg, j_2_w):
        self.j_1_deg = j_1_deg
        self.j_1_w = j_1_w
        self.j_2_deg = j_2_deg
        self.j_2_w = j_2_w
        
    def timer_callback_camera(self):
        msg = Cameradata()
        msg.deg = self.camera_deg
        msg.w = self.camera_w
        self.pub_camera.publish(msg)
        self.get_logger().info(f'camera {msg.deg} deg {msg.w} deg/s')

    def timer_callback_state(self):
        
        msg = Robotarmcontrol()
        msg.j_1_deg = self.j_1_deg
        msg.j_1_w = self.j_1_w
        msg.j_2_deg = self.j_2_deg
        msg.j_2_w = self.j_2_w
        self.pub_state.publish(msg)

def main(args=None):
    camera_deg = -1.0
    camera_w = -1.0
    j_1_deg = -1.0
    j_1_w = -1.0
    j_2_deg = -1.0
    j_2_w = -1.0

    rclpy.init(args=args)

    get_action_node = get_action()
    send_data_node = send_data()
    
    #for i in range(50):
    while True:
        get_action_node.get_logger().info("wait for action")
        while get_action_node.new_data_received == False:
            rclpy.spin_once(get_action_node) #로봇팔 행동
        
        j_1_deg, j_1_w, j_2_deg, j_2_w, camera_deg, camera_w = get_action_node.return_parameter()
        get_action_node.new_data_received = False

        send_data_node.get_camera_parameter(camera_deg, camera_w)
        send_data_node.get_state_parameter(j_1_deg,j_1_w,j_2_deg,j_2_w)
        rclpy.spin_once(send_data_node) #카메라 값 전송 #로봇팔이 던지고 나서의 값 전송
        
    # Destroy the node explicitly
    # (optional - selfotherwise it will be done automatically
    # when the garbage collector destroys the node object)
    get_action_node.destroy_node()
    send_data_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()