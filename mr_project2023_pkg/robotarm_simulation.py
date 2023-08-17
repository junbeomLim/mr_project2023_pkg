import rclpy
from rclpy.node import Node
import math

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
        
        l_1 = 0.25 #250 mm
        l_2 = 0.11 #110 mm
        x = 1/3 #물병 길이와 무게 중심 위치의 비
        g = 9.8 #중력 가속도


        t = ((l_1*self.j_1_w*math.cos(self.j_1_deg)**2+x*l_2*self.j_2_w*math.cos(self.j_2_deg))+math.sqrt((l_1*self.j_1_w*math.cos(self.j_1_deg)+x*(l_2*self.j_2_w*math.cos(self.j_2_deg)))**2+2*g*(1-math.cos(self.j_1_deg))))/g # 땅에 닿는 순간
        
        self.camera_deg = abs(self.j_2_w*t+self.j_1_deg+self.j_2_deg)%360
        self.camera_w = abs(self.j_2_w)

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
        self.j_2_w = j_2_wde
        
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