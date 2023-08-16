import rclpy
from rclpy.node import Node
import math

from my_msgs.msg import Cameradata
from my_msgs.msg import Robotarmcontrol

#---------------------------------------------------------------------------
camera_deg = -1.0
camera_w = -1.0
j_1_deg = -1.0
j_1_w = -1.0
j_2_deg = -1.0
j_2_w = -1.0
#--------------------------------------------------------------------------

class get_action(Node):
    def __init__(self):
        super().__init__('get_action')
        self.subscription = self.create_subscription(Robotarmcontrol, 'robotarm', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
            
    def listener_callback(self, msg):
        global j_1_deg
        global j_1_w
        global j_2_deg
        global j_2_w
#-------------------------------------------
        global camera_deg        
        global camera_w

        self.get_logger().info(f'j_1_w {msg.j_1_w} j_2_w {msg.j_2_w}')
        #약 3초간 움직인 후 물병을 던진다고 가정
        j_1_w = msg.j_1_w
        j_2_w = msg.j_2_w
        j_1_deg = 3*j_1_w
        j_2_deg = 3*j_1_w
        
        #임의로 가정
        l_1 = 0.25 #250 mm
        l_2 = 0.11 #110 mm
        x = 1/3 #물병 길이와 무게 중심 위치의 비
        g = 9.8 #중력 가속도
        t = ((l_1*j_1_w*math.cos(j_1_deg)**2+x*l_2*j_2_w*math.cos(j_2_deg))+math.sqrt((l_1*j_1_w*math.cos(j_1_deg)+x*(l_2*j_2_w*math.cos(j_2_deg)))**2+2*g*(1-math.cos(j_1_deg))))/g # 땅에 닿는 순간
        
        camera_deg = (j_2_w*t+j_1_deg+j_2_deg)%360
        camera_w = j_2_w


class send_camera(Node):
    def __init__(self):
        super().__init__('send_camera')
        self.pub_camera = self.create_publisher(Cameradata, 'camera', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global camera_deg
        global camera_w
        msg = Cameradata()
        msg.deg = camera_deg
        msg.w = camera_w
        self.pub_camera.publish(msg)
        self.get_logger().info(f'camera {msg.deg} deg {msg.w} deg/s')

class send_state(Node):
    def __init__(self):
        super().__init__('send_camera')
        self.pub_state = self.create_publisher(Robotarmcontrol, 'joint_1', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        global j_1_deg
        global j_1_w
        global j_2_deg
        global j_2_w

        msg = Robotarmcontrol()
        msg.j_1_deg = j_1_deg
        msg.j_1_w = j_1_w
        msg.j_2_deg = j_2_deg
        msg.j_2_w = j_2_w
        self.pub_state.publish(msg)
        self.get_logger().info(f'state {msg.j_1_deg} {msg.j_1_w} {msg.j_2_deg} {msg.j_2_w}')

def main(args=None):
    rclpy.init(args=args)

    get_action_node = get_action()
    send_camera_node = send_camera()
    send_state_node = send_state()

    while True:
        rclpy.spin_once(get_action_node)
        rclpy.spin_once(send_camera_node)
        rclpy.spin_once(send_state_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rclpy.destroy_node(get_action_node)
    rclpy.destroy_node(send_camera_node)
    rclpy.destroy_node(send_state_node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()