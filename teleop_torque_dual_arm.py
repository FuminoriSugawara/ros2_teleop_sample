import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import tty
import termios

class EffortControlPublisher(Node):
    def __init__(self):
        super().__init__('effort_control_publisher')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/effort_controller/commands', 1)
        self.efforts = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def publish_effort(self):
        msg = Float64MultiArray()
        msg.data = self.efforts
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing effort: {self.efforts}')

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def main(args=None):
    rclpy.init(args=args)
    
    effort_control_publisher = EffortControlPublisher()

    target_current = 0.6 # 0.6A
    
    print("Press 0-3 to change effort, or 'q' to quit:")
    print("0: 0, 1: 300, 2: 500, 3: -300, 4: -500")
    
    while True:
        key = getch()
        if key == '0':
            for i in range(14):
                effort_control_publisher.efforts[i] = 0.0
        elif key == '1':
            for i in range(14):
                effort_control_publisher.efforts[i] = target_current * 1000.0
        elif key == '2':
            for i in range(14):
                effort_control_publisher.efforts[i] = 2 * target_current * 1000.0
        elif key == '3':
            for i in range(14):
                effort_control_publisher.efforts[i] = - target_current * 1000.0
        elif key == '4':
            for i in range(14):
                effort_control_publisher.efforts[i] = -2 * target_current * 1000.0
        elif key.lower() == 'q':
            break
        else:
            continue
        
        effort_control_publisher.publish_effort()
    
    effort_control_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()