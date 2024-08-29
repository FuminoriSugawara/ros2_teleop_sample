import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys
import tty
import termios

class JointTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('joint_trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/trajectory_controller/joint_trajectory', 10)
        self.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # デフォルト値

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
        point.time_from_start = Duration(sec=2, nanosec=0)
        
        msg.points = [point]
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {self.positions}')

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
    
    joint_trajectory_publisher = JointTrajectoryPublisher()
    
    print("Press 0-3 to change position, or 'q' to quit:")
    print("0: 0, 1: 100000, 2: 200000, 3: 300000, 6, 7, 8")
    
    while True:
        key = getch()
        if key == '0':
            joint_trajectory_publisher.positions[0] = 0.0
            joint_trajectory_publisher.positions[1] = 0.0
            joint_trajectory_publisher.positions[2] = 0.0
            joint_trajectory_publisher.positions[3] = 0.0
            joint_trajectory_publisher.positions[4] = 0.0
            joint_trajectory_publisher.positions[5] = 0.0
        elif key == '1':
            joint_trajectory_publisher.positions[0] = 100000.0
            joint_trajectory_publisher.positions[1] = 100000.0
            joint_trajectory_publisher.positions[2] = 100000.0
            joint_trajectory_publisher.positions[3] = 100000.0
            joint_trajectory_publisher.positions[4] = 100000.0
            joint_trajectory_publisher.positions[5] = 100000.0 / 2.0
        elif key == '2':
            joint_trajectory_publisher.positions[0] = 200000.0
            joint_trajectory_publisher.positions[1] = 200000.0
            joint_trajectory_publisher.positions[2] = 200000.0
            joint_trajectory_publisher.positions[3] = 200000.0
            joint_trajectory_publisher.positions[4] = 200000.0
            joint_trajectory_publisher.positions[5] = 200000.0 / 2.0
        elif key == '3':
            joint_trajectory_publisher.positions[0] = 300000.0
            joint_trajectory_publisher.positions[1] = 300000.0
            joint_trajectory_publisher.positions[2] = 300000.0
            joint_trajectory_publisher.positions[3] = 300000.0
            joint_trajectory_publisher.positions[4] = 300000.0
            joint_trajectory_publisher.positions[5] = 300000.0 / 2.0
        elif key.lower() == 'q':
            break
        else:
            continue
        
        joint_trajectory_publisher.publish_trajectory()
    
    joint_trajectory_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()