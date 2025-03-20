import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import os

class Intig(Node):
    def __init__(self):
        super().__init__('intig_node')  # Initialize the ROS node

        # Subscriber for /joint_states
        self.joint_states_subscriber = self.create_subscription(
            JointState,      # Message type
            '/joint_states', # Topic name
            self.joint_states_callback,  # Callback function
            10                # QoS
        )


        # Subscriber for /goal_len
        self.goal_moving_subscriber = self.create_subscription(
            Twist,           # Message type
            '/goal_moving',     # Topic name
            self.goal_moving_callback,  # Callback function
            10                # QoS (Quality of Service) history depth
        )
        
        # Publisher for /cmd_vel
        self.cmd_vel_publisher = self.create_publisher(
            Twist,           # Message type
            '/cmd_vel',      # Topic name
            10                # QoS
        )
        timer_period = 0.3  # seconds
        self.timer = self.create_timer(timer_period, self.dimension_converter)
        
        self.right_encoder = None #오른쪽 모터 엔코더 값
        self.left_encoder = None #왼쪽 모터 엔코더 값
        self.goal_x = None #목표 직진 거리 값 
        self.goal_radian = None #목표 회전 거리 값
        self.slow_point = 0.03 #남은오차가 기입한 숫자(엔코더 값)만큼 남았을 때 감속 시작하는 지점
        self.tolerance = 1. #허용오차(로봇 정지 기준점)
        self.encoder_per_lenth = 1. #mm당 엔코더 값
        
        self.get_logger().info("vel2len loaded")
        
    #모터 엔코더 값 수신
    def joint_states_callback(self, msg):
        # This function is called every time a message is received on the /joint_states topic
        # Extract the joint names, positions, velocities, and efforts from the message
        joint_names = msg.name
        joint_positions = msg.position
        joint_velocities = msg.velocity
        joint_efforts = msg.effort

        # Log information about the joints
        for name, position, velocity, effort in zip(joint_names, joint_positions, joint_velocities, joint_efforts):
            self.get_logger().info(f"Joint: {name}, Position: {position}, Velocity: {velocity}, Effort: {effort}")
    
    #목표값 수신
    def goal_moving_callback(self, msg):
        # This function is called every time a message is received on the /goal_len topic
        self.goal_moving = msg.data  # Extract the integer value from the message
        self.get_logger().info(f"Received goal_len: {goal_len}")

        msg.data.linear.x = self.goal_x
        msg.data.angular.z = self.goal_radian
        self.get_logger().info("Published cmd_vel message.")
    
    #목표값만큼 동작
    def dimension_converter(self):
        if self.goal_x is not None:
            self.go_linear(self.goal_x)
        elif self.goal_radian is not None:
            self.go_angular(self.goal_radian)
            
    #전진 후진 함수
    def go_linear(self, x):
        msg = Twist()
        if self.total_error is None:
            self.total_error = self.right_encoder + self.goal_x * self.encoder_per_lenth
        elif self.total_error is None and self.goal_x != None:
            self.now_error = total_error - self.right_encoder
            if abs(now_error) < self.slow_point:
                msg.linear.x = 0.5 * self.goal_x / abs(self.x)
                self.cmd_vel_publisher.publish(msg)
                if ans(now_error) < tolerance:
                    msg.linear.x = 0.0
                    self.cmd_vel_publisher.publish(msg)
                    self.total_error = None
                    self.goal_x = None
            else:
                msg.linear.x = 1. * self.goal_x / abs(self.x)
                self.cmd_vel_publisher.publish(msg)
                
    #회전 함수
    def go_angular(self, radian):
        msg = Twist()
        if self.total_error is None:
            if self.goal_radian > 0.:
                self.total_error = self.right_encoder + self.goal_radian * self.encoder_per_lenth
            else:
                 self.total_error = self.left_encoder + self.goal_radian * self.encoder_per_lenth
            
        elif self.total_error is None and self.goal_radian != None:
            if self.goal_radian > 0.:
                self.now_error = total_error - self.right_encoder
            else:
                self.now_error = total_error - self.left_encoder
            if abs(now_error) < self.slow_point:
                msg.angular.z = 0.5 * self.goal_radian / abs(self.goal_radian)
                self.cmd_vel_publisher.publish(msg)
                if ans(now_error) < tolerance:
                    msg.angular.z = 0.0
                    self.cmd_vel_publisher.publish(msg)
                    self.total_error = None
                    self.goal_radian = None
            else:
                msg.angular.z = 1. * self.goal_radian / abs(self.goal_radian)
                self.cmd_vel_publisher.publish(msg) 

def main(args=None):
    rclpy.init(args=args)

    intig_node = Intig()

    rclpy.spin(intig_node)  # Keep the node running until it's shut down

    intig_node.destroy_node()  # Clean up when shutting down
    rclpy.shutdown()

if __name__ == '__main__':
    main()
