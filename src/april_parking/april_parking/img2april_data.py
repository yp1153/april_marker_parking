import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import apriltag
import cv2
import os
import math

class AprilTagPublisherNode(Node):
    def __init__(self):
        super().__init__('april_tag_publisher')

        # Publisher 설정
        self.id_subscription = self.create_subscription(Int32, '/id_value', self.id_value_callback, 10)
        self.pose_publisher = self.create_publisher(Pose, '/april_tag_pose', 10)

        # CvBridge 초기화 (ROS와 OpenCV 간 이미지 변환)
        self.bridge = CvBridge()

        # AprilTag 탐지기 설정
        self.detector = apriltag.Detector()
        
        # 카메라 메트릭스 및 왜곡 계수 로드
        self.matrix_coefficients = np.load('./src/april_parking/april_parking/calibration_value/camera_matrix.npy')
        self.distortion_coefficients = np.load('./src/april_parking/april_parking/calibration_value/dist_coefficients.npy')


        # Tag 크기
        tag_size = 0.031  # Tag size in meters
        self.tag_3d_points = np.array([
            [-tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, -tag_size / 2, 0],
            [tag_size / 2, tag_size / 2, 0],
            [-tag_size / 2, tag_size / 2, 0]
        ], dtype=np.float32)

        # 구독자 설정: image_raw 또는 gray 토픽을 받음
        self.image_sub = self.create_subscription(
            Image,
            '/raw_image/gray',  # 또는 'gray' 토픽을 사용할 수 있습니다.
            self.image_callback,
            10
        )
        
        # 수신된 ID 저장
        self.received_id = None
        
        # 수신한 이미지 저장
        self.gray_image = None
        
        self.pub_switch = True

        self.get_logger().info("img2april loaded")

    def id_value_callback(self, msg):
        # 수신된 ID 저장
        self.received_id = msg.data
        #self.get_logger().info(f'수신된 마커 ID: {self.received_id}')
        self.pub_switch = True

    def image_callback(self, msg):
        try:
            # ROS 이미지 메시지를 OpenCV 형식으로 변환
            frame = self.bridge.imgmsg_to_cv2(msg, 'mono8')

            # AprilTag 탐지
            detections = self.detector.detect(frame)

            for detection in detections:
                corners = detection.corners.astype(int)
                top_left, top_right, bottom_right, bottom_left = corners
                image_points = np.array([top_left, top_right, bottom_right, bottom_left], dtype=np.float32)

                # SolvePnP를 사용하여 회전 벡터와 이동 벡터 추정
                success, rotation_vector, translation_vector = cv2.solvePnP(self.tag_3d_points, image_points, self.matrix_coefficients, self.distortion_coefficients)

                if success:
                    # 메시지 발행
                    pose_msg = Pose()
                    pose_msg.position.x = float(translation_vector[0])  # x 좌표
                    pose_msg.position.y = float(translation_vector[1])  # y 좌표
                    pose_msg.position.z = float(translation_vector[2])  # z 좌표

                    pose_msg.orientation.x = float(rotation_vector[0])  # 회전 벡터 x
                    pose_msg.orientation.y = float(rotation_vector[1])  # 회전 벡터 y
                    pose_msg.orientation.z = float(rotation_vector[2])  # 회전 벡터 z
                    
                    pose_msg.orientation.x = float(rotation_vector[0]) * 180 / math.pi # 회전 벡터 x
                    pose_msg.orientation.y = float(rotation_vector[1]) * 180 / math.pi # 회전 벡터 y
                    pose_msg.orientation.z = float(rotation_vector[2]) * 180 / math.pi # 회전 벡터 z
                    
                    pose_msg.orientation.w = 0.0  # 회전 벡터 w는 0으로 설정 (예시)

                    self.pose_publisher.publish(pose_msg)
                    os.system('cls' if os.name == 'nt' else 'clear')

                    self.get_logger().info(f"\nPublished ID: {detection.tag_id}, \nPosition:\n {translation_vector[0]},\n {translation_vector[1]},\n {translation_vector[2]}, \nRotation:\n {rotation_vector[0]},\n {rotation_vector[1]},\n {rotation_vector[2]}")
        except Exception as e:
            self.get_logger().error(f"Error in processing image: {e}")


def main(args=None):
    rclpy.init(args=args)

    april_tag_publisher_node = AprilTagPublisherNode()

    rclpy.spin(april_tag_publisher_node)

    rclpy.shutdown()


if __name__ == '__main__':
    main()

