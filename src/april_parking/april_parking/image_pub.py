import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np
import cv2

class RealSenseCameraNode(Node):
    def __init__(self, width=640, height=480, fps=30):
        super().__init__('realsense_camera_node')

        # Set up RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()

        # Enable the streams (color and depth)
        self.config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)  # Color stream
        #self.config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)  # Depth stream
        
        # Start the pipeline
        self.pipeline.start(self.config)

        # Create ROS2 publishers
        self.color_pub = self.create_publisher(Image, '/raw_image/gray', 10)
        #self.depth_pub = self.create_publisher(Image, '/raw_image/depth', 10)

        # Initialize CvBridge to convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()

        # Timer to publish frames every 100 ms
        self.timer = self.create_timer(0.1, self.publish_frames)

    def get_frames(self):
        # Wait for the next set of frames
        frames = self.pipeline.wait_for_frames()

        # Get color and depth frames
        color_frame = frames.get_color_frame()
        #depth_frame = frames.get_depth_frame()

        #if not color_frame or not depth_frame:
        #    return None, None
        
        # Convert to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())  # 2D color image
        #depth_image = np.asanyarray(depth_frame.get_data())  # 2D depth image

        return color_image#, depth_image

    def publish_frames(self):
        # Get frames from the RealSense camera
        #color_image, depth_image = self.get_frames()
        color_image = self.get_frames()
        if color_image is not None:
            # Convert color image to ROS Image message
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
            #msg = self.bridge.cv2_to_imgmsg(color_image, encoding="bgr8")
            msg = self.bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
            # Publish the color image
            self.color_pub.publish(msg)

        #if depth_image is not None:
        #    # Convert depth image to ROS Image message
        #    # Note: Depth image should be 16-bit, so we use encoding "16UC1"
        #    depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1")
        #    # Publish the depth image
        #    self.depth_pub.publish(depth_msg)

    def release(self):
        # Stop the pipeline when done
        self.pipeline.stop()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseCameraNode(width=640, height=480, fps=30)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

