import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Twist

class LineFollowerNode(Node):
    def __init__(self):
        super().__init__('line_follower_node')
        self.subscription = self.create_subscription(Image,'/video_source/raw',self.image_callback,0)
        self.subscription
        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        blur = cv2.GaussianBlur(binary, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)

        height, width = frame.shape[:2]
        middle_x = width // 2

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                
                top_pixel = (x1, y1) if y1 < y2 else (x2, y2)
                bottom_pixel = (x1, y1) if y1 > y2 else (x2, y2)
                midpoint = ((top_pixel[0] + bottom_pixel[0]) // 2, (top_pixel[1] + bottom_pixel[1]) // 2)
                
                if midpoint[0] < middle_x - middle_x // 3:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.2  # Adjust the forward speed as needed
                    cmd_vel.angular.z = 0.1  # Adjust the angular speed as needed
                    self.cmd_pub.publish(cmd_vel)
                elif midpoint[0] > middle_x + middle_x // 3:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.2  # Adjust the forward speed as needed
                    cmd_vel.angular.z = -0.1  # Adjust the angular speed as needed
                    self.cmd_pub.publish(cmd_vel)
                else:
                    cmd_vel = Twist()
                    cmd_vel.linear.x = 0.2  # Adjust the forward speed as needed
                    self.cmd_pub.publish(cmd_vel)

        cv2.imshow('Binary Camera Feed', binary)
        cv2.imshow('Line Detection', frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

