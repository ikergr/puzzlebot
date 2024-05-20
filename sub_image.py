#!/usr/bin/env python3

import rclpy, time, cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

from geometry_msgs.msg import Twist

class OpenCVBridge(Node):
        def __init__(self):
                super().__init__("sub_image")
                self.get_logger().info("Image Subscription Node started [...]")
                self.img = None
                self.bridge = CvBridge()
                self.create_subscription(Image, '/video_source/raw', self.camera_callback, 10)
                self.pub1 = self.create_publisher(Int32, "/traffic_light", 1)
                #self.pub = self.create_publisher(Twist, "/cmd_vel", 1)
                self.timer = self.create_timer(0.01, self.timer_callback)
                
                #Color range:
                #Red
                self.lower_R1 = np.array([0, 100, 100])
                self.higher_R1 = np.array([10, 255, 255])
                self.lower_R2 = np.array([160, 100, 100])
                self.higher_R2 = np.array([179, 255, 255])
                
                #Green
                #self.lower_G1 = np.array([35, 40, 40])
                #self.higher_G1 = np.array([85, 255, 255])
                
                
                #Green
                self.lower_G1 = np.array([75, 54, 79])
                self.higher_G1 = np.array([101, 130, 255])
                
                #1Yellow
                #self.lower_Y1 = np.array([15,100,100])
                #self.higher_Y1 = np.array([35,255,255])
                
                #2Yellow
                self.lower_Y1 = np.array([16,88,120])
                self.higher_Y1 = np.array([50,136,255])
                
                #self.brightness_value = int((brightness - 50)
                #self.contrast_value = 50
                

      
        def camera_callback(self, msg):
                self.img = self.bridge.imgmsg_to_cv2(msg, "bgr8") #rgb8 (color) #bgr8 #mono8 (b&w)
                #NO AGREGAR NADA AQUÍ 
                
        def timer_callback(self):
                
                if self.img is not None:
                        
                        cv2.imshow('Original Image', self.img)
                        
                        right_side_image = self.img[:, self.img.shape[1]//2:]
                        #cv2.imshow('Received Image', self.img
                        
                        # IMG to HSV conversion 
                        frame_hsv = cv2.cvtColor(right_side_image, cv2.COLOR_BGR2HSV)
                        #Masks
                        mask_red1 = cv2.inRange(frame_hsv, self.lower_R1, self.higher_R1)
                        mask_red2 = cv2.inRange(frame_hsv, self.lower_R2, self.higher_R2)
                        mask_red = cv2.bitwise_or(mask_red1, mask_red2)
                        
                        mask_yellow = cv2.inRange(frame_hsv, self.lower_Y1, self.higher_Y1)
                        mask_green = cv2.inRange(frame_hsv, self.lower_G1, self.higher_G1)
                        
                        mask_RY = cv2.bitwise_or(mask_red, mask_yellow)
                        mask_All = cv2.bitwise_or(mask_RY, mask_green)
                         
                        color_detection = cv2.bitwise_and(right_side_image, right_side_image, mask=mask_All)
                        
                        
                        #Circle Detection
                        gray = cv2.cvtColor(color_detection, cv2.COLOR_BGR2GRAY)
                        gray_blurred = cv2.GaussianBlur(gray, (5,5), 0)
                        rows = gray_blurred.shape[0]
                        circles = cv2.HoughCircles(gray_blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=rows/8, param1=50, param2=30, minRadius=15, maxRadius=50)
                        msg = Int32()
                        #Circle Labeling
                        if circles is not None:
                                circles = np.uint16(np.around(circles))
                                color_pr="None"
                                
                                for i in circles[0, :]:
                                        center = (i[0], i[1])
                                        radius = i[2]
                                        color = (0,0,0)

                                        if mask_yellow[i[1], i[0]] > 0:
                                                color = (0, 255, 255) 
                                                color_pr = "Amarillo"
                                                msg.data = 1   
                                                
                                                
                                        elif mask_green[i[1], i[0]] > 0:
                                                color = (0, 255, 0)  
                                                color_pr = "Verde"
                                                msg.data = 2
                                                 
                                        elif mask_red[i[1], i[0]] > 0:
                                                color = (0, 0, 255) 
                                                color_pr = "Rojo"
                                                msg.data = 0
                                                
                                        #Twist message published
                                        self.pub1.publish(msg)
                                        #Circle marked
                                        cv2.circle(color_detection, center, radius, color,2)
                                        cv2.putText(color_detection, 'Color: ' + color_pr, (i[0]-10, i[1] + radius + 20),  cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
                        #Final image w/detectionSS
                        cv2.imshow('Colors', color_detection)
                        cv2.waitKey(1)
                        
                        

         
def main(args=None):
        rclpy.init(args=args)
        nodeh = OpenCVBridge()
        try: rclpy.spin(nodeh)
        except Exception as error: print(error)
        except KeyboardInterrupt: print("\n","Node terminated!")

if __name__ == "__main__":
        main()

