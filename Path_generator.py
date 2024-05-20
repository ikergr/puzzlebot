#!/usr/bin/env python3

import rclpy, time
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_interfaces.msg import Coordinates


class MyClassNode(Node):
  def __init__(self):
        super().__init__("path_generator")
        self.get_logger().info("Path generator node initialized ...")
        self.pub1 = self.create_publisher(Coordinates, "pose",1) 
        self.create_timer(0.03,self.timer_callback) #Seconds
        #Se declaran los parametros de cada coordenada
        self.declare_parameter('x1', 0.0)
        self.declare_parameter('y1', 0.0)
        self.declare_parameter('x2', 5.0)
        self.declare_parameter('y2', 0.0)
        self.declare_parameter('x3', 5.0)
        self.declare_parameter('y3', 5.0)
        
  def timer_callback(self):
     #Se establecen los mensajes a publicar en el tópico correspondiente
     msg = Coordinates()  	
     msg.x1 = self.get_parameter("x1").value
     msg.y1 = self.get_parameter("y1").value
     msg.x2 = self.get_parameter("x2").value
     msg.y2 = self.get_parameter("y2").value
     msg.x3 = self.get_parameter("x3").value
     msg.y3 = self.get_parameter("y3").value
     
     self.pub1.publish(msg)
    
        
def main(args=None):
   rclpy.init(args=args)
   nodeh = MyClassNode()
   try: rclpy.spin(nodeh)
   except Exception as error: print(error)
   except KeyboardInterrupt: print("Node terminated!")

if __name__ == "__main__":
   main()