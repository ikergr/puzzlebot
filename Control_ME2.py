#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from std_msgs.msg import Float32, Int8, Int32
from geometry_msgs.msg import Twist, Pose2D
from my_interfaces.msg import Coordinates


class PuzzlebotController(Node):
  def __init__(self):
    super().__init__("puzzlebot_controller")
    self.get_logger().info("puzzlebot controller node started! ...")
    
    #self.timer = self.create_timer(0.0066, self.callback_controller)
    
    self.pub_cmd_vel = self.create_publisher(Twist, "/cmd_vel", 1)
    #self.pub_stop = self.create_publisher(Int8,"stop",1)
    
    self.sub_point = self.create_subscription(Coordinates, "/point", self.callback_point, 1)
    self.sub_odom = self.create_subscription(Pose2D, "/odom", self.callback_odom, 1)
    self.sub_semaforo = self.create_subscription(Int32, "/traffic_light", self.callback_semaforo, 1)
    
    
    
    #Prueba con lista de puntos desde el controlador
    self.x1 = 0.0
    self.y1 = 0.0
    self.x2 = 0.0
    self.y2 = 0.0
    self.x3 = 0.0
    self.y3 = 0.0
    self.x4 = 0.0
    self.y4 = 0.0
    self.x5 = 0.0
    self.y5 = 0.0
    self.x6 = 0.0
    self.y6 = 0.0
    
    
    
    #se definen los valores a usar
    self.pose = None
    self.tolerance_p = 0.08
    self.tolerance_a = 0.188
    self.kW = 3.7
    self.kL = 1.7
    
    self.L = 0.9
    
    self.coord = 0
    self.stop = 0
    
    self.msg_cmd_vel=Twist()
    
    self.i = 0
    
    self.listaP = ((0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0),(0.0,0.0))
    
    self.mov = 1
    
    #Semaforo
    self.semaforo = 0  #0 = rojo, 1 = amarillo, 2 = verde
    self.state = "start"
    self.ME = 0
    
  def callback_point(self,msg):
    #Recibe los valores de la posicion deseada
    self.x1 = msg.x1
    self.y1 = msg.y1
    self.x2 = msg.x2
    self.y2 = msg.y2
    self.x3 = msg.x3
    self.y3 = msg.y3
    self.x4 = msg.x4
    self.y4 = msg.y4
    self.x5 = msg.x5
    self.y5 = msg.y5
    self.x6 = msg.x6
    self.y6 = msg.y6
    self.listaP = ((self.x1,self.y1),(self.x2,self.y2),(self.x3,self.y3),(self.x4,self.y4),(self.x5,self.y5),(self.x6,self.y6))
    self.coord = 1
    
    
  def callback_semaforo(self,msg):
    self.semaforo = msg.data
    if self.state == "start": self.ME = 0
    elif self.state == "red": self.ME = 0
    elif self.state == "yellow": self.ME = 0.5
    elif self.state == "green": self.ME = 1
    
    if self.state == "start" and self.semaforo == 0: self.state = "red"
    elif self.state == "start" and self.semaforo == 2: self.state = "green"
    elif self.state == "green" and self.semaforo == 1: self.state = "yellow"
    elif self.state == "yellow" and self.semaforo == 0: self.state = "red"
    elif self.state == "red" and self.semaforo == 2: self.state = "green"
    
    print(self.ME)
    
    
  def callback_odom(self,msg_odom):
    x = msg_odom.x
    y = msg_odom.y
    theta = msg_odom.theta
    x1 = self.listaP[self.i][0]
    y1 = self.listaP[self.i][1]
    dx = x1 - x
    dy = y1 - y
    errd = math.sqrt(math.pow(dx,2) + math.pow(dy,2))
    
    #print(errd)
    
    theta1 = math.atan2(dy,dx)
    
    errh = math.atan2(math.sin(theta1-theta), math.cos(theta1-theta))
    #print(errh)
    
    if self.coord == 1:
      if self.stop == 0:
      
        if self.mov == 1:
          if (abs(errh) >= self.tolerance_a):
            w = max(min(0.4,self.kW * errh),-0.4)
          
            self.msg_cmd_vel.angular.z = w * self.ME
            self.msg_cmd_vel.linear.x = 0.0
            self.pub_cmd_vel.publish(self.msg_cmd_vel)
            
          else:
            print("Angulo")
            self.mov = 2
            
        else:
          if (abs(errd) > self.tolerance_p):
            
            #Pure persuit
            sq = math.sin(theta)
            cq = math.cos(theta)
            w = (dy*cq - dx*sq)/(self.L)
            v = (dx + self.L*w*sq)/cq
            
            w = max(min(0.3,w),-0.3)
            v = max(min(0.08,v),-0.08)
              
            
            self.msg_cmd_vel.linear.x = v * self.ME
            self.msg_cmd_vel.angular.z = w * self.ME
            self.pub_cmd_vel.publish(self.msg_cmd_vel)
              
          else:
            print("distancia")
            self.mov = 1
            if self.i < 5:
              self.i += 1
            else:
              self.stop = 1
              print("Condition reached. Stopping robot")
      else:
        self.msg_cmd_vel.linear.x = 0.0
        self.msg_cmd_vel.angular.y = 0.0
        self.pub_cmd_vel.publish(self.msg_cmd_vel)
        
           
           
    
def main(args=None):
  rclpy.init(args=args)
  nodeh = PuzzlebotController()
  try: rclpy.spin(nodeh)
  except Exception as error: print(error)
  except KeyboardInterrupt: print("Node terminated by user!")
  
  
if __name__ == "__main__":
  main()
