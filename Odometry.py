#!/usr/bin/env python3
import rclpy, time, math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose2D

class PuzzlebotOdometry(Node):
  def __init__(self):
    super().__init__("puzzlebot_odometry")
    self.get_logger().info("Turtle controller node started! ...")
    
    qos_profile = QoSProfile(
      reliability = ReliabilityPolicy.BEST_EFFORT,
      durability = DurabilityPolicy.VOLATILE,
      depth=1
      )
    
    self.timer = self.create_timer(0.008, self.callback_odometry) #delta_t
    
    self.pub_odom = self.create_publisher(Pose2D,"/odom",1)
    
    self.sub_wr = self.create_subscription(Float32, "/VelocityEncR", self.callback_wr, qos_profile)
    self.sub_wl = self.create_subscription(Float32, "/VelocityEncL", self.callback_wl, qos_profile)
     
    #Inicializa los receptores de la subscripción
    self.wr = 0.0
    self.wl = 0.0
    
    #Inicializa los contadores de tiempo (delta t)
    #self.delta_t = 0.0 #Elapsed time
    #self.delta_t = 0.01 #tiempo del timer
    
    #Se inicializa la pose actual en 0,0,0
    self.pose_act = [0,0,0] #x,y,th
    
    
    
   
  def callback_wr(self,msg): #Recibe los mensajes del tópico wr (Vel llanta izq)
    self.wr = msg.data
    #print(self.wr)
    
  def callback_wl(self,msg): #Recibe los mensajes del tópico wl (Vel llanta der)
    self.wl = msg.data
    

  def callback_odometry(self):
    #Se declara el tipo de mensaje a enviar por /odom
    msg_od = Pose2D()
    
    #Valores de las dimensiones del puzzlebot
    #R = 0.05
    #d = 0.0175
  
    #Forma general para sacar odometria
    #x1 = x0+(R/2)*(wr+wl)*cos(th0)*delta_t
    #y1 = y0+(R/2)*(wr+wl)*sin(th0)*delta_t
    #th1 = th0+(R/d)*(wr-wl)*delta_t
    
    msg_od.x = self.pose_act[0]+(0.025)*(self.wr+self.wl)*math.cos(self.pose_act[2])*0.008
    msg_od.y = self.pose_act[1]+(0.025)*(self.wr+self.wl)*math.sin(self.pose_act[2])*0.008
    msg_od.theta = self.pose_act[2]+(0.2857142857)*(self.wr-self.wl)*0.008
    print(msg_od.x)
    
    self.pub_odom.publish(msg_od)
    
    self.pose_act[0] = msg_od.x
    self.pose_act[1] = msg_od.y
    self.pose_act[2] = msg_od.theta
    
    
def main(args=None):
  rclpy.init(args=args)
  nodeh = PuzzlebotOdometry()
  try: rclpy.spin(nodeh)
  except Exception as error: print(error)
  except KeyboardInterrupt: print("Node terminated by user!")
  
  
if __name__ == "__main__":
  main()
