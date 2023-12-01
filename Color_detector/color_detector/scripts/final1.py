#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from math import cos,sin
import numpy as np
import cv2

camera_image_array = []

def analyse(mask):
    a = 0
    n=len(mask)
    m=len(mask[0])
    b=-1
    for i in range(n):
        for j in range (m):
            if (mask[i][j] != 0):
                a = a+1
                if (b==-1):
                    b=j
    if (a>150):
        vitesse = 0
    elif (a>90):
        vitesse = 1200
    elif (a>40):
        vitesse = 18000-(200*a)
    else:
        vitesse = 12000

    alpha=0.02*((b-m/2)/360)**3


    return vitesse,alpha,a



class ImagePublisher:
    def __init__(self,args=None):
        # Initialiser le nœud ROS 2
        rclpy.init(args=args)
        self.ouuu=-1
        self.phase=1
        self.node = rclpy.create_node('camera_node')
        self.angle_pub = self.node.create_publisher(Float64, '/wamv/thrusters/main/pos', 10)
        self.thrust_pub = self.node.create_publisher(Float64, '/wamv/thrusters/main/thrust', 10)
        self.boue_pub = self.node.create_publisher(Bool, '/buoy/detect', 10)
        self.enemy_pub = self.node.create_publisher(Bool, '/ennemy/detect', 10)
        self.enemi_pose  = self.node.create_publisher(PoseStamped, '/ennemy_pose', 10)
        self.enemi_dist  = self.node.create_publisher(Float64, '/ennemy_dist', 10)

        # Créer un subscriber pour le topic image
        subscription = self.node.create_subscription(
                Image,
                '/wamv/sensors/cameras/main_camera_sensor/image_raw',
                self.image_callback,
                10
    		)
        print('en attente d image')




                

    def detector(self,a,b):
    # La fonction prend une image et renvoie si elle a trouvé la couleur du bateau ennemi dessus
    # La couleur du bateau ennemi se trouve dans cette intervalle
    # On va donc filtrer l'image obtenu pour ne garder que les pixels de la couleur du bateau ennemi
        mask = cv2.inRange(self.img, a, b)
    # Si maskred est une matrice nulle, alors le bateau ennemi n'a pas été détécté par la camera
    # Sinon, l'inverse
        vitesse,alpha,a = analyse(mask)
        trouve = not (a==0)

        return trouve,vitesse,alpha,a


    def image_callback(self,msg):

        msg_ang = Float64()
        msg_vit = Float64()
        msg_buoy = Bool()
        msg_enemy = Bool()
        self.msg_enemi_pose = PoseStamped()
        self.msg_enemi_dist = Float64()
        if self.ouuu<-0.0001:
            msg_ang.data = -0.05
        elif  self.ouuu>0.0001:
            msg_ang.data = 0.05
        msg_vit.data = 3000.0

        lowerred = np.array([3, 3, 65], dtype="uint8")
        upperred = np.array([8, 8, 75], dtype="uint8")
        loweryel = np.array([8, 98, 104], dtype="uint8")
        upperyel = np.array([14, 104, 110], dtype="uint8")
	
        bridge = CvBridge()
        self.img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


        if (self.phase ==1):
            buoy_trouve,v2,alpha2,a2 = self.detector(loweryel,upperyel)
            msg_buoy.data = buoy_trouve
            if (buoy_trouve):
                    msg_ang.data = float(alpha2)
                    msg_vit.data = float(v2)
            print(f"Boué : {buoy_trouve}, vitesse : {v2}, alpha : {msg_ang.data}")
            if (a2>120):
                self.phase = 2
                print("passage en mode chasseur")
        else:
            ennemy_trouve,v1,alpha1,a1 = self.detector(lowerred,upperred)
            msg_enemy.data = ennemy_trouve
            if (ennemy_trouve):
                msg_ang.data = float(alpha1)
                msg_vit.data = float(v1)
                if (ennemy_trouve and  abs(msg_ang.data) < 0.01):
                    self.msg_enemi_dist.data = v1/100+30
                    subscription2 = self.node.create_subscription(
                            Pose,
                            '/boat_pose',
                            self.position,
                            10
                            )
                    print(f"Enemy distance : {self.msg_enemi_dist.data},Enemy pose : {self.msg_enemi_pose.pose.position.x,self.msg_enemi_pose.pose.position.y}")

            print(f"Enemy : {ennemy_trouve}, vitesse : {msg_vit.data}, alpha : {msg_ang.data}")
            self.ouuu=msg_ang.data
        self.angle_pub.publish(msg_ang)
        self.thrust_pub.publish(msg_vit)
        self.boue_pub.publish(msg_buoy)
        self.enemy_pub.publish(msg_enemy)
        self.enemi_pose.publish(self.msg_enemi_pose)
        self.enemi_dist.publish(self.msg_enemi_dist)



    def position(self,msg):
        self.msg_enemi_pose.pose.position.x = msg.position.x + cos(msg.orientation)*self.msg_enemi_dist.data
        self.msg_enemi_pose.pose.position.y = msg.position.y + sin(msg.orientation)*self.msg_enemi_dist.data



    def run(self):
        # Tourner le nœud pour traiter les rappels
        rclpy.spin(self.node)

        # Nettoyer lorsque le nœud est arrêté
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    image_publisher = ImagePublisher()
    image_publisher.run()




