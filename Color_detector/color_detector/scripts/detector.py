#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
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
    if (a>200):
        vitesse = 0
    elif (a>120):
        vitesse = 1200
    elif (a>80):
        vitesse = 2000
    elif (a>70):
        vitesse = 4000
    elif (a>60):
        vitesse = 6000
    elif (a>50):
        vitesse = 8000
    elif (a>40):
        vitesse = 10000
    else:
        vitesse = 12000
    if (b)>520:
        alpha = 0.007
    elif (b)>380:
        alpha=-0.001
    elif (b)<200:
        alpha=-0.007
    elif (b)<340:
        alpha=-0.001
    else:
        alpha=0.0


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
        self.boue_pub = self.node.create_publisher(Bool, '/ennemy/detect', 10)

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
            print(f"Enemy : {ennemy_trouve}, vitesse : {msg_vit.data}, alpha : {msg_ang.data}")
            self.ouuu=msg_ang.data
        self.angle_pub.publish(msg_ang)
        self.thrust_pub.publish(msg_vit)

    def run(self):
        # Tourner le nœud pour traiter les rappels
        rclpy.spin(self.node)

        # Nettoyer lorsque le nœud est arrêté
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    image_publisher = ImagePublisher()
    image_publisher.run()




