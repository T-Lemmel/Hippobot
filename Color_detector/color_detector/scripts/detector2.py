#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2

camera_image_array = []


def compteur(mask):
    a = 0
    n=len(mask)
    m=len(mask[0])
    b=-1
    for i in range(n):
        for j in range (m):
            if (mask[i][j] != 0):
                a = a+1
                if (b==-1):
                    b=i
    if (a> 4300):
    	vitesse=0
    else:
    	vitesse = 12000*(1-(a/4300))**4
    alpha = (b-m/2)/m*2*1.3962634
    
    return vitesse,alpha
                


def detector(image,a,b):
    # La fonction prend une image et renvoie si elle a trouvé la couleur du bateau ennemi dessus
    
    # La couleur du bateau ennemi se trouve dans cette intervalle

    # On va donc filtrer l'image obtenu pour ne garder que les pixels de la couleur du bateau ennemi
    mask = cv2.inRange(image, a, b)
    # Si maskred est une matrice nulle, alors le bateau ennemi n'a pas été détécté par la camera
    # Sinon, l'inverse
    vitesse,alpha = compteur(mask)
    trouve = not (np.all(mask == 0)),vitesse,alpha

    return trouve

def image_callback(msg):
    lowerred = np.array([3, 3, 65], dtype="uint8")
    upperred = np.array([8, 8, 75], dtype="uint8")
    
    loweryel = np.array([8, 98, 104], dtype="uint8")
    upperyel = np.array([14, 104, 110], dtype="uint8")

    bridge = CvBridge()
    img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    ennemy_trouve,v1,alpha1 = detector(img,lowerred,upperred)
    buoy_trouve,v2,alpha2 = detector(img,loweryel,upperyel)
    

    msg_ang = Float32()
    msg_vit = Float32()
    msg_ang.data = 0.4
    msg_vit.data = 1200.0
    
    
    if (buoy_trouve):
    	msg_ang.data = float(alpha2)
    	msg_vit.data = float(v2)
    	
    
    
    
    
    
    
    print(f"Ennemy : {ennemy_trouve}, Boué : {buoy_trouve}, v : {v2}, alpha : {alpha2}")




def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('camera_node')
    
    angle_pub = node.create_publisher(Float32, '/wamv/thrusters/main/pos', 10)
        
    thrust_pub = node.create_publisher(Float32, '/wamv/thrusters/main/thrust', 10)

    # Créer un subscriber pour le topic image
    subscription = node.create_subscription(
        Image,
        '/wamv/sensors/cameras/main_camera_sensor/image_raw',
        image_callback,
        10
    )


    
    print("Waiting for images. Press Ctrl+C to exit.")

    # Entrer dans la boucle d'exécution
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")

    # Libérer les ressources
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


