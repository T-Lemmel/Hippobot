#!/usr/bin/env python3

from hashlib import algorithms_available
from geometry_msgs.msg import PoseArray, Point, Pose
from sensor_msgs.msg import PointCloud2
import copy
from rclpy.node import Node
import rclpy
import math

class EnemyShipPredictor(Node):
    def __init__(self):
        super().__init__('enemy_ship_predictor')
        self.subscription = self.create_subscription(
            PoseArray,
            '/vrx/patrolandfollow/alert_position',
            self.enemy_ship_pose_callback,
            10
        )
        self.enemy_positions = [None, None]  # Lista que contiene las dos posiciones del enemigo
        self.time_interval = 1  # Intervalo de tiempo entre mediciones en segundos
        self.publisher = self.create_publisher(Pose, 'predicted_enemy_ship_pose', 10)

    def enemy_ship_pose_callback(self, msg: PoseArray):
        enemy_position = msg.pose.position

        if self.enemy_positions[0] is None:
            self.enemy_positions[0] = enemy_position
            self.get_logger().info('Enemy''s initial position: (%f, %f, %f)' % (self.enemy_positions[0].x, self.enemy_positions[0].y, self.enemy_positions[0].z))
        else:
            self.enemy_positions[1] = self.enemy_positions[0]  # Movemos la última posición al historial
            self.enemy_positions[0] = enemy_position  # Actualizamos la posición actual

            predicted_enemy_pose = self.predict_enemy_position()
            self.publish_predicted_position(predicted_enemy_pose)
            self.get_logger().info('Enemy''s predicted position: (%f, %f, %f)' % (predicted_enemy_pose.position.x, predicted_enemy_pose.position.y, predicted_enemy_pose.position.z))

    def predict_enemy_position(self):
        if self.enemy_positions[1] is not None:
            delta_x = self.enemy_positions[0].x - self.enemy_positions[1].x
            delta_y = self.enemy_positions[0].y - self.enemy_positions[1].y

            velocity_x = delta_x / self.time_interval
            velocity_y = delta_y / self.time_interval

            predicted_pose = Pose()
            predicted_pose.position.x = self.enemy_positions[0].x + velocity_x * self.time_interval
            predicted_pose.position.y = self.enemy_positions[0].y + velocity_y * self.time_interval
            predicted_pose.position.z = self.enemy_positions[0].z  # No hay cambio en la coordenada z

            return predicted_pose

    def publish_predicted_position(self, predicted_pose):
        self.publisher.publish(predicted_pose)

def main(args=None):
    rclpy.init(args=args)
    enemy_ship_predictor = EnemyShipPredictor()
    rclpy.spin(enemy_ship_predictor)
    enemy_ship_predictor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

