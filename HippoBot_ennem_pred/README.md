# HippoBot_ennem_pred
This is a repository containing the package of prediction of a position based on a previous trajectory of a boat in ROS2

Rapport - Prédiction de la position des navires ennemis

Ce code a été développé pour prédire la position future des navires ennemis en se basant sur les positions initiales reçues à chaque message.

La classe EnemyShipPredictor est une extension de la classe Node de RCLPY. Elle est responsable de la réception des messages de type PoseArray contenant les positions des navires ennemis.

Fonctionnalités clés :
- Réception des positions initiales des navires ennemis dans le topic '/vrx/patrolandfollow/alert_position'.
- Stockage des deux dernières positions des navires ennemis dans une liste enemy_positions.
- Calcul de la position prédite du navire ennemi en fonction des deux dernières positions.
- Publication de la position prédite du navire ennemi dans le topic 'predicted_enemy_ship_pose'.

Fonctionnement :
1. À la réception du premier message de position initiale :
    - La position est enregistrée comme la position actuelle (self.enemy_positions[0]).
    - Attente du prochain message.

2. À la réception du deuxième message avec une nouvelle position :
    - La dernière position est stockée dans self.enemy_positions[1] (historique).
    - La position actuelle est mise à jour dans self.enemy_positions[0].
