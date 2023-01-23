# larm_thibault_lemoigne
Repository projet LARM

Ce projet contient les différents packages liés à l'UV LARM.

Développé par :
- Emile Thibault
- Quentin Lemoigne

# Installation 

Installer ROS2 : 
'''
https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
'''
Nous utiliserons la version Foxy de ROS2

Pour utiliser ce package, il suffit de le telecharger dans le dossier ros2_ws

Pour compiler, il suffit de lancer :
```
colcon build
```
et de sourcer :
```
source install/setup.bash
```
Enfin de lancer les launchfiles : 
```
ros2 launch <packages> <launchfile>
```

**Packages contenus :**
- tuto_move : Tutoriel lié au mouvements
- tuto_sim : Tutoriel lié à la simulation
- tuto_visio : Tutoriel lié à la vision
- grp_vincent : Lié aux challenges

# Informations de configuration
N'oubliez pas de modifier votre bashrc si vous travaillez sur differents ordis
NETWORK_ID : 42

# Challenge 2 :

## Résumé

Le robot trace une carte de son environnement et est capable d'aller à un point précis transmis sur RVIZ.

Il envoit aussi un message dans le topic /detection lorsqu'il detecte une bouteille rouge ou noire.

Ce package contient les nodes suivantes en plus de celles de base :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement
- objects : Détection des objets

## Calibration

Avant de lancer, il peut être utile de calibrer la détection en particulier pour les bouteilles orange.

Pour on peut lancer le fichier ```calibrer.py```

Deux fenêtre s'ouvrent alors :
- Retour vidéo
- Masque

TODO A compléter

## Principe de la dététection des objects

Pour les bouteilles oranges :

TODO A compléter

Pour les bouteilles orange :

TODO A compléter
On utilise un template.

## PC embarqué vincent :

Lancer Robot (Driver + Laser + Camera + Move + Détection + Mapper):

```
ros2 launch grp_vincent tbot.launch.py
```

## PC controleur robnet :

```
ros2 launch grp_vincent visiualize.launch.py
```

## Simulation :

```
ros2 launch grp_vincent simulation.launch.py
```

# Challenge 1 :

## Résumé

Ce package contient les nodes suivantes en plus de celles de base :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement selon les informations du laser
- camera : Transmet les données de la camera sur un topic

## PC embarqué vincent :

Lancer Robot (Driver + Laser + Camera + Move):

```
ros2 launch grp_vincent tbot.launch.py
```

## PC controleur robnet :

```
ros2 launch grp_vincent visiualize.launch.py
```

## Simulation :

```
ros2 launch grp_vincent simulation.launch.py
```

# Commandes interressantes

## base only
```ros2 launch tbot_start base.launch.py```

## base + laser
```ros2 launch tbot_start minimal.launch.py```

## base + with laser + camera
```ros2 launch tbot_start full.launch.py```


## Voir le graphique des frame
```
ros2 run tf2_tools view_frames.py
```

## RVIZ2 - Visualisation

```
rviz2
```
