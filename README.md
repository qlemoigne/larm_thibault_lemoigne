# larm_thibault_lemoigne
Repo projet LARM

Ce projet contient les différents packages liés à l'UV LARM.

**Packages contenus : **
- tuto_move : Tutoriel lié au mouvements
- tuto_sim : Tutoriel lié à la simulation
- tuto_visio : Tutoriel lié à la vision
- challenge1 : Lié au premier challenge 1

## Informations de configuration

NETWORK_ID : 4

## Challenge 1 :

### PC embarqué vincent :

Lancer simulator :
```
ros2 launch challenge1 simulation.launch.py
```

- Driver robot : ```ros2 run tbot_start start_base```
- Bridge : ```ros2 run ros1_bridge dynamic_bridge```
- Laser Scanner : ```ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0```
- Node Camera : ```ros2 run challenge1 camera```
- Node Scan Echo : ```ros2 run challenge1 scan_echo```
- Node Move : ```ros2 run challenge1 move```

### PC controleur robnet :

- RVIZ2 : ```ros2 run rvid2```

## Commandes globales

### base only
```ros2 launch tbot_start base.launch.py```r

### base + laser
```ros2 launch tbot_start minimal.launch.py```

### base + with laser + camera
```ros2 launch tbot_start full.launch.py```


### VOir le graphique des frame
```
ros2 run tf2_tools view_frames.py
```

### RVIZ2

```
rviz2
```

## Projets

### Lancer eviter obscale :
```
ros2 run tuto_move scan_echo
```
```
ros2 run tuto_move eviter_obstacle
```