# larm_thibault_lemoigne
Repository projet LARM

Ce projet contient les différents packages liés à l'UV LARM.

**Packages contenus :**
- tuto_move : Tutoriel lié au mouvements
- tuto_sim : Tutoriel lié à la simulation
- tuto_visio : Tutoriel lié à la vision
- challenge1 : Lié au premier challenge 1

## Informations de configuration

NETWORK_ID : 4

## Challenge 1 :

### PC embarqué vincent :

Lancer simulateur :
```
ros2 launch challenge1 simulation.launch.py
```

- Node Camera : ```ros2 run challenge1 camera```
- Node Scan Echo : ```ros2 run challenge1 scan_echo```
- Node Move : ```ros2 run challenge1 move```

### PC controleur robnet :

- RVIZ2 : ```ros2 run rvid2```

## Commandes globales

### base only
```ros2 launch tbot_start base.launch.py```

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