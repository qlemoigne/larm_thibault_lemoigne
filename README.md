# larm_thibault_lemoigne
Repository projet LARM

Ce projet contient les différents packages liés à l'UV LARM.

**Packages contenus :**
- tuto_move : Tutoriel lié au mouvements
- tuto_sim : Tutoriel lié à la simulation
- tuto_visio : Tutoriel lié à la vision
- grp_vincent : Lié aux challenges

## Informations de configuration

NETWORK_ID : 42

## Challenge 1 :

### PC embarqué vincent :

Lancer Robot (Driver + Laser + Camera + Move):
```
ros2 launch grp_vincent tbot.launch.py
```
### PC controleur robnet :

```
ros2 launch grp_vincent visiualize.launch.py
```

### Simulation :

```
ros2 launch grp_vincent simulation.launch.py
```

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