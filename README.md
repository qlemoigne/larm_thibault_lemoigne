# larm_thibault_lemoigne
Repo projet LARM

## Commandes globales

#### Lancer le driver du robot :

```
ros2 run tbot_start start_base
```

#### Lancer le bridge :

```
ros2 run ros1_bridge dynamic_bridge
```

#### Lancer le scanner laser :
```
ros2 run urg_node urg_node_driver --ros-args -p serial_port:=/dev/ttyACM0
```

## Outils

#### VOir le graphique des frame
```
ros2 run tf2_tools view_frames.py
```

## Projets

#### Lancer eviter obscale :
```
ros2 run tuto_move scan_echo
```
```
ros2 run tuto_move eviter_obstacle
```

#### Mapping