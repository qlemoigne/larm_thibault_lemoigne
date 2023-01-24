# larm_thibault_lemoigne

## Introduction

Ce repository contient les différents packages liés à l'UV LARM développé par Emile Thibault et Quentin Lemoigne. Il permet de créer une carte de l'environnement à travers un robot qui est capable d'aller à un point précis transmis sur RVIZ. Il envoie également un message dans le topic /detection lorsqu'il détecte une bouteille rouge ou noire.


## Installation

    Installer ROS2 en suivant les instructions de cette page : https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html. Nous utiliserons la version Foxy de ROS2.

    Télécharger ce package dans le dossier ros2_ws

    Compiler le package en exécutant la commande colcon build

    Sourcer le package en exécutant la commande source install/setup.bash

    Lancer les launchfiles en utilisant la commande ros2 launch <package> <launchfile>

## Packages contenus :

    tuto_move : Tutoriel lié aux mouvements
    tuto_sim : Tutoriel lié à la simulation
    tuto_visio : Tutoriel lié à la vision
    grp_vincent : Lié aux challenges

## Informations de configuration

N'oubliez pas de configurer votre fichier bashrc si vous travaillez sur différents ordinateurs. Il est important de définir la variable NETWORK_ID à 42.

## Calibration

Avant de lancer le package, il peut être utile de calibrer la détection en particulier pour les bouteilles orange. Pour cela, vous pouvez exécuter le fichier calibrer.py. Deux fenêtres s'ouvriront alors : une pour le retour vidéo et une pour le masque.


## Résumé

Le robot trace une carte de son environnement et est capable d'aller à un point précis transmis sur RVIZ.

Il envoit aussi un message dans le topic /detection lorsqu'il detecte une bouteille rouge ou noire.

Ce package contient les nodes suivantes en plus de celles de base :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement
- objects : Détection des objets

## PC embarqué vincent :

Lancer Robot (Driver + Laser + Camera + Move + Détection + Mapper):

```
ros2 launch grp_vincent tbot.launch.py
```

## PC controleur robnet - visualisation :

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

# Challenge 2 : Traitement d'image de la Kinect embarquée

Le challenge 2 de notre projet consiste à utiliser la Kinect embarquée sur notre robot pour détecter les bouteilles rouges et noires dans l'environnement. Pour ce faire, nous avons écrit un script Python qui utilise les bibliothèques `OpenCV` et `Pyrealsense2` pour traiter les images capturées par la Kinect.
### Prérequis

    > Avoir installé la bibliothèque Pyrealsense2
    >
    > Avoir installé OpenCV pour Python

### Utilisation

Pour utiliser ce script, il suffit de lancer la commande suivante :
```
ros2 run <nom_du_package> objects_detector.py
```
Ce script s'abonne aux topics de la Kinect pour récupérer les images de la caméra et de la profondeur. Il utilise ensuite divers filtres et techniques de traitement d'image pour détecter les bouteilles rouges et noires dans l'environnement. Il publie ensuite les résultats de ces détections sur un topic spécifique.

Il est important de noter que le script utilise des filtres de couleur prédéfinis pour détecter les bouteilles rouges et noires, et qu'il est possible de les calibrer en modifiant les variables staticLow et staticHigh pour les bouteilles rouges, et les variables orangeLow et orangeHigh pour les bouteilles oranges dans le script. Il est également possible de changer les paramètres de détection de bouteilles noires en modifiant le fichier de template utilisé et les paramètres de filtre de contours.

Il est important de noter que le script utilise également une méthode de détection basée sur un template pour détecter les bouteilles noires, ce qui peut entraîner des erreurs dans les résultats si les bouteilles noires dans l'environnement ont des dimensions ou des orientations différentes de celles utilisées pour créer le template.

Enfin, il est important de noter que le script utilise une méthode de détection basée sur la profondeur pour détecter les bouteilles rouges, qui peut entraîner des erreurs dans les résultats si les bouteilles rouges dans l'environnement ont des couleurs similaires à celles utilisées pour créer les filtres de couleur.

### Résultats

Les résultats de la détection des bouteilles rouges et noires sont publiés sur le topic `/detection`

# Arborescence :

*Fichiers executables*

calibrer.py
> /grp_vincent/grp_vincent/
>                          move.py
>                          objects.py
>                          scan_echo.py
>             /launch/_pycache/
>                              simulation.launch.py
>                              tbot.launch.py
>                              visualize.launch.py

## Description des fichiers
                        
      

## Commandes utiles

Les commandes suivantes sont utiles pour lancer les différents composants de notre projet :

**Base uniquement**
```
ros2 launch tbot_start base.launch.py
```
**Base + laser**
```
ros2 launch tbot_start minimal.launch.py
```
**Base + laser + camera**
```
ros2 launch tbot_start full.launch.py
```
**Visualisation des frames**
```
ros2 run tf2_tools view_frames.py
```
**Visualisation avec RVIZ2**
```
rviz2
```
Ces commandes permettent de lancer les différents composants de notre projet, de visualiser les frames de notre système et de visualiser les données de capteur à l'aide de RVIZ2. Il est important de noter que vous devrez peut-être ajuster les chemins et les noms de fichiers de lancement en fonction de votre configuration.
