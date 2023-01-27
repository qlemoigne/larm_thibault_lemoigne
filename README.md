# **larm_thibault_lemoigne**

## Introduction

Ce repository contient les différents packages liés à l'UV LARM développé par Emile Thibault et Quentin Lemoigne. Il permet de créer une carte de l'environnement à travers un robot qui est capable d'aller à un point précis transmis sur RVIZ. Il envoie également un message dans le topic /detection lorsqu'il détecte une bouteille orange ou noire.

## Presentation

Ce projet est un système de navigation autonome pour un robot utilisant ROS2 (Robot Operating System). Il utilise les données de capteurs de profondeur et de caméra pour créer une carte de l'environnement et permettre au robot de se déplacer vers un point spécifié via RVIZ. Le package contient également des fonctionnalités de détection d'objets, telles que la détection de bouteilles oranges et noires, qui envoient des messages dans le topic /detection lorsqu'elles sont détectées. Les nodes clés de ce package incluent scan_echo, qui nettoie et transmet les données du laser, move, qui gère les commandes de mouvement et objects, qui effectue la détection des objets.

*Pour lancer le projet, il est possible d'utiliser les fichiers launch suivants (présents dans le package grp_vincent)*

    tbot.launch.py : Lancer l'ensemble des éléments
    simulation.launch.py : Lancer la simulation
    visualize.launch.py : Lancer la visualisation (rviz2)


## Installation

    Installer ROS2 en suivant les instructions de cette page : https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html. Nous utiliserons la version Foxy de ROS2.

    Télécharger ce package dans le dossier ros2_ws

    Compiler le package en exécutant la commande colcon build

    Sourcer le package en exécutant la commande source install/setup.bash

    Lancer les launchfiles en utilisant la commande ros2.  launch <package> <launchfile>

## Packages contenus :

    tuto_move : Tutoriel lié aux mouvements
    tuto_sim : Tutoriel lié à la simulation
    tuto_visio : Tutoriel lié à la vision
    grp_vincent : Lié aux challenges

## Informations de configuration

N'oubliez pas de configurer votre fichier bashrc si vous travaillez sur différents ordinateurs. Il est important de définir une variable **NETWORK_ID** identique sur tous les ordis travaillant sur le projet, dans notre cas **42**.

## Calibration

Dans le cadre des challenges, une analyse de la caméra est réalisée, celle-ci se fait dans le noeud objects.py.

La détéction des bouteilles orange se base sur la couleur, il est nécessaire de réaliser une calibration du mask.

Pour cela, il suffit de lancer le fichier calibrer.py (avec python3), deux fenêtres s'ouvrent alors :
- retour vidéo
- vidéo filtrée (masque)

En cliquant gauche sur un point de la fenêtre vidéo, le point sera ajouté au mask. La zone d'acceptation est définie par les sliders et est appliquée à chaque point. Il est possible d'ajouter autant de points que l'on veut. Lorsqu'une bouteille est détéctée, un rectangle est desiné.

Le but est de voir et détecter les bouteilles dans le masque tout en évitant d'avoir le sol.

Un clic droit dans la fenêtre ferme le programme et envoit le code associé aux filtres qui est à copier dans le fichier objects.py. Il faut penser à récompiler.

---

# Challenge 1 :

## Résumé

Ce package contient les nodes suivantes en plus de celles de base :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement selon les informations du laser
- camera : Transmet les données de la camera sur un topic

## PC embarqué vincent :

Lancer Robot (Driver + Laser + Camera + Move + Détection + Mapper):

```
ros2 launch grp_vincent tbot.launch.py
```

## PC controleur robnet - visualisation :

```
ros2 launch grp_vincent visualize.launch.py
```

## Simulation :

```
ros2 launch grp_vincent simulation.launch.py
```

---

# Challenge 2 : Traitement d'image de la Kinect embarquée

Le challenge 2 de notre projet consiste à utiliser la Kinect embarquée sur notre robot pour détecter les bouteilles oranges et noires dans l'environnement. Pour ce faire, nous avons écrit un script Python qui utilise les bibliothèques `OpenCV` et `Pyrealsense2` pour traiter les images capturées par la Kinect.

### Prérequis

* Avoir installé la bibliothèque Pyrealsense2

* Avoir installé OpenCV pour Python

### Utilisation

Pour utiliser ce script, il suffit de lancer la commande suivante :
```
ros2 run <nom_du_package> objects_detector.py
```
Ce script s'abonne aux topics de la Kinect pour récupérer les images de la caméra et de la profondeur. Il utilise ensuite divers filtres et techniques de traitement d'image pour détecter les bouteilles oranges et noires dans l'environnement. Il publie ensuite les résultats de ces détections sur un topic spécifique.

Il est important de noter que le script utilise des filtres de couleur prédéfinis pour détecter les bouteilles oranges et noires, et qu'il est possible de les calibrer en modifiant les variables staticLow et staticHigh pour les bouteilles oranges, et les variables orangeLow et orangeHigh pour les bouteilles oranges dans le script. Il est également possible de changer les paramètres de détection de bouteilles noires en modifiant le fichier de template utilisé et les paramètres de filtre de contours.

Il est important de noter que le script utilise également une méthode de détection basée sur un template pour détecter les bouteilles noires, ce qui peut entraîner des erreurs dans les résultats si les bouteilles noires dans l'environnement ont des dimensions ou des orientations différentes de celles utilisées pour créer le template.

Enfin, il est important de noter que le script utilise une méthode de détection basée sur la profondeur pour détecter les bouteilles oranges, qui peut entraîner des erreurs dans les résultats si les bouteilles oranges dans l'environnement ont des couleurs similaires à celles utilisées pour créer les filtres de couleur.

### Résultats

Les résultats de la détection des bouteilles oranges
et noires sont publiés sur le topic `/detection`

# Challenge 3 :

Le challenge 3 est simplement une evolution du challenge 2 qui integre l'apparition de modele de bouteille dans RVIZ. Pour cela on estime leur position grace a la camera de profondeur que l'on transforme dans le plan de la map. Nous avons restreint l'affichage des markers aux bouteilles orange car la détéction est plus fiable. Un espace de 0.9m entre chaque bouteilles est nécessaire, car il s'agit de notre zone d'incertitude.

# Ameliorations possibles

L'utilisation de tensorFlow pour la reconnaissance de bouteilles pourrait etre envisagee. 
Calibrer la camera en utilisant les fonctions avancees de celle-ci est une source d'ameliorations possibles.
Moyennner les resultats publies par la webcam serait un bon moyen de reduire l'erreur caracteristique.
Exploiter la détéction par template est aussi possible.

# Commandes utiles

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
 
