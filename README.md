# **larm_thibault_lemoigne**

## Introduction

Ce repository contient les différents packages liés à l'UV LARM développé par Emile Thibault et Quentin Lemoigne. Il permet de créer une carte de l'environnement à travers un robot qui est capable d'aller à un point précis transmis sur RVIZ. Il envoie également un message dans le topic /detection lorsqu'il détecte une bouteille orange ou noire.

Le contenu actuel du repo est celui du challenge 3. Des releases sont disponible sur Github pour chaque challenge :
- Challenge 1 : https://github.com/qlemoigne/larm_thibault_lemoigne/releases/tag/challenge-1
- Challenge 2 : https://github.com/qlemoigne/larm_thibault_lemoigne/releases/tag/challenge-2

## Video

https://www.youtube.com/watch?v=wO5_Bc_lHiU&ab_channel=EmileThibault

## Presentation

Ce projet est un système de navigation autonome pour un robot utilisant ROS2 (Robot Operating System). Il utilise les données de capteurs de profondeur et de caméra pour créer une carte de l'environnement et permettre au robot de se déplacer vers un point spécifié via RVIZ. Le package contient également des fonctionnalités de détection d'objets, telles que la détection de bouteilles oranges et noires, qui envoient des messages dans le topic /detection lorsqu'elles sont détectées. Les nodes clés de ce package incluent scan_echo, qui nettoie et transmet les données du laser, move, qui gère les commandes de mouvement et objects, qui effectue la détection des objets.

*Pour lancer le projet, il est possible d'utiliser les fichiers launch suivants (présents dans le package grp_vincent)*

    tbot.launch.py : Lancer l'ensemble des éléments
    simulation.launch.py : Lancer la simulation
    visualize.launch.py : Lancer la visualisation (rviz2)

On utiliseral a commande suivante : '''ros2 launch grp_vincent <nom launch file>'''


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

Le challenge 1 consiste à permettre au robot de se déplacer dans une carte, tout en évitant les obstacles.

Pour ce challenge les nodes suivantes sont utilisées :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement selon les informations du laser
- camera : Transmet les données de la camera sur un topic

---

# Challenge 2

## Résumé

Le challenge 2 de notre projet consiste à utiliser la caméra embarquée sur notre robot pour détecter les bouteilles oranges et noires dans l'environnement. 

Le script utilise les couleurs pour détécter les bouteilles oranges, il est possible d'améliorer la précision en utilisant la calibration.
Pour la détection des nouteilles noires, elle est basée sur un template et un filtre de canny.

Par ailleurs, le robot est capable de réaliser une carte de son environnement en utilisant SLAM. Il est aussi possible de lui envoyer un ordre d'aller à un point en utilisant un goal pose.

Pour ce challenge les nodes suivantes sont utilisées :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement selon les informations de commande / du laser
- objects : Capture et traitement données camera

## Topic utiles

`/detection` : Un message est envoyé lors de la détéction d'une bouteille

# Challenge 3 :

## Résumé

Le challenge 3 est simplement une evolution du challenge 2 qui integre l'apparition de marker pour chaque bouteille dans RVIZ. Pour cela on estime leur position grace a la camera de profondeur que l'on transforme dans le plan de la map. Nous avons restreint l'affichage des markers aux bouteilles orange car la détéction est plus fiable. Un espace de 0.9m entre chaque bouteilles est nécessaire, car il s'agit de notre zone d'incertitude.

Pour ce challenge les nodes suivantes sont utilisées :
- scan_echo : Nettoie et transmet les données du laser sur un topic
- move : Gére le mouvement selon les informations de commande / du laser
- objects : Capture et traitement données camera

## Ameliorations possibles

- L'utilisation de tensorFlow pour la reconnaissance de bouteilles pourrait etre envisagee. 
- Calibrer la camera en utilisant les fonctions avancees de celle-ci est une source d'ameliorations possibles.
- Moyennner les resultats publies par la webcam serait un bon moyen de reduire l'erreur caracteristique.
- Exploiter la détéction par template est aussi possible.

# Commandes utiles au debogage

Les commandes suivantes sont utiles pour debug le projet :

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
 
