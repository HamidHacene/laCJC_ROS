# laCJC_ROS
## Introduction
Ce projet s’inscrit dans le cadre des enseignements de l’*ENSTA Bretagne* dispensés aux élèves de deuxième année de la filière *Robotique Autonome* au sein de l’Unité d'Enseignement 4.1: *Middleware*.

## Présentation du projet
La problématique de ce projet se résume à *rendre un véhicule autonome pour accomplir un tour complet de la piste d’athlétisme de l’école*. Les véhicules mis à disposition sont des voitures à l’échelle 1/10ème construites dans un autre cours.

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/voiture.jpg" width="300" height="200" style="float:left;">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/terrain.jpg" width="300" height="200" style="float:right;"> <br>
    <em>Véhicule utilisé et piste d'athlétisme</em>
</p>

## Démonstration
Voici deux démonstration des résultats obtenus : 
  * https://youtu.be/LQ9LN_navPc
  * https://youtu.be/2UC-Q0rVsu8

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/test_res.png" width="600" height="400"> <br>
    <em>Résultats</em>
</p>

## Installation des librairies
Pour utiliser les codes sources ou tester les programmes, il faut installer les librairies suivantes : 

###### xtl :
  * Download zip from : https://github.com/xtensor-stack/xtl
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
  ```


###### xtensor : 
  * Download zip from : https://github.com/xtensor-stack/xtensor
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
  ```

###### xtensor-blas : 
  * Download zip from : https://github.com/xtensor-stack/xtensor-blas
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
  ```


###### openBlas :
  ```
    $ sudo apt-get install libopenblas-dev
  ```

###### ROS PID : 
  ```
    $ sudo apt-get install ros-melodic-pid
  ```

## Usage
Afin de lancer le simulateur, il faut suivre les étapes suivantes :

###### Téléchargements :
1. Télécharger ce dépôt git :
  ```bash
    $ git clone https://github.com/HamidHacene/laCJC_ROS.git
  ```
2. Télécharger *V-REP* : https://coppeliarobotics.com/downloads

###### Lancement des programmes :
 
  * Lancer *ROS* :
  ```bash
    $ roscore
  ```
  * Lancer *V-REP* : (dans le dossier téléchargé précédemment)
  ```bash
    $ ./vrep.sh
  ```
  * Ouvrir la scène `laCJC_ROS/vrep_simulation/seance4_modele_realiste_et_piste.ttt` dans *V-REP*
  * Compiler les sources (sans oublier de faire un source avec `devel/setup.bash`): 
  ```bash
    $ cd laCJC_ROS/ROS_package
    $ catkin_make
  ```
  * Dans un autre terminal, lancer *rqt*
  ```bash
    $ rqt
  ```
  * Une fois la simulation sour *V-REP* lancé (bouton play), saisir  cette instruction dans un terminal : 
  ```bash
    $ roslaunch waypoints_follow vrep_launch.launch
  ```
  * Une fenêtre ressemblant à l'image ci-dessous doit apparaître :  
<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/Lane_Detection/data/visual.png" width="600" height="250"> <br>
    <em>Visualisation du traitement</em>
</p>

  * Il reste qu'à contrôler la vitesse d'avance et les coefficients du *PID* avec *rqt* :
<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/inter_ctrl.png" width="700" height="250"> <br>
    <em>Contrôle avec rqt </em>
</p>

## Auteurs
  * Antonin LIZÉ - [AntoninLize](https://github.com/AntoninLize)
  * Colin BAUMGARD - [ColinBaumgard](https://github.com/ColinBaumgard)
  * Corentin LEMOINE - [Pafnouti](https://github.com/Pafnouti)
  * Hamid HACENE - [HamidHacene](https://github.com/HamidHacene)
  * Ludovic DIGUET - [LudovicDiguet](https://github.com/LudovicDiguet) 


