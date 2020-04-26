# laCJC_ROS
## Introduction
Ce projet s’inscrit dans le cadre des enseignements de l’*ENSTA Bretagne* dispensés aux élèves de deuxième année de la filière *Robotique Autonome* au sein de l’Unité d'Eenseignement 4.1: *Middleware*.

## Présentation du projet
La problématique de ce projet se résume à *rendre un véhicule autonome pour accomplir un tour complet de la piste d’athlétisme de l’école*. Les véhicules mis à disposition sont des voitures à l’échelle 1/10ème construites dans un autre cours.

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/voiture.jpg"> <br>
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/docs/rapport_final/imgs/voiture.jpg"> <br>
    <em>Véhicule utilisé</em>
</p>



## Démonstration

https://youtu.be/LQ9LN_navPc

## Installation des librairies pour "TNI"


###### xtl : 
 
  * Download zip from : https://github.com/xtensor-stack/xtl
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
    $ cd ../..
  ```


###### xtensor : 
 
  * Download zip from : https://github.com/xtensor-stack/xtensor
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
    $ cd ../..
    $ rm -r xtensor-master
  ```

###### xtensor-blas : 
 
  * Download zip from : https://github.com/xtensor-stack/xtensor-blas
 
  * Extract files and open a terminal in the extracted folder : 
  ```bash
    $ mkdir build 
    $ cd build
    $ cmake -D CMAKE_INSTALL_PREFIX=/usr/local ..
    $ sudo make install
    $ cd ../..
  ```


###### openBlas : 

  ```
    $ sudo apt-get install libopenblas-dev
  ```

## Lane detection

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/Lane_Detection/data/virageG.png"> <br>
    <em>Raw image</em>
</p>

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/Lane_Detection/data/BEV.png"> <br>
    <em>Bird Eye View</em>
</p>

<p align="center">
    <img src="https://github.com/HamidHacene/laCJC_ROS/blob/master/Lane_Detection/data/fit.png"> <br>
    <em>Polynomial fit</em>
</p>
