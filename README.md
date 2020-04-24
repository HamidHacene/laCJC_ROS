# laCJC_ROS
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
