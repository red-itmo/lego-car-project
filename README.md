Lego car-like robot stuff
====================

The project idea is a mobile car-type robot capable of mimicking human-like trajectories to reach its assigned position while avoiding obstacles.

This repository includes the source code for both its control system and computer vision system (CV). The CV was designed for robot localization and obstacle detection. 


[Please take a loot at the video first.](https://www.youtube.com/watch?v=3-zPCO2_tgE)

## Table of Contents

- [Description](#description)
- [Installation](#installation)
- [Usage](#usage)
- [Support](#support)
- [Contributing](#contributing)

## Description

The repository consists of three parts:

* The `control_part` folder includes the source code of the control system
* The `cv_part` folder includes the source code of the computer vision systel
* The `scilab` folder includes files associated with the simulation of the control system in [Scilab](https://github.com/opencollab/scilab)

The Car-type robot was built using the LEGO MIDSTRMS EV3 set. The general view of the robot and its kinematic model are shown in the figures below.

Car-like robot | Kinematic model
:-------------------------:|:-------------------------:
<img src="https://pp.userapi.com/c848536/v848536279/87218/9GxvTIaiKUI.jpg" width="200"/> | <img src="https://pp.userapi.com/c848536/v848536279/87220/8wans4_NS30.jpg" width="200"/>

The whole structure of the project is illustrated on the figure below.

<div style="text-align: center;"><img src="https://pp.userapi.com/c851320/v851320556/106bc/aICo-xGfYvE.jpg" height="150"/></div>

## Installation

### Robot

1. Build the car-type robot ([Ackermann Steering tutorial](http://www.moc-pages.com/moc.php/320971)) using [LEGO mindstorms EV3](https://www.lego.com/ru-ru/mindstorms)
2. Creating a bootable Micro SD card with [Linux](http://www.ev3dev.org/docs/getting-started/)
3. Connect the EV3 to a WiFi network using a WiFi adapter (eg. a doodle)
4. Connect via SSH to your robot (user: robot, password: maker)

    ```bash
    ssh robot@XXX.XXX.XXX.XXX (IP adress of the robot)
    ```
5. Clone the repository or copy only the `control_part` folder to your robot

    ```bash 
    cd ~
    git clone https://github.com/red-itmo/lego-car-project.git     
    ```

6. Change permissions for Python scripts

    ```bash
    cd control_part/sources/carsystems/
    sudo chmod +x *.py
    ```

7. Intall `numpy` library for Python3

    ```bash
    pip3 install numpy
    ```

### Computer
1. Install `OpenCV` library to your computer ([installation on Linux](https://docs.opencv.org/3.4.0/d7/d9f/tutorial_linux_install.html))
2. Install `kivy` library([installation on Linux](https://kivy.org/doc/stable/installation/installation-linux.html))
3. Install `numpy`
    
    ```bash
    pip3 install numpy
    ```

## Usage

1. Mount your camera on the ceiling so that the optical axis is directed as a normal to the floor. Measure the distance from the camera to the floor. Calculate the length of the diagonal of the frame along the floor using the diagonal field view of camera. This value has to be inserted instead of the "D" variable in the file `cv_part/kivy_gui.py`.

    ```python
    px_to_m = D / math.hypot(cam_res[0], cam_res[1])
    ```

2. Creat a color marker for the robot and place it on its upper part
3. Calibrate the colors using this [script](https://github.com/jrosebr1/imutils/blob/master/bin/range-detector). Detection of the robot's position and orientation must be stable. The color ranges for the markers need to be written in the script `cv_part/libs/Mapping.py`. More precisely, inside the constructor of the class "Mapping" in the following lines:

    ```python
    self.blueLower = (96, 110, 126)
	self.blueUpper = (105, 255, 247)
	self.yellowLower = (0, 89, 122)
	self.yellowUpper = (94, 150, 255)
    ```

<div style="text-align: center;"><img src="https://raw.githubusercontent.com/kirillin/parking-lego-car/master/report/images/img/cv_1.png" height="300"/></div>    

4. Clone this repository to any directory

    ```bash
    git clone https://github.com/red-itmo/lego-car-project.git
    ```

5. Launch the CV part of the project

    ```bash
    $ python cv_part/kivy_gui.py
    ```

You will see a window showing a user friendly interface as seen in the figure below.

<div style="text-align: center;"><img src="https://pp.userapi.com/c848528/v848528566/84d08/w5jKwEIQ4nE.jpg" height="300"/></div> 


## Support

Please [open an issue](https://github.com/red-itmo/lego-car-project/issues/new) if you've found a bug.

## Contribution

Please contribute using [Github Flow](https://guides.github.com/introduction/flow/). Create a branch, add commits, and [open a pull request](https://github.com/red-itmo/lego-car-project/compare/).

## Authors

[Evgeniy Antonov](https://github.com/mrclient)

[Aleksandr Kapitonov](https://github.com/kap2fox)

[Aleksandr Karavaev](https://github.com/AlexKaravaev)

[Egor Zamotaev](https://github.com/EgorZamotaev)

[Kirill Artemov](https://github.com/kirillin)

[Oleg Souzdalev](https://github.com/OlegSouzdalev)

[Rami Al-Naim](https://github.com/RamiNaim)

[Dmitrii Dobriborsci]()


