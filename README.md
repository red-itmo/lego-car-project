Lego car-like robot stuff
====================

The repository includes source code for a control system for a mobile car-typed robot based on the Ackerman’s steering model and a computer vision system designed for localization and obstacle detection. 


[Take a loot at the video first.](https://www.youtube.com/watch?v=3-zPCO2_tgE)

## Table of Contents

- [Description]()
- [Installation](#installation)
- [Usage](#usage)
- [Support](#support)
- [Contributing](#contributing)

##Description

The repository consists of three parts:

* The `conntrol_part` folder includes source code of control system
* The `cv_part` folder includes source code of computer vision
* The `scilab` folder includes files associated with the simulation of the control system of the robot in [Scilab](https://github.com/opencollab/scilab)

Car-like robot was builded using LEGO MIDSTRMS EV3 set. The general view of the robot and its kinematic model are shown in figures below.

Car-like robot | Kinematic model
:-------------------------:|:-------------------------:
<img src="https://pp.userapi.com/c848536/v848536279/87218/9GxvTIaiKUI.jpg" width="200"/> | <img src="https://pp.userapi.com/c848536/v848536279/87220/8wans4_NS30.jpg" width="200"/>

All structure of the project may be shown as on the figure below.

<div style="text-align: center;"><img src="https://pp.userapi.com/c851320/v851320556/106bc/aICo-xGfYvE.jpg" height="150"/></div>

## Installation

### For robot

1. Build the car-like robot ([Ackermann Steering tutorial](http://www.moc-pages.com/moc.php/320971)) using [LEGO mindstorms EV3](https://www.lego.com/ru-ru/mindstorms)
2. Creating a bootable Micro SD card with Linux [HowTo](http://www.ev3dev.org/docs/getting-started/)
3. Connect EV3 to WiFi network using WiFi adapter
4. Connecting via SSH to your robot (user: robot, password: maker)

    ```bash
    ssh robot@XXX.XXX.XXX.XXX
    ```
    `XXX.XXX.XXX.XXX` - IP adress of robot
5. Clone repository or copy only one folder `control_part` to your robot

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

### For computer
1. Install `OpenCV` to your computer ([installation on Linux](https://docs.opencv.org/3.4.0/d7/d9f/tutorial_linux_install.html))
2. Install `kivy` library([installation on Linux](https://kivy.org/doc/stable/installation/installation-linux.html))
3. Install `numpy`

## Usage

1. Mount the camera on the ceiling so that the optical axis was directed normal to the floor and measure the distance from the camera to the floor. Now calculate the length of the diagonal of the frame along the floor using the diagonal field of view of camera. This value has to be inserted instead D varible in the file `cv_part/kivy_gui.py`.

    ```python
    px_to_m = D / math.hypot(cam_res[0], cam_res[1])
    ```

2. Creat color marker for robot and connect it to upper part
3. Calibrate colors using [script](https://github.com/jrosebr1/imutils/blob/master/bin/range-detector). Detection of robot posotion and orientation must be stability. Values of color range for markers should write to the script `cv_part/libs/Mapping.py` to the constructor of class Mapping in lines

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

5. Launch cv part of project

    ```bash
    $ python cv_part/kivy_gui.py
    ```

You will see a window with user friendly interface as on the figure below.

<div style="text-align: center;"><img src="https://pp.userapi.com/c848528/v848528566/84d08/w5jKwEIQ4nE.jpg" height="300"/></div> 


## Support

Please [open an issue](https://github.com/red-itmo/lego-car-project/issues/new) for support.

## Contributing

Please contribute using [Github Flow](https://guides.github.com/introduction/flow/). Create a branch, add commits, and [open a pull request](https://github.com/red-itmo/lego-car-project/compare/).

##Authors

[Evgeniy Antonov](https://github.com/mrclient)

[Aleksandr Kapitonov](https://github.com/kap2fox)

[Aleksandr Karavaev](https://github.com/AlexKaravaev)

[Egor Zamotaev](https://github.com/EgorZamotaev)

[Kirill Artemov](https://github.com/kirillin)

[Oleg Souzdalev](https://github.com/OlegSouzdalev)

[Rami Al-Naim](https://github.com/RamiNaim)

[Dmitrii Dobriborsci]()


