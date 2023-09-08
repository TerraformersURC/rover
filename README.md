
# Terraformers URC Rover

Software for the development rover. To be run on the Rover's Raspberry Pi 4.



## Installation and Usage

We are using ds4_driver in order to control the rover with a PlayStation 4 controller.

To set up and use the software, do the following the RPi4.

    1. Clone this repo into src
    2. cd into src
    3. git clone https://github.com/naoki-mizuno/ds4drv --branch devel
    4. cd into ds4drv
        4a. python3 setup.py install --prefix ~/.local
        4b. cp udev/50-ds4drv.rules /etc/udev/rules.d
        4c. udevadm control --reload-rules
        4d. udevadm trigger
    6. Back in src, git clone https://github.com/naoki-mizuno/ds4_driver.git
    7. Build your workspace
    8. ros2 launch Terraformers-Rover/launch/host.launch.py