
# Terraformers URC Rover

Software for the development rover. To be run partially on a laptop with a PS4 controller and partially on the rover's Raspberry Pi.



## Installation and Usage

We are using ds4_driver in order to control the rover with a PlayStation 4 controller.

To set up and use the software, do the following on your laptop.

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
    8. ros2 launch host.launch.py

Then, do the following on the RPi if it hasn't been already set up.
    
    1. Clone this repo into src
    2. Build your workspace
    3. ros2 launch rover.launch.py