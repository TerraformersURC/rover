
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

## Basic explanations of what stuff does
package drive_train:
    controller_node - basically extracts the linearx and angularz of the /cmd_vel topic (subscribed to this topic, which contains Twist messages), does some stuff with those values to apply to all the wheels, builds a MotorControl message, and publishes it to /motor_control.
package rover_pkg:
    driver_node_v2 - basically subscribes to /motor_control topic and converts each piece of the MotorControl msg to a PWM range which is formatted into a string and sent over serial to the Arduino. I remember wanting to make the port name /dev/Arduino in the Pi's udev rules but idk if i ever actually did it.
interfaces rover_interfaces:
    MotorControl msg - four float64's for each motor
To run the rover, there's a lunch file in /launch/host.launch.py although it needs to be updated to use the driver_node_v2 node. Also, there's a host. and rover. launch files but really what's only important is that driver_node_v2 runs onboard the rover because it uses the rover's hardware directly.
