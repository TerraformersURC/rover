# Terraformers-Rover

Simulation package for the rover leveraging ROS, Gazebo and RViz.

## Setting up

### Requirements

Working with this project at least requires Linux, Docker, Git and VSCode on your computer.

Refer the [Docker documentation](https://docs.docker.com/engine/install/) for installing Docker engine

> **Note:** Leveraging NVIDIA GPU requires their container toolkit as well. Refer the [documentation](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to install

### Cloning the repository

The base directory of this repository clones to the `src/` directory of the ROS workspace for the Rover.

Create a workspace folder for the Rover `roversim/` in the home directory and create an `src/` folder within it.

```
mkdir -p ~/roversim/src
```

Navigate to the root directory of `roversim/` workspace and clone the repository to the `src/` folder

```
cd ~/roversim
```

```
git clone -b rover-simulation https://github.com/TerraformersURC/Terraformers-Rover.git src
```
> **Note:** Generate and use a Personal Access Token (PAT) in the place of password when prompted. Refer [here](https://docs.github.com/en/authentication/keeping-your-account-and-data-secure/managing-your-personal-access-tokens) for more information.

With this, the files for rover simulation are downloaded to your computer.

### Get started with development

Open the `src/` folder of the workspace in VSCode and click on the remote icon at the bottom-left corner of the screen. Choose **Reopen in Container** to open the folder in the container. That option also could appear in the pop-up at the bottom right if VSCode detects the `.devcontainer/` folder.

> **Note:** Building the container takes some time for the first time.

If the container builds and runs cleanly, the development environment is successfully setup!! Use the terminal in VSCode to spawn the rover and start the ROS nodes.

> **Note:** Make sure the type of terminal is setup as **bash** and set the default profile in the drop down menu beside the + icon. 

There are no known limitations.

## Description of Contents

### Devcontainer folder

The `.devcontainer/` directory consists of the files for containerizing the development environment using Docker. Which means, having the specific Ubuntu distribution and ROS installation are not required!! This folder could be deleted and used as normal **if** the host environment matches the required development environment (possible, but not recommended).

`Dockerfile` describes the development environment for developing this package. The development environment mainly is:

  - Ubuntu Focal (20.04)
  - ROS Noetic full desktop installation (Includes Gazebo and RViz)
  - Additional ROS packages
    - Effort controllers
    - Velocity controllers
    - Joint state controllers

If the host development environment meets these requirements, a container is not required but recommended to maintain isolation from the host operating system.

> **Note:** The target environment will eventually shift to ROS2 Humble with Ubuntu Jammy (22.04) after the current package gets stable and ported a new ROS2 package.

### Description package

`rover_description/` is the ROS package that describes the rover's physical and visual properties, sets up ROS control plugins to actuate the joints and consists instructions for spawning the rover in the simulation or visualization software. 

 - `urdf/` and `description/` folder consists of the physical and visual properties of the rover using URDF/XACRO files. `description/` folder is currently being used among the both. 
  
 - `meshes/` folder consists of the resources for visual description of the rover (3D models). They could be .STL or .DAE for embedding color information.
  
 - `launch/` folder consists of the steps for spawing the rover in Gazebo (A physics simulator) or RViz (Visualization only). This involves setting the correct arguments to start the software, setting up ROS control nodes and parameters etc.
